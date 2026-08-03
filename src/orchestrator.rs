//! Orchestrator — convergência de MTOW (Task 3.1).
//!
//! Antes desta task, o pipeline usava `state.mtow_kg` (a estimativa inicial
//! de `[sizing].mtow_initial_guess_kg`, ex.: 1461 kg no baseline) para o
//! `AerodynamicsAgent` calcular CL/CD de cruzeiro, enquanto
//! `PerformanceAgent`/`StructuralAgent`/`LandingGearAgent`/`ConstraintChecker`
//! usavam `wb.spec.mtow_kg` — o MTOW do cenário "4 pax + bagagem + tanque
//! cheio" calculado pelo `WeightBalanceAgent` a partir da MESMA estimativa
//! inicial, nunca realimentada de volta para a aerodinâmica (bug B5: dois
//! MTOWs diferentes coexistindo no mesmo relatório, sem que nenhum dos dois
//! fosse necessariamente o MTOW de projeto real da missão).
//!
//! `size_aircraft` fecha esse laço: itera em ponto fixo com relaxação até o
//! MTOW convergir — `mtow_{k+1} = OEW(mtow_k) + payload + combustível de
//! missão(mtow_k)` — e devolve um único MTOW de projeto consistente, usado
//! por TODOS os agentes a jusante.

use crate::agents::aerodynamics::AerodynamicsAgent;
use crate::agents::constraint_diagram::{wing_loading_limits, WingLoadingReport};
use crate::agents::propulsion::PropulsionAgent;
use crate::agents::weight_balance::{WeightBalanceAgent, WeightBalanceOutput};
use crate::models::aircraft_config::AircraftConfig;
use crate::models::aircraft_state::AircraftState;
use crate::models::engine::EngineSpec;
use crate::models::requirements::Requirements;
use crate::models::specs::{PropulsionSpec, WingSpec};

/// Número máximo de iterações do laço de ponto fixo antes de desistir.
const MAX_ITERATIONS: u32 = 50;

/// Tolerância de convergência (kg) — |novo_mtow - mtow| abaixo disto encerra
/// o laço.
const CONVERGENCE_TOL_KG: f64 = 0.5;

/// Margem de tolerância numérica sobre a capacidade do tanque, para não
/// rejeitar uma missão que bate exatamente na borda por erro de ponto
/// flutuante (0.1% de folga).
const FUEL_CAPACITY_SLACK: f64 = 1.001;

/// Resultado completo do dimensionamento — MTOW convergido + todas as
/// specs intermediárias que o produziram, mais o histórico de iterações
/// (trajetória de MTOW, primeiro → último) para diagnóstico/relatório.
#[derive(Debug)]
pub struct SizedAircraft {
    /// Estado final da aeronave — `state.mtow_kg` é o MTOW de projeto
    /// convergido (a fonte única usada por Performance/Structural/
    /// LandingGear/ConstraintChecker a jusante).
    pub state: AircraftState,
    pub wing: WingSpec,
    pub prop: PropulsionSpec,
    /// Saída completa do `WeightBalanceAgent` na iteração convergida —
    /// `wb.oew_kg` é o peso vazio operacional usado para fechar o MTOW;
    /// `wb.spec.mtow_kg` continua sendo o MTOW do cenário estrutural
    /// "4 pax + bagagem + tanque cheio" (envelope estrutural, distinto do
    /// MTOW de missão em `state.mtow_kg` — ver docstring do módulo).
    pub wb: WeightBalanceOutput,
    /// Massa de combustível (kg) requerida pela missão (autonomia mínima +
    /// reserva) no MTOW convergido — não é o tanque cheio.
    pub mission_fuel_kg: f64,
    /// Diagrama de restrições clássico (W/S × P/W, Task 3.2) — calculado no
    /// MTOW convergido, com a asa/motor/estado finais. Puramente
    /// informativo: recomenda uma área de asa, não redimensiona a
    /// aeronave automaticamente.
    pub constraints: WingLoadingReport,
    /// Trajetória de MTOW ao longo das iterações (primeiro palpite → valor
    /// final convergido) — para diagnóstico/relatório, não é usado por
    /// nenhum agente a jusante.
    pub iterations: Vec<f64>,
}

/// Erros do laço de convergência de MTOW.
#[derive(Debug, Clone, PartialEq)]
pub enum SizingError {
    /// O laço não convergiu dentro de `MAX_ITERATIONS` iterações.
    NaoConvergiu { ultimo_mtow: f64 },
    /// O MTOW convergido (ou candidato, durante o laço) ultrapassou
    /// `[sizing].mtow_max_kg` — a aeronave sairia do envelope de projeto
    /// pretendido.
    MtowExcedido { mtow: f64, limite: f64 },
    /// O combustível necessário para cumprir a autonomia mínima da missão
    /// (com reserva) excede a capacidade física do tanque configurado —
    /// missão inviável com esta célula/motor, não um bug do laço.
    CombustivelInsuficiente { necessario_l: f64, capacidade_l: f64 },
}

impl std::fmt::Display for SizingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SizingError::NaoConvergiu { ultimo_mtow } => write!(
                f,
                "laço de convergência de MTOW não convergiu dentro do limite de iterações \
                 (último valor: {ultimo_mtow:.1} kg)"
            ),
            SizingError::MtowExcedido { mtow, limite } => write!(
                f,
                "MTOW ({mtow:.1} kg) excede o limite configurado sizing.mtow_max_kg \
                 ({limite:.1} kg)"
            ),
            SizingError::CombustivelInsuficiente { necessario_l, capacidade_l } => write!(
                f,
                "combustível necessário para a missão ({necessario_l:.1} L) excede a \
                 capacidade do tanque configurado ({capacidade_l:.1} L) — missão inviável \
                 com esta célula/motor"
            ),
        }
    }
}

impl std::error::Error for SizingError {}

/// Converge o MTOW de projeto em ponto fixo com relaxação (fator 0.5):
///
/// ```text
/// mtow_{k+1} = 0.5·mtow_k + 0.5·(OEW(mtow_k) + payload + combustível_missão(mtow_k))
/// ```
///
/// Cada iteração reconstrói `AircraftState` a partir de `cfg` com o MTOW
/// candidato da iteração, roda `AerodynamicsAgent` → `PropulsionAgent` →
/// `WeightBalanceAgent` nessa ordem (a mesma ordem de `main.rs`), calcula o
/// combustível de missão (autonomia mínima + reserva, não o tanque cheio) e
/// fecha um novo candidato de MTOW. Converge quando `|novo - mtow| < 0.5 kg`.
///
/// As checagens de aceite (`CombustivelInsuficiente`, `MtowExcedido`) só são
/// avaliadas no PONTO CONVERGIDO — nunca em iterações intermediárias. Rodar
/// a checagem a cada iteração faria o veredito depender do palpite inicial
/// (`sizing.mtow_initial_guess_kg`): um palpite mais pesado que o MTOW real
/// passa por candidatos intermediários mais altos, cujo combustível
/// transiente pode furar a capacidade do tanque mesmo quando o ponto fixo
/// (o único MTOW que efetivamente descreve a aeronave) cabe perfeitamente
/// (achado da revisão desta task: palpite 1.700 kg disparava
/// `CombustivelInsuficiente` com 260,7 L espúrios, enquanto o ponto fixo real
/// precisa de só 243,92 L). Só há uma checagem por-iteração — um bail-out de
/// divergência (`novo > 2× mtow_max_kg`) — que existe apenas para não gastar
/// as `max_iters` completas numa série que já visivelmente não vai convergir
/// dentro do envelope.
pub fn size_aircraft(
    cfg: &AircraftConfig,
    engine: &EngineSpec,
    req: &Requirements,
) -> Result<SizedAircraft, SizingError> {
    size_aircraft_with_max_iters(cfg, engine, req, MAX_ITERATIONS)
}

/// Mesmo laço de `size_aircraft`, com o número máximo de iterações
/// parametrizável — existe só para que `SizingError::NaoConvergiu` seja
/// testável sem depender de uma configuração real que genuinamente precise
/// de mais de 50 iterações para não convergir (o que seria artificial de
/// construir). `size_aircraft` delega para cá com `MAX_ITERATIONS` (50).
pub(crate) fn size_aircraft_with_max_iters(
    cfg: &AircraftConfig,
    engine: &EngineSpec,
    req: &Requirements,
    max_iters: u32,
) -> Result<SizedAircraft, SizingError> {
    let mut mtow = cfg.sizing.mtow_initial_guess_kg;
    let mut iterations: Vec<f64> = Vec::new();

    for _ in 0..max_iters {
        iterations.push(mtow);

        let mut state = AircraftState::from_config(cfg);
        state.mtow_kg = mtow;

        let wing = AerodynamicsAgent::run(&state, req);
        let prop = PropulsionAgent::run(&state, req, &wing, engine);

        // Combustível exigido pela MISSÃO (autonomia mínima requerida, com
        // reserva) — não o tanque cheio (isso é `state.fuel_capacity_l`,
        // usado só para a checagem de capacidade abaixo). Calculado aqui
        // (a cada iteração) porque `fuel_kg` alimenta `novo`, mas só é
        // CHECADO contra a capacidade do tanque no ponto convergido, abaixo.
        let fuel_req_l =
            prop.fc_cruise_lph * req.endurance_min_h / (1.0 - req.fuel_reserve_fraction);
        let fuel_kg = fuel_req_l * engine.fuel.density_kg_per_l;

        let wb = WeightBalanceAgent::run(&state, &wing, engine, cfg, req);

        let novo = wb.oew_kg + req.payload_kg() + fuel_kg;

        // Bail-out de DIVERGÊNCIA (não é a checagem de aceite): se o
        // candidato já dispara para mais do dobro do limite configurado, a
        // série não vai convergir dentro do envelope de projeto — não vale
        // a pena esperar `max_iters` para descobrir isso. Roda a cada
        // iteração de propósito (ao contrário das checagens de aceite
        // abaixo), porque seu objetivo é cortar séries que claramente não
        // vão convergir, não avaliar o resultado final.
        if novo > 2.0 * cfg.sizing.mtow_max_kg {
            return Err(SizingError::MtowExcedido {
                mtow: novo,
                limite: cfg.sizing.mtow_max_kg,
            });
        }

        if (novo - mtow).abs() < CONVERGENCE_TOL_KG {
            // Convergiu — as checagens de ACEITE rodam aqui, uma única vez,
            // sobre o ponto fixo (não sobre um transiente intermediário).
            if fuel_req_l > cfg.fuel_system.capacity_l * FUEL_CAPACITY_SLACK {
                return Err(SizingError::CombustivelInsuficiente {
                    necessario_l: fuel_req_l,
                    capacidade_l: cfg.fuel_system.capacity_l,
                });
            }
            if novo > cfg.sizing.mtow_max_kg {
                return Err(SizingError::MtowExcedido {
                    mtow: novo,
                    limite: cfg.sizing.mtow_max_kg,
                });
            }

            iterations.push(novo);
            state.mtow_kg = novo;
            let constraints = wing_loading_limits(novo, &wing, engine, &state, req);
            return Ok(SizedAircraft {
                state,
                wing,
                prop,
                wb,
                mission_fuel_kg: fuel_kg,
                iterations,
                constraints,
            });
        }

        mtow = 0.5 * mtow + 0.5 * novo;
    }

    Err(SizingError::NaoConvergiu { ultimo_mtow: mtow })
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::aircraft_config::test_fixtures::config_teste;
    use crate::models::engine::test_fixtures::motor_generico_teste as engine_teste;
    use crate::models::requirements::test_fixtures::requisitos_teste;

    #[test]
    fn converge_em_menos_de_50_iteracoes_no_baseline_sintetico() {
        let cfg = config_teste();
        let engine = engine_teste();
        let req = requisitos_teste();

        let sized = size_aircraft(&cfg, &engine, &req)
            .expect("baseline sintético deveria convergir");

        println!(
            "iterations = {:?} | mtow final = {:.3} kg | mission_fuel_kg = {:.3}",
            sized.iterations,
            sized.state.mtow_kg,
            sized.mission_fuel_kg
        );

        assert!(sized.iterations.len() >= 2,
            "deveria haver pelo menos 2 entradas no histórico (palpite inicial + convergido)");
        assert!(sized.iterations.len() < 51,
            "não deveria esgotar as 50 iterações (encontrado {} entradas)",
            sized.iterations.len());

        let n = sized.iterations.len();
        let delta_final = (sized.iterations[n - 1] - sized.iterations[n - 2]).abs();
        assert!(delta_final < CONVERGENCE_TOL_KG,
            "últimas duas entradas deveriam diferir por menos de {CONVERGENCE_TOL_KG} kg \
             (diferença encontrada: {delta_final:.4} kg)");

        // O MTOW convergido no state final deve bater com a última entrada
        // do histórico de iterações — mesma fonte, sem inconsistência B5.
        assert!((sized.state.mtow_kg - sized.iterations[n - 1]).abs() < 1e-9);
    }

    #[test]
    fn trajetoria_de_iteracoes_nao_produz_nan_nem_infinito() {
        let cfg = config_teste();
        let engine = engine_teste();
        let req = requisitos_teste();

        let sized = size_aircraft(&cfg, &engine, &req)
            .expect("baseline sintético deveria convergir");

        for (i, m) in sized.iterations.iter().enumerate() {
            assert!(m.is_finite(), "iterations[{i}] = {m} não é finito");
            assert!(*m > 0.0, "iterations[{i}] = {m} deveria ser positivo");
        }
        assert!(sized.mission_fuel_kg.is_finite() && sized.mission_fuel_kg > 0.0);
    }

    #[test]
    fn missao_impossivel_retorna_erro_nao_loop_infinito() {
        let cfg = config_teste();
        let engine = engine_teste();
        // Autonomia de 30h é fisicamente incompatível com o tanque de
        // 220 L da fixture sintética (`config_teste().fuel_system.capacity_l`)
        // — a missão exige mais combustível do que o tanque pode carregar,
        // não importa o quanto o MTOW cresça.
        let mut req = requisitos_teste();
        req.endurance_min_h = 30.0;

        let err = size_aircraft(&cfg, &engine, &req)
            .expect_err("autonomia de 30h com tanque de 220L deveria falhar, não convergir");

        println!("erro (esperado): {err}");
        match err {
            SizingError::CombustivelInsuficiente { necessario_l, capacidade_l } => {
                assert!(necessario_l > capacidade_l,
                    "necessario_l ({necessario_l:.1}) deveria exceder capacidade_l \
                     ({capacidade_l:.1})");
            }
            other => panic!(
                "esperava SizingError::CombustivelInsuficiente para missão com autonomia \
                 impossível, obtido: {other:?}"
            ),
        }
    }

    #[test]
    fn mtow_max_kg_pequeno_com_config_pesada_retorna_mtow_excedido() {
        let mut cfg = config_teste();
        let engine = engine_teste();
        let req = requisitos_teste();

        // Limite de MTOW artificialmente baixo (1000 kg) — a config
        // sintética (~600 kg de itens de OEW + payload + combustível de
        // missão) converge para um MTOW bem acima disso, então o laço deve
        // rejeitar em vez de aceitar uma aeronave fora do envelope
        // configurado.
        cfg.sizing.mtow_max_kg = 1_000.0;

        let err = size_aircraft(&cfg, &engine, &req)
            .expect_err("mtow_max_kg=1000 com config pesada deveria exceder o limite");

        println!("erro (esperado): {err}");
        match err {
            SizingError::MtowExcedido { mtow, limite } => {
                assert!((limite - 1_000.0).abs() < 1e-9);
                assert!(mtow > limite,
                    "mtow candidato ({mtow:.1}) deveria exceder o limite ({limite:.1})");
            }
            other => panic!(
                "esperava SizingError::MtowExcedido para mtow_max_kg=1000 com config \
                 pesada, obtido: {other:?}"
            ),
        }
    }

    /// Achado da revisão: `NaoConvergiu` não tinha cobertura de teste (a
    /// fixture sintética converge em ~9 iterações, bem abaixo de
    /// `MAX_ITERATIONS`=50, então nenhum teste existente conseguia disparar
    /// esgotamento sem ser artificial). `size_aircraft_with_max_iters`
    /// existe exatamente para isto: força `max_iters=1`, que não dá tempo
    /// nenhum do laço convergir, exercitando o branch de erro sem inventar
    /// uma configuração artificialmente lenta para convergir.
    #[test]
    fn max_iters_esgotado_retorna_nao_convergiu() {
        let cfg = config_teste();
        let engine = engine_teste();
        let req = requisitos_teste();

        let err = size_aircraft_with_max_iters(&cfg, &engine, &req, 1)
            .expect_err("com max_iters=1 o laço não tem chance de convergir \
                         (a fixture leva ~9 iterações normalmente)");

        println!("erro (esperado): {err}");
        match err {
            SizingError::NaoConvergiu { ultimo_mtow } => {
                assert!(ultimo_mtow.is_finite() && ultimo_mtow > 0.0,
                    "ultimo_mtow ({ultimo_mtow}) deveria ser um valor finito e positivo");
                // Com max_iters=1, o laço executa uma única iteração (o
                // palpite inicial relaxado uma vez em direção a `novo`) e
                // esgota antes de reavaliar convergência — `ultimo_mtow`
                // deveria ter se movido do palpite inicial, não ficado
                // parado nele (prova de que a iteração de fato rodou).
                assert!((ultimo_mtow - cfg.sizing.mtow_initial_guess_kg).abs() > 1e-6,
                    "ultimo_mtow ({ultimo_mtow}) deveria ter se movido do palpite inicial \
                     ({}) após uma iteração", cfg.sizing.mtow_initial_guess_kg);
            }
            other => panic!(
                "esperava SizingError::NaoConvergiu para max_iters=1, obtido: {other:?}"
            ),
        }
    }
}
