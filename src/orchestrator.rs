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
use crate::agents::empennage::EmpennageAgent;
use crate::agents::mission::{MissionAgent, MissionError};
use crate::agents::propulsion::PropulsionAgent;
use crate::agents::trim_authority::TrimAuthorityAgent;
use crate::agents::weight_balance::{WeightBalanceAgent, WeightBalanceOutput};
use crate::models::aircraft_config::AircraftConfig;
use crate::models::aircraft_state::AircraftState;
use crate::models::engine::EngineSpec;
use crate::models::requirements::Requirements;
use crate::models::specs::{EmpennageSpec, MissionSpec, PropulsionSpec, TrimSpec, WingSpec};

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
    /// Limite dianteiro FÍSICO do envelope de CG (task trim-authority) —
    /// flare + rotação de decolagem, ver `agents::trim_authority`. Já
    /// aplicado a `wb` (`WeightBalanceOutput::apply_trim` roda antes deste
    /// `SizedAircraft` ser devolvido) — `wb.scenarios[].inside_envelope`/
    /// `wb.spec.cg_limit_fwd_pct_mac` já refletem este `TrimSpec`.
    pub trim: TrimSpec,
    /// Empenagem dimensionada por coeficiente de volume (Task 4.1) —
    /// puramente geométrica (não depende de MTOW), mas recalculada a cada
    /// iteração junto com a asa por simplicidade; idêntica em todas as
    /// iterações, já que `wing`/`cfg.empennage` não mudam com o MTOW.
    pub emp: EmpennageSpec,
    /// Massa de combustível (kg) requerida pela missão (autonomia mínima +
    /// reserva) no MTOW convergido — não é o tanque cheio. Desde a Task 5.1
    /// é `mission.fuel_total_kg` (análise por segmentos), não mais
    /// `fc_cruise_lph · endurance_min_h / (1 − reserva)`.
    pub mission_fuel_kg: f64,
    /// Análise de missão por segmentos (Task 5.1, `agents::mission::
    /// MissionAgent`) que produziu `mission_fuel_kg` — táxi, subida
    /// integrada, cruzeiro Breguet, descida e reserva, calculada no MTOW
    /// convergido.
    pub mission: MissionSpec,
    /// Diagrama de restrições clássico (W/S × P/W, Task 3.2) — calculado no
    /// MTOW convergido, com a asa/motor/estado finais. Puramente
    /// informativo: recomenda uma área de asa, não redimensiona a
    /// aeronave automaticamente.
    pub constraints: WingLoadingReport,
    /// Trajetória de MTOW ao longo das iterações (primeiro palpite → valor
    /// final convergido) — para diagnóstico/relatório, não é usado por
    /// nenhum agente a jusante.
    pub iterations: Vec<f64>,
    /// Trajetória de `CL_h_trim` (arrasto de trim em cruzeiro, Task 4,
    /// refino-ciclo2) ao longo das iterações — o valor USADO em CADA
    /// iteração (lag-1: calculado com o CG da iteração ANTERIOR, ver
    /// comentário em `size_aircraft_with_max_iters`). Diagnóstico/teste de
    /// estabilidade da convergência do lag — não usado por nenhum agente a
    /// jusante (`TrimSpec::cl_h_trim_cruise`, no relatório final, é
    /// recalculado com o CG JÁ CONVERGIDO desta última iteração, não lido
    /// daqui).
    pub cl_h_trim_iterations: Vec<f64>,
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
    /// A análise de missão por segmentos (Task 5.1, `agents::mission::
    /// MissionAgent`) não conseguiu produzir uma missão fisicamente
    /// alcançável com o MTOW candidato desta iteração — subida travada ou
    /// distância de cruzeiro não positiva. Ver `agents::mission::
    /// MissionError`.
    ///
    /// Nota de projeto: ao contrário de `CombustivelInsuficiente`/
    /// `MtowExcedido` (checados só no PONTO CONVERGIDO — ver docstring de
    /// `size_aircraft_with_max_iters`), este erro pode disparar numa
    /// iteração INTERMEDIÁRIA do laço: sem um `MissionSpec` válido não há
    /// `fuel_kg` para fechar `novo = OEW + payload + fuel_kg` e continuar
    /// iterando. Um palpite inicial transitoriamente mais pesado que o
    /// ponto fixo real poderia, em tese, disparar este erro mesmo quando o
    /// MTOW convergido seria viável — mesma classe de risco documentada
    /// para `CombustivelInsuficiente` antes da correção da Task 3.1, mas
    /// aqui sem uma forma barata de diferir a checagem (o cálculo, não só
    /// o veredito, depende da subida ser viável). Não observado no
    /// baseline real nem nas fixtures sintéticas deste crate — registrado
    /// como refinamento futuro caso apareça na prática.
    MissaoInviavel(MissionError),
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
            SizingError::MissaoInviavel(e) => write!(f, "análise de missão inviável: {e}"),
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

    // Arrasto de trim em cruzeiro (Task 4, refino-ciclo2) — acoplamento
    // LAG-1 com o CG: `CL_h_trim`/`cd_trim` dependem do CG de referência da
    // missão (`WeightBalanceAgent::scenarios`, cenário `MID_MISSION_
    // SCENARIO_NAME`), que só fica disponível DEPOIS de `AerodynamicsAgent`
    // rodar nesta MESMA iteração (a aerodinâmica precisa do polar de
    // cruzeiro ANTES do CG existir). Em vez de resolver a dependência
    // circular por bisseção/iteração interna, usa-se o CG da iteração
    // ANTERIOR do próprio laço de convergência de MTOW (que já itera várias
    // vezes até o MTOW estabilizar) — `x_cg_trim_ref_prev` começa em 0,0
    // (fração de MAC no bordo de ataque, um seed deliberadamente simples e
    // não-físico) na primeira iteração, e é atualizado ao final de CADA
    // iteração com o CG real que acabou de ser calculado. O CG do cenário
    // de referência, por si só, já estabiliza quase imediatamente (não
    // depende de `state.mtow_kg` nem de `wing.cd_cruise` — só de massas/
    // braços fixos, ver docstring do teste de estabilidade abaixo); o
    // resíduo que ainda aparece quando o laço de MTOW para (tolerância
    // frouxa, `CONVERGENCE_TOL_KG`) vem de `wing.cl_cruise` (proporcional a
    // `state.mtow_kg`, sem lag) continuar mudando até o MTOW convergir —
    // ver `cl_h_trim_iterations` no `SizedAircraft` devolvido e o teste
    // `orchestrator::tests::cl_h_trim_converge_estavel_apos_muitas_
    // iteracoes_do_laco_completo`.
    let mut x_cg_trim_ref_prev: f64 = 0.0;
    let mut cl_h_trim_iterations: Vec<f64> = Vec::new();

    for _ in 0..max_iters {
        iterations.push(mtow);

        let mut state = AircraftState::from_config(cfg);
        state.mtow_kg = mtow;

        let mut wing = AerodynamicsAgent::run(&state, req);
        // Dimensionamento da empenagem (Task 4.1) — geometria pura (depende
        // só de `wing`/`cfg.empennage`, não de MTOW), calculada logo após a
        // asa e usada pelo NP dentro do WeightBalanceAgent abaixo.
        let emp = EmpennageAgent::run(&wing, cfg);

        // Arrasto de trim em cruzeiro (Task 4, refino-ciclo2) — usa o CG
        // lag-1 (`x_cg_trim_ref_prev`, ver comentário acima) para fechar
        // `CL_h_trim`/`cd_trim` SEM esperar o `WeightBalanceAgent` desta
        // iteração (que ainda não rodou). MAC calculada diretamente da asa
        // (mesma fórmula usada dentro de `WeightBalanceAgent::run` — barato
        // de recalcular, evita rodar o agente inteiro só por causa do MAC).
        let c_r_trim = crate::agents::weight_balance::chord_root(
            wing.area_m2, wing.span_m, wing.taper_ratio,
        );
        let mac_trim = crate::agents::weight_balance::mean_aerodynamic_chord(
            c_r_trim, wing.taper_ratio,
        );
        let l_h_over_mac_trim = emp.arm_h_m / mac_trim;
        let s_ratio_trim = emp.s_horizontal_m2 / wing.area_m2;
        let cl_h_trim = crate::agents::trim_authority::cl_h_trim_cruise(
            cfg.wing.cm_ac, wing.cl_cruise, x_cg_trim_ref_prev, emp.eta_h,
            s_ratio_trim, l_h_over_mac_trim,
        );
        let cd_trim = crate::agents::trim_authority::cd_trim_cruise(
            cl_h_trim, emp.ar_h, cfg.empennage.e_h, s_ratio_trim,
        );
        crate::agents::aerodynamics::apply_cruise_trim_drag(&mut wing, cd_trim);
        cl_h_trim_iterations.push(cl_h_trim);

        let prop = PropulsionAgent::run(&state, req, &wing, engine);

        // Combustível exigido pela MISSÃO (Task 5.1: análise por segmentos
        // — táxi, subida integrada, cruzeiro Breguet, descida e reserva —
        // ver `agents::mission::MissionAgent`), não o tanque cheio (isso é
        // `state.fuel_capacity_l`, usado só para a checagem de capacidade
        // abaixo). Calculado aqui (a cada iteração) porque `fuel_kg`
        // alimenta `novo`, mas a checagem de capacidade só roda no ponto
        // convergido, abaixo.
        let mission = MissionAgent::run(&state, &wing, &prop, engine, req, mtow)
            .map_err(SizingError::MissaoInviavel)?;
        let fuel_req_l = mission.fuel_total_l;
        let fuel_kg = mission.fuel_total_kg;

        let mut wb = WeightBalanceAgent::run(&state, &wing, engine, cfg, req, &emp);

        // Atualiza o lag-1 do CG de referência para a PRÓXIMA iteração —
        // ver comentário no topo do laço. O `WeightBalanceAgent` desta
        // iteração já rodou, então já sabemos o CG real do cenário de
        // meia-missão; a próxima iteração usará esse valor (em vez do
        // usado nesta, que veio da iteração anterior).
        if let Some(sc) = wb.scenarios.iter()
            .find(|s| s.name == crate::agents::weight_balance::MID_MISSION_SCENARIO_NAME)
        {
            x_cg_trim_ref_prev = sc.cg_pct_mac / 100.0;
        }

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

            // TrimAuthorityAgent (task trim-authority): roda DEPOIS de
            // WeightBalanceAgent + EmpennageAgent (consome os cenários já
            // calculados, `wb.scenarios`, mais a geometria da empenagem) e
            // ANTES do relatório final — `apply_trim` finaliza
            // `wb.scenarios[].inside_envelope`/`wb.spec.
            // cg_limit_fwd_pct_mac`, que até aqui só refletiam o critério
            // TRASEIRO (sm_min). Ver docstring de `WeightBalanceOutput::
            // apply_trim` para a dependência circular resolvida em duas
            // fases.
            let trim = TrimAuthorityAgent::run(cfg, &wing, &emp, &wb);
            wb.apply_trim(&trim);

            return Ok(SizedAircraft {
                state,
                wing,
                prop,
                wb,
                trim,
                emp,
                mission_fuel_kg: fuel_kg,
                mission,
                iterations,
                cl_h_trim_iterations,
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
    use crate::agents::mission::MissionError;
    use crate::models::aircraft_config::test_fixtures::config_teste;
    use crate::models::engine::test_fixtures::{
        motor_generico_fraco_teste as engine_fraco_teste,
        motor_generico_teste as engine_teste,
    };
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

    // ─── Arrasto de trim em cruzeiro — estabilidade do lag-1 (Task 4) ─────

    /// `SizedAircraft::cl_h_trim_iterations` (a trajetória de `CL_h_trim`
    /// USADO em cada passagem do laço, lag-1) mostra convergência
    /// geométrica clara — mas `size_aircraft` retorna assim que o MTOW
    /// estabiliza (`CONVERGENCE_TOL_KG=0,5kg`, um critério de aceite de
    /// PRODUÇÃO deliberadamente frouxo, sem relação com a precisão do lag
    /// de CG), o que tipicamente acontece com um resíduo de `CL_h_trim`
    /// ainda em torno de ~1e-5 (medido tanto na fixture sintética quanto
    /// no baseline real) — NÃO ainda abaixo de 1e-6. A causa NÃO é que o
    /// lag do CG em si convirja devagar: `WeightBalanceAgent` calcula o CG
    /// do cenário de referência a partir só de massas/braços fixos (motor,
    /// itens de `[[masses.items]]`, empenagem, pax, bagagem, MEIO tanque a
    /// CAPACIDADE fixa) — nenhum desses depende de `state.mtow_kg` nem de
    /// `wing.cd_cruise`, então o CG do cenário de referência já é
    /// PRATICAMENTE CONSTANTE a partir da 2ª iteração (verificado
    /// separadamente). O resíduo observado vem de `wing.cl_cruise`
    /// (proporcional a `state.mtow_kg`, recalculado do zero a cada
    /// iteração, SEM lag) continuar mudando enquanto o MTOW ainda não
    /// convergiu — a MESMA taxa de convergência do laço de MTOW.
    ///
    /// Este teste replica o CORPO do laço de `size_aircraft_with_max_iters`
    /// (mesma física, mesma relaxação 0,5/0,5) mas SEM o critério de parada
    /// por MTOW — roda um número FIXO e grande de iterações (60, bem além
    /// de qualquer ponto em que a produção já teria parado) para
    /// caracterizar o comportamento assintótico da recursão COMPLETA (MTOW
    /// + lag-1 do CG): confirma que, dado tempo suficiente, o sistema
    /// converge de fato a `|Δ| < 1e-6` — a alegação de "converge com o
    /// loop" do brief é sobre esse limite assintótico, não sobre onde o
    /// critério de aceite de MTOW (independente, mais frouxo) historicamente
    /// já para.
    #[test]
    fn cl_h_trim_converge_estavel_apos_muitas_iteracoes_do_laco_completo() {
        let cfg = config_teste();
        let engine = engine_teste();
        let req = requisitos_teste();

        let mut mtow = cfg.sizing.mtow_initial_guess_kg;
        let mut x_cg_trim_ref_prev = 0.0_f64;
        let mut history: Vec<f64> = Vec::new();

        for _ in 0..60 {
            let mut state = AircraftState::from_config(&cfg);
            state.mtow_kg = mtow;
            let mut wing = AerodynamicsAgent::run(&state, &req);
            let emp = EmpennageAgent::run(&wing, &cfg);

            let c_r = crate::agents::weight_balance::chord_root(
                wing.area_m2, wing.span_m, wing.taper_ratio,
            );
            let mac = crate::agents::weight_balance::mean_aerodynamic_chord(c_r, wing.taper_ratio);
            let l_h_over_mac = emp.arm_h_m / mac;
            let s_ratio = emp.s_horizontal_m2 / wing.area_m2;
            let cl_h_trim = crate::agents::trim_authority::cl_h_trim_cruise(
                cfg.wing.cm_ac, wing.cl_cruise, x_cg_trim_ref_prev, emp.eta_h, s_ratio, l_h_over_mac,
            );
            history.push(cl_h_trim);
            let cd_trim = crate::agents::trim_authority::cd_trim_cruise(
                cl_h_trim, emp.ar_h, cfg.empennage.e_h, s_ratio,
            );
            crate::agents::aerodynamics::apply_cruise_trim_drag(&mut wing, cd_trim);

            let prop = crate::agents::propulsion::PropulsionAgent::run(&state, &req, &wing, &engine);
            let mission = crate::agents::mission::MissionAgent::run(
                &state, &wing, &prop, &engine, &req, mtow,
            ).expect("fixture sintética deveria produzir missão viável em todas as iterações");
            let fuel_kg = mission.fuel_total_kg;

            let wb = WeightBalanceAgent::run(&state, &wing, &engine, &cfg, &req, &emp);
            let sc = wb.scenarios.iter()
                .find(|s| s.name == crate::agents::weight_balance::MID_MISSION_SCENARIO_NAME)
                .expect("cenário de referência deveria sempre existir");
            x_cg_trim_ref_prev = sc.cg_pct_mac / 100.0;

            let novo = wb.oew_kg + req.payload_kg() + fuel_kg;
            mtow = 0.5 * mtow + 0.5 * novo;
        }

        let n = history.len();
        let delta_final = (history[n - 1] - history[n - 2]).abs();
        println!("delta final (60 iterações) = {delta_final:.3e} | últimos 3 valores: {:?}",
            &history[n - 3..]);
        assert!(delta_final < 1e-6,
            "CL_h_trim deveria convergir a |Δ| < 1e-6 após 60 iterações completas (MTOW + \
             lag-1 do CG) — obtido {delta_final:.3e}; histórico completo: {history:?}");
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

    /// Task 5.1: `MissionAgent::run` pode retornar `MissionError::
    /// SubidaInviavel` (motor incapaz de sustentar a subida até
    /// `cruise_altitude_m` no MTOW candidato) — este teste confirma que o
    /// laço PROPAGA esse erro corretamente através de
    /// `SizingError::MissaoInviavel` (`.map_err(SizingError::
    /// MissaoInviavel)`), em vez de propagar um `Result` desencaixado ou
    /// entrar em pânico. O motor sintético "fraco" (`motor_generico_fraco_
    /// teste`, ~52 kW de pico) já é conhecido por não sustentar o cruzeiro
    /// exigido pela fixture sintética (`propulsion::tests::
    /// motor_fraco_marca_cruzeiro_inviavel`) — a mesma fraqueza de potência
    /// também trava a subida integrada antes de alcançar `cruise_altitude_m`.
    #[test]
    fn motor_fraco_retorna_missao_inviavel_nao_panico() {
        let cfg = config_teste();
        let engine = engine_fraco_teste();
        let req = requisitos_teste();

        let err = size_aircraft(&cfg, &engine, &req)
            .expect_err("motor sintético fraco não deveria conseguir subir até a altitude de \
                          cruzeiro com este MTOW — deveria falhar, não convergir nem entrar \
                          em pânico");
        println!("erro (esperado): {err}");

        match err {
            SizingError::MissaoInviavel(MissionError::SubidaInviavel { rc_ms, .. }) => {
                assert!(rc_ms.is_finite(), "rc_ms do erro deveria ser finito, obtido {rc_ms}");
            }
            other => panic!(
                "esperava SizingError::MissaoInviavel(MissionError::SubidaInviavel), \
                 obtido: {other:?}"
            ),
        }
    }
}
