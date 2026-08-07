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
use crate::agents::mass_model::{MassModelAgent, StructuralMasses};
use crate::agents::mission::{MissionAgent, MissionError};
use crate::agents::propulsion::PropulsionAgent;
use crate::agents::trim_authority::TrimAuthorityAgent;
use crate::agents::vn_diagram::VnDiagramAgent;
use crate::agents::weight_balance::{WeightBalanceAgent, WeightBalanceOutput};
use crate::models::aircraft_config::AircraftConfig;
use crate::models::aircraft_state::AircraftState;
use crate::models::engine::EngineSpec;
use crate::models::requirements::Requirements;
use crate::models::specs::{
    EmpennageSpec, MissionSpec, PropulsionSpec, TrimSpec, VnDiagramSpec, WingSpec,
};

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
    /// Massas estruturais por componente (Raymer, ciclo 3 do plano
    /// oew-parametrico), calculadas na iteração CONVERGIDA com o `n_design`
    /// lag-1 (ver `n_design_iterations` abaixo). Desde a Task 4 do plano
    /// são as massas que ALIMENTAM o `WeightBalanceAgent` (7 itens
    /// estruturais do OEW, `agents::weight_balance::oew_items`) — este
    /// campo é a MESMA `StructuralMasses` que produziu `wb`/`wb.oew_kg`,
    /// não um cálculo paralelo.
    pub structural_masses: StructuralMasses,
    /// Diagrama V-n (Task 4.3) da iteração CONVERGIDA — mesmo valor que
    /// `main.rs` computava localmente antes desta task (entradas idênticas:
    /// `wing`/`wb.spec.mtow_kg`/massa do cenário mais leve/`req`/categoria,
    /// todos já estáveis no ponto fixo de MTOW).
    pub vn: VnDiagramSpec,
    /// Trajetória de `n_design` (fator de carga de projeto do V-n, CS
    /// 23.333/.341) ao longo das iterações — o valor USADO em CADA
    /// iteração é o da iteração ANTERIOR (lag-1, mesmo padrão de
    /// `cl_h_trim_iterations`/`x_cg_trim_ref_prev`): `MassModelAgent::run`
    /// precisa de `n_design` para fechar N_z ultimate, mas o V-n desta
    /// MESMA iteração só fica disponível depois de `WeightBalanceAgent`
    /// rodar (precisa do MTOW/massa leve do cenário). O seed da primeira
    /// entrada é 3.8 (fator de manobra normal típico, N_z ultimate =
    /// 1.5×3.8 = 5.70 — spec do plano) — ver comentário em
    /// `size_aircraft_with_max_iters`. Diagnóstico/teste de estabilidade do
    /// lag — não usado por nenhum agente a jusante (`vn`, acima, é
    /// recalculado com os dados JÁ CONVERGIDOS desta última iteração, não
    /// lido daqui).
    pub n_design_iterations: Vec<f64>,
    /// Trajetória de W_dg de envelope (Ciclo 4, Task 2 — o peso máximo de
    /// projeto de Raymer, `wb.spec.mtow_kg`) ao longo das iterações — o
    /// valor USADO em CADA iteração é o da iteração ANTERIOR (lag-1,
    /// mesmo padrão de `n_design_iterations`/`cl_h_trim_iterations`/
    /// `x_cg_trim_ref_prev`): `MassModelAgent::run` precisa de W_dg/W_l
    /// (o MTOW de envelope) para as equações de massa estrutural, mas o
    /// `wb.spec.mtow_kg` desta MESMA iteração só fica disponível depois de
    /// `WeightBalanceAgent` rodar (que por sua vez consome `masses`). O
    /// seed da primeira entrada é `sizing.mtow_initial_guess_kg` (mesmo
    /// palpite inicial do MTOW de missão — o envelope estabiliza em poucas
    /// iterações). Diagnóstico/teste de estabilidade do lag — não usado
    /// por nenhum agente a jusante (`wb.spec.mtow_kg`, no relatório final,
    /// é o valor JÁ CONVERGIDO desta última iteração, não lido daqui).
    pub mtow_envelope_iterations: Vec<f64>,
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
    // `orchestrator::tests::cl_h_trim_iterations_do_campo_real_converge_
    // geometricamente`.
    let mut x_cg_trim_ref_prev: f64 = 0.0;
    let mut cl_h_trim_iterations: Vec<f64> = Vec::new();

    // `n_design` (Task 3, plano oew-parametrico) — mesmo acoplamento LAG-1
    // do CG acima, agora entre `MassModelAgent` e `VnDiagramAgent`:
    // `MassModelAgent::run` precisa de `n_design` para fechar N_z ultimate
    // (×1.5) das equações de massa estrutural, mas o `VnDiagramSpec` desta
    // MESMA iteração só existe DEPOIS de `WeightBalanceAgent` rodar (o V-n
    // precisa do MTOW de envelope e da massa do cenário mais leve,
    // `wb.spec.mtow_kg`/`wb.scenarios[].total_mass_kg`). Em vez de resolver
    // a dependência circular por iteração interna, usa-se o `n_design` da
    // iteração ANTERIOR do próprio laço de MTOW — seed 3.8 (fator de
    // manobra normal típico, N_z ultimate = 1.5×3.8 = 5.70) na primeira
    // iteração, atualizado ao final de CADA iteração com o `n_design` real
    // que acabou de ser calculado. Ver `n_design_iterations` no
    // `SizedAircraft` devolvido.
    let mut n_design_prev: f64 = 3.8;
    let mut n_design_iterations: Vec<f64> = Vec::new();

    // W_dg de envelope (Ciclo 4, Task 2) — terceiro uso do mesmo padrão
    // lag-1 dos dois acima: `MassModelAgent::run` precisa de W_dg/W_l de
    // Raymer (o peso máximo de projeto), que é o MTOW de ENVELOPE
    // (`wb.spec.mtow_kg`, cenário "4 pax + bagagem + tanque cheio"), não o
    // MTOW de MISSÃO (`mtow`, candidato desta iteração) — até esta task o
    // parâmetro recebia `mtow`, inconsistente com `StructuralAgent`/
    // `LandingGearAgent` (que já usavam o MTOW de envelope). `wb` desta
    // MESMA iteração só existe DEPOIS de `WeightBalanceAgent` rodar, que
    // por sua vez consome `masses` — mesma dependência circular resolvida
    // pelo lag-1 acima, agora com seed simples `sizing.
    // mtow_initial_guess_kg` (o envelope estabiliza em poucas iterações,
    // ver `mtow_envelope_iterations` no `SizedAircraft` devolvido).
    let mut mtow_envelope_prev: f64 = cfg.sizing.mtow_initial_guess_kg;
    let mut mtow_envelope_iterations: Vec<f64> = Vec::new();

    for _ in 0..max_iters {
        iterations.push(mtow);

        let mut state = AircraftState::from_config(cfg);
        state.mtow_kg = mtow;

        let mut wing = AerodynamicsAgent::run(&state, req);
        // Dimensionamento da empenagem (Task 4.1) — geometria pura (depende
        // só de `wing`/`cfg.empennage`, não de MTOW), calculada logo após a
        // asa e usada pelo NP dentro do WeightBalanceAgent abaixo.
        let emp = EmpennageAgent::run(&wing, cfg);

        // Massas estruturais (ciclo 3, plano oew-parametrico) — usa o
        // `n_design` LAG-1 da iteração anterior (ver comentário no topo do
        // laço) e o MTOW candidato desta iteração. Desde a Task 4 do plano
        // `masses` ALIMENTA o `WeightBalanceAgent` abaixo (as 7 massas
        // estruturais do OEW), fechando o laço MTOW→massa estrutural→OEW→
        // MTOW. Registra o valor de `n_design` efetivamente USADO nesta
        // iteração ANTES de ser sobrescrito abaixo (mesmo padrão de
        // `iterations.push(mtow)` no topo do laço — o histórico mostra a
        // entrada, não a saída, de cada iteração).
        let masses = MassModelAgent::run(
            cfg, engine, req, &wing, &emp, mtow_envelope_prev, n_design_prev,
        );
        n_design_iterations.push(n_design_prev);
        mtow_envelope_iterations.push(mtow_envelope_prev);

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

        let mut wb = WeightBalanceAgent::run(&state, &wing, engine, cfg, req, &emp, &masses);

        // Atualiza o lag-1 do W_dg de envelope (Ciclo 4, Task 2) para a
        // PRÓXIMA iteração — `wb` desta iteração já rodou, então já
        // sabemos o MTOW de envelope real; a próxima iteração usará esse
        // valor (em vez do usado nesta, que veio da iteração anterior).
        mtow_envelope_prev = wb.spec.mtow_kg;

        // Atualiza o lag-1 do CG de referência para a PRÓXIMA iteração —
        // ver comentário no topo do laço. O `WeightBalanceAgent` desta
        // iteração já rodou, então já sabemos o CG real do cenário de
        // meia-missão; a próxima iteração usará esse valor (em vez do
        // usado nesta, que veio da iteração anterior). PANIC (não no-op
        // silencioso) se o cenário não existir — mesmo invariante e mesma
        // postura de falha ALTA de `agents::trim_authority::
        // TrimAuthorityAgent::run` (que consome o MESMO cenário do MESMO
        // `wb.scenarios` para o `TrimSpec` final): um no-op aqui deixaria
        // o lag CONGELADO no valor da iteração anterior sem nenhum sinal
        // de que o invariante quebrou, divergindo silenciosamente do
        // comportamento de `TrimAuthorityAgent::run` para o MESMO caso.
        let sc = wb.scenarios.iter()
            .find(|s| s.name == crate::agents::weight_balance::MID_MISSION_SCENARIO_NAME)
            .unwrap_or_else(|| panic!(
                "cenário de referência '{}' não encontrado em wb.scenarios — deveria sempre \
                 existir (ver agents::weight_balance::scenarios_def)",
                crate::agents::weight_balance::MID_MISSION_SCENARIO_NAME
            ));
        x_cg_trim_ref_prev = sc.cg_pct_mac / 100.0;

        // V-n diagram (Task 4.3) desta iteração — roda aqui porque precisa
        // de `wb` (MTOW de envelope + massa do cenário mais leve), que só
        // ficou disponível acima. Atualiza o LAG-1 de `n_design` (ver
        // comentário no topo do laço) para a PRÓXIMA iteração.
        let mass_light_kg = wb.scenarios.iter()
            .map(|s| s.total_mass_kg)
            .fold(f64::INFINITY, f64::min);
        let vn = VnDiagramAgent::run(
            &wing, wb.spec.mtow_kg, mass_light_kg, req, &cfg.structure.design_category,
        );
        n_design_prev = vn.n_design;

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
                structural_masses: masses,
                vn,
                n_design_iterations,
                mtow_envelope_iterations,
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

    /// Testa o campo REAL `SizedAircraft::cl_h_trim_iterations`, populado
    /// pelo próprio `size_aircraft` (não uma réplica manual do corpo do
    /// laço — evita duplicar a lógica de produção num teste separado, o
    /// que arriscaria os dois divergirem sem aviso).
    ///
    /// **Achado honesto (correção pós-revisão)**: o brief original pedia
    /// `|Δ| < 1e-6` entre as duas últimas iterações. Medido contra o
    /// campo REAL: `size_aircraft` retorna assim que o MTOW estabiliza
    /// (`CONVERGENCE_TOL_KG=0,5kg`, um critério de aceite de PRODUÇÃO
    /// deliberadamente frouxo, sem relação com a precisão do lag de CG) —
    /// nesse ponto o resíduo de `CL_h_trim` ainda está em ~1,76e-5 na
    /// fixture sintética (e estava em ~9,9e-6 no baseline real, na época —
    /// medição do ciclo 2 anterior ao OEW estrutural computado, ver
    /// `task-4-report.md`), NÃO abaixo de 1e-6. Este teste pina o valor
    /// REAL medido (não o alvo original do brief, que a produção — com seu
    /// critério de parada atual — nunca alcança) e verifica GENUÍNA
    /// convergência geométrica (cada delta sucessivo estritamente menor
    /// que o anterior), que é a evidência de estabilidade que realmente
    /// importa: a recursão é uma contração, não uma oscilação/divergência
    /// mascarada por um número de iterações pequeno.
    ///
    /// Até a Task 3 do plano oew-parametrico, a causa do resíduo NÃO era o
    /// lag do CG em si: `WeightBalanceAgent` calculava o CG do cenário de
    /// referência a partir só de massas/braços FIXOS (motor, itens de
    /// `[[masses.items]]`, empenagem, pax, bagagem, MEIO tanque a
    /// CAPACIDADE fixa), nenhum dependente de `state.mtow_kg`, então o CG
    /// já era praticamente constante a partir da 2ª iteração e o resíduo
    /// vinha só de `wing.cl_cruise` (proporcional a `state.mtow_kg`, sem
    /// lag). Desde a Task 4 do mesmo plano as 7 massas ESTRUTURAIS do OEW
    /// são COMPUTADAS (`agents::mass_model`) em função do MTOW candidato —
    /// o CG do cenário de referência TAMBÉM se move com o laço agora, e as
    /// duas fontes de resíduo somam. Efeito medido: delta final
    /// 1,5041627731e-5 → 1,7579617312e-5 (Task 4 do plano oew-parametrico).
    ///
    /// **RE-MEDIDO (Ciclo 4, Task 2 — W_dg de envelope com lag-1)**: trocar
    /// o `W_dg`/`W_l` de `MassModelAgent::run` do candidato de MISSÃO desta
    /// iteração (`mtow`, sem lag) pelo envelope LAG-1
    /// (`mtow_envelope_prev`) desacopla as massas estruturais do
    /// transiente do candidato de MTOW cru — elas passam a reagir a uma
    /// entrada mais estável (o envelope já quase convergido), o que
    /// amortece a realimentação MTOW→massas→CG que alimentava este
    /// resíduo. Efeito medido: delta final **1,7579617312e-5 →
    /// 2,497194172366296e-6** (old→new, ~7× menor; convergência geométrica
    /// e ordem de grandeza continuam válidas, só o valor do pin e sua
    /// tolerância mudaram — a tolerância antiga, dimensionada para
    /// ~1,76e-5, não cobre mais o novo pin com folga honesta).
    #[test]
    fn cl_h_trim_iterations_do_campo_real_converge_geometricamente() {
        let cfg = config_teste();
        let engine = engine_teste();
        let req = requisitos_teste();

        let sized = size_aircraft(&cfg, &engine, &req)
            .expect("baseline sintético deveria convergir");
        let history = &sized.cl_h_trim_iterations;
        let n = history.len();
        assert!(n >= 4,
            "esperava pelo menos 4 iterações para caracterizar convergência geométrica \
             (obtido {n})");

        let deltas: Vec<f64> = (1..n).map(|i| (history[i] - history[i - 1]).abs()).collect();
        println!("cl_h_trim_iterations = {history:?}");
        println!("deltas = {deltas:?}");

        // Convergência GEOMÉTRICA genuína: cada delta (a partir do 2º, que
        // ainda carrega o transiente do seed 0,0 inicial) estritamente
        // menor que o anterior — prova que a recursão é uma CONTRAÇÃO, não
        // uma oscilação ou platô artificial.
        for i in 2..deltas.len() {
            assert!(deltas[i] < deltas[i - 1],
                "delta[{i}]={:.3e} deveria ser ESTRITAMENTE menor que delta[{}]={:.3e} — \
                 convergência geométrica quebrada; histórico completo: {history:?}",
                deltas[i], i - 1, deltas[i - 1]);
        }

        // Pin do resíduo REAL no ponto em que a produção retorna (critério
        // de MTOW, não de CL_h_trim) — ver achado honesto (RE-MEDIDO,
        // Ciclo 4 Task 2) na docstring acima. 2,497194172366296e-6 medido
        // (era 1,7579617312e-5 antes do W_dg de envelope com lag-1);
        // tolerância com folga (2x) para não ficar frágil a resíduo de
        // ponto flutuante entre plataformas, mas apertada o bastante para
        // pegar uma regressão real (ex.: lag desligado acidentalmente
        // produziria delta ~O(0,16), o tamanho do transiente do seed, não
        // ~1e-6).
        let delta_final = deltas[deltas.len() - 1];
        let delta_final_pin = 2.497194172366296e-6;
        assert!((delta_final - delta_final_pin).abs() < delta_final_pin,
            "delta final (campo real) = {delta_final:.10e} divergiu do pin honesto \
             ≈{delta_final_pin:.10e} — histórico completo: {history:?}");
        assert!(delta_final < 1e-4,
            "delta final (campo real) = {delta_final:.3e} deveria estar bem abaixo de 1e-4 \
             (ordem de grandeza esperada ~1e-6) — histórico completo: {history:?}");
    }

    // ─── n_design — estabilidade do lag-1 (Task 3, plano oew-parametrico) ──

    /// Testa o campo REAL `SizedAircraft::n_design_iterations`, populado
    /// pelo próprio `size_aircraft` (não uma réplica manual do corpo do
    /// laço — mesma lição de `cl_h_trim_iterations_do_campo_real_converge_
    /// geometricamente`: nunca duplicar a lógica de produção num teste
    /// separado, o que arriscaria os dois divergirem sem aviso).
    ///
    /// **Achado honesto (Task 4 do plano oew-parametrico — RE-MEDIDO)**: na
    /// Task 3 este resíduo era 0.0 EXATO (bit-a-bit), e o comentário antigo
    /// explicava por quê: `VnDiagramAgent::run` só depende da geometria da
    /// asa (fixa, de `[wing]`) e de `wb.spec.mtow_kg`/`mass_light_kg`, que
    /// naquele momento eram somados a partir de `[[masses.items]]`
    /// ESTÁTICOS — nenhuma entrada do V-n mudava com o candidato de MTOW do
    /// laço, então `n_design` já estava no ponto fixo desde a 1ª iteração.
    /// O corte desta task quebrou exatamente essa condição: agora as 7
    /// massas estruturais do OEW são COMPUTADAS (`agents::mass_model`) em
    /// função do MTOW candidato, logo `wb.spec.mtow_kg` (cenário de
    /// envelope) e `mass_light_kg` MUDAM a cada iteração — o lag de
    /// `n_design` passou a ter um resíduo GENUÍNO a medir. Pin antigo:
    /// ≈0.0 (`< 1e-9`). Pin (Task 4, medido na fixture sintética):
    /// 1,731522e-4 no ponto em que a produção retorna (critério de MTOW,
    /// `CONVERGENCE_TOL_KG=0,5kg` — deliberadamente frouxo, sem relação com
    /// a precisão do lag).
    ///
    /// **RE-MEDIDO (Ciclo 4, Task 2 — W_dg de envelope com lag-1)**: dois
    /// lags agora interagem — `MassModelAgent::run` passou a usar o
    /// envelope LAG-1 (`mtow_envelope_prev`) em vez do candidato de MISSÃO
    /// cru (`mtow`) desta iteração. Investigado (achado de revisão): isso
    /// NÃO piora o resíduo, MELHORA por ~7 ordens de grandeza — as massas
    /// estruturais passam a reagir a uma entrada já quase-convergida (o
    /// envelope, que estabiliza geometricamente rápido, ver
    /// `mtow_envelope_iterations`) em vez do candidato de MTOW cru desta
    /// iteração, o que amortece a realimentação MTOW→massas→`wb.spec.
    /// mtow_kg`/`mass_light_kg`→`n_design` — a mesma cadeia que antes
    /// carregava o transiente do candidato de missão a cada passo. Efeito
    /// medido: delta final **1,731522e-4 → 1,5766055128096923e-11**
    /// (old→new; ordem de grandeza cai de ruído físico genuíno para ruído
    /// de ponto flutuante — a convergência geométrica em si continua
    /// válida, só o valor do pin e sua tolerância mudaram).
    ///
    /// A convergência é geométrica a partir de h[3]: as duas primeiras
    /// entradas carregam o transiente do seed 3,8 (undershoot em h[2],
    /// overshoot em h[3]) antes da recursão virar uma contração monótona —
    /// por isso a checagem estrita de deltas decrescentes começa em i=3, e
    /// não em i=2 como em `cl_h_trim_iterations_do_campo_real_converge_
    /// geometricamente`.
    #[test]
    fn n_design_iterations_do_campo_real_converge() {
        let sized = size_aircraft(&config_teste(), &engine_teste(), &requisitos_teste())
            .expect("baseline sintético deveria convergir");
        let h = &sized.n_design_iterations;
        println!("n_design_iterations = {h:?}");
        assert!(h.len() >= 5,
            "esperava pelo menos 5 iterações para caracterizar a contração (obtido {})", h.len());
        // seed 3.8 na primeira entrada (N_z = 1.5×3.8 = 5.70, spec):
        assert!((h[0] - 3.8).abs() < 1e-12, "seed do lag deveria ser 3.8, obtido {}", h[0]);

        let deltas: Vec<f64> = (1..h.len()).map(|i| (h[i] - h[i - 1]).abs()).collect();
        println!("deltas (n_design) = {deltas:?}");
        // Contração GENUÍNA depois do transiente do seed (ver docstring):
        // cada delta a partir do 4º estritamente menor que o anterior.
        for i in 3..deltas.len() {
            assert!(deltas[i] < deltas[i - 1],
                "delta[{i}]={:.3e} deveria ser ESTRITAMENTE menor que delta[{}]={:.3e} — \
                 contração quebrada; histórico completo: {h:?}",
                deltas[i], i - 1, deltas[i - 1]);
        }

        let delta_final = deltas[deltas.len() - 1];
        println!("delta_final (n_design) = {delta_final:.6e}");
        // Pin do resíduo REAL medido (RE-MEDIDO, Ciclo 4 Task 2 — ver
        // achado honesto na docstring acima), com folga de 2× (mesma
        // disciplina do pin de `cl_h_trim`): apertado o bastante para
        // pegar uma regressão real (ex.: lag desligado produziria delta
        // ~O(0,4), o tamanho do transiente do seed), frouxo o bastante
        // para não quebrar com ruído de ponto flutuante entre plataformas.
        let delta_final_pin = 1.5766055128096923e-11;
        assert!((delta_final - delta_final_pin).abs() < 1.6e-11,
            "residual do lag de n_design = {delta_final:.6e} divergiu do pin honesto \
             ≈{delta_final_pin:.6e}; histórico completo: {h:?}");
        assert!(delta_final < 1e-3,
            "residual do lag de n_design = {delta_final:.3e} deveria estar bem abaixo de 1e-3 \
             (ordem de grandeza esperada ~1e-11); histórico completo: {h:?}");
        // + structural_masses do SizedAircraft finitas e positivas:
        for (nome, v) in [("asa", sized.structural_masses.asa_kg),
                          ("fuselagem", sized.structural_masses.fuselagem_kg),
                          ("tanques", sized.structural_masses.tanques_kg)] {
            assert!(v.is_finite() && v > 0.0, "{nome} = {v}");
        }
    }

    // ─── W_dg de envelope — estabilidade do lag-1 (Ciclo 4, Task 2) ────────

    /// Ciclo 4: W_dg do modelo de massas é o MTOW de ENVELOPE com lag-1.
    /// Testa o campo REAL: as massas do SizedAircraft devem ser EXATAMENTE
    /// as que MassModelAgent::run produz com o penúltimo envelope e o
    /// penúltimo n_design do histórico (os valores lag-1 da iteração final).
    #[test]
    fn massas_do_sized_vem_do_envelope_lag_1() {
        let cfg = config_teste();
        let engine = engine_teste();
        let req = requisitos_teste();
        let sized = size_aircraft(&cfg, &engine, &req).expect("fixture converge");

        let env = &sized.mtow_envelope_iterations;
        let nd = &sized.n_design_iterations;
        assert!(env.len() >= 2, "histórico do envelope: {env:?}");
        // seed na primeira entrada (mesmo padrão de n_design_iterations):
        assert!((env[0] - cfg.sizing.mtow_initial_guess_kg).abs() < 1e-12,
            "seed do lag deveria ser mtow_initial_guess_kg, obtido {}", env[0]);

        let w_dg_lag = env[env.len() - 2];
        let n_design_lag = nd[nd.len() - 2];
        let esperado = MassModelAgent::run(&cfg, &engine, &req, &sized.wing,
                                           &sized.emp, w_dg_lag, n_design_lag);
        assert!((sized.structural_masses.asa_kg - esperado.asa_kg).abs() < 1e-9);
        assert!((sized.structural_masses.trem_principal_kg - esperado.trem_principal_kg).abs() < 1e-9);

        // convergência: delta final do envelope pequeno e pinado honesto.
        // O envelope (`wb.spec.mtow_kg`) é quase uma função direta do MTOW
        // de missão convergido (mesma estrutura/masses/geometria) — o lag
        // já está praticamente no ponto fixo quando o laço externo de MTOW
        // para (`CONVERGENCE_TOL_KG=0,5kg`), então o resíduo medido é ruído
        // de ponto flutuante (~1e-8 kg), não um transiente físico genuíno
        // como os de `n_design`/`cl_h_trim`. Pin honesto medido na fixture
        // sintética: 6,397840479621664e-9 kg; folga 2× (mesma disciplina
        // dos outros pins de resíduo).
        let d = (env[env.len() - 1] - env[env.len() - 2]).abs();
        println!("mtow_envelope_iterations = {env:?}");
        println!("d (residual medido) = {d:e}");
        let d_pin = 6.397840479621664e-9;
        assert!((d - d_pin).abs() < d_pin,
            "residual do lag de envelope = {d:e} divergiu do pin honesto ≈{d_pin:e} — \
             histórico completo: {env:?}");
        assert!(d < 5.0, "residual do lag de envelope = {d:.4} kg — MEDIR E PINAR");
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
