//! RobustnessAgent — conjuntos adversariais ±σ (ciclo 4, spec
//! 2026-08-07-ciclo4-fidelidade-massas).
//!
//! As 7 massas estruturais (`agents::mass_model::StructuralMasses`) vêm de
//! equações de componente empíricas (Raymer, "Aircraft Design: A
//! Conceptual Approach", cap. 15.2 — GA) ajustadas a uma FROTA histórica,
//! não a ESTA aeronave: Raymer cap. 15/Roskam ("Airplane Design Part V",
//! Classe II) citam incerteza típica de ±10–20% em projeto conceitual.
//! Este módulo quantifica se checks que PASSAM com as massas NOMINAIS
//! (ponto central das equações) continuariam passando sob essa incerteza
//! — não uma análise probabilística (sem RNG, sem distribuição), mas um
//! PIOR-CASO DETERMINÍSTICO DIRECIONAL: dois conjuntos adversariais, um
//! que empurra o CG vazio o mais para a FRENTE possível (todo componente
//! dianteiro do CG nominal fica ×(1+σ), todo componente traseiro fica
//! ×(1−σ)) e outro que o empurra o mais para TRÁS possível (o oposto) —
//! ver `adversarial_masses`.
//!
//! A classificação dianteiro/traseiro usa o CG VAZIO (`x_cg_oew`) como
//! pivô, não o CG carregado de cada cenário — isso só é um PIOR-CASO EXATO
//! enquanto nenhum braço estrutural cair DENTRO da banda de CG carregado
//! dos cenários (verdadeiro no baseline atual: os 7 braços estruturais vão
//! de 1,40 a 7,40 m, fora da banda de CG carregado de 3,01–3,30 m); se um
//! braço estrutural algum dia cair dentro dessa banda, a classificação por
//! `x_cg_oew` deixa de garantir o pior caso para os cenários cujo CG fica
//! do lado oposto do CG vazio.
//!
//! Consumido por `main.rs` (chamado logo após o `LandingGearAgent`) e por
//! `validation::constraint_checker::ConstraintChecker::verify` (checagem
//! #19 — um `flip` gera uma violação nomeada) desde a Task 4 do ciclo
//! (wiring, schema v4.6) — antes disso o módulo era isolado do pipeline.
//!
//! Os limites de tipback/carga de nariz contra os quais os conjuntos
//! perturbados são avaliados são os NOMINAIS/de config
//! (`gear_cfg.tipback_min_deg`, os tetos/pisos de carga de nariz de
//! `validation::constraint_checker`) — derivados de geometria, não da
//! massa estrutural, logo invariantes à perturbação.
//!
//! **Os limites de CG, não** (mudança do ciclo 10, task 2). Até o ciclo 9
//! eles também eram tratados como invariantes, com a justificativa de que
//! "o `TrimAuthorityAgent` dianteiro/`sm_min` traseiro não dependem de
//! `StructuralMasses`" — verdade enquanto o limite de ROTAÇÃO era
//! invariante ao peso. O momento da LINHA DE TRAÇÃO
//! (`T(Vr(W))·z_eixo`, ver `agents::trim_authority::rotation_fwd_limit_m`)
//! matou essa invariância: o limite dianteiro passou a depender do peso do
//! cenário mais leve, que é função direta das massas estruturais
//! perturbadas. Desde então CADA mundo é avaliado contra a SUA PRÓPRIA
//! régua de CG (`TrimAuthorityAgent` re-rodado sobre o `wb` daquele mundo
//! nos dois casos direcionais; `sized_p.wb.spec`, já finalizado por
//! `apply_trim`, no caso massa-total) — o custo é uma chamada de agente a
//! mais por mundo. A semântica do flip não mudou: "o NOMINAL passava e o
//! mundo perturbado REPROVA".
//!
//! Ciclo 5 (spec robustez-total-e-solo) acrescenta um 3º caso, "massa-
//! total": em vez de perturbar as 7 massas direcionalmente (±σ conforme o
//! braço, mantendo o resto do pipeline NOMINAL fixo), multiplica os 5
//! fatores de composto (`[mass_model].composite_factor_*`) por (1+σ) — TODA
//! massa estrutural mais pesada — e RE-CONVERGE o laço completo
//! (`orchestrator::size_aircraft`), avaliando MTOW/combustível/VS0/
//! desempenho nesse mundo +σ contra os mesmos limites do pipeline nominal
//! (não contra os limites nominais numéricos já calculados, como os dois
//! casos de CG acima — este caso precisa do estado FÍSICO recalculado,
//! porque a asa/hélice/missão também respondem ao MTOW maior). Ver o corpo
//! de `RobustnessAgent::run` para o caso 3.
//!
//! Ciclo 6 (task massa-total-completo): até aqui, a divisão de trabalho
//! entre os 3 mundos era por TIPO de check — os dois casos direcionais
//! cobriam o envelope de CG/trem de pouso (pior-caso de DIREÇÃO de CG,
//! ±σ nas 7 massas mantendo o resto nominal), e o massa-total cobria só
//! MTOW/combustível/VS0/desempenho em nível (todas as massas +σ juntas,
//! `sized_p.wb` DESCARTADO após o re-sizing). Essa divisão era um atalho,
//! não uma garantia física: nada impede o mundo massa-total (MTOW maior,
//! geometria re-convergida) de também empurrar algum cenário de CG para
//! fora do envelope nominal, ou a pista de decolagem/pouso para além da
//! disponível — casos que a divisão anterior simplesmente não olhava. A
//! partir desta task, os 3 mundos avaliam TUDO: pista (decolagem/pouso
//! 50 ft), envelope de CG por cenário, carga de nariz máx/mín e tipback
//! (via `evaluate_world`, extraída da lógica antes embutida só nos dois
//! casos direcionais) além de MTOW/combustível/VS0/desempenho — cada um
//! contra os MESMOS limites nominais, só a fonte do `wb`/gear perturbado
//! muda por mundo (`WeightBalanceAgent::run` com massas ±σ para os
//! direcionais; `sized_p.wb`/`sized_p.structural_masses` re-convergidos
//! para o massa-total; e, desde o ciclo 10 task 2, a régua de CG de cada
//! mundo também vem do próprio mundo — ver o parágrafo sobre limites de CG
//! acima).

use crate::agents::landing_gear::LandingGearAgent;
use crate::agents::mass_model::StructuralMasses;
use crate::agents::weight_balance::{
    cg_from_items, oew_items, structural_arms, WeightBalanceAgent, WeightBalanceOutput,
};
use crate::models::aircraft_config::AircraftConfig;
use crate::models::aircraft_state::AircraftState;
use crate::models::engine::EngineSpec;
use crate::models::requirements::Requirements;
use crate::models::specs::{
    EmpennageSpec, GearSpec, MissionSpec, PerformanceSpec, PropellerSpec, RobustnessFlip,
    RobustnessSpec, WingSpec,
};
use crate::orchestrator::SizingError;
use crate::validation::constraint_checker::{
    NOSE_LOAD_MAX_CEILING_PCT, NOSE_LOAD_MIN_FLOOR_PCT, RC_SL_MIN_MS, SERVICE_CEILING_MIN_M,
};

/// Constrói os 2 conjuntos adversariais de massas estruturais (±σ).
/// Determinístico: classifica cada um dos 7 componentes comparando seu
/// braço de momento (MESMO mapeamento estático componente→braço de
/// `agents::weight_balance::oew_items`) com o CG VAZIO nominal
/// (`cg_from_items(oew_items(...))`) — componentes com braço À FRENTE do
/// CG vazio (`arm <= x_cg_oew`) são "dianteiros"; os demais, "traseiros".
/// Empates (`arm == x_cg_oew`, improvável na prática) vão para o lado
/// dianteiro, por convenção de `<=` (documentado aqui, não coincidência).
///
/// Devolve `(conjunto_cg_mais_dianteiro, conjunto_cg_mais_traseiro)`: no
/// primeiro, todo componente dianteiro fica MAIS pesado (×(1+σ)) e todo
/// componente traseiro fica MAIS leve (×(1−σ)) — o CG vazio resultante se
/// desloca o mais possível para a frente dado σ. O segundo é o espelho
/// exato (dianteiros mais leves, traseiros mais pesados) — desloca o CG
/// vazio o mais possível para trás.
pub fn adversarial_masses(
    cfg: &AircraftConfig,
    engine: &EngineSpec,
    masses: &StructuralMasses,
    sigma: f64,
) -> (StructuralMasses, StructuralMasses) {
    let items = oew_items(cfg, engine, masses);
    let (_, x_cg_oew) = cg_from_items(&items);
    // FONTE ÚNICA do mapeamento componente→braço (ver docstring de
    // `structural_arms`) — MESMA usada por `oew_items` para montar os 7
    // itens estruturais, evitando divergência silenciosa entre os dois.
    let [(_, asa_arm), (_, fuselagem_arm), (_, emp_h_arm), (_, emp_v_arm),
         (_, trem_principal_arm), (_, trem_nariz_arm), (_, tanques_arm)] = structural_arms(cfg);

    // `fwd_heavier`: true monta o conjunto CG-mais-DIANTEIRO (componentes
    // dianteiros ficam mais pesados); false monta o conjunto
    // CG-mais-TRASEIRO (componentes dianteiros ficam mais leves).
    let scale = |mass: f64, arm: f64, fwd_heavier: bool| -> f64 {
        let dianteiro = arm <= x_cg_oew;
        if dianteiro == fwd_heavier { mass * (1.0 + sigma) } else { mass * (1.0 - sigma) }
    };

    let fwd = StructuralMasses {
        asa_kg:            scale(masses.asa_kg,            asa_arm,            true),
        fuselagem_kg:      scale(masses.fuselagem_kg,      fuselagem_arm,      true),
        emp_h_kg:          scale(masses.emp_h_kg,           emp_h_arm,         true),
        emp_v_kg:          scale(masses.emp_v_kg,           emp_v_arm,         true),
        trem_principal_kg: scale(masses.trem_principal_kg,  trem_principal_arm, true),
        trem_nariz_kg:     scale(masses.trem_nariz_kg,      trem_nariz_arm,    true),
        tanques_kg:        scale(masses.tanques_kg,         tanques_arm,       true),
    };
    let aft = StructuralMasses {
        asa_kg:            scale(masses.asa_kg,            asa_arm,            false),
        fuselagem_kg:      scale(masses.fuselagem_kg,      fuselagem_arm,      false),
        emp_h_kg:          scale(masses.emp_h_kg,           emp_h_arm,         false),
        emp_v_kg:          scale(masses.emp_v_kg,           emp_v_arm,         false),
        trem_principal_kg: scale(masses.trem_principal_kg,  trem_principal_arm, false),
        trem_nariz_kg:     scale(masses.trem_nariz_kg,      trem_nariz_arm,    false),
        tanques_kg:        scale(masses.tanques_kg,         tanques_arm,       false),
    };
    (fwd, aft)
}

/// Avalia um `WeightBalanceOutput` JÁ COMPUTADO de um "mundo" perturbado
/// (`wb_p` — dos dois casos direcionais via `WeightBalanceAgent::run` com
/// massas ±σ, ou do caso massa-total via `sized_p.wb` re-convergido) contra
/// os limites NOMINAIS (`wb_nominal`/`gear_nominal`): cenários de CG
/// (envelope dianteiro/traseiro) + trem de pouso (tipback, carga de nariz
/// máx/mín) — devolve a faixa de CG observada e os flips encontrados.
/// `caso` rotula os flips gerados ("dianteiro"/"traseiro"/"massa-total").
///
/// Compartilhada pelos 3 mundos avaliados por `RobustnessAgent::run`
/// (ciclo 6, task massa-total-completo): antes desta extração, os dois
/// casos direcionais tinham essa lógica embutida numa closure local e o
/// caso massa-total DESCARTAVA `sized_p.wb` — este helper é o ÚNICO lugar
/// que compara cenários/gear perturbados contra limites nominais, evitando
/// duplicação entre os 3 mundos.
///
/// `trem_principal_kg_p`/`trem_nariz_kg_p`: massas do trem PERTURBADAS do
/// mundo avaliado (dos dois casos direcionais, `m_p.trem_*_kg`; do caso
/// massa-total, `sized_p.structural_masses.trem_*_kg`) — usadas pelo
/// `LandingGearAgent` junto com o MTOW/CG extremos de `wb_p`.
///
/// POR QUE os limites de envelope são os NOMINAIS mesmo no mundo
/// massa-total, onde o pipeline INTEIRO foi re-convergido (e portanto
/// `wb_p.spec.cg_limit_{fwd,aft}_pct_mac` existem, recalculados) — duas
/// razões, e as duas precisam valer:
///   1. Semântica de FLIP: um flip é "o nominal passava, o perturbado
///      não". A régua tem de ser a MESMA nos dois lados da comparação,
///      senão um limite que se mexe mascara (ou inventa) um flip.
///   2. Numérica: os limites de CG em %MAC são derivados de geometria da
///      asa/empenagem, margem de estabilidade e autoridade de profundor —
///      nenhuma dessas responde ao MTOW no modelo atual, então os limites
///      re-convergidos do mundo +σ são numericamente os nominais. É a
///      razão 2 que torna a razão 1 gratuita (nada é perdido por usar a
///      régua nominal). Se um dia a geometria da asa passar a responder
///      ao MTOW (área alar dimensionada por carga alar, por exemplo), a
///      razão 2 cai e a escolha vira uma decisão de modelagem de verdade
///      — daí o `debug_assert!` no chamador do caso massa-total (`run`),
///      que grita exatamente nesse dia em vez de deixar a divergência
///      passar silenciosa.
/// Devolve também o `GearSpec` perturbado (`gear_p`) — ciclo 8, task 2: o
/// caso "massa-total" (único chamador que usa o 3º elemento; os dois casos
/// direcionais descartam) precisa de `gear_p.nose_oleo_stroke_mm` para a
/// checagem #25 (folga de hélice em condição CRÍTICA, CS 23.925) — o curso
/// do amortecedor de nariz cresce com o MTOW re-convergido, ao contrário da
/// folga ESTÁTICA (`propeller.ground_clearance_m`), invariante à massa.
#[allow(clippy::too_many_arguments)]
fn evaluate_world(
    cfg: &AircraftConfig,
    caso: &str,
    wb_p: &WeightBalanceOutput,
    trem_principal_kg_p: f64,
    trem_nariz_kg_p: f64,
    wb_nominal: &WeightBalanceOutput,
    gear_nominal: &GearSpec,
    fwd_limit_p_pct_mac: f64,
    aft_limit_p_pct_mac: f64,
) -> ([f64; 2], Vec<RobustnessFlip>, GearSpec) {
    let mut flips = Vec::new();

    let mut range = [f64::INFINITY, f64::NEG_INFINITY];
    for (sc_nom, sc_p) in wb_nominal.scenarios.iter().zip(wb_p.scenarios.iter()) {
        range[0] = range[0].min(sc_p.cg_pct_mac);
        range[1] = range[1].max(sc_p.cg_pct_mac);

        // RÉGUA DO MUNDO PERTURBADO (ciclo 10, task 2 — reprojeção): até o
        // ciclo 9 os dois lados da comparação usavam os limites NOMINAIS,
        // justificado porque o limite dianteiro (`TrimAuthorityAgent`) não
        // dependia das massas estruturais — a rotação era INVARIANTE ao
        // peso e a flare não vê massa nenhuma. O momento da LINHA DE
        // TRAÇÃO matou essa invariância (`T(Vr(W))·z_eixo` não escala com
        // `W` — ver `agents::trim_authority::rotation_fwd_limit_m`): o
        // limite dianteiro passou a depender do peso do cenário MAIS LEVE,
        // que É função das massas estruturais perturbadas. Comparar o CG
        // do mundo +σ contra a régua NOMINAL passou a ser desonesto (régua
        // errada para aquele mundo), então cada mundo agora traz a SUA
        // régua (`fwd_limit_p_pct_mac`/`aft_limit_p_pct_mac`, calculada
        // pelo chamador com o `TrimAuthorityAgent` daquele mundo). A
        // semântica de FLIP não muda: "o NOMINAL passava (`sc_nom.
        // inside_envelope`, régua nominal) e o mundo perturbado REPROVA
        // (régua do próprio mundo perturbado)". O limite TRASEIRO continua
        // vindo de `sm_min`/NP e continua invariante às massas — passa a
        // ser conduzido pelo mesmo caminho só por simetria.
        let dentro_do_envelope_p = sc_p.cg_pct_mac >= fwd_limit_p_pct_mac
            && sc_p.cg_pct_mac <= aft_limit_p_pct_mac;
        if !dentro_do_envelope_p && sc_nom.inside_envelope {
            // `limite_nominal` é o limite do MESMO LADO na régua NOMINAL —
            // é a comparação que permite ao leitor separar "o CG andou" de
            // "a régua andou" (ciclo 10, task 2; ver `RobustnessFlip`).
            let cruzou_pela_frente = sc_p.cg_pct_mac < fwd_limit_p_pct_mac;
            let (limite, limite_nominal) = if cruzou_pela_frente {
                (fwd_limit_p_pct_mac, wb_nominal.spec.cg_limit_fwd_pct_mac)
            } else {
                (aft_limit_p_pct_mac, wb_nominal.spec.cg_limit_aft_pct_mac)
            };
            flips.push(RobustnessFlip {
                check: format!("Cenário '{}'", sc_nom.name),
                caso: caso.to_string(),
                valor: sc_p.cg_pct_mac,
                limite,
                limite_nominal,
            });
        }
    }

    let x_fwd_p = cfg.wing.le_root_x_m + wb_p.spec.cg_mac_fwd_pct / 100.0 * wb_p.mac_m;
    let x_aft_p = cfg.wing.le_root_x_m + wb_p.spec.cg_mac_aft_pct / 100.0 * wb_p.mac_m;
    let gear_p = LandingGearAgent::run(
        wb_p.spec.mtow_kg, x_fwd_p, x_aft_p, &cfg.gear,
        trem_principal_kg_p, trem_nariz_kg_p,
    );

    if gear_p.tipback_angle_deg < cfg.gear.tipback_min_deg
        && gear_nominal.tipback_angle_deg >= cfg.gear.tipback_min_deg
    {
        flips.push(RobustnessFlip {
            check: "Tipback".to_string(),
            caso: caso.to_string(),
            valor: gear_p.tipback_angle_deg,
            limite: cfg.gear.tipback_min_deg,
            // Régua de CONFIG, invariante à perturbação — ver `RobustnessFlip`.
            limite_nominal: cfg.gear.tipback_min_deg,
        });
    }
    if gear_p.nose_load_max_pct > NOSE_LOAD_MAX_CEILING_PCT
        && gear_nominal.nose_load_max_pct <= NOSE_LOAD_MAX_CEILING_PCT
    {
        flips.push(RobustnessFlip {
            check: "Carga de nariz máx".to_string(),
            caso: caso.to_string(),
            valor: gear_p.nose_load_max_pct,
            limite: NOSE_LOAD_MAX_CEILING_PCT,
            limite_nominal: NOSE_LOAD_MAX_CEILING_PCT, // teto fixo, invariante
        });
    }
    if gear_p.nose_load_min_pct < NOSE_LOAD_MIN_FLOOR_PCT
        && gear_nominal.nose_load_min_pct >= NOSE_LOAD_MIN_FLOOR_PCT
    {
        flips.push(RobustnessFlip {
            check: "Carga de nariz mín".to_string(),
            caso: caso.to_string(),
            valor: gear_p.nose_load_min_pct,
            limite: NOSE_LOAD_MIN_FLOOR_PCT,
            limite_nominal: NOSE_LOAD_MIN_FLOOR_PCT, // piso fixo, invariante
        });
    }

    (range, flips, gear_p)
}

pub struct RobustnessAgent;

impl RobustnessAgent {
    /// Avalia os dois conjuntos adversariais (`adversarial_masses`, com
    /// `cfg.mass_model.sigma_mass_fraction`) contra os limites NOMINAIS já
    /// calculados (`wb_nominal`/`gear_nominal` — ver docstring do módulo
    /// para o porquê de não reavaliar `TrimAuthorityAgent`). Um `flip` é
    /// registrado por (check, caso) sempre que o conjunto perturbado
    /// REPROVA um check que o NOMINAL passava.
    ///
    /// PRECONDIÇÃO (ciclo 5, caso massa-total): `state`/`wb_nominal`/
    /// `gear_nominal`/`mission_nominal`/`perf_nominal` devem vir TODOS do
    /// MESMO `orchestrator::size_aircraft` que CONVERGIU (`Ok(sized)`) —
    /// nunca de um MTOW candidato/palpite inicial não iterado, nem de
    /// agentes rodados isoladamente com um MTOW arbitrário. O 3º caso
    /// (massa-total) SEMPRE re-converge o laço completo para o mundo +σ; se
    /// o nominal recebido aqui não tivesse convergido de verdade, o flip
    /// "Dimensionamento" reportaria "passa no nominal mas reprova
    /// perturbado" para uma falha que já existia no nominal — ou, mais
    /// sutil, compararia margem/VS0/desempenho contra bases fisicamente
    /// inconsistentes (nominal não convergido vs. perturbado sempre
    /// convergido), enviesando os gates independente do efeito real de σ
    /// (achado de review, ciclo 5 — ver fixture corrigida em
    /// `validation::constraint_checker::tests::setup_with_cfg_and_req`).
    /// Os dois `debug_assert!`s abaixo checam a guarda mais direta e barata
    /// que um sizing convergido garante e um não-convergido tipicamente não
    /// (não é uma prova formal de convergência, mesmo espírito dos dois
    /// `debug_assert!`s pré-existentes sobre `wb_nominal`).
    pub fn run(
        cfg: &AircraftConfig,
        engine: &EngineSpec,
        req: &Requirements,
        state: &AircraftState,
        wing: &WingSpec,
        emp: &EmpennageSpec,
        masses: &StructuralMasses,
        wb_nominal: &WeightBalanceOutput,
        gear_nominal: &GearSpec,
        // Ciclo 8 (task 2): folga ESTÁTICA nominal + folga crítica nominal
        // já computada (`PropellerSpec::prop_clearance_critical_m`) — usada
        // pela checagem #25 no caso massa-total (ver docstring de
        // `evaluate_world` para o porquê de não precisar de um
        // `propeller_p` recalculado).
        propeller_nominal: &PropellerSpec,
        mission_nominal: &MissionSpec,
        perf_nominal: &PerformanceSpec,
    ) -> RobustnessSpec {
        debug_assert!(wb_nominal.spec.cg_limit_fwd_pct_mac.is_finite(),
            "RobustnessAgent exige um wb NOMINAL já com apply_trim (cg_limit_fwd_pct_mac = NaN)");
        debug_assert!(wb_nominal.spec.cg_limit_aft_pct_mac.is_finite(),
            "RobustnessAgent exige um wb NOMINAL já com apply_trim (cg_limit_aft_pct_mac = NaN)");
        // Ciclo 5 (review): guarda mínima de sanidade de que o nominal veio
        // de um sizing CONVERGIDO — `mission_nominal.fuel_total_l` só é
        // finito e positivo quando o `MissionAgent` (chamado dentro do laço
        // de `orchestrator::size_aircraft`) teve sucesso, e `state.mtow_kg`
        // só é o MTOW de missão real (não um palpite/candidato NaN/0) num
        // `SizedAircraft` convergido.
        debug_assert!(
            mission_nominal.fuel_total_l.is_finite() && mission_nominal.fuel_total_l > 0.0,
            "RobustnessAgent exige mission_nominal de um sizing NOMINAL CONVERGIDO \
             (fuel_total_l inválido: {})", mission_nominal.fuel_total_l
        );
        debug_assert!(
            state.mtow_kg.is_finite() && state.mtow_kg > 0.0,
            "RobustnessAgent exige state de um sizing NOMINAL CONVERGIDO (mtow_kg inválido: {})",
            state.mtow_kg
        );
        let sigma = cfg.mass_model.sigma_mass_fraction;
        let (m_fwd, m_aft) = adversarial_masses(cfg, engine, masses, sigma);

        // Avalia um conjunto adversarial (`caso` = "dianteiro"/"traseiro"):
        // computa `wb_p` deste mundo (massas ±σ, resto do pipeline
        // NOMINAL fixo) e delega a comparação contra os limites nominais a
        // `evaluate_world` (compartilhada com o caso massa-total abaixo).
        let evaluate_case = |caso: &str, m_p: &StructuralMasses| -> ([f64; 2], Vec<RobustnessFlip>) {
            let wb_p = WeightBalanceAgent::run(state, wing, engine, cfg, req, emp, m_p);
            // Régua DO MUNDO perturbado (ciclo 10, task 2 — ver o comentário
            // de reprojeção em `evaluate_world`): o limite dianteiro deixou
            // de ser invariante às massas estruturais (o momento da linha de
            // tração faz o limite de rotação depender do peso do cenário
            // mais leve), então cada caso direcional recalcula o SEU
            // `TrimAuthorityAgent` sobre o seu próprio `wb_p`.
            // `thrust_cruise_n = 0,0`: a tração de CRUZEIRO só alimenta
            // `cl_h_trim_cruise`/`cd_trim` (descartados aqui) — NÃO entra em
            // nenhum dos dois limites de CG, que é tudo que este bloco lê.
            let trim_p = crate::agents::trim_authority::TrimAuthorityAgent::run(
                cfg, wing, emp, &wb_p, state, engine, req, 0.0,
            );
            let fwd_p = trim_p.flare_limit_pct_mac.max(trim_p.rotation_limit_pct_mac);
            let aft_p = wb_p.spec.cg_limit_aft_pct_mac;
            // `gear_p` descartado aqui (checagem #25 só se aplica ao caso
            // massa-total — ver docstring de `evaluate_world`).
            let (range, flips, _gear_p) = evaluate_world(cfg, caso, &wb_p, m_p.trem_principal_kg,
                m_p.trem_nariz_kg, wb_nominal, gear_nominal, fwd_p, aft_p);
            (range, flips)
        };

        let (cg_fwd_case_pct_mac, mut flips) = evaluate_case("dianteiro", &m_fwd);
        let (cg_aft_case_pct_mac, flips_traseiro) = evaluate_case("traseiro", &m_aft);
        flips.extend(flips_traseiro);

        // ── Caso 3: MASSA-TOTAL (ciclo 5) — todas as massas estruturais +σ via
        // re-sizing COMPLETO: clona o config multiplicando os 5 fatores de
        // composto por (1+σ) e re-converge o laço inteiro. What-if físico em
        // memória — deliberadamente NÃO re-passa pelas faixas de parse (o
        // produto pode exceder a faixa de config; a faixa protege dados de
        // entrada, não experimentos adversariais). Autonomia não é
        // reavaliada aqui: o MissionAgent a garante por construção ou o
        // sizing falha (CombustivelInsuficiente) — coberto pelo flip de
        // Dimensionamento.
        let mut cfg_p = cfg.clone();
        cfg_p.mass_model.composite_factor_wing *= 1.0 + sigma;
        cfg_p.mass_model.composite_factor_tail *= 1.0 + sigma;
        cfg_p.mass_model.composite_factor_fuselage *= 1.0 + sigma;
        cfg_p.mass_model.composite_factor_gear *= 1.0 + sigma;
        cfg_p.mass_model.composite_factor_fuel_system *= 1.0 + sigma;

        let mtow_masstotal_kg;
        match crate::orchestrator::size_aircraft(&cfg_p, engine, req) {
            Err(e) => {
                mtow_masstotal_kg = 0.0; // sem ponto convergido; flip documenta
                // Achado de review (ciclo 5, Important 2): o `match` anterior
                // só cobria 2 das 4 variantes de `SizingError`, caindo em
                // `f64::NAN` para `NaoConvergiu`/`MissaoInviavel` — NaN vira
                // `null` no JSON (`RobustnessFlip::valor`/`limite` são
                // tipados `f64` no schema, sem variante "ausente") e a
                // mensagem de violação (#19, `ConstraintChecker::verify`)
                // imprimia literalmente "NaN vs NaN", sem diagnóstico
                // nenhum. As 4 variantes abaixo produzem valor/limite SEMPRE
                // finitos:
                //   - `CombustivelInsuficiente`/`MtowExcedido`: já tinham um
                //     par valor/limite natural (necessário vs. capacidade;
                //     MTOW vs. limite configurado).
                //   - `NaoConvergiu { ultimo_mtow }`: sem um "limite" natural
                //     (o laço simplesmente não fechou) — usa `ultimo_mtow`
                //     vs. `cfg.sizing.mtow_max_kg` (o teto de projeto, a
                //     referência mais informativa disponível).
                //   - `MissaoInviavel`: idem, sem par valor/limite natural
                //     (subida travou ou distância de cruzeiro
                //     não-positiva) — usa o MTOW NOMINAL (`state.mtow_kg`,
                //     parâmetro de `run`, já convergido, ver PRECONDIÇÃO na
                //     docstring de `run`) vs. `cfg.sizing.mtow_max_kg`, e o
                //     `check` carrega a mensagem completa do erro
                //     (`SizingError` implementa `Display`) para não perder o
                //     diagnóstico original.
                let (check, valor, limite) = match &e {
                    SizingError::CombustivelInsuficiente { necessario_l, capacidade_l } =>
                        ("Dimensionamento".to_string(), *necessario_l, *capacidade_l),
                    SizingError::MtowExcedido { mtow, limite } =>
                        ("Dimensionamento".to_string(), *mtow, *limite),
                    SizingError::NaoConvergiu { ultimo_mtow } =>
                        ("Dimensionamento".to_string(), *ultimo_mtow, cfg.sizing.mtow_max_kg),
                    SizingError::MissaoInviavel(_) =>
                        (format!("Dimensionamento ({e})"), state.mtow_kg, cfg.sizing.mtow_max_kg),
                };
                // Régua de config (`mtow_max_kg`/capacidade do tanque),
                // invariante à perturbação ⟹ `limite_nominal == limite`.
                flips.push(RobustnessFlip { check, caso: "massa-total".to_string(), valor, limite,
                                            limite_nominal: limite });
            }
            Ok(sized_p) => {
                mtow_masstotal_kg = sized_p.state.mtow_kg;
                let cap = cfg.fuel_system.capacity_l;
                // margem de combustível (fórmula do check #18):
                let margem_p = (cap - sized_p.mission.fuel_total_l) / cap;
                let margem_nom = (cap - mission_nominal.fuel_total_l) / cap;
                if margem_nom >= req.min_fuel_margin_fraction
                    && margem_p < req.min_fuel_margin_fraction {
                    flips.push(RobustnessFlip { check: "Margem de combustível".into(),
                        caso: "massa-total".into(), valor: margem_p * 100.0,
                        limite: req.min_fuel_margin_fraction * 100.0,
                        limite_nominal: req.min_fuel_margin_fraction * 100.0 });
                }
                // VS0 (fórmula do check #2):
                let vs0_lim = req.cruise_speed_min_kmh / 1.8;
                if wing.stall_speed_flaps_kmh <= vs0_lim
                    && sized_p.wing.stall_speed_flaps_kmh > vs0_lim {
                    flips.push(RobustnessFlip { check: "VS0".into(),
                        caso: "massa-total".into(),
                        valor: sized_p.wing.stall_speed_flaps_kmh, limite: vs0_lim,
                        limite_nominal: vs0_lim });
                }
                // desempenho no mundo +σ (mesmos gates do pipeline nominal).
                // `&cfg_p.performance` (não `&cfg.performance` — achado de
                // review, ciclo 5, Minor 4): `[performance]` não é mutado
                // entre `cfg`/`cfg_p` neste caso (só os 5 fatores de
                // composto de massa o são), então os dois eram
                // numericamente idênticos — mas `cfg_p` é a config do mundo
                // +σ que este bloco está de fato avaliando, e usar `cfg`
                // aqui era um desalinhamento de intenção silencioso (poderia
                // divergir silenciosamente se `[performance]` algum dia
                // passasse a depender de `sigma`).
                let perf_p = crate::agents::performance::PerformanceAgent::run(
                    &sized_p.state, &sized_p.wing, &sized_p.prop,
                    sized_p.state.mtow_kg, engine, req, &cfg_p.performance);
                // Pista (Ciclo 6, task 2/3): mesma comparação de
                // `ConstraintChecker::verify` #23/#24 (`perf.{to_50ft_grass,
                // ldg_50ft_grass}_m > req.runway_available_m` é violação,
                // logo "melhor" é MENOR distância — `maior_melhor = false`)
                // — acrescentadas à lista existente de gates de desempenho,
                // mesmo gate `nom_ok && !p_ok` dos demais. As DUAS
                // grandezas são de GRAMA, iguais às do checker (revisão
                // final do ciclo 6: o pouso passou a usar
                // `ldg_50ft_grass_m` — ver `PerformanceSpec`).
                for (nome, nom, p, lim, maior_melhor) in [
                    ("Razão de subida", perf_nominal.rc_sl_ms, perf_p.rc_sl_ms, RC_SL_MIN_MS, true),
                    ("Velocidade de cruzeiro", perf_nominal.v_cruise_kmh, perf_p.v_cruise_kmh,
                     req.cruise_speed_min_kmh, true),
                    ("Teto de serviço", perf_nominal.service_ceiling_m,
                     perf_p.service_ceiling_m, SERVICE_CEILING_MIN_M, true),
                    ("Decolagem (grama, 15 m)", perf_nominal.to_50ft_grass_m, perf_p.to_50ft_grass_m,
                     req.runway_available_m, false),
                    ("Pouso (grama, 15 m)", perf_nominal.ldg_50ft_grass_m,
                     perf_p.ldg_50ft_grass_m, req.runway_available_m, false),
                ] {
                    let nom_ok = if maior_melhor { nom >= lim } else { nom <= lim };
                    let p_ok = if maior_melhor { p >= lim } else { p <= lim };
                    if nom_ok && !p_ok {
                        flips.push(RobustnessFlip { check: nome.into(),
                            caso: "massa-total".into(), valor: p, limite: lim,
                            limite_nominal: lim });
                    }
                }

                // Envelope/nariz/tipback no mundo massa-total (ciclo 6,
                // task massa-total-completo): MESMA avaliação dos dois
                // casos direcionais (`evaluate_world`, extraída acima) —
                // `sized_p.wb` deixa de ser DESCARTADO. Massas do trem
                // PERTURBADAS (`sized_p.structural_masses`, não `masses`
                // nominais) — coerente com o resto do mundo +σ que este
                // bloco avalia. A faixa de CG devolvida não tem campo
                // próprio em `RobustnessSpec` (só os dois casos direcionais
                // têm `cg_fwd_case_pct_mac`/`cg_aft_case_pct_mac`) —
                // descartada aqui.
                //
                // RÉGUA REPROJETADA (ciclo 10, task 2) — aqui morava um
                // `debug_assert!` que exigia que os limites de CG do mundo
                // massa-total COINCIDISSEM com os nominais, porque a
                // comparação de flip usava a régua NOMINAL contra o CG
                // re-convergido. O texto do assert já antecipava este dia:
                // "se [os limites deixarem de ser invariantes], esta
                // comparação precisa ser REPROJETADA, não silenciada".
                //
                // O dia chegou: o momento da LINHA DE TRAÇÃO
                // (`T(Vr(W))·z_eixo`, ver `agents::trim_authority::
                // rotation_fwd_limit_m`) matou a invariância ao peso do
                // limite de ROTAÇÃO, que agora depende do peso do cenário
                // MAIS LEVE — e esse peso muda no mundo +σ. Medido no
                // baseline real (σ=15%): fwd nominal ≈13,3546% vs fwd
                // massa-total ≈13,0064% MAC — as duas réguas divergem (o
                // mundo massa-total AVANÇA ligeiramente frente à nominal, um
                // efeito pequeno mas MENSURÁVEL, não um artefato de
                // arredondamento). O assert foi então SUBSTITUÍDO pela
                // reprojeção que ele mesmo pedia: o mundo massa-total
                // passou a ser avaliado contra a SUA PRÓPRIA régua
                // (`sized_p.wb.spec`, já finalizada por `apply_trim` dentro
                // de `size_aircraft`), exatamente como os dois casos
                // direcionais passaram a usar a deles. A semântica de flip
                // ("nominal passava, mundo perturbado reprova") fica
                // intacta — só a régua do lado PERTURBADO deixou de ser
                // emprestada do mundo nominal.
                // `&cfg_p` (não `cfg` — mesmo ruling do ciclo 5, Minor 4,
                // já aplicado ao `PerformanceAgent` acima): este bloco
                // avalia o mundo +σ, e é a config DESSE mundo que deve
                // alimentar a geometria/gear usados na avaliação. Hoje
                // `[wing]`/`[gear]` não são mutados entre `cfg`/`cfg_p`
                // (só os 5 fatores de composto o são), então os dois são
                // numericamente idênticos — mas passar `cfg` era um
                // desalinhamento de intenção silencioso.
                let (_cg_range_masstotal, flips_masstotal, gear_p_masstotal) = evaluate_world(
                    &cfg_p, "massa-total", &sized_p.wb,
                    sized_p.structural_masses.trem_principal_kg,
                    sized_p.structural_masses.trem_nariz_kg,
                    wb_nominal, gear_nominal,
                    sized_p.wb.spec.cg_limit_fwd_pct_mac,
                    sized_p.wb.spec.cg_limit_aft_pct_mac,
                );
                flips.extend(flips_masstotal);

                // Checagem #25 no mundo massa-total (ciclo 8, task 2) —
                // folga de hélice em condição CRÍTICA (CS 23.925): a folga
                // ESTÁTICA (`propeller_nominal.ground_clearance_m`) é
                // INVARIANTE à massa neste modelo QUANDO `[propeller].
                // diameter_m` está FIXO na config (`source = "config"` —
                // `h_cg_ground_m`/`prop_axis_above_cg_m`/`diameter_m`
                // nenhum dos quais responde a σ nesse modo). No modo
                // DERIVADO (`source = "derivado"`, achado de review não
                // corrigido aqui), o diâmetro é o menor entre os limites de
                // Mach de cruzeiro/folga (`diameter_max_by_mach_cruise_m`,
                // que consome `v_cruise_ms`) — e a velocidade de cruzeiro
                // convergida pode responder ao MTOW re-sizado no mundo
                // massa-total, então a invariância NÃO está garantida nesse
                // modo — mesmo assim, este bloco usa o `ground_clearance_m`
                // NOMINAL como aproximação (não recalcula `propeller` para
                // o mundo massa-total), o que é honesto só no modo config;
                // no modo derivado esta aproximação é uma simplificação
                // adicional não corrigida aqui. Só o curso do amortecedor
                // de NARIZ é de fato recalculado, crescendo com o MTOW
                // re-convergido
                // (`gear_p_masstotal.nose_oleo_stroke_mm`, já produzido por
                // `evaluate_world` acima). Mesmo padrão `nom_ok && !p_ok`
                // dos gates de desempenho acima.
                // Ciclo 9 (transferência de atitude do #25): reaproveita a
                // fonte única de verdade, `PropellerSpec::
                // fill_critical_clearance`, em vez de reimplementar a
                // fórmula fechada aqui (achado de review — a versão
                // anterior deste comentário justificava uma reimplementação
                // manual alegando que o método não serviria para o mundo
                // perturbado, mas `fill_critical_clearance` já recebe
                // `gear`/`gear_cfg` como parâmetros justamente para isso;
                // `gear_p_masstotal` É o `gear` perturbado que o método
                // espera). `ground_clearance_m` (folga ESTÁTICA) continua
                // herdado do `propeller_nominal` clonado — não recalculado
                // para o mundo massa-total, pela mesma aproximação
                // documentada acima (honesta só no modo `source = "config"`
                // — ver parágrafo anterior).
                let mut propeller_p = propeller_nominal.clone();
                propeller_p.fill_critical_clearance(&gear_p_masstotal, &cfg_p.gear, &cfg_p.propeller);
                let folga_critica_p = propeller_p.prop_clearance_critical_m;
                let nom_ok = propeller_nominal.prop_clearance_critical_m > 0.0;
                let p_ok = folga_critica_p > 0.0;
                if nom_ok && !p_ok {
                    flips.push(RobustnessFlip {
                        check: "Hélice (condição crítica CS 23.925)".to_string(),
                        caso: "massa-total".to_string(),
                        valor: folga_critica_p,
                        limite: 0.0,
                        limite_nominal: 0.0, // piso fixo (folga > 0), invariante
                    });
                }
            }
        }

        RobustnessSpec {
            sigma_mass_fraction: sigma,
            cg_fwd_case_pct_mac,
            cg_aft_case_pct_mac,
            mtow_masstotal_kg,
            flips,
        }
    }
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::agents::performance::PerformanceAgent;
    use crate::agents::propeller::PropellerAgent;
    use crate::models::aircraft_config::test_fixtures::config_teste;
    use crate::models::engine::test_fixtures::motor_generico_teste;
    use crate::models::requirements::test_fixtures::requisitos_teste;
    use crate::models::specs::PerformanceSpec;

    /// Pipeline nominal COMPLETO via `orchestrator::size_aircraft` — mesma
    /// sequência de `main.rs`/`validation::constraint_checker` (fixture
    /// `setup_with_cfg_and_req`), incluindo o laço de convergência de MTOW
    /// (`wb` já sai com `apply_trim` — ver docstring de
    /// `orchestrator::SizedAircraft`). Ciclo 5 (task massa-total): trocado
    /// do MTOW/n_design FIXOS (1450.0/4.0, sem convergência) para o laço
    /// REAL porque o 3º caso adversarial de `RobustnessAgent::run`
    /// (massa-total) precisa de um `MissionSpec`/`PerformanceSpec`
    /// nominais fisicamente consistentes com `cfg` para comparar contra o
    /// mundo +σ re-convergido — um nominal sintético/desacoplado da
    /// convergência não teria essa garantia.
    struct Nominal {
        cfg: AircraftConfig,
        engine: EngineSpec,
        req: Requirements,
        state: AircraftState,
        wing: WingSpec,
        emp: EmpennageSpec,
        masses: StructuralMasses,
        wb: WeightBalanceOutput,
        gear: GearSpec,
        propeller: PropellerSpec,
        mission: MissionSpec,
        perf: PerformanceSpec,
    }

    fn nominal_pipeline(cfg: AircraftConfig) -> Nominal {
        nominal_pipeline_with_req(cfg, requisitos_teste())
    }

    /// Mesmo pipeline de `nominal_pipeline`, mas recebe `req` explícito —
    /// usado pelos testes do 3º caso (massa-total) que precisam apertar
    /// `min_fuel_margin_fraction` logo abaixo da margem nominal.
    fn nominal_pipeline_with_req(cfg: AircraftConfig, req: Requirements) -> Nominal {
        let engine = motor_generico_teste();
        let sized = crate::orchestrator::size_aircraft(&cfg, &engine, &req)
            .expect("fixture de teste (config_teste + requisitos_teste + motor_generico_teste) \
                     deveria convergir");
        let state = sized.state;
        let wing = sized.wing;
        let emp = sized.emp;
        let masses = sized.structural_masses;
        let wb = sized.wb;
        let prop = sized.prop;
        let mission = sized.mission;
        let perf = PerformanceAgent::run(&state, &wing, &prop, state.mtow_kg, &engine, &req,
                                          &cfg.performance);
        let x_cg_fwd = cfg.wing.le_root_x_m + wb.spec.cg_mac_fwd_pct / 100.0 * wb.mac_m;
        let x_cg_aft = cfg.wing.le_root_x_m + wb.spec.cg_mac_aft_pct / 100.0 * wb.mac_m;
        let gear = LandingGearAgent::run(
            wb.spec.mtow_kg, x_cg_fwd, x_cg_aft, &cfg.gear,
            masses.trem_principal_kg, masses.trem_nariz_kg,
        );
        // Ciclo 8 (task 2): preenche `prop_clearance_critical_m` (checagem
        // #25) no MESMO caminho de `main.rs` — depois que `gear` existe.
        let mut propeller = PropellerAgent::run(&cfg, &engine, &prop, &req);
        propeller.fill_critical_clearance(&gear, &cfg.gear, &cfg.propeller);
        Nominal { cfg, engine, req, state, wing, emp, masses, wb, gear, propeller, mission, perf }
    }

    /// Classificação direcional: cada um dos 7 componentes entra no lado
    /// certo do conjunto adversarial comparando o braço REAL — achado por
    /// nome na saída de `oew_items`, não uma cópia manual do mapeamento
    /// componente→braço — com o CG VAZIO nominal
    /// (`cg_from_items(oew_items(...))`).
    #[test]
    fn conjuntos_adversariais_perturbam_na_direcao_certa() {
        let n = nominal_pipeline(config_teste());
        let sigma = 0.20;

        let items = oew_items(&n.cfg, &n.engine, &n.masses);
        let (_, x_cg_oew) = cg_from_items(&items);
        println!("x_cg_oew = {x_cg_oew:.4}");

        let (fwd, aft) = adversarial_masses(&n.cfg, &n.engine, &n.masses, sigma);

        // Braço de cada componente vindo da saída REAL de `oew_items`
        // (achado pelo `MassItem::name`) — evita uma 3ª cópia manual do
        // mapeamento componente→braço (fonte única em
        // `agents::weight_balance::structural_arms`, consumida tanto por
        // `oew_items` quanto por `adversarial_masses`); se as duas
        // divergirem, este teste detecta.
        let braco = |nome_item: &str| items.iter()
            .find(|i| i.name == nome_item)
            .unwrap_or_else(|| panic!("oew_items deveria conter o item '{nome_item}'"))
            .arm_m;

        let componentes: [(&str, f64, f64, f64, f64); 7] = [
            ("asa",            n.masses.asa_kg,            braco("asa"),            fwd.asa_kg,            aft.asa_kg),
            ("fuselagem",      n.masses.fuselagem_kg,      braco("fuselagem"),      fwd.fuselagem_kg,      aft.fuselagem_kg),
            ("emp_h",          n.masses.emp_h_kg,          braco("emp_horizontal"), fwd.emp_h_kg,          aft.emp_h_kg),
            ("emp_v",          n.masses.emp_v_kg,          braco("emp_vertical"),   fwd.emp_v_kg,          aft.emp_v_kg),
            ("trem_principal", n.masses.trem_principal_kg, braco("trem_principal"), fwd.trem_principal_kg, aft.trem_principal_kg),
            ("trem_nariz",     n.masses.trem_nariz_kg,     braco("trem_nariz"),     fwd.trem_nariz_kg,     aft.trem_nariz_kg),
            ("tanques",        n.masses.tanques_kg,        braco("tanques"),        fwd.tanques_kg,        aft.tanques_kg),
        ];

        for (nome, massa_nominal, braco, massa_fwd, massa_aft) in componentes {
            let dianteiro = braco <= x_cg_oew;
            println!(
                "{nome}: braço={braco:.4} dianteiro={dianteiro} nominal={massa_nominal:.4} \
                 fwd={massa_fwd:.4} aft={massa_aft:.4}"
            );
            if dianteiro {
                // componente dianteiro: no conjunto CG-mais-dianteiro fica
                // MAIS pesado; no CG-mais-traseiro fica MAIS leve.
                assert!((massa_fwd - massa_nominal * (1.0 + sigma)).abs() < 1e-9,
                    "{nome} (dianteiro) deveria ficar ×(1+σ) no conjunto dianteiro");
                assert!((massa_aft - massa_nominal * (1.0 - sigma)).abs() < 1e-9,
                    "{nome} (dianteiro) deveria ficar ×(1−σ) no conjunto traseiro");
            } else {
                assert!((massa_fwd - massa_nominal * (1.0 - sigma)).abs() < 1e-9,
                    "{nome} (traseiro) deveria ficar ×(1−σ) no conjunto dianteiro");
                assert!((massa_aft - massa_nominal * (1.0 + sigma)).abs() < 1e-9,
                    "{nome} (traseiro) deveria ficar ×(1+σ) no conjunto traseiro");
            }
        }
    }

    /// σ→0 degenera no nominal: flips vazio e faixas de CG iguais às
    /// nominais (tolerância 1e-9) — construção (massas idênticas produzem
    /// `WeightBalanceOutput`/`GearSpec` bit-a-bit idênticos), não
    /// coincidência.
    #[test]
    fn sigma_zero_nao_produz_flips() {
        let mut cfg = config_teste();
        cfg.mass_model.sigma_mass_fraction = 1e-12; // σ efetivamente nulo — construído em memória, portanto não passa pela faixa validada (0.05, 0.30) de parse_aircraft
        let n = nominal_pipeline(cfg);

        let out = RobustnessAgent::run(
            &n.cfg, &n.engine, &n.req, &n.state, &n.wing, &n.emp, &n.masses, &n.wb, &n.gear,
            &n.propeller, &n.mission, &n.perf,
        );

        println!("flips={:?}", out.flips);
        assert!(out.flips.is_empty(), "σ≈0 não deveria produzir flips: {:?}", out.flips);
        assert!((out.cg_fwd_case_pct_mac[0] - n.wb.spec.cg_mac_fwd_pct).abs() < 1e-6,
            "faixa dianteira[0] deveria bater com o nominal: {} vs {}",
            out.cg_fwd_case_pct_mac[0], n.wb.spec.cg_mac_fwd_pct);
        assert!((out.cg_fwd_case_pct_mac[1] - n.wb.spec.cg_mac_aft_pct).abs() < 1e-6,
            "faixa dianteira[1] deveria bater com o nominal: {} vs {}",
            out.cg_fwd_case_pct_mac[1], n.wb.spec.cg_mac_aft_pct);
        assert!((out.cg_aft_case_pct_mac[0] - n.wb.spec.cg_mac_fwd_pct).abs() < 1e-6,
            "faixa traseira[0] deveria bater com o nominal: {} vs {}",
            out.cg_aft_case_pct_mac[0], n.wb.spec.cg_mac_fwd_pct);
        assert!((out.cg_aft_case_pct_mac[1] - n.wb.spec.cg_mac_aft_pct).abs() < 1e-6,
            "faixa traseira[1] deveria bater com o nominal: {} vs {}",
            out.cg_aft_case_pct_mac[1], n.wb.spec.cg_mac_aft_pct);
    }

    /// Config sintética MARGINAL: aperta `gear.tipback_min_deg` até ~0,5°
    /// ABAIXO do tipback nominal (nominal passa por pouco) — com σ=0.20 o
    /// conjunto CG-TRASEIRO empurra o CG mais para trás, reduzindo o
    /// tipback (`θ = atan((x_main−x_cg_aft)/h_cg)`, x_cg_aft maior ⇒ θ
    /// menor) o suficiente para derrubar o check. Ciclo 6 (task
    /// massa-total-completo): o mundo massa-total agora avalia tipback
    /// também (`evaluate_world` compartilhada, ver docstring do módulo) —
    /// as massas estruturais mais pesadas nesse mundo deslocam o CG aft o
    /// suficiente para cruzar o MESMO piso apertado, então esta fixture
    /// (desenhada só para o caso "traseiro") agora produz 2 flips de
    /// Tipback, não 1: "traseiro" (achado original) e "massa-total"
    /// (achado honesto desta task — não forçado, apenas verificado).
    #[test]
    fn config_marginal_gera_flip_nomeado() {
        let n0 = nominal_pipeline(config_teste());
        let theta_nominal = n0.gear.tipback_angle_deg;
        println!("theta_nominal = {theta_nominal:.3}");

        let mut cfg = config_teste();
        cfg.gear.tipback_min_deg = theta_nominal - 0.5;
        let n = nominal_pipeline(cfg);
        assert!(n.gear.tipback_angle_deg >= n.cfg.gear.tipback_min_deg,
            "pré-condição do teste: tipback nominal ({:.3}) deveria passar por pouco o piso \
             marginal ({:.3})", n.gear.tipback_angle_deg, n.cfg.gear.tipback_min_deg);

        let out = RobustnessAgent::run(
            &n.cfg, &n.engine, &n.req, &n.state, &n.wing, &n.emp, &n.masses, &n.wb, &n.gear,
            &n.propeller, &n.mission, &n.perf,
        );
        println!("flips={:?}", out.flips);

        assert_eq!(out.flips.len(), 2,
            "esperava exatamente 2 flips (Tipback/traseiro + Tipback/massa-total): {:?}",
            out.flips);
        for flip in &out.flips {
            assert_eq!(flip.check, "Tipback");
            assert!(flip.caso == "traseiro" || flip.caso == "massa-total",
                "caso inesperado para o flip de Tipback: {}", flip.caso);
            assert!(flip.valor < flip.limite,
                "valor ({}) deveria estar abaixo do limite ({}) — é isso que caracteriza o flip",
                flip.valor, flip.limite);
            assert!((flip.limite - n.cfg.gear.tipback_min_deg).abs() < 1e-9);
        }
    }

    /// Tanque apertado (`fuel_system.capacity_l = 172.0`, achado por sonda
    /// numérica: nominal converge com margem pequena — 168,55 L exigidos
    /// para 172 L de capacidade — e o mundo +σ, que exige mais combustível
    /// de missão porque o MTOW re-convergido sobe, estoura essa
    /// capacidade): `RobustnessAgent::run` produz um ÚNICO flip
    /// "Dimensionamento" (caso "massa-total") citando
    /// `SizingError::CombustivelInsuficiente` (necessário > capacidade) e
    /// `mtow_masstotal_kg = 0.0` (sem ponto convergido).
    #[test]
    fn sizing_inviavel_no_mundo_mais_sigma_gera_flip_de_dimensionamento() {
        let mut cfg = config_teste();
        cfg.fuel_system.capacity_l = 172.0;
        let n = nominal_pipeline(cfg);

        let out = RobustnessAgent::run(
            &n.cfg, &n.engine, &n.req, &n.state, &n.wing, &n.emp, &n.masses, &n.wb, &n.gear,
            &n.propeller, &n.mission, &n.perf,
        );
        println!("flips={:?}", out.flips);
        println!("mtow_masstotal_kg={}", out.mtow_masstotal_kg);

        assert_eq!(out.mtow_masstotal_kg, 0.0,
            "sizing +σ deveria falhar — sem ponto convergido, mtow_masstotal_kg deveria ser 0.0");
        assert_eq!(out.flips.len(), 1,
            "esperava exatamente 1 flip (Dimensionamento/massa-total): {:?}", out.flips);
        let flip = &out.flips[0];
        // `starts_with` (achado de review, ciclo 5, ver fix do Important 2):
        // este cenário dispara `SizingError::CombustivelInsuficiente`, cujo
        // `check` continua sendo exatamente "Dimensionamento" — mas
        // `starts_with` deixa o teste resiliente ao caso `MissaoInviavel`
        // (não exercitado por ESTA fixture), cujo `check` carrega a
        // mensagem completa do erro (`"Dimensionamento (...)"`).
        assert!(flip.check.starts_with("Dimensionamento"),
            "check do flip deveria começar com \"Dimensionamento\": {:?}", flip.check);
        assert_eq!(flip.caso, "massa-total");
        assert!(flip.valor > flip.limite,
            "CombustivelInsuficiente: necessario_l ({}) deveria exceder capacidade_l ({})",
            flip.valor, flip.limite);
        assert!((flip.limite - n.cfg.fuel_system.capacity_l).abs() < 1e-9,
            "limite do flip deveria ser a capacidade do tanque configurada ({})",
            n.cfg.fuel_system.capacity_l);
    }

    /// Margem de combustível NOMINAL folgada (`config_teste()`, ≈23,2% da
    /// capacidade) mas `min_fuel_margin_fraction` apertado logo ABAIXO
    /// dela (via `nominal_pipeline_with_req`): o nominal passa por pouco, e
    /// o mundo +σ — que exige mais combustível de missão (MTOW re-
    /// convergido maior) — derruba a margem bem abaixo do piso apertado,
    /// gerando o flip "Margem de combustível" (caso "massa-total"). Mesma
    /// fórmula do check #18 (`ConstraintChecker::verify`).
    #[test]
    fn margem_de_combustivel_marginal_flipa_no_caso_massa_total() {
        let n0 = nominal_pipeline(config_teste());
        let cap = n0.cfg.fuel_system.capacity_l;
        let margem_nom = (cap - n0.mission.fuel_total_l) / cap;
        println!("margem_nom = {margem_nom:.5}");

        let mut req = requisitos_teste();
        req.min_fuel_margin_fraction = margem_nom - 0.001; // logo abaixo da margem nominal
        let n = nominal_pipeline_with_req(config_teste(), req);

        let out = RobustnessAgent::run(
            &n.cfg, &n.engine, &n.req, &n.state, &n.wing, &n.emp, &n.masses, &n.wb, &n.gear,
            &n.propeller, &n.mission, &n.perf,
        );
        println!("flips={:?}", out.flips);

        let flips_margem: Vec<_> = out.flips.iter()
            .filter(|f| f.check == "Margem de combustível" && f.caso == "massa-total")
            .collect();
        assert_eq!(flips_margem.len(), 1,
            "esperava exatamente 1 flip Margem de combustível(massa-total): {:?}", out.flips);
        let flip = flips_margem[0];
        assert!(flip.valor < flip.limite,
            "margem sob +σ ({:.3}%) deveria ficar ABAIXO do piso apertado ({:.3}%) — é isso que \
             caracteriza o flip", flip.valor, flip.limite);
        assert!((flip.limite - n.req.min_fuel_margin_fraction * 100.0).abs() < 1e-9);
    }

    /// σ mínimo da faixa válida de `parse_aircraft` (0.05 — ver comentário
    /// de `sigma_zero_nao_produz_flips`) com as margens folgadas da
    /// fixture intacta (`config_teste()`): nenhum flip no caso
    /// "massa-total" (nem Dimensionamento, nem margem/VS0/desempenho) —
    /// perturbação pequena demais para derrubar qualquer check. O MTOW
    /// re-convergido (`mtow_masstotal_kg`) fica ACIMA do nominal — os 5
    /// fatores de composto só multiplicam por (1+σ) > 1, nunca reduzem
    /// massa.
    #[test]
    fn caso_massa_total_bem_formado_sem_flips_na_fixture_folgada() {
        let mut cfg = config_teste();
        cfg.mass_model.sigma_mass_fraction = 0.05;
        let n = nominal_pipeline(cfg);

        let out = RobustnessAgent::run(
            &n.cfg, &n.engine, &n.req, &n.state, &n.wing, &n.emp, &n.masses, &n.wb, &n.gear,
            &n.propeller, &n.mission, &n.perf,
        );
        println!("mtow nominal = {:.3}  mtow_masstotal_kg = {:.3}",
            n.state.mtow_kg, out.mtow_masstotal_kg);
        println!("flips={:?}", out.flips);

        assert!(!out.flips.iter().any(|f| f.caso == "massa-total"),
            "fixture folgada (σ=0.05) não deveria produzir flip no caso massa-total: {:?}",
            out.flips);
        assert!(out.mtow_masstotal_kg > n.state.mtow_kg,
            "mtow_masstotal_kg ({:.3}) deveria ficar ACIMA do MTOW nominal ({:.3}) — \
             perturbação para CIMA", out.mtow_masstotal_kg, n.state.mtow_kg);
    }

    /// Pista marginal no nominal flipa no massa-total (distância cresce com
    /// MTOW): `req.runway_available_m` ajustado para logo ACIMA do
    /// `to_50ft_grass_m` nominal (nominal passa por pouco); o mundo +σ
    /// re-convergido tem MTOW maior ⇒ distância de decolagem maior,
    /// estourando a pista apertada — gera o flip "Decolagem (grama, 15 m)"
    /// caso "massa-total". Valores achados por sonda numérica na fixture
    /// intacta (`config_teste()`): `to_50ft_grass_m` nominal ≈366,2 m,
    /// massa-total ≈409,0 m.
    #[test]
    fn decolagem_marginal_flipa_no_caso_massa_total() {
        let mut req = requisitos_teste();
        let n0 = nominal_pipeline_with_req(config_teste(), req.clone());
        req.runway_available_m = n0.perf.to_50ft_grass_m + 1.0; // logo acima do nominal
        let n = nominal_pipeline_with_req(config_teste(), req);
        assert!(n.perf.to_50ft_grass_m <= n.req.runway_available_m,
            "pré-condição do teste: decolagem nominal ({:.1} m) deveria passar por pouco a pista \
             marginal ({:.1} m)", n.perf.to_50ft_grass_m, n.req.runway_available_m);

        let out = RobustnessAgent::run(
            &n.cfg, &n.engine, &n.req, &n.state, &n.wing, &n.emp, &n.masses, &n.wb, &n.gear,
            &n.propeller, &n.mission, &n.perf,
        );
        println!("flips={:?}", out.flips);

        let flips_pista: Vec<_> = out.flips.iter()
            .filter(|f| f.check == "Decolagem (grama, 15 m)" && f.caso == "massa-total")
            .collect();
        assert_eq!(flips_pista.len(), 1,
            "esperava exatamente 1 flip Decolagem (grama, 15 m)/massa-total: {:?}", out.flips);
        let flip = flips_pista[0];
        assert!(flip.valor > flip.limite,
            "distância de decolagem sob +σ ({:.1} m) deveria exceder a pista marginal ({:.1} m) — \
             é isso que caracteriza o flip", flip.valor, flip.limite);
        assert!((flip.limite - n.req.runway_available_m).abs() < 1e-9);
    }


    /// Carga de NARIZ no mundo massa-total (deferred da Task 3 do ciclo 6,
    /// fechado na revisão final): fixture com `gear.x_main_m` recuado de
    /// 3,75 para 3,30 m (achado por sonda numérica) até a carga de nariz
    /// MÍNIMA nominal ficar logo ACIMA do piso de 8% (≈8,50%) — o mundo +σ
    /// re-convergido a derruba para ≈7,02%, cruzando o piso e gerando o
    /// flip "Carga de nariz mín" caso "massa-total".
    ///
    /// POR QUE o piso (mín) e não o TETO (máx), como o achado de revisão
    /// sugeria: no modelo atual, o mundo massa-total desloca o CG para
    /// TRÁS (as 5 massas de composto ×(1+σ) são dominadas por componentes
    /// atrás do CG vazio), e carga de nariz CAI com CG traseiro. Medido na
    /// mesma sonda, para `x_main_m` de 3,16 a 3,32 m: a carga de nariz
    /// MÁXIMA (medida no CG mais dianteiro) vai de ≈24,4→20,9%,
    /// 24,9→21,5%, ..., 29,9→26,6% — SEMPRE menor no mundo +σ que no
    /// nominal. Logo o TETO de 25% é inatingível por esse mundo: para o
    /// perturbado cruzá-lo, o nominal já teria de estar acima (e aí não há
    /// flip, por definição). O piso mín é o lado do gate de carga de
    /// nariz que este mundo de fato pressiona — testá-lo é o teste
    /// dirigido honesto; forçar o teto exigiria uma fixture em que o +σ
    /// movesse o CG para a FRENTE, o que este modelo de massas não
    /// produz.
    #[test]
    fn carga_de_nariz_no_mundo_massa_total_flipa_quando_marginal() {
        let mut cfg = config_teste();
        cfg.gear.x_main_m = 3.30; // sonda numérica: mín nominal ≈8,50% (piso 8%)
        let n = nominal_pipeline(cfg);
        assert!(n.gear.nose_load_min_pct >= NOSE_LOAD_MIN_FLOOR_PCT,
            "pré-condição do teste: carga de nariz MÍNIMA nominal ({:.3}%) deveria passar por \
             pouco o piso de {:.1}%", n.gear.nose_load_min_pct, NOSE_LOAD_MIN_FLOOR_PCT);
        assert!(n.gear.nose_load_max_pct > NOSE_LOAD_MAX_CEILING_PCT,
            "pré-condição/documentação da fixture: a carga de nariz MÁXIMA nominal ({:.3}%) já \
             está acima do teto de {:.1}% nesta posição de trem — nenhum flip de 'Carga de nariz \
             máx' é possível aqui, por definição de flip (ver docstring)",
             n.gear.nose_load_max_pct, NOSE_LOAD_MAX_CEILING_PCT);

        let out = RobustnessAgent::run(
            &n.cfg, &n.engine, &n.req, &n.state, &n.wing, &n.emp, &n.masses, &n.wb, &n.gear,
            &n.propeller, &n.mission, &n.perf,
        );
        println!("flips={:?}", out.flips);

        let flips_nariz: Vec<_> = out.flips.iter()
            .filter(|f| f.check == "Carga de nariz mín" && f.caso == "massa-total")
            .collect();
        assert_eq!(flips_nariz.len(), 1,
            "esperava exatamente 1 flip Carga de nariz mín/massa-total: {:?}", out.flips);
        let flip = flips_nariz[0];
        assert!((flip.limite - NOSE_LOAD_MIN_FLOOR_PCT).abs() < 1e-9,
            "limite do flip deveria ser o piso de carga de nariz: {} vs {}",
            flip.limite, NOSE_LOAD_MIN_FLOOR_PCT);
        assert!(flip.valor < flip.limite,
            "carga de nariz mín sob +σ ({:.3}%) deveria ficar ABAIXO do piso ({:.3}%) — é isso \
             que caracteriza o flip", flip.valor, flip.limite);
        assert!(flip.valor < n.gear.nose_load_min_pct,
            "o mundo +σ deveria REDUZIR a carga de nariz mínima (CG mais traseiro): {:.3}% \
             perturbado vs {:.3}% nominal", flip.valor, n.gear.nose_load_min_pct);

        // Fixture folgada (`config_teste()` intacta, trem em 3,75 m): a
        // carga de nariz mínima nominal tem folga ampla sobre o piso —
        // nenhum flip de nariz é gerado no caso massa-total.
        let n_folgada = nominal_pipeline(config_teste());
        let out_folgada = RobustnessAgent::run(
            &n_folgada.cfg, &n_folgada.engine, &n_folgada.req, &n_folgada.state, &n_folgada.wing,
            &n_folgada.emp, &n_folgada.masses, &n_folgada.wb, &n_folgada.gear, &n_folgada.propeller,
            &n_folgada.mission, &n_folgada.perf,
        );
        assert!(!out_folgada.flips.iter().any(|f| f.check.starts_with("Carga de nariz")
            && f.caso == "massa-total"),
            "fixture folgada não deveria produzir flip de carga de nariz no caso massa-total: {:?}",
            out_folgada.flips);
    }

    /// Envelope/nariz no mundo massa-total: fixture com `arms.pax_rear_m`
    /// deslocado (5.75→6.2, achado por sonda numérica) até o cenário "4 pax
    /// sem bagagem" ficar DENTRO do envelope nominal mas perto do limite
    /// TRASEIRO — o mundo +σ re-convergido (MTOW maior desloca o CG desse
    /// cenário para TRÁS, ≈37,35%) cruza esse limite, gerando o flip
    /// nomeado "Cenário '4 pax sem bagagem'" caso "massa-total".
    /// Fixture folgada (`config_teste()` intacta, sem o ajuste): NENHUM
    /// cenário passa no nominal (`inside_envelope` sempre `false` nessa
    /// fixture sintética — o envelope nominal nunca abraça a faixa de CG
    /// carregado dela, achado da sonda), logo nenhum flip de "Cenário" é
    /// sequer POSSÍVEL — confirmado abaixo, não só assumido.
    ///
    #[test]
    fn envelope_no_mundo_massa_total_flipa_quando_marginal() {
        let mut cfg = config_teste();
        cfg.arms.pax_rear_m = 6.2;
        let n = nominal_pipeline(cfg);
        let sc_marginal = n.wb.scenarios.iter().find(|s| s.name == "4 pax sem bagagem")
            .expect("fixture deveria ter o cenário '4 pax sem bagagem'");
        assert!(sc_marginal.inside_envelope,
            "pré-condição do teste: cenário '4 pax sem bagagem' deveria passar no envelope \
             nominal (cg={:.3}%, fwd={:.3}%, aft={:.3}%)", sc_marginal.cg_pct_mac,
            n.wb.spec.cg_limit_fwd_pct_mac, n.wb.spec.cg_limit_aft_pct_mac);

        let out = RobustnessAgent::run(
            &n.cfg, &n.engine, &n.req, &n.state, &n.wing, &n.emp, &n.masses, &n.wb, &n.gear,
            &n.propeller, &n.mission, &n.perf,
        );
        println!("flips={:?}", out.flips);

        let flips_cenario: Vec<_> = out.flips.iter()
            .filter(|f| f.check == "Cenário '4 pax sem bagagem'" && f.caso == "massa-total")
            .collect();
        assert_eq!(flips_cenario.len(), 1,
            "esperava exatamente 1 flip Cenário '4 pax sem bagagem'/massa-total: {:?}", out.flips);
        let flip = flips_cenario[0];
        assert!((flip.limite - n.wb.spec.cg_limit_aft_pct_mac).abs() < 1e-9,
            "limite do flip deveria ser o limite traseiro nominal (cruzado por trás): {} vs {}",
            flip.limite, n.wb.spec.cg_limit_aft_pct_mac);
        assert!(flip.valor > flip.limite,
            "cg sob +σ ({:.3}%) deveria exceder o limite traseiro ({:.3}%) — é isso que \
             caracteriza o flip", flip.valor, flip.limite);

        // Fixture folgada (config_teste() intacta): nenhum cenário passa no
        // envelope nominal nela — logo nenhum flip de "Cenário" é possível.
        let n_folgada = nominal_pipeline(config_teste());
        assert!(n_folgada.wb.scenarios.iter().all(|s| !s.inside_envelope),
            "pré-condição da fixture folgada: nenhum cenário deveria passar no envelope nominal \
             (achado da sonda numérica) — se isso mudou, a fixture não serve mais de contraste");
        let out_folgada = RobustnessAgent::run(
            &n_folgada.cfg, &n_folgada.engine, &n_folgada.req, &n_folgada.state, &n_folgada.wing,
            &n_folgada.emp, &n_folgada.masses, &n_folgada.wb, &n_folgada.gear, &n_folgada.propeller,
            &n_folgada.mission, &n_folgada.perf,
        );
        assert!(!out_folgada.flips.iter().any(|f| f.check.starts_with("Cenário")
            && f.caso == "massa-total"),
            "fixture folgada não deveria produzir flip de Cenário no caso massa-total: {:?}",
            out_folgada.flips);
    }

    /// Fixture intacta (`config_teste()`, σ=0.20 da fixture): saída
    /// bem-formada — faixas de CG do caso dianteiro À FRENTE das nominais
    /// e do caso traseiro ATRÁS (desigualdade estrita: perturbar 7
    /// componentes em ±20% desloca o CG vazio o suficiente para mover
    /// TODOS os cenários), flips só contêm checks que passam no nominal.
    #[test]
    fn casos_adversariais_movem_o_cg_nas_duas_direcoes() {
        let n = nominal_pipeline(config_teste());
        assert!((n.cfg.mass_model.sigma_mass_fraction - 0.20).abs() < 1e-9,
            "pré-condição: fixture deveria ter sigma_mass_fraction=0.20");

        let out = RobustnessAgent::run(
            &n.cfg, &n.engine, &n.req, &n.state, &n.wing, &n.emp, &n.masses, &n.wb, &n.gear,
            &n.propeller, &n.mission, &n.perf,
        );
        println!("nominal fwd/aft = {:.3}/{:.3}  caso dianteiro = {:?}  caso traseiro = {:?}",
            n.wb.spec.cg_mac_fwd_pct, n.wb.spec.cg_mac_aft_pct,
            out.cg_fwd_case_pct_mac, out.cg_aft_case_pct_mac);

        // Caso dianteiro: os dois extremos ficam estritamente À FRENTE
        // (menor %MAC) dos extremos nominais correspondentes.
        assert!(out.cg_fwd_case_pct_mac[0] < n.wb.spec.cg_mac_fwd_pct,
            "extremo dianteiro do caso dianteiro deveria ficar à frente do nominal");
        assert!(out.cg_fwd_case_pct_mac[1] < n.wb.spec.cg_mac_aft_pct,
            "extremo traseiro do caso dianteiro deveria ficar à frente do nominal");

        // Caso traseiro: os dois extremos ficam estritamente ATRÁS (maior
        // %MAC) dos extremos nominais correspondentes.
        assert!(out.cg_aft_case_pct_mac[0] > n.wb.spec.cg_mac_fwd_pct,
            "extremo dianteiro do caso traseiro deveria ficar atrás do nominal");
        assert!(out.cg_aft_case_pct_mac[1] > n.wb.spec.cg_mac_aft_pct,
            "extremo traseiro do caso traseiro deveria ficar atrás do nominal");

        // Todo flip reportado corresponde a um check que passava no
        // nominal — reconstrói o veredito nominal de cada tipo de check
        // citado em `flip.check` e confirma que ele passava.
        for flip in &out.flips {
            if flip.check == "Tipback" {
                assert!(n.gear.tipback_angle_deg >= n.cfg.gear.tipback_min_deg,
                    "flip de Tipback só deveria existir se o nominal passava");
            } else if flip.check == "Carga de nariz máx" {
                assert!(n.gear.nose_load_max_pct <= NOSE_LOAD_MAX_CEILING_PCT);
            } else if flip.check == "Carga de nariz mín" {
                assert!(n.gear.nose_load_min_pct >= NOSE_LOAD_MIN_FLOOR_PCT);
            } else if let Some(nome_cenario) = flip.check.strip_prefix("Cenário '").and_then(|s| s.strip_suffix('\'')) {
                let sc = n.wb.scenarios.iter().find(|s| s.name == nome_cenario)
                    .unwrap_or_else(|| panic!("cenário '{nome_cenario}' do flip não existe no nominal"));
                assert!(sc.inside_envelope,
                    "flip do cenário '{nome_cenario}' só deveria existir se ele passava no nominal");
            } else {
                panic!("check de flip desconhecido: {}", flip.check);
            }
        }
    }

    /// ACHADO HONESTO (ciclo 8, task 2): a checagem #25 no caso massa-total
    /// foi ESPECIFICADA no brief da task como um gate `nom_ok && !p_ok`
    /// análogo aos demais (pista/desempenho acima) — "o mundo massa-total
    /// já computa um trem perturbado; seu curso cresce com o MTOW". Isso é
    /// FALSO no modelo atual: `agents::landing_gear::min_oleo_stroke_m`
    /// calcula `stroke = E_cinética / (n_g_max × W_perna × η_oleo)` com
    /// `E_cinética = ½·mtow·v²` e `W_perna = mtow·G/2` — o `mtow_kg`
    /// CANCELA algebricamente do numerador e do denominador
    /// (`stroke = v² / (n_g_max·G·η_oleo)`, uma CONSTANTE que não depende
    /// de peso nenhum). `nose_oleo_stroke_mm` (60% do curso principal,
    /// `agents::landing_gear::LandingGearAgent::run`) herda essa
    /// invariância. Verificado numericamente abaixo: MTOW sobe de
    /// ≈1.285 kg (nominal) para ≈1.363 kg (massa-total, σ=20%) e
    /// `nose_oleo_stroke_mm` NÃO SE MOVE um bit. Como a folga ESTÁTICA
    /// (`propeller.ground_clearance_m`), a deflexão de pneu
    /// (`gear_cfg.tire_deflation_delta_m`), a fração de sag estática
    /// (`gear_cfg.static_sag_fraction`, ciclo 10) e, desde o ciclo 9, o
    /// `fator` de amplificação do pivô (`(gear.x_main_m−
    /// propeller.prop_plane_x_m)/(gear.x_main_m−gear.x_nose_m)` —
    /// geometria/config pura, nenhum dos quatro campos responde a σ)
    /// também são invariantes à massa, os CINCO termos de
    /// `prop_clearance_critical_m` são invariantes ao mundo
    /// massa-total — o gate #25 (`RobustnessAgent::run`, ramo `Ok(sized_p)`) está
    /// corretamente FIADO (mesmo padrão `nom_ok && !p_ok` dos demais) mas
    /// é estruturalmente MORTO sob o modelo de trem atual: nenhuma config
    /// pode fazê-lo flipar, porque o valor perturbado é sempre BIT-A-BIT
    /// igual ao nominal, não só numericamente próximo. Corrigir isso
    /// exigiria tornar `min_oleo_stroke_m` sensível ao peso de verdade
    /// (ex.: um `n_g_max` ou `sink_rate` que dependesse de carga alar/MTOW)
    /// — fora do escopo desta task (o gate em si, exigido pelo brief, está
    /// implementado e correto; este teste documenta por que ele nunca
    /// dispara, em vez de fabricar uma fixture "marginal" que só
    /// funcionaria mascarando a invariância com números artificiais).
    #[test]
    fn folga_critica_no_mundo_massa_total_e_invariante_ao_mtow_no_modelo_atual() {
        let n = nominal_pipeline(config_teste());
        assert!(n.propeller.prop_clearance_critical_m > 0.0,
            "pré-condição: folga crítica nominal deveria ser positiva na fixture padrão");

        let sigma = n.cfg.mass_model.sigma_mass_fraction;
        let mut cfg_p = n.cfg.clone();
        cfg_p.mass_model.composite_factor_wing *= 1.0 + sigma;
        cfg_p.mass_model.composite_factor_tail *= 1.0 + sigma;
        cfg_p.mass_model.composite_factor_fuselage *= 1.0 + sigma;
        cfg_p.mass_model.composite_factor_gear *= 1.0 + sigma;
        cfg_p.mass_model.composite_factor_fuel_system *= 1.0 + sigma;
        let sized_p = crate::orchestrator::size_aircraft(&cfg_p, &n.engine, &n.req)
            .expect("mundo massa-total (fixture padrão, σ=20%) deveria convergir");
        assert!(sized_p.state.mtow_kg > n.state.mtow_kg * 1.02,
            "pré-condição: MTOW do mundo massa-total ({:.1} kg) deveria crescer \
             claramente acima do nominal ({:.1} kg) — senão o teste não exercitaria \
             nenhuma perturbação de peso real", sized_p.state.mtow_kg, n.state.mtow_kg);

        let x_fwd_p = cfg_p.wing.le_root_x_m
            + sized_p.wb.spec.cg_mac_fwd_pct / 100.0 * sized_p.wb.mac_m;
        let x_aft_p = cfg_p.wing.le_root_x_m
            + sized_p.wb.spec.cg_mac_aft_pct / 100.0 * sized_p.wb.mac_m;
        let gear_p = LandingGearAgent::run(sized_p.wb.spec.mtow_kg, x_fwd_p, x_aft_p, &cfg_p.gear,
            sized_p.structural_masses.trem_principal_kg, sized_p.structural_masses.trem_nariz_kg);

        assert_eq!(gear_p.nose_oleo_stroke_mm, n.gear.nose_oleo_stroke_mm,
            "curso do amortecedor de nariz deveria ficar EXATAMENTE invariante ao MTOW \
             (cancelamento algébrico em min_oleo_stroke_m) — perturbado {:.6}mm vs \
             nominal {:.6}mm", gear_p.nose_oleo_stroke_mm, n.gear.nose_oleo_stroke_mm);

        // Ciclo 9/10: os CINCO termos de `prop_clearance_critical_m`
        // (`ground_clearance_m`/`nose_oleo_stroke_mm`/
        // `static_sag_fraction`/`tire_deflation_delta_m`/`fator`) são
        // invariantes ao mundo massa-total — `fator` é geometria pura
        // (`[gear].x_main_m`/`x_nose_m`/`[propeller].prop_plane_x_m`,
        // nenhum dos quais responde a σ) e `static_sag_fraction` (ciclo 10)
        // é config pura, idem. Verificação bit-exata, ponta a ponta,
        // reaproveitando a MESMA
        // `fill_critical_clearance` que a produção usa (não uma
        // reimplementação paralela da fórmula) — prova a invariância citada
        // acima em vez de só inferi-la da ausência de flip.
        let mut propeller_p = n.propeller.clone();
        propeller_p.fill_critical_clearance(&gear_p, &cfg_p.gear, &cfg_p.propeller);
        assert_eq!(propeller_p.prop_clearance_critical_m, n.propeller.prop_clearance_critical_m,
            "prop_clearance_critical_m deveria ficar EXATAMENTE invariante ao MTOW no mundo \
             massa-total — perturbado {:.9} vs nominal {:.9}",
            propeller_p.prop_clearance_critical_m, n.propeller.prop_clearance_critical_m);

        let out = RobustnessAgent::run(
            &n.cfg, &n.engine, &n.req, &n.state, &n.wing, &n.emp, &n.masses, &n.wb, &n.gear,
            &n.propeller, &n.mission, &n.perf,
        );
        assert!(!out.flips.iter().any(|f| f.check.starts_with("Hélice")),
            "consequência do achado acima: nenhum flip de folga de hélice crítica é \
             possível no mundo massa-total sob o modelo de trem atual, obteve: {:?}",
            out.flips);
    }

    /// F1 (ciclo 10, FIX WAVE): fixture DIRECIONAL onde a régua do mundo
    /// perturbado (`flip.limite`) diverge da régua nominal
    /// (`flip.limite_nominal`) — o caso que justifica o campo
    /// `RobustnessFlip::limite_nominal` mas que nenhum teste exercitava
    /// até aqui (o único assert pré-existente, em
    /// `constraint_checker::tests::check_19_transforma_flips_em_violacoes_nomeadas`,
    /// injeta um `RobustnessFlip` sintético com `limite_nominal == limite`
    /// — não prova que a REPROJEÇÃO da régua em `evaluate_world`
    /// funciona).
    ///
    /// Mesma fixture de `envelope_no_mundo_massa_total_flipa_quando_marginal`
    /// (`arms.pax_rear_m` 5.75→6.2, achado por sonda numérica): desloca o
    /// cenário "4 pax sem bagagem" para DENTRO do envelope nominal
    /// (cg≈34,97%, entre fwd≈34,10% e aft≈36,41%). Sob o conjunto
    /// adversarial DIANTEIRO (`caso="dianteiro"`, as 7 massas estruturais
    /// dianteiras ×(1+σ), as traseiras ×(1−σ)) o cenário some para
    /// cg≈31,72% — à FRENTE do limite dianteiro nominal (34,10%), gerando
    /// o flip. A régua contra a qual esse flip é medido (`fwd_limit_p_pct_mac`
    /// em `evaluate_world`) NÃO é a nominal: o momento da linha de tração
    /// faz `rotation_limit_pct_mac` responder ao peso dos cenários, e as
    /// massas estruturais do mundo dianteiro mudam a massa de TODOS os
    /// cenários (via OEW) — a régua recalculada sobe para ≈34,41%, ~0,32pp
    /// ATRÁS da régua nominal (≈34,10%). Uma regressão que voltasse a
    /// comparar contra `wb_nominal.spec.cg_limit_fwd_pct_mac` produziria o
    /// MESMO flip (mesmo `valor`, mesmo `check`), só com `limite` errado —
    /// por isso o teste reconstrói a régua do mundo independentemente (via
    /// `WeightBalanceAgent`/`TrimAuthorityAgent` no MESMO conjunto
    /// adversarial `m_fwd`, não uma cópia do número) e compara com
    /// `flip.limite`, não só com um pin numérico solto.
    #[test]
    fn regua_do_mundo_dianteiro_diverge_da_nominal_no_flip_de_cenario() {
        let mut cfg = config_teste();
        cfg.arms.pax_rear_m = 6.2;
        let n = nominal_pipeline(cfg);

        let sc_marginal = n.wb.scenarios.iter().find(|s| s.name == "4 pax sem bagagem")
            .expect("fixture deveria ter o cenário '4 pax sem bagagem'");
        assert!(sc_marginal.inside_envelope,
            "pré-condição do teste: cenário '4 pax sem bagagem' deveria passar no envelope \
             nominal (cg={:.3}%, fwd={:.3}%, aft={:.3}%)", sc_marginal.cg_pct_mac,
            n.wb.spec.cg_limit_fwd_pct_mac, n.wb.spec.cg_limit_aft_pct_mac);

        // Régua do mundo dianteiro, reconstruída INDEPENDENTEMENTE (mesmo
        // caminho de `RobustnessAgent::run`/`evaluate_case`, não uma cópia
        // do resultado): conjunto adversarial `m_fwd`, `WeightBalanceAgent`
        // sobre ele, `TrimAuthorityAgent` sobre o `wb_p` resultante.
        let sigma = n.cfg.mass_model.sigma_mass_fraction;
        let (m_fwd, _m_aft) = adversarial_masses(&n.cfg, &n.engine, &n.masses, sigma);
        let wb_p = WeightBalanceAgent::run(&n.state, &n.wing, &n.engine, &n.cfg, &n.req, &n.emp, &m_fwd);
        let trim_p = crate::agents::trim_authority::TrimAuthorityAgent::run(
            &n.cfg, &n.wing, &n.emp, &wb_p, &n.state, &n.engine, &n.req, 0.0,
        );
        let fwd_limit_p_esperado = trim_p.flare_limit_pct_mac.max(trim_p.rotation_limit_pct_mac);
        println!("fwd_limit_p_esperado = {fwd_limit_p_esperado:.4}  fwd_nominal = {:.4}",
            n.wb.spec.cg_limit_fwd_pct_mac);

        let out = RobustnessAgent::run(
            &n.cfg, &n.engine, &n.req, &n.state, &n.wing, &n.emp, &n.masses, &n.wb, &n.gear,
            &n.propeller, &n.mission, &n.perf,
        );
        println!("flips={:?}", out.flips);

        let flips_dianteiro: Vec<_> = out.flips.iter()
            .filter(|f| f.check == "Cenário '4 pax sem bagagem'" && f.caso == "dianteiro")
            .collect();
        assert_eq!(flips_dianteiro.len(), 1,
            "esperava exatamente 1 flip Cenário '4 pax sem bagagem'/dianteiro: {:?}", out.flips);
        let flip = flips_dianteiro[0];

        assert!(flip.valor < flip.limite,
            "cg sob o conjunto dianteiro ({:.3}%) deveria estar À FRENTE do limite dianteiro do \
             mundo perturbado ({:.3}%) — é isso que caracteriza o flip", flip.valor, flip.limite);

        // O CORAÇÃO do teste (F1): `limite` é a régua DO MUNDO perturbado,
        // NÃO a nominal — as duas divergem, e por uma margem grande o
        // bastante (>0.05pp) para não ser ruído de ponto flutuante.
        assert!((flip.limite - flip.limite_nominal).abs() > 0.05,
            "flip.limite ({:.4}) e flip.limite_nominal ({:.4}) deveriam DIVERGIR nesta fixture \
             (a régua do mundo dianteiro recalculada != a régua nominal) — se convergiram, a \
             fixture não exercita mais a reprojeção de `evaluate_world`",
            flip.limite, flip.limite_nominal);

        // `limite` bate com a régua reconstruída independentemente (o
        // mundo, não a nominal).
        assert!((flip.limite - fwd_limit_p_esperado).abs() < 1e-6,
            "flip.limite ({:.6}) deveria bater com a régua do mundo dianteiro reconstruída \
             independentemente ({:.6})", flip.limite, fwd_limit_p_esperado);

        // `limite_nominal` bate com a régua NOMINAL (`wb_nominal.spec.
        // cg_limit_fwd_pct_mac`, o parâmetro `wb_nominal` de `evaluate_world`)
        // — não com a régua do mundo.
        assert!((flip.limite_nominal - n.wb.spec.cg_limit_fwd_pct_mac).abs() < 1e-9,
            "flip.limite_nominal ({:.6}) deveria bater com o limite dianteiro NOMINAL ({:.6})",
            flip.limite_nominal, n.wb.spec.cg_limit_fwd_pct_mac);
    }
}
