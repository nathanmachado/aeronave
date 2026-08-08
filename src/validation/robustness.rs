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
//! Os limites contra os quais os conjuntos perturbados são avaliados são
//! os NOMINAIS (`wb_nominal.spec.cg_limit_{fwd,aft}_pct_mac`,
//! `gear_cfg.tipback_min_deg`, os tetos/pisos de carga de nariz de
//! `validation::constraint_checker`) — esses limites são derivados de
//! geometria/margem de estabilidade/autoridade de profundor, não da massa
//! estrutural em si (a massa entra no CG e nas cargas, não no limite),
//! logo são invariantes à perturbação: reavaliá-los para cada conjunto
//! adversarial recalcularia o MESMO número (o `TrimAuthorityAgent`
//! dianteiro/`sm_min` traseiro não dependem de `StructuralMasses`) a um
//! custo de mais uma chamada de agente — por isso as massas perturbadas
//! são comparadas numericamente contra os limites NOMINAIS já calculados,
//! sem re-rodar `TrimAuthorityAgent`.
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
    EmpennageSpec, GearSpec, MissionSpec, PerformanceSpec, RobustnessFlip, RobustnessSpec,
    WingSpec,
};
use crate::orchestrator::SizingError;
use crate::validation::constraint_checker::{NOSE_LOAD_MAX_CEILING_PCT, NOSE_LOAD_MIN_FLOOR_PCT};

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

pub struct RobustnessAgent;

impl RobustnessAgent {
    /// Avalia os dois conjuntos adversariais (`adversarial_masses`, com
    /// `cfg.mass_model.sigma_mass_fraction`) contra os limites NOMINAIS já
    /// calculados (`wb_nominal`/`gear_nominal` — ver docstring do módulo
    /// para o porquê de não reavaliar `TrimAuthorityAgent`). Um `flip` é
    /// registrado por (check, caso) sempre que o conjunto perturbado
    /// REPROVA um check que o NOMINAL passava.
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
        mission_nominal: &MissionSpec,
        perf_nominal: &PerformanceSpec,
    ) -> RobustnessSpec {
        debug_assert!(wb_nominal.spec.cg_limit_fwd_pct_mac.is_finite(),
            "RobustnessAgent exige um wb NOMINAL já com apply_trim (cg_limit_fwd_pct_mac = NaN)");
        debug_assert!(wb_nominal.spec.cg_limit_aft_pct_mac.is_finite(),
            "RobustnessAgent exige um wb NOMINAL já com apply_trim (cg_limit_aft_pct_mac = NaN)");
        let sigma = cfg.mass_model.sigma_mass_fraction;
        let (m_fwd, m_aft) = adversarial_masses(cfg, engine, masses, sigma);

        // Avalia um conjunto adversarial (`caso` = "dianteiro"/"traseiro")
        // contra os limites nominais: cenários de CG (envelope
        // dianteiro/traseiro) + trem de pouso (tipback, carga de nariz
        // máx/mín) — devolve a faixa de CG observada e os flips
        // encontrados.
        let evaluate_case = |caso: &str, m_p: &StructuralMasses| -> ([f64; 2], Vec<RobustnessFlip>) {
            let mut flips = Vec::new();
            let wb_p = WeightBalanceAgent::run(state, wing, engine, cfg, req, emp, m_p);

            let mut range = [f64::INFINITY, f64::NEG_INFINITY];
            for (sc_nom, sc_p) in wb_nominal.scenarios.iter().zip(wb_p.scenarios.iter()) {
                range[0] = range[0].min(sc_p.cg_pct_mac);
                range[1] = range[1].max(sc_p.cg_pct_mac);

                let dentro_do_envelope_nominal = sc_p.cg_pct_mac >= wb_nominal.spec.cg_limit_fwd_pct_mac
                    && sc_p.cg_pct_mac <= wb_nominal.spec.cg_limit_aft_pct_mac;
                if !dentro_do_envelope_nominal && sc_nom.inside_envelope {
                    let limite = if sc_p.cg_pct_mac < wb_nominal.spec.cg_limit_fwd_pct_mac {
                        wb_nominal.spec.cg_limit_fwd_pct_mac
                    } else {
                        wb_nominal.spec.cg_limit_aft_pct_mac
                    };
                    flips.push(RobustnessFlip {
                        check: format!("Cenário '{}'", sc_nom.name),
                        caso: caso.to_string(),
                        valor: sc_p.cg_pct_mac,
                        limite,
                    });
                }
            }

            let x_fwd_p = cfg.wing.le_root_x_m + wb_p.spec.cg_mac_fwd_pct / 100.0 * wb_p.mac_m;
            let x_aft_p = cfg.wing.le_root_x_m + wb_p.spec.cg_mac_aft_pct / 100.0 * wb_p.mac_m;
            let gear_p = LandingGearAgent::run(
                wb_p.spec.mtow_kg, x_fwd_p, x_aft_p, &cfg.gear,
                m_p.trem_principal_kg, m_p.trem_nariz_kg,
            );

            if gear_p.tipback_angle_deg < cfg.gear.tipback_min_deg
                && gear_nominal.tipback_angle_deg >= cfg.gear.tipback_min_deg
            {
                flips.push(RobustnessFlip {
                    check: "Tipback".to_string(),
                    caso: caso.to_string(),
                    valor: gear_p.tipback_angle_deg,
                    limite: cfg.gear.tipback_min_deg,
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
                });
            }

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
                flips.push(RobustnessFlip {
                    check: "Dimensionamento".to_string(),
                    caso: "massa-total".to_string(),
                    valor: match &e {
                        SizingError::CombustivelInsuficiente { necessario_l, .. } => *necessario_l,
                        SizingError::MtowExcedido { mtow, .. } => *mtow,
                        _ => f64::NAN,
                    },
                    limite: match &e {
                        SizingError::CombustivelInsuficiente { capacidade_l, .. } => *capacidade_l,
                        SizingError::MtowExcedido { limite, .. } => *limite,
                        _ => f64::NAN,
                    },
                });
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
                        limite: req.min_fuel_margin_fraction * 100.0 });
                }
                // VS0 (fórmula do check #2):
                let vs0_lim = req.cruise_speed_min_kmh / 1.8;
                if wing.stall_speed_flaps_kmh <= vs0_lim
                    && sized_p.wing.stall_speed_flaps_kmh > vs0_lim {
                    flips.push(RobustnessFlip { check: "VS0".into(),
                        caso: "massa-total".into(),
                        valor: sized_p.wing.stall_speed_flaps_kmh, limite: vs0_lim });
                }
                // desempenho no mundo +σ (mesmos gates do pipeline nominal):
                let perf_p = crate::agents::performance::PerformanceAgent::run(
                    &sized_p.state, &sized_p.wing, &sized_p.prop,
                    sized_p.state.mtow_kg, engine, req, &cfg.performance);
                for (nome, nom, p, lim, maior_melhor) in [
                    ("Razão de subida", perf_nominal.rc_sl_ms, perf_p.rc_sl_ms, 1.5, true),
                    ("Velocidade de cruzeiro", perf_nominal.v_cruise_kmh, perf_p.v_cruise_kmh,
                     req.cruise_speed_min_kmh, true),
                    ("Teto de serviço", perf_nominal.service_ceiling_m,
                     perf_p.service_ceiling_m, 3_000.0, true),
                ] {
                    let nom_ok = if maior_melhor { nom >= lim } else { nom <= lim };
                    let p_ok = if maior_melhor { p >= lim } else { p <= lim };
                    if nom_ok && !p_ok {
                        flips.push(RobustnessFlip { check: nome.into(),
                            caso: "massa-total".into(), valor: p, limite: lim });
                    }
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
        Nominal { cfg, engine, req, state, wing, emp, masses, wb, gear, mission, perf }
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
            &n.mission, &n.perf,
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
    /// menor) o suficiente para derrubar o check.
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
            &n.mission, &n.perf,
        );
        println!("flips={:?}", out.flips);

        assert_eq!(out.flips.len(), 1,
            "esperava exatamente 1 flip (Tipback/traseiro): {:?}", out.flips);
        let flip = &out.flips[0];
        assert_eq!(flip.check, "Tipback");
        assert_eq!(flip.caso, "traseiro");
        assert!(flip.valor < flip.limite,
            "valor ({}) deveria estar abaixo do limite ({}) — é isso que caracteriza o flip",
            flip.valor, flip.limite);
        assert!((flip.limite - n.cfg.gear.tipback_min_deg).abs() < 1e-9);
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
            &n.mission, &n.perf,
        );
        println!("flips={:?}", out.flips);
        println!("mtow_masstotal_kg={}", out.mtow_masstotal_kg);

        assert_eq!(out.mtow_masstotal_kg, 0.0,
            "sizing +σ deveria falhar — sem ponto convergido, mtow_masstotal_kg deveria ser 0.0");
        assert_eq!(out.flips.len(), 1,
            "esperava exatamente 1 flip (Dimensionamento/massa-total): {:?}", out.flips);
        let flip = &out.flips[0];
        assert_eq!(flip.check, "Dimensionamento");
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
            &n.mission, &n.perf,
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
            &n.mission, &n.perf,
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
            &n.mission, &n.perf,
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
}
