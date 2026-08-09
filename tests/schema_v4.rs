//! Teste de integração: contrato JSON versionado `AircraftReport` v4 (Task 6.1).
//!
//! Roda o pipeline completo (`size_aircraft` + todos os agentes), exatamente
//! como `main.rs`, monta o `AircraftReport` final e verifica:
//!   1. `schema_version == "4.0"` (a constante `SCHEMA_VERSION`).
//!   2. Todos os blocos de topo esperados estão presentes no JSON gerado
//!      (contrato mínimo com o time de CAD).
//!   3. `warnings` não está vazio (o baseline real tem um aviso conhecido de
//!      pico elétrico — ver `validation::constraint_checker`, item 15).
//!   4. `fidelity` não está vazio (mapa de honestidade por bloco).
//!   5. Round-trip serde: serializar → desserializar → campos-chave batem.

use std::path::PathBuf;

use aeronave::agents::control_surfaces::ControlSurfacesAgent;
use aeronave::agents::electrical::ElectricalAgent;
use aeronave::agents::landing_gear::LandingGearAgent;
use aeronave::agents::performance::PerformanceAgent;
use aeronave::agents::propeller::PropellerAgent;
use aeronave::agents::structural::StructuralAgent;
use aeronave::agents::trim_authority::TrimAuthorityAgent;
use aeronave::agents::vn_diagram::VnDiagramAgent;
use aeronave::agents::weight_balance::mac_spanwise_pos;
use aeronave::models::config::{load_aircraft, load_engine, load_mission};
use aeronave::models::specs::{
    AircraftReport, GeometrySpec, SizingReport, SCHEMA_VERSION,
};
use aeronave::orchestrator::size_aircraft;
use aeronave::validation::constraint_checker::{ConstraintChecker, VerifyInputs};
use aeronave::validation::robustness::RobustnessAgent;
use std::collections::BTreeMap;

fn config_path(rel: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(rel)
}

/// Monta o `AircraftReport` completo a partir da aeronave-base real
/// (`config/aircraft/baseline_4seat.toml` + Toyota 1GD-FTV + missão
/// default) — mesma sequência de agentes de `main.rs`.
fn build_baseline_report() -> AircraftReport {
    let cfg = load_aircraft(&config_path("config/aircraft/baseline_4seat.toml")).unwrap();
    let engine = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let req = load_mission(&config_path("config/missions/default.toml")).unwrap();

    let sized = size_aircraft(&cfg, &engine, &req)
        .expect("aeronave-base + Toyota deveria convergir com o tanque de 260 L");

    let design_mtow_kg = sized.state.mtow_kg;
    let envelope_mtow_kg = sized.wb.spec.mtow_kg;
    let state = &sized.state;
    let wing = &sized.wing;
    let prop = &sized.prop;
    let mission = &sized.mission;
    let wb = &sized.wb;
    let emp = &sized.emp;
    // task trim-authority: já aplicado a `sized.wb` por `size_aircraft`
    // (`WeightBalanceOutput::apply_trim`) — recalcula aqui só para popular
    // o bloco `trim` do relatório, mesma sequência de `main.rs`.
    let trim = TrimAuthorityAgent::run(&cfg, wing, emp, wb);

    let mut propeller = PropellerAgent::run(&cfg, &engine, prop, &req);
    let cs = ControlSurfacesAgent::run(wing, emp, &cfg);

    let mass_light_kg = wb.scenarios.iter()
        .map(|s| s.total_mass_kg)
        .fold(f64::INFINITY, f64::min);
    let vn = VnDiagramAgent::run(
        wing, envelope_mtow_kg, mass_light_kg, &req, &cfg.structure.design_category,
    );

    let perf = PerformanceAgent::run(state, wing, prop, design_mtow_kg, &engine, &req,
                                      &cfg.performance);

    // Ciclo 3 (oew-parametrico): massas estruturais COMPUTADAS
    // (`agents::mass_model` via `SizedAircraft::structural_masses`) —
    // mesma fiação de `main.rs`, não mais itens fixos de
    // `[[masses.items]]`.
    let wing_mass_kg = sized.structural_masses.asa_kg;
    let struc = StructuralAgent::run(wing, envelope_mtow_kg, wing_mass_kg, &req, &cfg.structure, vn.n_design);

    let x_cg_fwd = cfg.wing.le_root_x_m + wb.spec.cg_mac_fwd_pct / 100.0 * wb.mac_m;
    let x_cg_aft = cfg.wing.le_root_x_m + wb.spec.cg_mac_aft_pct / 100.0 * wb.mac_m;
    let mass_main_total = sized.structural_masses.trem_principal_kg;
    let mass_nose = sized.structural_masses.trem_nariz_kg;
    let gear = LandingGearAgent::run(envelope_mtow_kg, x_cg_fwd, x_cg_aft, &cfg.gear, mass_main_total, mass_nose);

    // Ciclo 8 (task 2): preenche `prop_clearance_critical_m` (checagem #25)
    // no MESMO caminho de `main.rs` — depois que `gear` existe.
    propeller.fill_critical_clearance(&gear, &cfg.gear, &cfg.propeller);

    let electrical = ElectricalAgent::run(&cfg);

    // Ciclo 4 (task robustez, wiring): `RobustnessSpec` na MESMA sequência
    // de `main.rs` — logo após o `LandingGearAgent`, contra os limites
    // NOMINAIS já calculados (`wb`/`gear`).
    let robustness = RobustnessAgent::run(&cfg, &engine, &req, state, wing, emp,
                                           &sized.structural_masses, wb, &gear, &propeller,
                                           mission, &perf);

    let report = ConstraintChecker::verify(&VerifyInputs {
        req: &req, wing, prop, mtow_kg: design_mtow_kg, engine: &engine, wb,
        propeller: &propeller, perf: &perf, mission, electrical: &electrical,
        gear: &gear, gear_cfg: &cfg.gear, fuel_capacity_l: cfg.fuel_system.capacity_l,
        robustness: &robustness, prop_cfg: &cfg.propeller,
    });
    let all_ok = report.all_satisfied();

    let geometry = GeometrySpec {
        wing_le_root_x_m: cfg.wing.le_root_x_m,
        chord_root_m: wb.chord_root_m,
        chord_tip_m: wb.chord_tip_m,
        mac_m: wb.mac_m,
        mac_le_x_m: wb.mac_le_x_m,
        y_mac_m: mac_spanwise_pos(wing.span_m, wing.taper_ratio),
        fuselage_length_m: cfg.fuselage.length_m,
        cabin_width_m: cfg.fuselage.cabin_width_m,
        cabin_height_m: cfg.fuselage.cabin_height_m,
    };

    let fuel_margin_l = cfg.fuel_system.capacity_l - mission.fuel_total_l;
    let sizing = SizingReport {
        mtow_mission_kg: design_mtow_kg,
        mtow_envelope_kg: envelope_mtow_kg,
        iterations: sized.iterations.clone(),
        converged: true,
        fuel_required_l: mission.fuel_total_l,
        fuel_capacity_l: cfg.fuel_system.capacity_l,
        fuel_margin_l,
        fuel_margin_pct: fuel_margin_l / cfg.fuel_system.capacity_l * 100.0,
        constraints: sized.constraints.clone(),
    };

    let mut fidelity: BTreeMap<String, String> = BTreeMap::new();
    fidelity.insert("wing".into(), "semi-empirical (polar por build-up)".into());
    fidelity.insert("propulsion".into(), "semi-empirical (curvas de catálogo + BSFC paramétrico)".into());
    fidelity.insert("structure".into(), "preliminary (vigas simplificadas; requer FEM); flutter preliminary — requer GVT".into());
    fidelity.insert("mission".into(), "computed (segmentos + Breguet L/D constante)".into());
    fidelity.insert("empennage".into(), "preliminary (coeficiente de volume; requer VLM/CFD)".into());
    fidelity.insert("trim".into(), "preliminary (semi-empírico; sensível a cl_h_max_down)".into());
    fidelity.insert("robustness".into(),
        "computed (pior-caso determinístico ±σ direcional sobre as 7 massas estruturais; \
         limites de envelope nominais — invariantes a massa)".into());

    AircraftReport {
        schema_version: SCHEMA_VERSION.to_string(),
        revision: SCHEMA_VERSION.to_string(),
        validation_status: if all_ok { "PASS".to_string() } else { "FAIL".to_string() },
        wing: wing.clone(),
        propulsion: prop.clone(),
        geometry: Some(geometry),
        empennage: Some(emp.clone()),
        control_surfaces: Some(cs.clone()),
        weight: Some(wb.spec.clone()),
        trim: Some(trim),
        performance: Some(perf),
        vn_diagram: Some(vn.clone()),
        structure: Some(struc),
        landing_gear: Some(gear),
        propeller: Some(propeller),
        mission: Some(mission.clone()),
        electrical: Some(electrical.clone()),
        sizing: Some(sizing),
        robustness: Some(robustness),
        fidelity,
        violations: report.violations,
        warnings: report.warnings,
    }
}

#[test]
fn schema_version_e_16_blocos_de_topo_presentes() {
    let report = build_baseline_report();
    assert_eq!(report.schema_version, "5.2");
    assert_eq!(report.schema_version, SCHEMA_VERSION);

    let json = serde_json::to_string_pretty(&report).expect("deveria serializar");
    let value: serde_json::Value = serde_json::from_str(&json).expect("deveria parsear como JSON");
    let obj = value.as_object().expect("raiz deveria ser um objeto JSON");

    let expected_keys = [
        "schema_version", "revision", "validation_status", "wing", "propulsion",
        "geometry", "empennage", "control_surfaces", "weight", "trim", "performance",
        "vn_diagram", "structure", "landing_gear", "propeller", "mission",
        "electrical", "sizing", "robustness", "fidelity", "violations", "warnings",
    ];
    assert!(expected_keys.len() >= 16, "lista de chaves esperadas deveria ter pelo menos 16 entradas");
    for key in expected_keys {
        assert!(obj.contains_key(key), "chave de topo ausente no JSON: '{key}'");
    }
    assert_eq!(obj.get("schema_version").unwrap().as_str().unwrap(), "5.2");
}

/// Schema 5.0 (Task 2, ciclo7-clmax-decolagem — bump MAJOR): `wing.cl_max_to`
/// (NOVO, derivado na Task 1 do mesmo ciclo) é numericamente ENTRE
/// `cl_max_clean` e `cl_max_flaps` — consistente com sua definição de
/// interpolação linear pela fração de deployment do flap de decolagem,
/// `cl_max_to = cl_max_clean + to_flap_fraction·(cl_max_flaps −
/// cl_max_clean)` com `0 < to_flap_fraction < 1`. `cl_max_flaps` não é
/// ecoado no JSON (só o `cl_max` de pouso, que é o mesmo valor internamente
/// — ver `WingSpec::cl_max`), então o teste usa `cl_max` como o teto de
/// pouso equivalente. O campo `trim.to_flap_fraction` (RENOMEADO de
/// `to_flap_cm_fraction` na Task 1 — motivo do bump MAJOR, não MINOR: a
/// política de versionamento do schema, `SCHEMA_VERSION`/§1 deste
/// documento, classifica renome de campo serializado como mudança que
/// QUEBRA compatibilidade) está presente, e o nome ANTIGO
/// `to_flap_cm_fraction` NÃO aparece mais em lugar nenhum do JSON.
#[test]
fn wing_cl_max_to_entre_clean_e_flaps_trim_to_flap_fraction_renomeado() {
    let report = build_baseline_report();

    assert!(
        report.wing.cl_max_to > report.wing.cl_max_clean
            && report.wing.cl_max_to < report.wing.cl_max,
        "wing.cl_max_to ({}) deveria ficar estritamente entre cl_max_clean ({}) e \
         cl_max/cl_max_flaps de pouso ({})",
        report.wing.cl_max_to, report.wing.cl_max_clean, report.wing.cl_max,
    );

    let json = serde_json::to_string(&report).expect("deveria serializar");
    let value: serde_json::Value = serde_json::from_str(&json).expect("deveria parsear como JSON");
    let trim = &value["trim"];
    assert!(
        trim.get("to_flap_fraction").and_then(|v| v.as_f64()).is_some(),
        "trim.to_flap_fraction deveria estar presente e ser numérico no JSON"
    );
    assert!(
        trim.get("to_flap_cm_fraction").is_none(),
        "trim.to_flap_cm_fraction (nome ANTIGO, renomeado na Task 1) não deveria mais \
         aparecer no JSON — o campo é to_flap_fraction"
    );
    assert!(
        !json.contains("to_flap_cm_fraction"),
        "o JSON completo não deveria conter nenhuma ocorrência do nome antigo \
         'to_flap_cm_fraction'"
    );
}

/// Ciclo 8 (task 1, arrasto de flap na polar — introduzido ainda dentro de
/// v5.0; o bump formal para v5.1 foi concluído na Task 3 do mesmo ciclo,
/// ver `docs/aircraft_spec.schema.md`): `wing.cd0_flap_to_extra` (NOVO) está
/// presente e numérico no JSON, e bate com a fórmula fechada
/// `to_flap_fraction · cd0_flap_delta` — mesmo precedente de
/// `wing_cl_max_to_entre_clean_e_flaps_trim_to_flap_fraction_renomeado`
/// acima para `cl_max_to`. `cd0_flap_delta` em si não é ecoado no JSON (só
/// o produto derivado), então o teste recomputa a partir da config do
/// baseline real.
#[test]
fn wing_cd0_flap_to_extra_presente_e_bate_com_formula_fechada() {
    let cfg = load_aircraft(&config_path("config/aircraft/baseline_4seat.toml")).unwrap();
    let report = build_baseline_report();

    let json = serde_json::to_string(&report).expect("deveria serializar");
    let value: serde_json::Value = serde_json::from_str(&json).expect("deveria parsear como JSON");
    let cd0_flap_to_extra_json = value["wing"].get("cd0_flap_to_extra")
        .and_then(|v| v.as_f64());
    assert!(
        cd0_flap_to_extra_json.is_some(),
        "wing.cd0_flap_to_extra deveria estar presente e ser numérico no JSON"
    );

    let esperado = cfg.stability.to_flap_fraction * cfg.wing.cd0_flap_delta;
    let obtido = cd0_flap_to_extra_json.unwrap();
    assert!(
        (obtido - esperado).abs() < 1e-9,
        "wing.cd0_flap_to_extra no JSON ({obtido:.9}) deveria bater com a fórmula fechada \
         to_flap_fraction·cd0_flap_delta ({esperado:.9})"
    );
    assert!(
        obtido > 0.0 && obtido < cfg.wing.cd0_flap_delta,
        "wing.cd0_flap_to_extra ({obtido:.6}) deveria ficar ESTRITAMENTE entre 0 e o delta \
         cheio de pouso ({:.6})", cfg.wing.cd0_flap_delta
    );
}

/// Schema 5.1 (Task 3, ciclo8-flap-e-solo — bump formal MINOR que
/// formaliza os dois campos aditivos das Tasks 1/2 do mesmo ciclo, ver
/// `docs/aircraft_spec.schema.md` §1): `propeller.prop_clearance_critical_m`
/// (NOVO, ciclo 8 task 2 — folga ponta de pá ↔ solo na condição CRÍTICA de
/// CS 23.925, checagem #25) está presente e numérico no JSON. Sem fórmula
/// fechada independente aqui (o campo já depende de agentes distintos
/// rodando em sequência — `PropellerAgent` + `LandingGearAgent`/
/// `PropellerSpec::fill_critical_clearance` — reproduzir a fórmula neste
/// teste duplicaria a lógica do pipeline sem adicionar cobertura; a fórmula
/// fechada já é coberta por
/// `models::specs::tests::fill_critical_clearance_bate_com_a_formula_fechada`).
///
/// ATUALIZAÇÃO (ciclo 9, transferência de atitude do #25 — old→new):
/// Baseline real E10 ≈+0,0325 m (PASS, simplificação 1:1) → **≈−0,06416 m
/// (FAIL)** — a fórmula ganha o fator de amplificação do pivô sobre o trem
/// principal (`(x_main−prop_plane_x_m)/(x_main−x_nose_m)` ≈ 1,46610 nesta
/// geometria), física corrigida do achado de review do ciclo 8
/// (`docs/backlog.md`, item 1). Nenhuma tolerância afrouxada — o pin
/// (±0,001) é o mesmo padrão de antes, só o valor central mudou.
///
/// ATUALIZAÇÃO (ciclo 10, task 1, deflexão estática — old→new): campo
/// novo `[gear].static_sag_fraction` corrige uma dupla contagem da
/// compressão estática do nariz (curso TOTAL → curso RESTANTE, ver
/// docstring de `GearCfg::static_sag_fraction`). Baseline real E10
/// **≈−0,06416 m (ciclo 9) → ≈−0,00249 m (ciclo 10)** — MESMO veredito
/// (checagem #25 continua FAIL), só o número muda, honestamente
/// ANTI-conservador. `fator` (≈1,46610) inalterado. Ver
/// `docs/backlog.md` (item 6, RESOLVIDO ciclo 10).
#[test]
fn propeller_prop_clearance_critical_m_presente_e_numerico_proximo_do_esperado() {
    let report = build_baseline_report();

    let json = serde_json::to_string(&report).expect("deveria serializar");
    let value: serde_json::Value = serde_json::from_str(&json).expect("deveria parsear como JSON");
    let obtido = value["propeller"].get("prop_clearance_critical_m")
        .and_then(|v| v.as_f64());
    assert!(
        obtido.is_some(),
        "propeller.prop_clearance_critical_m deveria estar presente e ser numérico no JSON"
    );
    let obtido = obtido.unwrap();
    assert!(
        (obtido - (-0.00249)).abs() < 0.001,
        "propeller.prop_clearance_critical_m ({obtido:.6}) deveria ficar próximo de ≈−0,00249 m \
         (baseline real E10 pós-ciclo-10, checagem #25 FAIL — old: ≈−0,06416 m ciclo 9)"
    );
    assert!(obtido < 0.0,
        "campanha ciclo 10: baseline real deveria continuar REPROVANDO a checagem #25 (folga \
         crítica negativa, mesmo veredito do ciclo 9 — só o número da violação muda)");
}

/// Schema 4.6 (Task 4, ciclo4-fidelidade-massas — check #19): o bloco
/// `robustness` (`RobustnessSpec`) está presente no JSON e traz
/// `sigma_mass_fraction` (eco de `[mass_model].sigma_mass_fraction`) e
/// `flips` como array (vazio ou não — o baseline real, σ=15%, não produz
/// nenhum flip, ver `tests/gear_tipback.rs`/`tests/cli.rs` para o achado
/// honesto completo). Ciclo 5 (task massa-total): `mtow_masstotal_kg`
/// também presente e, no baseline real (sem flip de Dimensionamento),
/// estritamente MAIOR que o MTOW de missão nominal
/// (`sizing.mtow_mission_kg`) — os 5 fatores de composto só multiplicam
/// por (1+σ) > 1. Schema 4.7 (Task 4, ciclo5-robustez-total-e-solo): o
/// bump de versão que formaliza `mtow_masstotal_kg` (e `electrical.loads`,
/// ver teste dedicado abaixo) como parte do contrato. Schema 4.8 (Task 4,
/// ciclo6-pista-e-robustez-final): NENHUM campo novo neste bloco — o mundo
/// "massa-total" passa a avaliar TAMBÉM pista (#23/#24) e envelope/nariz/
/// tipback (não só os gates de desempenho que já existiam), mas isso é
/// comportamento do `RobustnessAgent`/`ConstraintChecker`, não uma mudança
/// de forma do JSON; o bump formaliza o requisito `runway_available_m` e as
/// checagens #23/#24 (`ConstraintChecker::verify`) como parte do contrato
/// v4 — ver `docs/aircraft_spec.schema.md` §1.
#[test]
fn robustness_presente_com_sigma_e_flips_array() {
    let report = build_baseline_report();
    let robustness = report.robustness.as_ref().expect("robustness deveria estar presente");
    assert!(robustness.sigma_mass_fraction > 0.0,
        "sigma_mass_fraction deveria ser positivo, obteve {}", robustness.sigma_mass_fraction);
    let sizing = report.sizing.as_ref().expect("sizing deveria estar presente");
    assert!(robustness.mtow_masstotal_kg > sizing.mtow_mission_kg,
        "achado honesto (baseline real, sem flip de Dimensionamento): mtow_masstotal_kg ({:.2}) \
         deveria ficar ACIMA do MTOW de missão nominal ({:.2})",
        robustness.mtow_masstotal_kg, sizing.mtow_mission_kg);

    let json = serde_json::to_string(&report).expect("deveria serializar");
    let value: serde_json::Value = serde_json::from_str(&json).expect("deveria parsear como JSON");
    let rob = &value["robustness"];
    assert!(rob["sigma_mass_fraction"].is_number(),
        "robustness.sigma_mass_fraction deveria estar presente e ser numérico no JSON");
    assert!(rob["flips"].is_array(), "robustness.flips deveria ser um array no JSON");
    assert!(rob["cg_fwd_case_pct_mac"].is_array(), "robustness.cg_fwd_case_pct_mac deveria ser um array no JSON");
    assert!(rob["cg_aft_case_pct_mac"].is_array(), "robustness.cg_aft_case_pct_mac deveria ser um array no JSON");
    assert!(rob["mtow_masstotal_kg"].is_number(),
        "robustness.mtow_masstotal_kg deveria estar presente e ser numérico no JSON");
}

/// Schema 4.5 (Task 5, oew-parametrico): `weight.structural_masses` —
/// as 7 massas estruturais COMPUTADAS (`agents::mass_model`) + os 5
/// fatores de composto usados (`[mass_model]`), rastreáveis no JSON
/// final (antes só disponíveis internamente via `SizedAircraft::
/// structural_masses`, nunca ecoadas dentro do bloco `weight`).
#[test]
fn weight_structural_masses_presente_e_positivo() {
    let report = build_baseline_report();
    let weight = report.weight.as_ref().expect("weight deveria estar presente");
    let sm = &weight.structural_masses;

    assert!(sm.asa_kg > 0.0, "asa_kg deveria ser positivo, obteve {}", sm.asa_kg);
    assert!(sm.fuselagem_kg > 0.0, "fuselagem_kg deveria ser positivo, obteve {}", sm.fuselagem_kg);
    assert!(sm.emp_h_kg > 0.0, "emp_h_kg deveria ser positivo, obteve {}", sm.emp_h_kg);
    assert!(sm.emp_v_kg > 0.0, "emp_v_kg deveria ser positivo, obteve {}", sm.emp_v_kg);
    assert!(sm.trem_principal_kg > 0.0, "trem_principal_kg deveria ser positivo, obteve {}", sm.trem_principal_kg);
    assert!(sm.trem_nariz_kg > 0.0, "trem_nariz_kg deveria ser positivo, obteve {}", sm.trem_nariz_kg);
    assert!(sm.tanques_kg > 0.0, "tanques_kg deveria ser positivo, obteve {}", sm.tanques_kg);
    assert!(sm.composite_factor_wing > 0.0,
        "composite_factor_wing deveria ser positivo, obteve {}", sm.composite_factor_wing);

    // Rastreabilidade: as massas ecoadas em `weight.structural_masses` são
    // EXATAMENTE as mesmas que entraram no OEW (`SizedAircraft::
    // structural_masses`), não uma cópia recomputada independentemente.
    let json = serde_json::to_string(&report).expect("deveria serializar");
    assert!(json.contains("\"structural_masses\""),
        "JSON deveria conter a chave 'structural_masses' dentro de 'weight'");
}

/// Schema 4.7 (Task 4, ciclo5-robustez-total-e-solo — check #20):
/// `electrical.loads` (`Vec<ElectricalLoadSpec>`) ecoa individualmente
/// cada `[electrical].loads` configurada — nome, potência contínua e
/// potência de pico — para que `ConstraintChecker::verify` compare o pico
/// DECLARADO da carga 'trem_retratil' contra `landing_gear.
/// actuator_power_w` COMPUTADO (checagem só possível pós-convergência).
#[test]
fn electrical_loads_presente_nao_vazio_com_name_e_peak_w() {
    let report = build_baseline_report();
    let electrical = report.electrical.as_ref().expect("electrical deveria estar presente");
    assert!(!electrical.loads.is_empty(),
        "electrical.loads deveria ser um array NÃO-vazio (cargas configuradas do baseline)");
    for load in &electrical.loads {
        assert!(!load.name.is_empty(), "cada carga elétrica deveria ter um 'name' não-vazio");
        assert!(load.peak_w > 0.0, "carga '{}' deveria ter peak_w positivo, obteve {}", load.name, load.peak_w);
    }

    let json = serde_json::to_string(&report).expect("deveria serializar");
    let value: serde_json::Value = serde_json::from_str(&json).expect("deveria parsear como JSON");
    let loads = value["electrical"]["loads"].as_array()
        .expect("electrical.loads deveria ser um array no JSON");
    assert!(!loads.is_empty(), "electrical.loads não deveria estar vazio no JSON");
    for load in loads {
        assert!(load["name"].is_string(), "cada item de electrical.loads deveria ter 'name' string");
        assert!(load["peak_w"].is_number(), "cada item de electrical.loads deveria ter 'peak_w' numérico");
    }
}

#[test]
fn warnings_do_baseline_contem_aviso_de_pico_eletrico() {
    let report = build_baseline_report();
    assert!(!report.warnings.is_empty(), "baseline deveria ter ao menos um warning (pico elétrico)");
    assert!(
        report.warnings.iter().any(|w| w.contains("pico")),
        "esperava aviso mencionando 'pico' (elétrico), obteve: {:?}", report.warnings
    );
}

#[test]
fn fidelity_map_nao_vazio_e_contem_blocos_chave() {
    let report = build_baseline_report();
    assert!(!report.fidelity.is_empty(), "mapa de fidelidade não deveria estar vazio");
    for key in ["wing", "propulsion", "structure", "mission"] {
        assert!(report.fidelity.contains_key(key), "fidelity deveria conter a chave '{key}'");
    }
}

#[test]
fn sizing_report_reflete_mtow_missao_e_envelope() {
    let report = build_baseline_report();
    let sizing = report.sizing.as_ref().expect("sizing deveria estar presente");
    assert!(sizing.mtow_mission_kg > 0.0);
    assert!(sizing.mtow_envelope_kg >= sizing.mtow_mission_kg,
        "MTOW envelope (pior caso legal) deveria ser >= MTOW de missão");
    assert!(sizing.converged);
    assert!(sizing.iterations.len() >= 2);
    assert!(sizing.fuel_margin_l >= 0.0, "baseline deveria convergir com margem de combustível não-negativa");
}

#[test]
fn round_trip_serde_preserva_campos_chave() {
    let report = build_baseline_report();
    let json = serde_json::to_string(&report).expect("deveria serializar");
    let back: AircraftReport = serde_json::from_str(&json).expect("deveria desserializar de volta");

    // Cobertura do OBJETO INTEIRO (achado da revisão: os asserts abaixo só
    // comparavam alguns campos escolhidos a dedo — control_surfaces,
    // propeller, landing_gear, performance e vn_diagram nunca eram
    // checados). Reserializar `back` e comparar a string JSON byte a byte
    // contra a original cobre TODOS os blocos de uma vez — só é exato
    // graças à feature `float_roundtrip` do serde_json (Cargo.toml),
    // sem a qual esta asserção falharia por ruído de último-bit em
    // pontos flutuantes mesmo com dados logicamente idênticos.
    let json2 = serde_json::to_string(&back).expect("deveria reserializar");
    assert_eq!(json, json2,
        "reserializar o AircraftReport desserializado deveria produzir o \
         MESMO JSON byte a byte (round-trip completo, todos os blocos)");

    assert_eq!(back.schema_version, report.schema_version);
    assert_eq!(back.revision, report.revision);
    assert_eq!(back.validation_status, report.validation_status);
    assert_eq!(back.wing.span_m, report.wing.span_m);
    assert_eq!(back.propulsion.engine_model, report.propulsion.engine_model);
    assert_eq!(back.violations.len(), report.violations.len());
    assert_eq!(back.warnings.len(), report.warnings.len());
    assert_eq!(back.fidelity.len(), report.fidelity.len());

    let g_before = report.geometry.as_ref().unwrap();
    let g_after = back.geometry.as_ref().unwrap();
    assert_eq!(g_before.mac_m, g_after.mac_m);
    assert_eq!(g_before.fuselage_length_m, g_after.fuselage_length_m);

    let s_before = report.sizing.as_ref().unwrap();
    let s_after = back.sizing.as_ref().unwrap();
    assert_eq!(s_before.mtow_mission_kg, s_after.mtow_mission_kg);
    assert_eq!(s_before.iterations, s_after.iterations);
    assert_eq!(s_before.constraints.ws_actual_n_m2, s_after.constraints.ws_actual_n_m2);
}

/// Achado da própria checagem de round-trip acima (Task 6.1):
/// `StructuralSpec::fatigue_life_cycles` pode ser `f64::INFINITY`
/// (fisicamente correto — "vida infinita" abaixo do limite de fadiga).
/// `serde_json` serializa `Infinity` como `null` por padrão, e `null` não
/// desserializa de volta em `f64` — um consumidor de CAD rodando o schema
/// oficial quebraria sempre que a longarina caísse abaixo do limite de
/// fadiga. Confirma que o campo agora serializa como a string `"infinita"`,
/// não `null`, e volta corretamente para `f64::INFINITY`.
#[test]
fn fatigue_life_infinita_serializa_como_string_nao_null_e_faz_round_trip() {
    let report = build_baseline_report();
    let struc = report.structure.as_ref().expect("structure deveria estar presente");
    assert!(struc.fatigue_life_cycles.is_infinite(),
        "baseline real deveria ter vida em fadiga infinita (abaixo do limite Se) — \
         obteve {:.3e}; se isso mudou legitimamente, ajustar este teste",
        struc.fatigue_life_cycles);

    let json = serde_json::to_string(&report).expect("deveria serializar");
    assert!(json.contains("\"fatigue_life_cycles\":\"infinita\""),
        "esperava fatigue_life_cycles serializado como a string \"infinita\", \
         não null nem um número");
    assert!(!json.contains("\"fatigue_life_cycles\":null"),
        "fatigue_life_cycles NUNCA deveria serializar como null (não desserializa de volta em f64)");

    let back: AircraftReport = serde_json::from_str(&json).expect("deveria desserializar de volta");
    assert!(back.structure.unwrap().fatigue_life_cycles.is_infinite());
}
