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
use aeronave::agents::vn_diagram::VnDiagramAgent;
use aeronave::agents::weight_balance::mac_spanwise_pos;
use aeronave::models::config::{load_aircraft, load_engine, load_mission};
use aeronave::models::specs::{
    AircraftReport, GeometrySpec, SizingReport, SCHEMA_VERSION,
};
use aeronave::orchestrator::size_aircraft;
use aeronave::validation::constraint_checker::ConstraintChecker;
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

    let propeller = PropellerAgent::run(&cfg, &engine, prop, &req);
    let cs = ControlSurfacesAgent::run(wing, emp, &cfg);

    let mass_light_kg = wb.scenarios.iter()
        .map(|s| s.total_mass_kg)
        .fold(f64::INFINITY, f64::min);
    let vn = VnDiagramAgent::run(
        wing, envelope_mtow_kg, mass_light_kg, &req, &cfg.structure.design_category,
    );

    let perf = PerformanceAgent::run(state, wing, prop, design_mtow_kg, &engine, &req,
                                      &cfg.performance);

    let wing_mass_kg = cfg.masses.item_mass("asa").unwrap();
    let struc = StructuralAgent::run(wing, envelope_mtow_kg, wing_mass_kg, &req, &cfg.structure, vn.n_design);

    let x_cg_aft = cfg.wing.le_root_x_m + wb.spec.cg_mac_aft_pct / 100.0 * wb.mac_m;
    let mass_main_total = cfg.masses.item_mass("trem_principal").unwrap();
    let mass_nose = cfg.masses.item_mass("trem_nariz").unwrap();
    let gear = LandingGearAgent::run(envelope_mtow_kg, x_cg_aft, &cfg.gear, mass_main_total, mass_nose);

    let electrical = ElectricalAgent::run(&cfg);

    let report = ConstraintChecker::verify(&req, wing, prop, design_mtow_kg, &engine, wb,
                                            &propeller, &perf, mission, &electrical);
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
        performance: Some(perf),
        vn_diagram: Some(vn.clone()),
        structure: Some(struc),
        landing_gear: Some(gear),
        propeller: Some(propeller),
        mission: Some(mission.clone()),
        electrical: Some(electrical.clone()),
        sizing: Some(sizing),
        fidelity,
        violations: report.violations,
        warnings: report.warnings,
    }
}

#[test]
fn schema_version_e_15_blocos_de_topo_presentes() {
    let report = build_baseline_report();
    assert_eq!(report.schema_version, "4.0");
    assert_eq!(report.schema_version, SCHEMA_VERSION);

    let json = serde_json::to_string_pretty(&report).expect("deveria serializar");
    let value: serde_json::Value = serde_json::from_str(&json).expect("deveria parsear como JSON");
    let obj = value.as_object().expect("raiz deveria ser um objeto JSON");

    let expected_keys = [
        "schema_version", "revision", "validation_status", "wing", "propulsion",
        "geometry", "empennage", "control_surfaces", "weight", "performance",
        "vn_diagram", "structure", "landing_gear", "propeller", "mission",
        "electrical", "sizing", "fidelity", "violations", "warnings",
    ];
    assert!(expected_keys.len() >= 15, "lista de chaves esperadas deveria ter pelo menos 15 entradas");
    for key in expected_keys {
        assert!(obj.contains_key(key), "chave de topo ausente no JSON: '{key}'");
    }
    assert_eq!(obj.get("schema_version").unwrap().as_str().unwrap(), "4.0");
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
