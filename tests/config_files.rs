//! Testes de carregamento de arquivos de configuração reais do disco
//! (`config/engines/*.toml`, `config/aircraft/*.toml`, `config/missions/*.toml`).
//!
//! Vivem em `tests/` (crate de teste separada), não em `src/models/config.rs`
//! — `src/` deve permanecer livre de qualquer menção a um motor real
//! específico (ver grep de regressão no relatório da Task 1.4/2.3, e
//! `tests/generic_engine.rs`). Os testes de parse/validação com TOML inline
//! (sem tocar disco, sem nomear arquivos reais) continuam em
//! `src/models/config.rs`.

use std::path::PathBuf;

use aeronave::models::config::{load_aircraft, load_engine, load_mission};

fn config_path(rel: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(rel)
}

#[test]
fn carrega_os_dois_motores_do_disco() {
    let toyota = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let rotax = load_engine(&config_path("config/engines/rotax_915is.toml")).unwrap();
    assert!((toyota.torque_nm(2_400.0) - 500.0).abs() < 1.0);
    assert!((rotax.power_kw(5_800.0) - 67.4).abs() < 3.0); // 111 Nm @ 5800 ≈ 67 kW
    assert!(toyota.fuel.density_kg_per_l < rotax.fuel.density_kg_per_l + 1.0);
}

#[test]
fn carrega_baseline_do_disco_campo_a_campo() {
    let cfg = load_aircraft(&config_path("config/aircraft/baseline_4seat.toml")).unwrap();
    assert_eq!(cfg.sizing.mtow_initial_guess_kg, 1461.0);
    assert_eq!(cfg.sizing.mtow_max_kg, 1800.0);
    assert_eq!(cfg.wing.span_m, 11.94);
    assert_eq!(cfg.wing.area_m2, 14.2);
    assert_eq!(cfg.wing.taper_ratio, 0.45);
    assert_eq!(cfg.wing.airfoil, "NACA 23015");
    assert_eq!(cfg.wing.cl_max_clean, 1.45);
    assert_eq!(cfg.wing.cl_max_flaps, 1.72);
    assert_eq!(cfg.wing.le_root_x_m, 2.90);
    assert_eq!(cfg.propeller.psru_ratio, 1.867);
    assert_eq!(cfg.propeller.diameter_m, 1.95);
    // 260 L desde a correção pós-Task-3.1 (era 240 L; ~8% de margem sobre os
    // ~241 L exigidos pela missão no MTOW convergido — ver task-3.1-report.md).
    assert_eq!(cfg.fuel_system.capacity_l, 260.0);
    assert_eq!(cfg.gear.h_cg_ground_m, 1.05);
    assert_eq!(cfg.gear.x_nose_m, 1.40);
    assert_eq!(cfg.gear.x_main_m, 3.85);
    assert_eq!(cfg.arms.engine_cg_m, 0.65);
    assert_eq!(cfg.arms.empennage_cg_m, 7.40);
    assert_eq!(cfg.structure.spar_material, "AA7075-T6");
    assert_eq!(cfg.structure.frame_spacing_mm, 300.0);
    assert_eq!(cfg.masses.items.len(), 15);
    assert_eq!(cfg.masses.item_mass("asa"), Some(130.0));
    assert_eq!(cfg.masses.item_mass("trem_principal"), Some(55.0));
}

#[test]
fn carrega_missao_default_do_disco_campo_a_campo() {
    let req = load_mission(&config_path("config/missions/default.toml")).unwrap();
    assert_eq!(req.passengers, 4);
    assert_eq!(req.pax_mass_kg, 90.0);
    assert_eq!(req.baggage_kg, 80.0);
    assert_eq!(req.cruise_speed_min_kmh, 280.0);
    assert_eq!(req.endurance_min_h, 8.0);
    assert_eq!(req.fuel_reserve_fraction, 0.10);
    assert_eq!(req.cruise_altitude_m, 2500.0);
    assert_eq!(req.airfield_altitude_m, 0.0);
    assert_eq!(req.isa_delta_c, 0.0);
}
