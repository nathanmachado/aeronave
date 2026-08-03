//! Teste de integração: dimensionamento/validação da hélice (Task 4.5)
//! contra a aeronave-base REAL (`config/aircraft/baseline_4seat.toml` +
//! motor Toyota real + missão `default.toml`), carregados do disco — mesmo
//! padrão de `tests/empennage.rs`/`tests/generic_engine.rs`.

use std::path::PathBuf;

use aeronave::agents::propeller::PropellerAgent;
use aeronave::models::config::{load_aircraft, load_engine, load_mission};
use aeronave::orchestrator::size_aircraft;

fn config_path(rel: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(rel)
}

/// Hand-check (Task 4.5, controller) contra o baseline real:
///
/// D=1,95m, PSRU=1,867, Toyota rpm_rated=3.400:
///   rpm_static = 3.400/1,867 = 1.821,1 rpm → n_rps=30,35 → tip=185,9 m/s
///   a(0,0)=340,3 m/s → M_static = 185,9/340,3 = 0,546
///   prop_rpm_cruise (busca de BSFC, ver `generic_engine.rs`) = 2.640/1,867
///     = 1.414,0 rpm → n_rps=23,57 → tip=144,4 m/s
///   V=77,78 m/s → helicoidal=164,0 m/s; a(2500,0)=330,6 → M_cruise=0,496
///   clearance = 1,25 − 0,975 = 0,275 ≥ 0,23 (exato)
#[test]
fn golden_baseline_toyota_mach_e_folga() {
    let cfg = load_aircraft(&config_path("config/aircraft/baseline_4seat.toml")).unwrap();
    let req = load_mission(&config_path("config/missions/default.toml")).unwrap();
    let toyota = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();

    let sized = size_aircraft(&cfg, &toyota, &req)
        .expect("aeronave-base + Toyota deveria convergir");

    let propeller = PropellerAgent::run(&cfg, &toyota, &sized.prop, &req);

    println!(
        "diametro={:.2}m fonte={} M_static={:.4} M_cruise={:.4} clearance={:.4} \
         prop_rpm_cruise={:.2}",
        propeller.diameter_m, propeller.source, propeller.tip_mach_static,
        propeller.tip_mach_cruise_helical, propeller.ground_clearance_m,
        sized.prop.prop_rpm_cruise,
    );

    assert_eq!(propeller.source, "config");
    assert_eq!(propeller.diameter_m, 1.95);

    assert!((propeller.tip_mach_static - 0.546).abs() < 0.005,
        "M_static = {:.4} (esperado 0.546 ±0.005)", propeller.tip_mach_static);
    assert!((propeller.tip_mach_cruise_helical - 0.496).abs() < 0.005,
        "M_cruise = {:.4} (esperado 0.496 ±0.005)", propeller.tip_mach_cruise_helical);
    assert!((propeller.ground_clearance_m - 0.275).abs() < 1e-9,
        "clearance = {:.6} (esperado 0.275 exato)", propeller.ground_clearance_m);

    assert!(propeller.ok_mach_static, "Mach estático deveria estar OK no baseline");
    assert!(propeller.ok_mach_cruise, "Mach cruzeiro deveria estar OK no baseline");
    assert!(propeller.ok_clearance, "folga de solo deveria estar OK no baseline");
}
