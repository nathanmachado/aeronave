//! Teste de integração: dimensionamento de aileron, flap, profundor e leme
//! (Task 4.2) contra a aeronave-base REAL (`config/aircraft/baseline_4seat.toml`),
//! carregada do disco — não uma fixture sintética. Vive em `tests/` pelo
//! mesmo motivo de `tests/empennage.rs`: exercitar o pipeline completo
//! contra os arquivos TOML reais do projeto.

use std::path::PathBuf;

use aeronave::agents::aerodynamics::AerodynamicsAgent;
use aeronave::agents::control_surfaces::ControlSurfacesAgent;
use aeronave::agents::empennage::EmpennageAgent;
use aeronave::models::aircraft_state::AircraftState;
use aeronave::models::config::{load_aircraft, load_mission};

fn config_path(rel: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(rel)
}

/// Aileron calculado à mão a partir do baseline real (span=11.94m,
/// area=14.2m², taper=0.45; control_surfaces: aileron_span_start_frac=0.55,
/// aileron_span_end_frac=0.90, aileron_chord_frac=0.25):
///   c_r  = 2·14.2/(11.94·1.45)            = 1.64043 m
///   c(0.55) = c_r·(1−0.55·0.55) = c_r·0.6975 ≈ 1.14420 m
///   c(0.90) = c_r·(1−0.55·0.90) = c_r·0.5050 ≈ 0.82842 m
///   corda_média·chord_frac = 0.25·(1.1442+0.8284)/2 ≈ 0.24658 m
///   span/lado = (0.90−0.55)·(11.94/2)      = 0.35·5.97  = 2.0895 m
///   área/lado ≈ 0.24658·2.0895            ≈ 0.5152 m²
///   área total (×2)                        ≈ 1.0304 m²
#[test]
fn baseline_aileron_bate_calculo_manual() {
    let cfg = load_aircraft(&config_path("config/aircraft/baseline_4seat.toml")).unwrap();
    let state = AircraftState::from_config(&cfg);
    let req = load_mission(&config_path("config/missions/default.toml")).unwrap();
    let wing = AerodynamicsAgent::run(&state, &req);
    let emp = EmpennageAgent::run(&wing, &cfg);

    let cs = ControlSurfacesAgent::run(&wing, &emp, &cfg);
    println!(
        "aileron: span/lado={:.4}m  área(×2)={:.4}m²  corda_média={:.4}m  [{:.4}–{:.4}]m",
        cs.aileron.span_m, cs.aileron.area_m2, cs.aileron.chord_mean_m,
        cs.aileron.start_m, cs.aileron.end_m
    );

    let esperado_span = 2.0895_f64; // PIN: control_surfaces.aileron.span_m
    let esperado_area = 1.0304_f64; // PIN: control_surfaces.aileron.area_m2

    assert!((cs.aileron.span_m - esperado_span).abs() / esperado_span < 0.01,
        "span/lado = {:.4}m (esperado ≈{esperado_span:.4}m, ±1%)", cs.aileron.span_m);
    assert!((cs.aileron.area_m2 - esperado_area).abs() / esperado_area < 0.01,
        "área = {:.4}m² (esperado ≈{esperado_area:.4}m², ±1%)", cs.aileron.area_m2);
}

/// Pin exato (±1%) dos quatro valores de área computados no baseline real —
/// protege contra regressão silenciosa na fórmula do agente. Valores
/// obtidos rodando `ControlSurfacesAgent::run` sobre o baseline real (ver
/// task-4.2-report.md para a tabela completa, incluindo `span_m`/`start_m`/
/// `end_m` de cada superfície).
#[test]
fn baseline_areas_pin_exato() {
    let cfg = load_aircraft(&config_path("config/aircraft/baseline_4seat.toml")).unwrap();
    let state = AircraftState::from_config(&cfg);
    let req = load_mission(&config_path("config/missions/default.toml")).unwrap();
    let wing = AerodynamicsAgent::run(&state, &req);
    let emp = EmpennageAgent::run(&wing, &cfg);

    let cs = ControlSurfacesAgent::run(&wing, &emp, &cfg);
    println!(
        "aileron={:.6}m²  flap={:.6}m²  elevator={:.6}m²  rudder={:.6}m²",
        cs.aileron.area_m2, cs.flap.area_m2, cs.elevator.area_m2, cs.rudder.area_m2
    );

    // Campanha E1–E6 (2026-08-05): elevator_chord_frac 0.35→0.40 e S_h
    // (via v_h 0.70→0.85) elevam a área do profundor: 0.840087 → 1.165835
    // m² (+38,8%). Aileron/flap/rudder não dependem de v_h/elevator_chord_
    // frac — inalterados.
    let casos = [
        ("aileron", cs.aileron.area_m2, 1.030418_f64), // PIN: control_surfaces.aileron.area_m2
        ("flap", cs.flap.area_m2, 1.962538_f64), // PIN: control_surfaces.flap.area_m2
        ("elevator", cs.elevator.area_m2, 1.165835_f64), // PIN: control_surfaces.elevator.area_m2
        ("rudder", cs.rudder.area_m2, 0.459899_f64), // PIN: control_surfaces.rudder.area_m2
    ];
    for (nome, obtido, esperado) in casos {
        assert!((obtido - esperado).abs() / esperado < 0.01,
            "{nome}.area_m2 = {obtido:.6}m² (esperado {esperado:.6}m², ±1%)");
    }
}

/// Área do profundor deve ser coerente (±15%) com a aproximação simples
/// `elevator_chord_frac · elevator_span_frac · S_h` — o valor real difere
/// porque o cálculo do agente usa o trapézio real (corda varia ao longo da
/// envergadura), não uma corda constante.
#[test]
fn baseline_area_do_profundor_coerente_com_s_h() {
    let cfg = load_aircraft(&config_path("config/aircraft/baseline_4seat.toml")).unwrap();
    let state = AircraftState::from_config(&cfg);
    let req = load_mission(&config_path("config/missions/default.toml")).unwrap();
    let wing = AerodynamicsAgent::run(&state, &req);
    let emp = EmpennageAgent::run(&wing, &cfg);

    let cs = ControlSurfacesAgent::run(&wing, &emp, &cfg);
    let aprox = cfg.control_surfaces.elevator_chord_frac
        * cfg.control_surfaces.elevator_span_frac
        * emp.s_horizontal_m2;

    println!("elevator.area_m2={:.4}  aproximação={:.4}  S_h={:.4}",
        cs.elevator.area_m2, aprox, emp.s_horizontal_m2);

    assert!((cs.elevator.area_m2 - aprox).abs() / aprox < 0.15,
        "área do profundor {:.4}m² deveria estar a ±15% da aproximação {:.4}m²",
        cs.elevator.area_m2, aprox);
}

/// Regressão do finding de revisão (SurfaceGeom para o profundor divergia
/// da convenção "por lado" de aileron/flap/rudder): `elevator.end_m` é POR
/// LADO — nunca pode ultrapassar a semi-envergadura real do EH
/// (`emp.span_h_m/2`). Antes da correção, `end_m` valia
/// `elevator_span_frac·span_h_m` (a largura ponta-a-ponta do profundor,
/// ~2.89m no baseline real) — 1.8× a semi-envergadura real do EH (~1.61m),
/// o que posicionaria a superfície fora do estabilizador se um consumidor
/// de CAD lesse `end_m` como distância-da-linha-de-centro.
#[test]
fn baseline_elevator_end_m_nao_ultrapassa_semi_envergadura_do_eh() {
    let cfg = load_aircraft(&config_path("config/aircraft/baseline_4seat.toml")).unwrap();
    let state = AircraftState::from_config(&cfg);
    let req = load_mission(&config_path("config/missions/default.toml")).unwrap();
    let wing = AerodynamicsAgent::run(&state, &req);
    let emp = EmpennageAgent::run(&wing, &cfg);

    let cs = ControlSurfacesAgent::run(&wing, &emp, &cfg);
    let half_span_h = emp.span_h_m / 2.0;

    println!("elevator: span/lado={:.4}m  [{:.4}–{:.4}]m  semi-envergadura EH={:.4}m",
        cs.elevator.span_m, cs.elevator.start_m, cs.elevator.end_m, half_span_h);

    assert!(cs.elevator.end_m <= half_span_h + 1e-9,
        "elevator.end_m ({:.4}m) não deveria ultrapassar a semi-envergadura do EH ({:.4}m)",
        cs.elevator.end_m, half_span_h);
    assert_eq!(cs.elevator.start_m, 0.0); // PIN: control_surfaces.elevator.start_m
}

/// Consistência de espelhamento: a área TOTAL do profundor deve ser
/// EXATAMENTE `2 × (trapézio de um lado, de η=0 a elevator_span_frac)` —
/// mesma identidade algébrica que já valia para aileron/flap.
#[test]
fn baseline_elevator_area_bate_com_dois_trapezios_por_lado() {
    let cfg = load_aircraft(&config_path("config/aircraft/baseline_4seat.toml")).unwrap();
    let state = AircraftState::from_config(&cfg);
    let req = load_mission(&config_path("config/missions/default.toml")).unwrap();
    let wing = AerodynamicsAgent::run(&state, &req);
    let emp = EmpennageAgent::run(&wing, &cfg);

    let cs = ControlSurfacesAgent::run(&wing, &emp, &cfg);

    let chord_frac = cfg.control_surfaces.elevator_chord_frac;
    let c_start = chord_frac * aeronave::agents::weight_balance::chord_at(0.0, emp.chord_h_root_m, emp.taper_h);
    let c_end = chord_frac * aeronave::agents::weight_balance::chord_at(
        cfg.control_surfaces.elevator_span_frac, emp.chord_h_root_m, emp.taper_h);
    let area_per_side = 0.5 * (c_start + c_end) * cs.elevator.span_m;

    println!("elevator.area_m2={:.9}  2×área_por_lado={:.9}", cs.elevator.area_m2, 2.0 * area_per_side);
    // PIN: NAO-PUBLICADO — fator da fórmula (duas superfícies), não valor publicado
    assert!((cs.elevator.area_m2 - 2.0 * area_per_side).abs() < 1e-9,
        "elevator.area_m2 ({:.9}) deveria ser exatamente 2×área_por_lado ({:.9})",
        cs.elevator.area_m2, 2.0 * area_per_side);
}

/// Coerência geométrica: aileron e flap não se sobrepõem e ambos cabem
/// dentro da semi-envergadura da asa real.
#[test]
fn baseline_aileron_flap_nao_sobrepoem_e_cabem_na_semi_envergadura() {
    let cfg = load_aircraft(&config_path("config/aircraft/baseline_4seat.toml")).unwrap();
    let state = AircraftState::from_config(&cfg);
    let req = load_mission(&config_path("config/missions/default.toml")).unwrap();
    let wing = AerodynamicsAgent::run(&state, &req);
    let emp = EmpennageAgent::run(&wing, &cfg);

    let cs = ControlSurfacesAgent::run(&wing, &emp, &cfg);
    let half_span = wing.span_m / 2.0;

    println!(
        "flap [{:.3}–{:.3}]m  aileron [{:.3}–{:.3}]m  semi-envergadura={:.3}m",
        cs.flap.start_m, cs.flap.end_m, cs.aileron.start_m, cs.aileron.end_m, half_span
    );

    assert!(cs.flap.start_m >= 0.0);
    assert!(cs.flap.end_m <= cs.aileron.start_m + 1e-9,
        "flap termina em {:.3}m, depois do início do aileron em {:.3}m — sobreposição",
        cs.flap.end_m, cs.aileron.start_m);
    assert!(cs.aileron.end_m <= half_span + 1e-9,
        "aileron termina em {:.3}m, além da semi-envergadura ({:.3}m)",
        cs.aileron.end_m, half_span);
}
