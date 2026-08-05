//! Teste de integração: diagrama V-n completo com rajadas (Task 4.3).
//!
//! Roda o pipeline completo (`size_aircraft`) com a aeronave-base real
//! (`config/aircraft/baseline_4seat.toml`) + motor Toyota real + missão
//! real, exatamente como `main.rs`, e fixa (pin) os valores medidos do
//! diagrama V-n e o efeito em cascata no dimensionamento estrutural — para
//! detectar regressões nas fórmulas de rajada (CS 23.341) ou no
//! encadeamento VnDiagramAgent → StructuralAgent.

use std::path::PathBuf;

use aeronave::agents::structural::{load_factor_limit, StructuralAgent};
use aeronave::agents::vn_diagram::VnDiagramAgent;
use aeronave::models::config::{load_aircraft, load_engine, load_mission};
use aeronave::models::requirements::Requirements;
use aeronave::orchestrator::size_aircraft;

fn config_path(rel: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(rel)
}

fn baseline_state() -> aeronave::models::aircraft_config::AircraftConfig {
    load_aircraft(&config_path("config/aircraft/baseline_4seat.toml")).unwrap()
}

fn baseline_mission() -> Requirements {
    load_mission(&config_path("config/missions/default.toml")).unwrap()
}

/// Pins do baseline real (aeronave-base + Toyota 1GD-FTV + missão default):
/// medidos em 2026-08 (Task 4.3) — ver task-4.3-report.md para o
/// hand-check completo. Envelope MTOW = 1543.4 kg, massa leve (cenário
/// "Solo (piloto)") ≈ 1193 kg.
/// ATUALIZAÇÃO (Campanha E1–E6, 2026-08-05): envelope MTOW 1543.4 →
/// 1548.4 kg, massa leve ≈1193 → ≈1198.4 kg (mesmos cenários, massas de
/// OEW/itens deslocadas pela E6 — ver `config/aircraft/baseline_4seat.toml`
/// e `.superpowers/sdd/2026-08-05-baseline-e6/task-1-report.md`).
#[test]
fn vn_diagram_baseline_pin_velocidades_e_n_gust() {
    let cfg = baseline_state();
    let toyota = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let req = baseline_mission();
    let sized = size_aircraft(&cfg, &toyota, &req)
        .expect("aeronave-base + Toyota deveria convergir com o tanque de 260 L");

    let envelope_mtow_kg = sized.wb.spec.mtow_kg;
    let mass_light_kg = sized.wb.scenarios.iter()
        .map(|s| s.total_mass_kg)
        .fold(f64::INFINITY, f64::min);

    println!("envelope_mtow_kg={envelope_mtow_kg:.3}  mass_light_kg={mass_light_kg:.3}");

    let vn = VnDiagramAgent::run(
        &sized.wing, envelope_mtow_kg, mass_light_kg, &req, &cfg.structure.design_category,
    );

    println!(
        "VA={:.3} VB={:.3} VC={:.3} VD={:.3}  n_lim_pos={:.3} n_lim_neg={:.3} \
         n_gust_vc={:.4} n_gust_vd={:.4} n_gust_vc_light={:.4} n_design={:.4}",
        vn.va_kmh, vn.vb_kmh, vn.vc_kmh, vn.vd_kmh,
        vn.n_lim_pos, vn.n_lim_neg,
        vn.n_gust_vc, vn.n_gust_vd, vn.n_gust_vc_light, vn.n_design
    );

    // Velocidades características — CS 23.335. VA = VS1(limpa)·√n_lim_pos —
    // `wing.stall_speed_clean_kmh` vem de `AerodynamicsAgent::run`, calculado
    // no MTOW de PROJETO (não no `envelope_mtow_kg` usado aqui para VC/rajada
    // — inconsistência pré-existente, não introduzida por esta task). A Task
    // 5.1 (`MissionAgent`, análise por segmentos) desloca o MTOW de projeto
    // convergido de ~1.529,98 kg para ~1.517,54 kg (ver
    // `tests/generic_engine.rs::golden_toyota_baseline_regressao_task_2_1`),
    // reduzindo VS1 e portanto VA: 242.1 km/h → ~241.0 km/h.
    // ATUALIZAÇÃO (Task 5.2): `cooling_drag_fraction` eleva o MTOW de
    // projeto convergido de volta, de ~1.523,50 kg para ~1.532,33 kg (ver
    // mesma referência acima) — VS1 (e portanto VA) sobe de novo, proporcional
    // a √MTOW: ~241.0 km/h → 242.209167 km/h.
    // ATUALIZAÇÃO (Campanha E1–E6, 2026-08-05): MTOW de projeto convergido
    // sobe de novo, de ~1.532,33 kg para ~1.544,43 kg (+12,1 kg — mais CD0
    // do empennage reconverge o laço mais alto, ver
    // `tests/generic_engine.rs::golden_toyota_baseline_regressao_task_2_1`)
    // — VS1/VA sobem proporcional a √MTOW: 242.209167 → 243.176521 km/h.
    assert!((vn.va_kmh - 243.177).abs() < 1.0, "VA {:.1} km/h fora do pin (~243.177)", vn.va_kmh);
    assert!((vn.vc_kmh - 280.0).abs() < 0.1, "VC {:.1} km/h fora do pin (280.0, requisito de missão)", vn.vc_kmh);
    assert!((vn.vd_kmh - 350.0).abs() < 0.1, "VD {:.1} km/h fora do pin (350.0 = 1.25×VC)", vn.vd_kmh);
    assert!(vn.vb_kmh > 0.0 && vn.vb_kmh < vn.vd_kmh,
        "VB {:.1} km/h deveria estar entre 0 e VD ({:.1})", vn.vb_kmh, vn.vd_kmh);

    // Fatores de manobra — CS 23.337, categoria "normal".
    assert!((vn.n_lim_pos - 3.8).abs() < 1e-6);
    assert!((vn.n_lim_neg - (-1.52)).abs() < 1e-6);

    // Fator de rajada em VC, massa de envelope (hand-check do controller,
    // task-4.3-brief.md: μ≈27.63, Kg≈0.7385, n_gust_vc≈3.59).
    assert!((vn.n_gust_vc - 3.59).abs() < 0.05,
        "n_gust_vc {:.4} fora do hand-check (~3.59 ±0.05)", vn.n_gust_vc);
    // < n_lim_pos no envelope pesado → manobra governa aqui.
    assert!(vn.n_gust_vc < vn.n_lim_pos);

    // Fator de rajada em VC, massa LEVE — GOVERNA (CS 23.341, asa leve/
    // tanque quase vazio produz carga alar baixa → rajada mais crítica).
    // Faixa alinhada ao hand-check aproximado do brief (~4.2–4.4), pin
    // exato no valor medido com a massa leve REAL (não a estimativa do
    // brief, que era um palpite de ~1150 kg vs. o valor real medido acima).
    assert!(vn.n_gust_vc_light > vn.n_lim_pos,
        "n_gust_vc_light {:.4} deveria SUPERAR n_lim_pos {:.4} — achado central da Task 4.3 \
         (rajada governa no cenário mais leve)", vn.n_gust_vc_light, vn.n_lim_pos);
    assert!(vn.n_gust_vc_light > 4.0 && vn.n_gust_vc_light < 4.5,
        "n_gust_vc_light {:.4} fora da faixa esperada (4.0–4.5)", vn.n_gust_vc_light);

    // n_design deve ser exatamente o máximo dos três candidatos, e neste
    // baseline deve ser n_gust_vc_light (rajada, não manobra, governa).
    let esperado = vn.n_lim_pos.max(vn.n_gust_vc).max(vn.n_gust_vc_light);
    assert!((vn.n_design - esperado).abs() < 1e-9);
    assert!((vn.n_design - vn.n_gust_vc_light).abs() < 1e-9,
        "neste baseline a rajada em massa leve deveria governar (n_design == n_gust_vc_light)");
}

/// Efeito em cascata no dimensionamento estrutural: quando a rajada governa
/// (`n_design > n_lim_pos`), o momento fletor na raiz e a área de mesa da
/// longarina devem crescer na MESMA proporção `n_design/n_lim_pos` (o
/// momento fletor é linear em `n`, ver `wing_root_bending_nm`) frente ao
/// dimensionamento antigo (que usava só `n_lim_pos`, ignorando rajada).
#[test]
fn rajada_governando_aumenta_momento_fletor_na_proporcao_esperada() {
    let cfg = baseline_state();
    let toyota = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let req = baseline_mission();
    let sized = size_aircraft(&cfg, &toyota, &req).expect("deveria convergir");

    let envelope_mtow_kg = sized.wb.spec.mtow_kg;
    let mass_light_kg = sized.wb.scenarios.iter()
        .map(|s| s.total_mass_kg)
        .fold(f64::INFINITY, f64::min);
    let wing_mass_kg = cfg.masses.item_mass("asa").unwrap();

    let vn = VnDiagramAgent::run(
        &sized.wing, envelope_mtow_kg, mass_light_kg, &req, &cfg.structure.design_category,
    );
    assert!(vn.n_design > vn.n_lim_pos, "pré-condição do teste: rajada deve governar no baseline");

    let struc_com_rajada = StructuralAgent::run(
        &sized.wing, envelope_mtow_kg, wing_mass_kg, &req, &cfg.structure, vn.n_design,
    );
    let struc_so_manobra = StructuralAgent::run(
        &sized.wing, envelope_mtow_kg, wing_mass_kg, &req, &cfg.structure,
        load_factor_limit(&cfg.structure.design_category),
    );

    println!(
        "M_limit(n_design={:.3})={:.1}N·m  M_limit(n_lim={:.3})={:.1}N·m  razão={:.4} (esperado {:.4})",
        vn.n_design, struc_com_rajada.wing_root_bending_limit_nm,
        vn.n_lim_pos, struc_so_manobra.wing_root_bending_limit_nm,
        struc_com_rajada.wing_root_bending_limit_nm / struc_so_manobra.wing_root_bending_limit_nm,
        vn.n_design / vn.n_lim_pos,
    );

    assert!(struc_com_rajada.wing_root_bending_limit_nm > struc_so_manobra.wing_root_bending_limit_nm,
        "dimensionamento com rajada deveria produzir momento fletor MAIOR que só manobra");
    assert!(struc_com_rajada.spar_flange_area_cm2 > struc_so_manobra.spar_flange_area_cm2,
        "dimensionamento com rajada deveria produzir área de mesa da longarina MAIOR");

    // Momento fletor é linear em n (`wing_root_bending_nm`, salvo o termo de
    // alívio pelo peso da asa, pequeno face à sustentação) — a razão entre
    // os dois momentos deve bater com a razão n_design/n_lim_pos, com folga
    // para o termo de alívio.
    let razao_momento = struc_com_rajada.wing_root_bending_limit_nm
        / struc_so_manobra.wing_root_bending_limit_nm;
    let razao_n = vn.n_design / vn.n_lim_pos;
    assert!((razao_momento - razao_n).abs() < 0.02,
        "razão de momento fletor {razao_momento:.4} deveria bater com razão de n {razao_n:.4} \
         (±0.02, folga para o termo de alívio pelo peso da asa)");

    // design_load_factor_g reportado deve ser n_design (não mais n_lim fixo).
    assert!((struc_com_rajada.design_load_factor_g - vn.n_design).abs() < 1e-9);
}
