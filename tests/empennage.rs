//! Teste de integração: dimensionamento da empenagem (Task 4.1) contra a
//! aeronave-base REAL (`config/aircraft/baseline_4seat.toml`), carregada do
//! disco — não uma fixture sintética. Vive em `tests/` pelo mesmo motivo de
//! `tests/config_files.rs`/`tests/generic_engine.rs`: exercitar o pipeline
//! completo contra os arquivos TOML reais do projeto.

use std::path::PathBuf;

use aeronave::agents::aerodynamics::AerodynamicsAgent;
use aeronave::agents::empennage::EmpennageAgent;
use aeronave::models::aircraft_state::AircraftState;
use aeronave::models::config::{load_aircraft, load_engine, load_mission};
use aeronave::orchestrator::size_aircraft;

fn config_path(rel: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(rel)
}

/// S_h calculado à mão a partir do baseline real (span=11.94m, area=14.2m²,
/// taper=0.45, tail_arm_m=4.80, v_h=0.70):
///   c_r  = 2·14.2/(11.94·1.45)          = 1.6404 m
///   MAC  = (2/3)·c_r·(1+0.45+0.2025)/1.45 = 1.2463 m
///   S_h  = 0.70·14.2·1.2463/4.80        ≈ 2.581 m²
#[test]
fn baseline_s_h_bate_calculo_manual() {
    let cfg = load_aircraft(&config_path("config/aircraft/baseline_4seat.toml")).unwrap();
    let state = AircraftState::from_config(&cfg);
    let req = load_mission(&config_path("config/missions/default.toml")).unwrap();
    let wing = AerodynamicsAgent::run(&state, &req);

    let emp = EmpennageAgent::run(&wing, &cfg);
    println!(
        "S_h={:.4}m²  S_v={:.4}m²  span_h={:.4}m  span_v={:.4}m",
        emp.s_horizontal_m2, emp.s_vertical_m2, emp.span_h_m, emp.span_v_m
    );

    let esperado_s_h = 2.581_f64;
    assert!((emp.s_horizontal_m2 - esperado_s_h).abs() < 0.05,
        "S_h = {:.4} m² (esperado ≈{esperado_s_h:.3} m² ±0.05)", emp.s_horizontal_m2);
}

/// Com a empenagem REALMENTE dimensionada (V_h=0.70 → S_h/S_w≈0.182,
/// a_t/a_w≈0.753 — ambos MENORES que os antigos valores hardcoded
/// s_ratio=0.22/at_aw=0.85), o ponto neutro recua para a frente em relação
/// ao cálculo hardcoded anterior (NP: ~4.019m → ~3.803m — ver
/// task-4.1-report.md para a tabela completa por cenário). Mesmo com essa
/// queda honesta de margem estática, TODOS os cenários de carga da
/// aeronave-base real continuam estáveis (SM mínimo medido ≈43%, bem acima
/// do piso de 3% usado em `WeightBalanceAgent::run`) — não há violação de
/// estabilidade a reportar.
#[test]
fn baseline_todos_os_cenarios_estaveis_com_empenagem_dimensionada() {
    let cfg = load_aircraft(&config_path("config/aircraft/baseline_4seat.toml")).unwrap();
    let req = load_mission(&config_path("config/missions/default.toml")).unwrap();
    let toyota = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();

    let sized = size_aircraft(&cfg, &toyota, &req)
        .expect("aeronave-base + Toyota deveria convergir");

    println!("x_np = {:.4}m | SM mínima reportada = {:.2}%",
        sized.wb.x_np_m, sized.wb.spec.static_margin_pct);

    for sc in &sized.wb.scenarios {
        println!("  {:30} x_cg={:.4}m  SM={:.4}  estável={}",
            sc.name, sc.x_cg_m, sc.static_margin, sc.stable);
        assert!(sc.stable,
            "Cenário '{}': SM={:.4} — INSTÁVEL com a empenagem dimensionada por \
             coeficiente de volume (V_h={:.2})", sc.name, sc.static_margin, cfg.empennage.v_h);
    }

    // SM mínima com folga bem acima do piso de estabilidade (3%) — a queda
    // de margem é honesta (NP mais à frente que o modelo hardcoded antigo),
    // mas não chega perto de instabilizar nenhum cenário real.
    assert!(sized.wb.spec.static_margin_pct > 10.0,
        "SM mínima {:.2}% muito próxima do piso de estabilidade — revisar",
        sized.wb.spec.static_margin_pct);
}
