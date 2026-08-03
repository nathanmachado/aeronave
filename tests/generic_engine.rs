//! Teste de integração: genericidade do motor.
//!
//! Este é o teste central do pedido do usuário — "trocar de motor deve ser
//! trocar um arquivo TOML, não o código". Vive em `tests/` (crate de teste
//! separada) e não em `src/`, para que `src/` permaneça livre de qualquer
//! menção a um motor real específico (ver grep de regressão no relatório da
//! Task 1.4). Consome a biblioteca `aeronave` via `src/lib.rs`.

use std::path::PathBuf;

use aeronave::agents::aerodynamics::AerodynamicsAgent;
use aeronave::agents::propulsion::PropulsionAgent;
use aeronave::models::aircraft_state::AircraftState;
use aeronave::models::config::load_engine;
use aeronave::models::requirements::Requirements;

fn config_path(rel: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(rel)
}

#[test]
fn trocar_motor_muda_resultado_sem_mudar_codigo() {
    let state = AircraftState::initial();
    let req   = Requirements::project_default();
    let wing  = AerodynamicsAgent::run(&state, &req);
    let toyota = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let rotax  = load_engine(&config_path("config/engines/rotax_915is.toml")).unwrap();

    let p_toyota = PropulsionAgent::run(&state, &req, &wing, &toyota);
    let p_rotax  = PropulsionAgent::run(&state, &req, &wing, &rotax);

    // Mesmo código, dados diferentes → resultados diferentes e coerentes
    assert!(p_toyota.power_kw > p_rotax.power_kw);
    assert!(p_toyota.fc_cruise_lph != p_rotax.fc_cruise_lph);
    assert_eq!(p_toyota.engine_model, "Toyota 1GD-FTV 2.8 Turbo Diesel");
    assert_eq!(p_rotax.engine_model, "Rotax 915 iS");

    // Viabilidade de cruzeiro: o Toyota (~150 kW de pico) sustenta 280 km/h
    // com esta célula/hélice/PSRU; o Rotax 915iS (~70 kW de pico) não —
    // física honesta, não um número mágico ajustado para "dar certo".
    println!(
        "Toyota: {:.1} kW pico | P_req {:.1} kW vs P_disp {:.1} kW @ {:.0} rpm | feasible={}",
        p_toyota.power_kw, p_toyota.p_req_cruise_kw, p_toyota.p_shaft_cruise_kw,
        p_toyota.engine_rpm_cruise, p_toyota.cruise_feasible
    );
    println!(
        "Rotax:  {:.1} kW pico | P_req {:.1} kW vs P_disp {:.1} kW @ {:.0} rpm | feasible={}",
        p_rotax.power_kw, p_rotax.p_req_cruise_kw, p_rotax.p_shaft_cruise_kw,
        p_rotax.engine_rpm_cruise, p_rotax.cruise_feasible
    );

    assert!(p_toyota.cruise_feasible,
        "Toyota 1GD-FTV deveria sustentar 280 km/h de cruzeiro com esta célula/hélice/PSRU");
    assert!(!p_rotax.cruise_feasible,
        "Rotax 915iS (~70 kW de pico) não deveria sustentar 280 km/h com esta célula \
         dimensionada para o Toyota (~150 kW) — física honesta, não um bug");
}

// Histórico (Task 0.3 → Task 1.4): corrigir load_fraction para referenciar
// P_disponível no rpm de cruzeiro (em vez de POWER_KW_MAX em SL) elevou a
// carga de cruzeiro para ~0.99 a 2.400 rpm fixo, subindo o BSFC/consumo e
// derrubando a autonomia para ~7.46h (<8h) e o alcance para ~2.090km
// (<2.240km) — ver task-0.3-report.md. Este teste ficou `#[ignore]`d desde
// então como violação de requisito conhecida.
//
// A Task 1.4 substitui o rpm de cruzeiro fixo (2.400) por uma busca que
// varre `rpm_optimal ± 20%` (limitada por `rpm_max_continuous`) e escolhe o
// rpm de menor BSFC entre os que entregam a potência requerida. Para o
// motor Toyota 1GD-FTV isto desloca o cruzeiro para ~2.610 rpm, um pouco
// acima do rpm ótimo de BSFC (2.200) mas ainda dentro da banda de torque
// plano — o suficiente para reduzir o BSFC de ~221 para ~211 g/kWh e o
// consumo de ~28.9 para ~26.9 L/h em relação ao valor a 2.400 rpm fixo.
// Isso fecha a lacuna: autonomia medida ~8.02h (>= 8.0h) e alcance
// ~2.245km (>= 2.240km) — ver task-1.4-report.md para os números completos.
// Requisito NÃO foi enfraquecido; a física é que melhorou.
#[test]
fn autonomia_minima_8_horas() {
    let state = AircraftState::initial();
    let req   = Requirements::project_default();
    let wing  = AerodynamicsAgent::run(&state, &req);
    let toyota = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let prop  = PropulsionAgent::run(&state, &req, &wing, &toyota);

    println!("Motor cruzeiro: {:.0} rpm", prop.engine_rpm_cruise);
    println!("Consumo cruzeiro: {:.1} L/h", prop.fc_cruise_lph);
    println!("Autonomia: {:.2} h", prop.endurance_h);
    println!("Alcance: {:.0} km", prop.range_km);
    println!("BSFC: {:.0} g/kWh", prop.bsfc_cruise_gkwh);
    println!("Eficiência hélice: {:.3}", prop.prop_efficiency);

    assert!(prop.endurance_h >= 8.0,
        "Autonomia {:.2} h abaixo do requisito de 8 h", prop.endurance_h);
    assert!(prop.range_km >= 2_240.0,
        "Alcance {:.0} km abaixo do requisito de 2.240 km", prop.range_km);
}
