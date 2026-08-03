//! ConstraintDiagram — Diagrama de restrições clássico (W/S × P/W)
//!
//! Calcula os limites de carga alar (W/S) e razão peso-potência (P/W) do
//! "diagrama de restrições" clássico de projeto conceitual (Raymer cap. 5;
//! Gudmundsson cap. 3) e devolve uma área de asa RECOMENDADA — puramente
//! informativa, não redimensiona a aeronave automaticamente (isso mudaria
//! todos os "pinos" do projeto — trabalho futuro, se pedido).
//!
//! Convenções de unidades: SI puro (m, kg, N, m/s, Pa, W). `P/W` é reportado
//! em W/N — dimensionalmente idêntico a m/s (potência/peso = trabalho por
//! tempo dividido por força = velocidade), o que é esperado: o eixo P/W do
//! diagrama de restrições tem unidade de velocidade.

use serde::{Deserialize, Serialize};

use crate::agents::aerodynamics::dynamic_pressure;
use crate::agents::performance::shaft_power_kw;
use crate::models::aircraft_state::AircraftState;
use crate::models::atmosphere::{Isa, RHO_SL};
use crate::models::engine::EngineSpec;
use crate::models::requirements::Requirements;
use crate::models::specs::WingSpec;

const G: f64 = 9.807;      // m/s²

/// Razão de subida mínima exigida (CS-23), o mesmo piso usado em `main.rs`
/// para validar `perf.rc_sl_ms >= 1.5`.
const RC_REQ_MS: f64 = 1.5;

/// Eficiência da hélice em subida — tipicamente menor que a eficiência de
/// cruzeiro (ângulo de ataque de pá fora do ponto ótimo em baixa velocidade).
/// Valor constante de literatura (Raymer/Gudmundsson), não calibrado por
/// aeronave.
const ETA_P_CLIMB: f64 = 0.80;

/// Relatório do diagrama de restrições clássico W/S × P/W.
///
/// `Serialize`/`Deserialize` (Task 6.1): serializado dentro de
/// `specs::SizingReport` no relatório JSON final (`AircraftReport::sizing`)
/// — antes não era exposto ao consumidor de CAD.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WingLoadingReport {
    /// W/S máximo p/ atender V_stall com flap (N/m²):
    ///   W/S ≤ ½·ρ_SL·Vs²·CL_max_flaps
    pub ws_max_stall_n_m2: f64,
    /// V_stall de referência usada (m/s) — derivada do requisito de
    /// cruzeiro: Vs ≤ V_cruise/1.8 (regra prática CS-23 para a razão
    /// V_cruise/V_stall de aeronaves desta categoria).
    pub v_stall_ref_ms: f64,
    /// W/S ótimo de cruzeiro (N/m²): W/S = q_cruise·√(π·AR·e·CD0)
    /// (ponto de mínimo arrasto/máxima eficiência em cruzeiro).
    pub ws_optimal_cruise_n_m2: f64,
    /// W/S atual do projeto (N/m²): W/S = MTOW·g / S
    pub ws_actual_n_m2: f64,
    /// P/W mínimo (W/N) para sustentar a razão de subida requerida a nível
    /// do mar, na carga alar ATUAL do projeto — forma clássica de
    /// Raymer/Gudmundsson:
    ///   P/W = RC_req/η_p + √(2·(W/S)/(ρ_SL·√(3·CD0/K)))·(1.155/(L/D_max·η_p))
    /// onde K = 1/(π·AR·e) e L/D_max = 1/(2·√(K·CD0)).
    /// Referência: Gudmundsson, "General Aviation Aircraft Design", eq. 3-3;
    /// Raymer, "Aircraft Design: A Conceptual Approach", cap. 5.
    pub pw_min_climb_w_n: f64,
    /// P/W atual (W/N) usando a potência máxima contínua no eixo (após
    /// PSRU) ao nível do mar: P/W = P_shaft_max_contínua_SL / (MTOW·g)
    pub pw_actual_w_n: f64,
    /// Área de asa recomendada p/ o MTOW no W/S escolhido (m²):
    ///   S = MTOW·g / ws_chosen_n_m2
    /// Puramente informativa — não redimensiona a asa automaticamente.
    pub recommended_wing_area_m2: f64,
    /// W/S escolhido p/ a recomendação: min(ws_max_stall, ws_optimal_cruise)
    /// — o requisito de stall governa a escolha sempre que for o mais
    /// restritivo (o menor dos dois).
    pub ws_chosen_n_m2: f64,
}

/// Calcula os limites do diagrama de restrições clássico (W/S × P/W) para o
/// ponto de projeto atual (MTOW convergido + asa/motor/estado finais).
///
/// `mtow_kg`: MTOW de projeto (tipicamente `state.mtow_kg` após a
/// convergência de `orchestrator::size_aircraft`).
/// `wing`: saída do `AerodynamicsAgent` (AR, e, CD0, CL_max_flaps).
/// `engine`: motor genérico (potência de eixo via curva de torque).
/// `state`: estado da aeronave (geometria da asa, PSRU, hélice).
/// `req`: requisitos de missão (velocidade/altitude de cruzeiro).
pub fn wing_loading_limits(
    mtow_kg: f64,
    wing: &WingSpec,
    engine: &EngineSpec,
    state: &AircraftState,
    req: &Requirements,
) -> WingLoadingReport {
    let weight_n = mtow_kg * G;

    // ── W/S máximo por stall ────────────────────────────────────────────
    // Vs_ref deriva do requisito de cruzeiro (regra prática CS-23:
    // V_cruise/V_stall ≈ 1.8), não da V_stall real da asa atual — é o
    // limite que QUALQUER asa desta missão deveria respeitar.
    let v_stall_ref_ms = (req.cruise_speed_min_kmh / 3.6) / 1.8;
    let ws_max_stall_n_m2 = 0.5 * RHO_SL * v_stall_ref_ms * v_stall_ref_ms * wing.cl_max;

    // ── W/S ótimo de cruzeiro ───────────────────────────────────────────
    // Mesma densidade de cruzeiro usada por `AerodynamicsAgent` (Task 4.6):
    // atmosfera ISA completa, no ΔISA da missão — mantém os dois pontos de
    // W/S "de cruzeiro" fisicamente consistentes entre si.
    let rho_cruise = Isa::density_kgm3(req.cruise_altitude_m, req.isa_delta_c);
    let v_cruise_ms = req.cruise_speed_min_kmh / 3.6;
    let q_cruise = dynamic_pressure(rho_cruise, v_cruise_ms);
    let ws_optimal_cruise_n_m2 = q_cruise
        * (std::f64::consts::PI * wing.aspect_ratio * wing.oswald_efficiency * wing.cd0).sqrt();

    // ── W/S atual ────────────────────────────────────────────────────────
    let ws_actual_n_m2 = weight_n / state.wing_area_m2;

    // ── P/W mínimo para a razão de subida requerida (Raymer/Gudmundsson) ──
    let k = 1.0 / (std::f64::consts::PI * wing.aspect_ratio * wing.oswald_efficiency);
    let ld_max = 1.0 / (2.0 * (k * wing.cd0).sqrt());
    let sqrt_term = (2.0 * ws_actual_n_m2 / (RHO_SL * (3.0 * wing.cd0 / k).sqrt())).sqrt();
    let pw_min_climb_w_n =
        RC_REQ_MS / ETA_P_CLIMB + sqrt_term * (1.155 / (ld_max * ETA_P_CLIMB));

    // ── P/W atual (potência máxima contínua no eixo, SL) ────────────────
    let p_shaft_max_continuous_w =
        shaft_power_kw(engine, engine.rpm_max_continuous, 0.0, state.psru_efficiency) * 1_000.0;
    let pw_actual_w_n = p_shaft_max_continuous_w / weight_n;

    // ── Recomendação de área ────────────────────────────────────────────
    // O requisito de stall governa sempre que for o mais restritivo (o
    // menor W/S dos dois).
    let ws_chosen_n_m2 = ws_max_stall_n_m2.min(ws_optimal_cruise_n_m2);
    let recommended_wing_area_m2 = weight_n / ws_chosen_n_m2;

    WingLoadingReport {
        ws_max_stall_n_m2,
        v_stall_ref_ms,
        ws_optimal_cruise_n_m2,
        ws_actual_n_m2,
        pw_min_climb_w_n,
        pw_actual_w_n,
        recommended_wing_area_m2,
        ws_chosen_n_m2,
    }
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::agents::aerodynamics::AerodynamicsAgent;
    use crate::models::aircraft_config::test_fixtures::config_teste;
    use crate::models::engine::test_fixtures::motor_generico_teste as engine_teste;
    use crate::models::requirements::test_fixtures::requisitos_teste;

    fn setup() -> (AircraftState, WingSpec, EngineSpec, Requirements) {
        let cfg = config_teste();
        let state = AircraftState::from_config(&cfg);
        let req = requisitos_teste();
        let wing = AerodynamicsAgent::run(&state, &req);
        let engine = engine_teste();
        (state, wing, engine, req)
    }

    #[test]
    fn ws_max_stall_bate_calculo_manual() {
        let (state, wing, engine, req) = setup();
        let report = wing_loading_limits(state.mtow_kg, &wing, &engine, &state, &req);

        // Vs_ref = (260/3.6)/1.8 = 40.1235... m/s
        let vs_ref_esperado = (260.0_f64 / 3.6) / 1.8;
        assert!((report.v_stall_ref_ms - vs_ref_esperado).abs() < 1e-6,
            "v_stall_ref_ms {:.6} divergiu do esperado {:.6}",
            report.v_stall_ref_ms, vs_ref_esperado);

        // ws = 0.5 · 1.225 · Vs² · CL_max_flaps(1.65) ≈ 1627 N/m²
        let ws_esperado = 0.5 * 1.225 * vs_ref_esperado * vs_ref_esperado * 1.65;
        println!("ws_max_stall calculado = {:.3} N/m² | esperado ≈ {:.3} N/m²",
                 report.ws_max_stall_n_m2, ws_esperado);
        assert!((report.ws_max_stall_n_m2 - ws_esperado).abs() < ws_esperado * 0.01,
            "ws_max_stall_n_m2 {:.3} divergiu do cálculo manual {:.3} em mais de 1%",
            report.ws_max_stall_n_m2, ws_esperado);
        assert!((report.ws_max_stall_n_m2 - 1_627.0).abs() < 1_627.0 * 0.01,
            "ws_max_stall_n_m2 {:.3} divergiu do valor de referência do brief (~1627 N/m²) \
             em mais de 1%", report.ws_max_stall_n_m2);
    }

    #[test]
    fn ws_actual_bate_computacao_direta() {
        let (state, wing, engine, req) = setup();
        let report = wing_loading_limits(state.mtow_kg, &wing, &engine, &state, &req);

        let ws_direto = state.mtow_kg * G / state.wing_area_m2;
        assert!((report.ws_actual_n_m2 - ws_direto).abs() < 1e-9,
            "ws_actual_n_m2 {:.6} divergiu da computação direta {:.6}",
            report.ws_actual_n_m2, ws_direto);
    }

    #[test]
    fn pw_min_climb_positivo_e_pw_atual_com_margem_no_motor_forte() {
        let (state, wing, engine, req) = setup();
        let report = wing_loading_limits(state.mtow_kg, &wing, &engine, &state, &req);

        println!("pw_min_climb = {:.3} W/N | pw_actual = {:.3} W/N",
                 report.pw_min_climb_w_n, report.pw_actual_w_n);

        assert!(report.pw_min_climb_w_n > 0.0,
            "pw_min_climb_w_n {:.3} deveria ser positivo", report.pw_min_climb_w_n);
        assert!(report.pw_actual_w_n > report.pw_min_climb_w_n,
            "motor forte deveria ter P/W atual ({:.3} W/N) acima do mínimo exigido \
             para a razão de subida requerida ({:.3} W/N)",
            report.pw_actual_w_n, report.pw_min_climb_w_n);
    }

    #[test]
    fn area_recomendada_consistente_com_ws_escolhido() {
        let (state, wing, engine, req) = setup();
        let report = wing_loading_limits(state.mtow_kg, &wing, &engine, &state, &req);

        println!(
            "ws_max_stall={:.1} ws_optimal={:.1} ws_chosen={:.1} ws_actual={:.1} \
             recommended_area={:.3} actual_area={:.3}",
            report.ws_max_stall_n_m2, report.ws_optimal_cruise_n_m2, report.ws_chosen_n_m2,
            report.ws_actual_n_m2, report.recommended_wing_area_m2, state.wing_area_m2
        );

        // ws_chosen deve ser o menor dos dois limites (stall ou cruzeiro).
        let esperado_chosen = report.ws_max_stall_n_m2.min(report.ws_optimal_cruise_n_m2);
        assert!((report.ws_chosen_n_m2 - esperado_chosen).abs() < 1e-9);

        // recommended_area = MTOW·g / ws_chosen — consistência interna.
        let area_direta = state.mtow_kg * G / report.ws_chosen_n_m2;
        assert!((report.recommended_wing_area_m2 - area_direta).abs() < 1e-6,
            "recommended_wing_area_m2 {:.6} divergiu do cálculo direto {:.6}",
            report.recommended_wing_area_m2, area_direta);

        // Para a fixture sintética: stall governa (ws_max_stall < ws_optimal_cruise)
        // e a carga alar ATUAL do projeto (asa "grande" da fixture) é MENOR que
        // o limite de stall — ou seja, a asa atual está sobredimensionada em
        // relação ao que o requisito de stall exigiria, e a área recomendada
        // é MENOR que a área atual. Valores honestos medidos para esta
        // fixture, não assumidos a priori.
        assert!(report.ws_max_stall_n_m2 < report.ws_optimal_cruise_n_m2,
            "esperava stall governando para esta fixture (ws_max_stall {:.1} < \
             ws_optimal {:.1})", report.ws_max_stall_n_m2, report.ws_optimal_cruise_n_m2);
        assert!(report.ws_actual_n_m2 < report.ws_chosen_n_m2,
            "esperava ws_actual ({:.1}) < ws_chosen ({:.1}) para esta fixture",
            report.ws_actual_n_m2, report.ws_chosen_n_m2);
        assert!(report.recommended_wing_area_m2 < state.wing_area_m2,
            "área recomendada ({:.3} m²) deveria ser menor que a área atual \
             ({:.3} m²) quando ws_actual < ws_chosen",
            report.recommended_wing_area_m2, state.wing_area_m2);
    }
}
