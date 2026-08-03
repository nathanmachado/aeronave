/// AerodynamicsAgent
///
/// Calcula o polar de arrasto, parâmetros de asa e velocidade de stall.
/// Todas as equações seguem a Mecânica do Voo clássica (Anderson, Raymer).
///
/// Convenções de unidades: SI puro (m, kg, N, m/s, Pa).
/// Velocidades em km/h são convertidas internamente.

use crate::models::{
    atmosphere::RHO_SL,
    aircraft_state::AircraftState,
    requirements::Requirements,
    specs::WingSpec,
};

const G: f64 = 9.807;            // m/s²

// Trem retrátil e recolhido não contribui CD0 — só o trem FIXO soma um
// incremento (vem de `[gear] cd0_fixed_increment` do TOML de aeronave).
const CD0_GEAR_RETRACTABLE: f64 = 0.0;

/// Pressão dinâmica: q = 0.5 · ρ · V²
/// V em m/s, retorna Pa.
pub fn dynamic_pressure(rho: f64, v_ms: f64) -> f64 {
    0.5 * rho * v_ms * v_ms
}

/// Coeficiente de sustentação necessário para voo nivelado:
/// CL = W / (q · S)
pub fn cl_required(weight_n: f64, q: f64, wing_area_m2: f64) -> f64 {
    weight_n / (q * wing_area_m2)
}

/// Coeficiente de arrasto induzido (modelo elíptico generalizado):
/// CDi = CL² / (π · AR · e)
pub fn cd_induced(cl: f64, aspect_ratio: f64, oswald: f64) -> f64 {
    cl * cl / (std::f64::consts::PI * aspect_ratio * oswald)
}

/// CD0 total da aeronave (soma das parcelas de cada componente).
/// Segue o método de "component drag build-up" (Raymer cap. 12). Todas as
/// parcelas vêm de configuração (`[wing]`, `[fuselage]`, `[empennage]`,
/// `[drag]`, `[gear]` do TOML de aeronave) — nenhuma é constante de perfil
/// específico.
pub fn cd0_total(
    cd0_wing: f64,
    cd0_fuselage: f64,
    cd0_empennage: f64,
    cd0_misc: f64,
    gear_retractable: bool,
    cd0_gear_fixed_increment: f64,
) -> f64 {
    let gear_cd0 = if gear_retractable {
        CD0_GEAR_RETRACTABLE
    } else {
        cd0_gear_fixed_increment
    };
    cd0_wing + cd0_fuselage + cd0_empennage + gear_cd0 + cd0_misc
}

/// Arrasto total em Newton:
/// D = q · S · CD_total
pub fn drag_total_n(q: f64, wing_area_m2: f64, cd: f64) -> f64 {
    q * wing_area_m2 * cd
}

/// Velocidade de stall (voo nivelado):
/// V_s = sqrt(2·W / (ρ·S·CL_max))
/// Genérica — o chamador escolhe CL_max limpo (VS1) ou com flap (VS0).
pub fn stall_speed_ms(weight_n: f64, rho: f64, wing_area_m2: f64, cl_max: f64) -> f64 {
    (2.0 * weight_n / (rho * wing_area_m2 * cl_max)).sqrt()
}

/// Eficiência de Oswald estimada pelo método de Raymer:
/// e = 1.78·(1 - 0.045·AR^0.68) - 0.64
/// Válida para asa de baixa asa trapezoidal.
pub fn oswald_efficiency(aspect_ratio: f64) -> f64 {
    let e = 1.78 * (1.0 - 0.045 * aspect_ratio.powf(0.68)) - 0.64;
    e.clamp(0.70, 0.95)
}

/// Razão L/D em cruzeiro
pub fn ld_ratio(cl: f64, cd: f64) -> f64 {
    cl / cd
}

/// Número de Mach da ponta da hélice (verificação de compressibilidade).
/// `a_ms`: velocidade do som local (m/s) — de `Isa::speed_of_sound_ms`, na
/// altitude/ΔISA relevantes (ex.: cruzeiro). Antes da Task 4.6 este valor
/// era um literal fixo (340,0 m/s, aproximação @ 2.500m ISA); agora é
/// calculado a partir da atmosfera ISA completa.
pub fn mach_tip(prop_diameter_m: f64, prop_rpm: f64, v_cruise_ms: f64, a_ms: f64) -> f64 {
    let tip_speed = std::f64::consts::PI * prop_diameter_m * (prop_rpm / 60.0);
    let total_speed = (tip_speed * tip_speed + v_cruise_ms * v_cruise_ms).sqrt();
    total_speed / a_ms
}

// ─── AGENTE PRINCIPAL ────────────────────────────────────────────────────────

pub struct AerodynamicsAgent;

impl AerodynamicsAgent {
    /// Executa o agente e retorna a especificação aerodinâmica completa.
    pub fn run(state: &AircraftState, req: &Requirements) -> WingSpec {
        let rho_cruise = crate::models::atmosphere::Isa::density_kgm3(
            req.cruise_altitude_m, req.isa_delta_c,
        );
        let v_cruise_ms = req.cruise_speed_min_kmh / 3.6;
        let weight_n = state.mtow_kg * G;

        let q_cruise = dynamic_pressure(rho_cruise, v_cruise_ms);
        let ar = state.aspect_ratio();
        let oswald = oswald_efficiency(ar);

        let cd0 = cd0_total(
            state.cd0_wing, state.cd0_fuselage, state.cd0_empennage, state.cd0_misc,
            state.gear_retractable, state.cd0_gear_fixed_increment,
        );

        let cl_cruise = cl_required(weight_n, q_cruise, state.wing_area_m2);
        let cdi = cd_induced(cl_cruise, ar, oswald);
        let cd_cruise = cd0 + cdi;
        let ld = ld_ratio(cl_cruise, cd_cruise);

        // Velocidades de stall ao nível do mar (condição mais crítica).
        // VS0 — configuração com flap/pouso (CL_max maior → V_stall MENOR).
        let v_stall_flaps_ms = stall_speed_ms(weight_n, RHO_SL, state.wing_area_m2, state.cl_max_flaps);
        // VS1 — configuração limpa/cruzeiro (CL_max menor → V_stall MAIOR).
        let v_stall_clean_ms = stall_speed_ms(weight_n, RHO_SL, state.wing_area_m2, state.cl_max_clean);

        WingSpec {
            span_m:           state.wing_span_m,
            area_m2:          state.wing_area_m2,
            aspect_ratio:     ar,
            airfoil:          state.airfoil.clone(),
            taper_ratio:      state.taper_ratio,
            thickness_ratio:  state.thickness_ratio,
            oswald_efficiency: oswald,
            cd0,
            cl_cruise,
            cd_cruise,
            cl_max:                 state.cl_max_flaps,
            cl_max_clean:           state.cl_max_clean,
            stall_speed_flaps_kmh:  v_stall_flaps_ms * 3.6,
            stall_speed_clean_kmh:  v_stall_clean_ms * 3.6,
            ld_ratio_cruise:  ld,
        }
    }
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::aircraft_config::test_fixtures::config_teste;

    // Testes de densidade ISA (isa_density_sea_level, isa_density_decresce_com_altitude)
    // moveram-se para `src/models/atmosphere.rs` (Task 4.6) junto com a
    // função — `isa_density` foi removida deste módulo em favor de
    // `Isa::density_kgm3`.

    #[test]
    fn stall_speed_fisica() {
        // Aeronave de 1.461 kg, S=14.2m², CLmax=1.72 (flapada, valor literal
        // de teste — perfil real não é mais constante em src/) → V_stall ~ 90-115 km/h
        let vs = stall_speed_ms(1_461.0 * G, RHO_SL, 14.2, 1.72);
        let vs_kmh = vs * 3.6;
        assert!(vs_kmh > 90.0 && vs_kmh < 115.0,
            "V_stall esperada 90-115 km/h, obtida {vs_kmh:.1} km/h");
    }

    #[test]
    fn vs0_flapada_menor_que_vs1_limpa() {
        // Task 0.5: VS0 (com flap) deve ser MENOR que VS1 (limpa) — flap
        // aumenta CL_max, o que reduz a velocidade de stall.
        let cfg = config_teste();
        let state = AircraftState::from_config(&cfg);
        let req = crate::models::requirements::test_fixtures::requisitos_teste();
        let wing = AerodynamicsAgent::run(&state, &req);

        println!("VS0 (flap) = {:.1} km/h | VS1 (limpa) = {:.1} km/h",
                 wing.stall_speed_flaps_kmh, wing.stall_speed_clean_kmh);

        assert!(wing.stall_speed_flaps_kmh < wing.stall_speed_clean_kmh,
            "VS0 (flap) {:.1} km/h deveria ser menor que VS1 (limpa) {:.1} km/h",
            wing.stall_speed_flaps_kmh, wing.stall_speed_clean_kmh);

        // Faixas físicas para a fixture sintética (mtow_guess=1.400kg,
        // S=13.5m², CL_max_flaps=1.65, CL_max_clean=1.40): valores
        // observados empiricamente, com folga.
        assert!(wing.stall_speed_flaps_kmh > 100.0 && wing.stall_speed_flaps_kmh < 125.0,
            "VS0 {:.1} km/h fora da faixa esperada (100-125 km/h)",
            wing.stall_speed_flaps_kmh);
        assert!(wing.stall_speed_clean_kmh > 110.0 && wing.stall_speed_clean_kmh < 135.0,
            "VS1 {:.1} km/h fora da faixa esperada (110-135 km/h)",
            wing.stall_speed_clean_kmh);
    }

    #[test]
    fn cd0_retravel_menor_que_fixo() {
        let cd0_ret = cd0_total(0.005, 0.010, 0.004, 0.003, true, 0.008);
        let cd0_fix = cd0_total(0.005, 0.010, 0.004, 0.003, false, 0.008);
        assert!(cd0_ret < cd0_fix, "CD0 retrátil deve ser menor que fixo");
    }

    #[test]
    fn ld_ratio_razoavel() {
        // L/D esperado > 12 em cruzeiro para esta classe de aeronave.
        // Medido empiricamente para a fixture sintética (config_teste()):
        // ~12.3 — ainda satisfaz o limiar original de 12.0 (a Task 2.1 tinha
        // baixado este piso para 10.0 sem necessidade; corrigido de volta
        // após code review — ver task-2.1-report.md, correção pós-review).
        let cfg = config_teste();
        let state = AircraftState::from_config(&cfg);
        let req = crate::models::requirements::test_fixtures::requisitos_teste();
        let wing = AerodynamicsAgent::run(&state, &req);
        println!("L/D cruzeiro (fixture sintética) = {:.1}", wing.ld_ratio_cruise);
        assert!(wing.ld_ratio_cruise > 12.0,
            "L/D {:.1} abaixo do mínimo de 12", wing.ld_ratio_cruise);
    }

    #[test]
    fn mach_tip_abaixo_de_0_75() {
        // Regra: Mach na ponta da hélice < 0.75 para evitar ruído e perda de eficiência
        // Prop 1.95m @ 1.500 rpm, V=77.8 m/s, velocidade do som @ 2.500m ISA
        let a_ms = crate::models::atmosphere::Isa::speed_of_sound_ms(2_500.0, 0.0);
        let m = mach_tip(1.95, 1_500.0, 77.8, a_ms);
        assert!(m < 0.75, "Mach ponta hélice {m:.3} excede limite 0.75");
    }
}
