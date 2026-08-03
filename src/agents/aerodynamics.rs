/// AerodynamicsAgent
///
/// Calcula o polar de arrasto, parâmetros de asa e velocidade de stall.
/// Todas as equações seguem a Mecânica do Voo clássica (Anderson, Raymer).
///
/// Convenções de unidades: SI puro (m, kg, N, m/s, Pa).
/// Velocidades em km/h são convertidas internamente.

use crate::models::{
    aircraft_state::AircraftState,
    requirements::Requirements,
    specs::WingSpec,
};

// Constantes atmosféricas (ISA — International Standard Atmosphere)
const RHO_SL: f64 = 1.225;      // kg/m³ — densidade ao nível do mar
const G: f64 = 9.807;            // m/s²

// Parâmetros do perfil NACA 23015
const CL_MAX_23015: f64 = 1.72;  // com slats simples / flap Fowler parcial
const CL_MAX_CLEAN: f64 = 1.45;  // asa limpa (cruzeiro)
const CD0_FUSELAGE: f64 = 0.010; // parcela da fuselagem
const CD0_EMPENAGEM: f64 = 0.004;// parcela da empenagem
const CD0_GEAR_RETRACTABLE: f64 = 0.0;   // trem recolhido
const CD0_MISC: f64 = 0.003;     // antenas, juntas, imperfeições

/// Densidade atmosférica ISA em altitude h (metros), aproximação exponencial.
/// Válida até ~11.000 m (troposfera).
pub fn isa_density(altitude_m: f64) -> f64 {
    RHO_SL * (1.0 - 0.0000226 * altitude_m).powf(4.256)
}

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
/// Segue o método de "component drag build-up" (Raymer cap. 12).
pub fn cd0_total(cd0_wing: f64, gear_retractable: bool) -> f64 {
    let gear_cd0 = if gear_retractable {
        CD0_GEAR_RETRACTABLE
    } else {
        0.008  // trem fixo de pista de terra
    };
    cd0_wing + CD0_FUSELAGE + CD0_EMPENAGEM + gear_cd0 + CD0_MISC
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

/// CD0 da asa isolada pelo método de espessura relativa do perfil.
/// Para NACA 23015 (t/c = 0.15): CD0_asa ≈ 0.0050
pub fn cd0_wing_naca23015() -> f64 {
    0.0050
}

/// Razão L/D em cruzeiro
pub fn ld_ratio(cl: f64, cd: f64) -> f64 {
    cl / cd
}

/// Número de Mach da ponta da hélice (verificação de compressibilidade)
pub fn mach_tip(prop_diameter_m: f64, prop_rpm: f64, v_cruise_ms: f64) -> f64 {
    let tip_speed = std::f64::consts::PI * prop_diameter_m * (prop_rpm / 60.0);
    let total_speed = (tip_speed * tip_speed + v_cruise_ms * v_cruise_ms).sqrt();
    total_speed / 340.0  // velocidade do som @ 2.500m ISA ≈ 340 m/s
}

// ─── AGENTE PRINCIPAL ────────────────────────────────────────────────────────

pub struct AerodynamicsAgent;

impl AerodynamicsAgent {
    /// Executa o agente e retorna a especificação aerodinâmica completa.
    pub fn run(state: &AircraftState, req: &Requirements) -> WingSpec {
        let rho_cruise = isa_density(req.cruise_altitude_m);
        let v_cruise_ms = req.cruise_speed_min_kmh / 3.6;
        let weight_n = state.mtow_kg * G;

        let q_cruise = dynamic_pressure(rho_cruise, v_cruise_ms);
        let ar = state.aspect_ratio();
        let oswald = oswald_efficiency(ar);

        let cd0_wing = cd0_wing_naca23015();
        let cd0 = cd0_total(cd0_wing, state.gear_retractable);

        let cl_cruise = cl_required(weight_n, q_cruise, state.wing_area_m2);
        let cdi = cd_induced(cl_cruise, ar, oswald);
        let cd_cruise = cd0 + cdi;
        let ld = ld_ratio(cl_cruise, cd_cruise);

        // Velocidades de stall ao nível do mar (condição mais crítica).
        // VS0 — configuração com flap/pouso (CL_max maior → V_stall MENOR).
        let v_stall_flaps_ms = stall_speed_ms(weight_n, RHO_SL, state.wing_area_m2, CL_MAX_23015);
        // VS1 — configuração limpa/cruzeiro (CL_max menor → V_stall MAIOR).
        let v_stall_clean_ms = stall_speed_ms(weight_n, RHO_SL, state.wing_area_m2, CL_MAX_CLEAN);

        WingSpec {
            span_m:           state.wing_span_m,
            area_m2:          state.wing_area_m2,
            aspect_ratio:     ar,
            airfoil:          state.airfoil.clone(),
            taper_ratio:      state.taper_ratio,
            oswald_efficiency: oswald,
            cd0,
            cl_cruise,
            cd_cruise,
            cl_max:                 CL_MAX_23015,
            cl_max_clean:           CL_MAX_CLEAN,
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

    #[test]
    fn isa_density_sea_level() {
        let rho = isa_density(0.0);
        assert!((rho - RHO_SL).abs() < 1e-6, "densidade SL incorreta: {rho}");
    }

    #[test]
    fn isa_density_decresce_com_altitude() {
        assert!(isa_density(2_500.0) < isa_density(0.0));
        assert!(isa_density(5_000.0) < isa_density(2_500.0));
    }

    #[test]
    fn stall_speed_fisica() {
        // Aeronave de 1.461 kg, S=14.2m², CLmax=1.72 (flapada) → V_stall ~ 90-115 km/h
        let vs = stall_speed_ms(1_461.0 * G, RHO_SL, 14.2, CL_MAX_23015);
        let vs_kmh = vs * 3.6;
        assert!(vs_kmh > 90.0 && vs_kmh < 115.0,
            "V_stall esperada 90-115 km/h, obtida {vs_kmh:.1} km/h");
    }

    #[test]
    fn vs0_flapada_menor_que_vs1_limpa() {
        // Task 0.5: VS0 (com flap, CL_max=1.72) deve ser MENOR que VS1 (limpa,
        // CL_max=1.45) — flap aumenta CL_max, o que reduz a velocidade de stall.
        let state = AircraftState::initial();
        let req = Requirements::project_default();
        let wing = AerodynamicsAgent::run(&state, &req);

        println!("VS0 (flap) = {:.1} km/h | VS1 (limpa) = {:.1} km/h",
                 wing.stall_speed_flaps_kmh, wing.stall_speed_clean_kmh);

        assert!(wing.stall_speed_flaps_kmh < wing.stall_speed_clean_kmh,
            "VS0 (flap) {:.1} km/h deveria ser menor que VS1 (limpa) {:.1} km/h",
            wing.stall_speed_flaps_kmh, wing.stall_speed_clean_kmh);

        // Faixas físicas para MTOW=1.461kg, S=14.2m², ρ_SL=1.225, g=9.807:
        // VS0 (CL=1.72) ≈ 111 km/h; VS1 (CL=1.45) ≈ 121 km/h.
        assert!(wing.stall_speed_flaps_kmh > 100.0 && wing.stall_speed_flaps_kmh < 118.0,
            "VS0 {:.1} km/h fora da faixa esperada (100-118 km/h)",
            wing.stall_speed_flaps_kmh);
        assert!(wing.stall_speed_clean_kmh > 115.0 && wing.stall_speed_clean_kmh < 128.0,
            "VS1 {:.1} km/h fora da faixa esperada (115-128 km/h)",
            wing.stall_speed_clean_kmh);
    }

    #[test]
    fn cd0_retravel_menor_que_fixo() {
        let cd0_ret = cd0_total(cd0_wing_naca23015(), true);
        let cd0_fix = cd0_total(cd0_wing_naca23015(), false);
        assert!(cd0_ret < cd0_fix, "CD0 retrátil deve ser menor que fixo");
    }

    #[test]
    fn ld_ratio_razoavel() {
        // L/D esperado > 12 em cruzeiro para esta classe de aeronave
        let state = AircraftState::initial();
        let req = Requirements::project_default();
        let wing = AerodynamicsAgent::run(&state, &req);
        assert!(wing.ld_ratio_cruise > 12.0,
            "L/D {:.1} abaixo do mínimo de 12", wing.ld_ratio_cruise);
    }

    #[test]
    fn mach_tip_abaixo_de_0_75() {
        // Regra: Mach na ponta da hélice < 0.75 para evitar ruído e perda de eficiência
        // Prop 1.95m @ 1.500 rpm, V=77.8 m/s
        let m = mach_tip(1.95, 1_500.0, 77.8);
        assert!(m < 0.75, "Mach ponta hélice {m:.3} excede limite 0.75");
    }
}
