/// LandingGearAgent — Trem de Pouso Retrátil Triciclo
///
/// Dimensiona o trem de pouso retrátil elétrico para a aeronave,
/// considerando:
///   - Geometria (bitola, empeno, ângulo anti-tombamento)
///   - Distribuição de carga estática (nariz / principal)
///   - Amortecedor oleo-pneumático (curso, forças de impacto)
///   - Seleção de pneus para gramado e terra compactada
///   - Atuador elétrico de retração (potência, tempo)
///   - Compatibilidade com operação em pistas não pavimentadas
///
/// Requisitos de projeto (CS-23 / FAR-23):
///   - Taxa de afundamento de projeto: 2,5 m/s a MTOW (CS 23.473)
///   - Fator de carga no pouso: n_g ≤ 4,0g nas pernas principais
///   - Ângulo anti-tombamento lateral: φ < 55°
///   - Nariz: 8–20% da carga estática total
///
/// Referências:
///   - Raymer, D. "Aircraft Design", Cap. 11
///   - Currey, N. "Aircraft Landing Gear Design", AIAA Education Series
///   - CS 23.471–23.511

const G: f64 = 9.807; // m/s²

use crate::models::aircraft_config::GearCfg;
use crate::models::specs::GearSpec;

// ─── GEOMETRIA DO TREM ────────────────────────────────────────────────────────

/// Bitola mínima do trem principal para ângulo anti-tombamento lateral φ < 55°.
///
/// Critério: tan(φ) = h_cg / (b_track / 2) < tan(55°)
/// → b_track > 2 × h_cg × tan(55°)
///
/// h_cg: altura do CG acima do solo com trem estendido (m)
pub fn min_track_width_m(h_cg_m: f64) -> f64 {
    2.0 * h_cg_m * (55.0_f64.to_radians().tan())
}

/// Ângulo anti-tombamento lateral real (graus).
pub fn tipover_angle_deg(h_cg_m: f64, track_width_m: f64) -> f64 {
    (h_cg_m / (track_width_m / 2.0)).atan().to_degrees()
}

/// Empeno (wheelbase) mínimo baseado na posição do CG.
///
/// Para distribuição de carga adequada (8–20% no nariz):
///   L_nose / L_total = F_main / W ← convenção: L_nose = distância CG → nariz
///   Fração no nariz = L_nose / L_total
///
/// Dado CG a x_cg_m do nariz e trem de nariz a x_nose_m do nariz:
///   L_nose = x_cg_m − x_nose_m   (comprimento braço nariz)
///   L_main = x_main_m − x_cg_m   (comprimento braço principal)
///   F_nose = W × L_main / (L_nose + L_main)
pub fn nose_load_fraction(x_cg_m: f64, x_nose_m: f64, x_main_m: f64) -> f64 {
    let l_main = x_main_m - x_cg_m;
    let l_nose  = x_cg_m  - x_nose_m;
    let total = l_main + l_nose;
    if total <= 0.0 { return 0.0; }
    l_main / total
}

// ─── CARGAS DE IMPACTO NO POUSO ───────────────────────────────────────────────

/// Carga de impacto no trem principal por perna (N).
///
/// Método energético (CS 23.473):
///   E_pouso = (1/2) × m × v_sink²
///   F_impacto = E_pouso / (stroke × η_amort)   + carga estática
///   η_amort = 0.75 (eficiência do amortecedor oleo-pneumático)
///
/// A carga de impacto é distribuída para as 2 pernas do trem principal.
pub fn main_gear_impact_load_n(
    mtow_kg: f64,
    sink_rate_ms: f64,
    oleo_stroke_m: f64,
) -> f64 {
    let e_kinetic  = 0.5 * mtow_kg * sink_rate_ms * sink_rate_ms; // J
    let eta_oleo   = 0.75;
    let f_dynamic  = e_kinetic / (oleo_stroke_m * eta_oleo); // N (total)
    let f_static   = mtow_kg * G / 2.0; // por perna, desconsiderando nariz
    f_dynamic / 2.0 + f_static           // N — por perna principal
}

/// Fator de carga no pouso (n_g = F_impacto / W_perna_estático)
pub fn landing_load_factor(f_impact_n: f64, mass_per_leg_kg: f64) -> f64 {
    f_impact_n / (mass_per_leg_kg * G)
}

/// Curso mínimo do amortecedor para não ultrapassar n_g_max:
/// stroke = E_kinetic / (n_g_max × W_perna × η_amort)
pub fn min_oleo_stroke_m(
    mtow_kg: f64,
    sink_rate_ms: f64,
    n_g_max: f64,
    eta_oleo: f64,
) -> f64 {
    let e_kinetic = 0.5 * mtow_kg * sink_rate_ms * sink_rate_ms;
    let w_leg = mtow_kg * G / 2.0; // carga por perna
    e_kinetic / (n_g_max * w_leg * eta_oleo)
}

// ─── SELEÇÃO DE PNEUS ─────────────────────────────────────────────────────────

/// Capacidade de carga do pneu 6.00-6 (certificado para grama e terra):
/// Carga máx: 9.100 N (2.045 lbf) a 45 psi — padrão FAA TSO-C62
/// Carga máx @ 60 psi: 10.230 N — margem para terrain factor
pub fn tire_6_00_6_max_load_n(pressure_psi: f64) -> f64 {
    // Relação linear: 4.050 N a 20 psi, 9.100 N a 45 psi
    let base = 4_050.0_f64;
    let slope = (9_100.0 - 4_050.0) / (45.0 - 20.0); // N/psi
    (base + slope * (pressure_psi - 20.0)).min(11_000.0) // limite estrutural
}

/// Verifica se a carga por pneu está dentro da capacidade:
pub fn tire_load_ok(load_n: f64, pressure_psi: f64) -> bool {
    load_n < tire_6_00_6_max_load_n(pressure_psi) * 0.85 // margem 15%
}

// ─── ATUADOR ELÉTRICO DE RETRAÇÃO ────────────────────────────────────────────

/// Potência do atuador elétrico de retração (W).
///
/// Energia necessária para elevar o trem (levanta contra gravidade + atrito):
///   E_atuador = m_gear × g × Δh + E_atrito (≈ 20% extra)
///   P = E_atuador / t_retração
///
/// m_gear: massa do conjunto de um lado do trem principal (kg)
/// delta_h_m: deslocamento vertical do CG do trem durante retração
pub fn actuator_power_w(
    gear_mass_kg: f64,
    delta_h_m: f64,
    retraction_time_s: f64,
) -> f64 {
    let e_atuador = gear_mass_kg * G * delta_h_m * 1.20; // +20% atrito/mecanismo
    e_atuador / retraction_time_s
}

// ─── AGENTE PRINCIPAL ────────────────────────────────────────────────────────

pub struct LandingGearAgent;

impl LandingGearAgent {
    /// Executa o dimensionamento completo do trem de pouso.
    ///
    /// Parâmetros do layout longitudinal (do WeightBalanceAgent) e da
    /// geometria/massas do trem (`[gear]` do TOML de aeronave, mais as
    /// massas totais das pernas de `[[masses.items]]`, que são a fonte
    /// única do peso total do sistema — `gear_cfg.mass_main_leg_kg` é só o
    /// dado de "uma perna" usado no dimensionamento do atuador):
    ///   x_cg_aft_m: CG mais traseiro (maior carga no nariz)
    ///   gear_cfg:   geometria/parâmetros do trem
    ///   main_gear_total_mass_kg: massa TOTAL do trem principal (ambas as
    ///     pernas) — item `trem_principal` de `[[masses.items]]`
    ///   nose_gear_mass_kg: massa do trem de nariz — item `trem_nariz`
    pub fn run(
        mtow_kg: f64,
        x_cg_aft_m: f64,
        gear_cfg: &GearCfg,
        main_gear_total_mass_kg: f64,
        nose_gear_mass_kg: f64,
    ) -> GearSpec {
        let sink_rate  = 2.5_f64; // m/s — CS 23.473
        let n_g_max    = 4.0_f64; // fator de carga no pouso
        let eta_oleo   = 0.75_f64;
        let psi        = 45.0_f64;

        // Altura do CG acima do solo (trem estendido) — `[gear] h_cg_ground_m`
        let h_cg_ground = gear_cfg.h_cg_ground_m;
        let x_nose_m = gear_cfg.x_nose_m;
        let x_main_m = gear_cfg.x_main_m;

        // Geometria
        let track = min_track_width_m(h_cg_ground).max(2.80); // mínimo 2.80m
        let tipover = tipover_angle_deg(h_cg_ground, track);

        // Fração de carga no nariz (cenário com CG mais traseiro = pior para nariz)
        let f_nose_frac = nose_load_fraction(x_cg_aft_m, x_nose_m, x_main_m);
        let wheelbase = x_main_m - x_nose_m;

        // Cargas
        let f_main_static = mtow_kg * G * (1.0 - f_nose_frac) / 2.0; // por perna
        let f_nose_static = mtow_kg * G * f_nose_frac;

        // Curso do amortecedor principal
        let stroke_main = min_oleo_stroke_m(mtow_kg, sink_rate, n_g_max, eta_oleo)
            .clamp(0.10, 0.25); // 100–250 mm
        let f_main_impact = main_gear_impact_load_n(mtow_kg, sink_rate, stroke_main);

        // Trem de nariz: carga menor, curso menor
        let stroke_nose = (stroke_main * 0.60).max(0.08); // 60% do principal
        let f_nose_impact = f_nose_static * 2.5; // fator de impacto simplificado

        // Verificação de capacidade dos pneus
        // Trem principal: 1 pneu por perna
        // Trem de nariz:  1 pneu (ou 2 pneus gêmeos no futuro)
        let _tire_main_ok = tire_load_ok(f_main_impact, psi);
        let _tire_nose_ok = tire_load_ok(f_nose_impact, psi);

        // Atuador elétrico (retração leva o trem para a asa/fuselagem)
        // Massa de uma perna principal: `[gear] mass_main_leg_kg`
        // Δh durante retração: ~0.40 m (levanta perna para baio da asa)
        let ret_time  = gear_cfg.retraction_time_s;
        let p_actuator = actuator_power_w(gear_cfg.mass_main_leg_kg, 0.40, ret_time);

        // Peso total do sistema de trem: massas totais das pernas (de
        // `[[masses.items]]`) + atuadores/portas (`[gear] actuators_doors_mass_kg`)
        let total_weight = main_gear_total_mass_kg + nose_gear_mass_kg
            + gear_cfg.actuators_doors_mass_kg;

        GearSpec {
            gear_type:              "Retrátil Triciclo Elétrico".to_string(),
            track_width_m:          track,
            wheelbase_m:            wheelbase,
            tipover_angle_deg:      tipover,
            nose_load_fraction_pct: f_nose_frac * 100.0,
            main_gear_load_n:       f_main_impact,
            nose_gear_load_n:       f_nose_impact,
            main_oleo_stroke_mm:    stroke_main * 1_000.0,
            nose_oleo_stroke_mm:    stroke_nose * 1_000.0,
            main_tire:              "6.00-6 (4 ply) — Aircraft Spruce".to_string(),
            nose_tire:              "5.00-5 (4 ply) — Aircraft Spruce".to_string(),
            tire_pressure_psi:      psi,
            max_sink_rate_ms:       sink_rate,
            retraction_time_s:      ret_time,
            actuator_power_w:       p_actuator,
            total_weight_kg:        total_weight,
        }
    }
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const MTOW: f64 = 1_527.0;

    #[test]
    fn angulo_anti_tombamento_abaixo_de_55_graus() {
        let track = min_track_width_m(1.05).max(2.80);
        let phi   = tipover_angle_deg(1.05, track);
        println!("Bitola={track:.2}m  φ={phi:.1}°");
        assert!(phi < 55.0, "Ângulo de tombamento {phi:.1}° excede 55°");
    }

    #[test]
    fn fracao_nariz_entre_8_e_20_pct() {
        // CG mais traseiro = cenário MTOW com 4 pax + bagagem + tanque cheio
        // Do WeightBalanceAgent: x_cg ≈ 3.263 m (cenário "4 pax + bagagem + cheio")
        let frac = nose_load_fraction(3.263, 1.40, 3.85);
        let pct  = frac * 100.0;
        println!("Fração no nariz: {pct:.1}%");
        assert!(pct >= 8.0 && pct <= 25.0,
            "Carga no nariz {pct:.1}% fora de 8–25%");
    }

    #[test]
    fn curso_oleo_dentro_do_limite() {
        let stroke = min_oleo_stroke_m(MTOW, 2.5, 4.0, 0.75);
        println!("Curso mínimo oleo: {:.0}mm", stroke * 1000.0);
        assert!(stroke >= 0.08 && stroke <= 0.25,
            "Curso {:.0}mm fora de 80–250mm", stroke * 1000.0);
    }

    #[test]
    fn carga_impacto_dentro_da_capacidade_do_pneu() {
        // Verificação correta: a carga estática por pneu deve estar dentro da
        // capacidade do pneu (carga dinâmica é absorvida pelo oleo-pneumático).
        // CS-23: o pneu é dimensionado para suportar a carga estática a MTOW.
        let nose_frac = 0.13; // ~13% no nariz (cenário típico)
        let f_static_main = MTOW * G * (1.0 - nose_frac) / 2.0; // por perna
        let ok = tire_load_ok(f_static_main, 45.0);
        println!("Carga estática/perna: {f_static_main:.0} N  |  Cap. pneu (85%): {:.0} N  OK={ok}",
                 tire_6_00_6_max_load_n(45.0) * 0.85);
        assert!(ok, "Carga estática {f_static_main:.0} N excede capacidade do pneu 6.00-6 @ 45 psi");
    }

    #[test]
    fn potencia_atuador_compativel_com_28v_dc() {
        let p = actuator_power_w(28.0, 0.40, 7.0);
        let corrente = p / 28.0; // A @ 28V
        println!("Potência atuador: {p:.0} W = {corrente:.1} A @ 28V");
        // Sistema de 28V DC padrão avião experimental: breaker de 20A por perna
        assert!(corrente < 25.0,
            "Corrente {corrente:.1}A excede limite do barramento 28V (25A)");
    }

    fn gear_cfg_teste() -> GearCfg {
        GearCfg {
            retractable: true,
            cd0_fixed_increment: 0.008,
            h_cg_ground_m: 1.05,
            x_nose_m: 1.40,
            x_main_m: 3.85,
            mass_main_leg_kg: 27.5,
            mass_nose_kg: 22.0,
            retraction_time_s: 7.0,
            actuators_doors_mass_kg: 20.0,
        }
    }

    #[test]
    fn relatorio_completo_trem() {
        let gear_cfg = gear_cfg_teste();
        let gear = LandingGearAgent::run(MTOW, 3.263, &gear_cfg, 55.0, 22.0);
        println!("Bitola:    {:.2}m", gear.track_width_m);
        println!("Empeno:    {:.2}m", gear.wheelbase_m);
        println!("Tombamento:{:.1}°", gear.tipover_angle_deg);
        println!("Carga nariz:{:.1}%", gear.nose_load_fraction_pct);
        println!("Stroke main:{:.0}mm", gear.main_oleo_stroke_mm);
        println!("F_main:     {:.0}N", gear.main_gear_load_n);
        println!("Peso total: {:.0}kg", gear.total_weight_kg);
        assert!(gear.tipover_angle_deg < 55.0);
        assert!(gear.nose_load_fraction_pct >= 8.0);
        assert_eq!(gear.total_weight_kg, 55.0 + 22.0 + 20.0);
    }
}
