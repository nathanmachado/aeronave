//! MassModelAgent — massas estruturais por equações de componente
//! (Raymer, "Aircraft Design: A Conceptual Approach", cap. 15.2, equações
//! GA; fatores de composto da Tab. 15.4 vêm de [mass_model] no TOML).
//! Interface em SI; internamente as equações usam unidades imperiais
//! (fidelidade à fonte — expoentes empíricos não-dimensionalizáveis).

/// lb por kg (NIST)
pub const LB_PER_KG: f64 = 2.20462;
/// ft² por m²
pub const FT2_PER_M2: f64 = 10.7639;
/// ft por m
pub const FT_PER_M: f64 = 3.28084;
/// psf por Pa
pub const PSF_PER_PA: f64 = 0.020885;
/// gal US por litro
pub const GAL_PER_L: f64 = 0.264172;
/// polegadas por metro
pub const IN_PER_M: f64 = 39.3701;

/// Nº de tanques (asas integrais, um por semi-asa) — constante de layout
/// desta configuração, não dado de projeto variável (Raymer eq. 15.2,
/// expoente fraco 0.242).
const N_TANKS: f64 = 2.0;
/// Nº de motores (monomotor — expoente 0.157).
const N_ENGINES: f64 = 1.0;

/// As 7 massas estruturais computadas (kg) — spec ciclo 3.
#[derive(Debug, Clone)]
pub struct StructuralMasses {
    pub asa_kg: f64,
    pub fuselagem_kg: f64,
    pub emp_h_kg: f64,
    pub emp_v_kg: f64,
    pub trem_principal_kg: f64,
    pub trem_nariz_kg: f64,
    pub tanques_kg: f64,
}

/// Raymer 15.2 (GA), asa sem enflechamento (cos Λ = 1):
/// W = 0.036·S^0.758·W_fw^0.0035·A^0.6·q^0.006·λ^0.04·(100 t/c)^-0.3·(N_z·W_dg)^0.49
pub fn wing_mass_raymer_kg(
    s_w_m2: f64, w_fw_kg: f64, ar: f64, q_pa: f64,
    taper: f64, t_c: f64, n_z_ult: f64, w_dg_kg: f64,
) -> f64 {
    let s_w = s_w_m2 * FT2_PER_M2;
    let w_fw = w_fw_kg * LB_PER_KG;
    let q = q_pa * PSF_PER_PA;
    let w_dg = w_dg_kg * LB_PER_KG;
    let w_lb = 0.036 * s_w.powf(0.758) * w_fw.powf(0.0035) * ar.powf(0.6)
        * q.powf(0.006) * taper.powf(0.04) * (100.0 * t_c).powf(-0.3)
        * (n_z_ult * w_dg).powf(0.49);
    w_lb / LB_PER_KG
}

/// W_ht = 0.016·(N_z·W_dg)^0.414·q^0.168·S_ht^0.896·(100 t/c)^-0.12·A_h^0.043·λ_h^-0.02
pub fn htail_mass_raymer_kg(
    n_z_ult: f64, w_dg_kg: f64, q_pa: f64, s_ht_m2: f64,
    t_c: f64, ar_h: f64, taper_h: f64,
) -> f64 {
    let w_dg = w_dg_kg * LB_PER_KG;
    let q = q_pa * PSF_PER_PA;
    let s_ht = s_ht_m2 * FT2_PER_M2;
    let w_lb = 0.016 * (n_z_ult * w_dg).powf(0.414) * q.powf(0.168)
        * s_ht.powf(0.896) * (100.0 * t_c).powf(-0.12) * ar_h.powf(0.043)
        * taper_h.powf(-0.02);
    w_lb / LB_PER_KG
}

/// W_vt = 0.073·(1+0.2·H_t/H_v)·(N_z·W_dg)^0.376·q^0.122·S_vt^0.873·
///        (100 t/c)^-0.49·A_v^0.357·λ_v^0.039 — cauda convencional:
///        H_t/H_v = 0 (estabilizador na fuselagem), fator = 1.0.
pub fn vtail_mass_raymer_kg(
    n_z_ult: f64, w_dg_kg: f64, q_pa: f64, s_vt_m2: f64,
    t_c: f64, ar_v: f64, taper_v: f64,
) -> f64 {
    let w_dg = w_dg_kg * LB_PER_KG;
    let q = q_pa * PSF_PER_PA;
    let s_vt = s_vt_m2 * FT2_PER_M2;
    let w_lb = 0.073 * 1.0 * (n_z_ult * w_dg).powf(0.376) * q.powf(0.122)
        * s_vt.powf(0.873) * (100.0 * t_c).powf(-0.49) * ar_v.powf(0.357)
        * taper_v.powf(0.039);
    w_lb / LB_PER_KG
}

/// W_fus = 0.052·S_f^1.086·(N_z·W_dg)^0.177·L_t^-0.051·(L/D)^-0.072·q^0.241
/// — SEM o termo de pressurização (cabine não pressurizada, spec).
pub fn fuselage_mass_raymer_kg(
    s_f_m2: f64, n_z_ult: f64, w_dg_kg: f64, l_t_m: f64,
    l_over_d: f64, q_pa: f64,
) -> f64 {
    let s_f = s_f_m2 * FT2_PER_M2;
    let w_dg = w_dg_kg * LB_PER_KG;
    let l_t = l_t_m * FT_PER_M;
    let q = q_pa * PSF_PER_PA;
    let w_lb = 0.052 * s_f.powf(1.086) * (n_z_ult * w_dg).powf(0.177)
        * l_t.powf(-0.051) * l_over_d.powf(-0.072) * q.powf(0.241);
    w_lb / LB_PER_KG
}

/// W_mg = 0.095·(N_l·W_l)^0.768·(L_m/12)^0.409 — L_m em polegadas.
pub fn main_gear_mass_raymer_kg(n_l_ult: f64, w_l_kg: f64, strut_len_m: f64) -> f64 {
    let w_l = w_l_kg * LB_PER_KG;
    let l_m_in = strut_len_m * IN_PER_M;
    let w_lb = 0.095 * (n_l_ult * w_l).powf(0.768) * (l_m_in / 12.0).powf(0.409);
    w_lb / LB_PER_KG
}

/// W_ng = 0.125·(N_l·W_l)^0.566·(L_n/12)^0.845 — L_n em polegadas.
pub fn nose_gear_mass_raymer_kg(n_l_ult: f64, w_l_kg: f64, strut_len_m: f64) -> f64 {
    let w_l = w_l_kg * LB_PER_KG;
    let l_n_in = strut_len_m * IN_PER_M;
    let w_lb = 0.125 * (n_l_ult * w_l).powf(0.566) * (l_n_in / 12.0).powf(0.845);
    w_lb / LB_PER_KG
}

/// W_fs = 2.49·V_t^0.726·(1/(1+V_i/V_t))^0.363·N_t^0.242·N_en^0.157 —
/// V_t em galões US; tanques INTEGRAIS (V_i/V_t = 1, spec: "tanques
/// integrais compostos ≈ metálicos").
pub fn fuel_system_mass_raymer_kg(capacity_l: f64) -> f64 {
    let v_t = capacity_l * GAL_PER_L;
    let w_lb = 2.49 * v_t.powf(0.726) * (1.0_f64 / (1.0 + 1.0)).powf(0.363)
        * N_TANKS.powf(0.242) * N_ENGINES.powf(0.157);
    w_lb / LB_PER_KG
}

#[cfg(test)]
mod tests {
    use super::*;

    // Entradas E7 congeladas (ver tabela do plano — NÃO ler TOML real aqui).
    const W_DG_KG: f64 = 1548.4;
    const N_Z_ULT: f64 = 6.286149;
    const Q_PA: f64 = 3366.1331;

    #[test]
    fn hand_check_asa_raw_e_com_fator_de_composto() {
        let raw = wing_mass_raymer_kg(14.2, 218.4, 10.03969014084507, Q_PA, 0.45, 0.15, N_Z_ULT, W_DG_KG);
        assert!((raw - 176.17).abs() < 0.1, "asa raw = {raw:.2} kg (esperado 176.17 ±0.1)");
        let comp = raw * 0.85;
        assert!((comp - 149.74).abs() < 0.1, "asa ×0.85 = {comp:.2} kg (esperado 149.74 ±0.1)");
    }

    #[test]
    fn hand_check_empenagem_horizontal() {
        let raw = htail_mass_raymer_kg(N_Z_ULT, W_DG_KG, Q_PA, 3.133966, 0.15, 4.0, 0.5);
        assert!((raw - 16.76).abs() < 0.1, "EH raw = {raw:.2} kg (esperado 16.76 ±0.1)");
        assert!((raw * 0.83 - 13.91).abs() < 0.1);
    }

    #[test]
    fn hand_check_empenagem_vertical() {
        let raw = vtail_mass_raymer_kg(N_Z_ULT, W_DG_KG, Q_PA, 1.412900, 0.15, 1.5, 0.5);
        assert!((raw - 7.60).abs() < 0.1, "EV raw = {raw:.2} kg (esperado 7.60 ±0.1)");
        assert!((raw * 0.83 - 6.31).abs() < 0.1);
    }

    #[test]
    fn hand_check_fuselagem() {
        let raw = fuselage_mass_raymer_kg(25.117033, N_Z_ULT, W_DG_KG, 4.8, 6.307692, Q_PA);
        assert!((raw - 127.91).abs() < 0.1, "fuselagem raw = {raw:.2} kg (esperado 127.91 ±0.1)");
        assert!((raw * 0.90 - 115.12).abs() < 0.1);
    }

    #[test]
    fn hand_check_trem_principal() {
        let raw = main_gear_mass_raymer_kg(4.5, W_DG_KG, 0.67);
        assert!((raw - 97.60).abs() < 0.1, "trem principal raw = {raw:.2} kg (esperado 97.60 ±0.1)");
        assert!((raw * 0.95 - 92.72).abs() < 0.1);
    }

    #[test]
    fn hand_check_trem_nariz() {
        let raw = nose_gear_mass_raymer_kg(4.5, W_DG_KG, 0.53);
        assert!((raw - 21.19).abs() < 0.1, "trem nariz raw = {raw:.2} kg (esperado 21.19 ±0.1)");
        assert!((raw * 0.95 - 20.13).abs() < 0.1);
    }

    #[test]
    fn hand_check_sistema_de_combustivel() {
        let raw = fuel_system_mass_raymer_kg(260.0);
        assert!((raw - 22.39).abs() < 0.1, "tanques raw = {raw:.2} kg (esperado 22.39 ±0.1)");
    }

    // ─── Propriedades ESTRITAS de direção (spec, seção Testes item 2) ───
    #[test]
    fn massa_da_asa_cresce_com_area_e_com_n_z() {
        let base = wing_mass_raymer_kg(14.2, 218.4, 10.04, Q_PA, 0.45, 0.15, N_Z_ULT, W_DG_KG);
        let s_maior = wing_mass_raymer_kg(14.2 * 1.2, 218.4, 10.04, Q_PA, 0.45, 0.15, N_Z_ULT, W_DG_KG);
        let nz_maior = wing_mass_raymer_kg(14.2, 218.4, 10.04, Q_PA, 0.45, 0.15, N_Z_ULT * 1.2, W_DG_KG);
        assert!(s_maior > base, "∂m_asa/∂S > 0: base={base:.2} s_maior={s_maior:.2}");
        assert!(nz_maior > base, "∂m_asa/∂N_z > 0: base={base:.2} nz_maior={nz_maior:.2}");
    }

    #[test]
    fn massa_do_trem_cresce_com_peso_e_tanques_com_capacidade() {
        let mg_base = main_gear_mass_raymer_kg(4.5, W_DG_KG, 0.67);
        let mg_pesado = main_gear_mass_raymer_kg(4.5, W_DG_KG * 1.2, 0.67);
        assert!(mg_pesado > mg_base, "∂m_trem/∂MTOW > 0");
        let fs_base = fuel_system_mass_raymer_kg(260.0);
        let fs_maior = fuel_system_mass_raymer_kg(320.0);
        assert!(fs_maior > fs_base, "∂m_tanques/∂capacidade > 0");
    }
}
