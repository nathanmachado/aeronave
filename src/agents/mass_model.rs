//! MassModelAgent — massas estruturais por equações de componente
//! (Raymer, "Aircraft Design: A Conceptual Approach", cap. 15.2, equações
//! GA; fatores de composto da Tab. 15.4 vêm de [mass_model] no TOML).
//! Interface em SI; internamente as equações usam unidades imperiais
//! (fidelidade à fonte — expoentes empíricos não-dimensionalizáveis).
//!
//! **Nota histórica — t/c das empenagens (RESOLVIDO no ciclo 4)**: antes do
//! ciclo 4, `htail_mass_raymer_kg`/`vtail_mass_raymer_kg` usavam o MESMO
//! espessura relativa da ASA (`[wing].thickness_ratio`, 0,15 no baseline)
//! porque `EmpennageCfg` não tinha um campo dedicado — empenagens GA
//! tipicamente usam perfis mais finos (t/c ~0,10). O expoente de
//! `(100·t/c)` é −0,49 na equação do EV e −0,12 na do EH (Raymer Tab.
//! 15.2), então usar 0,15 em vez de ~0,10 SUBESTIMAVA a massa do EV em
//! ~21% e a do EH em ~5% — (1,5)^0,49 ≈ 1,22 e (1,5)^0,12 ≈ 1,05, onde
//! 1,5 = 0,15/0,10. Impacto medido no baseline (pré-ciclo-4): ~+2 kg de
//! massa de empenagem, a um braço de ~7,3 m do CG, o que deslocava o CG
//! vazio em ~0,8 pp de MAC para a frente — dentro do gap de 4,7 pp
//! identificado no achado do ciclo (ver task-4-report.md/task-1-report.md
//! da task de revisão que precedeu este ciclo). Resolvido no ciclo 4:
//! `[empennage].thickness_ratio`, campo dedicado consumido abaixo.

use crate::models::aircraft_config::AircraftConfig;
use crate::models::atmosphere::Isa;
use crate::models::engine::EngineSpec;
use crate::models::requirements::Requirements;
use crate::models::specs::{EmpennageSpec, WingSpec};

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

/// Agente que aplica as funções puras acima com as entradas derivadas da
/// configuração/estado corrente da aeronave (q de cruzeiro, W_fw, S_f
/// molhada, N_z ultimate) e os fatores de composto de `[mass_model]`.
pub struct MassModelAgent;

impl MassModelAgent {
    /// `n_design`: fator de carga LIMITE (o agente aplica ×1.5 para o
    /// ultimate N_z). No laço do orchestrator vem com LAG-1 (iteração
    /// anterior; seed 3.8 → N_z 5.70) — ver `orchestrator::size_aircraft`.
    /// q de cruzeiro vem do REQUISITO (cruise_speed_min_kmh + ISA na
    /// altitude de missão), não da velocidade real da iteração — expoentes
    /// de q são fracos (0.006–0.241), erro ≤3% (spec).
    pub fn run(
        cfg: &AircraftConfig, engine: &EngineSpec, req: &Requirements,
        wing: &WingSpec, emp: &EmpennageSpec, mtow_kg: f64, n_design: f64,
    ) -> StructuralMasses {
        assert!(mtow_kg > 0.0, "MTOW deve ser positivo, obtido {mtow_kg}");
        assert!(n_design > 0.0, "n_design deve ser positivo, obtido {n_design}");
        let mm = &cfg.mass_model;
        let rho = Isa::density_kgm3(req.cruise_altitude_m, req.isa_delta_c);
        let v_ms = req.cruise_speed_min_kmh / 3.6;
        let q_pa = 0.5 * rho * v_ms * v_ms;
        let w_fw_kg = cfg.fuel_system.capacity_l * engine.fuel.density_kg_per_l;
        let n_z_ult = 1.5 * n_design;
        let t_c_asa = cfg.wing.thickness_ratio;
        let t_c_emp = cfg.empennage.thickness_ratio; // ciclo 4: campo dedicado
        let s_f_m2 = mm.fuselage_wetted_coeff * std::f64::consts::PI
            * mm.d_fus_equiv_m * cfg.fuselage.length_m;
        let l_over_d = cfg.fuselage.length_m / mm.d_fus_equiv_m;
        StructuralMasses {
            asa_kg: wing_mass_raymer_kg(wing.area_m2, w_fw_kg, wing.aspect_ratio,
                q_pa, wing.taper_ratio, t_c_asa, n_z_ult, mtow_kg)
                * mm.composite_factor_wing,
            emp_h_kg: htail_mass_raymer_kg(n_z_ult, mtow_kg, q_pa,
                emp.s_horizontal_m2, t_c_emp, emp.ar_h, emp.taper_h)
                * mm.composite_factor_tail,
            emp_v_kg: vtail_mass_raymer_kg(n_z_ult, mtow_kg, q_pa,
                emp.s_vertical_m2, t_c_emp, emp.ar_v, emp.taper_v)
                * mm.composite_factor_tail,
            fuselagem_kg: fuselage_mass_raymer_kg(s_f_m2, n_z_ult, mtow_kg,
                emp.arm_h_m, l_over_d, q_pa)
                * mm.composite_factor_fuselage,
            trem_principal_kg: main_gear_mass_raymer_kg(
                mm.landing_load_factor_ult, mtow_kg, mm.main_strut_length_m)
                * mm.composite_factor_gear,
            trem_nariz_kg: nose_gear_mass_raymer_kg(
                mm.landing_load_factor_ult, mtow_kg, mm.nose_strut_length_m)
                * mm.composite_factor_gear,
            tanques_kg: fuel_system_mass_raymer_kg(cfg.fuel_system.capacity_l)
                * mm.composite_factor_fuel_system,
        }
    }
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

    // Ciclo 4 (t/c dedicado da empenagem): pins congelados do plano com
    // t/c=0.10 (perfil mais fino que o da asa, 0.15) — demais entradas E7.
    #[test]
    fn hand_check_empenagens_com_t_c_dedicado_0_10() {
        let eh = htail_mass_raymer_kg(N_Z_ULT, W_DG_KG, Q_PA, 3.133966, 0.10, 4.0, 0.5);
        assert!((eh - 17.596).abs() < 0.1, "EH raw t/c=0.10 = {eh:.3} (esperado 17.596 ±0.1)");
        assert!((eh * 0.83 - 14.605).abs() < 0.1);
        let ev = vtail_mass_raymer_kg(N_Z_ULT, W_DG_KG, Q_PA, 1.412900, 0.10, 1.5, 0.5);
        assert!((ev - 9.270).abs() < 0.1, "EV raw t/c=0.10 = {ev:.3} (esperado 9.270 ±0.1)");
        assert!((ev * 0.83 - 7.694).abs() < 0.1);
    }

    // Propriedade: empenagem mais FINA é mais PESADA (expoentes negativos
    // de t/c nas equações Raymer — EH^-0.12, EV^-0.49).
    #[test]
    fn empenagem_mais_fina_e_mais_pesada() {
        let ev_grosso = vtail_mass_raymer_kg(N_Z_ULT, W_DG_KG, Q_PA, 1.4129, 0.15, 1.5, 0.5);
        let ev_fino   = vtail_mass_raymer_kg(N_Z_ULT, W_DG_KG, Q_PA, 1.4129, 0.10, 1.5, 0.5);
        assert!(ev_fino > ev_grosso,
            "EV t/c=0.10 ({ev_fino:.2}) deveria pesar mais que t/c=0.15 ({ev_grosso:.2})");
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

    // ─── MassModelAgent — integração com as demais specs (Task 3) ─────────

    use crate::agents::aerodynamics::AerodynamicsAgent;
    use crate::agents::empennage::EmpennageAgent;
    use crate::models::aircraft_config::test_fixtures::config_teste;
    use crate::models::aircraft_state::AircraftState;
    use crate::models::engine::test_fixtures::motor_generico_teste;
    use crate::models::requirements::test_fixtures::requisitos_teste;

    #[test]
    fn agente_aplica_fatores_de_composto_sobre_as_funcoes_puras() {
        let cfg = config_teste();
        let engine = motor_generico_teste();
        let req = requisitos_teste();
        let state = AircraftState::from_config(&cfg);
        let wing = AerodynamicsAgent::run(&state, &req);
        let emp = EmpennageAgent::run(&wing, &cfg);
        let mtow = 1_400.0;
        let n_design = 4.0;
        let m = MassModelAgent::run(&cfg, &engine, &req, &wing, &emp, mtow, n_design);

        // Reconstrói as MESMAS entradas derivadas que o agente deve usar:
        let rho = Isa::density_kgm3(req.cruise_altitude_m, req.isa_delta_c);
        let v_ms = req.cruise_speed_min_kmh / 3.6;
        let q_pa = 0.5 * rho * v_ms * v_ms;
        let w_fw = cfg.fuel_system.capacity_l * engine.fuel.density_kg_per_l;
        let n_z_ult = 1.5 * n_design;
        let t_c_asa = cfg.wing.thickness_ratio;
        let t_c_emp = cfg.empennage.thickness_ratio;

        let esperado_asa = wing_mass_raymer_kg(
            wing.area_m2, w_fw, wing.aspect_ratio, q_pa, wing.taper_ratio,
            t_c_asa, n_z_ult, mtow,
        ) * cfg.mass_model.composite_factor_wing;
        assert!((m.asa_kg - esperado_asa).abs() < 1e-9,
            "asa_kg = {:.4} (esperado {esperado_asa:.4})", m.asa_kg);

        let esperado_emp_h = htail_mass_raymer_kg(
            n_z_ult, mtow, q_pa, emp.s_horizontal_m2, t_c_emp, emp.ar_h, emp.taper_h,
        ) * cfg.mass_model.composite_factor_tail;
        assert!((m.emp_h_kg - esperado_emp_h).abs() < 1e-9,
            "emp_h_kg = {:.4} (esperado {esperado_emp_h:.4})", m.emp_h_kg);

        let esperado_emp_v = vtail_mass_raymer_kg(
            n_z_ult, mtow, q_pa, emp.s_vertical_m2, t_c_emp, emp.ar_v, emp.taper_v,
        ) * cfg.mass_model.composite_factor_tail;
        assert!((m.emp_v_kg - esperado_emp_v).abs() < 1e-9,
            "emp_v_kg = {:.4} (esperado {esperado_emp_v:.4})", m.emp_v_kg);

        let mm = &cfg.mass_model;
        let s_f_m2 = mm.fuselage_wetted_coeff * std::f64::consts::PI
            * mm.d_fus_equiv_m * cfg.fuselage.length_m;
        let l_over_d = cfg.fuselage.length_m / mm.d_fus_equiv_m;
        let esperado_fuselagem = fuselage_mass_raymer_kg(
            s_f_m2, n_z_ult, mtow, emp.arm_h_m, l_over_d, q_pa,
        ) * cfg.mass_model.composite_factor_fuselage;
        assert!((m.fuselagem_kg - esperado_fuselagem).abs() < 1e-9,
            "fuselagem_kg = {:.4} (esperado {esperado_fuselagem:.4})", m.fuselagem_kg);

        let esperado_trem_principal = main_gear_mass_raymer_kg(
            mm.landing_load_factor_ult, mtow, mm.main_strut_length_m,
        ) * mm.composite_factor_gear;
        assert!((m.trem_principal_kg - esperado_trem_principal).abs() < 1e-9,
            "trem_principal_kg = {:.4} (esperado {esperado_trem_principal:.4})",
            m.trem_principal_kg);

        let esperado_trem_nariz = nose_gear_mass_raymer_kg(
            mm.landing_load_factor_ult, mtow, mm.nose_strut_length_m,
        ) * mm.composite_factor_gear;
        assert!((m.trem_nariz_kg - esperado_trem_nariz).abs() < 1e-9,
            "trem_nariz_kg = {:.4} (esperado {esperado_trem_nariz:.4})", m.trem_nariz_kg);

        let esperado_tanques = fuel_system_mass_raymer_kg(cfg.fuel_system.capacity_l)
            * mm.composite_factor_fuel_system;
        assert!((m.tanques_kg - esperado_tanques).abs() < 1e-9,
            "tanques_kg = {:.4} (esperado {esperado_tanques:.4})", m.tanques_kg);

        assert!(m.asa_kg > 0.0 && m.tanques_kg > 0.0);
    }

    // Empenagem responde a v_h NOS DOIS SENTIDOS (spec, Testes item 2) —
    // substitui a property de mass_per_area do ciclo 2.
    #[test]
    fn massa_da_empenagem_horizontal_responde_a_v_h_nos_dois_sentidos() {
        let cfg_base = config_teste();
        let engine = motor_generico_teste();
        let req = requisitos_teste();
        let state = AircraftState::from_config(&cfg_base);
        let wing = AerodynamicsAgent::run(&state, &req);
        let mtow = 1_400.0;
        let n_design = 4.0;

        let emp_base = EmpennageAgent::run(&wing, &cfg_base);
        let m_base = MassModelAgent::run(&cfg_base, &engine, &req, &wing, &emp_base, mtow, n_design);

        let mut cfg_maior = cfg_base.clone();
        cfg_maior.empennage.v_h *= 1.2;
        let emp_maior = EmpennageAgent::run(&wing, &cfg_maior);
        let m_maior = MassModelAgent::run(&cfg_maior, &engine, &req, &wing, &emp_maior, mtow, n_design);

        let mut cfg_menor = cfg_base.clone();
        cfg_menor.empennage.v_h *= 0.8;
        let emp_menor = EmpennageAgent::run(&wing, &cfg_menor);
        let m_menor = MassModelAgent::run(&cfg_menor, &engine, &req, &wing, &emp_menor, mtow, n_design);

        assert!(m_maior.emp_h_kg > m_base.emp_h_kg,
            "m_maior={:.4} deveria ser > m_base={:.4}", m_maior.emp_h_kg, m_base.emp_h_kg);
        assert!(m_base.emp_h_kg > m_menor.emp_h_kg,
            "m_base={:.4} deveria ser > m_menor={:.4}", m_base.emp_h_kg, m_menor.emp_h_kg);
    }
}
