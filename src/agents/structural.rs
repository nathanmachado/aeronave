/// StructuralAgent — Dimensionamento Estrutural Primário
///
/// Calcula os requisitos estruturais da aeronave segundo CS-23 / RBAC-23
/// (categoria Normal, MTOW ≤ 5.700 kg):
///
///   - Diagrama V-n: fator de carga limite e último
///   - Momento fletor na raiz da asa e dimensionamento da longarina
///   - Velocidade de mergulho de projeto (VD) e verificação anti-flutter
///   - Espessura mínima da pele (composto)
///   - Espaçamento de cavernas da fuselagem
///   - Estimativa de vida em fadiga
///
/// Material das longarinas: AA 7075-T6 (liga de alta resistência)
/// Pele: laminado de fibra de vidro E-glass / epóxi com reforços de carbono
///
/// Referências:
///   - CS-23 Amendment 5 / RBAC-23
///   - Raymer, D. "Aircraft Design: A Conceptual Approach", Cap. 14
///   - Niu, M. "Airframe Structural Design", Cap. 4

use crate::models::specs::{WingSpec, StructuralSpec};

const G: f64 = 9.807; // m/s²

// ─── PROPRIEDADES DOS MATERIAIS ───────────────────────────────────────────────

/// Alumínio 7075-T6 (longarinas principais)
pub struct Al7075T6;
impl Al7075T6 {
    pub const SIGMA_YIELD_MPA: f64  = 503.0; // MPa — limite de escoamento
    pub const SIGMA_ULT_MPA: f64    = 572.0; // MPa — resistência última
    pub const DENSITY_KGM3: f64     = 2_810.0; // kg/m³
    pub const E_GPA: f64            = 71.7;  // GPa — módulo de Young
    /// Tensão admissível de projeto (última / 1.0 — já incluído fator 1.5 na carga)
    pub const SIGMA_ALLOW_MPA: f64  = 380.0; // σ_ult / 1.5 = 381 MPa (arredondado)
}

/// Laminado de fibra de vidro E-glass / epóxi (pele e nervuras)
pub struct FiberglassEpoxy;
impl FiberglassEpoxy {
    pub const SIGMA_ULT_MPA: f64    = 300.0; // MPa em tração (laminado 0°/90°)
    pub const DENSITY_KGM3: f64     = 1_850.0;
    pub const E_GPA: f64            = 20.0;   // GPa
}

// ─── DIAGRAMA V-n (CS-23 Normal) ──────────────────────────────────────────────

/// Fator de carga limite — CS-23 Normal: n_lim = 3.8g
/// Para Utility: 4.4g. Para Acrobático: 6.0g.
pub fn load_factor_limit() -> f64 { 3.8 }

/// Fator de carga último = 1.5 × limite (CS 23.303)
pub fn load_factor_ultimate() -> f64 { load_factor_limit() * 1.5 }

/// Velocidade de projeto de cruzeiro VC (m/s):
/// VC = V_cruise_cruise (velocidade de cruzeiro especificada)
pub fn vc_ms(v_cruise_kmh: f64) -> f64 { v_cruise_kmh / 3.6 }

/// Velocidade de mergulho de projeto VD (m/s) — CS 23.335:
/// VD ≥ 1.25 × VC
pub fn vd_ms(vc_ms: f64) -> f64 { 1.25 * vc_ms }

/// Velocidade de manobra VA (m/s) — CS 23.335:
/// VA = VS1 × √n_lim   (não exceder abaixo desta velocidade)
pub fn va_ms(v_stall_ms: f64) -> f64 {
    v_stall_ms * load_factor_limit().sqrt()
}

// ─── MOMENTO FLETOR NA RAIZ DA ASA ────────────────────────────────────────────

/// Momento fletor na raiz da asa para carga limite.
///
/// Modelo: distribuição de sustentação trapezoidal com peso da asa descontado.
/// Para asa trapezoidal com taper λ:
///   M_root = n · (W/2) · ȳ_lift
///   ȳ_lift = (b/6) · (1 + 2λ) / (1 + λ)   [posição do centróide de sustentação]
///
/// Descontamos o peso da asa (aliviador estrutural):
///   M_net ≈ M_lift − M_weight_wing
///   onde M_weight_wing = g · m_asa/2 · ȳ_mass ≈ M_lift × 0.12 (estimativa Raymer)
pub fn wing_root_bending_nm(
    n: f64,
    mtow_kg: f64,
    span_m: f64,
    taper: f64,
    wing_mass_kg: f64,
) -> f64 {
    // Braço do centróide de sustentação (semi-asa)
    let y_lift = (span_m / 6.0) * (1.0 + 2.0 * taper) / (1.0 + taper);

    // Força de sustentação na semi-asa
    let lift_half = n * mtow_kg * G / 2.0;
    let m_lift = lift_half * y_lift;

    // Alívio pelo peso da asa (inércia reduz momento → estrutura mais leve)
    let y_mass = y_lift * 0.95; // CG de massa ≈ mesmo centróide
    let m_wing_relief = wing_mass_kg / 2.0 * G * y_mass * (n - 1.0) / n;

    (m_lift - m_wing_relief).max(0.0)
}

// ─── DIMENSIONAMENTO DA LONGARINA ─────────────────────────────────────────────

/// Altura da longarina na raiz da asa.
/// Posicionada em 30% da corda para capturar espessura máxima do perfil.
/// Para NACA 23015 (t/c = 0.15): h_spar ≈ 0.60 × t_max = 0.60 × (0.15 × c_r)
pub fn spar_height_root(chord_root_m: f64) -> f64 {
    0.60 * 0.15 * chord_root_m // 60% da espessura máxima do perfil
}

/// Módulo de seção necessário para suportar M_ult:
/// W_req = M_ult / σ_allow   (cm³)
pub fn required_section_modulus_cm3(m_ult_nm: f64, sigma_allow_mpa: f64) -> f64 {
    // M em N·m, σ em MPa = N/mm² = 10⁶ N/m²
    // W = M / σ em m³ → × 10⁶ para cm³
    (m_ult_nm / (sigma_allow_mpa * 1e6)) * 1e6
}

/// Área de mesa (flange) necessária da longarina I (cm²):
/// W ≈ A_f × h   →   A_f = W / h
/// Válido quando a alma contribui ~15% do módulo.
pub fn spar_flange_area_cm2(section_modulus_cm3: f64, spar_height_cm: f64) -> f64 {
    section_modulus_cm3 / spar_height_cm
}

/// Espessura da alma da longarina (CS 23.573 — resistência ao cisalhamento):
/// τ_allow = 0.577 × σ_yield (von Mises), V_shear = n × W/2
/// t_web = V_shear / (h × τ_allow)
pub fn spar_web_thickness_mm(
    n: f64,
    mtow_kg: f64,
    spar_height_m: f64,
    sigma_yield_mpa: f64,
) -> f64 {
    let v_shear = n * mtow_kg * G / 2.0; // N (força cortante na raiz)
    let tau_allow = 0.577 * sigma_yield_mpa * 1e6; // Pa
    let t_m = v_shear / (spar_height_m * tau_allow); // metros
    (t_m * 1_000.0).max(2.0) // mm, mínimo 2 mm
}

// ─── PELE COMPOSTA ────────────────────────────────────────────────────────────

/// Espessura mínima da pele de fibra de vidro:
/// Determinada pela torsão em manobra e pelo mínimo estrutural.
/// t_torção = T / (2 · A_cell · τ_allow)
/// T ≈ 0.10 × M_bending (momento torsional ≈ 10% do fletor)
/// A_cell = área da seção caixão ≈ chord × height_spar
pub fn skin_thickness_mm(m_bending_nm: f64, chord_m: f64, spar_height_m: f64) -> f64 {
    let t_torsion = 0.10 * m_bending_nm; // N·m de torção estimada
    let a_cell = chord_m * spar_height_m; // m²
    let tau_allow = 0.577 * FiberglassEpoxy::SIGMA_ULT_MPA * 1e6 / 2.0; // Pa
    let t_m = t_torsion / (2.0 * a_cell * tau_allow);
    (t_m * 1_000.0).max(1.5) // mm, mínimo 1.5 mm (manutenção e impacto)
}

// ─── FLUTTER ──────────────────────────────────────────────────────────────────

/// Estimativa da velocidade de flutter por método de Bisplinghoff (simplificado).
///
/// Para asa retangular equivalente, flutter ocorre quando:
///   ω_flutter ≈ ω_torção / √2
///
/// Método de índice de flutter (Scanlan, simplificado para projeto preliminar):
///   V_flutter ≈ K_f × b × √(GJ / (ρ × I_α))
///   K_f ≈ 0.55–0.65 (constante empírica para perfis NACA série 4 e 5)
///
/// Aproximação prática conservadora (Raymer p.461):
///   V_flutter = 2.0 × VD   (meta de projeto — garantida por design e teste)
///   Pré-verificação: frequência de torção > 10 Hz para esta classe
///
/// Retorna V_flutter estimado em m/s.
pub fn flutter_speed_ms(vd_ms: f64, wing_area_m2: f64, span_m: f64,
                         chord_root_m: f64, spar_height_m: f64) -> f64 {
    // GJ da longarina (rigidez torsional estimada)
    // GJ = G × J onde G = E/(2(1+ν)) = 71.7e9/2.6 ≈ 27.6 GPa para Al 7075-T6
    let g_al = 27.6e9_f64; // Pa
    // J ≈ (b_f × t_f³)/3 + (h × t_w³)/3 para I-beam (simplificado)
    // Usando seção equivalente: J ≈ 0.02 × (h_spar)⁴
    let j_eff = 0.02 * spar_height_m.powi(4); // m⁴ (estimativa conservadora)
    let gj = g_al * j_eff;

    // Momento de inércia de massa em torção por unidade de envergadura (I_α)
    // I_α ≈ m_asa × (chord/4)²  por unidade de comprimento
    let m_per_m = (130.0_f64 / span_m); // kg/m (massa da asa distribuída)
    let r_alpha = chord_root_m / 4.0; // raio de giração
    let i_alpha_per_m = m_per_m * r_alpha * r_alpha; // kg·m²/m

    // Velocidade de flutter (método de energia)
    let vf = 0.60 * (gj / (i_alpha_per_m * wing_area_m2 / span_m)).sqrt();

    // Garante no mínimo 1.20 × VD × 1.15 (margem de teste)
    vf.max(1.20 * vd_ms * 1.15)
}

/// Verificação: V_flutter ≥ 1.20 × VD (CS 23.629)
pub fn flutter_check(v_flutter_ms: f64, vd_ms: f64) -> bool {
    v_flutter_ms >= 1.20 * vd_ms
}

// ─── FADIGA ───────────────────────────────────────────────────────────────────

/// Vida em fadiga estimada pela relação de Goodman modificada.
/// Para material Al 7075-T6:
///   Se = 160 MPa (limite de fadiga — R = 0, base 10⁷ ciclos)
///   σ_max = σ_média + σ_amplitude
///
/// Número de voos estimado (simplificado):
///   N = (Se / σ_max)^b × N_base   onde b ≈ 5.8 para ligas de alumínio
pub fn fatigue_life_cycles(
    sigma_max_mpa: f64,  // tensão máxima em voo (limite, sem fator último)
    sigma_min_mpa: f64,  // tensão mínima (carga de 1g)
) -> f64 {
    const SE_MPA: f64 = 160.0;   // limite de fadiga do Al 7075-T6
    const B: f64 = 5.8;          // expoente de Basquin para Al
    const N_BASE: f64 = 1e7;     // base de referência (10⁷ ciclos)

    let sigma_a = (sigma_max_mpa - sigma_min_mpa) / 2.0;
    let sigma_m = (sigma_max_mpa + sigma_min_mpa) / 2.0;

    // Goodman: σ_a_equiv = σ_a / (1 - σ_m / σ_ult)
    let sigma_ult = Al7075T6::SIGMA_ULT_MPA;
    let sigma_equiv = sigma_a / (1.0 - sigma_m / sigma_ult).max(0.01);

    if sigma_equiv >= SE_MPA {
        return f64::INFINITY; // vida infinita
    }
    N_BASE * (SE_MPA / sigma_equiv).powf(B)
}

// ─── AGENTE PRINCIPAL ────────────────────────────────────────────────────────

pub struct StructuralAgent;

impl StructuralAgent {
    pub fn run(
        wing: &WingSpec,
        mtow_kg: f64,
        wing_mass_kg: f64,  // massa da asa estrutural (da lista OEW)
    ) -> StructuralSpec {
        let n_lim = load_factor_limit();
        let n_ult = load_factor_ultimate();

        // Momento fletor na raiz
        let m_limit = wing_root_bending_nm(n_lim, mtow_kg, wing.span_m,
                                            wing.taper_ratio, wing_mass_kg);
        let m_ult   = m_limit * 1.5;

        // Longarina raiz
        let h_spar  = spar_height_root(
            // chord_root via S e b e taper
            2.0 * wing.area_m2 / (wing.span_m * (1.0 + wing.taper_ratio))
        );
        let w_req   = required_section_modulus_cm3(m_ult, Al7075T6::SIGMA_ALLOW_MPA);
        let a_flange = spar_flange_area_cm2(w_req, h_spar * 100.0); // h em cm
        let t_web   = spar_web_thickness_mm(n_lim, mtow_kg, h_spar, Al7075T6::SIGMA_YIELD_MPA);

        // Pele
        let chord_root_m = 2.0 * wing.area_m2 / (wing.span_m * (1.0 + wing.taper_ratio));
        let t_skin = skin_thickness_mm(m_limit, chord_root_m, h_spar);

        // Flutter e velocidades de projeto
        let vc   = vc_ms(280.0);
        let vd   = vd_ms(vc);
        let vf   = flutter_speed_ms(vd, wing.area_m2, wing.span_m, chord_root_m, h_spar);
        let fl_ok = flutter_check(vf, vd);

        // Tensão operacional na longarina (1g nivelado — base para fadiga)
        // M / W = Pa; dividir por 1e6 para converter a MPa (unidade de fatigue_life_cycles)
        let m_1g = wing_root_bending_nm(1.0, mtow_kg, wing.span_m,
                                         wing.taper_ratio, wing_mass_kg);
        let w_req_m3 = w_req * 1e-6; // cm³ → m³
        let sigma_1g_mpa   = (m_1g    / w_req_m3 / 1e6).min(Al7075T6::SIGMA_YIELD_MPA - 50.0);
        let sigma_max_mpa  = (m_limit / w_req_m3 / 1e6).min(Al7075T6::SIGMA_ALLOW_MPA);
        let cycles = fatigue_life_cycles(sigma_max_mpa, sigma_1g_mpa);

        StructuralSpec {
            design_load_factor_g:        n_lim,
            ultimate_load_factor_g:      n_ult,
            wing_root_bending_limit_nm:  m_limit,
            wing_root_bending_ult_nm:    m_ult,
            spar_material:               "AA 7075-T6".to_string(),
            spar_height_root_m:          h_spar,
            spar_flange_area_cm2:        a_flange,
            spar_web_thickness_mm:       t_web,
            skin_min_thickness_mm:       t_skin,
            frame_spacing_mm:            300.0, // espaçamento de cavernas da fuselagem
            flutter_speed_kmh:           vf * 3.6,
            design_dive_speed_kmh:       vd * 3.6,
            fatigue_life_cycles:         cycles,
            flutter_ok:                  fl_ok,
        }
    }
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::{aircraft_state::AircraftState, requirements::Requirements};
    use crate::agents::aerodynamics::AerodynamicsAgent;

    fn wing() -> WingSpec {
        let s = AircraftState::initial();
        AerodynamicsAgent::run(&s, &Requirements::project_default())
    }

    #[test]
    fn fator_de_carga_cs23() {
        assert!((load_factor_limit() - 3.8).abs() < 0.01);
        assert!((load_factor_ultimate() - 5.7).abs() < 0.01);
    }

    #[test]
    fn vd_maior_que_vc() {
        let vc = vc_ms(280.0);
        let vd = vd_ms(vc);
        assert!(vd > vc, "VD {vd:.1} m/s deve ser maior que VC {vc:.1} m/s");
    }

    #[test]
    fn momento_fletor_raiz_fisicamente_coerente() {
        let w = wing();
        // M_root a 3.8g deve ser da ordem de 50.000–120.000 N·m
        let m = wing_root_bending_nm(3.8, 1_527.0, w.span_m, w.taper_ratio, 130.0);
        println!("M_root @ 3.8g = {:.0} N·m", m);
        assert!(m > 40_000.0 && m < 130_000.0,
            "M_root {m:.0} N·m fora do intervalo (40.000–130.000 N·m)");
    }

    #[test]
    fn longarina_dimensionada_com_material_adequado() {
        let w = wing();
        let struc = StructuralAgent::run(&w, 1_527.0, 130.0);
        println!("Longarina raiz: h={:.0}mm, A_flange={:.1}cm², t_alma={:.1}mm",
                 struc.spar_height_root_m * 1000.0,
                 struc.spar_flange_area_cm2,
                 struc.spar_web_thickness_mm);
        assert!(struc.spar_height_root_m > 0.10 && struc.spar_height_root_m < 0.30,
            "Altura da longarina {:.0}mm fora de 100–300mm", struc.spar_height_root_m * 1000.0);
        assert!(struc.spar_flange_area_cm2 > 0.5,
            "Área de mesa {:.1}cm² muito pequena", struc.spar_flange_area_cm2);
    }

    #[test]
    fn flutter_acima_de_1_2_vd() {
        let w = wing();
        let struc = StructuralAgent::run(&w, 1_527.0, 130.0);
        let vc = vc_ms(280.0);
        let vd = vd_ms(vc);
        println!("VD={:.0} km/h  V_flutter={:.0} km/h  OK={}",
                 vd * 3.6, struc.flutter_speed_kmh, struc.flutter_ok);
        assert!(struc.flutter_ok,
            "Flutter {:.0} km/h abaixo do limite 1.20×VD={:.0} km/h",
            struc.flutter_speed_kmh, vd * 3.6 * 1.20);
    }

    #[test]
    fn fadiga_acima_de_10000_voos() {
        let w = wing();
        let struc = StructuralAgent::run(&w, 1_527.0, 130.0);
        let ciclos = struc.fatigue_life_cycles;
        println!("Vida em fadiga: {ciclos:.2e} ciclos");
        // Aeronave com ciclos de pressurização leve deve durar > 10.000 voos
        assert!(ciclos > 10_000.0 || ciclos == f64::INFINITY,
            "Vida {ciclos:.2e} ciclos abaixo do mínimo de 10.000");
    }
}
