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
/// Material das longarinas: selecionável por nome via `[structure]
/// spar_material` do TOML de aeronave (ver `material_by_name`).
/// Pele: laminado de fibra de vidro E-glass / epóxi com reforços de carbono
///
/// Referências:
///   - CS-23 Amendment 5 / RBAC-23
///   - Raymer, D. "Aircraft Design: A Conceptual Approach", Cap. 14
///   - Niu, M. "Airframe Structural Design", Cap. 4

use crate::models::aircraft_config::StructureCfg;
use crate::models::requirements::Requirements;
use crate::models::specs::{WingSpec, StructuralSpec};

const G: f64 = 9.807; // m/s²

// ─── PROPRIEDADES DOS MATERIAIS ───────────────────────────────────────────────

/// Propriedades físicas de um material de longarina.
#[derive(Debug, Clone, Copy)]
pub struct Material {
    pub sigma_yield_mpa: f64, // MPa — limite de escoamento
    pub sigma_ult_mpa: f64,   // MPa — resistência última
    pub density_kgm3: f64,    // kg/m³
    pub e_gpa: f64,           // GPa — módulo de Young
}

impl Material {
    /// Tensão admissível de projeto (última / 1.5 — margem de segurança
    /// clássica para material dúctil sem fator de segurança adicional na
    /// carga, já que esta já vem em carga ÚLTIMA = 1.5 × limite).
    pub fn sigma_allow_mpa(&self) -> f64 {
        self.sigma_ult_mpa / 1.5
    }
}

/// Resolve um material de longarina cadastrado pelo nome usado em
/// `[structure] spar_material` do TOML de aeronave. `None` = material
/// desconhecido (rejeitado na validação de `models::config::load_aircraft`).
pub fn material_by_name(name: &str) -> Option<Material> {
    match name {
        // Alumínio 7075-T6 — alta resistência, longarinas principais.
        "AA7075-T6" => Some(Material {
            sigma_yield_mpa: 503.0,
            sigma_ult_mpa: 572.0,
            density_kgm3: 2_810.0,
            e_gpa: 71.7,
        }),
        // Alumínio 6061-T6 — mais soldável, resistência moderada.
        "AA6061-T6" => Some(Material {
            sigma_yield_mpa: 276.0,
            sigma_ult_mpa: 310.0,
            density_kgm3: 2_700.0,
            e_gpa: 68.9,
        }),
        _ => None,
    }
}

/// Laminado de fibra de vidro E-glass / epóxi (pele e nervuras) — mesmo
/// material de pele para toda a família de aeronaves modeladas por este
/// projeto (não parametrizado pelo TOML — só a longarina é selecionável).
pub struct FiberglassEpoxy;
impl FiberglassEpoxy {
    pub const SIGMA_ULT_MPA: f64    = 300.0; // MPa em tração (laminado 0°/90°)
    pub const DENSITY_KGM3: f64     = 1_850.0;
    pub const E_GPA: f64            = 20.0;   // GPa
}

// ─── DIAGRAMA V-n (CS-23 Normal) ──────────────────────────────────────────────

/// Fator de carga limite por categoria de projeto CS-23:
/// Normal = 3.8g | Utility = 4.4g | Acrobático = 6.0g.
/// `design_category` já validado em `models::config::load_aircraft`
/// (apenas "normal" | "utility" | "acrobatic" passam) — desconhecido cai no
/// padrão Normal (3.8g), nunca deveria ser alcançado em produção.
pub fn load_factor_limit(design_category: &str) -> f64 {
    match design_category {
        "utility" => 4.4,
        "acrobatic" => 6.0,
        _ => 3.8,
    }
}

/// Fator de carga último = 1.5 × limite (CS 23.303)
pub fn load_factor_ultimate(design_category: &str) -> f64 {
    load_factor_limit(design_category) * 1.5
}

/// Velocidade de projeto de cruzeiro VC (m/s):
/// VC = V_cruise_cruise (velocidade de cruzeiro especificada)
pub fn vc_ms(v_cruise_kmh: f64) -> f64 { v_cruise_kmh / 3.6 }

/// Velocidade de mergulho de projeto VD (m/s) — CS 23.335:
/// VD ≥ 1.25 × VC
pub fn vd_ms(vc_ms: f64) -> f64 { 1.25 * vc_ms }

/// Velocidade de manobra VA (m/s) — CS 23.335:
/// VA = VS1 × √n_lim   (não exceder abaixo desta velocidade)
/// IMPORTANTE: usa VS1 (stall em configuração LIMPA, wing.stall_speed_clean_kmh),
/// não VS0 (stall com flap) — CS 23.335 define VA a partir do stall limpo.
/// `n_lim` vem de `load_factor_limit(design_category)`.
pub fn va_ms(v_stall_clean_ms: f64, n_lim: f64) -> f64 {
    v_stall_clean_ms * n_lim.sqrt()
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
/// h_spar ≈ 0.60 × t_max = 0.60 × (thickness_ratio × c_r)
/// `thickness_ratio` (t/c) vem de `[wing] thickness_ratio` do TOML de
/// aeronave (perfil específico, ex.: 0.15 para NACA 23015).
pub fn spar_height_root(chord_root_m: f64, thickness_ratio: f64) -> f64 {
    0.60 * thickness_ratio * chord_root_m // 60% da espessura máxima do perfil
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
/// Retorna V_flutter estimado em m/s — estimativa física sem piso artificial.
pub fn flutter_speed_ms(vd_ms: f64, wing_area_m2: f64, span_m: f64,
                         chord_root_m: f64, spar_height_m: f64,
                         wing_mass_kg: f64) -> f64 {
    let _ = vd_ms; // não participa da estimativa — apenas do critério
    // GJ da longarina (rigidez torsional estimada)
    // GJ = G × J onde G = E/(2(1+ν)) = 71.7e9/2.6 ≈ 27.6 GPa para Al 7075-T6
    let g_al = 27.6e9_f64; // Pa
    // J ≈ (b_f × t_f³)/3 + (h × t_w³)/3 para I-beam (simplificado)
    // Usando seção equivalente: J ≈ 0.02 × (h_spar)⁴
    let j_eff = 0.02 * spar_height_m.powi(4); // m⁴ (estimativa conservadora)
    let gj = g_al * j_eff;

    // Momento de inércia de massa em torção por unidade de envergadura (I_α)
    // I_α ≈ m_asa × (chord/4)²  por unidade de comprimento
    let m_per_m = wing_mass_kg / span_m; // kg/m (massa da asa distribuída)
    let r_alpha = chord_root_m / 4.0; // raio de giração
    let i_alpha_per_m = m_per_m * r_alpha * r_alpha; // kg·m²/m

    // Velocidade de flutter (método de energia)
    0.60 * (gj / (i_alpha_per_m * wing_area_m2 / span_m)).sqrt()
}

/// Verificação: V_flutter ≥ 1.20 × VD (CS 23.629)
pub fn flutter_check(v_flutter_ms: f64, vd_ms: f64) -> bool {
    v_flutter_ms >= 1.20 * vd_ms
}

// ─── FADIGA ───────────────────────────────────────────────────────────────────

/// Vida em fadiga estimada pela relação de Goodman modificada.
/// Para ligas de alumínio (7075-T6 e 6061-T6, os materiais cadastrados em
/// `material_by_name`):
///   Se = 160 MPa (limite de fadiga — R = 0, base 10⁷ ciclos; aproximação
///        genérica para liga de Al — não recalibrada por liga específica)
///   σ_max = σ_média + σ_amplitude
///
/// Número de voos estimado (simplificado):
///   N = (Se / σ_max)^b × N_base   onde b ≈ 5.8 para ligas de alumínio
pub fn fatigue_life_cycles(
    sigma_max_mpa: f64,  // tensão máxima em voo (limite, sem fator último)
    sigma_min_mpa: f64,  // tensão mínima (carga de 1g)
    sigma_ult_mpa: f64,  // resistência última do material da longarina (MPa)
) -> f64 {
    const SE_MPA: f64 = 160.0;   // limite de fadiga (liga de Al genérica)
    const B: f64 = 5.8;          // expoente de Basquin para Al
    const N_BASE: f64 = 1e7;     // base de referência (10⁷ ciclos)

    let sigma_a = (sigma_max_mpa - sigma_min_mpa) / 2.0;
    let sigma_m = (sigma_max_mpa + sigma_min_mpa) / 2.0;

    // Goodman: σ_a_equiv = σ_a / (1 - σ_m / σ_ult)
    let sigma_equiv = sigma_a / (1.0 - sigma_m / sigma_ult_mpa).max(0.01);

    if sigma_equiv <= SE_MPA {
        return f64::INFINITY; // abaixo do limite de fadiga → vida infinita
    }
    N_BASE * (SE_MPA / sigma_equiv).powf(B) // < N_BASE quando σ > Se
}

// ─── AGENTE PRINCIPAL ────────────────────────────────────────────────────────

pub struct StructuralAgent;

impl StructuralAgent {
    pub fn run(
        wing: &WingSpec,
        mtow_kg: f64,
        wing_mass_kg: f64,  // massa da asa estrutural (item "asa" de [masses])
        req: &Requirements,
        structure_cfg: &StructureCfg,
    ) -> StructuralSpec {
        let material = material_by_name(&structure_cfg.spar_material).unwrap_or_else(|| {
            panic!(
                "material de longarina desconhecido '{}' — deveria ter sido rejeitado \
                 por models::config::load_aircraft",
                structure_cfg.spar_material
            )
        });

        let n_lim = load_factor_limit(&structure_cfg.design_category);
        let n_ult = load_factor_ultimate(&structure_cfg.design_category);

        // Momento fletor na raiz
        let m_limit = wing_root_bending_nm(n_lim, mtow_kg, wing.span_m,
                                            wing.taper_ratio, wing_mass_kg);
        let m_ult   = m_limit * 1.5;

        // Longarina raiz
        let chord_root_m = 2.0 * wing.area_m2 / (wing.span_m * (1.0 + wing.taper_ratio));
        let h_spar  = spar_height_root(chord_root_m, wing.thickness_ratio);
        let sigma_allow = material.sigma_allow_mpa();
        let w_req   = required_section_modulus_cm3(m_ult, sigma_allow);
        let a_flange = spar_flange_area_cm2(w_req, h_spar * 100.0); // h em cm
        let t_web   = spar_web_thickness_mm(n_lim, mtow_kg, h_spar, material.sigma_yield_mpa);

        // Pele
        let t_skin = skin_thickness_mm(m_limit, chord_root_m, h_spar);

        // Flutter e velocidades de projeto — VC vem do requisito de cruzeiro
        // do projeto, não de uma constante interna.
        let vc   = vc_ms(req.cruise_speed_min_kmh);
        let vd   = vd_ms(vc);
        let vf   = flutter_speed_ms(vd, wing.area_m2, wing.span_m, chord_root_m, h_spar, wing_mass_kg);
        let fl_ok = flutter_check(vf, vd);

        // Velocidade de manobra VA — CS 23.335, a partir de VS1 (stall limpa)
        let vs1_ms = wing.stall_speed_clean_kmh / 3.6;
        let va = va_ms(vs1_ms, n_lim);

        // Tensão operacional na longarina (1g nivelado — base para fadiga)
        // M / W = Pa; dividir por 1e6 para converter a MPa (unidade de fatigue_life_cycles)
        let m_1g = wing_root_bending_nm(1.0, mtow_kg, wing.span_m,
                                         wing.taper_ratio, wing_mass_kg);
        let w_req_m3 = w_req * 1e-6; // cm³ → m³
        let sigma_1g_mpa   = (m_1g    / w_req_m3 / 1e6).min(material.sigma_yield_mpa - 50.0);
        let sigma_max_mpa  = (m_limit / w_req_m3 / 1e6).min(sigma_allow);
        let cycles = fatigue_life_cycles(sigma_max_mpa, sigma_1g_mpa, material.sigma_ult_mpa);

        StructuralSpec {
            design_load_factor_g:        n_lim,
            ultimate_load_factor_g:      n_ult,
            wing_root_bending_limit_nm:  m_limit,
            wing_root_bending_ult_nm:    m_ult,
            spar_material:               structure_cfg.spar_material.clone(),
            spar_height_root_m:          h_spar,
            spar_flange_area_cm2:        a_flange,
            spar_web_thickness_mm:       t_web,
            skin_min_thickness_mm:       t_skin,
            frame_spacing_mm:            structure_cfg.frame_spacing_mm,
            flutter_speed_kmh:           vf * 3.6,
            design_dive_speed_kmh:       vd * 3.6,
            va_kmh:                      va * 3.6,
            fatigue_life_cycles:         cycles,
            flutter_ok:                  fl_ok,
        }
    }
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::aircraft_config::test_fixtures::config_teste;
    use crate::models::aircraft_state::AircraftState;
    use crate::agents::aerodynamics::AerodynamicsAgent;

    // MTOW/massa de asa sintéticos usados nos testes que não passam pelo
    // WeightBalanceAgent completo — arbitrários mas plausíveis, iguais aos
    // valores históricos deste arquivo (não coincidem com o baseline real).
    const MTOW_TESTE: f64 = 1_400.0;
    const WING_MASS_TESTE: f64 = 120.0;

    fn wing() -> WingSpec {
        let cfg = config_teste();
        let s = AircraftState::from_config(&cfg);
        AerodynamicsAgent::run(&s, &crate::models::requirements::test_fixtures::requisitos_teste())
    }

    fn structure_cfg_teste() -> StructureCfg {
        config_teste().structure
    }

    #[test]
    fn fator_de_carga_cs23() {
        assert!((load_factor_limit("normal") - 3.8).abs() < 0.01);
        assert!((load_factor_ultimate("normal") - 5.7).abs() < 0.01);
        assert!((load_factor_limit("utility") - 4.4).abs() < 0.01);
        assert!((load_factor_limit("acrobatic") - 6.0).abs() < 0.01);
    }

    #[test]
    fn material_by_name_resolve_materiais_cadastrados() {
        let a7075 = material_by_name("AA7075-T6").expect("AA7075-T6 deveria existir");
        let a6061 = material_by_name("AA6061-T6").expect("AA6061-T6 deveria existir");
        assert!(a7075.sigma_ult_mpa > a6061.sigma_ult_mpa,
            "AA7075-T6 deveria ser mais resistente que AA6061-T6");
        assert!(material_by_name("Unobtainium").is_none());
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
        // M_root a 3.8g deve ser proporcional a MTOW/span/taper — faixa
        // ampla o bastante para a fixture sintética (span/taper menores que
        // o baseline real).
        let m = wing_root_bending_nm(3.8, MTOW_TESTE, w.span_m, w.taper_ratio, WING_MASS_TESTE);
        println!("M_root @ 3.8g = {:.0} N·m", m);
        assert!(m > 30_000.0 && m < 100_000.0,
            "M_root {m:.0} N·m fora do intervalo (30.000–100.000 N·m)");
    }

    #[test]
    fn longarina_dimensionada_com_material_adequado() {
        let w = wing();
        let req = crate::models::requirements::test_fixtures::requisitos_teste();
        let struc = StructuralAgent::run(&w, MTOW_TESTE, WING_MASS_TESTE, &req, &structure_cfg_teste());
        println!("Longarina raiz ({}): h={:.0}mm, A_flange={:.1}cm², t_alma={:.1}mm",
                 struc.spar_material,
                 struc.spar_height_root_m * 1000.0,
                 struc.spar_flange_area_cm2,
                 struc.spar_web_thickness_mm);
        assert_eq!(struc.spar_material, "AA6061-T6");
        // Piso original 0.10 m — medido empiricamente para esta fixture:
        // ~0.137 m, folgadamente acima. A Task 2.1 tinha baixado este piso
        // para 0.08 m sem necessidade; corrigido de volta após code review
        // (ver task-2.1-report.md, correção pós-review).
        assert!(struc.spar_height_root_m > 0.10 && struc.spar_height_root_m < 0.30,
            "Altura da longarina {:.0}mm fora de 100–300mm", struc.spar_height_root_m * 1000.0);
        assert!(struc.spar_flange_area_cm2 > 0.5,
            "Área de mesa {:.1}cm² muito pequena", struc.spar_flange_area_cm2);
    }

    #[test]
    fn flutter_acima_de_1_2_vd() {
        let w = wing();
        let req = crate::models::requirements::test_fixtures::requisitos_teste();
        let struc = StructuralAgent::run(&w, MTOW_TESTE, WING_MASS_TESTE, &req, &structure_cfg_teste());
        let vc = vc_ms(req.cruise_speed_min_kmh);
        let vd = vd_ms(vc);
        println!("VD={:.0} km/h  V_flutter={:.0} km/h  OK={}",
                 vd * 3.6, struc.flutter_speed_kmh, struc.flutter_ok);
        assert!(struc.flutter_ok,
            "Flutter {:.0} km/h abaixo do limite 1.20×VD={:.0} km/h",
            struc.flutter_speed_kmh, vd * 3.6 * 1.20);
    }

    #[test]
    fn flutter_reprova_com_longarina_fraca() {
        // Longarina de 20 mm de altura em asa de 12 m: rigidez torsional GJ ~ h⁴
        // despenca e V_flutter deve cair abaixo de 1.2×VD.
        let vd = vd_ms(vc_ms(280.0));
        let vf = flutter_speed_ms(vd, 14.2, 11.94, 1.64, 0.020, 130.0);
        assert!(!flutter_check(vf, vd),
            "Flutter check passou com longarina de 20 mm — verificação vácua");
    }

    #[test]
    fn fadiga_alta_tensao_vida_curta() {
        let sigma_ult = material_by_name("AA7075-T6").unwrap().sigma_ult_mpa;
        // σ_a equivalente acima do limite de fadiga → vida FINITA e menor que 10⁷
        let vida_alta_tensao = fatigue_life_cycles(300.0, 50.0, sigma_ult);
        // σ_a equivalente abaixo do limite → vida "infinita" (≥ 10⁹ por convenção)
        let vida_baixa_tensao = fatigue_life_cycles(80.0, 40.0, sigma_ult);
        assert!(vida_alta_tensao < 1e7, "alta tensão deveria dar vida finita < 10⁷");
        assert!(vida_baixa_tensao >= 1e9, "baixa tensão deveria dar vida quase infinita");
    }

    #[test]
    fn va_usa_vs1_limpa_nao_vs0_flapada() {
        // Task 0.5: VA deve ser derivada de VS1 (limpa), que é MAIOR que VS0
        // (flapada) — logo VA calculada corretamente deve ser MAIOR do que
        // seria se (incorretamente) derivada de VS0.
        let w = wing();
        let req = crate::models::requirements::test_fixtures::requisitos_teste();
        let struc = StructuralAgent::run(&w, MTOW_TESTE, WING_MASS_TESTE, &req, &structure_cfg_teste());

        let va_esperada_kmh = w.stall_speed_clean_kmh * load_factor_limit("normal").sqrt();
        let va_se_fosse_vs0_kmh = w.stall_speed_flaps_kmh * load_factor_limit("normal").sqrt();

        println!("VA (VS1 correta) = {:.1} km/h | VA (se fosse VS0, incorreta) = {:.1} km/h",
                 struc.va_kmh, va_se_fosse_vs0_kmh);

        assert!((struc.va_kmh - va_esperada_kmh).abs() < 0.1,
            "VA {:.1} km/h não corresponde a VS1×√3.8 = {:.1} km/h",
            struc.va_kmh, va_esperada_kmh);
        assert!(struc.va_kmh > va_se_fosse_vs0_kmh,
            "VA {:.1} km/h deveria ser maior que a VA calculada (incorretamente) com VS0 {:.1} km/h",
            struc.va_kmh, va_se_fosse_vs0_kmh);
    }

    #[test]
    fn fadiga_acima_de_10000_voos() {
        let w = wing();
        let req = crate::models::requirements::test_fixtures::requisitos_teste();
        let struc = StructuralAgent::run(&w, MTOW_TESTE, WING_MASS_TESTE, &req, &structure_cfg_teste());
        let ciclos = struc.fatigue_life_cycles;
        println!("Vida em fadiga: {ciclos:.2e} ciclos");
        // Aeronave com ciclos de pressurização leve deve durar > 10.000 voos
        assert!(ciclos > 10_000.0 || ciclos == f64::INFINITY,
            "Vida {ciclos:.2e} ciclos abaixo do mínimo de 10.000");
    }
}
