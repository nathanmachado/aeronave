/// VnDiagramAgent — Diagrama V-n completo com rajadas (CS 23.333 / CS 23.341)
///
/// Task 4.3. Antes desta task, `structural.rs` dimensionava a longarina
/// usando SEMPRE o fator de carga de manobra (`load_factor_limit`,
/// 3.8/4.4/6.0g conforme categoria CS-23) — ignorando a condição de rajada
/// (CS 23.341), que a norma exige verificar separadamente e que pode, em
/// certas combinações de carga alar baixa (asa leve, tanque quase vazio),
/// governar o dimensionamento estrutural (produzir um fator de carga MAIOR
/// que o de manobra).
///
/// Este módulo:
///   1. Calcula as quatro velocidades características do diagrama V-n
///      (VA, VB, VC, VD) e os fatores de carga de manobra (positivo/negativo).
///   2. Calcula o fator de carga de rajada em VC e VD (CS 23.341) — na massa
///      de ENVELOPE (pior caso legal, tanque cheio + carga máxima) e também
///      na massa LEVE (cenário de carga mais leve dentre os da
///      `WeightBalanceAgent`), porque carga alar baixa (massa leve) AUMENTA
///      o fator de carga de rajada — ver `n_gust_vc_light` abaixo.
///   3. Consolida `n_design = max(n_lim_pos, n_gust_vc, n_gust_vc_light)` —
///      o fator de carga que efetivamente governa o dimensionamento
///      estrutural, consumido por `StructuralAgent::run`.
///   4. Produz um polígono de pontos [V_kmh, n] para plotagem/CAD.
///
/// Fórmulas de rajada (CS 23.341, método da rajada gradual — Pratt):
///
///   n = 1 ± (ρ₀·Ude·V·a·Kg) / (2·(W/S))
///
///   onde:
///     ρ₀   = 1.225 kg/m³ (densidade ao nível do mar, ISA)
///     Ude  = velocidade de rajada equivalente: 15.24 m/s (50 ft/s) em VC,
///            7.62 m/s (25 ft/s) em VD — abaixo de 6.096 m (20.000 ft)
///     V    = velocidade equivalente (EAS) em m/s — VC ou VD
///     a    = inclinação CLα da asa 3D, 1/rad (`weight_balance::lift_curve_slope`)
///     Kg   = fator de alívio de rajada = 0.88μ / (5.3 + μ)
///     W/S  = carga alar em N/m² = m·g / S
///     μ    = razão de massa alar = 2·(m/S) / (ρ₀·MAC·a)   [adimensional]
///            (m/S em kg/m² — NÃO confundir com W/S em N/m²: μ usa massa,
///            não peso, por isso não há `g` no denominador de μ; a fórmula
///            completa com peso seria μ = 2·(W/S)/(ρ₀·g·MAC·a), que é
///            algebricamente idêntica a 2·(m/S)/(ρ₀·MAC·a) já que W = m·g)
///
/// Referências:
///   - CS-23 Amendment 5, §23.333 (Flight envelope), §23.335 (Design
///     airspeeds), §23.337 (Limit manoeuvring load factors), §23.341
///     (Gust load factors)
///   - Raymer, D. "Aircraft Design: A Conceptual Approach", Cap. 16
///   - Roskam, J. "Airplane Design Part V", Cap. 2
use crate::agents::structural::{load_factor_limit, load_factor_limit_neg, va_ms, vc_ms, vd_ms};
use crate::agents::weight_balance::{chord_root, lift_curve_slope, mean_aerodynamic_chord};
use crate::models::requirements::Requirements;
use crate::models::specs::{VnDiagramSpec, WingSpec};

const RHO0_KG_M3: f64 = 1.225; // densidade ISA ao nível do mar
const G: f64 = 9.807; // m/s²

/// Velocidade de rajada equivalente em VC — 50 ft/s (CS 23.341, abaixo de
/// 6.096 m / 20.000 ft).
const UDE_VC_MS: f64 = 15.24;
/// Velocidade de rajada equivalente em VD — 25 ft/s.
const UDE_VD_MS: f64 = 7.62;

// ─── FORMULAS DE RAJADA (CS 23.341) ───────────────────────────────────────────

/// Razão de massa alar μ (CS 23.341):
///   μ = 2·(m/S) / (ρ₀·MAC·a)
/// `mass_kg` é a massa da aeronave no cenário considerado (envelope ou
/// leve), `area_m2` é a área alar, `mac_m` a corda aerodinâmica média,
/// `a_per_rad` a inclinação CLα da asa (1/rad).
pub fn mass_ratio_mu(mass_kg: f64, area_m2: f64, mac_m: f64, a_per_rad: f64) -> f64 {
    let wing_loading_kg_m2 = mass_kg / area_m2;
    2.0 * wing_loading_kg_m2 / (RHO0_KG_M3 * mac_m * a_per_rad)
}

/// Fator de alívio de rajada Kg (CS 23.341):
///   Kg = 0.88μ / (5.3 + μ)
pub fn gust_alleviation_factor(mu: f64) -> f64 {
    0.88 * mu / (5.3 + mu)
}

/// Fator de carga de rajada positivo (CS 23.341):
///   n = 1 + (ρ₀·Ude·V·a·Kg) / (2·(W/S))
/// `v_ms` é a velocidade EAS (VC ou VD) em m/s, `mass_kg`/`area_m2` definem
/// W/S = mass_kg·g/area_m2 (N/m²).
pub fn gust_load_factor(
    mass_kg: f64,
    area_m2: f64,
    v_ms: f64,
    a_per_rad: f64,
    kg: f64,
    ude_ms: f64,
) -> f64 {
    let ws_n_m2 = mass_kg * G / area_m2;
    1.0 + (RHO0_KG_M3 * ude_ms * v_ms * a_per_rad * kg) / (2.0 * ws_n_m2)
}

/// Calcula n_gust numa dada velocidade/massa, resolvendo μ e Kg
/// internamente — atalho usado pelo agente principal para não repetir os
/// três passos (μ → Kg → n) em cada chamada.
fn gust_n_at(mass_kg: f64, area_m2: f64, mac_m: f64, a_per_rad: f64, v_ms: f64, ude_ms: f64) -> f64 {
    let mu = mass_ratio_mu(mass_kg, area_m2, mac_m, a_per_rad);
    let kg = gust_alleviation_factor(mu);
    gust_load_factor(mass_kg, area_m2, v_ms, a_per_rad, kg, ude_ms)
}

// ─── AGENTE PRINCIPAL ────────────────────────────────────────────────────────

pub struct VnDiagramAgent;

impl VnDiagramAgent {
    /// `mtow_envelope_kg`: massa do pior caso de carga LEGAL (cenário "4 pax
    /// + bagagem + tanque cheio" do `WeightBalanceAgent` — `wb.spec.mtow_kg`).
    /// `mass_light_kg`: massa do cenário de carga MAIS LEVE dentre os do
    /// `WeightBalanceAgent` (tipicamente "Solo (piloto)") — carga alar
    /// baixa nesse cenário pode fazer a rajada governar sobre a manobra
    /// (CS 23.341, condição crítica de asa leve).
    /// `category`: `cfg.structure.design_category` ("normal"|"utility"|"acrobatic").
    pub fn run(
        wing: &WingSpec,
        mtow_envelope_kg: f64,
        mass_light_kg: f64,
        req: &Requirements,
        category: &str,
    ) -> VnDiagramSpec {
        let n_lim_pos = load_factor_limit(category);
        let n_lim_neg = load_factor_limit_neg(category, n_lim_pos);

        // Geometria necessária para μ (MAC) e para `a` (CLα 3D)
        let c_r = chord_root(wing.area_m2, wing.span_m, wing.taper_ratio);
        let mac_m = mean_aerodynamic_chord(c_r, wing.taper_ratio);
        let a_per_rad = lift_curve_slope(wing.aspect_ratio);

        // Velocidades características — reaproveitando as funções ÚNICAS de
        // `structural.rs` (mesma fonte usada pelo StructuralAgent para VA/VC/VD).
        let vs1_ms = wing.stall_speed_clean_kmh / 3.6;
        let va_ms_v = va_ms(vs1_ms, n_lim_pos);
        let vc_ms_v = vc_ms(req.cruise_speed_min_kmh);
        let vd_ms_v = vd_ms(vc_ms_v);

        // Rajada em VC/VD na massa de ENVELOPE (pior caso legal de carga).
        let n_gust_vc = gust_n_at(mtow_envelope_kg, wing.area_m2, mac_m, a_per_rad, vc_ms_v, UDE_VC_MS);
        let n_gust_vd = gust_n_at(mtow_envelope_kg, wing.area_m2, mac_m, a_per_rad, vd_ms_v, UDE_VD_MS);

        // Rajada em VC na massa LEVE — carga alar baixa aumenta n_gust
        // (CS 23.341, condição crítica de asa leve/tanque vazio).
        let n_gust_vc_light = gust_n_at(mass_light_kg, wing.area_m2, mac_m, a_per_rad, vc_ms_v, UDE_VC_MS);

        // VB — velocidade de rajada máxima de projeto (simplificação de
        // projeto preliminar, documentada): VB = VS1 × √n_gust_vc, análoga
        // à definição de VA mas usando o fator de carga de rajada em VC em
        // vez do fator de manobra — não é a definição formal e completa de
        // CS 23.335(d) (que envolve W/S e Ude explicitamente), mas captura a
        // mesma ideia física (velocidade mínima na qual a rajada de projeto
        // atinge o fator de carga de rajada em VC) com os dados já disponíveis.
        let vb_ms_v = vs1_ms * n_gust_vc.max(1.0).sqrt();

        let n_design = n_lim_pos.max(n_gust_vc).max(n_gust_vc_light);

        let va_kmh = va_ms_v * 3.6;
        let vb_kmh = vb_ms_v * 3.6;
        let vc_kmh = vc_ms_v * 3.6;
        let vd_kmh = vd_ms_v * 3.6;
        let vs1_kmh = wing.stall_speed_clean_kmh;

        let points = envelope_polygon(vs1_kmh, va_kmh, vc_kmh, vd_kmh, n_lim_pos, n_lim_neg, n_design);

        VnDiagramSpec {
            va_kmh,
            vb_kmh,
            vc_kmh,
            vd_kmh,
            n_lim_pos,
            n_lim_neg,
            n_gust_vc,
            n_gust_vd,
            n_gust_vc_light,
            n_design,
            points,
        }
    }
}

// ─── POLÍGONO DO ENVELOPE ─────────────────────────────────────────────────────

/// Constrói o polígono [V_kmh, n] do envelope de projeto (para
/// plotagem/CAD). Convenção usada (documentada aqui por ser uma
/// simplificação deliberada, não a forma completa CS-23 com parábola de
/// estol negativa e linhas de rajada separadas cruzando o envelope de
/// manobra):
///
///   1. (0, 0) — origem
///   2. Parábola de estol positiva: n = (V/VS1)², amostrada em 4 pontos
///      intermediários entre a origem e o ponto onde a parábola atinge
///      n_lim_pos — isto é, (VA, n_lim_pos), SEMPRE incluído como vértice
///      (VA é, por definição, VS1·√n_lim_pos — o ponto onde a parábola de
///      estol de MANOBRA cruza n_lim_pos, CS 23.335).
///   3. SE a rajada governa (`n_design > n_lim_pos`): um segmento adicional
///      da parábola de estol até (V_design, n_design), onde
///      V_design = VS1·√n_design é a velocidade em que a parábola de estol
///      atinge o fator de carga de PROJETO (não apenas o de manobra) —
///      representa o ponto em que a linha de rajada cruza o envelope de
///      manobra e passa a governar. Sem isso o topo do polígono ficaria
///      preso a n_lim_pos, escondendo visualmente que a rajada governa.
///   4. Topo plano de (V_design, n_design) até (VD, n_design).
///   5. Aresta direita: (VD, n_design) → (VD, 0) (linha de VD).
///   6. Diagonal: (VD, 0) → (VC, n_lim_neg) (aproximação linear simples do
///      lado negativo — CS-23 não exige manobra negativa acima de VC).
///   7. Fecha o polígono: (VC, n_lim_neg) → (0, 0) — reta simples em vez de
///      uma parábola de estol negativa completa (simplificação deliberada,
///      documentada, suficiente para plotagem/CAD preliminar).
fn envelope_polygon(
    vs1_kmh: f64,
    va_kmh: f64,
    vc_kmh: f64,
    vd_kmh: f64,
    n_lim_pos: f64,
    n_lim_neg: f64,
    n_design: f64,
) -> Vec<[f64; 2]> {
    let mut pts: Vec<[f64; 2]> = Vec::new();
    pts.push([0.0, 0.0]);

    // Parábola de estol positiva: 0 → VA (n_lim_pos), 4 pontos intermediários.
    for i in 1..=4 {
        let v = va_kmh * (i as f64) / 4.0;
        let n = (v / vs1_kmh).powi(2);
        pts.push([v, n]);
    }
    // (VA, n_lim_pos) já é o último ponto do loop (i=4 → v=va_kmh,
    // n=(va_kmh/vs1_kmh)² = n_lim_pos, por definição de va_ms).

    // Se a rajada governa, estende a parábola de estol até (V_design, n_design).
    let v_design_kmh = vs1_kmh * n_design.sqrt();
    if n_design > n_lim_pos {
        let v_design_kmh = v_design_kmh.min(vd_kmh);
        pts.push([v_design_kmh, n_design]);
    }

    // Topo plano até VD.
    pts.push([vd_kmh, n_design]);
    // Aresta de VD até n=0.
    pts.push([vd_kmh, 0.0]);
    // Diagonal até (VC, n_lim_neg).
    pts.push([vc_kmh, n_lim_neg]);
    // Fecha o polígono na origem.
    pts.push([0.0, 0.0]);

    pts
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::agents::aerodynamics::AerodynamicsAgent;
    use crate::models::aircraft_config::test_fixtures::config_teste;
    use crate::models::aircraft_state::AircraftState;
    use crate::models::requirements::test_fixtures::requisitos_teste;

    fn wing() -> WingSpec {
        let cfg = config_teste();
        let s = AircraftState::from_config(&cfg);
        AerodynamicsAgent::run(&s, &requisitos_teste())
    }

    // ── Hand-checks de μ / Kg / n_gust (CS 23.341) ────────────────────────────
    //
    // Valores do hand-check do controller (task-4.3-brief), com os
    // parâmetros do baseline real (W/S=1065.9 N/m² @ MTOW envelope
    // 1543.4kg/14.2m², MAC=1.246m, AR=10.04 → a=5.155):
    //   μ = 2·(1543.4/14.2)/(1.225·1.246·5.155) = 2·108.69/7.868 ≈ 27.63
    //   Kg = 0.88·27.63/(5.3+27.63) ≈ 0.7385
    //   n_gust_vc = 1 + (1.225·15.24·77.78·5.155·0.7385)/(2·1065.9) ≈ 3.59

    #[test]
    fn mu_hand_check_baseline() {
        let mtow = 1543.4;
        let area = 14.2;
        let mac = 1.246;
        let a = lift_curve_slope(10.04);
        let mu = mass_ratio_mu(mtow, area, mac, a);
        println!("a={a:.4}  mu={mu:.3}");
        assert!((mu - 27.63).abs() < 0.5, "mu = {mu:.3} (esperado ~27.63)");
    }

    #[test]
    fn kg_hand_check_baseline() {
        let mu = 27.63;
        let kg = gust_alleviation_factor(mu);
        println!("Kg={kg:.4}");
        assert!((kg - 0.7385).abs() < 0.01, "Kg = {kg:.4} (esperado ~0.7385)");
    }

    #[test]
    fn n_gust_vc_hand_check_baseline() {
        let mtow = 1543.4;
        let area = 14.2;
        let mac = 1.246;
        let a = lift_curve_slope(10.04);
        let vc_ms_v = vc_ms(280.0);
        let mu = mass_ratio_mu(mtow, area, mac, a);
        let kg = gust_alleviation_factor(mu);
        let n = gust_load_factor(mtow, area, vc_ms_v, a, kg, UDE_VC_MS);
        println!("VC={:.2}m/s  n_gust_vc={n:.3}", vc_ms_v);
        assert!((n - 3.59).abs() < 0.05, "n_gust_vc = {n:.3} (esperado ~3.59)");
    }

    /// Relação usada por `gust_alleviation_factor` verificada algebricamente
    /// contra a fórmula da norma, para qualquer μ (não só o hand-check
    /// fixo acima).
    #[test]
    fn kg_bate_com_formula_cs23341_para_mu_arbitrario() {
        for mu in [1.0, 5.0, 10.0, 27.63, 50.0, 100.0] {
            let kg = gust_alleviation_factor(mu);
            let esperado = 0.88 * mu / (5.3 + mu);
            assert!((kg - esperado).abs() < 1e-9, "Kg({mu}) = {kg} != {esperado}");
        }
    }

    // ── Propriedade física: massa menor → rajada MAIOR (CS 23.341) ────────────
    #[test]
    fn massa_menor_produz_n_gust_maior() {
        let area = 14.2;
        let mac = 1.246;
        let a = lift_curve_slope(10.04);
        let vc_ms_v = vc_ms(280.0);

        let n_pesado = gust_n_at(1543.4, area, mac, a, vc_ms_v, UDE_VC_MS);
        let n_leve = gust_n_at(1150.0, area, mac, a, vc_ms_v, UDE_VC_MS);

        println!("n_gust(pesado=1543.4kg)={n_pesado:.3}  n_gust(leve=1150kg)={n_leve:.3}");
        assert!(n_leve > n_pesado,
            "massa leve deveria produzir n_gust MAIOR: leve={n_leve:.3} pesado={n_pesado:.3}");
    }

    // ── Mapeamento de categoria (CS 23.337) ───────────────────────────────────
    #[test]
    fn n_lim_neg_normal_e_utility_04() {
        assert!((load_factor_limit_neg("normal", 3.8) - (-1.52)).abs() < 0.001);
        assert!((load_factor_limit_neg("utility", 4.4) - (-1.76)).abs() < 0.001);
    }

    #[test]
    fn n_lim_neg_acrobatic_05() {
        assert!((load_factor_limit_neg("acrobatic", 6.0) - (-3.0)).abs() < 0.001);
    }

    // ── Polígono ───────────────────────────────────────────────────────────
    #[test]
    fn poligono_comeca_e_fecha_na_origem() {
        let w = wing();
        let req = requisitos_teste();
        let vn = VnDiagramAgent::run(&w, 1543.4, 1150.0, &req, "normal");
        assert_eq!(vn.points[0], [0.0, 0.0]);
        assert_eq!(*vn.points.last().unwrap(), [0.0, 0.0]);
    }

    #[test]
    fn poligono_contem_va_no_lim_pos() {
        let w = wing();
        let req = requisitos_teste();
        let vn = VnDiagramAgent::run(&w, 1543.4, 1150.0, &req, "normal");
        let contains_va = vn.points.iter().any(|p| {
            (p[0] - vn.va_kmh).abs() < 1e-6 && (p[1] - vn.n_lim_pos).abs() < 1e-6
        });
        assert!(contains_va, "polígono deveria conter vértice (VA, n_lim_pos) = ({:.1}, {:.2})",
            vn.va_kmh, vn.n_lim_pos);
    }

    #[test]
    fn poligono_velocidade_maxima_e_vd() {
        let w = wing();
        let req = requisitos_teste();
        let vn = VnDiagramAgent::run(&w, 1543.4, 1150.0, &req, "normal");
        let v_max = vn.points.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max);
        assert!((v_max - vn.vd_kmh).abs() < 1e-6,
            "V máximo do polígono ({v_max:.1}) deveria ser VD ({:.1})", vn.vd_kmh);
    }

    #[test]
    fn poligono_faixa_de_n_e_lim_neg_ate_n_design() {
        let w = wing();
        let req = requisitos_teste();
        let vn = VnDiagramAgent::run(&w, 1543.4, 1150.0, &req, "normal");
        let n_max = vn.points.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max);
        let n_min = vn.points.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min);
        assert!((n_max - vn.n_design).abs() < 1e-6,
            "n máximo do polígono ({n_max:.3}) deveria ser n_design ({:.3})", vn.n_design);
        assert!((n_min - vn.n_lim_neg).abs() < 1e-6,
            "n mínimo do polígono ({n_min:.3}) deveria ser n_lim_neg ({:.3})", vn.n_lim_neg);
    }

    // ── Integração: baseline real (Toyota + config_teste NÃO usados aqui —
    // pins calculados a partir dos dados sintéticos de config_teste, único
    // fixture disponível a este nível) ────────────────────────────────────
    #[test]
    fn n_design_e_no_minimo_n_lim_pos() {
        let w = wing();
        let req = requisitos_teste();
        let vn = VnDiagramAgent::run(&w, 1543.4, 1150.0, &req, "normal");
        assert!(vn.n_design >= vn.n_lim_pos - 1e-9);
        assert!(vn.n_design >= vn.n_gust_vc - 1e-9);
        assert!(vn.n_design >= vn.n_gust_vc_light - 1e-9);
    }

    #[test]
    fn vd_maior_que_vc_maior_que_va() {
        let w = wing();
        let req = requisitos_teste();
        let vn = VnDiagramAgent::run(&w, 1543.4, 1150.0, &req, "normal");
        assert!(vn.vd_kmh > vn.vc_kmh, "VD {:.1} deveria ser > VC {:.1}", vn.vd_kmh, vn.vc_kmh);
        assert!(vn.va_kmh < vn.vd_kmh, "VA {:.1} deveria ser < VD {:.1}", vn.va_kmh, vn.vd_kmh);
    }
}
