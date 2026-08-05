/// TrimAuthorityAgent — Limite Dianteiro Físico do Envelope de CG (flare + rotação)
///
/// Substitui o antigo proxy `stability.sm_max` (margem estática máxima —
/// sem base física direta em autoridade de controle) por um limite
/// dianteiro calculado a partir da autoridade de profundor disponível nas
/// DUAS manobras críticas de arfagem nariz-para-cima da vida operacional da
/// aeronave:
///
///   - **Flare no pouso** (V_ref = 1,3·Vs0, flap de pouso): balanço de
///     momentos em torno do CG, voo 1g. CG mais à frente exige mais
///     download da empenagem para segurar o nariz — resolvido por
///     bisseção (`flare_fwd_limit_frac`).
///   - **Rotação na decolagem** (Vr = 1,1·Vs0, flap de decolagem): balanço
///     de momentos em torno do TREM PRINCIPAL (é o trem, não o CG, que faz
///     de pivô na rotação). CG mais à frente aumenta o braço de peso em
///     torno do trem, exigindo mais download — solução fechada (linear em
///     x_cg, `rotation_fwd_limit_m`), calculada POR CENÁRIO de carga (o
///     peso varia).
///
/// O limite dianteiro de cada cenário é `max(flare, rotação_do_cenário)` —
/// o mais restritivo das duas. Ver `models::specs::TrimSpec` para a saída
/// completa e `agents::weight_balance::WeightBalanceOutput::apply_trim`
/// para como este resultado finaliza `ScenarioResult::inside_envelope`.
///
/// ACHADO DE PROJETO (honesto, não um bug deste código): no baseline real,
/// a ROTAÇÃO governa (≈29,6% MAC no cenário solo, crescendo com o peso) —
/// muito mais restritiva que a flare (≈5,5% MAC) e que o antigo proxy
/// `sm_max` (16,6% MAC). Causa física: o trem principal (`[gear].x_main_m`)
/// fica muito atrás do CG desta célula (a carga no trem de nariz já está em
/// 20–24%, perto do teto de 25% da Task 4.5/CS-23) — o braço de peso em
/// torno do trem principal é grande, exigindo mais autoridade de profundor
/// do que a empenagem entrega. NÃO é uma decisão deste agente ajustar
/// `[gear].x_main_m` — é uma decisão de projeto humana, reportada aqui com
/// destaque para revisão.
///
/// Referências:
///   - Raymer, D. "Aircraft Design: A Conceptual Approach", cap. 16
///     (momentos de arfagem, flap) — ΔCm de flap.
///   - Gudmundsson, S. "General Aviation Aircraft Design", cap. 16/20 —
///     autoridade de profundor, rotação de decolagem.
///   - Abbott & von Doenhoff, "Theory of Wing Sections" — Cm_ac quase nulo
///     da série NACA 230.
use crate::agents::weight_balance::{cg_pct_mac, WeightBalanceOutput};
use crate::models::atmosphere::RHO_SL;
use crate::models::{
    aircraft_config::AircraftConfig,
    specs::{EmpennageSpec, ScenarioTrimLimit, TrimSensitivity, TrimSpec, WingSpec},
};

const G: f64 = 9.807; // m/s²

/// Variação de `cl_h_max_down` usada no recálculo de sensibilidade
/// (`TrimSensitivity`) — ±0,05, conforme o brief da task.
const SENSITIVITY_DELTA: f64 = 0.05;

// ─── FLARE (POUSO) ────────────────────────────────────────────────────────

/// CL de equilíbrio 1g na flare (V_ref = 1,3·Vs0) — independe do peso do
/// cenário: `CL_flare = CL_max_flaps / 1,3² = CL_max_flaps/1,69`.
pub fn cl_flare(cl_max_flaps: f64) -> f64 {
    cl_max_flaps / 1.69
}

/// CL_h requerido no CG `x̄` (fração da MAC, LE do MAC = 0, x̄_ac = 0,25)
/// para equilibrar o momento de arfagem em torno do CG durante a flare —
/// balanço de momentos adimensional (ver task-1-brief.md):
///
///   CL_h(x̄) = [cm_ac_total + CL_flare·(x̄ − 0,25)] / [η_h·(S_h/S_w)·(l_h/MAC + 0,25 − x̄)]
///
/// `cm_ac_total = cm_ac + cm_flap_delta` (perfil + flap de pouso cheio);
/// `l_h_over_mac = l_h/MAC` (braço CA-asa→CA-empenagem, em frações de MAC).
pub fn cl_h_required_flare(
    x_bar: f64,
    cm_ac_total: f64,
    cl_flare: f64,
    eta_h: f64,
    s_h_over_s_w: f64,
    l_h_over_mac: f64,
) -> f64 {
    let l_cg_over_mac = l_h_over_mac + 0.25 - x_bar;
    (cm_ac_total + cl_flare * (x_bar - 0.25)) / (eta_h * s_h_over_s_w * l_cg_over_mac)
}

/// CL_h disponível — download máximo da empenagem horizontal com o
/// profundor no batente, com a margem de trim (`trim_margin`) reservada
/// para efeito solo/certificação:
///
///   CL_h_avail = −cl_h_max_down·(1 − trim_margin)
pub fn cl_h_available(cl_h_max_down: f64, trim_margin: f64) -> f64 {
    -cl_h_max_down * (1.0 - trim_margin)
}

/// Limite dianteiro de flare (fração da MAC) — resolve por bisseção
/// `cl_h_required_flare(x̄) = cl_h_avail`. `cl_h_required_flare` CRESCE
/// (fica menos negativo) MONOTONICAMENTE com `x̄` no domínio físico
/// relevante (o numerador cresce com `cl_flare > 0`, o denominador — a
/// distância CG→CA-empenagem — encolhe, ambos os efeitos empurram o
/// quociente para cima) — CG mais à frente exige CL_h mais negativo
/// (mais download), CG mais atrás exige menos.
pub fn flare_fwd_limit_frac(
    cm_ac_total: f64,
    cl_flare: f64,
    eta_h: f64,
    s_h_over_s_w: f64,
    l_h_over_mac: f64,
    cl_h_avail: f64,
) -> f64 {
    let f = |x: f64| {
        cl_h_required_flare(x, cm_ac_total, cl_flare, eta_h, s_h_over_s_w, l_h_over_mac) - cl_h_avail
    };
    let mut lo = -1.0_f64;
    let mut hi = 1.5_f64;
    debug_assert!(
        f(lo) <= 0.0 && f(hi) >= 0.0,
        "flare_fwd_limit_frac: bracket [{lo}, {hi}] não contém a raiz — parâmetros fora do \
         domínio físico esperado (cm_ac_total={cm_ac_total}, cl_flare={cl_flare}, \
         cl_h_avail={cl_h_avail})"
    );
    for _ in 0..100 {
        let mid = 0.5 * (lo + hi);
        if f(mid) < 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

// ─── ROTAÇÃO (DECOLAGEM) ────────────────────────────────────────────────────

/// Limite dianteiro de rotação (m do datum, NÃO %MAC) para UM cenário de
/// peso — balanço de momentos em torno do TREM PRINCIPAL na rotação de
/// decolagem (Vr = 1,1·Vs0, flap de decolagem). Fechado (não precisa de
/// bisseção — a equação de equilíbrio é linear em `x_cg`):
///
///   F_h = q_r·S_h·η_h·cl_h_max_down·(1−trim_margin)      [download da EH, N]
///   L_g = q_r·S_w·cl_ground_rotation                      [sustentação da asa, N]
///   Cm_TO = cm_ac + to_flap_cm_fraction·cm_flap_delta      [perfil+flap TO]
///   M_cm = |Cm_TO|·q_r·S_w·MAC                             [momento de perfil, N·m]
///
///   x_cg_rot = x_main − [F_h·(x_ac_tail−x_main) + L_g·(x_main−x_ac_wing) − M_cm] / W
///
/// CG mais à frente que `x_cg_rot` NÃO tem autoridade de profundor
/// suficiente para rotacionar neste cenário de peso.
#[allow(clippy::too_many_arguments)]
pub fn rotation_fwd_limit_m(
    q_r: f64,
    s_h_m2: f64,
    eta_h: f64,
    cl_h_max_down: f64,
    trim_margin: f64,
    x_ac_tail_m: f64,
    x_main_m: f64,
    s_w_m2: f64,
    cl_ground_rotation: f64,
    x_ac_wing_m: f64,
    cm_ac: f64,
    to_flap_cm_fraction: f64,
    cm_flap_delta: f64,
    mac_m: f64,
    weight_n: f64,
) -> f64 {
    let f_h = q_r * s_h_m2 * eta_h * cl_h_max_down * (1.0 - trim_margin);
    let l_g = q_r * s_w_m2 * cl_ground_rotation;
    let cm_to = cm_ac + to_flap_cm_fraction * cm_flap_delta;
    let m_cm = cm_to.abs() * q_r * s_w_m2 * mac_m;
    x_main_m - (f_h * (x_ac_tail_m - x_main_m) + l_g * (x_main_m - x_ac_wing_m) - m_cm) / weight_n
}

// ─── AGENTE PRINCIPAL ────────────────────────────────────────────────────────

pub struct TrimAuthorityAgent;

impl TrimAuthorityAgent {
    /// Calcula o `TrimSpec` completo — limite de flare (único) + limite de
    /// rotação por cenário (`wb.scenarios`, saída já calculada do
    /// `WeightBalanceAgent`) — mais a sensibilidade a `cl_h_max_down` e os
    /// parâmetros ecoados. NÃO modifica `wb` — ver
    /// `WeightBalanceOutput::apply_trim` para a etapa que consome este
    /// resultado e finaliza `inside_envelope`/`cg_limit_fwd_pct_mac`.
    pub fn run(
        cfg: &AircraftConfig,
        wing: &WingSpec,
        emp: &EmpennageSpec,
        wb: &WeightBalanceOutput,
    ) -> TrimSpec {
        let mac = wb.mac_m;
        let mac_le = wb.mac_le_x_m;
        let l_h_over_mac = emp.arm_h_m / mac;
        let s_ratio = emp.s_horizontal_m2 / wing.area_m2;
        let cm_ac_total = cfg.wing.cm_ac + cfg.wing.cm_flap_delta;
        let clf = cl_flare(wing.cl_max);
        let cl_avail = cl_h_available(cfg.stability.cl_h_max_down, cfg.stability.trim_margin);

        let x_flare =
            flare_fwd_limit_frac(cm_ac_total, clf, emp.eta_h, s_ratio, l_h_over_mac, cl_avail);
        let flare_limit_pct = x_flare * 100.0;
        let cl_h_req_at_limit =
            cl_h_required_flare(x_flare, cm_ac_total, clf, emp.eta_h, s_ratio, l_h_over_mac);

        // Rotação: Vr = 1,1·Vs0 — Vs0 vem de `WingSpec::stall_speed_flaps_kmh`
        // (calculada no MTOW de PROJETO pelo AerodynamicsAgent — não
        // recalculada por cenário; só o termo de PESO na equação de
        // equilíbrio varia por cenário, ver task-1-brief.md hand-check).
        let vs0_ms = wing.stall_speed_flaps_kmh / 3.6;
        let vr_ms = 1.1 * vs0_ms;
        let q_r = 0.5 * RHO_SL * vr_ms * vr_ms;
        let x_ac_wing = cfg.wing.le_root_x_m + 0.25 * mac;
        let x_ac_tail = cfg.wing.le_root_x_m + 0.25 * mac + emp.arm_h_m;

        let mut per_scenario = Vec::with_capacity(wb.scenarios.len());
        for sc in &wb.scenarios {
            let w_n = sc.total_mass_kg * G;
            let x_rot = rotation_fwd_limit_m(
                q_r,
                emp.s_horizontal_m2,
                emp.eta_h,
                cfg.stability.cl_h_max_down,
                cfg.stability.trim_margin,
                x_ac_tail,
                cfg.gear.x_main_m,
                wing.area_m2,
                cfg.stability.cl_ground_rotation,
                x_ac_wing,
                cfg.wing.cm_ac,
                cfg.stability.to_flap_cm_fraction,
                cfg.wing.cm_flap_delta,
                mac,
                w_n,
            );
            let rot_pct = cg_pct_mac(x_rot, mac_le, mac);
            let governing = if rot_pct > flare_limit_pct { "rotacao" } else { "flare" };
            per_scenario.push(ScenarioTrimLimit {
                scenario: sc.name.to_string(),
                rotation_limit_pct_mac: rot_pct,
                governing_limit_pct_mac: flare_limit_pct.max(rot_pct),
                governing: governing.to_string(),
            });
        }

        let governing = if per_scenario.iter().all(|s| s.governing == "rotacao") {
            "rotacao"
        } else if per_scenario.iter().all(|s| s.governing == "flare") {
            "flare"
        } else {
            "misto"
        }
        .to_string();

        // Sensibilidade a cl_h_max_down (±0,05) — mesma resolução do
        // limite de flare "nominal" acima, só variando o parâmetro.
        let cl_minus = cfg.stability.cl_h_max_down - SENSITIVITY_DELTA;
        let cl_plus = cfg.stability.cl_h_max_down + SENSITIVITY_DELTA;
        let x_flare_minus = flare_fwd_limit_frac(
            cm_ac_total, clf, emp.eta_h, s_ratio, l_h_over_mac,
            cl_h_available(cl_minus, cfg.stability.trim_margin),
        );
        let x_flare_plus = flare_fwd_limit_frac(
            cm_ac_total, clf, emp.eta_h, s_ratio, l_h_over_mac,
            cl_h_available(cl_plus, cfg.stability.trim_margin),
        );

        TrimSpec {
            flare_limit_pct_mac: flare_limit_pct,
            rotation_limit_pct_mac_per_scenario: per_scenario,
            governing,
            cl_h_required_at_fwd_limit: cl_h_req_at_limit,
            cl_h_available: cl_avail,
            sensitivity: TrimSensitivity {
                cl_h_max_down_minus: cl_minus,
                flare_limit_pct_mac_minus: x_flare_minus * 100.0,
                cl_h_max_down_plus: cl_plus,
                flare_limit_pct_mac_plus: x_flare_plus * 100.0,
            },
            cm_ac: cfg.wing.cm_ac,
            cm_flap_delta: cfg.wing.cm_flap_delta,
            cl_h_max_down: cfg.stability.cl_h_max_down,
            trim_margin: cfg.stability.trim_margin,
            cl_ground_rotation: cfg.stability.cl_ground_rotation,
            to_flap_cm_fraction: cfg.stability.to_flap_cm_fraction,
        }
    }
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ─── Hand-checks (valores do baseline real, ver task-1-brief.md) ────────
    //
    // MAC=1,2463161m, l_h=4,80m → l_h/MAC=3,85135; S_h/S=2,580913/14,2=
    // 0,181754; η_h=0,90; CL_flare=1,72/1,69=1,017751;
    // cm_ac_total=−0,008−0,30=−0,308; avail=−0,85·0,90=−0,765.

    #[test]
    fn cl_flare_hand_check() {
        let clf = cl_flare(1.72);
        assert!((clf - 1.017751).abs() < 1e-4, "cl_flare = {clf:.6} (esperado ≈1.017751)");
    }

    #[test]
    fn cl_h_available_hand_check() {
        let avail = cl_h_available(0.85, 0.10);
        assert!((avail - (-0.765)).abs() < 1e-9, "avail = {avail:.6} (esperado -0.765)");
    }

    /// Hand-check do limite de flare (task-1-brief.md): x̄_flare ≈ 0,0551
    /// (5,5% MAC) ±0,5% MAC.
    #[test]
    fn flare_fwd_limit_frac_hand_check_baseline() {
        let mac = 1.2463161361039574;
        let l_h_over_mac = 4.80 / mac;
        let s_ratio = 2.5809129985152786 / 14.2;
        let cm_ac_total = -0.008 + -0.30;
        let clf = cl_flare(1.72);
        let avail = cl_h_available(0.85, 0.10);

        let x_flare = flare_fwd_limit_frac(cm_ac_total, clf, 0.90, s_ratio, l_h_over_mac, avail);
        println!("x_flare = {:.5} ({:.3}% MAC)", x_flare, x_flare * 100.0);
        assert!(
            (x_flare - 0.0551).abs() < 0.005,
            "x_flare = {:.4} (esperado ≈0.0551 ±0.005, i.e. ±0.5% MAC)",
            x_flare
        );

        // Sanidade direcional do brief: CL_h_req(16.6%) deveria ficar
        // DENTRO da autoridade (menos negativo que avail) — o limite
        // antigo (sm_max-proxy, 16.6% MAC) era conservador para a flare.
        let cl_req_166 = cl_h_required_flare(0.166, cm_ac_total, clf, 0.90, s_ratio, l_h_over_mac);
        assert!(
            cl_req_166 > avail,
            "CL_h_req(16.6%) = {cl_req_166:.4} deveria ser > avail = {avail:.4} (autoridade de \
             sobra a 16.6% MAC)"
        );
    }

    /// Hand-check do limite de rotação, cenário "Solo (piloto)" do baseline
    /// real (task-1-brief.md): W=1193,4·9,807N, Vs0=31,69m/s (114,08km/h),
    /// Vr=34,86m/s, q_r≈744,5Pa, x_main=3,85m, x_ac_wing=3,2116m,
    /// x_ac_tail=8,0116m → x_cg_rot ≈ 3,2690m → (3,2690−2,90)/1,2463 =
    /// 29,6% MAC ±1%.
    #[test]
    fn rotation_fwd_limit_m_hand_check_baseline_solo() {
        let mac = 1.2463161361039574;
        let mac_le = 2.90;
        let s_h = 2.5809129985152786;
        let s_w = 14.2;
        let x_ac_wing = 2.90 + 0.25 * mac;
        let x_ac_tail = 2.90 + 0.25 * mac + 4.80;
        let x_main = 3.85;

        let vs0_ms = 114.082375 / 3.6;
        let vr_ms = 1.1 * vs0_ms;
        let q_r = 0.5 * RHO_SL * vr_ms * vr_ms;
        println!("q_r = {q_r:.3} Pa (esperado ≈744.5 Pa)");

        let w_n = 1193.4 * G;
        let x_rot = rotation_fwd_limit_m(
            q_r, s_h, 0.90, 0.85, 0.10, x_ac_tail, x_main, s_w, 0.5, x_ac_wing,
            -0.008, 0.5, -0.30, mac, w_n,
        );
        println!("x_cg_rot = {x_rot:.4} m (esperado ≈3.2690 m)");
        assert!((x_rot - 3.2690).abs() < 0.02, "x_cg_rot = {x_rot:.4} (esperado ≈3.2690 ±0.02m)");

        let rot_pct = cg_pct_mac(x_rot, mac_le, mac);
        println!("rot_pct = {rot_pct:.3}% MAC (esperado ≈29.6% ±1%)");
        assert!((rot_pct - 29.6).abs() < 1.0, "rot_pct = {rot_pct:.3}% (esperado ≈29.6% ±1%)");
    }

    // ─── Propriedades ─────────────────────────────────────────────────────

    /// `cl_h_max_down` maior → mais autoridade de download disponível → o
    /// limite de flare AVANÇA (x̄ menor, estritamente) — mais autoridade
    /// permite CG mais à frente antes de esgotar a empenagem.
    #[test]
    fn flare_limit_avanca_quando_cl_h_max_down_aumenta() {
        let mac = 1.2463161361039574;
        let l_h_over_mac = 4.80 / mac;
        let s_ratio = 2.5809129985152786 / 14.2;
        let cm_ac_total = -0.308;
        let clf = cl_flare(1.72);

        let avail_baixo = cl_h_available(0.80, 0.10);
        let avail_alto = cl_h_available(0.90, 0.10);

        let x_baixo = flare_fwd_limit_frac(cm_ac_total, clf, 0.90, s_ratio, l_h_over_mac, avail_baixo);
        let x_alto = flare_fwd_limit_frac(cm_ac_total, clf, 0.90, s_ratio, l_h_over_mac, avail_alto);

        println!("x_flare(cl_h_max_down=0.80)={x_baixo:.4}  x_flare(0.90)={x_alto:.4}");
        assert!(
            x_alto < x_baixo,
            "limite de flare com cl_h_max_down maior ({x_alto:.4}) deveria ser MENOR (mais à \
             frente) que com cl_h_max_down menor ({x_baixo:.4})"
        );
    }

    /// Trem principal mais à FRENTE (x_main menor) → braço de peso em
    /// torno do trem menor → limite de rotação AVANÇA (x_cg_rot menor,
    /// estritamente) — fica mais fácil rotacionar, então o CG pode ir mais
    /// à frente antes de esgotar a autoridade.
    #[test]
    fn rotation_limit_avanca_quando_x_main_diminui() {
        let mac = 1.2463161361039574;
        let s_h = 2.5809129985152786;
        let s_w = 14.2;
        let x_ac_wing = 2.90 + 0.25 * mac;
        let x_ac_tail = 2.90 + 0.25 * mac + 4.80;
        let q_r = 744.258105;
        let w_n = 1193.4 * G;

        let x_rot_frente = rotation_fwd_limit_m(
            q_r, s_h, 0.90, 0.85, 0.10, x_ac_tail, 3.50, s_w, 0.5, x_ac_wing,
            -0.008, 0.5, -0.30, mac, w_n,
        );
        let x_rot_atras = rotation_fwd_limit_m(
            q_r, s_h, 0.90, 0.85, 0.10, x_ac_tail, 4.20, s_w, 0.5, x_ac_wing,
            -0.008, 0.5, -0.30, mac, w_n,
        );

        println!("x_cg_rot(x_main=3.50)={x_rot_frente:.4}  x_cg_rot(x_main=4.20)={x_rot_atras:.4}");
        assert!(
            x_rot_frente < x_rot_atras,
            "limite de rotação com trem mais à frente ({x_rot_frente:.4}) deveria ser MENOR \
             que com trem mais atrás ({x_rot_atras:.4})"
        );
    }

    /// Cenário mais PESADO → maior peso a segurar em torno do trem → limite
    /// de rotação RECUA (x_cg_rot maior, %MAC maior) — precisa de mais CG
    /// atrás para reduzir o braço de peso à autoridade disponível.
    #[test]
    fn rotation_limit_recua_quando_peso_do_cenario_aumenta() {
        let mac = 1.2463161361039574;
        let s_h = 2.5809129985152786;
        let s_w = 14.2;
        let x_ac_wing = 2.90 + 0.25 * mac;
        let x_ac_tail = 2.90 + 0.25 * mac + 4.80;
        let q_r = 744.258105;
        let x_main = 3.85;

        let x_rot_leve = rotation_fwd_limit_m(
            q_r, s_h, 0.90, 0.85, 0.10, x_ac_tail, x_main, s_w, 0.5, x_ac_wing,
            -0.008, 0.5, -0.30, mac, 1193.4 * G,
        );
        let x_rot_pesado = rotation_fwd_limit_m(
            q_r, s_h, 0.90, 0.85, 0.10, x_ac_tail, x_main, s_w, 0.5, x_ac_wing,
            -0.008, 0.5, -0.30, mac, 1543.4 * G,
        );

        println!("x_cg_rot(1193.4kg)={x_rot_leve:.4}  x_cg_rot(1543.4kg)={x_rot_pesado:.4}");
        assert!(
            x_rot_pesado > x_rot_leve,
            "limite de rotação do cenário mais pesado ({x_rot_pesado:.4}) deveria ser MAIOR \
             (mais atrás) que o do mais leve ({x_rot_leve:.4})"
        );
    }

    // ─── Integração via pipeline real (baseline_4seat.toml) ─────────────────

    /// Roda o `TrimAuthorityAgent` sobre o pipeline completo do baseline
    /// real (mesmo padrão de `weight_balance::tests::
    /// neutral_point_m_hand_check_downwash_e_fuselagem`) e confirma: (1) o
    /// limite de flare bate com o hand-check isolado acima; (2) o limite de
    /// rotação do cenário "Solo (piloto)" bate; (3) a ROTAÇÃO GOVERNA
    /// (achado honesto do projeto, ver docstring do módulo) — mais
    /// restritiva que a flare em TODOS os cenários.
    #[test]
    fn trim_authority_agent_run_hand_check_baseline_real() {
        let toml = std::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("config/aircraft/baseline_4seat.toml"),
        )
        .expect("falha ao ler baseline_4seat.toml do disco");
        let cfg = crate::models::config::parse_aircraft(&toml)
            .expect("baseline real deveria ser uma configuração válida");
        let req = crate::models::requirements::test_fixtures::requisitos_teste();
        let state = crate::models::aircraft_state::AircraftState::from_config(&cfg);
        let wing = crate::agents::aerodynamics::AerodynamicsAgent::run(&state, &req);
        let emp = crate::agents::empennage::EmpennageAgent::run(&wing, &cfg);
        let engine = crate::models::engine::test_fixtures::motor_generico_teste();
        let wb = crate::agents::weight_balance::WeightBalanceAgent::run(
            &state, &wing, &engine, &cfg, &req, &emp,
        );

        let trim = TrimAuthorityAgent::run(&cfg, &wing, &emp, &wb);
        println!("flare_limit_pct_mac = {:.3}", trim.flare_limit_pct_mac);
        assert!(
            (trim.flare_limit_pct_mac - 5.51).abs() < 1.0,
            "flare_limit_pct_mac = {:.3} (esperado ≈5.51% ±1%)",
            trim.flare_limit_pct_mac
        );

        let solo = trim.rotation_limit_pct_mac_per_scenario.iter()
            .find(|s| s.scenario == "Solo (piloto)")
            .expect("cenário 'Solo (piloto)' deveria estar presente");
        println!("rotation_limit_pct_mac (Solo) = {:.3}", solo.rotation_limit_pct_mac);
        assert!(
            (solo.rotation_limit_pct_mac - 29.6).abs() < 1.5,
            "rotation_limit_pct_mac (Solo) = {:.3} (esperado ≈29.6% ±1.5%)",
            solo.rotation_limit_pct_mac
        );

        // Achado honesto: a rotação governa em TODOS os cenários — mais
        // restritiva que a flare (ver docstring do módulo).
        for sc in &trim.rotation_limit_pct_mac_per_scenario {
            assert!(
                sc.rotation_limit_pct_mac > trim.flare_limit_pct_mac,
                "cenário '{}': rotação ({:.2}%) deveria ser MAIS restritiva que a flare \
                 ({:.2}%) no baseline real",
                sc.scenario, sc.rotation_limit_pct_mac, trim.flare_limit_pct_mac
            );
        }
        assert_eq!(trim.governing, "rotacao",
            "governing agregado deveria ser 'rotacao' (achado honesto do baseline real)");
    }

    /// Sensibilidade (task-1-brief.md): o limite de flare recomputado a
    /// `cl_h_max_down ± 0.05` deve mover na direção esperada — mais
    /// autoridade (plus) → limite mais à frente (menor); menos autoridade
    /// (minus) → limite mais atrás (maior).
    #[test]
    fn sensitivity_flare_limits_movem_na_direcao_esperada() {
        let toml = std::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("config/aircraft/baseline_4seat.toml"),
        )
        .expect("falha ao ler baseline_4seat.toml do disco");
        let cfg = crate::models::config::parse_aircraft(&toml)
            .expect("baseline real deveria ser uma configuração válida");
        let req = crate::models::requirements::test_fixtures::requisitos_teste();
        let state = crate::models::aircraft_state::AircraftState::from_config(&cfg);
        let wing = crate::agents::aerodynamics::AerodynamicsAgent::run(&state, &req);
        let emp = crate::agents::empennage::EmpennageAgent::run(&wing, &cfg);
        let engine = crate::models::engine::test_fixtures::motor_generico_teste();
        let wb = crate::agents::weight_balance::WeightBalanceAgent::run(
            &state, &wing, &engine, &cfg, &req, &emp,
        );

        let trim = TrimAuthorityAgent::run(&cfg, &wing, &emp, &wb);
        println!(
            "sensitivity: minus={:.3}% (cl={:.2})  nominal={:.3}%  plus={:.3}% (cl={:.2})",
            trim.sensitivity.flare_limit_pct_mac_minus, trim.sensitivity.cl_h_max_down_minus,
            trim.flare_limit_pct_mac,
            trim.sensitivity.flare_limit_pct_mac_plus, trim.sensitivity.cl_h_max_down_plus,
        );
        assert!(trim.sensitivity.flare_limit_pct_mac_minus > trim.flare_limit_pct_mac,
            "menos autoridade (cl_h_max_down−0.05) deveria mover o limite de flare para TRÁS \
             (%MAC maior)");
        assert!(trim.sensitivity.flare_limit_pct_mac_plus < trim.flare_limit_pct_mac,
            "mais autoridade (cl_h_max_down+0.05) deveria mover o limite de flare para a \
             FRENTE (%MAC menor)");
    }
}
