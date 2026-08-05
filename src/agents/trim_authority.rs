/// TrimAuthorityAgent — Limite Dianteiro Físico do Envelope de CG (flare + rotação)
///
/// Substitui o antigo proxy `stability.sm_max` (margem estática máxima —
/// sem base física direta em autoridade de controle) por um limite
/// dianteiro calculado a partir da autoridade de profundor disponível nas
/// DUAS manobras críticas de arfagem nariz-para-cima da vida operacional da
/// aeronave:
///
///   - **Flare no pouso** (V_ref = 1,3·Vs0, flap de pouso): balanço de
///     momentos em torno do CG, voo 1g, FECHADO pela contribuição de
///     sustentação da própria empenagem (ver `cl_h_required_flare`) —
///     resolvido por bisseção (`flare_fwd_limit_frac`).
///   - **Rotação na decolagem** (Vr = 1,1·Vs0(W), flap de decolagem):
///     balanço de momentos em torno do TREM PRINCIPAL (é o trem, não o CG,
///     que faz de pivô na rotação) — solução fechada
///     (`rotation_fwd_limit_m`). **INVARIANTE ao peso** — ver a docstring
///     de `rotation_fwd_limit_m` para a dedução completa do cancelamento
///     algébrico de `W`.
///
/// O limite dianteiro efetivo é `max(flare, rotação)` — o mais restritivo
/// das duas, e é o MESMO para todos os cenários de carga (nenhum dos dois
/// varia por cenário — ver `models::specs::TrimSpec`). A margem de
/// autoridade de rotação avaliada na CG/peso REAIS de cada cenário (que
/// essa sim varia) fica em `TrimSpec::rotation_margin_per_scenario` — ver
/// `rotation_available_moment_nm`.
///
/// ACHADO DE PROJETO (honesto, não um bug deste código): no baseline real,
/// a ROTAÇÃO governa (≈39,9% MAC) — muito mais restritiva que a flare
/// (≈7,9% MAC) e que o antigo proxy `sm_max` (16,6% MAC), E fica À FRENTE
/// do limite TRASEIRO de estabilidade (≈36,6% MAC) — **o envelope de CG
/// admissível fica VAZIO** (nenhuma posição de CG satisfaz os dois
/// critérios simultaneamente). Causa física: o trem principal
/// (`[gear].x_main_m`) fica muito atrás do CG desta célula (a carga no
/// trem de nariz já está em 20–24%, perto do teto de 25% da Task
/// 4.5/CS-23) — o braço de peso em torno do trem principal é grande,
/// exigindo mais autoridade de profundor do que a empenagem entrega. NÃO é
/// uma decisão deste agente ajustar `[gear].x_main_m` — é uma decisão de
/// projeto humana, reportada aqui (e em `ConstraintChecker::verify`, com um
/// item de violação DEDICADO "Envelope de CG VAZIO") com destaque para
/// revisão.
///
/// Referências:
///   - Raymer, D. "Aircraft Design: A Conceptual Approach", cap. 16
///     (momentos de arfagem, flap) — ΔCm de flap.
///   - Gudmundsson, S. "General Aviation Aircraft Design", cap. 16/20 —
///     autoridade de profundor, rotação de decolagem.
///   - Abbott & von Doenhoff, "Theory of Wing Sections" — Cm_ac quase nulo
///     da série NACA 230.
use crate::agents::weight_balance::{cg_pct_mac, WeightBalanceOutput};
use crate::models::{
    aircraft_config::AircraftConfig,
    specs::{EmpennageSpec, ScenarioTrimLimit, TrimSensitivity, TrimSpec, WingSpec},
};

const G: f64 = 9.807; // m/s²

/// Variação de `cl_h_max_down` usada no recálculo de sensibilidade
/// (`TrimSensitivity`) — ±0,05, conforme o brief da task.
const SENSITIVITY_DELTA: f64 = 0.05;

/// `Vr/Vs0` — Vr = 1,1·Vs0, usado tanto para a dinâmica da rotação quanto
/// (elevado ao quadrado) para o cancelamento algébrico de peso em
/// `rotation_fwd_limit_m`/`rotation_available_moment_nm` — ver a docstring
/// dessas funções.
const VR_OVER_VS0: f64 = 1.1;

// ─── FLARE (POUSO) ────────────────────────────────────────────────────────

/// CL de equilíbrio 1g na flare (V_ref = 1,3·Vs0) — independe do peso do
/// cenário: `CL_flare = CL_max_flaps / 1,3² = CL_max_flaps/1,69`. Este é o
/// CL de sustentação TOTAL da aeronave (peso/q/S), não o CL da asa sozinha
/// — ver `cl_h_required_flare` para o fechamento entre os dois.
pub fn cl_flare(cl_max_flaps: f64) -> f64 {
    cl_max_flaps / 1.69
}

/// CL_h requerido no CG `x̄` (fração da MAC, LE do MAC = 0, x̄_ac = 0,25)
/// para equilibrar o momento de arfagem em torno do CG durante a flare —
/// balanço de momentos adimensional, FECHADO pela contribuição de
/// sustentação da própria empenagem (fix de revisão — a versão original
/// tratava `CL_flare` como o CL da ASA, ignorando que parte da sustentação
/// total vem, com sinal negativo (download), da empenagem):
///
///   CL_total = CL_flare = CL_w + η_h·(S_h/S_w)·CL_h        [fechamento vertical]
///   0 = cm_ac_total + CL_w·(x̄−0,25) − η_h·(S_h/S_w)·CL_h·(l_h/MAC+0,25−x̄)   [Σ M_cg = 0]
///
/// Substituindo `CL_w = CL_flare − η_h·(S_h/S_w)·CL_h` na equação de
/// momento e isolando `CL_h`, o termo `(l_h/MAC+0,25−x̄)` do denominador
/// se CANCELA algebricamente com o termo `(x̄−0,25)` que aparece ao
/// expandir `CL_w·(x̄−0,25)`, sobrando só a distância CA-asa→CA-empenagem
/// (`l_h/MAC`, CONSTANTE — não depende mais de `x̄`):
///
///   CL_h(x̄) = [cm_ac_total + CL_flare·(x̄ − 0,25)] / [η_h·(S_h/S_w)·(l_h/MAC)]
///
/// (dedução: expandir `CL_w·(x̄−0,25)` na equação de momento, agrupar os
/// termos em `CL_h`, e notar que
/// `(l_h/MAC+0,25−x̄) + (x̄−0,25) = l_h/MAC` — os termos em `x̄` se
/// cancelam exatamente, sobrevive só a constante). Consequência prática: a
/// curva `CL_h(x̄)` deixa de ter um polo em `x̄ = l_h/MAC+0,25` (o
/// denominador agora é uma constante positiva, nunca zero) — mais robusta
/// numericamente que a versão original. `cm_ac_total = cm_ac +
/// cm_flap_delta` (perfil + flap de pouso cheio); `l_h_over_mac = l_h/MAC`
/// (braço CA-asa→CA-empenagem, em frações de MAC).
pub fn cl_h_required_flare(
    x_bar: f64,
    cm_ac_total: f64,
    cl_flare: f64,
    eta_h: f64,
    s_h_over_s_w: f64,
    l_h_over_mac: f64,
) -> f64 {
    (cm_ac_total + cl_flare * (x_bar - 0.25)) / (eta_h * s_h_over_s_w * l_h_over_mac)
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
/// `cl_h_required_flare(x̄) = cl_h_avail`. `cl_h_required_flare` é LINEAR
/// (portanto monotonicamente crescente, já que `cl_flare > 0`) em `x̄` —
/// mantido como bisseção (não solução fechada direta) por robustez a
/// futuras versões do modelo de `Cm` que não sejam mais lineares. Sem
/// polo no domínio físico desde a correção do fechamento vertical (ver
/// `cl_h_required_flare`) — ainda assim, `l_h/MAC` curto demais (config
/// `[empennage].tail_arm_m` pequeno relativo à MAC) é rejeitado na
/// validação (`models::config::validate_aircraft`, ≤1,5) por sair do
/// domínio de validade deste modelo linearizado.
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

/// Momento NARIZ-ACIMA disponível na rotação de decolagem (N·m), em torno
/// do TREM PRINCIPAL, para um peso `weight_n` — soma de TRÊS fontes,
/// TODAS com o sinal físico correto (fix de revisão — a versão anterior
/// usava `|Cm_TO|`, perdendo o sinal; ver nota abaixo):
///
///   q_r(W) = 0,5·ρ·Vr² ,  Vr = 1,1·Vs0(W) ,  Vs0(W) = √(2W/(ρ·S_w·CL_max_flaps))
///          ⟹ q_r(W) = 1,21·W / (S_w·CL_max_flaps)      [PROPORCIONAL a W]
///
///   F_h = q_r·S_h·η_h·cl_h_max_down·(1−trim_margin)    [download da EH, N — nariz-ACIMA]
///   L_g = q_r·S_w·cl_ground_rotation                    [sustentação da asa, N — nariz-ACIMA]
///   Cm_TO = cm_ac + to_flap_cm_fraction·cm_flap_delta    [perfil+flap TO, SINALIZADO]
///
///   M_disponível = F_h·(x_ac_tail−x_main) + L_g·(x_main−x_ac_wing) + Cm_TO·q_r·S_w·MAC
///
/// `Cm_TO` é SOMADO (não subtraído) com seu próprio sinal: por convenção
/// aerodinâmica, `Cm` POSITIVO é nariz-ACIMA (ajuda a rotação — deve
/// SOMAR) e `Cm` NEGATIVO é nariz-ABAIXO (atrapalha — soma um número
/// negativo, ou seja, SUBTRAI o valor absoluto, na prática). A versão
/// anterior usava `|Cm_TO|` sempre subtraído, o que dava o resultado
/// numericamente CORRETO só por coincidência quando `Cm_TO` é negativo
/// (caso do baseline real, perfil câmbrado + flap) — mas inverteria o
/// sinal incorretamente para um perfil/flap com `Cm_TO` positivo (trataria
/// uma contribuição nariz-ACIMA como se atrapalhasse a rotação — ver
/// `tests::cm_to_positivo_move_limite_de_rotacao_para_a_frente_vs_negativo`).
#[allow(clippy::too_many_arguments)]
pub fn rotation_available_moment_nm(
    weight_n: f64,
    s_w_m2: f64,
    cl_max_flaps: f64,
    s_h_m2: f64,
    eta_h: f64,
    cl_h_max_down: f64,
    trim_margin: f64,
    x_ac_tail_m: f64,
    x_main_m: f64,
    cl_ground_rotation: f64,
    x_ac_wing_m: f64,
    cm_ac: f64,
    to_flap_cm_fraction: f64,
    cm_flap_delta: f64,
    mac_m: f64,
) -> f64 {
    let q_r = VR_OVER_VS0 * VR_OVER_VS0 * weight_n / (s_w_m2 * cl_max_flaps);
    let f_h = q_r * s_h_m2 * eta_h * cl_h_max_down * (1.0 - trim_margin);
    let l_g = q_r * s_w_m2 * cl_ground_rotation;
    let cm_to = cm_ac + to_flap_cm_fraction * cm_flap_delta;
    f_h * (x_ac_tail_m - x_main_m) + l_g * (x_main_m - x_ac_wing_m) + cm_to * q_r * s_w_m2 * mac_m
}

/// Limite dianteiro de rotação (m do datum, NÃO %MAC) — balanço de
/// momentos em torno do TREM PRINCIPAL na rotação de decolagem. Fechado
/// (solução direta, sem bisseção):
///
///   x_cg_rot = x_main − M_disponível(W) / W
///
/// **INVARIANTE AO PESO** (achado da revisão desta task — a primeira
/// versão deste agente calculava um limite DIFERENTE por cenário de carga,
/// partindo da premissa de que `x_cg_rot` dependeria do peso, mas com
/// `Vr = 1,1·Vs0(W)` isso NÃO acontece — a dependência em `W` cancela
/// exatamente): `q_r(W)` é PROPORCIONAL a `W` (ver
/// `rotation_available_moment_nm`) — logo `F_h`, `L_g` e o termo de
/// `Cm_TO` são TODOS proporcionais a `W`, e `M_disponível(W)` também é
/// proporcional a `W`. Ao dividir por `W` em
/// `x_cg_rot = x_main − M_disponível(W)/W`, o `W` CANCELA EXATAMENTE — o
/// resultado é o MESMO para qualquer cenário de carga (prova numérica em
/// `tests::rotation_limit_e_invariante_a_massas_diferentes`). Fisicamente
/// isso faz sentido: sob esta política de velocidade, uma aeronave mais
/// pesada rotaciona a uma Vr proporcionalmente maior (`Vs0 ∝ √W`), o que
/// aumenta `q_r` na medida exata (`q_r ∝ W`) para que a autoridade de
/// profundor disponível cresça na MESMA proporção que o momento de peso
/// que precisa vencer — os dois efeitos se cancelam.
///
/// Implementação: chama `rotation_available_moment_nm` com `weight_n=1,0`
/// (peso unitário) — como o resultado é proporcional a `W`, o valor obtido
/// com `W=1` já É `M_disponível(W)/W` para qualquer `W` (não precisa
/// dividir de novo). A margem de autoridade REAL de cada cenário (que usa
/// a CG/peso VERDADEIROS, não o limite) fica em
/// `TrimAuthorityAgent::run`/`rotation_available_moment_nm` diretamente —
/// ver `models::specs::ScenarioTrimLimit`.
#[allow(clippy::too_many_arguments)]
pub fn rotation_fwd_limit_m(
    s_w_m2: f64,
    cl_max_flaps: f64,
    s_h_m2: f64,
    eta_h: f64,
    cl_h_max_down: f64,
    trim_margin: f64,
    x_ac_tail_m: f64,
    x_main_m: f64,
    cl_ground_rotation: f64,
    x_ac_wing_m: f64,
    cm_ac: f64,
    to_flap_cm_fraction: f64,
    cm_flap_delta: f64,
    mac_m: f64,
) -> f64 {
    let moment_per_unit_weight = rotation_available_moment_nm(
        1.0, s_w_m2, cl_max_flaps, s_h_m2, eta_h, cl_h_max_down, trim_margin, x_ac_tail_m,
        x_main_m, cl_ground_rotation, x_ac_wing_m, cm_ac, to_flap_cm_fraction, cm_flap_delta,
        mac_m,
    );
    x_main_m - moment_per_unit_weight
}

// ─── AGENTE PRINCIPAL ────────────────────────────────────────────────────────

pub struct TrimAuthorityAgent;

impl TrimAuthorityAgent {
    /// Calcula o `TrimSpec` completo — limite de flare + limite de rotação
    /// (ambos números ÚNICOS, ver docstring de `TrimSpec`) + a margem de
    /// autoridade de rotação por cenário (`wb.scenarios`, saída já
    /// calculada do `WeightBalanceAgent`) + a sensibilidade a
    /// `cl_h_max_down` + os parâmetros ecoados. NÃO modifica `wb` — ver
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

        // Rotação — limite ÚNICO (invariante ao peso, ver
        // `rotation_fwd_limit_m`). Geometria: x_ac_wing/x_ac_tail medidos
        // do datum (bordo de ataque da asa + 0,25·MAC [+ braço da EH]).
        let x_ac_wing = cfg.wing.le_root_x_m + 0.25 * mac;
        let x_ac_tail = cfg.wing.le_root_x_m + 0.25 * mac + emp.arm_h_m;
        let x_rot = rotation_fwd_limit_m(
            wing.area_m2, wing.cl_max, emp.s_horizontal_m2, emp.eta_h,
            cfg.stability.cl_h_max_down, cfg.stability.trim_margin, x_ac_tail, cfg.gear.x_main_m,
            cfg.stability.cl_ground_rotation, x_ac_wing, cfg.wing.cm_ac,
            cfg.stability.to_flap_cm_fraction, cfg.wing.cm_flap_delta, mac,
        );
        let rotation_limit_pct = cg_pct_mac(x_rot, mac_le, mac);

        // Margem de autoridade de rotação por cenário — diagnóstico
        // informativo na CG/peso REAIS de cada cenário (varia por
        // cenário, ao contrário do limite acima) — ver `ScenarioTrimLimit`.
        let mut rotation_margin_per_scenario = Vec::with_capacity(wb.scenarios.len());
        for sc in &wb.scenarios {
            let w_n = sc.total_mass_kg * G;
            let available = rotation_available_moment_nm(
                w_n, wing.area_m2, wing.cl_max, emp.s_horizontal_m2, emp.eta_h,
                cfg.stability.cl_h_max_down, cfg.stability.trim_margin, x_ac_tail,
                cfg.gear.x_main_m, cfg.stability.cl_ground_rotation, x_ac_wing, cfg.wing.cm_ac,
                cfg.stability.to_flap_cm_fraction, cfg.wing.cm_flap_delta, mac,
            );
            let required = w_n * (cfg.gear.x_main_m - sc.x_cg_m);
            let margin_pct = (available - required) / required * 100.0;
            rotation_margin_per_scenario.push(ScenarioTrimLimit {
                scenario: sc.name.to_string(),
                rotation_authority_margin_pct: margin_pct,
            });
        }

        let governing =
            if rotation_limit_pct >= flare_limit_pct { "rotacao" } else { "flare" }.to_string();

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
            rotation_limit_pct_mac: rotation_limit_pct,
            rotation_margin_per_scenario,
            governing,
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
    use crate::models::atmosphere::RHO_SL;

    // ─── Hand-checks (valores do baseline real) ──────────────────────────
    //
    // MAC=1,2463161m, l_h=4,80m → l_h/MAC=3,85135; S_h/S=2,580913/14,2=
    // 0,181754; η_h=0,90; CL_flare=1,72/1,69=1,017751;
    // cm_ac_total=−0,008−0,30=−0,308; avail=−0,85·0,90=−0,765;
    // cm_to (rotação, SINALIZADO) = −0,008+0,5·(−0,30) = −0,158.

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

    /// Hand-check do limite de flare CORRIGIDO (fechamento vertical
    /// CL_w=CL_flare−η·s_ratio·CL_h — ver docstring de
    /// `cl_h_required_flare`): x̄_flare ≈ 0,07908 (7,908% MAC).
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
            (x_flare * 100.0 - 7.908).abs() < 0.05,
            "x_flare = {:.4}% MAC (esperado ≈7.908% ±0.05)",
            x_flare * 100.0
        );

        // Checagem de sanidade: por construção, CL_h_required no próprio
        // limite deve bater exatamente com a autoridade disponível.
        let cl_req = cl_h_required_flare(x_flare, cm_ac_total, clf, 0.90, s_ratio, l_h_over_mac);
        assert!((cl_req - avail).abs() < 1e-6,
            "CL_h_required(x_flare) = {cl_req:.6} deveria coincidir com cl_h_avail = {avail:.6}");
    }

    /// Hand-check do limite de rotação (fix de revisão — INVARIANTE ao
    /// peso, ver docstring de `rotation_fwd_limit_m`): x_cg_rot ≈ 3,3976 m
    /// → (3,3976−2,90)/1,2463 ≈ 39,93% MAC.
    #[test]
    fn rotation_fwd_limit_m_hand_check_baseline() {
        let mac = 1.2463161361039574;
        let mac_le = 2.90;
        let s_h = 2.5809129985152786;
        let s_w = 14.2;
        let x_ac_wing = 2.90 + 0.25 * mac;
        let x_ac_tail = 2.90 + 0.25 * mac + 4.80;
        let x_main = 3.85;

        let x_rot = rotation_fwd_limit_m(
            s_w, 1.72, s_h, 0.90, 0.85, 0.10, x_ac_tail, x_main, 0.5, x_ac_wing,
            -0.008, 0.5, -0.30, mac,
        );
        println!("x_cg_rot = {x_rot:.4} m (esperado ≈3.3976 m)");
        assert!((x_rot - 3.3976).abs() < 0.02, "x_cg_rot = {x_rot:.4} (esperado ≈3.3976 ±0.02m)");

        let rot_pct = cg_pct_mac(x_rot, mac_le, mac);
        println!("rot_pct = {rot_pct:.3}% MAC (esperado ≈39.93% ±1%)");
        assert!((rot_pct - 39.93).abs() < 1.0, "rot_pct = {rot_pct:.3}% (esperado ≈39.93% ±1%)");
    }

    // ─── FIX 1 (crítico): invariância ao peso ────────────────────────────

    /// Prova numérica do cancelamento algébrico de `W` (ver docstring de
    /// `rotation_fwd_limit_m`): duas massas bem diferentes (extremos do
    /// baseline real, 1193,4 kg vs 1543,4 kg) devem produzir o MESMO limite
    /// dianteiro de rotação, avaliando `rotation_available_moment_nm(W,...)
    /// / W` INDEPENDENTEMENTE para cada uma (não via `rotation_fwd_limit_m`,
    /// que já assume `W=1` — este teste valida essa suposição).
    #[test]
    fn rotation_limit_e_invariante_a_massas_diferentes() {
        let mac = 1.2463161361039574;
        let s_h = 2.5809129985152786;
        let s_w = 14.2;
        let x_ac_wing = 2.90 + 0.25 * mac;
        let x_ac_tail = 2.90 + 0.25 * mac + 4.80;
        let x_main = 3.85;

        let x_cg_rot_para = |mass_kg: f64| -> f64 {
            let w_n = mass_kg * G;
            let m = rotation_available_moment_nm(
                w_n, s_w, 1.72, s_h, 0.90, 0.85, 0.10, x_ac_tail, x_main, 0.5, x_ac_wing,
                -0.008, 0.5, -0.30, mac,
            );
            x_main - m / w_n
        };

        let x_leve = x_cg_rot_para(1193.4);
        let x_pesado = x_cg_rot_para(1543.4);
        println!("x_cg_rot(1193.4kg)={x_leve:.9}  x_cg_rot(1543.4kg)={x_pesado:.9}");
        assert!((x_leve - x_pesado).abs() < 1e-9,
            "limite de rotação deveria ser IDÊNTICO independente do peso do cenário: \
             leve={x_leve:.9}m pesado={x_pesado:.9}m (diferença={:.2e})", (x_leve-x_pesado).abs());
    }

    /// `rotation_fwd_limit_m` (fechado, assume `W=1`) deve bater com uma
    /// avaliação TOTALMENTE INDEPENDENTE do balanço de momentos — computa
    /// `Vs0(W)` via `sqrt`, `Vr`, `q_r` explicitamente para um peso
    /// arbitrário (12.000 N), em vez de usar a substituição algébrica
    /// `q_r(W) = 1,21·W/(S·CL_max)` usada internamente pela função.
    #[test]
    fn rotation_fwd_limit_m_bate_com_balanco_de_momentos_independente() {
        let mac = 1.2463161361039574;
        let s_h = 2.5809129985152786;
        let s_w = 14.2;
        let cl_max_flaps = 1.72;
        let x_ac_wing = 2.90 + 0.25 * mac;
        let x_ac_tail = 2.90 + 0.25 * mac + 4.80;
        let x_main = 3.85;

        let w_n = 12_000.0_f64;
        let vs0 = (2.0 * w_n / (RHO_SL * s_w * cl_max_flaps)).sqrt();
        let vr = 1.1 * vs0;
        let q_r = 0.5 * RHO_SL * vr * vr;
        let f_h = q_r * s_h * 0.90 * 0.85 * (1.0 - 0.10);
        let l_g = q_r * s_w * 0.5;
        let cm_to = -0.008 + 0.5 * -0.30;
        let m_cm = cm_to * q_r * s_w * mac;
        let moment = f_h * (x_ac_tail - x_main) + l_g * (x_main - x_ac_wing) + m_cm;
        let x_independente = x_main - moment / w_n;

        let x_fechado = rotation_fwd_limit_m(
            s_w, cl_max_flaps, s_h, 0.90, 0.85, 0.10, x_ac_tail, x_main, 0.5, x_ac_wing,
            -0.008, 0.5, -0.30, mac,
        );

        println!("independente={x_independente:.9}m  fechado={x_fechado:.9}m");
        assert!((x_independente - x_fechado).abs() < 1e-9,
            "solução fechada ({x_fechado:.9}m) deveria bater com o balanço de momentos \
             independente ({x_independente:.9}m)");
    }

    // ─── FIX 2 (importante): correção de sinal de Cm_TO ──────────────────

    /// `Cm_TO` POSITIVO (nariz-acima) deve mover o limite de rotação para a
    /// FRENTE (x_cg_rot MENOR) em relação ao mesmo `Cm_TO` NEGATIVO
    /// (nariz-abaixo, mesma magnitude) — um Cm positivo AJUDA a rotação
    /// (soma momento nariz-acima disponível), então exige menos autoridade
    /// de CG, permitindo um CG mais à frente.
    #[test]
    fn cm_to_positivo_move_limite_de_rotacao_para_a_frente_vs_negativo() {
        let mac = 1.2463161361039574;
        let s_h = 2.5809129985152786;
        let s_w = 14.2;
        let x_ac_wing = 2.90 + 0.25 * mac;
        let x_ac_tail = 2.90 + 0.25 * mac + 4.80;
        let x_main = 3.85;

        // to_flap_cm_fraction=0 e cm_flap_delta=0 (irrelevantes) — só
        // cm_ac controla o sinal de Cm_TO aqui.
        let x_positivo = rotation_fwd_limit_m(
            s_w, 1.72, s_h, 0.90, 0.85, 0.10, x_ac_tail, x_main, 0.5, x_ac_wing,
            0.04, 0.0, 0.0, mac,
        );
        let x_negativo = rotation_fwd_limit_m(
            s_w, 1.72, s_h, 0.90, 0.85, 0.10, x_ac_tail, x_main, 0.5, x_ac_wing,
            -0.04, 0.0, 0.0, mac,
        );

        println!("x_cg_rot(cm=+0.04)={x_positivo:.4}  x_cg_rot(cm=-0.04)={x_negativo:.4}");
        assert!(x_positivo < x_negativo,
            "Cm_TO positivo (+0.04, {x_positivo:.4}m) deveria mover o limite de rotação para a \
             FRENTE (x menor) em relação ao Cm_TO negativo (-0.04, {x_negativo:.4}m) — Cm \
             positivo é nariz-acima e ajuda a rotação");
    }

    // ─── Propriedades da flare ────────────────────────────────────────────

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

        let x_rot_frente = rotation_fwd_limit_m(
            s_w, 1.72, s_h, 0.90, 0.85, 0.10, x_ac_tail, 3.50, 0.5, x_ac_wing,
            -0.008, 0.5, -0.30, mac,
        );
        let x_rot_atras = rotation_fwd_limit_m(
            s_w, 1.72, s_h, 0.90, 0.85, 0.10, x_ac_tail, 4.20, 0.5, x_ac_wing,
            -0.008, 0.5, -0.30, mac,
        );

        println!("x_cg_rot(x_main=3.50)={x_rot_frente:.4}  x_cg_rot(x_main=4.20)={x_rot_atras:.4}");
        assert!(
            x_rot_frente < x_rot_atras,
            "limite de rotação com trem mais à frente ({x_rot_frente:.4}) deveria ser MENOR \
             que com trem mais atrás ({x_rot_atras:.4})"
        );
    }

    // ─── Integração via pipeline real (baseline_4seat.toml) ─────────────────

    /// Roda o `TrimAuthorityAgent` sobre o pipeline completo do baseline
    /// real e confirma: (1) o limite de flare CORRIGIDO bate; (2) o limite
    /// de rotação (número ÚNICO) bate; (3) a ROTAÇÃO GOVERNA e fica À
    /// FRENTE do limite traseiro — envelope de CG VAZIO (achado honesto,
    /// ver docstring do módulo); (4) a margem de autoridade por cenário é
    /// negativa em TODOS os cenários reais (nenhum tem autoridade
    /// suficiente na sua própria CG).
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
            (trim.flare_limit_pct_mac - 7.908).abs() < 1.0,
            "flare_limit_pct_mac = {:.3} (esperado ≈7.908% ±1%)",
            trim.flare_limit_pct_mac
        );

        println!("rotation_limit_pct_mac = {:.3}", trim.rotation_limit_pct_mac);
        assert!(
            (trim.rotation_limit_pct_mac - 39.93).abs() < 1.5,
            "rotation_limit_pct_mac = {:.3} (esperado ≈39.93% ±1.5%)",
            trim.rotation_limit_pct_mac
        );

        // Achado honesto: a rotação governa E fica à frente do limite
        // traseiro (envelope vazio) — ver docstring do módulo.
        assert!(trim.rotation_limit_pct_mac > trim.flare_limit_pct_mac);
        assert_eq!(trim.governing, "rotacao",
            "governing deveria ser 'rotacao' (achado honesto do baseline real)");
        assert!(trim.rotation_limit_pct_mac > wb.spec.cg_limit_aft_pct_mac,
            "limite de rotação ({:.2}%) deveria ficar À FRENTE do limite traseiro ({:.2}%) — \
             envelope de CG vazio no baseline real",
            trim.rotation_limit_pct_mac, wb.spec.cg_limit_aft_pct_mac);

        for sc in &trim.rotation_margin_per_scenario {
            assert!(sc.rotation_authority_margin_pct < 0.0,
                "cenário '{}': margem de autoridade de rotação deveria ser NEGATIVA no \
                 baseline real (nenhum cenário tem CG atrás o bastante) — obtido {:.2}%",
                sc.scenario, sc.rotation_authority_margin_pct);
        }
    }

    /// Sensibilidade: o limite de flare recomputado a `cl_h_max_down ±
    /// 0.05` deve mover na direção esperada — mais autoridade (plus) →
    /// limite mais à frente (menor); menos autoridade (minus) → limite
    /// mais atrás (maior).
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
