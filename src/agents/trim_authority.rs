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
///   - **Rotação na decolagem** (Vr = 1,1·Vs0_TO(W), flap de DECOLAGEM):
///     balanço de momentos em torno do TREM PRINCIPAL (é o trem, não o CG,
///     que faz de pivô na rotação) — solução fechada. Ciclo 7 (task 1):
///     `Vs0_TO` usa `WingSpec::cl_max_to` (CLmax do flap PARCIAL de
///     decolagem), NÃO o `cl_max_flaps` de pouso — coerente com o `Cm_TO`
///     de flap parcial que este mesmo balanço já usava. A flare continua
///     com o CLmax de POUSO (`wing.cl_max`), que é a configuração certa
///     para ela.
///     (`rotation_fwd_limit_m`). **DEIXOU de ser invariante ao peso no
///     ciclo 10 (task 2)** — o momento da LINHA DE TRAÇÃO (`T(Vr)·z_eixo`,
///     nariz-abaixo) entrou no balanço e `T(Vr(W))` NÃO é proporcional a
///     `W`; ver a docstring de `rotation_fwd_limit_m` para a re-derivação
///     completa (o que ainda cancela, o que não cancela mais, e a variação
///     MEDIDA na faixa de pesos dos cenários).
///
/// O limite dianteiro efetivo é `max(flare, rotação)` — o mais restritivo
/// das duas. A flare é o MESMO para todos os cenários; a rotação é
/// avaliada no cenário MAIS LEVE (o mais restritivo desde o ciclo 10 —
/// ver `TrimAuthorityAgent::run`), e esse número único é aplicado a todos
/// os cenários (conservador e consistente). A margem de
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
    aircraft_state::AircraftState,
    atmosphere::Isa,
    engine::EngineSpec,
    requirements::Requirements,
    specs::{EmpennageSpec, ScenarioTrimLimit, TrimSensitivity, TrimSpec, WingSpec},
};

const G: f64 = 9.807; // m/s²

/// Variação de `cl_h_max_down` usada no recálculo de sensibilidade
/// (`TrimSensitivity`) — ±0,05, conforme o brief da task.
const SENSITIVITY_DELTA: f64 = 0.05;

/// `Vr/Vs0_TO` — Vr = 1,1·Vs0_TO, usado tanto para a dinâmica da rotação
/// quanto (elevado ao quadrado) para o cancelamento algébrico de peso em
/// `rotation_fwd_limit_m`/`rotation_available_moment_nm` — ver a docstring
/// dessas funções. `Vs0_TO` é a velocidade de estol na configuração de
/// DECOLAGEM (`WingSpec::cl_max_to`, flap parcial — ciclo 7, task 1), não a
/// de pouso.
const VR_OVER_VS0: f64 = 1.1;

/// Variação de `[control_surfaces].elevator_deflection_max_deg` usada no
/// recálculo de sensibilidade (segunda dimensão de `TrimSensitivity`, task
/// refino-ciclo2) — ±2°.
const DEFLECTION_SENSITIVITY_DELTA_DEG: f64 = 2.0;

// ─── AUTORIDADE CALCULADA POR GEOMETRIA (DATCOM/Nelson) ───────────────────
//
// Task refino-ciclo2 (1a): remove o parâmetro livre `[stability].
// cl_h_max_down` (palpite semi-empírico sem base geométrica) e o substitui
// por um cálculo DATCOM/Nelson a partir da geometria já dimensionada do
// profundor (`[control_surfaces].elevator_chord_frac`, a razão c_e/c entre
// a corda do profundor e a corda local do estabilizador horizontal — ver
// `agents::control_surfaces::wing_surface_per_side`/`tail_surface_mirrored`,
// que multiplicam `chord_frac` pela corda local: a razão é constante ao
// longo da envergadura, então `elevator_chord_frac` JÁ É c_e/c, sem
// precisar rodar `ControlSurfacesAgent`) e do alongamento da empenagem
// horizontal (`EmpennageSpec::ar_h`, via `weight_balance::lift_curve_slope`).

/// Eficácia de superfície do profundor τ(c_e/c) — ajuste empírico de Nelson
/// (Nelson, R. "Flight Stability and Automatic Control", fig. 2.21):
///
///   τ = 1,24·√(c_e/c) − 0,16
///
/// Válido no intervalo `c_e/c ∈ [0,1; 0,6]` (faixa coberta pelo ajuste
/// original de Nelson — fora dela a curva não tem base experimental).
/// `c_e/c` é a razão entre a corda do profundor e a corda LOCAL do
/// estabilizador horizontal (constante ao longo da envergadura neste
/// modelo — ver `models::aircraft_config::ControlSurfacesCfg::
/// elevator_chord_frac`).
pub fn tau_elevator(c_e_over_c: f64) -> f64 {
    1.24 * c_e_over_c.sqrt() - 0.16
}

/// `cl_h_max_down` calculado por geometria (DATCOM/Nelson) — substitui o
/// antigo parâmetro livre `[stability].cl_h_max_down`:
///
///   cl_h_max_down_calc = a_t · τ · δe_max_rad
///
/// TRUNCADO no teto de stall da empenagem (`cl_h_stall_limit`,
/// `[stability].cl_h_stall_limit`) — download de profundor acima do CL_max
/// da própria empenagem não é fisicamente alcançável (a superfície
/// estola antes). Retorna `(valor_usado, limitado_pelo_teto)`: o primeiro
/// elemento é `min(a_t·τ·δe_max_rad, cl_h_stall_limit)` (o valor
/// OPERACIONAL, usado no restante do balanço de momentos); o segundo indica
/// se o teto de stall foi o fator limitante. `a_t` é `weight_balance::
/// lift_curve_slope(emp.ar_h)`; `δe_max_rad` é `[control_surfaces].
/// elevator_deflection_max_deg` convertido para radianos.
pub fn cl_h_max_down_calc(
    a_t: f64,
    tau: f64,
    delta_e_max_rad: f64,
    cl_h_stall_limit: f64,
) -> (f64, bool) {
    let raw = a_t * tau * delta_e_max_rad;
    if raw > cl_h_stall_limit {
        (cl_h_stall_limit, true)
    } else {
        (raw, false)
    }
}

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

/// Velocidade de rotação `Vr = 1,1·Vs0_TO(W)` (m/s) — explicitada como
/// função própria no ciclo 10 (task 2) porque a TRAÇÃO na rotação depende
/// de `Vr` de verdade (não só de `q_r`, onde `ρ` cancela algebricamente —
/// ver `rotation_available_moment_nm`). `rho` é a densidade no ponto de
/// decolagem (nível do mar, ΔISA da missão).
///
///   Vs0_TO(W) = √(2W/(ρ·S_w·CL_max_TO))  ⟹  Vr = 1,1·Vs0_TO(W)
pub fn rotation_speed_ms(weight_n: f64, rho: f64, s_w_m2: f64, cl_max_to: f64) -> f64 {
    VR_OVER_VS0 * (2.0 * weight_n / (rho * s_w_m2 * cl_max_to)).sqrt()
}

/// Tração disponível NA ROTAÇÃO (N) — `performance::thrust_available_n`
/// avaliada em `Vr(W)` (ver `rotation_speed_ms`), ao nível do mar
/// (decolagem) e no rpm máximo CONTÍNUO do motor.
///
/// Ciclo 10 (task 2): é esta a tração que entra no termo `−T·z_eixo` do
/// balanço de rotação (ver `rotation_available_moment_nm`). Como
/// `Vr ∝ √W`, `T` VARIA com o peso do cenário — é a origem física da morte
/// da invariância a `W` do limite de rotação (ver `rotation_fwd_limit_m`).
pub fn thrust_at_rotation_n(
    weight_n: f64,
    s_w_m2: f64,
    cl_max_to: f64,
    engine: &EngineSpec,
    state: &AircraftState,
    isa_delta_c: f64,
    static_thrust_factor: f64,
) -> f64 {
    let rho_to = Isa::density_kgm3(0.0, isa_delta_c);
    let vr = rotation_speed_ms(weight_n, rho_to, s_w_m2, cl_max_to);
    crate::agents::performance::thrust_available_n(
        vr,
        engine,
        engine.rpm_max_continuous,
        state.psru_ratio,
        state.prop_diameter_m,
        0.0,
        isa_delta_c,
        static_thrust_factor,
        state.psru_efficiency,
    )
}

/// Momento NARIZ-ACIMA disponível na rotação de decolagem (N·m), em torno
/// do TREM PRINCIPAL, para um peso `weight_n` — soma de QUATRO fontes
/// (TRÊS até o ciclo 9; a linha de TRAÇÃO entrou no ciclo 10, task 2),
/// TODAS com o sinal físico correto (fix de revisão — a versão anterior
/// usava `|Cm_TO|`, perdendo o sinal; ver nota abaixo):
///
///   q_r(W) = 0,5·ρ·Vr² ,  Vr = 1,1·Vs0_TO(W) ,  Vs0_TO(W) = √(2W/(ρ·S_w·CL_max_TO))
///          ⟹ q_r(W) = 1,21·W / (S_w·CL_max_TO)         [PROPORCIONAL a W]
///
///   F_h = q_r·S_h·η_h·cl_h_max_down·(1−trim_margin)    [download da EH, N — nariz-ACIMA]
///   L_g = q_r·S_w·cl_ground_rotation                    [sustentação da asa, N — nariz-ACIMA]
///   Cm_TO = cm_ac + to_flap_fraction·cm_flap_delta       [perfil+flap TO, SINALIZADO]
///   M_T = T(Vr)·z_eixo                                   [linha de tração, N·m — nariz-ABAIXO]
///
///   M_disponível = F_h·(x_ac_tail−x_main) + L_g·(x_main−x_ac_wing)
///                  + Cm_TO·q_r·S_w·MAC − T(Vr)·z_eixo
///
/// ─── LINHA DE TRAÇÃO (ciclo 10, task 2) ─────────────────────────────────
/// `thrust_rot_n` é a tração disponível a `Vr` (ver `thrust_at_rotation_n`)
/// e `z_axis_m` é a ALTURA DO EIXO DA HÉLICE SOBRE O SOLO
/// (`gear.h_cg_ground_m + propeller.prop_axis_above_cg_m`) — na rotação o
/// pivô é o ponto de contato do TREM PRINCIPAL com o solo, então o braço
/// do vetor de tração (horizontal, para a FRENTE) em torno desse pivô é a
/// altura do eixo sobre o SOLO, não sobre o CG. (Em cruzeiro o pivô é o
/// CG e o braço é `prop_axis_above_cg_m` — ver `cm_thrust_cruise`.)
///
/// SINAL AUDITADO: uma força para a FRENTE aplicada ACIMA do pivô produz
/// um binário que empurra o NARIZ PARA BAIXO (a mesma física que faz o
/// nariz mergulhar numa frenagem forte). Nariz-abaixo ATRAPALHA a rotação,
/// logo o termo é SUBTRAÍDO do momento nariz-acima disponível — daí o
/// `− thrust_rot_n * z_axis_m` no corpo da função. Consequências
/// falseáveis, cobertas por properties ESTRITAS nos testes: eixo mais alto
/// (`z_axis_m` maior) ⟹ MENOS momento disponível ⟹ limite dianteiro de
/// rotação RECUA (`x_cg_rot` MAIOR); tração maior ⟹ idem. Passar
/// `thrust_rot_n = 0,0` reproduz EXATAMENTE o modelo pré-ciclo-10.
///
/// `CL_max_TO` (ciclo 7, task 1) é o CLmax do flap PARCIAL de DECOLAGEM
/// (`WingSpec::cl_max_to` = `cl_max_clean + to_flap_fraction·(cl_max_flaps
/// − cl_max_clean)`) — a MESMA `to_flap_fraction` que sinaliza o `Cm_TO`
/// logo abaixo. Antes deste ciclo, `Vs0` vinha do `cl_max_flaps` de POUSO
/// enquanto o `Cm` já era o do flap parcial: as duas metades do mesmo
/// balanço descreviam configurações de flap DIFERENTES. A incoerência era
/// inofensiva com flap simples (1,72 vs 1,585), mas com flap slotted
/// (CLmax 2,2) subestimava Vr em 13% (q_r −24%) e inflava artificialmente
/// o limite dianteiro de rotação (campanha E10).
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
    cl_max_to: f64,
    s_h_m2: f64,
    eta_h: f64,
    cl_h_max_down: f64,
    trim_margin: f64,
    x_ac_tail_m: f64,
    x_main_m: f64,
    cl_ground_rotation: f64,
    x_ac_wing_m: f64,
    cm_ac: f64,
    to_flap_fraction: f64,
    cm_flap_delta: f64,
    mac_m: f64,
    thrust_rot_n: f64,
    z_axis_m: f64,
) -> f64 {
    let q_r = VR_OVER_VS0 * VR_OVER_VS0 * weight_n / (s_w_m2 * cl_max_to);
    let f_h = q_r * s_h_m2 * eta_h * cl_h_max_down * (1.0 - trim_margin);
    let l_g = q_r * s_w_m2 * cl_ground_rotation;
    let cm_to = cm_ac + to_flap_fraction * cm_flap_delta;
    f_h * (x_ac_tail_m - x_main_m)
        + l_g * (x_main_m - x_ac_wing_m)
        + cm_to * q_r * s_w_m2 * mac_m
        - thrust_rot_n * z_axis_m
}

/// Limite dianteiro de rotação (m do datum, NÃO %MAC) — balanço de
/// momentos em torno do TREM PRINCIPAL na rotação de decolagem. Fechado
/// (solução direta, sem bisseção):
///
///   x_cg_rot(W) = x_main − M_disponível(W) / W
///
/// ─── A INVARIÂNCIA AO PESO MORREU (ciclo 10, task 2) ────────────────────
///
/// Até o ciclo 9 esta função NÃO recebia peso: provava-se que `W` cancelava
/// exatamente. A prova antiga (mantida abaixo porque continua VÁLIDA para a
/// parte aerodinâmica) era: `q_r(W) = 1,21·W/(S_w·CL_max_TO)` é
/// PROPORCIONAL a `W` (ver `rotation_available_moment_nm`) — logo `F_h`,
/// `L_g` e o termo de `Cm_TO` são TODOS proporcionais a `W`, o momento
/// disponível também é, e ao dividir por `W` o peso CANCELA. Fisicamente:
/// sob a política `Vr = 1,1·Vs0_TO(W)`, uma aeronave mais pesada rotaciona
/// a uma `Vr` proporcionalmente maior (`Vs0_TO ∝ √W`), o que aumenta `q_r`
/// na medida EXATA (`q_r ∝ W`) para que a autoridade de profundor cresça na
/// mesma proporção que o momento de peso a vencer.
///
/// O termo NOVO da LINHA DE TRAÇÃO **não** segue essa proporcionalidade:
///
///   M_disponível(W) = W·k_aero − T(Vr(W))·z_eixo
///   ⟹ x_cg_rot(W) = x_main − k_aero + T(Vr(W))·z_eixo / W
///
/// onde `k_aero = M_aero(W)/W` É a constante invariante da prova antiga.
/// `T` é tração de HÉLICE a velocidade `Vr`, ≈ `η(J)·P_eixo/Vr` — não tem
/// nenhuma razão física para escalar com `W`. Com `Vr ∝ √W` e `P_eixo`
/// fixo, `T ∝ η(J)/√W` e portanto `T/W ∝ η(J)·W^(−3/2)`: o termo de tração
/// por unidade de peso CAI com o peso. Consequência falseável e
/// contra-intuitiva à primeira vista, mas correta:
///
///   **aeronave mais LEVE ⟹ limite dianteiro de rotação mais RECUADO**
///
/// (rotaciona devagar, onde a tração é alta em relação ao peso, então o
/// binário nariz-abaixo da linha de tração pesa proporcionalmente MAIS).
/// Por isso `TrimAuthorityAgent::run` avalia o limite ÚNICO reportado no
/// cenário MAIS LEVE — o mais restritivo. Properties estritas:
/// `tests::limite_de_rotacao_recua_com_peso_menor` (direção) e
/// `tests::rotation_limit_variacao_medida_na_faixa_de_pesos_dos_cenarios`
/// (magnitude MEDIDA no baseline real, ~7 pp de MAC entre os extremos de
/// peso dos cenários — MATERIAL, não desprezível). A prova de que a parte
/// AERODINÂMICA continua cancelando exatamente sobrevive em
/// `tests::rotation_limit_e_invariante_a_massas_diferentes_sem_tracao`.
///
/// A troca de `cl_max_flaps` por `cl_max_to` (ciclo 7, task 1) NÃO afeta a
/// parte aerodinâmica desta derivação: `cl_max_to` é, como `cl_max_flaps`,
/// uma constante da CONFIGURAÇÃO, não uma função de `W`. O que muda é o
/// VALOR: `cl_max_to < cl_max_flaps` ⟹ `Vs0_TO`/`Vr` maiores ⟹ `q_r`
/// maior ⟹ mais autoridade disponível ⟹ limite dianteiro mais À FRENTE do
/// que o modelo antigo indicava. (Com a linha de tração ligada, `Vr` maior
/// também significa `T(Vr)` MENOR — os dois efeitos vão na mesma direção
/// aqui.)
///
/// A margem de autoridade REAL de cada cenário (que usa a CG/peso
/// VERDADEIROS, não o limite, e desde este ciclo também a `T(Vr(W))` do
/// próprio cenário) fica em `TrimAuthorityAgent::run`/
/// `rotation_available_moment_nm` diretamente — ver
/// `models::specs::ScenarioTrimLimit`.
#[allow(clippy::too_many_arguments)]
pub fn rotation_fwd_limit_m(
    weight_n: f64,
    s_w_m2: f64,
    cl_max_to: f64,
    s_h_m2: f64,
    eta_h: f64,
    cl_h_max_down: f64,
    trim_margin: f64,
    x_ac_tail_m: f64,
    x_main_m: f64,
    cl_ground_rotation: f64,
    x_ac_wing_m: f64,
    cm_ac: f64,
    to_flap_fraction: f64,
    cm_flap_delta: f64,
    mac_m: f64,
    thrust_rot_n: f64,
    z_axis_m: f64,
) -> f64 {
    let moment_nm = rotation_available_moment_nm(
        weight_n, s_w_m2, cl_max_to, s_h_m2, eta_h, cl_h_max_down, trim_margin, x_ac_tail_m,
        x_main_m, cl_ground_rotation, x_ac_wing_m, cm_ac, to_flap_fraction, cm_flap_delta,
        mac_m, thrust_rot_n, z_axis_m,
    );
    x_main_m - moment_nm / weight_n
}

// ─── ARRASTO DE TRIM EM CRUZEIRO (Task 4, refino-ciclo2) ─────────────────────
//
// Em cruzeiro (sem flap), a empenagem horizontal precisa gerar uma força
// (para cima OU para baixo, `CL_h_trim`) para equilibrar o momento de
// arfagem em torno do CG de referência da missão — diferente da flare/
// rotação (que usam o CL_max com flap no batente de profundor), aqui é o
// balanço de TRIM em voo nivelado 1g, sem flap: `cm_ac` isolado (SEM
// `cm_flap_delta`), no CG de referência da missão (não no CG do cenário mais
// crítico) e no `CL_cruise` do polar.
//
// Cenário de referência escolhido: **"4 pax + bagagem + meia"** (meia-
// missão, tanque pela metade) — representa o CG médio ao longo da missão de
// cruzeiro (nem o CG mais dianteiro do início do voo, tanque cheio, nem o
// mais traseiro do fim, tanque quase vazio — ver `agents::weight_balance::
// WeightBalanceAgent::run`, `scenarios_def`), consistente com a prática de
// projeto preliminar de reportar o arrasto de trim "típico" de cruzeiro, não
// um extremo. Documentado aqui e ecoado em `TrimSpec::cg_reference_scenario`
// para rastreabilidade.
//
// Balanço de momentos — Σ M_cg = 0, com `CL_cruise` (do polar da asa) usado
// DIRETO como o CL da asa isolada (SEM o fechamento vertical
// `CL_w = CL_total − η_h·(S_h/S_w)·CL_h` que `cl_h_required_flare` aplica —
// ver a nota de aproximação logo abaixo, "Aproximação documentada", para a
// justificativa de desprezar esse termo de 2ª ordem aqui). Desde o ciclo 10
// (task 2) o `cm_thrust` da LINHA DE TRAÇÃO entra somado ao `cm_ac` (ver
// `cm_thrust_cruise`):
//
//   0 = cm_ac + cm_thrust + CL_cruise·(x̄_cg − 0,25)
//       − η_h·(S_h/S_w)·CL_h_trim·(l_h/MAC + 0,25 − x̄_cg)
//   ⟹ CL_h_trim = [cm_ac + cm_thrust + CL_cruise·(x̄_cg − 0,25)]
//                 / [η_h·(S_h/S_w)·(l_h/MAC + 0,25 − x̄_cg)]
//
// Diferença chave em relação a `cl_h_required_flare`: LÁ o fechamento
// vertical faz o termo `(l_h/MAC+0,25−x̄)` do denominador se CANCELAR com o
// `(x̄−0,25)` que aparece ao expandir `CL_w·(x̄−0,25)`, sobrando só `l_h/MAC`
// (constante, sem pólo). AQUI, sem esse fechamento, o termo
// `(l_h/MAC+0,25−x̄)` COMPLETO permanece no denominador — a fórmula acima
// não tem essa simplificação algébrica (mantém `x̄` também no denominador,
// não só no numerador).
//
// Sinal: CG ATRÁS do CA da asa (x̄_cg > 0,25, caso típico deste baseline)
// produz um momento de peso PICANDO NARIZ-PARA-CIMA (nose-up, cauda-pesada)
// que precisa ser equilibrado por uma força para CIMA na cauda
// (`CL_h_trim > 0`, "upload") — contra-intuitivo à primeira vista (a
// empenagem SUSTENTA, não gera download), mas correto: CG atrás do CA
// desestabiliza no sentido nariz-para-cima, a cauda precisa empurrar para
// CIMA para conter esse momento. Ver hand-check no teste
// `cl_h_trim_cruise_hand_check_baseline_meia_missao` para o caso numérico.
//
// Aproximação documentada (brief): a contribuição de sustentação EXTRA que
// a asa precisaria gerar para compensar o upload/download da cauda (efeito
// de 2ª ordem sobre `CL_cruise`, análogo ao fechamento vertical de
// `cl_h_required_flare`) é DESPREZADA aqui — `CL_cruise` já vem fechado do
// polar da asa (`AerodynamicsAgent`, `W = q·S·CL_cruise`), sem realimentação
// do `CL_h_trim`. Justificativa: `CL_h_trim` em cruzeiro é tipicamente
// pequeno (ver hand-check, ~0,04), então `η_h·(S_h/S_w)·CL_h_trim` é uma
// fração pequena de `CL_cruise` — o erro de 2ª ordem introduzido é
// desprezível frente às demais incertezas do modelo (Cm_ac semi-empírico,
// eficiência de Oswald da cauda).

/// Contribuição da LINHA DE TRAÇÃO ao `Cm` de equilíbrio em cruzeiro
/// (adimensional, em torno do CG — ciclo 10, task 2):
///
///   cm_thrust = − T_cruzeiro · prop_axis_above_cg_m / (q·S_w·MAC)
///
/// BRAÇO: em voo o pivô é o CG, então o braço é o offset eixo↔CG
/// (`[propeller].prop_axis_above_cg_m`) — NÃO a altura do eixo sobre o solo
/// usada na rotação (`rotation_available_moment_nm`, onde o pivô é o trem).
/// É o mesmo campo de config nos dois lugares, com braços diferentes por
/// causa do pivô diferente.
///
/// SINAL AUDITADO: eixo ACIMA do CG (`prop_axis_above_cg_m > 0`) + tração
/// para a FRENTE ⟹ binário NARIZ-ABAIXO ⟹ `Cm` NEGATIVO (convenção
/// aerodinâmica: `Cm > 0` é nariz-acima) — daí o sinal `−` explícito na
/// fórmula. Efeito falseável em `cl_h_trim_cruise`: o `cm_thrust` negativo
/// entra no NUMERADOR, e como o termo da empenagem entra no balanço com
/// sinal `−η_h·(S_h/S_w)·CL_h·(...)` (ou seja, `CL_h > 0`/upload produz
/// momento nariz-ABAIXO), conter um momento nariz-abaixo extra exige mover
/// `CL_h_trim` na direção NEGATIVA (mais download / menos upload). Ver
/// `tests::cm_thrust_negativo_reduz_cl_h_trim_cruise`.
///
/// `thrust_n` é `PropulsionSpec::thrust_cruise_n` (que, em regime
/// permanente, iguala o arrasto de cruzeiro por construção); `q_pa` é a
/// pressão dinâmica de cruzeiro.
pub fn cm_thrust_cruise(
    thrust_n: f64,
    prop_axis_above_cg_m: f64,
    q_pa: f64,
    s_w_m2: f64,
    mac_m: f64,
) -> f64 {
    -thrust_n * prop_axis_above_cg_m / (q_pa * s_w_m2 * mac_m)
}

pub fn cl_h_trim_cruise(
    cm_ac: f64,
    cl_cruise: f64,
    x_bar_cg: f64,
    eta_h: f64,
    s_h_over_s_w: f64,
    l_h_over_mac: f64,
    cm_thrust: f64,
) -> f64 {
    let num = cm_ac + cm_thrust + cl_cruise * (x_bar_cg - 0.25);
    let den = eta_h * s_h_over_s_w * (l_h_over_mac + 0.25 - x_bar_cg);
    num / den
}

/// Arrasto INDUZIDO de trim (adimensional, somado ao `CD_cruise` do polar
/// da asa) — a empenagem horizontal, ao gerar `CL_h_trim` para equilibrar o
/// momento de arfagem, produz seu PRÓPRIO arrasto induzido, na mesma forma
/// de `agents::aerodynamics::cd_induced` (modelo elíptico generalizado),
/// mas com o alongamento/Oswald DA EMPENAGEM (`ar_h`/`e_h`, não da asa) e
/// referenciado à área da ASA (`×S_h/S_w`, para somar diretamente ao
/// `CD_cruise` da aeronave, que já é referenciado a `S_w`):
///
///   ΔCD_trim = (CL_h_trim² / (π·ar_h·e_h)) · (S_h/S_w)
///
/// `e_h` é `[empennage].e_h` (task refino-ciclo2, Task 4) — eficiência de
/// Oswald da empenagem horizontal, parâmetro semi-empírico distinto de
/// `agents::aerodynamics::oswald_efficiency` (que é calculado por AR para a
/// ASA; a EH não tem esse cálculo dedicado neste modelo).
pub fn cd_trim_cruise(cl_h_trim: f64, ar_h: f64, e_h: f64, s_h_over_s_w: f64) -> f64 {
    crate::agents::aerodynamics::cd_induced(cl_h_trim, ar_h, e_h) * s_h_over_s_w
}

// ─── AGENTE PRINCIPAL ────────────────────────────────────────────────────────

pub struct TrimAuthorityAgent;

impl TrimAuthorityAgent {
    /// Calcula o `TrimSpec` completo — autoridade de profundor por
    /// geometria DATCOM/Nelson (`cl_h_max_down_calc`/`tau_elevator`/
    /// `capped_by_stall`, task refino-ciclo2 1a) + limite de flare + limite
    /// de rotação (ambos números ÚNICOS, ver docstring de `TrimSpec`) + a
    /// margem de autoridade de rotação por cenário (`wb.scenarios`, saída
    /// já calculada do `WeightBalanceAgent`) + a sensibilidade (±0,05 em
    /// `cl_h_max_down` E ±2° em `elevator_deflection_max_deg`) + os
    /// parâmetros ecoados. NÃO modifica `wb` — ver `WeightBalanceOutput::
    /// apply_trim` para a etapa que consome este resultado e finaliza
    /// `inside_envelope`/`cg_limit_fwd_pct_mac`.
    ///
    /// Ciclo 10 (task 2) — parâmetros NOVOS: `state`/`engine`/`req` são
    /// necessários para avaliar `performance::thrust_available_n` em `Vr`
    /// (PSRU/diâmetro de hélice/rpm/ΔISA) no balanço de rotação, e
    /// `thrust_cruise_n` (`PropulsionSpec::thrust_cruise_n`) para o
    /// `cm_thrust` do trim de cruzeiro. Ver `thrust_at_rotation_n` e
    /// `cm_thrust_cruise`.
    #[allow(clippy::too_many_arguments)]
    pub fn run(
        cfg: &AircraftConfig,
        wing: &WingSpec,
        emp: &EmpennageSpec,
        wb: &WeightBalanceOutput,
        state: &AircraftState,
        engine: &EngineSpec,
        req: &Requirements,
        thrust_cruise_n: f64,
    ) -> TrimSpec {
        let mac = wb.mac_m;
        let mac_le = wb.mac_le_x_m;
        let l_h_over_mac = emp.arm_h_m / mac;
        let s_ratio = emp.s_horizontal_m2 / wing.area_m2;
        let cm_ac_total = cfg.wing.cm_ac + cfg.wing.cm_flap_delta;
        let clf = cl_flare(wing.cl_max);

        // Autoridade calculada por geometria (task refino-ciclo2, 1a) —
        // substitui o antigo parâmetro livre `[stability].cl_h_max_down`.
        let a_t = crate::agents::weight_balance::lift_curve_slope(emp.ar_h);
        let tau = tau_elevator(cfg.control_surfaces.elevator_chord_frac);
        let delta_e_max_rad = cfg.control_surfaces.elevator_deflection_max_deg.to_radians();
        let (cl_h_max_down, capped_by_stall) =
            cl_h_max_down_calc(a_t, tau, delta_e_max_rad, cfg.stability.cl_h_stall_limit);
        let cl_h_max_down_calc_raw = a_t * tau * delta_e_max_rad;

        let cl_avail = cl_h_available(cl_h_max_down, cfg.stability.trim_margin);

        let x_flare =
            flare_fwd_limit_frac(cm_ac_total, clf, emp.eta_h, s_ratio, l_h_over_mac, cl_avail);
        let flare_limit_pct = x_flare * 100.0;

        // Rotação. Geometria: x_ac_wing/x_ac_tail medidos do datum (bordo
        // de ataque da asa + 0,25·MAC [+ braço da EH]).
        let x_ac_wing = cfg.wing.le_root_x_m + 0.25 * mac;
        let x_ac_tail = cfg.wing.le_root_x_m + 0.25 * mac + emp.arm_h_m;

        // Braço da LINHA DE TRAÇÃO na rotação (ciclo 10, task 2): altura do
        // eixo da hélice sobre o SOLO — o pivô da rotação é o contato do
        // trem principal com o solo, não o CG. Ver
        // `rotation_available_moment_nm`.
        let z_axis = cfg.gear.h_cg_ground_m + cfg.propeller.prop_axis_above_cg_m;

        // Peso de referência do limite ÚNICO de rotação (ciclo 10, task 2):
        // o cenário MAIS LEVE. Até o ciclo 9 o limite era INVARIANTE ao
        // peso e essa escolha não existia; com a linha de tração no
        // balanço, `x_cg_rot(W) = x_main − k_aero + T(Vr(W))·z/W` CRESCE
        // (recua) quando `W` cai — ver a re-derivação em
        // `rotation_fwd_limit_m` e a property estrita
        // `tests::limite_de_rotacao_recua_com_peso_menor`. Avaliar no mais
        // leve é portanto o CONSERVADOR e, mais importante, o CONSISTENTE
        // com `rotation_margin_per_scenario` abaixo: como `x_cg_rot` é
        // decrescente em `W`, qualquer cenário com margem NEGATIVA (isto é,
        // `x_cg < x_cg_rot(W_do_cenário)`) fica necessariamente à frente
        // deste limite também, e será marcado fora do envelope por
        // `apply_trim`. O recíproco não vale (um cenário pesado pode ser
        // marcado fora do envelope tendo margem própria positiva) — é
        // conservadorismo assumido, documentado aqui e em `TrimSpec`.
        let mass_light_kg = wb.scenarios.iter()
            .map(|sc| sc.total_mass_kg)
            .fold(f64::INFINITY, f64::min);
        let w_ref_n = mass_light_kg * G;
        let thrust_rot_ref_n = thrust_at_rotation_n(
            w_ref_n, wing.area_m2, wing.cl_max_to, engine, state, req.isa_delta_c,
            cfg.performance.static_thrust_factor,
        );

        // Ciclo 7 (task 1): `wing.cl_max_to` (flap PARCIAL de decolagem),
        // não `wing.cl_max` (flap de POUSO) — a Vr da rotação agora é
        // coerente com o `Cm_TO` de flap parcial usado no mesmo balanço.
        let x_rot = rotation_fwd_limit_m(
            w_ref_n, wing.area_m2, wing.cl_max_to, emp.s_horizontal_m2, emp.eta_h,
            cl_h_max_down, cfg.stability.trim_margin, x_ac_tail, cfg.gear.x_main_m,
            cfg.stability.cl_ground_rotation, x_ac_wing, cfg.wing.cm_ac,
            cfg.stability.to_flap_fraction, cfg.wing.cm_flap_delta, mac,
            thrust_rot_ref_n, z_axis,
        );
        let rotation_limit_pct = cg_pct_mac(x_rot, mac_le, mac);

        // Margem de autoridade de rotação por cenário — diagnóstico
        // informativo na CG/peso REAIS de cada cenário (varia por
        // cenário, ao contrário do limite acima) — ver `ScenarioTrimLimit`.
        let mut rotation_margin_per_scenario = Vec::with_capacity(wb.scenarios.len());
        for sc in &wb.scenarios {
            let w_n = sc.total_mass_kg * G;
            // Ciclo 10 (task 2): a tração de rotação é avaliada na `Vr` do
            // PESO DESTE cenário (`Vr ∝ √W`), não na do cenário de
            // referência do limite único acima.
            let thrust_rot_n = thrust_at_rotation_n(
                w_n, wing.area_m2, wing.cl_max_to, engine, state, req.isa_delta_c,
                cfg.performance.static_thrust_factor,
            );
            let available = rotation_available_moment_nm(
                w_n, wing.area_m2, wing.cl_max_to, emp.s_horizontal_m2, emp.eta_h,
                cl_h_max_down, cfg.stability.trim_margin, x_ac_tail,
                cfg.gear.x_main_m, cfg.stability.cl_ground_rotation, x_ac_wing, cfg.wing.cm_ac,
                cfg.stability.to_flap_fraction, cfg.wing.cm_flap_delta, mac,
                thrust_rot_n, z_axis,
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

        // Sensibilidade — DUAS dimensões (task refino-ciclo2, 1a):
        //   (1) ±0,05 direto em `cl_h_max_down` (mesma resolução de antes,
        //       perturbação direta do valor OPERACIONAL/capado, sem
        //       recalcular τ/δe — captura a incerteza residual do próprio
        //       ajuste semi-empírico de Nelson);
        //   (2) ±2° em `elevator_deflection_max_deg`, recalculando
        //       `cl_h_max_down_calc` (τ/a_t fixos) e o limite de flare —
        //       captura a incerteza do batente mecânico do profundor.
        let cl_minus = cl_h_max_down - SENSITIVITY_DELTA;
        let cl_plus = cl_h_max_down + SENSITIVITY_DELTA;
        let x_flare_minus = flare_fwd_limit_frac(
            cm_ac_total, clf, emp.eta_h, s_ratio, l_h_over_mac,
            cl_h_available(cl_minus, cfg.stability.trim_margin),
        );
        let x_flare_plus = flare_fwd_limit_frac(
            cm_ac_total, clf, emp.eta_h, s_ratio, l_h_over_mac,
            cl_h_available(cl_plus, cfg.stability.trim_margin),
        );

        let deflection_minus_deg =
            cfg.control_surfaces.elevator_deflection_max_deg - DEFLECTION_SENSITIVITY_DELTA_DEG;
        let deflection_plus_deg =
            cfg.control_surfaces.elevator_deflection_max_deg + DEFLECTION_SENSITIVITY_DELTA_DEG;
        let (cl_deflection_minus, _) = cl_h_max_down_calc(
            a_t, tau, deflection_minus_deg.to_radians(), cfg.stability.cl_h_stall_limit,
        );
        let (cl_deflection_plus, _) = cl_h_max_down_calc(
            a_t, tau, deflection_plus_deg.to_radians(), cfg.stability.cl_h_stall_limit,
        );
        let x_flare_deflection_minus = flare_fwd_limit_frac(
            cm_ac_total, clf, emp.eta_h, s_ratio, l_h_over_mac,
            cl_h_available(cl_deflection_minus, cfg.stability.trim_margin),
        );
        let x_flare_deflection_plus = flare_fwd_limit_frac(
            cm_ac_total, clf, emp.eta_h, s_ratio, l_h_over_mac,
            cl_h_available(cl_deflection_plus, cfg.stability.trim_margin),
        );

        // Arrasto de trim em cruzeiro (Task 4, refino-ciclo2) — CG de
        // REFERÊNCIA da missão (meia-missão, `MID_MISSION_SCENARIO_NAME`),
        // JÁ CONVERGIDO (`wb.scenarios`, saída do `WeightBalanceAgent` desta
        // MESMA iteração — distinto do valor lag-1 usado dentro do laço de
        // `orchestrator::size_aircraft` para acoplar `cd_trim` ao polar
        // ANTES do `WeightBalanceAgent` rodar; ver docstring de
        // `TrimSpec::cl_h_trim_cruise`). `cm_ac` ISOLADO (sem
        // `cm_flap_delta` — cruzeiro é sem flap, ao contrário de
        // `cm_ac_total` usado em flare/rotação acima).
        let cg_ref = wb.scenarios.iter()
            .find(|sc| sc.name == crate::agents::weight_balance::MID_MISSION_SCENARIO_NAME)
            .unwrap_or_else(|| panic!(
                "cenário de referência '{}' não encontrado em wb.scenarios — deveria sempre \
                 existir (ver agents::weight_balance::scenarios_def)",
                crate::agents::weight_balance::MID_MISSION_SCENARIO_NAME
            ));
        let x_bar_cg_ref = cg_ref.cg_pct_mac / 100.0;
        // Momento da LINHA DE TRAÇÃO em cruzeiro (ciclo 10, task 2) — braço
        // sobre o CG (`prop_axis_above_cg_m`), não sobre o solo; ver
        // `cm_thrust_cruise`. `q` é a pressão dinâmica de cruzeiro (mesma
        // que `AerodynamicsAgent::run` usa para fechar `cl_cruise`).
        let q_cruise = crate::agents::aerodynamics::dynamic_pressure(
            Isa::density_kgm3(req.cruise_altitude_m, req.isa_delta_c),
            req.cruise_speed_min_kmh / 3.6,
        );
        let cm_thrust = cm_thrust_cruise(
            thrust_cruise_n, cfg.propeller.prop_axis_above_cg_m, q_cruise, wing.area_m2, mac,
        );
        let cl_h_trim_cruise_val = cl_h_trim_cruise(
            cfg.wing.cm_ac, wing.cl_cruise, x_bar_cg_ref, emp.eta_h, s_ratio, l_h_over_mac,
            cm_thrust,
        );
        let cd_trim_val = cd_trim_cruise(cl_h_trim_cruise_val, emp.ar_h, cfg.empennage.e_h, s_ratio);

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
                elevator_deflection_max_deg_minus: deflection_minus_deg,
                flare_limit_pct_mac_deflection_minus: x_flare_deflection_minus * 100.0,
                elevator_deflection_max_deg_plus: deflection_plus_deg,
                flare_limit_pct_mac_deflection_plus: x_flare_deflection_plus * 100.0,
            },
            cm_ac: cfg.wing.cm_ac,
            cm_flap_delta: cfg.wing.cm_flap_delta,
            cl_h_max_down,
            cl_h_max_down_calc: cl_h_max_down_calc_raw,
            tau_elevator: tau,
            capped_by_stall,
            trim_margin: cfg.stability.trim_margin,
            cl_ground_rotation: cfg.stability.cl_ground_rotation,
            to_flap_fraction: cfg.stability.to_flap_fraction,
            cl_h_trim_cruise: cl_h_trim_cruise_val,
            cd_trim: cd_trim_val,
            cg_reference_scenario: cg_ref.name.to_string(),
            cg_reference_pct_mac: cg_ref.cg_pct_mac,
        }
    }
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::atmosphere::RHO_SL;

    /// Massas estruturais COMPUTADAS (ciclo 3, `agents::mass_model`) para os
    /// testes que montam um `WeightBalanceOutput` a partir do baseline real
    /// sem passar pelo orchestrator. Usa o MTOW do estado (palpite inicial
    /// de `[sizing]`) e o seed 3,8 do lag-1 de `n_design` — mesmo par
    /// documentado nas demais fixtures de teste deste crate (ver
    /// `orchestrator::size_aircraft_with_max_iters`).
    fn masses_do_baseline(
        cfg: &crate::models::aircraft_config::AircraftConfig,
        engine: &crate::models::engine::EngineSpec,
        req: &crate::models::requirements::Requirements,
        wing: &crate::models::specs::WingSpec,
        emp: &crate::models::specs::EmpennageSpec,
        state: &crate::models::aircraft_state::AircraftState,
    ) -> crate::agents::mass_model::StructuralMasses {
        crate::agents::mass_model::MassModelAgent::run(
            cfg, engine, req, wing, emp, state.mtow_kg, 3.8,
        )
    }

    // ─── Hand-checks (valores do baseline real) ──────────────────────────
    //
    // MAC=1,2463161m, l_h=4,80m → l_h/MAC=3,85135; S_h/S=2,580913/14,2=
    // 0,181754; η_h=0,90; CL_flare=1,72/1,69=1,017751;
    // cm_ac_total=−0,008−0,30=−0,308; avail=−0,85·0,90=−0,765;
    // cm_to (rotação, SINALIZADO) = −0,008+0,5·(−0,30) = −0,158.

    // ─── Task refino-ciclo2 (1a): autoridade calculada por geometria ─────
    //
    // Hand-check baseline E6 (c_e/c=0,40, AR_h=4,0 → a_t=3,8832, δ=25°=
    // 0,436332 rad): τ = 1,24·√0,40−0,16 = 1,24·0,632456−0,16 = 0,62425;
    // cl = 3,8832·0,62425·0,436332 ≈ 1,0578 (< teto 1,10).

    #[test]
    fn tau_elevator_hand_check_baseline() {
        let tau = tau_elevator(0.40);
        println!("tau(0.40) = {tau:.6}");
        assert!((tau - 0.62425).abs() < 1e-4, "tau = {tau:.6} (esperado ≈0.62425)");
    }

    /// Sanidade: com a corda ANTIGA (pré-E6, 0.35) a mesma fórmula produz
    /// τ≈0.5735, cl≈0.9720 — próximo do palpite 0.95 assumido na E6 (o
    /// palpite era consistente com a física, mesmo sem tê-la calculado).
    #[test]
    fn tau_elevator_sanidade_corda_antiga_0_35() {
        let tau = tau_elevator(0.35);
        println!("tau(0.35) = {tau:.6}");
        assert!((tau - 0.5735).abs() < 1e-3, "tau = {tau:.6} (esperado ≈0.5735)");

        let a_t = crate::agents::weight_balance::lift_curve_slope(4.0);
        let delta_rad = 25.0_f64.to_radians();
        let (cl, capped) = cl_h_max_down_calc(a_t, tau, delta_rad, 1.10);
        println!("cl(corda=0.35) = {cl:.6}  capped={capped}");
        assert!((cl - 0.9720).abs() < 1e-3, "cl = {cl:.6} (esperado ≈0.9720)");
        assert!(!capped);
    }

    #[test]
    fn cl_h_max_down_calc_hand_check_baseline() {
        let a_t = crate::agents::weight_balance::lift_curve_slope(4.0);
        let tau = tau_elevator(0.40);
        let delta_rad = 25.0_f64.to_radians();
        let (cl, capped) = cl_h_max_down_calc(a_t, tau, delta_rad, 1.10);
        println!("a_t={a_t:.6}  tau={tau:.6}  cl={cl:.6}  capped={capped}");
        assert!((cl - 1.0578).abs() < 0.01, "cl = {cl:.6} (esperado ≈1.0578 ±0.01)");
        assert!(!capped, "cl (≈1.058) não deveria ser limitado pelo teto de stall (1.10)");
    }

    /// Quando o valor bruto (a_t·τ·δ) ultrapassa `cl_h_stall_limit`, o
    /// resultado deve ser TRUNCADO no teto, com `capped_by_stall=true`.
    #[test]
    fn cl_h_max_down_calc_e_limitado_pelo_teto_de_stall() {
        let a_t = crate::agents::weight_balance::lift_curve_slope(4.0);
        let tau = tau_elevator(0.60); // corda de profundor grande — τ maior
        let delta_rad = 35.0_f64.to_radians(); // deflexão grande
        let cl_h_stall_limit = 0.90; // teto baixo, propositalmente restritivo
        let (cl, capped) = cl_h_max_down_calc(a_t, tau, delta_rad, cl_h_stall_limit);
        println!("a_t={a_t:.6}  tau={tau:.6}  cl(bruto seria maior)  cl_limitado={cl:.6}  capped={capped}");
        assert_eq!(cl, cl_h_stall_limit, "cl deveria ser exatamente o teto quando limitado");
        assert!(capped, "capped_by_stall deveria ser true quando o bruto excede o teto");
    }

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

    /// Peso arbitrário (N) usado nos hand-checks/properties PURAMENTE
    /// AERODINÂMICOS da rotação (ciclo 10, task 2 — `rotation_fwd_limit_m`
    /// deixou de ser invariante ao peso e agora EXIGE um). Com
    /// `thrust_rot_n = 0,0` o resultado é o mesmo para QUALQUER valor aqui
    /// (o cancelamento algébrico da parte aerodinâmica sobrevive — ver
    /// `rotation_limit_e_invariante_a_massas_diferentes_sem_tracao`), então
    /// este número não é um pin: é só um valor legal para a assinatura.
    const W_AERO_TESTE_N: f64 = 1500.0 * G;

    /// Hand-check do limite de rotação PURAMENTE AERODINÂMICO (`thrust_rot_n
    /// = 0,0` — o modelo pré-ciclo-10, preservado bit-a-bit): x_cg_rot ≈
    /// 3,3976 m → (3,3976−2,90)/1,2463 ≈ 39,93% MAC. O termo NOVO da linha
    /// de tração tem hand-check próprio em
    /// `momento_da_linha_de_tracao_hand_check_com_literais`.
    #[test]
    fn rotation_fwd_limit_m_hand_check_baseline_sem_tracao() {
        let mac = 1.2463161361039574;
        let mac_le = 2.90;
        let s_h = 2.5809129985152786;
        let s_w = 14.2;
        let x_ac_wing = 2.90 + 0.25 * mac;
        let x_ac_tail = 2.90 + 0.25 * mac + 4.80;
        let x_main = 3.85;

        let x_rot = rotation_fwd_limit_m(
            W_AERO_TESTE_N, s_w, 1.72, s_h, 0.90, 0.85, 0.10, x_ac_tail, x_main, 0.5, x_ac_wing,
            -0.008, 0.5, -0.30, mac, 0.0, 0.0,
        );
        println!("x_cg_rot = {x_rot:.4} m (esperado ≈3.3976 m)");
        assert!((x_rot - 3.3976).abs() < 0.02, "x_cg_rot = {x_rot:.4} (esperado ≈3.3976 ±0.02m)");

        let rot_pct = cg_pct_mac(x_rot, mac_le, mac);
        println!("rot_pct = {rot_pct:.3}% MAC (esperado ≈39.93% ±1%)");
        assert!((rot_pct - 39.93).abs() < 1.0, "rot_pct = {rot_pct:.3}% (esperado ≈39.93% ±1%)");
    }

    // ─── FIX 1 (crítico): invariância ao peso ────────────────────────────
    //
    // Ciclo 10 (task 2): a invariância ao peso do limite de rotação MORREU
    // — o momento da linha de tração (`T(Vr(W))·z_eixo`) não é proporcional
    // a `W`. O teste abaixo foi RENOMEADO e restrito à parte AERODINÂMICA
    // (`thrust_rot_n = 0,0`), onde o cancelamento algébrico continua exato e
    // continua valendo a pena guardar; a morte da invariância (direção +
    // magnitude MEDIDA) tem testes próprios logo em seguida.

    /// Prova numérica do cancelamento algébrico de `W` na parte
    /// AERODINÂMICA (ver docstring de `rotation_fwd_limit_m`): com
    /// `thrust_rot_n = 0,0`, duas massas bem diferentes (extremos do
    /// baseline real, 1193,4 kg vs 1543,4 kg) devem produzir o MESMO limite
    /// dianteiro de rotação, avaliando `rotation_available_moment_nm(W,...)
    /// / W` INDEPENDENTEMENTE para cada uma.
    #[test]
    fn rotation_limit_e_invariante_a_massas_diferentes_sem_tracao() {
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
                -0.008, 0.5, -0.30, mac, 0.0, 0.0,
            );
            x_main - m / w_n
        };

        let x_leve = x_cg_rot_para(1193.4);
        let x_pesado = x_cg_rot_para(1543.4);
        println!("x_cg_rot(1193.4kg)={x_leve:.9}  x_cg_rot(1543.4kg)={x_pesado:.9}");
        assert!((x_leve - x_pesado).abs() < 1e-9,
            "limite de rotação SEM tração deveria ser IDÊNTICO independente do peso do cenário: \
             leve={x_leve:.9}m pesado={x_pesado:.9}m (diferença={:.2e})", (x_leve-x_pesado).abs());
    }

    // ─── CICLO 10 (task 2): momento da LINHA DE TRAÇÃO na rotação ─────────

    /// Hand-check do termo NOVO com LITERAIS: o momento da linha de tração
    /// é EXATAMENTE `T·z_eixo`, subtraído do momento nariz-acima disponível.
    ///
    ///   T = 4.000 N, z_eixo = 1,12 m (= h_cg_ground 0,92 + offset 0,20 do
    ///   baseline E10) ⟹ M_T = 4.480,0 N·m, NARIZ-ABAIXO.
    ///
    /// Verificado dos DOIS lados: (a) a diferença entre o momento com e sem
    /// tração é exatamente −4.480,0 N·m; (b) o deslocamento do limite
    /// dianteiro é exatamente `+M_T/W` (recuo).
    #[test]
    fn momento_da_linha_de_tracao_hand_check_com_literais() {
        let mac = 1.2463161361039574;
        let s_h = 2.5809129985152786;
        let s_w = 14.2;
        let x_ac_wing = 2.90 + 0.25 * mac;
        let x_ac_tail = 2.90 + 0.25 * mac + 4.80;
        let x_main = 3.66;
        let w_n = 1400.0 * G;

        let t_n = 4_000.0_f64;
        let z_axis = 0.92 + 0.20; // = 1,12 m
        let m_t_esperado = 4_480.0_f64; // 4000 × 1,12

        let m_sem = rotation_available_moment_nm(
            w_n, s_w, 1.6775, s_h, 0.90, 1.0577, 0.10, x_ac_tail, x_main, 0.5, x_ac_wing,
            -0.008, 0.35, -0.30, mac, 0.0, 0.0,
        );
        let m_com = rotation_available_moment_nm(
            w_n, s_w, 1.6775, s_h, 0.90, 1.0577, 0.10, x_ac_tail, x_main, 0.5, x_ac_wing,
            -0.008, 0.35, -0.30, mac, t_n, z_axis,
        );
        println!("M_sem={m_sem:.4} N·m  M_com={m_com:.4} N·m  Δ={:.4} N·m", m_com - m_sem);
        assert!(((m_sem - m_com) - m_t_esperado).abs() < 1e-9,
            "o termo da linha de tração deveria SUBTRAIR exatamente T·z = {m_t_esperado:.1} N·m \
             do momento disponível — obtido {:.6} N·m", m_sem - m_com);

        let x_sem = rotation_fwd_limit_m(
            w_n, s_w, 1.6775, s_h, 0.90, 1.0577, 0.10, x_ac_tail, x_main, 0.5, x_ac_wing,
            -0.008, 0.35, -0.30, mac, 0.0, 0.0,
        );
        let x_com = rotation_fwd_limit_m(
            w_n, s_w, 1.6775, s_h, 0.90, 1.0577, 0.10, x_ac_tail, x_main, 0.5, x_ac_wing,
            -0.008, 0.35, -0.30, mac, t_n, z_axis,
        );
        let delta_esperado_m = m_t_esperado / w_n;
        println!("x_sem={x_sem:.6}m  x_com={x_com:.6}m  Δ={:.6}m (esperado {delta_esperado_m:.6}m)",
                 x_com - x_sem);
        assert!(x_com > x_sem,
            "com tração o limite dianteiro deveria RECUAR (x maior): sem={x_sem:.6} com={x_com:.6}");
        assert!(((x_com - x_sem) - delta_esperado_m).abs() < 1e-9,
            "o recuo deveria ser exatamente T·z/W = {delta_esperado_m:.9} m — obtido {:.9} m",
            x_com - x_sem);
    }

    /// Property ESTRITA (a que a spec do ciclo 10 pede): eixo da hélice
    /// MAIS ALTO (`z_eixo` maior — a alavanca que o candidato E11 quer
    /// puxar, +12 cm) ⟹ mais binário nariz-abaixo da tração ⟹ limite
    /// dianteiro de rotação RECUA. Se este teste falhar com o limite
    /// AVANÇANDO, o sinal do termo está invertido.
    #[test]
    fn eixo_mais_alto_recua_o_limite_de_rotacao() {
        let mac = 1.2463161361039574;
        let s_h = 2.5809129985152786;
        let s_w = 14.2;
        let x_ac_wing = 2.90 + 0.25 * mac;
        let x_ac_tail = 2.90 + 0.25 * mac + 4.80;
        let x_main = 3.66;
        let w_n = 1400.0 * G;

        let x_para_z = |z: f64| rotation_fwd_limit_m(
            w_n, s_w, 1.6775, s_h, 0.90, 1.0577, 0.10, x_ac_tail, x_main, 0.5, x_ac_wing,
            -0.008, 0.35, -0.30, mac, 4_000.0, z,
        );

        let x_baixo = x_para_z(1.12);          // baseline E10
        let x_alto = x_para_z(1.12 + 0.12);    // candidato E11 (+12 cm)
        println!("x_cg_rot(z=1.12)={x_baixo:.6}m  x_cg_rot(z=1.24)={x_alto:.6}m");
        assert!(x_alto > x_baixo,
            "eixo da hélice MAIS ALTO (z=1.24, {x_alto:.6}m) deveria RECUAR ESTRITAMENTE o \
             limite dianteiro de rotação em relação ao eixo mais baixo (z=1.12, {x_baixo:.6}m) — \
             tração acima do pivô é nariz-ABAIXO e ATRAPALHA a rotação");
    }

    /// Property ESTRITA gêmea da anterior, na outra variável do termo:
    /// TRAÇÃO maior (mesmo braço) ⟹ limite dianteiro de rotação RECUA.
    #[test]
    fn tracao_maior_recua_o_limite_de_rotacao() {
        let mac = 1.2463161361039574;
        let s_h = 2.5809129985152786;
        let s_w = 14.2;
        let x_ac_wing = 2.90 + 0.25 * mac;
        let x_ac_tail = 2.90 + 0.25 * mac + 4.80;
        let x_main = 3.66;
        let w_n = 1400.0 * G;

        let x_para_t = |t: f64| rotation_fwd_limit_m(
            w_n, s_w, 1.6775, s_h, 0.90, 1.0577, 0.10, x_ac_tail, x_main, 0.5, x_ac_wing,
            -0.008, 0.35, -0.30, mac, t, 1.12,
        );

        let x_t3000 = x_para_t(3_000.0);
        let x_t5000 = x_para_t(5_000.0);
        println!("x_cg_rot(T=3000N)={x_t3000:.6}m  x_cg_rot(T=5000N)={x_t5000:.6}m");
        assert!(x_t5000 > x_t3000,
            "tração MAIOR (5000 N, {x_t5000:.6}m) deveria RECUAR ESTRITAMENTE o limite \
             dianteiro de rotação em relação a 3000 N ({x_t3000:.6}m)");
    }

    /// `Vr = 1,1·Vs0_TO(W)` — hand-check com literais do baseline E10
    /// (W = 1400·9,807 N, ρ = ρ_SL, S_w = 14,2 m², CL_max_TO = 1,6775):
    ///   Vs0_TO = √(2·13.729,8/(1,225·14,2·1,6775)) = √(27.459,6/29,182)
    ///          = √940,98 ≈ 30,676 m/s  ⟹  Vr ≈ 33,743 m/s
    #[test]
    fn rotation_speed_ms_hand_check_com_literais() {
        let vr = rotation_speed_ms(1400.0 * G, RHO_SL, 14.2, 1.6775);
        println!("Vr = {vr:.4} m/s (esperado ≈33.743)");
        assert!((vr - 33.743).abs() < 0.01, "Vr = {vr:.4} m/s (esperado ≈33.743 ±0.01)");
        // Vr ∝ √W — dobrar o peso multiplica Vr por √2.
        let vr_2w = rotation_speed_ms(2800.0 * G, RHO_SL, 14.2, 1.6775);
        assert!((vr_2w / vr - std::f64::consts::SQRT_2).abs() < 1e-12,
            "Vr deveria escalar com √W: Vr(2W)/Vr(W) = {:.12}", vr_2w / vr);
    }

    /// **A morte da invariância a `W`, com DIREÇÃO estrita** (ciclo 10,
    /// task 2 — ver a re-derivação na docstring de `rotation_fwd_limit_m`):
    /// com a linha de tração ligada e `T` FIXA (isolando o efeito de `W` no
    /// divisor, sem o efeito de `Vr(W)` sobre `T`), aeronave mais LEVE ⟹
    /// termo `T·z/W` MAIOR ⟹ limite dianteiro RECUA.
    ///
    /// É por isso que `TrimAuthorityAgent::run` avalia o limite ÚNICO
    /// reportado no cenário MAIS LEVE (o mais restritivo). Com `T` VARIÁVEL
    /// via `Vr(W)` o efeito só se ACENTUA (`T` cai quando `Vr` sobe), o que
    /// o teste de magnitude logo abaixo mede no baseline real.
    #[test]
    fn limite_de_rotacao_recua_com_peso_menor() {
        let mac = 1.2463161361039574;
        let s_h = 2.5809129985152786;
        let s_w = 14.2;
        let x_ac_wing = 2.90 + 0.25 * mac;
        let x_ac_tail = 2.90 + 0.25 * mac + 4.80;
        let x_main = 3.66;

        let x_para_w = |mass_kg: f64| rotation_fwd_limit_m(
            mass_kg * G, s_w, 1.6775, s_h, 0.90, 1.0577, 0.10, x_ac_tail, x_main, 0.5,
            x_ac_wing, -0.008, 0.35, -0.30, mac, 4_000.0, 1.12,
        );

        let x_leve = x_para_w(1193.4);
        let x_pesado = x_para_w(1543.4);
        println!("x_cg_rot(1193.4kg)={x_leve:.6}m  x_cg_rot(1543.4kg)={x_pesado:.6}m");
        assert!(x_leve > x_pesado,
            "com a linha de tração no balanço, o cenário mais LEVE (1193,4 kg, {x_leve:.6}m) \
             deveria ter o limite dianteiro MAIS RECUADO que o mais pesado (1543,4 kg, \
             {x_pesado:.6}m) — a invariância a W morreu no ciclo 10 (task 2)");
    }

    /// `rotation_fwd_limit_m` (fechado, assume `W=1`) deve bater com uma
    /// avaliação TOTALMENTE INDEPENDENTE do balanço de momentos — computa
    /// `Vs0_TO(W)` via `sqrt`, `Vr`, `q_r` explicitamente para um peso
    /// arbitrário (12.000 N), em vez de usar a substituição algébrica
    /// `q_r(W) = 1,21·W/(S·CL_max_TO)` usada internamente pela função.
    ///
    /// Ciclo 7 (task 1): o CLmax deste hand-check passa a ser o de
    /// DECOLAGEM — 1,585 = 1,45 + 0,5·(1,72 − 1,45), a interpolação do
    /// baseline real (`cl_max_clean` 1,45, `cl_max_flaps` 1,72,
    /// `to_flap_fraction` 0,5) — e não mais o 1,72 de POUSO.
    ///
    /// Ciclo 10 (task 2): o balanço independente ganha o termo da LINHA DE
    /// TRAÇÃO (`− T·z_eixo`, com T e z LITERAIS aqui — 4.200 N × 1,12 m),
    /// escrito à mão do lado independente também. Continua sendo um caminho
    /// TOTALMENTE separado da substituição algébrica `q_r(W) = 1,21·W/(S·
    /// CL_max_TO)` usada dentro da função.
    #[test]
    fn rotation_fwd_limit_m_bate_com_balanco_de_momentos_independente() {
        let mac = 1.2463161361039574;
        let s_h = 2.5809129985152786;
        let s_w = 14.2;
        let cl_max_to = 1.45 + 0.5 * (1.72 - 1.45); // = 1.585 (flap de DECOLAGEM)
        let x_ac_wing = 2.90 + 0.25 * mac;
        let x_ac_tail = 2.90 + 0.25 * mac + 4.80;
        let x_main = 3.85;

        let t_rot = 4_200.0_f64;
        let z_axis = 1.12_f64;

        let w_n = 12_000.0_f64;
        let vs0 = (2.0 * w_n / (RHO_SL * s_w * cl_max_to)).sqrt();
        let vr = 1.1 * vs0;
        let q_r = 0.5 * RHO_SL * vr * vr;
        let f_h = q_r * s_h * 0.90 * 0.85 * (1.0 - 0.10);
        let l_g = q_r * s_w * 0.5;
        let cm_to = -0.008 + 0.5 * -0.30;
        let m_cm = cm_to * q_r * s_w * mac;
        let moment = f_h * (x_ac_tail - x_main) + l_g * (x_main - x_ac_wing) + m_cm
            - t_rot * z_axis;
        let x_independente = x_main - moment / w_n;

        let x_fechado = rotation_fwd_limit_m(
            w_n, s_w, cl_max_to, s_h, 0.90, 0.85, 0.10, x_ac_tail, x_main, 0.5, x_ac_wing,
            -0.008, 0.5, -0.30, mac, t_rot, z_axis,
        );

        println!("independente={x_independente:.9}m  fechado={x_fechado:.9}m");
        assert!((x_independente - x_fechado).abs() < 1e-9,
            "solução fechada ({x_fechado:.9}m) deveria bater com o balanço de momentos \
             independente ({x_independente:.9}m)");

        // Sanidade do próprio `rotation_speed_ms` contra o `Vr` escrito à
        // mão aqui (mesma fórmula, caminho separado).
        let vr_fn = rotation_speed_ms(w_n, RHO_SL, s_w, cl_max_to);
        assert!((vr_fn - vr).abs() < 1e-12, "Vr(fn)={vr_fn:.12} vs Vr(mão)={vr:.12}");
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

        // to_flap_fraction=0 e cm_flap_delta=0 (irrelevantes) — só
        // cm_ac controla o sinal de Cm_TO aqui.
        let x_positivo = rotation_fwd_limit_m(
            W_AERO_TESTE_N, s_w, 1.72, s_h, 0.90, 0.85, 0.10, x_ac_tail, x_main, 0.5, x_ac_wing,
            0.04, 0.0, 0.0, mac, 0.0, 0.0,
        );
        let x_negativo = rotation_fwd_limit_m(
            W_AERO_TESTE_N, s_w, 1.72, s_h, 0.90, 0.85, 0.10, x_ac_tail, x_main, 0.5, x_ac_wing,
            -0.04, 0.0, 0.0, mac, 0.0, 0.0,
        );

        println!("x_cg_rot(cm=+0.04)={x_positivo:.4}  x_cg_rot(cm=-0.04)={x_negativo:.4}");
        assert!(x_positivo < x_negativo,
            "Cm_TO positivo (+0.04, {x_positivo:.4}m) deveria mover o limite de rotação para a \
             FRENTE (x menor) em relação ao Cm_TO negativo (-0.04, {x_negativo:.4}m) — Cm \
             positivo é nariz-acima e ajuda a rotação");
    }

    /// Ciclo 7 (task 1): o OUTRO lado do trade-off que
    /// `[stability].to_flap_fraction` carrega — mais deployment de flap no
    /// setting de decolagem ⟹ `cl_max_to` maior ⟹ `Vs0_TO`/`Vr` MENORES
    /// ⟹ `q_r` menor ⟹ menos momento nariz-acima disponível na rotação ⟹
    /// limite dianteiro de rotação SOBE (x_cg_rot maior, mais atrás,
    /// ESTRITAMENTE). Vem acompanhado, na mesma direção, do ΔCm de flap
    /// (mais nariz-abaixo com fração maior). O ganho (líquido, desde o
    /// ciclo 8 task 1 — a polar agora cobra arrasto de flap) de decolagem
    /// está em
    /// `performance::tests::mais_flap_de_decolagem_trade_off_liquido_na_decolagem_sobre_15m`.
    #[test]
    fn mais_flap_de_decolagem_sobe_o_limite_de_rotacao() {
        use crate::agents::aerodynamics::AerodynamicsAgent;
        use crate::models::aircraft_config::test_fixtures::config_teste;
        use crate::models::aircraft_state::AircraftState;

        let req = crate::models::requirements::test_fixtures::requisitos_teste();
        let mac = 1.2463161361039574;
        let s_h = 2.5809129985152786;
        let x_ac_wing = 2.90 + 0.25 * mac;
        let x_ac_tail = 2.90 + 0.25 * mac + 4.80;
        let x_main = 3.85;

        // Duas configurações idênticas EXCETO pela fração de flap de
        // decolagem — a asa (e portanto `cl_max_to`) é recomputada em cada.
        let x_rot_para = |fracao: f64| {
            let mut cfg = config_teste();
            cfg.stability.to_flap_fraction = fracao;
            let state = AircraftState::from_config(&cfg);
            let wing = AerodynamicsAgent::run(&state, &req);
            let x = rotation_fwd_limit_m(
                W_AERO_TESTE_N, wing.area_m2, wing.cl_max_to, s_h, 0.90, 0.85,
                cfg.stability.trim_margin,
                x_ac_tail, x_main, cfg.stability.cl_ground_rotation, x_ac_wing, cfg.wing.cm_ac,
                cfg.stability.to_flap_fraction, cfg.wing.cm_flap_delta, mac, 0.0, 0.0,
            );
            (wing.cl_max_to, x)
        };

        let (cl_03, x_03) = x_rot_para(0.3);
        let (cl_07, x_07) = x_rot_para(0.7);
        println!("fração 0.3: cl_max_to={cl_03:.4} x_cg_rot={x_03:.4}m  |  \
                  fração 0.7: cl_max_to={cl_07:.4} x_cg_rot={x_07:.4}m");
        assert!(cl_07 > cl_03, "fração maior deveria dar cl_max_to maior");
        assert!(x_07 > x_03,
            "mais flap de decolagem (fração 0.7, cl_max_to={cl_07:.4}) deveria SUBIR \
             ESTRITAMENTE o limite dianteiro de rotação (Vr menor ⟹ q_r menor ⟹ menos \
             autoridade) em relação a 0.3 (cl_max_to={cl_03:.4}): \
             x(0.7)={x_07:.4}m, x(0.3)={x_03:.4}m");
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
            W_AERO_TESTE_N, s_w, 1.72, s_h, 0.90, 0.85, 0.10, x_ac_tail, 3.50, 0.5, x_ac_wing,
            -0.008, 0.5, -0.30, mac, 0.0, 0.0,
        );
        let x_rot_atras = rotation_fwd_limit_m(
            W_AERO_TESTE_N, s_w, 1.72, s_h, 0.90, 0.85, 0.10, x_ac_tail, 4.20, 0.5, x_ac_wing,
            -0.008, 0.5, -0.30, mac, 0.0, 0.0,
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
    /// real e confirma: (1) o limite de flare bate (agora NEGATIVO — nunca
    /// governa, ver nota abaixo); (2) o limite de rotação (número ÚNICO)
    /// bate; (3) a ROTAÇÃO ainda GOVERNA mas agora fica À FRENTE do limite
    /// traseiro — envelope de CG FECHADO (campanha E1–E6, 2026-08-05); (4)
    /// a margem de autoridade por cenário é POSITIVA em TODOS os cenários
    /// reais.
    ///
    /// Campanha E1–E6 (2026-08-05): antes desta campanha (achado honesto
    /// original, Task 4.4/trim-authority), flare_limit_pct_mac≈7.908%,
    /// rotation_limit_pct_mac≈39.93% (À FRENTE do limite traseiro
    /// ≈36.6% — envelope VAZIO), e a margem de autoridade de rotação era
    /// NEGATIVA em todos os cenários. O trem principal recuado
    /// (`gear.x_main_m` 3.85→3.55) e a EH maior/mais autoridade de
    /// profundor (`v_h` 0.70→0.85, `cl_h_max_down` 0.85→0.95, palpite de
    /// config) fecham o envelope: rotation_limit_pct_mac cai para ≈10.95%
    /// (bem à frente do limite traseiro, ≈43.46%) e o flare_limit_pct_mac
    /// fica NEGATIVO (≈-9.00% — fisicamente "antes do bordo de ataque",
    /// nunca governa).
    ///
    /// Task refino-ciclo2 (2026-08-05, 1a): `cl_h_max_down` deixa de ser um
    /// palpite de config e passa a ser CALCULADO por geometria DATCOM/
    /// Nelson (`c_e/c=0.40`, `AR_h=4.0`, `δe_max=25°` →
    /// `cl_h_max_down_calc≈1.0577`, +11,3% sobre o palpite 0.95 da E6 —
    /// abaixo do teto de stall 1.10, `capped_by_stall=false`). Mais
    /// autoridade → mais download disponível → os DOIS limites avançam
    /// (%MAC menor): rotation_limit_pct_mac 10.948%→**6.099%** (margens de
    /// CG melhoram ainda mais) e flare_limit_pct_mac -9.004%→**-16.290%**
    /// (mais negativo, continua nunca governando). `cg_limit_aft_pct_mac`
    /// não muda (limite traseiro depende só de `sm_min`/NP, não de
    /// autoridade de profundor) — permanece ≈43.46%. Envelope de CG
    /// continua FECHADO, agora com margem MAIOR. O caminho de erro
    /// (envelope vazio) continua coberto por
    /// `trim_authority_agent_run_hand_check_baseline_mutado_parametros_
    /// pre_e6` logo abaixo.
    ///
    /// Campanha E7 (2026-08-06): `gear.x_main_m` 3.55→3.66m (fecha o
    /// tipback, ver `config/aircraft/baseline_4seat.toml`) recua o trem
    /// principal, o que por sua vez recua o limite de ROTAÇÃO (invariante
    /// ao peso — depende só de `x_main_m`, `x_nose_m`, geometria/
    /// autoridade de profundor, não do CG do cenário): rotation_limit_pct_
    /// mac **6.099%→12.995%** — ainda BEM à frente do limite traseiro
    /// (≈43.46%, inalterado pelo trem, só depende de NP/sm_min), envelope
    /// de CG continua FECHADO com margem ampla. flare_limit_pct_mac e
    /// cg_limit_aft_pct_mac não mudam (nenhum depende de x_main_m).
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
        // Campanha E10 (2026-08-08): massa de motor representativa da CLASSE
        // que esta célula assume (~195 kg) em vez dos 150 kg de
        // `motor_generico_teste()`. O motor está no braço mais dianteiro de
        // toda a aeronave (`[arms].engine_cg_m` = 0,65 m, ≈2,8 m à frente do
        // CG), então 45 kg de diferença ali valem ≈6,7 pp de MAC no CG de
        // TODOS os cenários — viés de fixture que domina qualquer pin
        // sensível ao CG (aqui: `cl_h_trim_cruise`/`cd_trim`, ancorados no
        // CG de meia-missão). Com o motor leve o CG de referência desta
        // fixture ia a 43,42% MAC, praticamente colado no limite traseiro
        // (43,46%) — um canto irreal, que tornava os pins frágeis e
        // desconectados do pipeline real. Com 195 kg o CG de referência cai
        // para 36,22% MAC, perto do valor do pipeline REAL (37,775%, ver
        // `aircraft_spec.json`/`cargo run`; o resíduo de ~1,6 pp vem de esta
        // fixture NÃO rodar o laço de convergência de MTOW — usa o palpite
        // de `[sizing]` — não do motor). O resto do motor segue
        // sintético/genérico: `src/` não conhece motores concretos (ver
        // `tests/acceptance.rs::src_nao_contem_nomes_de_motor_especificos`).
        // O limite de FLARE pinado acima NÃO depende do peso nem do CG (ver
        // derivação no código), então não se move com isto. O de ROTAÇÃO
        // deixou de ser invariante ao peso no ciclo 10 (task 2 — linha de
        // tração), mas depende do peso do cenário MAIS LEVE, não da massa
        // de motor por si só; ainda assim, mudar a massa de motor muda esse
        // cenário mais leve, então o pin dele acompanha a fixture.
        // Revisão final: constante compartilhada com o hand-check gêmeo de
        // `validation::constraint_checker` (mesma massa de motor, mesmo
        // motivo) — ver `models::engine::test_fixtures::MASSA_MOTOR_CLASSE_KG`.
        let mut engine = crate::models::engine::test_fixtures::motor_generico_teste();
        engine.mass_kg = crate::models::engine::test_fixtures::MASSA_MOTOR_CLASSE_KG;
        let masses = masses_do_baseline(&cfg, &engine, &req, &wing, &emp, &state);
        let wb = crate::agents::weight_balance::WeightBalanceAgent::run(
            &state, &wing, &engine, &cfg, &req, &emp, &masses,
        );

        // Ciclo 10 (task 2): tração de cruzeiro REAL desta fixture
        // (`PropulsionAgent` com o MESMO estado/asa/motor) — não um literal
        // solto, para o `cm_thrust` do trim de cruzeiro ser consistente com
        // o resto da fixture.
        let prop = crate::agents::propulsion::PropulsionAgent::run(&state, &req, &wing, &engine);
        let trim = TrimAuthorityAgent::run(&cfg, &wing, &emp, &wb, &state, &engine, &req,
                                           prop.thrust_cruise_n);
        println!("cl_h_max_down = {:.6}  cl_h_max_down_calc = {:.6}  tau_elevator = {:.6}  \
                   capped_by_stall = {}",
                 trim.cl_h_max_down, trim.cl_h_max_down_calc, trim.tau_elevator,
                 trim.capped_by_stall);
        // Hand-check (task refino-ciclo2, brief): τ=1.24·√0.40−0.16≈0.62425;
        // cl=3.8832·0.62425·0.43633≈1.0577 (a_t real de lift_curve_slope(4.0)).
        assert!((trim.tau_elevator - 0.62425).abs() < 1e-4,
            "tau_elevator = {:.6} (esperado ≈0.62425)", trim.tau_elevator);
        assert!((trim.cl_h_max_down_calc - 1.0577).abs() < 0.001,
            "cl_h_max_down_calc = {:.6} (esperado ≈1.0577)", trim.cl_h_max_down_calc);
        assert!(!trim.capped_by_stall,
            "cl_h_max_down_calc (≈1.058) não deveria ser limitado pelo teto de stall (1.10)");
        assert!((trim.cl_h_max_down - trim.cl_h_max_down_calc).abs() < 1e-9,
            "cl_h_max_down operacional deveria bater com cl_h_max_down_calc (não capado)");

        println!("flare_limit_pct_mac = {:.6}", trim.flare_limit_pct_mac);
        // Pin pré-refino-ciclo2 (cl_h_max_down=0.95 de config): ≈-9.004%.
        // Pin pré-E10 (cl_h_max_down_calc≈1.0577, geometria): ≈-16.290%.
        //
        // Pin NOVO (campanha E10, 2026-08-08): ≈**-8.819%**. Causa única e
        // fechada: `[wing].cl_max_flaps` 1,72→2,1 (flap SLOTTED). A flare é
        // avaliada em V_ref = 1,3·VS0, e VS0 ∝ 1/√CL_max ⟹ q_flare cai por
        // 1,72/2,1 = 0,819 (−18,1%). Todo o momento de profundor disponível
        // cai na mesma proporção, então o limite de flare RECUA (fica menos
        // negativo). Hand-check: o limite fica a (x_main − x_flare) do trem;
        // −16,290% · 0,819 ≈ −13,3% seria a conta ingênua, mas o ΔCm de
        // flap (cm_flap_delta, INALTERADO em −0,30) NÃO escala com q — só o
        // momento de profundor escala, e é a razão entre os dois que define
        // o ponto de equilíbrio, daí o recuo maior (−8,819%). Segue
        // NEGATIVO ("antes do bordo de ataque"), ou seja, continua nunca
        // governando: quem governa o limite dianteiro é a ROTAÇÃO (8,53%),
        // asserido logo abaixo. Tolerância INALTERADA (±1%).
        assert!(
            (trim.flare_limit_pct_mac - (-8.819)).abs() < 1.0,
            "flare_limit_pct_mac = {:.3} (esperado ≈-8.819% ±1%)",
            trim.flare_limit_pct_mac
        );

        println!("rotation_limit_pct_mac = {:.6}", trim.rotation_limit_pct_mac);
        println!("cg_limit_aft_pct_mac = {:.6}", wb.spec.cg_limit_aft_pct_mac);
        // Pin pré-refino-ciclo2: ≈10.948%. Pin pré-E7 (refino-ciclo2): ≈6.099%.
        // Pin pré-ciclo-7 (campanha E7, gear.x_main_m 3.55→3.66m): ≈12.995%.
        //
        // Pin ciclo 7 (task 1 — `cl_max_to`): ≈8.908%, uma queda de 4,087 pp
        // EXPLICADA EXATAMENTE pela física (não afrouxamento). A rotação
        // passou a usar o CLmax de DECOLAGEM (1,585 = 1,45 + 0,5·(1,72−1,45))
        // no lugar do CLmax de POUSO (1,72):
        //   Vs0_TO/Vr crescem √(1,72/1,585) = +4,21%
        //   q_r e, com ele, TODO o momento disponível: ×1,72/1,585 = +8,52%
        //   x_cg_rot = x_main − M/W ⟹ Δx = −0,08517·(x_main − x_cg_rot_antigo)
        //                            = −0,08517·(3,660 − 3,0620) = −0,0509 m
        //   em %MAC (MAC = 1,2463 m): −4,087 pp → 12,995 − 4,087 = 8,908 ✓
        // (verificado no baseline real com erro 0,0e0 m; o "≈2–3 pp"
        // estimado na spec §3 subestimava por ignorar a AMPLIFICAÇÃO do
        // braço: o limite fica 0,60 m à frente do trem, então 8,5% de
        // momento vale 5 cm de CG — e a MAC tem só 1,25 m.)
        // Sinal: com a Vr CORRETA (mais alta), há MAIS pressão dinâmica e
        // MAIS autoridade de profundor na rotação — o limite dianteiro
        // AVANÇA. O modelo antigo era pessimista, não conservador por
        // escolha.
        //
        // APERTO DELIBERADO (ciclo 8, task 2, §4 — dívida do ciclo 7): a
        // tolerância ±1.5 acima nunca foi reapertada depois que o valor
        // REAL do baseline convergiu para ≈8,533% (ver comentário mais
        // abaixo, linha ≈1291, e `tests/gear_tipback.rs`/`tests/cli.rs`,
        // que já documentavam esse número desde o ciclo 7) — o pin ficou
        // frouxo o bastante (0,375 pp de folga real dentro de uma banda de
        // ±1,5 pp) para não detectar uma regressão de até ~1,1 pp na
        // autoridade de rotação. RE-CENTRADO no valor MEDIDO atual
        // (8,533%, não recalculado — mesma fórmula/config de sempre) e a
        // tolerância aperta para ±0,05, a mesma disciplina de todos os
        // outros pins honestos deste arquivo. Isto NÃO é uma mudança de
        // física — é fechar uma folga de cobertura de teste.
        // ─── CICLO 10 (task 2): momento da LINHA DE TRAÇÃO ────────────────
        //
        // Pin NOVO desta fixture: **36,437%** MAC (era 8,533%). Um recuo de
        // +27,90 pp — de longe a maior mudança que este limite já sofreu, e
        // NÃO é afrouxamento nem bug: é um termo de momento que faltava.
        // A tração de decolagem age no eixo da hélice, 1,12 m ACIMA do solo
        // (`gear.h_cg_ground_m` 0,92 + `propeller.prop_axis_above_cg_m`
        // 0,20), e o pivô da rotação é o contato do trem principal com o
        // solo — logo a tração produz binário NARIZ-ABAIXO que consome
        // autoridade de profundor. Fechamento numérico (hand-check abaixo,
        // no próprio teste): Δx_cg_rot = T(Vr)·z_eixo / W_leve, com a
        // tração avaliada em Vr = 1,1·Vs0_TO(W_leve) — asserido com erro
        // < 1e-6 m logo em seguida, então o pin não é um número órfão.
        //
        // Por que o peso do cenário MAIS LEVE: o limite DEIXOU de ser
        // invariante ao peso (a prova antiga morreu — ver a re-derivação em
        // `rotation_fwd_limit_m`) e é MAIS restritivo quanto mais leve a
        // aeronave (T/W maior). Ver `limite_de_rotacao_recua_com_peso_menor`.
        //
        // Tolerância INALTERADA (±0,05 pp — mesma disciplina do aperto do
        // ciclo 8, task 2, §4). Valor do pipeline REAL (que converge o MTOW,
        // ao contrário desta fixture): 35,532% — mesma ordem, diferença
        // residual do laço de convergência, como nos demais pins daqui.
        //
        // Histórico dos pins anteriores preservado no comentário acima.
        assert!(
            (trim.rotation_limit_pct_mac - 36.437).abs() < 0.05,
            "rotation_limit_pct_mac = {:.3} (esperado ≈36.437% ±0.05% — pin pós-ciclo-10 task 2, \
             linha de tração)",
            trim.rotation_limit_pct_mac
        );

        // Hand-check FECHADO do recuo: reconstrói o limite SEM tração (a
        // física pré-ciclo-10, bit-a-bit) e confirma que a diferença é
        // EXATAMENTE `T(Vr(W_leve))·z_eixo / W_leve`.
        {
            let mass_light_kg = wb.scenarios.iter()
                .map(|sc| sc.total_mass_kg)
                .fold(f64::INFINITY, f64::min);
            let w_light_n = mass_light_kg * G;
            let z_axis = cfg.gear.h_cg_ground_m + cfg.propeller.prop_axis_above_cg_m;
            let vr = rotation_speed_ms(
                w_light_n,
                crate::models::atmosphere::Isa::density_kgm3(0.0, req.isa_delta_c),
                wing.area_m2, wing.cl_max_to,
            );
            let t_rot = thrust_at_rotation_n(
                w_light_n, wing.area_m2, wing.cl_max_to, &engine, &state, req.isa_delta_c,
                cfg.performance.static_thrust_factor,
            );
            println!("cenário mais leve = {mass_light_kg:.3} kg  Vr = {vr:.3} m/s  \
                      T(Vr) = {t_rot:.1} N  z_eixo = {z_axis:.3} m  \
                      T·z/W = {:.4} m", t_rot * z_axis / w_light_n);

            let x_ac_wing = cfg.wing.le_root_x_m + 0.25 * wb.mac_m;
            let x_ac_tail = x_ac_wing + emp.arm_h_m;
            let x_sem_tracao = rotation_fwd_limit_m(
                w_light_n, wing.area_m2, wing.cl_max_to, emp.s_horizontal_m2, emp.eta_h,
                trim.cl_h_max_down, cfg.stability.trim_margin, x_ac_tail, cfg.gear.x_main_m,
                cfg.stability.cl_ground_rotation, x_ac_wing, cfg.wing.cm_ac,
                cfg.stability.to_flap_fraction, cfg.wing.cm_flap_delta, wb.mac_m, 0.0, 0.0,
            );
            let x_com_tracao = wb.mac_le_x_m + trim.rotation_limit_pct_mac / 100.0 * wb.mac_m;
            let delta_medido = x_com_tracao - x_sem_tracao;
            let delta_esperado = t_rot * z_axis / w_light_n;
            println!("Δx_cg_rot medido = {delta_medido:.9} m  esperado (T·z/W) = \
                      {delta_esperado:.9} m");
            assert!((delta_medido - delta_esperado).abs() < 1e-6,
                "o recuo do limite de rotação deveria ser EXATAMENTE T·z/W = \
                 {delta_esperado:.9} m — medido {delta_medido:.9} m");
            assert!(delta_medido > 0.0, "o recuo deveria ser POSITIVO (limite vai para trás)");
        }

        // A rotação ainda governa (é o critério mais restritivo), mas
        // agora fica ATRÁS do limite traseiro — envelope de CG FECHADO
        // (achado honesto pós-refino-ciclo2, com margem maior que na E6).
        assert!(trim.rotation_limit_pct_mac > trim.flare_limit_pct_mac);
        assert_eq!(trim.governing, "rotacao",
            "governing deveria continuar 'rotacao' (mais restritiva que a flare, agora \
             negativa) no baseline real pós-refino-ciclo2");
        assert!(trim.rotation_limit_pct_mac <= wb.spec.cg_limit_aft_pct_mac,
            "limite de rotação ({:.2}%) deveria ficar ATRÁS (ou igual) do limite traseiro \
             ({:.2}%) — envelope de CG fechado no baseline real pós-refino-ciclo2",
            trim.rotation_limit_pct_mac, wb.spec.cg_limit_aft_pct_mac);

        // ACHADO HONESTO (ciclo 10, task 2) — a asserção original exigia
        // margem POSITIVA em TODOS os cenários (verdade do refino-ciclo2
        // até o ciclo 9). Com o momento da linha de tração no balanço, os
        // DOIS cenários mais LEVES do baseline real passam a ter margem
        // NEGATIVA: "Solo (piloto)" ≈−45% e "2 pax dianteiros" ≈−34% nesta
        // fixture (≈−41% e ≈−29% no pipeline real convergido). NÃO é um
        // bug e NÃO é mascarado aqui: é o preço físico da linha de tração
        // alta, que pesa MAIS quanto mais leve a aeronave (T/W maior a Vr
        // menor — ver `limite_de_rotacao_recua_com_peso_menor`). A decisão
        // de projeto (baixar o eixo, recuar o trem, limitar potência na
        // rotação, ou aceitar uma restrição operacional de carga mínima)
        // é HUMANA e está reportada no task-2-report do ciclo 10 — este
        // teste apenas GUARDA o achado, com o padrão medido:
        //   - os cenários LEVES (os dois primeiros) têm margem negativa;
        //   - os DEMAIS continuam positivos;
        //   - e a margem é MONOTONICAMENTE crescente com o peso? NÃO —
        //     ela mistura peso e CG do cenário, então só o padrão acima é
        //     asserido.
        let margens: Vec<(&str, f64)> = trim.rotation_margin_per_scenario.iter()
            .map(|sc| (sc.scenario.as_str(), sc.rotation_authority_margin_pct))
            .collect();
        println!("margens de rotação por cenário: {margens:?}");
        for (nome, margem) in &margens {
            let leve = *nome == "Solo (piloto)" || *nome == "2 pax dianteiros";
            if leve {
                assert!(*margem < 0.0,
                    "cenário '{nome}' (leve): margem de autoridade de rotação deveria ser \
                     NEGATIVA no baseline real pós-ciclo-10 (linha de tração) — obtido \
                     {margem:.2}%. Se isto virou positivo, a física da linha de tração foi \
                     enfraquecida sem que o achado fosse reavaliado.");
            } else {
                assert!(*margem > 0.0,
                    "cenário '{nome}': margem de autoridade de rotação deveria continuar \
                     POSITIVA no baseline real pós-ciclo-10 — obtido {margem:.2}%");
            }
        }

        // Arrasto de trim em cruzeiro (Task 4, refino-ciclo2) — hand-check
        // com o MTOW de PALPITE inicial (este teste NÃO passa pelo laço de
        // convergência do orchestrator, ver `AircraftState::from_config`
        // acima) — x̄_cg (meia-missão) ≈42,480201%, CL_cruise≈0,400245:
        //   num = −0,008+0,400245·(0,424802−0,25) = 0,061964
        //   den = 0,9·0,220702·(3,851350+0,25−0,424802) = 0,730279
        //   CL_h_trim = 0,061964/0,730279 ≈ +0,084849
        //   ΔCD_trim = (0,084849²/(π·4·0,70))·0,220702 ≈ 1,806e-4
        //
        // Campanha E7 (2026-08-06): `gear.x_main_m` 3.55→3.66m desloca o
        // braço do item de massa `trem_principal` (arm_ref="gear_main")
        // ~0,11m para trás, o que desloca x̄_cg (meia-missão) um pouco para
        // trás também: 42,480201%→42,834789% MAC. CL_cruise não muda
        // (independe do trem). Pin da E7: CL_h_trim_cruise 0,084849→
        // 0,086877, ΔCD_trim 1,806e-4→1,8937e-4.
        //
        // Ciclo 3 (oew-parametrico, Task 4): as 7 massas estruturais do OEW
        // passaram a ser COMPUTADAS (`agents::mass_model`) em vez de itens
        // fixos de `[[masses.items]]`/`mass_per_area`. O redimensionamento
        // é o achado do ciclo — em particular a fuselagem fica bem mais
        // leve (160,0→~110,6 kg, braço traseiro) e o trem principal bem
        // mais pesado (55,0→~90,7 kg, braço à frente do CG): o CG vazio
        // AVANÇA, e com ele o CG de todos os cenários. x̄_cg (meia-missão)
        // 42,834789%→**35,4739%** MAC. Novos pins (recalculados pela mesma
        // fórmula, TOLERÂNCIAS INALTERADAS): CL_h_trim_cruise
        // 0,086877→**0,045581**, ΔCD_trim 1,8937e-4→**5,213e-5**.
        //
        // Ciclo 4 (t/c dedicado da empenagem, `[empennage].thickness_ratio`,
        // 2026-08-07): a cauda fica mais pesada (braço TRASEIRO) — o CG
        // vazio RECUA, e com ele o CG de todos os cenários (efeito oposto
        // ao ciclo 3, mesma causa física — massa em braço traseiro recua o
        // CG). x̄_cg (meia-missão) 35,4739%→**35,9158%** MAC. Novos pins
        // (recalculados pela mesma fórmula, TOLERÂNCIAS INALTERADAS):
        // CL_h_trim_cruise 0,045581→**0,048015**, ΔCD_trim
        // 5,213e-5→**5,784e-5**.
        println!("cl_h_trim_cruise = {:.6}  cd_trim = {:.8}  cg_reference = '{}' ({:.4}% MAC)",
            trim.cl_h_trim_cruise, trim.cd_trim, trim.cg_reference_scenario, trim.cg_reference_pct_mac);
        assert_eq!(trim.cg_reference_scenario,
            crate::agents::weight_balance::MID_MISSION_SCENARIO_NAME);
        //
        // Campanha E10 (2026-08-08): DUAS mudanças entram, na mesma direção.
        // (a) A bateria híbrida de 53 kg a 7,80 m recua o CG de meia-missão
        //     desta fixture; (b) a massa de motor da fixture passa de 150
        //     para 195 kg (classe real — ver comentário acima), o que o
        //     AVANÇA de volta e mais um pouco. Saldo medido: x̄_cg
        //     35,9158%→**36,2181%** MAC. Novos pins (mesma fórmula,
        //     TOLERÂNCIAS INALTERADAS): CL_h_trim_cruise
        //     0,048015→**0,049682**, ΔCD_trim 5,784e-5→**6,193e-5**.
        // Sanidade contra o pipeline REAL (`aircraft_spec.json` de E10, que
        // converge o MTOW de verdade): x̄_cg 37,775%, CL_h_trim 0,052544,
        // ΔCD_trim 6,927e-5 — mesma ordem e mesmo sinal, a diferença
        // residual é o laço de convergência que esta fixture não roda.
        //
        // Ciclo 10 (task 2 — momento da linha de tração em CRUZEIRO): entra
        // `cm_thrust = −T_cruzeiro·prop_axis_above_cg_m/(q·S·MAC)`, NEGATIVO
        // (nariz-abaixo, eixo 0,20 m acima do CG). Ele SOMA ao `cm_ac`
        // (−0,008) no numerador do balanço, e vale ≈−0,0056 nesta fixture —
        // ou seja, quase DOBRA o momento nariz-abaixo de referência. A
        // empenagem responde com menos upload: CL_h_trim_cruise
        // 0,049682→**0,043152** (−13,1%), e como ΔCD_trim ∝ CL_h², o
        // arrasto de trim cai 6,193e-5→**4,672e-5** (−24,6%). O sinal está
        // auditado em `cm_thrust_negativo_reduz_cl_h_trim_cruise`. Note que
        // aqui o novo termo REDUZ o arrasto (o CG de referência está atrás
        // do CA, então o upload de trim estava POSITIVO e o `cm_thrust`
        // empurra `CL_h` na direção do ZERO); num CG mais dianteiro, onde o
        // trim já é download, o mesmo termo AUMENTARIA o arrasto.
        // TOLERÂNCIAS INALTERADAS (±1e-4 e ±1e-6). Valores do pipeline REAL
        // convergido: CL_h_trim 0,046201, ΔCD_trim 5,357e-5.
        assert!((trim.cl_h_trim_cruise - 0.043152).abs() < 1e-4,
            "cl_h_trim_cruise = {:.6} (esperado ≈0.043152 ±1e-4, pin pós-ciclo-10 task 2)",
            trim.cl_h_trim_cruise);
        assert!((trim.cd_trim - 4.672e-5).abs() < 1e-6,
            "cd_trim = {:.8} (esperado ≈4.672e-5 ±1e-6, pin pós-ciclo-10 task 2)", trim.cd_trim);
        assert!(trim.cl_h_trim_cruise > 0.0,
            "CG de referência atrás do CA (x̄≈36,2% > 25%) deveria produzir upload \
             (CL_h_trim_cruise > 0) mesmo com o cm_thrust nariz-abaixo — obtido {:.6}",
            trim.cl_h_trim_cruise);
    }

    /// Caminho de erro preservado (achado histórico pré-E6): mesma
    /// mutação de três parâmetros usada em
    /// `constraint_checker::tests::violacao_de_envelope_vazio_aparece_com_
    /// baseline_mutado_parametros_pre_e6` (`gear.x_main_m`, `empennage.v_h`,
    /// `control_surfaces.elevator_chord_frac` reduzidos a valores que
    /// reproduzem uma autoridade de download pré-E6 — desde a task
    /// refino-ciclo2, `[stability].cl_h_max_down` não existe mais como
    /// campo de config direto, ver comentário na mutação abaixo) —
    /// reproduz o achado honesto original: rotação governa E fica À
    /// FRENTE do limite traseiro (envelope de CG vazio). Nota: esta
    /// mutação reverte só os TRÊS parâmetros de trim/geometria da EH, não
    /// `arms.baggage_m` nem
    /// os itens de massa (2, 3, 8) — o cenário mais pesado ("4 pax +
    /// bagagem + cheio") já reflete o bagageiro avançado da E6, então
    /// tem CG deslocado o bastante para ter margem POSITIVA mesmo com a
    /// física de trim revertida; os demais cenários continuam com margem
    /// negativa. A checagem por-cenário abaixo exige só que ALGUM
    /// cenário fique com autoridade insuficiente (não mais TODOS, como no
    /// achado pré-E6 original com a config antiga completa).
    #[test]
    fn trim_authority_agent_run_hand_check_baseline_mutado_parametros_pre_e6() {
        let toml = std::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("config/aircraft/baseline_4seat.toml"),
        )
        .expect("falha ao ler baseline_4seat.toml do disco");
        let mut cfg = crate::models::config::parse_aircraft(&toml)
            .expect("baseline real deveria ser uma configuração válida");
        cfg.gear.x_main_m = 3.85;                       // valor pré-E6 — causa raiz original
        cfg.empennage.v_h = 0.70;                       // valor pré-E6
        // Task refino-ciclo2: `[stability].cl_h_max_down` foi REMOVIDO (a
        // autoridade agora é CALCULADA por geometria) — para reproduzir uma
        // autoridade reduzida equivalente ao antigo palpite pré-E6 (0.85),
        // reduz a corda do profundor (`elevator_chord_frac` 0.40→0.26 no
        // valor ATUAL do dial, τ menor — o histórico de reajustes do dial
        // vem logo abaixo).
        //
        // Ciclo 7 (task 1): a corda desta MUTAÇÃO passou de 0.30 para 0.28.
        // Motivo: com a rotação usando o CLmax de DECOLAGEM (`cl_max_to`),
        // a Vr correta é 4,2% maior, `q_r` 8,5% maior e TODO limite de
        // rotação avança ~4 pp — inclusive o desta config mutada, que caiu
        // de ≈38,9% para ≈35,7% e deixou de ficar À FRENTE do limite
        // traseiro (36,6%), ou seja, deixou de reproduzir o envelope VAZIO
        // que este teste existe para guardar. 0.28 dá
        // cl_h_max_down_calc≈0.841 — que, além de restaurar o achado
        // (≈37,2% > 36,6%), é uma reprodução MELHOR do palpite pré-E6
        // original (0.85) do que o 0.30 anterior (≈0.880). O achado
        // histórico é o mesmo; só o dial da mutação foi reajustado à
        // física corrigida. (Varredura empírica: 0.30→35,7% fechado;
        // 0.28→37,2% vazio; 0.26→38,7% vazio.)
        //
        // Campanha E10 (2026-08-08): a corda desta MUTAÇÃO passa de 0.28
        // para 0.26 — MESMO mecanismo do reajuste do ciclo 7, `cl_max_to`
        // de novo (mais o `Cm_TO`), NÃO o recuo de CG da bateria. Vale
        // insistir porque é contraintuitivo: `rotation_fwd_limit_m` não
        // recebe CG, massa nem `x_nose_m` — é invariante ao peso (prova
        // algébrica na docstring da função, prova numérica em
        // `rotation_limit_e_invariante_a_massas_diferentes`). Os ÚNICOS
        // parâmetros de E10 que o alcançam são `cl_max_flaps` e
        // `to_flap_fraction`, pelos dois termos que eles governam:
        //   M/W ∝ (1/cl_max_to)·[A + B + Cm_TO·S_w·MAC]
        //   cl_max_to = cl_max_clean + to_flap_fraction·(cl_max_flaps − cl_max_clean)
        //   Cm_TO     = cm_ac + to_flap_fraction·cm_flap_delta
        // (A = termo de profundor, B = termo de sustentação na corrida;
        // ambos só geometria/autoridade — é onde o DIAL desta mutação age.)
        // E10 move os dois em direções OPOSTAS: `cl_max_to` 1,585→1,6775
        // (+5,8%) reduz `q_r` e portanto M/W, RECUANDO o limite; `Cm_TO`
        // −0,158→−0,113 (menos flap na decolagem, menos nariz-para-baixo)
        // aumenta o colchete e M/W, AVANÇANDO o limite. No baseline real o
        // segundo ganha por pouco (8,908%→8,533% MAC, hand-check fechado no
        // pin de `rotation_limit_pct_mac` acima). NESTA config mutada o
        // segundo ganha por MUITO mais: com `v_h` 0,70 e a corda de
        // profundor reduzida, o termo A encolhe, então o termo de `Cm_TO`
        // pesa relativamente mais — o limite cai de ≈37,2% para 36,09% MAC,
        // ATRÁS do limite traseiro (36,615%). Ou seja: o envelope volta a
        // FECHAR e o achado histórico que este teste existe para guardar
        // desaparece. Varredura empírica NOVA (config E10 mutada; o limite
        // traseiro não se move, depende só de NP/`sm_min`):
        //   c_e/c=0.30 → rot 34,709%  (fechado)
        //   c_e/c=0.28 → rot 36,092%  (fechado — o dial do ciclo 7)
        //   c_e/c=0.27 → rot 36,803%  (vazio, mas por só 0,19 pp)
        //   c_e/c=0.26 → rot 37,526%  (vazio, 0,91 pp — ESCOLHIDO)
        //   c_e/c=0.24 → rot 39,017%  (vazio, exagerado)
        // 0.26 dá `cl_h_max_down_calc≈0.800` — ainda uma reprodução
        // razoável do palpite pré-E6 original (0.85; o 0.28 do ciclo 7 dava
        // 0.841) e com folga estável sobre o limite traseiro. 0.27 também
        // restauraria o achado, mas por uma margem fina demais para servir
        // de guarda. O achado histórico é o MESMO; só o dial da mutação foi
        // reajustado à nova geometria de massas do baseline.
        cfg.control_surfaces.elevator_chord_frac = 0.26;
        let req = crate::models::requirements::test_fixtures::requisitos_teste();
        let state = crate::models::aircraft_state::AircraftState::from_config(&cfg);
        let wing = crate::agents::aerodynamics::AerodynamicsAgent::run(&state, &req);
        let emp = crate::agents::empennage::EmpennageAgent::run(&wing, &cfg);
        let engine = crate::models::engine::test_fixtures::motor_generico_teste();
        let masses = masses_do_baseline(&cfg, &engine, &req, &wing, &emp, &state);
        let wb = crate::agents::weight_balance::WeightBalanceAgent::run(
            &state, &wing, &engine, &cfg, &req, &emp, &masses,
        );

        let prop = crate::agents::propulsion::PropulsionAgent::run(&state, &req, &wing, &engine);
        let trim = TrimAuthorityAgent::run(&cfg, &wing, &emp, &wb, &state, &engine, &req,
                                           prop.thrust_cruise_n);

        // Achado honesto: com os parâmetros pré-E6, a rotação governa E
        // fica à frente do limite traseiro (envelope vazio).
        assert!(trim.rotation_limit_pct_mac > trim.flare_limit_pct_mac);
        assert_eq!(trim.governing, "rotacao",
            "governing deveria ser 'rotacao' (achado honesto pré-E6)");
        assert!(trim.rotation_limit_pct_mac > wb.spec.cg_limit_aft_pct_mac,
            "limite de rotação ({:.2}%) deveria ficar À FRENTE do limite traseiro ({:.2}%) — \
             envelope de CG vazio com parâmetros pré-E6",
            trim.rotation_limit_pct_mac, wb.spec.cg_limit_aft_pct_mac);

        assert!(trim.rotation_margin_per_scenario.iter()
                .any(|sc| sc.rotation_authority_margin_pct < 0.0),
            "com parâmetros pré-E6, ao menos um cenário deveria ter margem de autoridade de \
             rotação NEGATIVA: {:?}",
            trim.rotation_margin_per_scenario.iter()
                .map(|sc| (sc.scenario.as_str(), sc.rotation_authority_margin_pct))
                .collect::<Vec<_>>());
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
        let masses = masses_do_baseline(&cfg, &engine, &req, &wing, &emp, &state);
        let wb = crate::agents::weight_balance::WeightBalanceAgent::run(
            &state, &wing, &engine, &cfg, &req, &emp, &masses,
        );

        let prop = crate::agents::propulsion::PropulsionAgent::run(&state, &req, &wing, &engine);
        let trim = TrimAuthorityAgent::run(&cfg, &wing, &emp, &wb, &state, &engine, &req,
                                           prop.thrust_cruise_n);
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

    // ─── Arrasto de trim em cruzeiro (Task 4, refino-ciclo2) ──────────────
    //
    // Hand-check com valores REAIS do runtime pós-refino-ciclo2 (Tasks 1-3
    // já aplicadas — geometria da empenagem/asa não muda, só os checks
    // novos foram adicionados): baseline real (motor padrão + missão de
    // projeto), MTOW convergido, cenário "4 pax + bagagem + meia": x̄_cg=35,444372%
    // MAC=0,35444372, CL_cruise=0,38847654, cm_ac=−0,008, MAC=1,24631614m,
    // l_h=4,80m ⟹ l_h/MAC=3,8513503; S_h/S_w=3,13396578/14,2=0,2207018;
    // η_h=0,90; e_h=0,70; ar_h=4,0 (ver task-4-report.md para a dedução
    // completa em Python).
    //
    //   num = −0,008+0,38847654·(0,35444372−0,25) = 0,0325739
    //   den = 0,90·0,2207018·(3,8513503+0,25−0,35444372) = 0,744254
    //   CL_h_trim = 0,0325739/0,744254 ≈ +0,043767  (upload — CG atrás do CA)
    //   ΔCD_trim = (0,043767²/(π·4·0,70))·0,2207018 ≈ 4,806e-5

    #[test]
    fn cl_h_trim_cruise_hand_check_baseline_meia_missao() {
        let mac = 1.24631614_f64;
        let l_h_over_mac = 4.80 / mac;
        let s_ratio = 3.13396578 / 14.2;
        // `cm_thrust = 0,0` — hand-check da parte pré-ciclo-10, preservada
        // bit-a-bit; o termo NOVO tem hand-check próprio em
        // `cm_thrust_cruise_hand_check_com_literais`.
        let cl_h_trim = cl_h_trim_cruise(-0.008, 0.38847654, 0.35444372, 0.90, s_ratio,
                                          l_h_over_mac, 0.0);
        println!("cl_h_trim (meia-missão) = {cl_h_trim:.6}");
        assert!((cl_h_trim - 0.043767).abs() < 1e-4,
            "cl_h_trim = {cl_h_trim:.6} (esperado ≈0.043767 ±1e-4)");
        assert!(cl_h_trim > 0.0,
            "CG atrás do CA (x̄=0.354 > 0.25) deveria produzir CL_h_trim POSITIVO (upload) — \
             obtido {cl_h_trim:.6}");
    }

    #[test]
    fn cd_trim_cruise_hand_check_baseline_meia_missao() {
        let s_ratio = 3.13396578 / 14.2;
        let cd_trim = cd_trim_cruise(0.043767, 4.0, 0.70, s_ratio);
        println!("cd_trim (meia-missão) = {cd_trim:.8}");
        assert!((cd_trim - 4.806e-5).abs() < 1e-6,
            "cd_trim = {cd_trim:.8} (esperado ≈4.806e-5 ±1e-6)");
    }

    /// CG à FRENTE do CA (x̄_cg < 0,25, ex.: cenário "Solo (piloto)" do
    /// baseline real, x̄≈15,59% MAC) inverte o sinal — a cauda precisa
    /// gerar DOWNLOAD (CL_h_trim negativo) para equilibrar o momento de
    /// peso picando NARIZ-PARA-BAIXO. Hand-check: x̄=0,15592128 →
    /// CL_h_trim≈−0,056843, ΔCD_trim≈8,107e-5 (maior que o de meia-missão —
    /// CG mais longe do CA exige mais autoridade de trim).
    #[test]
    fn cl_h_trim_cruise_hand_check_cg_dianteiro_e_negativo() {
        let mac = 1.24631614_f64;
        let l_h_over_mac = 4.80 / mac;
        let s_ratio = 3.13396578 / 14.2;
        let cl_h_trim = cl_h_trim_cruise(-0.008, 0.38847654, 0.15592128, 0.90, s_ratio,
                                          l_h_over_mac, 0.0);
        println!("cl_h_trim (CG dianteiro) = {cl_h_trim:.6}");
        assert!((cl_h_trim - (-0.056843)).abs() < 1e-4,
            "cl_h_trim = {cl_h_trim:.6} (esperado ≈-0.056843 ±1e-4)");
        assert!(cl_h_trim < 0.0,
            "CG à frente do CA (x̄=0.156 < 0.25) deveria produzir CL_h_trim NEGATIVO \
             (download) — obtido {cl_h_trim:.6}");

        let cd_trim = cd_trim_cruise(cl_h_trim, 4.0, 0.70, s_ratio);
        println!("cd_trim (CG dianteiro) = {cd_trim:.8}");
        assert!((cd_trim - 8.107e-5).abs() < 1e-6,
            "cd_trim = {cd_trim:.8} (esperado ≈8.107e-5 ±1e-6)");
    }

    /// Propriedade: `cd_trim_cruise` depende de `CL_h_trim²` — o sinal do
    /// CL_h_trim não importa, só a magnitude. Um download (negativo) e um
    /// upload (positivo) de mesma MAGNITUDE devem produzir o MESMO
    /// ΔCD_trim.
    #[test]
    fn cd_trim_cruise_e_simetrico_no_sinal_de_cl_h_trim() {
        let s_ratio = 0.22;
        let cd_pos = cd_trim_cruise(0.05, 4.0, 0.70, s_ratio);
        let cd_neg = cd_trim_cruise(-0.05, 4.0, 0.70, s_ratio);
        println!("cd_trim(+0.05)={cd_pos:.8}  cd_trim(-0.05)={cd_neg:.8}");
        assert!((cd_pos - cd_neg).abs() < 1e-12,
            "ΔCD_trim deveria ser simétrico no sinal de CL_h_trim: +0.05→{cd_pos:.8} \
             -0.05→{cd_neg:.8}");
    }

    // ─── CICLO 10 (task 2): linha de tração no trim de cruzeiro ───────────

    /// Hand-check do `cm_thrust` com LITERAIS (valores do baseline E10):
    ///   T = 1.200,85 N, z_cg = 0,20 m, q = 2.500 Pa, S_w = 14,2 m²,
    ///   MAC = 1,24631614 m
    ///   q·S·MAC = 2.500 · 14,2 · 1,24631614 = 44.244,22 N·m
    ///   cm_thrust = −1.200,85 · 0,20 / 44.244,22 = −0,00542805
    ///
    /// SINAL: NEGATIVO (nariz-abaixo) para eixo ACIMA do CG e tração para a
    /// frente. Comparável em magnitude ao próprio `cm_ac` (−0,008) do
    /// baseline — não é um termo desprezível.
    #[test]
    fn cm_thrust_cruise_hand_check_com_literais() {
        let cm_t = cm_thrust_cruise(1200.85, 0.20, 2500.0, 14.2, 1.24631614);
        println!("cm_thrust = {cm_t:.8} (esperado ≈-0.00542828)");
        assert!(cm_t < 0.0,
            "eixo ACIMA do CG + tração para a frente deveria dar Cm NEGATIVO (nariz-abaixo) — \
             obtido {cm_t:.8}");
        assert!((cm_t - (-0.00542828)).abs() < 1e-7,
            "cm_thrust = {cm_t:.8} (esperado ≈-0.00542828 ±1e-7)");

        // Eixo ABAIXO do CG (offset negativo — a faixa de config permite até
        // −0,3 m) inverte o sinal: tração abaixo do CG é nariz-ACIMA.
        let cm_t_baixo = cm_thrust_cruise(1200.85, -0.20, 2500.0, 14.2, 1.24631614);
        assert!((cm_t_baixo + cm_t).abs() < 1e-12,
            "offset simétrico deveria dar Cm simétrico: {cm_t_baixo:.8} vs {cm_t:.8}");
        assert!(cm_t_baixo > 0.0, "eixo ABAIXO do CG deveria dar Cm POSITIVO (nariz-acima)");

        // Tração nula ⟹ termo nulo ⟹ modelo pré-ciclo-10 exato.
        assert_eq!(cm_thrust_cruise(0.0, 0.20, 2500.0, 14.2, 1.24631614), 0.0);
    }

    /// Property ESTRITA de direção: `cm_thrust` NEGATIVO (nariz-abaixo, o
    /// caso físico do eixo acima do CG) move `cl_h_trim_cruise` na direção
    /// NEGATIVA (mais download / menos upload na empenagem).
    ///
    /// Auditoria do sinal: no balanço `Σ M_cg = 0`, o termo da empenagem
    /// entra como `−η_h·(S_h/S_w)·CL_h·(l_h/MAC+0,25−x̄)` — ou seja, um
    /// `CL_h` POSITIVO (upload) produz momento nariz-ABAIXO. Para conter um
    /// momento nariz-abaixo EXTRA (o da tração), a empenagem precisa ir na
    /// direção OPOSTA: `CL_h` mais NEGATIVO (download), que é nariz-acima.
    /// Consequência de projeto: o arrasto de trim NÃO cai monotonicamente —
    /// `cd_trim ∝ CL_h²`, então empurrar `CL_h` para o negativo pode
    /// primeiro REDUZIR o arrasto (se `CL_h` era positivo) e depois
    /// aumentá-lo; ver `cd_trim_cruise_e_simetrico_no_sinal_de_cl_h_trim`.
    #[test]
    fn cm_thrust_negativo_reduz_cl_h_trim_cruise() {
        let mac = 1.24631614_f64;
        let l_h_over_mac = 4.80 / mac;
        let s_ratio = 3.13396578 / 14.2;

        let sem = cl_h_trim_cruise(-0.008, 0.38847654, 0.35444372, 0.90, s_ratio,
                                    l_h_over_mac, 0.0);
        let com = cl_h_trim_cruise(-0.008, 0.38847654, 0.35444372, 0.90, s_ratio,
                                    l_h_over_mac, -0.00542828);
        println!("cl_h_trim sem tração = {sem:.6}  com cm_thrust=-0.00542828 = {com:.6}");
        assert!(com < sem,
            "cm_thrust NEGATIVO (nariz-abaixo) deveria mover cl_h_trim ESTRITAMENTE na direção \
             NEGATIVA (mais download): sem={sem:.6} com={com:.6}");

        // Magnitude fechada: o deslocamento é exatamente cm_thrust/den.
        let den = 0.90 * s_ratio * (l_h_over_mac + 0.25 - 0.35444372);
        assert!(((com - sem) - (-0.00542828 / den)).abs() < 1e-12,
            "Δcl_h_trim deveria ser exatamente cm_thrust/den = {:.9} — obtido {:.9}",
            -0.00542828 / den, com - sem);

        // E o simétrico: cm_thrust POSITIVO (eixo abaixo do CG) sobe.
        let com_pos = cl_h_trim_cruise(-0.008, 0.38847654, 0.35444372, 0.90, s_ratio,
                                        l_h_over_mac, 0.00542828);
        assert!(com_pos > sem,
            "cm_thrust POSITIVO deveria mover cl_h_trim na direção POSITIVA: sem={sem:.6} \
             com={com_pos:.6}");
    }

    /// Propriedade: `cd_trim_cruise` aumenta ESTRITAMENTE quando `e_h`
    /// (eficiência de Oswald da empenagem) DIMINUI — menos eficiência ⟹
    /// mais arrasto induzido para o mesmo CL_h_trim.
    #[test]
    fn cd_trim_cruise_aumenta_quando_e_h_diminui() {
        let s_ratio = 0.22;
        let cd_alto_e_h = cd_trim_cruise(0.05, 4.0, 0.90, s_ratio);
        let cd_baixo_e_h = cd_trim_cruise(0.05, 4.0, 0.55, s_ratio);
        println!("cd_trim(e_h=0.90)={cd_alto_e_h:.8}  cd_trim(e_h=0.55)={cd_baixo_e_h:.8}");
        assert!(cd_baixo_e_h > cd_alto_e_h,
            "cd_trim com e_h menor ({cd_baixo_e_h:.8}) deveria ser MAIOR que com e_h maior \
             ({cd_alto_e_h:.8})");
    }
}


