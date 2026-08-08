/// PerformanceAgent — Desempenho Completo da Aeronave
///
/// Calcula:
///   - Razão de subida (RC) e ângulo de subida em diferentes condições
///   - Velocidades ótimas de subida (Vx = melhor ângulo, Vy = melhor razão)
///   - Melhor planeio (V_bg, L/Dmax) e gradiente de subida (CS 23.65, Task 4.7)
///   - Distâncias de decolagem e pouso (pista pavimentada e gramado/terra),
///     tanto a estimativa simplificada baseada em rolagem de solo quanto a
///     distância física sobre obstáculo de 15m/50ft por segmentos (Task 4.7)
///   - Teto de serviço (RC = 0,5 m/s residual)
///   - Hodógrafo de voo (envelope de desempenho)
///
/// Todas as velocidades em m/s internamente; apresentadas em km/h na saída.
///
/// Referências:
///   - Anderson, J. "Introduction to Flight", Cap. 6
///   - Raymer, D. "Aircraft Design", Cap. 5 (takeoff e landing)
///   - RBAC 23 / CS-23 — requisitos de desempenho categoria Normal

use crate::models::atmosphere::Isa;
use crate::models::engine::EngineSpec;
use crate::models::requirements::Requirements;
use crate::models::specs::{WingSpec, PropulsionSpec, PerformanceSpec};
use crate::agents::propulsion::{prop_rpm, prop_efficiency, advance_ratio, thrust_n};
use crate::models::aircraft_state::AircraftState;
use crate::models::aircraft_config::PerformanceCfg;

const G: f64 = 9.807;   // m/s²

// ─── POTÊNCIA E TRAÇÃO DISPONÍVEIS ────────────────────────────────────────────

/// Potência de eixo disponível em altitude a RPM de cruzeiro (kW)
///
/// `psru_efficiency`: eficiência mecânica do PSRU (`AircraftState::
/// psru_efficiency`, dado de configuração — `[propeller].psru_efficiency`).
/// Finding 1 da revisão final: antes vinha de um `const PSRU_EFFICIENCY`
/// hardcoded em `agents::propulsion` que ignorava este campo do TOML.
pub fn shaft_power_kw(engine: &EngineSpec, engine_rpm: f64, altitude_m: f64,
                       psru_efficiency: f64) -> f64 {
    engine.power_kw_at(engine_rpm, altitude_m) * psru_efficiency
}

/// Tração estática IDEAL (Rankine-Froude, disco atuador) — sem correção
/// empírica. A teoria de disco atuador superestima a tração real por não
/// modelar perdas de ponta de pá, rotação de esteira e não-uniformidade da
/// distribuição de carga ao longo da pá.
fn static_thrust_ideal_n(engine: &EngineSpec, engine_rpm: f64, prop_diam_m: f64,
                          altitude_m: f64, isa_delta_c: f64, psru_efficiency: f64) -> f64 {
    let p_w = shaft_power_kw(engine, engine_rpm, altitude_m, psru_efficiency) * 1_000.0;
    let rho = Isa::density_kgm3(altitude_m, isa_delta_c);
    let disk_area = std::f64::consts::PI * (prop_diam_m / 2.0).powi(2);
    (2.0 * rho * disk_area * p_w * p_w).powf(1.0 / 3.0)
}

/// Tração disponível da hélice em função da velocidade e altitude.
///
/// `isa_delta_c`: desvio ISA da missão — usado no ramo de tração estática
/// (Rankine-Froude).
///
/// `static_thrust_factor` (Task 4.7, config `[performance]`): fator empírico
/// (McCormick, tipicamente ≈0.75) aplicado sobre a tração estática IDEAL —
/// ver `static_thrust_ideal_n`. Ignorado no ramo de voo (V ≥ 0,5 m/s), que já
/// usa o modelo de disco atuador com eficiência de hélice (`prop_efficiency`),
/// fisicamente distinto e já calibrado.
pub fn thrust_available_n(
    v_ms: f64,
    engine: &EngineSpec,
    engine_rpm: f64,
    psru_ratio: f64,
    prop_diam_m: f64,
    altitude_m: f64,
    isa_delta_c: f64,
    static_thrust_factor: f64,
    psru_efficiency: f64,
) -> f64 {
    if v_ms < 0.5 {
        // Tração estática (solo): estimativa por impulso de disco
        // (Rankine-Froude) × fator empírico de correção.
        return static_thrust_ideal_n(engine, engine_rpm, prop_diam_m, altitude_m, isa_delta_c,
                                      psru_efficiency)
            * static_thrust_factor;
    }
    let n_prop = prop_rpm(engine_rpm, psru_ratio);
    let j = advance_ratio(v_ms, n_prop, prop_diam_m);
    let eta = prop_efficiency(j);
    let p_shaft_w = shaft_power_kw(engine, engine_rpm, altitude_m, psru_efficiency) * 1_000.0;
    thrust_n(eta, p_shaft_w, v_ms)
}

// ─── ARRASTO E POTÊNCIA NECESSÁRIA ───────────────────────────────────────────

/// Arrasto total em Newton para voo nivelado a V_ms e MTOW_kg
pub fn drag_level_n(v_ms: f64, mass_kg: f64, rho: f64, wing: &WingSpec) -> f64 {
    let q = 0.5 * rho * v_ms * v_ms;
    let cl = (mass_kg * G) / (q * wing.area_m2);
    let cdi = cl * cl / (std::f64::consts::PI * wing.aspect_ratio * wing.oswald_efficiency);
    let cd = wing.cd0 + cdi;
    q * wing.area_m2 * cd
}

/// Potência necessária para voo nivelado (kW)
/// P_req = D·V  (sem divisão por eficiência — potência aerodinâmica bruta)
pub fn power_required_kw(v_ms: f64, drag_n: f64) -> f64 {
    drag_n * v_ms / 1_000.0
}

// ─── RAZÃO DE SUBIDA ──────────────────────────────────────────────────────────

/// Excesso de potência disponível sobre a necessária (kW)
pub fn excess_power_kw(
    v_ms: f64,
    mass_kg: f64,
    rho: f64,
    wing: &WingSpec,
    engine: &EngineSpec,
    engine_rpm: f64,
    psru_ratio: f64,
    prop_diam_m: f64,
    altitude_m: f64,
    isa_delta_c: f64,
    static_thrust_factor: f64,
    psru_efficiency: f64,
) -> f64 {
    let drag   = drag_level_n(v_ms, mass_kg, rho, wing);
    let p_req  = power_required_kw(v_ms, drag);
    let thrust = thrust_available_n(v_ms, engine, engine_rpm, psru_ratio, prop_diam_m,
                                     altitude_m, isa_delta_c, static_thrust_factor,
                                     psru_efficiency);
    let p_avail = thrust * v_ms / 1_000.0;
    p_avail - p_req
}

/// Razão de subida máxima (m/s) — varre velocidades para encontrar o pico
/// RC = P_excess / W
///
/// Nota de modelo (pré-existente, não introduzida por esta função): a
/// referência de estol usa `wing.cl_max`, que é o CL_max COM FLAP
/// (`cl_max_flaps` — ver `WingSpec::cl_max`/`aerodynamics.rs`), não o CL_max
/// limpo. Já `excess_power_kw`→`drag_level_n` usa a polar de arrasto
/// (`wing.cd0`) SEM nenhum incremento de arrasto de flap — não existe modelo
/// de flap na polar deste crate. O resultado é um híbrido "CL de estol
/// flapado + arrasto limpo", não uma condição limpa nem uma condição de
/// decolagem/pouso fisicamente consistente. Documentado como limitação
/// conhecida do gate CS 23.65 (`ConstraintChecker`), que consome
/// `climb_gradient_pct` calculado sobre este mesmo híbrido (ver
/// `best_climb_angle_ms` abaixo).
pub fn climb_rate_ms(
    mass_kg: f64,
    altitude_m: f64,
    isa_delta_c: f64,
    wing: &WingSpec,
    state: &AircraftState,
    engine: &EngineSpec,
    static_thrust_factor: f64,
) -> (f64, f64) {
    let rho = Isa::density_kgm3(altitude_m, isa_delta_c);
    // RPM de subida: máximo contínuo do motor (uso prolongado, não redline).
    let engine_rpm_climb = engine.rpm_max_continuous;

    let v_stall = ((2.0 * mass_kg * G) / (rho * wing.area_m2 * wing.cl_max)).sqrt();

    // Varre de 1.3·Vs até 1.8·Vs (faixa de Vy típica)
    let v_min = 1.30 * v_stall;
    let v_max = 1.80 * v_stall;
    let steps = 50;
    let dv = (v_max - v_min) / steps as f64;

    let mut best_rc = f64::NEG_INFINITY;
    let mut best_v  = v_min;

    for i in 0..=steps {
        let v = v_min + i as f64 * dv;
        let pex = excess_power_kw(v, mass_kg, rho, wing, engine,
                                   engine_rpm_climb, state.psru_ratio,
                                   state.prop_diameter_m, altitude_m, isa_delta_c,
                                   static_thrust_factor, state.psru_efficiency);
        let rc = pex * 1_000.0 / (mass_kg * G);
        if rc > best_rc {
            best_rc = rc;
            best_v  = v;
        }
    }
    (best_rc.max(0.0), best_v * 3.6) // (RC em m/s, Vy em km/h)
}

/// Vx — velocidade de MELHOR ÂNGULO de subida (Task 4.7): maximiza o
/// gradiente RC(V)/V = sin(γ), equivalente a maximizar (T−D)/W — não RC(V)
/// absoluto (isso é Vy, ver `climb_rate_ms`). Fisicamente Vx < Vy sempre (a
/// curva RC/V tem pico mais próximo do stall que a curva RC).
///
/// Varredura de 1.05·Vs a 1.8·Vs — faixa inferior mais baixa que
/// `climb_rate_ms` (1.3·Vs) para não truncar o pico de gradiente, que tende
/// a ocorrer mais perto do stall.
///
/// Retorna `(gradiente_max, Vx_kmh)` — gradiente como FRAÇÃO adimensional
/// (RC/V, não %); `PerformanceAgent::run` converte para `climb_gradient_pct`.
///
/// Mesma nota de modelo de `climb_rate_ms`: a referência de estol usa
/// `wing.cl_max` — CL_max COM FLAP, não limpo — enquanto o arrasto somado
/// (via `excess_power_kw`) não tem nenhum incremento de flap. É um híbrido
/// "CL de estol flapado + arrasto limpo" herdado de `climb_rate_ms`/Vy, não
/// uma condição de decolagem CS 23.65 fisicamente consistente (que exigiria
/// flap de decolagem tanto no CL_max de referência quanto no CD0). O
/// gradiente resultante (`climb_gradient_pct`) é checado contra o piso de
/// 8,3% da CS 23.65 sobre ESTE híbrido — ver limitação documentada em
/// `task-4.7-report.md`.
pub fn best_climb_angle_ms(
    mass_kg: f64,
    altitude_m: f64,
    isa_delta_c: f64,
    wing: &WingSpec,
    state: &AircraftState,
    engine: &EngineSpec,
    static_thrust_factor: f64,
) -> (f64, f64) {
    let rho = Isa::density_kgm3(altitude_m, isa_delta_c);
    let engine_rpm_climb = engine.rpm_max_continuous;

    let v_stall = ((2.0 * mass_kg * G) / (rho * wing.area_m2 * wing.cl_max)).sqrt();

    let v_min = 1.05 * v_stall;
    let v_max = 1.80 * v_stall;
    let steps = 80;
    let dv = (v_max - v_min) / steps as f64;

    let mut best_grad = f64::NEG_INFINITY;
    let mut best_v = v_min;

    for i in 0..=steps {
        let v = v_min + i as f64 * dv;
        let pex = excess_power_kw(v, mass_kg, rho, wing, engine,
                                   engine_rpm_climb, state.psru_ratio,
                                   state.prop_diameter_m, altitude_m, isa_delta_c,
                                   static_thrust_factor, state.psru_efficiency);
        let rc = pex * 1_000.0 / (mass_kg * G);
        let grad = rc / v;
        if grad > best_grad {
            best_grad = grad;
            best_v = v;
        }
    }
    (best_grad.max(0.0), best_v * 3.6) // (gradiente adimensional, Vx em km/h)
}

/// Teto de serviço: altitude onde RC = 0,5 m/s (padrão CS-23)
pub fn service_ceiling_m(
    mass_kg: f64,
    isa_delta_c: f64,
    wing: &WingSpec,
    state: &AircraftState,
    engine: &EngineSpec,
    static_thrust_factor: f64,
) -> f64 {
    let mut alt = 0.0_f64;
    let step = 100.0_f64;
    loop {
        let (rc, _) = climb_rate_ms(mass_kg, alt, isa_delta_c, wing, state, engine,
                                     static_thrust_factor);
        if rc <= 0.5 || alt > 8_000.0 {
            return alt;
        }
        alt += step;
    }
}

/// Melhor planeio (Task 4.7) — voo NÃO propulsado (motor cortado), portanto
/// sem dependência de tração/motor.
///
///   `V_bg = √(2W/(ρS))·(K/CD0)^0.25`
///   `L/D_max = 1/(2√(K·CD0))`
///   `K = 1/(π·AR·e)`
///
/// Usa `wing.cd0` — o CD0 TOTAL da aeronave (asa+fuselagem+empenagem+trem+
/// misc, ver `aerodynamics::cd0_total`), não um "CD0 de perfil" isolado: com
/// trem retrátil recolhido (`CD0_GEAR_RETRACTABLE = 0.0`), `wing.cd0` JÁ
/// representa a configuração limpa apropriada para planeio.
///
/// Retorna `(V_bg em km/h, L/D_max adimensional)`.
pub fn best_glide(mass_kg: f64, rho: f64, wing: &WingSpec) -> (f64, f64) {
    let w = mass_kg * G;
    let k = 1.0 / (std::f64::consts::PI * wing.aspect_ratio * wing.oswald_efficiency);
    let ld_max = 1.0 / (2.0 * (k * wing.cd0).sqrt());
    let v_bg_ms = ((2.0 * w) / (rho * wing.area_m2)).sqrt() * (k / wing.cd0).powf(0.25);
    (v_bg_ms * 3.6, ld_max)
}

// ─── DISTÂNCIAS DE DECOLAGEM E POUSO ─────────────────────────────────────────

/// Rolagem de solo na decolagem (método energético de Raymer, Cap. 5):
///
///   S_G = W² / (g · ρ · S · CL_TO · T_avg)
///     onde CL_TO = 0.8·CL_max (flap parcial)
///
/// `T_avg` (Task 4.7): tração ESTÁTICA corrigida (Rankine-Froude × fator
/// empírico `static_thrust_factor`, ver `thrust_available_n`/
/// `static_thrust_ideal_n`) — substitui a estimativa ad hoc anterior ("80%
/// da tração a V_lo/2"). Isso tende a ALONGAR a rolagem de solo (a tração
/// estática corrigida é, para esta célula/hélice/motor, menor que a antiga
/// estimativa em V_lo/2) — ver task-4.7-report.md para a tabela antes/depois.
fn takeoff_ground_roll_m(
    mass_kg: f64,
    rho: f64,
    wing: &WingSpec,
    state: &AircraftState,
    engine: &EngineSpec,
    isa_delta_c: f64,
    static_thrust_factor: f64,
) -> f64 {
    let w = mass_kg * G;
    let cl_to = 0.80 * wing.cl_max; // flap parcial na decolagem
    let t_avg = thrust_available_n(
        0.0, // tração estática (solo, V=0)
        engine,
        engine.rpm_max_continuous,
        state.psru_ratio,
        state.prop_diameter_m,
        0.0, // nível do mar
        isa_delta_c,
        static_thrust_factor,
        state.psru_efficiency,
    );
    w * w / (G * rho * wing.area_m2 * cl_to * t_avg)
}

/// Distância de decolagem — rolagem de solo × 1,5 (aproximação de
/// transição de Raymer) × fator de superfície. MANTIDA como estimativa
/// simplificada baseada em rolagem de solo (ver `PerformanceSpec::
/// to_distance_paved_m`/`to_distance_grass_m`) — a distância física sobre
/// obstáculo de 15m (50 ft) por segmentos vive em
/// `takeoff_distance_50ft_m` (Task 4.7), que NÃO usa este fator ad hoc de
/// 1,5.
///
/// Fator de superfície:
///   pista pavimentada seca: 1.00
///   gramado firme:          1.15–1.20
///   terra compactada:       1.25
pub fn takeoff_distance_m(
    mass_kg: f64,
    rho: f64,
    wing: &WingSpec,
    state: &AircraftState,
    surface_factor: f64,
    engine: &EngineSpec,
    isa_delta_c: f64,
    static_thrust_factor: f64,
) -> f64 {
    let s_ground = takeoff_ground_roll_m(mass_kg, rho, wing, state, engine, isa_delta_c,
                                          static_thrust_factor);
    s_ground * 1.5 * surface_factor
}

/// Distância de decolagem sobre obstáculo de 15m (50 ft), por segmentos
/// (Task 4.7) — substitui o fator ad hoc de transição ×1,5 por física real:
///
///   S_total = S_ground + S_rotação + S_subida
///     S_ground:  rolagem de solo (`takeoff_ground_roll_m`) × fator de
///                superfície (a superfície só afeta o atrito no solo, não
///                a rotação/subida no ar)
///     S_rotação: V_LOF × `rotation_time_s` (rotação a V_LOF ≈ constante)
///     S_subida:  15 / tan(γ), γ = arcsin(RC/V) avaliado a 1,2·V_s (flap
///                decolagem), potência de decolagem, nível do mar
pub fn takeoff_distance_50ft_m(
    mass_kg: f64,
    rho: f64,
    wing: &WingSpec,
    state: &AircraftState,
    surface_factor: f64,
    engine: &EngineSpec,
    isa_delta_c: f64,
    perf_cfg: &PerformanceCfg,
) -> f64 {
    let s_ground = takeoff_ground_roll_m(mass_kg, rho, wing, state, engine, isa_delta_c,
                                          perf_cfg.static_thrust_factor) * surface_factor;

    let w = mass_kg * G;
    let v_s_to = ((2.0 * w) / (rho * wing.area_m2 * wing.cl_max)).sqrt();
    let v_lo = 1.10 * v_s_to;
    let s_rotation = v_lo * perf_cfg.rotation_time_s;

    let v_climb = 1.20 * v_s_to;
    let engine_rpm_to = engine.rpm_max_continuous;
    let pex = excess_power_kw(v_climb, mass_kg, rho, wing, engine, engine_rpm_to,
                               state.psru_ratio, state.prop_diameter_m, 0.0, isa_delta_c,
                               perf_cfg.static_thrust_factor, state.psru_efficiency);
    let rc = pex * 1_000.0 / (mass_kg * G);
    if rc <= 0.0 {
        // Não consegue sustentar subida nesta condição — obstáculo
        // inatingível (distância "infinita" em vez de um número espúrio).
        return s_ground + s_rotation + f64::INFINITY;
    }
    let gamma = (rc / v_climb).clamp(-1.0, 1.0).asin();
    let s_climb = 15.0 / gamma.tan();

    s_ground + s_rotation + s_climb
}

/// Rolagem de solo no pouso (frenagem):
///
///   S_G = V_ref² / (2·g·μ)     V_ref = 1.3·V_s
///
/// `mu_brake` (Task 4.7, config `[performance]`): vem diretamente de
/// `mu_brake_paved`/`mu_brake_grass` — substitui os fatores ad hoc
/// `μ/surface_factor` e `·√surface_factor` do M5 (para pista pavimentada,
/// factor=1.0, os dois hacks eram identidade — `mu_brake_paved = 0.40`
/// reproduz numericamente o resultado antigo).
fn landing_ground_roll_m(mass_kg: f64, rho: f64, wing: &WingSpec, mu_brake: f64) -> f64 {
    let w = mass_kg * G;
    let v_s = ((2.0 * w) / (rho * wing.area_m2 * wing.cl_max)).sqrt();
    let v_ref = 1.30 * v_s;
    v_ref * v_ref / (2.0 * G * mu_brake)
}

/// Distância de pouso — rolagem de solo + distância aérea fixa de 200 m
/// (aproximação simplificada, típica para esta classe). MANTIDA como
/// estimativa baseada em rolagem de solo (`PerformanceSpec::
/// landing_distance_m`) — a distância física sobre obstáculo de 15m (50 ft)
/// por segmentos vive em `landing_distance_50ft_m` (Task 4.7).
pub fn landing_distance_m(mass_kg: f64, rho: f64, wing: &WingSpec, mu_brake: f64) -> f64 {
    let s_ground = landing_ground_roll_m(mass_kg, rho, wing, mu_brake);
    let s_air = 200.0;
    s_ground + s_air
}

/// Distância de pouso sobre obstáculo de 15m (50 ft), por segmentos (Task
/// 4.7) — substitui a distância aérea fixa de 200 m por física real:
///
///   S_total = S_ar + S_flare + S_ground
///     S_ar:    15 / tan(γ_app), γ_app = `approach_angle_deg` (padrão 3°)
///     S_flare: V_ref × `flare_time_s`
///     S_ground: rolagem de solo (`landing_ground_roll_m`, `mu_brake`)
pub fn landing_distance_50ft_m(
    mass_kg: f64,
    rho: f64,
    wing: &WingSpec,
    mu_brake: f64,
    perf_cfg: &PerformanceCfg,
) -> f64 {
    let s_ground = landing_ground_roll_m(mass_kg, rho, wing, mu_brake);

    let w = mass_kg * G;
    let v_s = ((2.0 * w) / (rho * wing.area_m2 * wing.cl_max)).sqrt();
    let v_ref = 1.30 * v_s;

    let gamma_app = perf_cfg.approach_angle_deg.to_radians();
    let s_air = 15.0 / gamma_app.tan();
    let s_flare = v_ref * perf_cfg.flare_time_s;

    s_air + s_flare + s_ground
}

// ─── VELOCIDADE MÁXIMA NIVELADA ───────────────────────────────────────────────

/// Velocidade máxima em voo nivelado: maior V com P_disp(V) ≥ P_req(V).
///
/// P_req(V) tem formato em U (arrasto induzido domina perto do estol), então
/// bissecção ingênua sobre [1.2·Vs, 200 m/s] não é segura: um excesso de
/// potência negativo no limite inferior não prova inviabilidade em toda a
/// faixa (pode haver uma velocidade de cruzeiro válida mais à frente), e uma
/// função com múltiplas trocas de sinal não garante que a bissecção simples
/// convirja para a raiz mais à direita (a velocidade máxima real).
///
/// Estratégia em duas etapas — varredura grosseira seguida de refinamento:
///   1. Amostra P_excesso(V) em passos uniformes sobre [1.2·Vs, 200 m/s] e
///      localiza a amostra de MAIOR V com excesso > 0.
///      - Nenhuma amostra positiva → inviável em toda a faixa modelada,
///        retorna 1.2·Vs (limite inferior).
///      - A última amostra do grid é positiva → voo nivelado sustentado até
///        o teto do modelo, retorna 200 m/s.
///   2. Caso contrário, bissecta dentro do subintervalo [V_k, V_{k+1}]
///      imediatamente após a última amostra positiva, refinando a raiz mais
///      à direita nesse intervalo (onde P_excesso muda de + para -).
pub fn max_level_speed_ms(mass_kg: f64, altitude_m: f64, isa_delta_c: f64, wing: &WingSpec,
                          state: &AircraftState, engine: &EngineSpec,
                          static_thrust_factor: f64) -> f64 {
    let rho = Isa::density_kgm3(altitude_m, isa_delta_c);
    let engine_rpm = engine.rpm_rated;
    let v_stall = ((2.0 * mass_kg * G) / (rho * wing.area_m2 * wing.cl_max)).sqrt();
    let (lo, hi) = (1.2 * v_stall, 200.0);
    let pex = |v: f64| excess_power_kw(v, mass_kg, rho, wing, engine, engine_rpm,
                                       state.psru_ratio, state.prop_diameter_m, altitude_m,
                                       isa_delta_c, static_thrust_factor, state.psru_efficiency);

    // Varredura grosseira: 100 passos (101 amostras) sobre [lo, hi]
    const STEPS: usize = 100;
    let dv = (hi - lo) / STEPS as f64;
    let mut last_positive: Option<usize> = None;
    for i in 0..=STEPS {
        let v = lo + i as f64 * dv;
        if pex(v) > 0.0 {
            last_positive = Some(i);
        }
    }

    let k = match last_positive {
        None => return lo,   // inviável em toda a faixa modelada
        Some(k) => k,
    };
    if k == STEPS {
        return hi;            // sustentado até o teto do modelo
    }

    // Refinamento: bissecta [V_k, V_{k+1}], a raiz mais à direita
    let mut v_lo = lo + k as f64 * dv;
    let mut v_hi = lo + (k + 1) as f64 * dv;
    for _ in 0..40 {
        let mid = 0.5 * (v_lo + v_hi);
        if pex(mid) > 0.0 { v_lo = mid; } else { v_hi = mid; }
    }
    0.5 * (v_lo + v_hi)
}

// ─── AGENTE PRINCIPAL ────────────────────────────────────────────────────────

pub struct PerformanceAgent;

impl PerformanceAgent {
    pub fn run(
        state: &AircraftState,
        wing: &WingSpec,
        prop: &PropulsionSpec,
        mtow_kg: f64,
        engine: &EngineSpec,
        req: &Requirements,
        perf_cfg: &PerformanceCfg,
    ) -> PerformanceSpec {
        // Task 4.6: densidades de desempenho (subida/teto/decolagem/pouso)
        // agora usam a atmosfera ISA completa (`Isa::density_kgm3`) no ΔISA
        // da missão (`req.isa_delta_c`), não mais a aproximação exponencial
        // fixa em dia padrão.
        let isa_delta_c = req.isa_delta_c;
        let rho_sl = Isa::density_kgm3(0.0, isa_delta_c);
        let fuel_density = engine.fuel.density_kg_per_l;
        let stf = perf_cfg.static_thrust_factor;

        // Razão de subida ao nível do mar (MTOW cheio)
        let (rc_sl, vy_sl_kmh) = climb_rate_ms(mtow_kg, 0.0, isa_delta_c, wing, state, engine, stf);

        // Vx/gradiente de subida ao nível do mar (MTOW cheio) — Task 4.7,
        // mesma condição de referência de `rc_sl` (CS 23.65 exige gradiente
        // ≥ 8.3% para esta categoria — ver `ConstraintChecker::verify`).
        let (gradient_sl, vx_sl_kmh) = best_climb_angle_ms(mtow_kg, 0.0, isa_delta_c, wing, state,
                                                             engine, stf);

        // Razão de subida na altitude de CRUZEIRO DA MISSÃO (Finding 2 da
        // revisão final: antes hardcoded em 2.500 m, ignorando
        // `req.cruise_altitude_m` — coincidia com a missão de projeto
        // padrão, mas divergia silenciosamente para qualquer missão com
        // outra altitude de cruzeiro, ex. uma missão de traslado a 2.000 m
        // em vez dos 2.500 m padrão — ver `config/missions/*.toml`). Massa:
        // cruzeiro parcialmente queimado (~35% do combustível gasto).
        let mass_mid = mtow_kg - (prop.fuel_capacity_l * fuel_density * 0.35); // 35% do combustível gasto
        let (rc_cruise_alt, _) =
            climb_rate_ms(mass_mid, req.cruise_altitude_m, isa_delta_c, wing, state, engine, stf);

        // Teto de serviço
        let ceiling = service_ceiling_m(mass_mid, isa_delta_c, wing, state, engine, stf);

        // Distâncias de decolagem (rolagem × 1,5, estimativa simplificada)
        let d_to_paved = takeoff_distance_m(mtow_kg, rho_sl, wing, state, 1.00, engine, isa_delta_c, stf);
        let d_to_grass  = takeoff_distance_m(mtow_kg, rho_sl, wing, state, 1.20, engine, isa_delta_c, stf);

        // Distâncias de decolagem sobre obstáculo de 15m/50ft, por segmentos
        // (Task 4.7)
        let d_to_50ft_paved = takeoff_distance_50ft_m(mtow_kg, rho_sl, wing, state, 1.00, engine,
                                                        isa_delta_c, perf_cfg);
        let d_to_50ft_grass = takeoff_distance_50ft_m(mtow_kg, rho_sl, wing, state, 1.20, engine,
                                                        isa_delta_c, perf_cfg);

        // Distância de pouso (massa de pouso ≈ MTOW - 60% combustível)
        let mass_ldg = mtow_kg - prop.fuel_capacity_l * fuel_density * 0.60;
        let d_ldg = landing_distance_m(mass_ldg, rho_sl, wing, perf_cfg.mu_brake_paved);
        let d_ldg_50ft = landing_distance_50ft_m(mass_ldg, rho_sl, wing, perf_cfg.mu_brake_paved,
                                                   perf_cfg);
        // Pouso na GRAMA sobre 15 m (revisão final do ciclo 6): MESMA
        // chamada do pavimentado acima, só o μ de frenagem troca
        // (`mu_brake_grass` < `mu_brake_paved` ⇒ rolagem mais longa). Até
        // esta correção, `mu_brake_grass` era validado em `config.rs` e
        // NUNCA consumido — o check #24 gateava a pista de grama com a
        // distância de pouso PAVIMENTADA, otimista por construção.
        let d_ldg_50ft_grass = landing_distance_50ft_m(mass_ldg, rho_sl, wing,
                                                         perf_cfg.mu_brake_grass, perf_cfg);

        // Melhor planeio (Task 4.7) — MTOW, nível do mar, motor cortado.
        let (v_bg_kmh, ld_max) = best_glide(mtow_kg, rho_sl, wing);

        // Velocidade máxima nivelada em cruzeiro (altitude de cruzeiro da
        // missão, rpm nominal do motor — Finding 2 da revisão final, mesma
        // razão do `rc_cruise_alt` acima).
        let v_max_cruise_ms =
            max_level_speed_ms(mtow_kg, req.cruise_altitude_m, isa_delta_c, wing, state, engine, stf);

        PerformanceSpec {
            v_cruise_kmh:         v_max_cruise_ms * 3.6,
            v_stall_kmh:          wing.stall_speed_flaps_kmh,
            rc_sl_ms:             rc_sl,
            rc_cruise_alt_ms:     rc_cruise_alt,
            service_ceiling_m:    ceiling,
            to_distance_paved_m:  d_to_paved,
            to_distance_grass_m:  d_to_grass,
            landing_distance_m:   d_ldg,
            range_km:             prop.range_km,
            endurance_h:          prop.endurance_h,
            vx_kmh:               vx_sl_kmh,
            vy_kmh:               vy_sl_kmh,
            best_glide_kmh:       v_bg_kmh,
            glide_ratio:          ld_max,
            climb_gradient_pct:   gradient_sl * 100.0,
            to_50ft_paved_m:      d_to_50ft_paved,
            to_50ft_grass_m:      d_to_50ft_grass,
            ldg_50ft_m:           d_ldg_50ft,
            ldg_50ft_grass_m:     d_ldg_50ft_grass,
        }
    }
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::atmosphere::RHO_SL;
    use crate::models::aircraft_state::AircraftState;
    use crate::models::aircraft_config::test_fixtures::config_teste;
    use crate::models::engine::test_fixtures::motor_generico_teste as engine_teste;
    use crate::agents::{aerodynamics::AerodynamicsAgent, propulsion::PropulsionAgent};

    fn setup() -> (AircraftState, WingSpec, PropulsionSpec, EngineSpec, Requirements, PerformanceCfg) {
        let cfg    = config_teste();
        let state  = AircraftState::from_config(&cfg);
        let req    = crate::models::requirements::test_fixtures::requisitos_teste();
        let wing   = AerodynamicsAgent::run(&state, &req);
        let engine = engine_teste();
        let prop   = PropulsionAgent::run(&state, &req, &wing, &engine);
        let perf_cfg = cfg.performance.clone();
        (state, wing, prop, engine, req, perf_cfg)
    }

    #[test]
    fn razao_subida_positiva_no_solo() {
        let (state, wing, _prop, engine, _req, perf_cfg) = setup();
        let (rc, vy) = climb_rate_ms(state.mtow_kg, 0.0, 0.0, &wing, &state, &engine,
                                      perf_cfg.static_thrust_factor);
        println!("RC SL = {rc:.2} m/s = {:.0} fpm, Vy = {vy:.1} km/h",
                 rc * 196.85);
        assert!(rc > 0.0, "RC SL deve ser positiva");
    }

    #[test]
    fn razao_subida_decresce_com_altitude() {
        let (state, wing, _prop, engine, _req, perf_cfg) = setup();
        let stf = perf_cfg.static_thrust_factor;
        let (rc0, _) = climb_rate_ms(state.mtow_kg, 0.0,     0.0, &wing, &state, &engine, stf);
        let (rc2, _) = climb_rate_ms(state.mtow_kg, 2_500.0, 0.0, &wing, &state, &engine, stf);
        let (rc5, _) = climb_rate_ms(state.mtow_kg, 5_000.0, 0.0, &wing, &state, &engine, stf);
        println!("RC SL={rc0:.2} | RC 2500m={rc2:.2} | RC 5000m={rc5:.2} m/s");
        assert!(rc0 > rc2 && rc2 > rc5, "RC deve diminuir com a altitude");
    }

    #[test]
    fn teto_de_servico_razoavel() {
        let (state, wing, _prop, engine, _req, perf_cfg) = setup();
        let ceiling = service_ceiling_m(state.mtow_kg * 0.85, 0.0, &wing, &state, &engine,
                                         perf_cfg.static_thrust_factor);
        println!("Teto de serviço: {ceiling:.0} m ({:.0} ft)", ceiling * 3.2808);
        // Valor observado empiricamente para a fixture sintética: ~7.600 m.
        // Janela apertada o suficiente para pegar regressões reais (não é só
        // "menor que o teto artificial de 8.000 m do laço de busca"). Sem
        // mudança na Task 4.7: `static_thrust_factor` só afeta o ramo
        // estático (V<0,5 m/s) de `thrust_available_n`, jamais atingido pela
        // varredura de `climb_rate_ms` (sempre V ≥ 1,3·Vs).
        assert!(ceiling > 7_000.0 && ceiling < 7_900.0,
            "Teto {ceiling:.0} m fora do intervalo esperado (7.000–7.900 m)");
    }

    // ─── Task 4.7: Vx, planeio, gradiente CS 23.65, tração estática ─────────

    #[test]
    fn vx_estritamente_menor_que_vy() {
        let (state, wing, _prop, engine, _req, perf_cfg) = setup();
        let stf = perf_cfg.static_thrust_factor;
        let (_rc, vy_kmh) = climb_rate_ms(state.mtow_kg, 0.0, 0.0, &wing, &state, &engine, stf);
        let (_grad, vx_kmh) = best_climb_angle_ms(state.mtow_kg, 0.0, 0.0, &wing, &state, &engine, stf);
        println!("Vx={vx_kmh:.1} km/h  Vy={vy_kmh:.1} km/h");
        assert!(vx_kmh < vy_kmh,
            "Vx ({vx_kmh:.1} km/h) deveria ser ESTRITAMENTE menor que Vy ({vy_kmh:.1} km/h) — \
             melhor ângulo de subida ocorre em velocidade mais baixa que melhor razão");
    }

    #[test]
    fn melhor_planeio_bate_com_formula_fechada() {
        let (state, wing, _prop, _engine, _req, _perf_cfg) = setup();
        let (v_bg_kmh, ld_max) = best_glide(state.mtow_kg, RHO_SL, &wing);

        // Fórmula fechada calculada independentemente na própria asserção —
        // cross-check contra os valores REAIS da fixture sintética em tempo
        // de execução (não um número copiado do handoff), K = 1/(π·AR·e).
        let k = 1.0 / (std::f64::consts::PI * wing.aspect_ratio * wing.oswald_efficiency);
        let ld_max_esperado = 1.0 / (2.0 * (k * wing.cd0).sqrt());
        let w = state.mtow_kg * G;
        let v_bg_esperado_ms = ((2.0 * w) / (RHO_SL * wing.area_m2)).sqrt()
            * (k / wing.cd0).powf(0.25);
        let v_bg_esperado_kmh = v_bg_esperado_ms * 3.6;

        println!("V_bg={v_bg_kmh:.1} km/h (esperado {v_bg_esperado_kmh:.1})  \
                   L/Dmax={ld_max:.2} (esperado {ld_max_esperado:.2})  \
                   AR={:.3} e={:.4} CD0={:.4} K={k:.5}",
                 wing.aspect_ratio, wing.oswald_efficiency, wing.cd0);

        assert!((ld_max - ld_max_esperado).abs() / ld_max_esperado < 0.02,
            "L/Dmax {ld_max:.3} diverge >2% do esperado {ld_max_esperado:.3}");
        assert!((v_bg_kmh - v_bg_esperado_kmh).abs() / v_bg_esperado_kmh < 0.02,
            "V_bg {v_bg_kmh:.1} km/h diverge >2% do esperado {v_bg_esperado_kmh:.1} km/h");
    }

    #[test]
    fn fator_de_tracao_estatica_e_proporcao_exata() {
        let (state, _wing, _prop, engine, _req, _perf_cfg) = setup();
        let t_ideal = static_thrust_ideal_n(&engine, engine.rpm_max_continuous,
                                             state.prop_diameter_m, 0.0, 0.0,
                                             state.psru_efficiency);
        let t_075 = thrust_available_n(0.0, &engine, engine.rpm_max_continuous,
                                        state.psru_ratio, state.prop_diameter_m, 0.0, 0.0, 0.75,
                                        state.psru_efficiency);
        let t_100 = thrust_available_n(0.0, &engine, engine.rpm_max_continuous,
                                        state.psru_ratio, state.prop_diameter_m, 0.0, 0.0, 1.0,
                                        state.psru_efficiency);
        println!("T_ideal={t_ideal:.1}N  T(factor=0.75)={t_075:.1}N  T(factor=1.0)={t_100:.1}N");
        // factor=1.0 deve reproduzir exatamente o valor IDEAL (sem correção).
        assert!((t_100 - t_ideal).abs() < 1e-6,
            "T(factor=1.0) deveria ser idêntica à tração ideal sem correção");
        // 0.75× do valor ideal — a razão exata exigida pela Task 4.7.
        assert!((t_075 - 0.75 * t_ideal).abs() < 1e-6,
            "T(factor=0.75) deveria ser exatamente 0.75 × tração ideal");
    }

    #[test]
    fn decolagem_50ft_maior_que_rolagem_de_solo_simples() {
        let (state, wing, _prop, engine, _req, perf_cfg) = setup();
        let s_ground = takeoff_ground_roll_m(state.mtow_kg, RHO_SL, &wing, &state, &engine, 0.0,
                                              perf_cfg.static_thrust_factor);
        let s_50ft = takeoff_distance_50ft_m(state.mtow_kg, RHO_SL, &wing, &state, 1.0, &engine,
                                              0.0, &perf_cfg);
        println!("S_ground={s_ground:.0}m  S_50ft={s_50ft:.0}m");
        assert!(s_50ft > s_ground,
            "distância sobre 15m ({s_50ft:.0}m) deveria ser MAIOR que a rolagem de solo pura \
             ({s_ground:.0}m) — inclui rotação + subida");
    }

    // ─── Ciclo 6 (task 2): sensibilidade ao diâmetro de hélice ─────────────
    //
    // A cadeia D → tração estática (disco atuador, T ∝ D^(2/3) — ver
    // `static_thrust_ideal_n`) → distância de decolagem sobre obstáculo
    // existe desde a Task 4.7, mas nenhum teste protegia o VEREDITO físico
    // esperado (hélice menor ⟹ pior desempenho). Direção ESTRITA (>), resto
    // fixo.

    #[test]
    fn helice_menor_tem_menos_tracao_estatica() {
        let (state, _wing, _prop, engine, _req, _perf_cfg) = setup();
        let t_1_9 = static_thrust_ideal_n(&engine, engine.rpm_max_continuous, 1.9, 0.0, 0.0,
                                           state.psru_efficiency);
        let t_1_6 = static_thrust_ideal_n(&engine, engine.rpm_max_continuous, 1.6, 0.0, 0.0,
                                           state.psru_efficiency);
        println!("T(D=1.9m)={t_1_9:.1}N  T(D=1.6m)={t_1_6:.1}N");
        assert!(t_1_9 > t_1_6,
            "hélice MAIOR (D=1.9m) deveria produzir tração estática ESTRITAMENTE maior que a \
             menor (D=1.6m): T(1.9)={t_1_9:.1}N, T(1.6)={t_1_6:.1}N");
    }

    #[test]
    fn helice_menor_alonga_decolagem_sobre_obstaculo() {
        let (mut state, wing, _prop, engine, _req, perf_cfg) = setup();
        state.prop_diameter_m = 1.9;
        let s_1_9 = takeoff_distance_50ft_m(state.mtow_kg, RHO_SL, &wing, &state, 1.0, &engine,
                                             0.0, &perf_cfg);
        state.prop_diameter_m = 1.6;
        let s_1_6 = takeoff_distance_50ft_m(state.mtow_kg, RHO_SL, &wing, &state, 1.0, &engine,
                                             0.0, &perf_cfg);
        println!("S_50ft(D=1.9m)={s_1_9:.1}m  S_50ft(D=1.6m)={s_1_6:.1}m");
        assert!(s_1_6 > s_1_9,
            "hélice MENOR (D=1.6m) deveria alongar ESTRITAMENTE a decolagem sobre obstáculo \
             de 15m em relação à maior (D=1.9m): S(1.6)={s_1_6:.1}m, S(1.9)={s_1_9:.1}m");
    }

    #[test]
    fn pouso_50ft_maior_que_pouso_ground_roll_mais_ar_fixo() {
        let (state, wing, prop, engine, _req, perf_cfg) = setup();
        let mass_ldg = state.mtow_kg - prop.fuel_capacity_l * engine.fuel.density_kg_per_l * 0.60;
        let d_legado = landing_distance_m(mass_ldg, RHO_SL, &wing, perf_cfg.mu_brake_paved);
        let d_50ft = landing_distance_50ft_m(mass_ldg, RHO_SL, &wing, perf_cfg.mu_brake_paved,
                                              &perf_cfg);
        println!("Pouso legado (200m ar fixo)={d_legado:.0}m  Pouso 50ft (segmentado)={d_50ft:.0}m");
        assert!(d_50ft > d_legado,
            "pouso sobre 15m por segmentos ({d_50ft:.0}m) deveria ser MAIOR que a estimativa \
             legada de 200m fixos + rolagem ({d_legado:.0}m) — aproximação a 3° percorre \
             ~286m antes do toque, mais o flare");
    }

    #[test]
    fn distancia_decolagem_plausivel() {
        let (state, wing, _prop, engine, _req, perf_cfg) = setup();
        let d = takeoff_distance_m(state.mtow_kg, RHO_SL, &wing, &state, 1.0, &engine, 0.0,
                                    perf_cfg.static_thrust_factor);
        println!("Decolagem pista pavimentada: {d:.0} m");
        // Task 4.7: T_avg passou a usar tração ESTÁTICA corrigida (Rankine-
        // Froude × static_thrust_factor da fixture, 0.72) em vez da antiga
        // estimativa ad hoc ("80% da tração a V_lo/2") — a distância cresceu
        // (T_avg menor). Valor observado empiricamente para a fixture
        // sintética pós-Task-4.7: ver task-4.7-report.md para a tabela
        // antes/depois. Janela alargada para cobrir o novo patamar.
        assert!(d > 200.0 && d < 700.0,
            "Distância TO {d:.0} m fora do esperado (200–700 m)");
    }

    #[test]
    fn distancia_pouso_plausivel() {
        let (state, wing, prop, engine, _req, perf_cfg) = setup();
        let mass_ldg = state.mtow_kg - prop.fuel_capacity_l * engine.fuel.density_kg_per_l * 0.60;
        let d = landing_distance_m(mass_ldg, RHO_SL, &wing, perf_cfg.mu_brake_paved);
        println!("Pouso pista pavimentada: {d:.0} m");
        // Valor observado empiricamente para a fixture sintética: ~390 m
        // (mu_brake_paved da fixture, 0.38, é próximo do antigo literal
        // hardcoded 0.40 — variação pequena).
        assert!(d > 300.0 && d < 520.0,
            "Distância LDG {d:.0} m fora do esperado (300–520 m)");
    }

    #[test]
    fn velocidade_maxima_resolvida_do_equilibrio() {
        let (state, wing, _prop, engine, _req, perf_cfg) = setup();
        let v_max = max_level_speed_ms(state.mtow_kg, 2_500.0, 0.0, &wing, &state, &engine,
                                        perf_cfg.static_thrust_factor);
        let v_max_kmh = v_max * 3.6;
        println!("V_max nivelada = {v_max_kmh:.2} km/h");
        // Deve ser um número resolvido (não o requisito ecoado) e > requisito
        assert!(v_max_kmh > 280.0 && v_max_kmh < 400.0,
            "V_max nivelada {v_max_kmh:.0} km/h implausível");
        // Regressão do resolvedor coarse-to-fine (bissecção): valor medido
        // empiricamente para a fixture sintética `config_teste`/`motor_generico_teste`
        // (não são dados reais — o pin contra o motor/célula reais vive em
        // tests/generic_engine.rs, carregado dos TOMLs de verdade). Este
        // teste aqui existe para pegar regressões no algoritmo de busca de
        // `max_level_speed_ms` em si, não para validar uma configuração
        // específica. Valor recalculado após a Task 4.6 (ISA completa
        // substitui a aproximação exponencial de densidade): pré-Task-4.6
        // 306.9409599205 km/h → pós-Task-4.6 306.9014368061 km/h (desvio
        // sub-0,1%, mesma origem do desvio documentado em
        // tests/generic_engine.rs).
        //
        // ATUALIZAÇÃO (Task 5.2): `cooling_drag_fraction` (Task 5.2, agora
        // um campo obrigatório de `[drag]`, 0.035 nesta fixture sintética —
        // ver `aircraft_config::test_fixtures::config_teste`) eleva o CD0
        // total em 3,5%, reduzindo L/D e portanto V_max — consequência
        // honesta esperada da task (documentada no brief), não uma
        // regressão do resolvedor: 306.9014368061 km/h → 303.5154833612 km/h
        // (-3,386 km/h, -1,10%).
        //
        // ATUALIZAÇÃO (Finding 1 da revisão final): `state.psru_efficiency`
        // passa a alimentar `shaft_power_kw`/`thrust_available_n` (antes
        // hardcoded em `agents::propulsion::PSRU_EFFICIENCY = 0.97`, agora
        // removido — ver `AircraftState::psru_efficiency`). A fixture
        // sintética `config_teste()` usa 0,965 (deliberadamente distinto de
        // 0,97 — ver seu doc-comment), levemente abaixo do valor antigo
        // implícito: menos potência de eixo disponível, V_max cai:
        // 303.5154833612 km/h → 302.9220524587 km/h (-0,593 km/h, -0,20%).
        //
        // ATUALIZAÇÃO (task refino-ciclo2, 1b): `cd0_empennage` da fixture
        // deixa de ser o `[empennage].cd0=0.0042` fixo e passa a ser
        // DERIVADO de `cd0_area_factor·(S_h+S_v)/S_w` (`[empennage].
        // cd0_area_factor=0.0135` — ver `aircraft_config::test_fixtures::
        // config_teste`; a massa da empenagem em si, separada do arrasto,
        // é alimentada desde o ciclo 3 (oew-parametrico) por `[mass_model]`
        // via `agents::mass_model`, não mais por campos `mass_per_area_*`
        // — ver docstring do módulo `agents::mass_model`): com
        // S_h≈2,3762/S_v≈1,4218/S_w=13,5, cd0_empennage cai para
        // ≈0,003798 (< 0,0042 antigo) — menos arrasto parasita, V_max sobe:
        // 302.9220524587 km/h → 304.6465774391 km/h (+1,725 km/h, +0,57%).
        let v_max_observado_kmh = 304.6465774391;
        assert!((v_max_kmh - v_max_observado_kmh).abs() < 0.5,
            "V_max nivelada {v_max_kmh:.2} km/h divergiu do valor observado \
             {v_max_observado_kmh:.2} km/h em mais de 0.5 km/h — possível \
             regressão no resolvedor coarse-to-fine");
    }

    #[test]
    fn velocidade_maxima_inviavel_com_massa_absurda() {
        // Massa absurdamente alta (20 t) para uma aeronave leve: nenhuma
        // amostra da varredura terá excesso de potência positivo, então a
        // função deve retornar o limite inferior (1.2·Vs) em vez de um
        // resultado espúrio ou o teto do modelo (200 m/s).
        let (state, wing, _prop, engine, _req, perf_cfg) = setup();
        let rho = Isa::density_kgm3(2_500.0, 0.0);
        let mass_kg = 20_000.0;
        let v_stall = ((2.0 * mass_kg * G) / (rho * wing.area_m2 * wing.cl_max)).sqrt();
        let v_lo_esperado = 1.2 * v_stall;

        let v_max = max_level_speed_ms(mass_kg, 2_500.0, 0.0, &wing, &state, &engine,
                                        perf_cfg.static_thrust_factor);
        println!("V_max (massa inviável) = {:.2} m/s, esperado ≈ {v_lo_esperado:.2} m/s",
                  v_max);
        assert!((v_max - v_lo_esperado).abs() < 1e-6,
            "Caso inviável deveria retornar 1.2·Vs ({v_lo_esperado:.2} m/s), \
             obteve {v_max:.2} m/s");
    }

    #[test]
    fn velocidade_cruzeiro_acima_do_requisito() {
        let (state, wing, prop, engine, req, perf_cfg) = setup();
        let perf = PerformanceAgent::run(&state, &wing, &prop, state.mtow_kg, &engine, &req, &perf_cfg);
        println!("V_cruise = {:.1} km/h", perf.v_cruise_kmh);
        assert!(perf.v_cruise_kmh >= 280.0,
            "V_cruise {:.1} km/h abaixo do requisito de 280 km/h", perf.v_cruise_kmh);
    }

    #[test]
    fn gradiente_de_subida_positivo_e_menor_que_100_por_cento() {
        // Sanidade: gradiente é uma fração RC/V — não pode ser negativo
        // (bloqueado por `.max(0.0)`) nem fisicamente >100% para esta
        // aeronave/motor sintéticos.
        let (state, wing, prop, engine, req, perf_cfg) = setup();
        let perf = PerformanceAgent::run(&state, &wing, &prop, state.mtow_kg, &engine, &req, &perf_cfg);
        println!("climb_gradient_pct = {:.2}%  Vx={:.1}km/h  Vy={:.1}km/h",
                 perf.climb_gradient_pct, perf.vx_kmh, perf.vy_kmh);
        assert!(perf.climb_gradient_pct > 0.0 && perf.climb_gradient_pct < 100.0,
            "gradiente {:.2}% fora da faixa físicamente plausível (0, 100)%",
            perf.climb_gradient_pct);
    }
}
