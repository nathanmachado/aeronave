/// PerformanceAgent — Desempenho Completo da Aeronave
///
/// Calcula:
///   - Razão de subida (RC) e ângulo de subida em diferentes condições
///   - Velocidades ótimas de subida (Vx = melhor ângulo, Vy = melhor razão)
///   - Distâncias de decolagem e pouso (pista pavimentada e gramado/terra)
///   - Teto de serviço (RC = 0,5 m/s residual)
///   - Hodógrafo de voo (envelope de desempenho)
///
/// Todas as velocidades em m/s internamente; apresentadas em km/h na saída.
///
/// Referências:
///   - Anderson, J. "Introduction to Flight", Cap. 6
///   - Raymer, D. "Aircraft Design", Cap. 5 (takeoff e landing)
///   - RBAC 23 / CS-23 — requisitos de desempenho categoria Normal

use crate::agents::aerodynamics::isa_density;
use crate::models::engine::EngineSpec;
use crate::models::specs::{WingSpec, PropulsionSpec, PerformanceSpec};
use crate::agents::propulsion::{prop_rpm, prop_efficiency, advance_ratio, thrust_n,
                                  PSRU_EFFICIENCY};
use crate::models::aircraft_state::AircraftState;

const G: f64 = 9.807;   // m/s²
const RHO_SL: f64 = 1.225; // kg/m³

// ─── POTÊNCIA E TRAÇÃO DISPONÍVEIS ────────────────────────────────────────────

/// Potência de eixo disponível em altitude a RPM de cruzeiro (kW)
pub fn shaft_power_kw(engine: &EngineSpec, engine_rpm: f64, altitude_m: f64) -> f64 {
    engine.power_kw_at(engine_rpm, altitude_m) * PSRU_EFFICIENCY
}

/// Tração disponível da hélice em função da velocidade e altitude
pub fn thrust_available_n(
    v_ms: f64,
    engine: &EngineSpec,
    engine_rpm: f64,
    psru_ratio: f64,
    prop_diam_m: f64,
    altitude_m: f64,
) -> f64 {
    if v_ms < 0.5 {
        // Tração estática (solo): estimativa por impulso de disco (Rankine-Froude)
        let p_w = shaft_power_kw(engine, engine_rpm, altitude_m) * 1_000.0;
        let rho = isa_density(altitude_m);
        let disk_area = std::f64::consts::PI * (prop_diam_m / 2.0).powi(2);
        return (2.0 * rho * disk_area * p_w * p_w).powf(1.0 / 3.0);
    }
    let n_prop = prop_rpm(engine_rpm, psru_ratio);
    let j = advance_ratio(v_ms, n_prop, prop_diam_m);
    let eta = prop_efficiency(j);
    let p_shaft_w = shaft_power_kw(engine, engine_rpm, altitude_m) * 1_000.0;
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
) -> f64 {
    let drag   = drag_level_n(v_ms, mass_kg, rho, wing);
    let p_req  = power_required_kw(v_ms, drag);
    let thrust = thrust_available_n(v_ms, engine, engine_rpm, psru_ratio, prop_diam_m, altitude_m);
    let p_avail = thrust * v_ms / 1_000.0;
    p_avail - p_req
}

/// Razão de subida máxima (m/s) — varre velocidades para encontrar o pico
/// RC = P_excess / W
pub fn climb_rate_ms(
    mass_kg: f64,
    altitude_m: f64,
    wing: &WingSpec,
    state: &AircraftState,
    engine: &EngineSpec,
) -> (f64, f64) {
    let rho = isa_density(altitude_m);
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
                                   state.prop_diameter_m, altitude_m);
        let rc = pex * 1_000.0 / (mass_kg * G);
        if rc > best_rc {
            best_rc = rc;
            best_v  = v;
        }
    }
    (best_rc.max(0.0), best_v * 3.6) // (RC em m/s, Vy em km/h)
}

/// Teto de serviço: altitude onde RC = 0,5 m/s (padrão CS-23)
pub fn service_ceiling_m(
    mass_kg: f64,
    wing: &WingSpec,
    state: &AircraftState,
    engine: &EngineSpec,
) -> f64 {
    let mut alt = 0.0_f64;
    let step = 100.0_f64;
    loop {
        let (rc, _) = climb_rate_ms(mass_kg, alt, wing, state, engine);
        if rc <= 0.5 || alt > 8_000.0 {
            return alt;
        }
        alt += step;
    }
}

// ─── DISTÂNCIAS DE DECOLAGEM E POUSO ─────────────────────────────────────────

/// Distância de decolagem (método energético de Raymer, Cap. 5)
///
/// S_G = W² / (g · ρ · S · CL_TO · T_avg)
///   onde CL_TO = 0.8·CL_max (flap parcial), T_avg = tração média no roll
///
/// Fator de superfície:
///   pista pavimentada seca: 1.00
///   gramado firme:          1.15
///   terra compactada:       1.25
pub fn takeoff_distance_m(
    mass_kg: f64,
    rho: f64,
    wing: &WingSpec,
    state: &AircraftState,
    surface_factor: f64,
    engine: &EngineSpec,
) -> f64 {
    let w = mass_kg * G;
    let cl_to = 0.80 * wing.cl_max; // flap parcial na decolagem
    // V_liftoff = 1.1 · V_stall
    let v_lo = 1.10 * ((2.0 * w) / (rho * wing.area_m2 * wing.cl_max)).sqrt();
    // Tração média no roll (80% da tração a V_lo/2), RPM de decolagem =
    // máximo contínuo do motor.
    let t_avg = thrust_available_n(
        v_lo * 0.5,
        engine,
        engine.rpm_max_continuous,
        state.psru_ratio,
        state.prop_diameter_m,
        0.0, // nível do mar
    ) * 0.85;

    // Distância de ground roll
    let s_ground = w * w / (G * rho * wing.area_m2 * cl_to * t_avg);

    // Distância de transição (≈ 1,5 × ground roll de acordo com Raymer)
    let s_total = s_ground * 1.5;

    s_total * surface_factor
}

/// Distância de pouso (método de Raymer)
///
/// S_L = V_ref² / (2·g·(μ + sin γ_approch))
///   V_ref = 1.3·V_s    (velocidade de aproximação de referência)
///   μ = 0.40 (frenagem com freios nas rodas — pista pavimentada)
///   μ = 0.30 (frenagem em gramado/terra)
///
/// Inclui distância aérea de 50 ft (15 m) até o toque.
pub fn landing_distance_m(
    mass_kg: f64,
    rho: f64,
    wing: &WingSpec,
    surface_factor: f64,
) -> f64 {
    let w = mass_kg * G;
    let v_s = ((2.0 * w) / (rho * wing.area_m2 * wing.cl_max)).sqrt();
    let v_ref = 1.30 * v_s;

    // Frenagem efetiva (pavimentado)
    let mu_brk = 0.40 / surface_factor; // menor em gramado

    // Distância de parada após toque
    let s_ground = v_ref * v_ref / (2.0 * G * mu_brk);

    // Distância aérea (50 ft obstacle) ≈ 200 m típico para esta classe
    let s_air = 200.0;

    (s_ground + s_air) * surface_factor.sqrt() // correção para superfície
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
pub fn max_level_speed_ms(mass_kg: f64, altitude_m: f64, wing: &WingSpec,
                          state: &AircraftState, engine: &EngineSpec) -> f64 {
    let rho = isa_density(altitude_m);
    let engine_rpm = engine.rpm_rated;
    let v_stall = ((2.0 * mass_kg * G) / (rho * wing.area_m2 * wing.cl_max)).sqrt();
    let (lo, hi) = (1.2 * v_stall, 200.0);
    let pex = |v: f64| excess_power_kw(v, mass_kg, rho, wing, engine, engine_rpm,
                                       state.psru_ratio, state.prop_diameter_m, altitude_m);

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
    ) -> PerformanceSpec {
        let rho_sl = RHO_SL;
        let fuel_density = engine.fuel.density_kg_per_l;

        // Razão de subida ao nível do mar (MTOW cheio)
        let (rc_sl, _vy_sl_kmh) = climb_rate_ms(mtow_kg, 0.0, wing, state, engine);

        // Razão de subida a 2.500 m (cruzeiro parcialmente queimado — ~1.350 kg)
        let mass_mid = mtow_kg - (prop.fuel_capacity_l * fuel_density * 0.35); // 35% do combustível gasto
        let (rc_cruise_alt, _) = climb_rate_ms(mass_mid, 2_500.0, wing, state, engine);

        // Teto de serviço
        let ceiling = service_ceiling_m(mass_mid, wing, state, engine);

        // Distâncias de decolagem
        let d_to_paved = takeoff_distance_m(mtow_kg, rho_sl, wing, state, 1.00, engine);
        let d_to_grass  = takeoff_distance_m(mtow_kg, rho_sl, wing, state, 1.20, engine);

        // Distância de pouso (massa de pouso ≈ MTOW - 60% combustível)
        let mass_ldg = mtow_kg - prop.fuel_capacity_l * fuel_density * 0.60;
        let d_ldg = landing_distance_m(mass_ldg, rho_sl, wing, 1.00);

        // Velocidade máxima nivelada em cruzeiro (2.500 m, rpm nominal do motor)
        let v_max_cruise_ms = max_level_speed_ms(mtow_kg, 2_500.0, wing, state, engine);

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
        }
    }
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::{aircraft_state::AircraftState, requirements::Requirements};
    use crate::models::aircraft_config::test_fixtures::config_teste;
    use crate::models::engine::test_fixtures::motor_generico_teste as engine_teste;
    use crate::agents::{aerodynamics::AerodynamicsAgent, propulsion::PropulsionAgent};

    fn setup() -> (AircraftState, WingSpec, PropulsionSpec, EngineSpec) {
        let cfg    = config_teste();
        let state  = AircraftState::from_config(&cfg);
        let req    = Requirements::project_default();
        let wing   = AerodynamicsAgent::run(&state, &req);
        let engine = engine_teste();
        let prop   = PropulsionAgent::run(&state, &req, &wing, &engine);
        (state, wing, prop, engine)
    }

    #[test]
    fn razao_subida_positiva_no_solo() {
        let (state, wing, _prop, engine) = setup();
        let (rc, vy) = climb_rate_ms(state.mtow_kg, 0.0, &wing, &state, &engine);
        println!("RC SL = {rc:.2} m/s = {:.0} fpm, Vy = {vy:.1} km/h",
                 rc * 196.85);
        assert!(rc > 0.0, "RC SL deve ser positiva");
    }

    #[test]
    fn razao_subida_decresce_com_altitude() {
        let (state, wing, _prop, engine) = setup();
        let (rc0, _) = climb_rate_ms(state.mtow_kg, 0.0,     &wing, &state, &engine);
        let (rc2, _) = climb_rate_ms(state.mtow_kg, 2_500.0, &wing, &state, &engine);
        let (rc5, _) = climb_rate_ms(state.mtow_kg, 5_000.0, &wing, &state, &engine);
        println!("RC SL={rc0:.2} | RC 2500m={rc2:.2} | RC 5000m={rc5:.2} m/s");
        assert!(rc0 > rc2 && rc2 > rc5, "RC deve diminuir com a altitude");
    }

    #[test]
    fn teto_de_servico_razoavel() {
        let (state, wing, _prop, engine) = setup();
        let ceiling = service_ceiling_m(state.mtow_kg * 0.85, &wing, &state, &engine);
        println!("Teto de serviço: {ceiling:.0} m ({:.0} ft)", ceiling * 3.2808);
        // Valor observado empiricamente para a fixture sintética: ~7.600 m.
        // Janela apertada o suficiente para pegar regressões reais (não é só
        // "menor que o teto artificial de 8.000 m do laço de busca").
        assert!(ceiling > 7_000.0 && ceiling < 7_900.0,
            "Teto {ceiling:.0} m fora do intervalo esperado (7.000–7.900 m)");
    }

    #[test]
    fn distancia_decolagem_plausivel() {
        let (state, wing, _prop, engine) = setup();
        let d = takeoff_distance_m(state.mtow_kg, RHO_SL, &wing, &state, 1.0, &engine);
        println!("Decolagem pista pavimentada: {d:.0} m");
        // CS-23 limita 580 m para categoria Normal MTOW < 1.500 kg; valor
        // observado empiricamente para a fixture sintética: ~268 m.
        assert!(d > 200.0 && d < 400.0,
            "Distância TO {d:.0} m fora do esperado (200–400 m)");
    }

    #[test]
    fn distancia_pouso_plausivel() {
        let (state, wing, prop, engine) = setup();
        let mass_ldg = state.mtow_kg - prop.fuel_capacity_l * engine.fuel.density_kg_per_l * 0.60;
        let d = landing_distance_m(mass_ldg, RHO_SL, &wing, 1.0);
        println!("Pouso pista pavimentada: {d:.0} m");
        // Valor observado empiricamente para a fixture sintética: ~390 m.
        assert!(d > 300.0 && d < 500.0,
            "Distância LDG {d:.0} m fora do esperado (300–500 m)");
    }

    #[test]
    fn velocidade_maxima_resolvida_do_equilibrio() {
        let (state, wing, _prop, engine) = setup();
        let v_max = max_level_speed_ms(state.mtow_kg, 2_500.0, &wing, &state, &engine);
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
        // específica. Valor abaixo recalculado após a Task 2.1 (aircraft.toml).
        let v_max_observado_kmh = 306.9409599205;
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
        let (state, wing, _prop, engine) = setup();
        let rho = isa_density(2_500.0);
        let mass_kg = 20_000.0;
        let v_stall = ((2.0 * mass_kg * G) / (rho * wing.area_m2 * wing.cl_max)).sqrt();
        let v_lo_esperado = 1.2 * v_stall;

        let v_max = max_level_speed_ms(mass_kg, 2_500.0, &wing, &state, &engine);
        println!("V_max (massa inviável) = {:.2} m/s, esperado ≈ {v_lo_esperado:.2} m/s",
                  v_max);
        assert!((v_max - v_lo_esperado).abs() < 1e-6,
            "Caso inviável deveria retornar 1.2·Vs ({v_lo_esperado:.2} m/s), \
             obteve {v_max:.2} m/s");
    }

    #[test]
    fn velocidade_cruzeiro_acima_do_requisito() {
        let (state, wing, prop, engine) = setup();
        let perf = PerformanceAgent::run(&state, &wing, &prop, state.mtow_kg, &engine);
        println!("V_cruise = {:.1} km/h", perf.v_cruise_kmh);
        assert!(perf.v_cruise_kmh >= 280.0,
            "V_cruise {:.1} km/h abaixo do requisito de 280 km/h", perf.v_cruise_kmh);
    }
}
