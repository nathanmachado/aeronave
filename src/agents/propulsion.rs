/// PropulsionAgent — Motor Genérico via `EngineSpec`
///
/// Modela o comportamento do motor, PSRU, hélice e consumo de combustível
/// nas condições de cruzeiro e decolagem da aeronave. Todos os dados
/// específicos de motor (curva de torque, BSFC, indução, combustível) vêm
/// de `EngineSpec`, carregado de um TOML em `config/engines/`. Este módulo
/// contém apenas física genérica — trocar de motor é trocar o TOML, não o
/// código.
///
/// Referências:
///   - Hepperle, M. "JavaProp" — Blade Element Theory simplificado
///   - Roskam, J. "Airplane Design Part I" — Estimativas de PSRU

use crate::models::{
    aircraft_state::AircraftState,
    engine::EngineSpec,
    requirements::Requirements,
    specs::{PropulsionSpec, WingSpec},
};

// ─── PSRU (Propeller Speed Reduction Unit) ───────────────────────────────────

/// RPM da hélice após PSRU
pub fn prop_rpm(engine_rpm: f64, psru_ratio: f64) -> f64 {
    engine_rpm / psru_ratio
}

// ─── HÉLICE ──────────────────────────────────────────────────────────────────

/// Razão de avanço J = V / (n·D)
/// V em m/s, n em rotações/s, D em metros
pub fn advance_ratio(v_ms: f64, prop_rpm: f64, prop_diameter_m: f64) -> f64 {
    let n_rps = prop_rpm / 60.0;
    v_ms / (n_rps * prop_diameter_m)
}

/// Eficiência da hélice de passo variável (curva empírica por razão de avanço J).
///
/// Hélice de passo variável ajusta continuamente o ângulo das pás para manter
/// eficiência alta numa ampla faixa de J. Para este projeto:
///   Diâmetro: 1,95 m | 2 pás | PSRU + motor em cruzeiro
///
/// O pico de eficiência (~83%) ocorre em J ≈ 1,3–1,5.
///
/// Modelo polinomial calibrado com dados do JavaProp (Hepperle, DLR):
///   η = -0.15·J² + 0.39·J + 0.58   (válido para 0 < J < 2.8)
pub fn prop_efficiency(j: f64) -> f64 {
    if j <= 0.0 || j > 2.8 {
        return 0.0;
    }
    let eta = -0.15 * j * j + 0.39 * j + 0.58;
    eta.clamp(0.0, 0.86)
}

/// Tração disponível da hélice em Newton:
/// T = η · P_shaft / V
/// P_shaft em W, V em m/s
pub fn thrust_n(eta: f64, power_shaft_w: f64, v_ms: f64) -> f64 {
    if v_ms < 1.0 { return 0.0; } // evita divisão por zero no static
    eta * power_shaft_w / v_ms
}

// ─── CONSUMO DE COMBUSTÍVEL ──────────────────────────────────────────────────

/// Consumo de combustível em L/h.
///
/// `power_kw`: potência consumida — DEVE ser a potência de VIRABREQUIM
/// (pré-PSRU), a mesma referência de `bsfc_gkwh` (`BsfcModel::bsfc_gkwh`
/// modela o motor, medido no virabrequim). Achado da revisão da Task 5.1
/// (Finding 2): potências de EIXO pós-PSRU (`p_req_cruise_kw`/
/// `shaft_power_kw`, já reduzidas por `state.psru_efficiency` — Finding 1
/// da revisão final, `AircraftState::psru_efficiency`) precisam ser
/// divididas por essa mesma eficiência ANTES de chegar aqui — ver o único
/// caller (`PropulsionAgent::run`) e a dedução em
/// `agents::mission` ("BSFC referencia o virabrequim").
/// bsfc_gkwh: consumo específico (g/kWh) — de `engine.bsfc.bsfc_gkwh(...)`
/// density_kg_per_l: densidade do combustível (kg/L) — de `engine.fuel.density_kg_per_l`
pub fn fuel_consumption_lph(power_kw: f64, bsfc_gkwh: f64, density_kg_per_l: f64) -> f64 {
    let mass_gh = power_kw * bsfc_gkwh;              // g/h
    mass_gh / (density_kg_per_l * 1_000.0)           // L/h
}

// ─── BUSCA DO RPM DE CRUZEIRO ────────────────────────────────────────────────

/// Ponto candidato de operação em cruzeiro para um dado rpm de motor.
#[derive(Debug, Clone, Copy)]
struct CruisePoint {
    engine_rpm: f64,
    prop_rpm: f64,
    eta: f64,
    p_shaft_kw: f64,
    p_req_kw: f64,
    bsfc_gkwh: f64,
}

/// Busca o rpm de cruzeiro na faixa `[rpm_optimal·0.8, min(rpm_max_continuous,
/// rpm_optimal·1.2)]`, em passos de 50 rpm.
///
/// Critério: dentre os rpms cuja potência de eixo disponível (`p_shaft_kw`)
/// atende ou excede a potência requerida em voo nivelado (`p_req_kw`),
/// escolhe o que minimiza o BSFC. Se NENHUM rpm da faixa entrega a potência
/// requerida (motor genuinamente incapaz de sustentar a velocidade de
/// cruzeiro exigida — ex.: motores mais leves/menos potentes), escolhe o rpm
/// de MAIOR potência de eixo disponível (melhor esforço possível) e sinaliza
/// a inviabilidade através do segundo elemento da tupla de retorno.
///
/// Isto evita qualquer `panic!`/`assert!` no caminho de cálculo: o resultado
/// é sempre um `CruisePoint` válido, e a viabilidade é reportada como dado
/// (`PropulsionSpec::cruise_feasible`), verificada depois pelo
/// `ConstraintChecker`.
fn search_cruise_rpm(
    engine: &EngineSpec,
    v_cruise_ms: f64,
    psru_ratio: f64,
    psru_efficiency: f64,
    prop_diameter_m: f64,
    altitude_m: f64,
    drag_n: f64,
) -> (CruisePoint, bool) {
    let rpm_hi = engine.rpm_max_continuous.min(engine.bsfc.rpm_optimal * 1.2);
    // Se rpm_optimal·0.8 > rpm_hi (motor com rpm_max_continuous baixo em
    // relação ao rpm ótimo de BSFC), o limite inferior é grampeado ao
    // superior, reduzindo a varredura a um único ponto em vez de deixar o
    // laço vazio.
    let rpm_lo = (engine.bsfc.rpm_optimal * 0.8).min(rpm_hi);

    let evaluate = |engine_rpm: f64| -> CruisePoint {
        let prop_rpm_c = prop_rpm(engine_rpm, psru_ratio);
        let j = advance_ratio(v_cruise_ms, prop_rpm_c, prop_diameter_m);
        let eta = prop_efficiency(j);
        let p_shaft_kw = engine.power_kw_at(engine_rpm, altitude_m) * psru_efficiency;
        let p_req_kw = if eta > 0.0 {
            drag_n * v_cruise_ms / (eta * 1_000.0)
        } else {
            f64::INFINITY
        };
        let load_fraction = (p_req_kw / p_shaft_kw).min(1.0);
        let bsfc_gkwh = engine.bsfc.bsfc_gkwh(engine_rpm, load_fraction);
        CruisePoint { engine_rpm, prop_rpm: prop_rpm_c, eta, p_shaft_kw, p_req_kw, bsfc_gkwh }
    };

    let mut best_feasible: Option<CruisePoint> = None;
    let mut best_effort: Option<CruisePoint> = None;

    let mut consider = |point: CruisePoint| {
        if point.p_shaft_kw >= point.p_req_kw {
            let better = match best_feasible {
                Some(b) => point.bsfc_gkwh < b.bsfc_gkwh,
                None => true,
            };
            if better { best_feasible = Some(point); }
        }

        let better_effort = match best_effort {
            Some(b) => point.p_shaft_kw > b.p_shaft_kw,
            None => true,
        };
        if better_effort { best_effort = Some(point); }
    };

    let steps = (((rpm_hi - rpm_lo) / 50.0).floor() as i64).max(0);
    let mut last_sampled_rpm = rpm_lo;
    for i in 0..=steps {
        let rpm = rpm_lo + i as f64 * 50.0;
        last_sampled_rpm = rpm;
        consider(evaluate(rpm));
    }
    // Garante que o limite superior da faixa (`rpm_hi`) é sempre amostrado,
    // mesmo quando (rpm_hi - rpm_lo) não é múltiplo exato de 50 rpm — sem
    // isto, o laço de passos de 50 em 50 acima nunca alcança `rpm_hi`
    // exatamente (ex.: uma faixa 1.760→2.640 rpm pára em 2.610, nunca avalia
    // 2.640, que pode entregar BSFC melhor por estar mais perto do limite
    // superior de potência disponível — ver tests/generic_engine.rs para um
    // caso real onde isso muda o resultado).
    if rpm_hi - last_sampled_rpm > 1e-6 {
        consider(evaluate(rpm_hi));
    }

    match best_feasible {
        Some(p) => (p, true),
        None => (best_effort.expect("faixa de rpm de busca não pode ser vazia"), false),
    }
}

// ─── AGENTE PRINCIPAL ────────────────────────────────────────────────────────

pub struct PropulsionAgent;

impl PropulsionAgent {
    /// Executa o agente e retorna a especificação completa de propulsão.
    pub fn run(
        state: &AircraftState,
        req: &Requirements,
        wing: &WingSpec,
        engine: &EngineSpec,
    ) -> PropulsionSpec {
        let v_cruise_ms = req.cruise_speed_min_kmh / 3.6;

        // Potência necessária para voo nivelado a V_cruise não depende do
        // rpm do motor (só de arrasto e velocidade) — calculada uma vez.
        let drag_n = {
            let rho = crate::models::atmosphere::Isa::density_kgm3(
                req.cruise_altitude_m, req.isa_delta_c,
            );
            let q   = crate::agents::aerodynamics::dynamic_pressure(rho, v_cruise_ms);
            crate::agents::aerodynamics::drag_total_n(q, wing.area_m2, wing.cd_cruise)
        };

        // RPM de cruzeiro: busca o rpm de menor BSFC que sustenta o voo
        // nivelado exigido, dentro da faixa ao redor do rpm ótimo de BSFC do
        // motor (ver `search_cruise_rpm`).
        let (cp, cruise_feasible) = search_cruise_rpm(
            engine, v_cruise_ms, state.psru_ratio, state.psru_efficiency, state.prop_diameter_m,
            req.cruise_altitude_m, drag_n,
        );

        // `cp.p_req_kw` é potência de EIXO (pós-PSRU, na hélice) — BSFC
        // referencia o VIRABREQUIM (pré-PSRU). Achado da revisão da Task
        // 5.1 (Finding 2): sem dividir por `state.psru_efficiency`, o consumo
        // era subestimado em ~3% (`1/0,97 − 1`) — ver doc de
        // `fuel_consumption_lph` e a dedução em `agents::mission`.
        let fc_lph = fuel_consumption_lph(
            cp.p_req_kw / state.psru_efficiency, cp.bsfc_gkwh, engine.fuel.density_kg_per_l,
        );

        // Tração em cruzeiro (por construção, iguala o arrasto em regime
        // permanente: T = η·P_req/V = η·(D·V/η)/V = D).
        let thrust = thrust_n(cp.eta, cp.p_req_kw * 1_000.0, v_cruise_ms);

        // Autonomia e alcance (inclui reserva)
        let endurance_h = state.fuel_capacity_l / fc_lph
            * (1.0 - req.fuel_reserve_fraction);
        let range_km = req.cruise_speed_min_kmh * endurance_h;

        PropulsionSpec {
            engine_model:      engine.name.clone(),
            power_hp:          engine.power_kw_max() / 0.7457,
            power_kw:          engine.power_kw_max(),
            max_torque_nm:     engine.torque_max_nm(),
            rated_rpm:         engine.rpm_rated,
            engine_mass_kg:    engine.mass_kg,
            psru_ratio:        state.psru_ratio,
            engine_rpm_cruise: cp.engine_rpm,
            prop_rpm_cruise:   cp.prop_rpm,
            prop_diameter_m:   state.prop_diameter_m,
            fuel_type:         engine.fuel.name.clone(),
            fuel_capacity_l:   state.fuel_capacity_l,
            fc_cruise_lph:     fc_lph,
            bsfc_cruise_gkwh:  cp.bsfc_gkwh,
            endurance_h,
            range_km,
            prop_efficiency:   cp.eta,
            thrust_cruise_n:   thrust,
            p_req_cruise_kw:   cp.p_req_kw,
            p_shaft_cruise_kw: cp.p_shaft_kw,
            cruise_feasible,
        }
    }
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::engine::test_fixtures::{motor_generico_teste as engine_teste,
                                                 motor_generico_fraco_teste as engine_fraco_teste};

    #[test]
    fn prop_rpm_correto() {
        // Motor a 2.400 rpm com PSRU 1.867 → hélice a 1.285 rpm
        let n = prop_rpm(2_400.0, 1.867);
        assert!((n - 1_285.0).abs() < 5.0, "RPM hélice = {n:.0} (esperado ~1.285)");
    }

    #[test]
    fn eficiencia_helice_cruzeiro() {
        // J típico de cruzeiro: V=77.8 m/s, n_prop=1.285 rpm, D=1.95m
        let j = advance_ratio(77.8, 1_285.0, 1.95);
        let eta = prop_efficiency(j);
        println!("J = {j:.3}, η_prop = {eta:.3}");
        assert!(eta > 0.78 && eta < 0.90,
            "Eficiência hélice {eta:.3} fora do intervalo esperado (0.78–0.90)");
    }

    #[test]
    fn consumo_cresce_com_bsfc() {
        // Para a mesma potência, BSFC maior → consumo maior (mesma densidade).
        let fc_baixo = fuel_consumption_lph(99.0, 200.0, 0.840);
        let fc_alto  = fuel_consumption_lph(99.0, 300.0, 0.840);
        assert!(fc_alto > fc_baixo, "consumo deveria crescer com o BSFC");
    }

    #[test]
    fn consumo_decresce_com_densidade_maior() {
        // Mesma massa de combustível/hora, mas combustível mais denso →
        // menos litros/hora para a mesma massa.
        let fc_denso  = fuel_consumption_lph(99.0, 220.0, 0.84);
        let fc_leve   = fuel_consumption_lph(99.0, 220.0, 0.72);
        assert!(fc_leve > fc_denso,
            "combustível menos denso deveria exigir mais L/h para a mesma massa/h");
    }

    #[test]
    fn run_com_motor_generico_produz_especificacao_coerente() {
        use crate::agents::aerodynamics::AerodynamicsAgent;
        use crate::models::aircraft_state::AircraftState;
        use crate::models::aircraft_config::test_fixtures::config_teste;

        let cfg    = config_teste();
        let state  = AircraftState::from_config(&cfg);
        let req    = crate::models::requirements::test_fixtures::requisitos_teste();
        let wing   = AerodynamicsAgent::run(&state, &req);
        let engine = engine_teste();

        let prop = PropulsionAgent::run(&state, &req, &wing, &engine);

        assert_eq!(prop.engine_model, "Motor Sintético de Teste");
        assert_eq!(prop.engine_mass_kg, 150.0);
        assert!(prop.fc_cruise_lph > 0.0);
        assert!(prop.bsfc_cruise_gkwh >= engine.bsfc.bsfc_min_gkwh);
        assert!(prop.endurance_h > 0.0);
        assert!(prop.prop_efficiency > 0.0 && prop.prop_efficiency <= 0.86);
    }

    #[test]
    fn motor_fraco_marca_cruzeiro_inviavel() {
        use crate::agents::aerodynamics::AerodynamicsAgent;
        use crate::models::aircraft_state::AircraftState;
        use crate::models::aircraft_config::test_fixtures::config_teste;

        let cfg    = config_teste();
        let state  = AircraftState::from_config(&cfg);
        let req    = crate::models::requirements::test_fixtures::requisitos_teste();
        let wing   = AerodynamicsAgent::run(&state, &req);
        let engine = engine_fraco_teste();

        let prop = PropulsionAgent::run(&state, &req, &wing, &engine);

        assert!(!prop.cruise_feasible,
            "motor fraco (potência de pico ~52 kW) não deveria sustentar 280 km/h");
        assert!(prop.p_req_cruise_kw > prop.p_shaft_cruise_kw);
    }

    #[test]
    fn motor_forte_marca_cruzeiro_viavel() {
        use crate::agents::aerodynamics::AerodynamicsAgent;
        use crate::models::aircraft_state::AircraftState;
        use crate::models::aircraft_config::test_fixtures::config_teste;

        let cfg    = config_teste();
        let state  = AircraftState::from_config(&cfg);
        let req    = crate::models::requirements::test_fixtures::requisitos_teste();
        let wing   = AerodynamicsAgent::run(&state, &req);
        let engine = engine_teste();

        let prop = PropulsionAgent::run(&state, &req, &wing, &engine);

        println!("motor_forte: P_req={:.1}kW P_shaft={:.1}kW @ {:.0}rpm",
            prop.p_req_cruise_kw, prop.p_shaft_cruise_kw, prop.engine_rpm_cruise);
        assert!(prop.cruise_feasible,
            "motor de teste (potência de pico ~147 kW) deveria sustentar 280 km/h");
        assert!(prop.p_req_cruise_kw <= prop.p_shaft_cruise_kw);
        // Margem positiva (não só tecnicamente viável) — confirma que a
        // fixture não está exatamente no ponto de equilíbrio (o que tornaria
        // o teste frágil a qualquer ajuste futuro de física com efeito de
        // poucos %). Margem observada empiricamente para `config_teste()`
        // (célula sintética da Task 2.1, mais "pesada" em arrasto/psru que o
        // baseline real): ~1.4% — bem menor que a folga anterior (~seria
        // maior com a célula antiga hardcoded), mas ainda estritamente
        // positiva e suficiente para pegar uma regressão real para
        // inviabilidade.
        assert!(prop.p_shaft_cruise_kw > prop.p_req_cruise_kw * 1.005,
            "margem de viabilidade P_shaft/P_req = {:.3} — muito apertada (esperado > 1.005)",
            prop.p_shaft_cruise_kw / prop.p_req_cruise_kw);
    }
}
