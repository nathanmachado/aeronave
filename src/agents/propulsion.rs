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

// `old→new` (ciclo 13, spec §4): `prop_efficiency(J)` (polinômio JavaProp,
// η = -0,15·J²+0,39·J+0,58) e `thrust_n(eta, P, V)` foram APAGADAS — a
// primeira violava o teto de quantidade de movimento em 4 dos 8 pontos de
// operação do baseline (spec §1.1) e devolvia η(0)=0,58 (fisicamente errado:
// por definição η=T·V/P→0 quando V→0); a segunda tinha a guarda
// `v_ms < 1.0 { return 0.0 }` que era a origem da janela de tração NULA em
// V ∈ [0,5; 1,0). As duas foram substituídas por `FigureOfMerit` (ver
// `agents::performance::thrust_available_n`) — η vira SAÍDA derivada da lei
// única (spec §5), não entrada polinomial.

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
///
/// **Ciclo 13 (spec §5): inversão em FORMA FECHADA, não mais polinômio.**
/// Antes, para cada rpm candidata: `eta = prop_efficiency(J)` (polinômio
/// JavaProp, apagado — spec §4) e `p_req_kw = drag_n·V/(eta·1000)`. Com a
/// lei única, η depende da potência efetivamente absorvida — e em cruzeiro
/// nivelado a tração exigida é conhecida (`T = drag_n`). Isso NÃO cria ponto
/// fixo: a quadrática do disco atuador inverte diretamente. De
/// `T = 2ρA·u·(u − V)`:
///
///   u = [ V + √(V² + 2T/(ρA)) ] / 2
///   P_ideal    = T · u
///   P_eixo_req = P_ideal / FoM(J)
///   η          = FoM(J) · V / u
///
/// `u` só depende de `T` (=drag_n, fixo), `V`, `ρ` e `A` — NÃO do rpm
/// candidato (a teoria de disco atuador abstrai a rotação: qualquer rpm que
/// entregue o mesmo `T` na mesma `V` precisa da MESMA velocidade induzida no
/// disco). Só `FoM(J)` varia entre candidatos, via `J = J(engine_rpm)`.
///
/// **Guarda obrigatória (spec §5):** se `FoM(J) ≤ 0` ou `u ≤ 0` (config
/// degenerada — nunca alcançável pelo caminho de produção validado, mesmo
/// tratamento do `eta > 0.0` de hoje), `p_req_kw = f64::INFINITY`. Nunca NaN.
fn search_cruise_rpm(
    engine: &EngineSpec,
    v_cruise_ms: f64,
    psru_ratio: f64,
    psru_efficiency: f64,
    prop_diameter_m: f64,
    altitude_m: f64,
    isa_delta_c: f64,
    fom: FigureOfMerit,
    drag_n: f64,
) -> (CruisePoint, bool) {
    let rpm_hi = engine.rpm_max_continuous.min(engine.bsfc.rpm_optimal * 1.2);
    // Se rpm_optimal·0.8 > rpm_hi (motor com rpm_max_continuous baixo em
    // relação ao rpm ótimo de BSFC), o limite inferior é grampeado ao
    // superior, reduzindo a varredura a um único ponto em vez de deixar o
    // laço vazio.
    let rpm_lo = (engine.bsfc.rpm_optimal * 0.8).min(rpm_hi);

    // `isa_delta_c` (achado da revisão de plano): a densidade da inversão
    // tem que ser a MESMA que gerou `drag_n` no chamador, senão a identidade
    // T = arrasto não fecha. Hoje todas as missões têm isa_delta_c = 0,0,
    // então hardcodar seria inofensivo E errado — bug latente para a
    // primeira missão com ISA ≠ 0, e violação da política "nunca hardcodar
    // dado de missão".
    let rho = crate::models::atmosphere::Isa::density_kgm3(altitude_m, isa_delta_c);
    let disk_area = std::f64::consts::PI * (prop_diameter_m / 2.0).powi(2);
    // Velocidade induzida no disco para produzir T=drag_n em V_cruise —
    // constante através da varredura de rpm (ver docstring acima).
    let u = (v_cruise_ms
             + (v_cruise_ms * v_cruise_ms + 2.0 * drag_n / (rho * disk_area)).sqrt())
            / 2.0;
    let p_ideal_kw = drag_n * u / 1_000.0;

    let evaluate = |engine_rpm: f64| -> CruisePoint {
        let prop_rpm_c = prop_rpm(engine_rpm, psru_ratio);
        let j = advance_ratio(v_cruise_ms, prop_rpm_c, prop_diameter_m);
        let fom_j = fom.at(j);
        let eta = if u > 0.0 { fom_j * v_cruise_ms / u } else { 0.0 };
        let p_req_kw = if fom_j > 0.0 && u > 0.0 {
            p_ideal_kw / fom_j
        } else {
            f64::INFINITY
        };
        let p_shaft_kw = engine.power_kw_at(engine_rpm, altitude_m) * psru_efficiency;
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
        // motor (ver `search_cruise_rpm`). `old→new` (ciclo 13): `fom` e
        // `req.isa_delta_c` são parâmetros novos — a inversão fechada do
        // disco atuador precisa das âncoras da hélice e da mesma densidade
        // que gerou `drag_n` acima (spec §5).
        let (cp, cruise_feasible) = search_cruise_rpm(
            engine, v_cruise_ms, state.psru_ratio, state.psru_efficiency, state.prop_diameter_m,
            req.cruise_altitude_m, req.isa_delta_c, state.figure_of_merit(), drag_n,
        );

        // `cp.p_req_kw` é potência de EIXO (pós-PSRU, na hélice) — BSFC
        // referencia o VIRABREQUIM (pré-PSRU). Achado da revisão da Task
        // 5.1 (Finding 2): sem dividir por `state.psru_efficiency`, o consumo
        // era subestimado em ~3% (`1/0,97 − 1`) — ver doc de
        // `fuel_consumption_lph` e a dedução em `agents::mission`.
        let fc_lph = fuel_consumption_lph(
            cp.p_req_kw / state.psru_efficiency, cp.bsfc_gkwh, engine.fuel.density_kg_per_l,
        );

        // Tração em cruzeiro: por CONSTRUÇÃO iguala o arrasto em regime
        // permanente (`T = drag_n`, a mesma premissa que `search_cruise_rpm`
        // usa para inverter a quadrática do disco atuador — spec §5).
        // `old→new` (ciclo 13): antes era `thrust_n(cp.eta, cp.p_req_kw·1000,
        // v_cruise_ms)` — `thrust_n` foi apagada (spec §4); usar `drag_n`
        // diretamente é MAIS direto, não uma aproximação nova (T=D já era a
        // identidade que o `thrust_n` antigo reproduzia algebricamente).
        let thrust = drag_n;

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

// ─── FIGURA DE MÉRITO DA HÉLICE ──────────────────────────────────────────────

/// Figura de mérito da hélice: tração REAL sobre tração IDEAL de disco
/// atuador na mesma potência de eixo (ciclo 13, spec §3).
///
///   FoM(J) = fom_static + (fom_design − fom_static)·min(J/j_design, 1)
///
/// Por definição `FoM ≤ 1` — uma hélice não produz mais tração que o disco
/// atuador ideal absorvendo a mesma potência (conservação de quantidade de
/// movimento, spec §1). Esta é a grandeza que substitui o polinômio
/// `prop_efficiency` apagado neste ciclo: aquele violava o teto físico em
/// QUATRO dos oito pontos de operação do baseline (spec §1.1) — rolagem a
/// V=10 (2,1432x) e V=20 (1,3417x), `V_LOF` (1,0372x) e `Vx` (1,0095x).
/// Os dois últimos alimentam gates que PASSAVAM: o balanço de rotação e o
/// gradiente CS 23.65.
///
/// As duas âncoras são propriedades da HÉLICE, vindas de `[propeller]` do
/// TOML — nunca hardcodadas aqui.
///   - `fom_static` (J=0): fator de McCormick, ≈0,75. Reproduz a tração
///     estática de hoje por IDENTIDADE algébrica (spec §3.1).
///   - `fom_design` (J=`j_design`): retro-derivada UMA VEZ do polinômio
///     JavaProp no ponto de cruzeiro do baseline, o que preserva
///     cruzeiro/alcance/autonomia por construção (spec §3.2).
///
/// PREMISSA CALIBRADA DECLARADA (spec §3.3): `j_design` foi derivada de
/// `prop_rpm_cruise`, que era SAÍDA da busca de rpm. Congelada em config, ela
/// NÃO se reajusta se a velocidade de cruzeiro, a razão de PSRU ou o diâmetro
/// mudarem — a âncora fica obsoleta em silêncio. Item de backlog nomeado.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FigureOfMerit {
    pub fom_static: f64,
    pub fom_design: f64,
    pub j_design: f64,
}

impl FigureOfMerit {
    /// Figura de mérito na razão de avanço `j`. Grampeada em `fom_design`
    /// acima de `j_design` (extrapolar levaria FoM > 1) e em `fom_static`
    /// abaixo de zero (J negativo não é alcançável, mas não pode virar NaN).
    pub fn at(&self, j: f64) -> f64 {
        if !(j > 0.0) {
            return self.fom_static;   // cobre j ≤ 0 e j NaN
        }
        let t = (j / self.j_design).min(1.0);
        self.fom_static + (self.fom_design - self.fom_static) * t
    }
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::engine::test_fixtures::{motor_generico_teste as engine_teste,
                                                 motor_generico_fraco_teste as engine_fraco_teste};

    /// Âncoras da figura de mérito — spec ciclo 13 §3. Os dois valores são
    /// EXATOS por construção da curva, não aproximados: `at(0)` devolve
    /// `fom_static` e `at(j_design)` devolve `fom_design` sem interpolação.
    #[test]
    fn figura_de_merito_reproduz_as_ancoras_exatamente() {
        let fom = FigureOfMerit {
            fom_static: 0.75,
            fom_design: 0.823_706_394_572_155_44,
            j_design:   1.875_143_480_257_116_75,
        };
        assert_eq!(fom.at(0.0), 0.75);
        assert_eq!(fom.at(1.875_143_480_257_116_75), 0.823_706_394_572_155_44);
    }

    /// Grampo acima de `j_design` (spec §3): a curva satura, não extrapola.
    /// Extrapolar linearmente levaria FoM acima de 1,0 em J alto — violaria o
    /// teto de quantidade de movimento que este ciclo inteiro existe para impor.
    #[test]
    fn figura_de_merito_satura_acima_do_j_de_projeto() {
        let fom = FigureOfMerit { fom_static: 0.75, fom_design: 0.82, j_design: 1.9 };
        assert_eq!(fom.at(1.9), 0.82);
        assert_eq!(fom.at(3.8), 0.82);
        assert_eq!(fom.at(50.0), 0.82);
    }

    /// FoM é uma FRAÇÃO da tração ideal — nunca pode passar de 1,0 nem chegar a
    /// zero com âncoras válidas. Guarda falseável do teto físico (spec §8.5).
    #[test]
    fn figura_de_merito_fica_estritamente_dentro_de_zero_e_um() {
        let fom = FigureOfMerit {
            fom_static: 0.75,
            fom_design: 0.823_706_394_572_155_44,
            j_design:   1.875_143_480_257_116_75,
        };
        for i in 0..=1000 {
            let j = i as f64 * 0.01;
            let v = fom.at(j);
            assert!(v > 0.0 && v <= 1.0, "FoM({j}) = {v} fora de (0, 1]");
        }
    }

    /// Monotonicidade não-decrescente (spec §8.5): as pás vão saindo do estol
    /// conforme a razão de avanço sobe; a figura de mérito não pode PIORAR com J
    /// dentro da faixa de projeto.
    #[test]
    fn figura_de_merito_e_monotonica_nao_decrescente() {
        let fom = FigureOfMerit {
            fom_static: 0.75,
            fom_design: 0.823_706_394_572_155_44,
            j_design:   1.875_143_480_257_116_75,
        };
        let mut anterior = fom.at(0.0);
        for i in 1..=1000 {
            let atual = fom.at(i as f64 * 0.01);
            assert!(atual >= anterior, "FoM caiu em J={}", i as f64 * 0.01);
            anterior = atual;
        }
    }

    /// J negativo não é fisicamente alcançável neste modelo (V ≥ 0), mas se
    /// chegar aqui a curva devolve o valor estático — nunca extrapola para baixo
    /// de `fom_static`, nunca NaN. Guarda de robustez, não de física.
    #[test]
    fn figura_de_merito_com_j_negativo_devolve_o_estatico() {
        let fom = FigureOfMerit { fom_static: 0.75, fom_design: 0.82, j_design: 1.9 };
        assert_eq!(fom.at(-1.0), 0.75);
    }

    #[test]
    fn prop_rpm_correto() {
        // Motor a 2.400 rpm com PSRU 1.867 → hélice a 1.285 rpm
        let n = prop_rpm(2_400.0, 1.867);
        assert!((n - 1_285.0).abs() < 5.0, "RPM hélice = {n:.0} (esperado ~1.285)");
    }

    /// `old→new` (ciclo 13, spec §4): `prop_efficiency(J)` (polinômio
    /// JavaProp) foi apagada — η vira SAÍDA derivada de `FigureOfMerit`, não
    /// entrada polinomial (spec §5). Âncoras SINTÉTICAS, deliberadamente
    /// diferentes do baseline real (spec §10 item 4) — mesmas de
    /// `aircraft_config::test_fixtures::config_teste().propeller`.
    #[test]
    fn figura_de_merito_em_j_tipico_de_cruzeiro() {
        // J típico de cruzeiro: V=77.8 m/s, n_prop=1.285 rpm, D=1.95m
        let j = advance_ratio(77.8, 1_285.0, 1.95);
        let fom = FigureOfMerit { fom_static: 0.72, fom_design: 0.80, j_design: 1.60 };
        let valor = fom.at(j);
        println!("J = {j:.3}, FoM = {valor:.3}");
        // J≈1,863 > j_design=1,60 ⟹ grampeado em fom_design (spec §3).
        assert_eq!(valor, 0.80, "FoM(J={j:.3}) deveria saturar em fom_design=0,80");
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
        // `old→new` (ciclo 13): 0.86 era o CLAMP DURO do polinômio apagado
        // (`prop_efficiency`, `.clamp(0.0, 0.86)`). A lei nova não tem esse
        // clamp — η = FoM(J)·V/u é estruturalmente < FoM(J) ≤ fom_design
        // (0,80 na fixture sintética, `config_teste().propeller`), então
        // 0,86 segue válido como teto de sanidade solto, não mais um limite
        // físico do modelo.
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
