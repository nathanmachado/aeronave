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

/// Tração VÁLIDA na faixa da rolagem de decolagem (0 → V_LOF) — ciclo 12,
/// spec §2. `thrust_available_n` não serve para este uso: seu ramo de voo
/// (`prop_efficiency(J)`, polinômio calibrado com dados de JavaProp na faixa
/// de CRUZEIRO, `J ≈ 1,3–1,5`) devolve η(0) = 0,58 — fisicamente errado (por
/// definição η = T·V/P ⟹ η→0 quando V→0) — e na corrida de decolagem desta
/// célula `J` varre 0 a 0,68, fora do domínio calibrado; medido: ≈80.000 N em
/// V=1 m/s (20× a tração estática), precedido por uma janela de tração NULA
/// em `[0,5; 1,0)` m/s (`thrust_n` tem uma guarda `if v_ms < 1.0 { return
/// 0.0 }`). Ver spec §1 para o histórico completo — o defeito ficou dormente
/// porque nenhum consumidor avaliava `prop_efficiency` nesta faixa até a
/// rolagem passar a integrar sobre `V ∈ [0, V_LOF]`.
///
/// Teoria de quantidade de movimento (Rankine-Froude) COM velocidade de
/// avanço, não só a tração estática em V=0. Com `u` = velocidade no disco
/// (metade da velocidade de esteira longe do disco mais V), `A = π·D²/4` e
/// `P` a potência de EIXO:
///
///   T = 2·ρ·A·u·(u − V)      [empuxo do disco]
///   P = T·u                   [potência ideal]
///   ⟹ u²·(u − V) = P/(2·ρ·A) =: K
///
/// Cúbica `u³ − V·u² − K = 0`. Para `K > 0` e `V ≥ 0` há exatamente uma raiz
/// real com `u > V` (`f(V) = −K < 0`, `f` estritamente crescente em
/// `u > 2V/3`, `f → +∞`) — resolvida por Newton a partir de
/// `u₀ = V + K^(1/3)`, que converge monotonicamente (bracket
/// `[V, V + K^(1/3) + V]`, provado no teste de identidade abaixo).
///
///   thrust_ground_roll_n(V) = static_thrust_factor · 2·ρ·A·u·(u − V)
///
/// **Continuidade exata com o modelo de hoje (spec §2.1).** Em V=0 a cúbica
/// degenera em `u³ = K` ⟹ `u = K^(1/3)` ⟹
/// `T = 2ρA·K^(2/3) = (2ρA·P²)^(1/3)` — ALGEBRICAMENTE idêntico a
/// `static_thrust_ideal_n`. `thrust_ground_roll_n(0)` reproduz
/// `thrust_available_n(0)` não por aproximação, mas por identidade —
/// verificado por `tracao_de_rolagem_em_v_zero_e_identica_ao_estatico_de_
/// hoje` a 1e-9 relativo. Esta função é uma REFINAÇÃO do modelo estático de
/// hoje, não uma substituição.
///
/// **Premissa calibrada esticada, declarada (spec §2.3).** `static_thrust_
/// factor` (McCormick, ≈0,75) é calibrado só para tração ESTÁTICA — esta
/// função o aplica como multiplicador PLANO em todo `V ∈ [0, V_LOF]`. A
/// alternativa (fator 1,0 acima de V=0) criaria um degrau de +33% na tração
/// logo depois da largada, pior. Direção do erro: se as perdas de ponta de
/// pá/rotação de esteira caírem com a velocidade de avanço, o modelo
/// SUBESTIMA a tração na segunda metade da corrida — é CONSERVADOR (a
/// rolagem sai maior, não menor). Assumido e registrado, não escondido.
///
/// **A costura em V_LOF não é mascarada (spec §2.4).** Em `V_LOF` esta
/// função e `thrust_available_n` divergem (domínios de validade disjuntos —
/// um calibrado para tração ESTÁTICA, o outro para `J` de CRUZEIRO); a
/// descontinuidade medida é item de backlog (unificar o modelo de tração),
/// fora de escopo deste ciclo.
///
/// NÃO recebe `psru_ratio` — teoria de quantidade de movimento não usa rpm
/// de hélice (a rotação da hélice entra em `prop_efficiency(J)`, que este
/// modelo não usa).
pub fn thrust_ground_roll_n(
    v_ms: f64,
    engine: &EngineSpec,
    engine_rpm: f64,
    prop_diam_m: f64,
    altitude_m: f64,
    isa_delta_c: f64,
    static_thrust_factor: f64,
    psru_efficiency: f64,
) -> f64 {
    let p_w = shaft_power_kw(engine, engine_rpm, altitude_m, psru_efficiency) * 1_000.0;
    let rho = Isa::density_kgm3(altitude_m, isa_delta_c);
    let disk_area = std::f64::consts::PI * (prop_diam_m / 2.0).powi(2);
    let k = p_w / (2.0 * rho * disk_area);

    // Guarda (Fix round 1 do report da task — achado do revisor): a spec
    // §2 assume `K > 0`. Se `K ≤ 0` (potência de eixo nula/negativa) E
    // `V = 0` ao mesmo tempo, `u₀ = V + K^(1/3) = 0` e a primeira iteração
    // de Newton calcula `f/fp = 0/0` — NaN SILENCIOSO, o modo de falha que
    // este projeto não aceita (o padrão é falhar no ponto de perigo, não
    // devolver um número que não é número). INALCANÇÁVEL hoje:
    // `psru_efficiency > 0` é validado em `models::config` (`require_
    // positive("propeller.psru_efficiency", ...)`) e `engine.power_kw_at`
    // é positiva para qualquer rpm válida — `K` nunca chega a zero pelo
    // caminho de produção. Se essa validação algum dia relaxar (ex.: motor
    // "desligado" modelado com P=0), sem esta guarda a rolagem devolveria
    // NaN em vez de `+INFINITY`/pane explícita — fisicamente correto seria
    // tração nula (sem potência, o disco atuador não produz empuxo).
    if k <= 0.0 {
        return 0.0;
    }

    // Newton a partir de u0 = V + K^(1/3) — em V=0, f(u0) = u0³ − K = 0
    // exatamente (converge em 1 iteração, prova a identidade do spec §2.1).
    let mut u = v_ms + k.cbrt();
    for _ in 0..100 {
        let f = u * u * u - v_ms * u * u - k;
        let fp = 3.0 * u * u - 2.0 * v_ms * u;
        let delta = f / fp;
        u -= delta;
        if delta.abs() < 1e-12 * u {
            break;
        }
    }
    static_thrust_factor * 2.0 * rho * disk_area * u * (u - v_ms)
}

/// CD da aeronave em atitude de solo, com trem ESTENDIDO (ciclo 12, spec
/// §3.1) — fonte única de verdade da rolagem de decolagem E de pouso (Task
/// 3) e do balanço de rotação (Task 4).
///
///   CD_roll = wing.cd0 + state.cd0_gear_fixed_increment + cd0_flap_extra
///             + cl_ground_roll²/(π·wing.aspect_ratio·wing.oswald_efficiency)
///
/// `state.cd0_gear_fixed_increment` (`[gear].cd0_fixed_increment`, hoje
/// consumido só quando `gear_retractable = false`): `wing.cd0` embute
/// `CD0_GEAR_RETRACTABLE = 0.0` — ou seja, é o CD0 de trem RECOLHIDO. Na
/// rolagem o trem está ESTENDIDO. Reusar este campo assume que um trem
/// retrátil ESTENDIDO tem arrasto próximo ao de um trem FIXO — levemente
/// OTIMISTA (o retrátil estendido não tem as carenagens de um trem fixo bem
/// projetado), assumido e declarado.
pub fn cd_ground_roll(
    wing: &WingSpec,
    state: &AircraftState,
    cl_ground_roll: f64,
    cd0_flap_extra: f64,
) -> f64 {
    wing.cd0
        + state.cd0_gear_fixed_increment
        + cd0_flap_extra
        + cl_ground_roll * cl_ground_roll
            / (std::f64::consts::PI * wing.aspect_ratio * wing.oswald_efficiency)
}

/// Simpson composto — usado pela integração de rolagem (decolagem, e pouso
/// na Task 3). `steps` é arredondado para cima ao próximo par (Simpson exige
/// um número par de subintervalos).
fn simpson_composto(f: impl Fn(f64) -> f64, a: f64, b: f64, steps: usize) -> f64 {
    let n = if steps % 2 == 1 { steps + 1 } else { steps.max(2) };
    let h = (b - a) / n as f64;
    let mut soma = f(a) + f(b);
    for i in 1..n {
        let x = a + i as f64 * h;
        let peso = if i % 2 == 0 { 2.0 } else { 4.0 };
        soma += peso * f(x);
    }
    soma * h / 3.0
}

/// Integra `S = ∫₀^{V_LOF} m·V / F_net(V) dV` por Simpson composto — o
/// integrador genérico da rolagem de decolagem (ciclo 12, spec §3.2),
/// parametrizado sobre tração/arrasto/sustentação para permitir o teste
/// contra a solução analítica fechada (`integrador_de_rolagem_reproduz_a_
/// solucao_analitica_sem_atrito_nem_arrasto`, spec §7.1) sem depender de
/// nenhum dos modelos físicos de produção.
///
/// `F_net(V) = thrust_fn(V) − drag_fn(V) − mu_roll·max(0, W − lift_fn(V))`,
/// `W = mass_kg·G`. Guarda falseável (spec §3.2/§7.6): se `F_net(V) ≤ 0` em
/// qualquer nó da malha de Simpson, o integrando naquele nó é `+INFINITY` —
/// como todos os pesos de Simpson são estritamente positivos, isso propaga
/// para o resultado inteiro (decolagem impossível nesta condição, resultado
/// FÍSICO, não erro; nunca NaN, porque o caso `F_net ≤ 0` é interceptado
/// ANTES da divisão, nunca por uma divisão por zero/negativo).
fn integra_rolagem_decolagem_com_passos(
    mass_kg: f64,
    v_lof: f64,
    thrust_fn: impl Fn(f64) -> f64,
    drag_fn: impl Fn(f64) -> f64,
    lift_fn: impl Fn(f64) -> f64,
    mu_roll: f64,
    steps: usize,
) -> f64 {
    let w = mass_kg * G;
    let integrando = |v: f64| -> f64 {
        let n = (w - lift_fn(v)).max(0.0);
        let f_net = thrust_fn(v) - drag_fn(v) - mu_roll * n;
        if f_net <= 0.0 {
            return f64::INFINITY;
        }
        mass_kg * v / f_net
    };
    simpson_composto(integrando, 0.0, v_lof, steps)
}

/// `integra_rolagem_decolagem_com_passos` na resolução de PRODUÇÃO (200
/// intervalos, spec §3.2) — convergência verificada contra 400 intervalos
/// por `integrador_de_rolagem_esta_convergido_na_resolucao_escolhida`
/// (spec §7.2). Só usada pelo teste contra a solução analítica fechada
/// (spec §7.1, `integrador_de_rolagem_reproduz_a_solucao_analitica_sem_
/// atrito_nem_arrasto`) — o caminho de PRODUÇÃO chama
/// `integra_rolagem_decolagem_com_passos` diretamente via
/// `takeoff_ground_roll_com_passos`/`takeoff_ground_roll_m`, `#[cfg(test)]`
/// evita o aviso de código morto fora de `cfg(test)`.
#[cfg(test)]
fn integra_rolagem_decolagem(
    mass_kg: f64,
    v_lof: f64,
    thrust_fn: impl Fn(f64) -> f64,
    drag_fn: impl Fn(f64) -> f64,
    lift_fn: impl Fn(f64) -> f64,
    mu_roll: f64,
) -> f64 {
    integra_rolagem_decolagem_com_passos(mass_kg, v_lof, thrust_fn, drag_fn, lift_fn, mu_roll, 200)
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
///
/// `cd0_extra` (ciclo 8, task 1): incremento de CD0 somado ao polar de
/// `drag_level_n` ANTES de calcular a potência necessária — fecha a lacuna
/// "não existe modelo de flap na polar deste crate" declarada desde o ciclo
/// 7. Cada chamador decide o valor: configuração de decolagem/subida em
/// configuração de decolagem passa `wing.cd0_flap_to_extra` (flap PARCIAL,
/// `to_flap_fraction · cd0_flap_delta`); configuração limpa/en-route (Vy,
/// teto de serviço, cruzeiro) passa `0.0` — ver auditoria de call sites em
/// cada função abaixo.
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
    cd0_extra: f64,
) -> f64 {
    let drag_limpo = drag_level_n(v_ms, mass_kg, rho, wing);
    let q = 0.5 * rho * v_ms * v_ms;
    let drag = drag_limpo + q * wing.area_m2 * cd0_extra;
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
/// Nota de modelo (ciclo 8, task 1 → ciclo 11, task 2 → ERRATUM ciclo 11,
/// rodada 2). O histórico preexistente desde ciclo 8 era uma inconsistência
/// — a referência de estol usava `wing.cl_max` (CL_max COM FLAP de pouso,
/// `cl_max_flaps` — ver `WingSpec::cl_max`), não o CL_max limpo, enquanto
/// `excess_power_kw` recebe `cd0_extra = 0.0` (configuração limpa). Vy é
/// referência de razão de subida EN-ROUTE (teto de serviço,
/// `service_ceiling_m`), não check de decolagem — não faz sentido misturar
/// CL de estol flapado com arrasto limpo.
///
/// CICLO 11, TASK 2 (primeira tentativa, INCOMPLETA — ver ERRATUM):
/// trocou a referência de estol para `wing.cl_max_clean` (1,45) mas manteve
/// a janela de varredura `[1,30·Vs, 1,80·Vs]` sem revisá-la. Essa janela
/// tinha sido calibrada (por observação, não por derivação) contra o Vs
/// FLAPADO — com o Vs LIMPO (20,3% maior), a janela inteira desloca para
/// velocidades maiores e PERDE o pico real de RC, que fica no baixo-médio da
/// janela antiga. `climb_rate_ms` passou a devolver o PISO da janela nova,
/// não o argmax de RC(V) — Vy 147,9→161,8 km/h, `rc_sl_ms` 5,0010→4,9533,
/// `service_ceiling_m` 5200→5100 m eram ARTEFATOS DE BUSCA: RC(V) é
/// IDÊNTICA antes e depois da troca de referência (CL_max não entra no
/// cálculo de RC — só posiciona a janela). Verificado por sondagem numérica
/// direta na revisão que escalou ao principal (ver ERRATUM,
/// `docs/superpowers/specs/2026-08-10-ciclo11-subida-honesta-design.md`).
///
/// CORRIGIDO (ERRATUM ciclo 11, rodada 2): a referência de estol limpa
/// (`wing.cl_max_clean`) fica — essa parte estava certa. A janela de
/// varredura muda de `[1,30·Vs, 1,80·Vs]` para **`[1,05·Vs, 2,00·Vs]`**
/// (`steps` 50→100) — larga o bastante para conter o argmax real em vez de
/// depender de uma heurística calibrada para outra referência de estol. Vy
/// é, por definição, o argmax de RC(V); a janela é ferramenta de busca, não
/// modelagem. Guarda de regressão: `climb_rate_search_window_kmh` expõe os
/// limites da janela e o argmax para o teste
/// `vy_argmax_e_estritamente_interior_a_janela_de_busca` (RED contra a
/// janela antiga, ver `tests/generic_engine.rs`) — exige argmax
/// ESTRITAMENTE INTERIOR no baseline real; argmax na fronteira de uma busca
/// por ótimo é defeito de modelo, não resultado. Efeito líquido esperado:
/// Vy/RC/teto voltam a ≈148 km/h / ≈5,001 m/s / 5200 m (o resultado
/// honesto — Vy genuinamente não depende do CL_max de referência, só a
/// janela de busca dependia, e agora é larga o bastante para não importar).
pub fn climb_rate_ms(
    mass_kg: f64,
    altitude_m: f64,
    isa_delta_c: f64,
    wing: &WingSpec,
    state: &AircraftState,
    engine: &EngineSpec,
    static_thrust_factor: f64,
) -> (f64, f64) {
    let r = climb_rate_search(mass_kg, altitude_m, isa_delta_c, wing, state, engine,
                               static_thrust_factor);
    (r.best_rc_ms.max(0.0), r.best_v_ms * 3.6) // (RC em m/s, Vy em km/h)
}

/// Detalhe da busca de Vy — expõe a janela `[v_min, v_max]` e o argmax
/// `best_v`, além do RC no argmax. Uso interno de `climb_rate_ms` (que
/// descarta os limites) e, via `climb_rate_search_window_kmh` abaixo,
/// da guarda de teste "argmax estritamente interior" (ERRATUM ciclo 11 §2).
struct ClimbRateSearch {
    best_rc_ms: f64,
    best_v_ms: f64,
    v_min_ms: f64,
    v_max_ms: f64,
}

fn climb_rate_search(
    mass_kg: f64,
    altitude_m: f64,
    isa_delta_c: f64,
    wing: &WingSpec,
    state: &AircraftState,
    engine: &EngineSpec,
    static_thrust_factor: f64,
) -> ClimbRateSearch {
    let rho = Isa::density_kgm3(altitude_m, isa_delta_c);
    // RPM de subida: máximo contínuo do motor (uso prolongado, não redline).
    let engine_rpm_climb = engine.rpm_max_continuous;

    let v_stall = ((2.0 * mass_kg * G) / (rho * wing.area_m2 * wing.cl_max_clean)).sqrt();

    // Janela de busca do argmax de RC(V) — ERRATUM ciclo 11 §2: `[1,05·Vs,
    // 2,00·Vs]` (era `[1,30·Vs, 1,80·Vs]`, calibrada para o Vs FLAPADO — ver
    // docstring de `climb_rate_ms` para o histórico completo). Larga o
    // bastante para conter o argmax real com a referência de estol LIMPA.
    let v_min = 1.05 * v_stall;
    let v_max = 2.00 * v_stall;
    let steps = 100;
    let dv = (v_max - v_min) / steps as f64;

    let mut best_rc = f64::NEG_INFINITY;
    let mut best_v  = v_min;

    for i in 0..=steps {
        let v = v_min + i as f64 * dv;
        // Vy é referência EN-ROUTE (teto de serviço), configuração limpa —
        // cd0_extra=0.0.
        let pex = excess_power_kw(v, mass_kg, rho, wing, engine,
                                   engine_rpm_climb, state.psru_ratio,
                                   state.prop_diameter_m, altitude_m, isa_delta_c,
                                   static_thrust_factor, state.psru_efficiency, 0.0);
        let rc = pex * 1_000.0 / (mass_kg * G);
        if rc > best_rc {
            best_rc = rc;
            best_v  = v;
        }
    }
    ClimbRateSearch { best_rc_ms: best_rc, best_v_ms: best_v, v_min_ms: v_min, v_max_ms: v_max }
}

/// Expõe a janela de busca de Vy e o argmax, todos em km/h, só para a guarda
/// de teste "argmax estritamente interior" (ERRATUM ciclo 11 §2, ver
/// docstring de `climb_rate_ms`). Não é consumida pelo pipeline de
/// produção — devolve os mesmos números que `climb_rate_ms` mais os limites
/// da janela que ele descarta.
///
/// Retorna `(best_v_kmh, v_min_kmh, v_max_kmh)`.
pub fn climb_rate_search_window_kmh(
    mass_kg: f64,
    altitude_m: f64,
    isa_delta_c: f64,
    wing: &WingSpec,
    state: &AircraftState,
    engine: &EngineSpec,
    static_thrust_factor: f64,
) -> (f64, f64, f64) {
    let r = climb_rate_search(mass_kg, altitude_m, isa_delta_c, wing, state, engine,
                               static_thrust_factor);
    (r.best_v_ms * 3.6, r.v_min_ms * 3.6, r.v_max_ms * 3.6)
}

/// Vx — velocidade de MELHOR ÂNGULO de subida (Task 4.7): maximiza o
/// gradiente RC(V)/V = sin(γ), equivalente a maximizar (T−D)/W — não RC(V)
/// absoluto (isso é Vy, ver `climb_rate_ms`). Fisicamente Vx < Vy sempre (a
/// curva RC/V tem pico mais próximo do stall que a curva RC).
///
/// Varredura de 1.2·Vs a 1.8·Vs (ciclo 11, task 1 — ver histórico abaixo).
///
/// Retorna `(gradiente_max, Vx_kmh)` — gradiente como FRAÇÃO adimensional
/// (RC/V, não %); `PerformanceAgent::run` converte para `climb_gradient_pct`.
///
/// HISTÓRICO (Task 4.7 → ciclo 7): a referência de estol usava `wing.cl_max`
/// — CL_max COM FLAP DE POUSO — enquanto o arrasto somado (via
/// `excess_power_kw`) não tinha nenhum incremento de flap: um híbrido "CL de
/// estol flapado (pouso) + arrasto limpo", não uma condição de decolagem CS
/// 23.65 fisicamente consistente (que exige flap de DECOLAGEM, parcial,
/// tanto no CL_max de referência quanto no CD0).
///
/// CORRIGIDO (ciclo 8, task 1): este é um check de configuração de
/// DECOLAGEM (CS 23.65), não de pouso — a referência de estol passa a ser
/// `wing.cl_max_to` (flap PARCIAL, mesma interpolação de `to_flap_fraction`
/// já usada pelas distâncias de decolagem) e `excess_power_kw` recebe
/// `wing.cd0_flap_to_extra` (`to_flap_fraction · cd0_flap_delta`) em vez de
/// 0.0 — CL_max de referência e CD0 agora refletem a MESMA configuração de
/// decolagem parcial. A queda do gradiente resultante tem DOIS
/// contribuintes, não só o arrasto: no baseline real (motor/célula de
/// referência), da tabela
/// old→new (15,129850%→13,896713%, −1,233137 p.p.) em
/// `tests/generic_engine.rs`, ~0,888 p.p. (≈72%) vêm do DESLOCAMENTO da
/// referência de estol (V_s_to > V_s0, muda o ponto de avaliação) e só
/// ~0,345 p.p. (≈28%) do arrasto de flap em si — ver pin
/// `climb_gradient_pct` (ciclo 8) para a decomposição completa.
///
/// CORRIGIDO (ciclo 11, task 1 — fecha `docs/backlog.md` item 2): até este
/// ciclo, a varredura cobria `[1,05·V_s, 1,80·V_s]`; para a célula/motor
/// real, RC/V é monotonicamente DECRESCENTE em toda essa faixa (achado da
/// revisão do ciclo 8), então a função devolvia, na prática, o LIMITE
/// INFERIOR da varredura (`1,05·V_s`), não um máximo interior genuíno — o
/// piso ficava ABAIXO da velocidade de avaliação típica da CS 23.65
/// (≥1,2·V_s), um viés OTIMISTA (avaliar mais cedo, com RC/V decrescente,
/// superestima o gradiente). O piso agora é `1,20·V_s_to`, delegado por
/// `best_climb_angle_com_piso` (função privada abaixo, piso exposto como
/// parâmetro só para permitir o teste de property
/// `gradiente_com_piso_de_varredura_maior_nao_aumenta` sem duplicar a
/// varredura). Valores MEDIDOS old→new no baseline real (piso
/// 1,05→1,20·V_s_to, tabela completa em `tests/generic_engine.rs`):
/// `climb_gradient_pct` 13,896713%→≈12,45% (queda esperada — RC/V decrescente
/// ⟹ avaliar mais tarde reduz o gradiente, este É o objetivo da correção,
/// não uma regressão); `vx_kmh` sobe proporcionalmente ao piso
/// (121,519501×(1,20/1,05) ≈ 138,9 km/h). O gradiente segue acima do piso
/// legal CS 23.65 (8,3%) — ver pin para a margem exata pós-correção.
fn best_climb_angle_com_piso(
    mass_kg: f64,
    altitude_m: f64,
    isa_delta_c: f64,
    wing: &WingSpec,
    state: &AircraftState,
    engine: &EngineSpec,
    static_thrust_factor: f64,
    piso: f64,
) -> (f64, f64) {
    let rho = Isa::density_kgm3(altitude_m, isa_delta_c);
    let engine_rpm_climb = engine.rpm_max_continuous;

    // Ciclo 8 (task 1): referência de estol de DECOLAGEM (flap PARCIAL) —
    // não mais `wing.cl_max` (pouso), ver docstring acima.
    let v_stall = ((2.0 * mass_kg * G) / (rho * wing.area_m2 * wing.cl_max_to)).sqrt();

    let v_min = piso * v_stall;
    let v_max = 1.80 * v_stall;
    let steps = 80;
    let dv = (v_max - v_min) / steps as f64;

    let mut best_grad = f64::NEG_INFINITY;
    let mut best_v = v_min;

    for i in 0..=steps {
        let v = v_min + i as f64 * dv;
        // Ciclo 8 (task 1): cd0_flap_to_extra — arrasto de flap PARCIAL de
        // decolagem, mesma fração da referência de estol acima.
        let pex = excess_power_kw(v, mass_kg, rho, wing, engine,
                                   engine_rpm_climb, state.psru_ratio,
                                   state.prop_diameter_m, altitude_m, isa_delta_c,
                                   static_thrust_factor, state.psru_efficiency,
                                   wing.cd0_flap_to_extra);
        let rc = pex * 1_000.0 / (mass_kg * G);
        let grad = rc / v;
        if grad > best_grad {
            best_grad = grad;
            best_v = v;
        }
    }
    (best_grad.max(0.0), best_v * 3.6) // (gradiente adimensional, Vx em km/h)
}

/// Gradiente CS 23.65 avaliado no piso legal da norma (≥1,2·Vs_to) — ver
/// docstring de `best_climb_angle_com_piso` para o histórico completo
/// (ciclo 11, task 1).
pub fn best_climb_angle_ms(
    mass_kg: f64,
    altitude_m: f64,
    isa_delta_c: f64,
    wing: &WingSpec,
    state: &AircraftState,
    engine: &EngineSpec,
    static_thrust_factor: f64,
) -> (f64, f64) {
    best_climb_angle_com_piso(mass_kg, altitude_m, isa_delta_c, wing, state, engine,
                               static_thrust_factor, 1.20)
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

/// Rolagem de solo na decolagem — integração numérica da equação de
/// movimento (ciclo 12, spec §2/§3), consumindo a polar completa:
///
///   m·dV/dt = T(V) − D(V) − μ_roll·N(V)
///   N(V) = max(0, W − L(V))
///   S = ∫₀^{V_LOF} m·V·dV / F_net(V)     (Simpson composto, 200 intervalos,
///                                          `integra_rolagem_decolagem`)
///   T(V) = `thrust_ground_roll_n`  (Rankine-Froude COM avanço, spec §2)
///   D(V) = q·S_w·CD_roll,  CD_roll = `cd_ground_roll` (spec §3.1)
///   L(V) = q·S_w·cl_ground_roll
///   V_LOF = 1,10·√(2W/(ρ·S_w·cl_max_to))
///
/// HISTÓRICO — DOCSTRING ANTIGA, RESCRITA `old→new` (ciclo 12): até este
/// ciclo a fórmula era o método ENERGÉTICO de Raymer (Cap. 5),
/// `S_G = W²/(g·ρ·S·CL_TO·T_avg)` com `CL_TO = 0,8·CL_max_TO` e `T_avg` a
/// tração estática corrigida — e a docstring anterior afirmava que o método
/// "por construção, não tem um termo de arrasto explícito ... T_avg já é a
/// tração LÍQUIDA média assumida constante ... Não há onde inserir
/// `cd0_flap_delta` sem reescrever o método inteiro — fora de escopo desta
/// task" (ciclo 8, task 1). **É exatamente esse método inteiro que este
/// ciclo reescreve** — ver spec §0/§1 para a motivação (a fórmula fechada
/// escondia a faixa `V ∈ [0, V_LOF]` onde `thrust_available_n` é
/// qualitativamente errada; integrar obriga a avaliar a tração nessa faixa
/// e força um modelo de tração válido nela — `thrust_ground_roll_n`).
/// `surface_factor` (1,00/1,15–1,20/1,25) SAI do caminho de decolagem —
/// com `mu_roll` explícito, multiplicar pelo fator de superfície contaria a
/// grama duas vezes (o 1,20 antigo ERA o atrito ausente, calibrado para
/// suprir justamente o que a fórmula energética não tinha — spec §4).
fn takeoff_ground_roll_com_passos(
    mass_kg: f64,
    rho: f64,
    wing: &WingSpec,
    state: &AircraftState,
    engine: &EngineSpec,
    isa_delta_c: f64,
    mu_roll: f64,
    cl_ground_roll: f64,
    static_thrust_factor: f64,
    steps: usize,
) -> f64 {
    let w = mass_kg * G;
    let v_lof = 1.10 * ((2.0 * w) / (rho * wing.area_m2 * wing.cl_max_to)).sqrt();
    // Ciclo 12, spec §3.1: flap PARCIAL de decolagem (`cd0_flap_to_extra`) —
    // a aeronave rola com o flap de decolagem já deflexionado.
    let cd_roll = cd_ground_roll(wing, state, cl_ground_roll, wing.cd0_flap_to_extra);
    let engine_rpm = engine.rpm_max_continuous;

    let thrust_fn = |v: f64| thrust_ground_roll_n(
        v, engine, engine_rpm, state.prop_diameter_m, 0.0, isa_delta_c,
        static_thrust_factor, state.psru_efficiency);
    let drag_fn = |v: f64| 0.5 * rho * v * v * wing.area_m2 * cd_roll;
    let lift_fn = |v: f64| 0.5 * rho * v * v * wing.area_m2 * cl_ground_roll;

    integra_rolagem_decolagem_com_passos(mass_kg, v_lof, thrust_fn, drag_fn, lift_fn, mu_roll,
                                          steps)
}

/// `takeoff_ground_roll_com_passos` na resolução de PRODUÇÃO (200
/// intervalos de Simpson) — ver docstring acima para o modelo físico
/// completo e o histórico `old→new` do método energético de Raymer que
/// esta função substitui.
fn takeoff_ground_roll_m(
    mass_kg: f64,
    rho: f64,
    wing: &WingSpec,
    state: &AircraftState,
    engine: &EngineSpec,
    isa_delta_c: f64,
    mu_roll: f64,
    cl_ground_roll: f64,
    static_thrust_factor: f64,
) -> f64 {
    takeoff_ground_roll_com_passos(mass_kg, rho, wing, state, engine, isa_delta_c, mu_roll,
                                    cl_ground_roll, static_thrust_factor, 200)
}

/// Distância de decolagem — rolagem de solo × 1,5 (aproximação de
/// transição de Raymer). MANTIDA como estimativa simplificada baseada em
/// rolagem de solo (ver `PerformanceSpec::to_distance_paved_m`/
/// `to_distance_grass_m`) — a distância física sobre obstáculo de 15m
/// (50 ft) por segmentos vive em `takeoff_distance_50ft_m` (Task 4.7), que
/// NÃO usa este fator ad hoc de 1,5.
///
/// Ciclo 12 (`old→new`): `surface_factor: f64` (1,00 pavimentado / 1,15–1,20
/// grama / 1,25 terra) SAI da assinatura, substituído por `mu_roll: f64` e
/// `cl_ground_roll: f64` — a rolagem de solo agora integra o atrito
/// explicitamente (`takeoff_ground_roll_m`), então multiplicar pelo fator de
/// superfície contaria a grama duas vezes (spec §4). `mu_roll` vem de
/// `[performance].mu_roll_paved`/`mu_roll_grass`; `cl_ground_roll` de
/// `[stability].cl_ground_rotation`.
pub fn takeoff_distance_m(
    mass_kg: f64,
    rho: f64,
    wing: &WingSpec,
    state: &AircraftState,
    mu_roll: f64,
    cl_ground_roll: f64,
    engine: &EngineSpec,
    isa_delta_c: f64,
    static_thrust_factor: f64,
) -> f64 {
    let s_ground = takeoff_ground_roll_m(mass_kg, rho, wing, state, engine, isa_delta_c,
                                          mu_roll, cl_ground_roll, static_thrust_factor);
    s_ground * 1.5
}

/// Distância de decolagem sobre obstáculo de 15m (50 ft), por segmentos
/// (Task 4.7) — substitui o fator ad hoc de transição ×1,5 por física real:
///
///   S_total = S_ground + S_rotação + S_subida
///     S_ground:  rolagem de solo integrada (`takeoff_ground_roll_m`, ciclo
///                12 — `mu_roll` já embute o atrito de superfície, sem
///                multiplicador adicional, ver docstring de
///                `takeoff_distance_m`)
///     S_rotação: V_LOF × `rotation_time_s` (rotação a V_LOF ≈ constante)
///     S_subida:  15 / tan(γ), γ = arcsin(RC/V) avaliado a 1,2·V_s_TO
///                (flap de decolagem, `cl_max_to`), potência de decolagem,
///                nível do mar
///
/// Ciclo 12 (`old→new`): mesma troca de assinatura de `takeoff_distance_m`
/// — `surface_factor` sai, `mu_roll`/`cl_ground_roll` entram.
pub fn takeoff_distance_50ft_m(
    mass_kg: f64,
    rho: f64,
    wing: &WingSpec,
    state: &AircraftState,
    mu_roll: f64,
    cl_ground_roll: f64,
    engine: &EngineSpec,
    isa_delta_c: f64,
    perf_cfg: &PerformanceCfg,
) -> f64 {
    let s_ground = takeoff_ground_roll_m(mass_kg, rho, wing, state, engine, isa_delta_c,
                                          mu_roll, cl_ground_roll, perf_cfg.static_thrust_factor);

    let w = mass_kg * G;
    // Ciclo 7 (task 1): `cl_max_to` (flap PARCIAL de decolagem), não o
    // `cl_max` de POUSO — V_s_to/V_LOF/V_climb são velocidades de
    // DECOLAGEM.
    let v_s_to = ((2.0 * w) / (rho * wing.area_m2 * wing.cl_max_to)).sqrt();
    let v_lo = 1.10 * v_s_to;
    let s_rotation = v_lo * perf_cfg.rotation_time_s;

    let v_climb = 1.20 * v_s_to;
    let engine_rpm_to = engine.rpm_max_continuous;
    // Ciclo 8 (task 1): cd0_flap_to_extra — segmento de SUBIDA consome a
    // polar de arrasto (rotação é puramente cinemática, V_LOF×tempo; o
    // segmento de SOLO consome a polar via `cd_ground_roll` desde o ciclo
    // 12, ver `takeoff_ground_roll_m`).
    let pex = excess_power_kw(v_climb, mass_kg, rho, wing, engine, engine_rpm_to,
                               state.psru_ratio, state.prop_diameter_m, 0.0, isa_delta_c,
                               perf_cfg.static_thrust_factor, state.psru_efficiency,
                               wing.cd0_flap_to_extra);
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
///
/// AUDITORIA (ciclo 8, task 1): nenhum dos três segmentos consome a polar de
/// arrasto (`wing.cd0`), logo nenhum recebe `cd0_flap_delta` — `cd0_flap_
/// ldg_extra` NÃO existe em `WingSpec` porque nada o consumiria (ver
/// docstring de `WingCfg::cd0_flap_delta`):
///   - S_ar usa um ângulo de aproximação FIXO (`approach_angle_deg`, dado de
///     projeto), não uma razão L/D derivada da polar — diferente de
///     `best_glide` (que usa `wing.cd0`, mas é planeio com motor cortado,
///     conceito distinto de aproximação de pouso com flap, fora de escopo);
///   - S_flare é puramente cinemático (V_ref × tempo fixo);
///   - S_ground é dominado por frenagem (`mu_brake`), sem termo de arrasto.
/// `wing.cl_max` (CL_max de POUSO, flap CHEIO) já é a referência correta de
/// V_s/V_ref aqui — isso não mudou.
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
    // Auditoria (ciclo 8, task 1): velocidade máxima nivelada é cruzeiro,
    // configuração limpa — cd0_extra=0.0.
    let pex = |v: f64| excess_power_kw(v, mass_kg, rho, wing, engine, engine_rpm,
                                       state.psru_ratio, state.prop_diameter_m, altitude_m,
                                       isa_delta_c, static_thrust_factor, state.psru_efficiency,
                                       0.0);

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
        // Ciclo 12 (task 2): CL de solo antes da rotação, consumido pela
        // rolagem de decolagem integrada (`takeoff_distance_m`/
        // `takeoff_distance_50ft_m`) — o mesmo `[stability].cl_ground_
        // rotation` que `agents::trim_authority` já usa na rotação, agora
        // reusado pela rolagem INTEIRA (assume atitude constante do início
        // da corrida até a rotação, ver spec §3.1). Parâmetro novo: antes
        // desta task `PerformanceAgent::run` não recebia nada de
        // `[stability]`.
        cl_ground_rotation: f64,
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

        // Distâncias de decolagem (rolagem × 1,5, estimativa simplificada).
        // Ciclo 12: `surface_factor` (1,00/1,20) sai, `mu_roll_paved`/
        // `mu_roll_grass` entram — a rolagem integrada já cobra o atrito de
        // superfície explicitamente (spec §4).
        let d_to_paved = takeoff_distance_m(mtow_kg, rho_sl, wing, state,
                                             perf_cfg.mu_roll_paved, cl_ground_rotation, engine,
                                             isa_delta_c, stf);
        let d_to_grass  = takeoff_distance_m(mtow_kg, rho_sl, wing, state,
                                              perf_cfg.mu_roll_grass, cl_ground_rotation, engine,
                                              isa_delta_c, stf);

        // Distâncias de decolagem sobre obstáculo de 15m/50ft, por segmentos
        // (Task 4.7)
        let d_to_50ft_paved = takeoff_distance_50ft_m(mtow_kg, rho_sl, wing, state,
                                                        perf_cfg.mu_roll_paved, cl_ground_rotation,
                                                        engine, isa_delta_c, perf_cfg);
        let d_to_50ft_grass = takeoff_distance_50ft_m(mtow_kg, rho_sl, wing, state,
                                                        perf_cfg.mu_roll_grass, cl_ground_rotation,
                                                        engine, isa_delta_c, perf_cfg);

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

    // ─── Ciclo 12 (task 2): fixtures locais para os testes de rolagem ───────
    //
    // `fixture_baseline()`, `MTOW_PIN_KG` não existem no repositório antes
    // desta task — são a forma que os testes de `thrust_ground_roll_n`/
    // integrador precisam (padrão orientado pelo plano: reaproveitar
    // `config_teste()`/`setup()`, nenhuma fixture nova em arquivo separado).
    // Nenhum destes testes precisa bater com os números congelados — são
    // todos propriedades relacionais, então `MTOW_PIN_KG` não precisa
    // coincidir com o MTOW real calculado por `config_teste()`. `RHO_SL` já
    // está importado acima (`crate::models::atmosphere::RHO_SL`).
    fn fixture_baseline() -> (EngineSpec, AircraftState, WingSpec) {
        let (state, wing, _prop, engine, _req, _perf_cfg) = setup();
        (engine, state, wing)
    }

    const MTOW_PIN_KG: f64 = 1500.0;

    /// CL de solo antes da rotação (`[stability].cl_ground_rotation`) da
    /// fixture sintética — helper para os testes que não desestruturam o
    /// `AircraftConfig` inteiro via `setup()`.
    fn cl_ground_rotation_teste() -> f64 {
        config_teste().stability.cl_ground_rotation
    }

    // ─── Ciclo 12 (task 2), spec §2: `thrust_ground_roll_n` ─────────────────

    /// Ciclo 12, spec §2.1: em V=0 a cúbica degenera em u³ = K e o empuxo
    /// resultante é ALGEBRICAMENTE (2ρA·P²)^(1/3) — exatamente o modelo
    /// estático de hoje. `thrust_ground_roll_n` é uma REFINAÇÃO do modelo
    /// atual, não uma substituição: o ponto V=0 tem de coincidir.
    #[test]
    fn tracao_de_rolagem_em_v_zero_e_identica_ao_estatico_de_hoje() {
        let (engine, state, _wing) = fixture_baseline();
        let novo = thrust_ground_roll_n(
            0.0, &engine, engine.rpm_max_continuous, state.prop_diameter_m,
            0.0, 0.0, 0.75, state.psru_efficiency);
        let hoje = thrust_available_n(
            0.0, &engine, engine.rpm_max_continuous, state.psru_ratio,
            state.prop_diameter_m, 0.0, 0.0, 0.75, state.psru_efficiency);
        let erro_rel = (novo - hoje).abs() / hoje;
        assert!(erro_rel < 1e-9, "novo={novo}, hoje={hoje}, erro_rel={erro_rel}");
    }

    /// A tração cai monotonicamente com a velocidade de avanço a potência
    /// constante. Falseável: se a resolução da cúbica errar o ramo, isto
    /// quebra.
    #[test]
    fn tracao_de_rolagem_cai_estritamente_com_a_velocidade() {
        let (engine, state, _wing) = fixture_baseline();
        let t = |v: f64| thrust_ground_roll_n(
            v, &engine, engine.rpm_max_continuous, state.prop_diameter_m,
            0.0, 0.0, 0.75, state.psru_efficiency);
        let (t0, t10, t20, t36) = (t(0.0), t(10.0), t(20.0), t(36.0));
        assert!(t0 > t10 && t10 > t20 && t20 > t36,
                "esperado estritamente decrescente: {t0} {t10} {t20} {t36}");
        assert!(t36 > 0.0, "tração tem de ser positiva em V_LOF: {t36}");
    }

    /// Fix round 1 do report da task (achado do revisor) — guarda de NaN.
    /// `K = P_eixo/(2ρA) ≤ 0` com `V = 0` faz a primeira iteração de Newton
    /// calcular `0/0`. Este cenário é INALCANÇÁVEL pelo caminho de
    /// produção (`psru_efficiency > 0` é validado em `models::config`),
    /// mas chamar a função pura diretamente com `psru_efficiency = 0.0`
    /// (fora da config validada, de propósito) prova a guarda: sem ela,
    /// este teste veria NaN.
    #[test]
    fn tracao_de_rolagem_nao_produz_nan_quando_potencia_de_eixo_e_nula() {
        let (engine, state, _wing) = fixture_baseline();
        let t = thrust_ground_roll_n(
            0.0, &engine, engine.rpm_max_continuous, state.prop_diameter_m,
            0.0, 0.0, 0.75, 0.0 /* psru_efficiency=0 ⟹ P_eixo=0 ⟹ K=0 */);
        assert!(!t.is_nan(), "K=0 com V=0 não pode devolver NaN: {t}");
        assert_eq!(t, 0.0, "sem potência de eixo, a tração devolvida deve ser exatamente zero: {t}");
    }

    // ─── Ciclo 12 (task 2), spec §3/§7: integrador de rolagem ───────────────

    /// Spec §7.1 — prova contra fechada analítica, não contra pin: com
    /// atrito e arrasto nulos e tração constante, a integração TEM de
    /// reproduzir S = ½·m·V²/T exatamente.
    #[test]
    fn integrador_de_rolagem_reproduz_a_solucao_analitica_sem_atrito_nem_arrasto() {
        let m = 1500.0_f64;
        let v_lof = 35.0_f64;
        let t_const = 3000.0_f64;
        let s = integra_rolagem_decolagem(m, v_lof, |_v| t_const, |_v| 0.0, |_v| 0.0, 0.0);
        let analitico = 0.5 * m * v_lof * v_lof / t_const;
        let erro_rel = (s - analitico).abs() / analitico;
        assert!(erro_rel < 1e-9, "s={s}, analitico={analitico}, erro_rel={erro_rel}");
    }

    /// Spec §7.2 — resultado em resolução não convergida é DEFEITO, não
    /// resultado. Mesma lição do argmax na fronteira (ciclo 11).
    #[test]
    fn integrador_de_rolagem_esta_convergido_na_resolucao_escolhida() {
        let (engine, state, wing) = fixture_baseline();
        let s_200 = takeoff_ground_roll_com_passos(MTOW_PIN_KG, RHO_SL, &wing, &state,
                                                    &engine, 0.0, 0.04, 0.5, 0.75, 200);
        let s_400 = takeoff_ground_roll_com_passos(MTOW_PIN_KG, RHO_SL, &wing, &state,
                                                    &engine, 0.0, 0.04, 0.5, 0.75, 400);
        let dif_rel = (s_400 - s_200).abs() / s_200;
        assert!(dif_rel < 1e-3, "não convergido: 200={s_200}, 400={s_400}, dif={dif_rel}");
    }

    /// Spec §7.3 — monotonicidades ESTRITAS: μ maior ⟹ rolagem maior; peso
    /// maior ⟹ rolagem maior. Cada uma é falseável: se o sinal de um termo
    /// estiver trocado, uma delas quebra. As outras duas exigidas pela spec
    /// (CD0 maior ⟹ maior; tração maior ⟹ menor) vivem em
    /// `rolagem_de_decolagem_aumenta_estritamente_com_cd0_maior` e
    /// `rolagem_de_decolagem_cai_estritamente_com_tracao_maior` — Fix round
    /// 1 do report da task (o plano só cobriu duas das quatro; achado do
    /// revisor: trocar o sinal do arrasto em `F_net` fazia os 27 testes
    /// locais de `performance.rs` passarem mesmo assim, inclusive este —
    /// só o pin de 1% de `tests/generic_engine.rs` e um flip de robustez,
    /// em outro arquivo, pegavam o bug).
    #[test]
    fn rolagem_de_decolagem_responde_no_sentido_certo_a_cada_termo() {
        let (engine, state, wing) = fixture_baseline();
        let roll = |mu: f64, mass: f64| takeoff_ground_roll_m(
            mass, RHO_SL, &wing, &state, &engine, 0.0, mu, 0.5, 0.75);
        assert!(roll(0.08, MTOW_PIN_KG) > roll(0.04, MTOW_PIN_KG), "atrito maior ⟹ rolagem maior");
        assert!(roll(0.04, MTOW_PIN_KG) > roll(0.04, MTOW_PIN_KG * 0.8), "peso maior ⟹ rolagem maior");
    }

    /// Spec §7.3 (Fix round 1) — CD0 maior ⟹ mais arrasto ⟹ rolagem maior.
    /// Falseável sozinha: se o sinal do termo de arrasto em `F_net` for
    /// trocado (arrasto SOMADO em vez de SUBTRAÍDO), CD0 maior passa a dar
    /// F_net MAIOR ⟹ rolagem MENOR — esta asserção quebra na hora. Muda só
    /// `wing.cd0`, resto da config fixo.
    #[test]
    fn rolagem_de_decolagem_aumenta_estritamente_com_cd0_maior() {
        let (engine, state, mut wing) = fixture_baseline();
        let roll_baixo = takeoff_ground_roll_m(
            MTOW_PIN_KG, RHO_SL, &wing, &state, &engine, 0.0, 0.04, 0.5, 0.75);
        wing.cd0 += 0.05; // CD0 bem maior — resto da fixture intacto
        let roll_alto = takeoff_ground_roll_m(
            MTOW_PIN_KG, RHO_SL, &wing, &state, &engine, 0.0, 0.04, 0.5, 0.75);
        assert!(roll_alto > roll_baixo,
            "CD0 maior deveria dar rolagem ESTRITAMENTE maior (mais arrasto): \
             roll(cd0 baixo)={roll_baixo}, roll(cd0 alto)={roll_alto}");
    }

    /// Spec §7.3 (Fix round 1) — tração maior ⟹ mais força líquida ⟹
    /// rolagem menor. Varia `static_thrust_factor` (parâmetro direto de
    /// `takeoff_ground_roll_m`, entra em `thrust_ground_roll_n` como
    /// multiplicador plano — spec §2.3), resto da config fixo. Falseável
    /// sozinha: se o sinal do termo de tração em `F_net` for trocado
    /// (tração SUBTRAÍDA em vez de SOMADA), tração maior passaria a dar
    /// F_net MENOR ⟹ rolagem MAIOR — esta asserção quebra na hora.
    #[test]
    fn rolagem_de_decolagem_cai_estritamente_com_tracao_maior() {
        let (engine, state, wing) = fixture_baseline();
        let roll = |stf: f64| takeoff_ground_roll_m(
            MTOW_PIN_KG, RHO_SL, &wing, &state, &engine, 0.0, 0.04, 0.5, stf);
        let roll_tracao_baixa = roll(0.60);
        let roll_tracao_alta  = roll(0.90);
        assert!(roll_tracao_baixa > roll_tracao_alta,
            "tração maior (static_thrust_factor maior) deveria dar rolagem ESTRITAMENTE menor: \
             roll(stf=0.60)={roll_tracao_baixa}, roll(stf=0.90)={roll_tracao_alta}");
    }

    /// Spec §7.4 — se a sustentação superar o peso antes de V_LOF, o
    /// atrito é ZERO, nunca negativo (atrito negativo empurraria a
    /// aeronave para a frente e ENCURTARIA a rolagem).
    #[test]
    fn atrito_nunca_fica_negativo_quando_a_sustentacao_supera_o_peso() {
        let (engine, state, mut wing) = fixture_baseline();
        wing.cl_max_to = 0.30; // V_LOF absurdamente alta ⟹ L ≫ W antes do fim
        let com_atrito_alto = takeoff_ground_roll_m(
            MTOW_PIN_KG, RHO_SL, &wing, &state, &engine, 0.0, 0.50, 3.0, 0.75);
        let com_atrito_zero = takeoff_ground_roll_m(
            MTOW_PIN_KG, RHO_SL, &wing, &state, &engine, 0.0, 0.00, 3.0, 0.75);
        assert!(com_atrito_alto >= com_atrito_zero,
                "atrito nunca pode ENCURTAR a rolagem: {com_atrito_alto} vs {com_atrito_zero}");
    }

    /// Spec §7.6 — o ramo que a Task 1 (serde de +INFINITY) pressupõe
    /// existir em produção. No baseline real a tração sobra folgadamente
    /// (em V_LOF: T=2.324,7 N contra D+μN≈900 N), então este ramo NUNCA é
    /// exercitado sem um cenário adversarial construído de propósito. Sem
    /// este teste, um `if F <= 0 { return INFINITY }` esquecido — ou um
    /// NaN silencioso — passa despercebido.
    #[test]
    fn tracao_insuficiente_devolve_infinito_e_nao_numero_espurio() {
        let (engine, state, wing) = fixture_baseline();
        // Atrito absurdo: nenhuma tração desta célula acelera a aeronave.
        let s = takeoff_ground_roll_m(
            MTOW_PIN_KG, RHO_SL, &wing, &state, &engine, 0.0, 5.0, 0.5, 0.75);
        assert!(s.is_infinite(), "esperado +INFINITY, veio {s}");
        assert!(!s.is_nan(), "NaN é o modo de falha silencioso a evitar");
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
        // varredura de `climb_rate_ms` (sempre V ≥ 1,05·Vs — ERRATUM ciclo 11
        // §2, era 1,3·Vs).
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

    // ─── Ciclo 8 (task 1): arrasto de flap na polar, gradiente honesto ──────

    /// Hand-check direto: `excess_power_kw` com `cd0_extra=0.01` deve ser
    /// ESTRITAMENTE menor que com `0.0` (mesmos demais argumentos) — mais
    /// CD0 ⟹ mais arrasto ⟹ mais potência necessária ⟹ menos excesso.
    #[test]
    fn excess_power_kw_cai_estritamente_com_cd0_extra() {
        let (state, wing, _prop, engine, _req, perf_cfg) = setup();
        let v = 60.0; // m/s, velocidade genérica de subida
        let rho = RHO_SL;
        let pex_limpo = excess_power_kw(v, state.mtow_kg, rho, &wing, &engine,
                                         engine.rpm_max_continuous, state.psru_ratio,
                                         state.prop_diameter_m, 0.0, 0.0,
                                         perf_cfg.static_thrust_factor, state.psru_efficiency, 0.0);
        let pex_com_extra = excess_power_kw(v, state.mtow_kg, rho, &wing, &engine,
                                             engine.rpm_max_continuous, state.psru_ratio,
                                             state.prop_diameter_m, 0.0, 0.0,
                                             perf_cfg.static_thrust_factor, state.psru_efficiency,
                                             0.01);
        println!("P_excesso(cd0_extra=0.0)={pex_limpo:.3}kW  P_excesso(cd0_extra=0.01)={pex_com_extra:.3}kW");
        assert!(pex_com_extra < pex_limpo,
            "excesso de potência com cd0_extra=0.01 ({pex_com_extra:.3}kW) deveria ser \
             ESTRITAMENTE menor que com cd0_extra=0.0 ({pex_limpo:.3}kW)");
    }

    /// Property (estrita): o gradiente CS 23.65 (`best_climb_angle_ms`, que
    /// desde esta task usa `cl_max_to` + `cd0_flap_to_extra`) CAI quando
    /// `[wing].cd0_flap_delta` sobe — mais arrasto de flap parcial na
    /// configuração de decolagem, menos gradiente. Resto da config idêntico.
    #[test]
    fn gradiente_cs2365_cai_estritamente_com_cd0_flap_delta_maior() {
        let req = crate::models::requirements::test_fixtures::requisitos_teste();
        let engine = engine_teste();

        let gradiente_para = |delta: f64| {
            let mut cfg = config_teste();
            cfg.wing.cd0_flap_delta = delta;
            let state = AircraftState::from_config(&cfg);
            let wing = AerodynamicsAgent::run(&state, &req);
            let (grad, _vx) = best_climb_angle_ms(state.mtow_kg, 0.0, 0.0, &wing, &state, &engine,
                                                   cfg.performance.static_thrust_factor);
            grad
        };

        let grad_baixo = gradiente_para(0.006);
        let grad_alto  = gradiente_para(0.045);
        println!("gradiente(cd0_flap_delta=0.006)={grad_baixo:.6}  \
                   gradiente(cd0_flap_delta=0.045)={grad_alto:.6}");
        assert!(grad_alto < grad_baixo,
            "gradiente com cd0_flap_delta MAIOR (0.045, {grad_alto:.6}) deveria ser \
             ESTRITAMENTE menor que com delta menor (0.006, {grad_baixo:.6})");
    }

    // ─── Ciclo 11 (task 1): gradiente CS 23.65 avaliado a 1,2·Vs_to ─────────

    /// Property (ESTRITA — `<`, não `<=`; Fix wave ciclo 11, 2026-08-10):
    /// para esta célula/motor RC/V é monotonicamente DECRESCENTE em
    /// `[1,05·V_s, 1,80·V_s]` (achado do ciclo 8/backlog item 2), então
    /// subir o PISO da varredura de 1,05·Vs para 1,20·Vs sempre DIMINUI o
    /// gradiente encontrado. Valores medidos: 0,146417 (piso 1,05·Vs) vs
    /// 0,128641 (piso 1,20·Vs) — folga grande o bastante para que `<`
    /// estrito não seja frágil, e com ele a property passa a detectar
    /// sozinha um mutante que ignore o parâmetro `piso` (com `<=`, um
    /// mutante que sempre devolvesse `grad_120 == grad_105` passaria).
    /// Usa `best_climb_angle_com_piso`
    /// (privada ao módulo, parâmetro de piso exposto só para teste) em vez
    /// de duplicar a varredura — decisão do implementador (task-1-brief.md):
    /// evita cópia da lógica de busca no teste.
    #[test]
    fn gradiente_com_piso_de_varredura_maior_nao_aumenta() {
        let (state, wing, _prop, engine, _req, perf_cfg) = setup();
        let stf = perf_cfg.static_thrust_factor;
        let (grad_105, _vx_105) = best_climb_angle_com_piso(state.mtow_kg, 0.0, 0.0, &wing, &state,
                                                              &engine, stf, 1.05);
        let (grad_120, _vx_120) = best_climb_angle_com_piso(state.mtow_kg, 0.0, 0.0, &wing, &state,
                                                              &engine, stf, 1.20);
        println!("gradiente(piso=1,05·Vs)={grad_105:.6}  gradiente(piso=1,20·Vs)={grad_120:.6}");
        // Fix wave ciclo 11 (2026-08-10): `<` estrito, não `<=` — valores
        // medidos (0,128641 vs 0,146417) têm folga enorme, sem risco de
        // flakiness, e a desigualdade estrita detecta sozinha um mutante que
        // ignore o parâmetro `piso` (`<=` passaria mesmo com grad_120 ==
        // grad_105).
        assert!(grad_120 < grad_105,
            "gradiente com piso de varredura MAIOR (1,20·Vs, {grad_120:.6}) deveria ser \
             ESTRITAMENTE menor que com piso menor (1,05·Vs, {grad_105:.6}) — RC/V é decrescente \
             na faixa modelada para esta célula");
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
        let cl_ground_roll = cl_ground_rotation_teste();
        let s_ground = takeoff_ground_roll_m(state.mtow_kg, RHO_SL, &wing, &state, &engine, 0.0,
                                              perf_cfg.mu_roll_paved, cl_ground_roll,
                                              perf_cfg.static_thrust_factor);
        let s_50ft = takeoff_distance_50ft_m(state.mtow_kg, RHO_SL, &wing, &state,
                                              perf_cfg.mu_roll_paved, cl_ground_roll, &engine,
                                              0.0, &perf_cfg);
        println!("S_ground={s_ground:.0}m  S_50ft={s_50ft:.0}m");
        assert!(s_50ft > s_ground,
            "distância sobre 15m ({s_50ft:.0}m) deveria ser MAIOR que a rolagem de solo pura \
             ({s_ground:.0}m) — inclui rotação + subida");
    }

    /// Ciclo 7 (task 1) → Ciclo 8 (task 1): a docstring original PREVIU a
    /// própria morte — "um futuro modelo de arrasto de flap (que penalizasse
    /// frações altas) pode quebrar esta direção LEGITIMAMENTE; se isso
    /// acontecer, revisar o teste, não achar que é regressão". Este ciclo
    /// implementou esse modelo (`cd0_flap_to_extra`), então o teste foi
    /// reescrito para medir o TRADE-OFF completo em vez de assumir uma lei
    /// monotônica de um lado só. Dois efeitos competem quando
    /// `to_flap_fraction` sobe:
    ///   - CL: `cl_max_to` sobe ⟹ `V_s_to` cai ⟹ rolagem de solo mais curta
    ///     (`S_G ∝ 1/CL_TO`, `takeoff_ground_roll_m` — que NÃO recebe o
    ///     delta de arrasto, ver sua docstring);
    ///   - CD0: `cd0_flap_to_extra = to_flap_fraction·cd0_flap_delta` sobe
    ///     ⟹ menos excesso de potência no segmento de SUBIDA
    ///     (`excess_power_kw` dentro de `takeoff_distance_50ft_m`) ⟹
    ///     gradiente pior ⟹ S_subida mais longa.
    /// Com o delta REAL da fixture (`cd0_flap_delta=0.020`, ver
    /// `aircraft_config::test_fixtures::config_teste`), o benefício de CL na
    /// rolagem de solo (dominante na distância total) supera o custo de
    /// arrasto no segmento de subida (que é uma fração menor da distância
    /// total) — a direção líquida MEDIDA continua sendo "mais flap encurta",
    /// mas por MENOS margem que antes (ver valores no `println!` abaixo). O
    /// pin é do RESULTADO observado, não de uma lei imposta a priori: se um
    /// ajuste de modelo futuro (ex.: `cd0_flap_delta` maior, ou um perfil com
    /// mais penalidade de arrasto) inverter a direção, revisar o teste, não
    /// achar que é regressão — mesmo espírito da docstring original.
    #[test]
    fn mais_flap_de_decolagem_trade_off_liquido_na_decolagem_sobre_15m() {
        let req = crate::models::requirements::test_fixtures::requisitos_teste();
        let engine = engine_teste();

        // Duas configurações idênticas EXCETO pela fração de flap de
        // decolagem — a asa é recomputada em cada uma (é o
        // `AerodynamicsAgent` que deriva `cl_max_to` E `cd0_flap_to_extra`).
        let s_50ft_para = |fracao: f64| {
            let mut cfg = config_teste();
            cfg.stability.to_flap_fraction = fracao;
            let state = AircraftState::from_config(&cfg);
            let wing = AerodynamicsAgent::run(&state, &req);
            let perf_cfg = cfg.performance.clone();
            let cl_ground_roll = cfg.stability.cl_ground_rotation;
            (
                wing.cl_max_to,
                wing.cd0_flap_to_extra,
                takeoff_distance_50ft_m(state.mtow_kg, RHO_SL, &wing, &state,
                                        perf_cfg.mu_roll_paved, cl_ground_roll, &engine,
                                        0.0, &perf_cfg),
            )
        };

        let (cl_03, cd0_03, s_03) = s_50ft_para(0.3);
        let (cl_07, cd0_07, s_07) = s_50ft_para(0.7);
        println!("fração 0.3: cl_max_to={cl_03:.4} cd0_flap_to_extra={cd0_03:.5} S_50ft={s_03:.3}m  |  \
                  fração 0.7: cl_max_to={cl_07:.4} cd0_flap_to_extra={cd0_07:.5} S_50ft={s_07:.3}m");
        assert!(cl_07 > cl_03, "fração maior deveria dar cl_max_to maior");
        assert!(cd0_07 > cd0_03, "fração maior deveria dar cd0_flap_to_extra maior (mais custo de arrasto)");
        // RESULTADO medido (ciclo 8, task 1): com o delta real da fixture
        // (0.020), o benefício de CL na rolagem de solo ainda supera o custo
        // de arrasto na subida — a decolagem continua encurtando com mais
        // flap, mas a margem relativa CAIU em relação à direção "livre" do
        // ciclo 7 (era ~14% de diferença sem custo de arrasto).
        assert!(s_07 < s_03,
            "trade-off líquido medido: mais flap de decolagem (fração 0.7, cl_max_to={cl_07:.4}, \
             cd0_flap_to_extra={cd0_07:.5}) deveria encurtar a decolagem sobre 15 m em relação a \
             0.3 (cl_max_to={cl_03:.4}, cd0_flap_to_extra={cd0_03:.5}) com o delta desta fixture: \
             S(0.7)={s_07:.3}m, S(0.3)={s_03:.3}m — se isso não vale mais, é um ACHADO de modelo, \
             não uma regressão de código: revisar a direção pinada, não forçá-la de volta.");
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
        let cl_ground_roll = cl_ground_rotation_teste();
        state.prop_diameter_m = 1.9;
        let s_1_9 = takeoff_distance_50ft_m(state.mtow_kg, RHO_SL, &wing, &state,
                                             perf_cfg.mu_roll_paved, cl_ground_roll, &engine,
                                             0.0, &perf_cfg);
        state.prop_diameter_m = 1.6;
        let s_1_6 = takeoff_distance_50ft_m(state.mtow_kg, RHO_SL, &wing, &state,
                                             perf_cfg.mu_roll_paved, cl_ground_roll, &engine,
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
        let cl_ground_roll = cl_ground_rotation_teste();
        let d = takeoff_distance_m(state.mtow_kg, RHO_SL, &wing, &state, perf_cfg.mu_roll_paved,
                                    cl_ground_roll, &engine, 0.0, perf_cfg.static_thrust_factor);
        println!("Decolagem pista pavimentada: {d:.0} m");
        // Task 4.7: T_avg passou a usar tração ESTÁTICA corrigida (Rankine-
        // Froude × static_thrust_factor da fixture, 0.72) em vez da antiga
        // estimativa ad hoc ("80% da tração a V_lo/2") — a distância cresceu
        // (T_avg menor). Valor observado empiricamente para a fixture
        // sintética pós-Task-4.7: ver task-4.7-report.md para a tabela
        // antes/depois. Janela alargada para cobrir o novo patamar.
        //
        // Ciclo 12 (task 2, `old→new`): a rolagem de solo passa de método
        // energético fechado (sem arrasto/atrito) para integração numérica
        // com arrasto e atrito explícitos — mais caro por construção (spec
        // §0). Valor medido na fixture sintética: 773 m (era < 700 m).
        // Janela alargada de (200, 700) para (200, 900) para cobrir o novo
        // patamar — mesmo padrão da atualização anterior, não é um pin
        // apertado contra o baseline real (esse vive em
        // `tests/generic_engine.rs`, tolerância 1%).
        assert!(d > 200.0 && d < 900.0,
            "Distância TO {d:.0} m fora do esperado (200–900 m)");
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
        let perf = PerformanceAgent::run(&state, &wing, &prop, state.mtow_kg, &engine, &req, &perf_cfg,
                                          cl_ground_rotation_teste());
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
        let perf = PerformanceAgent::run(&state, &wing, &prop, state.mtow_kg, &engine, &req, &perf_cfg,
                                          cl_ground_rotation_teste());
        println!("climb_gradient_pct = {:.2}%  Vx={:.1}km/h  Vy={:.1}km/h",
                 perf.climb_gradient_pct, perf.vx_kmh, perf.vy_kmh);
        assert!(perf.climb_gradient_pct > 0.0 && perf.climb_gradient_pct < 100.0,
            "gradiente {:.2}% fora da faixa físicamente plausível (0, 100)%",
            perf.climb_gradient_pct);
    }
}
