# Ciclo 12 — Solo Honesto — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Trocar a rolagem de decolagem e de pouso do método energético fechado (sem arrasto, sem atrito) por integração numérica da equação de movimento consumindo a polar completa; implementar os termos de solo do balanço de rotação; schema 5.5.

**Architecture:** Novo modelo de tração válido de 0 a V_LOF (Rankine-Froude com avanço, `thrust_ground_roll_n`), integração de Simpson sobre V nas duas rolagens, `surface_factor` substituído por `mu_roll` explícito no caminho da decolagem, e os dois termos nariz-abaixo de solo somados ao momento disponível de rotação.

**Tech Stack:** Rust, serde/TOML.

**Spec:** `docs/superpowers/specs/2026-08-15-ciclo12-solo-honesto-design.md` — **leia a spec inteira antes da sua task.** O plano argumenta a partir dela; as derivações, as premissas declaradas e as direções de erro estão lá.

## Global Constraints

- **TDD RED-first.** Todo comentário e docstring em Português.
- **Tolerâncias INALTERADAS — nunca alargar um assert para acomodar um número novo.** Pins são atualizados com o valor medido e comentário `old→new`, data "Campanha ciclo 12".
- **Este ciclo PIORA números por construção.** O veredito esperado é `validation_status: FAIL` com os checks #23 e #24 violados. Isso é o resultado, não um problema a contornar. **Nunca ajustar config, μ, tolerância ou gate para salvar o PASS.** Se um número surpreender por >5% contra o congelado, PARE e reporte — não investigue sozinho por mais de uma rodada.
- **Nenhuma docstring obsoleta é deletada.** Toda afirmação que este ciclo torna falsa é REESCRITA `old→new` dizendo o que era, o que passou a ser e por quê.
- `cargo test --release` verde ao fim de cada task. Genericidade verde (nenhum `toyota|1gd|rotax|915is` em `src/`).
- **`scripts/verifica-ciclo.sh` colado no report — sem a saída dele o report não é aceito.**
- Regen do JSON: `cargo run --release -- --engine config/engines/toyota_1gd_ftv.toml --aircraft config/aircraft/baseline_4seat.toml --mission config/missions/default.toml --out aircraft_spec.json`
- Baseline de partida: `deb8f82` (spec commitada), schema 5.4, 469 testes, veredito PASS.

## Contrato de report (v2 da equipe)

Reporte **o que MEDIU, o que MUDOU, e "anomalias observadas (sem interpretar)"**. Interpretar anomalia é trabalho do chefe. Se um número não bate com o congelado, escreva o número e a discrepância — não invente a causa. **Nunca cite uma norma, um parágrafo de regulamento ou um número que você não leu neste repositório.**

---

### Task 1: Schema 5.5 — infinito explícito nas três distâncias legado

**Tier: mecânica.** Sem física, sem mudança de número. A aceitação é inteiramente provável por `verifica-ciclo.sh`.

**Files:**
- Modify: `src/models/specs.rs` — `PerformanceSpec`, campos `to_distance_paved_m`, `to_distance_grass_m`, `landing_distance_m`; `SCHEMA_VERSION`; docstring do módulo `fatigue_life_serde`.
- Modify: pins de `schema_version` `"5.4"` → `"5.5"` onde existirem (procure com `grep -rn '5\.4' tests/ src/`).

**Interfaces:**
- Consumes: `mod fatigue_life_serde` (já existe em `src/models/specs.rs`, serializa `f64::INFINITY` como a string `"infinita"`).
- Produces: nada consumido por tasks posteriores além de `SCHEMA_VERSION = "5.5"`.

- [ ] **Step 1: Escrever o teste RED**

Em `src/models/specs.rs`, no módulo de testes, junto do
`performance_spec_roundtrip_serde_com_infinito` que já existe:

```rust
/// Ciclo 12: as três distâncias LEGADO (`to_distance_paved_m`,
/// `to_distance_grass_m`, `landing_distance_m`) passam a poder valer
/// `+INFINITY` — a rolagem integrada devolve infinito quando a tração
/// não basta para acelerar. Sem `fatigue_life_serde` elas virariam
/// `null` no JSON (RFC 8259 não representa infinito), quebrando
/// round-trip. Mesmo defeito que o ciclo 11 corrigiu em `to_50ft_*`.
#[test]
fn performance_spec_roundtrip_serde_com_infinito_nas_distancias_legado() {
    let mut p = performance_spec_fixture();
    p.to_distance_paved_m = f64::INFINITY;
    p.to_distance_grass_m = f64::INFINITY;
    p.landing_distance_m = f64::INFINITY;

    let json = serde_json::to_string(&p).expect("serializa");
    assert!(json.contains("\"to_distance_paved_m\":\"infinita\""), "{json}");
    assert!(json.contains("\"to_distance_grass_m\":\"infinita\""), "{json}");
    assert!(json.contains("\"landing_distance_m\":\"infinita\""), "{json}");
    assert!(!json.contains("null"), "nenhum campo pode virar null: {json}");

    let volta: PerformanceSpec = serde_json::from_str(&json).expect("desserializa");
    assert!(volta.to_distance_paved_m.is_infinite());
    assert!(volta.to_distance_grass_m.is_infinite());
    assert!(volta.landing_distance_m.is_infinite());
}
```

Use a MESMA fixture que o teste do ciclo 11 usa (leia-o e reaproveite;
se ele constrói o `PerformanceSpec` inline, construa igual — não crie
uma fixture nova só para isto).

- [ ] **Step 2: Rodar e confirmar que falha**

`cargo test --release performance_spec_roundtrip_serde_com_infinito_nas_distancias_legado`
Esperado: FAIL — o JSON traz `null` nos três campos.

- [ ] **Step 3: Implementar**

Adicionar `#[serde(with = "fatigue_life_serde")]` nos três campos.
Atualizar a docstring do módulo `fatigue_life_serde` para listar os
novos usuários (hoje ela nomeia `fatigue_life_cycles` e os `to_50ft_*`).
Bump `SCHEMA_VERSION` `"5.4"` → `"5.5"` com a entrada de histórico no
padrão das anteriores: MINOR com exceção registrada — o tipo de três
campos passa a admitir string, mesmo padrão de 5.2/5.3/5.4.

- [ ] **Step 4: Rodar a suíte inteira e atualizar os pins de versão**

`cargo test --release` — corrija os pins `"5.4"` → `"5.5"`.
**Rode a suíte INTEIRA, não só os alvos que você acha afetados.**

- [ ] **Step 5: Regen do JSON e verificação**

Regen; `git diff aircraft_spec.json` deve mostrar **apenas** a linha
`schema_version`. Qualquer outra linha mudando é anomalia — reporte.

- [ ] **Step 6: Rodar `scripts/verifica-ciclo.sh` e commitar**

```bash
scripts/verifica-ciclo.sh
git add -A
git commit -m "feat(schema): v5.5 — +INF explícito em to_distance_*/landing_distance_m"
```

---

### Task 2: Tração de rolagem + rolagem de decolagem por integração

**Tier: julgamento (física + cascata).**

**Files:**
- Modify: `src/agents/performance.rs` — nova `thrust_ground_roll_n`, nova `cd_ground_roll`, `takeoff_ground_roll_m` reescrita, `takeoff_distance_m` e `takeoff_distance_50ft_m` (assinaturas), orquestrador em `PerformanceAgent::run`.
- Modify: `src/models/aircraft_config.rs` — `PerformanceCfg::mu_roll_paved` / `mu_roll_grass`.
- Modify: `src/models/config.rs` — validação de faixa dos dois campos novos.
- Modify: `config/aircraft/baseline_4seat.toml` — bloco `[performance]`.
- Modify: pins em cascata (`tests/generic_engine.rs`, `tests/cli.rs`, e o que a suíte apontar).

**Interfaces:**

- Consumes: `agents::performance::static_thrust_ideal_n` (para o teste de identidade em V=0), `AircraftState::cd0_gear_fixed_increment`, `WingSpec::{cd0, cd0_flap_to_extra, aspect_ratio, oswald_efficiency, area_m2, cl_max_to}`.
- Produces — assinaturas exatas que as tasks 3 e 4 consomem:

```rust
/// Tração VÁLIDA na faixa da rolagem (0 → V_LOF). Rankine-Froude COM
/// velocidade de avanço. Ver spec §2.
pub fn thrust_ground_roll_n(
    v_ms: f64,
    engine: &EngineSpec,
    engine_rpm: f64,
    prop_diam_m: f64,
    altitude_m: f64,
    isa_delta_c: f64,
    static_thrust_factor: f64,
    psru_efficiency: f64,
) -> f64;

/// CD da aeronave em atitude de solo, com trem ESTENDIDO. Fonte única
/// de verdade — tasks 3 e 4 consomem esta MESMA função.
pub fn cd_ground_roll(
    wing: &WingSpec,
    state: &AircraftState,
    cl_ground_roll: f64,
    cd0_flap_extra: f64,
) -> f64;

pub fn takeoff_distance_m(
    mass_kg: f64, rho: f64, wing: &WingSpec, state: &AircraftState,
    mu_roll: f64, cl_ground_roll: f64,
    engine: &EngineSpec, isa_delta_c: f64, static_thrust_factor: f64,
) -> f64;

pub fn takeoff_distance_50ft_m(
    mass_kg: f64, rho: f64, wing: &WingSpec, state: &AircraftState,
    mu_roll: f64, cl_ground_roll: f64,
    engine: &EngineSpec, isa_delta_c: f64, perf_cfg: &PerformanceCfg,
) -> f64;
```

`thrust_ground_roll_n` **não recebe `psru_ratio`** — teoria de quantidade de movimento não usa rpm de hélice. Se você sentir falta dele, releia a spec §2.

- [ ] **Step 1: Escrever os testes RED do modelo de tração**

```rust
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
```

- [ ] **Step 2: Rodar e confirmar que falham** (função não existe).

- [ ] **Step 3: Implementar `thrust_ground_roll_n`**

`K = P_eixo / (2ρA)` com `P_eixo = shaft_power_kw(...) * 1000.0`,
`A = π·(D/2)²`, `ρ = Isa::density_kgm3(altitude_m, isa_delta_c)`.
Resolver `u³ − V·u² − K = 0` para a raiz `u > V` por Newton a partir de
`u₀ = V + K.cbrt()`, iterando até `|Δu| < 1e-12·u` ou 100 iterações.
`T = static_thrust_factor · 2ρA·u·(u − V)`.
Docstring com a derivação da spec §2 e a razão de a função existir
(§1: `η(J)` extrapolada devolve ≈80.000 N em V=1 m/s).

- [ ] **Step 4: Rodar; os dois testes passam.**

- [ ] **Step 5: Escrever os testes RED do integrador**

```rust
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

/// Spec §7.3 — monotonicidades ESTRITAS. Cada uma é falseável: se o
/// sinal de um termo estiver trocado, uma delas quebra.
#[test]
fn rolagem_de_decolagem_responde_no_sentido_certo_a_cada_termo() {
    let (engine, state, wing) = fixture_baseline();
    let roll = |mu: f64, mass: f64| takeoff_ground_roll_m(
        mass, RHO_SL, &wing, &state, &engine, 0.0, mu, 0.5, 0.75);
    assert!(roll(0.08, MTOW_PIN_KG) > roll(0.04, MTOW_PIN_KG), "atrito maior ⟹ rolagem maior");
    assert!(roll(0.04, MTOW_PIN_KG) > roll(0.04, MTOW_PIN_KG * 0.8), "peso maior ⟹ rolagem maior");
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
```

```rust
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
```

**Símbolos de teste que ainda não existem.** `fixture_baseline()`,
`MTOW_PIN_KG` e `RHO_SL` não existem no repositório — são a forma que
estes testes precisam. Defina-os localmente no módulo de teste (o padrão
mais próximo é `setup()` em `performance.rs`, que usa a config sintética
`config_teste()`; serve, porque nenhum destes testes precisa bater com os
números congelados — todos são propriedades relacionais). **Não crie
fixture nova em arquivo separado nem tente carregar o TOML real** só para
isto. Mesma orientação vale para as Tasks 3 e 4.

Os nomes `integra_rolagem_decolagem` / `takeoff_ground_roll_com_passos`
são a forma que o teste PRECISA — extraia o integrador e o parâmetro de
passos de modo que estes testes possam chamá-los (visibilidade
`pub(crate)` ou `#[cfg(test)]`, decisão sua, documentada). É o mesmo
padrão que o ciclo 11 usou em `climb_rate_search_window_kmh`.

- [ ] **Step 6: Rodar; confirmar RED.**

- [ ] **Step 7: Implementar `cd_ground_roll` e a rolagem integrada**

```
cd_ground_roll = wing.cd0
               + state.cd0_gear_fixed_increment
               + cd0_flap_extra
               + cl_ground_roll² / (π · wing.aspect_ratio · wing.oswald_efficiency)
```

Integração: `F(V) = T(V) − D(V) − mu_roll·max(0, W − L(V))`,
`D = q·S·CD_roll`, `L = q·S·CL_roll`, `q = 0.5·ρ·V²`.
`S = ∫₀^{V_LOF} m·V/F(V) dV` por Simpson composto, 200 intervalos,
`V_LOF = 1.10·√(2W/(ρ·S_w·cl_max_to))`.
Se `F(V) ≤ 0` em qualquer nó, devolver `f64::INFINITY`.

Reescrever a docstring de `takeoff_ground_roll_m` `old→new`: hoje ela
afirma "a fórmula é o método ENERGÉTICO de Raymer ... que, por
construção, não tem um termo de arrasto explícito ... Não há onde
inserir `cd0_flap_delta` sem reescrever o método inteiro — fora de
escopo desta task". **É exatamente esse método inteiro que esta task
reescreve.** Diga isso.

- [ ] **Step 8: Trocar `surface_factor` por `mu_roll`**

Config nova em `[performance]` com os comentários de origem
(Raymer Tab. 17.1): `mu_roll_paved = 0.04`, `mu_roll_grass = 0.08`.
Validação de faixa em `models::config` no padrão dos demais escalares
(rejeitar fora de `(0.0, 0.5)`), com teste de rejeição.

Nas assinaturas públicas, `surface_factor: f64` → `mu_roll: f64` mais o
novo `cl_ground_roll: f64`. Chamadores em `PerformanceAgent::run`
passam `perf_cfg.mu_roll_paved` / `mu_roll_grass` e
`cfg.stability.cl_ground_rotation`.

**Não deixe o `surface_factor` sobreviver em lugar nenhum do caminho de
decolagem** — multiplicar por 1,20 além do `μ_roll` conta a grama duas
vezes (spec §4). Reescreva `old→new` a docstring que documenta os
fatores de superfície 1,00/1,15–1,20/1,25.

- [ ] **Step 9: Rodar a suíte INTEIRA, medir, atualizar pins**

Números congelados desta task (divergência >5% ⟹ PARE e reporte). Vêm de
uma implementação de referência independente com as constantes MEDIDAS do
pipeline — ver spec §9, que lista as constantes e a armadilha da massa.

| Grandeza | Hoje | Congelado |
|---|---|---|
| rolagem TO pavimentada | 265,485094 | **496,4 m** |
| rolagem TO grama | 318,582113 | **664,2 m** |
| `to_50ft_paved_m` | 420,372451 | **651,3 m** |
| `to_50ft_grass_m` | 473,469470 | **819,1 m** |
| `to_distance_paved_m` | 398,227641 | **744,6 m** |
| `to_distance_grass_m` | 477,873169 | **996,3 m** |

**Armadilha da massa (spec §9.1).** A massa que `PerformanceAgent::run` usa
é `state.mtow_kg` = **1.537,389006 kg**, NÃO os 1.557,519935 kg do bloco
`weight` do JSON (esse é o MTOW do cenário ESTRUTURAL). O chefe E o revisor
de plano erraram isto independentemente nos respectivos hand-checks. Se
você conferir qualquer número acima à mão, use 1.537,389006 kg.

Referência auxiliar para depurar: `T_static = 3.740,0919357761986 N`,
`P_eixo = 144,241 kW`, `V_LOF = 35,361 m/s`. Se a sua `thrust_ground_roll_n`
não devolver exatamente `3.740,0919...` em V=0, pare aí — o resto não
importa até isso bater.

- [ ] **Step 10: Regen, `verifica-ciclo.sh`, commit**

`to_50ft_grass_m` deve passar de 600 m e o check #23 deve REPROVAR.
**`validation_status` vira FAIL — é o resultado esperado.** Registre a
violação exata no report.

```bash
git commit -m "feat(performance): rolagem de decolagem por integração com arrasto e atrito (backlog item 4)"
```

---

### Task 3: Rolagem de pouso por integração

**Tier: julgamento (física + cascata).**

**Files:**
- Modify: `src/models/specs.rs` — `WingSpec::cd0_flap_ldg_extra` e sua construção.
- Modify: `src/agents/performance.rs` — `cl_ground_roll_landing`, `landing_ground_roll_m`, `landing_distance_m`, `landing_distance_50ft_m`, orquestrador.
- Modify: `src/models/aircraft_config.rs` — docstring de `WingCfg::cd0_flap_delta` (reescrita `old→new`).
- Modify: pins em cascata.

**Interfaces:**
- Consumes da Task 2: `cd_ground_roll(wing, state, cl_ground_roll, cd0_flap_extra)`. **Use esta função — não reimplemente a soma do CD.**
- Produces:

```rust
/// CL em atitude de solo com flap de POUSO (cheio). Derivado, não
/// parâmetro livre — ver spec §5.2 e a premissa declarada lá.
pub fn cl_ground_roll_landing(
    cl_ground_roll_to: f64, to_flap_fraction: f64, wing: &WingSpec,
) -> f64;

pub fn landing_distance_m(
    mass_kg: f64, rho: f64, wing: &WingSpec, state: &AircraftState,
    mu_brake: f64, cl_ground_roll_ldg: f64,
) -> f64;

pub fn landing_distance_50ft_m(
    mass_kg: f64, rho: f64, wing: &WingSpec, state: &AircraftState,
    mu_brake: f64, cl_ground_roll_ldg: f64, perf_cfg: &PerformanceCfg,
) -> f64;
```

- [ ] **Step 1: Escrever os testes RED**

```rust
/// Spec §5.2: CL_roll_ldg = cl_ground_rotation + (1 − to_flap_fraction)
///                          · (cl_max_flaps − cl_max_clean)
/// Hand-check com literais do baseline: 0,50 + 0,65·(2,10 − 1,45).
#[test]
fn cl_de_rolagem_de_pouso_hand_check_do_baseline() {
    let (_engine, _state, wing) = fixture_baseline();
    let cl = cl_ground_roll_landing(0.50, 0.35, &wing);
    assert!((cl - 0.9225).abs() < 1e-9, "cl={cl}");
    assert!(cl > 0.50, "flap cheio tem de dar MAIS CL que o parcial de decolagem");
}

/// Spec §5.1 — o achado central desta task: a sustentação residual
/// ALIVIA o peso sobre as rodas e PIORA a frenagem. Falseável: se o
/// sinal de L no termo de atrito estiver trocado, isto quebra.
#[test]
fn sustentacao_residual_alonga_a_rolagem_de_pouso() {
    let (_engine, state, wing) = fixture_baseline();
    let com_cl = landing_ground_roll_m(M_LDG_PIN_KG, RHO_SL, &wing, &state, 0.40, 0.9225);
    let sem_cl = landing_ground_roll_m(M_LDG_PIN_KG, RHO_SL, &wing, &state, 0.40, 0.0);
    assert!(com_cl > sem_cl,
            "CL de solo maior ⟹ menos peso nas rodas ⟹ rolagem MAIOR: {com_cl} vs {sem_cl}");
}

/// Convergência (spec §7.2), mesma exigência da Task 2.
#[test]
fn integrador_de_pouso_esta_convergido_na_resolucao_escolhida() {
    let (_engine, state, wing) = fixture_baseline();
    let s200 = landing_ground_roll_com_passos(M_LDG_PIN_KG, RHO_SL, &wing, &state, 0.40, 0.9225, 200);
    let s400 = landing_ground_roll_com_passos(M_LDG_PIN_KG, RHO_SL, &wing, &state, 0.40, 0.9225, 400);
    assert!((s400 - s200).abs() / s200 < 1e-3, "não convergido: {s200} vs {s400}");
}

/// Freio melhor ⟹ rolagem menor. Estrita.
#[test]
fn frenagem_melhor_encurta_a_rolagem_de_pouso() {
    let (_engine, state, wing) = fixture_baseline();
    let pav = landing_ground_roll_m(M_LDG_PIN_KG, RHO_SL, &wing, &state, 0.40, 0.9225);
    let grama = landing_ground_roll_m(M_LDG_PIN_KG, RHO_SL, &wing, &state, 0.30, 0.9225);
    assert!(grama > pav, "grama (μ menor) tem de dar rolagem maior: {grama} vs {pav}");
}

/// Espelho do teste de +INFINITY da Task 2, spec §7.6: se a
/// desaceleração puder chegar a zero (sustentação alivia TODO o peso e
/// não há arrasto), a rolagem não converge. Aqui o caminho é diferente
/// do da decolagem — no pouso F é sempre ≥ D > 0 para CD > 0, então o
/// cenário adversarial precisa de CD nulo. Se a sua implementação NÃO
/// puder atingir esse estado, documente por quê no report em vez de
/// forçar o teste; é um resultado válido e diferente do da decolagem.
#[test]
fn desaceleracao_nula_devolve_infinito_e_nao_numero_espurio() {
    let (_engine, state, mut wing) = fixture_baseline();
    wing.cd0 = 0.0;
    wing.cd0_flap_ldg_extra = 0.0;
    let s = landing_ground_roll_m(M_LDG_PIN_KG, RHO_SL, &wing, &state, 0.0, 0.0);
    assert!(s.is_infinite() || s.is_nan() == false,
            "sem arrasto e sem freio a rolagem não pode ser um número finito: {s}");
}
```

**Símbolos inexistentes:** `fixture_baseline()`, `M_LDG_PIN_KG`, `RHO_SL`
— mesma orientação da Task 2 (defina localmente, padrão `setup()` de
`performance.rs`).

- [ ] **Step 2: Rodar; confirmar RED.**

- [ ] **Step 3: Adicionar `WingSpec::cd0_flap_ldg_extra`**

Valor: `cfg.wing.cd0_flap_delta` CHEIO (0,015), construído no mesmo
ponto em que `cd0_flap_to_extra` já é construído (que é o mesmo delta
multiplicado por `to_flap_fraction`).

Reescrever `old→new` a docstring de `WingCfg::cd0_flap_delta`, que hoje
afirma que **não há call site de pouso que consuma o delta CHEIO** —
conclusão da auditoria do ciclo 8, task 1. **Esta task é o primeiro
consumidor.** Diga o que a auditoria concluiu, por que valia então, e o
que mudou.

- [ ] **Step 4: Implementar a rolagem de pouso integrada**

`F(V) = D(V) + mu_brake·max(0, W_ldg − L(V))`, desaceleração;
`S = ∫₀^{V_ref} m·V/F(V) dV`, Simpson 200 intervalos,
`V_ref = 1.30·√(2·W_ldg/(ρ·S_w·wing.cl_max))` (inalterado).
`CD_roll_ldg = cd_ground_roll(wing, state, cl_ground_roll_ldg, wing.cd0_flap_ldg_extra)`.

`landing_ground_roll_m`, `landing_distance_m` e `landing_distance_50ft_m`
passam a receber `state: &AircraftState` (spec §5.3b) — atualize os
chamadores no orquestrador.

Reescrever `old→new` a docstring de `landing_distance_50ft_m`, que hoje
enumera os três segmentos e conclui que **nenhum consome a polar de
arrasto**. O segmento de solo passa a consumir.

- [ ] **Step 5: Rodar a suíte INTEIRA, medir, atualizar pins**

| Grandeza | Hoje | Congelado |
|---|---|---|
| rolagem pouso pavimentada | 162,66 | **242,5 m** |
| rolagem pouso grama | 216,88 | **306,6 m** |
| `ldg_50ft_m` | 502,458299 | **582,3 m** |
| `ldg_50ft_grass_m` | 556,677173 | **646,4 m** |
| `landing_distance_m` | 362,656622 | **442,5 m** |

Massa de pouso do pipeline: **1.406,349006 kg** (`state.mtow_kg` − 60% do
combustível, NÃO o MTOW estrutural do JSON — spec §9.1). `V_ref = 35,723 m/s`,
`CL_roll_ldg = 0,9225`, `CD_roll_ldg = 0,082213`.

- [ ] **Step 6: Regen, `verifica-ciclo.sh`, commit**

Espere o check #24 REPROVAR também. Registre a violação exata.

```bash
git commit -m "feat(performance): rolagem de pouso por integração com arrasto e alívio de sustentação (backlog item 4)"
```

---

### Task 4: Termos de solo do balanço de rotação

**Tier: julgamento (física + envelope de CG).**

**Files:**
- Modify: `src/agents/trim_authority.rs` — `rotation_available_moment_nm`, `rotation_fwd_limit_m`, `TrimAuthorityAgent::run`, docstrings.
- Modify: `src/models/aircraft_config.rs` + `src/models/specs.rs` — `z_drag_above_cg_m`.
- Modify: `src/models/config.rs` — validação de faixa.
- Modify: `config/aircraft/baseline_4seat.toml` — bloco `[wing]`.
- Modify: `tests/generic_engine.rs:2000` — ÚNICO call site de
  `rotation_fwd_limit_m`/`rotation_available_moment_nm` fora de
  `trim_authority.rs` (localizado pela revisão de plano).
- Modify: pins de limite de rotação e envelope.

**Interfaces:**
- Consumes da Task 2: `cd_ground_roll`. Da Task 3: nada.
- Produces: nada consumido por tasks posteriores.

- [ ] **Step 1: Escrever os testes RED**

```rust
/// Spec §6.3 — passar μ=0 e CD=0 reproduz EXATAMENTE o modelo
/// pré-ciclo-12. Mesmo padrão do ciclo 10 (`thrust_rot_n = 0` reproduzia
/// o modelo pré-ciclo-10). É a prova de que os termos novos são
/// ADITIVOS e não mexeram no que já estava certo.
#[test]
fn termos_de_solo_nulos_reproduzem_o_modelo_pre_ciclo_12() {
    let m_novo = rotation_available_moment_nm(/* ..., */ 0.0 /*mu_roll*/, H_CG, 0.0 /*cd_roll*/, 0.0);
    let m_antigo = momento_disponivel_pre_ciclo12(/* mesmos argumentos */);
    assert!((m_novo - m_antigo).abs() < 1e-9, "{m_novo} vs {m_antigo}");
}

/// Spec §6.3 — os dois termos são nariz-ABAIXO: subtraem do momento
/// disponível, logo o limite dianteiro RECUA (percentual de MAC MAIOR).
/// Falseável em cada variável separadamente.
#[test]
fn termos_de_solo_recuam_o_limite_dianteiro_de_rotacao() {
    let base = limite_rot(0.00, H_CG, 0.0);
    assert!(limite_rot(0.04, H_CG, 0.0) > base, "atrito maior ⟹ limite recua");
    assert!(limite_rot(0.04, H_CG * 1.5, 0.0) > limite_rot(0.04, H_CG, 0.0),
            "CG mais alto ⟹ braço maior ⟹ limite recua");
    assert!(limite_rot(0.04, H_CG, 0.10) < limite_rot(0.04, H_CG, 0.0),
            "centro de arrasto mais alto ⟹ braço LÍQUIDO menor ⟹ limite AVANÇA");
}
```

**Símbolos inexistentes:** `momento_disponivel_pre_ciclo12`, `H_CG`,
`limite_rot` — defina-os localmente no módulo de teste.
`momento_disponivel_pre_ciclo12` é uma cópia local da fórmula ANTIGA
(sem os dois termos novos), escrita à mão no teste; é isso que dá valor
à prova de que os termos são aditivos. `limite_rot` é um fechamento
sobre `rotation_fwd_limit_m` com os demais argumentos fixos.

- [ ] **Step 2: Rodar; confirmar RED.**

- [ ] **Step 3: Implementar**

Somar ao momento disponível:
```
− mu_roll · max(0, W − L_g) · h_cg  −  D · (h_cg − z_drag_above_cg_m)
```
com `L_g = q_r·S_w·cl_ground_rotation` (já calculado na função) e
`D = q_r·S_w·cd_ground_roll(wing, state, cl_ground_rotation, wing.cd0_flap_to_extra)`
— **flap de DECOLAGEM, não de pouso** (é uma rotação de decolagem).

Campo novo `z_drag_above_cg_m` em `[wing]`, default `0.0`, faixa
válida `[0.0, 0.30]`, com o comentário de origem da spec §6.2 (banda
plausível 0–0,10 m; `h_D = 0` é o caso CONSERVADOR).

- [ ] **Step 4: Reescrever `old→new` a estimativa de "≲2 pp"**

A docstring de `rotation_available_moment_nm` afirma hoje que os dois
termos valem "**≲2 pp de MAC** no limite dianteiro". **Meça e escreva o
valor medido.** O hand-check do chefe dá ≈4,5 pp e os dois termos juntos
(≈838 N·m) SUPERAM o termo de tração que já estava no balanço (≈665 N·m).
Explique por que a estimativa do ciclo 10 ficou baixa (foi feita sem
`CD_roll` explícito e sem campo de `μ_roll` — nenhum dos dois existia)
sem transformar isso em desculpa: o texto afirmava um número que a
medição desmente.

- [ ] **Step 5: Rodar a suíte INTEIRA, medir, atualizar pins**

Congelado: limite dianteiro de rotação **13,354637% MAC → ≈17,76% MAC**
(+4,40 pp). Referência para depurar (spec §6.1, medido):
`V_r = 35,361 m/s`, `q_r = 765,87 Pa`, `L_g = 5.437,7 N`, `N = 9.639,5 N`,
`D = 513,8 N`, `M_solo = 827,4 N·m`, `W = 15.077,1 N`, `MAC = 1,24632 m`.

**Não use 8,908% como "hoje"** — esse é o número do ciclo 7, três ciclos
desatualizado, e estava errado na primeira versão da spec.

**`inside_envelope` pode virar `false` em algum cenário de carga.** Se
virar, é achado honesto: registre qual cenário, com que margem, e
NÃO ajuste config. Se um cenário sair do envelope, isso é uma violação
nova a somar às da pista.

- [ ] **Step 6: Regen, `verifica-ciclo.sh`, commit**

```bash
git commit -m "feat(trim): termos de solo (atrito e arrasto) no balanço de rotação"
```

---

### Task 5: Documentação, backlog e report do ciclo

**Tier: julgamento (prosa que precisa ser verdadeira).**

**Files:**
- Modify: `docs/aircraft_spec.schema.md` — §5 e histórico 5.5; linhas de `to_distance_*`, `to_50ft_*`, `ldg_50ft_*`, `landing_distance_m`, limite de rotação.
- Modify: `docs/backlog.md` — item 4 → RESOLVIDO; itens NOVOS.
- Modify: `src/main.rs` — blocos `fidelity.performance` e `fidelity.propeller` (vão para dentro do JSON de produção).
- Create: report da task.

- [ ] **Step 1: Fechar o item 4 do backlog**

`## 4.` → `RESOLVIDO ciclo 12`, com a tabela `old→new` REAL (medida,
não a congelada do plano) das 11 grandezas, e o método novo descrito.

- [ ] **Step 2: Abrir os itens novos do backlog**

Um item por linha da spec §11, com os números MEDIDOS:
1. Unificar o modelo de tração — descontinuidade medida em `V_LOF`
   entre `thrust_ground_roll_n` e `thrust_available_n`. **Meça e
   escreva o número; não copie o ≈28% estimado da spec.**
2. `prop_efficiency` com `η(0) = 0,58` — polinômio extrapolado fora do
   domínio calibrado; guarda `if v_ms < 1.0 { return 0.0 }` e janela de
   tração nula em `[0,5; 1,0)` m/s permanecem, sem consumidor.
3. `z_drag_above_cg_m` ainda não consumido por `cm_thrust_cruise`.
4. Remoção de `to_distance_*` / `landing_distance_m` num bump MAJOR —
   fator 1,5 e os 200 m fixos ficaram visivelmente inconsistentes com
   os campos `*_50ft_*`. **Meça a razão real** `to_50ft/rolagem`.
5. Efeito solo omitido na rolagem (direção conservadora).

- [ ] **Step 3: Atualizar `src/main.rs`**

Os blocos `fidelity.*` são embutidos no JSON de produção — uma
afirmação falsa ali é uma afirmação falsa na entrega. Descreva o método
novo de rolagem e a descontinuidade de tração. **Não afirme nada que
você não mediu neste ciclo.**

- [ ] **Step 4: Report em `.superpowers/sdd/<plan>/task-5-report.md`**

Tabela `old→new` completa; estado de CADA gate (#23, #24, envelope,
tipback, folga de hélice); a lista literal de `violations` do JSON
final; e o que ficou fora com o item de backlog correspondente.

- [ ] **Step 5: `verifica-ciclo.sh` e commit**

```bash
git commit -m "docs: fecha backlog item 4 e registra os achados do ciclo 12"
```

---

## Auto-review do plano (feito, 2026-08-15)

- **Cobertura da spec:** §1→Task 2 (motivação da tração), §2→Task 2,
  §3→Task 2, §4→Task 2, §5→Task 3, §6→Task 4, §7→Tasks 2/3 (guardas),
  §8→Task 1, §9→números congelados por task, §10→Tasks 2/3 (vereditos),
  §11→Task 5 (backlog), §12→Global Constraints. Sem lacuna.
- **Consistência de tipos:** `cd_ground_roll` é produzida na Task 2 e
  consumida nas Tasks 3 e 4 com a mesma assinatura. `cl_ground_roll` é
  `f64` em todas as assinaturas. `state: &AircraftState` entra nas
  funções de pouso na Task 3 e não antes.
- **Ordem:** Task 1 não depende de número nenhum e pode ir primeiro;
  2 → 3 → 4 são seriais por cascata de pins no mesmo JSON; 5 fecha.
