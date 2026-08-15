# Ciclo 13 — Tração unificada: plano de implementação

> **Para operadores agênticos:** SUB-SKILL OBRIGATÓRIA — use
> `superpowers:subagent-driven-development` para executar task a task. Os
> passos usam checkbox (`- [ ]`).

**Goal:** substituir os dois modelos de tração do projeto por uma lei única
`T(V) = FoM(J)·T_ideal_momentum(V, P_eixo)`, ancorada na tração estática de
McCormick (J=0) e na eficiência JavaProp de cruzeiro (J=1,875), fechando os
itens de backlog #8, #9, #15 e #16.

**Architecture:** a cúbica de Rankine-Froude com avanço e seu solver de Newton
já existem e estão validados (`thrust_ground_roll_n`, ciclo 12). Este ciclo
troca o multiplicador constante `static_thrust_factor` por uma figura de mérito
`FoM(J)` linear entre duas âncoras medidas, funde as duas funções de tração em
uma só, apaga o polinômio `prop_efficiency`, e deriva a eficiência de cruzeiro
por inversão em forma fechada da mesma lei.

**Tech Stack:** Rust 2021, sem dependência nova. `cargo test`.

**Spec:** `docs/superpowers/specs/2026-08-15-ciclo13-tracao-unificada-design.md`
— **leia a spec inteira antes da sua task.** O plano argumenta a partir dela.

## Global Constraints

Copiadas literalmente da spec §13. Valem para TODA task.

- Rust 2021, **sem dependência nova**. `cargo test` inteiro tem que passar ao
  fim de cada task.
- **Nunca hardcodar dado de motor/célula em `src/`** — `tests/acceptance.rs`
  faz grep e reprova.
- **Nunca mascarar achado.** Escalar (parar e reportar, não seguir) quando:
  (a) um número diverge >5% do projetado na spec §11; (b) um relatório
  explicaria uma anomalia por causa não verificável; (c) uma tolerância ou um
  assert foi alterado; (d) um gate de validação flipou PASS→FAIL.
- Pins: bloco `old→new` comentado com valor antigo, valor novo e causa.
  **Tolerâncias INALTERADAS** — nenhuma pode ser alargada. Asserção relacional
  que deixou de valer é ACHADO: escreva a relação nova e verdadeira, viva, no
  lugar; não apague.
- `scripts/verifica-ciclo.sh` tem que voltar "Status geral: APROVADO".
- Commits frequentes e pequenos. Trailers:
  `Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>`
  `Claude-Session: https://claude.ai/code/session_01J8DCAdnLPaBhTHpu1rTQaT`
- **Armadilhas da spec §10 — leia antes de codar.** Em especial: a massa de
  performance é `state.mtow_kg` = 1537,389006 kg, NÃO `wb.spec.mtow_kg`; a
  fixture sintética `config_teste()` tem `static_thrust_factor = 0.72` (não
  0,75), `cl_max = 1,65` (não 2,10); `rotation_limit_pct_mac` HOJE é
  17,757974445030644%.

## Estrutura de arquivos

| Arquivo | Responsabilidade neste ciclo |
|---|---|
| `src/agents/propulsion.rs` | ganha `FigureOfMerit`; PERDE `prop_efficiency` e `thrust_n`; `search_cruise_rpm` passa a inverter em forma fechada |
| `src/agents/performance.rs` | `thrust_available_n` vira a lei única; PERDE `thrust_ground_roll_n`; ~40 assinaturas trocam `static_thrust_factor: f64` por `fom: FigureOfMerit` |
| `src/agents/trim_authority.rs` | `thrust_at_rotation_n` passa a chamar a lei única; balanço de rotação nas DUAS superfícies |
| `src/models/aircraft_config.rs` | `PropellerCfg` ganha 3 campos; `PerformanceCfg` perde `static_thrust_factor` |
| `src/models/config.rs` | validação dos 3 campos + guarda de migração |
| `src/models/specs.rs` | `SCHEMA_VERSION` 5.6; `TrimSpec` ganha 2 campos |
| `config/aircraft/*.toml` | migração dos campos |
| `docs/aircraft_spec.schema.md`, `docs/backlog.md` | contrato e backlog |

---

## Task 1 — `FigureOfMerit`: a curva e suas âncoras

**Tier declarado: MECÂNICA (Haiku).** Justificativa: puramente aditiva, nada
existente muda de comportamento, e a correção é integralmente provável por
`cargo test` — os valores das âncoras estão congelados na spec §3 a 17 dígitos.

**Files:**
- Modify: `src/agents/propulsion.rs` (novo tipo, topo do arquivo, depois de
  `advance_ratio`)
- Modify: `src/models/aircraft_config.rs:242` (`PropellerCfg`)
- Modify: `src/models/config.rs` (validação, perto da linha 1129)
- Modify: `config/aircraft/baseline_4seat.toml` (bloco `[propeller]`)
- Modify: TODOS os outros `config/aircraft/*.toml` (rodar `ls config/aircraft/`)
- Test: `src/agents/propulsion.rs` (módulo `tests` do próprio arquivo),
  `src/models/config.rs` (módulo `tests`)

**Interfaces:**
- Produces: `agents::propulsion::FigureOfMerit { fom_static, fom_design,
  j_design }` com `pub fn at(&self, j: f64) -> f64`; e
  `PropellerCfg::figure_of_merit(&self) -> FigureOfMerit`.
- Consumes: nada de tasks anteriores.

**NÃO faça nesta task:** não remova `[performance].static_thrust_factor`, não
toque em `thrust_available_n`, não toque em `thrust_ground_roll_n`. Esta task
termina com o baseline produzindo **exatamente os mesmos números de hoje**.

- [ ] **Passo 1: escreva os testes que falham** — em `src/agents/propulsion.rs`,
      dentro de `mod tests`:

```rust
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
```

- [ ] **Passo 2: rode e confirme que falha**

Run: `cargo test --lib propulsion::tests::figura_de_merito`
Expected: FAIL — `cannot find struct FigureOfMerit`.

- [ ] **Passo 3: implemente `FigureOfMerit`** — em `src/agents/propulsion.rs`,
      logo depois de `advance_ratio`:

```rust
/// Figura de mérito da hélice: tração REAL sobre tração IDEAL de disco
/// atuador na mesma potência de eixo (ciclo 13, spec §3).
///
///   FoM(J) = fom_static + (fom_design − fom_static)·min(J/j_design, 1)
///
/// Por definição `FoM ≤ 1` — uma hélice não produz mais tração que o disco
/// atuador ideal absorvendo a mesma potência (conservação de quantidade de
/// movimento, spec §1). Esta é a grandeza que substitui o polinômio
/// `prop_efficiency` apagado neste ciclo: aquele violava o teto físico em 5
/// dos 8 pontos de operação do baseline (spec §1.1).
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
```

- [ ] **Passo 4: rode e confirme que passa**

Run: `cargo test --lib propulsion::tests::figura_de_merito`
Expected: PASS (5 testes).

- [ ] **Passo 5: commit**

```bash
git add src/agents/propulsion.rs
git commit -m "feat(propulsion): FigureOfMerit — figura de mérito FoM(J) da hélice"
```

- [ ] **Passo 6: escreva o teste de config que falha** — em
      `tests/generic_engine.rs` (NÃO em `src/models/config.rs`: os literais
      abaixo são do baseline REAL, e os testes unitários de `config.rs` rodam
      contra TOML sintético — spec §10 item 4). Use os helpers que já existem
      lá (`baseline_state()`, linha 25):

```rust
/// As três âncoras da figura de mérito vêm do TOML, nunca do código
/// (política "nunca hardcodar dado de célula"). Valores do baseline real,
/// spec ciclo 13 §3.2 — derivados uma vez do polinômio JavaProp no ponto de
/// cruzeiro e congelados como propriedade da HÉLICE.
#[test]
fn baseline_declara_as_ancoras_da_figura_de_merito() {
    let cfg = baseline_state();
    assert_eq!(cfg.propeller.fom_static, 0.75);
    assert_eq!(cfg.propeller.fom_design, 0.823_706_394_572_155_44);
    assert_eq!(cfg.propeller.j_design,   1.875_143_480_257_116_75);

    // O construtor da curva lê os três campos e nada mais.
    let fom = cfg.propeller.figure_of_merit();
    assert_eq!(fom.at(0.0), 0.75);
    assert_eq!(fom.at(cfg.propeller.j_design), 0.823_706_394_572_155_44);
}
```

- [ ] **Passo 7: rode e confirme que falha**

Run: `cargo test --lib config::tests::carrega_ancoras`
Expected: FAIL — campo desconhecido / não existe em `PropellerCfg`.

- [ ] **Passo 8: adicione os campos** — em `src/models/aircraft_config.rs`,
      dentro de `pub struct PropellerCfg` (linha ~242), depois de
      `psru_efficiency`:

```rust
    /// Figura de mérito da hélice em J=0 (ciclo 13, spec §3.1) — fator
    /// empírico de McCormick sobre a tração estática ideal de disco atuador.
    /// MIGRADO de `[performance].static_thrust_factor`: mudou de lugar porque
    /// é propriedade da HÉLICE, não da política de performance, e porque
    /// deixou de ser um multiplicador plano para virar a âncora J=0 de uma
    /// curva (`FigureOfMerit`).
    pub fom_static: f64,
    /// Figura de mérito na razão de avanço de projeto da hélice (ciclo 13,
    /// spec §3.2). Retro-derivada uma vez do polinômio JavaProp no ponto de
    /// cruzeiro do baseline E12 — ver a premissa calibrada declarada em
    /// `agents::propulsion::FigureOfMerit`.
    pub fom_design: f64,
    /// Razão de avanço de projeto da hélice (ciclo 13, spec §3.2). Acima
    /// dela `FigureOfMerit::at` satura.
    pub j_design: f64,
```

- [ ] **Passo 8b: adicione o construtor da curva** — no mesmo arquivo, em
      `impl PropellerCfg` (crie o bloco se não existir):

```rust
impl PropellerCfg {
    /// Figura de mérito desta hélice a partir das três âncoras do TOML —
    /// fonte única de verdade da curva `FoM(J)` (ciclo 13, spec §3). Todo
    /// consumidor de tração chama isto em vez de ler os campos soltos.
    pub fn figure_of_merit(&self) -> crate::agents::propulsion::FigureOfMerit {
        crate::agents::propulsion::FigureOfMerit {
            fom_static: self.fom_static,
            fom_design: self.fom_design,
            j_design:   self.j_design,
        }
    }
}
```

- [ ] **Passo 9: adicione a validação** — em `src/models/config.rs`, junto das
      validações de `propeller` (procure `require_positive("propeller.`):

```rust
    require_positive("propeller.fom_static", cfg.propeller.fom_static)?;
    require_positive("propeller.fom_design", cfg.propeller.fom_design)?;
    require_positive("propeller.j_design",   cfg.propeller.j_design)?;
    // Teto de quantidade de movimento (spec §1): a figura de mérito é uma
    // FRAÇÃO da tração ideal. FoM > 1 significaria uma hélice produzindo mais
    // empuxo que o disco atuador ideal na mesma potência — impossível.
    for (nome, v) in [("propeller.fom_static", cfg.propeller.fom_static),
                      ("propeller.fom_design", cfg.propeller.fom_design)] {
        if v > 1.0 {
            return Err(ConfigError::Validation(format!(
                "configuração de aeronave inválida: {nome} = {v} excede 1,0 — a figura de \
                 mérito é a fração da tração IDEAL de disco atuador que a hélice real \
                 entrega, e passar de 1,0 violaria a conservação de quantidade de \
                 movimento (ver agents::propulsion::FigureOfMerit)"
            )));
        }
    }
```

- [ ] **Passo 10: adicione ao `[propeller]` de `config/aircraft/baseline_4seat.toml`**

```toml
# Figura de mérito da hélice (ciclo 13, spec §3) — tração real / tração ideal
# de disco atuador na mesma potência de eixo. Substitui o polinômio
# `prop_efficiency` e o multiplicador plano `[performance].static_thrust_factor`.
# fom_static: fator de McCormick em J=0 (era `static_thrust_factor = 0.75`).
# fom_design/j_design: retro-derivados UMA VEZ do polinômio JavaProp no ponto
# de cruzeiro do baseline E12, para que alcance e autonomia não se movam.
fom_static = 0.75
fom_design = 0.82370639457215544
j_design   = 1.87514348025711675
```

- [ ] **Passo 11: replique nos demais TOMLs de aeronave**

Run: `ls config/aircraft/`. Para cada arquivo, adicione o mesmo bloco. Se
algum tiver `[performance].static_thrust_factor` diferente de 0,75, use ESSE
valor como `fom_static` daquele arquivo (e registre no relatório) — nunca
uniformize valores entre células diferentes sem dizer.

- [ ] **Passo 12: atualize a fixture sintética** — `src/models/aircraft_config.rs:948`
      tem `static_thrust_factor: 0.72`. A fixture precisa dos 3 campos novos.
      Use `fom_static: 0.72` (preserva o valor da fixture), `fom_design: 0.82`,
      `j_design: 1.9`. **Não copie os números do baseline real para a fixture**
      — spec §10 item 4. Procure também o TOML sintético embutido em
      `src/models/config.rs:1646` e acrescente os campos lá.

- [ ] **Passo 13: rode a suíte inteira**

Run: `cargo test`
Expected: PASS, contagem de testes = 496 + 6 (5 de FoM + 1 de config).
**Nenhum número do baseline pode ter mudado nesta task** — se algum pin
quebrou, PARE e reporte: significa que os campos novos vazaram para um caminho
de cálculo, o que esta task proíbe.

- [ ] **Passo 14: commit**

```bash
git add src/models/aircraft_config.rs src/models/config.rs config/aircraft/
git commit -m "feat(config): âncoras da figura de mérito em [propeller]"
```

---

## Task 2 — A lei única: `thrust_available_n` reescrita, modelos velhos apagados

**Tier declarado: JULGAMENTO (Sonnet 5).** É a task central do ciclo. Apaga
três funções públicas, muda ~40 assinaturas e reescreve o modelo físico.

**Files:**
- Modify: `src/agents/performance.rs` (`thrust_available_n` linhas 65–87;
  `thrust_ground_roll_n` 145–192; ~40 assinaturas com `static_thrust_factor`)
- Modify: `src/agents/propulsion.rs` (apaga `prop_efficiency`, `thrust_n`)
- Modify: `src/agents/trim_authority.rs:272-292` (`thrust_at_rotation_n`)
- Modify: `src/models/aircraft_config.rs` (remove
  `PerformanceCfg::static_thrust_factor`)
- Modify: `src/models/config.rs` (remove validação; ADICIONA guarda de
  migração), `config/aircraft/*.toml` (remove a linha antiga)
- Test: `src/agents/performance.rs` (`mod tests`), `tests/generic_engine.rs`

**Interfaces:**
- Consumes: `FigureOfMerit` e `PropellerCfg::{fom_static, fom_design, j_design}`
  da Task 1.
- Produces:
  `performance::thrust_available_n(v_ms, engine, engine_rpm, psru_ratio,
  prop_diam_m, altitude_m, isa_delta_c, fom: FigureOfMerit, psru_efficiency)
  -> f64` — **a única função de tração do projeto**;
  `performance::thrust_ideal_momentum_n(v_ms, engine, engine_rpm, prop_diam_m,
  altitude_m, isa_delta_c, psru_efficiency) -> f64` — o teto físico, `pub`
  para que os testes possam medir contra ele.

### ONDE CADA TESTE MORA (leia antes do Passo 1)

`src/agents/performance.rs::tests::fixture_baseline()` (linha 1223) monta a
fixture **SINTÉTICA** (`config_teste()`), não o baseline real: ela tem
`static_thrust_factor = 0.72`, `cl_max = 1,65` e outro motor. Plantar nela um
literal do baseline real produz teste verde que não prova nada — spec §10
item 4.

Divisão obrigatória:

| Teste | Arquivo | Fixture | Por quê |
|---|---|---|---|
| teto de quantidade de movimento | `src/agents/performance.rs` | `fixture_baseline()` sintética | é propriedade RELACIONAL — vale para qualquer config, e tem que valer |
| continuidade | idem | idem | relacional |
| monotonicidade / positividade | idem | idem | relacional |
| **identidade estática = 3740,0919357793 N** | **`tests/generic_engine.rs`** | **`baseline_state()`** | literal do baseline REAL |
| **âncora de cruzeiro = 0,78388149656765982** (Task 3) | **`tests/generic_engine.rs`** | **`baseline_state()`** | literal do baseline REAL |

Nos três primeiros, use `fixture_baseline()` e as âncoras da fixture
sintética via `config_teste().propeller.figure_of_merit()` — **não** as do
baseline.

- [ ] **Passo 1: escreva a guarda central, contra o modelo de HOJE** — em
      `src/agents/performance.rs`, `mod tests`. Este teste tem que ser escrito
      e RODADO ANTES de qualquer mudança de implementação:

```rust
/// TETO DE QUANTIDADE DE MOVIMENTO (ciclo 13, spec §1 e §8.1).
///
/// Nenhuma hélice produz mais tração que um disco atuador IDEAL absorvendo a
/// mesma potência de eixo. Isso não é calibração — é conservação de momento,
/// e vale para qualquer modelo de tração que este projeto venha a ter.
///
/// Este teste foi escrito PRIMEIRO, contra o `thrust_available_n` do commit
/// `ed537ae`, e FALHOU em 5 dos 8 pontos de operação do baseline (spec §1.1):
/// 2,1432x em V=10 m/s, 1,3417x em V=20, 1,0372x em V_LOF, 1,0095x em Vx e
/// 1,0049x no teto de serviço em Vy — os três últimos alimentando gates que
/// PASSAVAM. É a guarda que 496 testes não tinham.
#[test]
fn tracao_nunca_excede_o_teto_de_quantidade_de_movimento() {
    let (engine, state, _wing) = fixture_baseline();
    let fom = config_teste().propeller.figure_of_merit();
    let rpm = engine.rpm_max_continuous;
    let d = state.prop_diameter_m;
    for i in 1..=1200 {
        let v = i as f64 * 0.1;              // 0,1 .. 120,0 m/s
        let t = thrust_available_n(v, &engine, rpm, state.psru_ratio, d,
                                   0.0, 0.0, fom, state.psru_efficiency);
        let teto = thrust_ideal_momentum_n(v, &engine, rpm, d, 0.0, 0.0,
                                           state.psru_efficiency);
        assert!(t <= teto * (1.0 + 1e-12),
                "V={v} m/s: T={t} N excede o teto ideal de {teto} N (razão {})",
                t / teto);
    }
}

/// CONTINUIDADE (spec §8.4). O modelo de hoje tem DOIS degraus artificiais:
/// o ramo `if v_ms < 0.5` de `thrust_available_n` e a guarda
/// `if v_ms < 1.0 { return 0.0 }` de `propulsion::thrust_n`. Juntos produzem
/// uma janela de tração NULA em [0,5; 1,0) m/s seguida de um salto para
/// 84.843,5 N em V=1,0 — 23x a tração estática. A lei única não tem ramo
/// nenhum, então não pode ter degrau.
#[test]
fn tracao_e_continua_em_toda_a_faixa() {
    let (engine, state, _wing) = fixture_baseline();
    let fom = config_teste().propeller.figure_of_merit();
    let rpm = engine.rpm_max_continuous;
    let d = state.prop_diameter_m;
    let t_de = |v: f64| thrust_available_n(v, &engine, rpm, state.psru_ratio, d,
                                           0.0, 0.0, fom, state.psru_efficiency);
    for i in 0..10_000 {
        let v = i as f64 * 0.01;
        let salto = (t_de(v + 0.01) - t_de(v)).abs();
        assert!(salto < 5.0, "degrau de {salto} N entre V={v} e V={}", v + 0.01);
    }
}

/// A tração cai ESTRITAMENTE com a velocidade a potência e densidade fixas —
/// a figura de mérito sobe com J, mas a tração ideal cai mais rápido. Se esta
/// falhar, a curva de FoM está subindo rápido demais para ser física.
#[test]
fn tracao_cai_estritamente_com_a_velocidade() {
    let (engine, state, _wing) = fixture_baseline();
    let fom = config_teste().propeller.figure_of_merit();
    let rpm = engine.rpm_max_continuous;
    let d = state.prop_diameter_m;
    let mut anterior = f64::INFINITY;
    for i in 0..=10_000 {
        let v = i as f64 * 0.01;
        let t = thrust_available_n(v, &engine, rpm, state.psru_ratio, d,
                                   0.0, 0.0, fom, state.psru_efficiency);
        assert!(t < anterior, "tração não caiu em V={v}: {t} >= {anterior}");
        assert!(t > 0.0, "tração nula ou negativa em V={v} — janela nula de novo");
        anterior = t;
    }
}

/// IDENTIDADE ESTÁTICA (spec §3.1 e §8.2). Em V=0 a cúbica degenera em
/// u³ = K, e a lei nova reproduz `static_thrust_ideal_n × fom_static`
/// ALGEBRICAMENTE, não por aproximação. Congelado no baseline real:
/// 3740,0919357793 N — o mesmo número de antes do ciclo. Se este valor
/// mudar, algo além da tração mudou.
/// A identidade estática (spec §3.1) precisa do BASELINE REAL — vive em
/// `tests/generic_engine.rs`, não aqui. Este teste só guarda a relação
/// invariante de config: em V=0 a lei nova coincide com
/// `static_thrust_ideal_n × fom_static`, qualquer que seja a hélice.
#[test]
fn tracao_estatica_e_o_ideal_vezes_a_ancora_de_j_zero() {
    let (engine, state, _wing) = fixture_baseline();
    let fom = config_teste().propeller.figure_of_merit();
    let t0 = thrust_available_n(0.0, &engine, engine.rpm_max_continuous,
                                state.psru_ratio, state.prop_diameter_m,
                                0.0, 0.0, fom, state.psru_efficiency);
    let ideal = static_thrust_ideal_n(&engine, engine.rpm_max_continuous,
                                      state.prop_diameter_m, 0.0, 0.0,
                                      state.psru_efficiency);
    let esperado = ideal * fom.fom_static;
    assert!((t0 - esperado).abs() / esperado < 1e-12,
            "V=0: T={t0}, ideal×fom_static={esperado}");
}
```

E em **`tests/generic_engine.rs`**, com o baseline REAL:

```rust
/// IDENTIDADE ESTÁTICA DO BASELINE REAL (ciclo 13, spec §3.1 e §8.2).
///
/// `old→new`: este número NÃO muda com o ciclo 13. A lei nova é uma
/// REFINAÇÃO do modelo estático, não uma substituição — em V=0 a cúbica de
/// Rankine-Froude degenera em u³ = K e T = fom_static·(2ρA·P²)^(1/3), que é
/// algebricamente o ramo estático de antes. Se este valor se mover, algo
/// além da tração mudou (potência de eixo, densidade, diâmetro ou PSRU).
#[test]
fn tracao_estatica_do_baseline_real_permanece_congelada() {
    let cfg = baseline_state();
    let state = AircraftState::from_config(&cfg);
    let engine = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let fom = cfg.propeller.figure_of_merit();
    let t0 = aeronave::agents::performance::thrust_available_n(
        0.0, &engine, engine.rpm_max_continuous, state.psru_ratio,
        state.prop_diameter_m, 0.0, 0.0, fom, state.psru_efficiency);
    assert!((t0 - 3740.091_935_779_3).abs() < 1e-6,
            "tração estática = {t0} N, congelado = 3740,0919357793 N");
}
```

> **Sobre o literal 3740,0919357793:** medido pelo chefe com os valores
> extraídos do próprio código (`P_eixo(3000 rpm, 0 m) = 144,240990702 kW`,
> `ρ_sl = 1,22501226599069457`, D = 1,76 m). Se der diferente já na 7ª casa,
> **reporte o valor medido e a diferença — não ajuste a tolerância.**

- [ ] **Passo 2: rode e confirme que falha DO JEITO CERTO**

Run: `cargo test --lib performance::tests::tracao_ -- --nocapture`
Expected: `tracao_nunca_excede_o_teto` FALHA já em V≈10 m/s com razão ≈2,14;
`tracao_e_continua` FALHA em V=0,5 e V=1,0; `tracao_cai_estritamente` FALHA na
janela nula. **Copie a saída dessas falhas para o relatório da task** — é a
prova documental do defeito, e o revisor vai pedir.

`thrust_ideal_momentum_n` ainda não existe, então será preciso criá-la antes
de o teste compilar. Crie-a NESTE passo (é só a cúbica sem o multiplicador),
sem tocar em `thrust_available_n` — assim a falha medida é do modelo velho.

- [ ] **Passo 3: extraia o teto físico como função pública**

Em `src/agents/performance.rs`, ao lado de `static_thrust_ideal_n`:

```rust
/// Tração IDEAL de disco atuador com velocidade de avanço (Rankine-Froude) —
/// o TETO físico de qualquer hélice absorvendo `P_eixo` (ciclo 13, spec §1).
///
///   T_ideal(V) = 2·ρ·A·u·(u − V),  u = raiz real de u³ − V·u² − K = 0,
///   K = P_eixo/(2ρA)
///
/// Solver: Newton a partir de `u₀ = V + K^(1/3)`, convergência monotônica
/// provada no ciclo 12. Em V=0 degenera em `u = K^(1/3)` e o resultado é
/// algebricamente idêntico a `static_thrust_ideal_n`.
///
/// `pub` de propósito: é a referência contra a qual
/// `tracao_nunca_excede_o_teto_de_quantidade_de_movimento` mede. Um teto que
/// só existe dentro da função que ele limita não é falseável.
pub fn thrust_ideal_momentum_n(
    v_ms: f64, engine: &EngineSpec, engine_rpm: f64, prop_diam_m: f64,
    altitude_m: f64, isa_delta_c: f64, psru_efficiency: f64,
) -> f64 {
    let p_w = shaft_power_kw(engine, engine_rpm, altitude_m, psru_efficiency) * 1_000.0;
    let rho = Isa::density_kgm3(altitude_m, isa_delta_c);
    let disk_area = std::f64::consts::PI * (prop_diam_m / 2.0).powi(2);
    let k = p_w / (2.0 * rho * disk_area);
    // Guarda do ciclo 12 preservada: K ≤ 0 ∧ V = 0 daria 0/0 no primeiro
    // Newton (NaN silencioso). Sem potência, o disco não produz empuxo.
    if k <= 0.0 {
        return 0.0;
    }
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
    2.0 * rho * disk_area * u * (u - v_ms)
}
```

- [ ] **Passo 4: reescreva `thrust_available_n` como a lei única**

Substitua o corpo INTEIRO (linhas ~65–87). A docstring nova precisa contar a
história — spec §1, §2 e §4:

```rust
/// Tração disponível da hélice — **a única lei de tração do projeto**
/// (ciclo 13, spec §2).
///
///   T(V) = FoM(J) · T_ideal_momentum(V, P_eixo)
///
/// `old→new` (ciclo 12 → ciclo 13). Antes deste ciclo esta função tinha DOIS
/// ramos: disco atuador estático × `static_thrust_factor` abaixo de
/// V=0,5 m/s, e `prop_efficiency(J)·P/V` acima. O ramo de voo violava o TETO
/// DE QUANTIDADE DE MOVIMENTO em 5 dos 8 pontos de operação do baseline
/// (spec §1.1) — 2,1432x em V=10 m/s, e também em `Vx`, em `V_LOF` e no teto
/// de serviço, os três alimentando gates que PASSAVAM. Nenhuma hélice
/// produz mais empuxo que um disco atuador ideal na mesma potência; o
/// polinômio dizia que sim.
///
/// A função paralela `thrust_ground_roll_n` (ciclo 12, quantidade de
/// movimento com avanço, usada só na rolagem) foi FUNDIDA aqui: os dois
/// modelos descreviam a mesma grandeza e divergiam 27,69% em `V_LOF`,
/// quebrando a identidade de d'Alembert do balanço de rotação (backlog #15).
/// Com uma lei só, o resíduo daquela identidade é ZERO por construção — ver
/// `agents::trim_authority::rotation_available_moment_nm`.
///
/// Sem ramos ⟹ sem degraus: morrem juntos o `η(0) = 0,58` (por definição
/// η = T·V/P → 0 quando V → 0), o salto de 84.843,5 N em V=1,0 m/s, a janela
/// de tração NULA em V ∈ [0,5; 1,0) e o corte duro em J > 2,8.
pub fn thrust_available_n(
    v_ms: f64,
    engine: &EngineSpec,
    engine_rpm: f64,
    psru_ratio: f64,
    prop_diam_m: f64,
    altitude_m: f64,
    isa_delta_c: f64,
    fom: FigureOfMerit,
    psru_efficiency: f64,
) -> f64 {
    let t_ideal = thrust_ideal_momentum_n(v_ms, engine, engine_rpm, prop_diam_m,
                                          altitude_m, isa_delta_c, psru_efficiency);
    let n_prop = prop_rpm(engine_rpm, psru_ratio);
    let j = advance_ratio(v_ms, n_prop, prop_diam_m);
    fom.at(j) * t_ideal
}
```

- [ ] **Passo 5: apague `thrust_ground_roll_n`** e faça o integrador de rolagem
      chamar `thrust_available_n`. **Preserve o teste de identidade do ciclo 12**
      (`tracao_de_rolagem_em_v_zero_e_identica_ao_estatico_no_baseline_real`)
      reescrito contra a função nova — não apague, spec §8.7.

- [ ] **Passo 6: apague `prop_efficiency` e `thrust_n`** de
      `src/agents/propulsion.rs`. A Task 3 conserta `search_cruise_rpm`; até
      lá, **use um `todo!()` explícito ali com comentário apontando a Task 3**,
      ou implemente já a inversão fechada se for natural. Diga no relatório
      qual caminho escolheu. `advance_ratio` e `prop_rpm` FICAM.

- [ ] **Passo 7: troque `static_thrust_factor: f64` por `fom: FigureOfMerit`**
      em todas as ~40 assinaturas. Comando para achar todas:

```bash
grep -rn "static_thrust_factor" --include=*.rs src/ | wc -l
```

Onde faltar `psru_ratio` na cadeia de chamada, acrescente. **Exceção a
conferir:** `landing_ground_roll_m`/`landing_distance_50ft_m` — a rolagem de
pouso NÃO tem termo de tração. Se não consomem tração, **não mude a
assinatura delas**; diga isso no relatório.

**Atenção a `src/agents/mission.rs:287`**, que hoje passa
`static_thrust_factor = 1.0` literal com um comentário dizendo ser "parâmetro
exigido pela assinatura". Com a lei nova, FoM=1,0 significa disco IDEAL — o
teto físico, não um valor neutro. Descubra o que aquela chamada realmente
precisa, corrija, e **reporte como achado** se o 1,0 estava mascarando algo.

- [ ] **Passo 8: remova `[performance].static_thrust_factor` e adicione a guarda
      de migração** — em `src/models/config.rs`, no mesmo padrão de
      `check_shaft_height_migration` (linha 429), registrando na lista de
      chamadas (linhas 207–215):

```rust
/// Guarda de migração (ciclo 13, spec §9.2): `[performance].
/// static_thrust_factor` foi MOVIDO para `[propeller].fom_static` e deixou de
/// ser um multiplicador plano — virou a âncora J=0 de uma figura de mérito
/// (`agents::propulsion::FigureOfMerit`). Sem `deny_unknown_fields`, um TOML
/// antigo seria aceito em SILÊNCIO com a tração vindo só dos defaults novos.
fn check_static_thrust_factor_migration(toml_str: &str) -> Result<(), ConfigError> {
    let raw: toml::Value = toml::from_str(toml_str)?;
    if raw.get("performance").and_then(|p| p.get("static_thrust_factor")).is_some() {
        return Err(ConfigError::Validation(
            "configuração de aeronave inválida: [performance].static_thrust_factor foi \
             substituído pela figura de mérito da hélice — remova o campo e adicione \
             [propeller].fom_static (mesmo valor), [propeller].fom_design e \
             [propeller].j_design (ver docs/aircraft_spec.schema.md e \
             config/aircraft/baseline_4seat.toml)"
                .to_string(),
        ));
    }
    Ok(())
}
```

Escreva o teste da guarda no mesmo padrão dos testes de migração existentes.
Atualize os dois testes de validação antigos
(`rejeita_static_thrust_factor_nao_positivo`,
`rejeita_static_thrust_factor_acima_de_1`, `config.rs:2644` e `2657`) para
apontarem aos campos novos — **reescrever, não apagar**: as duas propriedades
que eles guardam continuam valendo.

- [ ] **Passo 9: rode as guardas e confirme que agora PASSAM**

Run: `cargo test --lib performance::tests::tracao_ -- --nocapture`
Expected: PASS nas 4.

- [ ] **Passo 10: rode a suíte inteira e MEÇA o estrago**

Run: `cargo test 2>&1 | tail -60`

Vários pins vão quebrar — é esperado. Para CADA um: registre o valor antigo,
o novo, e a causa, e corrija o pin com bloco `old→new` comentado,
**mantendo a tolerância**. Se algum pin quebrar por mais de 5% do que a spec
§11 projeta, **PARE e escale**.

Rode também a sonda de decolagem para comparar com a spec §3.4 (previsto
`to_50ft_grass_m ≈ 784,5 m`).

- [ ] **Passo 11: commit**

```bash
git add -A
git commit -m "feat(performance): lei única de tração T(V) = FoM(J)·T_ideal"
```

---

## Task 3 — Cruzeiro: inversão em forma fechada, η derivada

**Tier declarado: JULGAMENTO (Sonnet 5).**

**Files:**
- Modify: `src/agents/propulsion.rs` (`search_cruise_rpm` ~linha 118–140,
  `PropulsionAgent::run` ~linha 252, e ~linha 280)
- Test: `src/agents/propulsion.rs` (`mod tests`), `tests/generic_engine.rs`

**Interfaces:**
- Consumes: `FigureOfMerit` (Task 1), `thrust_ideal_momentum_n` (Task 2).
- Produces: `PropulsionSpec::prop_efficiency` com o mesmo nome/tipo, agora
  DERIVADO. Nenhuma assinatura pública nova.

- [ ] **Passo 1: escreva a guarda da âncora de cruzeiro (spec §8.3)**

```rust
/// ÂNCORA DE CRUZEIRO (ciclo 13, spec §3.2 e §8.3). `fom_design` foi
/// retro-derivada para que a lei nova reproduza, no ponto de cruzeiro do
/// baseline, a eficiência que o polinômio JavaProp apagado entregava —
/// preservando alcance, autonomia e consumo POR CONSTRUÇÃO.
///
/// O que esta guarda de fato verifica não é física, é CONCORDÂNCIA ENTRE
/// IMPLEMENTAÇÕES: `fom_design` foi computada em Python (sonda do chefe) com
/// uma ISA, uma curva de potência e um Newton; este teste roda os três em
/// Rust. Divergência acima de 1e-9 significa que um dos três difere — o modo
/// de falha que o ciclo 12 só pegou por sonda manual.
#[test]
fn eficiencia_de_cruzeiro_reproduz_a_ancora_do_polinomio_apagado() {
    // η esperado = valor que `prop_efficiency(1.87514348025711675)` devolvia
    // no commit ed537ae, antes de a função ser apagada.
    const ETA_ANCORA: f64 = 0.783_881_496_567_659_82;
    let cfg = baseline_state();
    let state = AircraftState::from_config(&cfg);
    let req = baseline_mission();
    let wing = AerodynamicsAgent::run(&state, &req);
    let engine = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let prop = PropulsionAgent::run(&state, &req, &wing, &engine);
    assert!((prop.prop_efficiency - ETA_ANCORA).abs() < 1e-9,
            "η de cruzeiro = {}, âncora = {ETA_ANCORA}", prop.prop_efficiency);
}
```

> Este teste vai em **`tests/generic_engine.rs`** (literal do baseline real —
> mesma regra da tabela da Task 2). `baseline_state()`, `baseline_mission()`,
> `config_path()` e os agentes já estão importados lá.

- [ ] **Passo 2: rode e confirme que falha** (a Task 2 deixou `search_cruise_rpm`
      quebrada ou com `todo!()`).

- [ ] **Passo 3: implemente a inversão fechada** em `search_cruise_rpm`,
      substituindo `let eta = prop_efficiency(j);` e o cálculo de `p_req_kw`:

```rust
        // Ciclo 13 (spec §5): em cruzeiro nivelado a tração exigida é
        // conhecida (T = arrasto), então a quadrática do disco atuador
        // inverte em FORMA FECHADA — sem ponto fixo e sem iteração. De
        // T = 2ρA·u·(u − V):
        //     u = [ V + √(V² + 2T/(ρA)) ] / 2
        // e daí P_ideal = T·u, P_eixo_req = P_ideal/FoM(J), η = FoM(J)·V/u.
        //
        // `old→new`: antes era `eta = prop_efficiency(J)` (polinômio JavaProp,
        // apagado no ciclo 13 por violar o teto de quantidade de movimento) e
        // `p_req = drag·V/(eta·1000)`. O caminho novo é mais robusto: não
        // depende de η ser positivo para não divergir, porque η é SAÍDA.
        let disk_area = std::f64::consts::PI * (prop_diameter_m / 2.0).powi(2);
        let rho = Isa::density_kgm3(altitude_m, 0.0);
        let u = (v_cruise_ms
                 + (v_cruise_ms * v_cruise_ms + 2.0 * drag_n / (rho * disk_area)).sqrt())
                / 2.0;
        let fom_j = fom.at(j);
        let eta = if u > 0.0 { fom_j * v_cruise_ms / u } else { 0.0 };
        let p_req_kw = if fom_j > 0.0 && u > 0.0 {
            drag_n * u / (fom_j * 1_000.0)
        } else {
            f64::INFINITY   // config degenerada — nunca NaN (spec §5)
        };
```

> **Confira o `isa_delta_c`:** o código de hoje usa `Isa::density_kgm3` em
> vários pontos com o delta da missão. Descubra qual é o delta correto neste
> escopo e use-o — não assuma 0,0 se a função tiver acesso ao valor real.

- [ ] **Passo 4: rode e confirme que a âncora passa**

Run: `cargo test --lib propulsion::tests::eficiencia_de_cruzeiro`
Expected: PASS.

- [ ] **Passo 5: verifique os invariantes de missão**

Run: `cargo test`
`range_km`, `endurance_h`, `fc_cruise_lph`, `bsfc_cruise_gkwh` e
`breguet_range_full_tank_km` **não podem se mover** além de ruído numérico. Se
moverem, a §3.2 foi violada — **PARE e escale.**

- [ ] **Passo 6: commit**

```bash
git add src/agents/propulsion.rs
git commit -m "feat(propulsion): cruzeiro por inversão fechada do disco atuador"
```

---

## Task 4 — Resíduo de d'Alembert zero + as duas superfícies da rotação

**Tier declarado: JULGAMENTO (Sonnet 5).**

**Files:**
- Modify: `src/agents/trim_authority.rs` (`thrust_at_rotation_n` 272–292;
  bloco de superfície ~809–814; `rotation_limit_pct_mac` ~853, ~880)
- Modify: `src/models/specs.rs` (`TrimSpec`: 2 campos novos)
- Modify: `src/main.rs` (`fidelity.trim`, ~linha 920)
- Test: `src/agents/trim_authority.rs` (`mod tests`), `tests/generic_engine.rs`

**Interfaces:**
- Consumes: `thrust_available_n` (Task 2).
- Produces: `TrimSpec::rotation_limit_pct_mac_paved`,
  `TrimSpec::rotation_limit_pct_mac_grass` (ambos `f64`).

- [ ] **Passo 1: escreva a guarda do resíduo zero (spec §8.6)**

```rust
/// RESÍDUO DE d'ALEMBERT ZERO — fecha o backlog #15 (PRIORIDADE ALTA).
///
/// A identidade que dá o braço `prop_axis_above_cg_m` (e não `h_cg + offset`)
/// ao momento de tração cancela a porção `h_cg` contra o termo inercial
/// `m·aₓ·h_cg` **porque o mesmo `T` aparece nos dois lados de
/// `m·aₓ = T − D − μN`**. O ciclo 12 quebrou isso: o termo de momento usava
/// `thrust_available_n` enquanto `D`/`μN` vinham do modelo de solo, cujo `T`
/// na MESMA velocidade era 27,69% menor — resíduo de −1.005,97 N·m
/// (−6,816 pp de MAC) no cenário governante.
///
/// Com a lei única do ciclo 13 os dois são a MESMA chamada da MESMA função.
/// `Vr ≡ V_LOF` é identidade algébrica, não coincidência: `VR_OVER_VS0 = 1.1`
/// sobre `Vs0_TO` é a mesma fórmula que `v_lof` em
/// `performance::takeoff_ground_roll_com_passos`.
#[test]
fn tracao_do_momento_de_rotacao_e_identica_a_da_rolagem_no_mesmo_vr() {
    let cfg = baseline_state();
    let state = AircraftState::from_config(&cfg);
    let req = baseline_mission();
    let engine = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let sized = size_aircraft(&cfg, &engine, &req).expect("baseline converge");
    let wing = &sized.wing;
    let fom = cfg.propeller.figure_of_merit();

    for sc in &sized.wb.scenarios {
        let w_n = sc.total_mass_kg * G;
        // Tração usada no termo de MOMENTO do balanço de rotação.
        let t_momento = aeronave::agents::trim_authority::thrust_at_rotation_n(
            w_n, wing.area_m2, wing.cl_max_to, &engine, &state, req.isa_delta_c, fom);
        // Tração que o modelo de SOLO enxerga na MESMA velocidade.
        // `Vr ≡ V_LOF = 1,10·√(2W/(ρ·S·CL_max_TO))` — identidade algébrica
        // entre `VR_OVER_VS0` (trim_authority.rs:81) e `v_lof`
        // (performance.rs:672), não coincidência numérica.
        let rho = Isa::density_kgm3(0.0, req.isa_delta_c);
        let v_lof = 1.10 * (2.0 * w_n / (rho * wing.area_m2 * wing.cl_max_to)).sqrt();
        let t_rolagem = aeronave::agents::performance::thrust_available_n(
            v_lof, &engine, engine.rpm_max_continuous, state.psru_ratio,
            state.prop_diameter_m, 0.0, req.isa_delta_c, fom, state.psru_efficiency);

        let erro_rel = (t_momento - t_rolagem).abs() / t_momento;
        assert!(erro_rel < 1e-12,
                "cenário '{}': momento={t_momento} N, solo={t_rolagem} N, \
                 erro_rel={erro_rel} — o resíduo do backlog #15 voltou",
                sc.name);
    }
}
```

> **Confira o rpm** usado por `thrust_at_rotation_n` hoje (a docstring da
> linha 254 fala em "menos pessimista"). Se ela usar `rpm_max_continuous`, o
> teste acima está certo; se usar outro, **use o mesmo dos dois lados** — o
> ponto do teste é que os dois caminhos sejam a MESMA avaliação, então
> qualquer divergência de argumento também é achado.

> **Confira a API de `size_aircraft`** (`src/orchestrator`) — o nome dos
> campos de retorno (`sized.wing`, `sized.wb.scenarios`) pode diferir. Use o
> padrão que `tests/generic_engine.rs` já usa nos testes que chamam
> `size_aircraft`.

- [ ] **Passo 2: rode e confirme que falha** contra o estado atual (divergência
      ≈27,69%). Registre a saída no relatório.

- [ ] **Passo 3: faça `thrust_at_rotation_n` chamar a lei única** com
      `fom` de `cfg.propeller.figure_of_merit()`. A docstring hoje (linha 254)
      diz "`performance::thrust_available_n`" — atualize com bloco `old→new`
      explicando que a divergência de 27,69% deixou de existir.

- [ ] **Passo 4: rode e confirme que passa.**

- [ ] **Passo 5: escreva o teste das duas superfícies (spec §7)**

```rust
/// SUPERFÍCIE DA ROTAÇÃO (ciclo 13, spec §7 — fecha o backlog #16).
///
/// O ciclo 12 avaliava o balanço de rotação em `mu_roll_paved` enquanto as
/// checagens #23/#24 reprovavam a GRAMA — o mesmo JSON afirmava duas
/// superfícies para a MESMA decolagem. Agora as duas são computadas e
/// publicadas, e o gate usa a de operação (GRAMA, premissa declarada na
/// spec §7: é a superfície que #23/#24 já medem e que o TOML de missão
/// descreve).
///
/// Atrito maior ⟹ mais momento contrário à rotação ⟹ limite dianteiro mais
/// RECUADO (maior %MAC). Direção falseável, independente do valor.
#[test]
fn limite_de_rotacao_em_grama_e_mais_restritivo_que_em_pavimentado() {
    let spec = /* TrimAuthorityAgent::run no baseline real */;
    assert!(spec.rotation_limit_pct_mac_grass > spec.rotation_limit_pct_mac_paved,
            "grama {} deveria ser MAIOR que pavimentado {}",
            spec.rotation_limit_pct_mac_grass, spec.rotation_limit_pct_mac_paved);
    assert_eq!(spec.rotation_limit_pct_mac, spec.rotation_limit_pct_mac_grass,
               "o campo legado tem que valer a superfície de OPERAÇÃO (grama)");
}
```

- [ ] **Passo 6: implemente as duas superfícies.** Em
      `TrimAuthorityAgent::run`, a closure `x_rot_para_peso` (linha ~855)
      fecha sobre `mu_roll_ground`. Parametrize-a:

```rust
        // Ciclo 13 (spec §7 — fecha o backlog #16): o limite de rotação é
        // avaliado nas DUAS superfícies e publicado nas duas. O campo legado
        // `rotation_limit_pct_mac` passa a valer a de OPERAÇÃO (grama).
        //
        // `old→new` (ciclo 12 → ciclo 13): o comentário que estava aqui dizia
        // que "pavimentada é a superfície menos conservadora das duas, mesma
        // lógica de 'menos pessimista' já usada para o rpm de tração". Essa
        // escolha ficou insustentável quando o próprio ciclo 12 mediu que as
        // checagens #23/#24 REPROVAM a grama: o mesmo JSON afirmava duas
        // superfícies para a MESMA decolagem.
        let limite_para_superficie = |mu_roll_ground: f64| -> f64 {
            let x_rot_para_peso = |w_n: f64| -> f64 { /* corpo de hoje, com este mu */ };
            let x_rot = wb.scenarios.iter()
                .map(|sc| x_rot_para_peso(sc.total_mass_kg * G))
                .fold(f64::NEG_INFINITY, f64::max);
            cg_pct_mac(x_rot, mac_le, mac)
        };
        let rotation_limit_pct_paved = limite_para_superficie(cfg.performance.mu_roll_paved);
        let rotation_limit_pct_grass = limite_para_superficie(cfg.performance.mu_roll_grass);
        let rotation_limit_pct = rotation_limit_pct_grass;   // superfície de OPERAÇÃO
```

Faça o mesmo com `mu_roll_ground` no laço de
`rotation_margin_per_scenario` (linha ~876), usando a superfície de operação.

- [ ] **Passo 6b: atualize o comentário da agregação MAX** (linhas ~826–841).
      Ele diz que o máximo-sobre-cenários é robusto por construção e que a
      alternativa "avaliar no cenário mais leve" **"deixaria de valer em
      silêncio se a curva de eficiência da hélice ou a política de `Vr`
      mudassem"**. Este ciclo é exatamente esse evento: a curva de eficiência
      foi substituída. Registre num bloco `old→new` que a premissa antecipada
      se concretizou e que a construção MAX seguiu válida — é a defesa do
      ciclo 10 cobrando o prêmio dela.

- [ ] **Passo 7: atualize `fidelity.trim` em `src/main.rs`.** O texto de hoje
      declara a indeterminação de ≈8,3 pp do backlog #15. **Essa
      indeterminação acabou** — reescreva dizendo o que passou a valer, e não
      deixe o texto antigo. (Ciclo 11 teve dois textos MENTINDO dentro do JSON
      de produção; não repita.)

- [ ] **Passo 8: `cargo test`, corrija pins com `old→new`, commit.**

**MEÇA E REPORTE, sem ajustar nada:** `rotation_limit_pct_mac` nas duas
superfícies, a margem dos 6 cenários, e se 'Solo (piloto)' virou violação
NOMINAL em grama (spec §11 avisa que pode). Isso é resultado.

---

## Task 5 — Schema 5.6, JSON, backlog, pins e veredito

**Tier declarado: JULGAMENTO (Sonnet 5).**

**Files:**
- Modify: `src/models/specs.rs` (`SCHEMA_VERSION` linha ~1445)
- Modify: `docs/aircraft_spec.schema.md`, `docs/backlog.md`
- Modify: `src/main.rs` (`fidelity.propeller`, `fidelity.performance`)
- Modify: `aircraft_spec.json` (regenerado), `tests/*.rs` (pins)

- [ ] **Passo 1: `SCHEMA_VERSION` 5.5 → 5.6.** Atualize os testes de schema.

- [ ] **Passo 2: regenere o JSON** com o mesmo comando de regeneração que o
      projeto já usa (procure em `scripts/verifica-ciclo.sh`).

- [ ] **Passo 3: sincronize TODOS os pins** em `tests/generic_engine.rs`,
      `tests/cli.rs`, `tests/gear_tipback.rs`, `tests/schema_v4.rs`. Cada um
      com bloco `old→new` (valor antigo, novo, causa). **Tolerâncias
      inalteradas.**

- [ ] **Passo 4: `docs/aircraft_spec.schema.md`** — bloco `trim` (2 campos
      novos), bloco `propulsion` (mudança de ORIGEM de `prop_efficiency`),
      bloco `performance`, e histórico v5.6 com as duas exceções registradas
      da spec §9.1.

- [ ] **Passo 5: `docs/backlog.md`** — marque RESOLVIDO com a medição:
      **#8** (unificar tração), **#9** (η(0)=0,58 e janela nula),
      **#15** (inconsistência no balanço de rotação), **#16** (superfície).
      Registre o mecanismo e os números medidos, não só "resolvido".

      **Abra os itens NOVOS** (spec §12):
      - **#17** — segmento aéreo do pouso é `15/tan(3°)` = 286,2 m, 44% da
        distância de pouso em grama; o planeio power-off desta célula na
        própria configuração de pouso é 5,118° (L/D 11,165 a V_ref). A rampa
        de 3° é convenção de ILS de aeroporto pavimentado, nunca calibrada
        para pista de fazenda. Ciclo 14.
      - **#18** — `j_design` congelada em config não se reajusta se velocidade
        de cruzeiro, `psru_ratio` ou diâmetro mudarem: âncora obsoleta em
        silêncio (spec §3.3 item 2).
      - **#19** — o cruzeiro opera em J = 1,875 enquanto o polinômio apagado
        tinha pico em J = 1,30 (η 0,8335 contra 0,7839): ≈6% de eficiência
        propulsiva na mesa. Decisão de projeto de hélice/PSRU.

- [ ] **Passo 6: `fidelity.*` em `src/main.rs`.** Os textos de `propeller`,
      `performance` e `trim` descrevem o modelo de tração ANTIGO em vários
      pontos (linhas ~771, ~789, ~865, ~920). **Reescreva todos.** Um texto que
      mente dentro do JSON de produção é achado de revisão, não detalhe.

- [ ] **Passo 7: `scripts/verifica-ciclo.sh`**

Run: `bash scripts/verifica-ciclo.sh`
Expected: "Status geral: APROVADO".

- [ ] **Passo 8: relatório final da task** com a tabela `old→new` COMPLETA de
      todo campo de `aircraft_spec.json` que mudou, o `validation_status`
      final, a lista de violações, e a comparação item a item contra a
      projeção da spec §11 — **incluindo o que a projeção errou.**

- [ ] **Passo 9: commit.**
