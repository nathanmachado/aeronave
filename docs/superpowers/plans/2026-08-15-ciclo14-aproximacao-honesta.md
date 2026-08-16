# Ciclo 14 — Aproximação honesta: plano de implementação

> **Para operadores agênticos:** SUB-SKILL OBRIGATÓRIA — use
> `superpowers:subagent-driven-development`. Os passos usam checkbox (`- [ ]`).

**Goal:** substituir o segmento aéreo do pouso — hoje 52,5% da distância em
grama e composto por duas heurísticas não calibradas — por geometria de
flare correta e um ângulo de aproximação derivado da polar de pouso.

**Architecture:** `γ_app` deixa de ser config e passa a ser `atan(CD_ref/CL_ref)`
na configuração de pouso a `V_ref`; o flare deixa de ser `V_ref × tempo` e
passa a ser um arco de raio `R = V_ref²/(g(n−1))` que **consome altura**, de
modo que a rampa só desce `15 − h_flare`.

**Tech Stack:** Rust 2021, sem dependência nova.

**Spec:** `docs/superpowers/specs/2026-08-15-ciclo14-aproximacao-honesta-design.md`
— **leia inteira antes da sua task.**

## Global Constraints

Copiadas da spec §9. Valem para TODA task.

- Rust 2021, **sem dependência nova**. `cargo test` verde ao fim de cada task.
- **Nunca hardcodar dado de motor/célula em `src/`** — `tests/acceptance.rs`
  faz grep e reprova.
- **Nunca mascarar achado.** ESCALE (pare e reporte) se: um número diverge
  >5% do projetado na spec §7; **um número FORA do pouso se move** (a mudança
  é isolada por construção, spec §3); uma tolerância ou assert é alterado; um
  gate flipa de forma não explicada.
- Pins: `old→new` com valor antigo, novo, delta e causa. **Tolerâncias
  INALTERADAS.** Asserção relacional que deixou de valer é ACHADO — escreva a
  relação nova e verdadeira, viva, no lugar; não apague.
- `scripts/verifica-ciclo.sh` tem que voltar "Status geral: APROVADO".
- Trailers de commit:
  `Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>`
  `Claude-Session: https://claude.ai/code/session_01J8DCAdnLPaBhTHpu1rTQaT`
- Mensagens de commit multi-linha: escreva num arquivo em
  `/tmp/claude-1000/-home-nathan-Desenvolvimento-aeronave/c861d850-8b3e-42a2-8714-922b95bda85e/scratchpad/`
  e use `git commit -F <arquivo>`. **Não use heredoc no git.**

## Números do baseline (medidos, use-os)

    V_ref            = 35,7351 m/s (128,65 km/h)     m_ldg = 1407,292 kg
    CL_ref           = 1,2426     CD_ref = 0,1113    L/D = 11,165
    γ_app derivado   = 5,1181°
    n (flare)        = 1,20  →  R = 651,1 m, h_flare = 2,596 m, s_flare = 58,08 m
    s_air            = 138,49 m   →  aéreo = 196,57 m
    rolagem pouso    = 242,70 m (pavimentado) / 306,84 m (grama)  [NÃO muda]
    ldg_50ft_m       582,521767 → ≈439,3      ldg_50ft_grass_m 646,660942 → ≈503,4

## Estrutura de arquivos

| Arquivo | Responsabilidade |
|---|---|
| `src/agents/performance.rs` | `cd_ground_roll` → `cd_gear_extended`; `landing_distance_50ft_m` reescrita |
| `src/models/aircraft_config.rs` | `PerformanceCfg`: `flare_load_factor` entra, `approach_angle_deg`/`flare_time_s` saem |
| `src/models/config.rs` | validação + 2 guardas de migração |
| `src/models/specs.rs` | `SCHEMA_VERSION` 5.7; `PerformanceSpec` ganha 3 campos |
| `config/aircraft/baseline_4seat.toml` | migração |
| `src/main.rs` | `fidelity.performance` |
| `docs/aircraft_spec.schema.md`, `docs/backlog.md` | contrato e backlog |

---

## Task 1 — Renomear a polar e abrir espaço na config

**Tier declarado: MECÂNICA (Haiku).** Rename provado pelo compilador; adição
de config puramente aditiva. Ao final, **todos os números do baseline
inalterados**.

**Files:**
- Modify: `src/agents/performance.rs` (`cd_ground_roll`, linha ~186, e os 6
  call sites), `src/models/aircraft_config.rs` (`PerformanceCfg`),
  `src/models/config.rs`, `config/aircraft/baseline_4seat.toml`
- Test: `src/models/config.rs` (`mod tests`)

**Interfaces:**
- Produces: `performance::cd_gear_extended(wing, state, cl, cd0_flap_extra) -> f64`;
  `PerformanceCfg::flare_load_factor: f64`.

**NÃO faça nesta task:** não remova `approach_angle_deg` nem `flare_time_s`,
não toque em `landing_distance_50ft_m`.

- [ ] **Passo 1: renomeie `cd_ground_roll` → `cd_gear_extended`**

```bash
grep -rn "cd_ground_roll" --include=*.rs src/ tests/
```

São 6 call sites em `src/` (rolagem de decolagem, rolagem de pouso, balanço de
rotação) mais docstrings. Acrescente à docstring da função:

```rust
/// `old→new` (ciclo 14, spec §2.4): esta função chamava-se `cd_ground_roll`.
/// O nome era estreito — ela nunca calculou "CD de rolagem", e sim a POLAR da
/// aeronave com TREM ESTENDIDO e um incremento de flap, avaliada num CL
/// qualquer. Até o ciclo 13 todos os consumidores eram de solo, então o nome
/// passava. O ciclo 14 cria o primeiro consumidor EM VOO — o segmento de
/// aproximação do pouso (`landing_distance_50ft_m`, spec §2.1) — e "ground
/// roll" num cálculo de segmento aéreo passaria a MENTIR.
///
/// Consumidores: rolagem de decolagem, rolagem de pouso, balanço de rotação
/// e aproximação de pouso.
```

- [ ] **Passo 2: `cargo test`** — tem que passar sem NENHUM número mudar.
      Rename puro. Se algum pin quebrou, PARE e reporte.

- [ ] **Passo 3: commit**

```bash
git add -A
git commit -m "refactor(performance): cd_ground_roll -> cd_gear_extended"
```

- [ ] **Passo 4: escreva o teste de config que falha** — em
      `tests/generic_engine.rs` (literal do baseline REAL; os testes unitários
      de `config.rs` rodam contra TOML sintético):

```rust
/// O fator de carga do flare é técnica de pilotagem + limite estrutural,
/// não uma cronometragem. Ciclo 14, spec §2.2.
#[test]
fn baseline_declara_o_fator_de_carga_do_flare() {
    let cfg = baseline_state();
    assert_eq!(cfg.performance.flare_load_factor, 1.20);
}
```

- [ ] **Passo 5: rode e confirme que falha** (campo não existe).

- [ ] **Passo 6: adicione o campo** em `PerformanceCfg`
      (`src/models/aircraft_config.rs`), junto de `flare_time_s`:

```rust
    /// Fator de carga do FLARE de pouso (ciclo 14, spec §2.2) — substitui
    /// `flare_time_s`. O flare é um arco de recolhimento de raio
    /// `R = V_ref²/(g·(n−1))`, não uma cronometragem: com `n` maior o arco é
    /// mais fechado, consome menos altura e termina antes.
    ///
    /// Estritamente > 1,0 — em `n = 1` o raio DIVERGE (voo reto, o flare
    /// nunca termina). Teto de 2,0: fator de carga de flare acima disso não é
    /// pilotagem de pouso.
    pub flare_load_factor: f64,
```

- [ ] **Passo 7: validação** em `src/models/config.rs`, junto das demais de
      `performance`:

```rust
    if !(cfg.performance.flare_load_factor > 1.0) || cfg.performance.flare_load_factor >= 2.0 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: performance.flare_load_factor = {} deve estar \
             em (1,0; 2,0) — em n = 1,0 o raio do arco de flare DIVERGE (R = V_ref²/(g(n−1))) \
             e a aeronave nunca sai da rampa; acima de 2,0 não é pilotagem de pouso",
            cfg.performance.flare_load_factor
        )));
    }
```

- [ ] **Passo 8: TOML e fixtures.**
      `config/aircraft/baseline_4seat.toml`, bloco `[performance]`:

```toml
# Fator de carga do arco de flare (ciclo 14, spec §2.2) — substitui
# flare_time_s. R = V_ref²/(g(n-1)); n=1,20 dá R=651,1 m, h_flare=2,596 m.
flare_load_factor = 1.20
```

      Fixture sintética `src/models/aircraft_config.rs` (procure
      `flare_time_s: 1.4`): use `flare_load_factor: 1.25` — **deliberadamente
      diferente do baseline**, para que um literal do baseline plantado num
      teste sintético não passe por acidente. TOML sintético embutido em
      `src/models/config.rs` (procure `flare_time_s = 1.5`):
      `flare_load_factor = 1.25`.

- [ ] **Passo 9: teste de rejeição** — a validação precisa ser falseável:

```rust
    #[test]
    fn rejeita_flare_load_factor_fora_da_faixa() {
        for (linha_antiga, linha_nova) in [
            ("flare_load_factor = 1.25", "flare_load_factor = 1.0"),
            ("flare_load_factor = 1.25", "flare_load_factor = 0.8"),
            ("flare_load_factor = 1.25", "flare_load_factor = 2.5"),
        ] {
            let toml = aircraft_toml_valido().replace(linha_antiga, linha_nova);
            let err = parse_aircraft(&toml).unwrap_err();
            assert!(err.to_string().contains("performance.flare_load_factor"), "{err}");
            assert!(err.to_string().contains("DIVERGE"),
                    "a mensagem tem que dizer POR QUE n=1 é proibido: {err}");
        }
    }
```

- [ ] **Passo 10: `cargo test`** — todos verdes, **nenhum número do baseline
      mudou**. Verifique regenerando o JSON e comparando com o commitado:

```bash
cargo run --release -- --engine config/engines/toyota_1gd_ftv.toml \
  --aircraft config/aircraft/baseline_4seat.toml \
  --mission config/missions/default.toml --out /tmp/t1.json
diff -q aircraft_spec.json /tmp/t1.json && echo "IDENTICO"
```

- [ ] **Passo 11: commit.**

---

## Task 2 — O segmento aéreo: ângulo derivado e flare com altura

**Tier declarado: JULGAMENTO (Sonnet 5).** É a task de física do ciclo.

**Files:**
- Modify: `src/agents/performance.rs` (`landing_distance_50ft_m`, linhas
  ~960–993), `src/models/aircraft_config.rs` (remove 2 campos),
  `src/models/config.rs` (remove validações, ADICIONA 2 guardas de migração),
  `config/aircraft/baseline_4seat.toml`
- Test: `src/agents/performance.rs` (`mod tests`), `tests/generic_engine.rs`

**Interfaces:**
- Consumes: `cd_gear_extended` e `flare_load_factor` (Task 1).
- Produces: `landing_distance_50ft_m` com a mesma assinatura MENOS
  `perf_cfg.approach_angle_deg`/`flare_time_s`, MAIS
  `perf_cfg.flare_load_factor`; e uma struct de diagnóstico:

```rust
/// Decomposição do segmento aéreo do pouso (ciclo 14, spec §2.3) — exposta
/// para que o JSON possa publicá-la (§6.1) e para que as guardas geométricas
/// possam medir cada parte, em vez de só o total.
#[derive(Debug, Clone, Copy)]
pub struct LandingAirSegment {
    pub approach_angle_rad: f64,
    pub flare_height_m: f64,
    pub flare_distance_m: f64,
    pub approach_distance_m: f64,
}
impl LandingAirSegment {
    pub fn total_m(&self) -> f64 { self.approach_distance_m + self.flare_distance_m }
}
```

### PRIMEIRO: a sonda documental (spec §5.1)

- [ ] **Passo 1: rode a sonda e cole a saída no relatório.**

A spec §5.1 é explícita: **não fabrique um teste RED artificial** para o
defeito do flare sem altura. Um teste de "as alturas fecham 15 m" PASSARIA
hoje (a rampa desce 15 e o flare desce 0, soma 15). A prova documental é uma
SONDA. Escreva um teste temporário que imprime, com os números do baseline:

```
s_air(3°) = 15/tan(3°) = ......  m   <- ja cobre os 15 m INTEIROS
s_flare   = V_ref * 1,5s = ......  m <- somado com altura ZERO
altura consumida pelo flare = 0,00 m
```

Rode com `--nocapture`, **cole a saída no relatório**, e APAGUE a sonda. Ela
não vira teste permanente.

### Depois: as guardas vivas, RED-FIRST contra a função nova

- [ ] **Passo 2: escreva as guardas** em `src/agents/performance.rs`,
      `mod tests`. Use a fixture SINTÉTICA (`fixture_baseline()` /
      `config_teste()`) — estes são testes RELACIONAIS, valem para qualquer
      config. Literais do baseline real vão para `tests/generic_engine.rs`.

```rust
/// FECHAMENTO GEOMÉTRICO (ciclo 14, spec §5.2) — a guarda central.
/// A rampa desce até a altura em que o flare começa, e o flare consome o
/// resto. Se alguém reintroduzir um flare sem altura, trocar o sinal, ou
/// somar em vez de subtrair, isto quebra.
#[test]
fn segmento_aereo_de_pouso_fecha_os_quinze_metros() {
    let (engine, state, wing) = fixture_baseline();
    let cfg = config_teste();
    let seg = landing_air_segment(MTOW_PIN_KG, RHO_SL, &wing, &state,
                                  cfg.performance.flare_load_factor);
    assert!(seg.flare_height_m > 0.0,
            "flare sem altura — a aeronave pousaria duas vezes (spec §1.1)");
    assert!(seg.flare_height_m < 15.0, "flare começaria acima do obstáculo");
    assert!(seg.approach_distance_m > 0.0);
    let fechamento = (15.0 - seg.flare_height_m)
                     - seg.approach_distance_m * seg.approach_angle_rad.tan();
    assert!(fechamento.abs() < 1e-9, "as alturas não fecham 15 m: {fechamento}");
}

/// Rampa mais íngreme ⟹ aproximação estritamente MAIS CURTA (spec §5.4).
/// Numerador cai (o flare consome mais altura) e denominador sobe — as duas
/// na mesma direção.
#[test]
fn rampa_mais_ingreme_encurta_a_aproximacao() { /* varra L/D via cd0 sintético */ }

/// Fator de carga maior ⟹ arco mais fechado ⟹ segmento aéreo TOTAL
/// estritamente menor (spec §4.1, medido de 1,10 a 1,30).
#[test]
fn flare_mais_apertado_encurta_o_segmento_aereo() {
    let (engine, state, wing) = fixture_baseline();
    let mut anterior = f64::INFINITY;
    for n in [1.10, 1.15, 1.20, 1.25, 1.30] {
        let seg = landing_air_segment(MTOW_PIN_KG, RHO_SL, &wing, &state, n);
        let total = seg.total_m();
        assert!(total < anterior, "n={n} não encurtou: {total} >= {anterior}");
        anterior = total;
    }
}

/// CONTRA-INTUITIVO E POR ISSO VALIOSO (spec §5.4): uma aeronave mais LIMPA
/// plana melhor, aproxima mais raso, e portanto precisa de MAIS espaço a
/// partir de 15 m. Se este teste for "consertado" para a direção intuitiva,
/// o modelo está errado.
#[test]
fn aeronave_mais_limpa_precisa_de_mais_espaco_aereo() { /* varie wing.cd0 */ }

/// O flare não pode começar acima do obstáculo (spec §5.3). Com n → 1⁺ o
/// raio DIVERGE. Resultado FÍSICO: +INFINITY, nunca s_air negativo, nunca NaN.
#[test]
fn flare_alto_demais_devolve_infinito_e_nao_numero_espurio() {
    let (engine, state, wing) = fixture_baseline();
    let d = landing_distance_50ft_m(/* ... com n = 1.001 ... */);
    assert!(d.is_infinite(), "esperado +INFINITY, veio {d}");
    assert!(!d.is_nan());
}

/// FONTE ÚNICA DE POLAR (spec §5.5): o CD da aproximação vem da MESMA função
/// que a rolagem usa. Proíbe uma segunda polar de aproximação plantada.
#[test]
fn cd_da_aproximacao_vem_da_polar_unica() {
    /* recompute CL_ref e CD_ref à mão e confira contra cd_gear_extended */
}
```

- [ ] **Passo 3: rode e confirme que falham** (função nova não existe).

- [ ] **Passo 4: implemente** em `src/agents/performance.rs`:

```rust
/// Decomposição do segmento AÉREO do pouso (ciclo 14, spec §2).
///
/// `old→new`. Até o ciclo 13 este segmento era duas heurísticas somadas:
/// `s_air = 15/tan(3°)` (o *glideslope* de ILS — aproximação COM POTÊNCIA de
/// aeroporto pavimentado, nunca calibrada para a pista de fazenda de 600 m
/// que é a premissa do projeto) e `s_flare = V_ref × 1,5 s` (cinemática pura).
/// Juntas somavam 339,82 m = **52,5% da distância de pouso em grama**.
///
/// Havia ali DOIS defeitos de naturezas diferentes (spec §1):
///  1. GEOMÉTRICO, independente de premissa: `s_air` descia os 15 m INTEIROS
///     até o solo e o flare era somado com altura ZERO — a aeronave chegava
///     ao solo duas vezes.
///  2. DE PREMISSA: 3,0° é mais raso do que esta célula desce com o motor
///     cortado (L/D 11,165 em config de pouso ⟹ 5,1181°).
///
/// Modelo novo:
///   γ_app   = atan(CD_ref/CL_ref)          [derivado da polar, sem config]
///   R       = V_ref²/(g·(n−1))             [arco de recolhimento]
///   h_flare = R·(1 − cos γ_app)            [altura CONSUMIDA pelo flare]
///   s_flare = R·sin γ_app
///   s_air   = (15 − h_flare)/tan γ_app     [a rampa desce só o que sobra]
///
/// PREMISSA DECLARADA (spec §2.1): motor em MARCHA LENTA sobre o obstáculo —
/// procedimento padrão de campo curto, e como os números de POH de pista
/// curta são medidos. Uma aproximação COM potência é mais RASA e mais LONGA:
/// se a operação real for motorizada, este modelo é OTIMISTA. Nomeado, não
/// escondido.
pub fn landing_air_segment(
    mass_kg: f64, rho: f64, wing: &WingSpec, state: &AircraftState,
    flare_load_factor: f64,
) -> LandingAirSegment {
    let w = mass_kg * G;
    let v_s = ((2.0 * w) / (rho * wing.area_m2 * wing.cl_max)).sqrt();
    let v_ref = 1.30 * v_s;

    let cl_ref = 2.0 * w / (rho * wing.area_m2 * v_ref * v_ref);
    let cd_ref = cd_gear_extended(wing, state, cl_ref, wing.cd0_flap_ldg_extra);
    let gamma = (cd_ref / cl_ref).atan();

    let r = v_ref * v_ref / (G * (flare_load_factor - 1.0));
    let h_flare = r * (1.0 - gamma.cos());
    let s_flare = r * gamma.sin();
    // Guarda falseável (spec §5.3): flare acima do obstáculo é resultado
    // FÍSICO impossível, não erro numérico. +INFINITY, nunca negativo/NaN.
    let s_approach = if h_flare < 15.0 && gamma > 0.0 {
        (15.0 - h_flare) / gamma.tan()
    } else {
        f64::INFINITY
    };
    LandingAirSegment {
        approach_angle_rad: gamma, flare_height_m: h_flare,
        flare_distance_m: s_flare, approach_distance_m: s_approach,
    }
}
```

E reescreva `landing_distance_50ft_m` para `seg.total_m() + s_ground`.

- [ ] **Passo 5: rode e confirme que passam.**

- [ ] **Passo 6: remova `approach_angle_deg` e `flare_time_s`** de
      `PerformanceCfg`, da validação, do TOML e das fixtures. Adicione as
      **duas guardas de migração** em `src/models/config.rs`, no padrão de
      `check_shaft_height_migration` (registre na lista de chamadas):

```rust
fn check_approach_angle_migration(toml_str: &str) -> Result<(), ConfigError> {
    let raw: toml::Value = toml::from_str(toml_str)?;
    if raw.get("performance").and_then(|p| p.get("approach_angle_deg")).is_some() {
        return Err(ConfigError::Validation(
            "configuração de aeronave inválida: [performance].approach_angle_deg foi REMOVIDO — \
             o ângulo de aproximação passou a ser DERIVADO da polar de pouso \
             (atan(CD_ref/CL_ref) a V_ref, planeio power-off), não mais uma constante. \
             Remova o campo (ver docs/aircraft_spec.schema.md e a spec do ciclo 14)"
                .to_string(),
        ));
    }
    Ok(())
}
// idem check_flare_time_migration, apontando para [performance].flare_load_factor
```

Escreva o teste de cada guarda, no padrão dos testes de migração existentes.

- [ ] **Passo 7: atualize o comentário de `aircraft_config.rs:305`**, que
      descreve `rotation_attitude_deg` citando "`[performance]
      rotation_time_s/flare_time_s`, tempos — não ângulos — do mesmo evento".
      Metade dessa frase morreu: `flare_time_s` não existe mais e o flare
      agora É um ângulo/arco. `old→new`.

- [ ] **Passo 8: `cargo test` e MEÇA.** Reporte, contra a projeção da spec §7:
      `ldg_approach_angle_deg`, `ldg_flare_height_m`, `ldg_air_distance_m`,
      `ldg_50ft_m`, `ldg_50ft_grass_m`, e a checagem #24.

      **CONFIRME que NADA fora do pouso se moveu** — decolagem, cruzeiro,
      subida, rotação, massa, MTOW. Se algo se moveu, é ACHADO: pare e escale.

      Reproduza também a linha "só o conserto geométrico" da spec §4.2
      (γ = 3°, flare com altura ⟹ ≈610,1 m) como medição intermediária e
      reporte-a. É a evidência de que os dois defeitos precisavam ir juntos.

- [ ] **Passo 9: commit.**

---

## Task 3 — Schema 5.7, JSON, backlog e veredito

**Tier declarado: JULGAMENTO (Sonnet 5).**

**Files:** `src/models/specs.rs`, `src/main.rs`, `aircraft_spec.json`,
`docs/aircraft_spec.schema.md`, `docs/backlog.md`, `tests/*.rs`

- [ ] **Passo 1: 3 campos novos em `PerformanceSpec`** (spec §6.1):
      `ldg_approach_angle_deg`, `ldg_flare_height_m`, `ldg_air_distance_m`.
      **Confira se algum literal `PerformanceSpec { ... }` sem
      `..Default::default()` existe** (no ciclo 13 foi `trim_sintetico()` em
      `weight_balance.rs` que quebrou assim) — `grep -rn "PerformanceSpec {"`.

- [ ] **Passo 2: `SCHEMA_VERSION` 5.6 → 5.7** e testes de schema.

- [ ] **Passo 3: regenere o JSON** com o comando de `scripts/verifica-ciclo.sh`.

- [ ] **Passo 4: pins** em `tests/generic_engine.rs`, `tests/cli.rs`,
      `tests/gear_tipback.rs`, `tests/schema_v4.rs`, cada um com `old→new`.
      **Tolerâncias inalteradas.**

- [ ] **Passo 5: `docs/aircraft_spec.schema.md`** — bloco `performance` (3
      campos), histórico v5.7 (MINOR puro, sem exceção registrada: só
      adiciona campos; `ldg_50ft_*` mudam de VALOR mas não de significado).

- [ ] **Passo 6: `docs/backlog.md`**
      - **#17 RESOLVIDO**, com a medição: os dois defeitos separados, a tabela
        §4.2 (só geométrico 610,1 m; só premissa 527,9 m; os dois 503,4 m), a
        sensibilidade a `n` (§4.1), e o gate #24 flipando FAIL→PASS.
      - **ABRA #23** — `landing_ground_roll_m` integra a partir de `V_ref`
        (35,74 m/s), mas o flare sangra velocidade até ≈1,15·V_s (31,61 m/s).
        A rolagem real é MENOR: integrar de `V_ref` é **CONSERVADOR**.
        Direção do erro nomeada, magnitude não medida.

- [ ] **Passo 7: `fidelity.performance`** em `src/main.rs`. Tem que declarar
      (a) que `γ_app` é derivado da polar, (b) a **premissa de motor em marcha
      lenta** e que uma aproximação motorizada seria mais longa (o modelo é
      OTIMISTA nesse caso), (c) que o flare agora consome altura. Varredura de
      texto morto:

```bash
grep -rni "approach_angle\|flare_time\|glideslope\|3 graus\|3,0°" src/ docs/
```

- [ ] **Passo 8: `scripts/verifica-ciclo.sh`** → "Status geral: APROVADO".

- [ ] **Passo 9: relatório final** com a tabela `old→new` COMPLETA de todo
      campo do JSON que mudou entre `7d246b3` e HEAD, o `validation_status`
      final, a lista de violações, e a comparação item a item contra a
      projeção da spec §7 — **incluindo o que a projeção errou.**

- [ ] **Passo 10: commit.**
