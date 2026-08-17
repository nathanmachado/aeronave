# Ciclo 16 — o veredito que admite não saber — Plano de Implementação

> **Para trabalhadores agênticos:** SUB-SKILL OBRIGATÓRIA: use
> `superpowers:subagent-driven-development` para implementar este plano
> task a task. Os passos usam checkbox (`- [ ]`).

**Goal:** fazer o modelo publicar INDETERMINADO — com o breakeven medido ao
lado — em todo check cujo veredito vira dentro da banda de incerteza declarada
de `propeller.fom_static`, em vez de carimbar FAIL com quinze dígitos apoiados
no único parâmetro da lei de tração que nunca foi calibrado.

**Architecture:** o pipeline vira função de biblioteca (`pipeline::executa`);
cada restrição avaliada ganha identidade estável; um módulo novo
(`validation::incerteza`) re-executa o pipeline nos extremos da banda e no teto
de quantidade de movimento, pareia os checks por identidade, classifica cada um
em PASSA/FALHA/INDETERMINADO e bisseca o breakeven dos indeterminados,
publicando-o como INTERVALO medido.

**Tech Stack:** Rust 2021, serde/serde_json, clap. Sem dependência nova.

**Spec:** `docs/superpowers/specs/2026-08-16-ciclo16-veredito-indeterminado-design.md`

---

## Global Constraints

Valem para TODAS as tasks. Copiadas literalmente da spec.

1. **Nenhuma mudança de física.** `fom_static` continua `0.75`, a lei
   `FigureOfMerit::at` continua idêntica, nenhum agente muda de fórmula.
2. **Invariante das tasks 1, 2 e 3:** `aircraft_spec.json` e o **stdout do
   binário** byte-idênticos ao commit base `bfd4921`. Sem exceção.
3. **Invariante da task 5 (final):** o diff de `aircraft_spec.json` contra
   `bfd4921` é EXATAMENTE (a) `schema_version` e `revision` `"5.7"`→`"5.8"`,
   (b) o bloco novo `uncertainty`, (c) o texto da violação da CS 23.65.
   Todo o resto byte-idêntico.
4. **Tolerâncias INALTERADAS.** Nenhum `assert` afrouxa sua tolerância. Se um
   pin precisar mudar de valor, ele muda com comentário `old→new` e a
   tolerância fica onde estava.
5. **Nunca mascarar achado.** Se algo não fecha, reporte no relatório da task;
   não ajuste o teste para passar.
6. **Toda lista publicada tem que ser a SAÍDA da regra que a descreve, nunca
   uma paráfrase dela** (backlog #29). Onde este plano precisaria de uma lista
   longa derivada de código, ele dá a REGRA e o método de prova, não a lista.
7. `cargo test` inteiro verde e `scripts/verifica-ciclo.sh` APROVADO ao fim de
   cada task.
8. Mensagens de commit multi-linha vão em arquivo no scratchpad, usado com
   `git commit -q -F <arquivo>` — heredoc é bloqueado pela guarda de worktree.

### Vocabulário fixo (use exatamente estes nomes)

| conceito | identificador |
|---|---|
| função do pipeline | `pipeline::executa` |
| saída do pipeline | `pipeline::Resultado` |
| violação com identidade | `validation::constraint_checker::Violacao` |
| portão global com identidade | `pipeline::Portao` |
| módulo da banda | `validation::incerteza` |
| estado de veredito | `validation::incerteza::Veredito` |
| bloco JSON | chave de topo `uncertainty`, struct `UncertaintySpec` |
| campo novo de config | `[propeller].fom_static_tol_pct` |

---

## Estrutura de arquivos

| arquivo | responsabilidade | task |
|---|---|---|
| `src/pipeline.rs` | **novo.** `executa()` + `Resultado` + `Portao` | 1 |
| `src/lib.rs` | `pub mod pipeline;` | 1 |
| `src/main.rs` | carregar → `executa` → imprimir → serializar | 1, 2, 5 |
| `tests/schema_v4.rs` | passa a chamar `pipeline::executa` | 1 |
| `src/validation/constraint_checker.rs` | `Violacao { id, texto }` nos 25 sítios | 2 |
| `src/models/aircraft_config.rs` | campo `fom_static_tol_pct`, banda efetiva | 3 |
| `src/models/config.rs` | validações novas + fixture | 3 |
| `config/aircraft/baseline_4seat.toml` | declara a tolerância | 3 |
| `src/validation/incerteza.rs` | **novo.** varredura, classificação, bisseção | 4 |
| `src/validation/mod.rs` | `pub mod incerteza;` | 4 |
| `src/models/specs.rs` | `UncertaintySpec`, `SCHEMA_VERSION`, campo em `AircraftReport` | 5 |
| `docs/aircraft_spec.schema.md` | bloco novo, domínio de `validation_status`, §1 | 5 |
| `tests/pins_vs_json.rs` | pisos `MINIMO_DE_*` atualizados | 5 |
| `docs/backlog.md` | #21 resolvido, itens novos | 5 |

---

## Task 1: o pipeline vira função

**Roteamento:** operacional mecânico. A correção é PROVÁVEL por script
(`verifica-ciclo.sh` + diff de stdout), que é o critério de roteamento do
projeto.

**Files:**
- Create: `src/pipeline.rs`
- Modify: `src/lib.rs`, `src/main.rs`, `tests/schema_v4.rs`
- Test: `tests/pipeline_extracao.rs` (novo)

**Interfaces:**
- Produz: `aeronave::pipeline::{executa, Resultado, Portao, PipelineError}`
- Consumido por: Task 2 (adiciona ids), Task 4 (chama em laço).

### Por que esta task existe

Para varrer a banda é preciso executar o pipeline várias vezes com config
diferente. Hoje ele são 1.078 linhas de `main.rs` com cálculo e impressão
interpolados — não há o que chamar. Dividendo: `tests/schema_v4.rs:103-150`
**reimplementa** o pipeline e decide o veredito por `all_satisfied()` sozinho,
enquanto `main.rs:661` faz o AND de 9 portões. São duas definições de veredito
global que coincidem por acaso; o teste que deveria vigiar o pipeline mantém
uma cópia divergente dele.

### Como derivar os campos de `Resultado` (NÃO use uma lista minha)

Este plano **não** lista os campos. Uma lista minha seria uma paráfrase da
regra, e é exatamente o erro que o backlog #29 registra. Use a regra:

> Um campo de `Resultado` é cada binding de `fn main` que (a) é produzido por
> uma chamada de agente, de `size_aircraft`, ou por construção de spec
> (`GeometrySpec`, `SizingReport`, o mapa `fidelity`), **e** (b) é lido depois
> do ponto onde a extração termina.

O compilador prova (b) para você: mova o cálculo, e cada `println!` órfão vira
erro de compilação até que o campo exista em `Resultado`.

**Fica FORA da extração** (é diagnóstico de terminal, não relatório):
- o bloco `[ VARREDURA INFORMATIVA ]` de posição do trem (`main.rs:566-595`),
  que já clona e muta a config por conta própria;
- o bloco `[ ECONOMIA ]`, que depende de `cli.fuel_price_brl`/`avgas_price_brl`.

- [ ] **Passo 1: criar `src/pipeline.rs` com o esqueleto e registrar no lib**

```rust
//! O pipeline completo como FUNÇÃO — ciclo 16, spec §5.3.
//!
//! Existe porque `validation::incerteza` precisa re-executar o pipeline
//! inteiro com `[propeller].fom_static` alterado, e antes deste ciclo não
//! havia nada para chamar: o pipeline eram 1.078 linhas dentro de `main`.
//!
//! REGRA DE OURO: esta função NÃO faz varredura de banda. Ela é o que a
//! varredura chama. Chamar a varredura daqui é recursão infinita.

use crate::models::aircraft_config::AircraftConfig;
use crate::models::specs::*;
use crate::models::config::Requirements;
use crate::validation::constraint_checker::ConstraintReport;

/// Um dos portões do veredito global (`main.rs:641-663` antes do ciclo 16).
#[derive(Debug, Clone)]
pub struct Portao {
    /// Identidade estável — NÃO contém número que dependa da config.
    pub id: &'static str,
    pub ok: bool,
    /// Rótulo humano, pode conter números de requisito.
    pub rotulo: String,
}

#[derive(Debug)]
pub enum PipelineError {
    Sizing(String),
}

impl std::fmt::Display for PipelineError { /* mensagem em português */ }

pub struct Resultado {
    /* campos derivados pela REGRA acima */
    pub constraints: ConstraintReport,
    pub portoes: Vec<Portao>,
}

pub fn executa(
    cfg: &AircraftConfig,
    engine: &EngineSpec,
    req: &Requirements,
) -> Result<Resultado, PipelineError> { todo!() }
```

Em `src/lib.rs`, junto dos outros módulos: `pub mod pipeline;`

- [ ] **Passo 2: mover o cálculo, deixando os `println!` onde estão**

Recorte de `main.rs` para `pipeline::executa`, **na mesma ordem**, cada
binding de cálculo. Em `main`, cada uso vira `res.<campo>`. Não reordene
nada: a ordem das chamadas é observável através do laço de ponto fixo.

`size_aircraft` hoje faz `unwrap_or_else(|e| { eprintln!(…); exit(1) })` em
`main.rs:107`. Dentro de `executa` isso vira `.map_err(PipelineError::Sizing)?`;
`main` faz o `eprintln!`+`exit(1)` a partir do `Err`, com a **mesma mensagem
de antes** (o texto do stderr é observável e o invariante cobre stdout —
mantenha o stderr igual também).

- [ ] **Passo 3: mover os 9 portões para `executa`, com id**

Os ids, na ordem de `main.rs:641-660`:

```
"restricoes"     "v_cruzeiro"   "autonomia_bloco"   "rc_sl"    "teto_servico"
"flutter"        "antitombamento"   "estabilidade_long"   "envelope_cg_todos"
```

O `rotulo` é o mesmo `String` de hoje, `format!` incluído — o stdout tem que
sair idêntico. `main` imprime iterando `res.portoes` com o mesmo `if *ok { "✓" }
else { "✗" }`.

- [ ] **Passo 4: capturar a linha de base ANTES de comparar**

```bash
git stash
cargo build --release --quiet
./target/release/aeronave --out /tmp/base16.json > /tmp/base16.out 2> /tmp/base16.err
git stash pop
```

- [ ] **Passo 5: provar os dois invariantes**

```bash
cargo build --release --quiet
./target/release/aeronave --out /tmp/novo16.json > /tmp/novo16.out 2> /tmp/novo16.err
diff /tmp/base16.json /tmp/novo16.json && echo "JSON IDENTICO"
diff /tmp/base16.out  /tmp/novo16.out  && echo "STDOUT IDENTICO"
diff /tmp/base16.err  /tmp/novo16.err  && echo "STDERR IDENTICO"
```

Esperado: as três linhas. Qualquer diff é falha da task — **não ajuste o
esperado**.

- [ ] **Passo 6: teste versionado do invariante**

`tests/pipeline_extracao.rs`:

```rust
//! Ciclo 16, Task 1 — o pipeline extraído produz o MESMO relatório que o
//! artefato commitado. Não é redundante com `tests/cli.rs:943`: aquele roda
//! o BINÁRIO; este chama `pipeline::executa` direto, e é o que garante que a
//! função extraída (a que `validation::incerteza` vai chamar em laço) não
//! divergiu do caminho que gera o artefato.

use aeronave::models::config::{load_aircraft, load_engine, load_mission};

#[test]
fn pipeline_executa_reproduz_o_artefato_commitado() {
    let cfg = load_aircraft("config/aircraft/baseline_4seat.toml").unwrap();
    let engine = load_engine("config/engines/default.toml").unwrap();
    let req = load_mission("config/missions/default.toml").unwrap();

    let res = aeronave::pipeline::executa(&cfg, &engine, &req)
        .expect("pipeline do baseline tem que convergir");

    let commitado: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string("aircraft_spec.json").unwrap()).unwrap();

    // Ancoras suficientes para pegar divergência de caminho, com tolerância
    // ZERO — os dois lados vêm do mesmo pipeline determinístico.
    assert_eq!(res.perf.climb_gradient_pct,
               commitado["performance"]["climb_gradient_pct"].as_f64().unwrap());
    assert_eq!(res.constraints.violations.len(),
               commitado["violations"].as_array().unwrap().len());
    assert_eq!(res.portoes.len(), 9);
}
```

- [ ] **Passo 7: `tests/schema_v4.rs` para de reimplementar o pipeline**

Substituir a reconstrução de `schema_v4.rs:103-150` por uma chamada a
`pipeline::executa`, e o `let all_ok = report.all_satisfied();` da linha 109
pela **mesma** regra de veredito de `main` (AND dos 9 portões). Comentário
obrigatório no local, citando que eram duas regras divergentes que coincidiam
por acaso.

- [ ] **Passo 8: suíte e portão**

```bash
cargo test 2>&1 | tail -20
bash scripts/verifica-ciclo.sh
```

- [ ] **Passo 9: commit**

```
refactor(pipeline): extrai o pipeline de main.rs para pipeline::executa
```
Corpo: por que (a varredura da banda precisa chamá-lo), o dividendo (some a
segunda regra de veredito de `schema_v4.rs`), e as três provas de invariante.

---

## Task 2: identidade estável de check

**Roteamento:** operacional mecânico. Provável por script (invariante
byte-idêntico) mais um teste de unicidade.

**Files:**
- Modify: `src/validation/constraint_checker.rs`, `src/main.rs`, e todo sítio
  que lê `report.violations`
- Test: no próprio `constraint_checker.rs` (`mod tests`)

**Interfaces:**
- Produz: `Violacao { id: String, texto: String }`,
  `ConstraintReport::textos() -> Vec<String>`
- Consumido por: Task 4 (pareia checks entre corridas pelo `id`).

### Por que `String` e não `&'static str`

Checks parametrizados por cenário precisam do nome do cenário no id, e o nome
vem da config. `"envelope_cg::Solo (piloto)"` é estável sob variação de
`fom_static` (é texto de config), que é a única estabilidade exigida.

- [ ] **Passo 1: escrever o teste que falha primeiro**

Em `src/validation/constraint_checker.rs`, `mod tests`:

```rust
/// Ciclo 16, Task 2 — dois checks nunca compartilham `id`. Sem isso a
/// varredura de banda (`validation::incerteza`) parearia checks distintos
/// e publicaria veredito de um sobre o outro.
#[test]
fn ids_de_violacao_sao_unicos() {
    let report = /* fixture do baseline real, como em gear_tipback.rs:394 */;
    let mut vistos = std::collections::HashSet::new();
    for v in &report.violations {
        assert!(vistos.insert(v.id.clone()),
                "id duplicado: {} — a varredura de banda parearia checks distintos", v.id);
    }
}

/// `id` NÃO pode conter valor que dependa da config: é a chave de pareamento
/// entre corridas com `fom_static` diferente.
#[test]
fn ids_de_violacao_sao_estaveis_sob_fom_static() {
    // roda o pipeline em dois pontos e compara os CONJUNTOS de id dos
    // checks presentes em ambos.
}
```

- [ ] **Passo 2: rodar e ver falhar** — `cargo test ids_de_violacao` → não compila
  (o campo `id` não existe). É a falha esperada.

- [ ] **Passo 3: o tipo**

```rust
/// Uma violação com IDENTIDADE (ciclo 16, spec §5.2).
///
/// Antes deste ciclo uma restrição avaliada não era reificada: o texto
/// formatado era toda a identidade que existia. Como o texto carrega valores
/// ("7.9%" vs "7.4%"), ele não serve de chave entre duas corridas com
/// `fom_static` diferente — e sem chave não há como dizer QUAL check virou.
#[derive(Debug, Clone, PartialEq)]
pub struct Violacao {
    /// Estável sob variação de config. Nunca contém número calculado.
    pub id: String,
    pub texto: String,
}

impl ConstraintReport {
    pub fn all_satisfied(&self) -> bool { self.violations.is_empty() }
    /// Só os textos, na ordem — o que vai para o JSON.
    pub fn textos(&self) -> Vec<String> {
        self.violations.iter().map(|v| v.texto.clone()).collect()
    }
}
```

- [ ] **Passo 4: os 25 sítios**

Regra mecânica: cada `violations.push(format!(…))` vira
`violations.push(Violacao { id: …, texto: format!(…) })`, **com o `format!`
intocado**. O id sai do comentário numerado que já existe acima de cada check
(`// 12. Gradiente de subida (Task 4.7, CS 23.65)` → `"gradiente_cs2365"`).
Checks em laço sobre cenários concatenam o nome: `format!("envelope_cg::{}",
cenario.nome)`.

O id do gradiente da CS 23.65 é **`"gradiente_cs2365"`** — a Task 4 e a Task 5
dependem desse literal exato.

- [ ] **Passo 5: consertar os leitores**

`main.rs`: `violations: report.violations` → `violations: report.textos()`.
Nos testes, `v.contains(…)` → `v.texto.contains(…)`. O compilador aponta cada
sítio; não procure de cabeça.

- [ ] **Passo 6: provar o invariante** — os mesmos três `diff` do Passo 5 da
  Task 1, contra a mesma linha de base.

- [ ] **Passo 7: suíte, portão, commit**

```
feat(constraints): cada restrição avaliada ganha identidade estável
```

---

## Task 3: a banda em config

**Roteamento:** operacional mecânico.

**Files:**
- Modify: `src/models/aircraft_config.rs`, `src/models/config.rs`,
  `config/aircraft/baseline_4seat.toml`

**Interfaces:**
- Produz: `PropellerCfg::fom_static_tol_pct`, `PropellerCfg::banda() -> Banda`
- Consumido por: Task 4.

**Inventário exato** (gerado por grep, não de cabeça): existe **um único** TOML
de aeronave, `config/aircraft/baseline_4seat.toml`. As fixtures são
`src/models/config.rs:1699-1701` (string TOML) e
`src/models/aircraft_config.rs:872-874` (Rust). Os testes de validação que
mutam linha de fixture estão em `src/models/config.rs:2783-2784` e `:2800-2802`.

- [ ] **Passo 1: testes que falham primeiro**

```rust
/// Ciclo 16 — `fom_static_tol_pct` é OBRIGATÓRIO. Um TOML que não declara
/// quanto confia no próprio `fom_static` tem que falhar no carregamento, não
/// herdar um default silencioso: o default seria o modelo escolhendo sozinho
/// o quanto acredita em si mesmo.
#[test]
fn tolerancia_de_fom_static_ausente_falha_o_carregamento() { … }

/// FoM(J) tem que ser NÃO DECRESCENTE. `fom_design < fom_static` descreve uma
/// hélice que entrega FRAÇÃO MENOR da tração ideal quanto MAIS rápido voa.
/// Antes do ciclo 16 isso passava calado — `config.rs:853-869` só tinha
/// `require_positive` e o teto 1,0.
#[test]
fn fom_design_menor_que_fom_static_e_rejeitado() { … }

#[test]
fn tolerancia_fora_de_0_a_100_e_rejeitada() { … }

/// A banda efetiva do baseline é truncada em `fom_design`.
#[test]
fn banda_efetiva_do_baseline_e_truncada_em_fom_design() {
    let cfg = load_aircraft("config/aircraft/baseline_4seat.toml").unwrap();
    let b = cfg.propeller.banda();
    assert_eq!(b.lo, 0.75 * 0.90);
    assert_eq!(b.hi, cfg.propeller.fom_design);   // 0.81597699924588796
    assert!(b.truncada);
    assert!(b.motivo_truncagem.as_ref().unwrap().contains("fom_design"));
}
```

- [ ] **Passo 2: rodar e ver falhar.**

- [ ] **Passo 3: o campo e a banda**

```rust
    /// Tolerância epistêmica declarada sobre `fom_static` (%) — ciclo 16.
    ///
    /// NÃO é medição de hélice. É a declaração de quanto o projeto confia
    /// numa entrada que NUNCA foi calibrada: `fom_design` e `j_design` foram
    /// retro-derivados por ponto fixo até resíduo 0,000e0, `fom_static` é um
    /// fator de McCormick herdado com dois algarismos significativos.
    /// Obrigatório de propósito — ver spec do ciclo 16, §5.1.
    pub fom_static_tol_pct: f64,
```

```rust
/// Banda de incerteza EFETIVA de `fom_static` (ciclo 16, spec §5.1).
#[derive(Debug, Clone, PartialEq)]
pub struct Banda {
    pub nominal: f64,
    pub lo: f64,
    pub hi: f64,
    pub lo_declarado: f64,
    pub hi_declarado: f64,
    pub truncada: bool,
    pub motivo_truncagem: Option<String>,
}

impl PropellerCfg {
    /// Banda efetiva. O topo é truncado em `fom_design` (acima dele FoM(J)
    /// seria DECRESCENTE em J) e em 1,0 (teto de quantidade de movimento).
    /// O topo efetivo é, portanto, a maior tração estática que se pode alegar
    /// sem contradizer a única âncora que FOI calibrada.
    pub fn banda(&self) -> Banda {
        let lo_declarado = self.fom_static * (1.0 - self.fom_static_tol_pct / 100.0);
        let hi_declarado = self.fom_static * (1.0 + self.fom_static_tol_pct / 100.0);
        let mut hi = hi_declarado;
        let mut motivo = None;
        if self.fom_design < hi {
            hi = self.fom_design;
            motivo = Some(format!(
                "topo truncado em propeller.fom_design ({}): acima dele FoM(J) seria \
                 DECRESCENTE em J", self.fom_design));
        }
        if hi > 1.0 {
            hi = 1.0;
            motivo = Some("topo truncado em 1,0 — teto de quantidade de movimento".to_string());
        }
        Banda { nominal: self.fom_static, lo: lo_declarado, hi,
                lo_declarado, hi_declarado, truncada: motivo.is_some(),
                motivo_truncagem: motivo }
    }
}
```

- [ ] **Passo 4: validações em `validate_aircraft_config`**

Junto do bloco existente de `config.rs:853-869`:

```rust
    if cfg.propeller.fom_design < cfg.propeller.fom_static {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: propeller.fom_design = {} é MENOR que \
             propeller.fom_static = {} — FoM(J) ficaria DECRESCENTE em J, descrevendo uma \
             hélice que entrega fração MENOR da tração ideal quanto mais rápido voa \
             (ver agents::propulsion::FigureOfMerit)",
            cfg.propeller.fom_design, cfg.propeller.fom_static)));
    }
    if !(cfg.propeller.fom_static_tol_pct > 0.0 && cfg.propeller.fom_static_tol_pct < 100.0) {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: propeller.fom_static_tol_pct = {} fora de \
             (0 ; 100) — a banda de incerteza precisa ter largura positiva e o extremo \
             inferior precisa continuar positivo",
            cfg.propeller.fom_static_tol_pct)));
    }
```

- [ ] **Passo 5: TOML e fixtures**

`config/aircraft/baseline_4seat.toml`, logo após `fom_static = 0.75`, com o
comentário de proveniência da spec §5.1:

```toml
fom_static_tol_pct = 10.0
```

Fixtures: `config.rs:1699` e `aircraft_config.rs:872` ganham o campo. Escolha
para as fixtures um valor que **não** dispare truncagem (a fixture tem
`fom_static = 0.72`, `fom_design = 0.80`; `10.0` dá topo 0,792 < 0,80, sem
truncagem) — assim a fixture exercita o caminho não truncado e o baseline
exercita o truncado.

- [ ] **Passo 6: invariante** — os três `diff`. A banda ainda não é publicada,
  então o JSON e o stdout continuam byte-idênticos.

- [ ] **Passo 7: suíte, portão, commit**

```
feat(config): banda de incerteza declarada de fom_static + guarda de monotonicidade
```

---

## Task 4: a varredura

**Roteamento:** operacional de julgamento. A semântica (não monotonicidade,
bracket em vez de ponto, falha de convergência num extremo) não é provável por
script sozinho.

**Files:**
- Create: `src/validation/incerteza.rs`
- Modify: `src/validation/mod.rs`
- Test: dentro do módulo + `tests/incerteza.rs`

**Interfaces:**
- Consome: `pipeline::executa` (Task 1), `Violacao::id` (Task 2),
  `PropellerCfg::banda()` (Task 3)
- Produz: `analisa(...) -> Incerteza`, consumido pela Task 5.

- [ ] **Passo 1: tipos**

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Veredito { Passa, Falha, Indeterminado }

#[derive(Debug, Clone)]
pub struct CheckIncerto {
    pub id: String,
    pub veredito: Veredito,
    pub veredito_lo: Veredito,
    pub veredito_nominal: Veredito,
    pub veredito_hi: Veredito,
    /// `false` = falha TAMBÉM no teto de quantidade de movimento: nenhuma
    /// hélice conserta. NÃO afirma que existe hélice real capaz quando `true`
    /// — afirma apenas que a física não proíbe.
    pub alcance_de_helice: bool,
    /// Bracket MEDIDO do breakeven. `None` quando não há travessia única.
    pub breakeven: Option<(f64, f64)>,
    pub motivo: Option<String>,
}

#[derive(Debug, Clone)]
pub struct Incerteza {
    pub parametro: &'static str,   // "propeller.fom_static"
    pub banda: Banda,
    pub teto_avaliado: bool,
    pub checks: Vec<CheckIncerto>,
}
```

- [ ] **Passo 2: a classificação — teste primeiro**

```rust
/// A regra é PERTINÊNCIA IDÊNTICA NOS TRÊS PONTOS, não "virou entre os
/// extremos". Um check pode violar no nominal e em nenhum extremo (não
/// monotonicidade); a regra ingênua o daria como PASSA enquanto a corrida
/// nominal o tem na lista de violações — o modelo publicaria a violação e,
/// ao lado, a afirmação de que ela não existe.
#[test]
fn nao_monotonico_sai_indeterminado_sem_breakeven() { … }

#[test]
fn presente_nos_tres_pontos_sai_falha_determinada() { … }

#[test]
fn ausente_nos_tres_pontos_sai_passa() { … }

#[test]
fn presente_so_num_extremo_sai_indeterminado_com_breakeven() { … }
```

```rust
fn classifica(em_lo: bool, em_nominal: bool, em_hi: bool) -> Veredito {
    if em_lo == em_nominal && em_nominal == em_hi {
        if em_lo { Veredito::Falha } else { Veredito::Passa }
    } else {
        Veredito::Indeterminado
    }
}
```

- [ ] **Passo 3: a varredura**

```rust
pub fn analisa(
    cfg: &AircraftConfig,
    engine: &EngineSpec,
    req: &Requirements,
    nominal: &Resultado,
) -> Incerteza
```

1. `banda = cfg.propeller.banda()`
2. `executa` com `fom_static = banda.lo` → conjunto de ids `L`
3. `executa` com `fom_static = banda.hi` → `H`
4. `executa` com **as duas âncoras** em 1,0 → `T` (teto de quantidade de
   movimento; `fom_static` sozinho em 1,0 daria curva decrescente)
5. `N` = ids de `nominal`
6. classifica cada id de `L ∪ N ∪ H ∪ T`
7. bisseca os indeterminados com `em_lo != em_hi`

A config perturbada é `cfg.clone()` com o campo trocado — precedente exato em
`main.rs:571-575` e `validation/robustness.rs:428`.

Se um extremo devolver `Err(PipelineError::Sizing)`, isso vira `motivo` no
resultado e o veredito do ciclo é INDETERMINADO para todos os checks daquele
extremo — **nunca engolido**. Precedente:
`RobustnessSpec::mtow_masstotal_kg` documenta o `0.0` do sizing que falhou.

- [ ] **Passo 4: a bisseção, publicando bracket**

```rust
/// Bisseca `fom_static` até a largura do bracket ficar < `TOL_BREAKEVEN`,
/// e devolve o BRACKET, não o ponto médio.
///
/// Publicar um ponto com 17 dígitos e tolerância de 1e-6 seria exatamente a
/// falsa precisão que este ciclo existe para curar. O intervalo carrega a
/// própria incerteza no formato.
const TOL_BREAKEVEN: f64 = 1e-6;
const MAX_ITER_BREAKEVEN: usize = 60;
```

Invariante da bisseção: ao devolver `(a, b)`, o check pertence às violações em
exatamente um dos dois. Verificado por `debug_assert!` e pelo teste do Passo 6.

- [ ] **Passo 5: teste de integração no baseline real**

```rust
/// Ciclo 16 — o baseline tem EXATAMENTE um check indeterminado, e é o
/// gradiente da CS 23.65. Medido na spec §2.4: das quatro violações, as
/// outras três são determinadas contra o domínio físico INTEIRO.
#[test]
fn baseline_tem_exatamente_um_check_indeterminado() {
    let inc = /* analisa no baseline */;
    let indet: Vec<_> = inc.checks.iter()
        .filter(|c| c.veredito == Veredito::Indeterminado).collect();
    assert_eq!(indet.len(), 1, "esperado só o gradiente; achados: {:?}",
               indet.iter().map(|c| &c.id).collect::<Vec<_>>());
    assert_eq!(indet[0].id, "gradiente_cs2365");
}

/// CG e robustez falham TAMBÉM no teto de quantidade de movimento: nenhuma
/// hélice conserta, e mais tração PIORA as duas (o limite dianteiro é de
/// rotação e o balanço carrega −T·z_eixo).
#[test]
fn cg_e_robustez_falham_ate_no_teto() { … }
```

- [ ] **Passo 6: o teste que prova o breakeven re-rodando o modelo**

```rust
/// O breakeven publicado é VERIFICADO re-executando o pipeline nos dois lados
/// do bracket e exigindo vereditos opostos.
///
/// Sem isto o breakeven seria um PIN ESTIMADO — a terceira variante da doença
/// do #13, encontrada no ciclo 15: um número que nunca foi o valor do
/// pipeline em commit nenhum, ocupando o lugar de quem testemunharia.
#[test]
fn breakeven_publicado_e_provado_re_rodando_o_pipeline() {
    let inc = /* analisa no baseline */;
    let c = inc.checks.iter().find(|c| c.id == "gradiente_cs2365").unwrap();
    let (a, b) = c.breakeven.expect("check indeterminado tem que ter bracket");
    assert!(a < b);
    let viola_em = |fom: f64| { /* executa e diz se "gradiente_cs2365" está nas violações */ };
    assert!(viola_em(a),  "em breakeven_lo o check TEM que violar");
    assert!(!viola_em(b), "em breakeven_hi o check NÃO pode violar");
}
```

- [ ] **Passo 7: invariante** — os três `diff`. Nada publicado ainda.

- [ ] **Passo 8: suíte, portão, commit**

```
feat(incerteza): varredura da banda, classificação por check e breakeven medido
```

---

## Task 5: publicar

**Roteamento:** operacional de julgamento. Schema, documentação, pins e o
texto da violação.

**Files:**
- Modify: `src/models/specs.rs`, `src/main.rs`,
  `src/validation/constraint_checker.rs`, `docs/aircraft_spec.schema.md`,
  `tests/pins_vs_json.rs`, `docs/backlog.md`, `aircraft_spec.json`

- [ ] **Passo 1: `UncertaintySpec` em `specs.rs`**

Forma em `spec §5.7`. **O JSON da spec é ILUSTRATIVO** — o autoritativo é o
artefato regenerado. Não copie os valores de lá para teste nenhum sem
regenerar.

- [ ] **Passo 2: terceiro estado no veredito global**

```
FAIL           se existe QUALQUER check com falha DETERMINADA
INDETERMINADO  senão, se existe qualquer check indeterminado
PASS           senão
```

No baseline: três falhas determinadas ⇒ continua `"FAIL"`. **Este ciclo não
muda o veredito do projeto.** Teste dedicado com config sintética para o caso
em que só há indeterminado.

- [ ] **Passo 3: o texto da violação indeterminada**

Prefixo `INDETERMINADO — `, com a banda, o bracket e a frase final "O modelo
NÃO sustenta este veredito". A violação **continua na lista**: a contagem fica
em 4 e `.contains("Gradiente de subida")` continua verdadeiro, então
`tests/cli.rs:769` e `tests/gear_tipback.rs:670` seguem válidos por mérito —
**não afrouxe nenhum dos dois.**

Teste obrigatório: a contagem de `violations` com a banda ligada é IGUAL à
contagem com a banda colapsada. INDETERMINADO nunca remove violação.

- [ ] **Passo 4: `SCHEMA_VERSION` 5.7 → 5.8**

`specs.rs:1558`, e o pin `tests/schema_v4.rs:193`. Registrar em §1 do schema
doc, incluindo o **alargamento do domínio** de `validation_status` para três
valores e a justificativa de ter sido tratado como MINOR (spec §7).

- [ ] **Passo 5: regenerar e provar o diff exato**

```bash
cargo run --release --quiet
python3 - <<'EOF'
# compara aircraft_spec.json contra a versão de bfd4921 e exige que as
# ÚNICAS diferenças sejam: schema_version, revision, o bloco uncertainty,
# e violations[i] cujo texto começa com "INDETERMINADO — ".
EOF
```

Escreva esse script no scratchpad e cole a saída no relatório da task.
Qualquer quarta diferença é falha da task.

- [ ] **Passo 6: schema doc**

Bloco novo documentado com a tabela do padrão da casa. A linha
`docs/aircraft_spec.schema.md:1084` afirma "vazio se `validation_status ==
"PASS"`" — **essa invariante já era falsa antes deste ciclo** (os 8 portões
ad-hoc de `main.rs` reprovam sem empurrar string nenhuma, então FAIL com
`violations: []` sempre foi representável). Corrija a linha e registre o
achado no backlog; não o conserte em silêncio.

- [ ] **Passo 7: pins**

- Marcador para cada número novo citado em teste ou no schema doc.
- `tests/generic_engine.rs:2538` e `:2544`: `fom_static` **passa a ser
  publicado** (`uncertainty.nominal`) — o pin `NAO-PUBLICADO` vira vinculado.
  Se esquecer, o porteiro do ciclo 15 reprova. É para isso que ele existe.
- `MINIMO_DE_PINS_VINCULADOS` e `MINIMO_DE_NUMEROS_NO_DOC` sobem para o
  **valor medido** ao fim do ciclo, nunca "com folga".

- [ ] **Passo 8: backlog**

#21 RESOLVIDO com o registro do que foi medido. Abrir os itens da spec §11,
mais os dois achados desta task (a invariante falsa da linha 1084; a
incoerência de idioma do domínio de `validation_status`).

- [ ] **Passo 9: suíte, portão, commit**

```
feat(schema): publica a banda de incerteza e o veredito INDETERMINADO (schema 5.8)
```

---

## Auto-revisão deste plano

**Cobertura da spec.** §5.1→Task 3; §5.2→Task 2; §5.3→Task 1; §5.4→Task 4;
§5.5→Task 5 Passo 2; §5.6→Task 5 Passo 3; §5.7→Task 5 Passo 1; §5.8→Task 3
Passo 4; §6→Global Constraints 2 e 3; §7→Task 5 Passos 4 e 7; §8→distribuído
(1→T4P6, 2→T5P3, 3→T5P2, 4→T4P2, 5→T3P1, 6→T2P1, 7 e 8→T3P1, 9→T1P5/T2P6,
10→T1P7); §9→sem task, é declaração; §11→Task 5 Passo 8.

**Consistência de tipos.** `Violacao{id,texto}` (T2) é lido por `analisa` (T4)
e por `textos()` (T1/T2). `Banda` (T3) é campo de `Incerteza` (T4) e é
serializada em T5. `Portao{id,ok,rotulo}` nasce em T1 e é varrido em T4.
`"gradiente_cs2365"` é fixado em T2 Passo 4 e usado em T4 Passos 5-6.

**Lacuna conhecida e deliberada.** A Task 1 não lista os campos de
`Resultado`: dá a regra e o método de prova (o compilador). Publicar minha
lista seria a sétima ocorrência do backlog #29 num ciclo cuja spec abre
corrigindo a sexta.
