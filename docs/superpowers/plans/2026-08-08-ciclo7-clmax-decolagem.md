# Ciclo 7 — CLmax de Decolagem Consistente — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `cl_max_to` derivado (interpolação de deployment) consumido pela rotação (Vr) e pelas distâncias de decolagem; `to_flap_cm_fraction` → `to_flap_fraction` (papel duplo Cm+CL); schema 4.8→4.9; re-campanha E10 com o modelo consistente.

**Architecture:** Spec: `docs/superpowers/specs/2026-08-08-ciclo7-clmax-decolagem-design.md`. Fórmula: `cl_max_to = cl_max_clean + to_flap_fraction × (cl_max_flaps − cl_max_clean)`, computada no `AerodynamicsAgent` e exposta em `WingSpec.cl_max_to`. Pouso e VS0 (CS-23) INTOCADOS.

**Tech Stack:** Rust, serde/TOML, sem dependências novas.

## Global Constraints

- Pins honestos (old→new, tolerâncias INALTERADAS) — este ciclo MUDA o baseline (limite de rotação cai; decolagem sobe): as violações do baseline podem encolher/fechar; o que vier é o achado — investigar surpresa >5% vs §3 da spec antes de pinar; NUNCA mascarar.
- Renome com erro de migração citando o campo novo e o papel duplo; faixa/valores preservados (baseline 0,5).
- Determinismo; Português; TDD RED-first; `cargo test` verde ao fim de cada task; genericidade verde.

## Fatos do código atual (verificados em `a3c8b60`)

- `[stability].to_flap_cm_fraction = 0.5` (baseline TOML ~linha 270; `StabilityCfg.to_flap_cm_fraction` em aircraft_config.rs:~362; fixture com valor próprio); consumido em `trim_authority.rs` (rotação: fração do `cm_flap_delta`).
- `trim_authority.rs`: rotação usa `Vs0(W) = √(2W/(ρ·S_w·CL_max_flaps))` (docstring :232; código no corpo da rotação — TODAS as ocorrências de rotação migram para `cl_max_to`; flare/pouso mantém `cl_max_flaps`). Teste hand-check :777-792 reconstrói `vs0/vr` com `cl_max_flaps` — migra junto.
- `performance.rs` call sites de `wing.cl_max` (= CL com FLAP DE POUSO, ver `WingSpec::cl_max`): DECOLAGEM → migram: `:310` (`cl_to = 0.80·wing.cl_max` no ground roll), `:376-380` (`v_s_to`/`v_lo`/`v_climb` no 50 ft), e o legado `takeoff_distance_m` (:337-350). POUSO/ESTOL → ficam: `:160`, `:219` (referências de estol/potência), `:408`, `:441` (pouso), `:477`, `:894`. AUDITAR um a um no implementador: o critério é "esta velocidade descreve decolagem?" — na dúvida, reportar antes de mudar.
- `WingSpec` em specs.rs (campo `cl_max` já existe); `AerodynamicsAgent` (aerodynamics.rs) constrói o `WingSpec`.
- `SCHEMA_VERSION = "4.8"`; baseline: FAIL 4 violações (Solo 9,1% / 2 pax 12,5% vs limite 13,0; nariz 28,6%; pouso grama 605); `cl_max_clean = 1.55`? — CONFERIR no TOML (`cl_max_clean`) e usar o valor real nos hand-checks.
- Campanha E10 (scratchpad `campanha_e10.py`/`e10b`): pacote E9 (bateria 53+0,4; x_nose 1,30; h_cg 0,92; pernas 0,54/0,40; D 1,76) × flap slotted (clf 2,2 / cmf −0,35) × contra-alavancas.

---

### Task 1: Renome + `cl_max_to` + consumidores + golden update

**Files:**
- Modify: `src/models/aircraft_config.rs` (renome do campo + docstring papel duplo + fixture)
- Modify: `src/models/config.rs` (migração `to_flap_cm_fraction` + validação renomeada + string TOML de teste)
- Modify: `config/aircraft/baseline_4seat.toml` (renome comentado)
- Modify: `src/models/specs.rs` (`WingSpec.cl_max_to` + docstring — SEM bump ainda)
- Modify: `src/agents/aerodynamics.rs` (deriva `cl_max_to`)
- Modify: `src/agents/trim_authority.rs` (rotação usa `cl_max_to`; docstrings re-derivadas)
- Modify: `src/agents/performance.rs` (decolagem usa `cl_max_to`; property test novo)
- Modify: pins em cascata (`src/` + `tests/`)

**Interfaces:**
- Produces: `cfg.stability.to_flap_fraction: f64`; `WingSpec.cl_max_to: f64`.

- [ ] **Step 1: Testes (RED)**

```rust
// aerodynamics.rs (mod tests):
/// cl_max_to = clean + fração×(flaps−clean), estritamente entre os dois.
#[test]
fn cl_max_to_interpola_entre_limpo_e_pouso() {
    let cfg = config_teste();
    let state = AircraftState::from_config(&cfg);
    let req = requisitos_teste();
    let wing = AerodynamicsAgent::run(&state, &req);
    let esperado = cfg.wing.cl_max_clean
        + cfg.stability.to_flap_fraction * (cfg.wing.cl_max_flaps - cfg.wing.cl_max_clean);
    assert!((wing.cl_max_to - esperado).abs() < 1e-12);
    assert!(wing.cl_max_to > cfg.wing.cl_max_clean && wing.cl_max_to < cfg.wing.cl_max_flaps);
}

// performance.rs (mod tests) — o trade-off que o campo carrega:
/// Fração de flap de decolagem maior ⟹ decolagem 50 ft mais CURTA
/// (CL_to maior → Vs_to menor); a rotação fica mais exigente (coberto
/// no teste de trim_authority abaixo).
#[test]
fn mais_flap_de_decolagem_encurta_a_decolagem() {
    // duas cfgs: to_flap_fraction 0.3 vs 0.7 (resto igual), wing recomputada
    // via AerodynamicsAgent em cada; takeoff_distance_50ft_m estritamente menor na 0.7.
}

// trim_authority.rs (mod tests):
/// Fração maior ⟹ Vr menor ⟹ limite dianteiro de rotação SOBE (estrito).
#[test]
fn mais_flap_de_decolagem_sobe_o_limite_de_rotacao() { /* mesma técnica */ }
```

Migração em config.rs: TOML com `to_flap_cm_fraction` presente → erro citando `to_flap_fraction` e o papel duplo (padrão `check_*_migration` ANTES do parse).

- [ ] **Step 2: Confirmar RED.**

- [ ] **Step 3: Implementar** na ordem: renome (config/fixture/TOML/migração) → `WingSpec.cl_max_to` + derivação no `AerodynamicsAgent` (docstring com a fórmula e a cita da spec ciclo 7) → `trim_authority` (rotação: TODA ocorrência de `cl_max_flaps` no caminho de Vr/Vs0 da ROTAÇÃO vira `cl_max_to`, incluindo o hand-check :777-792; docstrings :13, :61, :232, :285-294 re-derivadas — a invariância a W se mantém, `cl_max_to` é constante em W) → `performance` (`:310`, `:376-380`, legado `:337-350`; call sites de pouso/estol INTOCADOS, com comentário de auditoria no report listando cada um e o veredito).

- [ ] **Step 4: Rodar o baseline real e golden update honesto.**

Run: `cargo run --release -- --engine config/engines/toyota_1gd_ftv.toml --aircraft config/aircraft/baseline_4seat.toml --mission config/missions/default.toml --out /tmp/c7_t1.json`
Expectativa (§3 da spec — verificar): limite de rotação cai ~1–2 pp (13,0→~11–12); Solo/2 pax podem fechar; decolagem grama 428→~450–460 (< 600). O NÚMERO DE VIOLAÇÕES PODE MUDAR — atualizar tests/cli.rs e afins honestamente (old→new, cada mudança explicada pela física). Investigar surpresa.

- [ ] **Step 5: `cargo test` completo + commit**

```bash
git add -A
git commit -m "feat(aero): cl_max_to consistente — rotação e decolagem com flap de DECOLAGEM (to_flap_fraction)"
```

---

### Task 2: Schema 4.9 + baseline regenerado

**Files:**
- Modify: `src/models/specs.rs` (`SCHEMA_VERSION = "4.9"` + histórico), `docs/aircraft_spec.schema.md` (histórico 4.9: `wing.cl_max_to`, renome do campo de missão? NÃO — de célula; consequências no veredito), `tests/schema_v4.rs`/`tests/acceptance.rs` (pins), `aircraft_spec.json` (regenerado)

- [ ] **Step 1: RED** (pins 4.9 + assert de `wing.cl_max_to` numérico entre clean e flaps).
- [ ] **Step 2: Implementar** bump + doc + regen (mesmo comando padrão, `--out aircraft_spec.json`).
- [ ] **Step 3: `cargo test` completo** — o JSON diff deve ser: versão + `cl_max_to` + as consequências REAIS da Task 1 (violações/limites/distâncias) — conferir coerência com o run da Task 1.
- [ ] **Step 4: Commit**

```bash
git add -A
git commit -m "feat(schema): v4.9 — wing.cl_max_to; aircraft_spec.json regenerado com o veredito pós-ciclo-7"
```

---

### Task 3: Re-campanha E10 e relatório (sem commit de código)

**Files:**
- Create: report da task (workspace SDD)

- [ ] **Step 1:** Reconstituir em /tmp o grid E10 sobre o baseline ATUAL (pacote E9: bateria 53,0 + `arm_offset_m = 0.4`; `x_nose_m` 1,30; `h_cg_ground_m` 0,92; pernas 0,54/0,40; `diameter_m` 1,76) × `cl_max_flaps` {2,1, 2,2, 2,3} × `cm_flap_delta` {−0,30, −0,35} × `to_flap_fraction` {0,5, 0,35} — 36 runs (script python no scratchpad da sessão, padrão das campanhas; NUNCA tocar o repo).
- [ ] **Step 2:** Para cada célula: status, violações, flips, pouso/decolagem grama vs 600, limites e faixa de CG, tipback, nariz, folga, autonomia. Identificar a região PASS robusto (0 violações E 0 flips) e a melhor pior-margem.
- [ ] **Step 3:** Relatório: o quadro E10 corrigido (antes: artefato dominava; agora: física), célula recomendada com margens, custos consolidados vs baseline de hoje (autonomia, MTOW, OEW) — SEM adoção (decisão humana, próximo passo).
- [ ] **Step 4:** `cargo test` completo (nada mudou) e encerrar.
