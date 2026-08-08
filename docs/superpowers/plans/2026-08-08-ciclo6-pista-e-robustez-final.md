# Ciclo 6 — Requisito de Pista, Massa-Total Completo e Refactor do Verify — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Requisito de pista 600 m com checks #23/#24 (grama 15 m), gates de pista + envelope/nariz/tipback no caso massa-total do #19, refactor `verify` → `VerifyInputs`, e a avaliação quantitativa da hélice 1,78 m na célula E9 — schema 4.7→4.8.

**Architecture:** Spec: `docs/superpowers/specs/2026-08-08-ciclo6-pista-e-robustez-final-design.md`. Refactor mecânico PRIMEIRO (diff isolado), depois as mudanças de comportamento. Nenhum campo novo no JSON de saída (distâncias 50 ft já existem); requisito novo em `Requirements`/missões.

**Tech Stack:** Rust, serde/TOML, sem dependências novas.

## Global Constraints

- Pins honestos (old→new, tolerâncias INALTERADAS); baseline esperado: FAIL com as MESMAS 3 violações (checks #23/#24 passam: 428,2/540,0 vs 600); surpresa >5% ou flip → investigar.
- Requisito novo com faixa (300, 2000) + rejection test + fixture distinta (700.0); campo obrigatório de missão (TOMLs antigos falham no parse).
- Determinismo; Português; referências citadas; `cargo test` verde ao fim de cada task; genericidade verde.
- Task 1 (refactor) é PURA: zero mudança de comportamento/mensagem/pin.

## Fatos do código atual (verificados em `6eb51aa`)

- `PerformanceSpec` já tem `to_50ft_paved_m` (381,4), `to_50ft_grass_m` (428,2), `ldg_50ft_m` (540,0) — baseline atual.
- `takeoff_distance_50ft_m(mass_kg, rho, wing, state, surface_factor, engine, isa_delta_c, perf_cfg) -> f64` (performance.rs:362; grama usa `surface_factor` 1.20); `landing_distance_50ft_m(...)` (:431); `static_thrust_ideal_n(engine, engine_rpm, prop_diam_m, altitude_m, isa_delta_c, psru_efficiency)` = disco atuador `(2ρA P²)^(1/3)` (:47-53) — diâmetro via `state.prop_diameter_m`.
- `ConstraintChecker::verify` tem 15 parâmetros (req, wing, prop, mtow_kg, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg, fuel_capacity_l, robustness) — call sites: `src/main.rs` (1), `constraint_checker.rs` mod tests (~24), `tests/gear_tipback.rs` (2), `tests/schema_v4.rs` (1). Consts públicos: `NOSE_LOAD_*`, `RC_SL_MIN_MS`, `SERVICE_CEILING_MIN_M`, `GEAR_ACTUATOR_LOAD_NAME`.
- `RobustnessAgent::run(cfg, engine, req, state, wing, emp, masses, wb_nominal, gear_nominal, mission_nominal, perf_nominal)`; caso massa-total roda `size_aircraft(&cfg_p, ...)` e `PerformanceAgent::run(&sized_p.state, &sized_p.wing, &sized_p.prop, sized_p.state.mtow_kg, engine, req, &cfg_p.performance)` → `perf_p`; `sized_p.wb` é descartado hoje. Gates existentes: margem #18, VS0 #2, rc `RC_SL_MIN_MS`, v_cruise, teto — padrão `nom_ok && !p_ok` → flip caso "massa-total".
- Casos direcionais avaliam envelope/nariz/tipback via `wb_p.scenarios` vs limites nominais + `LandingGearAgent::run(wb_p.spec.mtow_kg, x_fwd_p, x_aft_p, &cfg.gear, m_p.trem_principal_kg, m_p.trem_nariz_kg)` — REUSAR a mesma função de avaliação para o mundo massa-total (extrair helper se preciso).
- `Requirements` em requirements.rs (fixture `requisitos_teste()`); missões reais: `config/missions/default.toml`, `config/missions/rotax_ferry.toml`; parse/validação de missão em `models/config.rs` (`parse_mission`/`load_mission`).
- `SCHEMA_VERSION = "4.7"`; célula E9 (validação do ciclo 5): bateria 53 + offset 0.4, x_nose 1.30, h_cg 0.92, pernas 0.54/0.40; `[propeller]` baseline: `diameter_m = 1.95` (campo existente), `ground_clearance_min_m = 0.23`, `prop_axis_above_cg_m = 0.20`.

---

### Task 1: Refactor `verify` → `VerifyInputs` (puro, zero comportamento)

**Files:**
- Modify: `src/validation/constraint_checker.rs` (struct + assinatura + mod tests)
- Modify: `src/main.rs`, `tests/gear_tipback.rs`, `tests/schema_v4.rs` (call sites)

**Interfaces:**
- Produces:

```rust
/// Entradas do veredito global (ciclo 6) — struct de parâmetros no lugar
/// dos 15 posicionais que três ciclos seguidos incharam. Todas as
/// referências vêm do pipeline convergido (mesmos valores de antes).
pub struct VerifyInputs<'a> {
    pub req: &'a Requirements,
    pub wing: &'a WingSpec,
    pub prop: &'a PropulsionSpec,
    pub mtow_kg: f64,
    pub engine: &'a EngineSpec,
    pub wb: &'a WeightBalanceOutput,
    pub propeller: &'a PropellerSpec,
    pub perf: &'a PerformanceSpec,
    pub mission: &'a MissionSpec,
    pub electrical: &'a ElectricalSpec,
    pub gear: &'a GearSpec,
    pub gear_cfg: &'a GearCfg,
    pub fuel_capacity_l: f64,
    pub robustness: &'a RobustnessSpec,
}
pub fn verify(inputs: &VerifyInputs) -> ConstraintReport
```

- [ ] **Step 1:** Criar a struct; converter o corpo de `verify` para ler de `inputs.*` (mecânico — nenhum texto de violação, fórmula ou ordem muda).
- [ ] **Step 2:** Migrar TODOS os call sites (main.rs, ~24 no mod tests — um helper local no mod tests pode construir o `VerifyInputs` a partir da tupla da fixture para evitar 24 repetições, desde que os testes continuem podendo substituir campos individuais —, gear_tipback ×2, schema_v4 ×1).
- [ ] **Step 3:** `cargo test` completo — deve passar SEM nenhuma outra mudança (zero pins, zero mensagens). Qualquer falha é erro de transcrição.
- [ ] **Step 4: Commit**

```bash
git add -A
git commit -m "refactor(validation): verify recebe VerifyInputs — 15 posicionais viram struct (zero comportamento)"
```

---

### Task 2: Requisito de pista + checks #23/#24 + sensibilidade ao diâmetro

**Files:**
- Modify: `src/models/requirements.rs` (campo + fixture), `src/models/config.rs` (faixa + rejection + string TOML de missão de teste)
- Modify: `config/missions/default.toml`, `config/missions/rotax_ferry.toml`
- Modify: `src/validation/constraint_checker.rs` (checks #23/#24 + testes)
- Modify: `src/agents/performance.rs` (property tests de diâmetro)

**Interfaces:**
- Consumes: `VerifyInputs` (Task 1); `perf.to_50ft_grass_m`, `perf.ldg_50ft_m`.
- Produces: `req.runway_available_m: f64`; violações #23/#24.

- [ ] **Step 1: Testes (RED)**

```rust
// requirements.rs docstring + campo (implementação no Step 3); config.rs:
// rejection: runway_available_m = 250.0 → erro "(300, 2000) (valor: ...)".

// constraint_checker.rs (mod tests, padrão dos checks vizinhos):
/// #23: pista sintética mais curta que a decolagem na grama → violação
/// nomeando distância, pista e superfície.
#[test]
fn check_23_reprova_decolagem_grama_maior_que_pista() {
    // fixture; req.runway_available_m = perf.to_50ft_grass_m - 1.0;
    // verify → violação contendo "grama" e "pista disponível".
}
/// #24: idem pouso.
#[test]
fn check_24_reprova_pouso_maior_que_pista() { /* ldg_50ft_m - 1.0 */ }
/// Fixture intacta (700 m sintéticos): nenhuma violação de pista.
#[test]
fn checks_de_pista_passam_na_fixture_intacta() { /* ... */ }

// performance.rs (mod tests) — sensibilidade ao diâmetro (garantia do
// veredito da hélice menor; a cadeia existe mas nenhum teste a protege):
/// Hélice menor ⟹ menos tração estática (disco atuador, T ∝ D^(2/3)).
#[test]
fn helice_menor_tem_menos_tracao_estatica() {
    // static_thrust_ideal_n com prop_diam 1.9 vs 1.6, resto fixo — estrito >.
}
/// Hélice menor ⟹ decolagem 50 ft mais longa (tudo o mais fixo).
#[test]
fn helice_menor_alonga_decolagem_sobre_obstaculo() {
    // takeoff_distance_50ft_m com state.prop_diameter_m reduzido — estrito >.
}
```

- [ ] **Step 2: Confirmar RED.**
- [ ] **Step 3: Implementar.**

`requirements.rs`:

```rust
/// Pista disponível (m) — grama/terra, premissa de operação do projeto
/// (decisão do cliente, 2026-08-08: 600 m, deliberadamente apertada).
/// Gates: check #23 (decolagem na GRAMA sobre 15 m) e #24 (pouso sobre
/// 15 m). Faixa válida: (300, 2000).
pub runway_available_m: f64,
```

Fixture `requisitos_teste()`: `runway_available_m: 700.0` (distinto). Faixa em `config.rs` no padrão de validação de missão; string TOML de missão de teste ganha o campo. `default.toml`: `runway_available_m = 600.0` com comentário (premissa de pista de fazenda; decisão 2026-08-08). `rotax_ferry.toml`: `runway_available_m = 800.0` (ferry entre aeródromos — comentado).

Checks no `verify`, após o #22 (comentários numerando e citando a spec do ciclo 6):

```rust
// #23 — decolagem na GRAMA sobre obstáculo de 15 m dentro da pista
// disponível (premissa fundadora: operação em pista de terra/grama).
if perf.to_50ft_grass_m > req.runway_available_m {
    violations.push(format!(
        "Decolagem (grama, 15 m): {:.0} m excede a pista disponível de {:.0} m",
        perf.to_50ft_grass_m, req.runway_available_m));
}
// #24 — pouso sobre 15 m dentro da pista disponível.
if perf.ldg_50ft_m > req.runway_available_m {
    violations.push(format!(
        "Pouso (15 m): {:.0} m excede a pista disponível de {:.0} m",
        perf.ldg_50ft_m, req.runway_available_m));
}
```

- [ ] **Step 4: `cargo test` completo** — baseline real: 428,2/540,0 ≤ 600 → sem violação nova; as MESMAS 3 violações persistem (tests/cli.rs intacto). Pins que citem contagem de checks atualizam honesto.
- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat(validation): requisito de pista 600 m — checks #23/#24 e sensibilidade ao diâmetro protegida"
```

---

### Task 3: Massa-total completo — pista + envelope/nariz/tipback no mundo +σ

**Files:**
- Modify: `src/validation/robustness.rs` (gates novos no caso massa-total + testes)

**Interfaces:**
- Consumes: `sized_p` (já computado), `perf_p` (já computado), `req.runway_available_m` (Task 2); a avaliação de cenários/gear dos casos direcionais.
- Produces: flips caso `"massa-total"` para: decolagem grama 50 ft, pouso 50 ft, cenários de CG, carga de nariz máx/mín, tipback.

- [ ] **Step 1: Testes (RED)**

```rust
/// Pista marginal no nominal flipa no massa-total (distância cresce com MTOW):
/// fixture com runway_available_m logo acima de to_50ft_grass_m nominal.
#[test]
fn decolagem_marginal_flipa_no_caso_massa_total() { /* ... */ }

/// Envelope/nariz no mundo massa-total: fixture com limite apertado que o
/// mundo +σ re-convergido cruza (σ alto na fixture) → flip nomeado; e
/// fixture folgada → nenhum flip novo (baseline real também: verificar).
#[test]
fn envelope_no_mundo_massa_total_flipa_quando_marginal() { /* ... */ }
```

- [ ] **Step 2: Confirmar RED.**
- [ ] **Step 3: Implementar** — no braço `Ok(sized_p)` do caso massa-total:
  1. Pista: mesmos gates `nom_ok && !p_ok` com (`perf_nominal.to_50ft_grass_m`, `perf_p.to_50ft_grass_m`, `req.runway_available_m`) e (`ldg_50ft_m` idem) — acrescentar à lista existente de gates de desempenho.
  2. Envelope/nariz/tipback: aplicar ao `sized_p.wb` a MESMA avaliação dos casos direcionais (extrair a função helper existente que compara `wb_p.scenarios`/gear perturbado contra limites nominais, se ainda não for reutilizável, refatorá-la para receber o "mundo" como parâmetro — sem duplicar lógica; nomes de check idênticos aos direcionais, caso "massa-total"; gear via `LandingGearAgent::run(sized_p.wb.spec.mtow_kg, x_fwd_p, x_aft_p, &cfg.gear, sized_p.structural_masses.trem_principal_kg, sized_p.structural_masses.trem_nariz_kg)` com extremos de `sized_p.wb.scenarios`).
  3. Docstring do módulo: o racional "casos direcionais são o pior caso de CG; o massa-total cobre nível" atualizado — agora TODOS os mundos avaliam tudo.

- [ ] **Step 4: `cargo test` completo** — baseline: expectativa SEM flips novos (verificar, não forçar; investigar surpresa).
- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat(robustness): mundo massa-total avalia pista, envelope, nariz e tipback (sized_p.wb deixa de ser descartado)"
```

---

### Task 4: Schema 4.8 + baseline regenerado

**Files:**
- Modify: `src/models/specs.rs` (`SCHEMA_VERSION = "4.8"` + histórico)
- Modify: `docs/aircraft_spec.schema.md` (histórico 4.8: checks #23/#24, requisito de missão novo, gates massa-total ampliados)
- Modify: `tests/schema_v4.rs`, `tests/acceptance.rs` (pins 4.7→4.8), `aircraft_spec.json` (regenerado)

- [ ] **Step 1: RED** (pins 4.8 contra binário 4.7).
- [ ] **Step 2: Implementar** bump + histórico + doc; regenerar (`cargo run --release -- --engine config/engines/toyota_1gd_ftv.toml --aircraft config/aircraft/baseline_4seat.toml --mission config/missions/default.toml --out aircraft_spec.json`).
- [ ] **Step 3: `cargo test` completo** — baseline: FAIL, MESMAS 3 violações, 0 flips, #23/#24 limpos; investigar qualquer desvio.
- [ ] **Step 4: Commit**

```bash
git add -A
git commit -m "feat(schema): v4.8 — checks de pista #23/#24 e massa-total completo; aircraft_spec.json regenerado"
```

---

### Task 5: Avaliação da hélice 1,78 m na célula E9 (sem commit de código)

**Files:**
- Create: report da task (workspace SDD)

- [ ] **Step 1:** Construir em /tmp DOIS TOMLs a partir do baseline ATUAL com as mutações E9 (bateria 53,0 + `arm_offset_m = 0.4`; `x_nose_m` 1,30; `h_cg_ground_m` 0,92; `main_strut_length_m` 0,54; `nose_strut_length_m` 0,40) e:
  - variante A: `diameter_m = 1.78` (folga derivada: 0,92+0,20−0,89 = 0,23 — cravada no limite; conferir o operador do check);
  - variante B: `diameter_m = 1.76` (folga 0,24 — margem real).
- [ ] **Step 2:** Rodar ambas (`--out /tmp/e9_d178.json` / `_d176.json`) e registrar por variante: status, TODAS as violações, flips, folga de hélice, `to_50ft_grass_m`/`ldg_50ft_m` vs 600 m, Mach de ponta estático/cruzeiro, v_cruise, rc_sl, autonomia — o custo REAL da hélice menor, quantificado pelo modelo com os checks novos.
- [ ] **Step 3:** Relatório: tabela E9-1,95 (FAIL folga) vs E9-1,78 vs E9-1,76 vs baseline; SEM recomendação unilateral — o quadro alimenta a decisão E10 do usuário (próximo passo já combinado).
- [ ] **Step 4:** `cargo test` completo (nada mudou) e encerrar o ciclo.
