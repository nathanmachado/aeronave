# Ciclo 8 — Arrasto de Flap, Gradiente CS 23.65 e Folga Crítica — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Arrasto de flap na polar (`cd0_flap_delta` × fração de deployment), gradiente CS 23.65 em configuração de decolagem consistente, check #25 de folga de hélice em condição crítica (CS 23.925: batente + pneu murcho), pin de rotação ±0,05 — schema 5.0→5.1; E10 re-validado honestamente.

**Architecture:** Spec: `docs/superpowers/specs/2026-08-08-ciclo8-flap-e-solo-design.md`. O arrasto de flap entra pela `excess_power_kw` via parâmetro explícito novo `cd0_extra` (0.0 nos usos limpos/em-rota; `to_flap_fraction × cd0_flap_delta` nos segmentos de decolagem; delta CHEIO na polar de pouso) — sem clonar WingSpec. A rolagem de solo (método de energia, sem termo de arrasto) NÃO muda.

**Tech Stack:** Rust, serde/TOML.

## Global Constraints

- Pins honestos (old→new, tolerâncias INALTERADAS — exceção ÚNICA e nomeada: §4 APERTA o pin de rotação ±1,5→±0,05, deliberado); E10 esperado continuar PASS (folga crítica estimada +0,033 m; distâncias +poucos %) — o que o run disser é o achado; PARAR e reportar se o E10 reprovar (sem mascarar).
- 2 campos novos com faixa + rejection + fixture distinta; TDD RED-first; determinismo; Português; `cargo test` verde por task; genericidade verde.

## Fatos do código atual (verificados em `23c72c6`)

- `takeoff_ground_roll_m` (performance.rs:~310): método de energia `W²/(g·ρ·S·CL_TO·T_avg)` — SEM termo de arrasto; NÃO recebe o delta (documentar no docstring por quê).
- `takeoff_distance_50ft_m` (:~380-408): segmento de subida usa `excess_power_kw(v_climb, ...)` — a polar entra AQUI; ganha `cd0_extra = to_flap_fraction × cd0_flap_delta`.
- `excess_power_kw` (assinatura atual com ~12 params): ganha `cd0_extra: f64` no fim; TODOS os call sites explícitos (0.0 nos limpos: Vy/`climb_rate_ms`, teto, cruzeiro etc. — auditar um a um no report com veredito "configuração limpa? sim/não").
- `best_climb_angle_ms` (:~207) + `climb_rate_ms` (:~147): o GRADIENTE CS 23.65 (§2) migra para `cl_max_to` (referência de estol) + `cd0_extra` parcial; **Vy/`climb_rate_ms` (subida em rota) FICA como está** — híbrido conhecido, fora de escopo (limitação documentada; NÃO tocar).
- `landing_distance_50ft_m` (:~431): auditar os segmentos — onde a polar de pouso entra (aproximação/planeio), somar o delta CHEIO; rolagem de frenagem não usa polar.
- `[wing]` em aircraft_config.rs (WingCfg) — campo novo `cd0_flap_delta`; `[gear]` (GearCfg) — `tire_deflation_delta_m`.
- Folga: `PropellerAgent` computa `ground_clearance_m` (estática); curso do nariz COMPUTADO: `GearSpec.nose_oleo_stroke_mm` (conferir nome exato no landing_gear.rs — o diagnóstico do ciclo 3 lia `nose_oleo_stroke_mm` do JSON). Check #25 no `ConstraintChecker::verify` (via `VerifyInputs` — `propeller` e `gear` já presentes): `propeller.ground_clearance_m − (gear.nose_oleo_stroke_mm/1000 + gear_cfg.tire_deflation_delta_m) > 0.0`, violação nomeando os três números. Expor `prop_clearance_critical_m` em `PropellerSpec`? NÃO — a folga crítica depende do gear (computado depois da hélice); expor no bloco `landing_gear` do JSON (`GearSpec.prop_clearance_critical_m`, computado no LandingGearAgent que já recebe... NÃO recebe a hélice. DECISÃO: computar no `ConstraintChecker` é volátil demais para serializar; expor via `main.rs` no bloco `propeller` do relatório montando o valor com os dois specs prontos — campo `prop_clearance_critical_m` em `PropellerSpec` preenchido em main.rs pós-gear (Option<f64>? NÃO — preencher sempre; PropellerAgent seta NaN? NUNCA NaN no JSON (lição ciclo 5). SOLUÇÃO LIMPA: `PropellerSpec.prop_clearance_critical_m: f64` preenchido por um método `PropellerSpec::with_critical_clearance(gear, gear_cfg)` chamado em main.rs após o LandingGearAgent; fixture/testes usam o mesmo caminho.)
- Pin de rotação atual (pós-E10): conferir o valor corrente em trim_authority/gear_tipback (≈8,53?) e centrar ±0,05.
- `SCHEMA_VERSION = "5.0"`; robustez massa-total re-avalia distâncias/gradiente automaticamente (gates existentes) — a folga crítica é mass-sensível via curso computado: adicionar gate massa-total do #25 no `RobustnessAgent` (mesmo padrão nom_ok && !p_ok, usando o gear perturbado que o caso massa-total já computa).

---

### Task 1: §1 + §2 — arrasto de flap na polar e gradiente honesto

**Files:**
- Modify: `src/models/aircraft_config.rs` (`WingCfg.cd0_flap_delta` + fixture 0.020), `src/models/config.rs` (faixa (0.005, 0.05) + rejection + string TOML), `config/aircraft/baseline_4seat.toml` (0.015 comentado — slotted moderado, Raymer cap. 12)
- Modify: `src/agents/performance.rs` (`excess_power_kw` + `cd0_extra`; takeoff 50ft; gradiente com `cl_max_to` + delta parcial; landing 50ft com delta cheio; auditoria de call sites; teste de monotonicidade reescrito)
- Modify: pins em cascata (golden honesto)

**Interfaces:**
- Produces: `cfg.wing.cd0_flap_delta`; `excess_power_kw(..., cd0_extra: f64)`.

- [ ] **Step 1 (RED):** rejection test do campo; hand-check: `excess_power_kw` com `cd0_extra = 0.01` estritamente menor que com 0.0 (mesmos demais args); gradiente CS 23.65 cai com delta maior (property estrita); reescrita do teste de monotonicidade do flap: com o delta REAL da fixture, medir a direção líquida de `to_flap_fraction` 0.3 vs 0.7 na decolagem 50 ft e pinar o RESULTADO (não uma lei) com comentário do trade-off.
- [ ] **Step 2:** confirmar RED.
- [ ] **Step 3:** implementar (campo → excess_power_kw + call sites auditados → takeoff/gradiente/pouso). Docstrings: rolagem sem arrasto (por construção do método de energia); Vy em-rota fica híbrido (fora de escopo, apontar ciclo futuro).
- [ ] **Step 4:** rodar baseline E10 real; golden honesto (distâncias +poucos %, gradiente cai; PASS esperado — PARAR se reprovar); pins.
- [ ] **Step 5:** `cargo test` + commit `feat(aero): arrasto de flap na polar (cd0_flap_delta) e gradiente CS 23.65 em configuração de decolagem`.

---

### Task 2: §3 + §4 — folga crítica CS 23.925 (check #25) e pin de rotação

**Files:**
- Modify: `src/models/aircraft_config.rs` (`GearCfg.tire_deflation_delta_m` + fixture), `src/models/config.rs` (faixa (0.03, 0.15) + rejection + string TOML), `config/aircraft/baseline_4seat.toml` (0.08 comentado — deflexão total pneu 5.00-5)
- Modify: `src/models/specs.rs` (`PropellerSpec.prop_clearance_critical_m` + método de preenchimento), `src/main.rs` (preenche pós-gear + print), `src/validation/constraint_checker.rs` (check #25 + testes isolados dois ramos), `src/validation/robustness.rs` (gate massa-total do #25), `tests/gear_tipback.rs`/`trim_authority` (pin de rotação ±1,5→±0,05 centrado no valor corrente — exceção de aperto NOMEADA no comentário)
- Modify: fixture do checker (preenche o campo novo no mesmo caminho do main)

**Interfaces:**
- Consumes: `propeller.ground_clearance_m`, `gear.nose_oleo_stroke_mm` (conferir nome), `gear_cfg.tire_deflation_delta_m`.
- Produces: check #25; `PropellerSpec.prop_clearance_critical_m`.

- [ ] **Step 1 (RED):** rejection; property `tire_deflation_delta_m` maior ⟹ folga crítica menor; check #25 dois ramos (violação com folga crítica ≤ 0 via override sintético; sem violação na fixture); gate massa-total (fixture marginal flipa).
- [ ] **Step 2:** confirmar RED.
- [ ] **Step 3:** implementar (mensagem de violação nomeando estática, curso e deflexão: `"Hélice (condição crítica CS 23.925): folga estática {:.3} m − curso do nariz {:.3} m − pneu murcho {:.3} m = {:.3} m ≤ 0"`).
- [ ] **Step 4:** rodar baseline; esperado folga crítica ≈ +0,033 m → PASS; golden honesto; PARAR se reprovar.
- [ ] **Step 5:** `cargo test` + commit `feat(validation): check #25 — folga de hélice em condição crítica (CS 23.925) + pin de rotação apertado`.

---

### Task 3: Schema 5.1 + regen + re-validação E10 + relatório

**Files:**
- Modify: `src/models/specs.rs` (`SCHEMA_VERSION = "5.1"` + histórico), `docs/aircraft_spec.schema.md` (5.1: campo novo + consequências §1-§2 + check #25), `tests/schema_v4.rs`/`acceptance.rs` (pins 5.0→5.1 + assert do campo novo), `aircraft_spec.json` (regenerado — o golden-file test do E10 força coerência)

- [ ] **Step 1 (RED):** pins 5.1 + assert `propeller.prop_clearance_critical_m` numérico (esperado ≈0,033).
- [ ] **Step 2:** implementar bump + doc + regen (comando padrão).
- [ ] **Step 3:** `cargo test` completo — E10 re-validado: status final + todas as margens novas registradas no report (tabela old→new: distâncias, gradiente, folga crítica, flips).
- [ ] **Step 4:** commit `feat(schema): v5.1 — folga crítica CS 23.925 e polar com flap; E10 re-validado`.
