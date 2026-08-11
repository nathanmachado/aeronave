# Ciclo 11 — Subida Honesta e Robustez do JSON — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Avaliar o gradiente CS 23.65 a 1,2·Vs_to (não 1,05), tornar Vy/`climb_rate_ms` consistente (estol limpo + polar limpa), serializar `+INF` como `"infinita"` em `to_50ft_*`; schema 5.4; backlog itens 2/3/5/7 fechados.

**Architecture:** Spec: `docs/superpowers/specs/2026-08-10-ciclo11-subida-honesta-design.md`.

**Tech Stack:** Rust, serde/TOML.

## Global Constraints

- Pins honestos (tolerâncias INALTERADAS — NUNCA alargar): old→new comentado com data "Campanha ciclo 11". §1 e §2 são correções que PIORAM números (anti-otimistas) — o que os runs disserem; investigar surpresa >5% vs estimativas; NUNCA mascarar. Se um gate flipar (`RC_SL_MIN_MS` 1,5 / `SERVICE_CEILING_MIN_M` 3.000 / gradiente 8,3%), o veredito muda e o achado é documentado — não tunar config para salvar PASS.
- TDD RED-first; Português; `cargo test` verde por task; genericidade verde (sem nomes de motor em `src/`).
- Baseline de partida: E12 PASS (`20192b2`+spec, schema 5.3, 466 testes). Regen JSON: `cargo run --release -- --engine config/engines/toyota_1gd_ftv.toml --aircraft config/aircraft/baseline_4seat.toml --mission config/missions/default.toml --out aircraft_spec.json`.

## Números congelados (verificar nos runs, não forçar)

- §1: `climb_gradient_pct` 13,896713 (pin generic_engine.rs:1239) → **≈12,45** (±0,2 p.p. no hand-check); `vx_kmh` 121,519501 → ≈121,5×(1,20/1,05) ≈ **138,9 km/h** (válido se o argmax continuar no piso — verificar).
- §2: razão de pisos `√(2,1/1,45) ≈ 1,20344`; SE o Vy atual (147,915721) já for o piso da varredura antiga (RC decrescente), Vy → ≈**178 km/h** e `rc_sl_ms` (4,999902) CAI; `service_ceiling_m` 5.200 pode cair (busca com resolução de 100 m). Gates com folga: RC ≥ 1,5; teto ≥ 3.000.
- §2 ALERTA de asserts relacionais: `tests/generic_engine.rs:1078-1079` afirma `Vx < Vy` e `Vy < best_glide`. Com Vy ≈178, `Vy < best_glide` pode QUEBRAR — se quebrar, é MUDANÇA DE FATO físico da polar modelada: re-derivar a expectativa e atualizar o assert com justificativa (não deletar silenciosamente).

## Fatos do código (verificados em `8a19f39`)

- `best_climb_angle_ms`: `src/agents/performance.rs:253-294`, piso na linha 269 (`let v_min = 1.05 * v_stall;`); docstring :236-252 tem o "AINDA NÃO CORRIGIDO" que morre neste ciclo.
- `climb_rate_ms`: `src/agents/performance.rs:160-200`, referência de estol na linha 173 (`wing.cl_max`); docstring :148-159 nomeia o híbrido; comentário :186-188 ("NÃO tocar a referência") fica OBSOLETO — reescrever, não deletar.
- `WingSpec::cl_max_clean` existe (`src/models/specs.rs:77`, baseline 1,45); `wing.cl_max` = flap cheio 2,1.
- Serde: `mod fatigue_life_serde` em `src/models/specs.rs:22-52` (serializa `"infinita"`, round-trip testado); campos `to_50ft_paved_m`/`to_50ft_grass_m` em specs.rs:533/535; uso precedente `#[serde(with = "fatigue_life_serde")]` em :589.
- `SCHEMA_VERSION = "5.3"` em specs.rs:1385 (política de bump na docstring).
- Consumidores de `climb_rate_ms`: `service_ceiling_m` (performance.rs:296+), orchestrator/missão (performance.rs:631-648) — cascata de pins possível em `tests/generic_engine.rs` (missão) e `tests/cli.rs`/`tests/schema_v4.rs`.
- Checks: gradiente ≥ 8,3 (`constraint_checker.rs:11,324`); `RC_SL_MIN_MS = 1.5` (:35); `SERVICE_CEILING_MIN_M = 3_000.0` (:38).

---

### Task 1: §1 — gradiente CS 23.65 a 1,2·Vs

**Files:** performance.rs (piso 1,05→1,20 + docstring old→new), pins em cascata (generic_engine.rs:1235/1239 e quaisquer outros que o run quebrar), JSON regen.

- [ ] RED: hand-check congelado (gradiente ≈12,45 ±0,2 p.p. com o pipeline real via fixture existente do teste de pins); property estrita: rodar `best_climb_angle_ms` com piso 1,20 devolve gradiente ≤ o de piso 1,05 (comparação via cópia local da varredura OU expor o piso como parâmetro interno de teste — decisão limpa do implementador, documentada).
- [ ] Implementar; rodar baseline; pins old→new ("Campanha ciclo 11, CS 23.65 a 1,2·Vs"); `cargo test`; commit `feat(performance): gradiente CS 23.65 avaliado a 1,2·Vs_to (backlog item 2)`.

### Task 2: §2 — Vy com referência de estol limpa

**Files:** performance.rs (linha 173 `wing.cl_max` → `wing.cl_max_clean` + docstrings :148-159/:186-188 reescritas old→new), pins em cascata (vy/rc_sl/ceiling/missão), asserts relacionais re-derivados se quebrarem, JSON regen.

- [ ] RED: hand-check da razão de pisos (√(2,1/1,45) ≈ 1,20344 com literais); property estrita: com RC decrescente na faixa (célula real), referência limpa ⟹ Vy NÃO diminui.
- [ ] Implementar; rodar baseline; verificar gates (RC ≥ 1,5; teto ≥ 3.000) e asserts relacionais — flips/quebras são achados honestos documentados; pins old→new; `cargo test`; commit `feat(performance): Vy/climb_rate_ms com referência de estol limpa (backlog item 3)`.

### Task 3: §3 +INF→"infinita" + §4 schema 5.4 + housekeeping + report

**Files:** specs.rs (`#[serde(with = "fatigue_life_serde")]` nos dois campos `to_50ft_*` + docstrings; renomear o módulo para nome genérico é OPCIONAL — se renomear, atualizar o uso de :589 junto), specs.rs:1385 (5.4 + histórico), schema doc (§5 estendido + histórico 5.4 + linhas de `climb_gradient_pct`/`vx_kmh`/`vy_kmh`/`rc_sl_ms`/`to_50ft_*`), pins 5.3→5.4, `docs/backlog.md` (itens 2/3/5 → RESOLVIDO com números old→new; item 7 → RESOLVIDO pela fix wave do ciclo 10, commits `a7b561a`/`2d4fff7`/`a465e7b`), JSON regen (diff: só versão vs Task 2), report.

- [ ] RED: round-trip sintético (PerformanceSpec com `to_50ft_paved_m = f64::INFINITY` serializa `"infinita"` e desserializa `INFINITY`; caso finito serializa número); pins 5.4.
- [ ] Implementar; regen (verificar: baseline finito ⟹ nenhuma mudança nos valores `to_50ft_*` do JSON real); `cargo test`; commit `feat(schema): v5.4 — subida honesta e +INF explícito em to_50ft_* (backlog itens 2/3/5/7)`.
- [ ] Report em `.superpowers/sdd/<plan>/task-3-report.md`: tabela old→new completa (gradiente, Vx, Vy, RC, teto, missão se mover) + status dos gates + o que ficou fora (item 4, termos de solo).
