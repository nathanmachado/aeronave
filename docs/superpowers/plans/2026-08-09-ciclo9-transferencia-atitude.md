# Ciclo 9 — Transferência de Atitude no #25 + Campanha E11 — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Corrigir a transferência do colapso do nariz no check #25 (pivô nos mains, fator de braço `(x_main−x_prop)/(x_main−x_nose)`), com o FAIL honesto esperado do E10, schema 5.2, e a campanha E11 de resposta.

**Architecture:** Spec: `docs/superpowers/specs/2026-08-09-ciclo9-transferencia-atitude-design.md`. Campo novo `[propeller].prop_plane_x_m` (0,20); fórmula nova em `fill_critical_clearance`; caveat 1:1 morre; backlog item 1 fecha.

**Tech Stack:** Rust, serde/TOML; python p/ campanha (scratchpad).

## Global Constraints

- Pins honestos (tolerâncias INALTERADAS); baseline E10 esperado **FAIL com 1 violação nova** (#25: folga crítica ≈ −0,064 m — asserts de PASS invertem para o FAIL NOMEADO, cobertura do PASS nas sintéticas); investigar surpresa >5% vs a estimativa; NUNCA mascarar.
- Campo novo com faixa (0,0, 1,0) + validação composta `prop_plane_x_m < x_nose_m` + rejections + fixture distinta; TDD RED-first; Português; `cargo test` verde por task; genericidade verde.

## Números congelados (estimativa §3 da spec — verificar no run)

fator = (3,66−0,20)/(3,66−1,30) = 1,46610; Δ_prop = (0,12746+0,08)×1,46610 = 0,30416; folga crítica = 0,24000 − 0,30416 = **−0,06416 m**.

## Fatos do código (verificados em `8628a4d`)

- `PropellerSpec::fill_critical_clearance(&mut self, gear: &GearSpec, gear_cfg: &GearCfg)` — 8 call sites (main.rs:524, constraint_checker.rs:761, robustness.rs:642, gear_tipback.rs:314/444, schema_v4.rs:89, + 3 mod tests de specs.rs); fórmula atual `ground_clearance_m − (stroke + deflation)`; `debug_assert` do #25 em constraint_checker.rs:571-586 espelha a fórmula (atualizar JUNTO).
- `PropellerCfg` em aircraft_config.rs; validações de propeller em config.rs; caveats 1:1 vivem em: specs.rs (docstring do campo ~:762-778 e histórico ~:1156-1172), schema doc (~:559-570, ~:1001, ~:634), main.rs fidelity (~:797-813), backlog.md item 1.
- Assinatura de `fill_critical_clearance` precisa da estação: estender para receber `prop_cfg: &PropellerCfg` (tem `prop_plane_x_m`) — `gear_cfg` já dá `x_main_m`/`x_nose_m`/`tire_deflation_delta_m`.
- tests/cli.rs assere hoje PASS/0/0 (golden-file test força regenerar JSON junto); gear_tipback assere sem violação de folga.
- `SCHEMA_VERSION = "5.1"`.

---

### Task 1: §1 + §2 + golden do FAIL honesto

**Files:**
- Modify: `src/models/aircraft_config.rs` (campo + fixture), `src/models/config.rs` (faixa + composta + rejections + string TOML), `config/aircraft/baseline_4seat.toml` (0,20 comentado — validar no CAD)
- Modify: `src/models/specs.rs` (`fill_critical_clearance` com fator de braço; docstring nova old→new), `src/validation/constraint_checker.rs` (debug_assert + mensagem #25 com o fator), todos os 8 call sites, `src/main.rs` (fidelity atualizada), `docs/backlog.md` (item 1 → resolvido ciclo 9)
- Modify: golden (cli.rs PASS→FAIL nomeado; gear_tipback; JSON regenerado)

- [ ] **RED:** rejections (faixa + composta `prop_plane_x_m >= x_nose_m` rejeitado); hand-check da fórmula com os números congelados (fator 1,46610, folga −0,06416 ±0,001); property `prop_plane_x_m` menor ⟹ folga menor (estrito).
- [ ] Confirmar RED; implementar; rodar baseline (esperado FAIL, 1 violação #25 nomeando estática/Δ/fator); golden honesto completo + JSON.
- [ ] `cargo test` + commit `feat(validation): transferência de atitude no #25 — pivô nos mains; FAIL honesto do E10 (folga crítica −0,064 m)`.

### Task 2: Schema 5.2 + relatório do veredito

**Files:** `src/models/specs.rs` (5.2 + histórico: semântica corrigida do campo, veredito movido PASS→FAIL, campo de config novo), `docs/aircraft_spec.schema.md`, pins 5.1→5.2, `aircraft_spec.json` (regen — diff só versão), report (tabela old→new do veredito).

- [ ] RED (pins 5.2); implementar; regen; `cargo test`; commit `feat(schema): v5.2 — semântica corrigida da folga crítica; E10 reprova o #25 honestamente`.

### Task 3: Campanha E11 (sem commit de código)

- [ ] Grid no scratchpad da sessão sobre o baseline atual: `prop_axis_above_cg_m` {0,20, 0,27, 0,32} × `diameter_m` {1,76, 1,70, 1,60} × `x_nose_m` {1,30, 1,25, 1,20} (27 células; mutações regex; binário release do worktree). Extrair: status, violações, flips, folga crítica/estática, distâncias vs 600, tipback, nariz, autonomia, cruzeiro, margem combustível.
- [ ] Região PASS-robusto (0/0) e melhor pior-margem; se vazia, sondar combinações (ex.: eixo +0,10 com D 1,70) até mapear a fronteira; custos consolidados.
- [ ] Report: veredito por célula + recomendação SEM adoção (decisão humana E11); `cargo test` verde; repo limpo.
