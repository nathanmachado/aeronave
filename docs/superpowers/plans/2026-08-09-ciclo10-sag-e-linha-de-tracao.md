# Ciclo 10 — Deflexão Estática e Linha de Tração — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Corrigir o #25 para a letra do CS 23.925 (curso restante = `× (1 − static_sag_fraction)`; mains sem termo aditivo — deflexão estática já em `h_cg`) e modelar o momento da linha de tração (rotação + trim de cruzeiro); schema 5.3; E10 e célula E11 re-avaliados com o modelo completo para a decisão de adoção.

**Architecture:** Spec: `docs/superpowers/specs/2026-08-09-ciclo10-sag-e-linha-de-tracao-design.md`.

**Tech Stack:** Rust, serde/TOML; python p/ re-avaliação (scratchpad).

## Global Constraints

- Pins honestos (tolerâncias INALTERADAS); §1 é correção ANTI-conservadora honesta (E10 esperado: −0,0642→≈−0,0025, AINDA FAIL); §2 recua limites de rotação (margens encolhem) — o que os runs disserem; investigar surpresa >5% vs estimativas; NUNCA mascarar.
- Campo novo com faixa (0,15, 0,55) + rejection + fixture distinta; contrato de `h_cg_ground_m` documentado; TDD RED-first; Português; `cargo test` verde por task; genericidade verde.

## Números congelados (verificar nos runs)

- §1 E10: curso restante = 0,12746×(1−0,33) = 0,08540; Δ = (0,08540+0,08)×1,46610 = 0,24246; folga crítica = 0,2400−0,24246 = **−0,00246 m** (±0,001 no hand-check).
- §1 E11 (eixo 0,32/nariz 1,20): fator = 3,46/2,46 = 1,40650; Δ = 0,16540×1,40650 = 0,23264; estática = 0,36; crítica ≈ **+0,1274 m**.
- §2: `z_eixo` E10 = 0,92+0,20 = 1,12 m; termo de momento na rotação = T_rot × z_eixo (nariz-abaixo sobre os mains).

## Fatos do código (verificados em `4c4c4ea`)

- Fórmula atual do #25 em `PropellerSpec::fill_critical_clearance` (specs.rs ~:799-812; consumidores únicos via método — ciclo 9); `debug_assert` no checker ~:571-594; overrides sintéticos reconstruem `ground_clearance_m` pela fórmula (ajustar à nova).
- `GearCfg` ganha `static_sag_fraction`; `h_cg_ground_m` docstring em aircraft_config.rs (contrato).
- Rotação: `trim_authority.rs` — balanço de momentos sobre os mains (`rotation_fwd_limit_m` e funções vizinhas; peso-invariância documentada — o termo de tração é CONSTANTE em W (T a Vr... Vr varia com W? Vr=1,1·Vs_TO ∝ √W ⟹ T(Vr) varia ⟹ CUIDADO: a invariância a W do limite pode morrer; se morrer, é achado honesto — documentar e re-derivar a prova ou registrar a dependência; os testes de invariância existentes mudam de sentido com justificativa).
- Tração a Vr: `performance::thrust_available_n(v_ms, engine, rpm, psru, D, alt, isa, static_factor, psru_eff)` é `pub`; trim_authority NÃO depende de performance hoje — verificar ciclo de módulos (ambos em agents/, sem ciclo de crate; import direto ok). Se acoplamento ficar sujo, T estática corrigida como proxy DOCUMENTADO (decisão do implementador com justificativa no report).
- Trim de cruzeiro: `cl_h_trim_cruise(cm_ac, cl_cruise, x_cg, eta_h, s_ratio, l_h_over_mac)` em trim_authority (consumida pelo orchestrator no lag) — o Cm de tração entra como termo novo no numerador do equilíbrio (assinatura ganha `cm_thrust: f64` ou o Cm composto é somado antes — escolha limpa do implementador, documentada); `T_cruzeiro` = `prop.thrust_cruise_n` (PropulsionSpec, disponível no orchestrator).
- `SCHEMA_VERSION = "5.2"`; célula E11 p/ re-avaliação: `prop_axis_above_cg_m` 0,20→0,32; `x_nose_m` 1,30→1,20 sobre o baseline atual.

---

### Task 1: §1 — deflexão estática no #25

**Files:** aircraft_config.rs (campo + contrato de h_cg + fixture), config.rs (faixa + rejection + string TOML), baseline TOML (0,33 comentado), specs.rs (fórmula + docstring old→new), checker (debug_assert + overrides sintéticos), backlog item 6 → resolvido, golden honesto (E10 ainda FAIL: −0,0025 — cli/gear_tipback atualizam o NÚMERO da violação, não o sentido) + JSON.

- [ ] RED: rejection; hand-check congelado (−0,00246 ±0,001 com literais); property sag maior ⟹ folga MAIOR (estrito).
- [ ] Implementar; rodar baseline (FAIL por ~2,5 mm — investigar se divergir); golden + JSON; `cargo test`; commit `feat(validation): deflexão estática no #25 — CS 23.925 pela letra (curso restante; mains sem termo aditivo)`.

### Task 2: §2 — momento da linha de tração

**Files:** trim_authority.rs (termo `T_rot × z_eixo` na rotação — sinais auditados; prova de invariância a W re-derivada OU dependência registrada com testes ajustados justificadamente; termo `cm_thrust` no trim de cruzeiro), orchestrator (passa `thrust_cruise_n` ao lag do trim se necessário), performance/propulsion (imports), golden honesto (limites recuam; margens encolhem; robustez re-avalia) + JSON.

- [ ] RED: hand-check do termo (momento = T×z com literais); property z_eixo maior ⟹ limite de rotação RECUA (estrito); trim: cm_thrust ≠ 0 muda cl_h_trim na direção certa.
- [ ] Implementar; rodar baseline; golden (investigar >5%); `cargo test`; commit `feat(trim): momento da linha de tração na rotação e no trim de cruzeiro`.

### Task 3: Schema 5.3 + re-avaliação E10/E11 + relatório

**Files:** specs.rs (5.3 + histórico: semântica do #25 de novo + física nova do trim; exceção MINOR registrada padrão 5.2), schema doc, pins 5.2→5.3, JSON regen (diff só versão), report.

- [ ] RED (pins 5.3); implementar; regen; `cargo test`.
- [ ] Re-avaliação (scratchpad, sem commit): E10 atual e célula E11 (eixo 0,32/nariz 1,20) com o modelo completo — folga crítica, rotação (limites/margens com thrust-line), todos os checks + robustez; se a E11 degradar na rotação, sondar vizinhos (eixo 0,27; nariz 1,25) até mapear a fronteira. Report: tabela completa SEM adoção (decisão humana).
- [ ] Commit `feat(schema): v5.3 — sag estático e linha de tração; E10/E11 re-avaliados`.
