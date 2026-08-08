# Ciclo 7 — CLmax de Decolagem Consistente (Vr e Distâncias) — Design

**Data:** 2026-08-08 · **Baseline de partida:** pós-ciclo-6 (`f55b4da`, FAIL honesto com 4 violações, schema 4.8, 24 checks)

## Problema

A campanha E10 (flap slotted, 30 runs em scratchpad) expôs uma inconsistência interna que domina o veredito:

- **Rotação (trim_authority):** `Vr = 1,1·Vs0(W)` com `Vs0 = √(2W/(ρ·S·CL_max_flaps))` — o CLmax **de POUSO** — enquanto o momento de flap usa a fração PARCIAL de decolagem (`to_flap_cm_fraction`). Com flap 1,72 era inofensivo; com slotted 2,2 a Vr modelada fica 13% lenta demais (q −24%) e o limite dianteiro de rotação explode (13,0→18,6–27,7% MAC) — artefato, não física.
- **Decolagem (performance):** `v_s_to`, `v_lo`, `v_climb` e `cl_to = 0,80·cl_max` usam o MESMO CLmax de pouso — distâncias de decolagem ficam OTIMISTAS para flap grande (o espelho do pessimismo da rotação).

Ninguém decola com flap de pouso. O modelo precisa de um CLmax de DECOLAGEM consistente com a fração de flap que o próprio modelo já usa para o momento.

## Design (aprovado pelo usuário, 2026-08-08)

### §1 — Fração única de flap de decolagem

`[stability].to_flap_cm_fraction` → **renomeado** `to_flap_fraction` (erro de migração citando o renome e o novo papel duplo). Semântica: fração de DEPLOYMENT do flap no setting de decolagem, aplicada a AMBOS os efeitos do flap — ΔCm (como hoje, em `trim_authority`) e ΔCL (novo). Faixa/validação e valores atuais (baseline 0,5; fixture) preservados.

### §2 — `cl_max_to` derivado

- Fórmula: `cl_max_to = cl_max_clean + to_flap_fraction × (cl_max_flaps − cl_max_clean)` — interpolação linear de deployment, consistente com o tratamento do Cm.
- Computado no `AerodynamicsAgent`, exposto em `WingSpec.cl_max_to` (novo campo; serializado → schema 4.8→4.9).
- **Consumidores corrigidos:**
  - `trim_authority`: TODAS as ocorrências de Vs0/Vr da ROTAÇÃO passam a usar `cl_max_to` (o flare/pouso continua com `cl_max_flaps`); docstrings da invariância ao peso re-derivadas (a estrutura da prova não muda — `cl_max_to` também é constante em W).
  - `performance`: `takeoff_ground_roll_m` (`cl_to = 0,80·cl_max_to`), `takeoff_distance_50ft_m` (`v_s_to`, `v_lo`, `v_climb` de `cl_max_to`) e o legado `takeoff_distance_m`. POUSO (`landing_distance_m`, `landing_distance_50ft_m`) e VS0/estol de referência (checks CS-23, VS1 limpo) INTOCADOS — auditoria call site a call site no plano.
- Property test: `cl_max_clean < cl_max_to < cl_max_flaps` (com fração em (0,1)); direção: `to_flap_fraction` maior ⟹ rotação mais lenta (limite dianteiro sobe) E decolagem mais curta — o trade-off físico que o campo agora carrega.

### §3 — Consequências esperadas (verificar, não forçar)

- Baseline (flap 1,72, fração 0,5): `cl_max_to` ≈ 1,55 + 0,5×(1,72−1,55) ≈ 1,635 → Vr ~+2,6% → limite de rotação CAI ~1–2 pp → as violações de Solo (9,1%) e 2 pax (12,5%) contra 13,0% podem ENCOLHER ou FECHAR; decolagem grama sobe ~5–8% (428→~450–460, segue < 600). O que vier é golden update honesto — inclusive se o número de violações do baseline MUDAR.
- Campanha E10 re-rodada com o modelo consistente: expectativa de janela robusta com flap slotted + pacote E9 + D1,76 — o que o modelo disser.

## Sequência do ciclo

1. Implementação (SDD em worktree, ~3 tasks): renome+derivação+consumidores+golden; schema 4.9+regen; re-campanha E10 (scratchpad) + relatório.
2. O relatório E10 alimenta a decisão de adoção (usuário) — fora do ciclo.

## Tratamento de erros / Testes

- Migração do campo renomeado; faixa preservada; TDD RED-first (hand-check da fórmula, property tests, pins honestos em cascata); genericidade verde.

## Fora de escopo

- Curva CL×deflexão não-linear de flap; efeito de solo; adoção E10.
