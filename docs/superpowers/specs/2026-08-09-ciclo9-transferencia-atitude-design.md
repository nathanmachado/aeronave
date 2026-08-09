# Ciclo 9 — Transferência de Atitude no Check #25 e Campanha E11 — Design

**Data:** 2026-08-09 · **Baseline de partida:** E10 (`a4788e2`, PASS, schema 5.1, 449 testes, 25 checks)

## Problema (item nº 1 do `docs/backlog.md`)

O check #25 (folga de hélice crítica, CS 23.925) modela o colapso do nariz como translação vertical 1:1 — mas a célula PIVOTA sobre o trem principal e a hélice está À FRENTE do trem de nariz: o mergulho real do plano da hélice é amplificado por `(x_main − x_prop)/(x_main − x_nose)` ≈ 1,4–1,55×. O PASS de +32,5 mm do E10 é artefato da simplificação (caveat declarado no ciclo 8 em campo/schema/fidelity). A folga real é plausivelmente **negativa** (−5 a −8 cm).

## Design (aprovado pelo usuário, 2026-08-09)

### §1 — Estação do plano da hélice

- Campo novo `[propeller].prop_plane_x_m` — posição longitudinal do disco (datum no nariz): baseline **0,20** (spinner à frente do CG do motor a 0,65 m; validar no CAD — Fase 3), faixa **(0,0, 1,0)**, fixture sintética distinta. Deve ser < `x_nose_m` < `x_main_m` (validação composta: a hélice tratora fica à frente do trem de nariz — erro claro se violado).

### §2 — Transferência de atitude no #25

- Fórmula nova: `fator_de_braco = (x_main_m − prop_plane_x_m)/(x_main_m − x_nose_m)`; `Δ_prop = (curso_do_nariz + tire_deflation_delta_m) × fator_de_braco`; `prop_clearance_critical_m = ground_clearance_m − Δ_prop`.
- `fill_critical_clearance` ganha os dados de estação (assinatura estendida com `gear_cfg`/`propeller_cfg` conforme necessário); `debug_assert` do #25 atualizado para a fórmula nova; caveat da translação 1:1 morre (histórico old→new preservado em campo/schema/fidelity); item nº 1 do backlog marcado como resolvido neste ciclo.
- Property test: `prop_plane_x_m` menor (hélice mais à frente) ⟹ folga crítica menor (fator maior) — estrito.

### §3 — Veredito esperado (verificar, não forçar)

- E10: fator = (3,66−0,20)/(3,66−1,30) = 1,4661; Δ_prop = (0,12746+0,08)×1,4661 = 0,3042 m; folga crítica = 0,2400 − 0,3042 = **−0,0642 m → FAIL honesto** (violação #25 nomeando estática, Δ e fator). O baseline volta a FAIL — golden update honesto invertendo os asserts de PASS para o FAIL nomeado (cobertura do PASS nas sintéticas). Se o run divergir >5% da estimativa, investigar antes de pinar.

### §4 — Campanha E11 (pós-implementação, scratchpad da sessão)

- Alavancas: `prop_axis_above_cg_m` 0,20→+0,07~0,12 (eixo mais alto — remontagem do motor); `diameter_m` 1,76→1,60~1,70 (custo em pista/autonomia já quantificável); `x_nose_m` 1,30→1,20~1,25 (reduz o fator); combinações. Grid completo com os 25 checks + robustez; região PASS-robusto; melhor pior-margem; SEM adoção unilateral — decisão humana (E11).

### Schema 5.1 → 5.2

- MINOR: nenhum campo renomeado/removido; `prop_clearance_critical_m` mantém nome com SEMÂNTICA CORRIGIDA (histórico explica a fórmula nova e o veredito movido); campo novo serializado? `prop_plane_x_m` é input de config (não serializado por si — conferir se o bloco propeller ecoa; se ecoar, documentar).

## Testes / Erros

- Faixa + rejection + fixture do campo novo; validação composta (< x_nose) + rejection; hand-check da fórmula com os números E10 congelados; property §2; golden honesto do FAIL (asserts nomeados); TDD RED-first; genericidade verde.

## Sequência

1. Implementação (SDD em worktree, 2 tasks: §1+§2+golden; schema 5.2+regen+relatório do veredito).
2. Campanha E11 (task 3, sem commit de código) → relatório → decisão de adoção do usuário.

## Fora de escopo

- Adoção E11 (decisão humana); pneu principal murcho (nariz governa na tratora); demais itens do backlog.
