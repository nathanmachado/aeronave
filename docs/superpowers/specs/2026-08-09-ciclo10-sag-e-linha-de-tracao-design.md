# Ciclo 10 — Deflexão Estática (CS 23.925 correto) e Linha de Tração — Design

**Data:** 2026-08-09 · **Baseline de partida:** E10 FAIL no #25 (`9d11835`, schema 5.2, 453 testes)

## Problema

1. **Backlog item 6 (caveat dos mains) resolve-se pela LETRA da norma, nos dois sentidos.** CS 23.925: estático = trem em deflexão ESTÁTICA; crítico = o trem CRÍTICO no batente + pneu murcho, DEMAIS em deflexão estática. Como `h_cg_ground_m` é medido com a aeronave no chão (deflexão estática inclusa — contrato a documentar), (a) NÃO existe termo aditivo dos mains (ficam onde estão) e (b) o ciclo 9 conta DUAS VEZES a compressão estática do nariz ao usar o curso inteiro — o curso restante até o batente é `curso × (1 − fração_estática)`. Correção anti-conservadora honesta.
2. **A linha de tração não gera momento no modelo.** Eixo da hélice acima do CG ⟹ momento nariz-abaixo `T × z` — ausente da rotação (o custo real do eixo +12 cm da célula E11, hoje invisível) e do trim de cruzeiro.

## Design (aprovado pelo usuário, 2026-08-09)

### §1 — Deflexão estática no #25

- Campo novo `[gear].static_sag_fraction`: baseline **0,33** (compressão estática típica de amortecedor oleo ~1/3 do curso; faixa **(0,15, 0,55)**; fixture distinta). Contrato de `h_cg_ground_m` documentado na docstring: altura do CG com a aeronave CARREGADA em deflexão estática.
- Fórmula do #25 corrigida: `Δ_prop = (curso_nariz × (1 − static_sag_fraction) + tire_deflation_delta_m) × fator` — o batente só percorre o curso RESTANTE; mains sem termo (deflexão estática já em `h_cg`). Caveat do ciclo 9 (mains rígidos) morre old→new; backlog item 6 → resolvido.
- Estimativas (verificar, não forçar): E10: −0,0642 → ≈ **−0,0025 m** (FAIL por um fio); célula E11 (eixo 0,32/nariz 1,20): ≈ **+0,127 m**.

### §2 — Momento da linha de tração

- **Rotação** (`trim_authority`): termo novo no balanço de momentos sobre os mains: `T_rotação × z_eixo`, com `z_eixo = h_cg_ground_m + prop_axis_above_cg_m` (altura do eixo sobre o solo; tração acima do pivô ⟹ nariz-abaixo ⟹ MAIS demanda de profundor). `T_rotação` = tração disponível a Vr (funções existentes de `performance`/`propulsion` — verificar acoplamento limpo no plano; se criar dependência circular, T estática corrigida como proxy documentado). Sinais/convenções auditados contra o modelo existente.
- **Trim de cruzeiro** (`trim_authority`/polar): contribuição `Cm_thrust = −T_cruzeiro × prop_axis_above_cg_m / (q·S·MAC)` no Cm de equilíbrio → `cl_h_trim` → arrasto de trim. (Em cruzeiro o braço é sobre o CG, não sobre o solo.)
- Consequência esperada: limites de rotação RECUAM (mais demanda) — margens do baseline e da E11 encolhem; a tensão folga-de-hélice × autoridade-de-rotação passa a ser capturada. Robustez massa-total re-avalia via gates existentes.

### Schema 5.2 → 5.3

- Semântica corrigida do `prop_clearance_critical_m` (de novo — histórico honesto); campo de config novo; números de trim/rotação movem. MINOR com o mesmo padrão de exceção registrada do 5.2.

### Sequência

1. Implementação (SDD, ~3 tasks: §1; §2; schema 5.3 + regen + re-avaliação da célula E11 com o modelo completo).
2. Relatório: E10 e E11 sob o modelo completo (folga com sag correto × rotação com thrust-line) → **decisão de adoção do usuário**.

## Testes / Erros

- Faixa + rejection + fixture do campo novo; hand-checks congelados das duas fórmulas; properties (sag maior ⟹ folga crítica MAIOR; z_eixo maior ⟹ limite de rotação recua — estritos); pins honestos em cascata (tolerâncias intactas); TDD RED-first; genericidade verde; contrato de h_cg documentado onde é consumido.

## Fora de escopo

- Dinâmica de bounce/absorção; interferência hélice-fluxo na empenagem; adoção E11 (decisão humana no fim).

## ERRATUM (2026-08-09, revisão da Task 2)

O §2 prescrevia `z_eixo = h_cg_ground_m + prop_axis_above_cg_m` (braço sobre o solo) para o momento de tração na rotação. **Errado**: momentos sobre o pivô nos mains durante a corrida ACELERADA exigem o termo inercial (d'Alembert, `−m·a·h_cg`), que cancela a porção `h_cg` do braço — o braço sobrevivente é **`prop_axis_above_cg_m`** (+ termos de solo pequenos `μN·h_cg` e `D·(h_cg−h_D)`, ≲2 pp, documentáveis como desprezados). Referência: balanço de rotação padrão (Gudmundsson/Roskam), que carrega `T·(z_T−z_mg)` E `m·aₓ·(z_cg−z_mg)` juntos. Efeito correto: ~6–9 pp de recuo (não ~28); sensibilidade do eixo +12 cm ≈ +0,5 pp (não +3). A intenção do §2 ("capturar o custo real da linha de tração") governa; o braço literal está corrigido pela implementação.
