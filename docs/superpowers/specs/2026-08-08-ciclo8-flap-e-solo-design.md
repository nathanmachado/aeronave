# Ciclo 8 — Arrasto de Flap, Gradiente CS 23.65 e Folga Crítica CS 23.925 — Design

**Data:** 2026-08-08 · **Baseline de partida:** E10 (`b6ca56d`, PASS robusto, schema 5.0, 432 testes)

## Problema (backlog nomeado dos ciclos 6–7 + revisão E10)

1. **A polar não tem arrasto de flap** — lacuna declarada (fidelity + docstring do teste de monotonicidade do ciclo 7): as distâncias de decolagem/pouso usam CL flapado com arrasto limpo → otimistas; e `to_flap_fraction` é um dial meio-modelado (mais flap só ajuda).
2. **Gradiente CS 23.65 é um híbrido documentado** (Task 4.7): CL_max de POUSO + arrasto limpo, num check que é de configuração de DECOLAGEM.
3. **Folga de hélice só é avaliada em atitude estática** (caveat declarado na adoção E10): a condição crítica do CS 23.925 (amortecedor no batente + pneu murcho) não existe no modelo — e o E10 vive com 1 cm de folga estática sobre o piso de projeto.
4. **Pin de rotação** ±1,5 pp ficou 16,8% relativo após o ciclo 7 (dívida registrada).

## Design (aprovado pelo usuário, 2026-08-08)

### §1 — Arrasto de flap na polar

- Campo novo `[wing].cd0_flap_delta` — ΔCD₀ do flap CHEIO (pouso): baseline **0,015** (slotted moderado, Raymer cap. 12/Hoerner; faixa **(0,005, 0,05)**; fixture distinta 0,020).
- Aplicação pela MESMA fração de deployment dos demais efeitos (`to_flap_fraction`): decolagem soma `to_flap_fraction × cd0_flap_delta` à polar nos segmentos de decolagem (rolagem, rotação, subida a 15 m); pouso soma o delta CHEIO onde a polar de pouso é usada (segmento de aproximação/planeio do `landing_distance_50ft_m`; a rolagem de pouso é dominada por frenagem — auditar call sites no plano).
- O teste `mais_flap_de_decolagem_encurta_a_decolagem` (ciclo 7) morre como estava previsto na própria docstring: reescrever para o trade-off completo (CL ajuda, arrasto cobra — a direção líquida vira propriedade medida, não lei).
- Consequência esperada: decolagem grama +poucos % (folga ~140 m absorve); pouso +poucos % (folga 43 m absorve) — verificar, não forçar.

### §2 — Gradiente CS 23.65 honesto

- `climb_gradient_pct` (e o check do piso 8,3%) passa a usar `cl_max_to` E a polar com o arrasto de flap PARCIAL do §1 — configuração de decolagem consistente de ponta a ponta. O comentário do híbrido morre (histórico preservado).
- Consequência esperada: gradiente cai (arrasto ↑ e V_ref muda) — margem atual é folgada (14,5% vs 8,3%); verificar.

### §3 — Folga de hélice em condição crítica (CS 23.925)

- Piso estático 0,23 m MANTIDO (proxy de projeto, caveat existente).
- Check NOVO (nº 25): `folga_crítica = folga_estática − (curso_do_oleo_do_NARIZ_computado + tire_deflation_delta_m) > 0` — amortecedor dianteiro no batente + pneu dianteiro completamente murcho (a hélice é tratora: o trem de nariz governa). Campo novo `[gear].tire_deflation_delta_m`: baseline **0,08** (deflexão total de pneu 5.00-5 típico; faixa **(0,03, 0,15)**; fixture distinta).
- Expor `prop_clearance_critical_m` no bloco `propeller` do JSON (schema 5.1) e no print.
- Estimativa E10: 0,240 − (0,127 + 0,08) = **+0,033 m** — positivo por pouco; o run real decide, e a robustez massa-total re-avalia (o curso computado cresce com MTOW). O que vier é o achado.

### §4 — Pin de rotação

- ±1,5 → **±0,05** pp (absoluto; ~0,6% relativo do valor atual 8,533/8,908 — conferir o valor corrente pós-E10 e centrar).

### Schema 5.0 → 5.1

- `propeller.prop_clearance_critical_m` (novo); consequências numéricas de §1–§2 nas distâncias/gradiente; histórico + doc.

## Tratamento de erros / Testes

- Faixas + rejection tests + fixtures distintas para os 2 campos novos; TDD RED-first (hand-checks das fórmulas novas; property: mais `tire_deflation_delta_m` ⟹ folga crítica menor; o trade-off de flap medido); pins honestos em cascata (tolerâncias INALTERADAS — exceto o §4, que é APERTO deliberado e nomeado); genericidade verde.
- Se o E10 reprovar em qualquer check novo: NÃO mascarar — reportar com números; o destino (trem, pneu, hélice) é decisão humana.

## Sequência do ciclo

1. Implementação (SDD em worktree, ~3 tasks: §1+§2 na polar/performance; §3+§4; schema 5.1 + regen + validação E10).
2. Relatório: E10 re-validado com o modelo mais fiel — PASS mantido ou achado novo.

## Fora de escopo

- Curva CL/CD de flap não-linear; efeito de solo; pneu principal murcho (hélice tratora — nariz governa); campanha nova (só se o E10 reprovar, decisão humana).
