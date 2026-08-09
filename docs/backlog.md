# Backlog de ciclo futuro

Itens de modelagem NOMEADOS e conscientemente NÃO corrigidos nas revisões
até aqui — cada um tem um achado honesto registrado no código/schema, mas a
correção fica fora de escopo da task/revisão que o descobriu. Este arquivo
existe para que esses itens não se percam entre ciclos; cada entrada tem
uma linha de contexto e o ponteiro para onde o achado está documentado em
detalhe.

## 1. Transferência de atitude do #25 (folga crítica de hélice, CS 23.925) — RESOLVIDO ciclo 9

**RESOLVIDO** (ciclo 9, 2026-08-09). `propeller.prop_clearance_critical_m`
modelava o colapso do amortecedor de nariz + pneu murcho como uma
TRANSLAÇÃO VERTICAL 1:1 do nariz. `PropellerSpec::fill_critical_clearance`
agora modela o PIVÔ da célula sobre o trem PRINCIPAL: a hélice (à frente
do trem de nariz) mergulha um braço AMPLIFICADO por `fator =
(gear.x_main_m − propeller.prop_plane_x_m)/(gear.x_main_m −
gear.x_nose_m)` — campo novo `[propeller].prop_plane_x_m` (posição do
plano da hélice, m do datum no nariz; ESTIMATIVA de geometria, validar no
CAD). Achado confirmado: no baseline E10 real, fator ≈ 1,46610,
`prop_clearance_critical_m` vai de **+0,0325 m (PASS) para ≈−0,06416 m
(FAIL)** — o provável FAIL honesto previsto aqui se concretizou. A
checagem #25 REPROVA o baseline real desde este ciclo (1 violação nomeada,
`validation_status: FAIL`) — decisão de projeto (mover a hélice/trem, ou
aceitar a folga negativa até validação em CAD/ensaio) fica para revisão
humana; este ciclo mede, não tuna. Ponteiro: docstring de
`PropellerSpec::prop_clearance_critical_m` (`src/models/specs.rs`,
histórico old→new completo), checagem #25 em
`validation::constraint_checker::ConstraintChecker::verify`,
`docs/aircraft_spec.schema.md` (bloco `propeller` e histórico v5.1 §3-§4),
`tests/cli.rs`/`tests/gear_tipback.rs`/`tests/schema_v4.rs` (pins
honestos).

## 2. Gradiente CS 23.65 avaliado a 1,05·Vs, não ≥1,2·Vs

`agents::performance::best_climb_angle_ms` varre `[1,05·V_s, 1,80·V_s]` e,
para esta célula/motor, RC/V é monotonicamente decrescente na faixa —
então a função devolve o PISO da varredura (1,05·V_s_to), não um máximo
interior genuíno. A referência típica da CS 23.65 é ≥1,2·V_s; nessa
referência o gradiente do baseline real seria ≈12,4486%, não os
13,896713% hoje retornados (~1,45 p.p. de viés OTIMISTA remanescente).
Ponteiro: docstring de `agents::performance::best_climb_angle_ms`
(`src/agents/performance.rs`), `fidelity.performance`,
`docs/aircraft_spec.schema.md` (campos `vx_kmh`/`climb_gradient_pct`).

## 3. Vy híbrido (CL de estol flapado + polar limpa)

`agents::performance::climb_rate_ms` (usado para Vy/`rc_sl_ms`/
`service_ceiling_m`) usa `wing.cl_max` (CL_max COM FLAP) como referência
de estol para definir a faixa de varredura, mas chama `excess_power_kw`
com `cd0_extra = 0.0` (arrasto de configuração LIMPA) — um híbrido
"CL de estol flapado + arrasto limpo" inconsistente. Como Vy é referência
EN-ROUTE (não um check de decolagem), a correção não é óbvia: pode ser
mais correto usar `cl_max_clean` na referência de estol para ficar
consistente com o arrasto limpo, ou o inverso. Ponteiro: docstring de
`agents::performance::climb_rate_ms` (`src/agents/performance.rs`).

## 4. Rolagem de decolagem/pouso sem termo de arrasto (método de energia)

`takeoff_ground_roll_m`/`landing_ground_roll_m` usam o método ENERGÉTICO
de Raymer (V²/2gμ ajustado por fator de tração/frenagem), que não inclui
nenhum termo de arrasto aerodinâmico explícito — mesmo com o flap agora
modelado na polar (`cd0_flap_to_extra`, ciclo 8), o segmento DOMINANTE da
distância de decolagem/pouso continua sem custo de arrasto por construção
do método. Corrigir exigiria trocar o método de rolagem por integração
numérica (V, t) consumindo a polar completa segmento a segmento. Ponteiro:
`agents::performance::takeoff_ground_roll_m`/`landing_ground_roll_m`
(`src/agents/performance.rs`), `docs/aircraft_spec.schema.md` (bloco
`performance`, linha `cd0_flap_to_extra`).

## 5. `+INFINITY` → `null` no JSON quando `rc ≤ 0`

`takeoff_distance_50ft_m` devolve `s_ground + s_rotation + f64::INFINITY`
quando a razão de subida calculada não é positiva (obstáculo inatingível
nesta condição) — mas, ao contrário de `fatigue_life_cycles`
(`docs/aircraft_spec.schema.md` §5, que serializa infinito explicitamente
como a string `"infinita"` para não quebrar o round-trip), `to_50ft_paved_m`/
`to_50ft_grass_m` não têm esse tratamento: `serde_json` converteria o
`f64::INFINITY` silenciosamente para `null`, o que quebraria a
desserialização de volta em `f64` para qualquer consumidor downstream.
Ponteiro: `agents::performance::takeoff_distance_50ft_m`
(`src/agents/performance.rs`, ramo `if rc <= 0.0`), §5 de
`docs/aircraft_spec.schema.md` (precedente de tratamento de infinito).
