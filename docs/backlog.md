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
humana; este ciclo mede, não tuna. `old→new` (ciclo 9 → ciclo 11): a
checagem #25 REPROVOU do ciclo 9 ao ciclo 10 — desde o baseline E12
(`x_nose_m` 1,30→1,20, trem de nariz recuado) ela **PASSA**, com folga
crítica `prop_clearance_critical_m = +0,007367 m`. O texto acima descreve
o veredito NAQUELE momento (ciclo 9), não o estado atual. Ponteiro: docstring de
`PropellerSpec::prop_clearance_critical_m` (`src/models/specs.rs`,
histórico old→new completo), checagem #25 em
`validation::constraint_checker::ConstraintChecker::verify`,
`docs/aircraft_spec.schema.md` (bloco `propeller` e histórico v5.2),
`tests/cli.rs`/`tests/gear_tipback.rs`/`tests/schema_v4.rs` (pins
honestos).

## 2. Gradiente CS 23.65 avaliado a 1,05·V_s_to, não ≥1,2·V_s_to — RESOLVIDO ciclo 11

**RESOLVIDO** (ciclo 11, task 1, 2026-08-10). `agents::performance::
best_climb_angle_ms` varria `vx_kmh`/`climb_gradient_pct` a partir do piso
1,05·V_s_to. Como RC/V é monotonicamente DECRESCENTE nessa faixa, a
função sempre devolvia o próprio PISO da varredura — avaliar mais cedo
(mais devagar) dá gradiente MAIOR, então 1,05·V_s_to era um viés
OTIMISTA, não uma leitura conservadora da norma. Tarefa mudou o piso de
1,05·V_s_to para 1,20·V_s_to, alinhando com a referência típica da CS
23.65 (velocidade de subida ≥1,2·Vs1).

Baseline real E10: `climb_gradient_pct` **13,896713% (1,05·Vs, otimista)
→ 12,451842% (1,20·Vs, honesto)** (-1,444871 pp, viés OTIMISTA removido);
`vx_kmh` **121,519501 → 138,871480 km/h** (**+14,28%**, razão exata
1,20/1,05 = 1,142857 — acompanha a subida do piso de velocidade de
referência). Gate PASSA (≥ 8,3%), folga intacta. Ponteiro: docstring de
`agents::performance::best_climb_angle_com_piso`/`best_climb_angle_ms`
(`src/agents/performance.rs`), `docs/aircraft_spec.schema.md` (campos
`vx_kmh`/`climb_gradient_pct`), `tests/generic_engine.rs` (pins old→new).

## 3. Vy híbrido (CL de estol flapado + polar limpa) — RESOLVIDO ciclo 11

**RESOLVIDO** (ciclo 11, task 2, 2026-08-10, com DISCOVERY de artefato).
`agents::performance::climb_rate_ms` (Vy/`rc_sl_ms`/`service_ceiling_m`)
usava `wing.cl_max` (CL_max COM FLAP) como referência de estol para a
faixa de varredura [1,3;1,8]·V_s, mas `excess_power_kw` com `cd0_extra = 0.0`
(arrasto limpo, EN-ROUTE) — híbrido "CL flapado + arrasto limpo". Tarefa
mudou para referência consistente `cl_max_clean` (estol limpo, EN-ROUTE,
pois Vy é config de cruzeiro sem flap), e refinei a faixa conforme ERRATUM
da spec: [1,05;2,00]·V_s com guarda de argmax interior falseável (evita
artefato anterior: com [1,3;1,8] a faixa era calibrada demais para o CL
flapado; quando referência virou limpa, o pico real de RC saia da janela,
deixando o algoritmo retornando o PISO dela — artefato numérico zero física).

Baseline E10: **`vy_kmh` 147,915721 → 148,435393 km/h** (+0,35%, efeito
líquido ≈zero — Vy não depende de CL_max, o pico de RC é insensível; a
mudança de referência só corrigi a faixa de busca). **`rc_sl_ms`
4,999902 → 4,999905 m/s** (≈ +0,00003 m/s, insignificante). **`service_ceiling_m`
5.200 (inalterado)**. Gate PASSA (RC ≥ 1,5 m/s, teto ≥ 3.000 m), folga intacta.

DISCOVERY: a janela [1,3;1,8]·Vs da v5.3 era calibrada para o CL flapado,
não para EN-ROUTE limpo — histórico: alguma confusão sobre qual config de
flap havia na decolagem/cruzeiro foi embutida na faixa. Nenhuma anotação
de época justificava empiricamente [1,3;1,8]. Com a mudança para referência
limpa e faixa [1,05;2,00], o pico interior reaparece na busca; com a guarda
de argmax interior (testa se máximo está no interior da faixa, não no piso)
fica seguro.

Registro do ARTEFATO pego pelo ERRATUM (para o registro ficar autocontido):
durante a rodada 1 da task 2 (referência trocada para `cl_max_clean` mas
ainda com a janela antiga [1,3;1,8]·Vs), o argmax caiu no PISO da janela,
não em um máximo interior genuíno — `vy_kmh` chegou a **161,805734 km/h**,
`rc_sl_ms` a **4,9533 m/s** e `service_ceiling_m` a **5100 m**, todos
ARTEFATOS numéricos da janela de busca deslocada (não física real). A
guarda de argmax interior detectou o piso como resultado e o ERRATUM
ampliou a janela para [1,05;2,00]·Vs, restaurando o pico interior
verdadeiro — valores finais (já reportados acima): `vy_kmh`
**148,435393 km/h**, `rc_sl_ms` **4,999905 m/s**, `service_ceiling_m`
**5200 m**. Ponteiro: docstring de `agents::performance::climb_rate_ms`
(`src/agents/performance.rs`), comentário datado "Campanha ciclo 11
(2026-08-10)", ERRATUM em
`docs/superpowers/specs/2026-08-10-ciclo11-subida-honesta-design.md` (§3
DISCOVERY, §4 ERRATUM da spec), `docs/aircraft_spec.schema.md` (campos
`vy_kmh`/`rc_sl_ms`), `tests/generic_engine.rs` (pins old→new).

## 4. Rolagem de decolagem/pouso sem termo de arrasto (método de energia) — RESOLVIDO ciclo 12

**RESOLVIDO** (ciclo 12, tasks 2 e 3, 2026-08-15). `takeoff_ground_roll_m`/
`landing_ground_roll_m` usavam métodos ENERGÉTICOS fechados sem termo de
arrasto aerodinâmico explícito — mesmo com o flap já modelado na polar
(`cd0_flap_to_extra`, ciclo 8), o segmento DOMINANTE da distância de
decolagem/pouso ficava sem custo de arrasto por construção do método.
`old→new` (correção de descrição, fix wave ciclo 12): o texto acima dizia
que "ambos os segmentos usavam o método ENERGÉTICO de Raymer (`V²/2gμ`)"
— FALSO para a decolagem. `git show 1e11998:src/agents/performance.rs`
mostra `takeoff_ground_roll_m` como `w*w / (G*rho*wing.area_m2*cl_to*
t_avg)` — SEM coeficiente de atrito nenhum, um método de EMPUXO MÉDIO
distinto. A fórmula `V_ref²/(2·g·μ)` era usada SÓ no pouso
(`landing_ground_roll_m`). Ver detalhe correto na tabela `Método
substituído` abaixo.

**Método novo:** integração numérica da equação de movimento em `V`
(Simpson composto, 200 intervalos, convergência verificada — dobrar para
400 intervalos muda o resultado em menos de 0,1%), consumindo a polar
completa segmento a segmento:

- **Decolagem** (`agents::performance::takeoff_ground_roll_m`, via
  `integra_rolagem_decolagem_com_passos`): `S = ∫₀^{V_LOF} m·V/F_net(V) dV`,
  `F_net(V) = T(V) − D(V) − μ_roll·max(0, W − L(V))`, com `T(V)` de
  `thrust_ground_roll_n` (teoria de quantidade de movimento com velocidade
  de avanço, spec §2), `D(V)`/`L(V)` da polar completa via `cd_ground_roll`
  (CD0 + incremento de trem estendido + incremento de flap parcial de
  decolagem + induzido).
- **Pouso** (`agents::performance::landing_ground_roll_m`, via
  `integra_rolagem_pouso_com_passos`): `S = ∫₀^{V_ref} m·V/F_dec(V) dV`,
  `F_dec(V) = D(V) + μ_brake·max(0, W_ldg − L(V))` (arrasto e atrito de
  frenagem SOMAM, ao contrário da decolagem), com `L(V)` do
  `cl_ground_roll_landing` (CL de solo com flap CHEIO) e `D(V)` do
  `cd_ground_roll` com o incremento de flap de pouso.

**Método substituído (`old`):** `old→new` (correção de descrição, fix
wave ciclo 12) — os dois segmentos usavam fórmulas ENERGÉTICAS fechadas
DIFERENTES, não a mesma. **Pouso** (`landing_ground_roll_m`, verificado em
`1e11998`): `S_G = V_ref²/(2·g·μ)` (ajustada por fator de frenagem médio).
**Decolagem** (`takeoff_ground_roll_m`, verificado em
`git show 1e11998:src/agents/performance.rs`): `S_G = w²/(G·ρ·área·
cl_to·T_médio)` — método de EMPUXO MÉDIO, **sem coeficiente de atrito
nenhum** (μ não entrava na fórmula da decolagem). As duas fórmulas, por
construções distintas, não tinham NENHUM termo de arrasto aerodinâmico —
nem o CD0 de trem estendido, nem o induzido, nem o incremento de flap
entravam na conta da rolagem, só no gradiente de subida posterior.

**Tabela `old→new`, medida em `aircraft_spec.json`** (commit pré-ciclo-12
`1e11998` vs HEAD atual, baseline real Toyota 1GD-FTV):

| Campo (`performance.*`) | Old (`1e11998`) | New (ciclo 12) | Δ |
|---|---|---|---|
| `to_50ft_paved_m` | 420,372451 | 651,258408 | +54,92% |
| `to_50ft_grass_m` | 473,469470 | 819,110978 | +73,00% |
| `to_distance_paved_m` | 398,227641 | 744,556577 | +86,97% |
| `to_distance_grass_m` | 477,873169 | 996,335432 | +108,49% |
| `ldg_50ft_m` | 502,458299 | 582,341118 | +15,90% |
| `ldg_50ft_grass_m` | 556,677173 | 646,437301 | +16,12% |
| `landing_distance_m` | 362,656622 | 442,539441 | +22,03% |

Consequência de gate: `to_50ft_grass_m` (819,11 m) excede a pista de 600 m
da checagem #23 e `ldg_50ft_grass_m` (646,44 m) excede a de #24 —
`validation_status` do baseline real vira `FAIL` (ver report da task 5 para
o veredito completo). Não é regressão: o modelo passa a pagar o arrasto que
sempre esteve fisicamente presente. Ponteiro: docstrings de
`agents::performance::takeoff_ground_roll_m`/`landing_ground_roll_m`/
`thrust_ground_roll_n`/`cl_ground_roll_landing` (`src/agents/
performance.rs`), `docs/aircraft_spec.schema.md` (bloco `performance`,
§5 e histórico v5.5), `tests/generic_engine.rs` (pins old→new),
`docs/superpowers/specs/2026-08-15-ciclo12-solo-honesto-design.md` (§2,
§3, §5, §9 tabela congelada).

## 5. `+INFINITY` → `null` no JSON quando `rc ≤ 0` — RESOLVIDO ciclo 11

**RESOLVIDO** (ciclo 11, task 3, 2026-08-10). `takeoff_distance_50ft_m`
devolve `s_ground + s_rotation + f64::INFINITY` quando a razão de subida
calculada não é positiva (obstáculo de 15m inatingível nesta condição) —
resultado FÍSICO válido, não erro. Mas `to_50ft_paved_m`/`to_50ft_grass_m`
não tinham o tratamento de infinito que `fatigue_life_cycles` já
implementava: `serde_json` convertiria `f64::INFINITY` silenciosamente para
`null` (RFC 8259 não tem representação de infinito), quebrando round-trip
— um consumidor desserializando `null` falharia em conversão para `f64`.

Tarefa adicionou `#[serde(with = "fatigue_life_serde")]` em ambos os campos
— módulo existente desde ciclo 8 task 6.1 para tratar `fatigue_life_cycles`.
Ambos os campos agora serializam `f64::INFINITY` como a string `"infinita"`
em vez de `null` ou número. Teste RED-FIRST (ciclo 11 task 3): round-trip
sintético com `to_50ft_paved_m = f64::INFINITY` fallava antes (JSON tinha
`null`), passa agora (JSON tem `"infinita"`, desserializa de volta como
`INFINITY`). Casos finitos continuam numerais normais. Gate e baseline
real nunca disparam este caso especial (sempre têm razão de subida positiva)
— só documentado e testado para robustez de consumidores. Ponteiro:
`#[serde(...)]` em `src/models/specs.rs` (PerformanceSpec, linha ~533/535),
teste `models::specs::tests::performance_spec_roundtrip_serde_com_infinito`
(mesmo arquivo), `docs/aircraft_spec.schema.md` (§5 estendido, campos
`to_50ft_paved_m`/`to_50ft_grass_m`), SCHEMA_VERSION 5.4.

## 6. Condição composta CS 23.925: deflexão dos mains no pivô — RESOLVIDO ciclo 10

**RESOLVIDO** (ciclo 10, task 1, 2026-08-09). A preocupação nomeada no
ciclo 9 — `PropellerSpec::fill_critical_clearance` pivota a célula sobre o
trem PRINCIPAL para amplificar o mergulho do plano da hélice, mas
tratando o próprio trem PRINCIPAL como RÍGIDO e TOTALMENTE ESTENDIDO, sem
nenhum termo para a deflexão do amortecedor/pneu principal — partia de
uma premissa INCORRETA sobre o que `[gear].h_cg_ground_m` representa.
Lida com cuidado, essa altura SEMPRE foi a altura do CG com a aeronave
CARREGADA, em deflexão ESTÁTICA (mains e nariz parcialmente comprimidos
pelo peso da aeronave) — não "trem estendido sem carga". Como
`h_cg_ground_m`/`propeller.ground_clearance_m` já embutem essa deflexão
estática dos mains, a checagem #25 NUNCA precisou de um termo aditivo
para eles: CS 23.925 pela LETRA exige apenas que o trem CRÍTICO (aqui, o
de nariz — hélice TRATORA) atinja o batente nessa condição; os DEMAIS
trens permanecem na deflexão estática normal, já modelada. Não havia
condição COMPOSTA não-modelada — havia uma leitura imprecisa do
significado de `h_cg_ground_m`.

Pelo MESMO motivo, o amortecedor de NARIZ também PARTE da deflexão
estática (não estendido) — na condição crítica ele só percorre o curso
RESTANTE até o batente, não o curso TOTAL. A fórmula do ciclo 9 usava o
curso total, contando a compressão estática do nariz DUAS VEZES (uma
implícita em `h_cg_ground_m`, outra explícita no curso total do
batente). Campo novo `[gear].static_sag_fraction` (0,33 no baseline —
fração do curso já consumida pela compressão estática) corrige isso:
`Δ_prop` usa `nose_oleo_stroke_mm × (1 − static_sag_fraction)`, não
`nose_oleo_stroke_mm`. No baseline real, fator geométrico inalterado
(≈1,46610 — não depende de `static_sag_fraction`);
`prop_clearance_critical_m` vai de **≈−0,06416 m (ciclo 9) para
≈−0,00249 m (ciclo 10)** — honestamente ANTI-conservador (a correção
AUMENTA a folga calculada), mas fiel à letra da norma. `validation_status`
do baseline real PERMANECE `"FAIL"` com a MESMA 1 violação nomeada
(checagem #25) — só o NÚMERO da violação muda, não o veredito. `old→new`
(ciclo 10 → ciclo 11): esse veredito era válido no ciclo 10 — desde o
baseline E12 (`x_nose_m` 1,30→1,20) a checagem #25 **PASSA**
(`prop_clearance_critical_m = +0,007367 m`); ver item 1 acima.

Nota relacionada, sinal OPOSTO e pequena, NÃO resolvida por esta task
(item independente, permanece nomeado): o disco da hélice também não é
modelado como INCLINADO junto com o pitch da célula durante o evento —
tratar o disco como permanecendo vertical (ponta mais baixa sempre à
distância do raio abaixo do cubo) é CONSERVADOR em ≈+3,4 mm
(`raio × (1 − cos θ)`, θ ≈ 5,04° no baseline real, raio 0,88 m) frente a
uma modelagem exata do disco tombado — o tombamento ERGUE o ponto mais
baixo varrido em relação ao cubo, então ignorá-lo empurra a folga
calculada para o lado SEGURO. Ponteiro: docstring de
`PropellerSpec::prop_clearance_critical_m` e `GearCfg::h_cg_ground_m`/
`GearCfg::static_sag_fraction` (`src/models/specs.rs`,
`src/models/aircraft_config.rs`),
`validation::constraint_checker::ConstraintChecker::verify` (checagem
#25), `fidelity.propeller` (`src/main.rs`), `docs/aircraft_spec.schema.md`
(bloco `propeller`, linha `prop_clearance_critical_m`), `tests/cli.rs`/
`tests/gear_tipback.rs`/`tests/schema_v4.rs` (pins honestos).

## 7. Textos pré-erratum sobre o momento da linha de tração ficaram desatualizados (ciclo 10, task 2) — RESOLVIDO ciclo 10

**RESOLVIDO** (ciclo 10, fix wave, 2026-08-09, commits `a7b561a` / `2d4fff7`
/ `a465e7b`). O erratum do ciclo 10 (task 2, commit `713e846`) corrigiu o
braço do momento de rotação de "sobre o solo" (`h_cg_ground_m + prop_axis_above_cg_m`
≈ 1,12 m) para "sobre o CG" (`prop_axis_above_cg_m` ≈ 0,20 m, ver §2 do
erratum em
`docs/superpowers/specs/2026-08-09-ciclo10-sag-e-linha-de-tracao-design.md`).
A correção do CÓDIGO de produção (`agents::trim_authority::
rotation_available_moment_nm`/`rotation_fwd_limit_m`, chamados com
`cfg.propeller.prop_axis_above_cg_m`) estava correta e testada, mas TRÊS
textos descrevendo o modelo ANTES do erratum não foram atualizados junto
(achado da review da Task 3 do ciclo 10, fora de escopo então, nomeado no
backlog). Fix wave reescreveu todos:

1. **`fidelity.trim` em `src/main.rs`** (linha ~836): era *"desconsidera
   binário tração/arrasto/inércia"*, hoje (na época, ciclo 10) **"inclui
   binário de TRAÇÃO `−T(Vr)·prop_axis_above_cg_m` no balanço; desconsidera
   termos de SOLO (residual ≈ μ_roll·(W−L_g)·h_cg, ≲2 pp)"** — refletia o
   comportamento real NAQUELE momento (tração incluída, solo desprezado).
   `old→new` (ciclo 10 → ciclo 12): esta descrição ficou DESATUALIZADA — a
   task 4 deste ciclo IMPLEMENTOU os termos de solo (`src/main.rs:901`
   atual já diz o contrário: "os três recuam o limite dianteiro de rotação
   em conjunto"). Não é mais o texto vigente; ver `fidelity.trim` corrente
   e §1/§4 (bloco `trim`) de `docs/aircraft_spec.schema.md`.
2. **Docstring `momento_da_linha_de_tracao_hand_check_com_literais`** em
   `src/agents/trim_authority.rs` (linha ~1108): era *"z_eixo = 1,12 m
   (h_cg_ground 0,92 + offset 0,20 do E10)"* (descrição pré-erratum),
   hoje **"z_eixo = 0,20 m = prop_axis_above_cg_m do baseline E10"** —
   literal agora é correto e compatível com o código real (test segue
   válido, comentário parou de induzir erro).
3. **Docstring `eixo_mais_alto_recua_o_limite_de_rotacao`** mesmo arquivo
   (linha ~1179-1180): era *"z=1,12 (baseline E10), z=1,24 (candidato E11
   +12 cm)"*, hoje **"z=0,20 (baseline E10), z=0,32 (candidato E11 +12 cm)"**
   — literais sincronizados com o baseline real.

Nenhum era bug de física ou teste — eram apenas textos/comentários que
não acompanharam o erratum de braço de momento. Tarefa ciclo 11 (task 3)
marcou como RESOLVIDO aqui por registro histórico.

Ponteiro: `src/main.rs` (`fidelity.insert("trim"...)`), `src/agents/
trim_authority.rs` (duas docstrings acima), commits `a7b561a`/`2d4fff7`/
`a465e7b` (ciclo 10 fix wave).

## 8. Unificar o modelo de tração (`thrust_ground_roll_n` × `thrust_available_n` divergem em `V_LOF`) — RESOLVIDO ciclo 13

Nomeado no ciclo 12 (task 2, spec §2.4) como fora de escopo, medido aqui
(task 5). `agents::performance::thrust_ground_roll_n` (teoria de
quantidade de movimento com velocidade de avanço, usada na rolagem) e
`agents::performance::thrust_available_n` (disco atuador estático em
V=0 + `prop_efficiency(J)` acima de V=0,5, usada em cruzeiro/subida/teto)
são dois modelos de domínios de validade DISJUNTOS que nunca foram
conciliados. Em V=0 as duas são algebricamente IDÊNTICAS (identidade
provada e testada — `tracao_de_rolagem_em_v_zero_e_identica_ao_estatico_no_
baseline_real`), mas divergem ao longo da rolagem porque só uma delas paga
o modelo de quantidade de movimento com avanço.

**Medido no baseline real** (Toyota 1GD-FTV, HEAD atual, `V_LOF =
1,10·V_s_TO = 35,360970 m/s`):

| Função | `T(V_LOF)` medido |
|---|---|
| `thrust_ground_roll_n(V_LOF)` | 2.324,6885 N |
| `thrust_available_n(V_LOF)` | 3.214,9867 N |
| Divergência | −890,2982 N (**−27,69%**) |

A rolagem de decolagem usa `thrust_ground_roll_n` do início ao fim (nunca
faz costura com `thrust_available_n`) — mas os dois modelos descrevem a
MESMA grandeza física (tração da hélice nesta velocidade) e devolvem
números 27,69% distintos, o que é uma descontinuidade de modelo, não de
código. `old→new` (fix wave ciclo 12, este próprio ciclo falsificou a
frase acima): "este ciclo não introduz inconsistência NUM cálculo" —
FALSO. A task 4 (item de backlog dedicado, "INCONSISTÊNCIA DE MODELO DE
TRAÇÃO NO BALANÇO DE ROTAÇÃO", prioridade ALTA, abaixo) INTRODUZ uma
inconsistência de cálculo real: o balanço de `rotation_available_moment_nm`
usa `thrust_at_rotation_n` (via `thrust_available_n`) no termo de momento
enquanto os termos de solo, adicionados por esta mesma task, usam o
modelo de solo (`thrust_ground_roll_n`, implícito em `D`/`μN` calculados
com a física de solo) NA MESMA VELOCIDADE (`Vr = V_LOF`, por construção
— ver item dedicado) — um resíduo `(T_solo − T_momento)·h_cg` não
cancelado, de magnitude comparável ao próprio termo corrigido por este
ciclo. Unificar reabriria cruzeiro, teto de serviço, alcance e autonomia
(todos consumidores de `thrust_available_n`/`prop_efficiency`), por isso
fica fora de escopo deste ciclo. Ponteiro: docstrings de
`agents::performance::thrust_ground_roll_n`/`thrust_available_n`
(`src/agents/performance.rs`), spec §2.4 e §11 item 1
(`docs/superpowers/specs/2026-08-15-ciclo12-solo-honesto-design.md`).

**RESOLVIDO** (ciclo 13, task 2, 2026-08-15, spec
`2026-08-15-ciclo13-tracao-unificada-design.md` §1/§2/§4). Os dois
modelos foram FUNDIDOS numa lei única `T(V) = FoM(J)·T_ideal_momentum(V,
P_eixo)`: `T_ideal_momentum` é a cúbica de Rankine-Froude com avanço já
validada em `thrust_ground_roll_n` (ciclo 12), e `FoM(J)` é uma figura de
mérito linear entre duas âncoras medidas (`fom_static` em J=0,
`fom_design` em J=`j_design`) que substitui o multiplicador plano
`static_thrust_factor` e o polinômio `prop_efficiency`.
`thrust_ground_roll_n`, `prop_efficiency` e `thrust_n` foram APAGADOS —
não há mais dois caminhos de código para a mesma grandeza física.

**Medido no baseline real, mesma comparação do achado original**: a
divergência de 27,69% em `V_LOF` (2.324,6885 N vs 3.214,9867 N) deixou de
poder existir — as duas chamadas são agora a MESMA função com os MESMOS
argumentos. Verificado por teste dedicado (`tracao_do_momento_de_
rotacao_e_identica_a_da_rolagem_no_mesmo_vr`, `tests/generic_engine.rs`):
resíduo relativo `0e0` (zero exato) nos 6 cenários de CG, tolerância
1e-12 — ver item 15 abaixo para a consequência no balanço de rotação.

Consequência colateral medida (não é regressão — é o polinômio deixando
de mascarar tração fisicamente impossível, ver item 9 abaixo):
`climb_gradient_pct` **12,451842% → 7,913277%** (FLIPA de PASS para FAIL
contra o mínimo de 8,3% da CS 23.65); `to_50ft_grass_m` **819,110978 →
858,593425 m** (segue REPROVANDO #23); `v_cruise_kmh` **300,220683 →
291,076342 km/h** (−3,05%, `max_level_speed_ms` avalia com
`engine.rpm_rated`=3400, onde o polinômio apagado estava perto do próprio
pico — ver item 20 abaixo); `range_km`/`endurance_h`
**inalterados a −0,037%** (residual do laço de convergência de MTOW, ver
item 21 abaixo — o PONTO de cruzeiro em si é preservado por construção,
spec §3.2/§8.3). Ponteiro: docstrings de
`agents::performance::thrust_available_n`/`thrust_ideal_momentum_n`,
`agents::propulsion::FigureOfMerit` (`src/agents/`), spec ciclo13 §1/§2/
§8.1, `tests/generic_engine.rs`/`src/agents/performance.rs` (`mod tests`,
guardas do teto de quantidade de movimento).

## 9. `prop_efficiency` com `η(0) = 0,58` — fisicamente errado por definição, janela de tração nula sem consumidor — RESOLVIDO ciclo 13

O polinômio `agents::propulsion::prop_efficiency` (`η = −0,15·J² + 0,39·J +
0,58`, clampado em `[0, 0,86]`) é calibrado com dados de JavaProp na faixa
de `J` de CRUZEIRO (`J ≈ 1,3–1,5`) — nessa faixa serve bem subida, cruzeiro
e teto de serviço, os únicos consumidores de `thrust_available_n` no
ramo de voo (`v_ms ≥ 0,5`). Mas por CONSTRUÇÃO `η(0) = 0,58`, enquanto por
DEFINIÇÃO `η = T·V/P → 0` quando `V → 0` — o polinômio nunca foi calibrado
para `J` próximo de zero e não tem por que valer ali.

O defeito fica DORMENTE hoje porque `agents::propulsion::thrust_n` tem uma
guarda `if v_ms < 1.0 { return 0.0 }` que zera a tração antes que o `η(0)`
errado chegue a ser usado — mas essa guarda cria, por sua vez, uma janela
de tração NULA em `V ∈ [0,5; 1,0)` m/s (`thrust_available_n` já entra no
ramo de voo em V=0,5, calcula um `η` via `prop_efficiency`, e `thrust_n`
zera o resultado por causa da guarda de V<1,0) — sem consumidor hoje
porque nenhum código avalia `thrust_available_n` nesse intervalo estreito,
mas sem justificativa física registrada até este ciclo.

**Medido no baseline real:**

| `V` (m/s) | `thrust_available_n(V)` medido |
|---|---|
| 0,7 | 0,0 N (janela nula, guarda de `thrust_n`) |
| 1,0 | 84.843,5153 N (≈23× a tração estática de 3.740,09 N — salto artificial no limiar da guarda) |

O salto em V=1,0 e a janela nula em `[0,5; 1,0)` seguem sem consumidor de
produção (só a rolagem varre `V` continuamente perto de zero, e ela usa
`thrust_ground_roll_n`, não este caminho). Corrigir exigiria recalibrar
`prop_efficiency` com dados de `J` baixo (fora do escopo deste ciclo,
reabre o mesmo território do item 8) ou substituir o modelo de tração de
baixa velocidade por quantidade de movimento em todo o ramo de voo.
Ponteiro: `agents::propulsion::prop_efficiency`/`thrust_n`
(`src/agents/propulsion.rs`, linhas 47–61), docstring de
`agents::performance::thrust_ground_roll_n` (`src/agents/performance.rs`),
spec §11 item 2.

**RESOLVIDO** (ciclo 13, task 2, 2026-08-15, spec
`2026-08-15-ciclo13-tracao-unificada-design.md` §1.1/§2.1/§4). O polinômio
`prop_efficiency` e a função `thrust_n` foram APAGADOS. A lei única
`T(V) = FoM(J)·T_ideal_momentum(V, P_eixo)` não tem nenhum ramo de
velocidade — `η(0) = 0` por CONSTRUÇÃO (a cúbica de Rankine-Froude
degenera em `u = K^(1/3)` finito quando V → 0, e `η = FoM·V/u → 0`), não
por calibração. Consequência direta, todas verificadas por teste: morre o
`η(0) = 0,58` fisicamente errado; morre o salto de 84.843,5 N em V=1,0
m/s; morre a janela de tração NULA em V ∈ [0,5; 1,0); morre o corte duro
em J > 2,8.

**O número CORRIGIDO do achado original desta entrada** (medido na
revisão de plano do ciclo 13 contra as funções REAIS de produção, spec
`old→new` do §1.1 — a primeira versão da spec dizia "5 de 8 pontos, 3
alimentando gates que passavam"; ESSE número estava errado e nunca virou
registro permanente): dos 8 pontos de operação nomeados do baseline real
(rolagem 10/20 m/s, `V_LOF`, `Vx`, `Vy` nível do mar, teto de serviço,
cruzeiro, V máx), o polinômio apagado violava o teto físico de
conservação de quantidade de movimento (`T_real ≤ T_ideal`) em
**QUATRO** deles — rolagem a 10 m/s (2,1432×), rolagem a 20 m/s
(1,3417×), `V_LOF` (1,0372×) e `Vx` (1,0095×) —, **DOIS** alimentando
gates que PASSAVAM (`Vx` → gradiente CS 23.65; `V_LOF` → balanço de
rotação, item 15 abaixo). Os outros dois violadores (rolagem) alimentavam
um gate que já REPROVAVA. As duas linhas que a primeira versão da spec
contava como violação adicional (`V máx` e teto de serviço) foram
recomputadas com os argumentos REAIS de produção
(`max_level_speed_ms` usa `engine.rpm_rated`=3400, não o rpm de cruzeiro;
`service_ceiling_m` usa o `Vy` real da altitude do teto com `mass_mid`,
não o `Vy` de nível do mar) e NÃO violam o teto (`V máx`: razão 0,8604;
teto de serviço: razão 0,9313) — a afirmação original de que eram
violadoras era FALSA, corrigida antes de virar registro permanente aqui.

Ponteiro: `agents::propulsion::FigureOfMerit`
(`src/agents/propulsion.rs`), docstring de
`agents::performance::thrust_available_n`/`thrust_ideal_momentum_n`
(`src/agents/performance.rs`), teste
`tracao_nunca_excede_o_teto_de_quantidade_de_movimento` (mesmo arquivo,
`mod tests`) e `tracao_e_continua_em_toda_a_faixa`, spec ciclo13 §1.1
(tabela `old→new`) e §8.1/§8.4.

## 10. `z_drag_above_cg_m` ainda não consumido por `cm_thrust_cruise`

`[wing].z_drag_above_cg_m` foi criado neste ciclo (task 4, spec §6) para os
termos de solo do balanço de ROTAÇÃO
(`agents::trim_authority::rotation_available_moment_nm`, braço líquido
`h_cg_m − z_drag_above_cg_m` do termo de arrasto de solo) — hoje o único
consumidor. `agents::trim_authority::cm_thrust_cruise` (cruzeiro,
`src/agents/trim_authority.rs` linha ~698) continua assumindo `z_D = 0`
(centro de arrasto no mesmo datum do CG) por construção — a própria
docstring da função (linhas ~670–697, anterior a este ciclo) já registrava
essa aproximação como "exigiria campo novo (`z_drag_above_cg_m`) sem base
no CAD atual". O campo agora EXISTE (default 0,0 no baseline, mesmo valor
que a aproximação assumia implicitamente), mas não foi conectado a
`cm_thrust_cruise` — ligar os dois é trabalho futuro, não decorre
automaticamente da criação do campo. Direção do erro registrada na própria
docstring da função: SUPERESTIMA `cm_thrust` nariz-abaixo em até ≈50% do
valor calculado no pior caso plausível (`z_D` até ≈0,10 m), efeito
ANTI-conservador ou conservador dependendo do sinal de `CL_h_trim`
(depende do CG — ver docstring completa). Ponteiro: docstring de
`agents::trim_authority::cm_thrust_cruise` (`src/agents/trim_authority.rs`,
linhas ~670–706), docstring de
`agents::trim_authority::rotation_available_moment_nm` (mesmo arquivo,
linhas ~404–426, ciclo 12 task 4).

## 11. Remoção de `to_distance_*`/`landing_distance_m` num bump MAJOR futuro

Os campos `to_distance_paved_m`/`to_distance_grass_m` (rolagem de
decolagem × fator ad hoc 1,5, aproximação de transição de Raymer) e
`landing_distance_m` (rolagem de pouso + 200 m fixos de aproximação) são
estimativas SIMPLIFICADAS legadas, mantidas lado a lado com os campos
FÍSICOS `to_50ft_paved_m`/`to_50ft_grass_m`/`ldg_50ft_m`/`ldg_50ft_grass_m`
(distância por segmentos sobre obstáculo de 15 m/50 ft). O fator 1,5 foi
calibrado como razão `distância sobre 15m / rolagem` para o método
ENERGÉTICO fechado que a rolagem usava antes deste ciclo (item 4, agora
RESOLVIDO) — com a rolagem passando a integrar arrasto e atrito
explicitamente, essa calibração ficou obsoleta.

**Razão real medida** (baseline real, ciclo 12; registrada em
`tests/generic_engine.rs`, comentário da task 2):

    to_50ft_paved_m / rolagem_pavimentada = 651,258408 / 496,371051 ≈ 1,312

abaixo do fator legado de 1,5. Consequência medida: `to_distance_paved_m`
(= rolagem × 1,5 = 744,556577 m) passa a EXCEDER `to_50ft_paved_m`
(651,258408 m) — a estimativa legada, calibrada para o método antigo, fica
visivelmente inconsistente com o campo físico do MESMO JSON (a relação
inverteu: antes `to_50ft_paved_m` sempre excedia `to_distance_paved_m`,
ver asserção `old→new` em `tests/generic_engine.rs`,
`golden_toyota_baseline_task_4_7_novos_campos_de_performance`).

Remover `to_distance_*`/`landing_distance_m` é bump MAJOR de schema (fora
de escopo deste ciclo — spec §12, política de versionamento). Os campos
ficam por ora, com docstring avisando que os `*_50ft_*` são a referência
física. Ponteiro: `docs/aircraft_spec.schema.md` (bloco `performance`,
histórico v5.5), spec §8.1 e §11 item 4, `tests/generic_engine.rs`
(comentário e asserção `old→new` citados acima).

**Atualização (ciclo 14, backlog item 17, RESOLVIDO)**: a inconsistência
do lado do POUSO PIOROU, na mesma direção da decolagem acima — a relação
inverteu de novo. Antes do ciclo 14, `landing_distance_m` (estimativa
legada, rolagem + 200 m fixos) era MENOR que `ldg_50ft_m` (referência
física). Com o segmento aéreo honesto do ciclo 14 (γ_app derivado da
polar, flare consumindo altura — item 17), `ldg_50ft_m` caiu **582,52 →
439,28 m** enquanto `landing_distance_m` ficou INALTERADO (442,70 m — não
depende do segmento aéreo, só de rolagem + 200 m fixos). Medido no
baseline real (HEAD): `ldg_50ft_m` **439,275078 m < landing_distance_m
442,702122 m** — o campo LEGADO agora SUPERESTIMA a distância física real
tanto do lado da DECOLAGEM (item acima, desde o ciclo 12) quanto do lado
do POUSO (desde o ciclo 14). Reforça o caso para a remoção MAJOR: os 200 m
fixos de aproximação do campo legado nunca representaram nada calibrado
para esta célula, e agora estão do lado ERRADO da comparação nos dois
sentidos de voo.

## 12. Efeito solo omitido na rolagem de solo — direção conservadora

A rolagem de solo (decolagem e pouso, item 4) não modela o efeito solo
(redução do arrasto induzido e aumento da sustentação por proximidade da
asa ao chão). No baseline real, `h/b = h_cg_ground_m / span_m = 0,92/11,94
≈ 0,077` — pelo fator de Wieselsberger, essa razão reduziria o induzido em
≈40% e aumentaria a sustentação. As duas direções são FAVORÁVEIS à
decolagem/pouso (menos arrasto para vencer, mas TAMBÉM menos peso nas
rodas — o efeito líquido sobre a rolagem de pouso não é obviamente
unidirecional, já que menos peso nas rodas piora a frenagem, mesmo
mecanismo que o item central da task 3 deste ciclo). Omitir o efeito solo
é, na direção do arrasto induzido, CONSERVADOR (a rolagem calculada é
maior que a física). Registrado como aproximação assumida, não medido
numericamente neste ciclo — quantificar exigiria implementar o fator de
Wieselsberger (ou equivalente) na polar de solo, item de trabalho futuro.
Ponteiro: spec §3.1 e §11 item 5
(`docs/superpowers/specs/2026-08-15-ciclo12-solo-honesto-design.md`),
`agents::performance::cd_ground_roll` (`src/agents/performance.rs`).

## 13. Pin órfão mascarado por tolerância — checagem de pins vs JSON regenerado em `verifica-ciclo.sh` — RESOLVIDO ciclo 15

Achado da revisão da Task 3 (ciclo 12): o pin de `ldg_50ft_m` em
`tests/generic_engine.rs` ficou dessincronizado do `aircraft_spec.json`
desde o ERRATUM do ciclo 11 (commit `8f92c55`) — pin `502,431095` contra
JSON regenerado `502,458299`, desvio de 0,0054%. Sobreviveu quatro commits
sem ser pego porque 0,0054% cabe folgadamente dentro da tolerância de 1%
que os pins deste projeto usam para absorver ruído numérico legítimo
(diferenças de compilador/plataforma, convergência de laço). A MESMA
tolerância que absorve ruído numérico também absorve desatualização
silenciosa de pin — o teste continua "verde" mesmo quando o número
plantado no código já não é o que o pipeline produz.

**Proposta:** seção nova em `scripts/verifica-ciclo.sh` que regenera o
`aircraft_spec.json` (mesmo comando de regen da spec §12) e compara cada
pin conhecido (`tests/generic_engine.rs`, `tests/cli.rs`,
`tests/gear_tipback.rs`, `tests/schema_v4.rs`) contra o valor do JSON
regenerado com uma tolerância MUITO mais apertada que a dos testes
(ex.: 1e-4 relativo, não 1%) — reprovando o porteiro sempre que um pin
divergir do JSON regenerado por mais que ruído numérico de compilação
explica, mesmo quando o teste Rust correspondente continua passando dentro
de 1%. Não implementado neste ciclo (a task que descobriu o achado era de
revisão, não de execução). Ponteiro: `scripts/verifica-ciclo.sh`,
`tests/generic_engine.rs` (linha do pin `ldg_50ft_m`, histórico
`old→new` na tabela da task de pouso), commit `8f92c55` (ERRATUM ciclo
11).

**Implementado no ciclo 15**, com tolerância ZERO em vez de 1e-4 relativo
(o `aircraft_spec.json` commitado é referência suficiente — `tests/cli.rs`
já prova que bate com o pipeline exatamente — então a comparação não
precisa de tolerância nenhuma) e cobrindo TODOS os arquivos de teste, não
só os quatro nomeados na proposta original: `tests/pins_vs_json.rs`,
checagem `confere_vinculos`. Estendido também ao schema doc
(`confere_doc`), que a proposta original não cobria — ver a retratação e
os defeitos reais abaixo.

**RETRATAÇÃO (ciclo 15) — a "SEGUNDA MANIFESTAÇÃO" registrada abaixo pelo
commit `5119592` (fix wave do ciclo 14) é FALSA.** Preservada por
arqueologia — o texto original apagado deixaria de ser arqueologia —, e
substituída pela prova, pela causa e pelos defeitos reais que existem de
fato.

> **SEGUNDA MANIFESTAÇÃO, agora na DOCUMENTAÇÃO (achada pela Task 3 e
> confirmada pela revisão final de branch do ciclo 14).** O mesmo mecanismo
> apareceu num lugar que a proposta acima NÃO cobriria:
> `docs/aircraft_spec.schema.md:809-810` (histórico v5.5) registra
> `ldg_50ft_m = 582,341118 m` e `ldg_50ft_grass_m = 646,437301 m`, quando os
> valores reais imediatamente antes do ciclo 14 eram **582,521767 m** e
> **646,660942 m** (confirmado por `git show 7d246b3:aircraft_spec.json`).
> Drift introduzido em algum ponto entre os ciclos 12 e 13 — os números do
> schema doc ficaram nos valores do ciclo 12 enquanto o JSON avançou.
>
> Desvios de 0,03%: pequenos demais para saltarem aos olhos, grandes
> demais para serem ruído de compilação. **Nenhum teste guarda o schema
> doc**, então aqui não há nem a tolerância de 1% para culpar — não existe
> checagem nenhuma. A proposta acima cobre pins de TESTE contra o JSON
> regenerado; precisa ser estendida para cobrir também os números citados
> em `docs/aircraft_spec.schema.md`, ou esses números viram arqueologia
> impossível daqui a alguns ciclos.
>
> A Task 3 do ciclo 14 encontrou, documentou e **deliberadamente NÃO
> corrigiu** (fora de escopo, e corrigir sem a checagem automática só adia
> a próxima ocorrência). Ponteiro: `docs/aircraft_spec.schema.md` linhas
> 809-810 e o registro da própria Task 3 nas linhas ~926-935 e ~1433-1434.

**A prova de que é falsa.** As linhas 809-810 narram uma transição
`old→new` real e INTERNA à era v5.5 — não drift entre ciclos. Verificado
commit a commit: `619b4a0` (bump v5.5) publicava
`ldg_50ft_m = 502,4582990603992`; `e06e7e7`, ainda dentro da v5.5,
publicava `582,3411181885572` — exatamente os `582,341118` que o texto
acima chama de desatualizados. Ambos os valores EXISTIRAM de fato; as
linhas 809-810 são um registro correto de uma transição que aconteceu
dentro da mesma era de schema, não um número que ficou para trás.

O par `582,521767 / 646,660942` que o texto acima chama de "valores reais
imediatamente antes do ciclo 14" é de um momento POSTERIOR — `0a6136f`,
ainda dentro da v5.5 mas depois de `e06e7e7` — e o documento já o
registrava corretamente em OUTRO lugar, sem nenhuma correção necessária:
a entrada v5.7 na linha 981 (`582,521767 → 439,275078 m`) e a cadeia
`old→new` das linhas 1433-1434 do mesmo arquivo.

**A causa.** Comparei um valor da era v5.5 (`582,341118`, registrado nas
linhas 809-810) contra um valor pós-ciclo-13 (`582,521767`) e chamei a
diferença de deriva. Não abri o histórico commit a commit antes de afirmar
sobre ele — se tivesse, a tabela acima teria aparecido antes da
"descoberta", não depois dela.

**A lição, gêmea da do ciclo 14.** Lá afirmei uma correção que não
existia (uma nota dizendo "isso já está resolvido" que desativava uma
checagem); aqui, um defeito que não existia. As duas se curam com a mesma
disciplina: **quem afirma sobre o histórico abre o histórico.**

---

**OS DEFEITOS REAIS (ciclo 15).** Medidos contra o `aircraft_spec.json` de
`b8827e8` pelo porteiro novo (`tests/pins_vs_json.rs`, checagem 2,
`confere_doc`) e corrigidos nesta task. São CINCO, não os quatro da
retratação falsa acima — o quinto só apareceu na implementação:

| Local | O doc dizia | JSON real | Erro |
|---|---|---|---|
| `:1236` | `cg_limit_fwd_pct_mac` = 17,757974% "HOJE" | 18,268251143882534 | 2,87% |
| `:1424` | `rc_sl_ms` "Baseline real: 4,999905" | 3,460340693496421 | 44,5% |
| `:1429` | `vy_kmh` "Baseline real: 148,435393" | 167,4067945715867 | 12,8% |
| `:1601-1603` | flip `limite`/`limite_nominal` E o nome do cenário | `18,472333%` / `18,268251%`, cenário "2 pax dianteiros" | — |
| `:1437` | `ldg_air_distance_m` = 196,573247 m "no baseline real" | 196,57295565026521 | 1,48e-6 |

Os quatro primeiros pararam nos ciclos 11-12 e não acompanharam a mudança
do modelo de tração do ciclo 13; o quinto (`:1437`) é diferente — nunca foi
deriva, é **citação estimada desde a origem**: o campo está imóvel em
196,57295565026521 desde o bump v5.7 (`a1f9cc9`), o commit que criou o
campo, e `196,573247` nunca bateu com esse valor nem com nenhum outro.
Achado na implementação, registrado na spec do ciclo 15 §3.1, e é meu, do
ciclo 14 (o mesmo commit `a1f9cc9` que publicou o campo publicou a citação
errada). Ver item #24 abaixo — é a mesma classe de defeito que os pins
estimados de teste, só que em documentação.

Todos os cinco corrigidos nesta task, com marcador `<!-- PIN:... -->` no
sítio e guardados permanentemente por `confere_doc`/`confere_cobertura_doc`
em `tests/pins_vs_json.rs` — a proposta original deste item ("seção nova em
`scripts/verifica-ciclo.sh`") foi implementada, só que como teste do
próprio porteiro Rust em vez de uma seção do script. **RESOLVIDO ciclo
15.**

## 14. Achados da task 4 — recuo do limite de rotação e margem residual (registro, fix wave ciclo 12)

A task 4 (termos de solo do balanço de rotação, ver item 4/RESOLVIDO acima
sobre "≲2 pp") implementou a correção mas não registrou os próprios
achados numéricos no backlog — só no código/schema. Registro aqui, medido
no baseline real (HEAD, `aircraft_spec.json`):

- **Recuo do limite dianteiro de rotação**: `rotation_limit_pct_mac`
  **13,354637% → 17,757974% MAC (+4,403 pp)**, causado pelos dois termos
  de solo (`μ_roll·N·h_cg` e `D·(h_cg−z_drag_above_cg_m)`) somados ao
  balanço de `agents::trim_authority::rotation_available_moment_nm`.
- **Margem do cenário 'Solo (piloto)'** (`rotation_authority_margin_pct`
  em `trim.rotation_margin_per_scenario`): **+0,0011863088529595282%** de
  momento — essencialmente ZERO, equivalente a **≈0,000513 pp de %MAC**
  (≈6,4 micrômetros de posição de CG). Este é o cenário mais apertado dos
  6 e o que governa `rotation_limit_pct_mac` (o MÁXIMO por cenário).
  Confirmada ESTÁVEL: apertar a tolerância de convergência do laço de
  MTOW de 0,5 kg para 1e-9 kg muda o resultado só no 9º/10º dígito
  significativo — não é ruído de convergência disfarçado de margem real.
- **As 2 violações de robustez criadas por este ciclo** (ver item 5 desta
  lista, "prioridade ALTA", para o achado central relacionado): com a
  margem nominal de 'Solo (piloto)' praticamente em zero, o pior-caso
  direcional de robustez (`RobustnessAgent`, `±15%` nas massas
  estruturais) empurra o limite de rotação PARA CIMA no mundo perturbado
  (`limite = 18,094655% MAC` contra `limite_nominal = 17,757974% MAC`) e
  os cenários 'Solo (piloto)' e '2 pax dianteiros' FLIPAM de PASS para
  FAIL — `robustness.flips` tem 2 entradas, ambas NOVAS deste ciclo
  (`git show 1e11998:aircraft_spec.json` tinha `robustness.flips: []`).

Ponteiro: docstring de
`agents::trim_authority::rotation_available_moment_nm` (seção "TERMOS DE
SOLO", `src/agents/trim_authority.rs`), `docs/aircraft_spec.schema.md`
(bloco `trim`, parágrafo "Baseline real"), `aircraft_spec.json`
(`trim.rotation_limit_pct_mac`, `trim.rotation_margin_per_scenario`,
`robustness.flips`).

## 15. PRIORIDADE ALTA — inconsistência do modelo de tração no balanço de rotação (erro de spec, indeterminação ≈8,3 pp de MAC) — RESOLVIDO ciclo 13

Achado central da revisão final de branch do ciclo 12, erro de
especificação (não de implementação da task — a task seguiu a spec à
risca). **Não resolvido nesta wave** — resolver exige decidir qual modelo
de tração vale em `V_LOF`, o que reabre cruzeiro, teto de serviço,
alcance e autonomia (mesmo território do item 8 acima, "Unificar o modelo
de tração"). Decisão de usuário.

**O mecanismo:** a derivação de d'Alembert em
`agents::trim_authority::rotation_available_moment_nm` (ver docstring,
seção "POR QUE O BRAÇO É SOBRE O CG") cancela a porção `h_cg` do braço da
tração contra o termo inercial `m·aₓ·h_cg` **porque o mesmo símbolo `T`
aparece nos dois lados da equação de movimento da corrida**:
`m·aₓ = T − D − μN`. Essa álgebra pressupõe que o `T` do termo de momento
e o `T` (implícito em `D`/`μN`, via `aₓ`) do termo inercial sejam O MESMO
número.

A task 4 QUEBROU essa premissa: manteve no termo de momento
`T = thrust_at_rotation_n(Vr)` → `agents::performance::thrust_available_n`
(disco atuador + `prop_efficiency(J)`), enquanto `D` e `μN`, agora
explícitos no balanço (termos de solo da task 4), vêm do modelo de solo
— cujo próprio modelo de tração NA MESMA VELOCIDADE é
`agents::performance::thrust_ground_roll_n` (quantidade de movimento com
avanço), **27,69% menor** (ver item 8 acima, tabela medida em `V_LOF`).

**`Vr` e `V_LOF` são a MESMA velocidade por construção**, não uma
coincidência aproximada: `VR_OVER_VS0 = 1.1` sobre
`Vs0_TO = √(2W/(ρ·S_w·CL_max_TO))` (`trim_authority.rs:81`,
`rotation_speed_ms`) é ALGEBRICAMENTE IDÊNTICO a
`v_lof = 1.10·√(2W/(ρ·area_m2·cl_max_to))` (`performance.rs:672`,
`takeoff_ground_roll_com_passos`) — mesma fórmula, mesmos símbolos.

**Resíduo não cancelado `(T_solo − T_momento)·h_cg`, medido por
cenário** (T no momento via `thrust_at_rotation_n`; T no modelo de solo
via `thrust_ground_roll_n`; ambos em `Vr = V_LOF` do cenário):

| Cenário | T no momento (N) | T no modelo de solo (N) | Resíduo (N·m) | Resíduo (pp de MAC) |
|---|---|---|---|---|
| Solo (piloto) | 3.557,89 | 2.464,44 | −1.005,97 | **−6,816** |
| 2 pax dianteiros | 3.430,42 | 2.415,05 | −934,15 | −5,801 |
| 4 pax sem bagagem | 3.269,28 | 2.348,30 | −847,30 | −4,692 |
| 4 pax + bagagem + cheio | 3.197,46 | 2.316,94 | −810,08 | −4,255 |

**O ciclo corrigiu um erro de +4,403 pp (item 14 acima) e abriu um erro
de −6,816 pp na MESMA equação, na MESMA direção (nariz-abaixo)** — o novo
erro é MAIOR que o corrigido.

**Consequência:** `rotation_limit_pct_mac` é INDETERMINADO entre dois
extremos coerentes de modelagem:
- **≈16,28% MAC**: usar `thrust_ground_roll_n` nos DOIS termos (momento e
  solo) — resíduo ZERO por construção, mas abandona `thrust_available_n`
  no termo de momento.
- **≈24,57% MAC**: manter o polinômio `thrust_available_n` no termo de
  momento e SOMAR o resíduo faltante ao balanço de solo.

O valor PUBLICADO hoje, **17,757974%**, fica perto da ponta OTIMISTA
(menor recuo) dessa banda — não é um meio-termo, é um dos dois modelos
parcialmente aplicado. **Banda de indeterminação ≈8,3 pp de MAC**, contra
uma margem publicada no cenário mais apertado ('Solo (piloto)') de
**0,000513 pp** (item 14 acima) — a indeterminação de modelo é ~4 ordens
de grandeza MAIOR que a margem que ela deveria estar testando.

Isto FALSIFICA uma afirmação escrita neste próprio ciclo: ver `old→new`
no item 8 acima ("este ciclo não introduz inconsistência NUM cálculo" —
introduz, via a task 4).

**A resolução NÃO é desta fix wave** — escolher qual modelo de tração
vale em `V_LOF` é decisão de projeto/usuário, e reabre cruzeiro, teto,
alcance e autonomia. Ver também item 9 do JSON (`fidelity.trim`, fix wave
ciclo 12) para a qualificação de indeterminação publicada. Ponteiro:
docstring de `agents::trim_authority::rotation_available_moment_nm`
(seção "POR QUE O BRAÇO É SOBRE O CG" e "TERMOS DE SOLO"),
`agents::trim_authority::rotation_speed_ms`/`VR_OVER_VS0`
(`trim_authority.rs:81`), `agents::performance::
takeoff_ground_roll_com_passos` (`performance.rs:672`, `v_lof`), item 8
acima (`thrust_ground_roll_n` × `thrust_available_n`, divergência
medida).

**RESOLVIDO** (ciclo 13, task 4, 2026-08-15, spec
`2026-08-15-ciclo13-tracao-unificada-design.md` §6/§8.6). A decisão que
esta entrada registrava como "de usuário" — qual dos dois modelos de
tração vale em `V_LOF` — deixou de ser uma escolha: o ciclo 13 mediu que
o ramo `≈24,57% MAC` (manter o polinômio no termo de momento) mantinha,
em `Vr`, uma tração **3,72% acima do limite de conservação de quantidade
de movimento** (spec §1.2) — excluído por FÍSICA, não por preferência. O
ciclo substituiu os dois modelos por uma lei única `T(V) =
FoM(J)·T_ideal_momentum(V, P_eixo)`; `thrust_at_rotation_n` (termo de
momento) e a tração implícita nos termos de solo da rolagem passam a ser
a MESMA chamada da MESMA função nos MESMOS argumentos em `Vr ≡ V_LOF`.

**O resíduo de d'Alembert `(T_solo − T_momento)·h_cg` vai a ZERO por
construção** — medido, não assumido: teste
`tracao_do_momento_de_rotacao_e_identica_a_da_rolagem_no_mesmo_vr`
(`tests/generic_engine.rs`) dá erro relativo **0e0 (zero exato)** nos 6
cenários de CG, contra tolerância 1e-12. A banda de indeterminação de
≈8,3 pp de MAC **acabou**.

**REGISTRO DO NÚMERO CORRIGIDO** (a spec original desta task tinha um
erro de medição no achado companheiro — ver item 9 acima — que NÃO deve
se propagar aqui): o achado físico que exclui o ramo `≈24,57% MAC` não
depende da contagem "5 de 8"/"4 de 8" pontos violadores do teto de
quantidade de movimento — é a medição direta em `Vr`, isolada. Fica
registrado aqui, para que este item não repita o número errado da
primeira versão da spec: dos 8 pontos de operação nomeados na medição
original do backlog #9, **QUATRO** violam o teto físico (não cinco), e
**DOIS** deles alimentam gates que PASSAVAM — `Vx` (gradiente CS 23.65) e
**`V_LOF` (exatamente o balanço de rotação desta entrada)**. O terceiro
ponto que a primeira versão da spec contava como violador do teto
(`service_ceiling_m`, avaliado com o `Vy` de nível do mar) foi
recomputado com o `Vy` REAL da altitude do teto e `mass_mid` — razão
0,9313, **NÃO viola** — e não tem relação com o balanço de rotação em
`Vr`, então nunca fez parte do argumento físico que fecha este item.

**Valor publicado, medido no baseline real (HEAD)**: `rotation_limit_
pct_mac` (superfície de operação, GRAMA — ver item 16 abaixo) **18,268251%
MAC**; em pavimentado (informativo, `rotation_limit_pct_mac_paved`)
**16,380458%**. Nenhum dos dois cai fora do teto — ambos abaixo do valor
`≈24,57%` fisicamente excluído por §1.2 da spec, e o extremo `≈16,28%`
(resíduo zero nos dois termos com `thrust_ground_roll_n`) é
aproximadamente o valor pavimentado medido agora com a lei única
(16,380458%, pequena diferença porque a lei nova não é idêntica a
`thrust_ground_roll_n` em todo J — só coincidem em V=0). Ponteiro:
docstring de `agents::trim_authority::thrust_at_rotation_n`/
`rotation_available_moment_nm` (`old→new` do ciclo 13), teste
`tracao_do_momento_de_rotacao_e_identica_a_da_rolagem_no_mesmo_vr`
(`tests/generic_engine.rs`), spec ciclo13 §1.1 (`old→new`), §1.2, §6,
§8.6.

## 16. Assimetria de superfície entre a rotação e a decolagem/pouso (`mu_roll_paved` vs. grama, decisão de usuário) — RESOLVIDO ciclo 13

`agents::trim_authority` avalia o balanço de rotação com
`mu_roll_ground = cfg.performance.mu_roll_paved` (ver
`trim_authority.rs:809-814`), documentando a escolha como "pavimentada é
a superfície menos conservadora das duas, mesma lógica de 'menos
pessimista' já usada para o rpm de tração em `thrust_at_rotation_n`". Mas
este mesmo ciclo (item 4/RESOLVIDO acima) mediu que a pista de GRAMA de
600 m é INVIÁVEL para este baseline (`to_50ft_grass_m`/`ldg_50ft_grass_m`
excedem os 600 m — checagens #23/#24 REPROVAM).

**Medido com `μ` de grama (`mu_roll_grass = 0,08`) no lugar de
`mu_roll_paved` no balanço de rotação:**

| | Pavimentado (publicado) | Grama |
|---|---|---|
| `rotation_limit_pct_mac` | 17,7580% | **19,6458%** |
| Margem 'Solo (piloto)' | +0,0012% | **−4,3666%** (violação NOMINAL, não só sob perturbação de robustez) |

As duas metades deste ciclo descrevem operações em superfícies
DIFERENTES: a task 2/3 (pista) já trata a grama como referência de gate
(#23/#24 reprovam em grama), mas a task 4 (rotação) continua avaliando o
evento de rotação em pavimento — a MESMA decolagem, modelada com duas
superfícies distintas em dois pontos do mesmo pipeline. Se a pista real
de operação for de grama (consistente com #23/#24 já reprovando em
grama), a margem de 'Solo (piloto)' fica NEGATIVA no caso NOMINAL (sem
nem precisar da perturbação de robustez do item 15/item 5).

**Não resolvido nesta wave** — decisão de qual superfície a rotação deve
assumir (ou se deve reportar as duas) é de projeto/usuário. Ponteiro:
`agents::trim_authority` (`trim_authority.rs:809-814`, comentário
"pavimentada é a superfície menos conservadora das duas"),
`config/aircraft/baseline_4seat.toml` (`[performance].mu_roll_paved`/
`mu_roll_grass`).

**RESOLVIDO** (ciclo 13, task 4, 2026-08-15, spec
`2026-08-15-ciclo13-tracao-unificada-design.md` §7). A decisão de projeto
foi tomada: o limite dianteiro de rotação é calculado nas DUAS
superfícies e AMBAS publicadas (`trim.rotation_limit_pct_mac_paved`,
`trim.rotation_limit_pct_mac_grass`, campos novos, schema 5.6); o campo
legado `trim.rotation_limit_pct_mac` passa a valer a superfície de
OPERAÇÃO — **GRAMA** — porque é a mesma premissa que as checagens #23/#24
(decolagem/pouso) já usam para a MESMA decolagem: não existe campo
dizendo qual é a superfície, mas os gates existentes decidem por
extensão. Se essa premissa mudar, muda aqui e em #23/#24 juntos (spec
§7).

**Medido no baseline real, lei única de tração (não mais o
`thrust_available_n` que este item media originalmente)**:
`rotation_limit_pct_mac_paved` = **16,380458137686837%**,
`rotation_limit_pct_mac_grass` = **18,268251143882534%** MAC
(+1,887793 pp — mesma direção do achado original: mais atrito, mais
momento nariz-abaixo de solo, limite mais recuado; magnitude quase
idêntica, +1,888 pp medida aqui contra +1,888 pp medida no ciclo 12 com o
modelo antigo — o efeito de SUPERFÍCIE é aditivo ao efeito da unificação
de tração e praticamente ortogonal a ele, como a spec §11 previu).

**Consequência de gate, confirmada**: com a rotação em grama, a margem
'Solo (piloto)' (`rotation_authority_margin_pct`) fica **−1,179429677656323%**
— violação NOMINAL de autoridade de rotação, sem precisar de perturbação
de robustez — e o cenário cruza para violação NOMINAL de envelope de CG
(`validation_status` do baseline real inclui "Cenário 'Solo (piloto)': CG
17,8% MAC fora do envelope de CG admissível [18,3%–43,5%]" entre as 5
violações publicadas). Ponteiro: docstring de
`agents::trim_authority::TrimAuthorityAgent::run` (bloco `limite_para_
superficie`, `old→new` do ciclo 13), `aircraft_spec.json`
(`trim.rotation_limit_pct_mac_paved`/`_grass`), teste
`limite_de_rotacao_em_grama_e_mais_restritivo_que_em_pavimentado`
(`tests/generic_engine.rs`), spec ciclo13 §7/§8.6/§11.1.

## 17. Segmento aéreo do pouso em grama usa a rampa de 3° de ILS pavimentado, nunca calibrada para pista de fazenda — RESOLVIDO ciclo 14

Achado do chefe na abertura do ciclo 13 (spec
`2026-08-15-ciclo13-tracao-unificada-design.md` §12 item 1), fora de
escopo daquele ciclo — resolvido no ciclo 14 (spec
`2026-08-15-ciclo14-aproximacao-honesta-design.md`).

`agents::performance::landing_distance_50ft_m` calculava o segmento de
APROXIMAÇÃO (antes do toque) com um ângulo de descida FIXO de 3°
(`15/tan(3°) ≈ 286,2 m` para o obstáculo de 15 m/50 ft), convenção
herdada de procedimento de ILS de aeroporto PAVIMENTADO — nunca calibrada
para esta célula nem para pista de fazenda/grama.

**Dois defeitos separados, resolvidos JUNTOS** (spec §1), porque têm
naturezas diferentes e um deles não depende de premissa nenhuma:

1. **DE MODELO** (independe de premissa): o flare somava distância
   horizontal PURA (`V_ref × flare_time_s`) sem consumir altura nenhuma —
   a rampa de aproximação já descia os 15 m inteiros, então a aeronave
   "pousava duas vezes". Indefensável sob qualquer premissa de pilotagem.
2. **DE PREMISSA**: `[performance].approach_angle_deg = 3,0` era o
   *glideslope* de ILS pavimentado — nunca calibrado para pista de
   fazenda de 600 m em grama (premissa declarada do projeto).

**Corrigido** (`agents::performance::landing_air_segment`, ciclo 14 tasks
1-3): o flare vira um arco geométrico de fator de carga
`n = [performance].flare_load_factor` (baseline 1,20) que CONSOME altura
real (`h_flare = R·(1−cos γ_app)`); `γ_app` deixa de ser config livre e
passa a ser DERIVADO da polar de pouso (`atan(CD_ref/CL_ref)`, mesma
`cd_gear_extended` da rolagem) no planeio power-off, flap cheio/trem
embaixo, em `V_ref = 1,30·Vs` — **5,1181°** no baseline real, quase 71%
mais íngreme que os 3° assumidos antes. PREMISSA DECLARADA: motor em
MARCHA LENTA sobre o obstáculo (procedimento padrão de campo curto, como
o POH mede pista curta) — uma aproximação COM potência é mais RASA e
portanto mais LONGA; se a operação real usar aproximação motorizada,
este modelo é OTIMISTA, não neutro.

**Por que os dois foram JUNTOS, medido (spec §4.2)** — os dois erros
empurravam a distância para CIMA na mesma direção, então corrigir só um
deixaria o modelo certo pelo motivo errado, com os erros parcialmente se
cancelando:

| Configuração | Aéreo (m) | Pouso grama (m) | Gate 600 m |
|---|---|---|---|
| Hoje (3°, flare sem altura) | 339,82 | 646,7 | ❌ |
| **Só o conserto geométrico** (3°, flare com altura) | 303,27 | **610,1** | ❌ |
| **Só a premissa** (5,1181°, flare sem altura) | 221,08 | 527,9 | ✅ (mas geometricamente errado) |
| **Os dois (adotado)** | **196,57** | **503,4** | ✅ |

**O conserto geométrico sozinho NÃO salva o gate** (610,1 m — medido pela
Task 2 do ciclo 14, evidência de que os dois defeitos precisavam ir
juntos). A premissa sozinha salvaria, mas por um modelo ainda
geometricamente errado.

**Sensibilidade a `n` (spec §4.1, medida antes de implementar)**: o único
parâmetro novo (`flare_load_factor`), varrido de 1,10 a 1,30, move o
pouso em grama de 532,5 m a 493,7 m — **o veredito PASSA na faixa
inteira**, o flip do gate não é refém da escolha de `n`.

**Consequência de gate**: `ldg_50ft_m` **582,521767 → 439,275078 m**
(−24,60%), `ldg_50ft_grass_m` **646,660942 → 503,414253 m** (−22,17%).
Checagem #24 (pouso em grama, pista de 600 m) **FLIPA de FAIL para
PASS** — primeira violação removida desde o ciclo 11. A premissa de
pista de 600 m em grama permanece INTACTA; `validation_status` continua
`"FAIL"`, agora com 4 violações (era 5). Ponteiro:
`agents::performance::landing_air_segment` (`src/agents/performance.rs`),
`src/models/specs.rs` (`ldg_approach_angle_deg`/`ldg_flare_height_m`/
`ldg_air_distance_m`, novos campos v5.7), `docs/aircraft_spec.schema.md`
§1 (v5.7), spec ciclo14 §1/§2/§4/§6/§7.

## 18. `j_design` congelada em config não se reajusta se a velocidade de cruzeiro, `psru_ratio` ou o diâmetro da hélice mudarem

Achado da spec do ciclo 13 (§3.3 item 2), fora de escopo — a implementação
apenas declara o risco, não o mitiga.

`[propeller].j_design` (1,87514348025711675 no baseline) foi derivada de
`prop_rpm_cruise`, que HOJE é uma SAÍDA da busca de rpm de cruzeiro
(`agents::propulsion::search_cruise_rpm`). Congelá-la em config quebra
essa circularidade de propósito — a partir do ciclo 13, `j_design` é
ENTRADA de projeto da hélice, não resultado. Se a velocidade de cruzeiro
alvo, a razão de PSRU ou o diâmetro da hélice mudarem (troca de motor,
troca de hélice, nova missão), `j_design` **NÃO se reajusta sozinha** —
a âncora fica obsoleta em silêncio: o `J` real de cruzeiro se afasta de
`j_design`, `FoM(J_cruzeiro) ≠ fom_design`, e a preservação de
alcance/autonomia por construção (spec §3.2) deixa de ser exata.

**Já se manifestou, medido**: as missões com motor Rotax (fora do
baseline principal, mas exercitadas pelos testes de troca de motor)
tiveram combustível **+5,35%** e **+8,04%** em relação ao esperado, porque
o `J` de cruzeiro do Rotax cai fora da calibração desta hélice (a hélice
do baseline foi calibrada para o Toyota 1GD-FTV a 2640 rpm de motor).

**Guarda existente, mas parcial**: `rpm_de_cruzeiro_do_baseline_
permanece_2640` (`tests/generic_engine.rs`) pega o caso em que o rpm de
cruzeiro do PRÓPRIO baseline Toyota se afasta de 2640 — mas não pega
troca de hélice/motor/missão que mude `J` de cruzeiro de outra aeronave
sem que o `j_design` daquela hélice seja recalibrado. Corrigir exigiria
recalcular `j_design`/`fom_design` automaticamente a cada mudança de
config (reabre a circularidade que a spec §3.3 documentou) ou um alerta
explícito quando `J_cruzeiro` diverge de `j_design` além de um limiar.
Decisão de projeto, fora de escopo. Ponteiro: docstring de
`agents::propulsion::FigureOfMerit` (premissa calibrada declarada),
spec ciclo13 §3.3 item 2, teste `rpm_de_cruzeiro_do_baseline_permanece_
2640` (`tests/generic_engine.rs`), `tests/generic_engine.rs`
(`orchestrator_baseline_rotax_ainda_inviavel_com_tanque_260l`,
`trocar_motor_muda_resultado_sem_mudar_codigo`).

## 19. Cruzeiro opera em J=1,875, acima do pico de eficiência da hélice (J≈1,30) — ≈6% de eficiência propulsiva deixada na mesa

Achado da spec do ciclo 13 (§3.3, §12 item 6), fora de escopo — decisão de
projeto de hélice/PSRU, não de modelo de tração.

O ponto de operação de cruzeiro deste baseline é `J = j_design =
1,87514348025711675`, mas o polinômio JavaProp apagado neste ciclo tinha
pico de eficiência em `J ≈ 1,30` (`η ≈ 0,8335` contra `η(j_design) =
0,7838814965676598` — uma diferença de **≈6% de eficiência propulsiva**
disponível e não capturada no ponto de cruzeiro real).

Isso não é um defeito de MODELAGEM — a lei única de tração deste ciclo
não herda a curva de formato do polinômio, só as duas âncoras (`fom_static`
em J=0, `fom_design` em `j_design`); o pico em J≈1,30 nunca foi uma
propriedade FÍSICA desta hélice modelada explicitamente, era uma
característica do polinômio JavaProp original (calibrado para OUTRA
combinação hélice/rpm/diâmetro que gerou os dados de origem). O achado é
de PROJETO: esta combinação motor/PSRU/hélice/velocidade de cruzeiro
opera a célula fora do ponto ótimo de eficiência propulsiva da hélice
instalada — reduzir `psru_ratio` (rpm de hélice mais alta em cruzeiro,
J mais baixo) ou trocar o diâmetro/passo da hélice para deslocar o pico
de eficiência para perto de J≈1,875 recuperaria essa margem, com impacto
em consumo/alcance a avaliar. Decisão de projeto de hélice/PSRU, fora de
escopo. Ponteiro: spec ciclo13 §3.3, §3.2 (retro-derivação de
`fom_design`), `docs/backlog.md` item 18 (relacionado — mesma âncora).

## 20. `max_level_speed_ms` avalia V máx com `engine.rpm_rated` (3400) enquanto o cruzeiro opera a 2640 rpm — dois pontos de operação de motor nunca declarados

Achado da revisão de plano do ciclo 13 (spec §1.1 `old→new`), fora de
escopo — não é erro, mas nunca foi documentado.

`agents::performance::max_level_speed_ms` avalia a velocidade máxima de
nível com `engine.rpm_rated` = **3400 rpm** (potência máxima contínua do
motor), enquanto `PropulsionAgent::run`/`search_cruise_rpm` opera o
CRUZEIRO a **2640 rpm** (o argmin de BSFC entre os rpms que entregam a
potência requerida em cruzeiro). São DOIS pontos de operação de motor
diferentes coexistindo no mesmo `aircraft_spec.json`
(`propulsion.engine_rpm_cruise` = 2640, `performance.v_cruise_kmh`
implicitamente calculado a 3400) — a divergência só ficou visível porque
a revisão de plano do ciclo 13 recomputou a razão de avanço (`J`) nos
dois pontos separadamente para medir a tabela do §1.1 (`J=1,5611` em V
máx contra `J=1,875` em cruzeiro).

**Não é erro** — V máx por definição é avaliada em potência MÁXIMA do
motor, e cruzeiro é avaliado no rpm mais econômico que atende a potência
requerida; são perguntas diferentes com respostas em rpms diferentes por
natureza. Mas o JSON nunca declara explicitamente que os dois blocos
(`performance`/`propulsion`) usam rpms de motor distintos — um consumidor
de CAD/estrutura que assumisse "um único ponto de operação de motor" leria
os dois campos como coerentes entre si sem sê-lo. Documentar (campo novo
explícito, ou nota em `fidelity`) ou reconciliar (padronizar em um único
rpm de referência para ambos, com trade-off de precisão) é decisão de
projeto. Ponteiro: `agents::performance::max_level_speed_ms`
(`src/agents/performance.rs:1020`, uso de `engine.rpm_rated`),
`agents::propulsion::search_cruise_rpm` (uso do rpm buscado), spec
ciclo13 §1.1 (`old→new`, tabela de medição).

## 21. Âncora de cruzeiro preserva o PONTO exatamente, mas `range_km`/`endurance_h` residuam −0,037% via o laço de convergência de MTOW — RESOLVIDO ciclo 16

Achado medido na Task 5 do ciclo 13, fora de escopo de correção — resíduo
declarado, não corrigido (spec ciclo13 §3.2 nota "a âncora tem uma
dependência que ela não controla"; achado companheiro registrado aqui com
a medição completa).

A retro-derivação de `fom_design` (spec §3.2/§3.2.1, corrigida por
ERRATUM e convergida por ponto fixo) garante que a lei nova reproduz,
EXATAMENTE, a eficiência que o polinômio JavaProp apagado entregava NO
PONTO de cruzeiro do baseline — verificado a 1e-9 pelo teste
`eficiencia_de_cruzeiro_reproduz_a_ancora_do_polinomio_apagado`
(`tests/generic_engine.rs`). Isso preserva `prop_efficiency`,
`thrust_cruise_n`, `p_req_cruise_kw`/`p_shaft_cruise_kw` no PONTO de
cruzeiro por construção.

**Mas a promessa correta é "cruzeiro preservado por construção; MISSÃO
preservada a menos do laço de convergência"**, não "missão inalterada".
Medido no baseline real, `ed537ae` → HEAD:

| Campo | `ed537ae` | HEAD (ciclo 13) | Δ |
|---|---|---|---|
| `range_km` | 2027,070681 | 2026,312721 | **−0,0374%** |
| `endurance_h` | 7,239538 | 7,236831 | **−0,0374%** |
| `fuel_total_kg` | 198,269071 | 199,212369 | **+0,4758%** |
| `mtow_mission_kg` (sizing) | — | 1538,332304 | (+0,0614% sobre a iteração 10 anterior) |

**Causa**: o segmento de SUBIDA da missão (`agents::mission`,
`fuel_climb_kg`) usa a curva `FoM(J)` INTEIRA ao longo do perfil de
subida, não só o ponto J=`j_design` da âncora — e a FORMA dessa curva
(linear entre `fom_static` e `fom_design`) difere da forma do polinômio
JavaProp fora do ponto de projeto. Isso muda o combustível de SUBIDA
(`fuel_climb_kg`: **4,920084 → 7,198478 kg**, **+46,31%** — o efeito
DOMINANTE, não o de cruzeiro), que muda a massa convergida (laço de
ponto fixo de MTOW), que muda o arrasto, que muda a tração requerida em
cruzeiro — um resíduo de segunda ordem que retroalimenta o próprio ponto
que a âncora preserva.

**Não corrigido** — corrigir exigiria uma âncora de subida separada (fora
do escopo da spec §3.2, que só ancora CRUZEIRO) ou aceitar que "cruzeiro
preservado" nunca implicou "missão preservada" quando o modelo de tração
muda em qualquer OUTRO ponto do perfil de voo. Registrado como resíduo
MEDIDO e DECLARADO, não como bug. Ponteiro: spec ciclo13 §3.2 (nota
final), `agents::mission::MissionAgent::run` (`fuel_climb_kg`, consome
`state.figure_of_merit()` via `climb_rate_ms`/`excess_power_kw`),
`tests/generic_engine.rs` (pins `mission.*`, `sizing.*`, `old→new`
ciclo 13).

**RESOLVIDO** (ciclo 16, ciclo16-veredito-indeterminado, spec
`2026-08-16-ciclo16-veredito-indeterminado-design.md`). "Resolvido" aqui
**não** quer dizer "o resíduo de −0,037% desapareceu" — ele não desapareceu,
e não podia: nenhum número de física mudou neste ciclo (`fom_static`
continua 0,75, a lei `FoM(J)` continua a mesma). O que este ciclo ataca é a
CAUSA do item, não o sintoma: a assimetria descrita acima (cruzeiro
saturado em `fom_design`, calibrado; subida na região LINEAR da curva,
dependendo do parâmetro CRU `fom_static`) significa que qualquer veredito
avaliado perto de J baixo (a CS 23.65 é avaliada em Vx, J≈0,82) carrega até
56,3% de peso de um parâmetro que NUNCA foi calibrado
(`∂FoM/∂fom_static = 1 − min(J/j_design,1)`, ver spec §1) — e o modelo, até
este ciclo, publicava esse veredito com a mesma confiança de um check
totalmente determinado.

O ciclo publica a banda de incerteza declarada de `fom_static` (bloco novo
`uncertainty`, `docs/aircraft_spec.schema.md` §4) e um TERCEIRO estado de
veredito, `INDETERMINADO`, para todo check cujo resultado VIRA dentro dessa
banda. No baseline real, é exatamente o gradiente da CS 23.65 (o achado
original do #21, medido em §2.3 da spec): breakeven em `fom_static ≈
0,7849`, +4,6% sobre o nominal — o modelo agora DIZ que não sabe, em vez de
carimbar FAIL com quinze dígitos apoiados num fator herdado de McCormick
com dois algarismos significativos. `range_km`/`endurance_h` continuam
residuando −0,037% (não corrigido, não é o que este ciclo se propôs a
corrigir — ver acima) e `validation_status` do baseline real CONTINUA
`"FAIL"` (as outras 3 violações são determinadas). O que mudou é que o
modelo agora consegue DISTINGUIR "meu avião não atende" (FALHA) de "meu
modelo não sabe" (INDETERMINADO) — que é a promessa do título do ciclo.
Ponteiro: `validation::incerteza` (`src/validation/incerteza.rs`),
`UncertaintySpec` (`src/models/specs.rs`), `docs/aircraft_spec.schema.md`
§1 (entrada v6.0) e §4 (bloco `uncertainty`).

## 22. Nome do teste `envelope_de_cg_fechado_sem_violacao_no_baseline_real` ficou incoerente com o conteúdo (agora 1 violação de envelope)

Achado nomeado pela própria Task 4 do ciclo 12 e mantido pela Task 4 do
ciclo 13 — registrado aqui para consolidar o ponteiro no backlog.

O teste `envelope_de_cg_fechado_sem_violacao_no_baseline_real`
(`src/validation/constraint_checker.rs`, módulo `tests`) verifica que o
ENVELOPE de CG do baseline real é uma FAIXA fechada (limite dianteiro <
limite traseiro — não o caso degenerado "envelope vazio"), não que não
haja NENHUMA violação de CG. Desde o ciclo 12 (task 4, termos de solo do
balanço de rotação) o baseline real passou a ter exatamente **1 violação**
de envelope de CG (cenário 'Solo (piloto)', hoje NOMINAL desde o ciclo 13
— ver item 16 acima), então o nome do teste — que promete "sem violação"
— ficou parcialmente incoerente com o que o teste de fato verifica
(envelope FECHADO, questão diferente de "sem violação").

A Task 4 do ciclo 12 (e a Task 4 do ciclo 13, que tocou o mesmo balanço)
mantiveram o nome por **estabilidade de referências cruzadas** — o nome
do teste é citado por docstring/comentário em pelo menos dois outros
arquivos (`src/models/engine.rs:154`, `tests/empennage.rs:111`) — e
documentaram a incoerência DIRETAMENTE no próprio teste (bloco `old→new`
na docstring, `src/validation/constraint_checker.rs`, linhas ~951–961:
"o nome deste teste fica parcialmente impreciso, mantido por continuidade
histórica... 'Sem violação' deixa de ser literalmente verdadeiro para o
baseline completo; continua verdadeiro para a pergunta original do teste
(envelope fechado vs. vazio)").

**Não corrigido** — renomear o teste (e todas as referências cruzadas que
o citam pelo nome) é escopo maior que uma fix wave de task, decisão de
limpeza para um ciclo dedicado a housekeeping de testes. Ponteiro:
`src/validation/constraint_checker.rs` (teste e docstring `old→new`,
linhas ~940–979), `src/models/engine.rs:154`, `tests/empennage.rs:111`
(referências cruzadas pelo nome).

## 23. Rolagem de pouso integra a partir de `V_ref`, mas o flare sangra velocidade até ≈1,15·Vs antes do toque — direção CONSERVADORA, não medida

Achado nomeado pela spec do ciclo 14 (§3, "O que NÃO muda") — fora de
escopo daquele ciclo por decisão explícita de projeto, registrado aqui.

`agents::performance::landing_ground_roll_m` integra a rolagem de solo do
pouso a partir de `V_ref = 1,30·Vs` (35,7351 m/s no baseline real) — a
mesma velocidade de referência que `landing_air_segment` usa para
derivar `γ_app`/`R`/`h_flare`/`s_flare` (ciclo 14, item 17 acima). Mas na
física real do flare, a aeronave DESACELERA ao longo do arco antes de
tocar o solo — o modelo assume `V_ref` CONSTANTE durante o flare (mesma
aproximação declarada da spec §2.1, ver item 17 acima), e o toque
efetivo ocorre perto de `1,15·Vs` (≈31,61 m/s no baseline real, contra os
35,74 m/s de `V_ref` que a rolagem usa como velocidade INICIAL).

**Direção do erro nomeada, magnitude NÃO medida neste ciclo**: integrar a
rolagem a partir de uma velocidade INICIAL maior que a velocidade real de
toque superestima a energia cinética a dissipar — a rolagem real
(portanto `ldg_50ft_m`/`ldg_50ft_grass_m`) é MENOR que a calculada.
Integrar a partir de `V_ref` é **CONSERVADOR**, mantido de propósito no
ciclo 14 (spec §3): corrigir exigiria decidir a velocidade de toque real
(função do próprio `n = flare_load_factor` e de `γ_app`, não uma
constante como `1,15·Vs` sugere) e propagá-la como condição inicial da
integração de `landing_ground_roll_m` — mudança de acoplamento entre o
segmento aéreo e o de solo, fora do escopo de uma correção pontual.

Quantificar exigiria: (1) derivar a velocidade de toque a partir da
cinemática do flare (`V_toque` em função de `V_ref`, `n`, `γ_app` — não
necessariamente `1,15·Vs` fixo, que é só uma aproximação típica citada na
spec); (2) trocar a condição inicial de `landing_ground_roll_m` de
`V_ref` para `V_toque`; (3) medir o quanto `ldg_50ft_m`/`ldg_50ft_grass_m`
encolhem. Ponteiro: `agents::performance::landing_ground_roll_m`,
`agents::performance::landing_air_segment` (`src/agents/performance.rs`),
spec `2026-08-15-ciclo14-aproximacao-honesta-design.md` §3.

## 24. Pins estimados — terceira variante da doença do #13 — RESOLVIDO ciclo 15

Achado da spec do ciclo 15 §7.4, com medição completa por commit. Dois
pins de `tests/vn_diagram.rs` não batiam com o `aircraft_spec.json` — e,
diferente do #13 original, **não é deriva**:

| commit | `va_kmh` | `n_gust_vc` |
|---|---|---|
| `8f92c55` (ERRATUM ciclo 11) | 242,618735 | 3,572607 |
| `ed537ae` (pré-ciclo 13) | 242,618735 | 3,572607 |
| `7d246b3` (pós-ciclo 13) | **242,692244** | 3,572607 |
| `b8827e8` (hoje) | 242,692244 | 3,572607 |

`n_gust_vc` está **imóvel em 3,572607** por toda a janela — o pin `3.59`
nunca bateu, em commit nenhum, desde que a janela começa. `va_kmh` mudou
uma vez, no ciclo 13, mas o pin `242.633` não batia nem com o valor antigo
(242,618735) nem com o novo (242,692244) — não existe momento no histórico
em que o pin correspondesse ao pipeline.

**Nomeando a classe: pin estimado.** É a terceira variante da doença do
#13, e a pior das três. Um pin **envelhecido** (#13 original) ao menos
testemunha um estado que existiu — o pipeline produziu aquele número em
algum momento e o mundo mudou depois. Um pin **estimado** não testemunha
nada: é um número escrito a olho, dentro de uma tolerância larga o
bastante para nunca cobrar a conta, ocupando o lugar de um número que
deveria vir do pipeline.

O quinto defeito do schema doc corrigido no Passo 0 deste ciclo
(`docs/aircraft_spec.schema.md:1437`, `ldg_air_distance_m`, spec §3.1) é a
MESMA classe, só que em documentação em vez de teste: `196,573247` estava
imóvel desde o bump v5.7 (`a1f9cc9`, o commit que criou o campo) e nunca
bateu com o `196,572956` publicado.

**Correção**: os dois únicos literais autorizados a mudar neste ciclo
(spec §7.4, §10.7) — `tests/vn_diagram.rs:93` `242.633 → 242.692244` e
`:105` `3.59 → 3.572607`, cada um com `old→new` comentado no sítio e
tolerâncias (`abs < 1.0`, `abs < 0.05`) **inalteradas**. **RESOLVIDO ciclo
15.**

## 25. Cinco pins nunca verificados por nada, ausentes do inventário original — RESOLVIDO ciclo 15

Achado da spec do ciclo 15 §7.5/§7.5.1, ao reimplementar o inventário de
pins a partir do scanner (`tests/pins_vs_json.rs`) em vez de por leitura
manual. Cinco pins reais, todos corretos, apareceram nesta varredura que
não estavam no inventário levantado por leitura:
`control_surfaces.rs:44` (`2.0895` → `control_surfaces.aileron.span_m`),
`:45` (`1.0304` → `control_surfaces.aileron.area_m2`), `:138` (`0.0` →
`control_surfaces.elevator.start_m`), `generic_engine.rs:2600` (`2640.0`
→ `propulsion.engine_rpm_cruise`) e `propeller.rs:57` (`1.76` →
`propeller.diameter_m`).

Nenhum destes cinco é errado — todos batem, na precisão em que estão
escritos, com o `aircraft_spec.json` de `b8827e8`. O problema não é o
valor: é que **nada, em ciclo nenhum anterior a este, os conferia contra
coisa alguma.** Um pin que bate por sorte e um pin verificado que bate são
indistinguíveis a olho nu; só a checagem automática separa os dois, e até
este ciclo essa checagem não existia.

**Causa:** o inventário original (spec do ciclo 15, §7.1/§7.2, hoje
SUPERSEDIDAS pelo ERRATUM §7.5) foi levantado por leitura do código antes
de o scanner existir — e leitura manual erra por amostragem, não por
regra. Os mesmos cinco sítios que a leitura perdeu são exatamente os que a
regra automática (assert OU ≥4 casas) encontra sem esforço.

**Regra que sai daí:** um inventário de cobertura que não vem do próprio
verificador é palpite bem apresentado. Um inventário correto não é lido
para dentro de uma tabela e depois congelado — ele é a SAÍDA de rodar a
regra, recalculado sempre que a regra roda.

**Correção:** os cinco receberam marcador `// PIN:` nesta task, e agora
são conferidos por `confere_vinculos` a cada `cargo test`. Ganho líquido de
cobertura, sem mudança de valor. **RESOLVIDO ciclo 15.**

**Nota (fix wave, achada pela revisão final de branch):** a citação
`generic_engine.rs:2587` acima estava errada — a linha real é `2600`
(`assert_eq!(sized.prop.engine_rpm_cruise, 2640.0); // PIN:
propulsion.engine_rpm_cruise`). Valor e caminho sempre estiveram certos;
só o número de linha era falso. Isto é um erro de número de linha
**dentro do item que documenta erros de número de linha** (o mesmo
padrão do item #29 — leitura manual em vez de derivação pela regra). A
ironia não é anedota, é dado: nomear o hábito no item #29 não o curou
dentro do próprio item que o nomeia. Corrigido nesta fix wave.

## 26. Lacuna residual do cadeado — literal curto fora de `assert` escapa das duas regras (DECLARADO, sem correção)

Copiado da spec do ciclo 15 §9 item 1. A regra de cobrança de
`tests/pins_vs_json.rs` (`cobrados`) marca um literal como exigindo `//
PIN:` quando a linha contém `assert` OU o literal tem ≥4 casas decimais —
união deliberada, porque um piso só de casas deixaria passar `3.59` e
`242.633` (item #24 acima) e um piso só de `assert` deixaria passar a
tabela de tuplas de `generic_engine.rs:1735-1742`.

A união cobre os casos reais deste ciclo, mas não é completa: um literal
**fora** de uma linha de `assert` **e** com ≤3 casas decimais escapa das
duas regras ao mesmo tempo. A própria tabela de tuplas de
`generic_engine.rs:1734-1749` só é coberta hoje porque os oito literais
daquela tabela têm ≥4 casas — o `assert` está no laço que itera a tabela,
não na linha de cada literal. **Um pin novo escrito como entrada de tupla
com poucas casas decimais passaria pelo cadeado sem marcador nenhum.**

**Não corrigido.** Fechar esta lacuna sintaticamente exigiria análise
semântica de fluxo (rastrear se o literal alimenta, direta ou
indiretamente, uma comparação sob tolerância) que o motor deste ciclo —
funções puras sobre texto mascarado, sem parser de Rust — não faz por
desenho (spec §4). Lacuna CONHECIDA e DECLARADA. Ponteiro:
`tests/pins_vs_json.rs` (`fn cobrados`), `generic_engine.rs:1734-1749`,
spec do ciclo 15 §9 item 1.

## 27. Legibilidade de `docs/aircraft_spec.schema.md:1431-1432` — número histórico dentro de célula rotulada, fácil de ler como atual

`climb_gradient_pct` aparece citado como `12,451842%` (valor hoje é
`7,913277%`) dentro de uma narrativa da tabela de campos de `performance`,
explicitamente rotulada "ciclo 11" e "Histórico E10". **Não é defeito**: o
texto não reivindica que `12,451842%` seja o valor vigente, e por isso o
marcador `<!-- PIN:... -->` da checagem deste ciclo (que só se aplica a
afirmações de atualidade — gatilhos `HOJE`/`Baseline real`/`valor
publicado`, spec §5.6) corretamente NÃO se aplica ali.

O problema é de leitura, não de correção: quem lê a célula isolada, sem
acompanhar a narrativa completa da linha, pode concluir que 12,451842% é
o número de hoje — a célula é densa (uma linha só narra ciclos 11 e 13
juntos) e o rótulo "ciclo 11" fica distante do número, várias frases antes
dele.

**Não corrigido neste ciclo** — é item de legibilidade de prosa, não de
exatidão numérica, e está fora do escopo de um ciclo que só toca números
que afirmam ser o valor atual. Ponteiro: `docs/aircraft_spec.schema.md`
linhas 1431-1432, spec do ciclo 15 §3.1 (nota de escopo) e §9 item 5.

## 28. Interação entre os dois mecanismos de cobrança — a redundância esconde a falha de um deles (DECLARADO, sem correção)

Achado da revisão da Task 2 (ciclo 15), registrado no adendo da spec §7.5.1.
`confere_vinculos` filtra candidatos por `cobrados()` (assert OU ≥4 casas)
antes de tentar vincular um marcador — exceto quando `cobrados()` vem
vazio, caso em que cai num fallback para `literais()` puro, criado para
permitir o marcador voluntário de `empennage.rs:41` (3 casas, sem
`assert`, item #25 acima descreve o inventário que este marcador ajuda a
fechar).

Esse fallback tem um efeito colateral não previsto: com a fronteira de
cobrança mutada experimentalmente de `>= 4` para `> 4` (mutação usada
para provar que a fronteira importa), os pins de **4 casas** de
`control_surfaces.rs:44-45` deveriam deixar de ser cobrados — e
deveriam, portanto, reprovar `todo_literal_cobrado_em_teste_carrega_marcador`
se a cobertura dependesse só da regra de cobrança. Mas eles **continuam
passando**, porque, não sendo mais cobrados, caem no MESMO fallback que
existe para o marcador voluntário e são achados vinculados por ali.

**Os dois mecanismos se cobrem.** A proteção da fronteira de 4 casas — o
comportamento que distingue `cobrados()` de `literais()` e que a spec
descreve como semântica, não tipográfica — ficou inteiramente concentrada
num único teste unitário, `fronteira_de_quatro_casas_e_cobrada`. **Quem
apagar esse teste não verá nada quebrar**: nem `pins_de_teste_batem_com_o_json_commitado`
nem `todo_literal_cobrado_em_teste_carrega_marcador` acusam a mutação,
porque o fallback absorve o efeito.

**Lição:** redundância entre mecanismos parece robustez; quando ela
**esconde** a falha de um deles, é o contrário. Dois componentes que se
cobrem só aumentam a segurança enquanto se sabe que ambos ainda
funcionam — no momento em que um deixa de funcionar em silêncio, a
redundância vira o motivo de ninguém notar.

**Não corrigido.** Removê-la exigiria ou desacoplar o fallback do
marcador voluntário de uma checagem de fronteira separada, ou aceitar
que a fronteira de 4 casas só é garantida enquanto
`fronteira_de_quatro_casas_e_cobrada` existir e passar — o que já é
verdade hoje, só que implicitamente. Lacuna DECLARADA. Ponteiro:
`tests/pins_vs_json.rs` (`fn confere_vinculos`, teste
`fronteira_de_quatro_casas_e_cobrada`), spec do ciclo 15 §7.5.1 (adendo).

## 29. O hábito do chefe — uma lista publicada ao lado de uma regra correta, mas derivada por outro método (item de processo)

Registro de processo, não de código. O ciclo 15 expôs, de forma
independente, o mesmo padrão quatro vezes: a spec enunciava a regra
corretamente e depois publicava uma lista, uma contagem ou uma permissão
**derivada por outro método** — leitura manual, intuição, extrapolação —
em vez de derivada RODANDO a própria regra. As quatro ocorrências, cada
uma achada por revisão ou por implementação, nenhuma pelo autor no
momento de escrever:

(i) o inventário de pins das §7.1-§7.3 foi levantado por leitura do
código, não pelo scanner — resultou em 11 sítios sem classificação e
linhas erradas em até 143 linhas, corrigido só pelo ERRATUM §7.5 (ver
também item #25 acima, os cinco pins que a leitura perdeu);

(ii) a permissão de marcador voluntário da §7.5.1, para `empennage.rs:41`,
foi escrita como se fosse exercível — mas `confere_vinculos` filtrava os
candidatos por `cobrados()` antes de tentar vincular, o que tornava a
permissão IMPOSSÍVEL de exercer tal como especificada; corrigida com o
fallback descrito no item #28 acima;

(iii) a lista de gatilhos de atualidade da §5.6 previa nove padrões; a
implementação achou oito reais no texto, com três falsos positivos (padrões
que a lista previa mas que não ocorrem, ou não do jeito descrito) e dois
falsos negativos (o gatilho minúsculo "no baseline real" que o quinto
defeito do doc usa, item #24 acima, não estava na lista de gatilhos em
maiúscula `Baseline real`);

(iv) `docs/aircraft_spec.schema.md:1437` foi listado na §5.4 como "citação
verificada como correta, recebe marcador sem alteração de valor" — mas
nunca tinha sido verificado contra coisa alguma; a citação de
`196,573247` nunca bateu com o `196,572956` publicado (item #24 acima).

**Regra que sai disso:** uma lista publicada ao lado de uma regra deve
ser a SAÍDA da regra, nunca uma paráfrase dela. Toda vez que uma spec
escreve "a regra é X" e, no parágrafo seguinte, "portanto os sítios são
[lista]", a lista precisa ter vindo de rodar X — não de reconstruir X de
memória sobre o texto. Uma paráfrase engana pela aparência de derivação
sem carregar a garantia de tê-la. Item de processo, sem correção de
código associada — a correção é o hábito, aplicável aos próximos ciclos.

## 30. A fronteira de garantia do `pins_vs_json.rs` — o que o porteiro prova e o que ele NÃO prova

Achado mais importante da revisão final de branch do ciclo 15. Uma
ferramenta que dá mais segurança aparente do que real é exatamente o
defeito que este ciclo existe para combater (item #13 e suas três
variantes, itens #24/#25 acima) — deixar essa fronteira sem registro
repetiria o erro em escala maior, desta vez no próprio porteiro. As
quatro medições abaixo foram feitas executando as funções REAIS de
`tests/pins_vs_json.rs` contra o `aircraft_spec.json` commitado, não por
leitura do código.

**(a) A razão de uma isenção não é validada por nada.**
`interpreta_marcador` captura o texto depois de `NAO-PUBLICADO` como
`String` livre, sem checagem de formato ou de conteúdo. Silenciar um pin
genuíno custa **uma edição de uma linha**: trocando `// PIN:
vn_diagram.va_kmh` por `// PIN: NAO-PUBLICADO — valor ainda não
confirmado no manual de voo`, o contador de vinculados caiu de 48 para
47 e **só o piso** (`MINIMO_DE_PINS_VINCULADOS`) reprovou — nenhuma
outra checagem notou a isenção falsa.

**(b) A isenção de módulo silencia um arquivo inteiro por uma linha.**
`tests/generic_engine.rs` tem 23 marcadores vinculados. Isentando o
módulo inteiro, o contador caiu de 48 para 25 e a checagem do piso
reprovou sozinha — **o piso funciona contra a versão ingênua do
ataque.**

**(c) Mas o piso é uma CONTAGEM, e contagem é falsificável por
padding.** `MINIMO_DE_PINS_VINCULADOS = 48` está exatamente no valor de
hoje: **folga zero**. A revisão acrescentou 23 linhas triviais e sem
sentido do tipo `let _c15_bulk_N = 350.0; // PIN: vn_diagram.vd_kmh` em
outro arquivo; o contador voltou a 48, `cargo test --release` passou
**limpo com 553 testes**, e os 23 pins reais de `generic_engine.rs`
ficaram permanentemente fora de verificação sem nenhum teste do
repositório acusar. Custo do ataque completo: 1 linha de isenção de
módulo + ~23 linhas de padding mecânico, sem exigir conhecimento nenhum
do domínio da aeronave.

**(d) Caminho semanticamente errado com valor coincidente é
indetectável por desenho.** O verificador compara número contra
caminho; nunca verifica se o caminho citado é o campo que o código ao
redor de fato exercita. Colisões reais no `aircraft_spec.json` de hoje,
achadas por varredura sistemática: `empennage.ar_v` e
`structure.skin_min_thickness_mm` valem **ambos exatamente 1,5** —
razão de aspecto adimensional e espessura em mm, fisicamente sem
relação nenhuma entre si. O mesmo literal `1.5` passa marcado
igualmente com qualquer uma das duas strings de caminho. Outras
colisões: `empennage.taper_h` / `empennage.taper_v` /
`trim.cl_ground_rotation` = 0,5 (três campos fisicamente distintos, um
valor); `propulsion.fuel_capacity_l` / `sizing.fuel_capacity_l` = 260,0
(esta por construção real do modelo, não coincidência, mas ainda assim
indistinguível de coincidência pelo verificador).

**A fronteira, em prosa clara:** o porteiro prova, com certeza, que **um
número escrito no código corresponde ao número publicado no caminho
JSON que o marcador daquela linha cita**, e pune com precisão real
qualquer deriva, pin envelhecido ou pin estimado dentro desse conjunto —
é exatamente o que resolveu os itens #13, #24 e #25. Ele **não** prova
que a razão de uma isenção é verdadeira, que o caminho citado é o
semanticamente certo para o valor ao lado, nem que a contagem de
vínculos reflete cobertura de conteúdo — ela reflete contagem, e
contagem é falsificável sem nenhum custo de conhecimento de domínio.

**A garantia opera sob boa-fé.** Ele transforma erro honesto — que era
a causa das três variantes da doença do #13 — em falha alta e imediata
de `cargo test`. Não foi desenhado para resistir a alguém que queira
silenciá-lo deliberadamente, e as medições (a)-(c) acima mostram que,
de fato, não resiste.

**Lacuna DECLARADA, sem correção neste ciclo.** Duas direções óbvias de
endurecimento futuro, registradas sem implementar:

1. Dar folga ao piso e torná-lo **derivado** em vez de literal — hoje
   `MINIMO_DE_PINS_VINCULADOS = 48` é um número escrito à mão que por
   coincidência bate com a contagem de hoje (folga zero); um piso
   calculado a partir de uma fonte independente do próprio contador
   fecharia o ataque (c).
2. Exigir que o marcador cite, além do caminho JSON, o **nome do campo
   Rust** que a linha exercita (ex.: `// PIN: propulsion.engine_rpm_cruise
   AS sized.prop.engine_rpm_cruise`), para que caminho e uso possam ser
   confrontados automaticamente — fecharia o ataque (d).

Ponteiro: `tests/pins_vs_json.rs` (`fn interpreta_marcador`, `fn
confere_vinculos`, `const MINIMO_DE_PINS_VINCULADOS`), revisão final de
branch do ciclo 15 (aprovada para merge com este item como
não-bloqueante).

## 31. Calibrar `fom_static` por elemento de pá / JavaProp em J=0 — substituir a banda DECLARADA por banda MEDIDA

A banda de incerteza publicada em `uncertainty` (ciclo 16, ver item #21
acima) é **política de projeto** sobre uma entrada não calibrada
(`fom_static_tol_pct = 10%`), não uma medição de hélice. Ninguém mediu
hélice nenhuma no ciclo 16 — o que fecharia isto de verdade é análise de
elemento de pá (BEMT) ou uma corrida JavaProp em J=0, produzindo uma banda
MEDIDA (com sua própria incerteza de medição) em vez de uma banda
DECLARADA por confiança subjetiva.

Mitigação parcial já em vigor: o breakeven publicado (`uncertainty.checks[].
breakeven_lo`/`breakeven_hi`) é FATO medido do modelo — mesmo que a banda
declarada esteja errada, o número do breakeven continua certo; a banda só
escolhe a PALAVRA (`FALHA` determinada vs. `INDETERMINADO`) que se aplica a
ele. Ponteiro: spec `2026-08-16-ciclo16-veredito-indeterminado-design.md`
§9, item 1; `config/aircraft/baseline_4seat.toml` (bloco de proveniência de
`propeller.fom_static`).

## 32. Varrer as demais entradas declaradamente não validadas do baseline

O bloco `uncertainty` (ciclo 16) varre UM parâmetro: `propeller.fom_static`.
A maquinaria (`validation::incerteza::analisa`) é genérica — reexecuta o
pipeline com um campo de config perturbado e classifica cada check — mas a
APLICAÇÃO é de um parâmetro só. O baseline tem outras entradas
declaradamente não validadas que NUNCA são varridas por este mecanismo:
`ground_clearance_min_m` ("PROXY de projeto conservador"),
`prop_plane_x_m` ("ESTIMATIVA de geometria — validar no CAD"), entre
outras (ver `config/aircraft/baseline_4seat.toml`, comentários de
proveniência). `uncertainty.parameter` nomeia explicitamente qual entrada
foi varrida exatamente para NÃO ser lido como "o resto do JSON é certo" —
mas ninguém garante que um consumidor lê o nome do campo antes de assumir
isso. Ponteiro: spec ciclo16 §9, item 2.

## 33. Incoerência de idioma no domínio de `validation_status` (`"PASS"`/`"FAIL"` em inglês, `"INDETERMINADO"` em português)

Ciclo 16 (Task 5): o terceiro valor do domínio de `validation_status` foi
adicionado em PORTUGUÊS (`"INDETERMINADO"`) ao lado de dois valores em
INGLÊS (`"PASS"`/`"FAIL"`) herdados desde a v4.0. Decisão consciente,
registrada e não corrigida às escondidas — `"INDETERMINADO"` é o termo do
contrato acordado com o usuário nesta spec, e os TEXTOS de violação do
projeto (`violations[]`) já são todos em português; inglês só nos dois
literais herdados do enum original. Corrigir exigiria escolher entre (a)
traduzir `"PASS"`/`"FAIL"` para português (quebra MAJOR, todo consumidor
que compara string precisa mudar) ou (b) traduzir `"INDETERMINADO"` para
inglês (perde a aderência ao vocabulário desta spec e do resto do
projeto). Nenhuma das duas foi decidida — fica aberto. Ponteiro: spec
ciclo16 §5.5, `docs/aircraft_spec.schema.md` §1 (entrada v6.0).

## 34. Interações entre incertezas — banda multidimensional não explorada

`uncertainty` varre `propeller.fom_static` SOZINHO, um parâmetro por vez.
Duas entradas, cada uma dentro da sua própria banda declarada, podem
combinar-se para fazer um check VIRAR que nenhuma das duas vira sozinha —
esse efeito de interação não é explorado nem detectado por este mecanismo.
Fechar isto exigiria variar múltiplos parâmetros simultaneamente (custo
combinatório: `2^n` extremos para `n` parâmetros incertos, mais o teto),
fora de escopo do ciclo 16. Ponteiro: spec ciclo16 §9, item 3.

## 35. Achado de PROJETO (não de modelo): mais tração estática piora o limite dianteiro de CG e a robustez do cenário '2 pax dianteiros'

Medido durante o ciclo 16 (varredura da banda de `fom_static`,
`tests/incerteza.rs::cg_e_robustez_falham_ate_no_teto`): o check de
envelope de CG do cenário 'Solo (piloto)' e o flip de robustez de '2 pax
dianteiros' FALHAM em TODOS os quatro pontos avaliados, incluindo o TETO
de quantidade de movimento (`fom_static`/`fom_design` ambos em 1,0) —
`alcance_de_helice: false` nos dois. Ou seja: **nenhuma hélice, por melhor
que seja, resolve estas duas violações** — mais tração PIORA as duas (o
limite dianteiro do envelope é de ROTAÇÃO, que carrega um termo de momento
`−T·prop_axis_above_cg_m`; o balanço de robustez do cenário dianteiro
herda o mesmo limite). Isto não é um achado sobre o MODELO de tração — é
um achado sobre o PROJETO da aeronave: a linha de tração está posicionada
de um jeito que trocar de hélice/motor para um mais potente não vai
consertar o envelope de CG dianteiro nem a robustez de '2 pax dianteiros'.
Merece investigação de projeto própria (reposicionar `prop_axis_above_cg_m`,
recuar o limite dianteiro por outro caminho, ou aceitar a restrição de
carregamento) — fora de escopo de um ciclo de modelagem. Ponteiro:
`agents::trim_authority` (limite dianteiro de rotação),
`validation::robustness` (cenário '2 pax dianteiros'), spec ciclo16 §12.

## 36. Esclarecimento de documentação: `violations: []` NÃO implica `validation_status == "PASS"` (a linha da tabela de §4 estava certa; a prosa de "`fidelity`, `violations`, `warnings`" fazia uma alegação mais forte e FALSA)

Achado da Task 5 do ciclo 16 (revisão de plano). `docs/aircraft_spec.schema.md`
tinha DUAS afirmações sobre `violations` em lugares diferentes: a linha da
tabela de blocos de topo (§4) dizia "vazio se `validation_status ==
"PASS"`" — verdadeira, uma direção só, e por isso **não foi reescrita**. A
prosa da subseção "`fidelity`, `violations`, `warnings`" (mesmo §4) dizia
"vazio **se e somente se** `validation_status == "PASS"`" — a MESMA
implicação MAIS a recíproca, e a recíproca é FALSA: `violations: []` NÃO
implica PASS, porque 8 dos 9 `Portao` do veredito global
(`pipeline::Portao`) podem reprovar sem empurrar NENHUMA `Violacao`
companheira (`portao_v_cruzeiro`, `portao_flutter`, `portao_antitombamento`,
`portao_estabilidade_long` não têm violação correspondente nenhuma —
achado da revisão da Task 4 do próprio ciclo 16, que motivou incluir os 9
portões na varredura de `validation::incerteza::analisa`). Corrigido
in-loco na Task 5 (a prosa agora afirma só a direção verdadeira e nomeia a
recíproca falsa explicitamente); registrado aqui como esclarecimento de
documentação, não como defeito de comportamento — o CÓDIGO sempre se
comportou assim, só a PROSA de uma seção fazia a alegação mais forte que o
código nunca sustentou. Ponteiro: `docs/aircraft_spec.schema.md` §4
("`fidelity`, `violations`, `warnings`"), spec ciclo16, Task 5 Passo 6
(ERRATUM da revisão de plano).

## 37. `robustness.flips[]` é um canal paralelo que a incerteza do ciclo 16 NÃO alcança

Achado da revisão final do ciclo 16. **Não se manifesta no baseline de hoje** —
e é exatamente por isso que está escrito.

Cada `RobustnessFlip` tem uma violação companheira em `violations[]` (ver
`docs/aircraft_spec.schema.md`, seção de `robustness`). Quando um check vira
dentro da banda de `propeller.fom_static`, quem recebe o prefixo
`INDETERMINADO — ` é a **companheira em `violations[]`**, porque é sobre ela
que `incerteza::publica_violacoes` opera. O `flip` em si é calculado e
serializado ANTES e independentemente de `validation::incerteza::analisa`
(`src/main.rs`, `robustness: Some(robustness.clone())`).

**Consequência se um flip de robustez cair na banda:** o JSON passa a ter
**duas visões estruturadas do mesmo evento discordando**. `violations[]` diria
"o modelo não sustenta este veredito"; `robustness.flips[]` continuaria
narrando uma falha lisa e determinada, com `valor` e `limite` numéricos e
nenhuma marca de incerteza.

O consumidor prejudicado é o **mais bem-comportado**: quem lê o array
estruturado em vez de fazer parsing de prosa perde a incerteza inteira. É a
classe de risco que o ciclo 16 existe para fechar, num canal que ele não cobre.

Hoje não ocorre porque o único indeterminado do baseline (`gradiente_cs2365`)
não é um flip de robustez. Isso é propriedade do baseline, não do desenho.

**Conserto proposto:** dar a `RobustnessFlip` um campo `indeterminado: bool`
(ou `veredito`) alimentado pelo mesmo `inc.checks`, para que as duas visões não
possam divergir. Não feito no ciclo 16 porque exigiria mexer no caminho de
serialização durante a fix wave, com risco ao invariante do artefato — e a
regra da casa é uma fix wave só. Ponteiro: spec ciclo16 §9 item 6,
`src/validation/incerteza.rs::publica_violacoes`, `src/models/specs.rs`
(`RobustnessFlip`).

## 38. A bisseção do breakeven prova UMA travessia, não a unicidade dela

Achado da revisão final do ciclo 16. Não é defeito do número publicado.

O bracket publicado em `uncertainty.checks[].breakeven_lo/hi` **testemunha uma
travessia real**: `bisseca` exige, por `assert!` (de verdade, não
`debug_assert!`), que os dois extremos do bracket tenham vereditos opostos, e o
teste `breakeven_publicado_e_provado_re_rodando_o_pipeline` reconfirma
re-executando o pipeline nos dois lados.

O que o código **não** prova é que a travessia é a **única** dentro da banda.
Se `viola(fom)` não fosse monotônica ali — fisicamente improvável para este
parâmetro, mas não verificado —, existiriam outras travessias, e a bisseção
convergiria para uma delas sem dizer qual. O texto publicado ("breakeven em
[…]") sugere unicidade que não foi verificada.

A spec do ciclo 16 declara isso na §5.4 ("sob travessia dupla o número
publicado é UM breakeven medido, não 'o único'") e, após a revisão final,
também na §9 item 7 — porque §9 é onde o leitor procura lacunas.

**Conserto proposto:** um teste de propriedade que amostre `viola(fom)` em N
pontos da banda e falhe se detectar mais de uma transição, ou aceitar a lacuna
declarada. Ponteiro: `src/validation/incerteza.rs::bisseca`, spec ciclo16 §5.4
e §9.

## 39. A fronteira de garantia da INCERTEZA do ciclo 16 — o que o mecanismo prova e o que ele NÃO prova

Irmão do item #30, que fez a mesma medição para o `pins_vs_json.rs` do ciclo
15. Medido pela revisão final, rodando os ataques, não estimado.

**Quanto custa fazer o modelo esconder uma indeterminação?**

1. **Atacar o código** — fazer `classifica` ignorar os extremos e decidir só
   pelo nominal: **~4 linhas numa função**. Blast radius: **9 testes em 4
   arquivos** (2 unitários puros, `tests/cli.rs`, 4 em `tests/incerteza.rs`, 2
   em `tests/schema_v4.rs`). Nenhuma edição acidental sobrevive.
2. **Estreitar a banda em config** — `fom_static_tol_pct` de 10,0 para 4,0
   tira o `gradiente_cs2365` da banda (o breakeven está a +4,649% do nominal):
   **1 linha de TOML**. Reprovam **9 testes em 4 arquivos** SE o artefato não
   for regenerado. Se o atacante regenerar e commitar honestamente, a mudança
   de política fica **visível no `git diff`** — mas para a suíte voltar a verde
   ele ainda precisa reescrever ou apagar pelo menos **4 testes cujo NOME
   denuncia o que fazem**: `baseline_tem_exatamente_um_check_indeterminado`,
   `breakeven_publicado_e_provado_re_rodando_o_pipeline`,
   `contagem_de_violacoes_e_igual_com_banda_larga_ou_estreita`,
   `uncertainty_bloco_publica_banda_efetiva_e_o_gradiente_indeterminado`.
3. **Colapsar a banda a um ponto** — `fom_static_tol_pct = 1e-9` passa na
   validação `(0 ; 100)`, que não tem piso de largura. Elimina TODO
   indeterminado. **Isto é deliberadamente permitido**: a banda é declaração de
   confiança do projeto, e o modelo não deve sobrepor-se a ela. É aceitável
   **só porque `declared_tol_pct` é publicado no artefato** — quem lê vê a
   política que produziu o veredito. Um piso arbitrário seria o modelo fingindo
   saber quanta incerteza o usuário deveria declarar.
4. **Editar `uncertainty.checks: []` direto no JSON commitado**: pego
   **imediatamente**, por 3 testes em 2 arquivos, incluindo
   `cli.rs::aircraft_spec_json_commitado_bate_com_o_pipeline_real`, que
   regenera e compara.

**Conclusão declarada.** O mecanismo tem duas defesas reais: (a) o teste de
regeneração do `aircraft_spec.json` commitado, que pega qualquer divergência
entre binário e arquivo — bug ou edição manual; e (b) testes **nomeados pelo
propósito**, que pegam qualquer regressão de código.

**Mas a garantia é de AUDITORIA, não estrutural.** Quem esteja disposto a
reescrever deliberadamente os testes cujo nome anuncia "isto existe para provar
que o modelo admite não saber" ainda consegue. A única coisa no caminho é um
revisor lendo `git diff -- tests/` e reconhecendo que um teste com esse nome
sumiu ou mudou de sentido.

É a mesma fronteira do #30, e vale repetir a formulação de lá: **a ferramenta
prova mecanicamente contra ERRO, não contra um adversário disposto a editar a
prova junto com o alvo.** O ciclo 16 fez o modelo admitir que não sabe; não fez
o modelo resistir a quem queira silenciá-lo, e não pretende ter feito.
