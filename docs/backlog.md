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
`landing_ground_roll_m` usavam o método ENERGÉTICO de Raymer (`V²/2gμ`
ajustado por fator de tração/frenagem), que não incluía nenhum termo de
arrasto aerodinâmico explícito — mesmo com o flap já modelado na polar
(`cd0_flap_to_extra`, ciclo 8), o segmento DOMINANTE da distância de
decolagem/pouso ficava sem custo de arrasto por construção do método.

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

**Método substituído (`old`):** ambos os segmentos usavam a fórmula
energética fechada `S_G = V_ref²/(2·g·μ)` (ajustada por fator de
tração/frenagem médio), que por construção não tem NENHUM termo de arrasto
aerodinâmico — nem o CD0 de trem estendido, nem o induzido, nem o
incremento de flap entravam na conta da rolagem, só no gradiente de subida
posterior.

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
(checagem #25) — só o NÚMERO da violação muda, não o veredito.

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
   binário tração/arrasto/inércia"*, hoje **"inclui binário de TRAÇÃO
   `−T(Vr)·prop_axis_above_cg_m` no balanço; desconsidera termos de SOLO
   (residual ≈ μ_roll·(W−L_g)·h_cg, ≲2 pp)"** — reflete o comportamento real
   (tração incluída, solo desprezado).
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

## 8. Unificar o modelo de tração (`thrust_ground_roll_n` × `thrust_available_n` divergem em `V_LOF`)

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
faz costura com `thrust_available_n`), então este ciclo não introduz
inconsistência NUM cálculo — mas os dois modelos descrevem a MESMA
grandeza física (tração da hélice nesta velocidade) e devolvem números
27,69% distintos, o que é uma descontinuidade de modelo, não de código.
Unificar reabriria cruzeiro, teto de serviço, alcance e autonomia (todos
consumidores de `thrust_available_n`/`prop_efficiency`), por isso fica
fora de escopo deste ciclo. Ponteiro: docstrings de
`agents::performance::thrust_ground_roll_n`/`thrust_available_n`
(`src/agents/performance.rs`), spec §2.4 e §11 item 1
(`docs/superpowers/specs/2026-08-15-ciclo12-solo-honesto-design.md`).

## 9. `prop_efficiency` com `η(0) = 0,58` — fisicamente errado por definição, janela de tração nula sem consumidor

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

## 13. Pin órfão mascarado por tolerância — checagem de pins vs JSON regenerado em `verifica-ciclo.sh`

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
