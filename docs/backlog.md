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
