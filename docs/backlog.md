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

## 7. Textos pré-erratum sobre o momento da linha de tração ficaram desatualizados (ciclo 10, task 2)

O erratum do ciclo 10 (task 2, commit `713e846`) corrigiu o braço do
momento de rotação de "sobre o solo" (`h_cg_ground_m + prop_axis_above_cg_m`
≈ 1,12 m) para "sobre o CG" (`prop_axis_above_cg_m` ≈ 0,20 m, ver §2 do
erratum em
`docs/superpowers/specs/2026-08-09-ciclo10-sag-e-linha-de-tracao-design.md`).
A correção do CÓDIGO de produção (`agents::trim_authority::
rotation_available_moment_nm`/`rotation_fwd_limit_m`, chamados com
`cfg.propeller.prop_axis_above_cg_m`) está certa e testada — mas três
textos que descreviam o modelo ANTES do erratum não foram atualizados
junto, e hoje contradizem o comportamento real:

1. **`fidelity.trim` em `src/main.rs` (linha ~836)** diz *"rotação
   desconsidera binário tração/arrasto/inércia (residual ≈
   μ_roll·(W−L_g)·h_cg)"* — isso deixou de ser verdade na Task 2 do ciclo
   10, que passou a incluir explicitamente o binário de TRAÇÃO no balanço
   de rotação (`−T(Vr)·prop_axis_above_cg_m`). Só os termos de SOLO
   (`μN·h_cg`, `D·(h_cg−h_D)`, ≲2 pp, ver erratum §2) continuam
   desprezados — a string precisa ser reescrita para refletir isso, não
   apagada por completo.
2. **Docstring do hand-check `momento_da_linha_de_tracao_hand_check_com_literais`
   em `src/agents/trim_authority.rs` (linha ~1108)** afirma *"z_eixo =
   1,12 m (= h_cg_ground 0,92 + offset 0,20 do baseline E10)"* — essa
   equivalência era verdadeira ANTES do erratum; hoje o `z_eixo` real do
   baseline E10 é `prop_axis_above_cg_m` = 0,20 m sozinho, não 1,12 m. O
   teste em si continua correto (é um hand-check de sensibilidade com um
   literal arbitrário, `1,12` só precisa ser ALGUM número), mas o
   comentário induz o leitor a pensar que 1,12 m é o braço usado em
   produção hoje.
3. **Docstring da property `eixo_mais_alto_recua_o_limite_de_rotacao`
   (mesmo arquivo, linha ~1179-1180)** rotula os literais de teste
   `z=1,12`/`z=1,24` como *"baseline E10"*/*"candidato E11 (+12 cm)"* —
   mesma imprecisão: o E10 real usa `z=0,20`, o E11 usaria `z=0,32`. Achado
   da revisão da Task 3 (ciclo10-sag-e-linha-de-tracao): como o termo
   `T(Vr(W))·z_eixo` é AFIM (linear) em `z_eixo` com inclinação constante
   `T(Vr(W))/W`, o `Δx` medido para `Δz=0,12` **não depende de onde `z`
   começa** — então o teste segue válido numericamente mesmo com os
   literais errados, mas o rótulo "baseline E10"/"candidato E11" no
   comentário é enganoso.

Nenhum destes três é um bug de física ou de teste — são textos/comentários
que não acompanharam o erratum. Ponteiro: `src/main.rs` (`fidelity.insert("trim"...)`),
`src/agents/trim_authority.rs` (`momento_da_linha_de_tracao_hand_check_com_literais`,
`eixo_mais_alto_recua_o_limite_de_rotacao`),
`.superpowers/sdd/2026-08-09-ciclo10-sag-e-linha-de-tracao/task-3-report.md`
§3.3 (achado completo, com a verificação numérica da linearidade em `z`).
