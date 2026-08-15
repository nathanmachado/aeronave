# Ciclo 12 — Solo Honesto: rolagem por integração numérica com arrasto explícito

**Data:** 2026-08-15
**Backlog fechado:** item 4 (rolagem de decolagem/pouso sem termo de arrasto)
**Escopo adicional:** termos de solo do balanço de rotação (registrados como desprezados no ciclo 10)
**Baseline de partida:** `1e11998`, schema 5.4, 469 testes verdes, veredito PASS, 0 violações

---

## 0. Por que este ciclo existe

O método energético de Raymer (Cap. 5) calcula a rolagem de decolagem como

    S_G = W² / (g·ρ·S·CL_TO·T_avg)

que é a energia cinética em V_LOF dividida pelo trabalho da tração. Por
construção o método **não tem termo de arrasto nem de atrito**: `T_avg` é
assumida a tração LÍQUIDA média, e a calibração empírica de Raymer embute
tudo o mais. O mesmo vale para o pouso (`S_G = V_ref²/(2gμ)`): frenagem
constante, sem arrasto aerodinâmico e sem o alívio de peso que a
sustentação residual causa sobre as rodas.

Desde o ciclo 8 o flap está modelado na polar (`cd0_flap_to_extra`), mas o
segmento DOMINANTE da distância de decolagem e de pouso continuava sem
custo de arrasto. Este ciclo troca os dois métodos fechados por integração
numérica da equação de movimento, consumindo a polar completa.

## 1. O achado que muda a natureza do ciclo

Integrar obriga a avaliar a tração em toda a faixa `0 → V_LOF`. Nenhum
consumidor existente fazia isso: a rolagem avaliava tração só em `V = 0`, e
subida/cruzeiro/teto só acima de `1,2·V_s`. A faixa intermediária nunca foi
exercitada — e está quebrada.

### 1.1 O defeito

`agents::propulsion::prop_efficiency(J) = −0,15·J² + 0,39·J + 0,58` devolve
**η(0) = 0,58**. Por definição `η = T·V/P`, logo `η → 0` quando `V → 0`. O
polinômio foi ajustado com dados de JavaProp (Hepperle, DLR) na faixa de
CRUZEIRO — pico em `J ≈ 1,3–1,5`, docstring declara validade `0 < J < 2,8`.
Na corrida de decolagem desta célula `J` vai de 0 a 0,68: extrapolação onde
a curva não é apenas imprecisa, é **qualitativamente errada**.

`agents::performance::thrust_available_n` comuta em `V = 0,5 m/s`:

| Faixa | Ramo | Valor no baseline |
|---|---|---|
| `V < 0,5` | Rankine-Froude × `static_thrust_factor` | 3.841 N |
| `0,5 ≤ V < 1,0` | `thrust_n` com guarda `if v_ms < 1.0 { return 0.0 }` | **0 N** |
| `V ≥ 1,0` | `η(J)·P/V` | **≈ 80.000 N em V = 1 m/s** |

Vinte vezes a tração estática, precedido de uma janela de tração nula.

### 1.2 A forma do erro

É a terceira ocorrência do mesmo padrão neste projeto — uma constante ou
heurística válida sob uma hipótese que a mudança quebra em silêncio:

| Ciclo | Constante | Hipótese calada | Como quebrou |
|---|---|---|---|
| 10 | braço do momento de tração | pivô não-acelerado | corrida de decolagem É acelerada |
| 11 | janela de busca `[1,3; 1,8]·Vs` | `Vs` do estol FLAPADO | referência trocada para limpo |
| 12 | `η(J)` polinomial | `J` de cruzeiro | rolagem varre `J → 0` |

**Lição operacional:** o defeito estava dormente porque nenhum consumidor
avaliava a função naquela região. Trocar uma fórmula fechada por integração
não cria o bug — apenas deixa de torná-lo inalcançável. É o argumento a
favor de integrar: a fórmula fechada esconde a região onde o modelo
subjacente não vale.

### 1.3 Decisão (aprovada pelo usuário, 2026-08-15)

Escrever um modelo de tração válido na faixa da rolagem, **restrito à
rolagem**. Corrigir o polinômio globalmente reabriria cruzeiro, teto,
alcance e autonomia — escopo de um ciclo próprio, registrado no backlog.

---

## 2. §1 — `thrust_ground_roll_n`: tração válida de 0 a V_LOF

Teoria de quantidade de movimento (Rankine-Froude) COM velocidade de
avanço. Com `u = V + v_i` (velocidade na esteira longe do disco dividida
por 2, isto é, velocidade no disco), `A = π·D²/4` e `P` a potência de EIXO:

    T = 2·ρ·A·u·(u − V)          [empuxo do disco]
    P = T·u                       [potência ideal]
    ⟹ u²·(u − V) = P / (2·ρ·A) =: K

Cúbica `u³ − V·u² − K = 0`. Para `K > 0` e `V ≥ 0` há **exatamente uma raiz
real com `u > V`** (`f(V) = −K < 0`, `f` estritamente crescente em
`u > 2V/3`, `f → +∞`). Newton a partir de `u₀ = V + K^(1/3)` converge
monotonicamente; bracket `[V, V + K^(1/3) + V]` provado no teste.

    thrust_ground_roll_n(V) = static_thrust_factor · 2·ρ·A·u·(u − V)

### 2.1 Continuidade exata com o modelo de hoje

Em `V = 0`: `u³ = K` ⟹ `u = K^(1/3)` ⟹

    T = 2ρA·K^(2/3) = 2ρA·(P/(2ρA))^(2/3) = (2ρA·P²)^(1/3)

que é **algebricamente idêntico** a `static_thrust_ideal_n`. Multiplicado
pelo mesmo `static_thrust_factor`, `thrust_ground_roll_n(0)` reproduz
`thrust_available_n(0)` — não aproximadamente, exatamente. Isso torna a
função uma REFINAÇÃO do modelo atual, não uma substituição: a tração
estática calibrada de hoje é o ponto `V = 0` da nova curva.

### 2.2 A costura em V_LOF fica exposta, não escondida

Em `V_LOF ≈ 35,6 m/s` o novo modelo dá ≈ 2.400 N e `thrust_available_n` dá
≈ 3.300 N — **discontinuidade de ~28%** entre o fim da rolagem e o início
do segmento de subida do mesmo cálculo de `takeoff_distance_50ft_m`.

Isto NÃO é mascarado. Os dois modelos têm domínios de validade disjuntos:
`static_thrust_factor = 0,75` é uma correção empírica (McCormick) calibrada
para tração ESTÁTICA, e o polinômio é calibrado para `J` de cruzeiro.
Nenhum dos dois é confiável no domínio do outro. A descontinuidade vira
**item novo de backlog** (unificar o modelo de tração num único modelo de
hélice válido em todo o envelope), com a magnitude medida registrada.

---

## 3. §2 — Rolagem de decolagem por integração

    m·dV/dt = T(V) − D(V) − μ_roll·N(V)
    N(V) = max(0, W − L(V))
    S = ∫₀^{V_LOF} m·V·dV / F_net(V)

`N` travado em zero: se `L > W` antes de `V_LOF` a aeronave já voa, e o
atrito NÃO pode virar negativo (empurrar a aeronave para a frente). Guarda
falseável, não comentário.

### 3.1 Coeficientes no solo

    CL_roll = cfg.stability.cl_ground_rotation                        (0,50)
    CD_roll = wing.cd0 + state.cd0_gear_fixed_increment
              + wing.cd0_flap_to_extra
              + CL_roll²/(π·wing.aspect_ratio·wing.oswald_efficiency)

Acessores exatos, para não haver adivinhação: o incremento de trem é
`AircraftState::cd0_gear_fixed_increment` (já plumbado — é o mesmo campo que
`agents::aerodynamics::cd0_total` consome), NÃO um campo de `WingSpec`. As
funções de decolagem já recebem `state: &AircraftState`; nenhuma assinatura
nova é necessária no caminho da decolagem.

**`cl_ground_rotation` — checagem de premissa calibrada.** O campo existe
desde o início (`[stability]`, docstring "CL da asa na corrida de decolagem
antes da rotação") mas só alimentava o balanço de rotação, avaliado em
`V_r`. Reusá-lo na rolagem inteira assume ATITUDE CONSTANTE do início da
corrida até a rotação — verdadeiro (a aeronave rola com o trem no chão, em
atitude fixa, até rotacionar) e mesma configuração de flap. Reuso válido.

Consequência nova a registrar: `cl_ground_rotation` passa a acoplar duas
saídas antes independentes — mexer nele move a distância de decolagem E o
limite dianteiro de rotação. Coberto por property estrita.

**`cd0_fixed_increment` — a segunda premissa calibrada.**
`CD0_GEAR_RETRACTABLE = 0.0` (`agents::aerodynamics`), ou seja, o `wing.cd0`
de 0,023504 é o CD0 de **trem RECOLHIDO**. Na corrida o trem está
ESTENDIDO. O projeto já tem o número — `[gear] cd0_fixed_increment = 0.008`
— usado hoje apenas quando `gear_retractable = false`. Reusá-lo assume que
um trem retrátil ESTENDIDO tem arrasto próximo ao de um trem FIXO;
levemente OTIMISTA (o retrátil estendido não tem as carenagens de um trem
fixo bem projetado), assumido e declarado. Inventar um número novo sem base
seria pior.

**Efeito solo deliberadamente FORA.** A `h/b = 0,92/11,94 = 0,077` o fator
de Wieselsberger reduziria o induzido em ≈40% e aumentaria a sustentação.
As duas direções são FAVORÁVEIS (menos arrasto, menos peso nas rodas),
logo omitir é CONSERVADOR. Registrado como aproximação assumida.

### 3.2 Esquema numérico

Simpson composto em `V` com 200 intervalos. Integrando `g(V) = m·V/F_net(V)`
é finito nos dois extremos (`g(0) = 0`). Se `F_net(V) ≤ 0` em qualquer
ponto — tração insuficiente para acelerar — a função devolve `f64::INFINITY`
(decolagem impossível nesta condição), resultado FÍSICO, não erro.

---

## 4. §3 — `surface_factor` sai do caminho da decolagem

Hoje `takeoff_distance_m` e `takeoff_distance_50ft_m` recebem
`surface_factor` (1,00 pavimentado / 1,20 grama) e multiplicam a rolagem.

**Com `μ_roll` explícito isso é contagem dupla**: o 1,20 foi calibrado
justamente para representar o atrito de grama que a fórmula energética não
tinha. Terceira premissa calibrada do ciclo.

Decisão: substituir o parâmetro `surface_factor: f64` por `mu_roll: f64`
nas duas assinaturas. Os chamadores no orquestrador passam
`perf_cfg.mu_roll_paved` / `mu_roll_grass`, espelhando exatamente o que
`mu_brake_paved` / `mu_brake_grass` já fazem no pouso. Decolagem e pouso
ficam simétricos.

Config nova em `[performance]` (Raymer Tab. 17.1 / Gudmundsson cap. 17):

    mu_roll_paved = 0.04    # rolagem livre, asfalto/concreto seco (faixa 0,03–0,05)
    mu_roll_grass = 0.08    # grama firme de fazenda (faixa curta 0,05 / alta 0,10)

Validação de faixa em `models::config`, como os demais escalares físicos.

---

## 5. §4 — Rolagem de pouso por integração

    m·dV/dt = −[ D(V) + μ_brake·N(V) ]
    N(V) = max(0, W_ldg − L(V))
    S = ∫₀^{V_ref} m·V·dV / F_dec(V)

`V_ref = 1,30·V_s` (flap de pouso), massa de pouso inalterada
(MTOW − 60% do combustível). Nada disso muda neste ciclo — trocar duas
coisas ao mesmo tempo esconderia qual causou o quê.

### 5.1 Configuração modelada: flap MANTIDO (decisão do usuário)

A frenagem é modelada com o flap de pouso DEFLEXIONADO durante toda a
rolagem — a configuração em que CS 23 mede a distância de pouso desta
classe (sem spoilers, sem ação de piloto modelada). A sustentação residual
ALIVIA o peso sobre as rodas e PIORA a frenagem; o arrasto ajuda; o saldo
é uma rolagem MAIOR que a de hoje.

A alternativa (recolher flap após o toque) encurtaria a rolagem em grama de
≈307 m para ≈248 m e salvaria o check #24 por 12 m — mas o modelo passaria
a assumir uma ação de piloto executada em segundos, sempre, sem falha.
Rejeitada.

### 5.2 CL na rolagem de pouso — derivado, não parâmetro livre novo

    CL_roll_ldg = cl_ground_rotation
                  + (1 − to_flap_fraction)·(cl_max_flaps − cl_max_clean)
                = 0,50 + 0,65·(2,10 − 1,45) = 0,9225

**Premissa declarada:** o flap desloca a curva `CL(α)` quase paralelamente,
de modo que o incremento de `CL` em ATITUDE FIXA é aproximadamente igual ao
incremento de `CL_max`. Verdadeiro para flap simples e slotted dentro da
precisão deste modelo. `cl_ground_rotation` já embute a fração PARCIAL de
decolagem, então o que falta para o flap CHEIO é a fração complementar
`(1 − to_flap_fraction)`.

Derivar em vez de adicionar um campo de config mantém fonte única de
verdade e segue a política do projeto (`cl_h_max_down` deixou de ser
parâmetro livre pelo mesmo motivo).

    CD_roll_ldg = wing.cd0 + state.cd0_gear_fixed_increment
                  + wing.cd0_flap_ldg_extra (CHEIO, 0,015)
                  + CL_roll_ldg²/(π·wing.aspect_ratio·wing.oswald_efficiency)

### 5.3 Duas mudanças estruturais que este parágrafo obriga

**(a) `WingSpec::cd0_flap_ldg_extra` passa a EXISTIR.** Hoje `WingSpec` só
carrega `cd0_flap_to_extra` (a fração PARCIAL). A auditoria do ciclo 8
concluiu explicitamente que `cd0_flap_ldg_extra` NÃO deveria existir porque
"nada o consumiria" — ver a docstring de `WingCfg::cd0_flap_delta` e a de
`landing_distance_50ft_m`, que enumera os três segmentos e conclui que
nenhum toca a polar. **Essa conclusão morre aqui**: a rolagem integrada é o
primeiro consumidor do delta CHEIO. Campo novo em `WingSpec`, valor
`cfg.wing.cd0_flap_delta` (0,015 — o delta cheio, sem fração), construído
no mesmo ponto em que `cd0_flap_to_extra` já é construído. As DUAS
docstrings são reescritas old→new com o motivo, nunca deletadas.

**(b) `landing_ground_roll_m` e `landing_distance_50ft_m` passam a receber
`state: &AircraftState`.** Hoje não recebem — não precisavam, porque nenhum
segmento consumia a polar. Precisam agora para `cd0_gear_fixed_increment`.
Mudança de assinatura com atualização dos chamadores no orquestrador.

---

## 6. §5 — Termos de solo do balanço de rotação

A docstring de `rotation_available_moment_nm` (ciclo 10, task 2) deriva:

    M_T + M_in = − T·Δz − D·h_cg − μN·h_cg

e declara os dois últimos "deliberadamente DESPREZADOS", com magnitude
estimada "**≲2 pp de MAC** no limite dianteiro". Este ciclo os implementa:

    M_solo = − μ_roll·N·h_cg − D·(h_cg − h_D)
    N  = max(0, W − L_g),  L_g = q_r·S_w·cl_ground_rotation
    D  = q_r·S_w·CD_roll                        (mesmo CD_roll do §3.1)
    h_cg = cfg.gear.h_cg_ground_m               (0,92 m)
    h_D  = cfg.wing.z_drag_above_cg_m           (NOVO, default 0,0)

### 6.1 A estimativa de "≲2 pp" estava baixa

Hand-check no baseline (`q_r = 776,0 Pa`, `L_g = 5.510 N`, `N = 9.767 N`,
`D = 520 N`):

    μ·N·h_cg      = 0,04 · 9.767 · 0,92 = 359 N·m
    D·(h_cg − h_D)= 520 · 0,92          = 479 N·m
    total                                = 838 N·m

O termo de tração que JÁ está no balanço vale `T(V_r)·Δz ≈ 3.327·0,20 =
665 N·m`. **Os dois termos "desprezáveis" juntos são MAIORES que o termo
mantido.** Deslocamento do limite dianteiro: `838/15.277 = 0,055 m`, ou
≈4,5 pp de MAC — mais que o dobro do estimado.

Não é erro do ciclo 10: a estimativa foi feita sem `CD_roll` explícito (o
arrasto de solo com trem estendido e flap não existia no modelo) e sem
`μ_roll` (não havia campo). Mas o texto afirma um número que a medição
desmente, e por isso a docstring é **reescrita old→new com a magnitude
medida**, não corrigida em silêncio.

### 6.2 `z_drag_above_cg_m`

Campo novo em `[wing]`, default `0.0`. Faixa plausível 0–0,10 m numa célula
convencional (o centro de arrasto fica alguns centímetros ACIMA do CG). Com
`h_D = 0` o termo de arrasto usa o braço CHEIO `h_cg`, que é o caso
CONSERVADOR (`h_D > 0` encolhe o braço). A docstring de `cm_thrust_cruise`
já registrava a necessidade deste campo — a referência passa a ser real.

**Este campo NÃO entra em `cm_thrust_cruise` neste ciclo.** Fazê-lo mudaria
`CL_h_trim`, `cd_trim`, `cd_cruise` e portanto velocidade de cruzeiro,
alcance e autonomia — cascata desproporcional. Fica registrado no backlog
como consumidor pendente do campo, com a direção do erro já documentada na
docstring existente.

### 6.3 Direção e falseabilidade

Ambos os termos são nariz-ABAIXO: SUBTRAEM do momento disponível, o limite
dianteiro de rotação RECUA (percentual de MAC MAIOR), o envelope de CG
ESTREITA. Properties estritas: `μ_roll` maior ⟹ limite recua; `h_cg` maior
⟹ recua; `z_drag_above_cg_m` maior ⟹ limite AVANÇA (braço menor); passar
`μ_roll = 0` e `CD_roll = 0` reproduz EXATAMENTE o modelo pré-ciclo-12.

`inside_envelope` pode virar FALSE em algum cenário de carga. Se virar, é
achado honesto documentado — não se ajusta config para salvar o veredito.

---

## 7. §6 — Guardas falseáveis do integrador

A lição do ciclo 11 (argmax na fronteira é DEFEITO, não resultado) aplicada
a integração numérica: **um resultado em resolução não convergida é
defeito, não resultado**; e a lição do `verifica-ciclo.sh` (todo mecanismo
de verificação novo precisa provar as DUAS direções).

1. **Limite analítico.** Com `μ = 0`, `CD = 0` e `T` constante, o integrador
   deve reproduzir `S = ½·m·V_LOF²/T` com erro relativo < 1e-9. Prova o
   integrador contra fechada exata, não contra um pin.
2. **Convergência.** Dobrar o número de intervalos (200 → 400) muda o
   resultado em < 0,1%. Falha se a resolução escolhida não bastar.
3. **Monotonicidades estritas.** `μ` maior ⟹ rolagem maior; `CD0` maior ⟹
   maior; tração maior ⟹ menor; peso maior ⟹ maior.
4. **Atrito não-negativo.** Com `L > W` forçado, `N` é zero e a rolagem não
   encurta por atrito negativo.
5. **Continuidade em V=0.** `thrust_ground_roll_n(0, …)` idêntico a
   `thrust_available_n(0, …)` dentro de 1e-9 relativo.
6. **Tração insuficiente ⟹ `+INFINITY`**, não número espúrio.

---

## 8. §7 — Schema 5.5

`to_distance_paved_m`, `to_distance_grass_m` e `landing_distance_m` ganham
`#[serde(with = "fatigue_life_serde")]`. O integrador pode devolver
`+INFINITY` e hoje esses três virariam `null` no JSON, quebrando round-trip
— exatamente o defeito que o ciclo 11 corrigiu em `to_50ft_*`. Primeiro uso
real da infraestrutura do ciclo 11.

Bump MINOR 5.4 → 5.5 com exceção registrada (mesmo padrão de 5.2/5.3/5.4):
o tipo de três campos passa a admitir string.

### 8.1 `to_distance_*` e `landing_distance_m` ficam, mas com aviso

Estes três campos são estimativas simplificadas (`rolagem × 1,5` e
`rolagem + 200 m`). O fator 1,5 é de Raymer, calibrado como razão
`distância sobre 15 m / rolagem` para um método energético. Com a rolagem
integrada a razão real cai para ≈1,32 — **o 1,5 fica visivelmente
inconsistente com os campos `*_50ft_*` do mesmo JSON**.

Não são removidos aqui (remoção de campo é bump MAJOR, fora de escopo). São
mantidos consumindo a MESMA rolagem nova, com docstring reescrita dizendo
que são legado e que os campos `*_50ft_*` são a referência física. Item de
backlog novo propõe a remoção num MAJOR futuro.

---

## 9. Números congelados (hand-check do chefe; divergência > 5% escala)

| Grandeza | Hoje (`1e11998`) | Estimativa congelada |
|---|---|---|
| Rolagem TO pavimentada | 265,5 m | **≈ 492 m** |
| Rolagem TO grama | 318,6 m | **≈ 653 m** |
| `to_50ft_paved_m` | 420,372451 | ≈ 647 m |
| `to_50ft_grass_m` | 473,469470 | **≈ 808 m** |
| `to_distance_paved_m` | 398,227641 | ≈ 738 m |
| `to_distance_grass_m` | 477,873169 | ≈ 980 m |
| Rolagem pouso pavimentada | 162,7 m | ≈ 242 m |
| Rolagem pouso grama | 216,9 m | ≈ 307 m |
| `ldg_50ft_m` | 502,458299 | ≈ 582 m |
| `ldg_50ft_grass_m` | 556,677173 | **≈ 646 m** |
| `landing_distance_m` | 362,656622 | ≈ 442 m |
| Limite dianteiro de rotação | 8,908% MAC | **+3,5 a +5,5 pp** |

Hipóteses do hand-check, para o revisor conferir: `P_eixo ≈ 150,1 kW`
(retro-derivada da rolagem atual, autoconsistente com `T_static = 3.841 N`),
`A = π·1,76²/4 = 2,4328 m²`, `V_LOF = 1,10·V_s_TO = 35,594 m/s`,
`V_ref = 35,72 m/s`, `m_ldg = 1.407,4 kg`, Simpson com 4 intervalos.

## 10. Veredito esperado: REPROVADO

| Check | Grandeza | Limite | Estimativa | Situação |
|---|---|---|---|---|
| #23 | `to_50ft_grass_m` | 600 m | ≈ 808 m | **FAIL por ≈ 208 m** |
| #24 | `ldg_50ft_grass_m` | 600 m | ≈ 646 m | **FAIL por ≈ 46 m** |

`validation_status` deve virar **FAIL** com pelo menos 2 violações.

**Isto não é regressão — é o resultado.** O modelo passa a dizer a verdade
sobre operar 1.557 kg numa pista de grama de 600 m, com o segmento
dominante finalmente pagando arrasto e atrito. Diretriz permanente do
usuário: "se uma decisão é perigosa, o modelo deve FALHAR no ponto de
perigo". Nenhuma tolerância é alargada, nenhum `μ` é tunado, nenhuma config
é ajustada para salvar o PASS.

O ciclo 13 passa a ser a decisão de projeto: mais potência, mais asa, pista
maior, ou aceitar operação pavimentada.

## 11. Fora de escopo (registrado no backlog, não escondido)

1. **Unificar o modelo de tração** — descontinuidade de ≈28% em `V_LOF`
   entre `thrust_ground_roll_n` e `thrust_available_n`. Reabre cruzeiro,
   teto, alcance e autonomia.
2. **`prop_efficiency` com `η(0) = 0,58`** — o polinômio continua servindo
   subida/cruzeiro/teto, onde `J` está no domínio calibrado. A guarda
   `if v_ms < 1.0 { return 0.0 }` de `thrust_n` e a janela de tração nula em
   `[0,5; 1,0)` m/s permanecem, agora sem consumidor na faixa.
3. **`z_drag_above_cg_m` em `cm_thrust_cruise`** — campo criado e consumido
   só pela rotação neste ciclo.
4. **Remoção de `to_distance_*` / `landing_distance_m`** — bump MAJOR.
5. **Efeito solo** — omitido, direção conservadora, §3.1.
6. **`V_ref` do pouso e política de toque** — `1,30·V_s` inalterado.

## 12. Restrições globais

- TDD RED-first. Português. `cargo test` verde por task. Genericidade verde
  (sem nomes de motor em `src/`). `scripts/verifica-ciclo.sh` colado em
  TODO report de task — sem ele o report não é aceito.
- Pins honestos: old→new comentado com data "Campanha ciclo 12".
  **Tolerâncias INALTERADAS — nunca alargar.**
- Toda mudança de docstring que este ciclo torna obsoleta é REESCRITA
  old→new, nunca deletada (§5.2 auditoria do ciclo 8, §6.1 estimativa de
  2 pp, §8.1 fator 1,5, docstrings de `takeoff_ground_roll_m` e
  `landing_ground_roll_m` sobre "não tem termo de arrasto").
- Regen do JSON:
  `cargo run --release -- --engine config/engines/toyota_1gd_ftv.toml --aircraft config/aircraft/baseline_4seat.toml --mission config/missions/default.toml --out aircraft_spec.json`
