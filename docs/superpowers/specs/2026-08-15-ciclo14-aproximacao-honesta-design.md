# Ciclo 14 — Aproximação honesta: o segmento aéreo do pouso

**Data:** 2026-08-15
**Backlog fechado:** #17 (segmento aéreo do pouso).
**Schema:** 5.6 → 5.7 (MINOR).
**Base:** `7d246b3` (merge do ciclo 13).

---

## §0 — Por que este ciclo

Depois do ciclo 13 o baseline reprova em 5 checagens. Uma delas — pouso em
grama, 646,66 m contra a pista de 600 m — é decidida por um segmento que
**não é física desta aeronave**: mais da metade da distância vem de duas
heurísticas nunca calibradas para a operação do projeto.

Decomposição medida do `ldg_50ft_grass_m` de hoje (646,6609422476247 m):

| Segmento | Fórmula de hoje | Valor | Fração |
|---|---|---|---|
| Aproximação | `15/tan(3°)` | 286,22 m | 44,3% |
| Flare | `V_ref × 1,5 s` | 53,60 m | 8,3% |
| **Aéreo (soma)** | | **339,82 m** | **52,5%** |
| Rolagem em grama | integrada (ciclo 12) | 306,84 m | 47,5% |

**A rolagem — o único segmento com física integrada — é minoria da conta.**

---

## §1 — Dois defeitos independentes, apontando para o mesmo lado

É essencial separá-los, porque têm naturezas diferentes e um deles não
depende de premissa nenhuma.

### §1.1 — DEFEITO DE MODELO: o flare não consome altura

    let s_air   = 15.0 / gamma_app.tan();          // performance.rs:975-976
    let s_flare = v_ref * perf_cfg.flare_time_s;   // performance.rs:977
    s_air + s_flare + s_ground

`s_air` já desce **os 15 metros inteiros até o solo**. Depois `s_flare` é
somado como distância horizontal PURA, sem consumir altura nenhuma. A
aeronave chega ao solo duas vezes: uma no fim da rampa, outra no fim do
flare.

Isso é **indefensável sob qualquer premissa de pilotagem** — não é uma
escolha conservadora nem otimista, é geometria inconsistente. Corrigir isto
não depende de decisão de projeto.

### §1.2 — PREMISSA ERRADA PARA ESTA OPERAÇÃO: a rampa de 3°

`[performance].approach_angle_deg = 3.0` é o *glideslope* de ILS — a rampa de
aproximação estabilizada COM POTÊNCIA de aeroporto pavimentado. O projeto
opera numa pista de fazenda de 600 m em grama (premissa declarada em
`config/missions/default.toml`), onde o procedimento é de campo curto.

Medido nesta célula, a V_ref, em configuração de pouso:

| Grandeza | Valor |
|---|---|
| `V_ref = 1,30·V_s` | 35,7351 m/s (128,65 km/h) |
| `CL_ref = 2W/(ρ·S·V_ref²)` | 1,2426 |
| `CD_ref` (trem estendido + flap cheio + induzido) | 0,1113 |
| **L/D** | **11,165** |
| **Planeio power-off** | **5,1181°** |
| Razão de descida a 5,1181° | 3,188 m/s |
| Razão de descida a 3,0° | 1,870 m/s |

O modelo faz a aeronave aproximar **mais raso do que ela desce com o motor
cortado**.

### §1.3 — Por que os dois juntos importam

Os dois erros empurram a distância para CIMA, então corrigir só um deixaria o
modelo certo pelo motivo errado, com os erros parcialmente se cancelando.
Este é o mesmo risco que quase mascarou o ciclo 12. **As duas correções vão
juntas, e a spec mede a contribuição de cada uma separadamente** (§9).

---

## §2 — O modelo novo

### §2.1 — Ângulo de aproximação: DERIVADO, não configurado

    CL_ref = 2·W_ldg / (ρ · S_w · V_ref²)
    CD_ref = cd_gear_extended(wing, state, CL_ref, wing.cd0_flap_ldg_extra)
    γ_app  = atan(CD_ref / CL_ref)

**Aproximação de pequeno ângulo, declarada.** Em planeio a sustentação
equilibra `W·cos γ`, não `W` — então `CL_ref` acima está sobrestimado por
`1/cos γ`. A 5,1181°, `cos γ = 0,99601`: **erro de 0,4%**, uma ordem de
grandeza abaixo do limiar de escalação de 5% deste projeto, e muito abaixo da
incerteza da própria premissa de técnica de pilotagem (§2.1). Assumido e
registrado, não corrigido — corrigir exigiria um laço (γ depende de CL, que
depende de γ) para ganhar 0,4%. Confirmado pela revisão de plano como ruído
declarável.

**Velocidade constante no flare, declarada.** `R = V_ref²/(g(n−1))` trata a
velocidade como constante durante o arco. Na física real o flare desacelera,
então `R` diminui ao longo dele e o arco verdadeiro é mais fechado — ou seja,
`s_flare` real é MENOR que o modelado. **Direção: CONSERVADORA.** Nomeada
aqui, não medida neste ciclo.

`γ_app` deixa de ser parâmetro livre e passa a ser **propriedade da polar**.
`[performance].approach_angle_deg` é REMOVIDO com erro de migração (§6.2).

**PREMISSA DECLARADA — motor em marcha lenta sobre o obstáculo.** Este é o
procedimento PADRÃO de campo curto, e é como os números de POH de pista curta
são medidos (obstáculo de 50 ft, flap cheio, potência em marcha lenta). Uma
aproximação COM potência é mais RASA e portanto mais LONGA. **Direção do erro
nomeada:** se a operação real usar aproximação motorizada, este modelo é
OTIMISTA. Registrar no `fidelity.performance`, não esconder.

**Limite físico implícito, e por que não há guarda de teto.** O planeio
power-off é o mais íngreme que esta célula consegue de forma estabilizada sem
freio aerodinâmico (que ela não tem). Como `γ_app` agora É esse valor, e não
uma config que poderia excedê-lo, a guarda de teto que a alternativa exigiria
fica desnecessária por construção. Registrar o raciocínio na docstring.

### §2.2 — Flare: arco geométrico, não tempo

O flare é um recolhimento com fator de carga `n`, não uma cronometragem:

    R       = V_ref² / (g · (n − 1))          [raio do arco]
    h_flare = R · (1 − cos γ_app)             [altura CONSUMIDA pelo flare]
    s_flare = R · sin γ_app                   [distância horizontal do flare]

`n = [performance].flare_load_factor` (novo, baseline **1,20**).
`[performance].flare_time_s` é REMOVIDO com erro de migração (§6.2).

### §2.3 — Fechamento geométrico

    s_air   = (15 − h_flare) / tan(γ_app)
    S_total = s_air + s_flare + s_ground

Agora a aproximação desce só até a altura em que o flare começa. **A soma das
alturas fecha 15 m por construção** — e isso é asserção, não comentário
(§5.4).

### §2.4 — Fonte única de verdade da polar

`agents::performance::cd_ground_roll` é **RENOMEADA para `cd_gear_extended`**.

A função nunca calculou "CD de rolagem": ela calcula
`wing.cd0 + state.cd0_gear_fixed_increment + cd0_flap_extra + CL²/(π·AR·e)`
— a polar da aeronave com TREM ESTENDIDO e um incremento de flap, avaliada
num CL qualquer. O nome era estreito porque até aqui só havia consumidores de
solo. Este ciclo cria o primeiro consumidor **em voo** (a aproximação, §2.1),
e um nome que diz "ground roll" num cálculo de segmento aéreo passaria a
mentir. `old→new` obrigatório na docstring.

Consumidores após o ciclo: rolagem de decolagem, rolagem de pouso, balanço de
rotação e **aproximação**.

---

## §3 — O que NÃO muda (e por quê)

- **Decolagem.** A geometria dela já é consistente: `s_rotation` acontece no
  SOLO (sem variação de altura) e `s_climb` sobe de 0 a 15 m por excesso de
  potência. Não há duplo fechamento. Nada a corrigir.
- **`landing_distance_m`** (rolagem + 200 m fixos) — estimativa legada, sua
  remoção é bump MAJOR (backlog #11). Fica.
- **Velocidade inicial da rolagem de pouso.** `landing_ground_roll_m` integra
  a partir de `V_ref`. Na física real o flare sangra velocidade até ≈1,15·V_s
  (31,61 m/s contra 35,74), então a rolagem verdadeira é MENOR. **Integrar a
  partir de `V_ref` é CONSERVADOR** — mantido de propósito neste ciclo, e
  registrado como item de backlog novo com a direção do erro nomeada.
- **Massa, cruzeiro, subida, rotação.** O segmento aéreo de pouso não
  realimenta o laço de convergência de MTOW. A mudança é ISOLADA — se algum
  desses números se mover, é achado, não efeito (§7).

---

## §4 — Sensibilidade, medida antes de implementar

### §4.1 — Ao fator de carga do flare

`n` é o único parâmetro novo. Varrido na faixa plausível, com `γ_app`
derivado (5,1181°):

| `n` | R (m) | h_flare (m) | s_flare (m) | s_air (m) | aéreo (m) | pouso grama |
|---|---|---|---|---|---|---|
| 1,10 | 1302,1 | 5,192 | 116,16 | 109,51 | 225,67 | 532,5 ✅ |
| 1,15 | 868,1 | 3,461 | 77,44 | 128,83 | 206,27 | 513,1 ✅ |
| **1,20 (baseline)** | **651,1** | **2,596** | **58,08** | **138,49** | **196,57** | **503,4 ✅** |
| 1,25 | 520,9 | 2,077 | 46,46 | 144,29 | 190,75 | 497,6 ✅ |
| 1,30 | 434,0 | 1,731 | 38,72 | 148,15 | 186,87 | 493,7 ✅ |

**O veredito PASSA na faixa inteira.** A escolha de `n` move o resultado em
39 m sobre ≈500 m (8%), e o gate de 600 m tem ≥11% de folga no pior caso.
O flip do gate **não é refém desta escolha**.

### §4.2 — Contribuição de cada defeito, isolada

| Configuração | s_ar | s_flare | Aéreo | Pouso pav. | Pouso grama | Gate 600 m |
|---|---|---|---|---|---|---|
| Hoje (3°, flare cinemático sem altura) | 286,22 | 53,60 | 339,82 | 582,5 | 646,7 | ❌ |
| **Só o conserto geométrico** (3°, flare com altura) | 269,19 | 34,07 | 303,27 | 546,0 | 610,1 | ❌ |
| **Só a premissa** (5,1181°, flare cinemático) | 167,47 | 53,60 | 221,08 | 463,8 | 527,9 | ✅ |
| **Os dois (adotado)** | 138,49 | 58,08 | **196,57** | **439,3** | **503,4** | ✅ |

Leitura obrigatória desta tabela: **o conserto geométrico sozinho NÃO salva o
gate** (610,1 m). A premissa sozinha salvaria, mas por um modelo ainda
geometricamente errado. É por isso que os dois vão juntos (§1.3). A
implementação DEVE reproduzir a linha "só o conserto geométrico" como medição
intermediária e reportá-la.

---

## §5 — Guardas falseáveis

### §5.1 — RED-FIRST: o flare de hoje não consome altura

Escrever PRIMEIRO, contra o código de hoje, e demonstrar que FALHA:

    // A soma das alturas dos segmentos aéreos tem que fechar o obstáculo.
    h_percorrida_na_rampa + h_flare == 15.0   (a 1e-9)

Hoje `h_flare = 0` por construção e a rampa desce 15 m, então a soma dá 15 —
**o teste como escrito acima PASSARIA hoje e não serve.** A forma falseável é
outra: assertar que o flare **consome altura estritamente positiva**:

    assert!(h_flare > 0.0, "flare sem altura — a aeronave pousa duas vezes");

Hoje não existe `h_flare` como grandeza; o teste RED é escrito contra a
função nova e falha por não compilar, o que **não é prova de defeito**.
Portanto a prova documental do §1.1 é uma **SONDA**: um cálculo que mostra que
`s_air(3°) = 286,22 m` já cobre os 15 m inteiros e que `s_flare = 53,60 m` é
somado com altura zero. **A implementação DEVE colar a saída dessa sonda no
relatório**, e a sonda é apagada depois. Não fabrique um teste RED artificial.

### §5.2 — Fechamento geométrico (a guarda central, viva)

    (15.0 − h_flare) − s_air·tan(γ_app)  ≈ 0     a 1e-9
    h_flare > 0
    s_air   > 0
    h_flare < 15.0

Falseável só parcialmente: se alguém reintroduzir um flare sem altura, ou
trocar o sinal, ou somar em vez de subtrair, isto quebra. **Mas o fechamento
é TAUTOLÓGICO** — `s_air` foi DEFINIDO como `(15−h)/tan γ`, então a identidade
vale por construção. Ele pega typo, não erro de modelo. Declarado, não fingido.

### §5.2b — O arco é mesmo um arco (a guarda que NÃO é tautológica)

Acrescentada após a revisão de plano, justamente porque a §5.2 não basta.

O flare é um arco de círculo de raio `R`, tangente à rampa no início e
horizontal no fim. Para esse arco vale, exatamente:

    s_flare² + (R − h_flare)² = R²

`h_flare` e `s_flare` chegam por caminhos independentes (`R(1−cos γ)` e
`R sin γ`); Pitágoras os amarra. **Trocar seno por cosseno, errar o sinal do
`1−cos`, ou usar raios diferentes nos dois quebra isto — e nenhum desses erros
seria pego pela §5.2.** `R` tem que ser recomputado no teste a partir de
`V_ref`, não lido do resultado da função.

### §5.3 — O flare não pode começar acima do obstáculo

Com `n → 1⁺`, `R → ∞` e `h_flare → ∞`. Existe portanto um `n` mínimo para
cada `γ_app`. Se `h_flare ≥ 15`, o resultado tem que ser
**`f64::INFINITY`** (pouso sobre obstáculo de 15 m impossível nesta
condição — resultado FÍSICO), **nunca** um `s_air` negativo e nunca NaN.
Testar com um `n` deliberadamente pequeno.

`f64::INFINITY` em `ldg_50ft_m`/`ldg_50ft_grass_m` **já serializa como a
string `"infinita"`** (`fatigue_life_serde`, ciclo 12) — verificar que
continua valendo.

### §5.4 — Monotonicidades estritas

- `γ_app` maior ⟹ `s_air` estritamente MENOR (numerador cai, denominador
  sobe — as duas na mesma direção).
- `n` maior ⟹ segmento aéreo total estritamente MENOR (§4.1).
- `V_ref` maior ⟹ `R` maior ⟹ `s_flare` maior.
- L/D maior (menos arrasto) ⟹ `γ_app` menor ⟹ segmento aéreo MAIOR. Contra-
  intuitivo e por isso valioso: uma aeronave mais limpa pousa em MAIS espaço
  a partir de 15 m, porque plana melhor.

### §5.5 — Consistência de polar

O `CD_ref` da aproximação tem que vir da MESMA função que a rolagem de pouso
usa (`cd_gear_extended`), avaliada no CL da aproximação. Testar que chamar a
função com `CL_ref` reproduz o `CD_ref` usado no ângulo — proíbe que alguém
plante uma segunda polar de aproximação.

### §5.6 — Pins: `old→new`, TOLERÂNCIAS INALTERADAS

Todo pin que mudar ganha bloco `old→new` (valor antigo, novo, delta, causa).
Nenhuma tolerância pode ser alargada. Asserção relacional apagada é achado:
escreva a relação nova e verdadeira, viva, no lugar.

---

## §6 — Schema 5.7 e migração

### §6.0 — Classificação: MINOR com EXCEÇÃO REGISTRADA

**Correção da revisão de plano.** A 1ª versão desta spec dizia "MINOR puro,
sem exceção registrada". Errado, e contra o precedente que o próprio
`docs/aircraft_spec.schema.md` estabeleceu em v5.2, v5.3, v5.4, v5.5 e v5.6.

O bump tem **duas naturezas ao mesmo tempo**:

- **MINOR puro (aditivo):** os três campos novos do §6.1. Nada a registrar.
- **EXCEÇÃO REGISTRADA:** `ldg_50ft_m` e `ldg_50ft_grass_m` mantêm nome, tipo
  e unidade, mas **a PREMISSA DE OPERAÇÃO embutida no número muda**. Antes,
  esses campos respondiam "quanto a aeronave precisa numa aproximação
  estabilizada de 3° **com potência**". Depois, respondem "quanto ela precisa
  num pouso de campo curto, **motor em marcha lenta** sobre o obstáculo".

Essa distinção importa mais que "o valor mudou" — todo ciclo muda valores, e
isso sozinho nunca exigiu exceção. O que exige registro aqui é que **um
consumidor comparando v5.6 com v5.7 estaria comparando dois PROCEDIMENTOS
diferentes**, não duas medições do mesmo procedimento. Sem a exceção
registrada, essa troca de premissa fica invisível de fora do modelo.

### §6.1 — JSON

Campos NOVOS no bloco `performance` (a metade MINOR pura do bump):

    ldg_approach_angle_deg    // γ_app derivado — 5,1181 no baseline
    ldg_flare_height_m        // h_flare — 2,596 no baseline
    ldg_air_distance_m        // s_air + s_flare — 196,57 no baseline

Publicar os três torna o segmento AUDITÁVEL de fora: hoje ele é 52,5% da
distância de pouso e não aparece em lugar nenhum do JSON.

### §6.2 — Config: duas migrações com erro explícito

    [performance].approach_angle_deg  → REMOVIDO (γ_app agora é derivado)
    [performance].flare_time_s        → REMOVIDO, substituído por
    [performance].flare_load_factor   = 1.20   (novo)

Ambas com guarda de migração nomeada (padrão de `check_shaft_height_migration`
em `src/models/config.rs`). Validação de `flare_load_factor`:
**estritamente > 1,0** (em `n = 1` o raio diverge) e < 2,0 (fator de carga de
flare acima disso não é pilotagem de pouso).

Atualizar também o comentário de `aircraft_config.rs:305`, que descreve
`rotation_attitude_deg` citando "`[performance] rotation_time_s/flare_time_s`,
tempos — não ângulos — do mesmo evento". Metade dessa frase morre aqui.

---

## §7 — Resultado esperado (projeção do chefe — NÃO consertar)

| Grandeza | Hoje | Projetado | Confiança |
|---|---|---|---|
| `ldg_approach_angle_deg` | — | **5,1181°** | alta |
| `ldg_flare_height_m` | — | 2,596 m | alta |
| `ldg_air_distance_m` | 339,82 | **196,57** | alta |
| `ldg_50ft_m` | 582,521767 | **≈439,3** | alta |
| `ldg_50ft_grass_m` | 646,660942 | **≈503,4** | alta |
| **Checagem #24 (pouso grama)** | ❌ 647 > 600 | **✅ PASSA** | alta |
| **Total de violações** | 5 | **4** | alta |
| `landing_distance_m` (legado) | 442,702122 | **inalterado** | alta |
| Decolagem, cruzeiro, subida, rotação, massa | — | **INALTERADOS** | alta |

**Este é o primeiro ciclo desde o 11 que REMOVE uma violação.** Isso é
legítimo: a violação era sustentada por um erro geométrico e uma premissa de
aeroporto pavimentado, ambos nomeados e medidos. **Não é afrouxamento de
premissa** — a premissa de pista de 600 m em grama permanece INTACTA, e três
violações continuam de pé, incluindo a decolagem em grama a 859 m.

**Se algo fora do pouso se mover, é ACHADO, não efeito** (§3) — parar e
escalar.

---

## §8 — Fora de escopo

1. Velocidade inicial da rolagem de pouso (§3) — conservador, **registrar
   como backlog novo**.
2. Efeito solo (backlog #12).
3. `j_design` obsoleta (#18), pico de eficiência (#19), dois rpms (#20),
   residual de missão (#21), nome de teste (#22).
4. Remoção de `to_distance_*`/`landing_distance_m` (#11) — MAJOR.
5. Pin órfão vs JSON regenerado no `verifica-ciclo.sh` (#13) — **avaliado e
   deliberadamente NÃO incluído**: fazer isso direito exige uma fonte única
   de constantes compartilhada entre testes e JSON, o que é um refactor
   próprio. Misturá-lo num ciclo de física é como o ciclo 12 acumulou risco.

---

## §9 — Restrições globais

- Rust 2021, sem dependência nova. `cargo test` inteiro verde ao fim de cada
  task.
- **Nunca hardcodar dado de motor/célula em `src/`** — `tests/acceptance.rs`
  faz grep e reprova.
- **Nunca mascarar achado.** Escalar (parar e reportar) se: um número diverge
  >5% do projetado no §7; um número FORA do pouso se move; uma tolerância ou
  assert é alterado; um gate flipa de forma não explicada.
- Pins: `old→new` com causa. **Tolerâncias INALTERADAS.**
- `scripts/verifica-ciclo.sh` tem que voltar "Status geral: APROVADO".
- Trailers de commit:
  `Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>`
  `Claude-Session: https://claude.ai/code/session_01J8DCAdnLPaBhTHpu1rTQaT`
