# Ciclo 15 — o porteiro que prova (backlog #13)

**Data:** 2026-08-16
**Backlog:** #13 (pin órfão mascarado por tolerância) — e a **retratação** da
sua segunda manifestação, registrada erradamente no ciclo 14.
**Schema:** SEM bump. Este ciclo não muda nenhum campo do JSON.
**Invariante central:** `aircraft_spec.json` deve terminar o ciclo **byte-idêntico**
ao de `b8827e8`. Se ele mudar, alguma coisa saiu do escopo.

**Achados desta spec, antes de uma linha de código:**

| # | achado | §|
|---|---|---|
| 1 | A proposta do backlog ("regenerar o JSON e comparar") **já existe e é exata** — `tests/cli.rs:943`. O buraco é outro. | §1.1 |
| 2 | A "segunda manifestação" do #13, que **eu** registrei no ciclo 14, é **falsa**. Retratação. | §2 |
| 3 | Quatro blocos do schema doc rotulados "HOJE" estão obsoletos — o pior por **44,5%**. | §3 |
| 4 | **Dois pins vivos nunca bateram com o pipeline**, em commit nenhum. Terceira variante da doença. | §7.4 |
| 5 | O piso tipográfico do desenho aprovado não cobria **nenhum** dos defeitos reais. Regra trocada por prova. | §5.5 |

---

## 1. O problema, medido

Um pin de `ldg_50ft_m` em `tests/generic_engine.rs` ficou dessincronizado do
`aircraft_spec.json` desde o ERRATUM do ciclo 11 (`8f92c55`): pin `502,431095`
contra JSON `502,458299`, **0,0054%**. Sobreviveu **quatro commits**.

Não foi azar. Foi mecânica: os pins deste projeto usam tolerância de **1%** para
absorver ruído numérico legítimo (compilador, plataforma, convergência de laço).
A mesma tolerância que absorve ruído absorve desatualização. O teste fica verde
enquanto o número plantado no código já não é o que o pipeline produz.

O mesmo mecanismo, sem nem a tolerância para culpar, opera na documentação:
**nenhuma checagem valida os números de `docs/aircraft_spec.schema.md`.**
Confirmado por varredura em `tests/` (três referências textuais ao arquivo, em
comentário; nenhuma leitura) e em `scripts/verifica-ciclo.sh` (não toca `docs/`).

### 1.1 O que o porteiro prova hoje — mapa corrigido

| Camada | Prova? | Mecanismo |
|---|---|---|
| `aircraft_spec.json` commitado ≡ pipeline real | **SIM, exata** | `tests/cli.rs:943`, `aircraft_spec_json_commitado_bate_com_o_pipeline_real` |
| Pins de `tests/*.rs` ≡ JSON | **NÃO** | tolerância de 1% absorve deriva |
| Números de `docs/*.md` ≡ JSON | **NÃO** | não existe checagem |

A primeira linha corrige a proposta do próprio backlog #13, que pedia uma seção
nova em `verifica-ciclo.sh` para "regenerar o JSON e comparar". **Isso já
existe, e melhor:** `tests/cli.rs:943` roda o binário com os três TOMLs reais e
compara o resultado contra o `aircraft_spec.json` commitado via
`serde_json::Value` (`PartialEq`), com a feature `float_roundtrip` do
`serde_json` garantindo ida-e-volta f64 bit-exata. **Tolerância zero.** O
doc-comment do teste (`tests/cli.rs:925-943`) documenta a escolha: estrutural em
vez de byte-a-byte para não falsificar em reordenação de chave, mas exata para
números.

**Consequência de desenho, e é a alavanca deste ciclo:** se JSON ≡ pipeline é
exato, então checar um pin contra o **JSON commitado** é equivalente a checá-lo
contra o pipeline — sem regenerar nada, e **sem tolerância**, porque entre dois
arquivos commitados não existe ruído de compilador para absorver.

A cadeia fica: `pipeline ≡ JSON` (exato, já existe) `≡ pin` (exato, este ciclo).
Por transitividade, `pipeline ≡ pin`, exatamente — enquanto a tolerância de 1%
de cada teste permanece **INALTERADA**, guardando o que sempre guardou:
pipeline-vs-plataforma.

---

## 2. RETRATAÇÃO — a segunda manifestação do #13 é falsa

O commit `5119592` (fix wave do ciclo 14) registrou no backlog #13 uma "segunda
manifestação, agora na documentação", afirmando que
`docs/aircraft_spec.schema.md:809-810` registra `ldg_50ft_m = 582,341118` e
`ldg_50ft_grass_m = 646,437301` quando "os valores reais imediatamente antes do
ciclo 14 eram 582,521767 e 646,660942".

**Isso está errado. Não há deriva nessas linhas.** Verificado commit a commit:

| Commit | schema | `ldg_50ft_m` | `ldg_50ft_grass_m` |
|---|---|---|---|
| `619b4a0` (bump v5.5) | 5.5 | 502,4582990603992 | 556,6771728943696 |
| `e06e7e7` | 5.5 | **582,3411181885572** | **646,4373010985819** |
| `1755e35` | 5.5 | 582,3411181885572 | 646,4373010985819 |
| `cf7af14` | 5.5 | 581,9677435047217 | 645,9750729243794 |
| `0a6136f` | 5.5 | **582,5217673279861** | **646,6609422476247** |
| `1264b7f` … `7d246b3` | 5.6 | 582,5217673279861 | 646,6609422476247 |

As linhas 809-810 narram a transição `502,458299 → 582,341118` **dentro da era
v5.5**, e ambos os valores existiram de fato. O par `582,521767 / 646,660942` é
de um momento **posterior** (pós-ERRATUM do ciclo 13) e o documento o registra
corretamente em **outro lugar** — a entrada v5.7, linha 981, e a cadeia
`old→new` das linhas 1433-1434.

**A causa do erro:** comparei um valor da era v5.5 contra um valor pós-ciclo-13
e chamei a diferença de deriva. Não abri o histórico antes de afirmar sobre ele.

É a mesma família do erro §5.3 do ciclo 14 — afirmar sobre o histórico sem
consultá-lo — com o sinal invertido: lá afirmei uma correção que não existia,
aqui afirmei um defeito que não existia. **A lição do ciclo 14 ("uma afirmação
de 'isso já está resolvido' desativa uma checagem") tem gêmea: uma afirmação de
'isso está quebrado' fabrica trabalho e envenena o backlog.** Ambas se curam com
a mesma disciplina: quem afirma sobre o histórico abre o histórico.

**Ação:** reescrever o bloco "SEGUNDA MANIFESTAÇÃO" do backlog #13 como
retratação explícita, preservando o texto original citado para que a arqueologia
sobreviva, e substituí-lo pelos defeitos reais da §3.

---

## 3. Os defeitos reais da documentação

Quatro blocos rotulados como valor **atual** e obsoletos. Todos verificados
contra o `aircraft_spec.json` de `b8827e8`.

| Local | O doc diz | JSON hoje | Erro |
|---|---|---|---|
| `:1236` | `cg_limit_fwd_pct_mac` = **17,757974%** "HOJE" | **18,268251143882534** | 2,87% |
| `:1236` | `cg_limit_aft_pct_mac` = 43,460036% | 43,460036166746164 | correto |
| `:1424` | `rc_sl_ms` "Baseline real: **4,999905**" | **3,460340693496421** | **44,5%** |
| `:1429` | `vy_kmh` "Baseline real: **148,435393**" | **167,4067945715867** | **12,8%** |
| `:1429` | `vx_kmh` 138,871480 | 138,91407679224818 | 0,031% |
| `:1601-1603` | flip "Medido HOJE: `limite` = **18,094655%** vs `limite_nominal` = **17,757974%**" | `limite` = **18,47233349501252**, `limite_nominal` = **18,268251143882534** | — |

O bloco `:1601-1603` erra **também o cenário**: o texto descreve o flip como
sendo de "Solo (piloto)" (estado do ciclo 12), mas `robustness.flips` hoje traz
`"Cenário '2 pax dianteiros'"` — "Solo (piloto)" migrou para violação NOMINAL de
envelope no ciclo 13.

Três dos quatro pararam nos ciclos 11-12 e não acompanharam a mudança do modelo
de tração do ciclo 13. O doc chega a **contradizer a si mesmo**: a linha 1236 diz
17,757974% "HOJE" enquanto a linha 1381 diz corretamente 18,268251% "o valor
publicado HOJE".

**Nota de escopo:** `:1431-1432` cita `climb_gradient_pct` 12,451842 — hoje
7,913277 — mas dentro de uma narrativa explicitamente rotulada "ciclo 11", sem
reivindicar ser o valor vigente. **Não é defeito**; é desatualização por omissão.
Fica coberto pela marcação da §5 apenas se receber marcador de valor atual, o
que ele NÃO deve receber. Registrar em `docs/backlog.md` como item novo
(legibilidade do doc), não corrigir neste ciclo.

---

## 4. Princípio de desenho — por que NÃO uma tabela única

A resposta DRY seria uma tabela única de pins, lida tanto pelos testes quanto
pela checagem: cada valor escrito uma vez, deriva estruturalmente impossível.

**Rejeitada.** Um pin cujo valor é lido do artefato que ele deveria vigiar deixa
de ser um pin. O que dá poder ao literal `167.4067945716` é que **uma pessoa o
escreveu à mão**: se o pipeline parar de produzi-lo, alguém tem de decidir
conscientemente mudá-lo. Derivar o pin do JSON transforma a asserção num espelho
— sempre verdadeira, sempre inútil.

A duplicação entre pin e JSON **é a função**, não o defeito. O que falta não é
removê-la, é **detectá-la quando diverge**.

Corolário, que governa toda a §5: as checagens deste ciclo **leem, nunca
escrevem**. Nenhuma ferramenta deste ciclo pode atualizar um pin automaticamente.
Um porteiro que conserta o que deveria denunciar não é porteiro.

---

## 5. Arquitetura

### 5.1 Onde as checagens vivem

**Testes Rust em `tests/`, não seções de `scripts/verifica-ciclo.sh`** —
contrariando a proposta literal do backlog #13.

Razão: das seis seções do script, **quatro são INFORMATIVAS** e não reprovam
nada (regeneração do JSON, veredito do modelo, números que cascateiam, e o
resumo). O que reprova é `cargo test` (Seção 1) e o grep de genericidade
(Seção 4). Uma checagem nova como seção do script seria mais uma coisa impressa;
como teste, entra no portão automaticamente, roda em qualquer máquina, e roda
para quem executa `cargo test` sem passar pelo script.

Arquivo novo: **`tests/pins_vs_json.rs`**.

### 5.2 Gramática do marcador

Duas formas, ambas comentários no arquivo que já contém o literal:

**Forma vinculada** — o literal deve casar com um caminho do JSON:
```
PIN: <caminho.no.json>
```

**Forma isenta** — o literal declaradamente não é um valor publicado:
```
PIN: NAO-PUBLICADO — <razão em uma linha>
```

Em Rust o marcador é `// PIN: …`; em Markdown, `<!-- PIN:… -->`.

**Vínculo (Rust):** o marcador vale para o **primeiro literal de ponto flutuante
elegível** encontrado (a) no resto da própria linha, se o marcador for
comentário de fim de linha, ou (b) na linha imediatamente seguinte, se o
marcador ocupar a linha inteira.

Um literal é **elegível** quando não está em nenhuma destas três posições:

1. **Dentro de string.** `"VA {:.1} km/h fora do pin (~242.633)"` não contém
   literal — contém texto. Sem esta exclusão, quase toda asserção deste
   repositório ficaria ambígua, porque o costume da casa é repetir o pin na
   mensagem de erro.
2. **Em posição de tolerância** — imediatamente após `<`, `<=`, `>`, `>=`.
3. **Em comentário** — à direita de `//`, exceto o próprio marcador.

Sublinhados de legibilidade (`7.236_831_147`) contam como dígitos. Notação
científica (`1e-9`, `5e-5`) nunca é elegível.

**Sinal.** Um `-` imediatamente colado ao literal faz parte dele quando o
caractere não-branco anterior é `(`, `,`, `=`, `[` ou início de linha — menos
unário, não subtração. Sem isso, `assert!((vn.n_lim_neg - (-1.52)).abs() …)`
vincularia `1.52` contra um JSON que publica `-1.52` e reprovaria um pin
correto. Em `(obtido - 0.007367)` o `-` está separado por espaço e é binário:
não entra.

**Não há piso de casas decimais no vínculo.** A versão anterior desta seção
exigia ≥4 casas, o que a tornava incapaz de vincular `3.59` — justamente o
literal da §7.4. Piso tipográfico é critério de cobertura (§5.5), nunca de
vínculo.

**Ambiguidade — definida sobre COBRADOS, não sobre elegíveis.** O vínculo é ao
primeiro literal **cobrado** (§5.5) da linha, e a checagem só reprova por
ambiguidade se houver mais de um **cobrado**.

A distinção é obrigatória, não cosmética. Na linha
`("vy_kmh", perf.vy_kmh, 167.4067945716, 0.01),` há dois elegíveis, mas apenas
`167.4067945716` é cobrado (`0.01` tem 2 casas e a linha não contém `assert`).
Se a ambiguidade fosse definida sobre elegíveis, as oito linhas da tabela de
pins de `generic_engine.rs:1735-1742` reprovariam todas — a construção mais
importante do inventário.

Medido em `b8827e8` com o algoritmo completo **e as duas correções da §7.5**, a
ambiguidade ocorre em **duas linhas**, e ambas são isentas:

- `generic_engine.rs:2533`, `assert_eq!(fom.at(0.0), 0.75)` — os dois cobrados
  são entradas de config.
- `empennage.rs:117`, `assert!((9.2..10.2).contains(…))` — piso e teto de uma
  banda de aceitação. **Esta linha só tem dois cobrados depois do FIX 1**; antes
  dele o `10.2` era invisível, e a ambiguidade não aparecia porque metade do
  problema estava escondida.

A checagem **só reprova por ambiguidade quando o marcador é vinculado**, então
as duas passam.

Exemplos reais deste repositório:

```rust
// linha inteira, vincula à linha seguinte
// PIN: propulsion.endurance_h
let endurance_pin_h = 7.236_831_147;

// fim de linha, vincula no resto da própria linha
("vy_kmh", perf.vy_kmh, 167.4067945716, 0.01), // PIN: performance.vy_kmh
```

Na segunda forma, `0.01` tem 2 casas e não é candidato; `167.4067945716` é o
primeiro com ≥4. O vínculo é não-ambíguo.

**Vínculo (Markdown):** o marcador vincula ao **primeiro número** que aparece
depois dele no arquivo, aceitando vírgula decimal e separador de milhar por
ponto (`18.472,333` não ocorre hoje, mas `18,472333` sim).

```markdown
`limite` = **<!-- PIN:robustness.flips.0.limite -->18,472333% MAC**
```

**Caminho JSON:** segmentos separados por ponto; índice inteiro indexa array
(`robustness.flips.0.limite`). Caminho inexistente é **falha do teste**, não
aviso — um marcador que aponta para lugar nenhum é pior que marcador nenhum,
porque parece cobertura.

### 5.3 Checagem 1 — pins de teste ≡ JSON, exato

Teste `pins_de_teste_batem_com_o_json_commitado` em `tests/pins_vs_json.rs`.

1. Lê todo `tests/*.rs` **exceto `tests/pins_vs_json.rs`** (evita que os
   literais de exemplo do próprio checador virem alvo).
2. Extrai cada marcador `// PIN:` e o literal vinculado.
3. Para a forma vinculada: resolve o caminho em `aircraft_spec.json` e exige
   **igualdade na precisão escrita** — o valor do JSON, arredondado ao número de
   casas decimais que o literal exibe, deve ser exatamente igual ao literal.
4. Para a forma isenta: nada é comparado; a razão é apenas registrada.

**Por que "arredondado à precisão escrita" e não igualdade bit-a-bit:** os pins
são escritos truncados por legibilidade (`138.9140767922` para
`138.91407679224818`, `7.236_831_147` para `7.2368311470…`). Exigir todos os 17
dígitos seria uma mudança de estilo disfarçada de checagem. Arredondar à
precisão exibida detecta deriva a partir do último dígito escrito — que para
`138.9140767922` é 1e-10 relativo, oito ordens de grandeza mais apertado que a
deriva de 0,0054% que originou o #13.

**Mensagem de falha obrigatória:** deve nomear arquivo, linha, caminho JSON,
valor do pin, valor do JSON e o desvio relativo, e dizer explicitamente que a
correção é **decidir** se o pin deve mudar — nunca "rode X para atualizar".

### 5.4 Checagem 2 — números do doc ≡ JSON, na precisão impressa

Teste `numeros_atuais_do_schema_doc_batem_com_o_json` em `tests/pins_vs_json.rs`.
Mesma mecânica sobre `docs/aircraft_spec.schema.md`.

Marcar **todas** as citações de valor atual, não só as quatro quebradas — as
corretas de hoje são as candidatas a deriva de amanhã. Sítios conhecidos
(verificados como corretos, recebem marcador sem alteração de valor):
`:1137` `propulsion.prop_efficiency`; `:1381` `trim.rotation_limit_pct_mac`;
`:1410` `trim.flare_limit_pct_mac`; `:1435-1437` `performance.ldg_approach_angle_deg`
/ `ldg_flare_height_m` / `ldg_air_distance_m`; `:1504`
`propeller.prop_clearance_critical_m`.

### 5.5 Checagem 3 — o cadeado de cobertura (Rust)

Teste `todo_literal_longo_em_teste_carrega_marcador`.

**Regra:** em código executável de `tests/*.rs`, todo literal de ponto flutuante
que esteja **numa linha contendo `assert`** OU tenha **≥4 casas decimais** deve
carregar marcador — vinculado ou isento. Comentários são ignorados (linha
iniciada por `//`, `///` ou `*`, e tudo à direita de `//` numa linha de código,
exceto o próprio marcador).

Valem as três exclusões de elegibilidade da §5.2 — string, tolerância,
comentário — mais uma quarta, de granularidade de arquivo:

- **Arquivo isento.** Um marcador de módulo `//! PIN: NAO-PUBLICADO — <razão>`
  no topo do arquivo isenta o arquivo inteiro. Único uso hoje:
  `tests/config_files.rs`, que é round-trip de parsing de TOML de ponta a ponta
  — compara struct contra o literal do próprio arquivo de config, nunca saída de
  pipeline. São 50 literais que nada acrescentariam.

**ERRATUM — a regra mudou depois da aprovação do desenho, e por prova.** A
versão apresentada usava só o piso de ≥4 casas decimais, e a própria §9
declarava como lacuna conhecida que pins com 3 casas ou menos escapariam. Ao
verificar o inventário da §7 contra o JSON (obrigação da §10.7 antecipada para
antes da implementação), **os dois únicos pins encontrados fora do valor
publicado têm 3 e 2 casas decimais** — `vn_diagram.va_kmh` e
`vn_diagram.n_gust_vc` (§7.4). A lacuna declarada não era hipotética: era
exatamente onde os defeitos moravam. Um piso que não cobre nenhum dos defeitos
conhecidos não é um piso, é um álibi.

A regra por `assert` os cobre porque é **semântica em vez de tipográfica**: o
que obriga um número a ser verificável não é quantos dígitos ele tem, é o fato
de alguém estar afirmando algo com ele.

Sem esta checagem, o ciclo limpa a deriva de hoje e reconstrói a fábrica de
deriva: o próximo pin nasce desguardado exatamente como nasceram os três já
encontrados.

**Contagem medida em `b8827e8`:**

| regra | sítios |
|---|---|
| só ≥4 casas (versão apresentada) | 40 |
| `assert` OU ≥4 casas, sem exclusões | 190 |
| … com tolerância excluída | 131 |
| … com arquivo isento | 140 |
| **… algoritmo completo da §5.2 + correções da §7.5** | **70 literais em 68 linhas** |

Distribuição dos 70: `generic_engine`=39, `control_surfaces`=8, `vn_diagram`=6,
`gear_tipback`=5, `propeller`=4, `acceptance`=3, `empennage`=3, `cli`=1,
`schema_v4`=1.

Destes, **47 são pins vinculáveis cobrados** e 23 recebem isenção. Some-se
`empennage.rs:42`, que é pin real mas não é cobrado pela regra e recebe marcador
voluntário: **48 vinculados**. Todos na §7.5.1, com caminho JSON resolvido e
valor conferido contra o `aircraft_spec.json` de `b8827e8`.

**A regra semântica custa o mesmo que o piso tipográfico.** O desenho aprovado
projetava 70 sítios sob a regra de ≥4 casas; a regra por `assert`, uma vez
aplicadas as exclusões de string, tolerância e arquivo, dá **exatamente 70**
também — mas 70 diferentes, cobrindo os dois defeitos que a outra deixava
passar. A troca não foi de custo por cobertura; foi de cobertura errada por
cobertura certa ao mesmo custo.

### 5.6 Checagem 4 — o cadeado de cobertura (Markdown)

Teste `afirmacao_de_valor_atual_no_doc_exige_marcador`.

**Regra:** em `docs/aircraft_spec.schema.md`, linha que contenha um dos gatilhos
`HOJE`, `Baseline real`, `valor publicado`, `Medido HOJE` **e** um número com
≥4 casas decimais **e** nenhum marcador `<!-- PIN:` reprova.

Medido em `b8827e8`, o gatilho ocorre em 9 linhas depois da 1000: `:1050`,
`:1236`, `:1362`, `:1381`, `:1410`, `:1424`, `:1429`, `:1504`, `:1601`.

O gatilho é confiável porque é **a própria afirmação de atualidade** que cria a
obrigação. Um número sem afirmação de atualidade é histórico e não deve ser
verificado contra o JSON de hoje — foi exatamente confundir essas duas classes
que produziu a retratação da §2.

---

## 6. Autoteste das checagens

Cada uma das quatro checagens deve provar que **falha** quando deve. Testes
dedicados, com entrada sintética em memória ou em arquivo temporário — **nunca
mutando arquivos do repositório**:

| Teste | Prova |
|---|---|
| `pin_divergente_reprova` | literal `502.431095` contra JSON `502.458299` → falha, e a mensagem cita ambos |
| `pin_com_caminho_inexistente_reprova` | `PIN: performance.campo_que_nao_existe` → falha |
| `literal_longo_sem_marcador_reprova` | linha `let x = 1.23456;` sem marcador → falha |
| `literal_curto_em_assert_sem_marcador_reprova` | `assert!((v - 3.59).abs() < 0.05);` sem marcador → falha |
| `tolerancia_nao_exige_marcador` | o `0.05` depois do `<` na linha acima não é cobrado |
| `arquivo_com_isencao_de_modulo_e_ignorado` | `//! PIN: NAO-PUBLICADO — …` no topo isenta o arquivo |
| `literal_em_comentario_nao_exige_marcador` | `// valor antigo: 582.341118` → passa |
| `notacao_cientifica_nao_e_vinculada` | `< 1e-9` não é candidato a vínculo |
| `range_nao_engole_o_segundo_operando` | `(9.2..10.2).contains(…)` devolve **dois** literais — regressão do FIX 1 da §7.5 |
| `string_de_multiplas_linhas_nao_vaza_literal` | a mensagem de `gear_tipback.rs:787-789`, cortada em duas linhas, não produz o fantasma `8.7855` — regressão do FIX 2 |
| `doc_com_gatilho_sem_marcador_reprova` | linha com "HOJE" e `12,345678` sem marcador → falha |
| `doc_com_valor_divergente_reprova` | marcador aponta `rc_sl_ms`, texto diz `4,999905` → falha |

Dois desses testes usam deliberadamente **números reais deste repositório**, não
casos inventados que a checagem pega por construção:

- `pin_divergente_reprova` usa o caso que originou o #13: `502,431095` contra
  `502,458299`, 0,0054%, que sobreviveu quatro commits sob tolerância de 1%.
- `literal_curto_em_assert_sem_marcador_reprova` usa `3.59` / `< 0.05` — o caso
  real da §7.4, e a razão pela qual a regra deixou de ser tipográfica.

Uma checagem que não pega os defeitos que a motivaram não está verificada; está
apenas escrita.

---

## 7. Inventário de pins vinculáveis

> **AVISO — §7.1, §7.2 e §7.3 estão SUPERSEDIDAS pela §7.5.** Foram levantadas
> por leitura, antes de o scanner existir, e a revisão de plano provou que a
> leitura errou: 11 sítios sem classificação, números de linha deslocados em até
> 143 linhas, e um pin listado que a regra sequer cobra. Ficam registradas
> porque a §7.4 argumenta a partir delas. **Quem for implementar usa a §7.5.**

Levantado sobre `b8827e8`. Classe **(a)** = mapeável a campo do
`aircraft_spec.json` do baseline real (Toyota + `baseline_4seat` + `default`).

### 7.1 `tests/generic_engine.rs`

| linha | expressão | literal | caminho JSON |
|---|---|---|---|
| 317 | `sized.prop.endurance_h` | `7.236_831_147` | `propulsion.endurance_h` |
| 363 | `sized.prop.range_km` | `2_026.312721` | `propulsion.range_km` |
| 549 | `margem_l` | `22.842_418_337_7` | `sizing.fuel_margin_l` |
| 1087 | `sized.state.mtow_kg` | `1_538.332_303_517_7` | `sizing.mtow_mission_kg` |
| 1090 | `sized.prop.endurance_h` | `7.236_831_147_0` | `propulsion.endurance_h` |
| 1093 | `sized.prop.fc_cruise_lph` | `32.334_594_416_6` | `propulsion.fc_cruise_lph` |
| 1096 | `sized.wb.oew_kg` | `899.119_934_921` | `weight.oew_kg` |
| 1168 | `v_max_kmh` | `291.076_341_562_7` | `performance.v_cruise_kmh` |
| 1735 | `perf.vx_kmh` | `138.9140767922` | `performance.vx_kmh` |
| 1736 | `perf.vy_kmh` | `167.4067945716` | `performance.vy_kmh` |
| 1737 | `perf.best_glide_kmh` | `173.3095981182` | `performance.best_glide_kmh` |
| 1738 | `perf.glide_ratio` | `15.9211771869` | `performance.glide_ratio` |
| 1739 | `perf.climb_gradient_pct` | `7.9132771517` | `performance.climb_gradient_pct` |
| 1740 | `perf.to_50ft_paved_m` | `704.0912242361` | `performance.to_50ft_paved_m` |
| 1741 | `perf.to_50ft_grass_m` | `858.5934246438` | `performance.to_50ft_grass_m` |
| 1742 | `perf.ldg_50ft_m` | `439.2750776989` | `performance.ldg_50ft_m` |
| 1799 | `hand_check_esperado_pct` | `7.913_277_151_7` | `performance.climb_gradient_pct` |
| 1870 | `to_distance_paved_novo_pin` | `719.6387552401` | `performance.to_distance_paved_m` |
| 1875 | `to_distance_grass_novo_pin` | `951.3920558516` | `performance.to_distance_grass_m` |
| 1898 | `landing_distance_pin` | `442.702_122_048_7` | `performance.landing_distance_m` |
| 2370 | `ws_actual_esperado` | `1_062.424_288_774_5` | `sizing.constraints.ws_actual_n_m2` |
| 2562 | `const ETA_ANCORA` | `0.783_881_496_567_659_82` | `propulsion.prop_efficiency` |

### 7.2 Outros arquivos

| arquivo:linha | expressão | literal | caminho JSON |
|---|---|---|---|
| `cli.rs:912` | `structure.flutter_speed_kmh` | `698.5` | `structure.flutter_speed_kmh` |
| `gear_tipback.rs:182` | `gear.tipback_angle_deg` | `16.7940` | `landing_gear.tipback_angle_deg` |
| `gear_tipback.rs:216` | `gear.tail_strike_margin_deg` | `13.1865` | `landing_gear.tail_strike_margin_deg` |
| `gear_tipback.rs:277` | `gear.nose_load_max_pct` | `21.8973` | `landing_gear.nose_load_max_pct` |
| `gear_tipback.rs:305` | `gear.nose_load_min_pct` | `11.2869` | `landing_gear.nose_load_min_pct` |
| `gear_tipback.rs:787` | `fuel_margin_pct` | `8.785_545_514_5` | `sizing.fuel_margin_pct` |
| `vn_diagram.rs:93` | `vn.va_kmh` | `242.633` | `vn_diagram.va_kmh` — **§7.4** |
| `vn_diagram.rs:94` | `vn.vc_kmh` | `280.0` | `vn_diagram.vc_kmh` |
| `vn_diagram.rs:95` | `vn.vd_kmh` | `350.0` | `vn_diagram.vd_kmh` |
| `vn_diagram.rs:100` | `vn.n_lim_pos` | `3.8` | `vn_diagram.n_lim_pos` |
| `vn_diagram.rs:101` | `vn.n_lim_neg` | `-1.52` | `vn_diagram.n_lim_neg` |
| `vn_diagram.rs:105` | `vn.n_gust_vc` | `3.59` | `vn_diagram.n_gust_vc` — **§7.4** |
| `propeller.rs:61` | `propeller.tip_mach_static` | `0.493` | `propeller.tip_mach_static` |
| `propeller.rs:63` | `propeller.tip_mach_cruise_helical` | `0.459` | `propeller.tip_mach_cruise_helical` |
| `propeller.rs:65` | `propeller.ground_clearance_m` | `0.240` | `propeller.ground_clearance_m` |
| `schema_v4.rs:330` | `propeller.prop_clearance_critical_m` | `0.007367` | `propeller.prop_clearance_critical_m` |
| `control_surfaces.rs:47` | `cs.aileron.span_m` | `2.0895` | `control_surfaces.aileron.span_m` |
| `control_surfaces.rs:49,76-84` | `cs.{aileron,flap,elevator,rudder}.area_m2` | `1.030418`, `1.962538`, `1.165835`, `0.459899` | `control_surfaces.<nome>.area_m2` |
| `empennage.rs:42` | `emp.s_horizontal_m2` | `3.134` | `empennage.s_horizontal_m2` |

**Atenção:** `vn_diagram.rs:93/105` (`242.633`, `3.59`) e `propeller.rs:61/63/65`
(`0.493`, `0.459`, `0.240`) têm menos de 4 casas decimais. O piso tipográfico da
versão original do desenho não os obrigaria; a regra por `assert` adotada na
§5.5 obriga. Os dois de `vn_diagram` são precisamente os defeitos da §7.4.

### 7.4 Pins encontrados FORA do valor publicado — as duas únicas mudanças autorizadas

Verifiquei os 44 pins das §7.1/§7.2 contra o `aircraft_spec.json` de `b8827e8`
na precisão em que cada um está escrito. **42 batem. Dois não.**

| sítio | pin escrito | JSON hoje | desvio | tolerância que escondeu |
|---|---|---|---|---|
| `tests/vn_diagram.rs:93` `vn.va_kmh` | `242.633` | **242,692244** | 0,0244% | `abs < 1.0` (0,41%) |
| `tests/vn_diagram.rs:105` `vn.n_gust_vc` | `3.59` | **3,572607** | 0,487% | `abs < 0.05` (1,4%) |

**E não são deriva.** Rastreado no histórico:

| commit | `va_kmh` | `n_gust_vc` |
|---|---|---|
| `8f92c55` (ERRATUM ciclo 11) | 242,618735 | 3,572607 |
| `ed537ae` (pré-ciclo 13) | 242,618735 | 3,572607 |
| `7d246b3` (pós-ciclo 13) | **242,692244** | 3,572607 |
| `b8827e8` (hoje) | 242,692244 | 3,572607 |

`n_gust_vc` está **imóvel em 3,572607** por toda a janela: o pin `3,59` nunca
bateu, em commit nenhum. E `242,633` não bate com o valor antigo (242,618735)
nem com o novo (242,692244) — o ciclo 13 apenas afastou mais um número que já
estava errado.

**Terceira variante da doença, e a pior das três.** O #13 original é um pin que
*era* certo e envelheceu. Estes são pins que **nunca foram o valor do
pipeline** — números estimados a olho, escritos dentro de uma tolerância larga o
bastante para nunca cobrarem a conta. Um pin envelhecido ao menos testemunha um
estado que existiu; um pin estimado não testemunha nada e ocupa o lugar de quem
testemunharia.

**Autorização explícita e fechada.** Estes dois literais — e **somente** estes
dois — podem mudar neste ciclo:

```
242.633  →  242.692244      // old→new, ciclo 15: pin nunca bateu com o pipeline
3.59     →  3.572607        // old→new, ciclo 15: pin nunca bateu com o pipeline
```

Sob as regras de pin honesto do projeto: `old→new` comentado no sítio, com a
razão, e **tolerâncias INALTERADAS** (`abs < 1.0` e `abs < 0.05` permanecem
exatamente como estão). Reapertar a tolerância junto seria trocar um achado por
duas mudanças e perder a rastreabilidade de qual delas fez o quê.

Qualquer OUTRO pin que não bater é achado novo — **reportar, não consertar**
(§10.7).

### 7.3 Isenções obrigatórias (`NAO-PUBLICADO`)

Literais que **não** podem ser vinculados, com a razão que o marcador deve
carregar:

| sítio | razão |
|---|---|
| `generic_engine.rs:552` `9.631_747_034_0` | convenção de denominador diferente de `sizing.fuel_margin_pct` (% do exigido, não da capacidade) — ver comentário do próprio teste, linhas 465-478 |
| `generic_engine.rs:675` `293.331_418_679_4` | `max_level_speed_ms` com massa sintética 1.461 kg, não o MTOW convergido |
| `generic_engine.rs:1355` `3740.0919357761986` | tração estática isolada em V=0; não vira campo do JSON |
| `generic_engine.rs:2172` `236.863_067_404_9` | config MUTADA em memória (tanque 240 L), JSON hipotético |
| `generic_engine.rs:2290` `381.902_830_668_4` | caminho de erro Rotax; falha antes de gerar relatório |
| `generic_engine.rs:2494` `1.4621` | diferença entre dois limites de rotação recomputados; não é campo único |
| `generic_engine.rs:2527-2542` | `fom_static`/`fom_design`/`j_design`/`flare_load_factor` são **entradas** de config, não ecoadas no relatório |
| `acceptance.rs:228-278` `994.067254`, `75.218966`, `71.069629` | cenário **Rotax + missão ferry**, não o par que gera o `aircraft_spec.json` commitado |
| `config_files.rs` (bloco inteiro) | round-trip de parsing: compara struct contra o literal do próprio TOML, nunca saída de pipeline |

---

### 7.5 ERRATUM — inventário refeito a partir do scanner (AUTORITATIVO)

A revisão de plano reprovou o ciclo com cinco achados. Três eram meus, e a raiz
dos três é a mesma: **construí o inventário por leitura e depois escrevi a regra,
em vez de rodar a regra e classificar o que ela devolvesse.** Um inventário de
cobertura que não vem do próprio verificador não é inventário — é palpite bem
apresentado.

Além disso a revisão achou **dois bugs no algoritmo da §5.2**, ambos com
ocorrência viva:

- **Operador de range.** A supressão de "dígito precedido por `.`" (que existe
  para não confundir campo com literal) engole o segundo operando de um range.
  Em `empennage.rs:117`, `assert!((9.2..10.2).contains(…))`, o literal `10.2`
  ficava **invisível ao scanner** — cobrado pela regra, jamais cobrado na
  prática. **Correção:** `.` precedido de outro `.` é range, não separador
  decimal, e não suprime.
- **String de múltiplas linhas.** A máscara roda por linha e reinicia o estado
  `em_string` a cada uma. Em `gear_tipback.rs:787-789` a mensagem de erro
  continua na linha seguinte via `\`, e o `8.7855` do TEXTO era colhido como se
  fosse código — um literal FANTASMA, que não existe. **Correção:** a máscara
  passa a ser por arquivo, carregando `em_string` de uma linha para a próxima.

**Medido, não presumido.** Reimplementei o scanner com as duas correções e rodei
sobre `b8827e8`:

| | literais | linhas | ambíguas |
|---|---|---|---|
| algoritmo com os dois bugs | 70 | 69 | 1 |
| **algoritmo corrigido** | **70** | **68** | **2** |

Os dois defeitos erram para lados opostos — um perde um literal real, o outro
inventa um falso — e por isso **a contagem de literais não se move**. Foi só ao
abrir por linha e por ambiguidade que a compensação apareceu.

É a segunda vez neste mesmo ciclo em que um agregado imóvel esconde movimento
embaixo dele: no #21, `range_km` varia 0,017% enquanto o gradiente atravessa um
limite de aeronavegabilidade; aqui, "70" fica parado enquanto um pin real some e
um fantasma nasce. **Duas ocorrências independentes da mesma armadilha, no mesmo
ciclo, tornam a regra geral: um número que não se move não é evidência de que
nada se moveu — é um convite a abrir o número.**

**Contagem corrigida: 70 literais em 68 linhas** — perde o fantasma de
`gear_tipback.rs:789`, ganha o `10.2` de `empennage.rs:117`. **47 vinculados
cobrados + 1 vinculado voluntário + 23 isentos.**

#### 7.5.1 Vinculados — 48 no total, todos conferidos contra `b8827e8`

| sítio | literal | caminho JSON |
|---|---|---|
| `cli.rs:913` | `698.5` | `structure.flutter_speed_kmh` |
| `control_surfaces.rs:44` | `2.0895` | `control_surfaces.aileron.span_m` |
| `control_surfaces.rs:45` | `1.0304` | `control_surfaces.aileron.area_m2` |
| `control_surfaces.rs:77` | `1.030418` | `control_surfaces.aileron.area_m2` |
| `control_surfaces.rs:78` | `1.962538` | `control_surfaces.flap.area_m2` |
| `control_surfaces.rs:79` | `1.165835` | `control_surfaces.elevator.area_m2` |
| `control_surfaces.rs:80` | `0.459899` | `control_surfaces.rudder.area_m2` |
| `control_surfaces.rs:138` | `0.0` | `control_surfaces.elevator.start_m` |
| `empennage.rs:42` † | `3.134` | `empennage.s_horizontal_m2` |
| `gear_tipback.rs:182` | `16.7940` | `landing_gear.tipback_angle_deg` |
| `gear_tipback.rs:216` | `13.1865` | `landing_gear.tail_strike_margin_deg` |
| `gear_tipback.rs:277` | `21.8973` | `landing_gear.nose_load_max_pct` |
| `gear_tipback.rs:305` | `11.2869` | `landing_gear.nose_load_min_pct` |
| `gear_tipback.rs:787` | `8.785_545_514_5` | `sizing.fuel_margin_pct` |
| `generic_engine.rs:316` | `7.236_831_147` | `propulsion.endurance_h` |
| `generic_engine.rs:362` | `2_026.312721` | `propulsion.range_km` |
| `generic_engine.rs:548` | `22.842_418_337_7` | `sizing.fuel_margin_l` |
| `generic_engine.rs:944` | `1_538.332_303_517_7` | `sizing.mtow_mission_kg` |
| `generic_engine.rs:984` | `7.236_831_147_0` | `propulsion.endurance_h` |
| `generic_engine.rs:1024` | `32.334_594_416_6` | `propulsion.fc_cruise_lph` |
| `generic_engine.rs:1085` | `899.119934921` | `weight.oew_kg` |
| `generic_engine.rs:1167` | `291.076_341_562_7` | `performance.v_cruise_kmh` |
| `generic_engine.rs:1735` | `138.9140767922` | `performance.vx_kmh` |
| `generic_engine.rs:1736` | `167.4067945716` | `performance.vy_kmh` |
| `generic_engine.rs:1737` | `173.3095981182` | `performance.best_glide_kmh` |
| `generic_engine.rs:1738` | `15.9211771869` | `performance.glide_ratio` |
| `generic_engine.rs:1739` | `7.9132771517` | `performance.climb_gradient_pct` |
| `generic_engine.rs:1740` | `704.0912242361` | `performance.to_50ft_paved_m` |
| `generic_engine.rs:1741` | `858.5934246438` | `performance.to_50ft_grass_m` |
| `generic_engine.rs:1742` | `439.2750776989` | `performance.ldg_50ft_m` |
| `generic_engine.rs:1799` | `7.913_277_151_7` | `performance.climb_gradient_pct` |
| `generic_engine.rs:1870` | `719.6387552401` | `performance.to_distance_paved_m` |
| `generic_engine.rs:1875` | `951.3920558516` | `performance.to_distance_grass_m` |
| `generic_engine.rs:1898` | `442.702_122_048_7` | `performance.landing_distance_m` |
| `generic_engine.rs:2370` | `1_062.424_288_774_5` | `sizing.constraints.ws_actual_n_m2` |
| `generic_engine.rs:2562` | `0.783_881_496_567_659_82` | `propulsion.prop_efficiency` |
| `generic_engine.rs:2587` | `2640.0` | `propulsion.engine_rpm_cruise` |
| `propeller.rs:57` | `1.76` | `propeller.diameter_m` |
| `propeller.rs:61` | `0.493` | `propeller.tip_mach_static` |
| `propeller.rs:63` | `0.459` | `propeller.tip_mach_cruise_helical` |
| `propeller.rs:65` | `0.240` | `propeller.ground_clearance_m` |
| `schema_v4.rs:330` | `0.007367` | `propeller.prop_clearance_critical_m` |
| `vn_diagram.rs:93` | `242.633` | `vn_diagram.va_kmh` — **§7.4** |
| `vn_diagram.rs:94` | `280.0` | `vn_diagram.vc_kmh` |
| `vn_diagram.rs:95` | `350.0` | `vn_diagram.vd_kmh` |
| `vn_diagram.rs:100` | `3.8` | `vn_diagram.n_lim_pos` |
| `vn_diagram.rs:101` | `-1.52` | `vn_diagram.n_lim_neg` |
| `vn_diagram.rs:105` | `3.59` | `vn_diagram.n_gust_vc` — **§7.4** |

† `empennage.rs:42` **não é cobrado** pela regra (3 casas, linha sem `assert`).
É um pin real e recebe marcador **voluntariamente** — marcar mais do que a regra
exige é sempre permitido; marcar menos, nunca.

**46 dos 48 batem.** Os dois que não são os da §7.4, e continuam sendo as duas
únicas mudanças de literal autorizadas.

**Os seis que o inventário lido tinha perdido**, todos conferidos e todos batendo:
`control_surfaces.rs:44` (2,0894999999999997), `:45` (1,0304181034482756),
`:138` (0,0), `generic_engine.rs:2587` (2640,0), `propeller.rs:57` (1,76) e
`empennage.rs:42` (3,1339657839114095). **Cinco deles são pins que batiam por
sorte** — nunca verificados por nada, e a spec afirmava cobertura sem os
cobrir.

#### 7.5.2 Isentos — 23, cada um com a razão que o marcador deve carregar

| sítio | literal | razão |
|---|---|---|
| `acceptance.rs:228` | `994.067254` | cenário Rotax + missão ferry, não o par que gera o JSON commitado |
| `acceptance.rs:229` | `75.218966` | idem |
| `acceptance.rs:275` | `71.069629` | idem |
| `control_surfaces.rs:161` | `2.0` | fator da fórmula (duas superfícies), não valor publicado |
| `empennage.rs:117` | `9.2` | piso de uma banda de aceitação, não campo |
| `empennage.rs:117` | `10.2` | teto da mesma banda |
| `empennage.rs:124` | `100.0` | conversão fração→percentual |
| `generic_engine.rs:124` | `111.0` | diferença OEW Toyota−Rotax, comparação sintética |
| `generic_engine.rs:552` | `9.631_747_034_0` | convenção de denominador diferente de `sizing.fuel_margin_pct` (% do exigido, não da capacidade) — ver comentário do teste, linhas 465-478 |
| `generic_engine.rs:674` | `293.3314186794` | `max_level_speed_ms` com massa sintética 1.461 kg, não o MTOW convergido |
| `generic_engine.rs:1355` | `3740.0919357761986` | tração estática isolada em V=0; não vira campo do JSON |
| `generic_engine.rs:2111` | `240.0` | config MUTADA em memória (tanque 240 L) |
| `generic_engine.rs:2171` | `236.863_067_404_9` | resultado da config mutada acima; JSON hipotético |
| `generic_engine.rs:2217` | `260.0` | caminho de erro Rotax; falha antes de gerar relatório |
| `generic_engine.rs:2289` | `381.902_830_668_4` | idem |
| `generic_engine.rs:2494` | `1.4621` | diferença entre dois limites de rotação recomputados; não é campo único |
| `generic_engine.rs:2527` | `0.75` | `fom_static` é ENTRADA de config, não ecoada no relatório |
| `generic_engine.rs:2528` | `0.815_976_999_245_887_96` | `fom_design`, entrada de config |
| `generic_engine.rs:2529` | `1.875_143_480_257_116_75` | `j_design`, entrada de config |
| `generic_engine.rs:2533` | `0.0` e `0.75` | **linha ambígua** (dois cobrados): avalia `FoM.at()` sobre entradas de config |
| `generic_engine.rs:2534` | `0.815_976_999_245_887_96` | idem |
| `generic_engine.rs:2542` | `1.20` | `flare_load_factor`, entrada de config |

Mais a isenção de módulo em `tests/config_files.rs` (§5.5).

**O fantasma some.** `gear_tipback.rs:789` (`8.7855`) não aparece nesta lista
porque, corrigido o bug da máscara, ele deixa de existir: é texto de mensagem de
erro, não literal.

---

## 8. Impacto no schema e no veredito

**Nenhum.** Este ciclo não toca `src/`, não muda nenhum campo, não muda nenhuma
tolerância, não muda `SCHEMA_VERSION` (permanece **5.7**).

Veredito projetado: **idêntico** — `validation_status: FAIL`, **4 violações**,
as mesmas de `b8827e8`.

**Invariante verificável e obrigatório no fecho:**
`git diff b8827e8 -- aircraft_spec.json` deve sair **vazio**.

Se o JSON mudar, alguém alterou comportamento sob o disfarce de uma tarefa de
verificação — exatamente a classe de coisa que este ciclo existe para tornar
impossível.

---

## 9. O que este ciclo NÃO cobre — declarado

1. **Literal fora de asserção e com ≤3 casas segue desguardado.** A regra da
   §5.5 obriga por `assert` OU por ≥4 casas; um literal curto que não esteja
   numa linha de `assert` — por exemplo alimentando uma variável usada só três
   linhas abaixo — escapa das duas. A construção em tabela de tuplas de
   `generic_engine.rs:1734-1749` é exatamente essa forma (o `assert` está no
   laço, não na linha do literal), e só é coberta hoje porque aqueles oito
   literais têm ≥4 casas. **Um pin novo escrito como tupla com poucas casas
   passaria.** É a lacuna residual conhecida, e não tenho regra sintática que a
   feche sem análise semântica de fluxo.

2. **Só `docs/aircraft_spec.schema.md`.** `docs/backlog.md` e
   `docs/superpowers/specs/*` citam centenas de números, quase todos históricos
   por natureza. Estendê-los pioraria a razão marcador/ruído sem fechar risco
   real.

3. **Vínculo errado é silencioso na forma isenta.** Se um marcador
   `NAO-PUBLICADO` se vincular ao literal errado, nada quebra — nada é
   comparado. Na forma **vinculada** o vínculo errado é auto-detectável: o valor
   não bate e o teste reprova.

4. **Cenários não-baseline seguem sem pin verificável.** Os três valores de
   `acceptance.rs` (Rotax/ferry) não têm artefato commitado contra o qual
   comparar. Ficam isentos, e a isenção declara isso.

5. **`:1431-1432` do doc** (`climb_gradient_pct` 12,451842, hoje 7,913277) fica
   como está — narrativa rotulada "ciclo 11", sem reivindicar atualidade.
   Registrar em `docs/backlog.md` como item de legibilidade.

---

## 10. Verificação de fecho

1. `scripts/verifica-ciclo.sh` → **Status geral: APROVADO**.
2. `git diff b8827e8 -- aircraft_spec.json` → **vazio**.
3. `git diff b8827e8 -- src/` → **vazio** (nenhuma mudança de comportamento).
4. Contagem de testes ≥ 519 + os novos.
5. Prova de que cada checagem reprova quando deve (§6) — a saída de falha é
   entregável, não basta afirmar que existe.
6. Prova de que o `pin_divergente_reprova` usa os números reais do #13.
7. **A checagem mais importante da lista.** `git diff b8827e8 -- tests/` só pode
   conter (a) adição de comentários `// PIN:` / `//! PIN:`, (b) o arquivo novo
   `tests/pins_vs_json.rs`, e (c) **exatamente dois** literais alterados — os da
   §7.4, `242.633 → 242.692244` e `3.59 → 3.572607`, cada um com seu `old→new`
   comentado.

   **Nenhuma tolerância pode mudar.** Nenhum outro literal pode mudar.

   A tentação natural de quem implementa é "corrigir" todo pin que não bate. A
   §7.4 já mediu quais não batem e a lista está fechada em dois. Se um terceiro
   aparecer, é achado novo: **reportar no relatório da task, não consertar.** Um
   pin consertado em silêncio é o mesmo evento que originou o #13, só que no
   sentido contrário — e desta vez dentro do ciclo que existe para impedi-lo.
