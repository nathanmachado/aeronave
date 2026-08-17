# Ciclo 16 — o veredito que admite não saber (backlog #21)

**Data:** 2026-08-16
**Base:** `bfd4921` (ciclo 15 — o porteiro que prova)
**Backlog:** #21 (âncora de cruzeiro preserva o ponto, missão residua; subida
usa a curva inteira)

---

## 1. O problema

O modelo publica

```json
"performance": { "climb_gradient_pct": 7.913277151685835 }
```

e carimba

```json
"validation_status": "FAIL",
"violations": ["Gradiente de subida 7.9% abaixo do mínimo de 8.3% exigido pela CS 23.65 (Vx=138.9km/h)", …]
```

com quinze dígitos significativos. Mas o gradiente da CS 23.65 é avaliado em
Vx ≈ 138,9 km/h → **J ≈ 0,82**, e a lei de tração do projeto é

```
FoM(J) = fom_static + (fom_design − fom_static)·min(J/j_design, 1)
```

com `j_design = 1,87514348025711675`. A lei é uma combinação convexa: com
`t = min(J/j_design, 1)`,

```
FoM(J) = (1 − t)·fom_static + t·fom_design
```

Em J = 0,82 → `t = 0,4373`, logo

```
FoM(0,82) = 0,5627·0,75 + 0,4373·0,81598 = 0,778853
∂FoM/∂fom_static = 1 − t = 0,5627
```

**No ponto da CS 23.65, 56,3% do peso está no parâmetro não calibrado.** Em
CRUZEIRO, `J ≈ 1,875 ≥ j_design` ⇒ `t = 1` ⇒ **`∂FoM/∂fom_static = 0`**: o
cruzeiro é literalmente insensível a `fom_static`.

> **Nota de auto-revisão.** No fechamento do ciclo 15 eu enunciei isto como
> "96,3% vêm de `fom_static`", decompondo `FoM = 0,75 + 0,0289` na forma
> aditiva `fom_static + Δ·t`. O número não é falso, mas a forma é
> retoricamente inflada: ela conta o nível inteiro do primeiro termo como se
> fosse influência marginal. A grandeza que responde "quanto o resultado
> depende do número não calibrado" é a **derivada**, e ela vale 0,5627, não
> 0,963. Numa spec sobre falsa precisão, a estatística inflada sai.
>
> O argumento não enfraquece — sobrevive com folga no número medido de §2.3.

Essa assimetria é a chave do #21. O cruzeiro está saturado em `fom_design`
(calibrado) e **não deveria** se mover; a subida está na região linear e
depende do parâmetro cru. É por isso que `range_km` se move apenas −0,037%
enquanto `fuel_climb_kg` se move +46,31%: o pouco que o cruzeiro se move não
vem da lei diretamente, vem do laço de convergência de MTOW realimentado pela
subida. O #21 descreveu esse resíduo; esta spec ataca a causa dele.

E `fom_static` é o único dos três parâmetros da lei que **nunca foi
calibrado**. A proveniência está escrita no próprio baseline
(`config/aircraft/baseline_4seat.toml:167-185`):

- `fom_design` e `j_design`: "retro-derivados UMA VEZ do polinômio JavaProp no
  ponto de cruzeiro do baseline E12", recalibrados por ponto fixo
  autoconsistente até `|Δfom_design| < 1e-9`, "convergiu em 7 iterações",
  resíduo `0,000e0`.
- `fom_static`: "**fator de McCormick em J=0** (era `static_thrust_factor =
  0.75`)". Herdado de um multiplicador plano. Dois algarismos significativos.
  Nenhuma derivação, nenhuma convergência, nenhum resíduo.

**O veredito de aeronavegabilidade mais crítico do projeto está pendurado no
único número da lei que ninguém calibrou** — e o modelo não tem nenhum meio de
dizer isso, porque o modelo só sabe dizer PASSA e FALHA.

Esta spec não calibra `fom_static`. Ela faz o modelo **declarar quando não
sabe**.

---

## 2. O que foi medido neste ciclo

Todos os números abaixo saíram de **rodar o pipeline completo** com uma cópia
do baseline em que a única linha alterada é `fom_static = …`. Nenhum foi
estimado, derivado de cabeça, ou copiado de ciclo anterior. Os scripts de
medição estão em `§10`.

### 2.1 Inventário de sensibilidade (produzido pela regra, não por leitura)

Varrendo `fom_static` de 0,70 a 0,82 e diffando **todas as folhas** do JSON:

| | contagem |
|---|---|
| campos numéricos que **se movem** | **89** |
| campos numéricos **imóveis** | **198** |

Maiores sensibilidades relativas:

| Δ | campo | 0,70 → 0,82 |
|---|---|---|
| **+94,21%** | `trim.rotation_margin_per_scenario[0].rotation_authority_margin_pct` | −0,847 → −1,645 |
| −18,39% | `performance.to_distance_grass_m` | 1040,75 → 849,31 |
| **+18,09%** | `performance.climb_gradient_pct` | 7,3587 → 8,6897 |
| −17,05% | `performance.to_50ft_grass_m` | 932,49 → 773,54 |
| +14,27% | `performance.rc_sl_ms` | 3,2670 → 3,7334 |
| −12,27% | `mission.fuel_climb_kg` | 7,6156 → 6,6808 |
| +4,26% | `performance.service_ceiling_m` | 4700 → 4900 |
| **+1,90%** | `weight.cg_limit_fwd_pct_mac` | 18,1246 → 18,4694 |
| +1,90% | `trim.rotation_limit_pct_mac` (e `_grass`, `_paved`) | idem |
| +1,90% | `robustness.flips[0].limite_nominal` | idem |
| +1,36% | `performance.v_cruise_kmh` | 289,37 → 293,29 |
| −0,0374% | `mission.range_km` | (fora dos 35 primeiros) |

A última linha é o achado original do #21: **o alcance quase não se move
enquanto o gradiente atravessa um limite de aeronavegabilidade.**

### 2.2 Erro meu do ciclo 15, corrigido aqui

No fechamento do ciclo 15 eu afirmei, sobre a varredura de `fom_static`:

> "CG e robustez são violações de peso/balanceamento, intocadas pela
> propulsão."

**É falso.** A tabela acima mostra `weight.cg_limit_fwd_pct_mac` movendo
+1,90% na banda. O CG do cenário 'Solo (piloto)' fica parado em 17,8 %MAC; é o
**limite** que anda:

| `fom_static` | limite dianteiro | CG do cenário |
|---|---|---|
| 0,70 | 18,12 %MAC | 17,8 %MAC |
| 0,75 | 18,29 %MAC | 17,8 %MAC |
| 0,82 | 18,47 %MAC | 17,8 %MAC |

A causa é física e já estava documentada no repositório: o limite dianteiro é
um **limite de rotação**, e o balanço de rotação carrega o termo `−T·z_eixo`
(`src/agents/trim_authority.rs:294`, `thrust_at_rotation_n`). Mais tração
estática ⇒ mais momento de picada ⇒ limite dianteiro recua para trás ⇒ a
margem PIORA.

O doc-comment de `RobustnessFlip::limite_nominal` (`src/models/specs.rs:855`)
já distinguia, com todas as letras, "**o CG andou**" de "**a régua andou**",
citando a linha de tração como causa da segunda. Eu afirmei o contrário do que
o próprio repositório documenta.

É a **sexta ocorrência** do padrão do backlog #29 — publicar, ao lado de uma
regra correta, uma lista derivada por outro método. Desta vez foi pega antes
de virar spec, e foi pega pelo único antídoto que funciona: **a lista foi
gerada executando a regra** (varrer e diffar as 287 folhas), não parafraseando
ela. Nenhuma leitura minha teria acertado 89 campos, e a minha teria errado
exatamente os três que decidem o desenho.

### 2.3 O breakeven da CS 23.65

Bisseção sobre o pipeline completo, 40 iterações:

```
gradiente = 8,3% exatamente em  fom_static ∈ [0,78486774238670476 ; 0,78486774238757784]
                                largura do bracket = 8,7e−13
```

**`fom_static` = 0,784867742387 — a +4,6490% do 0,75 de hoje.**

Nos dois lados do bracket, verificado rodando o pipeline:

| `fom_static` | gradiente | violação CS 23.65 | nº de violações |
|---|---|---|---|
| 0,784867742387 | 8,299999999992% | presente | 4 |
| 0,784867742388 | 8,300000000002% | ausente | 3 |

### 2.4 O que a tração NÃO conserta

Rodando também nos extremos do domínio fisicamente admissível:

| | 0,675 | **0,75 (hoje)** | 0,81598 | FoM ≡ 1,0 |
|---|---|---|---|---|
| gradiente CS 23.65 (mín 8,3%) | 7,0814% | **7,9133%** | 8,6450% | 13,0533% |
| decolagem grama 15 m (pista 600 m) | 974,68 m | **858,59 m** | 777,95 m | 521,72 m |
| limite dianteiro de CG (CG = 17,8) | 18,0527% | **18,2683%** | 18,4579% | 19,2773% |
| nº de violações | 4 | **4** | 3 | 2 |

Duas leituras importantes:

1. **CG e robustez falham até no teto de quantidade de movimento** — e falham
   PIOR: o limite dianteiro sobe de 18,27 para 19,28 %MAC enquanto o CG fica
   em 17,8. Nenhuma hélice conserta essas duas violações, e **o remédio óbvio
   para o gradiente (mais tração estática) piora as duas.** O modelo nunca
   disse isso, porque nunca varreu nada.

2. **A decolagem na grama não é consertável por `fom_static` sozinho.**
   Medido: com `fom_design` intacto, `fom_static = 1,0` (o teto absoluto do
   parâmetro) ainda dá **618,70 m** sobre 15 m em grama — acima dos 600 m.
   **Não existe raiz em (0 ; 1].** Os 521,72 m da coluna do teto exigem
   levantar a curva INTEIRA (as duas âncoras a 1,0), o que passa por
   `fom_design` — que é justamente a âncora calibrada, e portanto não é livre.

Resultado: das quatro violações do baseline, **exatamente uma** é
indeterminada. As outras três são determinadas contra o domínio físico
inteiro, não apenas contra a banda declarada.

---

## 3. Decisões do usuário

Tomadas no ciclo 15 e confirmadas na abertura deste:

1. **Tratamento do veredito da CS 23.65:** declarar **INDETERMINADO**. O
   checador de restrições ganha um terceiro estado; todo check cujo veredito
   vira dentro da banda é publicado como indeterminado, com o breakeven
   medido ao lado.
2. **Forma da banda:** tolerância relativa declarada de **±10%** sobre o
   nominal (`fom_static_tol_pct = 10.0`). O número declarado deixa de ser uma
   afirmação sobre física de hélice — que eu estaria inventando, já que não
   medi hélice nenhuma — e passa a ser **política de projeto**: quanto se
   confia numa entrada que nunca foi calibrada. Reaproveitável para qualquer
   outra entrada não calibrada.
3. **Teto físico:** incluir a corrida no teto de quantidade de movimento
   (FoM ≡ 1,0), para separar "falha que hélice nenhuma conserta" de "falha
   dentro do alcance de propulsão".
4. **Escopo da extração:** extrair o pipeline para a lib (`src/pipeline.rs`),
   consertando de quebra a duplicação de `tests/schema_v4.rs`.

---

## 4. Por que não existe versão barata

Tentador: rodar o pipeline nos extremos e comparar o **veredito global**. Se
mudou, INDETERMINADO; se não, o veredito está firme.

Isso não publicaria absolutamente nada. Medido:

| | 0,675 | 0,75 | 0,81598 | FoM ≡ 1,0 |
|---|---|---|---|---|
| `validation_status` | FAIL | FAIL | FAIL | FAIL |

O veredito global é FAIL em toda a banda **e no teto físico**, porque as
outras três violações o sustentam sozinhas. Um sweep de agregado devolveria
"veredito firme" e a indeterminação do gradiente continuaria invisível.

**Isso é literalmente a doença do #21**: o agregado esconde o segmento. O #21
foi aberto porque `range_km` variava −0,037% enquanto `fuel_climb_kg` variava
+46,31%. Resolver o #21 com um sweep de agregado seria reencenar o bug dentro
da própria correção.

Logo a comparação **tem** que ser por check. O que exige duas coisas que hoje
não existem: **identidade estável de check** (§5.2) e **o pipeline chamável
como função** (§5.3).

---

## 5. Desenho

### 5.1 A banda em config

`[propeller]` ganha **um** campo, obrigatório:

```toml
fom_static         = 0.75
# Tolerância epistêmica declarada sobre `fom_static` (%). NÃO é uma medição
# de hélice — é a declaração de quanto o projeto confia numa entrada que
# nunca foi calibrada (ver §1 da spec do ciclo 16). Consumida por
# `validation::incerteza` para varrer a banda e classificar cada check.
fom_static_tol_pct = 10.0
```

Campo **obrigatório**, sem `#[serde(default)]`. Um TOML de aeronave que não
declare quanto confia no próprio `fom_static` deve falhar no carregamento, não
herdar um default silencioso. Todo arquivo de config e toda fixture (TOML e
Rust) passam a declará-lo — a lista completa está no plano.

#### Banda declarada × banda efetiva

```
lo            = fom_static · (1 − tol/100)
hi_declarado  = fom_static · (1 + tol/100)
hi            = min(hi_declarado, fom_design, 1.0)
```

O topo é truncado por duas razões físicas:

- **`fom_design`**: acima dele `FoM(J)` seria **decrescente** em J — mais
  velocidade de avanço dando menos fração da tração ideal. O topo efetivo da
  banda é, portanto, exatamente *a maior tração estática que se pode alegar
  sem contradizer a única âncora que FOI calibrada*.
- **1,0**: teto de quantidade de movimento.

No baseline isso morde:

```
banda declarada  [0,675000 ; 0,825000]              (−10,0000% / +10,0000%)
banda EFETIVA    [0,675000 ; 0,81597699924588796]   (−10,0000% /  +8,7969%)
                                       ↑ truncada em fom_design
```

**A truncagem é publicada com a razão, nunca silenciosa** (§5.7). Uma banda
que encolhe sem avisar é uma banda que produz vereditos determinados por
motivo invisível.

### 5.2 Identidade de check

Hoje uma restrição avaliada **não é reificada**: cada um dos 25 sítios de
checagem faz `violations.push(format!("…"))` e o texto formatado é toda a
identidade que existe. O texto carrega valores (`"7.9%"` vs `"7.4%"`), então
não serve de chave entre duas corridas.

```rust
/// Uma violação com identidade estável. `id` NÃO contém números que variem
/// com a configuração — é a chave usada para parear o mesmo check entre
/// corridas com `fom_static` diferente (`validation::incerteza`).
#[derive(Debug, Clone, PartialEq)]
pub struct Violacao {
    pub id: String,
    pub texto: String,
}

pub struct ConstraintReport {
    pub violations: Vec<Violacao>,
    pub warnings: Vec<String>,
}

impl ConstraintReport {
    pub fn all_satisfied(&self) -> bool { self.violations.is_empty() }
    /// Só os textos, na ordem — o que vai para o JSON.
    pub fn textos(&self) -> Vec<String> { … }
}
```

Regras do `id`:

- **Nenhum número que dependa da config.** `"gradiente_cs2365"`, não
  `"gradiente_7.9"`.
- Checks parametrizados por cenário embutem o nome do cenário, que é texto de
  config e não varia com `fom_static`:
  `"envelope_cg::Solo (piloto)"`, `"robustez_flip::2 pax dianteiros"`.
- Os **9 portões** de `main.rs:641-663` também ganham id. Eles hoje não
  empurram string nenhuma, o que significa que um FAIL pode não ter nenhuma
  linha em `violations` — e um portão pode virar dentro da banda sem deixar
  rastro. Sem id eles ficariam fora da varredura.

Um teste dedicado exige que os ids sejam **únicos** e **estáveis sob variação
de `fom_static`** (roda em dois pontos da banda e compara os conjuntos de id
esperados).

### 5.3 O pipeline como função

`src/main.rs` tem 1.078 linhas com cálculo e impressão interpolados. Extrair
para `src/pipeline.rs`:

```rust
/// Executa o pipeline completo. NÃO faz varredura de banda — é a função que
/// a varredura chama, e chamá-la de dentro dela mesma seria recursão infinita.
pub fn executa(
    cfg: &AircraftConfig,
    engine: &EngineSpec,
    req: &Requirements,
) -> Result<Resultado, PipelineError>;

pub struct Resultado {
    pub sized: SizingOutput,
    /* … todas as saídas de agente que main.rs imprime e serializa … */
    pub constraints: ConstraintReport,
    pub portoes: Vec<Portao>,   // os 9 gates, com id
}

pub struct Portao { pub id: &'static str, pub ok: bool, pub rotulo: String }
```

`main.rs` fica: carregar → `executa` → imprimir → montar `AircraftReport` →
escrever JSON.

Dividendo: `tests/schema_v4.rs:103-150` hoje **reimplementa** o pipeline e usa
uma regra de veredito **diferente** da de `main.rs` — só `all_satisfied()`,
sem os outros 8 portões. São duas definições de "veredito global" que
coincidem por acaso. O teste que deveria vigiar o pipeline mantém uma cópia
divergente dele: é a doença do #13 em outra roupa. Passa a chamar
`pipeline::executa`.

**Invariante desta extração: `aircraft_spec.json` e o stdout do binário
byte-idênticos.** Provado por script no plano, não por inspeção.

### 5.4 A varredura e o breakeven

`src/validation/incerteza.rs`, irmão de `validation::robustness` (que já tem o
precedente de re-executar `size_aircraft` com config perturbada,
`robustness.rs:428`/`:502`).

```
1. banda efetiva ← §5.1
2. executa(cfg com fom_static = lo)          → conjunto de ids violados L
3. executa(cfg com fom_static = hi)          → conjunto de ids violados H
4. executa(cfg com FoM ≡ 1,0)                → conjunto de ids violados T
   (as DUAS âncoras a 1,0 — teto de quantidade de movimento)
5. para cada id em L ∪ N ∪ H ∪ T:
     DETERMINADO   se a pertinência é IDÊNTICA nos três pontos {L, N, H}
                   → FALHA se pertence aos três, PASSA se a nenhum
     INDETERMINADO em qualquer outro caso
     alcance_de_helice = false  se o id também está em T
6. para cada id INDETERMINADO com pertinência DIFERENTE entre L e H:
   bisseca fom_static em [lo, hi] até a largura do bracket < 1e-6 e
   publica o BRACKET (não um ponto)
7. para cada id INDETERMINADO cuja pertinência em L e H COINCIDE (só o
   nominal discorda): não há travessia única na banda — publica
   `breakeven_lo/hi = null` com `motivo = "não monotônico na banda"`
```

O critério do passo 5 é **pertinência idêntica nos três pontos**, não "vira
entre os extremos". A diferença importa: um check pode violar no nominal e não
violar em nenhum dos dois extremos (não monotonicidade). A regra ingênua
("virou entre L e H?") o classificaria como PASSA enquanto a corrida nominal o
tem na lista de violações — o modelo publicaria uma violação e, ao lado, a
afirmação de que ela não existe. Nenhuma não monotonicidade foi observada no
baseline, mas a regra não pode depender disso.

**O breakeven é publicado como intervalo medido, nunca como número pontual.**
Um breakeven com 17 dígitos e tolerância de 1e−6 seria exatamente a falsa
precisão que este ciclo existe para curar. O intervalo carrega a própria
incerteza no formato.

Custo: 3 corridas fixas + ~18 por check indeterminado × 57 ms ≈ **1,2 s** no
baseline (1 check indeterminado). O binário sai de 57 ms para ~1,2 s.

Se uma corrida de extremo **falhar em convergir** (`SizingError`), isso é
publicado como causa, não engolido — precedente exato em
`RobustnessSpec::mtow_masstotal_kg`, que documenta o `0.0` do sizing
perturbado que falhou. Medido: o pipeline converge em todo `fom_static` de
0,55 a 1,0, então isso não morde no baseline — mas é o comportamento exigido.

### 5.5 O terceiro estado e o veredito global

```rust
pub enum Veredito { Passa, Falha, Indeterminado }
```

Regra do veredito global:

```
FAIL           se existe QUALQUER check com falha DETERMINADA
INDETERMINADO  senão, se existe qualquer check indeterminado
PASS           senão
```

**Falha determinada domina indeterminação.** No baseline: três violações
determinadas ⇒ `validation_status` continua **FAIL**. Este ciclo **não muda o
veredito do projeto** — muda o que o modelo consegue dizer sobre ele.

`validation_status` passa a ter domínio `"PASS" | "FAIL" | "INDETERMINADO"`.
O terceiro valor fica em português junto de dois em inglês: incoerência
assumida, porque o termo é o contrato acordado com o usuário e os textos de
violação do projeto já são todos em português. Registrado como item de
backlog, não corrigido às escondidas.

### 5.6 O texto da violação

Um check indeterminado **continua na lista de violações**. Não sai, não vira
warning, não é rebaixado. Só o texto muda:

```
INDETERMINADO — Gradiente de subida 7.9% vs mínimo de 8.3% exigido pela
CS 23.65 (Vx=138.9km/h). O veredito VIRA dentro da banda declarada de
propeller.fom_static [0.675000–0.815977]: breakeven em [0.784867–0.784868],
+4.6% sobre o nominal 0.750. O modelo NÃO sustenta este veredito.
```

Três razões:

1. **Um estado novo que reduzisse o ruído seria uma máquina de lavar
   reprovação.** INDETERMINADO tem que ser mais alto que FALHA, nunca mais
   baixo: FALHA diz "seu avião não atende"; INDETERMINADO diz "seu modelo não
   sabe" — o que é pior, porque não dá nem para consertar o avião com base
   nele.
2. A contagem de violações fica em **4**, e `.contains("Gradiente de subida")`
   continua verdadeiro: os dois testes que afirmam `violations.len() == 4`
   (`tests/cli.rs:769`, `tests/gear_tipback.rs:670`) seguem válidos **por
   mérito**, não por terem sido afrouxados.
3. O texto é a interface pública de facto (todas as asserções de violação são
   `.contains(<substring>)`), então a mudança tem que estar no texto para ser
   vista por quem só lê o texto.

### 5.7 O bloco `uncertainty` no JSON

Novo bloco de topo em `AircraftReport`. **O JSON abaixo é ILUSTRATIVO da
forma** — os valores com `…` e os que dependem de aritmética f64
(`band_declared_hi = 0,75 · 1,10`) são os que o pipeline produzir, e o valor
autoritativo é sempre o do artefato regenerado, nunca este exemplo. (O ciclo
15 fechou o backlog #13 porque números citados em documentação envelheceram
sem que nada reclamasse; um exemplo de spec não pode virar a próxima
citação estimada.)

```json
"uncertainty": {
  "parameter": "propeller.fom_static",
  "nominal": 0.75,
  "declared_tol_pct": 10.0,
  "band_declared_lo": 0.675,
  "band_declared_hi": 0.825…,
  "band_lo": 0.675,
  "band_hi": 0.81597699924588796,
  "band_truncated": true,
  "band_truncated_reason": "topo truncado em propeller.fom_design (0.81597699924588796): acima dele FoM(J) seria DECRESCENTE em J",
  "ceiling_evaluated": true,
  "checks": [
    {
      "id": "gradiente_cs2365",
      "veredito": "INDETERMINADO",
      "veredito_lo": "FALHA",
      "veredito_nominal": "FALHA",
      "veredito_hi": "PASSA",
      "alcance_de_helice": true,
      "breakeven_lo": 0.7848672…,
      "breakeven_hi": 0.7848682…,
      "motivo": null
    },
    {
      "id": "decolagem_grama_15m",
      "veredito": "FALHA",
      "veredito_lo": "FALHA", "veredito_nominal": "FALHA", "veredito_hi": "FALHA",
      "alcance_de_helice": true,
      "breakeven_lo": null, "breakeven_hi": null
    },
    {
      "id": "envelope_cg::Solo (piloto)",
      "veredito": "FALHA",
      "veredito_lo": "FALHA", "veredito_nominal": "FALHA", "veredito_hi": "FALHA",
      "alcance_de_helice": false,
      "breakeven_lo": null, "breakeven_hi": null
    }
  ]
}
```

`alcance_de_helice: false` significa: **falha também no teto de quantidade de
movimento** — nenhuma hélice conserta.

Só entram em `checks` os ids que violam em pelo menos um dos quatro pontos
avaliados. Um check que passa em todos não gera linha — senão o bloco viraria
um despejo de 25 linhas idênticas por corrida.

### 5.8 Validações novas em `models::config`

1. **`fom_design >= fom_static`** — hoje **não existe**. Uma curva `FoM(J)`
   decrescente passa na validação atual. Confirmado por leitura de
   `src/models/config.rs:853-869`: só há `require_positive` nos três e teto
   1,0 em dois. Este ciclo fecha isso, e a mensagem de erro diz por quê.
2. **`fom_static_tol_pct`** presente, `> 0` e `< 100` (para que `lo > 0`).
3. A banda efetiva resultante tem `hi > lo`.

Item 1 é independente da banda e vale por si: é o "modelo deve FALHAR no ponto
de perigo" aplicado a uma curva de tração fisicamente impossível que hoje
passa calada.

---

## 6. Invariantes

**O diff de `aircraft_spec.json` contra `bfd4921` tem que ser EXATAMENTE:**

1. `schema_version` (e `revision`): `"5.7"` → `"5.8"`
2. o bloco novo `uncertainty`
3. o texto da violação da CS 23.65 (prefixo `INDETERMINADO — …`)

**Todo o resto byte-idêntico.** Nenhum número de física muda neste ciclo:
`fom_static` continua 0,75, a lei continua a mesma, os agentes continuam os
mesmos. Provado por script, não por inspeção — o plano exige o script.

Nas tasks 1 e 2 (extração e reificação) o invariante é mais duro ainda:
**`aircraft_spec.json` e o stdout do binário byte-idênticos**, sem exceção
nenhuma.

---

## 7. Schema e pins

**Bump 5.7 → 5.8 (MINOR).** Pela política de `docs/aircraft_spec.schema.md:1739-1747`,
aditivo é MINOR. Nenhum campo foi renomeado, removido ou teve tipo/unidade
mudados. O ponto discutível é o **domínio** de `validation_status`, que
alarga de dois para três valores — um consumidor que faça match exaustivo
sobre `PASS`/`FAIL` quebra. Julgamento declarado: MINOR, porque (a) nenhum
campo mudou de tipo, (b) o único consumidor na árvore é a suíte de testes, e
(c) o valor novo só aparece quando NÃO existe falha determinada, ou seja,
nunca no lugar de um `FAIL` que existiria antes. **O julgamento é declarado
para poder ser contestado** pela revisão de plano; se ela discordar, vira
MAJOR 6.0.

**Pins (porteiro do ciclo 15).** Este é o primeiro ciclo em que o
`tests/pins_vs_json.rs` enfrenta campos novos. Consequências:

- Todo número novo citado em teste ou no schema doc precisa de marcador.
- `MINIMO_DE_PINS_VINCULADOS` (48) e `MINIMO_DE_NUMEROS_NO_DOC` (12) sobem
  para os valores medidos ao fim do ciclo — nunca "com folga".
- `tests/generic_engine.rs:2538-2540` hoje pina `fom_static`/`fom_design`/
  `j_design` como `NAO-PUBLICADO`. Com a banda, **`fom_static` passa a ser
  publicado** (`uncertainty.nominal`) e o pin vira vinculado. Se o
  implementador esquecer, o porteiro reprova — é exatamente para isso que ele
  foi construído.

---

## 8. Testes exigidos

Além dos unitários de cada peça:

1. **O breakeven publicado é verificado re-rodando o modelo.** Roda o
   pipeline em `breakeven_lo` e em `breakeven_hi` e exige vereditos opostos
   para aquele id. Um breakeven que não se prova assim é um pin estimado —
   a terceira variante da doença do #13, encontrada no ciclo 15.
2. **INDETERMINADO nunca remove violação.** A contagem em `violations` com a
   banda ligada é igual à contagem com a banda colapsada (`tol_pct` → 0).
3. **Falha determinada domina.** Config sintética com um check indeterminado
   e um determinado ⇒ `validation_status == "FAIL"`.
4. **Um check que não vira sai determinado**, com o veredito certo.
5. **A truncagem da banda é reportada**: `band_truncated == true` e a razão
   cita `fom_design`.
6. **Ids únicos e estáveis** sob variação de `fom_static`.
7. **`fom_design < fom_static` é rejeitado** no carregamento, com mensagem
   que explica a monotonicidade.
8. **`fom_static_tol_pct` ausente falha** o carregamento.
9. **Extração:** JSON e stdout byte-idênticos (tasks 1 e 2).
10. **`tests/schema_v4.rs` usa `pipeline::executa`** — some a segunda regra de
    veredito.

---

## 9. Lacunas declaradas

Escritas aqui porque o ciclo 15 estabeleceu que **uma ferramenta contra
auto-engano que não declara o quanto ela própria pode ser enganada é a versão
mais perigosa do problema que resolve.**

1. **A banda é declarada, não medida.** ±10% é política de projeto sobre uma
   entrada não calibrada. Não medi hélice nenhuma neste ciclo. O que fecharia
   isso é análise de elemento de pá / JavaProp em J=0 — vai para o backlog.
   Mitigação parcial: o **breakeven é fato medido e é publicado sempre**, então
   mesmo com a banda errada o número publicado continua certo; a banda só
   escolhe a palavra.
2. **Só `fom_static` é varrido.** A maquinaria é geral, a aplicação é de um
   parâmetro só. O bloco `uncertainty` nomeia seu `parameter` justamente para
   não ser lido como "todo o resto é certo". **Não é.** O baseline tem outras
   entradas declaradamente não validadas — `ground_clearance_min_m` ("PROXY de
   projeto conservador"), `prop_plane_x_m` ("ESTIMATIVA de geometria — validar
   no CAD"), entre outras — e nenhuma delas é varrida. Backlog.
3. **Um parâmetro por vez.** Interações entre incertezas não são exploradas.
   Duas entradas cada uma dentro da sua banda podem virar um check que nenhuma
   vira sozinha.
4. **`fom_design` é calibrado contra um polinômio apagado.** É a âncora "boa"
   deste ciclo e o topo da banda depende dela. A qualidade da própria
   calibração é outra pergunta, não respondida aqui.
5. **O teto de quantidade de movimento é teto, não meta.** `alcance_de_helice:
   true` NÃO diz que existe hélice real capaz — diz apenas que a física não
   proíbe. A decolagem na grama é o exemplo vivo: dentro do alcance do teto,
   fora do alcance de `fom_static` sozinho em todo (0 ; 1].

---

## 10. Reprodutibilidade das medições

Scripts usados (scratchpad da sessão), todos rodando o binário de release
contra cópias do baseline com uma única linha alterada:

| script | o que mede |
|---|---|
| `bisseca16.py` | breakeven da CS 23.65 por bisseção, 40 iterações |
| `sensi16.py` | inventário de sensibilidade: diff de todas as folhas do JSON entre extremos, e conjunto de violações em 8 pontos |
| `banda16.py` | banda efetiva, valores nos extremos, teto, e os dois breakevens |

O plano deve portar o essencial disto para testes versionados — um número
medido em scratchpad que ninguém consegue re-rodar é um número estimado com
passos extras.

---

## 11. Backlog a abrir

- Calibrar `fom_static` por elemento de pá / JavaProp em J=0, substituindo a
  banda declarada por banda medida.
- Varrer as demais entradas declaradamente não validadas do baseline.
- Incoerência de idioma no domínio de `validation_status`.
- Interações entre incertezas (banda multidimensional).
- O achado do ciclo: **mais tração estática piora o limite dianteiro de CG e
  a robustez do cenário '2 pax dianteiros'** — as duas violações que hélice
  nenhuma conserta. Isso é um achado de PROJETO, não de modelo, e merece item
  próprio.
