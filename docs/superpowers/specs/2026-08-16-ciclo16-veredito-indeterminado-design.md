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

- **Um id por sítio `violations.push`**, não por comentário numerado. São
  **25 sítios** (26 ocorrências de `violations.push` em
  `constraint_checker.rs`, uma delas dentro de um comentário na linha 455).
- **Nenhum número que dependa da config.** `"gradiente_cs2365"`, não
  `"gradiente_7.9"`.
- Checks parametrizados por cenário embutem o nome do cenário, que é texto de
  config e não varia com `fom_static`:
  `"envelope_cg::Solo (piloto)"`, `"robustez_flip::2 pax dianteiros"`.
- Os **9 portões** de `main.rs:641-663` também ganham id, **todos com prefixo
  `portao_`**. Eles hoje não empurram string nenhuma, o que significa que um
  FAIL pode não ter nenhuma linha em `violations` — e um portão pode virar
  dentro da banda sem deixar rastro. Sem id ficariam fora da varredura.

> **ERRATUM (revisão de plano).** A primeira versão desta seção dizia que o id
> "sai do comentário numerado que já existe acima de cada check". A revisão
> mostrou que a regra **colide**: vários comentários numerados cobrem mais de
> um `push` independente — `#10` (`:287`, `:294`, `:301`: Mach estático, Mach
> de cruzeiro, folga de solo), `#17` (`:407`, `:414`: teto e piso de carga de
> nariz, dois `if` separados que podem disparar JUNTOS), e `#9a` (`:260`,
> envelope vazio), cujo próprio comentário já avisa ser "distinta das
> violações por cenário abaixo". Pior: o comentário do check `#1` diz
> "Velocidade de cruzeiro" mas o código testa `ld_ratio_cruise < 10.0`, e
> colidiria com o portão homônimo.
>
> Duas colisões silenciosas fariam a varredura publicar o veredito de um check
> sobre o outro — e o fixture do baseline nunca dispara essas condições
> juntas, então a suíte passaria verde.

O conserto **não** é eu publicar a lista dos ids ambíguos: seria consertar o
caso conhecido, deixar o desconhecido de pé, e me pôr outra vez publicando uma
lista derivada à mão (#29). O conserto é estrutural, em três camadas:

1. **Prefixo `portao_`** nos 9 portões: as duas famílias de id passam a não
   poder colidir *por construção*.
2. **Teste que varre o FONTE**, na técnica que o ciclo 15 já estabeleceu em
   `tests/pins_vs_json.rs`: extrai todo literal `id:` de
   `constraint_checker.rs` e de `pipeline.rs` e exige (a) que a contagem de
   literais bata com a contagem de sítios `violations.push`, e (b) unicidade
   global entre as duas famílias. Um mecânico que dê o mesmo id aos dois
   `push` do `#17` vê vermelho imediatamente; um que acrescente um `push` sem
   id, também.
3. **Teste de tempo de execução** sobre a união `Violacao::id ∪ Portao::id`, e
   um **teste adversarial** com config sintética que force as DUAS condições
   do `#17` a disparar simultaneamente — a combinação que o baseline nunca
   produz.

Mais o teste de **estabilidade sob variação de `fom_static`** (roda em dois
pontos da banda e compara os conjuntos de id).

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

> **ERRATUM (revisão da Task 4) — CONTRADIÇÃO INTERNA DESTA SPEC.** O
> algoritmo abaixo dizia "conjunto de ids **violados**", e a implementação
> leu isso, corretamente, como `report.violations`. Mas a §5.2 diz que os 9
> portões ganham id **exatamente para entrar na varredura** ("sem id ficariam
> fora da varredura"). As duas seções se contradizem, e a Task 4 seguiu esta.
>
> Resultado medido pela revisão: a varredura ignorava os 9 portões, e
> **`portao_v_cruzeiro`, `portao_flutter`, `portao_antitombamento` e
> `portao_estabilidade_long` não têm nenhuma `Violacao` correspondente** — são
> quatro gates de aeronavegabilidade inteiramente invisíveis ao mecanismo que
> este ciclo existe para construir. No baseline de hoje nenhum deles vira
> dentro da banda, então os números não mudam; isso é propriedade do baseline,
> não do desenho.
>
> A forma como foi achado merece registro: a revisão tentou aplicar a mutação
> "faça a varredura ignorar os portões" e **não conseguiu, porque já era o
> comportamento**. Uma mutação inaplicável porque o defeito já ocupa o lugar
> dela é o achado mais limpo possível.
>
> **A definição correta do conjunto de um ponto é a UNIÃO:**
>
> ```
> ids(ponto) = { v.id  para v em report.violations }
>            ∪ { p.id  para p em portoes, se !p.ok }
> ```
>
> **Menos os portões que são função determinística do conjunto de violações** —
> esses não são checks independentes, são agregados, e publicar um agregado ao
> lado dos seus próprios componentes é a doença do #21 na direção contrária.
> Hoje **exatamente um** portão satisfaz isso: `portao_restricoes`, que é
> literalmente `violations.is_empty()`. A exclusão é por REGRA provável, não
> por lista escolhida a dedo.
>
> Alguns portões duplicam uma `Violacao` em SIGNIFICADO (`portao_rc_sl` e o id
> `rc_sl`, `portao_teto_servico` e `teto_servico`, `portao_envelope_cg_todos` e
> os `envelope_cg::*`). Eles **permanecem**, e a duplicação fica visível na
> saída. Removê-los exigiria uma lista de equivalências mantida à mão — e o
> histórico deste projeto (backlog #29, sete ocorrências) diz que uma lista
> dessas envelhece errado e em silêncio. Duplicação visível é melhor que
> supressão frágil.

```
1. banda efetiva ← §5.1
2. executa(cfg com fom_static = lo)          → ids(lo) = L
3. executa(cfg com fom_static = hi)          → ids(hi) = H
4. executa(cfg com FoM ≡ 1,0)                → ids(teto) = T
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

> **ERRATUM (revisão da Task 4) — o caso que esta seção não fechava.** O texto
> abaixo pressupõe que o check indeterminado JÁ ESTÁ na lista de violações, e
> só precisa ter o texto reescrito. Isso vale quando ele viola no nominal. Mas
> a regra da §5.4 admite `(lo=não, nominal=não, hi=sim)` — um check
> INDETERMINADO **ausente** da lista nominal. Não ocorre no baseline; a regra
> não pode depender disso (é o mesmo argumento da não monotonicidade).
>
> Regra completa, então:
> - indeterminado **presente** no nominal → reescreve o texto, prefixo
>   `INDETERMINADO — `; a contagem de violações não muda;
> - indeterminado **ausente** do nominal → **INSERE** uma violação nova com o
>   mesmo prefixo, dizendo que o check passa no nominal mas vira dentro da
>   banda; a contagem SOBE.
>
> A segunda metade é a que importa para a honestidade do artefato: um check que
> passa hoje e reprova dentro da banda declarada é precisamente o que o usuário
> precisa ver, e seria o único caso em que o silêncio favoreceria o projeto.
> Um portão de aeronavegabilidade que "passa" por 0,4% e vira com uma hipótese
> não calibrada não é um portão que passa.

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

1. `schema_version` (e `revision`): `"5.7"` → `"6.0"`
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

**Bump 5.7 → 6.0 (MAJOR).**

> **ERRATUM (revisão de plano).** A primeira versão desta seção propunha
> MINOR 5.8, argumentando que (a) nenhum campo mudou de tipo, (b) o único
> consumidor na árvore é a suíte de testes, e (c) o valor novo só aparece
> quando não existe falha determinada, nunca no lugar de um `FAIL` que
> existiria antes. A revisão contestou e está certa. O argumento (c) mede o
> risco errado: o perigo não é substituir um FAIL antigo, é que **um
> consumidor que teste `status == "FAIL"` trata `INDETERMINADO` como
> seguro** — e "o modelo não sabe" sendo lido como "está tudo bem" é
> exatamente a auto-decepção que este projeto existe para impedir. O
> argumento (b) também cai: `tests/schema_v4.rs` descreve o JSON como
> "contrato mínimo com o time de CAD", ou seja há consumidor fora da
> árvore. Alargar o domínio de um campo tipo-enum é quebra de
> compatibilidade, e a política do projeto
> (`docs/aircraft_spec.schema.md:1743`) manda MAJOR para quebra.

O bump é MAJOR porque o domínio de `validation_status` alarga de
`"PASS" | "FAIL"` para `"PASS" | "FAIL" | "INDETERMINADO"`. Um número de
versão não impede ninguém de escrever `== "FAIL"`; o que ele faz é obrigar
quem atualiza a olhar. É para isso que MAJOR serve.

**Declaração explícita sobre o backlog #11.** O item #11 tem remoções
enfileiradas para "um bump MAJOR futuro"
(`performance.to_distance_paved_m`/`to_distance_grass_m`/`landing_distance_m`).
**A 6.0 deste ciclo NÃO as carrega.** A remoção exige analisar quem consome
esses campos, o que é assunto do #11 e não deste ciclo; gastar o MAJOR aqui
não obriga a gastá-lo inteiro, e MAJOR é barato num projeto com um consumidor
em árvore. Fica registrado para que ninguém leia "6.0" e presuma que o #11
foi junto — o #11 continua aberto e agora precisa do próximo MAJOR.

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

## 11. Registro de execução

Escrito à medida que as tasks fecham. Existe porque a spec é o lugar onde este
ciclo guarda o que aprendeu, e um achado de processo perdido entre a task e o
merge é um achado que não aconteceu.

### Task 1 — extração do pipeline

**Primeira tentativa, operacional MECÂNICO: falhou.** Dois defeitos:

1. A extração ficou pela metade — `main.rs` passou a chamar `pipeline::executa`
   **e continuou calculando tudo inline**. Cada agente rodava duas vezes.
2. Os dois MTOWs do projeto foram colapsados: `design_mtow_kg` (missão, de
   `state.mtow_kg`) e `envelope_mtow_kg` (envelope, de `wb.spec.mtow_kg`)
   passaram ambos a ler `wb.spec.mtow_kg`. MTOW convergido 1538,3 → 1557,5 kg.
   E o comentário de **20 linhas** logo acima, que explicava que colapsá-los era
   o *bug B5* corrigido num ciclo anterior, foi **apagado no mesmo edit que
   cometeu o bug que ele descrevia**.

O invariante byte-idêntico pegou na primeira execução. O operacional
**reportou a divergência em vez de mascará-la** — registro a favor: ajustar a
base para casar com o obtido estava a um comando de distância, e teria feito o
ciclo seguir com uma mudança de física dentro de um refactor que promete zero
mudança.

**ERRO DE ROTEAMENTO — meu, e é item de processo novo.** A regra da casa diz
"operacional mecânico só recebe task cuja correção o `verifica-ciclo.sh`
consegue PROVAR". Eu apliquei a regra e classifiquei a Task 1 como mecânica
porque a **prova** era mecânica: script, diff, byte a byte.

Mas prova mecânica não torna o **trabalho** mecânico. Esta task exigia entender
uma distinção de domínio que o repositório documenta em vinte linhas. **A
provabilidade por script é condição NECESSÁRIA, não suficiente.** A condição
que faltava enunciar: o trabalho também não pode depender de uma distinção
semântica que o código carrega só em prosa.

É irmão do #29. Lá eu derivava as consequências de uma regra sem executá-la;
aqui apliquei uma regra de roteamento sem checar se a premissa dela cobria o
caso.

**Segunda tentativa, operacional de JULGAMENTO: fechou.** Invariante provado,
554 testes, gate APROVADO, comentário de 20 linhas restaurado byte a byte.

**Achado do script de prova.** O operacional precisou normalizar uma linha do
stdout — a última, que ecoa o argumento `--out`, e que diverge por construção
porque a base foi escrita com um nome de arquivo e a corrida nova com outro. A
alegação era legítima (verificada: o diff bruto tem exatamente uma linha
divergente, só no caminho, com a mesma versão de schema e o mesmo sufixo), e o
defeito era do script, meu.

Mas normalização **legítima** e normalização **segura** são coisas diferentes:
um `sed` aberto dentro de um script de prova é uma superfície que alguém alarga
depois. O guarda foi reescrito para se auto-limitar — compara bruto primeiro e,
se divergir, exige um único par de linhas, exige que seja a linha `[ SAÍDA ]`,
e exige que tirando o caminho as duas sejam idênticas — e foi **provado por
mutação** (`mutaprova16.sh`): passa no legítimo e reprova em (a) duas linhas
divergentes, (b) uma linha divergente que não é a `[ SAÍDA ]` — o caso que é
literalmente a divergência de MTOW da primeira tentativa — e (c) uma linha
`[ SAÍDA ]` que difira além do caminho, como uma versão de schema trocada.

**Alegação de proveniência falsa na mensagem de commit.** O corpo de `c9ed8f8`
diz que `tests/pipeline_extracao.rs` "já estava presente, conferido, passa sem
alteração". O arquivo foi **criado** por essa task — `git log --all` mostra um
único commit para ele, o próprio `c9ed8f8`. O conteúdo está correto e bate com
o Passo 6; o defeito é só a alegação.

Não emendei a mensagem. Emendar apagaria o fato de que a alegação foi feita, e
num ciclo cujo tema é não lavar veredito, lavar o registro seria incoerente —
mesmo sendo pequeno. Fica aqui, e vai para a mensagem de merge.

**Mutação (revisão).** Trocar `design_mtow_kg` por `envelope_mtow_kg` na
chamada do `PerformanceAgent` — o bug exato da primeira tentativa — é pego por
**dois** testes independentes (`tests/cli.rs` e `tests/pipeline_extracao.rs`).
Remover `propeller.fill_critical_clearance` é pego por quatro. Reordenar
agentes independentes não é pego **e não deveria ser**: não tem efeito
observável, e as dependências reais são impostas estaticamente pelo borrow
checker, garantia mais forte que teste. Renomear o id de um portão não é pego
por nada — **esperado**, porque o id ainda não é serializado nem verificado; é
trabalho da Task 2, e fica registrado que a garantia não existe até lá.

**Consequência para o roteamento da Task 2.** Reclassificada de mecânica para
**julgamento**. A atribuição dos ids exige decidir o que distingue as três
condições do `#10` e as duas do `#17` — julgamento de nomeação, não
transcrição. O teste de varredura de fonte prova a unicidade, mas não escolhe
os nomes.

### Task 2 — identidade de check

Fechada pelo operacional de **julgamento** (reclassificada, ver acima). 558
testes (554 + 4), invariante provado, gate APROVADO, `aircraft_spec.json`
byte-idêntico. 25 ids atribuídos, um por sítio de `push`.

**SÉTIMA ocorrência do #29, e outra vez contra mim.** O plano declarava, como
lacuna da verificação estática, que `envelope_cg::{}` era **o único** id não
literal. **São dois.** O check `#19` (laço sobre `robustness.flips`) é
estruturalmente idêntico ao `#9` — um sítio de `push`, 0..N violações por
corrida — e um id fixo colidiria sempre que dois flips ocorressem juntos, o que
tem registro histórico no próprio arquivo. Eu examinei o `#9` e escrevi "o
único" sem examinar o `#19`.

O implementador pegou, olhando o código que eu não olhei, e resolveu com
`format!("robustez::{}::{}", flip.check, flip.caso)` — mesma lógica.

O padrão do #29 é teimoso porque a afirmação de escopo ("é o único") parece
parte da regra e não é: a regra estava certa, o **censo** é que foi feito por
leitura parcial. Vale registrar que as duas ocorrências deste ciclo (§2.2 e
esta) foram pegas por quem executou algo, não por quem leu.

**Consequência que estava em aberto — FECHADA pela revisão da Task 2.** A
unicidade de `(flip.check, flip.caso)` dentro de uma corrida não havia sido
verificada por ninguém. A revisão verificou, lendo
`src/validation/robustness.rs`, e o par **é** único:

- `caso` assume 3 valores por corrida, cada um de exatamente uma chamada —
  `evaluate_case("dianteiro", …)` (`:407`), `evaluate_case("traseiro", …)`
  (`:408`), `evaluate_world(…, "massa-total", …)` (`:580`) — nunca em laço.
- Dentro de cada chamada, os checks de nome fixo (`"Tipback"`, `"Carga de nariz
  máx"`, `"Carga de nariz mín"`) estão cada um atrás de um único `if`, não de
  laço: no máximo um push por chamada.
- Os checks por cenário usam `format!("Cenário '{}'", …)` sobre os 5 nomes
  fixos e distintos de `weight_balance.rs:553-558`.
- E o prefixo `"Cenário '"` funciona como **namespace textual**: nenhum outro
  `check` literal do arquivo começa com ele, então colisão entre um flip de
  cenário e um de nome fixo é impossível *por construção de string*,
  independentemente da unicidade dos nomes de cenário.

Vale mais que o resultado: a lacuna foi fechada por **leitura dirigida do
código que a produzia**, não por argumento de plausibilidade. Era exatamente o
que faltava quando eu escrevi "o único".

**Desvio de localização declarado.** O teste de varredura de fonte não coube em
`constraint_checker.rs::mod tests` — precisa de `mascara_arquivo`, que vive no
crate de testes de integração e não é visível de um teste unitário de lib. A
máscara foi movida para `tests/common/mod.rs` (convenção do Cargo) e
`tests/pins_vs_json.rs` passou a importá-la. Reuso real, não duplicação: os
**três autotestes da máscara** (aspa escapada, máscara presa reprovando alto,
conteúdo de string e comentário) continuam em `pins_vs_json.rs`, exercitando
agora a função importada, e os 34 testes daquele arquivo seguem verdes — mesma
contagem do ciclo 15.

Risco novo a declarar: se alguém remover `mod common;` de `pins_vs_json.rs`, os
autotestes da máscara vão junto e `identidade_de_checks.rs` passa a usar uma
máscara não testada. É o item #28 numa roupa nova — dois consumidores, um só
com testes.

**Revisão da Task 2: APROVADA, sem bloqueadores e sem achados.** Verificou o
invariante do zero; confirmou que a máscara mudou apenas de `fn` para `pub fn`,
corpo byte-idêntico; confirmou os pisos 48/12 intocados e nenhuma asserção
afrouxada nas ~60 conversões de leitor.

Mutações que a revisão rodou, todas revertidas:

| mutação | reprovou? |
|---|---|
| push com `id` não literal (escapa o scanner) | **sim** — "26 sítios, 25 ids" |
| dois pushes com o mesmo `id` | **sim** — "id duplicado entre violações e portões" |
| remover um marcador `// PIN:` de um teste | **sim** — dois testes do porteiro do ciclo 15 |

A primeira linha é a que importa: o scanner de unicidade **reprova quando
enganado**, e não só quando obedecido.

### Task 3 — a banda em config

Executada pelo operacional **mecânico**, e desta vez o roteamento acertou: o
plano entregava o código literal e o conteúdo de julgamento era baixo. 562
testes, invariante provado, gate APROVADO, banda do baseline saindo
`[0,675 ; 0,81597699924588796]`, truncada em `fom_design` com a razão escrita.

Entrou junto a validação que faltava desde o ciclo 13: **`fom_design ≥
fom_static`**. Até aqui, uma curva de tração DECRESCENTE em J — uma hélice
entregando fração menor da tração ideal quanto mais rápido voasse — passava
calada pelo carregador.

**REPROVADA na revisão. O bloqueador é meu, e é o #29 numa terceira forma.**

O plano escolheu deliberadamente `fom_static_tol_pct = 10.0` para a fixture,
com a justificativa escrita de que "assim a fixture exercita o caminho não
truncado e o baseline exercita o truncado". Mas **nenhum teste chama `banda()`
sobre a fixture** — existe uma única chamada a `.banda()` no repositório
inteiro, e é sobre o baseline.

A revisão provou por mutação: forçar `truncada = true` com
`motivo_truncagem = Some("BOGUS")` **mesmo com `motivos` vazio** passa nos 562
testes. Nada impedia a banda de afirmar uma truncagem que não houve.

As duas formas anteriores do #29 neste ciclo foram censos derivados de leitura
parcial (§2.2, e o "único id não literal"). Esta é diferente e pior: publiquei
uma **intenção de cobertura como se fosse cobertura**. Escolher a fixture certa
não testa nada — testar testa. E o buraco caiu exatamente nos dois campos que a
Task 5 vai publicar para explicar ao usuário por que a banda encolheu.

A spec §5.1 promete que a truncagem é publicada com a razão e nunca em
silêncio. Uma banda que **afirma** ter encolhido sem ter encolhido é o mesmo
defeito espelhado, e estava desprotegida.

**Segundo achado, também meu.** O comentário que prova a inatingibilidade do
ramo `hi > 1.0` cita **duas** validações; a prova precisa de **uma**. Se
`fom_design ≤ 1,0`, então ou o primeiro `if` truncou e `hi = fom_design ≤ 1,0`,
ou não truncou — o que significa `fom_design ≥ hi_declarado`, logo
`hi = hi_declarado ≤ fom_design ≤ 1,0`. A guarda de monotonicidade não entra em
passo nenhum. O perigo de deixar assim: quem um dia mexer na monotonicidade
concluirá, lendo o comentário, que a prova caiu — quando ela nunca dependeu
dela.

**Registrado sem conserto** (achado 3 da revisão): cada uma das duas validações
novas é pega por **exatamente um** teste, sem rede secundária. Se o teste
dedicado for enfraquecido num ciclo futuro, nada mais denuncia.

**Confirmado pela revisão, contra o meu briefing:** o inventário de fixtures
estava completo (ela refez a busca em vez de confiar em mim), `NaN`/`inf`/`-inf`
na tolerância são rejeitados só pela semântica IEEE754 de `!(v > 0 && v < 100)`,
e `hi_declarado` do baseline é de fato `0.8250000000000001` — sem nenhum
literal `0.825` escondido em teste ou comentário.

**Conserto APROVADO na segunda passada** (563 testes), com uma discrepância
registrada. O operacional relatou que a mutação derrubava **2** testes; a
revisão aplicou a mesma mutação duas vezes, com `--no-fail-fast` para garantir
que nenhum binário fosse pulado, e obteve **1** — só
`banda_da_fixture_nao_e_truncada`. E confirmou por busca que existem exatamente
duas chamadas a `.banda()` no repositório, ambas nos testes dedicados: não há
candidato a segundo teste.

A explicação provável é `cargo test` sem `--no-fail-fast` abortando após o
primeiro binário falhar, e o corte sendo lido como falha adicional. O achado
original permanece correto e o requisito do bloqueador foi cumprido — o teste
novo mata a mutação sozinho.

Fica registrado mesmo sendo imaterial, porque é a mesma classe dos erros deste
ciclo: **um número relatado que quem relatou não conferiu**. A diferença é que
desta vez o número foi conferido por outro antes de virar registro.

### Task 4 — a varredura

572 testes (563 + 9), invariante provado, gate APROVADO. **Todos os números
medidos bateram com os da spec §2** — banda, breakeven, vereditos por ponto,
alcance de hélice. A varredura leva **330–420 ms em release**, abaixo da
estimativa de ~1,2 s da §5.4 (que era estimativa de custo, não medição a
reproduzir).

Saída da varredura no baseline:

| id | veredito | lo | nom | hi | alcance de hélice | breakeven |
|---|---|---|---|---|---|---|
| `gradiente_cs2365` | **INDETERMINADO** | Falha | Falha | Passa | sim | `[0,784867237236 ; 0,784867775020]` |
| `decolagem_grama` | Falha | Falha | Falha | Falha | sim | — |
| `envelope_cg::Solo (piloto)` | Falha | Falha | Falha | Falha | **não** | — |
| `robustez::Cenário '2 pax dianteiros'::dianteiro` | Falha | Falha | Falha | Falha | **não** | — |

O bracket contém `0,784867742387`, o valor medido independentemente na §2.3.

**ERRO DE ORQUESTRAÇÃO — meu, e vira regra.** Eu despachei a re-revisão da
Task 3 e a Task 4 **em paralelo**, raciocinando que "a revisão é somente
leitura e a Task 4 cria arquivo novo, então não colidem".

Errado. **Uma revisão que testa por mutação NÃO é somente leitura** — mutação
escreve na árvore. Foi eu mesmo quem instruiu o revisor a mutar e reverter.

O que aconteceu: a Task 4 começou e encontrou uma mutação órfã
(`MUTACAO_REVISOR_BOGUS`) viva em `src/models/aircraft_config.rs`, sobra da
revisão em andamento. Ela percebeu, reverteu com `git checkout --` e reportou —
que é o comportamento certo.

Dois danos possíveis, nenhum realizado por sorte:
1. A Task 4 poderia ter medido a varredura inteira contra uma `banda()` mutada.
   Naquela mutação específica só `truncada`/`motivo` mudavam, e `lo`/`hi` não —
   os resultados teriam saído iguais. **Uma mutação que mexesse em `hi` teria
   contaminado toda a medição do ciclo.**
2. O `git checkout --` da Task 4 poderia ter revertido a mutação do revisor **no
   meio da medição dele**, fazendo-o observar zero falhas e concluir que o teste
   novo não pegava nada.

Por isso a medição-chave da Task 3 foi **refeita pelo chefe**, sozinha na
árvore, depois do incidente (`remuta16.sh`): com a mutação aplicada, exatamente
**um** teste reprova — `banda_da_fixture_nao_e_truncada` — confirmando o
revisor e não o operacional. E a suíte foi reexecutada, também pelo chefe:
**572 testes, 0 falhas**, com `git diff d9e8b3b 6339fb9 -- src/models/` vazio,
provando que a Task 4 não alterou os arquivos da Task 3.

**Regra nova:** revisão por mutação e task de implementação **nunca** correm em
paralelo na mesma árvore. Em geral: antes de paralelizar dois agentes,
perguntar não "eles editam os mesmos arquivos?" mas **"algum dos dois escreve
na árvore em algum momento, ainda que temporariamente?"**. Uma medição feita
durante concorrência não é medição — e uma medição de segunda mão feita durante
concorrência é pior, porque parece uma.

**Decisões do implementador que o plano não cobria**, todas declaradas:
1. Falha de convergência num extremo → check vira INDETERMINADO com `motivo`
   citando o extremo e a mensagem do `PipelineError`, sem tentar bissecar (não
   há garantia de que pontos internos convirjam). `avalia()` foi separada de
   `analisa()` para tornar a decisão pura testável com `viola` sintética.
2. Teto falhando → `alcance_de_helice` marcado **conservadoramente** `false`
   com motivo: nunca afirma alcance sem ter medido.
3. Falha num ponto INTERNO durante a bisseção → `.expect(...)` com mensagem
   pedindo que seja reportado como achado de primeira ordem. Não observado (o
   pipeline converge de 0,55 a 1,0) e fora do domínio medido.

#### Task 4 — consertos da revisão

583 testes (572 + 11), invariante provado, gate APROVADO, artefato intocado.
Cada teste novo **visto reprovando** sob a mutação correspondente, com a saída
dos dois estados no relatório.

Com os portões incluídos, a varredura do baseline ganhou **uma** entrada:
`portao_envelope_cg_todos`, FALHA determinada, sem alcance de hélice —
duplicando `envelope_cg::Solo (piloto)` em significado, visível por regra e não
suprimida. Nenhum número de física mudou.

**O achado do Conserto 3, que eu pedi para ser reportado e que mudou o
desenho do teste.** No baseline de hoje, os conjuntos de ids violados em
`banda.hi` (0,815976999245888) e em `banda.hi_declarado` (0,8250000000000001)
**coincidem** — mesmos três ids, portões incluídos. Só as magnitudes nos textos
diferem ("778 m" vs "768 m").

Ou seja: **um teste baseado na saída não conseguiria distinguir os dois.** Usar
a banda declarada em vez da efetiva — rodar até 0,825, fora do domínio onde
`FoM(J)` é não decrescente — produziria exatamente o mesmo veredito por check.

O conserto foi mudar o que se amarra: `Incerteza` ganhou `fom_lo_usado` e
`fom_hi_usado`, e o teste afirma `fom_hi_usado == banda.hi`, nunca
`hi_declarado`. **Quando duas entradas candidatas produzem saída
indistinguível, o teste tem que pinar a ENTRADA, não a saída.**

É a terceira aparição do padrão do ciclo — a primeira foi `range_km` variando
0,037% enquanto `fuel_climb_kg` variava 46% (o #21 original); a segunda foram
os dois bugs do scanner do ciclo 15, que erravam para lados opostos e mantinham
a contagem em 70. Aqui é a forma mais pura: **a saída observável é idêntica e a
entrada é fisicamente inadmissível.** Um agregado que não se move não é
evidência de que nada se moveu — e, no limite, nem a saída inteira é.

---

## 12. Backlog a abrir

- Calibrar `fom_static` por elemento de pá / JavaProp em J=0, substituindo a
  banda declarada por banda medida.
- Varrer as demais entradas declaradamente não validadas do baseline.
- Incoerência de idioma no domínio de `validation_status`.
- Interações entre incertezas (banda multidimensional).
- O achado do ciclo: **mais tração estática piora o limite dianteiro de CG e
  a robustez do cenário '2 pax dianteiros'** — as duas violações que hélice
  nenhuma conserta. Isso é um achado de PROJETO, não de modelo, e merece item
  próprio.
