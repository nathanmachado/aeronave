# Ciclo 13 — Tração unificada: uma lei só, ancorada em duas medidas

**Data:** 2026-08-15
**Backlog fechado:** #8 (unificar modelo de tração), #9 (`prop_efficiency` com
η(0)=0,58 e janela nula), #15 (PRIORIDADE ALTA — inconsistência de tração no
balanço de rotação), #16 (assimetria de superfície da rotação).
**Schema:** 5.5 → 5.6 (MINOR com exceção registrada, §9).
**Base:** `ed537ae` (merge do ciclo 12).

---

## §0 — Por que este ciclo existe

O ciclo 12 deixou o projeto com **dois modelos de tração descrevendo a mesma
grandeza física na mesma velocidade e devolvendo números 27,69% distintos**:

- `agents::performance::thrust_available_n` — disco atuador estático abaixo de
  V=0,5 m/s, e acima disso `η(J)·P_eixo/V` com `agents::propulsion::
  prop_efficiency` (polinômio JavaProp). Consumido por cruzeiro, subida, teto,
  velocidade máxima e pelo **balanço de rotação** (`thrust_at_rotation_n`).
- `agents::performance::thrust_ground_roll_n` — quantidade de movimento
  (Rankine-Froude) COM velocidade de avanço, × `static_thrust_factor` plano.
  Consumido só pela rolagem de decolagem.

A consequência mais grave está registrada no backlog #15: a identidade de
d'Alembert do balanço de rotação cancela a porção `h_cg` do braço da tração
contra o termo inercial **apenas porque o mesmo símbolo `T` aparece nos dois
lados de `m·aₓ = T − D − μN`**. O ciclo 12 quebrou essa premissa ao deixar `T`
do termo de momento vindo de um modelo e `D`/`μN` do outro. Resíduo não
cancelado no cenário governante: **−1.005,97 N·m = −6,816 pp de MAC**, contra
uma margem publicada de 0,000513 pp. `rotation_limit_pct_mac` ficou
**INDETERMINADO entre ≈16,28% e ≈24,57% de MAC**.

Este ciclo não escolhe entre os dois modelos. **Ele demonstra que um deles é
fisicamente impossível** (§1) e substitui os dois por uma lei única (§2).

---

## §1 — O achado que fecha a decisão: o teto de quantidade de movimento

Para qualquer hélice, real ou ideal, a tração produzida com uma dada potência
de eixo é limitada por conservação de quantidade de movimento. A teoria de
disco atuador (Rankine-Froude) dá o **máximo teórico**:

    T_ideal(V) = 2·ρ·A·u·(u − V),   com u a raiz real de u²·(u − V) = P_eixo/(2ρA)

Qualquer hélice real produz **menos** — perdas de perfil de pá, de ponta e de
rotação de esteira. Portanto, para todo V > 0:

    T_real(V) ≤ T_ideal(V)        ⟺        FoM(V) := T_real(V)/T_ideal(V) ≤ 1

Isto **não é preferência de modelagem**. É conservação de momento, e não
depende de nenhuma calibração.

### §1.1 — Medição contra o teto

Medido no baseline real (Toyota 1GD-FTV, `ed537ae`, valores extraídos do
próprio código por sonda: `ρ_sl = 1,22501226599069457`,
`P_eixo(3000 rpm, 0 m) = 144,240990702 kW`,
`P_eixo(2640 rpm, 2500 m) = 122,887229388 kW`,
`ρ_cr = 0,95685612333150516`):

| Ponto de operação | V (m/s) | J | `thrust_available_n` / `T_ideal` |
|---|---|---|---|
| rolagem | 10,0 | 0,2122 | **2,1432** ❌ |
| rolagem | 20,0 | 0,4243 | **1,3417** ❌ |
| **V_LOF ≡ Vr** | 35,361 | 0,7502 | **1,0372** ❌ |
| **Vx** (138,87 km/h) | 38,575 | 0,8184 | **1,0095** ❌ |
| Vy (148,44 km/h), nível do mar | 41,232 | 0,8748 | 0,9908 |
| teto 5200 m, no Vy real de lá | 48,241 | 1,0235 | 0,9313 |
| cruzeiro 280 km/h | 77,778 | 1,8751 | 0,8237 |
| V máx | 83,395 | 1,5611 | 0,8604 |

**Quatro dos oito pontos violam o teto físico, e dois deles alimentam gates que
hoje PASSAM** (gradiente CS 23.65 via Vx; balanço de rotação via V_LOF). Os
outros dois violadores estão na rolagem de decolagem, cujo gate já REPROVA.

**`old→new` (correção da revisão de plano, antes de qualquer código).** Duas
linhas desta tabela estavam medidas com metodologia errada na primeira versão
desta spec, e a revisão de plano as recomputou contra as funções REAIS de
produção:

- **V máx**: a spec usava `J = 2,0106`, derivado de `prop_rpm_cruise` (2640 rpm
  de motor). Mas `agents::performance::max_level_speed_ms` usa
  `engine.rpm_rated` = **3400 rpm** (`performance.rs:1020`). Com o rpm certo,
  `J = 1,5611` e a razão é **0,8604**, não 0,7895. Não muda veredito (já estava
  abaixo de 1).
- **Teto de serviço**: a spec reusava o `Vy` de NÍVEL DO MAR (41,232 m/s) na
  altitude do teto. Mas `service_ceiling_m` é chamado com `mass_mid`
  (≈35% do combustível queimado, `performance.rs:1112`), e o `Vy` real a 5200 m
  com essa massa é **48,241 m/s**, `J = 1,0235` — razão **0,9313**, que **NÃO
  viola o teto**. A afirmação de que o teto de serviço era alimentado por uma
  tração impossível era FALSA e foi retirada.

A conclusão do ciclo não depende dessas duas linhas: rolagem, `V_LOF` e `Vx`
seguem violando, e são exatamente os pontos que alimentam o backlog #15 e o
gate de gradiente. Mas o número errado não pode virar registro permanente de
backlog, então fica corrigido aqui antes de a Task 5 escrevê-lo.

### §1.2 — Consequência para o backlog #15

A banda de indeterminação de ≈8,3 pp de MAC do #15 tinha dois extremos
descritos como "coerentes de modelagem":

- **≈16,28% MAC** — modelo de solo nos dois termos, resíduo zero.
- **≈24,57% MAC** — manter o polinômio no termo de momento e somar o resíduo.

O segundo ramo mantém, em `Vr`, uma tração **3,72% acima do limite de
quantidade de movimento**. Está **excluído por física**, não por preferência.
A decisão que o backlog #15 registrava como "de usuário" é, medida, uma
decisão já tomada pela conservação de momento. Este ciclo registra isso.

---

## §2 — A lei unificada

Uma única função devolve a tração da hélice em qualquer velocidade:

    T(V) = FoM(J) · T_ideal(V, P_eixo, ρ, A)

    T_ideal(V) = 2·ρ·A·u·(u − V),   u = raiz real de u³ − V·u² − K = 0,
                                    K = P_eixo/(2·ρ·A)
    J = V / (n_hélice · D),          n_hélice = (rpm_motor / psru_ratio)/60

`P_eixo = engine.power_kw_at(rpm, alt) · psru_efficiency · 1000` — potência de
EIXO, pós-PSRU (mesma referência de `shaft_power_kw`, inalterada).

A cúbica e o Newton **já existem e já estão validados** em
`thrust_ground_roll_n` (ciclo 12): partida `u₀ = V + K^(1/3)`, convergência
monotônica provada, guarda `if k <= 0.0 { return 0.0 }` para potência nula.
Este ciclo **não reescreve o solver** — só troca o multiplicador constante
`static_thrust_factor` pela curva `FoM(J)` e passa a usar a função em todo
lugar.

### §2.1 — Sem ramos, sem emendas

A função nova **não tem ramo `if v_ms < 0.5`**, **não chama
`prop_efficiency`** e **não chama `propulsion::thrust_n`**. Consequências
diretas, todas verificáveis:

- morre o `η(0) = 0,58` (por definição η = T·V/P → 0 quando V → 0; a lei nova
  dá η = FoM·V/u → 0 automaticamente, porque u → K^(1/3) finito);
- morre o salto de **84.843,5 N em V = 1,0 m/s**;
- morre a **janela de tração NULA em V ∈ [0,5; 1,0)** m/s (guarda
  `if v_ms < 1.0 { return 0.0 }` de `thrust_n`);
- morre o corte duro em J > 2,8 (a lei nova dá T → 0 suavemente quando V → ∞,
  porque u → V).

### §2.2 — Como as âncoras chegam às funções

`static_thrust_factor: f64` some das assinaturas e é substituído por um valor
único que carrega as três âncoras:

```rust
/// Figura de mérito da hélice — tração real / tração ideal de disco atuador.
/// Ver spec ciclo 13 §3. Âncoras são propriedades da HÉLICE (config
/// `[propeller]`), não da missão.
#[derive(Debug, Clone, Copy)]
pub struct FigureOfMerit {
    pub fom_static: f64,   // J = 0
    pub fom_design: f64,   // J = j_design
    pub j_design:   f64,
}

impl FigureOfMerit {
    pub fn at(&self, j: f64) -> f64 { /* §3, linear + grampo */ }
}
```

**Onde a curva mora (correção da revisão de plano).** A primeira versão desta
spec dizia apenas "troque o parâmetro". A revisão mediu o estrago: os AGENTES
(`PropulsionAgent::run`, `MissionAgent::run`, `PerformanceAgent::run`) **não
recebem `cfg`**, então um parâmetro novo neles quebraria 12+ call sites em
`src/orchestrator.rs`, `src/main.rs`, `tests/gear_tipback.rs` e
`tests/schema_v4.rs` — vários deles fora de qualquer lista de arquivos do
plano original.

Solução adotada: **as três âncoras viajam em `AircraftState`**, exatamente como
`psru_ratio`, `psru_efficiency` e `prop_diameter_m` já viajam.
`AircraftState::from_config` passa a copiá-las, e `AircraftState` ganha
`pub fn figure_of_merit(&self) -> FigureOfMerit`. Como todo agente já recebe
`state: &AircraftState`, **nenhuma assinatura de agente muda**. Só as funções
de baixo nível trocam o parâmetro, e numa razão 1:1 (mesma aridade).

Assinaturas afetadas — trocar `static_thrust_factor: f64` por
`fom: FigureOfMerit`, e **acrescentar `psru_ratio: f64` onde faltar**. São
**13 funções** (12 em `performance.rs` + `trim_authority::thrust_at_rotation_n`)
e ≈55 call sites:

- `performance::thrust_available_n` (já tem `psru_ratio`)
- `performance::excess_power_kw`
- `performance::max_level_speed_ms`
- `performance::takeoff_ground_roll_m` / `takeoff_distance_m` /
  `takeoff_distance_50ft_m`
- `performance::landing_ground_roll_m` / `landing_distance_50ft_m` (consomem
  tração? NÃO — o pouso não tem termo de tração; **conferir e, se não
  consumirem, NÃO alterar a assinatura**)
- `trim_authority::thrust_at_rotation_n`

**`thrust_ground_roll_n` não ganha `psru_ratio` — ela deixa de existir.** A
docstring dela hoje afirma "NÃO recebe `psru_ratio` — teoria de quantidade de
movimento não usa rpm de hélice". Essa frase deixa de valer: a figura de mérito
é uma propriedade de PÁ indexada por J, e J precisa da rpm da hélice. Registrar
como `old→new` na docstring da função que a substitui.

---

## §3 — A figura de mérito e suas duas âncoras

    FoM(J) = fom_static + (fom_design − fom_static) · min(J / j_design, 1)

Linear em J, grampeada em `fom_design` para J ≥ `j_design`. Monótona
não-decrescente. Domínio: todo J ≥ 0.

### §3.1 — Âncora estática (J = 0): identidade exata com hoje

`fom_static = 0.75` é **exatamente** o `static_thrust_factor` de McCormick que
o baseline já usa. Em V = 0 a cúbica degenera em `u³ = K`, e

    T(0) = fom_static · (2ρA·P²)^(1/3)

que é **algebricamente idêntico** a `static_thrust_ideal_n × static_thrust_
factor`, o ramo estático de hoje. Valor congelado, computado com os números do
próprio código:

    T(0) = 3740,0919357793 N     (inalterado pelo ciclo)

### §3.2 — Âncora de projeto (J = j_design): cruzeiro preservado por construção

`fom_design` é retro-derivada UMA VEZ para que a lei nova reproduza, no ponto
de cruzeiro real do baseline, a eficiência que o polinômio JavaProp entrega
hoje:

    η_poly(J_cr)  = −0,15·J_cr² + 0,39·J_cr + 0,58
    η_ideal(J_cr) = V_cr / u(V_cr, P_eixo_cr, ρ_cr)
    fom_design    = η_poly(J_cr) / η_ideal(J_cr)

Computado com os valores exatos do código (V_cr = 280/3,6 = 77,7̄ m/s,
`prop_rpm_cruise = 1414,0332083556507`, `P_eixo_cr = 122,887229388 kW`,
`ρ_cr = 0,95685612333150516`):

| Grandeza | Valor |
|---|---|
| `j_design` | **1,87514348025711675** |
| η_poly(j_design) | 0,78388149656765982 |
| u no disco | 81,72925779175839978 m/s |
| η_ideal | 0,95165158572651209 |
| **`fom_design`** | **0,82370639457215544** |

**Cruzeiro, consumo, alcance e autonomia ficam inalterados por construção.**
Esta é a razão de escolher esta âncora e não o pico do polinômio.

**A âncora tem uma dependência que ela não controla** (achado da revisão de
plano). `j_design` só coincide com o `J` de cruzeiro real enquanto
`search_cruise_rpm` continuar escolhendo **2640 rpm**. Essa escolha é o
argmin de BSFC entre os rpms que entregam a potência requerida — e este ciclo
muda justamente como a potência requerida é calculada (§5). Se o rpm ótimo
mudar, `J_cruzeiro ≠ j_design`, `FoM(J_cruzeiro) ≠ fom_design`, e a
preservação de alcance/autonomia deixa de ser exata.

Isso NÃO torna a guarda §8.3 tautológica — ao contrário, é exatamente o que
ela pega. **A task de cruzeiro DEVE reportar explicitamente o `engine_rpm`
escolhido**, e um valor diferente de 2640 é gatilho de escalação, não um
detalhe.

### §3.3 — Premissa calibrada, DECLARADA (padrão do projeto, 6ª ocorrência)

`fom_design` e `j_design` são retro-derivadas do polinômio JavaProp no ponto de
cruzeiro do baseline E12. Isso as torna válidas **enquanto o polinômio for
confiável em J ≈ 1,875** — dentro do domínio que ele declara (0 < J < 2,8),
mas **fora da faixa onde a própria docstring diz que o pico ocorre (J ≈
1,3–1,5)**. Duas consequências que a implementação DEVE registrar, não
esconder:

1. Depois deste ciclo o polinômio é **apagado**; as duas âncoras passam a ser
   propriedades declaradas da HÉLICE, em `[propeller]` do TOML. Trocar de
   hélice é trocar o TOML — mesma política de "nunca hardcodar dados" do
   projeto.
2. `j_design` foi derivada de `prop_rpm_cruise`, que hoje é uma **saída** da
   busca de rpm de cruzeiro. Congelá-la em config quebra essa
   circularidade de propósito: a partir daqui é entrada de projeto da hélice,
   não resultado. Se a velocidade de cruzeiro, a razão de PSRU ou o diâmetro
   mudarem, `j_design` NÃO se ajusta sozinha — a âncora fica obsoleta em
   silêncio. **Registrar como item de backlog novo, com esta direção de erro
   nomeada.**

### §3.4 — A forma entre as âncoras não decide nada, e isso foi medido

A faixa de FoM é estreita (0,750 → 0,8237), então a forma escolhida é de baixa
alavancagem. Medido na decolagem em grama (o gate mais sensível a tração):

| Forma de FoM(J) | FoM(J_LOF) | Rolagem | `to_50ft_grass_m` |
|---|---|---|---|
| 0,75 constante (= publicado hoje) | 0,7500 | 664,2 | 819,1 |
| quadrática `(J/J_d)²` | 0,7618 | 652,7 | 807,6 |
| **linear (adotada)** | 0,7795 | 629,6 | **784,5** |
| raiz `(J/J_d)^0,5` | 0,7966 | 603,9 | 758,8 |
| `fom_design` constante (limite otimista) | 0,8237 | 559,5 | 714,4 |

Amplitude total da escolha de forma: **≈93 m em 819 m (11%)**, e **nenhuma
forma faz a decolagem caber nos 600 m**. A adoção da linear é registrada como
escolha de simplicidade com sensibilidade medida, não como resultado físico.
A implementação DEVE reportar o valor medido; divergência > 5% do previsto
acima é gatilho de escalação.

---

## §4 — O que é apagado

Este ciclo **remove código**, não só adiciona. Nada fica como alias morto.

| Símbolo | Destino | Motivo |
|---|---|---|
| `agents::propulsion::prop_efficiency` | **APAGADO** | η vira saída derivada (§5), não entrada polinomial. Viola o teto físico em 5 de 8 pontos (§1.1). |
| `agents::propulsion::thrust_n` | **APAGADO** | único consumidor era `thrust_available_n`; sua guarda `v_ms < 1.0` é a origem da janela nula. |
| `agents::performance::thrust_ground_roll_n` | **APAGADO** | funde-se em `thrust_available_n`. |
| `agents::performance::static_thrust_ideal_n` | **MANTIDO**, privado | continua sendo a prova independente da identidade estática do §3.1. |
| `[performance].static_thrust_factor` | **MIGRA** para `[propeller].fom_static` | com erro de migração explícito (§9.2), padrão já usado em `shaft_height_m`→`prop_axis_above_cg_m` (ciclo 5) e nos itens fixos (ciclo 3). |

`agents::propulsion::advance_ratio` e `prop_rpm` **permanecem** — a lei nova
usa J, então continuam sendo a fonte única de verdade de razão de avanço.

---

## §5 — Cruzeiro: a inversão fica em forma FECHADA

Hoje `agents::propulsion::search_cruise_rpm` faz, para cada rpm candidata:

    eta      = prop_efficiency(J)
    p_req_kw = drag_n · V / (eta · 1000)

Com a lei nova, η depende da potência efetivamente absorvida — e em cruzeiro
nivelado a tração exigida é conhecida (`T = drag_n`). Isso NÃO cria ponto fixo:
a quadrática do disco atuador inverte diretamente. De `T = 2ρA·u·(u − V)`:

    u = [ V + √(V² + 2T/(ρA)) ] / 2
    P_ideal   = T · u
    P_eixo_req = P_ideal / FoM(J)
    η          = T·V / P_eixo_req = FoM(J) · V / u

**Fórmula fechada, sem iteração, sem ponto fixo.** É um ganho de robustez
sobre o caminho de hoje. `PropulsionSpec::prop_efficiency` continua existindo
como campo de SAÍDA do JSON, com o mesmo nome e tipo; muda só a origem do
número. `agents::mission` (Breguet, `range_km`/`endurance_h`) consome
`prop.prop_efficiency` sem alteração de assinatura.

**Guarda obrigatória:** se `FoM(J) ≤ 0` (J negativo impossível, ou config
degenerada), `p_req_kw = f64::INFINITY` — mesmo tratamento que o `eta > 0.0`
de hoje já dá. Nunca NaN.

---

## §6 — O invariante de d'Alembert, restaurado

Com uma lei só, `thrust_at_rotation_n` e a tração implícita em `D`/`μN` da
rolagem passam a ser **a mesma chamada da mesma função com os mesmos
argumentos** em `Vr ≡ V_LOF`. O resíduo `(T_solo − T_momento)·h_cg` do backlog
#15 vai a **zero por construção**, e isso é asserção, não comentário (§8.6).

`Vr ≡ V_LOF` continua sendo identidade algébrica e não coincidência:
`VR_OVER_VS0 = 1.1` sobre `Vs0_TO` (`trim_authority.rs:81`) é a mesma fórmula
que `v_lof = 1.10·√(2W/(ρ·area_m2·cl_max_to))` (`performance.rs:672`).

**Nenhum termo do balanço muda além do valor de `T`.** Os termos de solo
`−μ_roll·N·h_cg` e `−D·(h_cg − z_drag_above_cg_m)` introduzidos pela task 4 do
ciclo 12 permanecem exatamente como estão. Este ciclo corrige o `T`, não a
equação.

---

## §7 — A superfície da rotação (fecha o backlog #16)

Hoje `agents::trim_authority` avalia o balanço de rotação com
`mu_roll_ground = cfg.performance.mu_roll_paved` (`trim_authority.rs:809-814`)
enquanto os gates #23/#24 reprovam a GRAMA — o mesmo JSON afirma duas
superfícies para a mesma decolagem.

**Mudança:** o limite dianteiro de rotação passa a ser computado nas DUAS
superfícies e publicado nas duas; o gate avalia a superfície de operação.

Campos novos no bloco `trim` do JSON:

    rotation_limit_pct_mac_paved
    rotation_limit_pct_mac_grass

`rotation_limit_pct_mac` **permanece** e passa a valer o da **superfície de
operação, que este ciclo declara ser a GRAMA** — não o pavimentado. É mudança
de VALOR de um campo existente, com `old→new` obrigatório.

**Por que grama, explicitamente:** não existe hoje campo dizendo qual é a
superfície; a resposta está em quais gates existem. As checagens #23 e #24
avaliam decolagem e pouso **em grama** contra `runway_available_m`, e o TOML de
missão declara a pista como "grama/terra, pista de fazenda típica". A rotação
acontece na mesma pista da mesma decolagem que a #23 mede. Adotar grama é
alinhar os dois; é premissa DECLARADA, não derivada de um campo. Se essa
premissa mudar, muda aqui e nos gates #23/#24 juntos.

`rotation_margin_per_scenario` passa a ser computado sobre a grama, e portanto
muda de valor.

**Direção do efeito, medida no ciclo 12 com o modelo ANTIGO de tração:**
pavimentado 17,7580% → grama 19,6458% (**+1,888 pp**). O efeito da superfície
é aditivo ao efeito da tração (§11) e vai na direção OPOSTA — precisa ser
medido de novo com a lei nova, não transposto.

---

## §8 — Guardas falseáveis

Toda guarda abaixo é obrigatória e cada uma tem que ser capaz de REPROVAR.
Guarda que não consegue falhar não conta.

### §8.1 — Teto de quantidade de movimento (a guarda central, RED-FIRST)

Sobre uma varredura de V em [0,1; 120] m/s (≥ 200 pontos) no baseline real, e
também na fixture sintética:

    assert!(T(V) <= T_ideal(V) * (1.0 + 1e-12))

**Escrever este teste PRIMEIRO, contra a `thrust_available_n` de HOJE, e
demonstrar que ele FALHA** — na varredura contínua ele falha já perto de
V=0,5 m/s, bem antes dos quatro pontos nomeados do §1.1. Só então implementar.

**Honestidade sobre o que esta guarda vira DEPOIS do ciclo** (mesma ressalva
que o §8.3, achado da revisão de plano). Implementada a lei nova,
`thrust_available_n` e `thrust_ideal_momentum_n` chamam a MESMA cúbica, e
`T = FoM(J)·T_ideal` com `FoM ≤ 1` garantido em três camadas (validação de
config, o `min(·, 1)` de `FigureOfMerit::at`, e o grampo). O teste passa a ser
quase tautológico: só pode falhar por bug dentro de `at()`.

Ele continua valendo por dois motivos, e nenhum é "descobrir física agora":
(1) o valor dele é ser RED contra o código de hoje — é a prova documental do
defeito; (2) ele é a rede que pega uma FUTURA reintrodução de modelo de tração
paralelo, que é exatamente como o projeto chegou aqui. Guarda de arquitetura,
não de física. Registrar isso no comentário do teste, não deixar implícito.

### §8.2 — Identidade estática

    |T(0) − static_thrust_ideal_n(...)·fom_static| / T(0) < 1e-12

Congelado: `3740,0919357793 N`. Se este valor mudar, algo além da tração mudou.

### §8.3 — Âncora de cruzeiro (guarda de regressão de alcance)

No baseline real, com `V = 280/3,6`, `rpm = 2640`, `alt = 2500`:

    |η_novo − 0,78388149656765982| < 1e-9

Se esta falhar, `range_km`/`endurance_h`/`fc_cruise_lph` mudaram, e a §3.2 foi
violada.

**Honestidade sobre o que esta guarda é.** Ela não descobre física — o valor é
o alvo do qual `fom_design` foi retro-derivada (§3.2). O que ela verifica é
**concordância entre duas implementações independentes**: `fom_design` foi
computada com a ISA, a curva de potência e o Newton em Python (sonda do chefe),
e o teste roda com a ISA, a curva de potência e o Newton em Rust. Divergência
acima de 1e-9 significa que uma das três difere entre as duas implementações —
exatamente o modo de falha que o ciclo 12 só pegou por sonda.

### §8.4 — Continuidade (mata as emendas)

Para V em [0; 100] m/s, passo 0,01: `|T(V+δ) − T(V)| < 5,0 N` para δ = 0,01.
Falha hoje espetacularmente em V=0,5 e V=1,0 (salto de 84.843 N). Escrever
também RED-FIRST.

### §8.5 — Monotonicidade e positividade

- `T(V)` **estritamente decrescente** em V ∈ [0; 100] a ρ e P fixos.
- `T(V) > 0` para todo V ∈ [0; 100] com P_eixo > 0 (mata a janela nula).
- `FoM(J) ∈ (0; 1]` para todo J ∈ [0; 10]; `FoM` monótona não-decrescente.
- `FoM(0) == fom_static` exatamente; `FoM(j_design) == fom_design` exatamente;
  `FoM(2·j_design) == fom_design` (grampo).

### §8.6 — Resíduo de d'Alembert nulo (fecha o #15)

Para cada um dos 6 cenários de CG, com `Vr` do cenário:

    thrust_at_rotation_n(cenário) == thrust_available_n(v_lof(cenário))

igualdade a 1e-12 relativo. Antes deste ciclo a divergência é **27,69%**.

### §8.7 — Convergência do Newton preservada

O teste de identidade do ciclo 12 (`tracao_de_rolagem_em_v_zero_e_identica_ao_
estatico_no_baseline_real`) e a guarda `k <= 0.0` permanecem válidos e não
podem ser removidos nem afrouxados.

### §8.8 — Pins existentes: `old→new` comentado, TOLERÂNCIA INALTERADA

Todo pin que mudar de valor ganha bloco `old→new` com o valor antigo, o novo e
a causa. **Nenhuma tolerância pode ser alargada.** Um pin que não cabe na
tolerância antiga com o valor novo é um pin que muda de valor, não um pin que
muda de tolerância. Asserção relacional apagada é achado, não conserto: se uma
relação deixou de valer, escreva a relação NOVA e verdadeira, viva.

---

## §9 — Schema 5.6

### §9.1 — Classificação

MINOR com exceção registrada, mesmo padrão de 5.2/5.3/5.4/5.5. Justificativa:

- **Campos adicionados** (MINOR puro): `trim.rotation_limit_pct_mac_paved`,
  `trim.rotation_limit_pct_mac_grass`.
- **Exceção registrada:** `propulsion.prop_efficiency` mantém nome, tipo e
  faixa, mas muda de ORIGEM (polinômio → derivada da lei unificada). No
  baseline real o valor é **idêntico por construção da âncora** (§3.2), então
  nenhum consumidor de JSON quebra. Registrar a mudança de semântica no
  histórico de `docs/aircraft_spec.schema.md`.
- **Exceção registrada:** `trim.rotation_limit_pct_mac` muda de valor E de
  significado (passa da superfície pavimentada para a de operação).

Remoção de `to_distance_*`/`landing_distance_m` continua sendo MAJOR e continua
FORA de escopo (backlog #11).

### §9.2 — Config: migração com erro explícito

`[performance].static_thrust_factor` → `[propeller].fom_static`, mais
`[propeller].fom_design` e `[propeller].j_design`.

Carregar um TOML que ainda tem `[performance].static_thrust_factor` deve
produzir **erro de migração nomeado**, não default silencioso. Todos os TOMLs
de `config/aircraft/` precisam ser atualizados no mesmo commit.

Valores do baseline:

    [propeller]
    fom_static = 0.75
    fom_design = 0.82370639457215544
    j_design   = 1.87514348025711675

---

## §10 — Armadilhas conhecidas (leia antes de codar)

Cada item abaixo custou retrabalho num ciclo anterior.

1. **A armadilha da massa.** `PerformanceAgent::run` usa `state.mtow_kg` =
   **1537,389006 kg**, NÃO `wb.spec.mtow_kg` = 1557,519935 kg (o MTOW de
   envelope estrutural, que é o que aparece no bloco `weight` do JSON). Toda
   conferência à mão de distância de decolagem/pouso tem que usar a primeira.
   No ciclo 12 o chefe E o revisor de plano erraram isso independentemente.
2. **P_eixo é MEDIDO, não retro-derivado.** `P_eixo(3000 rpm, 0 m) =
   144,240990702 kW`. Extraído por sonda do próprio código, não por álgebra
   reversa a partir de um resultado.
3. **Números correntes, não históricos.** `rotation_limit_pct_mac` HOJE é
   **17,757974445030644%** (não 8,908%, que é do ciclo 7, nem 13,354637%, que é
   do ciclo 11). `cg_mac_fwd_pct` = 17,758487182256133%.
4. **Fixture sintética ≠ baseline real.** `config_teste()` tem
   `cl_max = 1,65` / `cl_max_clean = 1,40`, não 2,10 / 1,45. Literais do
   baseline real plantados em teste sintético produzem tautologia ou falso
   vermelho. Se um teste RED precisa de um literal do baseline, use o baseline.
5. **Este ciclo APAGA símbolos públicos.** `prop_efficiency`, `thrust_n` e
   `thrust_ground_roll_n` somem. Cada teste que os chama tem que ser reescrito
   contra a lei nova ou removido com justificativa escrita — não comentado,
   não `#[ignore]`.
6. **A convergência do MTOW é um laço.** Mudar a tração muda a subida, que muda
   o combustível, que muda a massa, que muda a tração. Todos os números do §11
   são projeções de primeira ordem; a implementação mede.

---

## §11 — FAIL esperado (projeção do chefe — não "consertar")

O baseline real está em `validation_status: FAIL` com 4 violações. Este ciclo
**muito provavelmente não reduz esse número, e pode aumentá-lo.** Nenhuma
task deve tunar config para evitar qualquer item abaixo.

| Grandeza | Hoje | Projetado | Confiança |
|---|---|---|---|
| `rotation_limit_pct_mac_paved` | 17,757974% | **≈16,4%** | alta — o resíduo do #15 vai a zero |
| Margem 'Solo (piloto)', **em pavimentado** | +0,000513 pp | ≈+1,4 pp | alta |
| Robustez '2 pax dianteiros', **em pavimentado** | ❌ 16,80 vs 18,09 | provavelmente **resolve** | média |
| Robustez 'Solo (piloto)', **em pavimentado** | ❌ 13,74 vs 18,09 | **persiste** | alta |
| `to_50ft_grass_m` | 819,1 m | **≈784 m** — segue ❌ | alta (§3.4) |
| `ldg_50ft_grass_m` | 646,4 m | ≈inalterado — segue ❌ | alta |
| `range_km` / `endurance_h` / `fc_cruise_lph` | — | **inalterados** | alta (§3.2, §8.3) |
| `v_cruise_kmh` (V máx) | 300,22 km/h | ≈+4% | média |
| **`climb_gradient_pct`** | 12,451842% | **≈7,9% — gate é ≥8,3%** | **BAIXA — pode flipar PASS→FAIL** |
| `rc_sl_ms` | 4,999905 | ≈3,4 (gate ≥1,5) | média |
| `service_ceiling_m` | 5.200 m | cai, porém MENOS que o previsto na 1ª versão desta spec (gate ≥3.000) | baixa |

**Correção da projeção do teto** (mesma origem do `old→new` do §1.1): a versão
anterior desta tabela dizia "cai bastante" supondo que o ponto de operação do
teto violasse o limite físico em 1,0049. Medido corretamente, ele está em
**0,9313** — ou seja, o polinômio ali já era conservador. `FoM` no `J` real
daquele ponto (1,0235) vale ≈0,790, então a tração de subida no teto cai
≈15%, não ≈22%. O teto ainda desce; menos do que eu projetei.
| Superfície da rotação (§7) | pavimentado | grama: **+~1,9 pp**, pode reabrir 'Solo (piloto)' NOMINALMENTE | média |

**O gradiente CS 23.65 é o risco central deste ciclo.** A tração cai ≈21% em
Vx/Vy porque é exatamente ali que o polinômio mais infringe o teto físico
(§1.1). Se ele flipar para FAIL, isso é **resultado**, não regressão: o número
de hoje é sustentado por uma tração fisicamente impossível. Reportar com o
valor medido e a decomposição, nunca ajustar `[performance]` para salvá-lo.

**Interação §7 × §6, atenção:** a unificação de tração AFROUXA o limite de
rotação (≈−1,4 pp) e a mudança para grama o APERTA (≈+1,9 pp). O líquido pode
ser um `rotation_limit_pct_mac` ACIMA do `cg_mac_fwd_pct` de 17,758487%, ou
seja, **'Solo (piloto)' virando violação NOMINAL**. Medir e reportar, não
escolher a superfície que dá o número bonito.

---

## §12 — Fora de escopo

1. **Segmento aéreo do pouso** (`15/tan(3°)` = 286,2 m, 44% da distância de
   pouso em grama; o planeio power-off desta célula na própria configuração de
   pouso é 5,118°). Achado do chefe na abertura deste ciclo — **registrar no
   backlog como item novo**, resolver no ciclo 14.
2. **Efeito solo** (backlog #12) — medido agora em −1,6% na decolagem,
   conservador, segue omitido.
3. **`z_drag_above_cg_m` em `cm_thrust_cruise`** (backlog #10).
4. **Remoção de `to_distance_*`** (backlog #11) — MAJOR.
5. **Pin órfão vs JSON regenerado no `verifica-ciclo.sh`** (backlog #13).
6. **Cruzeiro além do pico de eficiência da hélice.** O ponto de operação é
   J = 1,875 enquanto o polinômio tem pico em J = 1,30 (η 0,8335 contra
   0,7839 — ≈6% de eficiência propulsiva na mesa). Achado NOVO deste ciclo,
   registrar no backlog; é decisão de projeto de hélice/PSRU, não de modelo.
7. **Reprovar margem abaixo do piso de incerteza** (decisão de usuário #3 do
   pós-ciclo 12).
8. **Obsolescência silenciosa de `j_design`** (§3.3 item 2) — registrar no
   backlog.

---

## §13 — Restrições globais

Valem para TODA task deste ciclo.

- Rust 2021, sem dependência nova. `cargo test` inteiro tem que passar.
- **Nunca hardcodar dado de motor/célula em `src/`** — `tests/acceptance.rs`
  faz grep e reprova.
- **Nunca mascarar achado.** Número que diverge >5% do projetado no §11,
  tolerância ou assert alterado, gate que flipa, ou anomalia com causa não
  verificável: **escalar, não seguir.**
- Pins: `old→new` comentado com causa. **Tolerâncias INALTERADAS.**
- `scripts/verifica-ciclo.sh` tem que voltar "Status geral: APROVADO" antes do
  merge.
- Commits frequentes e pequenos, cada task fecha com o `cargo test` verde.
- Trailers de commit:
  `Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>`
  `Claude-Session: https://claude.ai/code/session_01J8DCAdnLPaBhTHpu1rTQaT`
