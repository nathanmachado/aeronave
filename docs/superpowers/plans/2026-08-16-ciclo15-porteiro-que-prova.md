# Ciclo 15 — o porteiro que prova: plano de implementação

> **Para trabalhadores agênticos:** SUB-SKILL OBRIGATÓRIA — use
> `superpowers:subagent-driven-development` para executar este plano tarefa a
> tarefa. Os passos usam checkbox (`- [ ]`).

**Objetivo:** fazer o portão **provar** que os pins de teste e os números da
documentação ainda são o que o pipeline produz, e tornar impossível adicionar um
pin novo desguardado.

**Arquitetura:** um arquivo de teste novo (`tests/pins_vs_json.rs`) com um motor
de análise composto de **funções puras sobre conteúdo** — recebem texto,
devolvem lista de falhas — e uma casca fina que as alimenta com os arquivos
reais. A pureza é o que permite provar, de forma **permanente**, que cada
checagem reprova quando deve: os autotestes montam entrada sintética em memória
e chamam a mesma função que roda em produção. Nada é regenerado e nada é
reescrito: `tests/cli.rs:943` já prova que o JSON commitado ≡ pipeline com
tolerância zero, então o JSON commitado é referência suficiente.

**Stack:** Rust 2021 (toolchain 1.95), `serde_json` (já é dependência, com
`float_roundtrip`). **Nenhuma dependência nova.** O scanner é escrito à mão — o
crate tem quatro dependências e não vale gastar a quinta em `regex`.

**Spec:** `docs/superpowers/specs/2026-08-16-ciclo15-porteiro-que-prova-design.md`
— leia-a inteira antes da sua tarefa, **incluindo o ERRATUM §7.5**, que é o
inventário autoritativo e supersede §7.1/§7.2/§7.3.

## Restrições globais

Valem para TODAS as tarefas, sem exceção:

1. **`aircraft_spec.json` não pode mudar.** `git diff b8827e8 -- aircraft_spec.json`
   vazio ao fim de cada tarefa. Nenhuma tarefa regenera o JSON.
2. **`src/` não pode mudar.** `git diff b8827e8 -- src/` vazio.
3. **`SCHEMA_VERSION` permanece `5.7`.** Sem bump.
4. **Nenhuma tolerância existente pode ser alterada.** Nem apertada, nem
   afrouxada. Se você acha que uma deveria mudar, é achado para o relatório.
5. **Exatamente dois literais podem mudar no ciclo inteiro** — os da spec §7.4,
   em `tests/vn_diagram.rs`: `242.633 → 242.692244` e `3.59 → 3.572607`.
   Qualquer outro que não bata é **achado novo: reporte, não conserte.**
6. **TDD literal.** O passo "rode e confirme que FALHA" não é formalidade: a
   saída da falha é entregável do relatório. Um teste que você nunca viu falhar
   não está verificado, está escrito.
7. Comandos rodam da raiz do worktree. `cargo test --release` para o portão
   completo; `cargo test --test pins_vs_json` para iterar.
8. **Mensagem de commit vai em ARQUIVO.** Heredoc é rejeitado pela guarda de
   isolamento do worktree. Escreva num arquivo do scratchpad e use
   `git commit -q -F <caminho>`. Encerre toda mensagem com:
   ```
   Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
   Claude-Session: https://claude.ai/code/session_01J8DCAdnLPaBhTHpu1rTQaT
   ```

## Estrutura de arquivos

| arquivo | responsabilidade | tarefa |
|---|---|---|
| `tests/pins_vs_json.rs` | **novo.** Motor puro, casca de arquivos reais, autotestes | 1, 2, 3 |
| `tests/generic_engine.rs` | marcadores (39 literais) | 2 |
| `tests/control_surfaces.rs` | marcadores (8) | 2 |
| `tests/vn_diagram.rs` | marcadores (6) + os DOIS literais autorizados | 2 |
| `tests/gear_tipback.rs` | marcadores (5) | 2 |
| `tests/propeller.rs` | marcadores (4) | 2 |
| `tests/acceptance.rs` | marcadores (3) | 2 |
| `tests/empennage.rs` | marcadores (3 cobrados + 1 voluntário na linha 42) | 2 |
| `tests/cli.rs` | marcadores (1) | 2 |
| `tests/schema_v4.rs` | marcadores (1) | 2 |
| `tests/config_files.rs` | isenção de módulo (`//!`) | 2 |
| `docs/aircraft_spec.schema.md` | marcadores + correção dos 4 defeitos | 3 |
| `docs/backlog.md` | retratação do #13 + achados novos | 4 |

---

## Task 1: o motor de análise e a prova de que ele reprova

**Arquivos:**
- Criar: `tests/pins_vs_json.rs`

**Interfaces produzidas** (as tasks 2 e 3 dependem destes nomes exatos):

```rust
struct Fonte { nome: String, conteudo: String }
struct Literal { texto: String, casas: usize }
enum Marcador { Vinculado(String), Isento(String) }

fn mascara_arquivo(conteudo: &str) -> Vec<String>
fn literais(codigo_mascarado: &str) -> Vec<Literal>
fn cobrados(codigo_mascarado: &str) -> Vec<Literal>
fn marcador_rust(linha: &str) -> Option<Marcador>
fn marcadores_markdown(linha: &str) -> Vec<(Marcador, usize)>
fn numero_ptbr(trecho: &str) -> Option<String>
fn valor_json(raiz: &serde_json::Value, caminho: &str) -> Option<f64>
fn casa_na_precisao(literal: &str, real: f64) -> bool
fn confere_vinculos(json: &serde_json::Value, fontes: &[Fonte]) -> (usize, Vec<String>)
fn confere_cobertura(fontes: &[Fonte]) -> Vec<String>
fn confere_doc(json: &serde_json::Value, conteudo: &str) -> (usize, Vec<String>)
fn confere_cobertura_doc(conteudo: &str) -> Vec<String>
```

Nesta tarefa **nenhuma checagem toca arquivo real do repositório**. Só o motor e
a prova. As cascas entram nas Tasks 2 e 3.

**Duas armadilhas que já custaram uma reprovação de plano.** O algoritmo tinha
dois bugs, cada um com ocorrência viva no repositório, e eles se cancelavam na
contagem agregada (70 literais antes e depois). Leia a spec §7.5 antes de
escrever qualquer linha:

- **FIX 1 — range.** A supressão de "dígito precedido por `.`" engole o segundo
  operando de `(9.2..10.2)`. Um `.` precedido de outro `.` é range, não
  separador decimal, e **não** suprime.
- **FIX 2 — string de múltiplas linhas.** A máscara precisa carregar o estado
  `em_string` de uma linha para a próxima, senão a continuação de uma mensagem
  de erro vira literal fantasma.

- [ ] **Passo 1: escreva o cabeçalho e a máscara, com os autotestes dos dois bugs**

Crie `tests/pins_vs_json.rs`:

```rust
//! Checagem de pins contra o `aircraft_spec.json` — ciclo 15, backlog #13.
//!
//! Um pin é um literal escrito À MÃO que afirma qual valor o pipeline produz.
//! A tolerância de 1% que os pins deste projeto usam existe para absorver ruído
//! de compilador/plataforma — e absorve, com a mesma eficiência, um pin que
//! envelheceu. Foi assim que `ldg_50ft_m` ficou 0,0054% fora por quatro commits,
//! e assim que `va_kmh`/`n_gust_vc` passaram ciclos inteiros sem NUNCA terem
//! sido o valor do pipeline.
//!
//! Este arquivo NÃO regenera nada e NÃO reescreve nenhum pin. `tests/cli.rs`
//! (`aircraft_spec_json_commitado_bate_com_o_pipeline_real`) já prova, com
//! tolerância ZERO, que o `aircraft_spec.json` commitado é o que o pipeline
//! produz. Logo o JSON commitado basta como referência, e a comparação aqui
//! pode ser exata.
//!
//! O motor é feito de FUNÇÕES PURAS sobre conteúdo. É o que permite provar, de
//! forma permanente, que cada checagem reprova quando deve: os autotestes
//! montam entrada sintética e chamam a MESMA função que roda em produção.

/// Um arquivo carregado para análise.
struct Fonte {
    nome: String,
    conteudo: String,
}

impl Fonte {
    fn nova(nome: &str, conteudo: &str) -> Self {
        Fonte { nome: nome.to_string(), conteudo: conteudo.to_string() }
    }
}

/// Mascara um arquivo INTEIRO, devolvendo uma linha mascarada por linha de
/// entrada, de mesmo comprimento em caracteres, com conteúdo de string e
/// comentário `//` substituídos por espaço.
///
/// Por arquivo, e não por linha, porque strings deste repositório atravessam
/// linhas: `gear_tipback.rs:787-789` corta a mensagem de erro com `\` e
/// continua na linha seguinte. Uma máscara sem memória trataria essa
/// continuação como código e colheria o `8.7855` do TEXTO como se fosse um
/// literal — um pin fantasma, que não existe em lugar nenhum.
fn mascara_arquivo(conteudo: &str) -> Vec<String> {
    let mut saida = Vec::new();
    let mut em_string = false;
    for linha in conteudo.lines() {
        let c: Vec<char> = linha.chars().collect();
        let mut m: Vec<char> = Vec::with_capacity(c.len());
        let mut i = 0usize;
        while i < c.len() {
            if em_string {
                if c[i] == '\\' && i + 1 < c.len() {
                    m.push(' ');
                    m.push(' ');
                    i += 2;
                    continue;
                }
                if c[i] == '"' {
                    em_string = false;
                    m.push('"');
                } else {
                    m.push(' ');
                }
                i += 1;
                continue;
            }
            if c[i] == '"' {
                em_string = true;
                m.push('"');
                i += 1;
                continue;
            }
            if c[i] == '/' && i + 1 < c.len() && c[i + 1] == '/' {
                break;
            }
            m.push(c[i]);
            i += 1;
        }
        while m.len() < c.len() {
            m.push(' ');
        }
        saida.push(m.into_iter().collect());
    }
    // Carregar estado pelo arquivo inteiro tem um modo de falha próprio: se a
    // máscara ficar PRESA em modo string — um `r#"..."#`, um literal de
    // caractere `'"'`, uma aspa desemparelhada — ela apaga em silêncio todo
    // literal do resto do arquivo. Cobertura a menos, sem erro, sem aviso:
    // exatamente a doença que este ciclo existe para curar, só que dentro do
    // próprio verificador. Um arquivo Rust válido nunca termina dentro de uma
    // string, então terminar dentro de uma é prova de que a máscara errou.
    assert!(
        !em_string,
        "a máscara terminou o arquivo DENTRO de uma string — construção não \
         suportada (raw string `r#\"…\"#`, literal de caractere `'\\\"'`, ou aspa \
         desemparelhada). A partir do ponto em que ela se perdeu, TODO literal \
         foi apagado e a cobertura caiu sem aviso. Estenda a máscara antes de \
         confiar nesta varredura."
    );
    saida
}

#[test]
fn mascara_apaga_conteudo_de_string_e_comentario() {
    let entrada = r#"    assert!((v - 242.633).abs() < 1.0, "VA fora do pin (~242.633)", v); // nota 3.14"#;
    let m = mascara_arquivo(entrada);
    assert_eq!(m.len(), 1);
    assert_eq!(m[0].chars().count(), entrada.chars().count(),
        "a máscara deve preservar o comprimento");
    assert_eq!(m[0].matches("242.633").count(), 1,
        "só o literal de CÓDIGO sobrevive; a cópia da mensagem é apagada");
    assert!(!m[0].contains("3.14"), "o comentário deve ter sido apagado");
}

#[test]
fn mascara_respeita_aspa_escapada() {
    let m = mascara_arquivo(r#"let s = "diz \"oi\" 1.2345"; let x = 9.8765;"#);
    assert!(!m[0].contains("1.2345"), "literal DENTRO da string não sobrevive");
    assert!(m[0].contains("9.8765"), "literal FORA da string sobrevive");
}

#[test]
#[should_panic(expected = "DENTRO de uma string")]
fn mascara_presa_em_string_reprova_alto() {
    // Uma aspa desemparelhada faria a máscara apagar TODO o resto do arquivo em
    // silêncio. A guarda converte perda de cobertura em falha alta.
    mascara_arquivo("let s = \"nunca fecha;\nlet x = 1.23456;\n");
}

#[test]
fn string_de_multiplas_linhas_nao_vaza_literal() {
    // forma real de tests/gear_tipback.rs:787-789
    let fonte = "assert!((pct - 8.785_545_514_5).abs() < 0.1,\n    \"margem {pct:.4}% divergiu do pin honesto \\\n     ≈8.7855%\");\n";
    let m = mascara_arquivo(fonte);
    assert!(m[0].contains("8.785_545_514_5"), "o pin de código sobrevive");
    assert!(!m[2].contains("8.7855"),
        "o `8.7855` da CONTINUAÇÃO da mensagem é texto, não literal — a máscara \
         precisa lembrar que ainda está dentro da string");
}
```

- [ ] **Passo 2: rode e confirme**

```
cargo test --test pins_vs_json
```

Esperado: 4 testes passando. Se `string_de_multiplas_linhas_nao_vaza_literal`
falhar, o `em_string` não está sendo carregado entre linhas — é o FIX 2 e é o
ponto inteiro deste passo. **Cole a saída no relatório.**

- [ ] **Passo 3: escreva `literais` e `cobrados`, com o autoteste do range**

Acrescente:

```rust
#[derive(Debug, Clone, PartialEq)]
struct Literal {
    /// texto como escrito, já com sinal se houver, ainda com `_`
    texto: String,
    /// casas decimais, desconsiderando `_`
    casas: usize,
}

/// Literais ELEGÍVEIS de uma linha JÁ MASCARADA: os que não estão em posição de
/// tolerância e não são notação científica.
fn literais(codigo: &str) -> Vec<Literal> {
    let c: Vec<char> = codigo.chars().collect();
    let n = c.len();
    let mut out = Vec::new();
    let mut i = 0usize;
    while i < n {
        if !c[i].is_ascii_digit() {
            i += 1;
            continue;
        }
        // Continuação de identificador (`v2`), de outro número, ou de campo
        // (`x.5` não existe, mas `sized.wb` sim). FIX 1: um `.` precedido de
        // OUTRO `.` é o operador de range `..`, não separador decimal — nesse
        // caso o dígito COMEÇA um literal novo e não pode ser suprimido.
        // Sem isto, `(9.2..10.2).contains(…)` perde o `10.2` inteiro.
        if i > 0 {
            let ant = c[i - 1];
            let continuacao = ant.is_alphanumeric()
                || ant == '_'
                || (ant == '.' && !(i > 1 && c[i - 2] == '.'));
            if continuacao {
                i += 1;
                continue;
            }
        }
        let inicio = i;
        let mut j = i;
        while j < n && (c[j].is_ascii_digit() || c[j] == '_') {
            j += 1;
        }
        if j >= n || c[j] != '.' {
            i = j;
            continue; // inteiro, não é literal de ponto flutuante
        }
        let mut k = j + 1;
        if k >= n || !c[k].is_ascii_digit() {
            i = k;
            continue; // `1.` ou `x.abs()`
        }
        while k < n && (c[k].is_ascii_digit() || c[k] == '_') {
            k += 1;
        }
        if k < n && (c[k] == 'e' || c[k] == 'E') {
            i = k;
            continue; // notação científica: nunca elegível
        }

        // menos unário colado ao literal faz parte dele
        let mut inicio_real = inicio;
        if inicio > 0 && c[inicio - 1] == '-' {
            let mut q = inicio as isize - 2;
            while q >= 0 && c[q as usize] == ' ' {
                q -= 1;
            }
            if q < 0 || matches!(c[q as usize], '(' | ',' | '=' | '[') {
                inicio_real = inicio - 1;
            }
        }

        // posição de tolerância, medida ANTES do eventual sinal
        let mut p = inicio_real as isize - 1;
        while p >= 0 && c[p as usize] == ' ' {
            p -= 1;
        }
        let tolerancia = p >= 0
            && (c[p as usize] == '<'
                || c[p as usize] == '>'
                || (c[p as usize] == '='
                    && p > 0
                    && (c[p as usize - 1] == '<' || c[p as usize - 1] == '>')));

        if !tolerancia {
            let texto: String = c[inicio_real..k].iter().collect();
            let casas = texto
                .split('.')
                .nth(1)
                .map(|d| d.chars().filter(|ch| ch.is_ascii_digit()).count())
                .unwrap_or(0);
            out.push(Literal { texto, casas });
        }
        i = k;
    }
    out
}

/// Literais COBRADOS: elegíveis numa linha com `assert` OU com ≥4 casas.
///
/// A cobrança é SEMÂNTICA, não tipográfica. Um piso de casas decimais deixaria
/// passar `3.59` e `242.633` — exatamente os dois pins que este ciclo descobriu
/// fora do valor publicado (spec §7.4). O que obriga um número a ser verificável
/// não é quantos dígitos ele tem; é alguém estar afirmando algo com ele.
fn cobrados(codigo_mascarado: &str) -> Vec<Literal> {
    let tem_assert = codigo_mascarado.contains("assert");
    literais(codigo_mascarado)
        .into_iter()
        .filter(|l| tem_assert || l.casas >= 4)
        .collect()
}

/// Atalho de teste: mascara e devolve só os textos elegíveis de UMA linha.
fn textos(linha: &str) -> Vec<String> {
    literais(&mascara_arquivo(linha)[0]).into_iter().map(|l| l.texto).collect()
}

#[test]
fn range_nao_engole_o_segundo_operando() {
    // forma real de tests/empennage.rs:117
    assert_eq!(
        textos("    assert!((9.2..10.2).contains(&sized.wb.spec.static_margin_pct),"),
        vec!["9.2", "10.2"],
        "`..` é operador de range: o segundo operando é um literal, não a \
         continuação de um campo"
    );
    // e o caso que a supressão existe para tratar continua funcionando
    assert_eq!(textos("let x = v2.0;"), Vec::<String>::new());
}

#[test]
fn literal_apos_operador_de_comparacao_e_tolerancia_e_nao_conta() {
    assert_eq!(textos("assert!((obtido - 0.007367).abs() < 0.001,"), vec!["0.007367"]);
    assert_eq!(textos("assert!(x > 4.0 && x < 4.5);"), Vec::<String>::new());
    assert_eq!(textos("assert!(v >= 1.5);"), Vec::<String>::new());
}

#[test]
fn notacao_cientifica_nao_e_vinculada() {
    assert_eq!(textos("assert!((a - b).abs() < 1e-9);"), Vec::<String>::new());
    assert_eq!(textos("let x = 1.5e3;"), Vec::<String>::new());
}

#[test]
fn menos_unario_entra_no_literal_menos_binario_nao() {
    // o JSON publica -1.52; sem o sinal o vínculo compararia 1.52
    assert_eq!(textos("assert!((vn.n_lim_neg - (-1.52)).abs() < 1e-6);"), vec!["-1.52"]);
    assert_eq!(textos("assert!((obtido - 0.007367).abs() < 0.001);"), vec!["0.007367"]);
}

#[test]
fn sublinhado_de_legibilidade_conta_como_digito() {
    let l = literais(&mascara_arquivo("let p = 7.236_831_147;")[0]);
    assert_eq!(l.len(), 1);
    assert_eq!(l[0].texto, "7.236_831_147");
    assert_eq!(l[0].casas, 9);
}

#[test]
fn cobranca_e_semantica_nao_tipografica() {
    // `3.59` tem 2 casas: um piso de ≥4 o deixaria passar. O `assert` o pega.
    let a = cobrados(&mascara_arquivo("    assert!((vn.n_gust_vc - 3.59).abs() < 0.05,")[0]);
    assert_eq!(a.iter().map(|l| l.texto.as_str()).collect::<Vec<_>>(), vec!["3.59"]);

    // Fora de assert, `0.01` (2 casas) não é cobrado, mas o pin de 10 casas é.
    // É assim que a tabela de generic_engine.rs:1735-1742 fica com UM cobrado
    // por linha, e portanto sem ambiguidade.
    let b = cobrados(&mascara_arquivo("        (\"vy_kmh\", perf.vy_kmh, 167.4067945716, 0.01),")[0]);
    assert_eq!(b.iter().map(|l| l.texto.as_str()).collect::<Vec<_>>(), vec!["167.4067945716"]);
}
```

- [ ] **Passo 4: rode e confirme que FALHA no range antes de aplicar o FIX 1**

Para ver o TDD funcionar, escreva primeiro a condição de continuação SEM o
tratamento de range (`ant == '.'` puro), rode, e observe
`range_nao_engole_o_segundo_operando` falhar com
`left: ["9.2"], right: ["9.2", "10.2"]`. **Cole essa saída.** Só então aplique o
FIX 1 e rode de novo.

```
cargo test --test pins_vs_json
```

Esperado ao fim: **10 testes passando.**

- [ ] **Passo 5: escreva marcadores, resolução de caminho e precisão**

Acrescente:

```rust
#[derive(Debug, Clone, PartialEq)]
enum Marcador {
    /// o literal deve casar com este caminho do `aircraft_spec.json`
    Vinculado(String),
    /// o literal declaradamente não é valor publicado; guarda a razão
    Isento(String),
}

fn interpreta_marcador(depois_de_pin: &str) -> Marcador {
    let resto = depois_de_pin.trim();
    if let Some(razao) = resto.strip_prefix("NAO-PUBLICADO") {
        Marcador::Isento(
            razao
                .trim_start_matches([' ', '—', '-', ':'])
                .trim_end_matches([' ', '-', '>'])
                .trim()
                .to_string(),
        )
    } else {
        Marcador::Vinculado(resto.split_whitespace().next().unwrap_or("").to_string())
    }
}

/// Marcador de uma linha Rust. Exige `PIN:` DEPOIS de um `//`, para que um
/// `PIN:` dentro de string não seja confundido com marcador.
fn marcador_rust(linha: &str) -> Option<Marcador> {
    let com = linha.find("//")?;
    let p = linha[com..].find("PIN:")? + com;
    Some(interpreta_marcador(&linha[p + 4..]))
}

/// TODOS os marcadores Markdown da linha, cada um com o offset de BYTE onde o
/// seu `-->` termina — que é onde a busca pelo número correspondente começa.
///
/// Devolve lista, não `Option`, porque uma linha do schema doc pode afirmar
/// DOIS valores atuais: a 1236 é assim (`cg_limit_fwd_pct_mac` e
/// `cg_limit_aft_pct_mac` lado a lado). Um leitor de "primeiro marcador da
/// linha" conferiria um e deixaria o outro sem verificação nenhuma — cobertura
/// invisível de menos, que é o defeito que este ciclo inteiro combate.
fn marcadores_markdown(linha: &str) -> Vec<(Marcador, usize)> {
    let mut out = Vec::new();
    let mut base = 0usize;
    while let Some(rel) = linha[base..].find("<!--") {
        let abre = base + rel;
        let Some(f) = linha[abre..].find("-->") else { break };
        let fecha = abre + f + 3;
        // `<!-->` é um comentário HTML degenerado: o `-->` começa a menos de 4
        // bytes do `<!--`, e `abre + 4 > fecha - 3`. Fatiar com início maior
        // que fim é PANIC do Rust, não falha de teste — o arquivo de teste
        // inteiro morre sem dizer por quê. Não há ocorrência hoje, mas o custo
        // da guarda é uma linha e o custo de não tê-la é um panic opaco no dia
        // em que alguém escrever um comentário HTML malformado em qualquer
        // ponto do documento.
        if fecha < abre + 7 {
            base = fecha;
            continue;
        }
        let interior = &linha[abre + 4..fecha - 3];
        if let Some(p) = interior.find("PIN:") {
            out.push((interpreta_marcador(&interior[p + 4..]), fecha));
        }
        base = fecha;
    }
    out
}

/// Primeiro número em português do trecho, devolvido com ponto decimal.
///
/// Recebe uma FATIA, nunca um índice: `str::find` devolve offset de BYTE e uma
/// varredura em `Vec<char>` usa índice de CARACTERE. O schema doc é acentuado —
/// misturar os dois recortaria no meio de um multibyte.
fn numero_ptbr(trecho: &str) -> Option<String> {
    let c: Vec<char> = trecho.chars().collect();
    let mut i = 0usize;
    while i < c.len() && !c[i].is_ascii_digit() {
        i += 1;
    }
    if i >= c.len() {
        return None;
    }
    let inicio = if i > 0 && c[i - 1] == '-' { i - 1 } else { i };
    let mut k = i;
    while k < c.len() && (c[k].is_ascii_digit() || c[k] == ',') {
        k += 1;
    }
    while k > i && c[k - 1] == ',' {
        k -= 1; // vírgula de pontuação não faz parte do número
    }
    let bruto: String = c[inicio..k].iter().collect();
    Some(bruto.replace(',', "."))
}

fn valor_json(raiz: &serde_json::Value, caminho: &str) -> Option<f64> {
    let mut v = raiz;
    for seg in caminho.split('.') {
        v = match seg.parse::<usize>() {
            Ok(i) => v.get(i)?,
            Err(_) => v.get(seg)?,
        };
    }
    v.as_f64()
}

/// O pin bate quando é o valor real ARREDONDADO à precisão em que foi escrito.
///
/// Comparar os 17 dígitos seria mudança de estilo disfarçada de checagem — os
/// pins do projeto são escritos truncados por legibilidade. Arredondar à
/// precisão exibida detecta deriva a partir do último dígito ESCRITO, o que
/// para `138.9140767922` significa 1e-10 relativo: oito ordens de grandeza mais
/// apertado que os 0,0054% que originaram o backlog #13.
fn casa_na_precisao(literal: &str, real: f64) -> bool {
    let limpo: String = literal.chars().filter(|c| *c != '_').collect();
    let casas = limpo.split('.').nth(1).map(|d| d.len()).unwrap_or(0);
    let Ok(escrito) = limpo.parse::<f64>() else { return false };
    format!("{:.*}", casas, real) == format!("{:.*}", casas, escrito)
}

#[test]
fn marcador_rust_reconhece_as_duas_formas() {
    assert_eq!(
        marcador_rust("    let p = 7.236_831_147; // PIN: propulsion.endurance_h"),
        Some(Marcador::Vinculado("propulsion.endurance_h".into()))
    );
    match marcador_rust("    // PIN: NAO-PUBLICADO — cenário Rotax/ferry") {
        Some(Marcador::Isento(r)) => assert_eq!(r, "cenário Rotax/ferry"),
        outro => panic!("esperava Isento, veio {outro:?}"),
    }
    assert_eq!(marcador_rust("    let x = 1.0;"), None);
    assert_eq!(
        marcador_rust(r#"    panic!("texto com PIN: performance.vy_kmh dentro");"#),
        None,
        "`PIN:` dentro de string NÃO é marcador"
    );
}

#[test]
fn marcadores_markdown_leem_todos_os_da_linha() {
    let um = marcadores_markdown("subida **<!-- PIN:performance.rc_sl_ms -->3,460341 m/s**");
    assert_eq!(um.len(), 1);
    assert_eq!(um[0].0, Marcador::Vinculado("performance.rc_sl_ms".into()));

    let dois = marcadores_markdown(
        "**<!-- PIN:weight.cg_limit_fwd_pct_mac -->18,268251% < \
         <!-- PIN:weight.cg_limit_aft_pct_mac -->43,460036%** HOJE",
    );
    assert_eq!(dois.len(), 2, "os DOIS marcadores da linha precisam ser lidos");
    assert_eq!(dois[0].0, Marcador::Vinculado("weight.cg_limit_fwd_pct_mac".into()));
    assert_eq!(dois[1].0, Marcador::Vinculado("weight.cg_limit_aft_pct_mac".into()));
    assert!(dois[0].1 < dois[1].1);
}

#[test]
fn comentario_html_degenerado_nao_causa_panic() {
    // `<!-->` tem o `-->` a menos de 4 bytes do `<!--`: sem guarda, o fatiamento
    // de `interior` teria início maior que fim e o arquivo de teste inteiro
    // morreria com um panic opaco.
    assert!(marcadores_markdown("texto <!--> mais texto").is_empty());
    assert!(marcadores_markdown("<!---->").is_empty());
    // e um marcador legítimo logo depois de um degenerado continua sendo lido
    let m = marcadores_markdown("<!--> **<!-- PIN:performance.rc_sl_ms -->3,460341**");
    assert_eq!(m.len(), 1);
    assert_eq!(m[0].0, Marcador::Vinculado("performance.rc_sl_ms".into()));
}

#[test]
fn numero_ptbr_le_virgula_decimal_e_sinal() {
    assert_eq!(numero_ptbr("18,268251% < 43,460036%").as_deref(), Some("18.268251"));
    assert_eq!(numero_ptbr(" -8,818504% MAC (HOJE)").as_deref(), Some("-8.818504"));
    assert_eq!(numero_ptbr("18,268251, ver acima").as_deref(), Some("18.268251"));
    assert_eq!(numero_ptbr("sem número nenhum"), None);
}

#[test]
fn caminho_json_resolve_campo_e_indice_de_array() {
    let j = serde_json::json!({
        "performance": { "vy_kmh": 167.4067945715867 },
        "robustness": { "flips": [ { "limite": 18.47233349501252 } ] }
    });
    assert_eq!(valor_json(&j, "performance.vy_kmh"), Some(167.4067945715867));
    assert_eq!(valor_json(&j, "robustness.flips.0.limite"), Some(18.47233349501252));
    assert_eq!(valor_json(&j, "performance.nao_existe"), None);
}

#[test]
fn precisao_escrita_e_o_que_manda() {
    assert!(casa_na_precisao("138.9140767922", 138.91407679224818));
    assert!(casa_na_precisao("7.236_831_147", 7.2368311470_f64));
    assert!(casa_na_precisao("280.0", 280.0));
    assert!(casa_na_precisao("-1.52", -1.52));
    // os DOIS casos reais da spec §7.4
    assert!(!casa_na_precisao("242.633", 242.69224416885424));
    assert!(!casa_na_precisao("3.59", 3.572607178479214));
    // o caso que originou o backlog #13
    assert!(!casa_na_precisao("502.431095", 502.4582990603992));
}
```

- [ ] **Passo 6: rode**

```
cargo test --test pins_vs_json
```

Esperado: **16 testes passando.** Se `precisao_escrita_e_o_que_manda` falhar em
qualquer das três últimas asserções, a função está frouxa e deixaria passar
exatamente os defeitos que este ciclo existe para pegar.

- [ ] **Passo 7: escreva as quatro funções de conferência (puras)**

Acrescente:

```rust
/// Marcador de uma linha, considerando também a linha anterior se ela for
/// comentário puro.
fn marcador_de(linhas: &[&str], idx: usize) -> Option<Marcador> {
    marcador_rust(linhas[idx]).or_else(|| {
        let anterior = linhas.get(idx.checked_sub(1)?)?;
        if anterior.trim_start().starts_with("//") {
            marcador_rust(anterior)
        } else {
            None
        }
    })
}

/// Linha de COMENTÁRIO puro — não contém código, logo não contém pin.
fn e_comentario(linha: &str) -> bool {
    let t = linha.trim_start();
    t.starts_with("//") || t.starts_with('*')
}

/// Isenção de MÓDULO: só vale no cabeçalho `//!`, antes de qualquer código.
/// Uma isenção de linha no meio do arquivo não pode desligar a checagem inteira.
fn tem_isencao_de_modulo(conteudo: &str) -> bool {
    conteudo
        .lines()
        .take_while(|l| l.trim_start().starts_with("//!") || l.trim().is_empty())
        .any(|l| l.contains("PIN:") && l.contains("NAO-PUBLICADO"))
}

/// Confere cada marcador VINCULADO contra o JSON.
/// Devolve (quantos vínculos foram conferidos, lista de falhas).
fn confere_vinculos(json: &serde_json::Value, fontes: &[Fonte]) -> (usize, Vec<String>) {
    let mut vinculados = 0usize;
    let mut falhas = Vec::new();
    for fonte in fontes {
        if tem_isencao_de_modulo(&fonte.conteudo) {
            continue;
        }
        let linhas: Vec<&str> = fonte.conteudo.lines().collect();
        let masc = mascara_arquivo(&fonte.conteudo);
        for idx in 0..linhas.len() {
            if e_comentario(linhas[idx]) {
                continue;
            }
            let Some(Marcador::Vinculado(caminho)) = marcador_de(&linhas, idx) else {
                continue;
            };
            let n = idx + 1;
            let nome = &fonte.nome;
            let cob = cobrados(&masc[idx]);
            if cob.is_empty() {
                falhas.push(format!(
                    "{nome}:{n} — marcador vinculado a '{caminho}' mas a linha não tem \
                     literal cobrado"
                ));
                continue;
            }
            if cob.len() > 1 {
                falhas.push(format!(
                    "{nome}:{n} — marcador AMBÍGUO: {} literais cobrados na mesma linha \
                     ({:?}). Divida a linha, ou use isenção se nenhum for pin.",
                    cob.len(),
                    cob.iter().map(|l| &l.texto).collect::<Vec<_>>()
                ));
                continue;
            }
            vinculados += 1;
            let pin = &cob[0].texto;
            let Some(real) = valor_json(json, &caminho) else {
                falhas.push(format!(
                    "{nome}:{n} — caminho '{caminho}' NÃO EXISTE em aircraft_spec.json. \
                     Um marcador que aponta para lugar nenhum é pior que marcador \
                     nenhum: parece cobertura."
                ));
                continue;
            };
            if !casa_na_precisao(pin, real) {
                let escrito: f64 = pin.replace('_', "").parse().unwrap_or(f64::NAN);
                falhas.push(format!(
                    "{nome}:{n} — pin '{pin}' DIVERGE de {caminho} = {real:.12} \
                     (desvio relativo {:.3e}).\n    \
                     NÃO atualize o pin automaticamente: decida se o valor NOVO está \
                     certo. Se estiver, troque o literal registrando `old→new` com a \
                     razão, e deixe a tolerância INALTERADA.",
                    ((escrito - real) / real).abs()
                ));
            }
        }
    }
    (vinculados, falhas)
}

/// Literais cobrados sem marcador nenhum.
fn confere_cobertura(fontes: &[Fonte]) -> Vec<String> {
    let mut nus = Vec::new();
    for fonte in fontes {
        if tem_isencao_de_modulo(&fonte.conteudo) {
            continue;
        }
        let linhas: Vec<&str> = fonte.conteudo.lines().collect();
        let masc = mascara_arquivo(&fonte.conteudo);
        for idx in 0..linhas.len() {
            if e_comentario(linhas[idx]) || cobrados(&masc[idx]).is_empty() {
                continue;
            }
            if marcador_de(&linhas, idx).is_none() {
                nus.push(format!("{}:{} — {}", fonte.nome, idx + 1, linhas[idx].trim()));
            }
        }
    }
    nus
}

/// Gatilhos que transformam um número numa AFIRMAÇÃO DE ATUALIDADE.
///
/// Um número sem afirmação de atualidade é histórico e NÃO deve ser conferido
/// contra o JSON de hoje — confundir essas duas classes foi o que produziu a
/// retratação da spec §2.
const GATILHOS: [&str; 3] = ["HOJE", "Baseline real", "valor publicado"];

fn confere_doc(json: &serde_json::Value, conteudo: &str) -> (usize, Vec<String>) {
    let mut conferidos = 0usize;
    let mut falhas = Vec::new();
    for (idx, linha) in conteudo.lines().enumerate() {
        let n = idx + 1;
        for (marca, fim) in marcadores_markdown(linha) {
            let Marcador::Vinculado(caminho) = marca else { continue };
            let Some(escrito) = numero_ptbr(&linha[fim..]) else {
                falhas.push(format!("linha {n} — marcador '{caminho}' sem número depois dele"));
                continue;
            };
            let Some(real) = valor_json(json, &caminho) else {
                falhas.push(format!(
                    "linha {n} — caminho '{caminho}' NÃO EXISTE em aircraft_spec.json"
                ));
                continue;
            };
            conferidos += 1;
            if !casa_na_precisao(&escrito, real) {
                falhas.push(format!(
                    "linha {n} — o doc afirma {caminho} = {escrito}, o JSON publica \
                     {real:.12}. Corrija o TEXTO do documento; nunca o JSON."
                ));
            }
        }
    }
    (conferidos, falhas)
}

fn confere_cobertura_doc(conteudo: &str) -> Vec<String> {
    let mut nuas = Vec::new();
    for (idx, linha) in conteudo.lines().enumerate() {
        if !GATILHOS.iter().any(|g| linha.contains(g)) || linha.contains("<!-- PIN:") {
            continue;
        }
        let tem_numero_longo = linha
            .split(|c: char| !(c.is_ascii_digit() || c == ','))
            .any(|t| t.split(',').nth(1).is_some_and(|d| d.len() >= 4));
        if tem_numero_longo {
            nuas.push(format!("linha {} — {}", idx + 1, linha.trim()));
        }
    }
    nuas
}
```

- [ ] **Passo 8: escreva os autotestes que provam que CADA checagem reprova**

Estes são os testes da spec §6 e são **permanentes**: sem eles, a prova de que a
checagem funciona vive só no output efêmero do TDD, e nada impede que alguém
afrouxe `casa_na_precisao` num momento em que nenhum pin real esteja errado.

```rust
fn json_de_teste() -> serde_json::Value {
    serde_json::json!({
        "performance": { "ldg_50ft_m": 502.4582990603992, "rc_sl_ms": 3.460340693496421 }
    })
}

#[test]
fn pin_divergente_reprova() {
    // os números REAIS que originaram o backlog #13: 0,0054% de desvio,
    // sobrevivendo quatro commits sob tolerância de 1%
    let f = [Fonte::nova(
        "sintetico.rs",
        "// PIN: performance.ldg_50ft_m\nlet pin = 502.431095;\n",
    )];
    let (n, falhas) = confere_vinculos(&json_de_teste(), &f);
    assert_eq!(n, 1);
    assert_eq!(falhas.len(), 1, "esperava exatamente uma falha, veio {falhas:?}");
    assert!(falhas[0].contains("502.431095") && falhas[0].contains("502.458299"),
        "a mensagem tem de citar OS DOIS valores: {}", falhas[0]);
}

#[test]
fn pin_correto_nao_reprova() {
    let f = [Fonte::nova(
        "sintetico.rs",
        "// PIN: performance.ldg_50ft_m\nlet pin = 502.458299;\n",
    )];
    let (n, falhas) = confere_vinculos(&json_de_teste(), &f);
    assert_eq!((n, falhas.len()), (1, 0), "falhas: {falhas:?}");
}

#[test]
fn pin_com_caminho_inexistente_reprova() {
    let f = [Fonte::nova(
        "sintetico.rs",
        "// PIN: performance.campo_que_nao_existe\nlet pin = 502.458299;\n",
    )];
    let (_, falhas) = confere_vinculos(&json_de_teste(), &f);
    assert_eq!(falhas.len(), 1);
    assert!(falhas[0].contains("NÃO EXISTE"), "{}", falhas[0]);
}

#[test]
fn literal_longo_sem_marcador_reprova() {
    let f = [Fonte::nova("sintetico.rs", "let x = 1.23456;\n")];
    assert_eq!(confere_cobertura(&f).len(), 1);
}

#[test]
fn literal_curto_em_assert_sem_marcador_reprova() {
    // o caso real de vn_diagram.rs:105 — 2 casas, invisível a piso tipográfico
    let f = [Fonte::nova("sintetico.rs", "assert!((v - 3.59).abs() < 0.05);\n")];
    assert_eq!(confere_cobertura(&f).len(), 1);
}

#[test]
fn tolerancia_nao_exige_marcador() {
    // só o `0.05` na linha; ele está em posição de tolerância
    let f = [Fonte::nova("sintetico.rs", "assert!((v - w).abs() < 0.05);\n")];
    assert!(confere_cobertura(&f).is_empty());
}

#[test]
fn literal_em_comentario_nao_exige_marcador() {
    let f = [Fonte::nova("sintetico.rs", "// valor antigo: 582.341118\n")];
    assert!(confere_cobertura(&f).is_empty());
}

#[test]
fn arquivo_com_isencao_de_modulo_e_ignorado() {
    let com = Fonte::nova(
        "sintetico.rs",
        "//! PIN: NAO-PUBLICADO — round-trip de TOML\n\nlet x = 1.23456;\n",
    );
    assert!(confere_cobertura(std::slice::from_ref(&com)).is_empty());

    // uma isenção de LINHA no meio do arquivo NÃO isenta o arquivo inteiro —
    // senão qualquer isenção desligaria a checagem
    let no_meio = Fonte::nova(
        "sintetico.rs",
        "//! Testes.\n\n// PIN: NAO-PUBLICADO — só desta linha\nlet a = 1.23456;\nlet b = 9.87654;\n",
    );
    assert_eq!(confere_cobertura(&[no_meio]).len(), 1, "só o `b` fica nu");
}

#[test]
fn marcador_ambiguo_reprova_quando_vinculado() {
    let f = [Fonte::nova(
        "sintetico.rs",
        "assert_eq!(f(0.0), 0.75); // PIN: performance.rc_sl_ms\n",
    )];
    let (_, falhas) = confere_vinculos(&json_de_teste(), &f);
    assert_eq!(falhas.len(), 1);
    assert!(falhas[0].contains("AMBÍGUO"), "{}", falhas[0]);
}

#[test]
fn doc_com_valor_divergente_reprova() {
    let doc = "razão **<!-- PIN:performance.rc_sl_ms -->4,999905 m/s** HOJE\n";
    let (n, falhas) = confere_doc(&json_de_teste(), doc);
    assert_eq!(n, 1);
    assert_eq!(falhas.len(), 1);
    assert!(falhas[0].contains("4.999905"), "{}", falhas[0]);
}

#[test]
fn doc_com_valor_correto_nao_reprova() {
    let doc = "razão **<!-- PIN:performance.rc_sl_ms -->3,460341 m/s** HOJE\n";
    let (n, falhas) = confere_doc(&json_de_teste(), doc);
    assert_eq!((n, falhas.len()), (1, 0), "falhas: {falhas:?}");
}

#[test]
fn doc_com_gatilho_sem_marcador_reprova() {
    assert_eq!(confere_cobertura_doc("o valor é 12,345678 HOJE\n").len(), 1);
    // número histórico, sem gatilho: não é cobrado
    assert!(confere_cobertura_doc("no ciclo 11 era 12,345678\n").is_empty());
    // gatilho com número curto: não é cobrado
    assert!(confere_cobertura_doc("são 3,8 g HOJE\n").is_empty());
}
```

- [ ] **Passo 9: rode, confirme e commite**

```
cargo test --test pins_vs_json
cargo test --release
git diff b8827e8 -- src/ aircraft_spec.json
```

Esperado: **28 testes** no arquivo novo, suíte completa verde, `git diff` vazio.

```
git add tests/pins_vs_json.rs
git commit -q -F <arquivo-de-mensagem>
```

---

## Task 2: marcar os testes e ligar as duas checagens do lado Rust

**Arquivos:**
- Modificar: `tests/pins_vs_json.rs` (casca de arquivos reais)
- Modificar: os nove arquivos de teste da tabela de estrutura (marcadores)
- Modificar: `tests/config_files.rs` (isenção de módulo)

**Interfaces consumidas:** `Fonte`, `confere_vinculos`, `confere_cobertura`.

**Inventário: use a spec §7.5** — §7.1/§7.2/§7.3 estão SUPERSEDIDAS e têm
números de linha errados em até 143 linhas. A §7.5.1 traz os 48 vinculados com
caminho JSON e a §7.5.2 os 23 isentos com a razão de cada um. **Copie de lá; não
deduza.**

- [ ] **Passo 1: escreva a casca dos arquivos reais (vai falhar)**

```rust
use std::path::PathBuf;

fn raiz() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn json_commitado() -> serde_json::Value {
    let p = raiz().join("aircraft_spec.json");
    let texto = std::fs::read_to_string(&p)
        .unwrap_or_else(|e| panic!("falha ao ler '{}': {e}", p.display()));
    serde_json::from_str(&texto).expect("aircraft_spec.json deveria ser JSON válido")
}

/// Arquivos de teste varridos. `pins_vs_json.rs` fica DE FORA: seus literais de
/// exemplo são dados de teste, não pins.
fn fontes_reais() -> Vec<Fonte> {
    let mut caminhos: Vec<PathBuf> = std::fs::read_dir(raiz().join("tests"))
        .expect("diretório tests/ deveria existir")
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "rs"))
        .filter(|p| p.file_name().is_some_and(|n| n != "pins_vs_json.rs"))
        .collect();
    caminhos.sort();
    caminhos
        .into_iter()
        .map(|p| {
            let nome = p.file_name().unwrap().to_string_lossy().to_string();
            Fonte::nova(&nome, &std::fs::read_to_string(&p).unwrap())
        })
        .collect()
}

/// Piso de marcadores vinculados. Uma checagem que passa porque não encontrou
/// nada é o mesmo defeito que ela existe para combater: verde sem prova. Se a
/// sintaxe do marcador quebrar, este piso reprova em vez de degradar em
/// silêncio.
///
/// 48 = os 47 cobrados vinculáveis da spec §7.5.1 mais o voluntário de
/// `empennage.rs:42`.
const MINIMO_DE_PINS_VINCULADOS: usize = 48;

#[test]
fn pins_de_teste_batem_com_o_json_commitado() {
    let (vinculados, falhas) = confere_vinculos(&json_commitado(), &fontes_reais());
    assert!(
        falhas.is_empty(),
        "{} pin(s) fora do aircraft_spec.json commitado:\n{}",
        falhas.len(),
        falhas.join("\n")
    );
    assert!(
        vinculados >= MINIMO_DE_PINS_VINCULADOS,
        "só {vinculados} marcadores vinculados encontrados, mínimo é \
         {MINIMO_DE_PINS_VINCULADOS} — a varredura degradou e estaria passando \
         sem provar nada"
    );
}

#[test]
fn todo_literal_cobrado_em_teste_carrega_marcador() {
    let nus = confere_cobertura(&fontes_reais());
    assert!(
        nus.is_empty(),
        "{} literal(is) cobrado(s) sem marcador `// PIN:`.\n\
         Cada um precisa de um caminho do aircraft_spec.json ou de \
         `// PIN: NAO-PUBLICADO — <razão>`:\n{}",
        nus.len(),
        nus.join("\n")
    );
}
```

- [ ] **Passo 2: rode e confirme que FALHA, e CONFIRA A CONTAGEM**

```
cargo test --test pins_vs_json -- --nocapture
```

Esperado: `pins_de_teste_batem_com_o_json_commitado` falha com
"só 0 marcadores vinculados"; `todo_literal_cobrado_em_teste_carrega_marcador`
falha listando **70 literais em 68 linhas**.

Distribuição esperada: `generic_engine`=39, `control_surfaces`=8,
`vn_diagram`=6, `gear_tipback`=5, `propeller`=4, `acceptance`=3, `empennage`=3,
`cli`=1, `schema_v4`=1.

**Se o total não for 70, ou se `gear_tipback` vier 6 em vez de 5, ou
`empennage` vier 2 em vez de 3, PARE e reporte.** Esses três números são o
teste de aceitação dos dois FIXes da Task 1: `gear_tipback`=6 significa que o
fantasma da string multi-linha voltou; `empennage`=2 significa que o `10.2` do
range continua invisível.

**A lista é o seu roteiro para os passos 4 e 5, e é entregável do relatório.**

- [ ] **Passo 3: isente `tests/config_files.rs` por módulo**

Como PRIMEIRA linha do arquivo, antes de qualquer `use`:

```rust
//! PIN: NAO-PUBLICADO — round-trip de parsing de TOML de ponta a ponta: todo
//! literal deste arquivo compara uma struct Rust contra o literal do próprio
//! `config/**/*.toml`, nunca contra saída de pipeline. São 50 literais que nada
//! acrescentariam ao inventário de pins (ciclo 15, backlog #13).
```

- [ ] **Passo 4: marque os 48 vinculados (spec §7.5.1)**

Duas formas, escolha pela que couber:

```rust
// forma de linha inteira, quando o literal está sozinho numa atribuição
// PIN: propulsion.endurance_h
let endurance_pin_h = 7.236_831_147;

// forma de fim de linha, quando a linha já é densa
("vy_kmh", perf.vy_kmh, 167.4067945716, 0.01), // PIN: performance.vy_kmh
```

**Não mude nenhum literal neste passo.** Só comentários.

Cinco destes são pins que **nunca foram verificados por nada** e batem por sorte
— `control_surfaces.rs:44/45/138`, `generic_engine.rs:2587`, `propeller.rs:57`.
Marcá-los é o ganho líquido de cobertura desta tarefa.

`empennage.rs:42` (`3.134`) não é cobrado pela regra (3 casas, linha sem
`assert`); recebe marcador **voluntariamente**. Marcar mais do que a regra exige
é sempre permitido; marcar menos, nunca.

- [ ] **Passo 5: marque as 23 isenções (spec §7.5.2)**

A razão é obrigatória e precisa dizer POR QUE aquele número não é valor
publicado. "não é pin" não é razão.

```rust
// PIN: NAO-PUBLICADO — cenário Rotax + missão ferry, não o par Toyota+default
// que gera o aircraft_spec.json commitado
let mtow_expected = 994.067254_f64;

// PIN: NAO-PUBLICADO — tração estática isolada em V=0; não vira campo do JSON
let congelado = 3740.0919357761986;
```

Duas linhas têm **dois** literais cobrados e por isso são ambíguas — as duas
recebem isenção, onde a ambiguidade é inofensiva porque nada é comparado:

```rust
// PIN: NAO-PUBLICADO — fom_static/fom_design são ENTRADAS de config, não são
// ecoadas no relatório
assert_eq!(fom.at(0.0), 0.75);

// PIN: NAO-PUBLICADO — piso e teto de uma banda de aceitação, não campo do JSON
assert!((9.2..10.2).contains(&sized.wb.spec.static_margin_pct),
```

- [ ] **Passo 6: rode e confirme que o cadeado passa e sobram DOIS pins divergentes**

```
cargo test --test pins_vs_json -- --nocapture
```

Esperado: `todo_literal_cobrado_em_teste_carrega_marcador` **PASSA**;
`pins_de_teste_batem_com_o_json_commitado` **ainda falha**, com exatamente
**dois** divergentes — `vn_diagram.rs:93` e `vn_diagram.rs:105`.

**Cole essa saída. É a prova de que a checagem pega os defeitos reais.**

Se aparecer um TERCEIRO divergente, **PARE**. É achado novo: reporte arquivo,
linha, pin, valor do JSON e desvio. **Não conserte.**

- [ ] **Passo 7: aplique as DUAS únicas mudanças de literal autorizadas**

`tests/vn_diagram.rs`, linhas 93 e 105. Tolerâncias `< 1.0` e `< 0.05`
permanecem **exatamente como estão**:

```rust
    // PIN ATUALIZADO (ciclo 15, backlog #13, spec §7.4): `242.633 → 242.692244`.
    // Este pin NUNCA bateu com o pipeline: o JSON trazia 242,618735 desde o
    // ERRATUM do ciclo 11 e 242,692244 desde o ciclo 13 — o valor escrito não
    // corresponde a NENHUM dos dois. Não é deriva; é pin estimado a olho,
    // sobrevivendo dentro de uma tolerância de 0,41%. Tolerância INALTERADA.
    assert!((vn.va_kmh - 242.692244).abs() < 1.0, "VA {:.1} km/h fora do pin (~242.692244)", vn.va_kmh); // PIN: vn_diagram.va_kmh

    // PIN ATUALIZADO (ciclo 15, backlog #13, spec §7.4): `3.59 → 3.572607`.
    // `n_gust_vc` está IMÓVEL em 3,572607 desde o ciclo 11; o pin 3,59 vinha do
    // hand-check aproximado do brief da task 4.3 e nunca foi o valor do
    // pipeline, em commit nenhum. Tolerância INALTERADA (0,05).
    assert!((vn.n_gust_vc - 3.572607).abs() < 0.05, // PIN: vn_diagram.n_gust_vc
        "n_gust_vc {:.4} fora do pin (~3.572607 ±0.05)", vn.n_gust_vc);
```

Ajuste também os comentários de contexto que citam os valores antigos: o bloco
acima da linha 93 narra `~241.074 → ~242.633 km/h`, e o bloco acima da 105 cita
`n_gust_vc≈3.59` vindo do brief.

- [ ] **Passo 8: rode o portão completo**

```
cargo test --release
git diff b8827e8 -- src/ aircraft_spec.json
```

Esperado: tudo verde, `git diff` vazio.

- [ ] **Passo 9: prove que só dois literais mudaram**

```
git diff b8827e8 -- tests/ | grep -E '^[+-]' | grep -vE '^[+-][+-]' | grep -vE '^[+-]\s*//' | grep -vE '^[+-]\s*$'
```

Esperado: só as linhas dos dois asserts de `vn_diagram.rs`, mais as do arquivo
novo. Qualquer outra linha de código alterada viola a restrição global 5.
**Cole a saída.**

- [ ] **Passo 10: commite**

```
git add tests/
git commit -q -F <arquivo-de-mensagem>
```

---

## Task 3: marcar o schema doc, ligar as checagens e corrigir os quatro defeitos

**Arquivos:**
- Modificar: `tests/pins_vs_json.rs` (casca do doc)
- Modificar: `docs/aircraft_spec.schema.md`

**Interfaces consumidas:** `confere_doc`, `confere_cobertura_doc`,
`json_commitado`, `raiz`.

- [ ] **Passo 1: escreva a casca do doc (vai falhar)**

```rust
const SCHEMA_DOC: &str = "docs/aircraft_spec.schema.md";

fn conteudo_do_schema_doc() -> String {
    std::fs::read_to_string(raiz().join(SCHEMA_DOC)).expect("schema doc deveria existir")
}

/// Piso de números atuais conferidos no doc — mesma razão do piso de pins.
const MINIMO_DE_NUMEROS_NO_DOC: usize = 12;

#[test]
fn numeros_atuais_do_schema_doc_batem_com_o_json() {
    let (conferidos, falhas) = confere_doc(&json_commitado(), &conteudo_do_schema_doc());
    assert!(
        falhas.is_empty(),
        "{} divergência(s) em {SCHEMA_DOC}:\n{}",
        falhas.len(),
        falhas.join("\n")
    );
    assert!(
        conferidos >= MINIMO_DE_NUMEROS_NO_DOC,
        "só {conferidos} números atuais conferidos, mínimo {MINIMO_DE_NUMEROS_NO_DOC} \
         — a varredura degradou e estaria passando sem provar nada"
    );
}

#[test]
fn afirmacao_de_valor_atual_no_doc_exige_marcador() {
    let nuas = confere_cobertura_doc(&conteudo_do_schema_doc());
    assert!(
        nuas.is_empty(),
        "{} linha(s) de {SCHEMA_DOC} afirmam um valor ATUAL sem marcador \
         `<!-- PIN:caminho -->`.\nOu marque o número, ou reescreva a frase para não \
         reivindicar atualidade:\n{}",
        nuas.len(),
        nuas.join("\n")
    );
}
```

- [ ] **Passo 2: rode e confirme que FALHA**

```
cargo test --test pins_vs_json -- --nocapture
```

Esperado: o primeiro falha com "só 0 números conferidos"; o segundo lista as
linhas com gatilho. Segundo a spec §5.6, os gatilhos ocorrem em `:1050`,
`:1236`, `:1362`, `:1381`, `:1410`, `:1424`, `:1429`, `:1504`, `:1601` depois da
linha 1000. **Cole a lista.**

- [ ] **Passo 3: corrija o defeito de `:1236`**

O doc diz `**17,757974% < 43,460036%** HOJE`. O real de
`weight.cg_limit_fwd_pct_mac` é **18,268251**; `cg_limit_aft_pct_mac`
(43,460036) está correto. A linha 1381 do MESMO documento já diz 18,268251% "o
valor publicado HOJE" — o doc se contradizia.

```markdown
ciclo 12; era "≈6,1% < ≈43,5%") **<!-- PIN:weight.cg_limit_fwd_pct_mac -->18,268251% <
<!-- PIN:weight.cg_limit_aft_pct_mac -->43,460036%** HOJE, ver
```

- [ ] **Passo 4: corrija os defeitos de `:1424` e `:1429`**

`rc_sl_ms` está como `4,999905` e o real é **3,460341**. `vy_kmh` está como
`148,435393` e o real é **167,406795**. `vx_kmh` está `138,871480` contra
`138,914077`. Os três pararam no ciclo 11 e não acompanharam a mudança do modelo
de tração do ciclo 13.

**Preserve a narrativa histórica da célula** e acrescente a atualização com
marcador, no padrão que o resto do documento já usa:

```markdown
Baseline real (ciclo 11): 4,999902 → 4,999905. **Ciclo 13 (lei única de
tração)**: <!-- PIN:performance.rc_sl_ms -->3,460341 m/s HOJE.
```

Idem para `<!-- PIN:performance.vy_kmh -->167,406795` e
`<!-- PIN:performance.vx_kmh -->138,914077`.

- [ ] **Passo 5: corrija o defeito de `:1601-1603`**

O texto diz `limite = 18,094655%` contra `limite_nominal = 17,757974%` e
descreve o flip como do cenário "Solo (piloto)". Hoje `robustness.flips` traz
`"Cenário '2 pax dianteiros'"` com `limite` = **18,472333** e `limite_nominal` =
**18,268251**. "Solo (piloto)" migrou para violação NOMINAL de envelope no ciclo
13, e o parágrafo ficou parado no fix wave do ciclo 12.

```markdown
sua régua de rotação também difere. Medido HOJE (flips do baseline real,
cenário `2 pax dianteiros`): `limite = <!-- PIN:robustness.flips.0.limite -->18,472333% MAC`
sob perturbação contra `limite_nominal = <!-- PIN:robustness.flips.0.limite_nominal -->18,268251% MAC`
no nominal — a régua ANDA. Os demais limites (tipback,
```

- [ ] **Passo 6: marque as citações atuais que já estão CORRETAS**

Não mudam de valor — ganham marcador para que a deriva de amanhã seja pega:

| linha | caminho |
|---|---|
| `:1137` | `propulsion.prop_efficiency` |
| `:1381` | `trim.rotation_limit_pct_mac` |
| `:1410` | `trim.flare_limit_pct_mac` |
| `:1435` | `performance.ldg_approach_angle_deg` |
| `:1436` | `performance.ldg_flare_height_m` |
| `:1437` | `performance.ldg_air_distance_m` |
| `:1504` | `propeller.prop_clearance_critical_m` |

Para `:1050` e `:1362`, que têm gatilho mas cujo número mais próximo é
histórico: **não invente marcador.** Reescreva a frase para não reivindicar
atualidade, ou afaste o gatilho do número histórico. Registre no relatório qual
das duas você fez e por quê.

- [ ] **Passo 7: rode e confirme que passa**

```
cargo test --test pins_vs_json -- --nocapture
cargo test --release
```

Se `numeros_atuais_do_schema_doc_batem_com_o_json` ainda reprovar em alguma
linha, é achado novo — reporte antes de editar.

- [ ] **Passo 8: confirme o invariante e commite**

```
git diff b8827e8 -- src/ aircraft_spec.json
git add tests/pins_vs_json.rs docs/aircraft_spec.schema.md
git commit -q -F <arquivo-de-mensagem>
```

---

## Task 4: retratar o achado falso e registrar os novos

**Arquivos:**
- Modificar: `docs/backlog.md`

Esta tarefa não escreve código. Escreve o registro — que neste projeto é o
artefato que sobrevive aos ciclos.

- [ ] **Passo 1: reescreva o bloco "SEGUNDA MANIFESTAÇÃO" do item 13**

O bloco atual (commit `5119592`, fix wave do ciclo 14) afirma que
`docs/aircraft_spec.schema.md:809-810` registra valores errados. **É falso.**

Substitua por uma retratação que **preserve a afirmação original citada** —
arqueologia que se apaga não é arqueologia — e explique a causa. Obrigatório:

- A afirmação original, citada, e o commit que a introduziu (`5119592`).
- A prova de que é falsa: as linhas 809-810 narram uma transição `old→new`
  DENTRO da era v5.5 e ambos os valores existiram — `619b4a0` publicava
  `ldg_50ft_m = 502,4582990603992`, `e06e7e7` publicava `582,3411181885572`.
- Que o par `582,521767 / 646,660942` é posterior (`0a6136f`) e está registrado
  corretamente em OUTRO lugar do mesmo documento: entrada v5.7, linha 981, e a
  cadeia `old→new` das linhas 1433-1434.
- A causa: comparou-se um valor da era v5.5 contra um pós-ciclo-13.
- A lição, gêmea da do ciclo 14: **lá se afirmou uma correção que não existia;
  aqui, um defeito que não existia. As duas se curam com a mesma disciplina —
  quem afirma sobre o histórico abre o histórico.**

- [ ] **Passo 2: registre os defeitos REAIS do schema doc**

No lugar da segunda manifestação falsa, os quatro que existem de fato, com os
números da spec §3: `:1236` (`cg_limit_fwd_pct_mac`, 2,9%), `:1424`
(`rc_sl_ms`, 44,5%), `:1429` (`vy_kmh`, 12,8%) e `:1601-1603` (o bloco de
`robustness`, que errava os dois números E o nome do cenário). Marque o item 13
como **RESOLVIDO ciclo 15**.

- [ ] **Passo 3: item novo #24 — pins que nunca bateram**

Com a medição completa da spec §7.4: a tabela mostrando `n_gust_vc` imóvel em
3,572607 desde `8f92c55` e `va_kmh` mudando de 242,618735 para 242,692244 no
ciclo 13, contra pins `3.59` e `242.633` que não correspondem a nenhum estado
que existiu. Nomeie a classe: **pin estimado**, terceira variante da doença do
#13 — pior que a original, porque um pin envelhecido ao menos testemunha um
estado que existiu, e um pin estimado não testemunha nada enquanto ocupa o lugar
de quem testemunharia. **RESOLVIDO ciclo 15.**

- [ ] **Passo 4: item novo #25 — cinco pins que batiam por sorte**

`control_surfaces.rs:44/45/138`, `generic_engine.rs:2587`, `propeller.rs:57`:
pins reais, todos corretos, **nunca verificados por nada**, e ausentes do
inventário original da spec. Registre a causa (o inventário foi levantado por
leitura antes de o scanner existir) e a regra que sai dela: **um inventário de
cobertura que não vem do próprio verificador é palpite bem apresentado.**
**RESOLVIDO ciclo 15.**

- [ ] **Passo 5: item novo #26 — lacuna residual do cadeado**

Copiando a spec §9 item 1: um literal fora de linha de `assert` e com ≤3 casas
escapa das duas regras de cobrança. A tabela de tuplas de
`generic_engine.rs:1735-1742` é essa forma e só é coberta hoje porque aqueles
oito literais têm ≥4 casas. **Um pin novo escrito como tupla com poucas casas
passaria.** Lacuna CONHECIDA e DECLARADA, sem correção — fechá-la exigiria
análise semântica de fluxo.

- [ ] **Passo 6: item novo #27 — legibilidade de `:1431-1432`**

A célula cita `climb_gradient_pct` 12,451842 (hoje 7,913277) dentro de narrativa
rotulada "ciclo 11". **Não é defeito** — não reivindica atualidade, e por isso o
marcador não se aplica. Mas quem ler a célula isolada conclui que 12,451842% é o
valor vigente. Item de legibilidade, não de correção.

- [ ] **Passo 7: rode o portão e commite**

```
bash scripts/verifica-ciclo.sh
git add docs/backlog.md
git commit -q -F <arquivo-de-mensagem>
```

Esperado: **Status geral: APROVADO**.

---

## Verificação de fecho do ciclo

Rodar depois da Task 4, antes da revisão final de branch:

1. `bash scripts/verifica-ciclo.sh` → **Status geral: APROVADO**
2. `git diff b8827e8 -- aircraft_spec.json` → **vazio**
3. `git diff b8827e8 -- src/` → **vazio**
4. `grep -n "SCHEMA_VERSION" src/models/specs.rs` → ainda **5.7**
5. `cargo test --test pins_vs_json` → **32 testes** (28 da Task 1, 2 da Task 2,
   2 da Task 3); suíte total ≥ **551**
6. `git diff b8827e8 -- tests/` contém apenas: comentários `PIN:`, o arquivo
   novo, e **exatamente dois** literais alterados com seus `old→new`
7. Nenhuma tolerância alterada em lugar nenhum
