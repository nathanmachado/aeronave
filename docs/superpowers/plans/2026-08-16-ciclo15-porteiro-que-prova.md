# Ciclo 15 — o porteiro que prova: plano de implementação

> **Para trabalhadores agênticos:** SUB-SKILL OBRIGATÓRIA — use
> `superpowers:subagent-driven-development` para executar este plano tarefa a
> tarefa. Os passos usam checkbox (`- [ ]`).

**Objetivo:** fazer o portão **provar** que os pins de teste e os números da
documentação ainda são o que o pipeline produz, e tornar impossível adicionar um
pin novo desguardado.

**Arquitetura:** um arquivo de teste novo (`tests/pins_vs_json.rs`) que varre
`tests/*.rs` e `docs/aircraft_spec.schema.md` procurando marcadores `PIN:`,
resolve o caminho declarado dentro do `aircraft_spec.json` commitado e exige
igualdade na precisão escrita. Dois cadeados de cobertura garantem que todo
literal sujeito à regra carregue marcador. Nada é regenerado e nada é
reescrito: `tests/cli.rs:943` já prova que o JSON commitado ≡ pipeline com
tolerância zero, então o JSON commitado é referência suficiente.

**Stack:** Rust 2021, `serde_json` (já é dependência, com `float_roundtrip`).
**Nenhuma dependência nova.** O scanner é escrito à mão — o crate tem apenas
quatro dependências e não vale gastar a quinta em `regex` para um analisador de
~120 linhas.

**Spec:** `docs/superpowers/specs/2026-08-16-ciclo15-porteiro-que-prova-design.md`
— leia-a inteira antes da sua tarefa. O plano argumenta a partir dela.

## Restrições globais

Valem para TODAS as tarefas, sem exceção:

1. **`aircraft_spec.json` não pode mudar.** `git diff b8827e8 -- aircraft_spec.json`
   deve sair vazio no fim de cada tarefa. Nenhuma tarefa deste ciclo regenera o
   JSON.
2. **`src/` não pode mudar.** `git diff b8827e8 -- src/` deve sair vazio. Este
   ciclo não altera comportamento nenhum.
3. **`SCHEMA_VERSION` permanece `5.7`.** Sem bump.
4. **Nenhuma tolerância existente pode ser alterada.** Nem apertada, nem
   afrouxada. Se você acha que uma tolerância deveria mudar, isso é um achado
   para o relatório, não uma edição.
5. **Exatamente dois literais podem mudar em todo o ciclo** — os da §7.4 da
   spec, em `tests/vn_diagram.rs`: `242.633 → 242.692244` e `3.59 → 3.572607`.
   Qualquer outro literal que não bata é **achado novo: reporte, não conserte.**
6. **TDD literal.** O passo "rode e confirme que FALHA" não é formalidade: a
   saída da falha é entregável do relatório da tarefa. Um teste que você nunca
   viu falhar não está verificado, está escrito.
7. Comandos rodam da raiz do worktree. `cargo test --release` para o portão
   completo; `cargo test --test pins_vs_json` para iterar rápido.
8. **Mensagem de commit vai em ARQUIVO.** Heredoc (`git commit -F - <<'EOF'`) é
   rejeitado pela guarda de isolamento do worktree. Escreva a mensagem num
   arquivo do scratchpad e use `git commit -q -F <caminho>`. Encerre toda
   mensagem com os dois trailers do projeto:
   ```
   Co-Authored-By: Claude Opus 5 (1M context) <noreply@anthropic.com>
   Claude-Session: https://claude.ai/code/session_01J8DCAdnLPaBhTHpu1rTQaT
   ```

## Estrutura de arquivos

| arquivo | responsabilidade | tarefa |
|---|---|---|
| `tests/pins_vs_json.rs` | **novo.** Scanner de marcadores, resolução de caminho JSON, as quatro checagens e os autotestes | 1, 2, 3 |
| `tests/vn_diagram.rs` | marcadores + os DOIS literais autorizados | 2 |
| `tests/generic_engine.rs` | marcadores (39 sítios) | 2 |
| `tests/control_surfaces.rs` | marcadores (8) | 2 |
| `tests/gear_tipback.rs` | marcadores (6) | 2 |
| `tests/propeller.rs` | marcadores (4) | 2 |
| `tests/acceptance.rs` | marcadores (3) | 2 |
| `tests/empennage.rs` | marcadores (2) | 2 |
| `tests/cli.rs` | marcadores (1) | 2 |
| `tests/schema_v4.rs` | marcadores (1) | 2 |
| `tests/config_files.rs` | isenção de módulo (`//!`) | 2 |
| `docs/aircraft_spec.schema.md` | marcadores + correção dos 4 defeitos | 3 |
| `docs/backlog.md` | retratação do #13 + achados novos | 4 |

---

## Task 1: o scanner e seus autotestes

**Arquivos:**
- Criar: `tests/pins_vs_json.rs`

**Interfaces produzidas** (as tasks 2 e 3 dependem destes nomes exatos):
- `enum Marcador { Vinculado(String), Isento(String) }`
- `fn mascara(linha: &str) -> String`
- `fn literais(codigo_mascarado: &str) -> Vec<Literal>` com
  `struct Literal { texto: String, casas: usize }`
- `fn cobrados(linha: &str) -> Vec<Literal>`
- `fn marcador_rust(linha: &str) -> Option<Marcador>`
- `fn marcadores_markdown(linha: &str) -> Vec<(Marcador, usize)>` — todos os da
  linha, cada um com o offset de BYTE onde o `-->` termina
- `fn valor_json(raiz: &serde_json::Value, caminho: &str) -> Option<f64>`
- `fn casa_na_precisao(literal: &str, real: f64) -> bool`

(`fn numero_ptbr` nasce na Task 3, junto de quem a consome.)

**11 testes** ao fim desta tarefa.

Nesta tarefa NÃO existe nenhuma checagem sobre os arquivos reais do
repositório. Só o motor e a prova de que ele funciona. As checagens reais
entram nas tasks 2 e 3.

**Por que a máscara vem primeiro:** separar "apagar string e comentário" de
"achar literal" torna o segundo trivial e testável isoladamente. Tentar fazer os
dois no mesmo laço é a forma mais rápida de errar aspas escapadas.

- [ ] **Passo 1: escreva o autoteste da máscara (vai falhar — o arquivo não existe)**

Crie `tests/pins_vs_json.rs` contendo APENAS isto:

```rust
//! Checagem de pins contra o `aircraft_spec.json` — ciclo 15, backlog #13.
//!
//! Um pin é um literal escrito À MÃO que afirma qual valor o pipeline produz.
//! A tolerância de 1% que os pins deste projeto usam existe para absorver ruído
//! de compilador/plataforma — e absorve, com a mesma eficiência, um pin que
//! envelheceu. Foi assim que `ldg_50ft_m` ficou 0,0054% fora por quatro commits.
//!
//! Este arquivo NÃO regenera nada e NÃO reescreve nenhum pin. `tests/cli.rs`
//! (`aircraft_spec_json_commitado_bate_com_o_pipeline_real`) já prova, com
//! tolerância ZERO, que o `aircraft_spec.json` commitado é o que o pipeline
//! produz. Logo o JSON commitado basta como referência, e a comparação pode ser
//! exata.

/// Devolve a linha com o MESMO comprimento em caracteres, mas com o conteúdo
/// de strings e o comentário `//` substituídos por espaço.
///
/// Separar esta etapa da busca por literais é deliberado: com a linha
/// mascarada, achar um literal é varredura de dígitos sem estado nenhum.
fn mascara(linha: &str) -> String {
    let c: Vec<char> = linha.chars().collect();
    let mut out: Vec<char> = Vec::with_capacity(c.len());
    let mut i = 0usize;
    let mut em_string = false;
    while i < c.len() {
        if em_string {
            if c[i] == '\\' && i + 1 < c.len() {
                out.push(' ');
                out.push(' ');
                i += 2;
                continue;
            }
            if c[i] == '"' {
                em_string = false;
                out.push('"');
            } else {
                out.push(' ');
            }
            i += 1;
            continue;
        }
        if c[i] == '"' {
            em_string = true;
            out.push('"');
            i += 1;
            continue;
        }
        if c[i] == '/' && i + 1 < c.len() && c[i + 1] == '/' {
            while out.len() < c.len() {
                out.push(' ');
            }
            break;
        }
        out.push(c[i]);
        i += 1;
    }
    while out.len() < c.len() {
        out.push(' ');
    }
    out.into_iter().collect()
}

#[test]
fn mascara_apaga_conteudo_de_string_e_comentario() {
    let entrada = r#"    assert!((v - 242.633).abs() < 1.0, "VA fora do pin (~242.633)", v); // nota 3.14"#;
    let saida = mascara(entrada);
    assert_eq!(saida.chars().count(), entrada.chars().count(),
        "a máscara deve preservar o comprimento para não deslocar posições");
    assert!(saida.contains("242.633"), "o literal de CÓDIGO deve sobreviver");
    assert_eq!(saida.matches("242.633").count(), 1,
        "a cópia dentro da string de mensagem deve ter sido apagada");
    assert!(!saida.contains("3.14"), "o comentário deve ter sido apagado");
    assert!(!saida.contains("VA fora"), "o texto da mensagem deve ter sido apagado");
}

#[test]
fn mascara_respeita_aspa_escapada() {
    let entrada = r#"let s = "diz \"oi\" 1.2345"; let x = 9.8765;"#;
    let saida = mascara(entrada);
    assert!(!saida.contains("1.2345"), "literal DENTRO da string não pode sobreviver");
    assert!(saida.contains("9.8765"), "literal FORA da string deve sobreviver");
}
```

- [ ] **Passo 2: rode e confirme que FALHA**

```
cargo test --test pins_vs_json
```

Esperado: os dois testes rodam. Se `mascara` estiver correta já passam — o que
é aceitável para este passo, porque o alvo do TDD aqui é o passo 3. **Cole a
saída no relatório de qualquer modo.**

- [ ] **Passo 3: escreva o autoteste dos literais (vai falhar — `literais` não existe)**

Acrescente ao arquivo:

```rust
#[derive(Debug, Clone, PartialEq)]
struct Literal {
    /// texto como escrito, já com o sinal se houver, ainda com `_`
    texto: String,
    /// casas decimais, desconsiderando `_`
    casas: usize,
}

/// Literais ELEGÍVEIS de uma linha JÁ MASCARADA.
///
/// Elegível = não está em posição de tolerância (logo após `<`, `<=`, `>`,
/// `>=`) e não é notação científica. String e comentário já foram apagados
/// pela `mascara`.
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
        // continuação de identificador (`v2`), de outro número, ou de um campo
        if i > 0 && (c[i - 1].is_alphanumeric() || c[i - 1] == '_' || c[i - 1] == '.') {
            i += 1;
            continue;
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
            continue; // `1.` ou `x.abs()` — não é literal
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
            let unario = q < 0 || matches!(c[q as usize], '(' | ',' | '=' | '[');
            if unario {
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

fn textos(linha: &str) -> Vec<String> {
    literais(&mascara(linha)).into_iter().map(|l| l.texto).collect()
}

#[test]
fn literal_apos_operador_de_comparacao_e_tolerancia_e_nao_conta() {
    assert_eq!(textos("assert!((v - 0.007367).abs() < 0.001,"), vec!["0.007367"]);
    assert_eq!(textos("assert!(x > 4.0 && x < 4.5);"), Vec::<String>::new());
    assert_eq!(textos("assert!(v >= 1.5);"), Vec::<String>::new());
}

#[test]
fn notacao_cientifica_nunca_e_elegivel() {
    assert_eq!(textos("assert!((a - b).abs() < 1e-9);"), Vec::<String>::new());
    assert_eq!(textos("let x = 1.5e3;"), Vec::<String>::new());
}

#[test]
fn menos_unario_entra_no_literal_menos_binario_nao() {
    // n_lim_neg: o JSON publica -1.52; sem o sinal o vínculo compararia 1.52
    assert_eq!(textos("assert!((vn.n_lim_neg - (-1.52)).abs() < 1e-6);"), vec!["-1.52"]);
    // subtração: o `-` é binário e não pertence ao literal
    assert_eq!(textos("assert!((obtido - 0.007367).abs() < 0.001);"), vec!["0.007367"]);
}

#[test]
fn sublinhado_de_legibilidade_conta_como_digito() {
    let l = literais(&mascara("let p = 7.236_831_147;"));
    assert_eq!(l.len(), 1);
    assert_eq!(l[0].texto, "7.236_831_147");
    assert_eq!(l[0].casas, 9, "os `_` não contam, os dígitos sim");
}
```

- [ ] **Passo 4: rode e confirme que FALHA**

```
cargo test --test pins_vs_json
```

Esperado: **erro de compilação** — `literais`, `Literal` e `textos` não existem
ainda se você acrescentou só os testes. Se você colou também as funções, os
testes devem passar. **Nos dois casos, cole a saída no relatório.** Se algum
teste falhar, o defeito está na função, não no teste: os quatro casos acima são
todos casos REAIS deste repositório.

- [ ] **Passo 5: escreva os autotestes de cobrança, marcador e precisão**

Acrescente:

```rust
/// Literais COBRADOS de uma linha de código: elegíveis que estejam numa linha
/// com `assert` OU tenham ≥4 casas decimais.
///
/// A cobrança é SEMÂNTICA, não tipográfica. Um piso de casas decimais deixaria
/// passar `3.59` e `242.633` — que são exatamente os dois pins que este ciclo
/// descobriu fora do valor publicado (spec §7.4). O que obriga um número a ser
/// verificável não é quantos dígitos ele tem; é alguém estar afirmando algo com
/// ele.
fn cobrados(linha: &str) -> Vec<Literal> {
    let codigo = mascara(linha);
    let tem_assert = codigo.contains("assert");
    literais(&codigo)
        .into_iter()
        .filter(|l| tem_assert || l.casas >= 4)
        .collect()
}

#[derive(Debug, Clone, PartialEq)]
enum Marcador {
    /// o literal deve casar com este caminho do `aircraft_spec.json`
    Vinculado(String),
    /// o literal declaradamente não é um valor publicado; guarda a razão
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

/// Marcador de uma linha Rust: `// PIN: <caminho>` ou
/// `// PIN: NAO-PUBLICADO — <razão>`. Exige que o `PIN:` esteja DEPOIS de um
/// `//`, para que um `PIN:` dentro de string não seja confundido com marcador.
fn marcador_rust(linha: &str) -> Option<Marcador> {
    let com = linha.find("//")?;
    let p = linha[com..].find("PIN:")? + com;
    Some(interpreta_marcador(&linha[p + 4..]))
}

/// TODOS os marcadores Markdown da linha, cada um com o offset (em bytes) onde
/// o seu `-->` termina — que é onde a busca pelo número correspondente começa.
///
/// Devolve uma lista, não um `Option`, porque uma única linha do schema doc
/// pode afirmar DOIS valores atuais. A linha 1236 é exatamente assim
/// (`cg_limit_fwd_pct_mac` e `cg_limit_aft_pct_mac` lado a lado). Um leitor de
/// "primeiro marcador da linha" conferiria o primeiro e deixaria o segundo sem
/// verificação nenhuma — cobertura invisível de menos, que é o defeito que este
/// ciclo inteiro combate.
fn marcadores_markdown(linha: &str) -> Vec<(Marcador, usize)> {
    let mut out = Vec::new();
    let mut base = 0usize;
    while let Some(rel) = linha[base..].find("<!--") {
        let abre = base + rel;
        let fecha = match linha[abre..].find("-->") {
            Some(f) => abre + f + 3,
            None => break,
        };
        let interior = &linha[abre + 4..fecha - 3];
        if let Some(p) = interior.find("PIN:") {
            out.push((interpreta_marcador(&interior[p + 4..]), fecha));
        }
        base = fecha;
    }
    out
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
/// Comparar os 17 dígitos seria uma mudança de estilo disfarçada de checagem —
/// os pins deste projeto são escritos truncados por legibilidade. Arredondar à
/// precisão exibida detecta deriva a partir do último dígito ESCRITO, o que
/// para `138.9140767922` significa 1e-10 relativo: oito ordens de grandeza mais
/// apertado que os 0,0054% que originaram o backlog #13.
fn casa_na_precisao(literal: &str, real: f64) -> bool {
    let limpo: String = literal.chars().filter(|c| *c != '_').collect();
    let casas = limpo.split('.').nth(1).map(|d| d.len()).unwrap_or(0);
    let Ok(escrito) = limpo.parse::<f64>() else {
        return false;
    };
    format!("{:.*}", casas, real) == format!("{:.*}", casas, escrito)
}

#[test]
fn cobranca_e_semantica_nao_tipografica() {
    // `3.59` tem 2 casas: um piso de ≥4 casas o deixaria passar. O `assert` o pega.
    assert_eq!(
        cobrados("    assert!((vn.n_gust_vc - 3.59).abs() < 0.05,")
            .into_iter().map(|l| l.texto).collect::<Vec<_>>(),
        vec!["3.59"]
    );
    // fora de assert, `0.01` (2 casas) não é cobrado, mas o pin de 10 casas é —
    // é assim que a tabela de pins de generic_engine.rs:1735-1742 fica com UM
    // cobrado por linha, e portanto sem ambiguidade.
    assert_eq!(
        cobrados("        (\"vy_kmh\", perf.vy_kmh, 167.4067945716, 0.01),")
            .into_iter().map(|l| l.texto).collect::<Vec<_>>(),
        vec!["167.4067945716"]
    );
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

    // a linha 1236 do schema doc afirma DOIS valores atuais de uma vez
    let dois = marcadores_markdown(
        "**<!-- PIN:weight.cg_limit_fwd_pct_mac -->18,268251% < \
         <!-- PIN:weight.cg_limit_aft_pct_mac -->43,460036%** HOJE",
    );
    assert_eq!(dois.len(), 2, "os DOIS marcadores da linha precisam ser lidos");
    assert_eq!(dois[0].0, Marcador::Vinculado("weight.cg_limit_fwd_pct_mac".into()));
    assert_eq!(dois[1].0, Marcador::Vinculado("weight.cg_limit_aft_pct_mac".into()));
    assert!(dois[0].1 < dois[1].1, "os offsets de fim devem vir em ordem");
}

#[test]
fn caminho_json_resolve_campo_e_indice_de_array() {
    let j: serde_json::Value = serde_json::json!({
        "performance": { "vy_kmh": 167.4067945715867 },
        "robustness": { "flips": [ { "limite": 18.47233349501252 } ] }
    });
    assert_eq!(valor_json(&j, "performance.vy_kmh"), Some(167.4067945715867));
    assert_eq!(valor_json(&j, "robustness.flips.0.limite"), Some(18.47233349501252));
    assert_eq!(valor_json(&j, "performance.nao_existe"), None);
}

#[test]
fn precisao_escrita_e_o_que_manda() {
    // o pin é uma truncagem legítima do valor real
    assert!(casa_na_precisao("138.9140767922", 138.91407679224818));
    assert!(casa_na_precisao("7.236_831_147", 7.2368311470_f64));
    assert!(casa_na_precisao("280.0", 280.0));
    assert!(casa_na_precisao("-1.52", -1.52));
    // os DOIS casos reais da spec §7.4: pins que nunca bateram
    assert!(!casa_na_precisao("242.633", 242.69224416885424));
    assert!(!casa_na_precisao("3.59", 3.572607178479214));
    // o caso que originou o backlog #13
    assert!(!casa_na_precisao("502.431095", 502.4582990603992));
}
```

- [ ] **Passo 6: rode e confirme que passam**

```
cargo test --test pins_vs_json
```

Esperado: **11 testes, todos passando.** Se `precisao_escrita_e_o_que_manda`
falhar em qualquer das três últimas asserções, a `casa_na_precisao` está
frouxa demais e deixaria passar exatamente os defeitos que este ciclo existe
para pegar.

- [ ] **Passo 7: confirme que nada mais quebrou e commite**

```
cargo test --release
git diff b8827e8 --stat -- src/ aircraft_spec.json
git add tests/pins_vs_json.rs
git commit -F <arquivo-de-mensagem>
```

O `git diff` deve sair **vazio**. Escreva a mensagem num arquivo e use
`-F` — heredoc é bloqueado pela guarda de isolamento do worktree.

---

## Task 2: marcar os testes e ligar as duas checagens do lado Rust

**Arquivos:**
- Modificar: `tests/pins_vs_json.rs` (acrescenta as duas checagens reais)
- Modificar: `tests/generic_engine.rs`, `tests/control_surfaces.rs`,
  `tests/gear_tipback.rs`, `tests/vn_diagram.rs`, `tests/propeller.rs`,
  `tests/acceptance.rs`, `tests/empennage.rs`, `tests/cli.rs`,
  `tests/schema_v4.rs` (marcadores)
- Modificar: `tests/config_files.rs` (isenção de módulo)

**Interfaces consumidas da Task 1:** `Marcador`, `mascara`, `literais`,
`cobrados`, `marcador_rust`, `valor_json`, `casa_na_precisao`.

**Inventário completo:** spec §7.1 (22 pins de `generic_engine.rs`, com linha e
caminho JSON), §7.2 (22 pins nos demais arquivos), §7.3 (isenções obrigatórias
com a razão de cada uma), §7.4 (os dois literais autorizados a mudar). **Copie
os caminhos JSON de lá; não os deduza.**

- [ ] **Passo 1: escreva as duas checagens reais (vão falhar — nada está marcado)**

Acrescente a `tests/pins_vs_json.rs`:

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
fn arquivos_de_teste() -> Vec<PathBuf> {
    let mut v: Vec<PathBuf> = std::fs::read_dir(raiz().join("tests"))
        .expect("diretório tests/ deveria existir")
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "rs"))
        .filter(|p| p.file_name().is_some_and(|n| n != "pins_vs_json.rs"))
        .collect();
    v.sort();
    v
}

fn tem_isencao_de_modulo(conteudo: &str) -> bool {
    conteudo
        .lines()
        .take_while(|l| l.trim_start().starts_with("//!") || l.trim().is_empty())
        .any(|l| l.contains("PIN:") && l.contains("NAO-PUBLICADO"))
}

/// Linha de COMENTÁRIO puro — não contém código, logo não contém pin.
fn e_comentario(linha: &str) -> bool {
    let t = linha.trim_start();
    t.starts_with("//") || t.starts_with('*')
}

/// Piso de marcadores vinculados. Uma checagem que passa porque não encontrou
/// nada é o mesmo defeito que ela existe para combater: verde sem prova. Se a
/// sintaxe do marcador quebrar, este piso reprova em vez de degradar em
/// silêncio.
const MINIMO_DE_PINS_VINCULADOS: usize = 44;

#[test]
fn isencao_de_modulo_so_vale_no_cabecalho() {
    let com = "//! PIN: NAO-PUBLICADO — round-trip de TOML\n\nuse std::fs;\nlet x = 1.2345;";
    assert!(tem_isencao_de_modulo(com));

    let sem = "//! Testes de config.\n\nuse std::fs;\nlet x = 1.2345;";
    assert!(!tem_isencao_de_modulo(sem));

    // um marcador de isenção lá no meio do arquivo NÃO isenta o arquivo todo —
    // senão qualquer isenção de linha desligaria a checagem inteira
    let no_meio = "//! Testes de config.\n\nuse std::fs;\n// PIN: NAO-PUBLICADO — só desta linha\nlet x = 1.2345;";
    assert!(!tem_isencao_de_modulo(no_meio));
}

#[test]
fn linha_de_comentario_nao_carrega_pin() {
    assert!(e_comentario("    // to_50ft_grass_m: 436.750941 → 429.914523"));
    assert!(e_comentario("/// doc-comment com 502.458299 dentro"));
    assert!(e_comentario("     * continuação de bloco 1.2345"));
    assert!(!e_comentario("    let x = 1.2345; // PIN: performance.vx_kmh"));
}

#[test]
fn pins_de_teste_batem_com_o_json_commitado() {
    let json = json_commitado();
    let mut vinculados = 0usize;
    let mut falhas: Vec<String> = Vec::new();

    for caminho in arquivos_de_teste() {
        let conteudo = std::fs::read_to_string(&caminho).unwrap();
        if tem_isencao_de_modulo(&conteudo) {
            continue;
        }
        let nome = caminho.file_name().unwrap().to_string_lossy().to_string();
        let linhas: Vec<&str> = conteudo.lines().collect();

        for (idx, linha) in linhas.iter().enumerate() {
            if e_comentario(linha) {
                continue;
            }
            // marcador na própria linha, ou na linha imediatamente anterior
            let m = marcador_rust(linha).or_else(|| {
                let anterior = linhas.get(idx.wrapping_sub(1))?;
                if anterior.trim_start().starts_with("//") {
                    marcador_rust(anterior)
                } else {
                    None
                }
            });
            let Some(Marcador::Vinculado(caminho_json)) = m else {
                continue;
            };
            let cob = cobrados(linha);
            let n = idx + 1;
            if cob.is_empty() {
                falhas.push(format!(
                    "{nome}:{n} — marcador vinculado a '{caminho_json}' mas a linha não tem \
                     literal cobrado"
                ));
                continue;
            }
            if cob.len() > 1 {
                falhas.push(format!(
                    "{nome}:{n} — marcador AMBÍGUO: {} literais cobrados na mesma linha ({:?}). \
                     Divida a linha.",
                    cob.len(),
                    cob.iter().map(|l| &l.texto).collect::<Vec<_>>()
                ));
                continue;
            }
            vinculados += 1;
            let pin = &cob[0].texto;
            let Some(real) = valor_json(&json, &caminho_json) else {
                falhas.push(format!(
                    "{nome}:{n} — caminho '{caminho_json}' NÃO EXISTE em aircraft_spec.json. \
                     Um marcador que aponta para lugar nenhum é pior que marcador nenhum: \
                     parece cobertura."
                ));
                continue;
            };
            if !casa_na_precisao(pin, real) {
                let escrito: f64 = pin.replace('_', "").parse().unwrap_or(f64::NAN);
                falhas.push(format!(
                    "{nome}:{n} — pin '{pin}' DIVERGE de {caminho_json} = {real:.12} \
                     (desvio relativo {:.3e}).\n    \
                     NÃO atualize o pin automaticamente: decida se o valor NOVO está certo. \
                     Se estiver, troque o literal registrando `old→new` com a razão, e deixe a \
                     tolerância INALTERADA.",
                    ((escrito - real) / real).abs()
                ));
            }
        }
    }

    assert!(
        falhas.is_empty(),
        "{} pin(s) fora do aircraft_spec.json commitado:\n{}",
        falhas.len(),
        falhas.join("\n")
    );
    assert!(
        vinculados >= MINIMO_DE_PINS_VINCULADOS,
        "só {vinculados} marcadores vinculados encontrados, mínimo é \
         {MINIMO_DE_PINS_VINCULADOS} — a varredura degradou (sintaxe de marcador quebrada?) \
         e estaria passando sem provar nada"
    );
}

#[test]
fn todo_literal_cobrado_em_teste_carrega_marcador() {
    let mut nus: Vec<String> = Vec::new();

    for caminho in arquivos_de_teste() {
        let conteudo = std::fs::read_to_string(&caminho).unwrap();
        if tem_isencao_de_modulo(&conteudo) {
            continue;
        }
        let nome = caminho.file_name().unwrap().to_string_lossy().to_string();
        let linhas: Vec<&str> = conteudo.lines().collect();

        for (idx, linha) in linhas.iter().enumerate() {
            if e_comentario(linha) {
                continue;
            }
            if cobrados(linha).is_empty() {
                continue;
            }
            let tem = marcador_rust(linha).is_some()
                || linhas
                    .get(idx.wrapping_sub(1))
                    .is_some_and(|a| a.trim_start().starts_with("//") && marcador_rust(a).is_some());
            if !tem {
                nus.push(format!("{nome}:{} — {}", idx + 1, linha.trim()));
            }
        }
    }

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

- [ ] **Passo 2: rode e confirme que FALHA, e ANOTE A CONTAGEM**

```
cargo test --test pins_vs_json -- --nocapture
```

Esperado: `pins_de_teste_batem_com_o_json_commitado` falha com
"só 0 marcadores vinculados", e `todo_literal_cobrado_em_teste_carrega_marcador`
falha listando **70 literais nus, em 69 linhas**.

**A lista de 70 é o seu roteiro de trabalho para o passo 3 e é entregável do
relatório.** Se o número não for 70, PARE e reporte: a spec §5.5 mediu 70 sobre
`b8827e8` e uma divergência significa que o scanner discorda da medição.

Distribuição esperada por arquivo: `generic_engine`=39, `control_surfaces`=8,
`gear_tipback`=6, `vn_diagram`=6, `propeller`=4, `acceptance`=3, `empennage`=2,
`cli`=1, `schema_v4`=1.

- [ ] **Passo 3: isente `tests/config_files.rs` por módulo**

Acrescente como PRIMEIRA linha do arquivo (antes de qualquer `use`):

```rust
//! PIN: NAO-PUBLICADO — round-trip de parsing de TOML de ponta a ponta: todo
//! literal deste arquivo compara uma struct Rust contra o literal do próprio
//! `config/**/*.toml`, nunca contra saída de pipeline. São 50 literais que
//! nada acrescentariam ao inventário de pins (ciclo 15, backlog #13).
```

- [ ] **Passo 4: marque os 44 pins VINCULADOS**

Use as tabelas §7.1 e §7.2 da spec. Duas formas, escolha pela que couber:

```rust
// forma de linha inteira, quando o literal está sozinho numa atribuição
// PIN: propulsion.endurance_h
let endurance_pin_h = 7.236_831_147;

// forma de fim de linha, quando a linha já é densa
("vy_kmh", perf.vy_kmh, 167.4067945716, 0.01), // PIN: performance.vy_kmh
```

**Não mude nenhum literal neste passo.** Só comentários.

- [ ] **Passo 5: marque as 26 isenções**

Use a tabela §7.3 da spec, que traz a razão de cada uma. A razão é obrigatória e
precisa dizer POR QUE aquele número não é um valor publicado — "não é pin" não é
razão.

Exemplos vindos direto da §7.3:

```rust
// PIN: NAO-PUBLICADO — cenário Rotax + missão ferry, não o par Toyota+default
// que gera o aircraft_spec.json commitado
let mtow_esperado = 994.067254;

// PIN: NAO-PUBLICADO — tração estática isolada em V=0; não vira campo do JSON
let congelado = 3740.0919357761986;
```

Para `generic_engine.rs:2533` (`assert_eq!(fom.at(0.0), 0.75);`), que é a única
linha com dois literais cobrados no repositório, use uma isenção — a
ambiguidade é inofensiva porque nada é comparado:

```rust
// PIN: NAO-PUBLICADO — fom_static/fom_design são ENTRADAS de config, não são
// ecoadas no relatório
assert_eq!(fom.at(0.0), 0.75);
```

- [ ] **Passo 6: rode e confirme que o cadeado de cobertura passa**

```
cargo test --test pins_vs_json -- --nocapture
```

Esperado: `todo_literal_cobrado_em_teste_carrega_marcador` **PASSA**;
`pins_de_teste_batem_com_o_json_commitado` **ainda falha**, com exatamente
**dois** pins divergentes — `vn_diagram.rs:93` e `vn_diagram.rs:105`.

**Cole essa saída no relatório. Ela é a prova de que a checagem pega os
defeitos reais.**

Se aparecer um TERCEIRO pin divergente, **PARE**. É achado novo: reporte o
arquivo, a linha, o pin, o valor do JSON e o desvio. **Não conserte.**

- [ ] **Passo 7: aplique as DUAS únicas mudanças de literal autorizadas**

Em `tests/vn_diagram.rs`, linhas 93 e 105. Tolerâncias `< 1.0` e `< 0.05`
permanecem **exatamente como estão**:

```rust
    // PIN ATUALIZADO (ciclo 15, backlog #13, spec §7.4): `242.633 → 242.692244`.
    // Este pin NUNCA bateu com o pipeline: o JSON trazia 242,618735 desde o
    // ERRATUM do ciclo 11 e 242,692244 desde o ciclo 13 — o valor escrito não
    // corresponde a NENHUM dos dois. Não é deriva; é um pin estimado a olho,
    // sobrevivendo dentro de uma tolerância de 0,41%. Tolerância INALTERADA.
    assert!((vn.va_kmh - 242.692244).abs() < 1.0, "VA {:.1} km/h fora do pin (~242.692244)", vn.va_kmh); // PIN: vn_diagram.va_kmh

    // PIN ATUALIZADO (ciclo 15, backlog #13, spec §7.4): `3.59 → 3.572607`.
    // `n_gust_vc` está IMÓVEL em 3,572607 desde o ciclo 11; o pin 3,59 vinha do
    // hand-check aproximado do brief da task 4.3 e nunca foi o valor do
    // pipeline, em commit nenhum. Tolerância INALTERADA (0,05).
    assert!((vn.n_gust_vc - 3.572607).abs() < 0.05, // PIN: vn_diagram.n_gust_vc
        "n_gust_vc {:.4} fora do pin (~3.572607 ±0.05)", vn.n_gust_vc);
```

Ajuste também o comentário de contexto acima da linha 93 (que narra
`~241.074 → ~242.633 km/h`) para encerrar em `242.692244`, e o comentário do
bloco `n_gust_vc` que hoje diz `n_gust_vc≈3.59` vindo do brief.

- [ ] **Passo 8: rode o portão completo**

```
cargo test --release
git diff b8827e8 -- src/ aircraft_spec.json
```

Esperado: **tudo verde**, e o `git diff` **vazio**.

- [ ] **Passo 9: prove que só dois literais mudaram**

```
git diff b8827e8 -- tests/ | grep -E '^[+-]' | grep -vE '^[+-][+-]' | grep -vE '^[+-]\s*//' | grep -vE '^[+-]\s*$'
```

Esperado: **só as linhas dos dois asserts de `vn_diagram.rs`**, mais as linhas
do arquivo novo `pins_vs_json.rs`. Qualquer outra linha de código alterada é
violação da restrição global 5. **Cole a saída no relatório.**

- [ ] **Passo 10: commite**

```
git add tests/
git commit -F <arquivo-de-mensagem>
```

---

## Task 3: marcar o schema doc, ligar as checagens e corrigir os quatro defeitos

**Arquivos:**
- Modificar: `tests/pins_vs_json.rs` (as duas checagens do lado Markdown)
- Modificar: `docs/aircraft_spec.schema.md`

**Interfaces consumidas:** `marcador_markdown`, `valor_json`,
`casa_na_precisao`, `json_commitado`, `raiz` (Tasks 1 e 2).

**Formato numérico do documento:** pt-BR, **vírgula decimal** (`18,268251`).
O comparador precisa normalizar a vírgula para ponto antes de `parse::<f64>()`.
Separador de milhar por ponto não ocorre nos campos marcados — se você
encontrar um, **reporte** em vez de tentar suportá-lo.

- [ ] **Passo 1: escreva as duas checagens do doc (vão falhar)**

Acrescente a `tests/pins_vs_json.rs`:

```rust
const SCHEMA_DOC: &str = "docs/aircraft_spec.schema.md";

/// Gatilhos que transformam um número numa AFIRMAÇÃO DE ATUALIDADE.
///
/// Um número sem afirmação de atualidade é histórico e NÃO deve ser conferido
/// contra o JSON de hoje — foi exatamente confundir essas duas classes que
/// produziu a retratação da spec §2.
const GATILHOS: [&str; 4] = ["HOJE", "Baseline real", "valor publicado", "Medido HOJE"];

/// Primeiro número em português do trecho dado, devolvido com ponto decimal.
///
/// Recebe uma FATIA, nunca um índice. O schema doc é acentuado, então índice de
/// byte (que é o que `str::find` devolve) e índice de caractere (que é o que uma
/// varredura em `Vec<char>` usa) não são a mesma coisa — misturá-los recortaria
/// no meio de um caractere multibyte. Quem chama passa `&linha[fim_do_marcador..]`.
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
    // vírgula final de pontuação não faz parte do número ("18,268251, ver")
    while k > i && c[k - 1] == ',' {
        k -= 1;
    }
    let bruto: String = c[inicio..k].iter().collect();
    Some(bruto.replace(',', "."))
}

#[test]
fn numero_ptbr_le_virgula_decimal_e_sinal() {
    assert_eq!(numero_ptbr("18,268251% < 43,460036%").as_deref(), Some("18.268251"));
    assert_eq!(numero_ptbr(" -8,818504% MAC (HOJE)").as_deref(), Some("-8.818504"));
    assert_eq!(numero_ptbr("3,460341 m/s ao nível do mar").as_deref(), Some("3.460341"));
    assert_eq!(
        numero_ptbr("18,268251, ver acima").as_deref(),
        Some("18.268251"),
        "vírgula de pontuação não entra no número"
    );
    assert_eq!(numero_ptbr("sem número nenhum aqui"), None);
}

#[test]
fn numeros_atuais_do_schema_doc_batem_com_o_json() {
    let json = json_commitado();
    let texto = std::fs::read_to_string(raiz().join(SCHEMA_DOC)).unwrap();
    let mut conferidos = 0usize;
    let mut falhas: Vec<String> = Vec::new();

    for (idx, linha) in texto.lines().enumerate() {
        let n = idx + 1;
        for (marca, fim) in marcadores_markdown(linha) {
            let Marcador::Vinculado(caminho) = marca else {
                continue;
            };
            let Some(escrito) = numero_ptbr(&linha[fim..]) else {
                falhas.push(format!(
                    "{SCHEMA_DOC}:{n} — marcador '{caminho}' sem número depois dele"
                ));
                continue;
            };
            let Some(real) = valor_json(&json, &caminho) else {
                falhas.push(format!(
                    "{SCHEMA_DOC}:{n} — caminho '{caminho}' NÃO EXISTE em aircraft_spec.json"
                ));
                continue;
            };
            conferidos += 1;
            if !casa_na_precisao(&escrito, real) {
                falhas.push(format!(
                    "{SCHEMA_DOC}:{n} — o doc afirma {caminho} = {escrito}, o JSON publica \
                     {real:.12}. Corrija o TEXTO do documento; nunca o JSON."
                ));
            }
        }
    }

    assert!(falhas.is_empty(), "{} divergência(s):\n{}", falhas.len(), falhas.join("\n"));
    assert!(
        conferidos >= 12,
        "só {conferidos} números atuais conferidos no schema doc, mínimo 12 — \
         a varredura degradou e estaria passando sem provar nada"
    );
}

#[test]
fn afirmacao_de_valor_atual_no_doc_exige_marcador() {
    let texto = std::fs::read_to_string(raiz().join(SCHEMA_DOC)).unwrap();
    let mut nuas: Vec<String> = Vec::new();

    for (idx, linha) in texto.lines().enumerate() {
        if !GATILHOS.iter().any(|g| linha.contains(g)) {
            continue;
        }
        let tem_numero_longo = linha.split(|c: char| !(c.is_ascii_digit() || c == ','))
            .any(|t| t.split(',').nth(1).is_some_and(|d| d.len() >= 4));
        if !tem_numero_longo || linha.contains("<!-- PIN:") {
            continue;
        }
        nuas.push(format!("{SCHEMA_DOC}:{} — {}", idx + 1, linha.trim()));
    }

    assert!(
        nuas.is_empty(),
        "{} linha(s) afirmam um valor ATUAL sem marcador `<!-- PIN:caminho -->`.\n\
         Ou marque o número, ou reescreva a frase para não reivindicar atualidade:\n{}",
        nuas.len(),
        nuas.join("\n")
    );
}
```

- [ ] **Passo 2: rode e confirme que FALHA**

```
cargo test --test pins_vs_json -- --nocapture
```

Esperado: `numeros_atuais_do_schema_doc_batem_com_o_json` falha com
"só 0 números conferidos"; `afirmacao_de_valor_atual_no_doc_exige_marcador`
falha listando as linhas com gatilho. Segundo a spec §5.6, os gatilhos ocorrem
em `:1050`, `:1236`, `:1362`, `:1381`, `:1410`, `:1424`, `:1429`, `:1504`,
`:1601` (depois da linha 1000). **Cole a lista no relatório.**

- [ ] **Passo 3: corrija o defeito de `:1236`**

O documento diz `**17,757974% < 43,460036%** HOJE`. O valor real de
`weight.cg_limit_fwd_pct_mac` é **18,268251**; `cg_limit_aft_pct_mac`
(43,460036) está correto. A linha 1381 do MESMO documento já diz 18,268251% "o
valor publicado HOJE" — o doc se contradizia.

```markdown
ciclo 12; era "≈6,1% < ≈43,5%") **<!-- PIN:weight.cg_limit_fwd_pct_mac -->18,268251% <
<!-- PIN:weight.cg_limit_aft_pct_mac -->43,460036%** HOJE, ver
```

- [ ] **Passo 4: corrija os defeitos de `:1424` e `:1429`**

`rc_sl_ms` está registrado como `4,999905` (valor do ciclo 11) e o real é
**3,460341** — 44,5% de erro. `vy_kmh` está como `148,435393` e o real é
**167,406795** — 12,8%. `vx_kmh` está `138,871480` contra `138,914077`.

Os três pararam no ciclo 11 e não acompanharam a mudança do modelo de tração do
ciclo 13. **Preserve a narrativa histórica da célula** e acrescente a
atualização com marcador, no padrão que o resto do documento já usa:

```markdown
Baseline real (ciclo 11): 4,999902 → 4,999905. **Ciclo 13 (lei única de
tração)**: <!-- PIN:performance.rc_sl_ms -->3,460341 m/s HOJE.
```

Mesma forma para `vy_kmh` (`<!-- PIN:performance.vy_kmh -->167,406795`) e
`vx_kmh` (`<!-- PIN:performance.vx_kmh -->138,914077`).

- [ ] **Passo 5: corrija o defeito de `:1601-1603`**

O texto diz `limite = 18,094655% MAC` contra `limite_nominal = 17,757974% MAC`,
e descreve o flip como sendo do cenário "Solo (piloto)". Hoje
`robustness.flips` traz `"Cenário '2 pax dianteiros'"` com `limite` =
**18,472333** e `limite_nominal` = **18,268251**. "Solo (piloto)" migrou para
violação NOMINAL de envelope no ciclo 13, e o parágrafo ficou parado no fix wave
do ciclo 12.

```markdown
sua régua de rotação também difere. Medido HOJE (flips do baseline real,
cenário `2 pax dianteiros`): `limite = <!-- PIN:robustness.flips.0.limite -->18,472333% MAC`
sob perturbação contra `limite_nominal = <!-- PIN:robustness.flips.0.limite_nominal -->18,268251% MAC`
no nominal — a régua ANDA. Os demais limites (tipback,
```

- [ ] **Passo 6: marque as citações atuais que já estão CORRETAS**

Estas não mudam de valor — ganham marcador para que a deriva de amanhã seja
pega. Lista da spec §5.4:

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
atualidade, ou mova o gatilho para longe do número histórico. Registre no
relatório qual das duas você fez e por quê.

- [ ] **Passo 7: rode e confirme que passa**

```
cargo test --test pins_vs_json -- --nocapture
cargo test --release
```

Esperado: **tudo verde.** Se `numeros_atuais_do_schema_doc_batem_com_o_json`
ainda reprovar em alguma linha, é achado novo — reporte antes de editar.

- [ ] **Passo 8: confirme o invariante e commite**

```
git diff b8827e8 -- src/ aircraft_spec.json
git add tests/pins_vs_json.rs docs/aircraft_spec.schema.md
git commit -F <arquivo-de-mensagem>
```

`git diff` **vazio**.

---

## Task 4: retratar o achado falso e registrar os novos

**Arquivos:**
- Modificar: `docs/backlog.md` (item 13, e itens novos ao fim da lista)

Esta tarefa não escreve código. Escreve o registro — que neste projeto é o
artefato que sobrevive aos ciclos.

- [ ] **Passo 1: reescreva o bloco "SEGUNDA MANIFESTAÇÃO" do item 13**

O bloco atual (introduzido pelo commit `5119592`, fix wave do ciclo 14) afirma
que `docs/aircraft_spec.schema.md:809-810` registra valores errados. **É falso.**

Substitua o bloco inteiro por uma retratação que **preserve a afirmação
original citada** — arqueologia que se apaga não é arqueologia — e explique a
causa. Conteúdo obrigatório:

- A afirmação original, citada, e o commit que a introduziu (`5119592`).
- A prova de que é falsa: as linhas 809-810 narram uma transição `old→new`
  DENTRO da era v5.5 e ambos os valores existiram — `619b4a0` publicava
  `ldg_50ft_m = 502,4582990603992`, `e06e7e7` publicava `582,3411181885572`.
- Que o par `582,521767 / 646,660942` é posterior (`0a6136f`) e está registrado
  corretamente em OUTRO lugar do mesmo documento: entrada v5.7, linha 981, e a
  cadeia `old→new` das linhas 1433-1434.
- A causa: comparou-se um valor da era v5.5 contra um pós-ciclo-13.
- A lição, que é a gêmea da do ciclo 14: **lá se afirmou uma correção que não
  existia; aqui, um defeito que não existia. As duas se curam com a mesma
  disciplina — quem afirma sobre o histórico abre o histórico.**

- [ ] **Passo 2: registre os defeitos REAIS do schema doc como resolvidos no ciclo 15**

No lugar da segunda manifestação falsa, os quatro que existem de fato, com os
números da spec §3: `:1236` (`cg_limit_fwd_pct_mac`, 2,9%), `:1424`
(`rc_sl_ms`, 44,5%), `:1429` (`vy_kmh`, 12,8%) e `:1601-1603` (o bloco de
`robustness`, que errava os dois números E o nome do cenário). Marque o item 13
como **RESOLVIDO ciclo 15**.

- [ ] **Passo 3: registre o achado dos pins que nunca bateram**

Item novo, **#24**, com a medição completa da spec §7.4: a tabela de histórico
mostrando `n_gust_vc` imóvel em 3,572607 desde `8f92c55` e `va_kmh` mudando de
242,618735 para 242,692244 no ciclo 13, contra pins `3.59` e `242.633` que não
correspondem a nenhum estado que existiu. Nomeie a classe: **pin estimado**, a
terceira variante da doença do #13 — pior que a original, porque um pin
envelhecido ao menos testemunha um estado que existiu, e um pin estimado não
testemunha nada enquanto ocupa o lugar de quem testemunharia. Marque como
**RESOLVIDO ciclo 15** (os dois pins foram corrigidos na Task 2).

- [ ] **Passo 4: registre a lacuna residual do cadeado**

Item novo, **#25**, copiando a spec §9 item 1: um literal fora de linha de
`assert` e com ≤3 casas decimais escapa das duas regras de cobrança. A tabela de
tuplas de `generic_engine.rs:1735-1742` é exatamente essa forma e só é coberta
hoje porque aqueles oito literais têm ≥4 casas. **Um pin novo escrito como tupla
com poucas casas passaria.** Registrado como lacuna CONHECIDA e DECLARADA, sem
correção — fechá-la exigiria análise semântica de fluxo.

- [ ] **Passo 5: registre a legibilidade de `:1431-1432`**

Item novo, **#26**: a célula cita `climb_gradient_pct` 12,451842 (hoje
7,913277) dentro de narrativa rotulada "ciclo 11". **Não é defeito** — não
reivindica atualidade, e por isso o marcador não se aplica. Mas quem ler a
célula isolada conclui que 12,451842% é o valor vigente. Item de legibilidade,
não de correção.

- [ ] **Passo 6: rode o portão e commite**

```
bash scripts/verifica-ciclo.sh
git add docs/backlog.md
git commit -F <arquivo-de-mensagem>
```

Esperado: **Status geral: APROVADO**.

---

## Verificação de fecho do ciclo

Rodar depois da Task 4, antes da revisão final de branch:

1. `bash scripts/verifica-ciclo.sh` → **Status geral: APROVADO**
2. `git diff b8827e8 -- aircraft_spec.json` → **vazio**
3. `git diff b8827e8 -- src/` → **vazio**
4. `grep -n "SCHEMA_VERSION" src/models/specs.rs` → ainda **5.7**
5. `cargo test --test pins_vs_json` → **18 testes** (11 da Task 1, 4 da Task 2,
   3 da Task 3), e a suíte total ≥ **537**
6. `git diff b8827e8 -- tests/` contém apenas: comentários `PIN:`, o arquivo
   novo `tests/pins_vs_json.rs`, e **exatamente dois** literais alterados
   (`vn_diagram.rs:93` e `:105`) com seus `old→new`
7. Nenhuma tolerância alterada em lugar nenhum
