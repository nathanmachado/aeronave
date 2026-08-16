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
