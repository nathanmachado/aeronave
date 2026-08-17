//! Suporte COMPARTILHADO entre binários de teste de integração.
//!
//! `tests/*.rs` são compilados como crates SEPARADOS — não há `mod`
//! implícito entre eles. `mod common;` (convenção do Cargo: um arquivo
//! chamado `mod.rs` dentro de `tests/common/` NÃO vira um alvo de teste
//! próprio) é o jeito idiomático de compartilhar código entre eles sem
//! promovê-lo à biblioteca.
//!
//! Ciclo 16 (Task 2): `mascara_arquivo` foi MOVIDA para cá a partir de
//! `tests/pins_vs_json.rs` (onde nasceu no ciclo 15, backlog #13) porque
//! `tests/identidade_de_checks.rs` (novo, Task 2) precisa da MESMA função —
//! não de uma reimplementação, que poderia divergir e reintroduzir os dois
//! bugs que a revisão de plano do ciclo 15 pegou antes de virarem código
//! (ver o `assert!(!em_string, …)` abaixo). `tests/pins_vs_json.rs` passou a
//! importar daqui (`mod common; use common::mascara_arquivo;`) — mesmo
//! comportamento, fonte única.

/// Mascara um arquivo INTEIRO, devolvendo uma linha mascarada por linha de
/// entrada, de mesmo comprimento em caracteres, com conteúdo de string e
/// comentário `//` substituídos por espaço.
///
/// Por arquivo, e não por linha, porque strings deste repositório atravessam
/// linhas: `gear_tipback.rs:787-789` corta a mensagem de erro com `\` e
/// continua na linha seguinte. Uma máscara sem memória trataria essa
/// continuação como código e colheria o `8.7855` do TEXTO como se fosse um
/// literal — um pin fantasma, que não existe em lugar nenhum.
pub fn mascara_arquivo(conteudo: &str) -> Vec<String> {
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
