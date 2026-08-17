//! Ciclo 16, Task 2 — identidade estável de check (spec §5.2).
//!
//! Este arquivo tem UM teste: `todo_sitio_de_violacao_tem_id_e_os_ids_sao_
//! globalmente_unicos`, que varre o FONTE (não observa uma corrida) e prova
//! mecanicamente que (a) cada sítio `violations.push` tem exatamente um
//! `id:`, e (b) os ids são globalmente únicos entre `Violacao` e `Portao`.
//!
//! Vive num arquivo de integração separado (não em `src/validation/
//! constraint_checker.rs::mod tests`, onde os outros três testes do Passo 1
//! do plano vivem) porque precisa reusar `common::mascara_arquivo` — a
//! MESMA função que `tests/pins_vs_json.rs` usa desde o ciclo 15 — e um
//! teste unitário da lib (`#[cfg(test)]` dentro de `src/`) não enxerga o
//! crate de testes de integração: são binários separados, sem módulo em
//! comum. `tests/common/mod.rs` é o ponto de encontro dos dois.
//!
//! Por que a unicidade é provada VARRENDO O FONTE, não observando uma
//! corrida: o fixture do baseline real nunca dispara `#9a` (envelope de CG
//! vazio), nem as três condições do `#10` juntas, nem as duas do `#17`
//! juntas — uma implementação com ids colididos passaria verde na suíte
//! inteira e só quebraria com uma config futura. Técnica idêntica à de
//! `tests/pins_vs_json.rs` (ciclo 15): o fonte é a fonte da verdade.

mod common;
use common::mascara_arquivo;

fn caminho(rel: &str) -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(rel)
}

/// Conta ocorrências de `violations.push` FORA de comentário/string, usando
/// a máscara. Há 26 ocorrências textuais em `constraint_checker.rs`, uma
/// delas dentro de um comentário (linha ~455, explicando a checagem #19) —
/// a máscara apaga essa, deixando os 25 sítios reais.
fn conta_pushes_fora_de_comentario(conteudo: &str) -> usize {
    mascara_arquivo(conteudo)
        .iter()
        .map(|linha| linha.matches("violations.push").count())
        .sum()
}

/// Extrai os literais `id: "..."` de um arquivo-fonte Rust.
///
/// Usa a MÁSCARA para achar a POSIÇÃO de cada declaração `id:` seguida de
/// aspas EM CÓDIGO REAL (nunca dentro de comentário ou doc-comment — a
/// máscara já apagou essas regiões), e o conteúdo ORIGINAL da linha para
/// recuperar o VALOR do literal — a máscara apaga justamente o conteúdo de
/// toda string, inclusive a que estamos tentando ler.
///
/// O padrão de busca é `id:` seguido (com espaços opcionais) de `"` —
/// dois-pontos SEGUIDO DE ASPAS, checando fronteira de identificador à
/// esquerda (`i == 0` ou o char anterior não é alfanumérico nem `_`). Isso é
/// o que impede `id:` de casar com a declaração do campo (`pub id: String,`
/// — não tem aspas logo depois) e com identificadores que TERMINAM em "id"
/// (`flip_id: "x"` teria `_` antes de "id:", reprovando a fronteira).
///
/// TAMBÉM aceita `id: format!("...", ...)` — captura o TEMPLATE (com `{}`),
/// não o valor resolvido. É o caso dos dois ids não-literais deste arquivo:
/// `envelope_cg::{}` (contrato da spec §5.2, checagem #9 por cenário) e
/// `robustez::{}::{}` (checagem #19 por flip — decisão desta task, análoga
/// à de envelope_cg: um id FIXO colidiria sempre que duas violações do
/// MESMO sítio de loop ocorressem na mesma corrida). Sem isso, estes DOIS
/// sítios ficariam de fora da contagem e `ids_cc.len() != sitios` sempre —
/// o teste abaixo NUNCA fecharia por um motivo estrutural, não por um bug
/// real. O TEMPLATE capturado (com `{}` dentro) nunca colide com nenhum id
/// literal de verdade, então a unicidade global continua válida sobre ele.
fn literais_de_id(conteudo: &str) -> Vec<String> {
    let mut out = Vec::new();
    for (orig, masc) in conteudo.lines().zip(mascara_arquivo(conteudo)) {
        let oc: Vec<char> = orig.chars().collect();
        let mc: Vec<char> = masc.chars().collect();
        let n = mc.len();
        let mut i = 0usize;
        while i + 2 < n {
            let fronteira = i == 0 || !(mc[i - 1].is_alphanumeric() || mc[i - 1] == '_');
            if fronteira && mc[i] == 'i' && mc[i + 1] == 'd' && mc[i + 2] == ':' {
                let mut j = i + 3;
                while j < n && mc[j] == ' ' { j += 1; }
                // `id: format!("template"...)` — pula o `format!(` antes de
                // procurar a aspa.
                if j + 6 < n && mc[j..j + 7].iter().collect::<String>() == "format!" {
                    j += 7;
                    while j < n && mc[j] == ' ' { j += 1; }
                    if j < n && mc[j] == '(' {
                        j += 1;
                        while j < n && mc[j] == ' ' { j += 1; }
                    }
                }
                if j < n && mc[j] == '"' {
                    let ini = j + 1;
                    let mut k = ini;
                    while k < n && mc[k] != '"' { k += 1; }
                    if k < n {
                        out.push(oc[ini..k].iter().collect::<String>());
                        i = k + 1;
                        continue;
                    }
                }
            }
            i += 1;
        }
    }
    out
}

/// Ciclo 16, Task 2 — unicidade provada VARRENDO O FONTE, não observando
/// uma corrida.
#[test]
fn todo_sitio_de_violacao_tem_id_e_os_ids_sao_globalmente_unicos() {
    let cc = std::fs::read_to_string(caminho("src/validation/constraint_checker.rs")).unwrap();
    let pl = std::fs::read_to_string(caminho("src/pipeline.rs")).unwrap();

    let sitios = conta_pushes_fora_de_comentario(&cc);
    let ids_cc = literais_de_id(&cc);
    assert_eq!(ids_cc.len(), sitios,
        "cada sítio `violations.push` precisa de EXATAMENTE um `id:` — {sitios} sítios, \
         {} ids encontrados: {ids_cc:?}", ids_cc.len());

    let ids_pl = literais_de_id(&pl);
    assert_eq!(ids_pl.len(), 9, "os 9 portões precisam de id: {ids_pl:?}");
    assert!(ids_pl.iter().all(|i| i.starts_with("portao_")),
        "id de portão tem que ter prefixo `portao_` — é o que impede colisão com o \
         namespace das violações por construção: {ids_pl:?}");

    let mut vistos = std::collections::HashSet::new();
    for id in ids_cc.iter().chain(ids_pl.iter()) {
        assert!(vistos.insert(id.clone()),
            "id duplicado entre violações e portões: '{id}' — a varredura de banda \
             publicaria o veredito de um check sobre o outro, em silêncio");
    }
    assert_eq!(vistos.len(), 25 + 9,
        "esperava exatamente 25 ids de violação + 9 de portão, sem colisão nenhuma");
}
