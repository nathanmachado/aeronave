//! A varredura da banda de incerteza de `propeller.fom_static` — ciclo 16,
//! spec §5.4.
//!
//! Re-executa `pipeline::executa` nos dois extremos da banda efetiva
//! (`PropellerCfg::banda()`, ciclo 16 Task 3) e no teto de quantidade de
//! movimento (as DUAS âncoras, `fom_static` e `fom_design`, em 1,0), pareia
//! os checks pelo `Violacao::id` estável (ciclo 16 Task 2) e classifica cada
//! um em PASSA / FALHA / INDETERMINADO. Para os indeterminados cuja
//! pertinência muda entre os extremos, bisseca o breakeven e publica o
//! BRACKET medido — nunca um ponto.
//!
//! Irmão de `validation::robustness`, que já estabelece o precedente de
//! re-executar `size_aircraft` (aqui, `pipeline::executa`) com uma cópia da
//! config perturbada (`robustness.rs:428`).

use std::collections::{BTreeSet, HashSet};

use crate::models::aircraft_config::{AircraftConfig, Banda};
use crate::models::engine::EngineSpec;
use crate::models::requirements::Requirements;
use crate::pipeline::{Portao, Resultado};
use crate::validation::constraint_checker::{ConstraintReport, Violacao};

/// Estado de veredito de um check sob incerteza (ciclo 16, spec §5.5).
///
/// Usado em dois níveis: como resultado FINAL de um check (pode ser
/// `Indeterminado`) e como leitura de um único ponto do domínio
/// (`veredito_lo`/`veredito_hi`), onde `Indeterminado` tem um segundo
/// significado — "aquele extremo não convergiu", nunca "pertinência
/// ambígua num ponto só" (um único ponto é sempre PASSA ou FALHA quando o
/// pipeline converge).
/// `#[serde(rename_all = "UPPERCASE")]` — Task 5, spec §5.7: o JSON publica
/// `"PASSA"`/`"FALHA"`/`"INDETERMINADO"`, não `"Passa"`/`"Falha"`/
/// `"Indeterminado"` (o que o derive produziria sem a anotação).
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "UPPERCASE")]
pub enum Veredito {
    Passa,
    Falha,
    Indeterminado,
}

fn veredito_de(presente: bool) -> Veredito {
    if presente { Veredito::Falha } else { Veredito::Passa }
}

/// Um dos quatro pontos avaliados pela varredura: ou o pipeline convergiu e
/// produziu o conjunto de ids violados, ou o sizing falhou e o motivo é
/// PUBLICADO — nunca engolido. Precedente exato: `RobustnessSpec::
/// mtow_masstotal_kg` documenta o `0.0` do sizing perturbado que falhou em
/// vez de descartar o flip em silêncio.
#[derive(Debug, Clone)]
enum Ponto {
    Ok(HashSet<String>),
    Falha(String),
}

impl Ponto {
    fn contains(&self, id: &str) -> bool {
        match self {
            Ponto::Ok(ids) => ids.contains(id),
            Ponto::Falha(_) => false,
        }
    }
}

/// Portões que são FUNÇÃO DETERMINÍSTICA do conjunto de violações — não são
/// checks independentes, são agregados, e publicar um agregado ao lado dos
/// próprios componentes que o compõem é a doença do #21 na direção
/// contrária (spec §5.4, ERRATUM da revisão da Task 4).
///
/// Hoje EXATAMENTE UM portão satisfaz isso: `portao_restricoes`, cujo `ok`
/// é literalmente `report.all_satisfied()` == `violations.is_empty()`
/// (`pipeline.rs`: `Portao { id: "portao_restricoes", ok:
/// report.all_satisfied(), .. }`). Isso é PROVADO, não assumido, pelo teste
/// `portao_restricoes_e_funcao_deterministica_das_violacoes`
/// (`tests/incerteza.rs`) — se um dia deixar de valer (por exemplo, se
/// `portao_restricoes` ganhar uma condição própria), aquele teste reprova
/// primeiro, antes de qualquer consequência silenciosa aqui.
///
/// Os demais portões que DUPLICAM uma `Violacao` em SIGNIFICADO
/// (`portao_rc_sl`/`rc_sl`, `portao_teto_servico`/`teto_servico`,
/// `portao_envelope_cg_todos`/`envelope_cg::*`) **permanecem** na
/// varredura — a duplicação fica visível na saída. Suprimi-los exigiria uma
/// lista de equivalências mantida à mão, e este projeto tem sete
/// ocorrências documentadas (backlog #29) de uma lista dessas envelhecendo
/// errada em silêncio. Duplicação visível é melhor que supressão frágil.
const PORTOES_AGREGADOS: &[&str] = &["portao_restricoes"];

/// `ids(ponto) = { v.id para v em report.violations } ∪ { p.id para p em
/// portoes, se !p.ok }`, menos `PORTOES_AGREGADOS` (spec §5.4, ERRATUM).
///
/// Antes deste conserto a varredura só lia `report.violations`, e os 9
/// portões (que ganharam id na Task 2 EXATAMENTE para entrar aqui — spec
/// §5.2: "sem id ficariam fora da varredura") nunca eram consultados.
/// Achado da revisão: `portao_v_cruzeiro`, `portao_flutter`,
/// `portao_antitombamento` e `portao_estabilidade_long` não têm NENHUMA
/// `Violacao` correspondente — eram quatro gates de aeronavegabilidade
/// inteiramente invisíveis a este módulo.
fn ids_do_ponto(report: &ConstraintReport, portoes: &[Portao]) -> HashSet<String> {
    let mut ids: HashSet<String> = report.violations.iter().map(|v| v.id.clone()).collect();
    for p in portoes {
        if PORTOES_AGREGADOS.contains(&p.id) {
            continue;
        }
        if !p.ok {
            ids.insert(p.id.to_string());
        }
    }
    ids
}

/// Roda o pipeline em `fom_static` e devolve, JUNTO com o `Ponto`, o próprio
/// `fom_static` que foi de fato escrito em `cfg2` e usado na corrida
/// (ciclo 16, CONSERTO do achado da revisão da Task 4: "fom_hi_usado é
/// decoração").
///
/// Antes deste conserto, `analisa` tinha DUAS variáveis independentes —
/// `fom_lo_usado`/`fom_hi_usado`, atribuídas de `banda.lo`/`banda.hi` — e
/// SEPARADAMENTE passava `banda.lo`/`banda.hi` como argumento desta função. A
/// revisão mutou só a CHAMADA (para `banda.hi_declarado`) mantendo a
/// ATRIBUIÇÃO (`fom_hi_usado = banda.hi`) intocada, e os 583 testes do ciclo
/// passaram: o teste que existia (`fom_hi_usado == banda.hi`) comparava duas
/// CÓPIAS estáticas, nunca o argumento que a corrida efetivamente recebeu.
///
/// Agora só existe UMA variável: quem chama esta função recebe de volta
/// exatamente o `fom_static` que ela usou, e é ESSE valor — nunca uma cópia
/// reconstruída à parte — que `analisa` publica como `fom_lo_usado`/
/// `fom_hi_usado`. Divergir deixou de ser um erro detectável por teste e
/// passou a ser impossível de escrever: não há uma segunda linha para
/// desalinhar.
fn roda_com_fom(
    cfg: &AircraftConfig,
    engine: &EngineSpec,
    req: &Requirements,
    fom_static: f64,
    fom_design: Option<f64>,
) -> (f64, Ponto) {
    let mut cfg2 = cfg.clone();
    cfg2.propeller.fom_static = fom_static;
    // O teto de quantidade de movimento usa AS DUAS âncoras em 1,0. Com
    // `fom_design` intacto, `fom_static = 1,0` sozinho produziria uma curva
    // FoM(J) DECRESCENTE em J (t=1 satura em fom_design < 1,0) — fisicamente
    // inadmissível, e é exatamente o que `PropellerCfg::banda()`/
    // `validate_aircraft_config` rejeitam em config carregada de disco.
    if let Some(fd) = fom_design {
        cfg2.propeller.fom_design = fd;
    }
    let ponto = match crate::pipeline::executa(&cfg2, engine, req) {
        Ok(res) => Ponto::Ok(ids_do_ponto(&res.report, &res.portoes)),
        Err(e) => Ponto::Falha(e.to_string()),
    };
    // `cfg2.propeller.fom_static` (não o parâmetro `fom_static`) é o valor
    // devolvido DE PROPÓSITO: é literalmente o campo lido pela chamada a
    // `pipeline::executa` acima, a MESMA leitura, não uma reconstrução.
    (cfg2.propeller.fom_static, ponto)
}

/// Um check sob a banda de incerteza (ciclo 16, spec §5.7).
#[derive(Debug, Clone)]
pub struct CheckIncerto {
    pub id: String,
    pub veredito: Veredito,
    pub veredito_lo: Veredito,
    pub veredito_nominal: Veredito,
    pub veredito_hi: Veredito,
    /// `false` = falha TAMBÉM no teto de quantidade de movimento: nenhuma
    /// hélice conserta. NÃO afirma que existe hélice real capaz quando
    /// `true` — afirma apenas que a física não proíbe (spec §9, item 5).
    pub alcance_de_helice: bool,
    /// Bracket MEDIDO do breakeven. `None` quando não há travessia única
    /// (não monotônico) ou quando um extremo não convergiu.
    pub breakeven: Option<(f64, f64)>,
    pub motivo: Option<String>,
}

/// Resultado completo da varredura (ciclo 16, spec §5.7).
#[derive(Debug, Clone)]
pub struct Incerteza {
    pub parametro: &'static str,
    pub banda: Banda,
    /// Valor de `fom_static` EFETIVAMENTE usado para o ponto inferior/
    /// superior — sempre `banda.lo`/`banda.hi` (nunca `lo_declarado`/
    /// `hi_declarado`). Exposto como dado, não como suposição, porque a
    /// proteção por conjunto-de-ids não basta sozinha: a revisão do
    /// CONSERTO 3 mediu que, no baseline de hoje, os CONJUNTOS de id
    /// violados em `banda.hi` e em `banda.hi_declarado` COINCIDEM (só os
    /// TEXTOS, com as magnitudes, diferem) — então um teste apoiado só em
    /// vereditos não distinguiria o valor certo do errado. Com o valor
    /// usado publicado aqui, a distinção fica checável diretamente.
    pub fom_lo_usado: f64,
    pub fom_hi_usado: f64,
    pub teto_avaliado: bool,
    pub checks: Vec<CheckIncerto>,
}

/// A regra é PERTINÊNCIA IDÊNTICA NOS TRÊS PONTOS, não "virou entre os
/// extremos". Um check pode violar no nominal e em nenhum extremo (não
/// monotonicidade); a regra ingênua ("virou entre lo e hi?") o daria como
/// PASSA enquanto a corrida nominal o tem na lista de violações — o modelo
/// publicaria a violação e, ao lado, a afirmação de que ela não existe.
/// Nenhuma não monotonicidade foi observada no baseline (spec §2.4), mas a
/// regra não pode depender disso.
fn classifica(em_lo: bool, em_nominal: bool, em_hi: bool) -> Veredito {
    if em_lo == em_nominal && em_nominal == em_hi {
        if em_lo { Veredito::Falha } else { Veredito::Passa }
    } else {
        Veredito::Indeterminado
    }
}

/// Bisseca `fom_static` até a largura do bracket ficar < `TOL_BREAKEVEN`, e
/// devolve o BRACKET, não o ponto médio.
///
/// Publicar um ponto com 17 dígitos e tolerância de 1e-6 seria exatamente a
/// falsa precisão que este ciclo existe para curar. O intervalo carrega a
/// própria incerteza no formato.
const TOL_BREAKEVEN: f64 = 1e-6;
const MAX_ITER_BREAKEVEN: usize = 60;

/// Pré-condição: `viola(lo) != viola(hi)`. `analisa` só chama esta função
/// nesse caso — o caso não monotônico (`viola(lo) == viola(hi)`, só o
/// nominal discordando) sai com `breakeven = None` e motivo, sem bissecar.
fn bisseca(
    lo: f64,
    hi: f64,
    viola: &mut dyn FnMut(f64) -> bool,
) -> (f64, f64) {
    let (mut a, mut b) = (lo, hi);
    let va = viola(a);
    assert_ne!(va, viola(b), "pré-condição da bisseção: os extremos têm que discordar");
    let mut i = 0;
    while b - a > TOL_BREAKEVEN && i < MAX_ITER_BREAKEVEN {
        let m = a + (b - a) / 2.0;      // não `(a+b)/2.0`: evita overflow e
                                        // preserva o bracket em f64
        if viola(m) == va { a = m; } else { b = m; }
        i += 1;
    }
    // `assert!` DE VERDADE, não `debug_assert!`: o binário que gera
    // aircraft_spec.json é compilado em release, e o Cargo.toml não tem
    // `[profile.release]`, então `debug-assertions` é false ali. Um
    // `debug_assert!` seria inerte exatamente no caminho que publica. Custa
    // uma comparação por check indeterminado (um, no baseline).
    assert_ne!(viola(a), viola(b),
        "invariante da bisseção: o bracket publicado tem que ter vereditos OPOSTOS \
         nos dois extremos — senão o breakeven publicado não testemunha travessia \
         nenhuma");
    (a, b)
}

/// Motivo padrão para o caso não monotônico — extraído para ficar idêntico
/// entre a decisão e o texto que a Task 5 vai publicar.
const MOTIVO_NAO_MONOTONICO: &str =
    "não monotônico na banda: os extremos concordam entre si, mas divergem do \
     nominal — não há travessia única para bissecar";

/// Decide veredito + (quando aplicável) o bracket do breakeven, dados só os
/// três booleanos de pertinência e uma função `viola` que decide pertinência
/// num `fom_static` arbitrário do domínio `[lo, hi]`.
///
/// Separado de `analisa` DE PROPÓSITO: fica testável sem rodar o pipeline
/// nenhuma vez — os testes deste módulo usam uma `viola` sintética (função
/// degrau); `analisa` usa uma que chama `pipeline::executa` de verdade.
fn avalia(
    lo: f64,
    hi: f64,
    em_lo: bool,
    em_nominal: bool,
    em_hi: bool,
    viola: &mut dyn FnMut(f64) -> bool,
) -> (Veredito, Option<(f64, f64)>, Option<String>) {
    let veredito = classifica(em_lo, em_nominal, em_hi);
    if veredito != Veredito::Indeterminado {
        return (veredito, None, None);
    }
    if em_lo != em_hi {
        let bracket = bisseca(lo, hi, viola);
        (veredito, Some(bracket), None)
    } else {
        // `em_lo == em_hi` mas a classificação ainda deu Indeterminado ⇒
        // só o nominal discorda dos dois extremos. Não há travessia única
        // na banda para bissecar (§5.4, passo 7).
        (veredito, None, Some(MOTIVO_NAO_MONOTONICO.to_string()))
    }
}

/// Resultado de decidir os dois extremos ANTES de tentar bissecar — separado
/// para que a decisão "um extremo não convergiu" seja pura e testável com
/// `Ponto::Falha` sintético, sem rodar o pipeline (ciclo 16, CONSERTO 2 da
/// revisão da Task 4: "`Ponto::Falha` aparece só nos `match` de `analisa` e
/// nunca em teste nenhum" — a lição da Task 3, comportamento declarado e
/// nunca observado sob teste).
#[derive(Debug, Clone, PartialEq)]
enum DecisaoExtremos {
    /// Os dois extremos convergiram — segue para `avalia()` com estes
    /// booleanos de pertinência.
    Convergiu { em_lo: bool, em_hi: bool },
    /// Um dos dois extremos (ou os dois) não convergiu. O veredito final já
    /// é `Indeterminado`, SEM bissecar — bissecar exige uma função `viola`
    /// avaliável em todo o domínio, e um extremo que não converge não
    /// garante isso para os pontos internos.
    Falhou { veredito_lo: Veredito, veredito_hi: Veredito, motivo: String },
}

/// Pura: não chama o pipeline. Recebe os `Ponto`s já resolvidos (reais ou
/// sintéticos) e a `Banda` só para citar `lo`/`hi` na mensagem do motivo.
fn decide_extremos(id: &str, banda: &Banda, ponto_lo: &Ponto, ponto_hi: &Ponto) -> DecisaoExtremos {
    match (ponto_lo, ponto_hi) {
        (Ponto::Ok(ids_lo), Ponto::Ok(ids_hi)) => DecisaoExtremos::Convergiu {
            em_lo: ids_lo.contains(id),
            em_hi: ids_hi.contains(id),
        },
        (Ponto::Falha(msg), ph) => DecisaoExtremos::Falhou {
            veredito_lo: Veredito::Indeterminado,
            veredito_hi: veredito_de(ph.contains(id)),
            motivo: format!(
                "extremo inferior da banda (fom_static={:.6}) não convergiu: {msg} \
                 — precedente: RobustnessSpec::mtow_masstotal_kg",
                banda.lo
            ),
        },
        (pl, Ponto::Falha(msg)) => DecisaoExtremos::Falhou {
            veredito_lo: veredito_de(pl.contains(id)),
            veredito_hi: Veredito::Indeterminado,
            motivo: format!(
                "extremo superior da banda (fom_static={:.6}) não convergiu: {msg} \
                 — precedente: RobustnessSpec::mtow_masstotal_kg",
                banda.hi
            ),
        },
    }
}

/// Pura: `alcance_de_helice` a partir do `Ponto` do teto. Se o teto não
/// convergiu, marca CONSERVADORAMENTE `false` (não provado que a física
/// alcança) e publica o motivo — nunca afirma alcance sem ter medido.
fn decide_teto(id: &str, ponto_teto: &Ponto) -> (bool, Option<String>) {
    match ponto_teto {
        Ponto::Ok(ids_teto) => (!ids_teto.contains(id), None),
        Ponto::Falha(msg) => (
            false,
            Some(format!(
                "teto de quantidade de movimento não convergiu: {msg} — \
                 alcance_de_helice marcado conservadoramente como falso, \
                 não provado que a física alcança"
            )),
        ),
    }
}

/// Combina `decide_extremos` + `avalia`: a peça central testável sem
/// pipeline. Só o ramo `Convergiu` chama `viola` (e só quando a
/// classificação pede travessia) — os ramos `Falhou` NUNCA chamam.
fn decide_check(
    id: &str,
    banda: &Banda,
    ponto_lo: &Ponto,
    em_nominal: bool,
    ponto_hi: &Ponto,
    viola: &mut dyn FnMut(f64) -> bool,
) -> (Veredito, Veredito, Veredito, Option<(f64, f64)>, Option<String>) {
    match decide_extremos(id, banda, ponto_lo, ponto_hi) {
        DecisaoExtremos::Falhou { veredito_lo, veredito_hi, motivo } => {
            (Veredito::Indeterminado, veredito_lo, veredito_hi, None, Some(motivo))
        }
        DecisaoExtremos::Convergiu { em_lo, em_hi } => {
            let (veredito, breakeven, motivo) =
                avalia(banda.lo, banda.hi, em_lo, em_nominal, em_hi, viola);
            (veredito, veredito_de(em_lo), veredito_de(em_hi), breakeven, motivo)
        }
    }
}

/// Roda a varredura completa da banda de `propeller.fom_static` e classifica
/// cada check afetado (ciclo 16, spec §5.4).
///
/// `nominal` é o resultado JÁ CONVERGIDO no `fom_static` de config — não é
/// recalculado aqui (o chamador já rodou o pipeline uma vez para produzir o
/// artefato; rodar de novo seria a quarta corrida redundante).
pub fn analisa(
    cfg: &AircraftConfig,
    engine: &EngineSpec,
    req: &Requirements,
    nominal: &Resultado,
) -> Incerteza {
    let banda = cfg.propeller.banda();

    // Valores EFETIVAMENTE usados, publicados em `Incerteza` — ver o
    // doc-comment do campo para o porquê (CONSERTO 3 da revisão da Task 4) e
    // o doc-comment de `roda_com_fom` para o conserto ESTRUTURAL da Task 5:
    // `fom_lo_usado`/`fom_hi_usado` vêm do RETORNO da chamada, nunca de uma
    // segunda leitura de `banda.lo`/`banda.hi` — uma fonte, não duas.
    let (fom_lo_usado, ponto_lo) = roda_com_fom(cfg, engine, req, banda.lo, None);
    let (fom_hi_usado, ponto_hi) = roda_com_fom(cfg, engine, req, banda.hi, None);
    // Teto de quantidade de movimento: as DUAS âncoras em 1,0 — ver
    // comentário em `roda_com_fom`. O `fom` devolvido aqui não é publicado
    // (não há campo `fom_teto_usado` — o teto é sempre 1,0 por definição).
    let (_fom_teto_usado, ponto_teto) = roda_com_fom(cfg, engine, req, 1.0, Some(1.0));

    let ids_nominal: HashSet<String> = ids_do_ponto(&nominal.report, &nominal.portoes);

    // L ∪ N ∪ H ∪ T (spec §5.4, passo 5 — agora incluindo os portões via
    // `ids_do_ponto`, CONSERTO 1). `BTreeSet` só para ordem determinística
    // de saída — não é requisito de corretude, é reprodutibilidade de
    // teste/diff.
    let mut todos: BTreeSet<String> = ids_nominal.iter().cloned().collect();
    if let Ponto::Ok(ids) = &ponto_lo { todos.extend(ids.iter().cloned()); }
    if let Ponto::Ok(ids) = &ponto_hi { todos.extend(ids.iter().cloned()); }
    if let Ponto::Ok(ids) = &ponto_teto { todos.extend(ids.iter().cloned()); }

    let mut checks = Vec::with_capacity(todos.len());

    for id in todos {
        let em_nominal = ids_nominal.contains(&id);
        let veredito_nominal = veredito_de(em_nominal);

        let id_bissecao = id.clone();
        let mut viola = |fom: f64| -> bool {
            let mut cfg2 = cfg.clone();
            cfg2.propeller.fom_static = fom;
            let res = crate::pipeline::executa(&cfg2, engine, req)
                .expect(
                    "bisseção: um ponto INTERNO da banda deixou de convergir \
                     mesmo com os dois extremos convergindo — fora do escopo \
                     medido neste ciclo (pipeline converge de fom_static 0,55 \
                     a 1,0), reportar como achado de primeira ordem",
                );
            ids_do_ponto(&res.report, &res.portoes).contains(&id_bissecao)
        };

        // Falha ao convergir num extremo é PUBLICADA, não engolida
        // (precedente: `RobustnessSpec::mtow_masstotal_kg`) — decidido pela
        // função PURA `decide_check` (CONSERTO 2 da revisão da Task 4), que
        // só chama `viola` no ramo em que os dois extremos convergem E a
        // classificação pede travessia.
        let (veredito, veredito_lo, veredito_hi, breakeven, motivo_extremo) =
            decide_check(&id, &banda, &ponto_lo, em_nominal, &ponto_hi, &mut viola);

        // `alcance_de_helice`: presente no teto ⇒ nenhuma hélice conserta.
        let (alcance_de_helice, motivo_teto) = decide_teto(&id, &ponto_teto);

        let mut partes: Vec<String> = Vec::new();
        if let Some(m) = motivo_extremo { partes.push(m); }
        if let Some(m) = motivo_teto { partes.push(m); }
        let motivo = if partes.is_empty() { None } else { Some(partes.join(" ; ")) };

        checks.push(CheckIncerto {
            id,
            veredito,
            veredito_lo,
            veredito_nominal,
            veredito_hi,
            alcance_de_helice,
            breakeven,
            motivo,
        });
    }

    Incerteza {
        parametro: "propeller.fom_static",
        teto_avaliado: matches!(ponto_teto, Ponto::Ok(_)),
        fom_lo_usado,
        fom_hi_usado,
        banda,
        checks,
    }
}

// ─── Task 5 — publicação ───────────────────────────────────────────────────
// `analisa` (acima) só CLASSIFICA. As duas funções abaixo são a fronteira
// entre a classificação e o que o usuário lê — o veredito global de três
// estados (spec §5.5) e a reescrita do texto de violação (spec §5.6 + o
// ERRATUM que cobre o caso indeterminado AUSENTE do nominal).

/// Terceiro estado do veredito global (ciclo 16, spec §5.5).
///
/// ```text
/// FAIL           se existe QUALQUER check com falha DETERMINADA
/// INDETERMINADO  senão, se existe qualquer check indeterminado
/// PASS           senão
/// ```
///
/// Falha determinada DOMINA indeterminação — de propósito (spec §5.6, razão
/// 1): INDETERMINADO tem que ser lido como "o modelo não sabe", nunca como
/// "está tudo bem". `inc.checks` já é a união dos quatro pontos avaliados
/// (nominal ∪ lo ∪ hi ∪ teto, ver `analisa`), incluindo os 9 portões — logo
/// esta função não precisa (e não deve) reconsultar `report.violations`
/// separadamente: um check que nunca aparece em `checks` nunca violou em
/// ponto nenhum, e portanto é PASSA por definição.
pub fn veredito_global(inc: &Incerteza) -> &'static str {
    if inc.checks.iter().any(|c| c.veredito == Veredito::Falha) {
        "FAIL"
    } else if inc.checks.iter().any(|c| c.veredito == Veredito::Indeterminado) {
        "INDETERMINADO"
    } else {
        "PASS"
    }
}

/// Descreve a variação de um check indeterminado para o texto publicado —
/// ou o bracket do breakeven (caso normal, travessia única), ou o `motivo`
/// que `analisa` já produziu quando não há bracket (não monotônico, ou um
/// extremo que não convergiu). Extraída de `texto_indeterminado_*` para as
/// duas variantes do ERRATUM da spec §5.6 citarem a MESMA frase.
fn cita_variacao(banda: &Banda, c: &CheckIncerto) -> String {
    match c.breakeven {
        Some((a, b)) => {
            let delta_pct = (a - banda.nominal) / banda.nominal * 100.0;
            format!(
                "breakeven em [{a:.6}–{b:.6}], {delta_pct:+.1}% sobre o nominal {:.3}",
                banda.nominal
            )
        }
        None => {
            let motivo = c.motivo.as_deref()
                .unwrap_or("sem bracket medido — motivo não registrado, achado de primeira ordem");
            format!("sem breakeven único ({motivo})")
        }
    }
}

/// Caso 1 do ERRATUM (spec §5.6): indeterminado PRESENTE no nominal.
/// Reescreve o texto ORIGINAL — não o substitui por um texto genérico — com
/// prefixo `INDETERMINADO — ` e a variação medida.
fn texto_indeterminado_presente(original: &str, banda: &Banda, c: &CheckIncerto) -> String {
    format!(
        "INDETERMINADO — {original}. O veredito VIRA dentro da banda declarada de \
         propeller.fom_static [{:.6}–{:.6}]: {}. O modelo NÃO sustenta este veredito.",
        banda.lo, banda.hi, cita_variacao(banda, c),
    )
}

/// Caso 2 do ERRATUM (spec §5.6) — o que a primeira versão da seção não
/// cobria: indeterminado AUSENTE do nominal. Não há texto original para
/// reescrever (o check nem aparece em `report.violations` no nominal), então
/// esta função MONTA uma violação nova a partir do `id` do check — é o único
/// caso em que o silêncio favoreceria o projeto (um check que "passa" hoje e
/// vira dentro da banda declarada).
fn texto_indeterminado_ausente(id: &str, banda: &Banda, c: &CheckIncerto) -> String {
    format!(
        "INDETERMINADO — check '{id}' passa no nominal ({:.3}) mas VIRA dentro da banda \
         declarada de propeller.fom_static [{:.6}–{:.6}]: {}. O modelo NÃO sustenta este \
         veredito.",
        banda.nominal, banda.lo, banda.hi, cita_variacao(banda, c),
    )
}

/// Publica a lista final de violações (ciclo 16, spec §5.6 + ERRATUM da
/// revisão da Task 4 — os DOIS casos).
///
/// - indeterminado PRESENTE no nominal → reescreve o texto ORIGINAL daquela
///   `Violacao` (prefixo `INDETERMINADO — `); a contagem NÃO muda.
/// - indeterminado AUSENTE do nominal → INSERE uma violação nova (mesmo
///   prefixo); a contagem SOBE.
///
/// INDETERMINADO NUNCA remove violação: toda `Violacao` de `nominal` sai
/// desta função, reescrita ou intocada — nunca descartada. É por isso que a
/// função itera `nominal` primeiro (preservando cada entrada, uma a uma) e só
/// DEPOIS considera inserções.
pub fn publica_violacoes(nominal: &[Violacao], inc: &Incerteza) -> Vec<String> {
    use std::collections::HashMap;

    let indeterminados: HashMap<&str, &CheckIncerto> = inc.checks.iter()
        .filter(|c| c.veredito == Veredito::Indeterminado)
        .map(|c| (c.id.as_str(), c))
        .collect();

    let mut saida: Vec<String> = Vec::with_capacity(nominal.len());
    for v in nominal {
        match indeterminados.get(v.id.as_str()) {
            Some(c) => saida.push(texto_indeterminado_presente(&v.texto, &inc.banda, c)),
            None => saida.push(v.texto.clone()),
        }
    }

    let ids_nominais: HashSet<&str> = nominal.iter().map(|v| v.id.as_str()).collect();
    // Ordem determinística: `inc.checks` já vem de um `BTreeSet` em `analisa`
    // (ordem alfabética de id) — não itero `indeterminados` (HashMap, ordem
    // não determinística) para as inserções.
    for c in inc.checks.iter().filter(|c| c.veredito == Veredito::Indeterminado) {
        if !ids_nominais.contains(c.id.as_str()) {
            saida.push(texto_indeterminado_ausente(&c.id, &inc.banda, c));
        }
    }

    saida
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Passo 2: a classificação pura, sem pipeline ─────────────────────

    #[test]
    fn presente_nos_tres_pontos_sai_falha_determinada() {
        assert_eq!(classifica(true, true, true), Veredito::Falha);
    }

    #[test]
    fn ausente_nos_tres_pontos_sai_passa() {
        assert_eq!(classifica(false, false, false), Veredito::Passa);
    }

    /// A regra é PERTINÊNCIA IDÊNTICA NOS TRÊS PONTOS, não "virou entre os
    /// extremos". Um check pode violar no nominal e em nenhum extremo — a
    /// regra ingênua o daria como PASSA enquanto a corrida nominal o tem na
    /// lista de violações.
    #[test]
    fn nao_monotonico_sai_indeterminado_sem_breakeven() {
        assert_eq!(classifica(false, true, false), Veredito::Indeterminado);

        // Nível de `avalia`: o bracket tem que sair `None`, o motivo tem
        // que citar "não monotônico", e a função `viola` NUNCA pode ser
        // chamada — bissecar aqui não testemunharia travessia nenhuma
        // porque os extremos concordam entre si.
        let mut viola = |_: f64| -> bool {
            panic!("não deveria bissecar no caso não monotônico — os extremos concordam");
        };
        let (veredito, breakeven, motivo) = avalia(0.675, 0.816, false, true, false, &mut viola);
        assert_eq!(veredito, Veredito::Indeterminado);
        assert_eq!(breakeven, None);
        let motivo = motivo.expect("caso não monotônico tem que publicar motivo");
        assert!(motivo.contains("não monotônico"),
            "motivo tem que citar a não monotonicidade — obtido: {motivo}");
    }

    #[test]
    fn presente_so_num_extremo_sai_indeterminado_com_breakeven() {
        assert_eq!(classifica(true, true, false), Veredito::Indeterminado);

        // `viola` sintética: função degrau que viola abaixo de 0.79 —
        // simula um check que deixa de violar conforme `fom_static` sobe,
        // igual ao gradiente CS 23.65 medido na spec §2.3.
        let mut viola = |fom: f64| -> bool { fom < 0.79 };
        let (veredito, breakeven, motivo) =
            avalia(0.70, 0.82, true, true, false, &mut viola);
        assert_eq!(veredito, Veredito::Indeterminado);
        assert_eq!(motivo, None, "caso com travessia não deveria ter motivo de não monotonicidade");
        let (a, b) = breakeven.expect("caso com travessia tem que produzir bracket");
        assert!(a < b, "bracket tem que ser (lo, hi) com lo < hi — obtido ({a}, {b})");
        assert!(b - a < TOL_BREAKEVEN, "bracket tem que ser mais estreito que a tolerância");
        assert!(viola(a), "no lado esquerdo do bracket publicado o check TEM que violar");
        assert!(!viola(b), "no lado direito do bracket publicado o check NÃO pode violar");
        assert!((a - 0.79).abs() < 1e-4 && (b - 0.79).abs() < 1e-4,
            "bracket tem que estar em torno do degrau sintético (0.79) — obtido ({a}, {b})");
    }

    /// A bisseção reprova ALTO se os extremos não discordarem — prova que o
    /// `assert!` é real (não `debug_assert!`, inerte em release).
    #[test]
    #[should_panic(expected = "pré-condição da bisseção")]
    fn bisseca_reprova_se_extremos_concordam() {
        let mut viola = |_: f64| -> bool { true }; // sempre viola — não discorda
        let _ = bisseca(0.0, 1.0, &mut viola);
    }

    // ── CONSERTO 1 (revisão da Task 4): portões entram na varredura ────
    // A §5.2 deu id aos 9 portões EXATAMENTE para isto ("sem id ficariam
    // fora da varredura"); a §5.4 escrevia o algoritmo só em termos de
    // `report.violations`, contradizendo a §5.2, e a Task 4 seguiu a §5.4.
    // Estes testes cobrem `ids_do_ponto` isoladamente (sem pipeline); o
    // teste que prova que a UNIÃO chega ao baseline real está em
    // `tests/incerteza.rs`.

    #[test]
    fn ids_do_ponto_inclui_portoes_reprovados_e_ignora_os_aprovados() {
        let report = ConstraintReport { violations: vec![], warnings: vec![] };
        let portoes = vec![
            Portao { id: "portao_flutter", ok: false, rotulo: "x".to_string() },
            Portao { id: "portao_v_cruzeiro", ok: true, rotulo: "y".to_string() },
        ];
        let ids = ids_do_ponto(&report, &portoes);
        assert!(ids.contains("portao_flutter"),
            "portão REPROVADO tem que entrar no conjunto — obtido: {ids:?}");
        assert!(!ids.contains("portao_v_cruzeiro"),
            "portão APROVADO não entra no conjunto — obtido: {ids:?}");
    }

    /// `portao_restricoes` é excluído POR REGRA (é função determinística do
    /// conjunto de violações), não por lista escolhida a dedo — a exclusão
    /// vive em `PORTOES_AGREGADOS`, uma constante nomeada e documentada.
    #[test]
    fn ids_do_ponto_exclui_portao_restricoes_por_regra() {
        let report = ConstraintReport {
            violations: vec![Violacao { id: "x".to_string(), texto: "x".to_string() }],
            warnings: vec![],
        };
        let portoes = vec![Portao {
            id: "portao_restricoes", ok: false, rotulo: "agregado".to_string(),
        }];
        let ids = ids_do_ponto(&report, &portoes);
        assert!(ids.contains("x"), "a violação em si continua entrando");
        assert!(!ids.contains("portao_restricoes"),
            "portao_restricoes é agregado — não pode entrar como check independente, \
             senão publicaria o agregado ao lado dos próprios componentes (doença do \
             #21 na direção contrária)");
    }

    /// Portões que DUPLICAM uma violação em SIGNIFICADO (mas não em id)
    /// permanecem — a duplicação fica visível, em vez de suprimida por uma
    /// lista de equivalências mantida à mão.
    #[test]
    fn ids_do_ponto_mantem_portoes_que_duplicam_violacao_em_significado() {
        let report = ConstraintReport {
            violations: vec![Violacao {
                id: "envelope_cg::Solo (piloto)".to_string(),
                texto: "x".to_string(),
            }],
            warnings: vec![],
        };
        let portoes = vec![Portao {
            id: "portao_envelope_cg_todos", ok: false, rotulo: "y".to_string(),
        }];
        let ids = ids_do_ponto(&report, &portoes);
        assert!(ids.contains("envelope_cg::Solo (piloto)"));
        assert!(ids.contains("portao_envelope_cg_todos"),
            "o portão duplicado EM SIGNIFICADO continua entrando com seu próprio id — \
             obtido: {ids:?}");
        assert_eq!(ids.len(), 2, "os dois ids ficam visíveis, sem supressão");
    }

    // ── CONSERTO 2 (revisão da Task 4): falha de convergência, pura ────
    // `Ponto::Falha` só aparecia nos `match` de `analisa` e nunca em teste
    // nenhum. `decide_extremos`/`decide_teto`/`decide_check` são puras —
    // testáveis com `Ponto::Falha` sintético, sem quebrar o sizing de
    // verdade.

    fn banda_teste() -> Banda {
        Banda {
            nominal: 0.75,
            lo: 0.675,
            hi: 0.816,
            lo_declarado: 0.675,
            hi_declarado: 0.825,
            truncada: true,
            motivo_truncagem: Some("teste".to_string()),
        }
    }

    /// (i) falha no extremo INFERIOR → Indeterminado com motivo citando
    /// "extremo inferior" e a mensagem do erro; o extremo superior (que
    /// convergiu) mantém seu veredito pontual de verdade.
    #[test]
    fn decide_extremos_com_lo_falho_da_indeterminado_sem_bissecar() {
        let banda = banda_teste();
        let ponto_lo = Ponto::Falha("sizing não convergiu".to_string());
        let mut ids_hi = HashSet::new();
        ids_hi.insert("algum_id".to_string());
        let ponto_hi = Ponto::Ok(ids_hi);

        match decide_extremos("algum_id", &banda, &ponto_lo, &ponto_hi) {
            DecisaoExtremos::Falhou { veredito_lo, veredito_hi, motivo } => {
                assert_eq!(veredito_lo, Veredito::Indeterminado);
                assert_eq!(veredito_hi, Veredito::Falha, "hi convergiu e contém o id");
                assert!(motivo.contains("extremo inferior"), "motivo: {motivo}");
                assert!(motivo.contains("sizing não convergiu"), "motivo: {motivo}");
            }
            other => panic!("esperava DecisaoExtremos::Falhou, obtido {other:?}"),
        }
    }

    /// (i) falha no extremo SUPERIOR — espelho do teste acima.
    #[test]
    fn decide_extremos_com_hi_falho_da_indeterminado_sem_bissecar() {
        let banda = banda_teste();
        let ponto_lo = Ponto::Ok(HashSet::new());
        let ponto_hi = Ponto::Falha("MTOW não convergiu".to_string());

        match decide_extremos("algum_id", &banda, &ponto_lo, &ponto_hi) {
            DecisaoExtremos::Falhou { veredito_lo, veredito_hi, motivo } => {
                assert_eq!(veredito_lo, Veredito::Passa, "lo convergiu e não contém o id");
                assert_eq!(veredito_hi, Veredito::Indeterminado);
                assert!(motivo.contains("extremo superior"), "motivo: {motivo}");
                assert!(motivo.contains("MTOW não convergiu"), "motivo: {motivo}");
            }
            other => panic!("esperava DecisaoExtremos::Falhou, obtido {other:?}"),
        }
    }

    /// (iii) confirma que a bisseção NÃO é chamada quando um extremo
    /// falhou — a `viola` sintética PANICA se for invocada.
    #[test]
    fn decide_check_nao_bisseca_quando_extremo_falha() {
        let banda = banda_teste();
        let ponto_lo = Ponto::Falha("sizing não convergiu".to_string());
        let ponto_hi = Ponto::Ok(HashSet::new());
        let mut viola = |_: f64| -> bool {
            panic!("decide_check não deveria chamar viola quando um extremo falhou");
        };
        let (veredito, veredito_lo, veredito_hi, breakeven, motivo) =
            decide_check("id", &banda, &ponto_lo, true, &ponto_hi, &mut viola);
        assert_eq!(veredito, Veredito::Indeterminado);
        assert_eq!(veredito_lo, Veredito::Indeterminado);
        assert_eq!(veredito_hi, Veredito::Passa);
        assert_eq!(breakeven, None);
        assert!(motivo.unwrap().contains("extremo inferior"));
    }

    /// (ii) teto falhando → `alcance_de_helice = false` com motivo.
    #[test]
    fn decide_teto_falho_marca_alcance_falso_com_motivo() {
        let ponto_teto = Ponto::Falha("sizing do teto não convergiu".to_string());
        let (alcance, motivo) = decide_teto("id", &ponto_teto);
        assert!(!alcance, "sem prova de alcance, marca conservadoramente falso");
        let motivo = motivo.expect("teto falho tem que publicar motivo");
        assert!(motivo.contains("teto"), "motivo: {motivo}");
        assert!(motivo.contains("sizing do teto não convergiu"), "motivo: {motivo}");
    }

    #[test]
    fn decide_teto_ok_deriva_alcance_da_pertinencia() {
        let mut ids = HashSet::new();
        ids.insert("id_presente".to_string());
        let ponto_teto = Ponto::Ok(ids);
        assert_eq!(decide_teto("id_presente", &ponto_teto), (false, None));
        assert_eq!(decide_teto("id_ausente", &ponto_teto), (true, None));
    }

    // ── Task 5, Passo 2: o terceiro estado do veredito global ─────────────
    // Puro — `veredito_global` só olha `inc.checks`, então os testes abaixo
    // constroem `Incerteza` sintética, sem rodar o pipeline nenhuma vez.

    fn check_teste(id: &str, veredito: Veredito) -> CheckIncerto {
        CheckIncerto {
            id: id.to_string(),
            veredito,
            veredito_lo: veredito,
            veredito_nominal: veredito,
            veredito_hi: veredito,
            alcance_de_helice: true,
            breakeven: None,
            motivo: None,
        }
    }

    fn incerteza_teste(checks: Vec<CheckIncerto>) -> Incerteza {
        Incerteza {
            parametro: "propeller.fom_static",
            banda: banda_teste(),
            fom_lo_usado: 0.675,
            fom_hi_usado: 0.816,
            teto_avaliado: true,
            checks,
        }
    }

    #[test]
    fn veredito_global_pass_sem_checks() {
        assert_eq!(veredito_global(&incerteza_teste(vec![])), "PASS");
    }

    #[test]
    fn veredito_global_indeterminado_sem_falha_determinada() {
        let inc = incerteza_teste(vec![check_teste("a", Veredito::Passa),
                                        check_teste("b", Veredito::Indeterminado)]);
        assert_eq!(veredito_global(&inc), "INDETERMINADO",
            "só há Passa e Indeterminado — sem NENHUMA falha determinada, o veredito global \
             tem que ser INDETERMINADO, não PASS (silenciar a indeterminação seria a máquina \
             de lavar reprovação que a spec §5.6 proíbe)");
    }

    /// A regra central da spec §5.5: falha determinada DOMINA indeterminação.
    /// Config sintética com um check indeterminado E um determinado ⇒ FAIL,
    /// nunca INDETERMINADO — INDETERMINADO teria que ser lido como "pior que
    /// FAIL" (o modelo não sabe), nunca como um meio-termo que abranda FAIL.
    #[test]
    fn veredito_global_falha_determinada_domina_indeterminado() {
        let inc = incerteza_teste(vec![check_teste("indet", Veredito::Indeterminado),
                                        check_teste("det", Veredito::Falha)]);
        assert_eq!(veredito_global(&inc), "FAIL",
            "falha determinada tem que DOMINAR — mesmo com um check indeterminado presente, \
             o veredito global não pode amaciar para INDETERMINADO");
    }

    #[test]
    fn veredito_global_pass_so_com_checks_passa() {
        let inc = incerteza_teste(vec![check_teste("a", Veredito::Passa)]);
        assert_eq!(veredito_global(&inc), "PASS");
    }

    // ── Task 5, Passo 3: publicação do texto de violação ──────────────────
    // Cobre os DOIS casos do ERRATUM da spec §5.6 com config sintética —
    // "presente no nominal" (reescreve, contagem NÃO muda) e "ausente do
    // nominal" (insere, contagem SOBE) — o segundo não ocorre no baseline
    // real, mas a regra tem que valer em código, não só em spec.

    fn violacao_teste(id: &str, texto: &str) -> Violacao {
        Violacao { id: id.to_string(), texto: texto.to_string() }
    }

    fn check_indeterminado_com_breakeven(id: &str) -> CheckIncerto {
        CheckIncerto {
            id: id.to_string(),
            veredito: Veredito::Indeterminado,
            veredito_lo: Veredito::Falha,
            veredito_nominal: Veredito::Falha,
            veredito_hi: Veredito::Passa,
            alcance_de_helice: true,
            breakeven: Some((0.700_000, 0.700_001)),
            motivo: None,
        }
    }

    #[test]
    fn publica_violacoes_indeterminado_presente_no_nominal_reescreve_sem_mudar_contagem() {
        let nominal = vec![violacao_teste("x", "X falhou por Y")];
        let inc = incerteza_teste(vec![check_indeterminado_com_breakeven("x")]);

        let saida = publica_violacoes(&nominal, &inc);
        assert_eq!(saida.len(), 1, "contagem NÃO muda — só reescreve");
        assert!(saida[0].starts_with("INDETERMINADO — X falhou por Y."),
            "texto original tem que sobreviver, com o prefixo — obtido: {}", saida[0]);
        assert!(saida[0].contains("banda declarada de propeller.fom_static"), "{}", saida[0]);
        assert!(saida[0].contains("breakeven em"), "{}", saida[0]);
        assert!(saida[0].ends_with("O modelo NÃO sustenta este veredito."), "{}", saida[0]);
    }

    /// O caso que o ERRATUM da revisão da Task 4 adicionou à spec §5.6: o
    /// indeterminado AUSENTE do nominal. Não existe `Violacao` de origem —
    /// esta função tem que MONTAR uma violação nova, e a contagem SOBE.
    #[test]
    fn publica_violacoes_indeterminado_ausente_do_nominal_insere_e_sobe_a_contagem() {
        let nominal: Vec<Violacao> = vec![]; // "y" nunca apareceu no nominal
        let inc = incerteza_teste(vec![check_indeterminado_com_breakeven("y")]);

        let saida = publica_violacoes(&nominal, &inc);
        assert_eq!(saida.len(), 1, "contagem SOBE de 0 para 1 — a inserção é o único caso em \
                                     que o silêncio favoreceria o projeto");
        assert!(saida[0].starts_with("INDETERMINADO — check 'y' passa no nominal"),
            "obtido: {}", saida[0]);
        assert!(saida[0].ends_with("O modelo NÃO sustenta este veredito."), "{}", saida[0]);
    }

    /// INDETERMINADO nunca REMOVE violação: toda `Violacao` do nominal sai
    /// da função, reescrita ou intocada, mesmo quando NENHUM check é
    /// indeterminado (`Incerteza` vazia) — o caso degenerado que provaria
    /// uma remoção acidental primeiro.
    #[test]
    fn publica_violacoes_sem_indeterminados_preserva_tudo_intocado() {
        let nominal = vec![violacao_teste("a", "A"), violacao_teste("b", "B")];
        let inc = incerteza_teste(vec![]);
        let saida = publica_violacoes(&nominal, &inc);
        assert_eq!(saida, vec!["A".to_string(), "B".to_string()]);
    }

    /// Um check FALHA determinada (não indeterminado) presente no nominal
    /// não é tocado — só INDETERMINADO reescreve.
    #[test]
    fn publica_violacoes_ignora_checks_com_falha_determinada() {
        let nominal = vec![violacao_teste("a", "A original")];
        let inc = incerteza_teste(vec![check_teste("a", Veredito::Falha)]);
        let saida = publica_violacoes(&nominal, &inc);
        assert_eq!(saida, vec!["A original".to_string()]);
    }
}
