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
use crate::pipeline::Resultado;

/// Estado de veredito de um check sob incerteza (ciclo 16, spec §5.5).
///
/// Usado em dois níveis: como resultado FINAL de um check (pode ser
/// `Indeterminado`) e como leitura de um único ponto do domínio
/// (`veredito_lo`/`veredito_hi`), onde `Indeterminado` tem um segundo
/// significado — "aquele extremo não convergiu", nunca "pertinência
/// ambígua num ponto só" (um único ponto é sempre PASSA ou FALHA quando o
/// pipeline converge).
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
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

fn roda_com_fom(
    cfg: &AircraftConfig,
    engine: &EngineSpec,
    req: &Requirements,
    fom_static: f64,
    fom_design: Option<f64>,
) -> Ponto {
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
    match crate::pipeline::executa(&cfg2, engine, req) {
        Ok(res) => Ponto::Ok(res.report.violations.iter().map(|v| v.id.clone()).collect()),
        Err(e) => Ponto::Falha(e.to_string()),
    }
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

    let ponto_lo = roda_com_fom(cfg, engine, req, banda.lo, None);
    let ponto_hi = roda_com_fom(cfg, engine, req, banda.hi, None);
    // Teto de quantidade de movimento: as DUAS âncoras em 1,0 — ver
    // comentário em `roda_com_fom`.
    let ponto_teto = roda_com_fom(cfg, engine, req, 1.0, Some(1.0));

    let ids_nominal: HashSet<String> =
        nominal.report.violations.iter().map(|v| v.id.clone()).collect();

    // L ∪ N ∪ H ∪ T (spec §5.4, passo 5). `BTreeSet` só para ordem
    // determinística de saída — não é requisito de corretude, é
    // reprodutibilidade de teste/diff.
    let mut todos: BTreeSet<String> = ids_nominal.iter().cloned().collect();
    if let Ponto::Ok(ids) = &ponto_lo { todos.extend(ids.iter().cloned()); }
    if let Ponto::Ok(ids) = &ponto_hi { todos.extend(ids.iter().cloned()); }
    if let Ponto::Ok(ids) = &ponto_teto { todos.extend(ids.iter().cloned()); }

    let mut checks = Vec::with_capacity(todos.len());

    for id in todos {
        let em_nominal = ids_nominal.contains(&id);
        let veredito_nominal = veredito_de(em_nominal);

        // Falha ao convergir num extremo é PUBLICADA, não engolida
        // (precedente: `RobustnessSpec::mtow_masstotal_kg`). Não monta
        // `avalia`/bisseção quando falta um dos dois pontos: bissecar exige
        // uma função `viola` avaliável em todo o domínio, e um extremo que
        // não converge não garante isso para os pontos internos.
        let (veredito, veredito_lo, veredito_hi, breakeven, motivo_extremo) =
            match (&ponto_lo, &ponto_hi) {
                (Ponto::Ok(ids_lo), Ponto::Ok(ids_hi)) => {
                    let em_lo = ids_lo.contains(&id);
                    let em_hi = ids_hi.contains(&id);
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
                        res.report.violations.iter().any(|v| v.id == id_bissecao)
                    };
                    let (veredito, breakeven, motivo) =
                        avalia(banda.lo, banda.hi, em_lo, em_nominal, em_hi, &mut viola);
                    (veredito, veredito_de(em_lo), veredito_de(em_hi), breakeven, motivo)
                }
                (Ponto::Falha(msg), ponto_hi) => (
                    Veredito::Indeterminado,
                    Veredito::Indeterminado,
                    veredito_de(ponto_hi.contains(&id)),
                    None,
                    Some(format!(
                        "extremo inferior da banda (fom_static={:.6}) não convergiu: {msg} \
                         — precedente: RobustnessSpec::mtow_masstotal_kg",
                        banda.lo
                    )),
                ),
                (ponto_lo, Ponto::Falha(msg)) => (
                    Veredito::Indeterminado,
                    veredito_de(ponto_lo.contains(&id)),
                    Veredito::Indeterminado,
                    None,
                    Some(format!(
                        "extremo superior da banda (fom_static={:.6}) não convergiu: {msg} \
                         — precedente: RobustnessSpec::mtow_masstotal_kg",
                        banda.hi
                    )),
                ),
            };

        // `alcance_de_helice`: presente no teto ⇒ nenhuma hélice conserta.
        // Se o teto não convergiu, marca conservadoramente `false` (não
        // provado que a física alcança) e publica o motivo — nunca afirma
        // alcance sem ter medido.
        let (alcance_de_helice, motivo_teto) = match &ponto_teto {
            Ponto::Ok(ids_teto) => (!ids_teto.contains(&id), None),
            Ponto::Falha(msg) => (
                false,
                Some(format!(
                    "teto de quantidade de movimento não convergiu: {msg} — \
                     alcance_de_helice marcado conservadoramente como falso, \
                     não provado que a física alcança"
                )),
            ),
        };

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
        banda,
        checks,
    }
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
}
