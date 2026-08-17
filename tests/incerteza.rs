//! Ciclo 16, Task 4 — a varredura da banda de incerteza, testada contra o
//! baseline REAL (não uma fixture sintética): os números medidos aqui têm
//! que reproduzir a spec §2, não redescobri-los por acaso.
//!
//! Nada é publicado no JSON por esta task — a Task 5 publica. Estes testes
//! só provam que `validation::incerteza::analisa` produz o que a spec
//! registrou como medido.

use aeronave::models::config::{load_aircraft, load_engine, load_mission};
use aeronave::validation::incerteza::{analisa, Veredito};

fn carrega_baseline() -> (
    aeronave::models::aircraft_config::AircraftConfig,
    aeronave::models::engine::EngineSpec,
    aeronave::models::requirements::Requirements,
) {
    let manifest = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let cfg = load_aircraft(&manifest.join("config/aircraft/baseline_4seat.toml")).unwrap();
    let engine = load_engine(&manifest.join("config/engines/default.toml")).unwrap();
    let req = load_mission(&manifest.join("config/missions/default.toml")).unwrap();
    (cfg, engine, req)
}

/// Ciclo 16 — o baseline tem EXATAMENTE um check indeterminado, e é o
/// gradiente da CS 23.65. Medido na spec §2.4: das quatro violações, as
/// outras três são determinadas contra o domínio físico INTEIRO.
#[test]
fn baseline_tem_exatamente_um_check_indeterminado() {
    let (cfg, engine, req) = carrega_baseline();
    let nominal = aeronave::pipeline::executa(&cfg, &engine, &req)
        .expect("baseline tem que convergir");

    let inicio = std::time::Instant::now();
    let inc = analisa(&cfg, &engine, &req, &nominal);
    let duracao = inicio.elapsed();
    eprintln!("[timing] analisa() no baseline: {duracao:?}");

    let indet: Vec<_> = inc.checks.iter()
        .filter(|c| c.veredito == Veredito::Indeterminado)
        .collect();
    assert_eq!(indet.len(), 1, "esperado só o gradiente; achados: {:?}",
               indet.iter().map(|c| &c.id).collect::<Vec<_>>());
    assert_eq!(indet[0].id, "gradiente_cs2365");

    // Vereditos por ponto, medidos na spec §2.4: FALHA em 0,675 e 0,75,
    // PASSA em 0,81598.
    assert_eq!(indet[0].veredito_lo, Veredito::Falha);
    assert_eq!(indet[0].veredito_nominal, Veredito::Falha);
    assert_eq!(indet[0].veredito_hi, Veredito::Passa);

    // A física alcança: no teto de quantidade de movimento o gradiente é
    // 13,05% — bem acima do mínimo de 8,3% (spec §2.4).
    assert!(indet[0].alcance_de_helice,
        "o gradiente tem que estar dentro do alcance de propulsão — a física não proíbe");

    assert!(inc.teto_avaliado, "o teto de quantidade de movimento tem que ter convergido");

    // Banda efetiva do baseline (spec §2, Task 3): [0,675 ; 0,81597699924588796].
    assert_eq!(inc.banda.lo, 0.75 * 0.90); // PIN: NAO-PUBLICADO — banda de incerteza não publicada nesta task (Task 5)
    assert_eq!(inc.banda.hi, cfg.propeller.fom_design);
    assert!(inc.banda.truncada);
}

/// CG e robustez falham TAMBÉM no teto de quantidade de movimento: nenhuma
/// hélice conserta, e mais tração PIORA as duas (o limite dianteiro é de
/// rotação e o balanço carrega −T·z_eixo). Medido na spec §2.4.
#[test]
fn cg_e_robustez_falham_ate_no_teto() {
    let (cfg, engine, req) = carrega_baseline();
    let nominal = aeronave::pipeline::executa(&cfg, &engine, &req)
        .expect("baseline tem que convergir");
    let inc = analisa(&cfg, &engine, &req, &nominal);

    let cg = inc.checks.iter().find(|c| c.id == "envelope_cg::Solo (piloto)")
        .expect("o check de envelope de CG do cenário 'Solo (piloto)' tem que estar na lista \
                 — ele viola em todos os quatro pontos avaliados");
    assert_eq!(cg.veredito, Veredito::Falha, "CG é determinado — falha no domínio inteiro");
    assert!(!cg.alcance_de_helice,
        "CG tem que falhar também no teto — nenhuma hélice conserta esta violação");
    assert_eq!(cg.breakeven, None, "check determinado não tem breakeven");

    let robustez = inc.checks.iter().find(|c| c.id.starts_with("robustez::"))
        .expect("esperava um check de robustez ('2 pax dianteiros') na lista de checks");
    assert_eq!(robustez.veredito, Veredito::Falha, "robustez é determinada — falha no domínio inteiro");
    assert!(!robustez.alcance_de_helice,
        "robustez tem que falhar também no teto — nenhuma hélice conserta esta violação");
    assert_eq!(robustez.breakeven, None, "check determinado não tem breakeven");

    // A decolagem em grama também é determinada (spec §2.4: 618,70 m em
    // fom_static=1,0 sozinho, ainda acima dos 600 m — mas com AS DUAS
    // âncoras em 1,0 a spec mede 521,72 m, dentro do alcance).
    let decolagem = inc.checks.iter().find(|c| c.id == "decolagem_grama")
        .expect("esperava o check de decolagem em grama na lista");
    assert_eq!(decolagem.veredito, Veredito::Falha);
    assert!(decolagem.alcance_de_helice,
        "com as DUAS âncoras em 1,0 a decolagem em grama fica DENTRO do alcance de propulsão \
         (521,72 m medido na spec §2.4) — se isto falhar, o teto está usando só fom_static \
         (618,70 m, fora do alcance), que é exatamente o erro que a Task 4 Passo 3 proíbe");
}

/// O breakeven publicado é VERIFICADO re-executando o pipeline nos dois
/// lados do bracket e exigindo vereditos opostos.
///
/// Sem isto o breakeven seria um PIN ESTIMADO — a terceira variante da
/// doença do #13, encontrada no ciclo 15: um número que nunca foi o valor
/// do pipeline em commit nenhum, ocupando o lugar de quem testemunharia.
#[test]
fn breakeven_publicado_e_provado_re_rodando_o_pipeline() {
    let (cfg, engine, req) = carrega_baseline();
    let nominal = aeronave::pipeline::executa(&cfg, &engine, &req)
        .expect("baseline tem que convergir");
    let inc = analisa(&cfg, &engine, &req, &nominal);

    let c = inc.checks.iter().find(|c| c.id == "gradiente_cs2365")
        .expect("o gradiente tem que estar na lista de checks");
    let (a, b) = c.breakeven.expect("check indeterminado tem que ter bracket");
    assert!(a < b, "bracket tem que ser (lo, hi) com lo < hi");
    assert!(b - a < 1e-6, "largura do bracket tem que respeitar a tolerância publicada");

    // Medido na spec §2.3: breakeven em fom_static ≈ 0,784867742387.
    // PIN: NAO-PUBLICADO — breakeven não publicado nesta task (Task 5); valor medido na spec §2.3
    assert!((a - 0.784867742387).abs() < 1e-6 && (b - 0.784867742387).abs() < 1e-6,
        "bracket tem que estar em torno do breakeven medido na spec §2.3 — obtido ({a}, {b})");

    let viola_em = |fom: f64| -> bool {
        let mut cfg2 = cfg.clone();
        cfg2.propeller.fom_static = fom;
        let res = aeronave::pipeline::executa(&cfg2, &engine, &req)
            .expect("os dois lados do bracket já convergiram na varredura — têm que convergir de novo");
        res.report.violations.iter().any(|v| v.id == "gradiente_cs2365")
    };
    assert!(viola_em(a), "em breakeven_lo o check TEM que violar");
    assert!(!viola_em(b), "em breakeven_hi o check NÃO pode violar");
}

/// Um check que aparece em TODOS os pontos ou em NENHUM não gera bracket, e
/// os vereditos por ponto batem com o veredito final. Cobertura direta do
/// Passo 3 (a varredura em si, não só a classificação pura já testada em
/// `src/validation/incerteza.rs`).
#[test]
fn checks_determinados_nao_tem_breakeven_e_vereditos_por_ponto_sao_consistentes() {
    let (cfg, engine, req) = carrega_baseline();
    let nominal = aeronave::pipeline::executa(&cfg, &engine, &req)
        .expect("baseline tem que convergir");
    let inc = analisa(&cfg, &engine, &req, &nominal);

    for c in &inc.checks {
        match c.veredito {
            Veredito::Falha => {
                assert_eq!(c.veredito_lo, Veredito::Falha, "id={}", c.id);
                assert_eq!(c.veredito_nominal, Veredito::Falha, "id={}", c.id);
                assert_eq!(c.veredito_hi, Veredito::Falha, "id={}", c.id);
                assert_eq!(c.breakeven, None, "id={}", c.id);
            }
            Veredito::Passa => {
                assert_eq!(c.veredito_lo, Veredito::Passa, "id={}", c.id);
                assert_eq!(c.veredito_nominal, Veredito::Passa, "id={}", c.id);
                assert_eq!(c.veredito_hi, Veredito::Passa, "id={}", c.id);
                assert_eq!(c.breakeven, None, "id={}", c.id);
            }
            Veredito::Indeterminado => {
                // já coberto em detalhe pelos dois testes acima para o
                // único caso do baseline.
            }
        }
    }
}

// ── CONSERTO 1 (revisão da Task 4) — os portões entram na varredura ────
//
// A §5.2 deu id aos 9 portões EXATAMENTE para isto ("sem id ficariam fora
// da varredura"); a §5.4 escrevia o algoritmo só em termos de
// `report.violations`, contradizendo a §5.2. `portao_v_cruzeiro`,
// `portao_flutter`, `portao_antitombamento` e `portao_estabilidade_long`
// não têm NENHUMA `Violacao` correspondente — sem a união eram
// inteiramente invisíveis à varredura.

/// Suporte do `PORTOES_AGREGADOS` de `src/validation/incerteza.rs`:
/// `portao_restricoes.ok` é literalmente `report.all_satisfied()` ==
/// `violations.is_empty()`. PROVADO aqui, não assumido — se um dia deixar
/// de valer, este teste reprova primeiro.
#[test]
fn portao_restricoes_e_funcao_deterministica_das_violacoes() {
    let (cfg, engine, req) = carrega_baseline();
    let res = aeronave::pipeline::executa(&cfg, &engine, &req)
        .expect("baseline tem que convergir");
    let p = res.portoes.iter().find(|p| p.id == "portao_restricoes")
        .expect("portao_restricoes tem que existir entre os 9 portões");
    assert_eq!(p.ok, res.report.violations.is_empty(),
        "portao_restricoes.ok tem que ser EXATAMENTE violations.is_empty() — se isto \
         falhar, a exclusão de PORTOES_AGREGADOS deixou de ser válida e precisa ser \
         revista, não só re-testada");
}

/// `portao_envelope_cg_todos` reprova de VERDADE no baseline (duplica
/// `envelope_cg::Solo (piloto)` em SIGNIFICADO, mas tem id DIFERENTE — a
/// duplicação fica visível por regra, não é suprimida) e TEM que aparecer
/// na varredura. `portao_restricoes` NUNCA aparece — é o agregado excluído.
///
/// Verificado por mutação: revertendo `ids_do_ponto` para só
/// `report.violations` (o comportamento de antes deste conserto), este
/// teste reprova — ver relatório da task.
#[test]
fn portoes_entram_na_varredura_do_baseline() {
    let (cfg, engine, req) = carrega_baseline();
    let nominal = aeronave::pipeline::executa(&cfg, &engine, &req)
        .expect("baseline tem que convergir");
    let inc = analisa(&cfg, &engine, &req, &nominal);

    let ids: Vec<&str> = inc.checks.iter().map(|c| c.id.as_str()).collect();

    let portao_cg = inc.checks.iter().find(|c| c.id == "portao_envelope_cg_todos")
        .unwrap_or_else(|| panic!(
            "portao_envelope_cg_todos reprova no baseline (duplica envelope_cg::* em \
             significado) e TEM que estar na varredura — se isto falhar, os portões \
             pararam de entrar no conjunto de ids. ids obtidos: {ids:?}"));
    assert_eq!(portao_cg.veredito, Veredito::Falha);
    assert_eq!(portao_cg.veredito_lo, Veredito::Falha);
    assert_eq!(portao_cg.veredito_nominal, Veredito::Falha);
    assert_eq!(portao_cg.veredito_hi, Veredito::Falha);

    assert!(!ids.contains(&"portao_restricoes"),
        "portao_restricoes é agregado — não pode aparecer como check independente. \
         ids obtidos: {ids:?}");

    // A duplicação em SIGNIFICADO fica visível: o id da violação de origem
    // e o id do portão que a duplica aparecem os DOIS.
    assert!(ids.contains(&"envelope_cg::Solo (piloto)"));
}

// ── CONSERTO 3 (revisão da Task 4) — a banda EFETIVA está protegida ────
//
// ACHADO reportado ao coordenador: no baseline de hoje, os CONJUNTOS de id
// violados em `banda.hi` (0,815976999245888) e em `banda.hi_declarado`
// (0,8250000000000001) COINCIDEM — {"decolagem_grama",
// "envelope_cg::Solo (piloto)", "robustez::Cenário '2 pax
// dianteiros'::dianteiro"} nos dois pontos, mesmo incluindo os portões do
// CONSERTO 1. Só os TEXTOS (magnitudes: "778 m" vs "768 m"; "18.68" vs
// "18.70") diferem. Uma proteção apoiada só em vereditos/ids NÃO
// distinguiria `banda.hi` de `banda.hi_declarado` neste baseline — por
// isso `Incerteza` agora expõe o valor de `fom_static` EFETIVAMENTE usado
// (`fom_lo_usado`/`fom_hi_usado`), e a proteção é sobre ESSE valor, não
// sobre o efeito indireto que ele produziria hoje.
#[test]
fn varredura_usa_banda_hi_nao_hi_declarado_mesmo_quando_os_ids_coincidem() {
    let (cfg, engine, req) = carrega_baseline();
    let nominal = aeronave::pipeline::executa(&cfg, &engine, &req)
        .expect("baseline tem que convergir");
    let inc = analisa(&cfg, &engine, &req, &nominal);

    assert_eq!(inc.fom_lo_usado, inc.banda.lo);
    assert_eq!(inc.fom_hi_usado, inc.banda.hi);
    assert_ne!(inc.fom_hi_usado, inc.banda.hi_declarado,
        "a varredura tem que rodar em banda.hi, NUNCA em banda.hi_declarado — mesmo \
         sabendo (achado acima) que no baseline de hoje os dois pontos produzem o \
         MESMO conjunto de ids violados");

    // Proteção estrutural, independente de qualquer coincidência do
    // baseline: hi_declarado cai FORA do domínio fisicamente admissível
    // (fom_static > fom_design ⇒ FoM(J) decrescente em J — spec §5.1), então
    // usá-lo nunca pode ser correto, mesmo que o conjunto de ids não denuncie.
    assert!(inc.banda.hi <= cfg.propeller.fom_design,
        "banda.hi tem que respeitar o domínio admissível");
    assert!(inc.banda.hi_declarado > cfg.propeller.fom_design,
        "banda.hi_declarado tem que estar FORA do domínio admissível no baseline — é \
         o motivo da truncagem existir; se isto falhar, a fixture mudou e este teste \
         perde a base");

    // Prova complementar, mais fraca (não se apoia nela sozinha): os TEXTOS
    // das violações em hi e em hi_declarado DIFEREM, mesmo com os IDS iguais.
    let mut cfg_hi = cfg.clone();
    cfg_hi.propeller.fom_static = inc.banda.hi;
    let res_hi = aeronave::pipeline::executa(&cfg_hi, &engine, &req).unwrap();

    let mut cfg_decl = cfg.clone();
    cfg_decl.propeller.fom_static = inc.banda.hi_declarado;
    let res_decl = aeronave::pipeline::executa(&cfg_decl, &engine, &req).unwrap();

    assert_ne!(res_hi.report.textos(), res_decl.report.textos(),
        "os textos têm que diferir (magnitudes diferentes) mesmo com os ids \
         coincidindo — achado desta task: os CONJUNTOS de id em hi e hi_declarado \
         coincidem no baseline de hoje, então este teste NÃO se apoia neles sozinhos");
}
