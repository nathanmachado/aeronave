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
