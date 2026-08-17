//! Teste de integração: contrato JSON versionado `AircraftReport` v4 (Task 6.1).
//!
//! Roda o pipeline completo (`size_aircraft` + todos os agentes), exatamente
//! como `main.rs`, monta o `AircraftReport` final e verifica:
//!   1. `schema_version == "4.0"` (a constante `SCHEMA_VERSION`).
//!   2. Todos os blocos de topo esperados estão presentes no JSON gerado
//!      (contrato mínimo com o time de CAD).
//!   3. `warnings` não está vazio (o baseline real tem um aviso conhecido de
//!      pico elétrico — ver `validation::constraint_checker`, item 15).
//!   4. `fidelity` não está vazio (mapa de honestidade por bloco).
//!   5. Round-trip serde: serializar → desserializar → campos-chave batem.

use std::path::PathBuf;

use aeronave::agents::weight_balance::mac_spanwise_pos;
use aeronave::models::config::{load_aircraft, load_engine, load_mission};
use aeronave::models::specs::{
    AircraftReport, GeometrySpec, SizingReport, UncertaintySpec, SCHEMA_VERSION,
};
use aeronave::validation::incerteza;
use std::collections::BTreeMap;

fn config_path(rel: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(rel)
}

/// Monta o `AircraftReport` completo a partir da aeronave-base real
/// (`config/aircraft/baseline_4seat.toml` + Toyota 1GD-FTV + missão
/// default).
///
/// Ciclo 16 (Task 1, dividendo — spec §5.3): antes desta task, esta função
/// REIMPLEMENTAVA o pipeline inteiro (uma cópia da sequência de agentes de
/// `main.rs`) e decidia o veredito global só por `report.all_satisfied()`
/// — o portão `#0` de `main.rs`, sem os outros 8 (V_cruzeiro, autonomia de
/// bloco, RC, teto de serviço, flutter, anti-tombamento, estabilidade
/// longitudinal, envelope de CG). Eram DUAS definições de "veredito
/// global" DIVERGENTES coincidindo por acaso no baseline (nenhum dos 8
/// portões extras jamais reprovava sozinho enquanto `#0` passava) — a
/// doença do #13 (`docs/backlog.md`) em outra roupa: o teste que deveria
/// vigiar o pipeline mantinha uma cópia dele que podia divergir em
/// silêncio. Agora chama `pipeline::executa` — a MESMA função que
/// `main.rs` chama.
///
/// `old→new` (ciclo 16, Task 5): o veredito global deixou de ser o AND dos 9
/// `res.portoes` — vira `validation::incerteza::veredito_global`, os TRÊS
/// estados PASS/FAIL/INDETERMINADO da spec §5.5, calculados sobre a MESMA
/// união de ids que os 9 portões cobriam (`Incerteza::checks`, ver
/// `ids_do_ponto`), então nenhuma cobertura foi perdida — só ganhou um
/// terceiro valor. `violations` também deixou de ser `report.textos()` cru:
/// vira `incerteza::publica_violacoes`, que reescreve/insere o texto
/// INDETERMINADO (spec §5.6 + ERRATUM). Mesma função que `main.rs` chama —
/// mantendo o precedente do parágrafo acima.
fn build_baseline_report() -> AircraftReport {
    let cfg = load_aircraft(&config_path("config/aircraft/baseline_4seat.toml")).unwrap();
    let engine = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let req = load_mission(&config_path("config/missions/default.toml")).unwrap();

    let res = aeronave::pipeline::executa(&cfg, &engine, &req)
        .expect("aeronave-base + Toyota deveria convergir com o tanque de 260 L");

    let design_mtow_kg = res.state.mtow_kg;
    let envelope_mtow_kg = res.wb.spec.mtow_kg;

    let inc = incerteza::analisa(&cfg, &engine, &req, &res);
    let validation_status = incerteza::veredito_global(&inc).to_string();
    let violations = incerteza::publica_violacoes(&res.report.violations, &inc);
    let uncertainty = UncertaintySpec::from_incerteza(cfg.propeller.fom_static_tol_pct, &inc);

    let geometry = GeometrySpec {
        wing_le_root_x_m: cfg.wing.le_root_x_m,
        chord_root_m: res.wb.chord_root_m,
        chord_tip_m: res.wb.chord_tip_m,
        mac_m: res.wb.mac_m,
        mac_le_x_m: res.wb.mac_le_x_m,
        y_mac_m: mac_spanwise_pos(res.wing.span_m, res.wing.taper_ratio),
        fuselage_length_m: cfg.fuselage.length_m,
        cabin_width_m: cfg.fuselage.cabin_width_m,
        cabin_height_m: cfg.fuselage.cabin_height_m,
    };

    let fuel_margin_l = cfg.fuel_system.capacity_l - res.mission.fuel_total_l;
    let sizing = SizingReport {
        mtow_mission_kg: design_mtow_kg,
        mtow_envelope_kg: envelope_mtow_kg,
        iterations: res.iterations.clone(),
        converged: true,
        fuel_required_l: res.mission.fuel_total_l,
        fuel_capacity_l: cfg.fuel_system.capacity_l,
        fuel_margin_l,
        fuel_margin_pct: fuel_margin_l / cfg.fuel_system.capacity_l * 100.0,
        constraints: res.constraints.clone(),
    };

    let mut fidelity: BTreeMap<String, String> = BTreeMap::new();
    fidelity.insert("wing".into(), "semi-empirical (polar por build-up)".into());
    fidelity.insert("propulsion".into(), "semi-empirical (curvas de catálogo + BSFC paramétrico)".into());
    fidelity.insert("structure".into(), "preliminary (vigas simplificadas; requer FEM); flutter preliminary — requer GVT".into());
    fidelity.insert("mission".into(), "computed (segmentos + Breguet L/D constante)".into());
    fidelity.insert("empennage".into(), "preliminary (coeficiente de volume; requer VLM/CFD)".into());
    fidelity.insert("trim".into(), "preliminary (semi-empírico; sensível a cl_h_max_down)".into());
    fidelity.insert("robustness".into(),
        "computed (pior-caso determinístico ±σ direcional sobre as 7 massas estruturais; \
         limites de envelope nominais — invariantes a massa)".into());

    AircraftReport {
        schema_version: SCHEMA_VERSION.to_string(),
        revision: SCHEMA_VERSION.to_string(),
        validation_status,
        wing: res.wing.clone(),
        propulsion: res.prop.clone(),
        geometry: Some(geometry),
        empennage: Some(res.empennage.clone()),
        control_surfaces: Some(res.control_surfaces.clone()),
        weight: Some(res.wb.spec.clone()),
        trim: Some(res.trim.clone()),
        performance: Some(res.perf.clone()),
        vn_diagram: Some(res.vn.clone()),
        structure: Some(res.struc.clone()),
        landing_gear: Some(res.gear.clone()),
        propeller: Some(res.propeller.clone()),
        mission: Some(res.mission.clone()),
        electrical: Some(res.electrical.clone()),
        sizing: Some(sizing),
        robustness: Some(res.robustness.clone()),
        fidelity,
        violations,
        warnings: res.report.warnings.clone(),
        uncertainty,
    }
}

#[test]
fn schema_version_e_16_blocos_de_topo_presentes() {
    let report = build_baseline_report();
    // `old→new` (ciclo 16, Task 5 — bump MAJOR, ver docstring de
    // `SCHEMA_VERSION` para a justificativa completa): "5.7" → "6.0".
    assert_eq!(report.schema_version, "6.0");
    assert_eq!(report.schema_version, SCHEMA_VERSION);

    let json = serde_json::to_string_pretty(&report).expect("deveria serializar");
    let value: serde_json::Value = serde_json::from_str(&json).expect("deveria parsear como JSON");
    let obj = value.as_object().expect("raiz deveria ser um objeto JSON");

    let expected_keys = [
        "schema_version", "revision", "validation_status", "wing", "propulsion",
        "geometry", "empennage", "control_surfaces", "weight", "trim", "performance",
        "vn_diagram", "structure", "landing_gear", "propeller", "mission",
        "electrical", "sizing", "robustness", "fidelity", "violations", "warnings",
        // Ciclo 16, Task 5: bloco novo — ver `UncertaintySpec`.
        "uncertainty",
    ];
    assert!(expected_keys.len() >= 17, "lista de chaves esperadas deveria ter pelo menos 17 entradas");
    for key in expected_keys {
        assert!(obj.contains_key(key), "chave de topo ausente no JSON: '{key}'");
    }
    assert_eq!(obj.get("schema_version").unwrap().as_str().unwrap(), "6.0");
}

/// Ciclo 16 (Task 5, spec §5.7): o bloco `uncertainty` publica a banda
/// EFETIVA (truncada em `fom_design`) de `propeller.fom_static` e o único
/// check indeterminado do baseline (gradiente CS 23.65), com o breakeven
/// MEDIDO — não um valor recalculado à mão, o mesmo `UncertaintySpec` que
/// `main.rs` escreve no `aircraft_spec.json` commitado.
///
/// Ordem de `checks`: alfabética por `id` (`validation::incerteza::analisa`
/// itera um `BTreeSet<String>`) — `gradiente_cs2365` cai no índice 2 entre
/// os 5 ids do baseline (`decolagem_grama` < `envelope_cg::Solo (piloto)` <
/// `gradiente_cs2365` < `portao_envelope_cg_todos` < `robustez::…`). Se um
/// dia um sexto check entrar em ordem alfabética anterior, este índice
/// precisa ser revisto — não é acidente, é a ordem publicada de hoje.
#[test]
fn uncertainty_bloco_publica_banda_efetiva_e_o_gradiente_indeterminado() {
    let report = build_baseline_report();
    let json = serde_json::to_string(&report).expect("deveria serializar");
    let value: serde_json::Value = serde_json::from_str(&json).expect("deveria parsear como JSON");
    let u = &value["uncertainty"];

    assert_eq!(u["parameter"].as_str().unwrap(), "propeller.fom_static");
    assert_eq!(u["nominal"].as_f64().unwrap(), 0.75); // PIN: uncertainty.nominal
    assert_eq!(u["declared_tol_pct"].as_f64().unwrap(), 10.0); // PIN: uncertainty.declared_tol_pct
    assert_eq!(u["band_declared_lo"].as_f64().unwrap(), 0.675); // PIN: uncertainty.band_declared_lo
    // PIN: uncertainty.band_declared_hi
    assert_eq!(u["band_declared_hi"].as_f64().unwrap(), 0.825_000_000_000_000_1);
    assert_eq!(u["band_lo"].as_f64().unwrap(), 0.675); // PIN: uncertainty.band_lo
    // PIN: uncertainty.band_hi
    assert_eq!(u["band_hi"].as_f64().unwrap(), 0.815_976_999_245_888);
    assert!(u["band_truncated"].as_bool().unwrap(),
        "baseline real: a banda tem que estar truncada em fom_design");
    assert!(u["band_truncated_reason"].as_str().unwrap().contains("fom_design"),
        "a razão de truncagem tem que citar fom_design — publicada, não em silêncio");
    assert!(u["ceiling_evaluated"].as_bool().unwrap());

    let checks = u["checks"].as_array().expect("checks deveria ser um array");
    assert_eq!(checks.len(), 5,
        "baseline real: 5 checks entram no bloco (violam em pelo menos um dos 4 pontos) — \
         {checks:#?}");

    let gradiente = &checks[2];
    assert_eq!(gradiente["id"].as_str().unwrap(), "gradiente_cs2365");
    assert_eq!(gradiente["veredito"].as_str().unwrap(), "INDETERMINADO");
    assert_eq!(gradiente["veredito_lo"].as_str().unwrap(), "FALHA");
    assert_eq!(gradiente["veredito_nominal"].as_str().unwrap(), "FALHA");
    assert_eq!(gradiente["veredito_hi"].as_str().unwrap(), "PASSA");
    assert!(gradiente["alcance_de_helice"].as_bool().unwrap());
    // PIN: uncertainty.checks.2.breakeven_lo
    assert_eq!(gradiente["breakeven_lo"].as_f64().unwrap(), 0.784_867_237_235_786_1);
    // PIN: uncertainty.checks.2.breakeven_hi
    assert_eq!(gradiente["breakeven_hi"].as_f64().unwrap(), 0.784_867_775_020_359_6);
    assert!(gradiente["motivo"].is_null(),
        "o gradiente tem bracket medido — não precisa de motivo textual");

    // Os outros 4 checks são FALHA determinada, sem breakeven — cobertura
    // direta de que `UncertaintyCheckSpec::from` NÃO inventa bracket para
    // quem não tem.
    for c in checks {
        if c["id"].as_str().unwrap() == "gradiente_cs2365" {
            continue;
        }
        assert_eq!(c["veredito"].as_str().unwrap(), "FALHA", "id={}", c["id"]);
        assert!(c["breakeven_lo"].is_null(), "id={}", c["id"]);
        assert!(c["breakeven_hi"].is_null(), "id={}", c["id"]);
    }
}

/// `validation::incerteza::publica_violacoes` reescreveu o texto da CS 23.65
/// com o prefixo `INDETERMINADO — ` — a interface pública de facto é o
/// TEXTO (spec §5.6, razão 3), então o vínculo com o JSON tem que ser
/// verificado no `violations` publicado, não só no bloco `uncertainty`.
#[test]
fn violations_publica_o_texto_indeterminado_para_o_gradiente() {
    let report = build_baseline_report();
    assert_eq!(report.violations.len(), 4,
        "contagem PERMANECE 4 — INDETERMINADO reescreve, não insere, porque o gradiente JÁ \
         violava no nominal: {:#?}", report.violations);

    let gradiente = report.violations.iter()
        .find(|v| v.contains("Gradiente de subida"))
        .expect("uma das 4 violações tem que ser o gradiente CS 23.65");
    assert!(gradiente.starts_with("INDETERMINADO — "), "obtido: {gradiente}");
    assert!(gradiente.contains("banda declarada de propeller.fom_static"), "obtido: {gradiente}");
    assert!(gradiente.contains("breakeven em"), "obtido: {gradiente}");
    assert!(gradiente.ends_with("O modelo NÃO sustenta este veredito."), "obtido: {gradiente}");

    // As outras 3 violações NÃO carregam o prefixo — só o check indeterminado
    // é reescrito.
    let nao_indeterminadas: Vec<&String> = report.violations.iter()
        .filter(|v| !v.starts_with("INDETERMINADO — "))
        .collect();
    assert_eq!(nao_indeterminadas.len(), 3, "obtido: {nao_indeterminadas:#?}");
}

/// Falha determinada domina indeterminação (spec §5.5): mesmo com o
/// gradiente CS 23.65 indeterminado, `validation_status` continua FAIL —
/// as outras 3 violações são falha DETERMINADA. Este ciclo não muda o
/// veredito do projeto, só o que o modelo consegue dizer sobre ele.
#[test]
fn validation_status_do_baseline_continua_fail_com_indeterminado_presente() {
    let report = build_baseline_report();
    assert_eq!(report.validation_status, "FAIL");
}

/// Schema 5.0 (Task 2, ciclo7-clmax-decolagem — bump MAJOR): `wing.cl_max_to`
/// (NOVO, derivado na Task 1 do mesmo ciclo) é numericamente ENTRE
/// `cl_max_clean` e `cl_max_flaps` — consistente com sua definição de
/// interpolação linear pela fração de deployment do flap de decolagem,
/// `cl_max_to = cl_max_clean + to_flap_fraction·(cl_max_flaps −
/// cl_max_clean)` com `0 < to_flap_fraction < 1`. `cl_max_flaps` não é
/// ecoado no JSON (só o `cl_max` de pouso, que é o mesmo valor internamente
/// — ver `WingSpec::cl_max`), então o teste usa `cl_max` como o teto de
/// pouso equivalente. O campo `trim.to_flap_fraction` (RENOMEADO de
/// `to_flap_cm_fraction` na Task 1 — motivo do bump MAJOR, não MINOR: a
/// política de versionamento do schema, `SCHEMA_VERSION`/§1 deste
/// documento, classifica renome de campo serializado como mudança que
/// QUEBRA compatibilidade) está presente, e o nome ANTIGO
/// `to_flap_cm_fraction` NÃO aparece mais em lugar nenhum do JSON.
#[test]
fn wing_cl_max_to_entre_clean_e_flaps_trim_to_flap_fraction_renomeado() {
    let report = build_baseline_report();

    assert!(
        report.wing.cl_max_to > report.wing.cl_max_clean
            && report.wing.cl_max_to < report.wing.cl_max,
        "wing.cl_max_to ({}) deveria ficar estritamente entre cl_max_clean ({}) e \
         cl_max/cl_max_flaps de pouso ({})",
        report.wing.cl_max_to, report.wing.cl_max_clean, report.wing.cl_max,
    );

    let json = serde_json::to_string(&report).expect("deveria serializar");
    let value: serde_json::Value = serde_json::from_str(&json).expect("deveria parsear como JSON");
    let trim = &value["trim"];
    assert!(
        trim.get("to_flap_fraction").and_then(|v| v.as_f64()).is_some(),
        "trim.to_flap_fraction deveria estar presente e ser numérico no JSON"
    );
    assert!(
        trim.get("to_flap_cm_fraction").is_none(),
        "trim.to_flap_cm_fraction (nome ANTIGO, renomeado na Task 1) não deveria mais \
         aparecer no JSON — o campo é to_flap_fraction"
    );
    assert!(
        !json.contains("to_flap_cm_fraction"),
        "o JSON completo não deveria conter nenhuma ocorrência do nome antigo \
         'to_flap_cm_fraction'"
    );
}

/// Ciclo 8 (task 1, arrasto de flap na polar — introduzido ainda dentro de
/// v5.0; o bump formal para v5.1 foi concluído na Task 3 do mesmo ciclo,
/// ver `docs/aircraft_spec.schema.md`): `wing.cd0_flap_to_extra` (NOVO) está
/// presente e numérico no JSON, e bate com a fórmula fechada
/// `to_flap_fraction · cd0_flap_delta` — mesmo precedente de
/// `wing_cl_max_to_entre_clean_e_flaps_trim_to_flap_fraction_renomeado`
/// acima para `cl_max_to`. `cd0_flap_delta` em si não é ecoado no JSON (só
/// o produto derivado), então o teste recomputa a partir da config do
/// baseline real.
#[test]
fn wing_cd0_flap_to_extra_presente_e_bate_com_formula_fechada() {
    let cfg = load_aircraft(&config_path("config/aircraft/baseline_4seat.toml")).unwrap();
    let report = build_baseline_report();

    let json = serde_json::to_string(&report).expect("deveria serializar");
    let value: serde_json::Value = serde_json::from_str(&json).expect("deveria parsear como JSON");
    let cd0_flap_to_extra_json = value["wing"].get("cd0_flap_to_extra")
        .and_then(|v| v.as_f64());
    assert!(
        cd0_flap_to_extra_json.is_some(),
        "wing.cd0_flap_to_extra deveria estar presente e ser numérico no JSON"
    );

    let esperado = cfg.stability.to_flap_fraction * cfg.wing.cd0_flap_delta;
    let obtido = cd0_flap_to_extra_json.unwrap();
    assert!(
        (obtido - esperado).abs() < 1e-9,
        "wing.cd0_flap_to_extra no JSON ({obtido:.9}) deveria bater com a fórmula fechada \
         to_flap_fraction·cd0_flap_delta ({esperado:.9})"
    );
    assert!(
        obtido > 0.0 && obtido < cfg.wing.cd0_flap_delta,
        "wing.cd0_flap_to_extra ({obtido:.6}) deveria ficar ESTRITAMENTE entre 0 e o delta \
         cheio de pouso ({:.6})", cfg.wing.cd0_flap_delta
    );
}

/// Schema 5.1 (Task 3, ciclo8-flap-e-solo — bump formal MINOR que
/// formaliza os dois campos aditivos das Tasks 1/2 do mesmo ciclo, ver
/// `docs/aircraft_spec.schema.md` §1): `propeller.prop_clearance_critical_m`
/// (NOVO, ciclo 8 task 2 — folga ponta de pá ↔ solo na condição CRÍTICA de
/// CS 23.925, checagem #25) está presente e numérico no JSON. Sem fórmula
/// fechada independente aqui (o campo já depende de agentes distintos
/// rodando em sequência — `PropellerAgent` + `LandingGearAgent`/
/// `PropellerSpec::fill_critical_clearance` — reproduzir a fórmula neste
/// teste duplicaria a lógica do pipeline sem adicionar cobertura; a fórmula
/// fechada já é coberta por
/// `models::specs::tests::fill_critical_clearance_bate_com_a_formula_fechada`).
///
/// ATUALIZAÇÃO (ciclo 9, transferência de atitude do #25 — old→new):
/// Baseline real E10 ≈+0,0325 m (PASS, simplificação 1:1) → **≈−0,06416 m
/// (FAIL)** — a fórmula ganha o fator de amplificação do pivô sobre o trem
/// principal (`(x_main−prop_plane_x_m)/(x_main−x_nose_m)` ≈ 1,46610 nesta
/// geometria), física corrigida do achado de review do ciclo 8
/// (`docs/backlog.md`, item 1). Nenhuma tolerância afrouxada — o pin
/// (±0,001) é o mesmo padrão de antes, só o valor central mudou.
///
/// ATUALIZAÇÃO (ciclo 10, task 1, deflexão estática — old→new): campo
/// novo `[gear].static_sag_fraction` corrige uma dupla contagem da
/// compressão estática do nariz (curso TOTAL → curso RESTANTE, ver
/// docstring de `GearCfg::static_sag_fraction`). Baseline real E10
/// **≈−0,06416 m (ciclo 9) → ≈−0,00249 m (ciclo 10)** — MESMO veredito
/// (checagem #25 continua FAIL), só o número muda, honestamente
/// ANTI-conservador. `fator` (≈1,46610) inalterado. Ver
/// `docs/backlog.md` (item 6, RESOLVIDO ciclo 10).
///
/// ATUALIZAÇÃO (campanha E12 "nariz-only", 2026-08-10, adoção pós-ciclo-10
/// — old→new): `[gear].x_nose_m` 1,30→1,20 (metade barata da célula E11
/// do ciclo 9 — só o nariz, `[propeller].prop_axis_above_cg_m` mantido em
/// 0,20). O denominador do fator geométrico `(x_main−prop_plane_x_m)/
/// (x_main−x_nose_m)` alonga, e o fator CAI de ≈1,46610 para ≈1,40650 —
/// menos amplificação do mergulho do nariz. `prop_clearance_critical_m`
/// **≈−0,00249 m (ciclo 10) → +0,007367 m (E12) — VIRA POSITIVO**: esta é
/// a checagem #25 fechando, primeiro PASS do baseline com o modelo
/// completo. Veredito muda: FAIL → PASS.
#[test]
fn propeller_prop_clearance_critical_m_presente_e_numerico_proximo_do_esperado() {
    let report = build_baseline_report();

    let json = serde_json::to_string(&report).expect("deveria serializar");
    let value: serde_json::Value = serde_json::from_str(&json).expect("deveria parsear como JSON");
    let obtido = value["propeller"].get("prop_clearance_critical_m")
        .and_then(|v| v.as_f64());
    assert!(
        obtido.is_some(),
        "propeller.prop_clearance_critical_m deveria estar presente e ser numérico no JSON"
    );
    let obtido = obtido.unwrap();
    assert!(
        // PIN: propeller.prop_clearance_critical_m
        (obtido - 0.007367).abs() < 0.001,
        "propeller.prop_clearance_critical_m ({obtido:.6}) deveria ficar próximo de ≈+0,007367 m \
         (baseline real pós-E12 nariz-only, checagem #25 PASS — old: ≈−0,00249 m ciclo 10)"
    );
    assert!(obtido > 0.0,
        "campanha E12 nariz-only: baseline real deveria APROVAR a checagem #25 (folga crítica \
         positiva pela primeira vez com o modelo completo)");
}

/// Schema 4.6 (Task 4, ciclo4-fidelidade-massas — check #19): o bloco
/// `robustness` (`RobustnessSpec`) está presente no JSON e traz
/// `sigma_mass_fraction` (eco de `[mass_model].sigma_mass_fraction`) e
/// `flips` como array (vazio ou não — o baseline real, σ=15%, não produz
/// nenhum flip, ver `tests/gear_tipback.rs`/`tests/cli.rs` para o achado
/// honesto completo). Ciclo 5 (task massa-total): `mtow_masstotal_kg`
/// também presente e, no baseline real (sem flip de Dimensionamento),
/// estritamente MAIOR que o MTOW de missão nominal
/// (`sizing.mtow_mission_kg`) — os 5 fatores de composto só multiplicam
/// por (1+σ) > 1. Schema 4.7 (Task 4, ciclo5-robustez-total-e-solo): o
/// bump de versão que formaliza `mtow_masstotal_kg` (e `electrical.loads`,
/// ver teste dedicado abaixo) como parte do contrato. Schema 4.8 (Task 4,
/// ciclo6-pista-e-robustez-final): NENHUM campo novo neste bloco — o mundo
/// "massa-total" passa a avaliar TAMBÉM pista (#23/#24) e envelope/nariz/
/// tipback (não só os gates de desempenho que já existiam), mas isso é
/// comportamento do `RobustnessAgent`/`ConstraintChecker`, não uma mudança
/// de forma do JSON; o bump formaliza o requisito `runway_available_m` e as
/// checagens #23/#24 (`ConstraintChecker::verify`) como parte do contrato
/// v4 — ver `docs/aircraft_spec.schema.md` §1.
#[test]
fn robustness_presente_com_sigma_e_flips_array() {
    let report = build_baseline_report();
    let robustness = report.robustness.as_ref().expect("robustness deveria estar presente");
    assert!(robustness.sigma_mass_fraction > 0.0,
        "sigma_mass_fraction deveria ser positivo, obteve {}", robustness.sigma_mass_fraction);
    let sizing = report.sizing.as_ref().expect("sizing deveria estar presente");
    assert!(robustness.mtow_masstotal_kg > sizing.mtow_mission_kg,
        "achado honesto (baseline real, sem flip de Dimensionamento): mtow_masstotal_kg ({:.2}) \
         deveria ficar ACIMA do MTOW de missão nominal ({:.2})",
        robustness.mtow_masstotal_kg, sizing.mtow_mission_kg);

    let json = serde_json::to_string(&report).expect("deveria serializar");
    let value: serde_json::Value = serde_json::from_str(&json).expect("deveria parsear como JSON");
    let rob = &value["robustness"];
    assert!(rob["sigma_mass_fraction"].is_number(),
        "robustness.sigma_mass_fraction deveria estar presente e ser numérico no JSON");
    assert!(rob["flips"].is_array(), "robustness.flips deveria ser um array no JSON");
    assert!(rob["cg_fwd_case_pct_mac"].is_array(), "robustness.cg_fwd_case_pct_mac deveria ser um array no JSON");
    assert!(rob["cg_aft_case_pct_mac"].is_array(), "robustness.cg_aft_case_pct_mac deveria ser um array no JSON");
    assert!(rob["mtow_masstotal_kg"].is_number(),
        "robustness.mtow_masstotal_kg deveria estar presente e ser numérico no JSON");
}

/// Schema 4.5 (Task 5, oew-parametrico): `weight.structural_masses` —
/// as 7 massas estruturais COMPUTADAS (`agents::mass_model`) + os 5
/// fatores de composto usados (`[mass_model]`), rastreáveis no JSON
/// final (antes só disponíveis internamente via `SizedAircraft::
/// structural_masses`, nunca ecoadas dentro do bloco `weight`).
#[test]
fn weight_structural_masses_presente_e_positivo() {
    let report = build_baseline_report();
    let weight = report.weight.as_ref().expect("weight deveria estar presente");
    let sm = &weight.structural_masses;

    assert!(sm.asa_kg > 0.0, "asa_kg deveria ser positivo, obteve {}", sm.asa_kg);
    assert!(sm.fuselagem_kg > 0.0, "fuselagem_kg deveria ser positivo, obteve {}", sm.fuselagem_kg);
    assert!(sm.emp_h_kg > 0.0, "emp_h_kg deveria ser positivo, obteve {}", sm.emp_h_kg);
    assert!(sm.emp_v_kg > 0.0, "emp_v_kg deveria ser positivo, obteve {}", sm.emp_v_kg);
    assert!(sm.trem_principal_kg > 0.0, "trem_principal_kg deveria ser positivo, obteve {}", sm.trem_principal_kg);
    assert!(sm.trem_nariz_kg > 0.0, "trem_nariz_kg deveria ser positivo, obteve {}", sm.trem_nariz_kg);
    assert!(sm.tanques_kg > 0.0, "tanques_kg deveria ser positivo, obteve {}", sm.tanques_kg);
    assert!(sm.composite_factor_wing > 0.0,
        "composite_factor_wing deveria ser positivo, obteve {}", sm.composite_factor_wing);

    // Rastreabilidade: as massas ecoadas em `weight.structural_masses` são
    // EXATAMENTE as mesmas que entraram no OEW (`SizedAircraft::
    // structural_masses`), não uma cópia recomputada independentemente.
    let json = serde_json::to_string(&report).expect("deveria serializar");
    assert!(json.contains("\"structural_masses\""),
        "JSON deveria conter a chave 'structural_masses' dentro de 'weight'");
}

/// Schema 4.7 (Task 4, ciclo5-robustez-total-e-solo — check #20):
/// `electrical.loads` (`Vec<ElectricalLoadSpec>`) ecoa individualmente
/// cada `[electrical].loads` configurada — nome, potência contínua e
/// potência de pico — para que `ConstraintChecker::verify` compare o pico
/// DECLARADO da carga 'trem_retratil' contra `landing_gear.
/// actuator_power_w` COMPUTADO (checagem só possível pós-convergência).
#[test]
fn electrical_loads_presente_nao_vazio_com_name_e_peak_w() {
    let report = build_baseline_report();
    let electrical = report.electrical.as_ref().expect("electrical deveria estar presente");
    assert!(!electrical.loads.is_empty(),
        "electrical.loads deveria ser um array NÃO-vazio (cargas configuradas do baseline)");
    for load in &electrical.loads {
        assert!(!load.name.is_empty(), "cada carga elétrica deveria ter um 'name' não-vazio");
        assert!(load.peak_w > 0.0, "carga '{}' deveria ter peak_w positivo, obteve {}", load.name, load.peak_w);
    }

    let json = serde_json::to_string(&report).expect("deveria serializar");
    let value: serde_json::Value = serde_json::from_str(&json).expect("deveria parsear como JSON");
    let loads = value["electrical"]["loads"].as_array()
        .expect("electrical.loads deveria ser um array no JSON");
    assert!(!loads.is_empty(), "electrical.loads não deveria estar vazio no JSON");
    for load in loads {
        assert!(load["name"].is_string(), "cada item de electrical.loads deveria ter 'name' string");
        assert!(load["peak_w"].is_number(), "cada item de electrical.loads deveria ter 'peak_w' numérico");
    }
}

#[test]
fn warnings_do_baseline_contem_aviso_de_pico_eletrico() {
    let report = build_baseline_report();
    assert!(!report.warnings.is_empty(), "baseline deveria ter ao menos um warning (pico elétrico)");
    assert!(
        report.warnings.iter().any(|w| w.contains("pico")),
        "esperava aviso mencionando 'pico' (elétrico), obteve: {:?}", report.warnings
    );
}

#[test]
fn fidelity_map_nao_vazio_e_contem_blocos_chave() {
    let report = build_baseline_report();
    assert!(!report.fidelity.is_empty(), "mapa de fidelidade não deveria estar vazio");
    for key in ["wing", "propulsion", "structure", "mission"] {
        assert!(report.fidelity.contains_key(key), "fidelity deveria conter a chave '{key}'");
    }
}

#[test]
fn sizing_report_reflete_mtow_missao_e_envelope() {
    let report = build_baseline_report();
    let sizing = report.sizing.as_ref().expect("sizing deveria estar presente");
    assert!(sizing.mtow_mission_kg > 0.0);
    assert!(sizing.mtow_envelope_kg >= sizing.mtow_mission_kg,
        "MTOW envelope (pior caso legal) deveria ser >= MTOW de missão");
    assert!(sizing.converged);
    assert!(sizing.iterations.len() >= 2);
    assert!(sizing.fuel_margin_l >= 0.0, "baseline deveria convergir com margem de combustível não-negativa");
}

#[test]
fn round_trip_serde_preserva_campos_chave() {
    let report = build_baseline_report();
    let json = serde_json::to_string(&report).expect("deveria serializar");
    let back: AircraftReport = serde_json::from_str(&json).expect("deveria desserializar de volta");

    // Cobertura do OBJETO INTEIRO (achado da revisão: os asserts abaixo só
    // comparavam alguns campos escolhidos a dedo — control_surfaces,
    // propeller, landing_gear, performance e vn_diagram nunca eram
    // checados). Reserializar `back` e comparar a string JSON byte a byte
    // contra a original cobre TODOS os blocos de uma vez — só é exato
    // graças à feature `float_roundtrip` do serde_json (Cargo.toml),
    // sem a qual esta asserção falharia por ruído de último-bit em
    // pontos flutuantes mesmo com dados logicamente idênticos.
    let json2 = serde_json::to_string(&back).expect("deveria reserializar");
    assert_eq!(json, json2,
        "reserializar o AircraftReport desserializado deveria produzir o \
         MESMO JSON byte a byte (round-trip completo, todos os blocos)");

    assert_eq!(back.schema_version, report.schema_version);
    assert_eq!(back.revision, report.revision);
    assert_eq!(back.validation_status, report.validation_status);
    assert_eq!(back.wing.span_m, report.wing.span_m);
    assert_eq!(back.propulsion.engine_model, report.propulsion.engine_model);
    assert_eq!(back.violations.len(), report.violations.len());
    assert_eq!(back.warnings.len(), report.warnings.len());
    assert_eq!(back.fidelity.len(), report.fidelity.len());

    let g_before = report.geometry.as_ref().unwrap();
    let g_after = back.geometry.as_ref().unwrap();
    assert_eq!(g_before.mac_m, g_after.mac_m);
    assert_eq!(g_before.fuselage_length_m, g_after.fuselage_length_m);

    let s_before = report.sizing.as_ref().unwrap();
    let s_after = back.sizing.as_ref().unwrap();
    assert_eq!(s_before.mtow_mission_kg, s_after.mtow_mission_kg);
    assert_eq!(s_before.iterations, s_after.iterations);
    assert_eq!(s_before.constraints.ws_actual_n_m2, s_after.constraints.ws_actual_n_m2);
}

/// Achado da própria checagem de round-trip acima (Task 6.1):
/// `StructuralSpec::fatigue_life_cycles` pode ser `f64::INFINITY`
/// (fisicamente correto — "vida infinita" abaixo do limite de fadiga).
/// `serde_json` serializa `Infinity` como `null` por padrão, e `null` não
/// desserializa de volta em `f64` — um consumidor de CAD rodando o schema
/// oficial quebraria sempre que a longarina caísse abaixo do limite de
/// fadiga. Confirma que o campo agora serializa como a string `"infinita"`,
/// não `null`, e volta corretamente para `f64::INFINITY`.
#[test]
fn fatigue_life_infinita_serializa_como_string_nao_null_e_faz_round_trip() {
    let report = build_baseline_report();
    let struc = report.structure.as_ref().expect("structure deveria estar presente");
    assert!(struc.fatigue_life_cycles.is_infinite(),
        "baseline real deveria ter vida em fadiga infinita (abaixo do limite Se) — \
         obteve {:.3e}; se isso mudou legitimamente, ajustar este teste",
        struc.fatigue_life_cycles);

    let json = serde_json::to_string(&report).expect("deveria serializar");
    assert!(json.contains("\"fatigue_life_cycles\":\"infinita\""),
        "esperava fatigue_life_cycles serializado como a string \"infinita\", \
         não null nem um número");
    assert!(!json.contains("\"fatigue_life_cycles\":null"),
        "fatigue_life_cycles NUNCA deveria serializar como null (não desserializa de volta em f64)");

    let back: AircraftReport = serde_json::from_str(&json).expect("deveria desserializar de volta");
    assert!(back.structure.unwrap().fatigue_life_cycles.is_infinite());
}
