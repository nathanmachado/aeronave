//! Ciclo 16, Task 1 — o pipeline extraído produz o MESMO relatório que o
//! artefato commitado. Não é redundante com `tests/cli.rs:943`: aquele roda
//! o BINÁRIO; este chama `pipeline::executa` direto, e é o que garante que a
//! função extraída (a que `validation::incerteza` vai chamar em laço) não
//! divergiu do caminho que gera o artefato.

use std::path::PathBuf;
use aeronave::models::config::{load_aircraft, load_engine, load_mission};

fn config_path(rel: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(rel)
}

#[test]
fn pipeline_executa_reproduz_o_artefato_commitado() {
    let cfg = load_aircraft(&config_path("config/aircraft/baseline_4seat.toml")).unwrap();
    let engine = load_engine(&config_path("config/engines/default.toml")).unwrap();
    let req = load_mission(&config_path("config/missions/default.toml")).unwrap();

    let res = aeronave::pipeline::executa(&cfg, &engine, &req)
        .expect("pipeline do baseline tem que convergir");

    let commitado: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string("aircraft_spec.json").unwrap()).unwrap();

    // Ancoras suficientes para pegar divergência de caminho, com tolerância
    // ZERO — os dois lados vêm do mesmo pipeline determinístico.
    assert_eq!(res.perf.climb_gradient_pct,
               commitado["performance"]["climb_gradient_pct"].as_f64().unwrap());
    assert_eq!(res.report.violations.len(),
               commitado["violations"].as_array().unwrap().len());
    assert_eq!(res.portoes.len(), 9);
}
