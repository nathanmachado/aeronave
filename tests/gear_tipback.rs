//! Teste de integração: tipback/tail-strike + carga de nariz nos dois
//! extremos (Task 2, refino-ciclo2) contra a aeronave-base REAL
//! (`config/aircraft/baseline_4seat.toml`), carregada do disco — mesmo
//! padrão de `tests/empennage.rs`/`tests/schema_v4.rs`: exercitar o
//! pipeline completo (`size_aircraft` + `LandingGearAgent`), não uma
//! fixture sintética.
//!
//! Números pinados abaixo são HONESTOS (calculados pelo pipeline real),
//! não os hand-checks aproximados do brief da task
//! (`task-2-brief.md`), que usavam CGs de cenário arredondados a partir de
//! %MAC (ex.: "x_cg aft ≈ 3.35 m (36.1% MAC)"). O runtime real da
//! aeronave-base (pós Task 1: `[gear].x_main_m=3.55m`, limite de rotação
//! recuado a 6.10% MAC) dá:
//!   - x_cg mais dianteiro real (cenário "Solo (piloto)"): 3.094768 m
//!     (15.6275% MAC) — brief hand-check usava ≈3.077 m (14.2% MAC).
//!   - x_cg mais traseiro real (cenário "4 pax + bagagem + cheio"):
//!     3.363323 m (37.1754% MAC) — brief hand-check usava ≈3.35 m
//!     (36.1% MAC).
//! O delta vem só de o brief ter sido escrito contra números arredondados
//! de %MAC; a física (fórmulas) é idêntica — ver `agents::landing_gear`.

use std::path::PathBuf;

use aeronave::agents::landing_gear::LandingGearAgent;
use aeronave::models::config::{load_aircraft, load_engine, load_mission};
use aeronave::orchestrator::size_aircraft;
use aeronave::validation::constraint_checker::ConstraintChecker;

fn config_path(rel: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(rel)
}

/// Monta o `GearSpec` real do baseline (Toyota + missão default) — CG mais
/// dianteiro/traseiro vêm dos CENÁRIOS REAIS de carga (`WeightBalanceAgent`
/// via `size_aircraft`), não do limite admissível do envelope de CG (ver
/// docstring de `LandingGearAgent::run`).
fn gear_real() -> aeronave::models::specs::GearSpec {
    let cfg = load_aircraft(&config_path("config/aircraft/baseline_4seat.toml")).unwrap();
    let engine = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let req = load_mission(&config_path("config/missions/default.toml")).unwrap();
    let sized = size_aircraft(&cfg, &engine, &req)
        .expect("aeronave-base + Toyota deveria convergir com o tanque de 260 L");
    let wb = &sized.wb;
    let x_cg_fwd = wb.mac_le_x_m + wb.spec.cg_mac_fwd_pct / 100.0 * wb.mac_m;
    let x_cg_aft = wb.mac_le_x_m + wb.spec.cg_mac_aft_pct / 100.0 * wb.mac_m;
    let mass_main_total = cfg.masses.item_mass("trem_principal").unwrap();
    let mass_nose = cfg.masses.item_mass("trem_nariz").unwrap();
    LandingGearAgent::run(wb.spec.mtow_kg, x_cg_fwd, x_cg_aft, &cfg.gear, mass_main_total, mass_nose)
}

/// Achado honesto ESPERADO (Task 2, refino-ciclo2): o trem principal foi
/// recuado pela Task 1/campanha E1–E6 (`x_main_m` 3.85→3.55m) para abrir o
/// envelope de CG via autoridade de rotação — o preço é um ângulo de
/// tipback abaixo do piso de 15° (Raymer cap. 11). NÃO tunar
/// `[gear].tipback_min_deg` nem `x_main_m` para mascarar este resultado —
/// é um achado de projeto genuíno (ver `ConstraintChecker::verify` #15 e
/// `tests/cli.rs::engine_padrao_explicito_com_out_tempfile_converge_e_
/// reporta_fail_honesto_de_tipback`).
#[test]
fn tipback_do_baseline_real_fica_abaixo_do_piso_pin_honesto() {
    let gear = gear_real();
    println!("θ tipback (baseline real) = {:.4}°", gear.tipback_angle_deg);
    // old (brief hand-check, pré-precisão): ≈10.8° — new (runtime honesto): ≈10.08°
    assert!((gear.tipback_angle_deg - 10.08).abs() < 0.05,
        "θ tipback = {:.4}° — pin honesto esperado ≈10.08° (tolerância ±0.05°)",
        gear.tipback_angle_deg);
    assert!(gear.tipback_angle_deg < 15.0,
        "achado honesto esperado: θ={:.2}° deveria ficar ABAIXO do piso de 15° \
         (tipback_min_deg) — NÃO É BUG", gear.tipback_angle_deg);
}

/// Tail-strike do baseline real: independe do CG (só geometria de
/// `[gear]`), então bate quase exatamente com o hand-check do brief —
/// `x_main_m=3.55` é o MESMO valor usado no brief.
#[test]
fn tail_strike_do_baseline_real_satisfaz_o_piso_pin_honesto() {
    let gear = gear_real();
    println!("Folga tail-strike (baseline real) = {:.4}°", gear.tail_strike_margin_deg);
    // old (brief hand-check) == new (runtime honesto): 14.5° — geometria
    // independe do CG, só de gear.tail_cone_x_m/height_m/x_main_m.
    assert!((gear.tail_strike_margin_deg - 14.51).abs() < 0.05,
        "folga tail-strike = {:.4}° — pin honesto esperado ≈14.51° (tolerância ±0.05°)",
        gear.tail_strike_margin_deg);
    assert!(gear.tail_strike_margin_deg >= 11.0,
        "folga tail-strike deveria satisfazer o piso de 11° (rotation_attitude_deg)");
}

/// Carga de nariz nos dois extremos reais dos cenários — pin honesto.
///
/// Investigação pedida pelo brief: "o 8.7% reportado usava outro CG —
/// investigar e documentar qual". RESPOSTA: usava o MESMO CG mais traseiro
/// real (`wb.spec.cg_mac_aft_pct`, cenário "4 pax + bagagem + cheio") que
/// hoje alimenta `nose_load_min_pct` — não é outro CG, é o MESMO cálculo,
/// só renomeado/reclassificado como o extremo MÍNIMO (piso de 8%) em vez
/// de ser o único valor reportado. O que é genuinamente NOVO nesta task é
/// `nose_load_max_pct` (CG mais dianteiro, teto de 25%) — nunca antes
/// calculado.
#[test]
fn carga_de_nariz_dois_extremos_do_baseline_real_pin_honesto() {
    let gear = gear_real();
    println!("nose_load_max_pct={:.4}%  nose_load_min_pct={:.4}%",
             gear.nose_load_max_pct, gear.nose_load_min_pct);
    // old (brief hand-check, CG dianteiro≈3.077m): ≈22.0% — new (runtime
    // honesto, CG dianteiro=3.094768m): ≈21.17%.
    assert!((gear.nose_load_max_pct - 21.17).abs() < 0.1,
        "nose_load_max_pct = {:.4}% — pin honesto esperado ≈21.17% (tolerância ±0.1%)",
        gear.nose_load_max_pct);
    assert!(gear.nose_load_max_pct <= 25.0, "nose_load_max_pct deveria satisfazer o teto de 25%");
    // old (relatado antes desta task como "Carga nariz: 8.7%", único
    // valor, mesmo cálculo) == new (agora nose_load_min_pct): ≈8.68%.
    assert!((gear.nose_load_min_pct - 8.68).abs() < 0.05,
        "nose_load_min_pct = {:.4}% — pin honesto esperado ≈8.68% (tolerância ±0.05%)",
        gear.nose_load_min_pct);
    assert!(gear.nose_load_min_pct >= 8.0, "nose_load_min_pct deveria satisfazer o piso de 8%");
}

/// `ConstraintChecker::verify` contra o baseline real: exatamente UMA
/// violação nova de trem de pouso (tipback) — tail-strike e carga de
/// nariz (dois extremos) PASSAM. Regressão de ponta a ponta (não só as
/// funções puras acima) — confirma a fiação real em `main.rs`.
#[test]
fn constraint_checker_reporta_so_tipback_entre_as_tres_checagens_novas() {
    let cfg = load_aircraft(&config_path("config/aircraft/baseline_4seat.toml")).unwrap();
    let engine = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let req = load_mission(&config_path("config/missions/default.toml")).unwrap();
    let sized = size_aircraft(&cfg, &engine, &req).unwrap();

    let wing = &sized.wing;
    let prop = &sized.prop;
    let wb = &sized.wb;
    let mission = &sized.mission;

    let propeller = aeronave::agents::propeller::PropellerAgent::run(&cfg, &engine, prop, &req);
    let perf = aeronave::agents::performance::PerformanceAgent::run(
        &sized.state, wing, prop, sized.state.mtow_kg, &engine, &req, &cfg.performance,
    );
    let electrical = aeronave::agents::electrical::ElectricalAgent::run(&cfg);
    let gear = gear_real();

    let report = ConstraintChecker::verify(
        &req, wing, prop, sized.state.mtow_kg, &engine, wb, &propeller, &perf, mission,
        &electrical, &gear, &cfg.gear, cfg.fuel_system.capacity_l,
    );

    assert!(report.violations.iter().any(|v| v.starts_with("Tipback:")),
        "esperava violação de tipback, obteve: {:?}", report.violations);
    assert!(!report.violations.iter().any(|v| v.starts_with("Tail-strike:")),
        "não deveria haver violação de tail-strike no baseline real: {:?}", report.violations);
    assert!(!report.violations.iter().any(|v| v.starts_with("Carga de nariz:")),
        "não deveria haver violação de carga de nariz no baseline real: {:?}", report.violations);
}

/// Margem mínima de combustível (Task 3, refino-ciclo2, checagem #18 de
/// `ConstraintChecker::verify`) contra o baseline real (Toyota + missão de
/// projeto completa): achado honesto — margem ≈1,82% da capacidade do
/// tanque (260 L), abaixo do piso de 5% (`config/missions/default.toml`,
/// `min_fuel_margin_fraction`). NÃO tunar o tanque nem a missão para
/// mascarar este resultado — é o requisito novo funcionando (PASS→FAIL
/// honesto, ver `task-3-report.md`). Mesma folga física de
/// `tests/generic_engine.rs::margem_de_combustivel_no_mtow_convergido`
/// (que mede a MESMA margem com a convenção %-do-combustível-NECESSÁRIO,
/// não %-da-capacidade — ver nota de convenção nesse teste).
#[test]
fn margem_de_combustivel_do_baseline_real_fica_abaixo_do_piso_pin_honesto() {
    let cfg = load_aircraft(&config_path("config/aircraft/baseline_4seat.toml")).unwrap();
    let engine = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let req = load_mission(&config_path("config/missions/default.toml")).unwrap();
    let sized = size_aircraft(&cfg, &engine, &req)
        .expect("aeronave-base + Toyota deveria convergir com o tanque de 260 L");
    let mission = &sized.mission;

    let fuel_margin_pct = (cfg.fuel_system.capacity_l - mission.fuel_total_l)
        / cfg.fuel_system.capacity_l * 100.0;
    println!("margem de combustível (baseline real) = {fuel_margin_pct:.4}% da capacidade");
    // Pin honesto pós-Task-3: ≈1.8184%. Task 4 (refino-ciclo2, arrasto de
    // trim em cruzeiro) soma ΔCD_trim≈4.86e-5 ao polar de cruzeiro — mais
    // arrasto ⟹ mais combustível de cruzeiro (Breguet) ⟹ MTOW converge
    // levemente mais pesado (1544.43→1544.96 kg) ⟹ margem cai
    // **1.8184%→1.5767%** (old→new; ~0.63 L a mais de combustível exigido
    // sobre 260 L de tanque — pequeno, honesto, ver task-4-report.md).
    // Mesmo número (novo) pinado em `tests/generic_engine.rs`
    // (`sizing.fuel_margin_pct`).
    assert!((fuel_margin_pct - 1.5767).abs() < 0.05,
        "margem de combustível {fuel_margin_pct:.4}% divergiu do pin honesto pós-Task-4 \
         ≈1.5767%");
    assert!(fuel_margin_pct < 5.0,
        "achado honesto esperado: margem ({fuel_margin_pct:.2}%) deveria ficar ABAIXO do piso \
         de 5% (min_fuel_margin_fraction) — NÃO É BUG");

    let propeller = aeronave::agents::propeller::PropellerAgent::run(&cfg, &engine, &sized.prop, &req);
    let perf = aeronave::agents::performance::PerformanceAgent::run(
        &sized.state, &sized.wing, &sized.prop, sized.state.mtow_kg, &engine, &req,
        &cfg.performance,
    );
    let electrical = aeronave::agents::electrical::ElectricalAgent::run(&cfg);
    let gear = gear_real();

    let report = ConstraintChecker::verify(
        &req, &sized.wing, &sized.prop, sized.state.mtow_kg, &engine, &sized.wb, &propeller,
        &perf, mission, &electrical, &gear, &cfg.gear, cfg.fuel_system.capacity_l,
    );

    assert!(report.violations.iter().any(|v| v.contains("Margem de combustível")),
        "achado honesto esperado: margem real (~1,82%) abaixo do piso de 5% \
         (min_fuel_margin_fraction) deveria gerar violação, obteve: {:?}", report.violations);
}
