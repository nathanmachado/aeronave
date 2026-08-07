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
//!
//! ATUALIZAÇÃO (campanha E7, 2026-08-06): `[gear].x_main_m` 3.55→3.66m
//! (fecha o tipback, decisão de projeto — ver
//! `config/aircraft/baseline_4seat.toml`) e `mission.endurance_min_h`
//! 8.0→7.0h (decisão de requisito do cliente — ver
//! `config/missions/default.toml`) mudam os números honestos abaixo:
//! tipback sobe de ≈10.08° (abaixo do piso de 15°) para ≈15.58° (acima),
//! tail-strike/carga de nariz/margem de combustível se movem também (o
//! trem mais atrás e o MTOW de missão mais leve deslocam ligeiramente o
//! CG e a massa de todos os cenários — ver comentários em cada teste
//! abaixo). `validation_status` do baseline real vira `PASS`.

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
    // Ciclo 3 (oew-parametrico): as massas do trem vêm COMPUTADAS de
    // `SizedAircraft::structural_masses` (`agents::mass_model`), não mais
    // de itens fixos de `[[masses.items]]` — mesma fiação de `main.rs`.
    LandingGearAgent::run(
        wb.spec.mtow_kg, x_cg_fwd, x_cg_aft, &cfg.gear,
        sized.structural_masses.trem_principal_kg, sized.structural_masses.trem_nariz_kg,
    )
}

/// Achado honesto HISTÓRICO (Task 2, refino-ciclo2): o trem principal foi
/// recuado pela Task 1/campanha E1–E6 (`x_main_m` 3.85→3.55m) para abrir o
/// envelope de CG via autoridade de rotação — o preço era um ângulo de
/// tipback abaixo do piso de 15° (Raymer cap. 11). NÃO mascarado por
/// `[gear].tipback_min_deg`/`x_main_m` na época — era um achado de projeto
/// genuíno (ver `ConstraintChecker::verify` #15).
///
/// RESOLVIDO na campanha E7 (2026-08-06): `x_main_m` 3.55→3.66m recua o
/// trem mais um pouco, viável porque a autoridade de rotação DATCOM
/// (ciclo 2) alargou o limite dianteiro do envelope de CG para ≈13,0% MAC
/// nesta posição — fecha o tipback ACIMA do piso de 15° (ver comentário em
/// `config/aircraft/baseline_4seat.toml`). Renomeado (o nome antigo dizia
/// "fica_abaixo"). O caminho de erro (tipback abaixo do piso) continua
/// coberto por config sintética mutada em código — ver
/// `validation::constraint_checker::tests::violacao_de_tipback_aparece_
/// quando_abaixo_do_piso` (em `src/`, checagem #15 — sobrescreve só
/// `gear.tipback_angle_deg`, sem depender de nenhum `x_main_m` real).
#[test]
fn tipback_do_baseline_real_fecha_o_piso_pin_honesto() {
    let gear = gear_real();
    println!("θ tipback (baseline real) = {:.4}°", gear.tipback_angle_deg);
    // old (pré-E7, x_main=3.55m): ≈10.08° (abaixo do piso); E7
    // (x_main=3.66m): ≈15.58°. Ciclo 3 (oew-parametrico, massas
    // estruturais COMPUTADAS): o CG vazio AVANÇA e com ele o CG mais
    // TRASEIRO real dos cenários (37,5%→31,7% MAC), o que AUMENTA a
    // distância `(x_main − x_cg_aft)` e portanto FOLGA o tipback:
    // 15.58°→**≈19.17°** (old→new, tolerância INALTERADA).
    // Ciclo 4, Task 1 (t/c dedicado da empenagem, `[empennage].
    // thickness_ratio`, 2026-08-07): a cauda mais pesada (braço TRASEIRO)
    // RECUA o CG vazio e o CG mais traseiro real, o que REDUZ a distância
    // `(x_main − x_cg_aft)` e portanto aperta ligeiramente o tipback:
    // 19.17°→18.91° (old→new).
    // Ciclo 4, Task 2 (W_dg = MTOW de envelope com lag-1): `MassModelAgent
    // ::run` passa a usar o MTOW de ENVELOPE (~1.543,7kg) em vez do
    // candidato de MISSÃO (~1.505,6kg antes) como W_dg — estrutura
    // dimensionada para um peso maior fica um pouco mais pesada em TODOS
    // os componentes (efeito uniforme, não recua nem avança o CG
    // seletivamente), o que muda ligeiramente as massas/braços que
    // definem o CG vazio e portanto o CG mais traseiro real: 18.91°→
    // **≈18.85°** (old→new). Continua bem acima do piso de 15°.
    assert!((gear.tipback_angle_deg - 18.8533).abs() < 0.05,
        "θ tipback = {:.4}° — pin honesto esperado ≈18.8533° (tolerância ±0.05°)",
        gear.tipback_angle_deg);
    assert!(gear.tipback_angle_deg >= 15.0,
        "achado honesto esperado: θ={:.2}° deveria ficar NO piso de 15° ou acima \
         (tipback_min_deg) — folga ampliada pelo avanço do CG no ciclo 3", gear.tipback_angle_deg);
}

/// Tail-strike do baseline real: independe do CG (só geometria de
/// `[gear]`), então bate quase exatamente com o hand-check do brief —
/// `x_main_m=3.55` era o MESMO valor usado no brief.
///
/// ATUALIZAÇÃO (campanha E7, 2026-08-06): `x_main_m` 3.55→3.66m (fecha o
/// tipback) reduz a distância entre o trem principal e o cone de cauda
/// (`tail_cone_x_m` fixo), o que aumenta ligeiramente o ângulo de folga —
/// continua bem acima do piso de 11°.
#[test]
fn tail_strike_do_baseline_real_satisfaz_o_piso_pin_honesto() {
    let gear = gear_real();
    println!("Folga tail-strike (baseline real) = {:.4}°", gear.tail_strike_margin_deg);
    // old (pré-E7, x_main=3.55m): 14.51° — new (campanha E7, x_main=3.66m): ≈14.88°.
    assert!((gear.tail_strike_margin_deg - 14.88).abs() < 0.05,
        "folga tail-strike = {:.4}° — pin honesto esperado ≈14.88° (tolerância ±0.05°)",
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
/// ATUALIZAÇÃO (campanha E7, 2026-08-06): `x_main_m` 3.55→3.66m recua o
/// trem principal, o que reduz o braço `(x_main − x_nose)` na base da
/// fração de carga de nariz e aumenta AMBOS os extremos — nose_load_max
/// sobe para perto do teto de 25% (a folga que a autoridade de rotação
/// DATCOM alargada torna aceitável), nose_load_min sobe também mas segue
/// bem acima do piso de 8%.
#[test]
fn carga_de_nariz_dois_extremos_do_baseline_real_pin_honesto() {
    let gear = gear_real();
    println!("nose_load_max_pct={:.4}%  nose_load_min_pct={:.4}%",
             gear.nose_load_max_pct, gear.nose_load_min_pct);
    // old (pré-E7, x_main=3.55m): ≈21.17%; E7 (x_main=3.66m): ≈24.79%.
    // Ciclo 3 (oew-parametrico): o CG mais DIANTEIRO avança de 16,0% para
    // 8,3% MAC (estrutura redistribuída — ver o comentário do módulo), e
    // quanto mais perto do trem de nariz fica o CG, MAIOR a fração de
    // carga nele: 24.79%→**≈29.03%** (old→new, tolerância INALTERADA) —
    // ACIMA do teto de 25%. FAIL honesto, asserido abaixo, não mascarado.
    // Ciclo 4, Task 1 (t/c dedicado da empenagem): a cauda mais pesada
    // RECUA o CG mais dianteiro também (mesma causa física de todos os
    // pins deste ciclo — massa em braço TRASEIRO recua o CG), afastando-o
    // um pouco do trem de nariz: 29.03%→28.71% (old→new).
    // Ciclo 4, Task 2 (W_dg de envelope com lag-1): estrutura um pouco
    // mais pesada em todos os componentes (W_dg sobe do candidato de
    // missão ~1.505,6kg para o envelope ~1.543,7kg, ver docstring do
    // teste de tipback acima) desloca o CG vazio ligeiramente — o CG mais
    // DIANTEIRO real recua um pouco mais: 28.71%→**≈28.60%** (old→new).
    // Continua ACIMA do teto de 25% — achado honesto do ciclo 3 permanece.
    assert!((gear.nose_load_max_pct - 28.5966).abs() < 0.1,
        "nose_load_max_pct = {:.4}% — pin honesto esperado ≈28.5966% (tolerância ±0.1%)",
        gear.nose_load_max_pct);
    assert!(gear.nose_load_max_pct > 25.0,
        "achado honesto do ciclo 3: nose_load_max_pct = {:.2}% deveria EXCEDER o teto de 25% \
         (CG mais dianteiro avançou com a estrutura computada) — decisão de projeto para \
         revisão humana, não mascarada aqui", gear.nose_load_max_pct);
    // old (pré-E7, x_main=3.55m): ≈8.68%; E7: ≈12.95%. Ciclo 3: o CG mais
    // TRASEIRO também avançou (37,5%→31,7% MAC), subindo a carga MÍNIMA de
    // nariz: 12.95%→**≈16.15%** — continua bem acima do piso de 8%.
    // Ciclo 4, Task 1 (t/c dedicado da empenagem): CG mais traseiro RECUA
    // (cauda mais pesada), reduzindo a carga MÍNIMA de nariz: 16.15%→
    // 15.92% (old→new).
    // Ciclo 4, Task 2 (W_dg de envelope com lag-1): mesmo deslocamento
    // uniforme do CG vazio da nota acima reduz também a carga MÍNIMA de
    // nariz: 15.92%→**≈15.86%** (old→new). Continua bem acima do piso de
    // 8%.
    assert!((gear.nose_load_min_pct - 15.8646).abs() < 0.05,
        "nose_load_min_pct = {:.4}% — pin honesto esperado ≈15.8646% (tolerância ±0.05%)",
        gear.nose_load_min_pct);
    assert!(gear.nose_load_min_pct >= 8.0, "nose_load_min_pct deveria satisfazer o piso de 8%");
}

/// `ConstraintChecker::verify` contra o baseline real: HISTORICAMENTE
/// (pré-E7) exatamente UMA violação de trem de pouso (tipback) — tail-
/// strike e carga de nariz (dois extremos) já PASSAVAM.
///
/// RESOLVIDO na campanha E7 (2026-08-06): `gear.x_main_m` 3.55→3.66m fecha
/// também o tipback — as TRÊS checagens de trem de pouso (#15 tipback, #16
/// tail-strike, #17 carga de nariz) PASSAVAM no baseline real.
///
/// ATUALIZAÇÃO (ciclo 3 — oew-parametrico, 2026-08-07): a carga de nariz
/// (#17) volta a VIOLAR — ≈29,0% no CG mais dianteiro, acima do teto de
/// 25%, porque a estrutura COMPUTADA (`agents::mass_model`) avança o CG
/// vazio (ver o comentário do módulo). Tipback (#15, folgado a ≈19,17°) e
/// tail-strike (#16) continuam passando. Renomeado de novo (o nome
/// anterior dizia "nao_reporta_violacoes_de_trem"). Regressão de ponta a ponta
/// (não só as funções puras acima) — confirma a fiação real em `main.rs`.
/// O caminho de erro (violação de tipback) continua coberto por config
/// sintética mutada em código — ver
/// `validation::constraint_checker::tests::violacao_de_tipback_aparece_
/// quando_abaixo_do_piso` (em `src/`).
#[test]
fn constraint_checker_reporta_so_carga_de_nariz_como_violacao_de_trem_no_baseline_real() {
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

    assert!(!report.violations.iter().any(|v| v.starts_with("Tipback:")),
        "não deveria haver violação de tipback no baseline real pós-E7: {:?}", report.violations);
    assert!(!report.violations.iter().any(|v| v.starts_with("Tail-strike:")),
        "não deveria haver violação de tail-strike no baseline real: {:?}", report.violations);
    // Ciclo 3 (oew-parametrico): a carga de nariz PASSOU a violar — o CG
    // mais dianteiro avançou com a estrutura computada e a fração de carga
    // no nariz subiu para ≈29,0%, acima do teto de 25% (checagem #17).
    // FAIL honesto asserido (não mascarado): tipback e tail-strike
    // continuam PASSANDO, a carga de nariz não.
    // Ciclo 4, Task 2 (W_dg de envelope): ≈29,0%→**≈28,6%** (old→new, ver
    // `carga_de_nariz_dois_extremos_do_baseline_real_pin_honesto` acima) —
    // continua ACIMA do teto de 25%, mesmo achado honesto.
    assert!(report.violations.iter().any(|v| v.starts_with("Carga de nariz:")
            && v.contains("excede o teto")),
        "achado honesto do ciclo 3/4: deveria haver violação de carga de nariz (≈28,6% > 25,0%) \
         no baseline real: {:?}", report.violations);
}

/// Margem mínima de combustível (Task 3, refino-ciclo2, checagem #18 de
/// `ConstraintChecker::verify`) contra o baseline real (Toyota + missão de
/// projeto completa): achado honesto HISTÓRICO — margem ≈1,58% da
/// capacidade do tanque (260 L), abaixo do piso de 5%
/// (`config/missions/default.toml`, `min_fuel_margin_fraction`). NÃO
/// mascarado por tanque/missão na época — era o requisito novo funcionando
/// (PASS→FAIL honesto, ver `task-3-report.md`).
///
/// RESOLVIDO na campanha E7 (2026-08-06): `mission.endurance_min_h`
/// 8,0→7,0h (decisão de requisito do cliente — autonomia 7h + reserva, ver
/// comentário em `config/missions/default.toml`) reduz o combustível
/// exigido pela missão (255,9→≈223,7 L) — margem sobe para ≈13,97% da
/// capacidade, bem acima do piso de 5%. Renomeado (o nome antigo dizia
/// "fica_abaixo"). Mesma folga física de
/// `tests/generic_engine.rs::margem_de_combustivel_no_mtow_convergido`
/// (que mede a MESMA margem com a convenção %-do-combustível-NECESSÁRIO,
/// não %-da-capacidade — ver nota de convenção nesse teste). O caminho de
/// erro (margem abaixo do piso) continua coberto por configs sintéticas
/// mutadas em código — ver
/// `validation::constraint_checker::tests::violacao_de_margem_de_
/// combustivel_aparece_quando_abaixo_do_minimo` (em `src/`).
#[test]
fn margem_de_combustivel_do_baseline_real_fica_acima_do_piso_pin_honesto() {
    let cfg = load_aircraft(&config_path("config/aircraft/baseline_4seat.toml")).unwrap();
    let engine = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let req = load_mission(&config_path("config/missions/default.toml")).unwrap();
    let sized = size_aircraft(&cfg, &engine, &req)
        .expect("aeronave-base + Toyota deveria convergir com o tanque de 260 L");
    let mission = &sized.mission;

    let fuel_margin_pct = (cfg.fuel_system.capacity_l - mission.fuel_total_l)
        / cfg.fuel_system.capacity_l * 100.0;
    println!("margem de combustível (baseline real) = {fuel_margin_pct:.4}% da capacidade");
    // Pin honesto pós-Task-4 (pré-E7): ≈1.5767%; campanha E7
    // (endurance_min_h 8h→7h): ≈13.9712%. Ciclo 3 (oew-parametrico):
    // 13.9712→14.5581% (old→new, tolerância INALTERADA) — a estrutura
    // computada deixa a aeronave ~11 kg mais leve no OEW, o que reduz o
    // MTOW de missão convergido (1.517,9→1.505,6 kg) e o combustível
    // exigido (223,7→222,1 L) com a capacidade do tanque inalterada.
    // Ciclo 4, Task 2 (W_dg de envelope com lag-1): W_dg sobe do candidato
    // de missão (~1.505,6kg) para o envelope (~1.543,7kg) — estrutura mais
    // pesada em todos os componentes aumenta o OEW, o MTOW de missão
    // convergido e o combustível exigido: 14.5581%→**≈14.3273%**
    // (old→new). Continua bem acima do piso de 5%.
    assert!((fuel_margin_pct - 14.3273).abs() < 0.1,
        "margem de combustível {fuel_margin_pct:.4}% divergiu do pin honesto pós-ciclo-4 \
         ≈14.3273%");
    assert!(fuel_margin_pct >= 5.0,
        "achado honesto esperado (campanha E7): margem ({fuel_margin_pct:.2}%) deveria ficar NO \
         piso de 5% (min_fuel_margin_fraction) ou acima — resolvido por endurance_min_h 8h→7h");

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

    assert!(!report.violations.iter().any(|v| v.contains("Margem de combustível")),
        "não deveria haver violação de margem de combustível no baseline real pós-E7: {:?}",
        report.violations);
}
