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
//!
//! ATUALIZAÇÃO (campanha E10, 2026-08-08): a bateria híbrida de 53 kg a
//! 7,80 m, `[gear].x_nose_m` 1,40→1,30 e `[gear].h_cg_ground_m` 1,05→0,92
//! (ver `config/aircraft/baseline_4seat.toml`) recuam o CG de todos os
//! cenários ≈+6,5 pp MAC e diluem a carga de nariz. Consequências nos pins
//! deste arquivo: tipback 18,85°→16,74°, carga de nariz máx 28,60%→22,77%
//! (deixa de violar o teto de 25%), mín 15,86%→11,72%, margem de
//! combustível 14,33%→9,14%. `validation_status` volta a `PASS` — agora com
//! ZERO violações E zero flips de robustez (primeiro PASS completo).
//!
//! ATUALIZAÇÃO (ciclo 9, transferência de atitude do #25, 2026-08-09 —
//! old→new): o "PASS completo" acima NÃO sobrevive a este ciclo.
//! `PropellerSpec::fill_critical_clearance` corrige a simplificação
//! conhecida do ciclo 8 (translação vertical 1:1 do nariz — `docs/
//! backlog.md` item 1) para o pivô real da célula sobre o trem principal;
//! no baseline real o fator de amplificação resultante (≈1,46610) reprova
//! a checagem #25 (folga crítica de hélice, CS 23.925): `prop_clearance_
//! critical_m` +0,0325 m (PASS) → ≈−0,06416 m (FAIL). `validation_status`
//! volta a `FAIL` com EXATAMENTE 1 violação nomeada — todos os outros pins
//! deste arquivo (tipback/tail-strike/carga de nariz/margem de
//! combustível/robustez) permanecem INALTERADOS, ver
//! `constraint_checker_sem_violacoes_de_trem_nem_de_robustez_no_baseline_
//! real` abaixo.

use std::path::PathBuf;

use aeronave::agents::landing_gear::LandingGearAgent;
use aeronave::models::config::{load_aircraft, load_engine, load_mission};
use aeronave::orchestrator::size_aircraft;
use aeronave::validation::constraint_checker::{ConstraintChecker, VerifyInputs};

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
/// nesta posição (valor da época, ciclos 2-6; o ciclo 7/`cl_max_to` recuou
/// esse limite ainda mais, para 8,908% MAC — MAIS folga, não menos; a
/// conclusão de viabilidade abaixo fica FORTALECIDA) — fecha o tipback
/// ACIMA do piso de 15° (ver comentário em
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
    // Campanha E10 (2026-08-08): DOIS efeitos opostos, saldo negativo mas
    // ainda acima do piso. (a) `[gear].h_cg_ground_m` 1,05→0,92 baixa o CG
    // 13 cm, o que AUMENTA o tipback (arctan do braço sobre a altura);
    // (b) a bateria de 53 kg a 7,80 m RECUA o CG mais traseiro real
    // (32,2%→38,8% MAC), o que ENCURTA `(x_main − x_cg_aft)` e derruba o
    // tipback bem mais do que (a) recupera: 18.85°→**≈16.74°** (old→new,
    // tolerância INALTERADA). Segue acima do piso de 15° — e agora também
    // nos DOIS mundos adversariais de ±15% de massa (0 flips, era o objetivo
    // de (a)); antes de E10 o tipback nominal era mais folgado mas os
    // cenários dianteiros flipavam sob σ.
    assert!((gear.tipback_angle_deg - 16.7356).abs() < 0.05,
        "θ tipback = {:.4}° — pin honesto esperado ≈16.7356° (tolerância ±0.05°)",
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
///
/// CORREÇÃO (revisão final, campanha E10, 2026-08-08): a rodada original de
/// E10 baixou a célula inteira 13 cm (`[gear].h_cg_ground_m` 1,05→0,92) sem
/// baixar `[gear].tail_cone_height_m` junto — o campo é a altura do FUNDO
/// do cone acima do SOLO em atitude ESTÁTICA (ver
/// `agents::landing_gear::tail_strike_margin_deg`), não uma dimensão
/// independente do trem, então ficou 13 cm ALTO DEMAIS por engano (pin
/// antigo, nunca correto: 14,88°, herdado sem mudança da campanha E7).
/// Corrigido para 0,97 m (era 1,10 m,
/// ver `config/aircraft/baseline_4seat.toml`): tail-strike recalculado
/// 14,88°→**≈13,1865°** (old→new) — folga MENOR (o cone de cauda está mais
/// perto do solo de verdade), mas ainda bem acima do piso de 11°.
#[test]
fn tail_strike_do_baseline_real_satisfaz_o_piso_pin_honesto() {
    let gear = gear_real();
    println!("Folga tail-strike (baseline real) = {:.4}°", gear.tail_strike_margin_deg);
    // old (pré-E7, x_main=3.55m): 14.51° — E7 (x_main=3.66m): ≈14.88° — E10,
    // corrigido na revisão final (tail_cone_height_m 1.10→0.97): ≈13.1865°.
    assert!((gear.tail_strike_margin_deg - 13.1865).abs() < 0.05,
        "folga tail-strike = {:.4}° — pin honesto esperado ≈13.1865° (tolerância ±0.05°)",
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
    //
    // RESOLVIDO na campanha E10 (2026-08-08) — o achado honesto do ciclo 3
    // deixa de ocorrer, por DUAS mudanças de projeto somadas (não por
    // afrouxamento: o teto de 25% e a tolerância ±0.1 seguem os mesmos):
    //   (a) `[gear].x_nose_m` 1,40→1,30 alonga o braço `(x_main − x_nose)`
    //       que aparece no denominador da fração de carga de nariz;
    //   (b) a bateria híbrida de 53 kg a 7,80 m RECUA o CG mais dianteiro
    //       real de 9,1% para 17,9% MAC, afastando-o do trem de nariz.
    // 28.60%→**≈22.77%** (old→new). A ASSERÇÃO INVERTE: o teste passa a
    // exigir que a carga fique ABAIXO do teto. O caminho de erro (carga de
    // nariz acima do teto) segue coberto por config sintética mutada — ver
    // `validation::constraint_checker::tests` (checagem #17) em `src/`.
    assert!((gear.nose_load_max_pct - 22.7693).abs() < 0.1,
        "nose_load_max_pct = {:.4}% — pin honesto esperado ≈22.7693% (tolerância ±0.1%)",
        gear.nose_load_max_pct);
    assert!(gear.nose_load_max_pct <= 25.0,
        "campanha E10: nose_load_max_pct = {:.2}% deveria ficar NO teto de 25% ou abaixo \
         (x_nose_m 1,40→1,30 + bateria de 53 kg no cone de cauda) — o FAIL honesto do ciclo 3 \
         foi resolvido por projeto, não mascarado", gear.nose_load_max_pct);
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
    // Campanha E10 (2026-08-08): o CG mais TRASEIRO recua (32,2%→38,8% MAC,
    // bateria de 53 kg no cone de cauda) e o braço `(x_main − x_nose)` cresce
    // (x_nose 1,40→1,30) — ambos reduzem a carga MÍNIMA de nariz:
    // 15.86%→**≈11.72%** (old→new, tolerância INALTERADA). Continua acima do
    // piso de 8%, mas com bem menos folga: é o extremo do envelope que E10
    // aperta (o outro extremo, o teto de 25%, é o que ela abre). O check #19
    // (robustez ±15% de massa) confirma 0 flips também neste extremo.
    assert!((gear.nose_load_min_pct - 11.7219).abs() < 0.05,
        "nose_load_min_pct = {:.4}% — pin honesto esperado ≈11.7219% (tolerância ±0.05%)",
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
///
/// ATUALIZAÇÃO (campanha E10, 2026-08-08): as TRÊS checagens de trem voltam
/// a passar E as duas violações de ROBUSTEZ (#19) desaparecem — o baseline
/// real passa a reportar ZERO violações de qualquer tipo (primeiro PASS
/// completo do projeto). Renomeado de novo (o nome anterior dizia
/// "reporta_so_carga_de_nariz"). O que mudou, por violação:
///   - Carga de nariz (#17): 28,60%→22,77%, abaixo do teto de 25%
///     (`x_nose_m` 1,40→1,30 + bateria de 53 kg a 7,80 m).
///   - Robustez (#19), 'Solo (piloto)' e '2 pax dianteiros': o CG desses
///     cenários recua de 9,1%/12,5% para 17,9%/20,5% MAC, enquanto o limite
///     de rotação praticamente não se move (8,91%→8,53%) — a margem sai de
///     0,4%/7,4% para 21,6%/29,4% e sobrevive aos dois mundos de ±15% de
///     massa estrutural. `RobustnessAgent` reporta 0 flips.
/// Este teste inverte de "asserir os FAILs honestos por nome" para "asserir
/// que não há NENHUMA violação"; a cobertura dos caminhos de erro continua
/// nas configs sintéticas mutadas de `src/validation/constraint_checker.rs`
/// (checks #15/#17/#19) e de `src/validation/robustness.rs`.
///
/// CORREÇÃO (revisão final): até esta rodada a afirmação acima era FALSA
/// para o check #17 (carga de nariz) — nenhum teste sintético cobria os
/// dois ramos (teto de 25% / piso de 8%) desde que o baseline real
/// inverteu para PASS, buraco confirmado por mutação manual (`if false &&`
/// em cada ramo não quebrava nenhum teste). Agora coberto por
/// `constraint_checker::tests::violacao_de_carga_de_nariz_aparece_quando_
/// max_acima_do_teto`/`..._quando_min_abaixo_do_piso` (mais os dois
/// negativos gêmeos) — a afirmação passa a ser verdadeira.
///
/// ATUALIZAÇÃO (ciclo 9, transferência de atitude do #25 — O TESTE INVERTE
/// DE NOVO, parcialmente): a checagem #25 (folga de hélice em condição
/// crítica) volta a REPROVAR no baseline real — física corrigida, não
/// regressão (fator de amplificação do pivô sobre o trem principal, ver
/// `docs/backlog.md` item 1). O nome do teste ("sem_violacoes... ") fica
/// tecnicamente impreciso mas NÃO é renomeado aqui — as asserções por check
/// nomeado (tipback/tail-strike/carga de nariz/robustez) continuam todas
/// verdadeiras, só o total deixa de ser zero. O caminho PASS de #25
/// continua coberto por `constraint_checker::tests::check_25_sem_
/// violacao_na_fixture_padrao` (fixture sintética).
#[test]
fn constraint_checker_sem_violacoes_de_trem_nem_de_robustez_no_baseline_real() {
    let cfg = load_aircraft(&config_path("config/aircraft/baseline_4seat.toml")).unwrap();
    let engine = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let req = load_mission(&config_path("config/missions/default.toml")).unwrap();
    let sized = size_aircraft(&cfg, &engine, &req).unwrap();

    let wing = &sized.wing;
    let emp = &sized.emp;
    let prop = &sized.prop;
    let wb = &sized.wb;
    let mission = &sized.mission;

    let mut propeller = aeronave::agents::propeller::PropellerAgent::run(&cfg, &engine, prop, &req);
    let perf = aeronave::agents::performance::PerformanceAgent::run(
        &sized.state, wing, prop, sized.state.mtow_kg, &engine, &req, &cfg.performance,
    );
    let electrical = aeronave::agents::electrical::ElectricalAgent::run(&cfg);
    let gear = gear_real();
    // Ciclo 8 (task 2): preenche `prop_clearance_critical_m` (checagem #25)
    // no MESMO caminho de `main.rs` — depois que `gear` existe.
    propeller.fill_critical_clearance(&gear, &cfg.gear, &cfg.propeller);
    // Ciclo 4 (task robustez, wiring): `RobustnessSpec` na MESMA sequência
    // de `main.rs`, contra os limites NOMINAIS já calculados (`wb`/`gear`).
    let robustness = aeronave::validation::robustness::RobustnessAgent::run(
        &cfg, &engine, &req, &sized.state, wing, emp, &sized.structural_masses, wb, &gear,
        &propeller, mission, &perf,
    );

    let report = ConstraintChecker::verify(&VerifyInputs {
        req: &req, wing, prop, mtow_kg: sized.state.mtow_kg, engine: &engine, wb,
        propeller: &propeller, perf: &perf, mission, electrical: &electrical,
        gear: &gear, gear_cfg: &cfg.gear, fuel_capacity_l: cfg.fuel_system.capacity_l,
        robustness: &robustness, prop_cfg: &cfg.propeller,
    });

    assert!(!report.violations.iter().any(|v| v.starts_with("Tipback:")),
        "não deveria haver violação de tipback no baseline real pós-E7: {:?}", report.violations);
    assert!(!report.violations.iter().any(|v| v.starts_with("Tail-strike:")),
        "não deveria haver violação de tail-strike no baseline real: {:?}", report.violations);
    // Ciclo 3 (oew-parametrico): a carga de nariz PASSOU a violar (≈29,0%,
    // depois ≈28,6% no ciclo 4) — FAIL honesto asserido por nome aqui até a
    // campanha E9. E10 (2026-08-08) resolve por PROJETO (`x_nose_m`
    // 1,40→1,30 + bateria de 53 kg a 7,80 m): 28,6%→22,77%, abaixo do teto.
    // A asserção INVERTE — ver docstring.
    assert!(!report.violations.iter().any(|v| v.starts_with("Carga de nariz:")),
        "campanha E10: não deveria haver violação de carga de nariz (≈22,77% ≤ 25,0%) no \
         baseline real: {:?}", report.violations);
    // Checagem #19 (robustez à incerteza de massa estrutural, σ=15% =
    // `[mass_model].sigma_mass_fraction`): no ciclo 7 havia EXATAMENTE duas
    // violações — 'Solo (piloto)' (4,55 vs limite 8,91 %MAC) e '2 pax
    // dianteiros' (8,46 vs 8,91), cenários que passavam no nominal com
    // margem de rotação apertadíssima (0,4% e 7,4%) e reprovavam sob ±15%
    // de massa. A campanha E10 recua o CG desses cenários para 17,9%/20,5%
    // MAC (bateria de 53 kg no cone de cauda) enquanto o limite de rotação
    // fica praticamente parado (8,908%→8,533%): as margens sobem para 21,6%
    // e 29,4% e sobrevivem aos dois mundos adversariais. Quem fecha os
    // flips é o CG dos CENÁRIOS, não o limite — este é invariante ao peso e
    // ao CG (não recebe nenhum dos dois, ver
    // `agents::trim_authority::rotation_fwd_limit_m`) e se move só pelos
    // 0,375 pp de saldo entre `cl_max_to` 1,585→1,6775 (recuaria sozinho
    // para 11,78%) e `Cm_TO` −0,158→−0,113 (avançaria sozinho para 5,47%),
    // ambos governados por `to_flap_fraction` 0,5→0,35.
    // `RobustnessAgent` reporta **0 flips** (era 2).
    // A cobertura do caminho de erro de #19 continua nas configs sintéticas
    // marginais de `src/validation/robustness.rs` (`config_marginal_gera_
    // flip_nomeado`, `envelope_no_mundo_massa_total_flipa_quando_marginal`,
    // `carga_de_nariz_no_mundo_massa_total_flipa_quando_marginal`, …) e em
    // `constraint_checker::tests::check_19_transforma_flips_em_violacoes_
    // nomeadas`.
    let robustez: Vec<&String> = report.violations.iter()
        .filter(|v| v.starts_with("Robustez:")).collect();
    assert!(robustez.is_empty(),
        "campanha E10: esperava ZERO violações de robustez (σ=15%) no baseline real — eram 2 \
         (Solo/2 pax) até o ciclo 7: {:?}", report.violations);
    // ATUALIZAÇÃO (ciclo 9, transferência de atitude do #25 — old→new): a
    // afirmação abaixo ("o baseline real não deve reportar NENHUMA
    // violação") era o PASS completo da campanha E10 — verdadeira até este
    // ciclo. `PropellerSpec::fill_critical_clearance` ganha o fator de
    // amplificação do pivô sobre o trem principal (não mais uma translação
    // vertical 1:1 do nariz — ver docstring do campo e `docs/backlog.md`
    // item 1, RESOLVIDO): no baseline real o fator (≈1,46610) vira a folga
    // crítica de +0,0325 m para ≈−0,06416 m — checagem #25 REPROVA. Nenhuma
    // OUTRA violação muda (tipback/tail-strike/carga de nariz/robustez
    // continuam PASSANDO, verificado pelos asserts acima) — exatamente 1
    // violação nomeada, a hélice.
    assert_eq!(report.violations.len(), 1,
        "ciclo 9: esperava EXATAMENTE 1 violação (hélice, checagem #25) no baseline real — \
         obteve: {:?}", report.violations);
    assert!(report.violations.iter().any(|v| v.contains("condição crítica CS 23.925")),
        "a única violação esperada é a de folga crítica de hélice (checagem #25): {:?}",
        report.violations);
    assert!(report.violations.iter().any(|v| v.contains("-0.064")),
        "violação de hélice deveria citar a folga crítica observada (≈−0,064 m): {:?}",
        report.violations);
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
    // Campanha E10 (2026-08-08): 14.3273%→**≈9.1369%** (old→new, tolerância
    // INALTERADA) — CUSTO HONESTO da campanha, de duas fontes somadas:
    //   (a) hélice Ø1,95→1,76 m: menos área de disco → η_p 81,0%→78,4%, o
    //       que sobe a potência requerida em cruzeiro (114,3→119,2 kW) e o
    //       consumo (30,4→32,4 L/h);
    //   (b) +25 kg de bateria: OEW 885,3→899,1 kg reconverge o MTOW de
    //       missão para cima (1.512,4→1.537,6 kg), pedindo mais combustível.
    // Combustível de missão 222,7→236,2 L com o tanque inalterado em 260 L.
    // Continua acima do piso de 5%, com ~4,1 pp de folga (era ~9,3 pp).
    assert!((fuel_margin_pct - 9.1369).abs() < 0.1,
        "margem de combustível {fuel_margin_pct:.4}% divergiu do pin honesto pós-E10 \
         ≈9.1369%");
    assert!(fuel_margin_pct >= 5.0,
        "achado honesto esperado (campanha E7): margem ({fuel_margin_pct:.2}%) deveria ficar NO \
         piso de 5% (min_fuel_margin_fraction) ou acima — resolvido por endurance_min_h 8h→7h");

    let mut propeller = aeronave::agents::propeller::PropellerAgent::run(&cfg, &engine, &sized.prop, &req);
    let perf = aeronave::agents::performance::PerformanceAgent::run(
        &sized.state, &sized.wing, &sized.prop, sized.state.mtow_kg, &engine, &req,
        &cfg.performance,
    );
    let electrical = aeronave::agents::electrical::ElectricalAgent::run(&cfg);
    let gear = gear_real();
    // Ciclo 8 (task 2): preenche `prop_clearance_critical_m` (checagem #25)
    // no MESMO caminho de `main.rs` — depois que `gear` existe.
    propeller.fill_critical_clearance(&gear, &cfg.gear, &cfg.propeller);
    let robustness = aeronave::validation::robustness::RobustnessAgent::run(
        &cfg, &engine, &req, &sized.state, &sized.wing, &sized.emp, &sized.structural_masses,
        &sized.wb, &gear, &propeller, mission, &perf,
    );

    let report = ConstraintChecker::verify(&VerifyInputs {
        req: &req, wing: &sized.wing, prop: &sized.prop, mtow_kg: sized.state.mtow_kg,
        engine: &engine, wb: &sized.wb, propeller: &propeller, perf: &perf, mission,
        electrical: &electrical, gear: &gear, gear_cfg: &cfg.gear,
        fuel_capacity_l: cfg.fuel_system.capacity_l, robustness: &robustness,
        prop_cfg: &cfg.propeller,
    });

    assert!(!report.violations.iter().any(|v| v.contains("Margem de combustível")),
        "não deveria haver violação de margem de combustível no baseline real pós-E7: {:?}",
        report.violations);
}
