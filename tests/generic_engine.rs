//! Teste de integração: genericidade do motor.
//!
//! Este é o teste central do pedido do usuário — "trocar de motor deve ser
//! trocar um arquivo TOML, não o código". Vive em `tests/` (crate de teste
//! separada) e não em `src/`, para que `src/` permaneça livre de qualquer
//! menção a um motor real específico (ver grep de regressão no relatório da
//! Task 1.4). Consome a biblioteca `aeronave` via `src/lib.rs`.

use std::path::PathBuf;

use aeronave::agents::aerodynamics::AerodynamicsAgent;
use aeronave::agents::empennage::EmpennageAgent;
use aeronave::agents::performance::{max_level_speed_ms, PerformanceAgent};
use aeronave::agents::propulsion::PropulsionAgent;
use aeronave::agents::weight_balance::WeightBalanceAgent;
use aeronave::models::aircraft_state::AircraftState;
use aeronave::models::config::{load_aircraft, load_engine, load_mission, parse_aircraft};
use aeronave::models::requirements::Requirements;
use aeronave::orchestrator::{size_aircraft, SizingError};

fn config_path(rel: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(rel)
}

fn baseline_state() -> aeronave::models::aircraft_config::AircraftConfig {
    load_aircraft(&config_path("config/aircraft/baseline_4seat.toml")).unwrap()
}

fn baseline_mission() -> Requirements {
    load_mission(&config_path("config/missions/default.toml")).unwrap()
}

/// Massas estruturais COMPUTADAS (ciclo 3, `agents::mass_model`) para os
/// testes que montam um `WeightBalanceOutput` sem passar pelo orchestrator
/// — MTOW do estado (palpite inicial de `[sizing]`) e seed 3,8 do lag-1 de
/// `n_design`, o mesmo par documentado nas demais fixtures de teste.
/// Recebe o motor porque a massa da asa depende (fracamente, expoente
/// 0,0035 de Raymer) do peso do combustível na asa, e este é função da
/// densidade do combustível do motor instalado.
fn masses_do_baseline(
    cfg: &aeronave::models::aircraft_config::AircraftConfig,
    engine: &aeronave::models::engine::EngineSpec,
    req: &Requirements,
    wing: &aeronave::models::specs::WingSpec,
    emp: &aeronave::models::specs::EmpennageSpec,
    state: &AircraftState,
) -> aeronave::agents::mass_model::StructuralMasses {
    aeronave::agents::mass_model::MassModelAgent::run(
        cfg, engine, req, wing, emp, state.mtow_kg, 3.8,
    )
}

#[test]
fn trocar_motor_muda_resultado_sem_mudar_codigo() {
    let cfg   = baseline_state();
    let state = AircraftState::from_config(&cfg);
    let req   = baseline_mission();
    let wing  = AerodynamicsAgent::run(&state, &req);
    let toyota = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let rotax  = load_engine(&config_path("config/engines/rotax_915is.toml")).unwrap();

    let p_toyota = PropulsionAgent::run(&state, &req, &wing, &toyota);
    let p_rotax  = PropulsionAgent::run(&state, &req, &wing, &rotax);

    // Mesmo código, dados diferentes → resultados diferentes e coerentes
    assert!(p_toyota.power_kw > p_rotax.power_kw);
    assert!(p_toyota.fc_cruise_lph != p_rotax.fc_cruise_lph);
    assert_eq!(p_toyota.engine_model, "Toyota 1GD-FTV 2.8 Turbo Diesel");
    assert_eq!(p_rotax.engine_model, "Rotax 915 iS");

    // Viabilidade de cruzeiro: o Toyota (~150 kW de pico) sustenta 280 km/h
    // com esta célula/hélice/PSRU; o Rotax 915iS (~70 kW de pico) não —
    // física honesta, não um número mágico ajustado para "dar certo".
    println!(
        "Toyota: {:.1} kW pico | P_req {:.1} kW vs P_disp {:.1} kW @ {:.0} rpm | feasible={}",
        p_toyota.power_kw, p_toyota.p_req_cruise_kw, p_toyota.p_shaft_cruise_kw,
        p_toyota.engine_rpm_cruise, p_toyota.cruise_feasible
    );
    println!(
        "Rotax:  {:.1} kW pico | P_req {:.1} kW vs P_disp {:.1} kW @ {:.0} rpm | feasible={}",
        p_rotax.power_kw, p_rotax.p_req_cruise_kw, p_rotax.p_shaft_cruise_kw,
        p_rotax.engine_rpm_cruise, p_rotax.cruise_feasible
    );

    assert!(p_toyota.cruise_feasible,
        "Toyota 1GD-FTV deveria sustentar 280 km/h de cruzeiro com esta célula/hélice/PSRU");
    assert!(!p_rotax.cruise_feasible,
        "Rotax 915iS (~70 kW de pico) não deveria sustentar 280 km/h com esta célula \
         dimensionada para o Toyota (~150 kW) — física honesta, não um bug");
}

/// Task 1.5: a massa do motor no orçamento de peso (`WeightBalanceAgent`)
/// vem de `EngineSpec::mass_kg`, não de um valor hardcoded em `src/`.
/// Trocar o motor Toyota (195 kg) pelo Rotax (84 kg) deve reduzir o OEW em
/// ~111 kg e deslocar o CG para trás (motor mais leve no nariz).
#[test]
fn massa_do_motor_afeta_oew_e_cg() {
    let cfg   = baseline_state();
    let state = AircraftState::from_config(&cfg);
    let req   = baseline_mission();
    let wing  = AerodynamicsAgent::run(&state, &req);
    let toyota = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let rotax  = load_engine(&config_path("config/engines/rotax_915is.toml")).unwrap();

    println!("Toyota mass_kg = {:.1} | Rotax mass_kg = {:.1}", toyota.mass_kg, rotax.mass_kg);

    let emp = EmpennageAgent::run(&wing, &cfg);
    let m_toyota = masses_do_baseline(&cfg, &toyota, &req, &wing, &emp, &state);
    let m_rotax  = masses_do_baseline(&cfg, &rotax,  &req, &wing, &emp, &state);
    let wb_toyota = WeightBalanceAgent::run(&state, &wing, &toyota, &cfg, &req, &emp, &m_toyota);
    let wb_rotax  = WeightBalanceAgent::run(&state, &wing, &rotax, &cfg, &req, &emp, &m_rotax);

    println!(
        "OEW Toyota = {:.1} kg | OEW Rotax = {:.1} kg | delta = {:.1} kg",
        wb_toyota.oew_kg, wb_rotax.oew_kg, wb_toyota.oew_kg - wb_rotax.oew_kg
    );

    let mass_delta = toyota.mass_kg - rotax.mass_kg;
    assert!((wb_toyota.oew_kg - wb_rotax.oew_kg - mass_delta).abs() < 0.5,
        "delta de OEW ({:.1} kg) deveria refletir exatamente o delta de massa \
         do motor ({:.1} kg) — a massa do item 'Motor + acessórios' flui direto \
         do EngineSpec para o item de peso, sem outros termos dependentes dela",
        wb_toyota.oew_kg - wb_rotax.oew_kg, mass_delta);
    assert!((wb_toyota.oew_kg - wb_rotax.oew_kg - 111.0).abs() < 5.0,
        "delta de OEW esperado ~111 kg (195 kg Toyota - 84 kg Rotax), obtido {:.1} kg",
        wb_toyota.oew_kg - wb_rotax.oew_kg);

    // Cenários alinhados por índice (mesma ordem/definição em ambas as rodadas)
    for (sc_toyota, sc_rotax) in wb_toyota.scenarios.iter().zip(wb_rotax.scenarios.iter()) {
        assert_eq!(sc_toyota.name, sc_rotax.name);
        assert!(sc_rotax.x_cg_m > sc_toyota.x_cg_m,
            "Cenário '{}': motor mais leve no nariz (Rotax {:.1} kg vs Toyota {:.1} kg) \
             deveria deslocar o CG para trás — x_cg Toyota={:.3}m, Rotax={:.3}m",
            sc_toyota.name, rotax.mass_kg, toyota.mass_kg, sc_toyota.x_cg_m, sc_rotax.x_cg_m);
    }
}

// Histórico (Task 0.3 → Task 1.4): corrigir load_fraction para referenciar
// P_disponível no rpm de cruzeiro (em vez de POWER_KW_MAX em SL) elevou a
// carga de cruzeiro para ~0.99 a 2.400 rpm fixo, subindo o BSFC/consumo e
// derrubando a autonomia para ~7.46h (<8h) e o alcance para ~2.090km
// (<2.240km) — ver task-0.3-report.md. Este teste ficou `#[ignore]`d desde
// então como violação de requisito conhecida.
//
// A Task 1.4 substitui o rpm de cruzeiro fixo (2.400) por uma busca que
// varre `rpm_optimal ± 20%` (limitada por `rpm_max_continuous`) e escolhe o
// rpm de menor BSFC entre os que entregam a potência requerida. Para o
// motor Toyota 1GD-FTV isto desloca o cruzeiro para 2.640 rpm — o limite
// superior exato da faixa de busca (`min(rpm_max_continuous, rpm_optimal
// ·1.2)` = min(3.000, 2.640) = 2.640) — um pouco acima do rpm ótimo de BSFC
// (2.200) mas ainda dentro da banda de torque plano. Isso reduz o BSFC de
// ~221 para ~211 g/kWh e o consumo de ~28.9 para ~26.8 L/h em relação ao
// valor a 2.400 rpm fixo.
//
// Fix pós-review (code review da Task 1.4): a varredura original amostrava
// só rpm_lo, rpm_lo+50, rpm_lo+100, ... e nunca avaliava rpm_hi=2.640
// exatamente quando (rpm_hi-rpm_lo) não era múltiplo de 50 — parava em
// 2.610 (17 passos de 50 a partir de 1.760), perdendo o ponto de menor BSFC
// viável (2.640, BSFC ~211 vs ~211 em 2.610 — a diferença é pequena aqui,
// mas em outras configurações de motor pode ser maior). `search_cruise_rpm`
// agora sempre avalia `rpm_hi` como amostra final quando ele não coincide
// com o último passo de 50 em 50. Números finais medidos após o fix: rpm
// 2.640, BSFC ~211 g/kWh, consumo ~26.8 L/h, autonomia ~8.07h (era ~8.02h),
// alcance ~2.258km (era ~2.245km) — ver task-1.4-report.md para os números
// completos e a comparação com a linha de base pré-Task-1.4 (2.400 rpm
// fixo: 221 g/kWh, 28.9 L/h, 7.46h, 2.090km).
//
// Requisito (>= 8.0h, >= 2.240km) NÃO foi enfraquecido; a física é que
// melhorou.
//
// ATUALIZAÇÃO (Task 3.1 + correção do controller): este teste rodava a um
// MTOW fixo (o palpite `sizing.mtow_initial_guess_kg` = 1.461 kg, nunca
// realimentado — bug B5). A Task 3.1 fechou o laço de convergência
// (`orchestrator::size_aircraft`) e revelou que, ao MTOW convergido
// (~1.529,9 kg), o tanque de 240 L original ficava 3,92 L / 1,6% curto no
// PONTO CONVERGIDO (achado NEEDS_CONTEXT — ver
// `orchestrator_toyota_240l_insuficiente_regressao_sintetica` abaixo, que
// preserva essa descoberta como regressão). O controller decidiu a
// remediação de projeto (não deste código): `fuel_system.capacity_l`
// 240 → 260 L em `config/aircraft/baseline_4seat.toml`, dando 16,08 L
// (~6,6%) de margem sobre os 243,92 L exigidos no MTOW convergido. Este
// teste agora roda o pipeline completo e honesto (`size_aircraft`, não mais
// o palpite fixo) contra a aeronave-base real.
//
// ATUALIZAÇÃO 2 (revisão da Task 3.1, achado de teste quase-tautológico):
// dado `Ok(sized)`, `endurance_h >= 8.0` é garantido por construção pela
// checagem de aceite do laço (`fuel_req_l <= capacity_l · 1.001`) dentro de
// ~0,1% — ver a derivação algébrica no comentário de `size_aircraft` em
// `src/orchestrator.rs`. `range_km >= 2.240` é a MESMA asserção (range =
// v_cruise · endurance, uma constante multiplicativa da mesma desigualdade).
// Ou seja: este teste não está verificando "a física dá 8h", está verificando
// "o laço aceitou" — que já é garantido por `size_aircraft` não ter retornado
// `Err`. Os asserts abaixo são mantidos ESTRITOS de propósito (documentam o
// requisito do projeto, não uma tolerância mais fraca), mas o teste foi
// renomeado para refletir o que ele de fato mede — autonomia/alcance
// derivados da capacidade do tanque no MTOW de projeto, não uma verificação
// independente do laço. A margem REAL (a que de fato distingue "cabe" de
// "não cabe") está no tanque, não na autonomia — ver
// `margem_de_combustivel_no_mtow_convergido` logo abaixo, que pina o número
// que realmente varia.
//
// ATUALIZAÇÃO (Task 5.2) — A CLAIM "GARANTIDO POR CONSTRUÇÃO" ERA FALSA EM
// GERAL, NÃO SÓ COINCIDENTEMENTE VERDADEIRA: `cooling_drag_fraction` eleva
// `fc_cruise_lph` (consumo de cruzeiro), o que reduz `prop.endurance_h =
// capacity_l/fc_lph·(1−reserva)` — um cálculo de tanque cheio/consumo
// CONSTANTE completamente independente de `mission.fuel_total_l` (o cálculo
// por segmentos que o critério de aceite do laço de fato compara contra
// `capacity_l`). A suposição "ATUALIZAÇÃO 2" acima (dado `Ok`, `endurance_h
// >= 8.0` é garantido por construção) só valia enquanto o CD0/consumo eram
// baixos o bastante para que as duas fórmulas concordassem numericamente —
// não há relação algébrica que force isso em geral. Medido: com
// `cooling_drag_fraction=0.04`, `endurance_h` cai para ~7,90h e `range_km`
// para ~2.212,8km, ambos ABAIXO do requisito, mesmo com `Ok(sized)`. Isto
// não é uma regressão de projeto: `prop.endurance_h`/`range_km` são
// INFORMATIVOS por design (ver doc-comment de `PropulsionSpec::
// endurance_h`) — o GATE REAL do projeto é `mission.block_time_h >=
// req.endurance_min_h` (`ConstraintChecker`), que continua satisfeito — ver
// `mission_block_time_h_atende_autonomia_minima_no_mtow_convergido` abaixo.
// Este teste é reescrito para não afirmar mais uma garantia que não existe:
// em vez de `>= 8.0`/`>= 2.240`, pina os valores INFORMATIVOS observados
// (que agora ficam abaixo do requisito) e documenta explicitamente por que
// isso é esperado e não um bug.
#[test]
fn autonomia_e_alcance_informativos_tanque_cheio_no_mtow_convergido() {
    let cfg   = baseline_state();
    let req   = baseline_mission();
    let toyota = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let sized = size_aircraft(&cfg, &toyota, &req)
        .expect("aeronave-base + Toyota deveria convergir com o tanque de 260 L");

    println!("MTOW convergido: {:.1} kg", sized.state.mtow_kg);
    println!("Motor cruzeiro: {:.0} rpm", sized.prop.engine_rpm_cruise);
    println!("Consumo cruzeiro: {:.1} L/h", sized.prop.fc_cruise_lph);
    println!("Autonomia (tanque cheio, informativo): {:.6} h", sized.prop.endurance_h);
    println!("Alcance (tanque cheio, informativo): {:.6} km", sized.prop.range_km);
    println!("BSFC: {:.0} g/kWh", sized.prop.bsfc_cruise_gkwh);
    println!("Eficiência hélice: {:.3}", sized.prop.prop_efficiency);

    // NÃO são mais requisitos de aceite (ver comentário acima) — são pins
    // de regressão do campo INFORMATIVO `prop.endurance_h`/`range_km`
    // (modelo de tanque cheio, consumo constante). O gate real do projeto
    // (`mission.block_time_h`) é verificado em outro teste, não aqui.
    // Campanha E1–E6 (2026-08-05): 7.902862 → 7.619800 h (mais CD0/MTOW
    // reduz a autonomia informativa de tanque cheio/consumo constante).
    // Task 4 (refino-ciclo2, arrasto de trim): 7.619800 → 7.599257 h
    // (old→new, ΔCD_trim eleva o consumo/hora).
    // Campanha E7 (2026-08-06): `endurance_min_h` 8h→7h reduz o MTOW de
    // projeto convergido (menos combustível exigido) — menos MTOW ⟹ menos
    // arrasto ⟹ menos consumo/hora ⟹ mais horas com o mesmo tanque cheio:
    // 7.599257 → 7.676424619 h.
    // Ciclo 3 (oew-parametrico, Task 4, 2026-08-07): as 7 massas
    // estruturais do OEW passaram a ser COMPUTADAS (Raymer cap. 15.2,
    // agents::mass_model) em vez de itens fixos — OEW 890,0→879,0 kg e
    // MTOW de missão 1.517,9→1.505,6 kg (aeronave mais leve ⟹ menos
    // arrasto induzido ⟹ menos consumo/hora).
    // Autonomia informativa: 7.676424619 → **7.7292726508 h** (old→new).
    // Ciclo 4, Task 1 (t/c dedicado da empenagem): MTOW convergido sobe
    // +2,37 kg (cauda mais pesada, ver
    // `golden_toyota_baseline_regressao_task_2_1`) ⟹ mais arrasto
    // induzido ⟹ menos horas com o mesmo tanque cheio: 7.7292726508 →
    // 7.7219126689 h (old→new).
    // Ciclo 4, Task 2 (W_dg de envelope com lag-1): MTOW convergido sobe
    // mais (+4,43 kg, ver `golden_toyota_baseline_regressao_task_2_1`) ⟹
    // mais arrasto induzido ⟹ menos horas com o mesmo tanque cheio:
    // 7.7219126689 → **7.709253917 h** (old→new).
    // Campanha E10 (2026-08-08): a hélice menor (Ø1,95→1,76 m) derruba a
    // eficiência propulsiva (η_p 81,0%→78,4%) e a bateria de 53 kg eleva o
    // MTOW convergido (1.512,4→1.537,6 kg) — mais consumo/hora nas duas
    // frentes: 7.709253917 → **7.232495587 h** (old→new, −6,2%; tolerância
    // INALTERADA). Bate com o valor esperado no plano da campanha (~7,23 h).
    let endurance_pin_h = 7.232495587;
    assert!((sized.prop.endurance_h - endurance_pin_h).abs() < 1e-3,
        "Autonomia (informativa) {:.6} h divergiu do pin pós-E7 {:.6} h",
        sized.prop.endurance_h, endurance_pin_h);
    // 2.212,801240 → 2.133,543977 km. Task 4: 2.133,543977 →
    // 2.127,792006 km. Campanha E7: MTOW menor ⟹ menos arrasto ⟹ mais
    // alcance com o mesmo tanque cheio: 2.127,792006 → 2.149,398893 km.
    // Ciclo 3 (oew-parametrico): 2.149,398893 → **2.164,196342 km**
    // (old→new, mesma causa — aeronave mais leve).
    // Ciclo 4, Task 1 (t/c dedicado da empenagem): MTOW convergido sobe
    // (cauda mais pesada) ⟹ mais arrasto induzido ⟹ menos alcance com o
    // mesmo tanque cheio: 2.164,196342 → 2.162,135547 km (old→new).
    // Ciclo 4, Task 2 (W_dg de envelope com lag-1): MTOW convergido sobe
    // mais ⟹ mais arrasto induzido ⟹ menos alcance com o mesmo tanque
    // cheio: 2.162,135547 → **2.158,591097 km** (old→new).
    // Campanha E10 (2026-08-08): mesma causa da autonomia acima (hélice
    // menor + MTOW maior) — 2.158,591097 → **2.025,098764 km** (old→new,
    // −6,2%). Continua sendo um número INFORMATIVO (tanque cheio, consumo
    // constante); o gate real do projeto é `mission.block_time_h` ≥ 7,0 h,
    // que segue satisfeito com 7,06 h.
    let range_pin_km = 2_025.098764;
    assert!((sized.prop.range_km - range_pin_km).abs() < 1e-2,
        "Alcance (informativo) {:.6} km divergiu do pin pós-E7 {:.6} km",
        sized.prop.range_km, range_pin_km);

    // Achado honesto (Task 5.2, pré-E7): o campo informativo ficava ABAIXO
    // do requisito de missão (8h) — prova de que ele não era (e talvez
    // nunca tenha sido logicamente) uma garantia do laço de convergência.
    //
    // ATUALIZAÇÃO (campanha E7, 2026-08-06): a decisão de requisito do
    // cliente (`endurance_min_h` 8h→7h) reduz o próprio requisito abaixo do
    // valor informativo de tanque cheio (7,68h) — a relação se INVERTE:
    // `prop.endurance_h` volta a ficar ACIMA do requisito. Isto não
    // restaura nenhuma garantia matemática (a observação da Task 5.2
    // continua válida: o gate real do projeto é `mission.block_time_h`, não
    // este campo informativo — ver
    // `mission_block_time_h_atende_autonomia_minima_no_mtow_convergido`
    // abaixo) — é só o requisito ter ficado mais folgado. Pin invertido
    // (não mascarado): reflete o valor honesto medido.
    assert!(sized.prop.endurance_h > req.endurance_min_h,
        "achado esperado da campanha E7: prop.endurance_h ({:.2}h, informativo) deveria ficar \
         ACIMA do novo requisito ({:.1}h) — o requisito caiu abaixo do valor informativo de \
         tanque cheio; o gate real (mission.block_time_h) continua sendo verificado em outro \
         teste, não este",
        sized.prop.endurance_h, req.endurance_min_h);
}

/// Achado da revisão da Task 3.1 (teste quase-tautológico acima): a margem
/// que de fato distingue uma missão viável de uma inviável é a folga entre
/// `fuel_system.capacity_l` e o combustível exigido pela missão NO PONTO
/// CONVERGIDO — não a autonomia em si (que é derivada da mesma folga por uma
/// transformação algébrica, sempre ≥ requisito quando `Ok`). Este teste pina
/// essa margem diretamente: no MTOW convergido (~1.529,9 kg), a missão exige
/// 243,92 L; o tanque de 260 L sobra 16,08 L (~6,6%). Nota importante: a
/// autonomia NO PESO DE PROJETO é exatamente 8,0h por construção (margem
/// zero no ponto de missão — é assim que o MTOW convergiu: `fuel_kg` é
/// calculado exatamente para `endurance_min_h`, nem mais nem menos); a
/// margem real do projeto está inteiramente no tanque (a diferença entre
/// `capacity_l` e o combustível de missão), não na autonomia reportada a
/// tanque cheio (`prop.endurance_h`, que é maior que 8h só porque o tanque
/// tem mais litros do que a missão mínima exige).
/// ATUALIZAÇÃO (Task 5.1): `MissionAgent` substitui o modelo de consumo
/// constante como fonte de `fuel_req_l`/`fuel_kg` do laço — ver comentário
/// de `golden_toyota_baseline_regressao_task_2_1` acima para a tabela
/// completa old→new. A margem de tanque SOBE (de ~6,55% para ~13,43%): o
/// cruzeiro Breguet queima menos combustível que consumo constante (a
/// massa cai ao longo do cruzeiro, aliviando arrasto induzido), e essa
/// economia supera com folga o combustível extra da subida a potência
/// plena (não modelado no cálculo antigo) e a reserva agora calculada
/// sobre táxi+subida+cruzeiro+descida (não sobre autonomia×consumo).
///
/// ATUALIZAÇÃO 2 (revisão da Task 5.1, Finding 2): a correção de BSFC
/// referenciado ao virabrequim (ver comentário de
/// `golden_toyota_baseline_regressao_task_2_1`) aumenta o combustível de
/// missão em ~3% — a margem sobre 260 L cai de volta parcialmente (de
/// ~13,43% para ~10,03%), mas continua folgada.
#[test]
fn margem_de_combustivel_no_mtow_convergido() {
    let cfg   = baseline_state();
    let req   = baseline_mission();
    let toyota = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let sized = size_aircraft(&cfg, &toyota, &req)
        .expect("aeronave-base + Toyota deveria convergir com o tanque de 260 L");

    let fuel_req_l_convergido = sized.mission.fuel_total_l;
    let margem_l = cfg.fuel_system.capacity_l - fuel_req_l_convergido;
    let margem_pct = margem_l / fuel_req_l_convergido * 100.0;

    println!(
        "combustível de missão convergido: {fuel_req_l_convergido:.6} L | capacidade: {:.1} L | \
         margem: {margem_l:.6} L ({margem_pct:.4}%)",
        cfg.fuel_system.capacity_l
    );

    // Task 4.6: 16.084075 L (~6,5941%) → 15.980075 L (~6,5487%) — desvio
    // sub-0,1% da atmosfera ISA completa (ver comentário de
    // `golden_toyota_baseline_regressao_task_2_1`).
    // Task 5.1 (pré-Finding-2): análise por segmentos substitui consumo
    // constante — medido: 30.790697 L (~13,4334%).
    // Task 5.1 (pós-Finding-2, BSFC no virabrequim): 23.691947 L (~10,0259%).
    // Task 5.2 (cooling_drag_fraction, valor autoritativo): a margem CAI —
    // CD0 maior eleva o consumo em cruzeiro (fc_cruise_lph), então a missão
    // por segmentos exige mais combustível: 23.691947 L (~10,0259%) →
    // 13.173515 L (~5,3372%). A missão CONTINUA CABENDO nos 260 L (achado
    // esperado pelo controller: "verify mission still fits 260 L") — a
    // queda medida (~10,52 L) é maior que os "~2-3 L" estimados no brief,
    // mas ainda sobra folga confortável de 5,3%.
    //
    // Campanha E1–E6 (2026-08-05): a margem CAI de novo, e mais forte que a
    // projeção do brief (~4,5-5%) — cd0 empennage 0.004→0.0046 (+2,6% no
    // CD0 total) eleva o consumo de cruzeiro, que compõe com o MTOW maior
    // (+12,1 kg, reconvergência do laço) para exigir mais combustível de
    // missão: 246,826485 L → 255,271883 L (+8,445 L, +3,42%). Margem:
    // 13,173515 L → 4,728117 L. Investigado antes de prosseguir (ver
    // task-1-report.md): a física é consistente e na mesma direção
    // qualitativa da própria história deste teste (Task 5.2 já mostrou a
    // margem caindo MAIS que o projetado no brief pela mesma razão — a
    // margem é um RESÍDUO de dois números grandes, então amplifica
    // desproporcionalmente pequenas variações de CD0/MTOW). Ainda
    // POSITIVA — não é um caso NEEDS_CONTEXT (que exigiria margem
    // negativa) — mas está bem mais apertada que o histórico recente;
    // vale monitorar em revisões futuras.
    //
    // NOTA DE CONVENÇÃO (duas percentagens diferentes coexistem no
    // projeto, ambas corretas, cada uma com seu próprio denominador — não
    // confundir): `margem_pct` AQUI NESTE TESTE é definida acima como
    // `margem_l / fuel_req_l_convergido · 100` — % do COMBUSTÍVEL
    // NECESSÁRIO (~5,3372% pré-E6 → ~1,8522% pós-E6). Já
    // `sizing.fuel_margin_pct` (`src/main.rs`, `tests/schema_v4.rs`, JSON
    // de saída) é `fuel_margin_l / capacity_l · 100` — % da CAPACIDADE DO
    // TANQUE (~1,8185% pós-E6, ligeiramente menor que o número acima
    // porque o denominador, capacity_l=260L, é maior que
    // fuel_req_l_convergido≈255,27L). Os dois números convergem quando a
    // margem é pequena (denominadores próximos) e divergem mais quanto
    // maior a margem — por isso este teste, que só olha a razão relativa
    // ao combustível exigido, não deve ser comparado byte-a-byte com o
    // campo `fuel_margin_pct` do JSON de saída.
    // Task 4 (refino-ciclo2, arrasto de trim em cruzeiro): ΔCD_trim≈4,86e-5
    // eleva o consumo de cruzeiro, exigindo mais combustível de missão —
    // margem cai de novo: 4,728117 L (~1,8522%) → 4,099348 L (~1,6019%)
    // (old→new). Continua POSITIVA — achado central pós-E6 permanece válido.
    //
    // Campanha E7 (2026-08-06): `endurance_min_h` 8h→7h (decisão de
    // requisito do cliente) reduz diretamente o combustível exigido pela
    // missão — a margem SOBE de forma acentuada, resolvendo o achado
    // "apertada" de E6: 4,099348 L (~1,6019%) → **36,325136 L (~16,2402%)**
    // (old→new, ~9× maior). Nota de convenção (ver abaixo): este
    // ~16,2402% é a margem sobre o COMBUSTÍVEL EXIGIDO, não sobre a
    // CAPACIDADE do tanque (`sizing.fuel_margin_pct` ≈13,9712%, ver
    // `tests/gear_tipback.rs::margem_de_combustivel_do_baseline_real_fica_
    // acima_do_piso_pin_honesto`).
    // Ciclo 3 (oew-parametrico, Task 4, 2026-08-07): as 7 massas
    // estruturais do OEW passaram a ser COMPUTADAS (Raymer cap. 15.2,
    // agents::mass_model) em vez de itens fixos — OEW 890,0→879,0 kg e
    // MTOW de missão 1.517,9→1.505,6 kg (aeronave mais leve ⟹ menos
    // arrasto induzido ⟹ menos consumo/hora).
    // Margem: 36,325136 L (~16,2402%) → **37,851123 L (~17,0386%)**
    // (old→new) — menos combustível exigido, mesma capacidade de tanque.
    // Ciclo 4, Task 1 (t/c dedicado da empenagem): cauda mais pesada → laço
    // de MTOW realimenta → mais combustível de missão exigido (mais
    // arrasto induzido) → margem CAI ligeiramente: 37,851123 L (~17,0386%)
    // → 37,632378 L (~16,9235%) (old→new).
    // Ciclo 4, Task 2 (W_dg de envelope com lag-1): estrutura mais pesada
    // (W_dg sobe do candidato de missão para o envelope, ver
    // `golden_toyota_baseline_regressao_task_2_1`) → MTOW convergido sobe
    // → mais combustível de missão exigido → margem CAI de novo:
    // 37,632378 L (~16,9235%) → **37,250858 L (~16,7232%)** (old→new).
    // Continua CONFORTAVELMENTE POSITIVA — achado central pós-E7
    // permanece válido.
    // Campanha E10 (2026-08-08): a hélice menor (η_p 81,0%→78,4%, consumo
    // 30,4→32,4 L/h) e a bateria de 53 kg (MTOW de missão 1.512,4→1.537,6
    // kg) elevam o combustível exigido de 222,749 para 236,244 L com a mesma
    // capacidade de 260 L — margem: 37,250858 L (~16,7232%) →
    // **23,755819 L (~10,0556%)** (old→new, tolerâncias INALTERADAS).
    // Continua POSITIVA e acima do piso de projeto (`min_fuel_margin_
    // fraction`=5% da CAPACIDADE, ≈9,14% nessa outra convenção — ver nota
    // de convenção abaixo), mas é a FOLGA QUE E10 MAIS CONSOME: passou de
    // ~9× o piso para ~2×.
    let margem_pin_l = 23.755819;
    assert!((margem_l - margem_pin_l).abs() < 0.1,
        "margem de combustível {margem_l:.4} L divergiu do valor medido pós-E10 \
         {margem_pin_l:.4} L");
    assert!((margem_pct - 10.0556).abs() < 0.1,
        "margem percentual {margem_pct:.4}% divergiu do valor medido pós-E10 ~10,0556%");
    assert!(margem_l > 0.0,
        "achado central pós-E7: com endurance_min_h reduzido, a missão cabe no tanque de 260 L \
         com folga confortável (margem {margem_l:.2} L)");

    // Invariante honesto do modelo por segmentos (substitui a checagem
    // tautológica antiga de "autonomia no peso de projeto == requisito",
    // que só fazia sentido para o modelo de consumo constante): o alcance
    // recomputado a partir dos três segmentos de distância
    // (subida+cruzeiro+descida) bate EXATAMENTE o alcance exigido pela
    // missão (`cruise_speed_min_kmh · endurance_min_h`) — por construção,
    // já que `cruise_distance_km` é definido como o que falta para fechar
    // essa soma (ver docstring de `MissionSpec::range_no_wind_km`).
    //
    // ATUALIZAÇÃO (Finding 4 da revisão final): esta identidade era, até
    // então, também uma checagem de aceite em `ConstraintChecker::verify`
    // (antiga #7) — removida de lá por ser vazia por construção (sempre
    // verdadeira dado `MissionAgent::run` `Ok`, não uma propriedade da
    // célula/motor/missão candidata). O assert abaixo é agora o ÚNICO
    // guarda-corpo desta identidade — se algum refactor futuro de
    // `MissionAgent` quebrar a construção (`cruise_distance_m` deixar de
    // fechar exatamente a distância exigida), é este teste que pega.
    let alcance_exigido_km = req.cruise_speed_min_kmh * req.endurance_min_h;
    assert!((sized.mission.range_no_wind_km - alcance_exigido_km).abs() < 1e-6,
        "range_no_wind_km {:.6} km deveria bater EXATAMENTE o alcance exigido {:.6} km",
        sized.mission.range_no_wind_km, alcance_exigido_km);
}

/// Finding 1 da revisão da Task 5.1: o gate de autonomia do projeto passou a
/// usar `mission.block_time_h` (tempo de bloco da análise por segmentos —
/// subida+cruzeiro+descida) em vez de `prop.endurance_h` (modelo antigo de
/// consumo constante a tanque cheio, que virou informativo).
///
/// ATUALIZAÇÃO (Finding 4 da revisão final): `block_time_h ≥
/// endurance_min_h` NÃO é mais uma checagem de aceite em
/// `ConstraintChecker::verify` (era a antiga #3, removida por ser vazia por
/// construção — dado `MissionAgent::run` `Ok`, a subida/descida sempre
/// voam a velocidade ≤ V_cruzeiro, então o tempo de bloco nunca fica abaixo
/// do tempo que a mesma distância levaria inteira em cruzeiro). Este teste
/// é agora o ÚNICO guarda-corpo dessa invariante — cobre o gate honesto
/// diretamente contra o baseline real, sem depender de `ConstraintChecker`
/// reafirmá-la.
#[test]
fn mission_block_time_h_atende_autonomia_minima_no_mtow_convergido() {
    let cfg    = baseline_state();
    let req    = baseline_mission();
    let toyota = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let sized  = size_aircraft(&cfg, &toyota, &req)
        .expect("aeronave-base + Toyota deveria convergir com o tanque de 260 L");

    println!(
        "block_time_h={:.6} h | requisito={:.1} h | prop.endurance_h (informativo)={:.6} h",
        sized.mission.block_time_h, req.endurance_min_h, sized.prop.endurance_h
    );

    assert!(sized.mission.block_time_h >= req.endurance_min_h,
        "block_time_h {:.6} h deveria atender a autonomia mínima de {:.1} h — este é o gate \
         honesto (Finding 1 da revisão), não `prop.endurance_h` (informativo, tanque cheio)",
        sized.mission.block_time_h, req.endurance_min_h);
}

// Regressão do resolvedor coarse-to-fine de `max_level_speed_ms` (bissecção
// em duas etapas) contra o motor Toyota 1GD-FTV real, medida antes do
// refactor da Task 1.4 (quando o motor ainda era um `const` hardcoded em
// `propulsion.rs`): 310.25137319753946 km/h. Esse pin foi originalmente
// mantido dentro de `src/agents/performance.rs`, mas usava uma fixture de
// teste local cujos valores coincidiam byte-a-byte com o Toyota real — o
// que reintroduzia dados de motor real em `src/` (ver code review da Task
// 1.4). Ele foi movido para cá: `src/` agora usa uma fixture sintética
// própria (com seu próprio pin, ~309.50 km/h, não coincidente com este),
// e o pin do motor real vive apenas aqui, contra o TOML de verdade.
#[test]
fn toyota_v_max_regressao_310kmh() {
    let cfg   = baseline_state();
    let state = AircraftState::from_config(&cfg);
    let req   = baseline_mission();
    let wing  = AerodynamicsAgent::run(&state, &req);
    let toyota = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();

    // Massa fixa (1.461 kg) igual ao antigo `AircraftState::initial().mtow_kg`
    // — mantida como literal para não acoplar este pin de regressão ao
    // `mtow_guess_kg` do baseline (que poderia mudar por outros motivos).
    let v_max_ms = max_level_speed_ms(1_461.0, 2_500.0, 0.0, &wing, &state, &toyota,
                                       cfg.performance.static_thrust_factor);
    let v_max_kmh = v_max_ms * 3.6;
    println!("Toyota V_max nivelada = {v_max_kmh:.6} km/h");

    // ATUALIZAÇÃO (Task 5.2): `cooling_drag_fraction=0.04` (baseline real,
    // `[drag]`) eleva o CD0 desta asa em 4%, reduzindo V_max — consequência
    // honesta esperada da task, não uma regressão do resolvedor:
    // 310.25137319753946 km/h (pré-Task-5.2) → 306.066251 km/h (-4,185
    // km/h, -1,35%).
    // Campanha E1–E6 (2026-08-05): cd0 empennage 0.004→0.0046 eleva o CD0
    // total desta asa mais um pouco (massa fixa de 1.461 kg aqui, não o
    // MTOW convergido — só o efeito do CD0, sem o efeito composto de MTOW
    // maior que aparece em `golden_toyota_baseline_regressao_task_2_1`):
    // 306.066251 → 303.259465 km/h (-2,807 km/h, -0,92%).
    // Campanha E10 (2026-08-08): a hélice Ø1,95→1,76 m reduz η_p em
    // cruzeiro (81,0%→78,4%) — menos potência propulsiva disponível na mesma
    // massa fixa de 1.461 kg deste teste (nenhum efeito de MTOW aqui):
    // 303.259465 → **301.964596 km/h** (−1,29 km/h, −0,43%; tolerância
    // INALTERADA, 1 km/h). O CLmax de pouso 1,72→2,1 não entra em V_max
    // (regime limpo).
    let v_max_pre_refactor_kmh = 301.964596;
    assert!((v_max_kmh - v_max_pre_refactor_kmh).abs() < 1.0,
        "V_max nivelada {v_max_kmh:.2} km/h divergiu do valor pós-E10 \
         {v_max_pre_refactor_kmh:.2} km/h em mais de 1 km/h");
}

// ─── REGRESSÃO DE OURO (TASK 2.1, atualizada na TASK 3.1) ─────────────────────
//
// A Task 2.1 moveu a célula inteira (geometria, braços, massas, material
// estrutural, trem, hélice) de constantes Rust hardcoded para
// `config/aircraft/baseline_4seat.toml`. Este teste fixava os números-chave
// do pipeline Toyota+baseline medidos ANTES do refactor de Task 2.1 — mas
// esses números vinham do MTOW-palpite fixo (1.461 kg, nunca realimentado —
// bug B5), não do MTOW de projeto real da aeronave.
//
// ATUALIZAÇÃO (Task 3.1 + correção do controller): `size_aircraft` fecha o
// laço de convergência; ao rodar contra a aeronave-base real, o MTOW honesto
// converge para ~1.529,9 kg, revelando que o tanque original de 240 L
// ficava 3,92 L / 1,6% curto no PONTO CONVERGIDO (achado NEEDS_CONTEXT
// documentado na primeira rodada desta task — ver `task-3.1-report.md`; os
// "0,73 L / 0,3%" reportados originalmente eram um artefato de a checagem
// de aceite rodar a cada iteração em vez de só no ponto convergido, corrigido
// na revisão desta task — ver `size_aircraft` em `src/orchestrator.rs`). O
// controller decidiu a remediação (fora deste código): `fuel_system.capacity_l`
// 240 → 260 L. Com essa correção, o MTOW converge e este teste passa a
// pinar os números do pipeline REAL (via `size_aircraft`, não mais o
// palpite fixo) — a nova "regressão de ouro" desta task.
//
// Tabela antigo (palpite 1.461 kg, tanque 240 L) → Task 3.1 (convergido
// ~1.529,9 kg, tanque 260 L), medidos via `cargo run` / esta suíte:
//   endurance_h:    8.065599 h  → 8.527529 h   (tanque maior, mais margem)
//   fc_cruise_lph: 26.780406 L/h → 27.440542 L/h (MTOW maior → mais arrasto)
//   oew_kg:           885.0 kg  →   885.0 kg    (não depende do MTOW)
//   v_cruise_kmh: 308.721471 km/h → 308.643232 km/h (leve queda: mesmo V_cruise
//                                    alvo, mas a busca de rpm de cruzeiro
//                                    reflete o CD_cruise correto ao MTOW real)
//
// ATUALIZAÇÃO (Task 4.6): `Isa::density_kgm3` (atmosfera ISA completa: T, p,
// ρ) substitui a aproximação exponencial de densidade em TODOS os agentes
// (aerodinâmica, propulsão, desempenho) — desvio sub-0,1% em ρ de cruzeiro
// (2.500 m: ISA real 0,95695 kg/m³ vs. exponencial ~0,9564 kg/m³, ~0,06%), que
// se propaga pelo laço de convergência de MTOW. Tabela Task 3.1 → Task 4.6
// (mesmo tanque 260 L, ΔISA=0 no `default.toml`):
//   mtow_kg:        1.529,889377 → 1.529,976737  (+0,087 kg, +0,006%)
//   endurance_h:    8,527528503  → 8,523894095   (-0,0036 h, -0,043%)
//   fc_cruise_lph: 27,440541527  → 27,452241593   (+0,0117 L/h, +0,043%)
//   oew_kg:           885,0 kg  →   885,0 kg     (inalterado — não depende
//                                    de densidade do ar)
//   v_cruise_kmh: 308,643232 km/h → 308,599033 km/h (-0,044 km/h, -0,014%)
//
// ATUALIZAÇÃO (Task 5.1): `MissionAgent` (análise por segmentos — táxi,
// subida integrada a potência plena, cruzeiro Breguet com massa
// decrescente, descida a potência parcial, reserva) substitui o modelo de
// consumo constante (`fc_cruise_lph · endurance_min_h / (1 − reserva)`)
// como fonte do combustível de missão do laço de convergência de MTOW. O
// MTOW convergido CAI (a missão honesta exige MENOS combustível: o
// cruzeiro Breguet queima menos que consumo constante porque a massa cai
// ao longo do voo, e essa economia mais que compensa o combustível extra
// da subida a potência plena) — `sized.prop.endurance_h`/`fc_cruise_lph`
// continuam vindo do `PropulsionAgent` (modelo de tanque cheio / consumo
// constante, inalterado nesta task — ver `agents::propulsion`), então
// batem valores DIFERENTES dos de antes só porque o MTOW convergido (que
// alimenta `wing.cd_cruise` e portanto `fc_cruise_lph`) mudou, não porque
// a fórmula de `PropulsionSpec::endurance_h` mudou. Tabela Task 4.6 →
// Task 5.1 (mesmo tanque 260 L, ΔISA=0 no `default.toml`):
//   mtow_kg:        1.529,976737 → 1.517,535815  (-12,44 kg, -0,81%)
//   endurance_h:    8,523894095  → 8,562282924   (+0,038 h, +0,45% — tanque
//                                    cheio/MTOW menor → menos arrasto)
//   fc_cruise_lph: 27,452241593  → 27,329159999   (-0,123 L/h, -0,45%)
//   oew_kg:           885,0 kg  →   885,0 kg     (inalterado)
//   v_cruise_kmh: 308,599033 km/h → 308,893470 km/h (+0,294 km/h, +0,095% —
//                                    MTOW menor → menos arrasto induzido)
//   combustível de missão: 244,02 L (constante) → 229,21 L (Breguet) —
//                                    margem sobre o tanque de 260 L sobe de
//                                    ~6,55% para ~13,43% (ver
//                                    `margem_de_combustivel_no_mtow_convergido`
//                                    abaixo para a tabela completa).
//
// ATUALIZAÇÃO (revisão da Task 5.1, Finding 2): BSFC referencia potência de
// VIRABREQUIM (pré-PSRU), mas a subida e `PropulsionAgent::fc_cruise_lph`
// multiplicavam BSFC por potência de EIXO pós-PSRU (já reduzida por
// `PSRU_EFFICIENCY=0,97`), subestimando TODO o consumo em ~3%
// (`1/0,97 − 1`) — corrigido dividindo a potência de eixo por `η_PSRU`
// antes de aplicar o BSFC (subida, cruzeiro Breguet — que também ganhou o
// fator `η_PSRU` na dedução, ver `agents::mission` — e `fc_cruise_lph`).
// Combustível de missão sobe de volta parcialmente (229,21 L → 236,31 L),
// então o MTOW convergido sobe também, mas continua ABAIXO do valor
// pré-Task-5.1 (consumo constante nunca refletia a massa caindo em
// cruzeiro). Tabela Task 5.1 (pré-Finding-2) → Task 5.1 (pós-Finding-2,
// valor autoritativo):
//   mtow_kg:        1.517,535815 → 1.523,498764   (+5,96 kg, +0,39%)
//   endurance_h:    8,562282924  → 8,287605988    (-0,275 h, -3,21% — tanque
//                                    cheio, consumo constante MAIOR agora
//                                    reflete o fc_cruise_lph corrigido)
//   fc_cruise_lph: 27,329159999  → 28,234933024    (+0,906 L/h, +3,31%)
//   oew_kg:           885,0 kg  →   885,0 kg      (inalterado)
//   v_cruise_kmh: 308,893470 km/h → 308,752803 km/h (-0,141 km/h, -0,046% —
//                                    MTOW ligeiramente maior → mais arrasto)
//   combustível de missão: 229,21 L → 236,31 L (margem sobre 260 L: 13,43%
//                                    → 10,03% — ver
//                                    `margem_de_combustivel_no_mtow_convergido`
//                                    abaixo).
//
// ATUALIZAÇÃO (Task 5.2): `[drag].cooling_drag_fraction` (arrasto de
// refrigeração do motor, 4% do CD0 total, típico Raymer/Hoerner p/
// instalação a pistão bem carenada) eleva CD0 de 0.022 para 0.02288 (+4%).
// CONSEQUÊNCIA HONESTA prevista pelo controller (e confirmada aqui): mais
// arrasto → menos L/D → mais potência requerida em cruzeiro → mais
// combustível → MTOW convergido sobe → V_max cai. Tabela Task 5.1
// (pós-Finding-2) → Task 5.2:
//   mtow_kg:        1.523,498764 → 1.532,334247   (+8,84 kg, +0,58%)
//   endurance_h:    8,287605988  →  7,902861573   (-0,385 h, -4,64% — tanque
//                                    cheio, consumo constante MAIOR agora
//                                    reflete o CD0 elevado; ver achado
//                                    abaixo sobre este campo ser puramente
//                                    INFORMATIVO)
//   fc_cruise_lph: 28,234933024  → 29,609527870    (+1,375 L/h, +4,87%)
//   oew_kg:           885,0 kg  →   885,0 kg      (inalterado — não depende
//                                    de arrasto)
//   v_cruise_kmh: 308,752803 km/h → 304,412480255 km/h (-4,340 km/h, -1,41%
//                                    — dentro da faixa "poucos km/h" que o
//                                    controller previu)
//   combustível de missão: 236,31 L → 246,826485 L (margem sobre 260 L:
//                                    10,03% → 5,34% — MISSÃO CONTINUA
//                                    CABENDO, ver
//                                    `margem_de_combustivel_no_mtow_convergido`
//                                    abaixo; a margem cai mais do que os
//                                    "~2-3 L" estimados pelo controller no
//                                    brief — cai ~10,5 L — mas ainda sobra
//                                    folga confortável de 13,17 L).
//
// Achado honesto adicional (Task 5.2): `sized.prop.endurance_h`/`range_km`
// (modelo INFORMATIVO de tanque cheio/consumo constante, ver doc-comment de
// `PropulsionSpec::endurance_h`) caem para 7,90h/2.212,8km — ABAIXO do
// requisito de 8h/2.240km, mesmo com `Ok(sized)`. Isto viola a suposição do
// comentário histórico deste teste ("dado Ok, endurance_h >= 8.0 é
// garantido por construção") — essa suposição nunca foi uma garantia
// matemática real (o critério de aceite do laço compara
// `mission.fuel_total_l` contra a capacidade do tanque, um cálculo por
// segmentos completamente diferente da fórmula simples
// `capacity_l/fc_lph·(1−reserva)` de `prop.endurance_h`); ela só parecia
// valer enquanto o CD0/consumo eram baixos o bastante. O GATE REAL do
// projeto (`mission.block_time_h >= req.endurance_min_h`, verificado por
// `ConstraintChecker`) continua satisfeito — ver
// `mission_block_time_h_atende_autonomia_minima_no_mtow_convergido` abaixo,
// que passa normalmente. O teste que pinava esse achado tautológico
// (`autonomia_e_alcance_informativos_tanque_cheio_no_mtow_convergido`) foi
// reescrito para refletir isso honestamente — ver seu comentário abaixo.
#[test]
fn golden_toyota_baseline_regressao_task_2_1() {
    let cfg    = baseline_state();
    let req    = baseline_mission();
    let toyota = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let sized  = size_aircraft(&cfg, &toyota, &req)
        .expect("aeronave-base + Toyota deveria convergir com o tanque de 260 L");

    println!(
        "golden: mtow_kg={:.6} endurance_h={:.6} fc_cruise_lph={:.6} oew_kg={:.6}",
        sized.state.mtow_kg, sized.prop.endurance_h, sized.prop.fc_cruise_lph, sized.wb.oew_kg
    );

    // Campanha E1–E6 (2026-08-05): 1.532,334247 → 1.544,428382 kg (+12,09
    // kg, +0,79%) — arrasto extra (cd0 empennage 0.004→0.0046) + massa
    // extra da EH (22→27kg) reconvergem para um MTOW maior; ver tabela
    // completa no comentário de `margem_de_combustivel_no_mtow_convergido`.
    //
    // Task refino-ciclo2 (2026-08-05, 1a+1b): a autoridade de profundor
    // calculada (1a) NÃO realimenta o laço de convergência de MTOW (o
    // `TrimAuthorityAgent` roda só no ponto já convergido, ver
    // `orchestrator::size_aircraft`) — só a massa/arrasto DERIVADOS da
    // empenagem (1b) afetam MTOW/OEW/consumo aqui, e a calibração de
    // `mass_per_area_{h,v}_kg_m2`/`cd0_area_factor` foi feita para
    // reproduzir os itens fixos antigos (27,0/16,0 kg, cd0=0,0046) quase
    // exatamente na área runtime — resíduo de arredondamento MINÚSCULO
    // (5ª–6ª casa decimal), não uma mudança de comportamento: 1.544,428382
    // → 1.544,428619 kg (+0,000237 kg, +0,000015%).
    //
    // Task 4 (refino-ciclo2): arrasto de trim em cruzeiro
    // (`agents::trim_authority::cd_trim_cruise`, ΔCD_trim≈4,86e-5 somado ao
    // polar) — mais arrasto ⟹ mais combustível de cruzeiro (Breguet) ⟹
    // MTOW converge mais pesado: 1.544,428619 → 1.544,956565 kg (old→new,
    // +0,528 kg).
    //
    // Campanha E7 (2026-08-06): `mission.endurance_min_h` 8,0→7,0h (decisão
    // de requisito do cliente) reduz o combustível exigido pela missão, e
    // portanto o MTOW convergido: 1.544,956565 → 1.517,886903 kg
    // (-27,07 kg, -1,75%).
    //
    // Ciclo 3 (oew-parametrico, Task 4, 2026-08-07): as 7 massas
    // estruturais do OEW passaram a ser COMPUTADAS (Raymer cap. 15.2,
    // `agents::mass_model`) em vez de itens fixos de `[[masses.items]]`/
    // `mass_per_area` — o total estrutural cai 422,0→411,0 kg e o laço
    // realimenta (menos OEW ⟹ menos MTOW ⟹ estrutura ainda mais leve):
    // 1.517,886903 → **1.505,634264 kg** (old→new, -12,25 kg, -0,81%).
    //
    // Ciclo 4 (t/c dedicado da empenagem, `[empennage].thickness_ratio`,
    // 2026-08-07): antes deste ciclo `htail_mass_raymer_kg`/
    // `vtail_mass_raymer_kg` usavam `wing.thickness_ratio` (0,15) para a
    // empenagem por falta de campo dedicado — subestimava a massa do EV
    // (~21%) e do EH (~5%), ver nota histórica em `agents::mass_model`.
    // Com o t/c real da empenagem (0,10), a cauda fica mais pesada e o
    // laço de MTOW realimenta (mais OEW ⟹ mais combustível de missão ⟹
    // mais MTOW): 1.505,634264 → **1.508,008307 kg** (old→new, +2,37 kg,
    // +0,16%).
    //
    // Ciclo 4, Task 2 (W_dg = MTOW de envelope com lag-1, 2026-08-07):
    // `MassModelAgent::run` recebia até aqui o candidato de MTOW de
    // MISSÃO desta iteração (`mtow`) como W_dg/W_l de Raymer — inconsistente
    // com `StructuralAgent`/`LandingGearAgent`, que já usavam o MTOW de
    // ENVELOPE (`wb.spec.mtow_kg`, cenário fixo "4 pax + bagagem + tanque
    // cheio"). Corrigido para usar o mesmo envelope, com LAG-1 (mesmo
    // padrão de `n_design_prev`, seed `sizing.mtow_initial_guess_kg`). O
    // envelope (~1.543,7 kg) é mais pesado que o candidato de missão
    // (~1.505,6 kg) neste baseline (4 pax reais ≈ o cenário de envelope,
    // mas o TANQUE CHEIO do envelope excede o combustível de missão) —
    // estrutura dimensionada para o peso maior fica um pouco mais pesada
    // em TODOS os componentes, e o laço realimenta (mais OEW ⟹ mais
    // combustível de missão ⟹ mais MTOW): 1.508,008307 →
    // **1.512,442570 kg** (old→new, +4,43 kg, +0,29% — dentro da faixa
    // "~1-2%" prevista, sem surpresa).
    //
    // Campanha E10 (2026-08-08): duas fontes somadas, ambas para cima —
    // (a) +25 kg de bateria híbrida no OEW (885,3→899,1 kg) e (b) hélice
    // Ø1,95→1,76 m, que baixa η_p de 81,0% para 78,4% e portanto exige mais
    // combustível de missão (222,7→236,2 L); o laço realimenta (mais massa
    // ⟹ mais arrasto induzido ⟹ mais combustível ⟹ mais massa):
    // 1.512,442570 → **1.537,565047 kg** (old→new, +25,12 kg, +1,66% —
    // batendo com a expectativa da campanha, sem surpresa). Tolerância do
    // assert INALTERADA (0,5 kg).
    let mtow_convergido_kg = 1_537.565047159;
    // 7.599257165 h (pré-E7). Campanha E7: MTOW convergido menor ⟹ menos
    // arrasto ⟹ menos consumo de cruzeiro (informativo, tanque cheio) ⟹
    // mais horas com o mesmo tanque: 7.599257 → **7.676424619 h** (old→new).
    // Ciclo 3 (oew-parametrico): 7.676424619 → **7.7292726508 h**.
    // Ciclo 4, Task 1 (t/c dedicado): MTOW convergido sobe (cauda mais
    // pesada) ⟹ mais arrasto induzido ⟹ menos horas com o mesmo tanque
    // (informativo): 7.7292726508 → 7.7219126689 h (old→new).
    // Ciclo 4, Task 2 (W_dg de envelope): MTOW convergido sobe mais (ver
    // `mtow_convergido_kg` acima) ⟹ mais arrasto induzido ⟹ menos horas
    // com o mesmo tanque: 7.7219126689 → **7.709253917 h** (old→new).
    // Campanha E10: hélice menor (η_p 81,0%→78,4%) + MTOW maior ⟹ mais
    // consumo/hora: 7.709253917 → **7.232495587 h** (old→new, −6,2%).
    let endurance_h = 7.232495587;
    // 30.792483387 L/h (pré-E7). Campanha E7: MTOW convergido menor ⟹
    // menos arrasto ⟹ menos potência requerida em cruzeiro: 30.792483 →
    // **30.482941164 L/h** (old→new).
    // Ciclo 3 (oew-parametrico): 30.482941164 → **30.274517483 L/h**
    // (old→new) — aeronave mais leve, menos potência de cruzeiro.
    // Ciclo 4, Task 1 (t/c dedicado): MTOW convergido sobe ⟹ mais potência
    // requerida em cruzeiro: 30.274517483 → 30.3033730156 L/h (old→new).
    // Ciclo 4, Task 2 (W_dg de envelope): MTOW convergido sobe mais ⟹ mais
    // potência requerida em cruzeiro: 30.3033730156 → **30.353131770 L/h**
    // (old→new).
    // Campanha E10: hélice Ø1,95→1,76 m derruba η_p (81,0%→78,4%) e o MTOW
    // maior eleva a potência requerida (114,3→119,2 kW) — o consumo sobe nas
    // duas frentes: 30.353131770 → **32.353977572 L/h** (old→new, +6,6%).
    let fc_lph = 32.353977572;
    // 885.0 → 890.0 kg (+5 kg, item emp_horizontal 22→27kg — único item de
    // massa alterado que afeta o OEW; avionicos/bateria se cancelam). Task
    // refino-ciclo2 (1b): 890.0 → 890.000018 kg — a massa da empenagem
    // agora é DERIVADA (EmpennageSpec × mass_per_area), calibrada para
    // reproduzir 27,0/16,0 kg na área runtime; resíduo de +0,000018 kg
    // (calibração com 4 casas decimais, não exata) — documentado, não
    // investigado (bem abaixo de qualquer tolerância de fabricação);
    // tolerância do assert abaixo apertada para 5e-5, ~2,8× de margem. Task
    // 4 (arrasto de trim) NÃO afeta massa — OEW inalterado. Campanha E7
    // (endurance_min_h, x_main_m): NENHUM dos dois muda item de massa —
    // OEW continua 890.000018 kg (inalterado).
    //
    // Ciclo 3 (oew-parametrico, Task 4, 2026-08-07): as 7 massas
    // estruturais do OEW passaram a ser COMPUTADAS (Raymer cap. 15.2,
    // `agents::mass_model`) em vez de itens fixos de `[[masses.items]]`/
    // `mass_per_area`. Detalhe (old→new, kg): asa 130,0→147,96;
    // fuselagem 160,0→110,56; emp_h 27,0→13,44; emp_v 16,0→6,14;
    // trem_principal 55,0→90,73; trem_nariz 22,0→19,81; tanques
    // 12,0→22,39 — total estrutural 422,0→411,03. OEW 890.000018 →
    // **879.029207 kg** (old→new). Tolerância do assert INALTERADA (5e-5).
    //
    // Ciclo 4 (t/c dedicado da empenagem, 2026-08-07): golden update
    // honesto (achado esperado pelo brief da task, confirmado aqui) — t/c
    // da empenagem passa a ser o campo dedicado (0,10) em vez do t/c da
    // asa (0,15) usado por aproximação. Efeito DIRETO nas equações Raymer
    // (expoentes (100·t/c)^-0,12 no EH e ^-0,49 no EV — mais fina, mais
    // pesada): emp_h 13,44→14,11 kg (+0,67 kg), emp_v 6,14→7,49 kg (+1,35
    // kg). Efeito INDIRETO (laço de MTOW realimenta com a cauda mais
    // pesada — mais OEW, mais combustível, MTOW converge ligeiramente
    // maior — ver acima): asa 147,96→147,99; fuselagem 110,56→110,57;
    // trem_principal 90,73→90,84; trem_nariz 19,81→19,83; tanques
    // 22,39→22,39 (efeito de loop, +0,17 kg somado nos demais itens).
    // Total estrutural 411,03→413,22 kg (+2,19 kg). OEW 879,029207 →
    // **881,219504 kg** (old→new, +2,190 kg, +0,25% — dentro da faixa
    // "~+2,3 kg" prevista pelo brief, sem surpresa >5%). Tolerância do
    // assert INALTERADA (5e-5).
    //
    // Ciclo 4, Task 2 (W_dg de envelope com lag-1, 2026-08-07): W_dg sobe
    // do candidato de missão (~1.505,6 kg) para o envelope de projeto
    // (~1.543,7 kg de `wb.spec.mtow_kg`, ver `mtow_convergido_kg` acima)
    // — TODOS os itens estruturais com expoente positivo em W_dg/W_l
    // ficam um pouco mais pesados (kg, old→new): asa 147,99→149,56;
    // fuselagem 110,57→110,99; emp_h 14,11→14,23; emp_v 7,49→7,55;
    // trem_principal 90,84→92,51; trem_nariz 19,83→20,10; tanques
    // 22,39→22,39 (independe de W_dg, inalterado). Total estrutural
    // 413,22→417,33 kg (+4,11 kg). OEW 881,219504 → **885,333291 kg**
    // (old→new, +4,114 kg, +0,47% — dentro da faixa "~1-2%" prevista,
    // sem surpresa). Tolerância do assert INALTERADA (5e-5).
    //
    // Campanha E10 (2026-08-08): saldo de TRÊS mudanças de massa —
    //   +25,00 kg  bateria híbrida (28→53 kg, item de [[masses.items]])
    //    −7,23 kg  trem_principal (92,51→85,28) e
    //    −4,17 kg  trem_nariz (20,10→15,92): pernas mais curtas
    //               (`main/nose_strut_length_m` 0,67/0,53→0,54/0,40)
    //    +0,19 kg  realimentação do laço nos demais itens estruturais
    //               (asa/fuselagem/empenagens sobem uns gramas com o MTOW
    //               de envelope maior)
    // Total: 885,333291 → **899,119935 kg** (old→new, +13,79 kg, +1,56% —
    // bate com o "~+13,8 kg" previsto no plano da campanha). Tolerância do
    // assert INALTERADA (5e-5).
    let oew_kg = 899.119934921;

    assert!((sized.state.mtow_kg - mtow_convergido_kg).abs() < 0.5,
        "MTOW convergido {:.6} kg divergiu do valor medido na Task 5.2 (cooling_drag_fraction) \
         {:.6} kg", sized.state.mtow_kg, mtow_convergido_kg);
    assert!((sized.prop.endurance_h - endurance_h).abs() < 1e-5,
        "Autonomia {:.6} h divergiu do valor pós-refino-ciclo2 {:.6} h",
        sized.prop.endurance_h, endurance_h);
    assert!((sized.prop.fc_cruise_lph - fc_lph).abs() < 5e-5,
        "Consumo cruzeiro {:.6} L/h divergiu do valor pós-refino-ciclo2 {:.6} L/h",
        sized.prop.fc_cruise_lph, fc_lph);
    assert!((sized.wb.oew_kg - oew_kg).abs() < 5e-5,
        "OEW {:.6} kg divergiu do valor pós-refino-ciclo2 {:.6} kg",
        sized.wb.oew_kg, oew_kg);

    // V_max nivelada @ MTOW convergido — mesmo pipeline que alimenta
    // `perf.v_cruise_kmh` em `main.rs` (agora com `design_mtow_kg`, não
    // mais `wb.spec.mtow_kg`).
    let v_max_ms = max_level_speed_ms(sized.state.mtow_kg, 2_500.0, 0.0, &sized.wing, &sized.state,
                                       &toyota, cfg.performance.static_thrust_factor);
    let v_max_kmh = v_max_ms * 3.6;
    println!("golden: v_cruise_kmh={v_max_kmh:.6}");
    // Pré-Task-5.2: 308.752803 km/h (CD0 sem arrasto de refrigeração).
    // Campanha E1–E6 (2026-08-05): 304.412480 → 301.304773 km/h (-3,11
    // km/h, -1,02%) — cd0 empennage 0.004→0.0046 (+2,6% no CD0 total,
    // 0.0229→0.0235) e MTOW +12,1 kg (mais peso → mais arrasto induzido)
    // reduzem V_max; queda maior que os "~0,5-1 km/h" estimados no brief
    // (mesmo padrão de subestimativa de projeções já visto na Task 5.2 —
    // efeitos compostos do laço de convergência de MTOW não são lineares).
    // Task refino-ciclo2 (1a+1b): 301.304773 → 301.304678 km/h (-0,000095
    // km/h, resíduo de arredondamento da calibração de cd0_area_factor —
    // dentro da tolerância original de 1e-3, não precisou de novo pin).
    // Task 4 (arrasto de trim em cruzeiro): ΔCD_trim eleva o CD_cruise no
    // MTOW convergido — 301.304773 → 301.291776 km/h (old→new, -0,013 km/h).
    //
    // Campanha E7 (2026-08-06): MTOW convergido cai (endurance_min_h
    // 8h→7h, ver acima) — menos peso ⟹ menos arrasto induzido ⟹ V_max
    // sobe: 301.291776 → **301.944536 km/h** (old→new, +0,653 km/h).
    // Ciclo 3 (oew-parametrico): MTOW convergido cai mais 12,25 kg
    // (massas estruturais computadas) — 301.944536 → **302.234169 km/h**
    // (old→new).
    // Ciclo 4, Task 1 (t/c dedicado da empenagem): MTOW convergido sobe
    // +2,37 kg (cauda mais pesada, ver acima) — mais peso ⟹ mais arrasto
    // induzido ⟹ V_max cai: 302.234169 → 302.178330 km/h (old→new,
    // -0,056 km/h).
    // Ciclo 4, Task 2 (W_dg de envelope com lag-1): MTOW convergido sobe
    // mais +4,43 kg (ver `mtow_convergido_kg` acima) — mais peso ⟹ mais
    // arrasto induzido ⟹ V_max cai de novo: 302.178330 →
    // **302.073675 km/h** (old→new, -0,105 km/h).
    // Campanha E10 (2026-08-08): dois efeitos, mesma direção — a hélice
    // Ø1,95→1,76 m corta η_p (81,0%→78,4%) e o MTOW convergido sobe +25,12
    // kg (mais arrasto induzido): 302.073675 → **300.216508 km/h** (old→new,
    // -1,857 km/h, -0,61%; tolerância INALTERADA, 1e-3). Continua acima do
    // requisito de 280 km/h, com ~20 km/h de folga (era ~22).
    let v_max_pos_task_5_2_kmh = 300.216508;
    assert!((v_max_kmh - v_max_pos_task_5_2_kmh).abs() < 1e-3,
        "V_cruise nivelada {v_max_kmh:.6} km/h divergiu do valor pós-E10 \
         {v_max_pos_task_5_2_kmh:.6} km/h", );
}

// ─── TASK 4.7: Vx/Vy, planeio, gradiente CS 23.65, distâncias sobre 15m ────
//
// Regressão de ouro contra o baseline Toyota real (`size_aircraft`, MTOW de
// missão convergido ~1.529,98 kg — mesmo pipeline de `golden_toyota_
// baseline_regressao_task_2_1` acima). Valores medidos empiricamente,
// pré/pós Task 4.7 (`cargo run` / esta suíte, ANTES vs. DEPOIS desta task):
//
//   campo                     ANTES (M5, ad hoc)   DEPOIS (Task 4.7)   Δ
//   to_distance_paved_m       295 m                 359 m              +22%
//   to_distance_grass_m       354 m                 431 m              +22%
//   landing_distance_m        398 m                 398 m              ~0%
//     (T_avg de decolagem passou a usar tração ESTÁTICA corrigida em V=0 —
//      Rankine-Froude × static_thrust_factor=0.75 — em vez da estimativa
//      ad hoc "80% da tração a V_lo/2"; a distância de pouso NÃO mudou
//      porque `mu_brake_paved=0.40` no TOML reproduz exatamente o antigo
//      literal hardcoded `0.40/surface_factor` com surface_factor=1.0)
//
// Campos NOVOS desta task (não existiam antes — sem "ANTES" para comparar):
//   vx_kmh=119.712  vy_kmh=151.636  best_glide_kmh=175.719  glide_ratio=16.456
//   climb_gradient_pct=14.232  (CS 23.65 exige ≥8.3% — folga de quase 6 p.p.)
//   to_50ft_paved_m=388.852  to_50ft_grass_m=436.751  ldg_50ft_m=542.815
//
// ATUALIZAÇÃO (Task 5.1): `MissionAgent` (análise por segmentos) desloca o
// MTOW convergido de ~1.529,98 kg para ~1.517,54 kg (ver comentário de
// `golden_toyota_baseline_regressao_task_2_1`) — MTOW MENOR reduz W/S em
// toda a varredura de desempenho, então RC/gradiente/L-D efetivo mudam
// junto. Tabela Task 4.7 → Task 5.1 (mesmo baseline/Toyota, ΔISA=0):
//   vx_kmh:              119.712275 → 119.224565  (-0,41%)
//   vy_kmh:               151.635549 → 151.017782  (-0,41%)
//   best_glide_kmh:       175.719442 → 175.003557  (-0,41%)
//   glide_ratio:           16.456403 →  16.456403  (inalterado — L/D_max
//                                        não depende de MTOW, só de CD0/AR/e)
//   climb_gradient_pct:    14.231571 →  14.489845  (+1,82% — MTOW menor →
//                                        mais excesso de potência relativo)
//   to_50ft_paved_m:      388.852096 → 382.791485  (-1,56%)
//   to_50ft_grass_m:      436.750941 → 429.914523  (-1,57%)
//   ldg_50ft_m:           542.815218 → 540.795252  (-0,37%)
//
// ATUALIZAÇÃO (revisão da Task 5.1, Finding 2 — BSFC no virabrequim):
// combustível de missão sobe ~3% de volta, MTOW convergido sobe de
// ~1.517,54 kg para ~1.523,50 kg (ver comentário de
// `golden_toyota_baseline_regressao_task_2_1`) — deslocando estes campos
// de novo, na direção OPOSTA à tabela acima (MTOW maior → menos excesso de
// potência relativo). Tabela Task 5.1 (pré-Finding-2) → Task 5.1
// (pós-Finding-2, valor autoritativo):
//   vx_kmh:              119.224565 → 119.458573  (+0,20%)
//   vy_kmh:               151.017782 → 151.314193  (+0,20%)
//   best_glide_kmh:       175.003557 → 175.347047  (+0,20%)
//   glide_ratio:           16.456403 →  16.456403  (inalterado)
//   climb_gradient_pct:    14.489845 →  14.365419  (-0,86%)
//   to_50ft_paved_m:      382.791485 → 385.688557  (+0,76%)
//   to_50ft_grass_m:      429.914523 → 433.182649  (+0,76%)
//   ldg_50ft_m:           540.795252 → 541.763571  (+0,18%)
//
// ATUALIZAÇÃO (Task 5.2): `cooling_drag_fraction` eleva CD0 em 4% (ver
// comentário de `golden_toyota_baseline_regressao_task_2_1`), deslocando o
// MTOW convergido de ~1.523,50 kg para ~1.532,33 kg (+0,58%) e reduzindo
// L/D efetivo em toda a varredura. Tabela Task 5.1 (pós-Finding-2) →
// Task 5.2:
//   vx_kmh:              119.458573 → 119.804471   (+0,29% — MTOW maior)
//   vy_kmh:               151.314193 → 150.611335   (-0,46% — CD0 maior
//                                        reduz a razão de subida ótima)
//   best_glide_kmh:       175.347047 → 174.138910   (-0,69%)
//   glide_ratio:           16.456403 →  16.136831   (-1,94% — L/Dmax cai
//                                        diretamente com o CD0 mais alto,
//                                        não depende de MTOW)
//   climb_gradient_pct:    14.365419 →  14.126792   (-1,66% — mais arrasto
//                                        reduz o excesso de potência
//                                        relativo em Vx)
//   to_50ft_paved_m:      385.688557 → 390.676592   (+1,29%)
//   to_50ft_grass_m:      433.182649 → 438.723163   (+1,28%)
//   ldg_50ft_m:           541.763571 → 543.197862   (+0,26%)
#[test]
fn golden_toyota_baseline_task_4_7_novos_campos_de_performance() {
    let cfg    = baseline_state();
    let req    = baseline_mission();
    let toyota = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let sized  = size_aircraft(&cfg, &toyota, &req).unwrap();
    let perf = PerformanceAgent::run(&sized.state, &sized.wing, &sized.prop, sized.state.mtow_kg,
                                      &toyota, &req, &cfg.performance);

    println!(
        "vx={:.6}km/h vy={:.6}km/h v_bg={:.6}km/h ld_max={:.6} gradiente={:.6}% \
         to_50ft_pav={:.6}m to_50ft_grama={:.6}m ldg_50ft={:.6}m",
        perf.vx_kmh, perf.vy_kmh, perf.best_glide_kmh, perf.glide_ratio,
        perf.climb_gradient_pct, perf.to_50ft_paved_m, perf.to_50ft_grass_m, perf.ldg_50ft_m
    );

    // Vx < Vy < V_bg — ordenação física esperada (melhor ângulo < melhor
    // razão < velocidade de planeio limpo, tipicamente a maior das três para
    // esta célula/polar).
    assert!(perf.vx_kmh < perf.vy_kmh, "Vx deveria ser < Vy");
    assert!(perf.vy_kmh < perf.best_glide_kmh, "Vy deveria ser < V_bg para esta polar");

    // Pins ±1% (ou mais apertado quando o valor é robusto/determinístico) —
    // valores pós-Task 5.2 (cooling_drag_fraction, MTOW convergido
    // ~1.532,33 kg, ver tabela old→new acima).
    // Campanha E1–E6 (2026-08-05): mais CD0 (empennage 0.004→0.0046) e MTOW
    // (+12,1 kg) reduzem L/D em toda a varredura de desempenho — mesmo
    // padrão qualitativo da Task 5.2, tabela old→new:
    //   vx_kmh:             119.804471 → 120.276327  (+0,39% — MTOW maior)
    //   vy_kmh:             150.611335 → 150.059037  (-0,37% — CD0 maior)
    //   best_glide_kmh:     174.138910 → 173.652690  (-0,28%)
    //   glide_ratio:         16.136831 →  15.921184  (-1,34% — L/Dmax cai
    //                                       direto com CD0 mais alto)
    //   climb_gradient_pct:  14.126792 →  13.841444  (-2,02%)
    //   to_50ft_paved_m:    390.676592 → 397.158919  (+1,66%)
    //   to_50ft_grass_m:    438.723163 → 445.966910  (+1,65%)
    //   ldg_50ft_m:         543.197862 → 545.160189  (+0,36%)
    //
    // Campanha E7 (2026-08-06): `endurance_min_h` 8h→7h reduz o MTOW de
    // projeto convergido (1.544,96→1.517,89 kg, -1,75%) — menos peso ⟹
    // menos arrasto induzido/carga alar ⟹ velocidades características
    // caem, gradiente de subida e distâncias melhoram (menos peso a
    // acelerar/sustentar):
    //   vx_kmh:             120.276327 → 119.238356  (-0,86%)
    //   vy_kmh:             150.059037 → 148.764044  (-0,86%)
    //   best_glide_kmh:     173.652690 → 172.154049  (-0,86%)
    //   glide_ratio:         15.921184 →  15.921177  (~0%, L/Dmax não
    //                                       depende do MTOW)
    //   climb_gradient_pct:  13.841444 →  14.386080  (+3,94% — menos peso
    //                                       melhora o gradiente)
    //   to_50ft_paved_m:    397.158919 → 384.063691  (-3,30%)
    //   to_50ft_grass_m:    445.966910 → 431.208536  (-3,31%)
    //   ldg_50ft_m:         545.160189 → 540.852273  (-0,79%)
    // Ciclo 3 (oew-parametrico, Task 4, 2026-08-07): MTOW convergido cai
    // mais 12,25 kg (massas estruturais COMPUTADAS por
    // `agents::mass_model`) — mesma direção qualitativa da campanha E7,
    // menos peso melhora tudo. old→new:
    //   vx_kmh:             119.238356 → 118.756124  (-0,40%)
    //   vy_kmh:             148.764044 → 148.162403  (-0,40%)
    //   best_glide_kmh:     172.154049 → 171.457813  (-0,40%)
    //   glide_ratio:         15.921177 →  15.921177  (~0%, independe do peso)
    //   climb_gradient_pct:  14.386080 →  14.645344  (+1,80%)
    //   to_50ft_paved_m:    384.063691 → 378.116151  (-1,55%)
    //   to_50ft_grass_m:    431.208536 → 424.502945  (-1,56%)
    //   ldg_50ft_m:         540.852273 → 538.861754  (-0,37%)
    // Ciclo 7 (task 1, `cl_max_to`, 2026-08-08): as distâncias de
    // DECOLAGEM passam a usar o CLmax de DECOLAGEM (1,585 = 1,45 +
    // 0,5·(1,72−1,45)) em vez do de POUSO (1,72) — ninguém decola com
    // flap de pouso, e o modelo antigo era OTIMISTA. Só os pins de
    // decolagem se movem; subida, planeio e POUSO ficam INTOCADOS (nenhum
    // deles descreve decolagem). Valores MEDIDOS old→new (o "old" aqui é
    // o valor medido HOJE com o código antigo, não o pin de ciclo 3, que
    // já estava ~0,9% abaixo dentro da tolerância de 1%):
    //   to_50ft_paved_m:    381.413415 → 406.902652  (+6,68%)
    //   to_50ft_grass_m:    428.220670 → 457.696644  (+6,88%)
    //   (o ganho é menor que os +8,52% da rolagem pura porque S_rotação e
    //    S_subida escalam com V_LOF/γ, não com 1/CL_TO)
    //   vx/vy/best_glide/glide_ratio/climb_gradient_pct/ldg_50ft_m:
    //     INALTERADOS (`wing.cl_max` de pouso segue sendo a referência de
    //     estol da subida/planeio e do pouso).
    // Campanha E10 (2026-08-08): três mudanças de projeto entram aqui, com
    // efeitos em direções DIFERENTES por pin — valores MEDIDOS old→new (o
    // "old" é o valor medido com a config E7, não necessariamente o pin
    // anterior, que em alguns casos já estava até ~0,9% deslocado dentro da
    // tolerância de 1%):
    //   (i)   `cl_max_flaps` 1,72→2,1 (flap SLOTTED): VS0 cai 8,7%
    //         (113,3→103,4 km/h). Vx e o POUSO são referenciados ao CLmax de
    //         pouso ⟹ ambos caem forte.
    //   (ii)  `to_flap_fraction` 0,5→0,35: `cl_max_to` = 1,45+0,35·0,65 =
    //         1,6775, MAIOR que o 1,585 de E7 ⟹ a decolagem MELHORARIA
    //         sozinha…
    //   (iii) …mas a hélice Ø1,95→1,76 m corta a tração estática (menos
    //         área de disco) e a bateria de 53 kg eleva o MTOW (+25 kg) ⟹
    //         a decolagem PIORA no saldo.
    //   vx_kmh:             119.024322 → 108.609445  (-8,75%, efeito (i):
    //                                       Vx ∝ VS0)
    //   vy_kmh:             148.497012 → 147.915721  (-0,39%)
    //   best_glide_kmh:     171.845032 → 173.266373  (+0,83%, MTOW maior)
    //   glide_ratio:         15.921177 →  15.921177  (~0%, L/Dmax independe
    //                                       de peso e de flap)
    //   climb_gradient_pct:  14.500655 →  15.129850  (+4,34% — o gradiente
    //                                       é avaliado em Vx, que caiu com
    //                                       (i); CS 23.65 fica MAIS folgado)
    //   to_50ft_paved_m:    406.902652 → 416.222778  (+2,29%, saldo (ii) vs
    //                                       (iii): a hélice menor e o MTOW
    //                                       maior superam o cl_max_to maior)
    //   to_50ft_grass_m:    457.696644 → 469.331958  (+2,54%; segue com
    //                                       folga sobre a pista de 600 m)
    //   ldg_50ft_m:         539.967949 → 502.482013  (-6,94%, efeito (i):
    //                                       V_ref ∝ VS0 — é ESTA queda que,
    //                                       na grama, tira o pouso de 605 m
    //                                       para 557 m e fecha a violação
    //                                       do check #24)
    // TOLERÂNCIAS INALTERADAS (1%).
    let pins: [(&str, f64, f64, f64); 8] = [
        ("vx_kmh",             perf.vx_kmh,             108.609445, 0.01),
        ("vy_kmh",              perf.vy_kmh,             147.915721, 0.01),
        ("best_glide_kmh",      perf.best_glide_kmh,     173.266373, 0.01),
        ("glide_ratio",         perf.glide_ratio,         15.921177, 0.01),
        ("climb_gradient_pct",  perf.climb_gradient_pct,  15.129850, 0.01),
        ("to_50ft_paved_m",     perf.to_50ft_paved_m,    416.222778, 0.01),
        ("to_50ft_grass_m",     perf.to_50ft_grass_m,    469.331958, 0.01),
        ("ldg_50ft_m",          perf.ldg_50ft_m,         502.482013, 0.01),
    ];
    for (nome, obtido, esperado, tol_frac) in pins {
        let tol = esperado.abs() * tol_frac;
        assert!((obtido - esperado).abs() < tol,
            "{nome} = {obtido:.6} divergiu do pin {esperado:.6} em mais de {:.1}%",
            tol_frac * 100.0);
    }

    // CS 23.65: gradiente mínimo de 8.3% para esta categoria — o baseline
    // real passa com folga confortável (~13.8%, mais de 5 p.p. acima do piso).
    assert!(perf.climb_gradient_pct >= 8.3,
        "gradiente {:.2}% abaixo do mínimo CS 23.65 de 8.3%", perf.climb_gradient_pct);

    // Tabela old→new (ver comentário acima) — `to_distance_*`/
    // `landing_distance_m` continuam "baseados em rolagem de solo"
    // (decisão do controller). ATUALIZAÇÃO (Campanha E1–E6, 2026-08-05):
    // CD0 elevado (empennage 0.004→0.0046) e MTOW +12,1 kg alongam a
    // corrida de decolagem/pouso, mesmo padrão qualitativo da Task 5.2:
    // to_distance_paved_m 360.349282 → 366.059931 (+1,58%),
    // to_distance_grass_m 432.419139 → 439.271917 (+1,58%),
    // landing_distance_m 397.878598 → 399.586426 (+0,43%).
    // Campanha E7 (2026-08-06): MTOW convergido cai (-1,75%, ver acima) —
    // menos peso encurta as corridas de decolagem/pouso:
    // to_distance_paved_m 366.059931 → 353.586335 (-3,41%),
    // to_distance_grass_m 439.271917 → 424.303602 (-3,41%),
    // landing_distance_m 399.586426 → 395.838469 (-0,94%).
    // Ciclo 3 (oew-parametrico): 353.586335 → **347.900958** (-1,61%),
    // 424.303602 → **417.481149** (-1,61%), 395.838469 → **394.108258**
    // (-0,44%) — old→new, tolerâncias INALTERADAS (1%).
    // Ciclo 7 (task 1, `cl_max_to`): a rolagem de solo é `S_G ∝ 1/CL_TO`,
    // então as duas distâncias de DECOLAGEM crescem por EXATAMENTE
    // 1,72/1,585 = +8,5173%. Medidos old→new: to_distance_paved_m
    // 351.054408 → **380.954942**, to_distance_grass_m 421.265290 →
    // **457.145930**. `landing_distance_m` INALTERADO (395.069668, pouso
    // segue com `wing.cl_max`). Tolerâncias INALTERADAS (1%).
    // Campanha E10 (2026-08-08): a DECOLAGEM piora e o POUSO melhora, por
    // causas distintas.
    //   Decolagem — `S_G ∝ 1/(CL_TO·T)`: `cl_max_to` SOBE (1,585→1,6775,
    //   `to_flap_fraction` 0,5→0,35 aplicado ao flap slotted mais potente),
    //   o que sozinho encurtaria a corrida; mas a hélice Ø1,95→1,76 m corta
    //   a tração estática (menos área de disco) e o MTOW sobe +25 kg — o
    //   saldo é mais longo: to_distance_paved_m 380.954942 → **398.318846**
    //   (+4,56%), to_distance_grass_m 457.145930 → **477.982615** (+4,56%).
    //   Pouso — `S ∝ V_ref² ∝ 1/CL_max`: `cl_max_flaps` 1,72→2,1 (flap
    //   SLOTTED) encurta a corrida em ~1 − 1,72/2,1 = −18% na parte de
    //   energia, atenuado pelo MTOW maior: landing_distance_m 395.838469 →
    //   **362.676982** (-8,38%). É esta melhora que fecha o check #24
    //   (pouso na grama sobre 15 m: 605 → 557 m, pista de 600 m).
    // Tolerâncias INALTERADAS (1%).
    let to_distance_paved_novo_pin = 398.318846;
    assert!((perf.to_distance_paved_m - to_distance_paved_novo_pin).abs()
                < to_distance_paved_novo_pin * 0.01,
        "to_distance_paved_m {:.3} divergiu do pin pós-E10 {:.3}",
        perf.to_distance_paved_m, to_distance_paved_novo_pin);
    let to_distance_grass_novo_pin = 477.982615;
    assert!((perf.to_distance_grass_m - to_distance_grass_novo_pin).abs()
                < to_distance_grass_novo_pin * 0.01,
        "to_distance_grass_m {:.3} divergiu do pin pós-E10 {:.3}",
        perf.to_distance_grass_m, to_distance_grass_novo_pin);
    // landing_distance_m: pequena variação refletindo o MTOW convergido —
    // tolerância alargada de "praticamente inalterado" (Task 4.7) para 1%
    // (Task 5.1/5.2/E6/E7 deslocam o MTOW, não a fórmula).
    // Campanha E10: `cl_max_flaps` 1,72→2,1 encurta o pouso — ver bloco
    // acima. 395.838469 → **362.676982** (old→new, tolerância INALTERADA).
    let landing_distance_pin = 362.676982;
    assert!((perf.landing_distance_m - landing_distance_pin).abs()
                < landing_distance_pin * 0.01,
        "landing_distance_m {:.3} divergiu do pin pós-E10 {:.3}",
        perf.landing_distance_m, landing_distance_pin);

    // Decolagem/pouso sobre 15m devem exceder as estimativas ground-roll-
    // based simplificadas — segmentos adicionais (rotação/subida,
    // aproximação/flare) só somam distância.
    assert!(perf.to_50ft_paved_m > perf.to_distance_paved_m,
        "TO sobre 15m ({:.1}m) deveria exceder a estimativa ground-roll×1.5 ({:.1}m)",
        perf.to_50ft_paved_m, perf.to_distance_paved_m);
    assert!(perf.ldg_50ft_m > perf.landing_distance_m,
        "Pouso sobre 15m ({:.1}m) deveria exceder a estimativa legada de 200m fixos ({:.1}m)",
        perf.ldg_50ft_m, perf.landing_distance_m);
}

/// Achado da revisão da Task 3.1 (checagem de aceite rodando a cada
/// iteração, em vez de só no ponto convergido — corrigido em
/// `src/orchestrator.rs::size_aircraft_with_max_iters`): demonstrado que um
/// palpite inicial de 1.700 kg disparava `CombustivelInsuficiente` espúrio
/// (260,7 L, transiente de uma iteração intermediária mais pesada) mesmo
/// quando o ponto fixo real da aeronave-base+Toyota precisa de só 243,92 L
/// — ou seja, o veredito de viabilidade dependia do palpite inicial, que é
/// só um ponto de partida numérico, não um dado de projeto. Este teste
/// prova que, com a correção, o MTOW convergido é o MESMO
/// (dentro da tolerância de convergência do laço, 0,5 kg) qualquer que seja
/// o palpite inicial — incluindo palpites próximos do limite superior
/// `sizing.mtow_max_kg` (1.800 kg no baseline).
#[test]
fn convergencia_independe_do_palpite_inicial() {
    let base_cfg = baseline_state();
    let toyota = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let req = baseline_mission();

    let palpites = [1_461.0, 1_600.0, 1_700.0, 1_790.0];
    let mut convergidos = Vec::with_capacity(palpites.len());

    for &palpite in &palpites {
        let mut cfg = base_cfg.clone();
        cfg.sizing.mtow_initial_guess_kg = palpite;
        let sized = size_aircraft(&cfg, &toyota, &req).unwrap_or_else(|e| {
            panic!("palpite inicial {palpite} kg deveria convergir (Ok), obtido erro: {e}")
        });
        println!("palpite {palpite:.0} kg → convergido {:.6} kg", sized.state.mtow_kg);
        convergidos.push(sized.state.mtow_kg);
    }

    let referencia = convergidos[0]; // palpite 1.461 kg (o do baseline real)
    for (palpite, mtow) in palpites.iter().zip(convergidos.iter()) {
        assert!((mtow - referencia).abs() < 0.5,
            "palpite inicial {palpite} kg convergiu para {mtow:.4} kg, divergindo da \
             referência {referencia:.4} kg (palpite 1.461 kg) por mais de 0,5 kg — o MTOW \
             convergido não deveria depender do palpite inicial (achado da revisão da \
             Task 3.1)");
    }
}

// ─── TASK 3.1: CONVERGÊNCIA DE MTOW — ACHADO E CORREÇÃO ────────────────────────
//
// `orchestrator::size_aircraft` fecha o laço de ponto fixo entre
// aerodinâmica e peso/balanceamento que faltava (bug B5: `AerodynamicsAgent`
// usava `sizing.mtow_initial_guess_kg` = 1.461 kg — um palpite nunca
// realimentado — enquanto `PerformanceAgent`/`StructuralAgent`/
// `LandingGearAgent`/`ConstraintChecker` usavam `wb.spec.mtow_kg`, o MTOW do
// cenário "4 pax + bagagem + tanque cheio", calculado a partir do MESMO
// palpite — dois MTOWs diferentes coexistindo no mesmo relatório).
//
// Ao fechar esse laço honestamente, a primeira rodada desta task descobriu
// (NEEDS_CONTEXT, reportado em `task-3.1-report.md`) que a aeronave-base
// real com o tanque ORIGINAL de 240 L era fuel-inviável: o MTOW converge
// para ~1.529,9 kg (bem acima do palpite de 1.461 kg — que já era menor que
// só OEW + payload, 885 + 440 = 1.325 kg, antes mesmo de somar combustível:
// ele SEMPRE precisaria subir), e no PONTO CONVERGIDO a missão exige
// 243,92 L contra os 240,0 L de capacidade — 3,92 L (1,6%) a mais do que o
// tanque original carregava. (Correção da revisão desta task: o número
// originalmente reportado, 240,73 L / 0,3%, vinha de a checagem de aceite
// rodar a cada iteração — capturava um transiente do laço, não o ponto fixo.
// Depois de mover a checagem para rodar só no ponto convergido, o valor real
// é 243,92 L / 1,6% — ver `size_aircraft` em `src/orchestrator.rs`.)
//
// O controller decidiu a remediação de PROJETO (não deste código):
// `fuel_system.capacity_l` 240 → 260 L em
// `config/aircraft/baseline_4seat.toml`, dando 16,08 L (~6,6%) de margem
// sobre os 243,92 L necessários no ponto convergido. Com essa correção,
// `size_aircraft` converge normalmente para a aeronave-base real (ver
// `golden_toyota_baseline_regressao_task_2_1` e
// `autonomia_e_alcance_informativos_tanque_cheio_no_mtow_convergido` acima, ambos
// atualizados para rodar o pipeline completo via `size_aircraft`).
//
// ATUALIZAÇÃO (Task 5.1) — REVERSÃO DO ACHADO: `MissionAgent` (análise de
// missão por segmentos — táxi, subida integrada a potência plena, cruzeiro
// Breguet com massa decrescente, descida a potência parcial, reserva sobre
// o consumo sem reserva) substitui o modelo de consumo constante como
// fonte do combustível de missão. No ponto convergido, o combustível
// exigido pela aeronave-base + Toyota CAI de 244,02 L (modelo antigo) para
// 229,21 L (Breguet honesto) — o cruzeiro Breguet queima menos combustível
// porque a massa cai ao longo da distância percorrida (menos arrasto
// induzido conforme o tanque esvazia), e essa economia supera com folga o
// combustível extra da subida a potência plena (não modelado antes) e a
// reserva recalculada sobre o novo subtotal por segmentos.
//
// Consequência honesta: o tanque ORIGINAL de 240 L — que a Task 3.1 havia
// encontrado insuficiente por 3,92 L/1,6% sob o modelo antigo, motivando o
// aumento para 260 L — na verdade TERIA SIDO SUFICIENTE sob o modelo por
// segmentos (229,21 L cabem em 240 L, com ~10,79 L/4,5% de margem). Isso
// não invalida a decisão de aumentar para 260 L (tomada com o modelo então
// disponível, e que continua dando mais margem — ~13,4% — sob o modelo
// novo); registra apenas que o "achado" original (240 L insuficiente) foi
// um artefato do modelo de consumo constante ESPECÍFICO usado então, não
// uma restrição física permanente. O teste abaixo, que antes pinava
// `CombustivelInsuficiente` para a mutação sintética de 240 L, é reescrito
// para pinar a REVERSÃO — preserva a mesma mutação sintética (não depende
// de um arquivo de configuração inviável estar commitado) como regressão
// do modelo por segmentos em si.
//
// ATUALIZAÇÃO 2 (revisão da Task 5.1, Finding 2 — BSFC no virabrequim): o
// combustível de missão sobe ~3% (229,21 L → 236,31 L), estreitando a
// margem sobre 240 L de ~10,79 L (~4,5%) para ~3,69 L (~1,5%) — a reversão
// CONTINUA válida (240 L ainda é suficiente), mas por uma margem bem mais
// apertada. Ver Finding 4 da mesma revisão (comentário de
// `config/aircraft/baseline_4seat.toml::[fuel_system]`): esta margem
// apertada é exatamente por que a alegação "não uma restrição física
// permanente" foi suavizada lá — um modelo de cruzeiro nivelado (não
// Breguet a L/D constante) estimado pelo revisor fica ~1% de distância de
// virar a conclusão de volta para "insuficiente".
//
// ATUALIZAÇÃO 3 (Task 5.2) — A REVERSÃO REVERTE: exatamente a distância de
// ~1% que a Finding 4 apontava como o quão perto a margem estava de virar
// foi consumida por `cooling_drag_fraction=0.04` (CD0 +4%, ver comentário
// de `golden_toyota_baseline_regressao_task_2_1`). Combustível de missão
// sobe de 236,31 L para 246,826485 L — ULTRAPASSANDO os 240 L da mutação
// sintética por 6,826485 L (~2,84%). A "reversão" da Task 5.1 (240 L
// suficiente) deixa de valer sob este modelo — não porque o modelo de
// missão por segmentos estivesse errado, mas porque o CD0 honesto (agora
// incluindo arrasto de refrigeração) é maior do que o assumido até aqui.
// Isto NÃO afeta o tanque REAL de 260 L, que continua com margem
// confortável (~5,3%, ver `margem_de_combustivel_no_mtow_convergido`) —
// apenas a mutação sintética de 240 L (que já vinha sendo descrita como
// "achado historicamente instável", tendo virado de insuficiente→suficiente
// na Task 5.1 e agora de volta para insuficiente na Task 5.2). O teste
// tinha sido renomeado para refletir o estado da Task 5.2 e reescrito para
// esperar `CombustivelInsuficiente`, preservando o histórico completo acima
// como registro de por que este número já oscilou duas vezes.
//
// ATUALIZAÇÃO 4 (campanha E7, 2026-08-06) — REVIRAVOLTA #3: `mission.
// endurance_min_h` 8,0→7,0h (decisão de requisito do cliente) reduz
// diretamente o combustível exigido pela missão — 246,826485 L (Task 5.2)
// →≈223,66 L, VOLTANDO a caber nos 240 L da mutação sintética (com folga de
// ≈16,3 L, ~6,8%). Terceira reviravolta deste número (insuficiente→
// suficiente→insuficiente→suficiente), desta vez por decisão de requisito
// de missão, não por física de arrasto/CD0. Renomeado de volta e reescrito
// para esperar `Ok(sized)` — preserva a mesma mutação sintética de 240 L
// (não depende de um arquivo de configuração inviável estar commitado)
// como registro do histórico completo.
//
// ATUALIZAÇÃO 5 (campanha E10, 2026-08-08) — NÃO vira de novo, mas quase:
// a hélice Ø1,95→1,76 m (η_p 81,0%→78,4%) e a bateria de 53 kg (MTOW de
// missão 1.512,4→1.537,6 kg) elevam o combustível exigido de 222,502043 L
// para **235,961035 L** (old→new, +6,05%). Ainda CABE nos 240 L da mutação
// sintética, mas a folga desaba de ≈17,50 L (~7,3%) para **4,04 L (~1,7%)**
// — de volta ao território "apertadíssimo" da ATUALIZAÇÃO 2, onde a Finding
// 4 já avisava que um modelo de cruzeiro NIVELADO (em vez de Breguet a L/D
// constante) estaria a ~1% de virar a conclusão. Este número segue sendo o
// termômetro mais sensível do projeto; o tanque REAL de 260 L continua com
// margem (9,14% da capacidade, ver `margem_de_combustivel_no_mtow_
// convergido` e `tests/gear_tipback.rs`). Nome e expectativa `Ok(sized)`
// INALTERADOS — a reviravolta #4 não aconteceu.
#[test]
fn orchestrator_toyota_240l_suficiente_de_novo_com_missao_de_7h() {
    let toml_real = std::fs::read_to_string(config_path("config/aircraft/baseline_4seat.toml"))
        .expect("falha ao ler baseline_4seat.toml do disco");
    assert!(toml_real.contains("capacity_l = 260.0"),
        "este teste espera mutar 260.0 → 240.0 a partir do valor real atual do TOML; se o \
         valor real mudou, atualize esta string (e reavalie se a mutação ainda faz sentido)");
    let toml_240l = toml_real.replace("capacity_l = 260.0", "capacity_l = 240.0");
    let cfg = parse_aircraft(&toml_240l)
        .expect("TOML mutado (só capacity_l trocado) deveria continuar válido");
    assert_eq!(cfg.fuel_system.capacity_l, 240.0, "mutação sintética não teve efeito");

    let toyota = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let req = baseline_mission();

    let sized = size_aircraft(&cfg, &toyota, &req).expect(
        "campanha E7 (4ª reviravolta): com endurance_min_h reduzido para 7h, o tanque de 240 L \
         (mutação sintética) volta a ser SUFICIENTE para a aeronave-base real (Toyota) — ver \
         comentário acima",
    );

    let necessario_l = sized.mission.fuel_total_l;
    let margem_l = cfg.fuel_system.capacity_l - necessario_l;
    println!(
        "achado (4ª reviravolta): combustível exigido {necessario_l:.6} L, capacidade 240.0 L, \
         margem {margem_l:.6} L"
    );

    // Valor medido sob endurance_min_h=7h (campanha E7) — volta a caber
    // dentro dos 240 L da mutação sintética. Levemente diferente do
    // combustível exigido pelo tanque REAL de 260 L (223,674864 L, ver
    // `margem_de_combustivel_no_mtow_convergido`) porque o MTOW reconverge
    // de forma um pouco diferente com capacidade de tanque menor.
    // Ciclo 3 (oew-parametrico): aeronave mais leve exige menos
    // combustível — 223,663329 → **222,101240 L** (old→new).
    // Ciclo 4, Task 1 (t/c dedicado da empenagem): cauda mais pesada eleva
    // ligeiramente o combustível exigido — 222,101240 → 222,319874 L
    // (old→new).
    // Ciclo 4, Task 2 (W_dg de envelope com lag-1): estrutura dimensionada
    // pelo envelope (mais pesada, ver `golden_toyota_baseline_regressao_
    // task_2_1`) eleva o combustível exigido de novo: 222,319874 →
    // **222,502043 L** (old→new). Continua com folga confortável sobre
    // os 240 L.
    // Campanha E10 (2026-08-08): 222,502043 → **235,961035 L** (old→new,
    // +6,05% — hélice menor + bateria de 53 kg; ver ATUALIZAÇÃO 5 acima).
    // Tolerância INALTERADA (1e-2).
    let necessario_pin_l = 235.961035;
    assert!((necessario_l - necessario_pin_l).abs() < 1e-2,
        "necessario_l {necessario_l:.6} L divergiu do valor medido pós-E10 {necessario_pin_l:.6} L");
    assert!(necessario_l < cfg.fuel_system.capacity_l,
        "4ª reviravolta: a missão volta a exigir MENOS combustível do que os 240 L da mutação \
         sintética têm — {necessario_l:.2} L < {:.2} L", cfg.fuel_system.capacity_l);
    assert!(margem_l > 0.0,
        "achado central pós-E7: com a missão reduzida para 7h, o tanque de 240 L (mutação \
         sintética) volta a bastar (margem {margem_l:.2} L)");
}

// Motor Rotax 915iS: já conhecido por não sustentar 280 km/h de cruzeiro com
// esta célula (ver `trocar_motor_muda_resultado_sem_mudar_codigo` acima —
// `cruise_feasible = false`). Ao convergir o MTOW honestamente, isso se
// reflete numa inviabilidade de combustível muito mais severa que a do
// Toyota (não é um caso de borda de 1,6%): o rpm de melhor esforço
// encontrado por `search_cruise_rpm` para tentar (sem sucesso) sustentar a
// velocidade mínima exigida consome combustível muito acima da capacidade do
// tanque. O aumento do tanque para 260 L (correção do controller para o
// caso Toyota) não resolve o caso Rotax — mesmo com o modelo de missão por
// segmentos da Task 5.1 (que reduziu o combustível exigido do caso Toyota
// em ~6%), o Rotax ainda precisa de bem mais que a capacidade — não uma
// borda de alguns litros — então este teste roda contra a aeronave-base
// REAL (não uma mutação sintética). (O valor pinado abaixo mudou de 404,3 L
// para 409,3 L na revisão da Task 3.1: o número antigo era o transiente da
// PRIMEIRA iteração, calculado quando a checagem de aceite ainda rodava a
// cada passo — corrigido para o valor no PONTO CONVERGIDO, ver Finding 1 do
// relatório da revisão. ATUALIZAÇÃO Task 5.1: 409,45 L → 370,66 L —
// `MissionAgent` também reduz o combustível exigido pelo Rotax, mas não o
// suficiente para caber nos 260 L: ainda ~42,6% acima da capacidade.
// ATUALIZAÇÃO 2 (revisão da Task 5.1, Finding 2 — BSFC no virabrequim):
// 370,66 L → 382,03 L (sobe ~3% de volta, mesma origem das outras tabelas
// desta revisão) — ainda ~46,9% acima dos 260 L, nem perto de virar.)
#[test]
fn orchestrator_baseline_rotax_ainda_inviavel_com_tanque_260l() {
    let cfg = baseline_state();
    let rotax = load_engine(&config_path("config/engines/rotax_915is.toml")).unwrap();
    let req = baseline_mission();

    let err = size_aircraft(&cfg, &rotax, &req)
        .expect_err("Rotax 915iS (motor fraco demais) deveria falhar por combustível insuficiente \
                      mesmo com o tanque de 260 L");
    println!("achado: {err}");

    match err {
        SizingError::CombustivelInsuficiente { necessario_l, capacidade_l } => {
            assert!((capacidade_l - 260.0).abs() < 1e-9,
                "capacidade_l divergiu do config/aircraft/baseline_4seat.toml atual ({capacidade_l})");
            // Task 4.6: 409.32412997472005 L (pré-ISA-completa) → 409.452959981169 L
            // (pós — mesma origem de desvio sub-0,1% da tabela acima).
            // Task 5.1 (pré-Finding-2): 409.452959981169 L → 370.663832773 L
            // (`MissionAgent` — análise por segmentos reduz o combustível
            // exigido, mas não o suficiente para caber nos 260 L).
            // Task 5.1 (pós-Finding-2, BSFC no virabrequim): 370.663832773 L
            // → 382.025744943 L.
            // Task 5.2 (cooling_drag_fraction, valor autoritativo): CD0 mais
            // alto eleva ainda mais o combustível exigido — 382.025744943 L
            // → 393.298621188 L (continua MUITO acima dos 260 L, ~51,3% —
            // não perto de virar, mesmo achado qualitativo da Task 5.1).
            // Campanha E1–E6 (2026-08-05): 393.298621188 → 401.843487 L
            // (mais CD0/MTOW, mesma causa das outras tabelas desta task) —
            // continua MUITO acima dos 260 L (~54,6%), nem perto de virar.
            // Task 4 (refino-ciclo2, arrasto de trim): 401.843487 →
            // 407.563944 L (old→new).
            // Campanha E7 (2026-08-06): `endurance_min_h` 8h→7h (decisão de
            // requisito do cliente, ver `config/missions/default.toml`)
            // reduz diretamente o combustível exigido pela missão — 407.563944
            // → **357.080029 L** (old→new, -12,4%). Continua MUITO acima dos
            // 260 L (~37,3%, motor fraco demais para esta célula/missão,
            // independente do tamanho da missão) — nem perto de virar.
            // Ciclo 3 (oew-parametrico): massas estruturais computadas
            // deixam a célula mais leve também neste caso — 357,080029 →
            // **353,967160 L** (old→new). Continua MUITO acima dos 260 L
            // (~36,1%): o achado qualitativo não muda.
            // Ciclo 4, Task 1 (t/c dedicado da empenagem): cauda mais pesada
            // eleva ligeiramente o combustível exigido também neste caso —
            // 353,967160 → 354,344831 L (old→new).
            // Ciclo 4, Task 2 (W_dg de envelope com lag-1): a estrutura
            // passa a ser dimensionada pelo MTOW de ENVELOPE em vez do
            // candidato de missão desta iteração — o motor Rotax nunca
            // converge (é o caminho de ERRO deste teste), então o laço
            // não estabiliza num único ponto fixo como nos casos Toyota
            // acima; a mudança de W_dg desloca ligeiramente os
            // intermediários que alimentam o `CombustivelInsuficiente`
            // final: 354,344831 → **353,309519 L** (old→new, -0,29% —
            // pequeno e na direção OPOSTA à do caso Toyota, consistente
            // com não ser o mesmo tipo de ponto fixo). Continua MUITO
            // acima dos 260 L (~35,9%): achado qualitativo inalterado.
            // Campanha E10 (2026-08-08): 353,309519 → **351,876358 L**
            // (old→new, −0,41%). Direção OPOSTA à do caso Toyota (que sobe)
            // pelo mesmo motivo já documentado acima: o Rotax nunca
            // converge, então os intermediários que alimentam o
            // `CombustivelInsuficiente` não são um ponto fixo. Continua
            // MUITO acima dos 260 L (~35,3%): achado qualitativo inalterado.
            let necessario_pin_l = 351.876358;
            assert!((necessario_l - necessario_pin_l).abs() < 1e-2,
                "necessario_l {necessario_l:.6} L divergiu do valor medido pós-E7 \
                 {necessario_pin_l:.6} L");
        }
        other => panic!("esperava CombustivelInsuficiente para o Rotax, obtido: {other:?}"),
    }
}

// ─── TASK 3.2: DIAGRAMA DE RESTRIÇÕES (W/S × P/W) ──────────────────────────────
//
// `orchestrator::size_aircraft` agora calcula `sized.constraints`
// (`WingLoadingReport`) no MTOW convergido, com a asa/motor/estado finais —
// ver `src/agents/constraint_diagram.rs`. Este teste roda contra a
// aeronave-base + Toyota REAIS (não fixtures sintéticas) e verifica que
// ambos os vereditos do diagrama de restrições são satisfeitos no baseline
// de projeto: a carga alar atual respeita o limite de stall, e a razão
// peso-potência atual (potência máxima contínua no eixo, SL) excede o
// mínimo exigido para a razão de subida requerida (CS-23, 1,5 m/s).
#[test]
fn golden_toyota_baseline_restricoes_ws_pw_ambos_satisfeitos() {
    let cfg    = baseline_state();
    let req    = baseline_mission();
    let toyota = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let sized  = size_aircraft(&cfg, &toyota, &req)
        .expect("aeronave-base + Toyota deveria convergir com o tanque de 260 L");

    let c = &sized.constraints;
    println!(
        "ws_max_stall={:.2} N/m² | ws_optimal_cruise={:.2} N/m² | ws_actual={:.2} N/m² | \
         pw_min_climb={:.4} W/N | pw_actual={:.4} W/N | recommended_area={:.3} m²",
        c.ws_max_stall_n_m2, c.ws_optimal_cruise_n_m2, c.ws_actual_n_m2,
        c.pw_min_climb_w_n, c.pw_actual_w_n, c.recommended_wing_area_m2
    );

    let ws_ok = c.ws_actual_n_m2 <= c.ws_max_stall_n_m2;
    let pw_ok = c.pw_actual_w_n >= c.pw_min_climb_w_n;
    assert!(ws_ok,
        "veredito W/S deveria ser ✓ no baseline: ws_actual ({:.2} N/m²) deveria ser ≤ \
         ws_max_stall ({:.2} N/m²)", c.ws_actual_n_m2, c.ws_max_stall_n_m2);
    assert!(pw_ok,
        "veredito P/W deveria ser ✓ no baseline: pw_actual ({:.4} W/N) deveria ser ≥ \
         pw_min_climb ({:.4} W/N)", c.pw_actual_w_n, c.pw_min_climb_w_n);

    // Pin: ws_actual = MTOW_convergido·g / S. ATUALIZAÇÃO (Task 5.1):
    // `MissionAgent` desloca o MTOW convergido — ver comentário de
    // `golden_toyota_baseline_regressao_task_2_1` para a tabela completa
    // (Task 4.6: 1.529,98 kg → Task 5.1 pré-Finding-2: 1.517,54 kg →
    // Task 5.1 pós-Finding-2/BSFC no virabrequim: 1.523,50 kg). ATUALIZAÇÃO
    // (Task 5.2): `cooling_drag_fraction` eleva o MTOW convergido de novo,
    // para 1.532,33 kg (ver mesma tabela) → ws_actual = 1.532,33 kg ·
    // 9,807 / 14,2 m² ≈ 1.058,28 N/m² (era ≈1.052,18 N/m² pós-Finding-2).
    // Campanha E1–E6 (2026-08-05): MTOW convergido 1.532,33 → 1.544,43 kg
    // (+12,1 kg) → ws_actual ≈ 1.058,28 → 1.066,63 N/m².
    // Campanha E7 (2026-08-06): MTOW convergido cai (endurance_min_h
    // 8h→7h) 1.544,96 → 1.517,89 kg → ws_actual ≈ 1.066,63 → 1.048,30 N/m².
    // Ciclo 3 (oew-parametrico): MTOW convergido cai de novo (massas
    // estruturais computadas) 1.517,89 → 1.505,63 kg → ws_actual ≈
    // 1.048,30 → **1.039,84 N/m²** (old→new).
    // Ciclo 4, Task 1 (t/c dedicado da empenagem): MTOW convergido sobe
    // +2,37 kg (cauda mais pesada) → ws_actual ≈ 1.039,84 → 1.041,48 N/m²
    // (old→new).
    // Ciclo 4, Task 2 (W_dg de envelope com lag-1): MTOW convergido sobe
    // mais +4,43 kg (ver `golden_toyota_baseline_regressao_task_2_1`) →
    // ws_actual ≈ 1.041,48 → **1.044,54 N/m²** (old→new).
    // Campanha E10 (2026-08-08): MTOW convergido sobe +25,12 kg (bateria de
    // 53 kg + hélice menos eficiente, ver `golden_toyota_baseline_regressao_
    // task_2_1`) → ws_actual ≈ 1.044,54 → **1.061,89 N/m²** (old→new).
    // O veredito W/S ✓ fica MAIS folgado, não menos: `cl_max_flaps` 1,72→2,1
    // eleva `ws_max_stall` de 1.967,0 para 2.401,6 N/m² — a asa segue muito
    // abaixo do teto de stall (a área recomendada cai de 7,54 para 6,96 m²,
    // reforçando o achado pré-existente de que 14,2 m² é generoso).
    let ws_actual_esperado = 1_061.89;
    assert!((c.ws_actual_n_m2 - ws_actual_esperado).abs() < 1.0,
        "ws_actual_n_m2 {:.4} divergiu do valor pinado {:.4} N/m² em mais de 1 N/m²",
        c.ws_actual_n_m2, ws_actual_esperado);
}
