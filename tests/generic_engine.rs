//! Teste de integração: genericidade do motor.
//!
//! Este é o teste central do pedido do usuário — "trocar de motor deve ser
//! trocar um arquivo TOML, não o código". Vive em `tests/` (crate de teste
//! separada) e não em `src/`, para que `src/` permaneça livre de qualquer
//! menção a um motor real específico (ver grep de regressão no relatório da
//! Task 1.4). Consome a biblioteca `aeronave` via `src/lib.rs`.

use std::path::PathBuf;

use aeronave::agents::aerodynamics::AerodynamicsAgent;
use aeronave::agents::performance::max_level_speed_ms;
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

    let wb_toyota = WeightBalanceAgent::run(&state, &wing, &toyota, &cfg, &req);
    let wb_rotax  = WeightBalanceAgent::run(&state, &wing, &rotax, &cfg, &req);

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
// (`orchestrator::size_aircraft`) e revelou que, ao MTOW real (~1.530 kg),
// o tanque de 240 L original ficava 0,3% curto (achado NEEDS_CONTEXT — ver
// `orchestrator_toyota_240l_insuficiente_regressao_sintetica` abaixo, que
// preserva essa descoberta como regressão). O controller decidiu a
// remediação de projeto (não deste código): `fuel_system.capacity_l`
// 240 → 260 L em `config/aircraft/baseline_4seat.toml`, com margem ~8% no
// MTOW convergido. Este teste agora roda o pipeline completo e honesto
// (`size_aircraft`, não mais o palpite fixo) contra a aeronave-base real.
#[test]
fn autonomia_minima_8_horas() {
    let cfg   = baseline_state();
    let req   = baseline_mission();
    let toyota = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let sized = size_aircraft(&cfg, &toyota, &req)
        .expect("aeronave-base + Toyota deveria convergir com o tanque de 260 L");

    println!("MTOW convergido: {:.1} kg", sized.state.mtow_kg);
    println!("Motor cruzeiro: {:.0} rpm", sized.prop.engine_rpm_cruise);
    println!("Consumo cruzeiro: {:.1} L/h", sized.prop.fc_cruise_lph);
    println!("Autonomia (tanque cheio): {:.2} h", sized.prop.endurance_h);
    println!("Alcance: {:.0} km", sized.prop.range_km);
    println!("BSFC: {:.0} g/kWh", sized.prop.bsfc_cruise_gkwh);
    println!("Eficiência hélice: {:.3}", sized.prop.prop_efficiency);

    assert!(sized.prop.endurance_h >= 8.0,
        "Autonomia {:.2} h abaixo do requisito de 8 h", sized.prop.endurance_h);
    assert!(sized.prop.range_km >= 2_240.0,
        "Alcance {:.0} km abaixo do requisito de 2.240 km", sized.prop.range_km);
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
    let v_max_ms = max_level_speed_ms(1_461.0, 2_500.0, &wing, &state, &toyota);
    let v_max_kmh = v_max_ms * 3.6;
    println!("Toyota V_max nivelada = {v_max_kmh:.6} km/h");

    let v_max_pre_refactor_kmh = 310.25137319753946;
    assert!((v_max_kmh - v_max_pre_refactor_kmh).abs() < 1.0,
        "V_max nivelada {v_max_kmh:.2} km/h divergiu do valor pré-refactor \
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
// ficava 0,3% curto para a missão (achado NEEDS_CONTEXT documentado na
// primeira rodada desta task — ver `task-3.1-report.md`). O controller
// decidiu a remediação (fora deste código): `fuel_system.capacity_l`
// 240 → 260 L. Com essa correção, o MTOW converge e este teste passa a
// pinar os números do pipeline REAL (via `size_aircraft`, não mais o
// palpite fixo) — a nova "regressão de ouro" desta task.
//
// Tabela antigo (palpite 1.461 kg, tanque 240 L) → novo (convergido
// ~1.529,9 kg, tanque 260 L), medidos via `cargo run` / esta suíte:
//   endurance_h:    8.065599 h  → 8.527529 h   (tanque maior, mais margem)
//   fc_cruise_lph: 26.780406 L/h → 27.440542 L/h (MTOW maior → mais arrasto)
//   oew_kg:           885.0 kg  →   885.0 kg    (não depende do MTOW)
//   v_cruise_kmh: 308.721471 km/h → 308.643232 km/h (leve queda: mesmo V_cruise
//                                    alvo, mas a busca de rpm de cruzeiro
//                                    reflete o CD_cruise correto ao MTOW real)
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

    let mtow_convergido_kg = 1_529.889377;
    let endurance_h = 8.527528502699363;
    let fc_lph = 27.440541526882967;
    let oew_kg = 885.0;

    assert!((sized.state.mtow_kg - mtow_convergido_kg).abs() < 0.5,
        "MTOW convergido {:.6} kg divergiu do valor medido na Task 3.1 {:.6} kg",
        sized.state.mtow_kg, mtow_convergido_kg);
    assert!((sized.prop.endurance_h - endurance_h).abs() < 1e-6,
        "Autonomia {:.6} h divergiu do valor pós-Task-3.1 {:.6} h",
        sized.prop.endurance_h, endurance_h);
    assert!((sized.prop.fc_cruise_lph - fc_lph).abs() < 1e-6,
        "Consumo cruzeiro {:.6} L/h divergiu do valor pós-Task-3.1 {:.6} L/h",
        sized.prop.fc_cruise_lph, fc_lph);
    assert!((sized.wb.oew_kg - oew_kg).abs() < 1e-6,
        "OEW {:.6} kg divergiu do valor pós-Task-3.1 {:.6} kg",
        sized.wb.oew_kg, oew_kg);

    // V_max nivelada @ MTOW convergido — mesmo pipeline que alimenta
    // `perf.v_cruise_kmh` em `main.rs` (agora com `design_mtow_kg`, não
    // mais `wb.spec.mtow_kg`).
    let v_max_ms = max_level_speed_ms(sized.state.mtow_kg, 2_500.0, &sized.wing, &sized.state, &toyota);
    let v_max_kmh = v_max_ms * 3.6;
    println!("golden: v_cruise_kmh={v_max_kmh:.6}");
    let v_max_pos_task_3_1_kmh = 308.64323162934545;
    assert!((v_max_kmh - v_max_pos_task_3_1_kmh).abs() < 1e-3,
        "V_cruise nivelada {v_max_kmh:.6} km/h divergiu do valor pós-Task-3.1 \
         {v_max_pos_task_3_1_kmh:.6} km/h", );
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
// ele SEMPRE precisaria subir), e a esse MTOW a missão exige 240,73 L contra
// os 240,0 L de capacidade — 0,73 L (0,3%) a mais do que o tanque original
// carregava.
//
// O controller decidiu a remediação de PROJETO (não deste código):
// `fuel_system.capacity_l` 240 → 260 L em
// `config/aircraft/baseline_4seat.toml`, dando ~8% de margem sobre os
// ~241 L necessários. Com essa correção, `size_aircraft` converge
// normalmente para a aeronave-base real (ver `golden_toyota_baseline_regressao_task_2_1`
// e `autonomia_minima_8_horas` acima, ambos atualizados para rodar o
// pipeline completo via `size_aircraft`).
//
// O teste abaixo preserva a cobertura de regressão do caminho
// `CombustivelInsuficiente` contra o motor Toyota real SEM depender de um
// arquivo de configuração inviável estar commitado: ele lê o TOML real do
// disco e sobrescreve `capacity_l` de volta para 240 L em código (mutação
// sintética, não um arquivo checked-in) — assim o achado original continua
// coberto por teste mesmo depois que `baseline_4seat.toml` passou a usar
// 260 L.
#[test]
fn orchestrator_toyota_240l_insuficiente_regressao_sintetica() {
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

    let err = size_aircraft(&cfg, &toyota, &req).expect_err(
        "regressão: com o tanque ORIGINAL de 240 L (mutação sintética), o MTOW convergido da \
         aeronave-base real (Toyota) deveria continuar exigindo mais combustível do que a \
         capacidade — ver comentário acima e task-3.1-report.md",
    );
    println!("regressão do achado original: {err}");

    match err {
        SizingError::CombustivelInsuficiente { necessario_l, capacidade_l } => {
            assert!((capacidade_l - 240.0).abs() < 1e-9,
                "capacidade_l deveria ser 240.0 (mutação sintética), obtido {capacidade_l}");
            let necessario_pre_pin_l = 240.73219593771384;
            assert!((necessario_l - necessario_pre_pin_l).abs() < 1e-3,
                "necessario_l {necessario_l:.6} L divergiu do valor medido na Task 3.1 \
                 {necessario_pre_pin_l:.6} L");
            assert!(necessario_l > capacidade_l,
                "o ponto central do achado original: a missão precisa de mais combustível do \
                 que o tanque de 240 L tinha — {necessario_l:.2} L > {capacidade_l:.2} L");
        }
        other => panic!(
            "esperava SizingError::CombustivelInsuficiente para a mutação sintética de 240 L, \
             obtido: {other:?} — a física mudou desde que este achado foi documentado; revise \
             o comentário e o relatório da Task 3.1"
        ),
    }
}

// Motor Rotax 915iS: já conhecido por não sustentar 280 km/h de cruzeiro com
// esta célula (ver `trocar_motor_muda_resultado_sem_mudar_codigo` acima —
// `cruise_feasible = false`). Ao convergir o MTOW honestamente, isso se
// reflete numa inviabilidade de combustível muito mais severa que a do
// Toyota (não é um caso de borda de 0,3%): o rpm de melhor esforço
// encontrado por `search_cruise_rpm` para tentar (sem sucesso) sustentar a
// velocidade mínima exigida consome combustível muito acima da capacidade do
// tanque. O aumento do tanque para 260 L (correção do controller para o
// caso Toyota) não resolve o caso Rotax — 404,3 L necessários é quase o
// dobro da capacidade nova, não uma borda de alguns litros — então este
// teste roda contra a aeronave-base REAL (não uma mutação sintética).
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
            let necessario_pre_pin_l = 404.3027554583842;
            assert!((necessario_l - necessario_pre_pin_l).abs() < 1e-2,
                "necessario_l {necessario_l:.6} L divergiu do valor medido na Task 3.1 \
                 {necessario_pre_pin_l:.6} L");
        }
        other => panic!("esperava CombustivelInsuficiente para o Rotax, obtido: {other:?}"),
    }
}
