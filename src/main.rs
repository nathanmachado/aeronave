use std::path::PathBuf;

use clap::Parser;

use aeronave::agents::control_surfaces::ControlSurfacesAgent;
use aeronave::agents::performance::PerformanceAgent;
use aeronave::agents::structural::StructuralAgent;
use aeronave::agents::landing_gear::LandingGearAgent;
use aeronave::models::config::{load_aircraft, load_engine, load_mission};
use aeronave::models::specs::AircraftReport;
use aeronave::orchestrator::size_aircraft;
use aeronave::validation::constraint_checker::ConstraintChecker;

fn sep() { println!("{}", "─".repeat(64)); }

/// Modelagem matemática de aeronave experimental — motor, célula e missão
/// são arquivos TOML: trocar qualquer combinação é trocar um caminho, não
/// recompilar o binário.
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Cli {
    /// Caminho do TOML de especificação do motor.
    #[arg(long, default_value = "config/engines/default.toml")]
    engine: PathBuf,

    /// Caminho do TOML de configuração da célula (aeronave-base).
    #[arg(long, default_value = "config/aircraft/baseline_4seat.toml")]
    aircraft: PathBuf,

    /// Caminho do TOML de requisitos de missão.
    #[arg(long, default_value = "config/missions/default.toml")]
    mission: PathBuf,

    /// Caminho do JSON de saída com o relatório final da aeronave.
    #[arg(long, default_value = "aircraft_spec.json")]
    out: PathBuf,

    /// Preço do diesel S-10 em R$/L, usado na estimativa de custo operacional.
    #[arg(long, default_value_t = 6.5)]
    fuel_price_brl: f64,

    /// Preço do AVGAS em R$/L, usado na estimativa de custo operacional
    /// equivalente (comparação diesel vs. avgas).
    #[arg(long, default_value_t = 18.0)]
    avgas_price_brl: f64,
}

fn main() {
    let cli = Cli::parse();

    // Motor, célula e missão: carregados de TOML — trocar de motor, de
    // aeronave-base ou de missão é trocar um arquivo (ou um argumento de
    // linha de comando), não o código. Erros de carregamento imprimem a
    // mensagem em português do loader e saem com código != 0, sem panic.
    let engine = load_engine(&cli.engine).unwrap_or_else(|e| {
        eprintln!("Erro ao carregar configuração do motor: {e}");
        std::process::exit(1);
    });
    let cfg = load_aircraft(&cli.aircraft).unwrap_or_else(|e| {
        eprintln!("Erro ao carregar configuração da aeronave: {e}");
        std::process::exit(1);
    });
    let req = load_mission(&cli.mission).unwrap_or_else(|e| {
        eprintln!("Erro ao carregar requisitos de missão: {e}");
        std::process::exit(1);
    });

    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║   AERONAVE — Modelagem Matemática v3.0  (6 Agentes)         ║");
    println!("╚══════════════════════════════════════════════════════════════╝\n");
    println!("Motor: {}  |  Trem: Retrátil Elétrico\n", engine.name);

    // ── Sizing: convergência de MTOW (Task 3.1) ─────────────────────────────────
    // Fecha o laço de ponto fixo entre aerodinâmica (que precisa do peso para
    // CL/CD de cruzeiro) e peso/balanceamento (que precisa do arrasto/consumo
    // para fechar OEW + combustível de missão). Antes desta task, a
    // aerodinâmica usava `sizing.mtow_initial_guess_kg` (um palpite nunca
    // realimentado) enquanto Performance/Structural/LandingGear/
    // ConstraintChecker usavam `wb.spec.mtow_kg` — dois MTOWs diferentes
    // coexistindo sem que nenhum fosse necessariamente correto (bug B5). A
    // correção NÃO colapsa para um único número: são dois MTOWs com
    // significados físicos distintos, cada um usado onde faz sentido
    // (decisão da revisão da Task 3.1 — ver task-3.1-report.md, Finding 3):
    //   - `design_mtow_kg` (missão, convergido pelo laço): peso da aeronave
    //     levando exatamente o combustível da missão mínima — alimenta
    //     PerformanceAgent e os checks de requisito de missão (V_cruzeiro,
    //     autonomia, alcance).
    //   - `envelope_mtow_kg` (`wb.spec.mtow_kg`, cenário "4 pax + bagagem +
    //     tanque cheio"): pior caso de carregamento LEGAL da aeronave —
    //     alimenta StructuralAgent/LandingGearAgent, que precisam
    //     dimensionar para a carga máxima que a aeronave pode legalmente
    //     carregar (tipicamente MAIOR que o MTOW de missão, que não enche o
    //     tanque até a borda no payload máximo).
    let sized = size_aircraft(&cfg, &engine, &req).unwrap_or_else(|e| {
        eprintln!("Erro ao convergir MTOW: {e}");
        std::process::exit(1);
    });
    let design_mtow_kg = sized.state.mtow_kg;
    let envelope_mtow_kg = sized.wb.spec.mtow_kg;

    println!("[ SIZING ] Convergência de MTOW");
    // `iterations.len() - 1`: o vetor tem N palpites intermediários MAIS o
    // valor final convergido anexado ao término do laço — o número de
    // PASSAGENS do laço de ponto fixo é `len() - 1`, não `len()`.
    println!("  Iterações: {}  |  MTOW inicial: {:.1}kg → MTOW missão (convergido): {:.1}kg",
             sized.iterations.len() - 1, sized.iterations[0], design_mtow_kg);
    println!("  MTOW envelope (cenário máximo: 4 pax + bagagem + tanque cheio): {:.1}kg",
             envelope_mtow_kg);
    println!("  Combustível de missão (autonomia mínima + reserva): {:.1}kg\n",
             sized.mission_fuel_kg);

    // ── Diagrama de restrições W/S × P/W (Task 3.2) ─────────────────────────────
    // Puramente informativo — recomenda uma área de asa a partir dos limites
    // clássicos de stall/cruzeiro/subida, mas não redimensiona a aeronave
    // automaticamente (ver `agents::constraint_diagram`).
    println!("[ RESTRIÇÕES ] Diagrama W/S × P/W (Raymer cap. 5 / Gudmundsson cap. 3)");
    let cons = &sized.constraints;
    println!("  W/S máximo (stall, Vs_ref={:.1}m/s): {:.1}N/m²  |  W/S ótimo (cruzeiro): {:.1}N/m²",
             cons.v_stall_ref_ms, cons.ws_max_stall_n_m2, cons.ws_optimal_cruise_n_m2);
    println!("  W/S atual: {:.1}N/m²  |  W/S escolhido p/ recomendação: {:.1}N/m²",
             cons.ws_actual_n_m2, cons.ws_chosen_n_m2);
    println!("  P/W mínimo (RC≥1.5m/s): {:.3}W/N  |  P/W atual (pot. máx. contínua, SL): {:.3}W/N",
             cons.pw_min_climb_w_n, cons.pw_actual_w_n);
    println!("  Área de asa recomendada: {:.2}m²  |  Área atual: {:.2}m²",
             cons.recommended_wing_area_m2, sized.wing.area_m2);
    let ws_ok = cons.ws_actual_n_m2 <= cons.ws_max_stall_n_m2;
    let pw_ok = cons.pw_actual_w_n >= cons.pw_min_climb_w_n;
    println!("  {} W/S atual ≤ W/S máximo (stall)", if ws_ok { "✓" } else { "✗" });
    println!("  {} P/W atual ≥ P/W mínimo (subida)\n", if pw_ok { "✓" } else { "✗" });

    let state = &sized.state;

    // ── Agente 1: Aerodinâmica ────────────────────────────────────────────────
    println!("[ AGENTE 1 ] AerodynamicsAgent");
    let wing = &sized.wing;
    println!("  Envergadura: {:.2}m  |  Área: {:.1}m²  |  AR: {:.2}",
             wing.span_m, wing.area_m2, wing.aspect_ratio);
    println!("  Perfil: {}  |  e={:.3}  |  L/D={:.1}",
             wing.airfoil, wing.oswald_efficiency, wing.ld_ratio_cruise);
    println!("  CD0={:.4}  CL_cruise={:.3}  CD_cruise={:.4}  CL_max(flap)={:.2}  CL_max(limpa)={:.2}",
             wing.cd0, wing.cl_cruise, wing.cd_cruise, wing.cl_max, wing.cl_max_clean);
    println!("  VS0 (flap, SL): {:.1} km/h  |  VS1 (limpa, SL): {:.1} km/h\n",
             wing.stall_speed_flaps_kmh, wing.stall_speed_clean_kmh);

    // ── Agente 2: Propulsão ───────────────────────────────────────────────────
    println!("[ AGENTE 2 ] PropulsionAgent — {} + PSRU", engine.name);
    let prop = &sized.prop;
    println!("  {:.0} hp / {:.1} kW  |  {:.0} Nm  |  PSRU {:.3}:1",
             prop.power_hp, prop.power_kw, prop.max_torque_nm, prop.psru_ratio);
    println!("  Motor {:.0} rpm (cruzeiro)  |  Hélice {:.0} rpm  Ø{:.2}m  η={:.1}%  Tração: {:.0}N",
             prop.engine_rpm_cruise, prop.prop_rpm_cruise, prop.prop_diameter_m,
             prop.prop_efficiency * 100.0, prop.thrust_cruise_n);
    println!("  Cruzeiro viável: {}  |  P_req {:.1}kW  vs  P_disp {:.1}kW",
             if prop.cruise_feasible { "✓ SIM" } else { "✗ NÃO" },
             prop.p_req_cruise_kw, prop.p_shaft_cruise_kw);
    println!("  {}  |  {:.0}L  |  {:.1}L/h  |  BSFC {:.0}g/kWh",
             prop.fuel_type, prop.fuel_capacity_l,
             prop.fc_cruise_lph, prop.bsfc_cruise_gkwh);
    println!("  Autonomia: {:.2}h  |  Alcance: {:.0}km\n",
             prop.endurance_h, prop.range_km);

    // ── Empenagem ──────────────────────────────────────────────────────────────
    // Dimensionada por coeficiente de volume (Task 4.1) — geometria pura,
    // consumida pelo NP dentro do AGENTE 3 abaixo.
    println!("[ EMPENAGEM ] EmpennageAgent — dimensionamento por coeficiente de volume");
    let emp = &sized.emp;
    println!("  Horizontal: S={:.2}m²  b={:.2}m  c_raiz={:.2}m  c_ponta={:.2}m  AR={:.1}  V_h={:.2}",
             emp.s_horizontal_m2, emp.span_h_m, emp.chord_h_root_m, emp.chord_h_tip_m,
             emp.ar_h, emp.volume_h);
    println!("  Vertical:   S={:.2}m²  b={:.2}m  c_raiz={:.2}m  c_ponta={:.2}m  AR={:.1}  V_v={:.2}",
             emp.s_vertical_m2, emp.span_v_m, emp.chord_v_root_m, emp.chord_v_tip_m,
             emp.ar_v, emp.volume_v);
    println!("  Braço (CA asa → CA empenagem): {:.2}m\n", emp.arm_h_m);

    // ── Agente 8: Superfícies de Controle ─────────────────────────────────────
    // Dimensionamento de aileron/flap/profundor/leme por razões históricas
    // (Task 4.2, Raymer Tab. 6.5) — puramente geométrico (não depende de
    // MTOW), calculado uma única vez a partir da asa e da empenagem já
    // dimensionadas, sem participar do laço de convergência de MTOW.
    println!("[ AGENTE 8 ] ControlSurfacesAgent — Aileron, Flap, Profundor e Leme");
    let cs = ControlSurfacesAgent::run(wing, emp, &cfg);
    println!("  Aileron:   span/lado={:.3}m  área(2 lados)={:.3}m²  corda_média={:.3}m  [{:.3}–{:.3}]m por lado, da linha de centro",
             cs.aileron.span_m, cs.aileron.area_m2, cs.aileron.chord_mean_m,
             cs.aileron.start_m, cs.aileron.end_m);
    println!("  Flap:      span/lado={:.3}m  área(2 lados)={:.3}m²  corda_média={:.3}m  [{:.3}–{:.3}]m por lado, da linha de centro",
             cs.flap.span_m, cs.flap.area_m2, cs.flap.chord_mean_m,
             cs.flap.start_m, cs.flap.end_m);
    println!("  Profundor: span/lado={:.3}m  área(2 lados)={:.3}m²  corda_média={:.3}m  [{:.3}–{:.3}]m por lado, da linha de centro",
             cs.elevator.span_m, cs.elevator.area_m2, cs.elevator.chord_mean_m,
             cs.elevator.start_m, cs.elevator.end_m);
    println!("  Leme:      span={:.3}m  área(painel único)={:.3}m²  corda_média={:.3}m  [{:.3}–{:.3}]m da raiz\n",
             cs.rudder.span_m, cs.rudder.area_m2, cs.rudder.chord_mean_m,
             cs.rudder.start_m, cs.rudder.end_m);

    // ── Agente 3: Peso e Balanceamento ────────────────────────────────────────
    println!("[ AGENTE 3 ] WeightBalanceAgent — CG e Estabilidade");
    let wb = &sized.wb;
    println!("  Corda: raiz {:.3}m  ponta {:.3}m  MAC {:.3}m",
             wb.chord_root_m, wb.chord_tip_m, wb.mac_m);
    println!("  OEW: {:.1}kg  |  MTOW (cenário estrutural, tanque cheio): {:.1}kg  |  NP: {:.3}m do nariz",
             wb.oew_kg, wb.spec.mtow_kg, wb.x_np_m);
    println!("  CG observado nos cenários: {:.1}%–{:.1}% MAC  |  SM mín: {:.1}%",
             wb.spec.cg_mac_fwd_pct, wb.spec.cg_mac_aft_pct,
             wb.spec.static_margin_pct);
    let all_stable = wb.scenarios.iter().all(|s| s.stable);
    println!("  Todos os cenários estáveis (SM>3%): {}",
             if all_stable { "✓ SIM" } else { "✗ NÃO" });

    // Envelope de CG ADMISSÍVEL (Task 4.4) — limites vindos de
    // `[stability]` (sm_min/sm_max), não dos extremos observados acima.
    println!("  Envelope de CG ADMISSÍVEL: {:.1}%–{:.1}% MAC  (sm_min={:.2}, sm_max={:.2})",
             wb.spec.cg_limit_fwd_pct_mac, wb.spec.cg_limit_aft_pct_mac,
             cfg.stability.sm_min, cfg.stability.sm_max);
    for sc in &wb.scenarios {
        println!("    {} {:30} CG={:5.1}% MAC  SM={:.3}",
                 if sc.inside_envelope { "✓" } else { "✗" },
                 sc.name, sc.cg_pct_mac, sc.static_margin);
    }
    let all_inside_envelope = wb.scenarios.iter().all(|s| s.inside_envelope);
    println!("  Todos os cenários dentro do envelope admissível: {}\n",
             if all_inside_envelope { "✓ SIM" } else { "✗ NÃO" });

    // ── Diagrama V-n completo com rajadas (Task 4.3, CS 23.333/.341) ───────────
    // Roda após o WeightBalanceAgent (precisa do MTOW de envelope e da massa
    // do cenário mais LEVE dentre os cenários de carga) e antes do
    // StructuralAgent (que consome `n_design` — o fator de carga que governa
    // o dimensionamento, manobra OU rajada, o que for maior).
    println!("[ V-n ] VnDiagramAgent — Diagrama V-n completo com rajadas (CS 23.333/.341)");
    let mass_light_kg = wb.scenarios.iter()
        .map(|s| s.total_mass_kg)
        .fold(f64::INFINITY, f64::min);
    let vn = aeronave::agents::vn_diagram::VnDiagramAgent::run(
        wing, envelope_mtow_kg, mass_light_kg, &req, &cfg.structure.design_category,
    );
    println!("  VA={:.0}km/h  VB={:.0}km/h  VC={:.0}km/h  VD={:.0}km/h",
             vn.va_kmh, vn.vb_kmh, vn.vc_kmh, vn.vd_kmh);
    println!("  n_lim: +{:.2}g / {:.2}g (manobra, CS 23.337)  |  n_gust_vc: {:.2}g (envelope, {:.0}kg)  |  n_gust_vc_light: {:.2}g (leve, {:.0}kg)",
             vn.n_lim_pos, vn.n_lim_neg, vn.n_gust_vc, envelope_mtow_kg, vn.n_gust_vc_light, mass_light_kg);
    if vn.n_design > vn.n_lim_pos + 1e-9 {
        println!("  ⚠ Rajada governa: n_design = {:.2}g > n_manobra = {:.2}g\n",
                 vn.n_design, vn.n_lim_pos);
    } else {
        println!("  Manobra governa: n_design = {:.2}g (= n_manobra)\n", vn.n_design);
    }

    // ── Agente 4: Desempenho ──────────────────────────────────────────────────
    println!("[ AGENTE 4 ] PerformanceAgent");
    let perf = PerformanceAgent::run(state, wing, prop, design_mtow_kg, &engine, &req);
    println!("  V_cruzeiro: {:.1}km/h  |  V_stall: {:.1}km/h",
             perf.v_cruise_kmh, perf.v_stall_kmh);
    println!("  RC (SL/MTOW): {:.2}m/s ({:.0}fpm)  |  RC (2500m): {:.2}m/s",
             perf.rc_sl_ms, perf.rc_sl_ms * 196.85, perf.rc_cruise_alt_ms);
    println!("  Teto: {:.0}m ({:.0}ft)",
             perf.service_ceiling_m, perf.service_ceiling_m * 3.281);
    println!("  TO pav: {:.0}m  TO grama: {:.0}m  Pouso: {:.0}m\n",
             perf.to_distance_paved_m, perf.to_distance_grass_m,
             perf.landing_distance_m);

    // ── Agente 5: Estrutura ───────────────────────────────────────────────────
    println!("[ AGENTE 5 ] StructuralAgent — Longarina e Flutter");
    let wing_mass_kg = cfg.masses.item_mass("asa")
        .expect("item de massa 'asa' ausente na configuração da aeronave");
    // Estrutura dimensiona para o pior caso de carga LEGAL (envelope), não
    // para o MTOW de missão — ver comentário do bloco [ SIZING ] acima.
    let struc = StructuralAgent::run(wing, envelope_mtow_kg, wing_mass_kg, &req, &cfg.structure, vn.n_design);
    println!("  Fator de carga: {:.2}g projeto (n_design, manobra ou rajada)  |  {:.2}g último",
             struc.design_load_factor_g, struc.ultimate_load_factor_g);
    println!("  M_raiz (limite): {:.0}N·m  |  (último): {:.0}N·m",
             struc.wing_root_bending_limit_nm, struc.wing_root_bending_ult_nm);
    println!("  Longarina {}: h={:.0}mm  A_mesa={:.1}cm²  t_alma={:.1}mm",
             struc.spar_material,
             struc.spar_height_root_m * 1_000.0,
             struc.spar_flange_area_cm2,
             struc.spar_web_thickness_mm);
    println!("  Pele mín: {:.1}mm  |  Cavernas: {}mm",
             struc.skin_min_thickness_mm, struc.frame_spacing_mm as u32);
    println!("  VD: {:.0}km/h  |  VA (VS1 limpa): {:.0}km/h  |  V_flutter: {:.0}km/h  |  Flutter OK: {}",
             struc.design_dive_speed_kmh, struc.va_kmh, struc.flutter_speed_kmh,
             if struc.flutter_ok { "✓" } else { "✗ RISCO" });
    println!("  Vida fadiga: {:.2e} ciclos\n", struc.fatigue_life_cycles);

    // ── Agente 6: Trem de Pouso ───────────────────────────────────────────────
    println!("[ AGENTE 6 ] LandingGearAgent — Trem Retrátil Elétrico");
    // CG mais traseiro do envelope = cenário "4 pax + bagagem + cheio" (29.1% MAC)
    // x_cg = x_mac_le + 0.291 × MAC — x_mac_le vem de [wing] le_root_x_m
    // (única fonte da posição do bordo de ataque).
    let x_cg_aft = cfg.wing.le_root_x_m + wb.spec.cg_mac_aft_pct / 100.0 * wb.mac_m;
    let mass_main_total = cfg.masses.item_mass("trem_principal")
        .expect("item de massa 'trem_principal' ausente na configuração da aeronave");
    let mass_nose = cfg.masses.item_mass("trem_nariz")
        .expect("item de massa 'trem_nariz' ausente na configuração da aeronave");
    // Trem de pouso também dimensiona para o envelope estrutural (mesma
    // razão da StructuralAgent acima) — as cargas de pouso/solo devem
    // suportar o pior caso legal, não o MTOW de missão.
    let gear = LandingGearAgent::run(envelope_mtow_kg, x_cg_aft, &cfg.gear, mass_main_total, mass_nose);
    println!("  Tipo: {}",     gear.gear_type);
    println!("  Bitola: {:.2}m  |  Empeno: {:.2}m  |  Anti-tombamento: {:.1}°",
             gear.track_width_m, gear.wheelbase_m, gear.tipover_angle_deg);
    println!("  Carga nariz: {:.1}%  |  Main: {:.0}N/perna  |  Nariz: {:.0}N",
             gear.nose_load_fraction_pct, gear.main_gear_load_n, gear.nose_gear_load_n);
    println!("  Oleo main: {:.0}mm  |  Nariz: {:.0}mm  |  Sink rate: {:.1}m/s",
             gear.main_oleo_stroke_mm, gear.nose_oleo_stroke_mm, gear.max_sink_rate_ms);
    println!("  Pneu main: {}  |  {:.0}psi",
             gear.main_tire, gear.tire_pressure_psi);
    println!("  Retração: {:.0}s  |  Atuador: {:.0}W ({:.1}A @28V)  |  Peso total: {:.0}kg\n",
             gear.retraction_time_s, gear.actuator_power_w,
             gear.actuator_power_w / 28.0, gear.total_weight_kg);

    // ── Validação Global ──────────────────────────────────────────────────────
    println!("[ VALIDAÇÃO ] Todos os requisitos do projeto:");
    sep();
    let report  = ConstraintChecker::verify(&req, wing, prop, design_mtow_kg, &engine, wb);
    let rc_ok   = perf.rc_sl_ms   >= 1.5;
    let ceil_ok = perf.service_ceiling_m >= 3_000.0;
    let fl_ok   = struc.flutter_ok;
    let tip_ok  = gear.tipover_angle_deg < 55.0;
    let nose_ok = gear.nose_load_fraction_pct >= 8.0 && gear.nose_load_fraction_pct <= 25.0;

    let checks = [
        (report.all_satisfied(),              "Autonomia, consumo, alcance, V_stall, envelope de CG"),
        (perf.v_cruise_kmh >= 280.0,          "V_cruzeiro ≥ 280 km/h"),
        (prop.endurance_h  >= 8.0,            "Autonomia ≥ 8 h"),
        (rc_ok,                               "RC ≥ 1.5 m/s ao nível do mar"),
        (ceil_ok,                             "Teto de serviço ≥ 3.000 m"),
        (fl_ok,                               "V_flutter ≥ 1.20 × VD (CS-23)"),
        (tip_ok,                              "Anti-tombamento < 55°"),
        (nose_ok,                             "Carga nariz 8–25%"),
        (all_stable,                          "Estabilidade longitudinal (todos cenários, SM>3%, referência)"),
        (all_inside_envelope,                 "Envelope de CG admissível (todos cenários, Task 4.4)"),
    ];

    let all_ok = checks.iter().all(|(ok, _)| *ok);
    for (ok, label) in &checks {
        println!("  {} {}", if *ok { "✓" } else { "✗" }, label);
    }

    println!();
    if all_ok {
        println!("  ══ TODOS OS REQUISITOS SATISFEITOS ══");
    } else {
        println!("  ✗ REQUISITOS PENDENTES — revisar parâmetros de projeto");
        for v in &report.violations { println!("    VIOLAÇÃO: {v}"); }
    }
    for w in &report.warnings { println!("  ⚠ {w}"); }

    // ── Economia ──────────────────────────────────────────────────────────────
    println!();
    println!("[ ECONOMIA ] Custo operacional estimado:");
    let custo_h = prop.fc_cruise_lph * cli.fuel_price_brl;
    // 1.67: fator volumétrico diesel→avgas-equivalente (mesma energia por
    // litro de diesel consome ~1.67x em litros de avgas, dada a menor
    // densidade energética do avgas) — mantido inline, não é um preço.
    let avgas_h = prop.fc_cruise_lph * 1.67 * cli.avgas_price_brl;
    println!("  Diesel S-10:  R$ {:.0}/h  |  AVGAS equiv: R$ {:.0}/h  |  Economia: R$ {:.0}/h",
             custo_h, avgas_h, avgas_h - custo_h);
    println!("  Economia/100h de voo: R$ {:.0}", (avgas_h - custo_h) * 100.0);

    // ── JSON Final ────────────────────────────────────────────────────────────
    let report_final = AircraftReport {
        revision:         "3.0".to_string(),
        validation_status: if all_ok { "PASS".to_string() } else { "FAIL".to_string() },
        wing:             wing.clone(),
        propulsion:       prop.clone(),
        empennage:        Some(emp.clone()),
        control_surfaces: Some(cs.clone()),
        weight:           Some(wb.spec.clone()),
        performance:      Some(perf),
        vn_diagram:       Some(vn.clone()),
        structure:        Some(struc),
        landing_gear:     Some(gear),
        violations:       report.violations,
    };

    let json = serde_json::to_string_pretty(&report_final)
        .expect("Falha ao serializar");
    std::fs::write(&cli.out, &json)
        .unwrap_or_else(|e| panic!("Falha ao escrever '{}': {e}", cli.out.display()));

    println!("\n[ SAÍDA ] {} v3.0 gerado — 6 agentes completos.", cli.out.display());
    println!("\nPróximas etapas:");
    println!("  Fase 3 — CAD: FreeCad + Agente Python (socket localhost:9999)");
    println!("  Fase 4 — Plano de construção, BOM e documentação ANAC");
}
