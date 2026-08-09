use std::path::PathBuf;

use clap::Parser;

use aeronave::agents::control_surfaces::ControlSurfacesAgent;
use aeronave::agents::electrical::ElectricalAgent;
use aeronave::agents::performance::PerformanceAgent;
use aeronave::agents::propeller::PropellerAgent;
use aeronave::agents::structural::StructuralAgent;
use aeronave::agents::landing_gear::LandingGearAgent;
use std::collections::BTreeMap;

use aeronave::agents::weight_balance::mac_spanwise_pos;
use aeronave::models::config::{load_aircraft, load_engine, load_mission};
use aeronave::models::specs::{AircraftReport, GeometrySpec, SizingReport, SCHEMA_VERSION};
use aeronave::orchestrator::size_aircraft;
use aeronave::validation::constraint_checker::{
    ConstraintChecker, VerifyInputs, RC_SL_MIN_MS, SERVICE_CEILING_MIN_M,
};
use aeronave::validation::robustness::RobustnessAgent;

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

    // Finding 6b da revisão final: banner estava travado em "v3.0 (6
    // Agentes)" — desatualizado desde a Task 6.1 (schema v4) e sempre
    // desatualizado quanto à contagem real de agentes (10: Aerodinâmica,
    // Propulsão, Missão, Peso/Balanceamento, Autoridade de Trim, Desempenho,
    // Estrutura, Trem de Pouso, Superfícies de Controle, Hélice — ver os
    // blocos `[ AGENTE N ]`/`[ TRIM ]` abaixo). Versão agora lida de
    // `SCHEMA_VERSION` (fonte única).
    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║   AERONAVE — Dimensionamento Paramétrico v{SCHEMA_VERSION} (10 Agentes)     ║");
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
    println!("  CD0={:.4}  CL_cruise={:.3}  CD_cruise={:.4}  CL_max: limpa={:.2} / decolagem={:.3} / pouso={:.2}",
             wing.cd0, wing.cl_cruise, wing.cd_cruise, wing.cl_max_clean, wing.cl_max_to,
             wing.cl_max);
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
    println!("  Autonomia (tanque cheio, consumo constante — informativo): {:.2}h  |  \
              Alcance (idem): {:.0}km\n",
             prop.endurance_h, prop.range_km);

    // ── Missão por Segmentos (Task 5.1) ───────────────────────────────────────
    // Substitui o consumo constante acima (`fc_cruise_lph · endurance`) como
    // fonte do combustível de missão do laço de convergência de MTOW — ver
    // `agents::mission::MissionAgent`.
    println!("[ MISSÃO ] MissionAgent — táxi, subida integrada, cruzeiro Breguet, descida");
    let mission = &sized.mission;
    println!("  Táxi:     {:.2}kg", mission.fuel_taxi_kg);
    println!("  Subida:   {:.2}kg  |  {:.1}min  |  {:.1}km",
             mission.fuel_climb_kg, mission.climb_time_min, mission.climb_distance_km);
    println!("  Cruzeiro: {:.2}kg  |  {:.1}km", mission.fuel_cruise_kg, mission.cruise_distance_km);
    println!("  Descida:  {:.2}kg  |  {:.1}km", mission.fuel_descent_kg, mission.descent_distance_km);
    println!("  Reserva:  {:.2}kg  ({:.0}% do consumo sem reserva)",
             mission.fuel_reserve_kg, req.fuel_reserve_fraction * 100.0);
    println!("  Total:    {:.2}kg  |  {:.1}L  |  Tempo de bloco: {:.2}h",
             mission.fuel_total_kg, mission.fuel_total_l, mission.block_time_h);
    println!("  Alcance sem vento: {:.0}km  |  Alcance Breguet (tanque cheio): {:.0}km\n",
             mission.range_no_wind_km, mission.breguet_range_full_tank_km);

    // ── Agente 9: Hélice ──────────────────────────────────────────────────────
    // Dimensionamento/validação da hélice (Task 4.5) — Mach de ponta de pá
    // (estático e cruzeiro) e folga de solo (CS 23.925). Roda logo após a
    // propulsão porque precisa de `prop.prop_rpm_cruise` (rpm de cruzeiro já
    // escolhido pela busca de BSFC do PropulsionAgent).
    println!("[ AGENTE 9 ] PropellerAgent — Mach de Ponta e Folga de Solo");
    let mut propeller = PropellerAgent::run(&cfg, &engine, prop, &req);
    println!("  Diâmetro: {:.2}m ({})  |  Pás: {}",
             propeller.diameter_m, propeller.source, propeller.blades);
    println!("  Mach de ponta: {:.3} estático / {:.3} cruzeiro (helicoidal)  |  Folga de solo: {:.3}m",
             propeller.tip_mach_static, propeller.tip_mach_cruise_helical, propeller.ground_clearance_m);
    println!("  D_máx por Mach: {:.2}m  |  D_máx por folga: {:.2}m",
             propeller.diameter_max_by_mach_m, propeller.diameter_max_by_clearance_m);
    println!("  {} Mach estático  {} Mach cruzeiro  {} Folga de solo",
             if propeller.ok_mach_static { "✓" } else { "✗" },
             if propeller.ok_mach_cruise { "✓" } else { "✗" },
             if propeller.ok_clearance { "✓" } else { "✗" });
    // Mitigação (revisão da Task 4.5): quando o diâmetro é derivado (config
    // omite `diameter_m`) e o Mach de ponta — não a folga de solo — governa,
    // o diâmetro AUTORITATIVO acima pode divergir do diâmetro PROVISÓRIO que
    // `PropulsionAgent` de fato usou para escolher o rpm/BSFC/consumo de
    // cruzeiro (`prop.prop_diameter_m`) — avisa alto, em vez de deixar essa
    // inconsistência silenciosa (mesmo aviso que `ConstraintChecker::verify`
    // reporta em `report.warnings`, impresso aqui também para visibilidade
    // imediata na seção do próprio agente).
    if let Some(aviso) = aeronave::agents::propeller::diameter_mismatch_warning(&propeller, prop) {
        println!("  ⚠ {aviso}");
    }
    println!();

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

    // Envelope de CG ADMISSÍVEL (Task 4.4 + task trim-authority) — traseiro
    // vem de `[stability].sm_min`; dianteiro vem do `TrimAuthorityAgent`
    // (bloco [ TRIM ] abaixo) — um número ÚNICO (`max(flare, rotação)`,
    // nenhum dos dois varia por cenário, ver `TrimSpec`), o MESMO para
    // todos os cenários.
    //
    // ENVELOPE VAZIO (fix de revisão, FIX4): quando o dianteiro fica À
    // FRENTE do traseiro (`cg_limit_fwd_pct_mac > cg_limit_aft_pct_mac`),
    // um intervalo "X%–Y%" com X>Y seria confuso/invertido — imprime
    // "VAZIO" explicitamente em vez disso.
    let envelope_vazio = wb.spec.cg_limit_fwd_pct_mac > wb.spec.cg_limit_aft_pct_mac;
    if envelope_vazio {
        println!("  Envelope de CG ADMISSÍVEL: **VAZIO** (dianteiro {:.1}% MAC > traseiro {:.1}% \
                  MAC — ver bloco [ TRIM ])",
                 wb.spec.cg_limit_fwd_pct_mac, wb.spec.cg_limit_aft_pct_mac);
    } else {
        println!("  Envelope de CG ADMISSÍVEL: {:.1}%–{:.1}% MAC  (sm_min={:.2}, dianteiro: \
                  TrimAuthorityAgent — ver bloco [ TRIM ])",
                 wb.spec.cg_limit_fwd_pct_mac, wb.spec.cg_limit_aft_pct_mac,
                 cfg.stability.sm_min);
    }
    for sc in &wb.scenarios {
        println!("    {} {:30} CG={:5.1}% MAC  SM={:.3}",
                 if sc.inside_envelope { "✓" } else { "✗" },
                 sc.name, sc.cg_pct_mac, sc.static_margin);
    }
    let all_inside_envelope = wb.scenarios.iter().all(|s| s.inside_envelope);
    println!("  Todos os cenários dentro do envelope admissível: {}\n",
             if all_inside_envelope { "✓ SIM" } else { "✗ NÃO" });

    // ── Massas Estruturais Computadas (Task 5, oew-parametrico) ────────────────
    // As 7 massas ESTRUTURAIS que entraram no OEW acima (`agents::
    // mass_model`, Raymer 15.2 GA × fatores de composto Tab. 15.4) — mesmos
    // valores de `sized.structural_masses`, com o braço de momento de cada
    // uma (mesma `ArmConfig` usada por `weight_balance::oew_items`).
    println!("[ MASSAS ESTRUTURAIS ]  (Raymer 15.2 GA × composto)");
    let sm_arms = aeronave::agents::weight_balance::ArmConfig::from_config(&cfg);
    let sm = &sized.structural_masses;
    let sm_items: [(&str, f64, f64); 7] = [
        ("asa",             sm.asa_kg,             sm_arms.wing_struct_m),
        ("fuselagem",       sm.fuselagem_kg,       sm_arms.fuselage_struct_m),
        ("emp_horizontal",  sm.emp_h_kg,           sm_arms.empenagem_cg_m),
        ("emp_vertical",    sm.emp_v_kg,
            sm_arms.empenagem_cg_m + aeronave::agents::weight_balance::EMP_VERTICAL_ARM_OFFSET_M),
        ("trem_principal",  sm.trem_principal_kg,  sm_arms.gear_main_m),
        ("trem_nariz",      sm.trem_nariz_kg,      sm_arms.gear_nose_m),
        ("tanques",         sm.tanques_kg,         sm_arms.fuel_cg_m),
    ];
    for (name, mass_kg, arm_m) in sm_items {
        println!("  {:<15} {:6.1} kg  @ {:.2} m", name, mass_kg, arm_m);
    }
    println!();

    // ── TrimAuthorityAgent — Autoridade de Profundor (flare + rotação) ─────────
    // Limite dianteiro FÍSICO do envelope de CG acima — substitui o antigo
    // proxy `stability.sm_max`. Roda depois de WB+Empenagem (consome
    // `wb.scenarios`); `orchestrator::size_aircraft` já aplicou o
    // resultado a `wb` acima (`WeightBalanceOutput::apply_trim`) antes
    // deste bloco imprimir os detalhes.
    println!("[ TRIM ] TrimAuthorityAgent — Autoridade de Profundor (flare + rotação)");
    let trim = &sized.trim;
    println!("  Autoridade calculada por geometria DATCOM/Nelson: τ={:.5} (c_e/c={:.2})  |  \
              cl_h_max_down_calc={:.4}{}  |  cl_h_max_down (operacional)={:.4}",
             trim.tau_elevator, cfg.control_surfaces.elevator_chord_frac,
             trim.cl_h_max_down_calc,
             if trim.capped_by_stall { " (LIMITADO pelo teto de stall)" } else { "" },
             trim.cl_h_max_down);
    println!("  Limite de FLARE (pouso, número único, independe do peso): {:.2}% MAC  |  \
              CL_h disp={:.3}",
             trim.flare_limit_pct_mac, trim.cl_h_available);
    // Fix de revisão (FIX1): a rotação, apesar de fisicamente depender do
    // peso, resulta INVARIANTE ao peso do cenário sob Vr=1.1·Vs0(W) — ver
    // agents::trim_authority::rotation_fwd_limit_m — por isso também é um
    // número ÚNICO, não mais "por cenário" como na primeira versão deste
    // agente.
    println!("  Limite de ROTAÇÃO (decolagem, número único — INVARIANTE ao peso do cenário sob \
              Vr=1.1·Vs0(W), ver derivação no código): {:.2}% MAC",
             trim.rotation_limit_pct_mac);
    println!("  Manobra que GOVERNA o limite dianteiro: {}", trim.governing);
    println!("  Margem de autoridade de rotação por cenário (diagnóstico, na CG/peso REAIS de \
              cada um — negativo = autoridade insuficiente):");
    for sc in &trim.rotation_margin_per_scenario {
        println!("    {:30} margem={:7.1}%{}",
                 sc.scenario, sc.rotation_authority_margin_pct,
                 if sc.rotation_authority_margin_pct < 0.0 { "  ⚠ insuficiente" } else { "" });
    }
    if trim.governing == "rotacao" {
        println!("  ⚠ ACHADO DE PROJETO: a ROTAÇÃO de decolagem governa o limite dianteiro, \
                  MAIS RESTRITIVA que a flare — o trem principal (x_main={:.2}m) fica muito \
                  atrás do CG desta célula (carga de nariz já perto do teto de 25%, ver \
                  [ AGENTE 6 ]). Decisão de projeto do layout do trem requer revisão humana \
                  — este agente NÃO ajusta gear.x_main_m automaticamente.",
                 cfg.gear.x_main_m);
    }
    if envelope_vazio {
        // Caixa de destaque com largura interna fixa (`BOX_W` caracteres) —
        // `box_line` preenche cada linha com espaços até a largura, então
        // as bordas '│' sempre alinham independente do conteúdo.
        const BOX_W: usize = 68;
        let box_line = |s: &str| println!("  │ {:<width$} │", s, width = BOX_W);
        println!("  ┌{}┐", "─".repeat(BOX_W + 2));
        box_line("⚠⚠⚠  ENVELOPE DE CG VAZIO — NENHUM CG É ADMISSÍVEL  ⚠⚠⚠");
        println!("  ├{}┤", "─".repeat(BOX_W + 2));
        box_line(&format!("Limite dianteiro ({}): {:.1}% MAC",
                           trim.governing, wb.spec.cg_limit_fwd_pct_mac));
        box_line(&format!("Limite traseiro (sm_min): {:.1}% MAC", wb.spec.cg_limit_aft_pct_mac));
        box_line("Causa: trem principal muito atrás do CG (gear.x_main_m).");
        box_line("Revisar posição do trem — decisão de projeto humana, NÃO automatizada.");
        println!("  └{}┘", "─".repeat(BOX_W + 2));
    }
    println!("  Sensibilidade a cl_h_max_down (±0.05): {:.2}→{:.2}% MAC  |  {:.2}(nominal)={:.2}% \
              |  {:.2}→{:.2}% MAC",
             trim.sensitivity.cl_h_max_down_minus, trim.sensitivity.flare_limit_pct_mac_minus,
             trim.cl_h_max_down, trim.flare_limit_pct_mac,
             trim.sensitivity.cl_h_max_down_plus, trim.sensitivity.flare_limit_pct_mac_plus);
    println!("  Sensibilidade a elevator_deflection_max_deg (±2°): {:.0}°→{:.2}% MAC  |  \
              {:.0}°(nominal)={:.2}%  |  {:.0}°→{:.2}% MAC",
             trim.sensitivity.elevator_deflection_max_deg_minus,
             trim.sensitivity.flare_limit_pct_mac_deflection_minus,
             cfg.control_surfaces.elevator_deflection_max_deg, trim.flare_limit_pct_mac,
             trim.sensitivity.elevator_deflection_max_deg_plus,
             trim.sensitivity.flare_limit_pct_mac_deflection_plus);
    // Arrasto de trim em cruzeiro (Task 4, refino-ciclo2) — já somado a
    // wing.cd_cruise/ld_ratio_cruise acima ([ AGENTE 1 ]); aqui só o
    // detalhamento do balanço de momentos que o produziu.
    println!("  Arrasto de trim em cruzeiro: CL_h_trim={:.4} ({})  |  ΔCD_trim={:.2e}  |  \
              CG de referência: '{}' ({:.1}% MAC)\n",
             trim.cl_h_trim_cruise,
             if trim.cl_h_trim_cruise >= 0.0 { "upload" } else { "download" },
             trim.cd_trim, trim.cg_reference_scenario, trim.cg_reference_pct_mac);

    // ── Diagrama V-n completo com rajadas (Task 4.3, CS 23.333/.341) ───────────
    // Roda após o WeightBalanceAgent (precisa do MTOW de envelope e da massa
    // do cenário mais LEVE dentre os cenários de carga) e antes do
    // StructuralAgent (que consome `n_design` — o fator de carga que governa
    // o dimensionamento, manobra OU rajada, o que for maior).
    println!("[ V-n ] VnDiagramAgent — Diagrama V-n completo com rajadas (CS 23.333/.341)");
    // Calculado dentro do laço de convergência (`orchestrator::
    // size_aircraft_with_max_iters`) na iteração já convergida — mesmas
    // entradas que este bloco computava localmente antes desta task,
    // valores idênticos. `mass_light_kg` recalculado aqui só para o print
    // abaixo (mesma expressão usada dentro do orchestrator).
    let vn = &sized.vn;
    let mass_light_kg = wb.scenarios.iter()
        .map(|s| s.total_mass_kg)
        .fold(f64::INFINITY, f64::min);
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
    let perf = PerformanceAgent::run(state, wing, prop, design_mtow_kg, &engine, &req,
                                      &cfg.performance);
    println!("  V_cruzeiro: {:.1}km/h  |  V_stall: {:.1}km/h",
             perf.v_cruise_kmh, perf.v_stall_kmh);
    println!("  RC (SL/MTOW): {:.2}m/s ({:.0}fpm)  |  RC (2500m): {:.2}m/s",
             perf.rc_sl_ms, perf.rc_sl_ms * 196.85, perf.rc_cruise_alt_ms);
    println!("  Teto: {:.0}m ({:.0}ft)",
             perf.service_ceiling_m, perf.service_ceiling_m * 3.281);
    println!("  Vx (melhor ângulo): {:.1}km/h  |  Vy (melhor razão): {:.1}km/h  |  \
              Gradiente (CS 23.65): {:.1}%",
             perf.vx_kmh, perf.vy_kmh, perf.climb_gradient_pct);
    println!("  Melhor planeio: {:.1}km/h  |  L/Dmax: {:.1}",
             perf.best_glide_kmh, perf.glide_ratio);
    println!("  TO pav: {:.0}m  TO grama: {:.0}m  Pouso: {:.0}m  \
              (rolagem ×1,5 — estimativa simplificada)",
             perf.to_distance_paved_m, perf.to_distance_grass_m,
             perf.landing_distance_m);
    // As duas distâncias GATEADAS (#23/#24) são as de grama — ecoadas com a
    // pista disponível ao lado para que o print sozinho já mostre folga ou
    // estouro; as pavimentadas seguem informativas.
    println!("  Sobre 15m/50ft — TO pav: {:.0}m  TO grama: {:.0}m (vs pista {:.0}m)  \
              Pouso pav: {:.0}m  Pouso grama: {:.0}m (vs pista {:.0}m)\n",
             perf.to_50ft_paved_m, perf.to_50ft_grass_m, req.runway_available_m,
             perf.ldg_50ft_m, perf.ldg_50ft_grass_m, req.runway_available_m);

    // ── Agente 5: Estrutura ───────────────────────────────────────────────────
    println!("[ AGENTE 5 ] StructuralAgent — Longarina e Flutter");
    // Massa da asa COMPUTADA (ciclo 3, `agents::mass_model` via
    // `SizedAircraft::structural_masses`) — a MESMA massa que entrou no OEW
    // do `WeightBalanceAgent`, não mais um item fixo de `[[masses.items]]`.
    let wing_mass_kg = sized.structural_masses.asa_kg;
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
    // CG mais dianteiro/traseiro REAIS dos cenários de carga
    // (`WeightBalanceAgent` — não o limite ADMISSÍVEL do envelope de CG,
    // ver bloco [ AGENTE 3 ] acima) — Task 2 (refino-ciclo2): tipback/
    // tail-strike e carga de nariz nos dois extremos usam o envelope de
    // carregamento REAL observado, não o limite físico de autoridade de
    // profundor. x_cg = x_mac_le + %MAC/100 × MAC — x_mac_le vem de
    // [wing] le_root_x_m (única fonte da posição do bordo de ataque).
    let x_cg_fwd = cfg.wing.le_root_x_m + wb.spec.cg_mac_fwd_pct / 100.0 * wb.mac_m;
    let x_cg_aft = cfg.wing.le_root_x_m + wb.spec.cg_mac_aft_pct / 100.0 * wb.mac_m;
    // Massas do trem COMPUTADAS (ciclo 3, `agents::mass_model`) — as MESMAS
    // que entraram no OEW; `LandingGearAgent` deriva a massa de uma perna
    // (atuador de retração) como metade da total.
    let mass_main_total = sized.structural_masses.trem_principal_kg;
    let mass_nose = sized.structural_masses.trem_nariz_kg;
    // Trem de pouso também dimensiona para o envelope estrutural (mesma
    // razão da StructuralAgent acima) — as cargas de pouso/solo devem
    // suportar o pior caso legal, não o MTOW de missão.
    let gear = LandingGearAgent::run(envelope_mtow_kg, x_cg_fwd, x_cg_aft, &cfg.gear, mass_main_total, mass_nose);
    println!("  Tipo: {}",     gear.gear_type);
    println!("  Bitola: {:.2}m  |  Empeno: {:.2}m  |  Anti-tombamento (lateral): {:.1}°",
             gear.track_width_m, gear.wheelbase_m, gear.tipover_angle_deg);
    println!("  Carga nariz: máx(CG dianteiro)={:.1}%  min(CG traseiro)={:.1}%  |  \
              Main: {:.0}N/perna  |  Nariz: {:.0}N",
             gear.nose_load_max_pct, gear.nose_load_min_pct,
             gear.main_gear_load_n, gear.nose_gear_load_n);
    println!("  Tipback (Raymer cap. 11): {:.1}°  |  piso: {:.1}°  |  Folga tail-strike: {:.1}°  |  piso: {:.1}°",
             gear.tipback_angle_deg, cfg.gear.tipback_min_deg,
             gear.tail_strike_margin_deg, cfg.gear.rotation_attitude_deg);
    println!("  Oleo main: {:.0}mm  |  Nariz: {:.0}mm  |  Sink rate: {:.1}m/s",
             gear.main_oleo_stroke_mm, gear.nose_oleo_stroke_mm, gear.max_sink_rate_ms);
    println!("  Pneu main: {}  |  {:.0}psi",
             gear.main_tire, gear.tire_pressure_psi);
    println!("  Retração: {:.0}s  |  Atuador: {:.0}W ({:.1}A @28V)  |  Peso total: {:.0}kg\n",
             gear.retraction_time_s, gear.actuator_power_w,
             gear.actuator_power_w / 28.0, gear.total_weight_kg);

    // Ciclo 8 (task 2): preenche `prop_clearance_critical_m` (checagem #25,
    // CS 23.925) — só possível AGORA que `gear` existe (ver docstring de
    // `PropellerSpec::prop_clearance_critical_m`).
    propeller.fill_critical_clearance(&gear, &cfg.gear, &cfg.propeller);
    println!("  Folga crítica CS 23.925 (batente + pneu murcho): {:.3}m  |  {}",
             propeller.prop_clearance_critical_m,
             if propeller.prop_clearance_critical_m > 0.0 { "✓" } else { "✗" });

    // ── Robustez à incerteza do modelo de massas (ciclo 4, task robustez) ──────
    // Pior-caso determinístico ±σ sobre as 7 massas estruturais (equações
    // Raymer cap. 15.2, incerteza típica de projeto conceitual ±10-20%,
    // Raymer/Roskam Classe II) contra os limites NOMINAIS já calculados
    // acima (`wb`/`gear`) — ver `validation::robustness` para a dedução
    // completa. Checagem #19 de `ConstraintChecker::verify` transforma cada
    // flip numa violação nomeada.
    let robustness = RobustnessAgent::run(&cfg, &engine, &req, state, wing, emp, sm, wb, &gear,
                                           &propeller, mission, &perf);
    println!("[ ROBUSTEZ ] RobustnessAgent — Incerteza de Massa Estrutural (±σ)");
    // `mtow_masstotal_kg` (achado de review, ciclo 5, Minor 7): MTOW
    // re-convergido do 3º caso adversarial (massa-total, todas as 5 massas
    // compostas ×(1+σ)) — `0.0` quando esse sizing perturbado FALHOU (ver
    // docstring de `RobustnessSpec::mtow_masstotal_kg`), nesse caso o flip
    // "Dimensionamento" (impresso no loop abaixo) documenta a causa.
    println!("  σ={:.0}%: {} flip(s)  |  MTOW nominal: {:.1}kg  |  MTOW mundo +σ (massa-total): {:.1}kg",
             robustness.sigma_mass_fraction * 100.0, robustness.flips.len(),
             design_mtow_kg, robustness.mtow_masstotal_kg);
    for flip in &robustness.flips {
        println!("    {} (caso {}): {:.2} vs limite {:.2}",
                 flip.check, flip.caso, flip.valor, flip.limite);
    }
    println!();

    // Varredura INFORMATIVA de posição do trem principal (Task 2,
    // refino-ciclo2) — o achado acima (tipback abaixo do piso de 15°) é a
    // tensão fundamental do triciclo: recuar o trem (x_main maior) melhora
    // o tipback, mas empurra o limite de rotação (TrimAuthorityAgent) para
    // trás, arriscando tirar cenários de CG dianteiro do envelope admissível
    // — e reduz ainda mais a carga de nariz (piso de 8%). Reexecuta o
    // pipeline REAL (WeightBalanceAgent + TrimAuthorityAgent, não uma
    // aproximação) para cada x_main candidato — só impresso aqui, NUNCA
    // altera `cfg`/o baseline.
    println!("  [ VARREDURA INFORMATIVA ] posição do trem principal (x_main) — NÃO altera o baseline:");
    println!("  {:>8}  {:>9}  {:>10}  {:>10}  {:>18}  {:>10}",
             "x_main", "tipback", "tail-str.", "nose_max%", "envelope CG (fwd)", "6/6 dentro");
    let x_main_candidatos = [cfg.gear.x_main_m, 3.60, 3.65, 3.70, 3.75, 3.80, 3.85];
    for &x_main in &x_main_candidatos {
        let mut cfg_var = cfg.clone();
        cfg_var.gear.x_main_m = x_main;
        match aeronave::orchestrator::size_aircraft(&cfg_var, &engine, &req) {
            Ok(sized_var) => {
                let wb_var = &sized_var.wb;
                let x_cg_fwd_var = cfg_var.wing.le_root_x_m + wb_var.spec.cg_mac_fwd_pct / 100.0 * wb_var.mac_m;
                let x_cg_aft_var = cfg_var.wing.le_root_x_m + wb_var.spec.cg_mac_aft_pct / 100.0 * wb_var.mac_m;
                let theta_var = aeronave::agents::landing_gear::tipback_angle_deg(
                    x_main, x_cg_aft_var, cfg_var.gear.h_cg_ground_m);
                let folga_var = aeronave::agents::landing_gear::tail_strike_margin_deg(
                    cfg_var.gear.tail_cone_height_m, cfg_var.gear.tail_cone_x_m, x_main);
                let nose_max_var = aeronave::agents::landing_gear::nose_load_fraction(
                    x_cg_fwd_var, cfg_var.gear.x_nose_m, x_main) * 100.0;
                let todos_dentro = wb_var.scenarios.iter().all(|s| s.inside_envelope);
                println!("  {:>7.2}m  {:>8.1}°  {:>9.1}°  {:>9.1}%  {:>17.1}%  {:>10}",
                         x_main, theta_var, folga_var, nose_max_var,
                         wb_var.spec.cg_limit_fwd_pct_mac,
                         if todos_dentro { "✓ sim" } else { "✗ não" });
            }
            Err(e) => println!("  {x_main:>7.2}m  (não convergiu: {e})"),
        }
    }
    println!();

    // ── Orçamento Elétrico (Task 5.2) ─────────────────────────────────────────
    println!("[ ELÉTRICO ] ElectricalAgent — Orçamento de Cargas");
    let electrical = ElectricalAgent::run(&cfg);
    println!("  Barramento: {:.0}V  |  Alternador: {:.0}W", electrical.bus_voltage_v, electrical.alternator_w);
    println!("  Carga contínua: {:.0}W  ({:.1}% de margem sobre o alternador)",
             electrical.continuous_load_w, electrical.margin_continuous_pct);
    println!("  Carga de pico (pior caso, todas simultâneas): {:.0}W\n", electrical.peak_load_w);

    // ── Validação Global ──────────────────────────────────────────────────────
    println!("[ VALIDAÇÃO ] Todos os requisitos do projeto:");
    sep();
    let report  = ConstraintChecker::verify(&VerifyInputs {
        req: &req, wing, prop, mtow_kg: design_mtow_kg, engine: &engine, wb,
        propeller: &propeller, perf: &perf, mission, electrical: &electrical,
        gear: &gear, gear_cfg: &cfg.gear, fuel_capacity_l: cfg.fuel_system.capacity_l,
        robustness: &robustness, prop_cfg: &cfg.propeller,
    });
    // Achado de review (ciclo 5): estes dois pisos agora são consumidos de
    // `validation::constraint_checker` (fonte única) em vez de literais
    // hardcoded aqui — `ConstraintChecker::verify` também os checa
    // nominalmente (#21/#22), então `rc_ok`/`ceil_ok` abaixo ficam
    // redundantes com `report.all_satisfied()`, mas mantidos como gates
    // explícitos no relatório de console (rótulo próprio, mais legível que
    // procurar dentro de `report.violations`).
    let rc_ok   = perf.rc_sl_ms   >= RC_SL_MIN_MS;
    let ceil_ok = perf.service_ceiling_m >= SERVICE_CEILING_MIN_M;
    let fl_ok   = struc.flutter_ok;
    let tip_ok  = gear.tipover_angle_deg < 55.0;
    // Carga de nariz (dois extremos), tipback e tail-strike (Task 2,
    // refino-ciclo2) migraram para dentro de `ConstraintChecker::verify`
    // (checks #15-#17) — substituem o antigo `nose_ok` ad hoc daqui, que só
    // checava um único CG (o traseiro) contra os dois limites. Margem
    // mínima de combustível (Task 3, refino-ciclo2, check #18) também vive
    // só em `ConstraintChecker::verify` — não há gate ad hoc equivalente
    // aqui.

    // Finding 3 da revisão final: literais de requisito hardcoded (280 km/h,
    // "≥ 8 h") divergiam de `req.*` para qualquer missão diferente da
    // missão de projeto padrão — o gate de V_cruzeiro chegava a marcar
    // "requisito não satisfeito" para missões que pedem uma velocidade
    // menor (ex.: uma missão de traslado a 200 km/h), mesmo quando a
    // aeronave sustentava com folga o requisito REAL daquela missão.
    // Rótulos agora formatados com `req.cruise_speed_min_kmh`/
    // `req.endurance_min_h` — corretos para qualquer `mission.toml`.
    let checks: [(bool, String); 9] = [
        (report.all_satisfied(),
            "Autonomia, consumo, alcance, V_stall, envelope de CG, hélice, gradiente CS 23.65, \
             tipback, tail-strike, carga de nariz, margem de combustível".to_string()),
        (perf.v_cruise_kmh >= req.cruise_speed_min_kmh,
            format!("V_cruzeiro ≥ {:.0} km/h", req.cruise_speed_min_kmh)),
        // Task 5.1 (achado da revisão, Finding 1): gate honesto sobre o
        // tempo de bloco da missão por segmentos, não mais o endurance a
        // tanque cheio/consumo constante (`prop.endurance_h`, agora
        // informativo — ver seu doc-comment em `specs.rs`).
        (mission.block_time_h >= req.endurance_min_h,
            format!("Autonomia da missão (block_time_h) ≥ {:.1} h", req.endurance_min_h)),
        (rc_ok,                               format!("RC ≥ {RC_SL_MIN_MS:.1} m/s ao nível do mar")),
        (ceil_ok,                             format!("Teto de serviço ≥ {SERVICE_CEILING_MIN_M:.0} m")),
        (fl_ok,                               "V_flutter ≥ 1.20 × VD (CS-23)".to_string()),
        (tip_ok,                              "Anti-tombamento (lateral) < 55°".to_string()),
        (all_stable,                          "Estabilidade longitudinal (todos cenários, SM>3%, referência)".to_string()),
        (all_inside_envelope,                 "Envelope de CG admissível (todos cenários, Task 4.4)".to_string()),
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
    // Finding 6c da revisão final: o rótulo "Diesel S-10" e a comparação
    // com AVGAS (fator 1,67) eram hardcoded — corretos só para um motor a
    // diesel, mas enganosos para qualquer motor a gasolina de aviação (que
    // já USA avgas — a "comparação" não faz sentido nesse caso). O
    // combustível agora é identificado por `engine.fuel.name` (dado de
    // config, `[fuel]` do TOML do motor); o fator 1,67 diesel→avgas só é
    // aplicado quando o combustível instalado é denso o bastante para ser
    // plausivelmente diesel (heurística: densidade > 0,8 kg/L — gasolina de
    // aviação fica tipicamente em ~0,72 kg/L, diesel ~0,83–0,85 kg/L).
    // Caso contrário, imprime só o custo por hora do combustível instalado,
    // sem inventar uma comparação.
    println!();
    println!("[ ECONOMIA ] Custo operacional estimado:");
    let custo_h = prop.fc_cruise_lph * cli.fuel_price_brl;
    if engine.fuel.density_kg_per_l > 0.8 {
        // 1.67: fator volumétrico diesel→avgas-equivalente (mesma energia
        // por litro de diesel consome ~1.67x em litros de avgas, dada a
        // menor densidade energética do avgas) — mantido inline, não é um
        // preço.
        let avgas_h = prop.fc_cruise_lph * 1.67 * cli.avgas_price_brl;
        println!("  {}:  R$ {:.0}/h  |  AVGAS equiv: R$ {:.0}/h  |  Economia: R$ {:.0}/h",
                 engine.fuel.name, custo_h, avgas_h, avgas_h - custo_h);
        println!("  Economia/100h de voo: R$ {:.0}", (avgas_h - custo_h) * 100.0);
    } else {
        println!("  {}:  R$ {:.0}/h  |  R$ {:.0}/100h de voo",
                 engine.fuel.name, custo_h, custo_h * 100.0);
    }

    // ── Geometria consolidada p/ CAD (Task 6.1) ─────────────────────────────────
    // Campos que já existiam internamente (`wb`, `cfg`) mas não eram
    // ecoados no JSON antes desta task — ver `specs::GeometrySpec`.
    let geometry = GeometrySpec {
        wing_le_root_x_m: cfg.wing.le_root_x_m,
        chord_root_m:     wb.chord_root_m,
        chord_tip_m:      wb.chord_tip_m,
        mac_m:            wb.mac_m,
        mac_le_x_m:       wb.mac_le_x_m,
        y_mac_m:          mac_spanwise_pos(wing.span_m, wing.taper_ratio),
        fuselage_length_m: cfg.fuselage.length_m,
        cabin_width_m:    cfg.fuselage.cabin_width_m,
        cabin_height_m:   cfg.fuselage.cabin_height_m,
    };

    // ── Dimensionamento/convergência (Task 6.1) ─────────────────────────────────
    // `sized.iterations`/`sized.constraints` já existiam em `SizedAircraft`
    // desde as Tasks 3.1/3.2 mas não eram serializados no relatório final —
    // ver `specs::SizingReport`.
    let fuel_margin_l = cfg.fuel_system.capacity_l - mission.fuel_total_l;
    let sizing = SizingReport {
        mtow_mission_kg:  design_mtow_kg,
        mtow_envelope_kg: envelope_mtow_kg,
        iterations:       sized.iterations.clone(),
        converged:        true,
        fuel_required_l:  mission.fuel_total_l,
        fuel_capacity_l:  cfg.fuel_system.capacity_l,
        fuel_margin_l,
        fuel_margin_pct:  fuel_margin_l / cfg.fuel_system.capacity_l * 100.0,
        constraints:      sized.constraints.clone(),
    };

    // ── Mapa de fidelidade por bloco (Task 6.1) ─────────────────────────────────
    // Honestidade explícita para o consumidor de CAD: blocos "preliminary"
    // exigem análise posterior (FEM, GVT, VLM/CFD conforme o caso) antes de
    // fabricação — ver doc-comment de `AircraftReport::fidelity` e
    // `docs/aircraft_spec.schema.md`.
    let mut fidelity: BTreeMap<String, String> = BTreeMap::new();
    fidelity.insert("wing".into(),
        "semi-empirical (polar por build-up: CD0 por componente + Oswald empírico)".into());
    fidelity.insert("propulsion".into(),
        "semi-empirical (curvas de catálogo do motor + BSFC paramétrico)".into());
    fidelity.insert("geometry".into(),
        "computed (derivado da configuração + WeightBalanceAgent)".into());
    fidelity.insert("empennage".into(),
        "preliminary (dimensionado por coeficiente de volume — Raymer Tab. 6.4; \
         requer VLM/CFD para eficiência real)".into());
    fidelity.insert("control_surfaces".into(),
        "preliminary (frações históricas — Raymer Tab. 6.5; requer análise de \
         autoridade/eficiência de controle)".into());
    fidelity.insert("weight".into(),
        "semi-empirical (estruturas: Raymer 15.2 GA × fatores de composto Tab. 15.4; \
         hardware: itens configurados não pesados — validar na balança)".into());
    fidelity.insert("performance".into(),
        "computed (equações de desempenho em forma fechada, atmosfera ISA padrão); CL de \
         decolagem (cl_max_to) interpolado JUNTO com o incremento de arrasto de flap parcial na \
         polar (ciclo 8, task 1: cd0_flap_to_extra = to_flap_fraction·cd0_flap_delta, semi-\
         empírico Raymer cap. 12/Hoerner) no segmento de SUBIDA da decolagem e no gradiente CS \
         23.65; a rolagem de solo de decolagem (método energético de Raymer) e a aproximação de \
         pouso (ângulo fixo, não L/D) não consomem a polar, por construção — sem incremento de \
         arrasto ali; Vy/teto de serviço seguem em configuração limpa (híbrido pré-existente, \
         fora de escopo); gradiente CS 23.65 (climb_gradient_pct) AINDA tem um viés otimista \
         REMANESCENTE (achado da revisão, pré-existente, não introduzido pelo ciclo 8): a busca \
         devolve o piso da varredura (1,05·Vs, não um máximo interior — RC/V é monotonicamente \
         decrescente na faixa modelada para esta célula), abaixo da velocidade de avaliação \
         típica da CS 23.65 (≥1,2·Vs) — no baseline real o gradiente a 1,2·Vs seria ≈12,45%, não \
         os ≈13,90% retornados (~1,45 p.p. de viés otimista); ver docstring de \
         agents::performance::best_climb_angle_ms".into());
    fidelity.insert("vn_diagram".into(),
        "computed (CS 23.333/.335/.337/.341, fórmulas fechadas)".into());
    fidelity.insert("structure".into(),
        "preliminary (vigas simplificadas — viga I equivalente; requer FEM); \
         flutter: preliminary — estimativa analítica, requer GVT (ensaio de \
         vibração em solo)".into());
    fidelity.insert("landing_gear".into(),
        "preliminary (dimensionamento estático de cargas; requer análise \
         dinâmica de pouso/afundamento)".into());
    fidelity.insert("propeller".into(),
        "semi-empirical (Mach de ponta; folga de solo ESTÁTICA — trem \
         totalmente estendido, aeronave nivelada — E folga em condição \
         CRÍTICA de CS 23.925 desde o ciclo 8: amortecedor de nariz no \
         batente + pneu murcho, checagem #25; ambas piso de projeto, não \
         verificação regulatória direta. Ciclo 9 (transferência de \
         atitude do #25): a folga crítica agora modela o PIVÔ da célula \
         sobre o trem principal (não mais uma translação vertical 1:1 do \
         nariz) — a hélice, à frente do trem de nariz, mergulha um braço \
         amplificado por um fator geométrico \
         (gear.x_main_m−propeller.prop_plane_x_m)/(gear.x_main_m−\
         gear.x_nose_m) sobre o curso do nariz/deflexão de pneu. Achado \
         honesto: no baseline real esse fator (≈1,466) vira a folga \
         crítica de +0,0325 m (PASS, simplificação 1:1) para ≈−0,064 m \
         (FAIL) — a simplificação antiga era OTIMISTA e mascarava este \
         resultado, como o achado de review do ciclo 8 previu. Ciclo 10 \
         (task 1, deflexão estática — CS 23.925 pela LETRA): o CAVEAT dos \
         mains rígidos nomeado no ciclo 9 (deflexão do amortecedor/pneu \
         principal precisaria entrar como termo aditivo, condição \
         COMPOSTA de CS 23.925) está RESOLVIDO — `[gear].h_cg_ground_m` \
         sempre foi a altura da aeronave CARREGADA, em deflexão estática \
         (não trem estendido sem carga), então os mains JÁ estão nessa \
         deflexão dentro de `ground_clearance_m`; a norma, pela letra, só \
         exige o trem CRÍTICO (nariz) no batente, os demais ficam na \
         deflexão estática que já é modelada. Não faltava termo nenhum. \
         Campo novo `[gear].static_sag_fraction` (0,33 no baseline) corrige \
         a fórmula do nariz do curso TOTAL do batente para o curso \
         RESTANTE (o amortecedor de nariz também parte da mesma deflexão \
         estática, não estendido) — dupla contagem do ciclo 9 corrigida. \
         Fator geométrico inalterado (≈1,466); folga crítica \
         ≈−0,064 m (ciclo 9) → ≈−0,0025 m (ciclo 10) — MESMO veredito \
         (checagem #25 continua FAIL), honestamente ANTI-conservador. Ver \
         docstring de PropellerSpec::prop_clearance_critical_m e \
         docs/backlog.md (item 1, RESOLVIDO ciclo 9; item 6, RESOLVIDO \
         ciclo 10). Requer mapa de \
         desempenho de hélice real do fabricante)".into());
    fidelity.insert("mission".into(),
        "computed (segmentos táxi/subida/cruzeiro/descida + equação de Breguet, \
         L/D constante em cruzeiro)".into());
    fidelity.insert("electrical".into(),
        "preliminary (soma de cargas nominais configuradas; requer análise \
         transiente/térmica real)".into());
    fidelity.insert("sizing".into(),
        "computed (laço de convergência de ponto fixo, MTOW de missão vs. OEW+combustível)".into());
    fidelity.insert("robustness".into(),
        "computed (pior-caso determinístico ±σ direcional sobre as 7 massas estruturais; \
         limites de envelope nominais — invariantes a massa; caso massa-total: re-sizing \
         completo com fatores ×(1+σ))".into());
    // task refino-ciclo2: limite dianteiro físico do envelope de CG, agora
    // com a autoridade de profundor CALCULADA por geometria DATCOM/Nelson
    // (não mais um `cl_h_max_down` de config direto — ver
    // `agents::trim_authority::cl_h_max_down_calc`). "preliminary" —
    // Cm_ac/Cm_flap continuam semi-empíricos (literatura NACA 230/Raymer
    // cap. 16), e o ajuste de Nelson (τ) em si é uma curva empírica; ainda
    // SENSÍVEL a `elevator_deflection_max_deg` (ver `trim.sensitivity` no
    // JSON, ±2°, além do ±0.05 residual em `cl_h_max_down`) — requer
    // validação em ensaio de voo antes de tratar o limite dianteiro como
    // definitivo.
    fidelity.insert("trim".into(),
        "preliminary (semi-empírico — Cm_ac/Cm_flap de literatura NACA 230/Raymer cap. 16; \
         cl_h_max_down_calc por geometria DATCOM/Nelson (τ(c_e/c), ajuste empírico); SENSÍVEL a \
         elevator_deflection_max_deg (±2°) e a cl_h_max_down (±0.05 residual), ver \
         trim.sensitivity; rotação desconsidera binário tração/arrasto/inércia (residual ≈ \
         μ_roll·(W−L_g)·h_cg); validar em ensaio de voo antes de tratar como definitivo)".into());

    // ── JSON Final ────────────────────────────────────────────────────────────
    let report_final = AircraftReport {
        schema_version:   SCHEMA_VERSION.to_string(),
        revision:         SCHEMA_VERSION.to_string(),
        validation_status: if all_ok { "PASS".to_string() } else { "FAIL".to_string() },
        wing:             wing.clone(),
        propulsion:       prop.clone(),
        geometry:         Some(geometry),
        empennage:        Some(emp.clone()),
        control_surfaces: Some(cs.clone()),
        weight:           Some(wb.spec.clone()),
        trim:             Some(trim.clone()),
        performance:      Some(perf),
        vn_diagram:       Some(vn.clone()),
        structure:        Some(struc),
        landing_gear:     Some(gear),
        propeller:        Some(propeller),
        mission:          Some(mission.clone()),
        electrical:       Some(electrical.clone()),
        sizing:           Some(sizing),
        robustness:       Some(robustness.clone()),
        fidelity,
        violations:       report.violations,
        warnings:         report.warnings,
    };

    let json = serde_json::to_string_pretty(&report_final)
        .expect("Falha ao serializar");
    std::fs::write(&cli.out, &json)
        .unwrap_or_else(|e| panic!("Falha ao escrever '{}': {e}", cli.out.display()));

    println!("\n[ SAÍDA ] {} v{} gerado — 10 agentes completos.", cli.out.display(), SCHEMA_VERSION);
    println!("\nPróximas etapas:");
    println!("  Fase 3 — CAD: FreeCad + Agente Python (socket localhost:9999)");
    println!("  Fase 4 — Plano de construção, BOM e documentação ANAC");
}
