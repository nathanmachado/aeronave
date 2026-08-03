mod agents;
mod models;
mod validation;

use agents::aerodynamics::AerodynamicsAgent;
use agents::propulsion::PropulsionAgent;
use agents::weight_balance::WeightBalanceAgent;
use agents::performance::PerformanceAgent;
use agents::structural::StructuralAgent;
use agents::landing_gear::LandingGearAgent;
use models::aircraft_state::AircraftState;
use models::requirements::Requirements;
use models::specs::AircraftReport;
use validation::constraint_checker::ConstraintChecker;

fn sep() { println!("{}", "─".repeat(64)); }

fn main() {
    println!("╔══════════════════════════════════════════════════════════════╗");
    println!("║   AERONAVE — Modelagem Matemática v3.0  (6 Agentes)         ║");
    println!("║   Motor: Toyota 1GD-FTV 2.8T  |  Trem: Retrátil Elétrico   ║");
    println!("╚══════════════════════════════════════════════════════════════╝\n");

    let state = AircraftState::initial();
    let req   = Requirements::project_default();

    // ── Agente 1: Aerodinâmica ────────────────────────────────────────────────
    println!("[ AGENTE 1 ] AerodynamicsAgent");
    let wing = AerodynamicsAgent::run(&state, &req);
    println!("  Envergadura: {:.2}m  |  Área: {:.1}m²  |  AR: {:.2}",
             wing.span_m, wing.area_m2, wing.aspect_ratio);
    println!("  Perfil: {}  |  e={:.3}  |  L/D={:.1}",
             wing.airfoil, wing.oswald_efficiency, wing.ld_ratio_cruise);
    println!("  CD0={:.4}  CL_cruise={:.3}  CD_cruise={:.4}  CL_max(flap)={:.2}  CL_max(limpa)={:.2}",
             wing.cd0, wing.cl_cruise, wing.cd_cruise, wing.cl_max, wing.cl_max_clean);
    println!("  VS0 (flap, SL): {:.1} km/h  |  VS1 (limpa, SL): {:.1} km/h\n",
             wing.stall_speed_flaps_kmh, wing.stall_speed_clean_kmh);

    // ── Agente 2: Propulsão ───────────────────────────────────────────────────
    println!("[ AGENTE 2 ] PropulsionAgent — Toyota 1GD-FTV 2.8T + PSRU");
    let prop = PropulsionAgent::run(&state, &req, &wing);
    println!("  {:.0} hp / {:.1} kW  |  {:.0} Nm  |  PSRU {:.3}:1",
             prop.power_hp, prop.power_kw, prop.max_torque_nm, prop.psru_ratio);
    println!("  Hélice {:.0} rpm  Ø{:.2}m  η={:.1}%  Tração: {:.0}N",
             prop.prop_rpm_cruise, prop.prop_diameter_m,
             prop.prop_efficiency * 100.0, prop.thrust_cruise_n);
    println!("  {}  |  {:.0}L  |  {:.1}L/h  |  BSFC {:.0}g/kWh",
             prop.fuel_type, prop.fuel_capacity_l,
             prop.fc_cruise_lph, prop.bsfc_cruise_gkwh);
    println!("  Autonomia: {:.2}h  |  Alcance: {:.0}km\n",
             prop.endurance_h, prop.range_km);

    // ── Agente 3: Peso e Balanceamento ────────────────────────────────────────
    println!("[ AGENTE 3 ] WeightBalanceAgent — CG e Estabilidade");
    let wb = WeightBalanceAgent::run(&state, &wing);
    println!("  Corda: raiz {:.3}m  ponta {:.3}m  MAC {:.3}m",
             wb.chord_root_m, wb.chord_tip_m, wb.mac_m);
    println!("  OEW: {:.1}kg  |  MTOW: {:.1}kg  |  NP: {:.3}m do nariz",
             wb.oew_kg, wb.spec.mtow_kg, wb.x_np_m);
    println!("  Envelope CG: {:.1}%–{:.1}% MAC  |  SM mín: {:.1}%",
             wb.spec.cg_mac_fwd_pct, wb.spec.cg_mac_aft_pct,
             wb.spec.static_margin_pct);
    let all_stable = wb.scenarios.iter().all(|s| s.stable);
    println!("  Todos os cenários estáveis: {}\n",
             if all_stable { "✓ SIM" } else { "✗ NÃO" });

    // ── Agente 4: Desempenho ──────────────────────────────────────────────────
    println!("[ AGENTE 4 ] PerformanceAgent");
    let perf = PerformanceAgent::run(&state, &wing, &prop, wb.spec.mtow_kg);
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
    let struc = StructuralAgent::run(&wing, wb.spec.mtow_kg, 130.0);
    println!("  Fator de carga: {:.1}g limite  |  {:.1}g último (CS-23 Normal)",
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
    // x_cg = x_mac_le + 0.291 × MAC = 2.90 + 0.291 × 1.246 ≈ 3.263m
    let x_cg_aft = 2.90 + wb.spec.cg_mac_aft_pct / 100.0 * wb.mac_m;
    let gear = LandingGearAgent::run(wb.spec.mtow_kg, x_cg_aft, 1.40, 3.85);
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
    let report  = ConstraintChecker::verify(&req, &wing, &prop, wb.spec.mtow_kg);
    let rc_ok   = perf.rc_sl_ms   >= 1.5;
    let ceil_ok = perf.service_ceiling_m >= 3_000.0;
    let fl_ok   = struc.flutter_ok;
    let tip_ok  = gear.tipover_angle_deg < 55.0;
    let nose_ok = gear.nose_load_fraction_pct >= 8.0 && gear.nose_load_fraction_pct <= 25.0;

    let checks = [
        (report.all_satisfied(),              "Autonomia, consumo, alcance, V_stall"),
        (perf.v_cruise_kmh >= 280.0,          "V_cruzeiro ≥ 280 km/h"),
        (prop.endurance_h  >= 8.0,            "Autonomia ≥ 8 h"),
        (rc_ok,                               "RC ≥ 1.5 m/s ao nível do mar"),
        (ceil_ok,                             "Teto de serviço ≥ 3.000 m"),
        (fl_ok,                               "V_flutter ≥ 1.20 × VD (CS-23)"),
        (tip_ok,                              "Anti-tombamento < 55°"),
        (nose_ok,                             "Carga nariz 8–25%"),
        (all_stable,                          "Estabilidade longitudinal (todos cenários)"),
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
    let custo_h = prop.fc_cruise_lph * 6.5;
    let avgas_h = prop.fc_cruise_lph * 1.67 * 18.0;
    println!("  Diesel S-10:  R$ {:.0}/h  |  AVGAS equiv: R$ {:.0}/h  |  Economia: R$ {:.0}/h",
             custo_h, avgas_h, avgas_h - custo_h);
    println!("  Economia/100h de voo: R$ {:.0}", (avgas_h - custo_h) * 100.0);

    // ── JSON Final ────────────────────────────────────────────────────────────
    let report_final = AircraftReport {
        revision:         "3.0".to_string(),
        validation_status: if all_ok { "PASS".to_string() } else { "FAIL".to_string() },
        wing:             wing.clone(),
        propulsion:       prop.clone(),
        weight:           Some(wb.spec),
        performance:      Some(perf),
        structure:        Some(struc),
        landing_gear:     Some(gear),
        violations:       report.violations,
    };

    let json = serde_json::to_string_pretty(&report_final)
        .expect("Falha ao serializar");
    std::fs::write("aircraft_spec.json", &json)
        .expect("Falha ao escrever aircraft_spec.json");

    println!("\n[ SAÍDA ] aircraft_spec.json v3.0 gerado — 6 agentes completos.");
    println!("\nPróximas etapas:");
    println!("  Fase 3 — CAD: FreeCad + Agente Python (socket localhost:9999)");
    println!("  Fase 4 — Plano de construção, BOM e documentação ANAC");
}
