//! O pipeline completo como FUNÇÃO — ciclo 16, spec §5.3.
//!
//! Existe porque `validation::incerteza` precisa re-executar o pipeline
//! inteiro com `[propeller].fom_static` alterado, e antes deste ciclo não
//! havia nada para chamar: o pipeline eram 1.078 linhas dentro de `main`.
//!
//! REGRA DE OURO: esta função NÃO faz varredura de banda. Ela é o que a
//! varredura chama. Chamar a varredura daqui é recursão infinita.

use crate::agents::constraint_diagram::WingLoadingReport;
use crate::agents::control_surfaces::ControlSurfacesAgent;
use crate::agents::electrical::ElectricalAgent;
use crate::agents::landing_gear::LandingGearAgent;
use crate::agents::mass_model::StructuralMasses;
use crate::agents::performance::PerformanceAgent;
use crate::agents::propeller::PropellerAgent;
use crate::agents::structural::StructuralAgent;
use crate::agents::weight_balance::WeightBalanceOutput;
use crate::models::aircraft_config::AircraftConfig;
use crate::models::engine::EngineSpec;
use crate::models::requirements::Requirements;
use crate::models::specs::*;
use crate::validation::constraint_checker::{ConstraintChecker, VerifyInputs, RC_SL_MIN_MS, SERVICE_CEILING_MIN_M};
use crate::validation::robustness::RobustnessAgent;
use crate::orchestrator::size_aircraft;

/// Um dos portões do veredito global (main.rs:641-663 antes do ciclo 16).
#[derive(Debug, Clone)]
pub struct Portao {
    /// Identidade estável — NÃO contém número que dependa da config.
    pub id: &'static str,
    pub ok: bool,
    /// Rótulo humano, pode conter números de requisito.
    pub rotulo: String,
}

#[derive(Debug)]
pub enum PipelineError {
    Sizing(String),
}

impl std::fmt::Display for PipelineError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PipelineError::Sizing(msg) => write!(f, "{}", msg),
        }
    }
}

#[derive(Debug)]
pub struct Resultado {
    pub state: crate::models::aircraft_state::AircraftState,
    pub constraints: WingLoadingReport,
    pub wing: WingSpec,
    pub prop: PropulsionSpec,
    pub mission: MissionSpec,
    pub empennage: EmpennageSpec,
    pub control_surfaces: ControlSurfacesSpec,
    pub wb: WeightBalanceOutput,
    pub structural_masses: StructuralMasses,
    pub trim: TrimSpec,
    pub propeller: PropellerSpec,
    pub perf: PerformanceSpec,
    pub vn: VnDiagramSpec,
    pub struc: StructuralSpec,
    pub gear: GearSpec,
    pub electrical: ElectricalSpec,
    pub robustness: RobustnessSpec,
    pub report: crate::validation::constraint_checker::ConstraintReport,
    pub portoes: Vec<Portao>,
    pub iterations: Vec<f64>,
    pub mission_fuel_kg: f64,
}

pub fn executa(
    cfg: &AircraftConfig,
    engine: &EngineSpec,
    req: &Requirements,
) -> Result<Resultado, PipelineError> {
    // Sizing: convergência de MTOW
    let sized = size_aircraft(cfg, engine, req)
        .map_err(|e| PipelineError::Sizing(e.to_string()))?;

    let design_mtow_kg = sized.state.mtow_kg;
    let envelope_mtow_kg = sized.wb.spec.mtow_kg;
    let state = &sized.state;
    let wing = &sized.wing;
    let prop = &sized.prop;
    let mission = &sized.mission;
    let wb = &sized.wb;
    let emp = &sized.emp;

    // Trim já aplicado a `sized.wb` por `size_aircraft` — use diretamente
    let trim = &sized.trim;

    let mut propeller = PropellerAgent::run(&cfg, engine, prop, req);
    let cs = ControlSurfacesAgent::run(wing, emp, &cfg);

    let perf = PerformanceAgent::run(state, wing, prop, design_mtow_kg, engine, req,
                                      &cfg.performance, cfg.stability.cl_ground_rotation);

    let wing_mass_kg = sized.structural_masses.asa_kg;
    let struc = StructuralAgent::run(wing, envelope_mtow_kg, wing_mass_kg, req, &cfg.structure, sized.vn.n_design);

    let x_cg_fwd = cfg.wing.le_root_x_m + wb.spec.cg_mac_fwd_pct / 100.0 * wb.mac_m;
    let x_cg_aft = cfg.wing.le_root_x_m + wb.spec.cg_mac_aft_pct / 100.0 * wb.mac_m;
    let mass_main_total = sized.structural_masses.trem_principal_kg;
    let mass_nose = sized.structural_masses.trem_nariz_kg;
    let gear = LandingGearAgent::run(envelope_mtow_kg, x_cg_fwd, x_cg_aft, &cfg.gear, mass_main_total, mass_nose);

    propeller.fill_critical_clearance(&gear, &cfg.gear, &cfg.propeller);

    let electrical = ElectricalAgent::run(&cfg);

    let robustness = RobustnessAgent::run(&cfg, engine, req, state, wing, emp,
                                           &sized.structural_masses, wb, &gear, &propeller,
                                           mission, &perf);

    let report = ConstraintChecker::verify(&VerifyInputs {
        req, wing, prop, mtow_kg: design_mtow_kg, engine, wb,
        propeller: &propeller, perf: &perf, mission, electrical: &electrical,
        gear: &gear, gear_cfg: &cfg.gear, fuel_capacity_l: cfg.fuel_system.capacity_l,
        robustness: &robustness, prop_cfg: &cfg.propeller,
    });

    // Portões do veredito global
    let rc_ok = perf.rc_sl_ms >= RC_SL_MIN_MS;
    let ceil_ok = perf.service_ceiling_m >= SERVICE_CEILING_MIN_M;
    let fl_ok = struc.flutter_ok;
    let tip_ok = gear.tipover_angle_deg < 55.0;
    let all_stable = wb.scenarios.iter().all(|s| s.stable);
    let all_inside_envelope = wb.scenarios.iter().all(|s| s.inside_envelope);

    let portoes = vec![
        Portao {
            id: "portao_restricoes",
            ok: report.all_satisfied(),
            rotulo: "Autonomia, consumo, alcance, V_stall, envelope de CG, hélice, gradiente CS 23.65, \
                     tipback, tail-strike, carga de nariz, margem de combustível".to_string(),
        },
        Portao {
            id: "portao_v_cruzeiro",
            ok: perf.v_cruise_kmh >= req.cruise_speed_min_kmh,
            rotulo: format!("V_cruzeiro ≥ {:.0} km/h", req.cruise_speed_min_kmh),
        },
        Portao {
            id: "portao_autonomia_bloco",
            ok: mission.block_time_h >= req.endurance_min_h,
            rotulo: format!("Autonomia da missão (block_time_h) ≥ {:.1} h", req.endurance_min_h),
        },
        Portao {
            id: "portao_rc_sl",
            ok: rc_ok,
            rotulo: format!("RC ≥ {RC_SL_MIN_MS:.1} m/s ao nível do mar"),
        },
        Portao {
            id: "portao_teto_servico",
            ok: ceil_ok,
            rotulo: format!("Teto de serviço ≥ {SERVICE_CEILING_MIN_M:.0} m"),
        },
        Portao {
            id: "portao_flutter",
            ok: fl_ok,
            rotulo: "V_flutter ≥ 1.20 × VD (CS-23)".to_string(),
        },
        Portao {
            id: "portao_antitombamento",
            ok: tip_ok,
            rotulo: "Anti-tombamento (lateral) < 55°".to_string(),
        },
        Portao {
            id: "portao_estabilidade_long",
            ok: all_stable,
            rotulo: "Estabilidade longitudinal (todos cenários, SM>3%, referência)".to_string(),
        },
        Portao {
            id: "portao_envelope_cg_todos",
            ok: all_inside_envelope,
            rotulo: "Envelope de CG admissível (todos cenários, Task 4.4)".to_string(),
        },
    ];

    Ok(Resultado {
        state: state.clone(),
        constraints: sized.constraints.clone(),
        wing: wing.clone(),
        prop: prop.clone(),
        mission: mission.clone(),
        empennage: emp.clone(),
        control_surfaces: cs.clone(),
        wb: WeightBalanceOutput {
            spec: wb.spec.clone(),
            oew_kg: wb.oew_kg,
            chord_root_m: wb.chord_root_m,
            chord_tip_m: wb.chord_tip_m,
            mac_m: wb.mac_m,
            mac_le_x_m: wb.mac_le_x_m,
            x_np_m: wb.x_np_m,
            scenarios: wb.scenarios.clone(),
        },
        structural_masses: sized.structural_masses.clone(),
        trim: trim.clone(),
        propeller: propeller.clone(),
        perf: perf.clone(),
        vn: sized.vn.clone(),
        struc: struc.clone(),
        gear: gear.clone(),
        electrical: electrical.clone(),
        robustness: robustness.clone(),
        report,
        portoes,
        iterations: sized.iterations.clone(),
        mission_fuel_kg: sized.mission_fuel_kg,
    })
}
