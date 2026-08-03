use super::aircraft_config::AircraftConfig;

/// Estado mutável da aeronave durante o loop de otimização.
/// O Orchestrator ajusta estes parâmetros entre iterações até
/// todos os requisitos serem satisfeitos.
///
/// Construído a partir de uma `AircraftConfig` (ver `from_config`) — a
/// aeronave inteira (geometria, perfil aerodinâmico, propulsão, trem) é
/// dado de configuração TOML, não constante Rust.
#[derive(Debug, Clone)]
pub struct AircraftState {
    // --- Geometria da asa ---
    pub wing_span_m: f64,
    pub wing_area_m2: f64,
    pub taper_ratio: f64,
    pub airfoil: String,
    /// Espessura relativa do perfil (t/c) — usada no dimensionamento
    /// estrutural da longarina.
    pub thickness_ratio: f64,

    // --- Perfil aerodinâmico e build-up de arrasto ---
    /// CL_max em configuração limpa (cruzeiro, sem flap) — usado para VS1.
    pub cl_max_clean: f64,
    /// CL_max com flap/slat (pouso/decolagem) — usado para VS0.
    pub cl_max_flaps: f64,
    pub cd0_wing: f64,
    pub cd0_fuselage: f64,
    pub cd0_empennage: f64,
    /// CD0 residual (antenas, juntas, imperfeições).
    pub cd0_misc: f64,
    /// Incremento de CD0 do trem FIXO — só se soma quando `!gear_retractable`.
    pub cd0_gear_fixed_increment: f64,

    // --- Peso (estimativa inicial para o laço iterativo de projeto) ---
    pub mtow_kg: f64,

    // --- Propulsão ---
    pub psru_ratio: f64,
    pub prop_diameter_m: f64,
    pub fuel_capacity_l: f64,

    // --- Trem de pouso ---
    pub gear_retractable: bool,
}

impl AircraftState {
    /// Constrói o estado inicial da aeronave a partir de uma configuração
    /// carregada de TOML (ver `models::config::load_aircraft`).
    pub fn from_config(cfg: &AircraftConfig) -> Self {
        Self {
            wing_span_m: cfg.wing.span_m,
            wing_area_m2: cfg.wing.area_m2,
            taper_ratio: cfg.wing.taper_ratio,
            airfoil: cfg.wing.airfoil.clone(),
            thickness_ratio: cfg.wing.thickness_ratio,

            cl_max_clean: cfg.wing.cl_max_clean,
            cl_max_flaps: cfg.wing.cl_max_flaps,
            cd0_wing: cfg.wing.cd0_wing,
            cd0_fuselage: cfg.fuselage.cd0,
            cd0_empennage: cfg.empennage.cd0,
            cd0_misc: cfg.drag.cd0_misc,
            cd0_gear_fixed_increment: cfg.gear.cd0_fixed_increment,

            mtow_kg: cfg.mtow_guess_kg,

            psru_ratio: cfg.propeller.psru_ratio,
            prop_diameter_m: cfg.propeller.diameter_m,
            fuel_capacity_l: cfg.fuel_system.capacity_l,

            gear_retractable: cfg.gear.retractable,
        }
    }

    pub fn aspect_ratio(&self) -> f64 {
        self.wing_span_m.powi(2) / self.wing_area_m2
    }
}
