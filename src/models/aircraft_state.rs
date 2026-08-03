/// Estado mutável da aeronave durante o loop de otimização.
/// O Orchestrator ajusta estes parâmetros entre iterações até
/// todos os requisitos serem satisfeitos.
#[derive(Debug, Clone)]
pub struct AircraftState {
    // --- Geometria da asa ---
    pub wing_span_m: f64,
    pub wing_area_m2: f64,
    pub taper_ratio: f64,
    pub airfoil: String,

    // --- Peso estimado (realimentado pelo WeightBalanceAgent) ---
    pub mtow_kg: f64,
    pub oew_kg: f64,

    // --- Propulsão ---
    pub psru_ratio: f64,
    pub prop_diameter_m: f64,
    pub fuel_capacity_l: f64,

    // --- Trem de pouso ---
    pub gear_retractable: bool,
    pub cd0_gear_increment: f64,   // 0.0 se retrátil, ~0.008 se fixo
}

impl AircraftState {
    /// Ponto de partida do projeto v2.0 (motor turbo diesel ~204hp, trem retrátil)
    pub fn initial() -> Self {
        Self {
            wing_span_m: 11.94,
            wing_area_m2: 14.2,
            taper_ratio: 0.45,
            airfoil: "NACA 23015".to_string(),

            mtow_kg: 1_461.0,
            oew_kg: 1_021.0,

            psru_ratio: 1.867,
            prop_diameter_m: 1.95,
            fuel_capacity_l: 240.0,  // 240L: 8.18h autonomia @ 26.4 L/h (margem de +11 min)

            gear_retractable: true,
            cd0_gear_increment: 0.0,
        }
    }

    pub fn aspect_ratio(&self) -> f64 {
        self.wing_span_m.powi(2) / self.wing_area_m2
    }
}
