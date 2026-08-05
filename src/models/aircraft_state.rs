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
    /// CD0 da empenagem — task refino-ciclo2 (1b): deixou de ser um eco
    /// direto de `[empennage].cd0` (campo REMOVIDO da config) e passou a
    /// ser DERIVADO de `[empennage].cd0_area_factor·(S_h+S_v)/S_w` — ver
    /// `AircraftState::from_config` para o cálculo (usa
    /// `agents::empennage::tail_areas_m2`, que não depende de
    /// `AerodynamicsAgent` já ter rodado).
    pub cd0_empennage: f64,
    /// CD0 residual (antenas, juntas, imperfeições).
    pub cd0_misc: f64,
    /// Incremento de CD0 do trem FIXO — só se soma quando `!gear_retractable`.
    pub cd0_gear_fixed_increment: f64,
    /// Fração do arrasto parasita total atribuída à refrigeração do motor
    /// (Task 5.2) — aplicada como multiplicador `(1 + fração)` sobre o CD0
    /// já somado em `agents::aerodynamics::cd0_total`.
    pub cooling_drag_fraction: f64,

    // --- Peso (estimativa inicial para o laço iterativo de projeto) ---
    pub mtow_kg: f64,

    // --- Propulsão ---
    pub psru_ratio: f64,
    /// Eficiência mecânica do PSRU (correia/engrenagens, `[propeller]
    /// psru_efficiency` do TOML) — Finding 1 da revisão final: antes vinha de
    /// um `const PSRU_EFFICIENCY = 0.97` hardcoded em `agents::propulsion`
    /// que IGNORAVA este campo (validado por `models::config` mas nunca
    /// lido pela física). Agora é dado de configuração de ponta a ponta —
    /// todo cálculo que envolve perdas mecânicas do PSRU (busca de rpm de
    /// cruzeiro, consumo, potência de eixo em `agents::performance`, Breguet
    /// em `agents::mission`, P/W do diagrama de restrições) lê este campo.
    pub psru_efficiency: f64,
    pub prop_diameter_m: f64,
    pub fuel_capacity_l: f64,

    // --- Trem de pouso ---
    pub gear_retractable: bool,
}

impl AircraftState {
    /// Constrói o estado inicial da aeronave a partir de uma configuração
    /// carregada de TOML (ver `models::config::load_aircraft`).
    pub fn from_config(cfg: &AircraftConfig) -> Self {
        // CD0 da empenagem (task refino-ciclo2, 1b) — derivado da área
        // REALMENTE dimensionada (S_h+S_v, `agents::empennage::
        // tail_areas_m2`), não mais um valor fixo de config. `tail_areas_m2`
        // usa só geometria de `[wing]`/`[empennage]` (span/área/afilamento
        // — sem nenhuma dependência de MTOW/cd0), então pode ser calculada
        // aqui, ANTES do `AerodynamicsAgent` rodar, sem reordenar o laço de
        // convergência em `orchestrator::size_aircraft` — ver docstring de
        // `tail_areas_m2` para a dedução completa.
        let (s_h, s_v) = crate::agents::empennage::tail_areas_m2(cfg);
        let cd0_empennage = cfg.empennage.cd0_area_factor * (s_h + s_v) / cfg.wing.area_m2;

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
            cd0_empennage,
            cd0_misc: cfg.drag.cd0_misc,
            cd0_gear_fixed_increment: cfg.gear.cd0_fixed_increment,
            cooling_drag_fraction: cfg.drag.cooling_drag_fraction,

            mtow_kg: cfg.sizing.mtow_initial_guess_kg,

            psru_ratio: cfg.propeller.psru_ratio,
            psru_efficiency: cfg.propeller.psru_efficiency,
            // Quando `[propeller].diameter_m` está presente, usa-o
            // diretamente (comportamento inalterado). Quando OMITIDO, usa
            // como palpite provisório o maior diâmetro que respeita a folga
            // de solo (`agents::propeller::diameter_max_by_clearance_m`,
            // única restrição calculável aqui sem `EngineSpec`/
            // `Requirements`, que `from_config` não recebe) — o valor
            // AUTORITATIVO, que também respeita os limites de Mach de ponta
            // (estático e cruzeiro, usando o rpm de cruzeiro real da busca
            // de BSFC), é calculado por `agents::propeller::PropellerAgent`,
            // rodado após o laço de convergência de MTOW (ver `main.rs`).
            prop_diameter_m: cfg.propeller.diameter_m.unwrap_or_else(|| {
                crate::agents::propeller::round_down_cm(
                    crate::agents::propeller::diameter_max_by_clearance_m(
                        cfg.propeller.shaft_height_m,
                        cfg.propeller.ground_clearance_min_m,
                    ) - 0.02,
                )
            }),
            fuel_capacity_l: cfg.fuel_system.capacity_l,

            gear_retractable: cfg.gear.retractable,
        }
    }

    pub fn aspect_ratio(&self) -> f64 {
        self.wing_span_m.powi(2) / self.wing_area_m2
    }
}
