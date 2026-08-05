//! Carregamento e validação de especificações de motor e de célula a partir
//! de arquivos TOML.

use std::path::Path;

use super::aircraft_config::AircraftConfig;
use super::engine::EngineSpec;
use super::requirements::Requirements;

/// Erros de carregamento/validação de configuração de motor.
#[derive(Debug)]
pub enum ConfigError {
    Io(std::io::Error),
    Parse(toml::de::Error),
    Validation(String),
}

impl std::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConfigError::Io(e) => write!(f, "erro ao ler arquivo de configuração: {e}"),
            ConfigError::Parse(e) => write!(f, "TOML de configuração inválido: {e}"),
            ConfigError::Validation(msg) => write!(f, "{msg}"),
        }
    }
}

impl std::error::Error for ConfigError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            ConfigError::Io(e) => Some(e),
            ConfigError::Parse(e) => Some(e),
            ConfigError::Validation(_) => None,
        }
    }
}

impl From<std::io::Error> for ConfigError {
    fn from(e: std::io::Error) -> Self {
        ConfigError::Io(e)
    }
}

impl From<toml::de::Error> for ConfigError {
    fn from(e: toml::de::Error) -> Self {
        ConfigError::Parse(e)
    }
}

/// Faz o parse de uma especificação de motor a partir de uma string TOML,
/// validando as invariantes físicas da curva de torque e dos limites de rpm.
pub fn parse_engine(toml_str: &str) -> Result<EngineSpec, ConfigError> {
    let engine: EngineSpec = toml::from_str(toml_str)?;
    validate_engine(&engine)?;
    Ok(engine)
}

/// Lê e faz o parse de uma especificação de motor a partir de um arquivo TOML no disco.
pub fn load_engine(path: &Path) -> Result<EngineSpec, ConfigError> {
    let content = std::fs::read_to_string(path).map_err(|e| {
        ConfigError::Io(std::io::Error::new(
            e.kind(),
            format!("não foi possível ler o arquivo de motor '{}': {e}", path.display()),
        ))
    })?;
    parse_engine(&content)
}

/// Valida as invariantes físicas de uma `EngineSpec` recém-carregada.
fn validate_engine(engine: &EngineSpec) -> Result<(), ConfigError> {
    if engine.torque_curve.len() < 2 {
        return Err(ConfigError::Validation(format!(
            "curva de torque inválida: são necessários pelo menos 2 pontos, encontrados {}",
            engine.torque_curve.len()
        )));
    }

    // Valida mass_kg é finito
    if !engine.mass_kg.is_finite() {
        return Err(ConfigError::Validation(format!(
            "configuração inválida: mass_kg deve ser finitos (valores NaN/infinito não permitidos)"
        )));
    }

    // Valida rpm_idle é finito
    if !engine.rpm_idle.is_finite() {
        return Err(ConfigError::Validation(format!(
            "configuração inválida: rpm_idle deve ser finitos (valores NaN/infinito não permitidos)"
        )));
    }

    // Valida rpm_rated é finito
    if !engine.rpm_rated.is_finite() {
        return Err(ConfigError::Validation(format!(
            "configuração inválida: rpm_rated deve ser finitos (valores NaN/infinito não permitidos)"
        )));
    }

    // Valida rpm_redline é finito
    if !engine.rpm_redline.is_finite() {
        return Err(ConfigError::Validation(format!(
            "configuração inválida: rpm_redline deve ser finitos (valores NaN/infinito não permitidos)"
        )));
    }

    // Valida rpm_max_continuous é finito
    if !engine.rpm_max_continuous.is_finite() {
        return Err(ConfigError::Validation(format!(
            "configuração inválida: rpm_max_continuous deve ser finitos (valores NaN/infinito não permitidos)"
        )));
    }

    let mut rpm_anterior: Option<f64> = None;
    for (i, [rpm, torque]) in engine.torque_curve.iter().enumerate() {
        // Valida RPM é finito
        if !rpm.is_finite() {
            return Err(ConfigError::Validation(format!(
                "curva de torque inválida: rpm deve ser finitos no ponto {i} (valores NaN/infinito não permitidos)"
            )));
        }
        // Valida torque é finito
        if !torque.is_finite() {
            return Err(ConfigError::Validation(format!(
                "curva de torque inválida: torque deve ser finitos no ponto {i} (valores NaN/infinito não permitidos)"
            )));
        }
        if *rpm < 0.0 {
            return Err(ConfigError::Validation(format!(
                "curva de torque inválida: rpm negativo no ponto {i} ({rpm})"
            )));
        }
        if *torque < 0.0 {
            return Err(ConfigError::Validation(format!(
                "curva de torque inválida: torque negativo no ponto {i} ({torque} Nm)"
            )));
        }
        if let Some(anterior) = rpm_anterior {
            if *rpm <= anterior {
                return Err(ConfigError::Validation(format!(
                    "curva de torque inválida: rpm deve ser estritamente crescente (ponto {i}: {rpm} rpm não é maior que {anterior} rpm)"
                )));
            }
        }
        rpm_anterior = Some(*rpm);
    }

    // Valida BSFC fields são finitos
    if !engine.bsfc.bsfc_min_gkwh.is_finite() {
        return Err(ConfigError::Validation(format!(
            "configuração inválida: bsfc_min_gkwh deve ser finitos (valores NaN/infinito não permitidos)"
        )));
    }
    if !engine.bsfc.rpm_optimal.is_finite() {
        return Err(ConfigError::Validation(format!(
            "configuração inválida: bsfc.rpm_optimal deve ser finitos (valores NaN/infinito não permitidos)"
        )));
    }
    if !engine.bsfc.load_optimal.is_finite() {
        return Err(ConfigError::Validation(format!(
            "configuração inválida: bsfc.load_optimal deve ser finitos (valores NaN/infinito não permitidos)"
        )));
    }
    if !engine.bsfc.rpm_penalty_gkwh.is_finite() {
        return Err(ConfigError::Validation(format!(
            "configuração inválida: bsfc.rpm_penalty_gkwh deve ser finitos (valores NaN/infinito não permitidos)"
        )));
    }
    if !engine.bsfc.load_penalty_gkwh.is_finite() {
        return Err(ConfigError::Validation(format!(
            "configuração inválida: bsfc.load_penalty_gkwh deve ser finitos (valores NaN/infinito não permitidos)"
        )));
    }
    if !engine.bsfc.bsfc_max_gkwh.is_finite() {
        return Err(ConfigError::Validation(format!(
            "configuração inválida: bsfc_max_gkwh deve ser finitos (valores NaN/infinito não permitidos)"
        )));
    }

    // Valida fuel fields são finitos
    if !engine.fuel.density_kg_per_l.is_finite() {
        return Err(ConfigError::Validation(format!(
            "configuração inválida: fuel.density_kg_per_l deve ser finitos (valores NaN/infinito não permitidos)"
        )));
    }
    if !engine.fuel.lhv_mj_per_kg.is_finite() {
        return Err(ConfigError::Validation(format!(
            "configuração inválida: fuel.lhv_mj_per_kg deve ser finitos (valores NaN/infinito não permitidos)"
        )));
    }

    if engine.rpm_max_continuous > engine.rpm_redline {
        return Err(ConfigError::Validation(format!(
            "configuração inválida: rpm_max_continuous ({}) não pode ser maior que rpm_redline ({})",
            engine.rpm_max_continuous, engine.rpm_redline
        )));
    }

    Ok(())
}

// ─── AERONAVE (CÉLULA) ────────────────────────────────────────────────────────

/// Faz o parse de uma configuração de célula a partir de uma string TOML,
/// validando as invariantes físicas e de consistência (braços conhecidos,
/// material estrutural cadastrado, etc).
pub fn parse_aircraft(toml_str: &str) -> Result<AircraftConfig, ConfigError> {
    check_sm_max_migration(toml_str)?;
    let cfg: AircraftConfig = toml::from_str(toml_str)?;
    validate_aircraft(&cfg)?;
    Ok(cfg)
}

/// Guarda de migração (task trim-authority): `[stability].sm_max` foi
/// REMOVIDO — o limite dianteiro do envelope de CG agora é calculado
/// fisicamente pelo `TrimAuthorityAgent` a partir de `[stability].
/// cl_h_max_down`/`trim_margin`/`cl_ground_rotation`/`to_flap_cm_fraction` +
/// `[wing].cm_ac`/`cm_flap_delta` (ver `models::aircraft_config::
/// StabilityCfg`). Como `AircraftConfig`/`StabilityCfg` não usam
/// `#[serde(deny_unknown_fields)]` em lugar nenhum deste crate, um TOML
/// antigo com `sm_max` seria simplesmente IGNORADO pelo parser (silêncio,
/// não erro) — a configuração carregaria "com sucesso" usando um envelope
/// dianteiro completamente diferente do que o operador pretendia, sem
/// nenhum aviso. Faz o parse do TOML bruto como `toml::Value` primeiro só
/// para checar essa chave específica e falhar alto e claro, ANTES do parse
/// tipado — não é validação física (isso é `validate_aircraft`), é
/// detecção de configuração de uma versão anterior do schema.
fn check_sm_max_migration(toml_str: &str) -> Result<(), ConfigError> {
    let raw: toml::Value = toml::from_str(toml_str)?;
    if raw.get("stability").and_then(|s| s.get("sm_max")).is_some() {
        return Err(ConfigError::Validation(
            "configuração de aeronave inválida: [stability].sm_max foi substituído por um \
             limite dianteiro de CG calculado fisicamente (TrimAuthorityAgent) — remova \
             sm_max e adicione [stability].cl_h_max_down/trim_margin/cl_ground_rotation/\
             to_flap_cm_fraction + [wing].cm_ac/cm_flap_delta (ver docs/aircraft_spec.schema.md \
             e config/aircraft/baseline_4seat.toml para valores de referência)"
                .to_string(),
        ));
    }
    Ok(())
}

/// Lê e faz o parse de uma configuração de célula a partir de um arquivo
/// TOML no disco.
pub fn load_aircraft(path: &Path) -> Result<AircraftConfig, ConfigError> {
    let content = std::fs::read_to_string(path).map_err(|e| {
        ConfigError::Io(std::io::Error::new(
            e.kind(),
            format!("não foi possível ler o arquivo de aeronave '{}': {e}", path.display()),
        ))
    })?;
    parse_aircraft(&content)
}

/// Garante que `v` é finito (nem NaN nem infinito), com mensagem em
/// português nomeando o campo — mesmo padrão usado em `validate_engine`.
fn require_finite(field: &str, v: f64) -> Result<(), ConfigError> {
    if !v.is_finite() {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: {field} deve ser finito (valores NaN/infinito não permitidos)"
        )));
    }
    Ok(())
}

fn require_positive(field: &str, v: f64) -> Result<(), ConfigError> {
    require_finite(field, v)?;
    if v <= 0.0 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: {field} deve ser positivo (valor: {v})"
        )));
    }
    Ok(())
}

fn require_non_negative(field: &str, v: f64) -> Result<(), ConfigError> {
    require_finite(field, v)?;
    if v < 0.0 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: {field} não pode ser negativo (valor: {v})"
        )));
    }
    Ok(())
}

/// Garante que `v` (fração) é finito e está em (0, 1] — usado pelos dez
/// campos de `[control_surfaces]` (Task 4.2).
fn require_frac(field: &str, v: f64) -> Result<(), ConfigError> {
    require_finite(field, v)?;
    if v <= 0.0 || v > 1.0 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: {field} deve estar em (0, 1] (valor: {v})"
        )));
    }
    Ok(())
}

/// Valida as invariantes físicas e de consistência de uma `AircraftConfig`
/// recém-carregada.
fn validate_aircraft(cfg: &AircraftConfig) -> Result<(), ConfigError> {
    // [sizing]
    require_positive("sizing.mtow_initial_guess_kg", cfg.sizing.mtow_initial_guess_kg)?;
    require_positive("sizing.mtow_max_kg", cfg.sizing.mtow_max_kg)?;
    if cfg.sizing.mtow_initial_guess_kg >= cfg.sizing.mtow_max_kg {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: sizing.mtow_initial_guess_kg ({}) deve ser \
             menor que sizing.mtow_max_kg ({})",
            cfg.sizing.mtow_initial_guess_kg, cfg.sizing.mtow_max_kg
        )));
    }

    // [wing]
    require_positive("wing.span_m", cfg.wing.span_m)?;
    require_positive("wing.area_m2", cfg.wing.area_m2)?;
    require_finite("wing.taper_ratio", cfg.wing.taper_ratio)?;
    if cfg.wing.taper_ratio <= 0.0 || cfg.wing.taper_ratio > 1.0 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: wing.taper_ratio deve estar em (0, 1] (valor: {})",
            cfg.wing.taper_ratio
        )));
    }
    require_positive("wing.thickness_ratio", cfg.wing.thickness_ratio)?;
    require_positive("wing.cl_max_clean", cfg.wing.cl_max_clean)?;
    require_positive("wing.cl_max_flaps", cfg.wing.cl_max_flaps)?;
    if cfg.wing.cl_max_flaps < cfg.wing.cl_max_clean {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: wing.cl_max_flaps ({}) deve ser >= wing.cl_max_clean ({})",
            cfg.wing.cl_max_flaps, cfg.wing.cl_max_clean
        )));
    }
    require_non_negative("wing.cd0_wing", cfg.wing.cd0_wing)?;
    require_non_negative("wing.le_root_x_m", cfg.wing.le_root_x_m)?;
    // Task trim-authority: Cm_ac do perfil + ΔCm de flap de pouso,
    // consumidos pelo balanço de momentos de flare/rotação
    // (`agents::trim_authority`).
    require_finite("wing.cm_ac", cfg.wing.cm_ac)?;
    if cfg.wing.cm_ac <= -0.15 || cfg.wing.cm_ac >= 0.05 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: wing.cm_ac deve estar em (-0.15, 0.05) \
             (valor: {})",
            cfg.wing.cm_ac
        )));
    }
    require_finite("wing.cm_flap_delta", cfg.wing.cm_flap_delta)?;
    if cfg.wing.cm_flap_delta <= -0.6 || cfg.wing.cm_flap_delta >= 0.0 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: wing.cm_flap_delta deve estar em (-0.6, 0.0) \
             (valor: {})",
            cfg.wing.cm_flap_delta
        )));
    }

    // [fuselage]
    require_positive("fuselage.length_m", cfg.fuselage.length_m)?;
    require_positive("fuselage.cabin_width_m", cfg.fuselage.cabin_width_m)?;
    require_positive("fuselage.cabin_height_m", cfg.fuselage.cabin_height_m)?;
    require_non_negative("fuselage.cd0", cfg.fuselage.cd0)?;

    // [empennage]
    require_non_negative("empennage.cd0", cfg.empennage.cd0)?;
    require_positive("empennage.tail_arm_m", cfg.empennage.tail_arm_m)?;
    require_finite("empennage.v_h", cfg.empennage.v_h)?;
    if cfg.empennage.v_h <= 0.2 || cfg.empennage.v_h >= 1.5 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: empennage.v_h deve estar em (0.2, 1.5) \
             (valor: {})",
            cfg.empennage.v_h
        )));
    }
    require_finite("empennage.v_v", cfg.empennage.v_v)?;
    if cfg.empennage.v_v <= 0.01 || cfg.empennage.v_v >= 0.15 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: empennage.v_v deve estar em (0.01, 0.15) \
             (valor: {})",
            cfg.empennage.v_v
        )));
    }
    require_positive("empennage.ar_h", cfg.empennage.ar_h)?;
    require_positive("empennage.ar_v", cfg.empennage.ar_v)?;
    require_positive("empennage.taper_h", cfg.empennage.taper_h)?;
    require_positive("empennage.taper_v", cfg.empennage.taper_v)?;
    require_finite("empennage.eta_h", cfg.empennage.eta_h)?;
    if cfg.empennage.eta_h <= 0.5 || cfg.empennage.eta_h > 1.0 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: empennage.eta_h deve estar em (0.5, 1.0] \
             (valor: {})",
            cfg.empennage.eta_h
        )));
    }

    // [propeller]
    if let Some(d) = cfg.propeller.diameter_m {
        require_positive("propeller.diameter_m", d)?;
    }
    if cfg.propeller.blades < 1 {
        return Err(ConfigError::Validation(
            "configuração de aeronave inválida: propeller.blades deve ser >= 1".to_string(),
        ));
    }
    require_finite("propeller.psru_ratio", cfg.propeller.psru_ratio)?;
    if cfg.propeller.psru_ratio < 1.0 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: propeller.psru_ratio deve ser >= 1 (valor: {})",
            cfg.propeller.psru_ratio
        )));
    }
    require_positive("propeller.psru_efficiency", cfg.propeller.psru_efficiency)?;
    if cfg.propeller.psru_efficiency > 1.0 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: propeller.psru_efficiency deve estar em (0, 1] (valor: {})",
            cfg.propeller.psru_efficiency
        )));
    }
    // Task 4.5: altura do eixo, limites de Mach de ponta e folga mínima de solo.
    require_positive("propeller.shaft_height_m", cfg.propeller.shaft_height_m)?;
    require_finite("propeller.tip_mach_max_static", cfg.propeller.tip_mach_max_static)?;
    if cfg.propeller.tip_mach_max_static <= 0.5 || cfg.propeller.tip_mach_max_static >= 1.0 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: propeller.tip_mach_max_static deve estar em \
             (0.5, 1.0) (valor: {})",
            cfg.propeller.tip_mach_max_static
        )));
    }
    require_finite("propeller.tip_mach_max_cruise", cfg.propeller.tip_mach_max_cruise)?;
    if cfg.propeller.tip_mach_max_cruise <= 0.5 || cfg.propeller.tip_mach_max_cruise >= 1.0 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: propeller.tip_mach_max_cruise deve estar em \
             (0.5, 1.0) (valor: {})",
            cfg.propeller.tip_mach_max_cruise
        )));
    }
    require_finite("propeller.ground_clearance_min_m", cfg.propeller.ground_clearance_min_m)?;
    if cfg.propeller.ground_clearance_min_m < 0.18 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: propeller.ground_clearance_min_m deve ser >= \
             0.18 m (CS 23.925) (valor: {})",
            cfg.propeller.ground_clearance_min_m
        )));
    }

    // [fuel_system]
    require_positive("fuel_system.capacity_l", cfg.fuel_system.capacity_l)?;

    // [gear]
    require_non_negative("gear.cd0_fixed_increment", cfg.gear.cd0_fixed_increment)?;
    require_positive("gear.h_cg_ground_m", cfg.gear.h_cg_ground_m)?;
    require_non_negative("gear.x_nose_m", cfg.gear.x_nose_m)?;
    require_non_negative("gear.x_main_m", cfg.gear.x_main_m)?;
    if cfg.gear.x_main_m <= cfg.gear.x_nose_m {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: gear.x_main_m ({}) deve ser maior que gear.x_nose_m ({})",
            cfg.gear.x_main_m, cfg.gear.x_nose_m
        )));
    }
    require_positive("gear.mass_main_leg_kg", cfg.gear.mass_main_leg_kg)?;
    require_positive("gear.mass_nose_kg", cfg.gear.mass_nose_kg)?;
    require_positive("gear.retraction_time_s", cfg.gear.retraction_time_s)?;
    require_non_negative("gear.actuators_doors_mass_kg", cfg.gear.actuators_doors_mass_kg)?;

    // [arms] — braços de momento: não-negativos e finitos.
    require_non_negative("arms.engine_cg_m", cfg.arms.engine_cg_m)?;
    require_non_negative("arms.avionics_m", cfg.arms.avionics_m)?;
    require_non_negative("arms.pax_front_m", cfg.arms.pax_front_m)?;
    require_non_negative("arms.fuel_cg_m", cfg.arms.fuel_cg_m)?;
    require_non_negative("arms.wing_struct_m", cfg.arms.wing_struct_m)?;
    require_non_negative("arms.pax_rear_m", cfg.arms.pax_rear_m)?;
    require_non_negative("arms.fuselage_struct_m", cfg.arms.fuselage_struct_m)?;
    require_non_negative("arms.baggage_m", cfg.arms.baggage_m)?;
    require_non_negative("arms.empennage_cg_m", cfg.arms.empennage_cg_m)?;

    // [structure]
    require_positive("structure.frame_spacing_mm", cfg.structure.frame_spacing_mm)?;
    if !matches!(cfg.structure.design_category.as_str(), "normal" | "utility" | "acrobatic") {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: structure.design_category '{}' desconhecida \
             (esperado: normal | utility | acrobatic)",
            cfg.structure.design_category
        )));
    }
    if crate::agents::structural::material_by_name(&cfg.structure.spar_material).is_none() {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: structure.spar_material '{}' desconhecido \
             (materiais cadastrados: AA7075-T6, AA6061-T6)",
            cfg.structure.spar_material
        )));
    }

    // [drag]
    require_non_negative("drag.cd0_misc", cfg.drag.cd0_misc)?;
    // Task 5.2: fração de arrasto de refrigeração — (0, 0.10], típico
    // 3–5% (Raymer/Hoerner) para instalação a pistão bem carenada. Um teto
    // de 10% evita um erro de digitação (ex.: 0.4 em vez de 0.04) passar
    // silenciosamente e degradar o CD0 em 40%.
    require_finite("drag.cooling_drag_fraction", cfg.drag.cooling_drag_fraction)?;
    if cfg.drag.cooling_drag_fraction <= 0.0 || cfg.drag.cooling_drag_fraction > 0.10 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: drag.cooling_drag_fraction deve estar em \
             (0.0, 0.10] (valor: {})",
            cfg.drag.cooling_drag_fraction
        )));
    }

    // [stability] (Task 4.4 + task trim-authority) — sm_min segue definindo
    // o limite TRASEIRO (0 < sm_min); sm_max foi REMOVIDO (ver
    // `check_sm_max_migration`) — o limite DIANTEIRO agora vem da
    // autoridade de profundor (`cl_h_max_down`/`trim_margin`/
    // `cl_ground_rotation`/`to_flap_cm_fraction`), validada abaixo.
    require_finite("stability.sm_min", cfg.stability.sm_min)?;
    if cfg.stability.sm_min <= 0.0 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: stability.sm_min deve ser positivo (valor: {})",
            cfg.stability.sm_min
        )));
    }
    if cfg.stability.sm_min >= 0.5 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: stability.sm_min deve ser menor que 0.5 \
             (valor: {})",
            cfg.stability.sm_min
        )));
    }
    require_finite("stability.cl_h_max_down", cfg.stability.cl_h_max_down)?;
    if cfg.stability.cl_h_max_down <= 0.5 || cfg.stability.cl_h_max_down >= 1.2 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: stability.cl_h_max_down deve estar em \
             (0.5, 1.2) (valor: {})",
            cfg.stability.cl_h_max_down
        )));
    }
    require_finite("stability.trim_margin", cfg.stability.trim_margin)?;
    if cfg.stability.trim_margin < 0.0 || cfg.stability.trim_margin > 0.3 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: stability.trim_margin deve estar em [0, 0.3] \
             (valor: {})",
            cfg.stability.trim_margin
        )));
    }
    require_finite("stability.cl_ground_rotation", cfg.stability.cl_ground_rotation)?;
    if cfg.stability.cl_ground_rotation <= 0.0 || cfg.stability.cl_ground_rotation >= 1.0 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: stability.cl_ground_rotation deve estar em \
             (0.0, 1.0) (valor: {})",
            cfg.stability.cl_ground_rotation
        )));
    }
    require_finite("stability.to_flap_cm_fraction", cfg.stability.to_flap_cm_fraction)?;
    if cfg.stability.to_flap_cm_fraction < 0.0 || cfg.stability.to_flap_cm_fraction > 1.0 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: stability.to_flap_cm_fraction deve estar em \
             [0, 1] (valor: {})",
            cfg.stability.to_flap_cm_fraction
        )));
    }
    // fuselage_kf (Multhopp simplificado, Raymer fig. 16.14) — faixa típica
    // 0.01–0.03 conforme a posição da asa na fuselagem; teto de folga
    // (0.005, 0.05) evita erros de digitação grosseiros sem travar
    // configurações plausíveis fora da faixa "típica".
    require_finite("stability.fuselage_kf", cfg.stability.fuselage_kf)?;
    if cfg.stability.fuselage_kf <= 0.005 || cfg.stability.fuselage_kf >= 0.05 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: stability.fuselage_kf deve estar em \
             (0.005, 0.05) (valor: {})",
            cfg.stability.fuselage_kf
        )));
    }

    // [[masses.items]]
    if cfg.masses.items.is_empty() {
        return Err(ConfigError::Validation(
            "configuração de aeronave inválida: masses.items não pode ser vazio".to_string(),
        ));
    }
    let arms = crate::agents::weight_balance::ArmConfig::from_config(cfg);
    for item in &cfg.masses.items {
        require_positive(&format!("masses.items[{}].mass_kg", item.name), item.mass_kg)?;
        require_finite(&format!("masses.items[{}].arm_offset_m", item.name), item.arm_offset_m)?;
        if arms.by_name(&item.arm_ref).is_none() {
            return Err(ConfigError::Validation(format!(
                "configuração de aeronave inválida: item de massa '{}' referencia arm_ref \
                 desconhecido '{}'",
                item.name, item.arm_ref
            )));
        }
    }

    // Itens de massa que o código de produção resolve por nome (main.rs,
    // ao montar StructuralAgent::run/LandingGearAgent::run) — sem estes, o
    // pipeline passa da validação e só quebra (panic) no meio da execução,
    // depois de já ter impresso parte do relatório. Exigir aqui é mais
    // barato que descobrir isso em produção.
    for nome_obrigatorio in ["asa", "trem_principal", "trem_nariz"] {
        if cfg.masses.item_mass(nome_obrigatorio).is_none() {
            return Err(ConfigError::Validation(format!(
                "configuração de aeronave inválida: masses.items não contém o item \
                 obrigatório '{nome_obrigatorio}' (usado por nome no código de produção — \
                 StructuralAgent para 'asa', LandingGearAgent para 'trem_principal'/'trem_nariz')"
            )));
        }
    }

    // [control_surfaces] (Task 4.2) — frações históricas (Raymer Tab. 6.5)
    // que dimensionam aileron, flap, profundor e leme.
    let cs = &cfg.control_surfaces;
    require_frac("control_surfaces.aileron_span_start_frac", cs.aileron_span_start_frac)?;
    require_frac("control_surfaces.aileron_span_end_frac", cs.aileron_span_end_frac)?;
    require_frac("control_surfaces.aileron_chord_frac", cs.aileron_chord_frac)?;
    require_frac("control_surfaces.flap_span_start_frac", cs.flap_span_start_frac)?;
    require_frac("control_surfaces.flap_span_end_frac", cs.flap_span_end_frac)?;
    require_frac("control_surfaces.flap_chord_frac", cs.flap_chord_frac)?;
    require_frac("control_surfaces.elevator_span_frac", cs.elevator_span_frac)?;
    require_frac("control_surfaces.elevator_chord_frac", cs.elevator_chord_frac)?;
    require_frac("control_surfaces.rudder_span_frac", cs.rudder_span_frac)?;
    require_frac("control_surfaces.rudder_chord_frac", cs.rudder_chord_frac)?;

    if cs.aileron_span_start_frac >= cs.aileron_span_end_frac {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: control_surfaces.aileron_span_start_frac ({}) \
             deve ser menor que control_surfaces.aileron_span_end_frac ({})",
            cs.aileron_span_start_frac, cs.aileron_span_end_frac
        )));
    }
    if cs.flap_span_start_frac >= cs.flap_span_end_frac {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: control_surfaces.flap_span_start_frac ({}) \
             deve ser menor que control_surfaces.flap_span_end_frac ({})",
            cs.flap_span_start_frac, cs.flap_span_end_frac
        )));
    }
    if cs.flap_span_end_frac > cs.aileron_span_start_frac {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: control_surfaces.flap_span_end_frac ({}) não \
             pode ultrapassar control_surfaces.aileron_span_start_frac ({}) — flap e aileron \
             não podem se sobrepor na semi-envergadura da asa",
            cs.flap_span_end_frac, cs.aileron_span_start_frac
        )));
    }

    // [performance] (Task 4.7) — atrito de frenagem por superfície, fator
    // empírico de tração estática, tempos de rotação/flare, ângulo de
    // aproximação.
    require_finite("performance.mu_brake_paved", cfg.performance.mu_brake_paved)?;
    if cfg.performance.mu_brake_paved <= 0.05 || cfg.performance.mu_brake_paved >= 0.8 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: performance.mu_brake_paved deve estar em \
             (0.05, 0.8) (valor: {})",
            cfg.performance.mu_brake_paved
        )));
    }
    require_finite("performance.mu_brake_grass", cfg.performance.mu_brake_grass)?;
    if cfg.performance.mu_brake_grass <= 0.05 || cfg.performance.mu_brake_grass >= 0.8 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: performance.mu_brake_grass deve estar em \
             (0.05, 0.8) (valor: {})",
            cfg.performance.mu_brake_grass
        )));
    }
    require_positive("performance.static_thrust_factor", cfg.performance.static_thrust_factor)?;
    if cfg.performance.static_thrust_factor > 1.0 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: performance.static_thrust_factor deve estar em \
             (0, 1.0] — é uma correção que reduz a tração estática IDEAL de Rankine-Froude para \
             a real (perdas de ponta de pá/rotação de esteira), nunca aumenta (valor: {})",
            cfg.performance.static_thrust_factor
        )));
    }
    require_positive("performance.rotation_time_s", cfg.performance.rotation_time_s)?;
    require_positive("performance.flare_time_s", cfg.performance.flare_time_s)?;
    require_finite("performance.approach_angle_deg", cfg.performance.approach_angle_deg)?;
    if cfg.performance.approach_angle_deg <= 1.0 || cfg.performance.approach_angle_deg >= 10.0 {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: performance.approach_angle_deg deve estar em \
             (1.0, 10.0) graus (valor: {})",
            cfg.performance.approach_angle_deg
        )));
    }

    // Consistência trem principal: a massa TOTAL do item 'trem_principal'
    // (ambas as pernas) deve ser exatamente 2× a massa de UMA perna
    // (`[gear].mass_main_leg_kg`) — é essa relação que justifica usar
    // `mass_main_leg_kg` (não um valor independente) no dimensionamento do
    // atuador de retração em `landing_gear.rs`. Ver task-2.1-report.md
    // ("bug fix: actuator_power_w agora usa a massa de perna real").
    if let Some(trem_principal_kg) = cfg.masses.item_mass("trem_principal") {
        let esperado = 2.0 * cfg.gear.mass_main_leg_kg;
        if (trem_principal_kg - esperado).abs() > 1e-6 {
            return Err(ConfigError::Validation(format!(
                "configuração de aeronave inválida: masses.items 'trem_principal' \
                 ({trem_principal_kg} kg) deveria ser exatamente 2× gear.mass_main_leg_kg \
                 ({} kg × 2 = {esperado} kg) — as duas pernas do trem principal",
                cfg.gear.mass_main_leg_kg
            )));
        }
    }

    // [electrical] (Task 5.2) — orçamento elétrico: barramento, alternador
    // e cargas individuais.
    require_finite("electrical.bus_voltage_v", cfg.electrical.bus_voltage_v)?;
    const BARRAMENTOS_PADRAO_V: [f64; 4] = [12.0, 14.0, 24.0, 28.0];
    let bus_reconhecido = BARRAMENTOS_PADRAO_V.iter()
        .any(|v| (cfg.electrical.bus_voltage_v - v).abs() <= 0.1);
    if !bus_reconhecido {
        return Err(ConfigError::Validation(format!(
            "configuração de aeronave inválida: electrical.bus_voltage_v ({}) não corresponde \
             a nenhum barramento aeronáutico padrão (12, 14, 24 ou 28 V, ±0.1 V)",
            cfg.electrical.bus_voltage_v
        )));
    }
    require_positive("electrical.alternator_w", cfg.electrical.alternator_w)?;

    if cfg.electrical.loads.is_empty() {
        return Err(ConfigError::Validation(
            "configuração de aeronave inválida: electrical.loads não pode ser vazio"
                .to_string(),
        ));
    }
    let mut nomes_vistos = std::collections::HashSet::new();
    for load in &cfg.electrical.loads {
        require_non_negative(
            &format!("electrical.loads['{}'].continuous_w", load.name), load.continuous_w,
        )?;
        require_non_negative(
            &format!("electrical.loads['{}'].peak_w", load.name), load.peak_w,
        )?;
        if !nomes_vistos.insert(load.name.as_str()) {
            return Err(ConfigError::Validation(format!(
                "configuração de aeronave inválida: electrical.loads contém nome duplicado \
                 '{}' — os nomes das cargas devem ser únicos",
                load.name
            )));
        }
    }

    // Consistência trem retrátil × orçamento elétrico (Task 5.2): a carga
    // elétrica 'trem_retratil' (pico na retração) não pode subestimar a
    // potência MECÂNICA do atuador calculada por
    // `agents::landing_gear::actuator_power_w` — mesma filosofia da guarda
    // 'trem_principal' × `gear.mass_main_leg_kg` acima (dois lugares que
    // descrevem o mesmo sistema físico não podem divergir silenciosamente).
    // Nota: o valor mecânico aqui é só um PISO — o pico elétrico real de um
    // atuador linear inclui perdas de eficiência do motor/mecanismo e
    // corrente de partida, tipicamente bem acima do trabalho mecânico
    // idealizado (ver comentário de `[[electrical.loads]]` 'trem_retratil'
    // em `config/aircraft/baseline_4seat.toml`).
    if let Some(carga_trem) = cfg.electrical.loads.iter().find(|l| l.name == "trem_retratil") {
        let atuador_w = crate::agents::landing_gear::actuator_power_w(
            cfg.gear.mass_main_leg_kg,
            crate::agents::landing_gear::GEAR_RETRACTION_DELTA_H_M,
            cfg.gear.retraction_time_s,
        );
        if carga_trem.peak_w < atuador_w {
            return Err(ConfigError::Validation(format!(
                "configuração de aeronave inválida: electrical.loads 'trem_retratil' peak_w \
                 ({} W) deve ser >= à potência mecânica do atuador de retração calculada \
                 ({atuador_w:.1} W, agents::landing_gear::actuator_power_w) — o orçamento \
                 elétrico não pode subestimar o pico real de retração",
                carga_trem.peak_w
            )));
        }
    }

    Ok(())
}

// ─── MISSÃO (REQUISITOS) ──────────────────────────────────────────────────────

/// Faz o parse de requisitos de missão a partir de uma string TOML,
/// validando as invariantes físicas e de consistência (altitudes, frações,
/// desvio ISA).
pub fn parse_mission(toml_str: &str) -> Result<Requirements, ConfigError> {
    let req: Requirements = toml::from_str(toml_str)?;
    validate_mission(&req)?;
    Ok(req)
}

/// Lê e faz o parse de requisitos de missão a partir de um arquivo TOML no
/// disco.
pub fn load_mission(path: &Path) -> Result<Requirements, ConfigError> {
    let content = std::fs::read_to_string(path).map_err(|e| {
        ConfigError::Io(std::io::Error::new(
            e.kind(),
            format!("não foi possível ler o arquivo de missão '{}': {e}", path.display()),
        ))
    })?;
    parse_mission(&content)
}

/// Garante que `v` é finito, com mensagem em português nomeando o campo —
/// mesmo padrão de `require_finite`, mas com prefixo "missão" em vez de
/// "aeronave" (mensagens de erro devem nomear o arquivo de configuração
/// certo, não um genérico compartilhado entre os três loaders).
fn require_finite_missao(field: &str, v: f64) -> Result<(), ConfigError> {
    if !v.is_finite() {
        return Err(ConfigError::Validation(format!(
            "configuração de missão inválida: {field} deve ser finito (valores NaN/infinito não permitidos)"
        )));
    }
    Ok(())
}

fn require_positive_missao(field: &str, v: f64) -> Result<(), ConfigError> {
    require_finite_missao(field, v)?;
    if v <= 0.0 {
        return Err(ConfigError::Validation(format!(
            "configuração de missão inválida: {field} deve ser positivo (valor: {v})"
        )));
    }
    Ok(())
}

fn require_non_negative_missao(field: &str, v: f64) -> Result<(), ConfigError> {
    require_finite_missao(field, v)?;
    if v < 0.0 {
        return Err(ConfigError::Validation(format!(
            "configuração de missão inválida: {field} não pode ser negativo (valor: {v})"
        )));
    }
    Ok(())
}

/// Valida as invariantes físicas e de consistência de uma `Requirements`
/// recém-carregada.
fn validate_mission(req: &Requirements) -> Result<(), ConfigError> {
    if req.passengers < 1 {
        return Err(ConfigError::Validation(format!(
            "configuração de missão inválida: passengers deve ser >= 1 (valor: {})",
            req.passengers
        )));
    }

    require_positive_missao("pax_mass_kg", req.pax_mass_kg)?;
    require_non_negative_missao("baggage_kg", req.baggage_kg)?;
    require_positive_missao("cruise_speed_min_kmh", req.cruise_speed_min_kmh)?;
    require_positive_missao("endurance_min_h", req.endurance_min_h)?;

    require_finite_missao("fuel_reserve_fraction", req.fuel_reserve_fraction)?;
    if req.fuel_reserve_fraction < 0.0 || req.fuel_reserve_fraction > 0.5 {
        return Err(ConfigError::Validation(format!(
            "configuração de missão inválida: fuel_reserve_fraction deve estar em [0, 0.5] \
             (valor: {})",
            req.fuel_reserve_fraction
        )));
    }

    require_non_negative_missao("cruise_altitude_m", req.cruise_altitude_m)?;
    if req.cruise_altitude_m > 10_000.0 {
        return Err(ConfigError::Validation(format!(
            "configuração de missão inválida: cruise_altitude_m deve estar em [0, 10000] \
             (valor: {})",
            req.cruise_altitude_m
        )));
    }

    require_non_negative_missao("airfield_altitude_m", req.airfield_altitude_m)?;
    if req.airfield_altitude_m > req.cruise_altitude_m {
        return Err(ConfigError::Validation(format!(
            "configuração de missão inválida: airfield_altitude_m ({}) não pode ser maior \
             que cruise_altitude_m ({})",
            req.airfield_altitude_m, req.cruise_altitude_m
        )));
    }

    require_finite_missao("isa_delta_c", req.isa_delta_c)?;
    if req.isa_delta_c < -40.0 || req.isa_delta_c > 40.0 {
        return Err(ConfigError::Validation(format!(
            "configuração de missão inválida: isa_delta_c deve estar em [-40, 40] (valor: {})",
            req.isa_delta_c
        )));
    }

    require_positive_missao("analysis.taxi_fuel_l", req.analysis.taxi_fuel_l)?;
    require_positive_missao("analysis.descent_rate_ms", req.analysis.descent_rate_ms)?;

    require_finite_missao("analysis.descent_power_fraction", req.analysis.descent_power_fraction)?;
    if req.analysis.descent_power_fraction < 0.05 || req.analysis.descent_power_fraction > 0.5 {
        return Err(ConfigError::Validation(format!(
            "configuração de missão inválida: analysis.descent_power_fraction deve estar em \
             [0.05, 0.5] (valor: {})",
            req.analysis.descent_power_fraction
        )));
    }

    if req.analysis.climb_speed_policy != "vy" {
        return Err(ConfigError::Validation(format!(
            "configuração de missão inválida: analysis.climb_speed_policy só suporta \"vy\" \
             (melhor razão de subida) hoje (valor: \"{}\")",
            req.analysis.climb_speed_policy
        )));
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn erro_claro_para_curva_invalida() {
        let toml_ruim = r#"
            name = "X"
            mass_kg = 1.0
            rpm_idle = 700.0
            rpm_rated = 3000.0
            rpm_redline = 3500.0
            rpm_max_continuous = 2800.0
            torque_curve = [[700.0, 200.0]]
            induction = "naturally_aspirated"
            [bsfc]
            bsfc_min_gkwh = 200.0
            rpm_optimal = 2200.0
            load_optimal = 0.7
            rpm_penalty_gkwh = 18.0
            load_penalty_gkwh = 22.0
            bsfc_max_gkwh = 380.0
            [fuel]
            name = "d"
            density_kg_per_l = 0.84
            lhv_mj_per_kg = 42.5
        "#;
        let err = parse_engine(toml_ruim).unwrap_err();
        assert!(err.to_string().contains("pelo menos 2 pontos"));
    }

    #[test]
    fn rejeita_valores_nao_finitos_na_curva() {
        let toml_com_nan = r#"
            name = "X"
            mass_kg = 1.0
            rpm_idle = 700.0
            rpm_rated = 3000.0
            rpm_redline = 3500.0
            rpm_max_continuous = 2800.0
            torque_curve = [[nan, 200.0], [1600.0, 500.0]]
            induction = "naturally_aspirated"
            [bsfc]
            bsfc_min_gkwh = 200.0
            rpm_optimal = 2200.0
            load_optimal = 0.7
            rpm_penalty_gkwh = 18.0
            load_penalty_gkwh = 22.0
            bsfc_max_gkwh = 380.0
            [fuel]
            name = "d"
            density_kg_per_l = 0.84
            lhv_mj_per_kg = 42.5
        "#;
        let err = parse_engine(toml_com_nan).unwrap_err();
        assert!(err.to_string().contains("finitos"));
        assert!(err.to_string().contains("NaN") || err.to_string().contains("nan") || err.to_string().contains("infinito"));
    }

    #[test]
    fn rejeita_valores_infinitos() {
        let toml_com_inf = r#"
            name = "X"
            mass_kg = inf
            rpm_idle = 700.0
            rpm_rated = 3000.0
            rpm_redline = 3500.0
            rpm_max_continuous = 2800.0
            torque_curve = [[700.0, 200.0], [1600.0, 500.0]]
            induction = "naturally_aspirated"
            [bsfc]
            bsfc_min_gkwh = 200.0
            rpm_optimal = 2200.0
            load_optimal = 0.7
            rpm_penalty_gkwh = 18.0
            load_penalty_gkwh = 22.0
            bsfc_max_gkwh = 380.0
            [fuel]
            name = "d"
            density_kg_per_l = 0.84
            lhv_mj_per_kg = 42.5
        "#;
        let err = parse_engine(toml_com_inf).unwrap_err();
        assert!(err.to_string().contains("finitos") || err.to_string().contains("infinito"));
    }

    // ─── AERONAVE (CÉLULA) ────────────────────────────────────────────────

    /// TOML de aeronave mínimo porém válido, usado como base para os testes
    /// de validação abaixo (cada teste sobrescreve um trecho para violar
    /// exatamente uma invariante).
    fn aircraft_toml_valido() -> String {
        r#"
            [sizing]
            mtow_initial_guess_kg = 1000.0
            mtow_max_kg = 1800.0
            [wing]
            span_m = 10.0
            area_m2 = 12.0
            taper_ratio = 0.5
            airfoil = "Teste"
            thickness_ratio = 0.15
            cl_max_clean = 1.4
            cl_max_flaps = 1.6
            cd0_wing = 0.005
            le_root_x_m = 2.5
            cm_ac = -0.008
            cm_flap_delta = -0.30
            [fuselage]
            length_m = 7.5
            cabin_width_m = 1.1
            cabin_height_m = 1.1
            cd0 = 0.01
            [empennage]
            cd0 = 0.004
            tail_arm_m = 4.5
            v_h = 0.70
            v_v = 0.04
            ar_h = 4.0
            ar_v = 1.5
            taper_h = 0.5
            taper_v = 0.5
            eta_h = 0.90
            [propeller]
            diameter_m = 1.8
            blades = 2
            psru_ratio = 1.5
            psru_efficiency = 0.95
            shaft_height_m = 1.20
            tip_mach_max_static = 0.85
            tip_mach_max_cruise = 0.80
            ground_clearance_min_m = 0.23
            [fuel_system]
            capacity_l = 200.0
            [gear]
            retractable = true
            cd0_fixed_increment = 0.008
            h_cg_ground_m = 1.0
            x_nose_m = 1.3
            x_main_m = 3.5
            mass_main_leg_kg = 25.0
            mass_nose_kg = 20.0
            retraction_time_s = 7.0
            actuators_doors_mass_kg = 18.0
            [arms]
            engine_cg_m = 0.6
            avionics_m = 1.0
            pax_front_m = 3.0
            fuel_cg_m = 3.3
            wing_struct_m = 3.4
            pax_rear_m = 4.2
            fuselage_struct_m = 3.9
            baggage_m = 5.2
            empennage_cg_m = 7.0
            [structure]
            spar_material = "AA7075-T6"
            frame_spacing_mm = 300.0
            design_category = "normal"
            [drag]
            cd0_misc = 0.003
            cooling_drag_fraction = 0.04
            [stability]
            sm_min = 0.05
            cl_h_max_down = 0.85
            trim_margin = 0.10
            cl_ground_rotation = 0.5
            to_flap_cm_fraction = 0.5
            fuselage_kf = 0.02
            [performance]
            mu_brake_paved = 0.40
            mu_brake_grass = 0.30
            static_thrust_factor = 0.75
            rotation_time_s = 1.0
            flare_time_s = 1.5
            approach_angle_deg = 3.0
            [control_surfaces]
            aileron_span_start_frac = 0.55
            aileron_span_end_frac = 0.90
            aileron_chord_frac = 0.25
            flap_span_start_frac = 0.10
            flap_span_end_frac = 0.50
            flap_chord_frac = 0.30
            elevator_span_frac = 0.90
            elevator_chord_frac = 0.35
            rudder_span_frac = 0.90
            rudder_chord_frac = 0.35
            [[masses.items]]
            name = "asa"
            mass_kg = 100.0
            arm_ref = "wing_struct"
            [[masses.items]]
            name = "trem_principal"
            mass_kg = 50.0
            arm_ref = "gear_main"
            [[masses.items]]
            name = "trem_nariz"
            mass_kg = 20.0
            arm_ref = "gear_nose"
            [electrical]
            bus_voltage_v = 28.0
            alternator_w = 900.0
            [[electrical.loads]]
            name = "avionicos"
            continuous_w = 180.0
            peak_w = 220.0
            [[electrical.loads]]
            name = "luzes_nav_strobe"
            continuous_w = 45.0
            peak_w = 90.0
            [[electrical.loads]]
            name = "bomba_combustivel"
            continuous_w = 60.0
            peak_w = 120.0
            [[electrical.loads]]
            name = "trem_retratil"
            continuous_w = 0.0
            peak_w = 520.0
            [[electrical.loads]]
            name = "flaps"
            continuous_w = 0.0
            peak_w = 150.0
            [[electrical.loads]]
            name = "pitot_aquecido"
            continuous_w = 90.0
            peak_w = 90.0
            [[electrical.loads]]
            name = "radio_transponder"
            continuous_w = 55.0
            peak_w = 70.0
        "#
        .to_string()
    }

    #[test]
    fn aircraft_toml_valido_carrega_sem_erro() {
        parse_aircraft(&aircraft_toml_valido()).expect("TOML de teste deveria ser válido");
    }

    #[test]
    fn rejeita_mtow_initial_guess_maior_ou_igual_ao_max() {
        let toml = aircraft_toml_valido()
            .replace("mtow_initial_guess_kg = 1000.0", "mtow_initial_guess_kg = 2000.0");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("mtow_initial_guess_kg"), "{err}");
        assert!(err.to_string().contains("mtow_max_kg"), "{err}");
    }

    #[test]
    fn rejeita_taper_ratio_fora_de_0_1() {
        let toml = aircraft_toml_valido().replace("taper_ratio = 0.5", "taper_ratio = 1.5");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("taper_ratio"), "{err}");
    }

    #[test]
    fn rejeita_cl_max_flaps_menor_que_cl_max_clean() {
        let toml = aircraft_toml_valido().replace("cl_max_flaps = 1.6", "cl_max_flaps = 1.0");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("cl_max_flaps"), "{err}");
    }

    #[test]
    fn rejeita_psru_ratio_abaixo_de_1() {
        let toml = aircraft_toml_valido().replace("psru_ratio = 1.5", "psru_ratio = 0.8");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("psru_ratio"), "{err}");
    }

    // ─── [propeller] (Task 4.5) ─────────────────────────────────────────────

    #[test]
    fn aceita_diameter_m_omitido() {
        let base = aircraft_toml_valido();
        let toml = base.replace("diameter_m = 1.8\n", "");
        let cfg = parse_aircraft(&toml).expect("diameter_m omitido deveria ser válido (derivado)");
        assert!(cfg.propeller.diameter_m.is_none());
    }

    #[test]
    fn rejeita_diameter_m_nao_positivo_quando_presente() {
        let toml = aircraft_toml_valido().replace("diameter_m = 1.8", "diameter_m = 0.0");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("propeller.diameter_m"), "{err}");
    }

    #[test]
    fn rejeita_shaft_height_m_nao_positivo() {
        let toml = aircraft_toml_valido().replace("shaft_height_m = 1.20", "shaft_height_m = 0.0");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("propeller.shaft_height_m"), "{err}");
    }

    #[test]
    fn rejeita_tip_mach_max_static_fora_da_faixa() {
        let toml = aircraft_toml_valido()
            .replace("tip_mach_max_static = 0.85", "tip_mach_max_static = 1.2");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("propeller.tip_mach_max_static"), "{err}");
    }

    #[test]
    fn rejeita_tip_mach_max_cruise_fora_da_faixa() {
        let toml = aircraft_toml_valido()
            .replace("tip_mach_max_cruise = 0.80", "tip_mach_max_cruise = 0.3");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("propeller.tip_mach_max_cruise"), "{err}");
    }

    #[test]
    fn rejeita_ground_clearance_min_m_abaixo_de_0_18() {
        let toml = aircraft_toml_valido()
            .replace("ground_clearance_min_m = 0.23", "ground_clearance_min_m = 0.10");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("propeller.ground_clearance_min_m"), "{err}");
    }

    #[test]
    fn rejeita_capacidade_de_combustivel_nao_positiva() {
        let toml = aircraft_toml_valido().replace("capacity_l = 200.0", "capacity_l = 0.0");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("capacity_l"), "{err}");
    }

    #[test]
    fn rejeita_material_estrutural_desconhecido() {
        let toml = aircraft_toml_valido()
            .replace(r#"spar_material = "AA7075-T6""#, r#"spar_material = "Unobtainium""#);
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("spar_material"), "{err}");
    }

    #[test]
    fn rejeita_categoria_de_projeto_desconhecida() {
        let toml = aircraft_toml_valido()
            .replace(r#"design_category = "normal""#, r#"design_category = "estranha""#);
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("design_category"), "{err}");
    }

    #[test]
    fn rejeita_arm_ref_desconhecido() {
        let toml = aircraft_toml_valido().replace(
            r#"arm_ref = "gear_main""#,
            r#"arm_ref = "lugar_nenhum""#,
        );
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("arm_ref"), "{err}");
        assert!(err.to_string().contains("lugar_nenhum"), "{err}");
    }

    #[test]
    fn rejeita_v_h_fora_da_faixa() {
        let toml = aircraft_toml_valido().replace("v_h = 0.70", "v_h = 1.6");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("empennage.v_h"), "{err}");
    }

    #[test]
    fn rejeita_v_h_muito_baixo() {
        let toml = aircraft_toml_valido().replace("v_h = 0.70", "v_h = 0.1");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("empennage.v_h"), "{err}");
    }

    #[test]
    fn rejeita_v_v_fora_da_faixa() {
        let toml = aircraft_toml_valido().replace("v_v = 0.04", "v_v = 0.2");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("empennage.v_v"), "{err}");
    }

    #[test]
    fn rejeita_eta_h_fora_da_faixa() {
        let toml = aircraft_toml_valido().replace("eta_h = 0.90", "eta_h = 1.5");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("empennage.eta_h"), "{err}");
    }

    #[test]
    fn rejeita_eta_h_nao_positivo() {
        let toml = aircraft_toml_valido().replace("eta_h = 0.90", "eta_h = 0.3");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("empennage.eta_h"), "{err}");
    }

    #[test]
    fn rejeita_ar_h_nao_positivo() {
        let toml = aircraft_toml_valido().replace("ar_h = 4.0", "ar_h = 0.0");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("empennage.ar_h"), "{err}");
    }

    #[test]
    fn rejeita_taper_v_nao_positivo() {
        let toml = aircraft_toml_valido().replace("taper_v = 0.5", "taper_v = -0.1");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("empennage.taper_v"), "{err}");
    }

    #[test]
    fn rejeita_x_main_nao_maior_que_x_nose() {
        let toml = aircraft_toml_valido().replace("x_main_m = 3.5", "x_main_m = 1.0");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("x_main_m"), "{err}");
    }

    #[test]
    fn rejeita_valores_nao_finitos_na_aeronave() {
        let toml = aircraft_toml_valido().replace("span_m = 10.0", "span_m = nan");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("finito"), "{err}");
    }

    #[test]
    fn rejeita_braco_de_momento_negativo() {
        let toml = aircraft_toml_valido().replace("engine_cg_m = 0.6", "engine_cg_m = -0.6");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("engine_cg_m"), "{err}");
    }

    #[test]
    fn rejeita_lista_de_massas_vazia() {
        let base = aircraft_toml_valido();
        let head = base.split("[[masses.items]]").next().unwrap();
        // A seção [electrical] (Task 5.2) vem DEPOIS de [[masses.items]] no
        // fixture — precisa ser preservada aqui, senão o TOML fica inválido
        // por um campo obrigatório ausente em vez da violação isolada que
        // este teste quer exercitar.
        let electrical_section = base.split("[electrical]").nth(1)
            .map(|s| format!("[electrical]{s}"))
            .expect("fixture deveria conter [electrical]");
        let toml = format!("{head}\n[masses]\nitems = []\n{electrical_section}");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("masses.items"), "{err}");
    }

    #[test]
    fn rejeita_item_de_massa_asa_ausente() {
        let toml = aircraft_toml_valido().replace(r#"name = "asa""#, r#"name = "asa_renomeada""#);
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("'asa'"), "{err}");
    }

    #[test]
    fn rejeita_item_de_massa_trem_principal_ausente() {
        let toml = aircraft_toml_valido()
            .replace(r#"name = "trem_principal""#, r#"name = "trem_principal_renomeado""#);
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("'trem_principal'"), "{err}");
    }

    #[test]
    fn rejeita_item_de_massa_trem_nariz_ausente() {
        let toml = aircraft_toml_valido()
            .replace(r#"name = "trem_nariz""#, r#"name = "trem_nariz_renomeado""#);
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("'trem_nariz'"), "{err}");
    }

    #[test]
    fn rejeita_massa_trem_principal_inconsistente_com_mass_main_leg_kg() {
        // trem_principal = 50 kg no template, mas mass_main_leg_kg passa a
        // valer 30 kg (2×30=60 ≠ 50) — deve ser rejeitado.
        let toml = aircraft_toml_valido()
            .replace("mass_main_leg_kg = 25.0", "mass_main_leg_kg = 30.0");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("trem_principal"), "{err}");
        assert!(err.to_string().contains("mass_main_leg_kg"), "{err}");
    }

    // ─── SUPERFÍCIES DE CONTROLE (Task 4.2) ─────────────────────────────────

    #[test]
    fn rejeita_fracao_de_superficie_de_controle_fora_de_0_1() {
        let toml = aircraft_toml_valido()
            .replace("aileron_chord_frac = 0.25", "aileron_chord_frac = 1.5");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("aileron_chord_frac"), "{err}");
    }

    #[test]
    fn rejeita_fracao_de_superficie_de_controle_nao_positiva() {
        let toml = aircraft_toml_valido()
            .replace("rudder_chord_frac = 0.35", "rudder_chord_frac = 0.0");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("rudder_chord_frac"), "{err}");
    }

    #[test]
    fn rejeita_aileron_start_maior_ou_igual_ao_end() {
        let toml = aircraft_toml_valido()
            .replace("aileron_span_start_frac = 0.55", "aileron_span_start_frac = 0.95");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("aileron_span_start_frac"), "{err}");
        assert!(err.to_string().contains("aileron_span_end_frac"), "{err}");
    }

    #[test]
    fn rejeita_flap_start_maior_ou_igual_ao_end() {
        let toml = aircraft_toml_valido()
            .replace("flap_span_start_frac = 0.10", "flap_span_start_frac = 0.60");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("flap_span_start_frac"), "{err}");
        assert!(err.to_string().contains("flap_span_end_frac"), "{err}");
    }

    #[test]
    fn rejeita_sobreposicao_entre_flap_e_aileron() {
        // flap_span_end_frac (0.50) fica ACIMA de aileron_span_start_frac
        // (0.45 após a alteração) — as duas superfícies passam a se
        // sobrepor na semi-envergadura da asa.
        let toml = aircraft_toml_valido()
            .replace("aileron_span_start_frac = 0.55", "aileron_span_start_frac = 0.45");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("flap_span_end_frac"), "{err}");
        assert!(err.to_string().contains("aileron_span_start_frac"), "{err}");
        assert!(err.to_string().contains("sobrepor"), "{err}");
    }

    // ─── [stability] (Task 4.4) ─────────────────────────────────────────────

    #[test]
    fn rejeita_sm_min_nao_positivo() {
        let toml = aircraft_toml_valido().replace("sm_min = 0.05", "sm_min = 0.0");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("stability.sm_min"), "{err}");
    }

    #[test]
    fn rejeita_sm_min_maior_ou_igual_a_0_5() {
        let toml = aircraft_toml_valido().replace("sm_min = 0.05", "sm_min = 0.55");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("stability.sm_min"), "{err}");
    }

    #[test]
    fn rejeita_sm_min_nao_finito() {
        let toml = aircraft_toml_valido().replace("sm_min = 0.05", "sm_min = nan");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("finito"), "{err}");
    }

    #[test]
    fn rejeita_fuselage_kf_fora_da_faixa() {
        let toml = aircraft_toml_valido().replace("fuselage_kf = 0.02", "fuselage_kf = 0.10");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("stability.fuselage_kf"), "{err}");
    }

    #[test]
    fn rejeita_fuselage_kf_nao_positivo() {
        let toml = aircraft_toml_valido().replace("fuselage_kf = 0.02", "fuselage_kf = 0.0");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("stability.fuselage_kf"), "{err}");
    }

    #[test]
    fn rejeita_fuselage_kf_nao_finito() {
        let toml = aircraft_toml_valido().replace("fuselage_kf = 0.02", "fuselage_kf = nan");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("finito"), "{err}");
    }

    // ─── migração sm_max → TrimAuthorityAgent (task trim-authority) ────────

    /// Config de uma versão anterior do schema (com `[stability].sm_max`)
    /// deve ser rejeitada com um erro de migração claro, ANTES de qualquer
    /// checagem física — não deve carregar silenciosamente ignorando o
    /// campo desconhecido (comportamento padrão do serde sem
    /// `deny_unknown_fields`).
    #[test]
    fn rejeita_config_antiga_com_sm_max_com_erro_de_migracao_claro() {
        let toml = aircraft_toml_valido().replace(
            "sm_min = 0.05\n",
            "sm_min = 0.05\n            sm_max = 0.25\n",
        );
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("sm_max"), "{err}");
        assert!(err.to_string().contains("substituído"), "{err}");
        assert!(err.to_string().contains("TrimAuthorityAgent"), "{err}");
    }

    // ─── [wing] cm_ac / cm_flap_delta (task trim-authority) ─────────────────

    #[test]
    fn rejeita_cm_ac_fora_da_faixa() {
        let toml = aircraft_toml_valido().replace("cm_ac = -0.008", "cm_ac = -0.20");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("wing.cm_ac"), "{err}");
    }

    #[test]
    fn rejeita_cm_ac_nao_finito() {
        let toml = aircraft_toml_valido().replace("cm_ac = -0.008", "cm_ac = nan");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("finito"), "{err}");
    }

    #[test]
    fn rejeita_cm_flap_delta_fora_da_faixa() {
        let toml = aircraft_toml_valido().replace("cm_flap_delta = -0.30", "cm_flap_delta = -0.70");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("wing.cm_flap_delta"), "{err}");
    }

    #[test]
    fn rejeita_cm_flap_delta_nao_negativo() {
        let toml = aircraft_toml_valido().replace("cm_flap_delta = -0.30", "cm_flap_delta = 0.0");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("wing.cm_flap_delta"), "{err}");
    }

    // ─── [stability] cl_h_max_down / trim_margin / cl_ground_rotation /
    //     to_flap_cm_fraction (task trim-authority) ──────────────────────────

    #[test]
    fn rejeita_cl_h_max_down_fora_da_faixa() {
        let toml = aircraft_toml_valido().replace("cl_h_max_down = 0.85", "cl_h_max_down = 1.5");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("stability.cl_h_max_down"), "{err}");
    }

    #[test]
    fn rejeita_cl_h_max_down_muito_baixo() {
        let toml = aircraft_toml_valido().replace("cl_h_max_down = 0.85", "cl_h_max_down = 0.3");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("stability.cl_h_max_down"), "{err}");
    }

    #[test]
    fn rejeita_trim_margin_negativo() {
        let toml = aircraft_toml_valido().replace("trim_margin = 0.10", "trim_margin = -0.05");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("stability.trim_margin"), "{err}");
    }

    #[test]
    fn rejeita_trim_margin_acima_de_0_3() {
        let toml = aircraft_toml_valido().replace("trim_margin = 0.10", "trim_margin = 0.5");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("stability.trim_margin"), "{err}");
    }

    #[test]
    fn aceita_trim_margin_no_limite_zero() {
        let toml = aircraft_toml_valido().replace("trim_margin = 0.10", "trim_margin = 0.0");
        parse_aircraft(&toml).expect("trim_margin = 0.0 (limite inferior) deveria ser válido");
    }

    #[test]
    fn rejeita_cl_ground_rotation_fora_da_faixa() {
        let toml = aircraft_toml_valido().replace("cl_ground_rotation = 0.5", "cl_ground_rotation = 1.2");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("stability.cl_ground_rotation"), "{err}");
    }

    #[test]
    fn rejeita_cl_ground_rotation_nao_positivo() {
        let toml = aircraft_toml_valido().replace("cl_ground_rotation = 0.5", "cl_ground_rotation = 0.0");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("stability.cl_ground_rotation"), "{err}");
    }

    #[test]
    fn rejeita_to_flap_cm_fraction_fora_de_0_1() {
        let toml = aircraft_toml_valido().replace("to_flap_cm_fraction = 0.5", "to_flap_cm_fraction = 1.3");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("stability.to_flap_cm_fraction"), "{err}");
    }

    #[test]
    fn aceita_to_flap_cm_fraction_no_limite_zero() {
        let toml = aircraft_toml_valido().replace("to_flap_cm_fraction = 0.5", "to_flap_cm_fraction = 0.0");
        parse_aircraft(&toml).expect("to_flap_cm_fraction = 0.0 (limite inferior) deveria ser válido");
    }

    // ─── [performance] (Task 4.7) ────────────────────────────────────────────

    #[test]
    fn rejeita_mu_brake_paved_fora_da_faixa() {
        let toml = aircraft_toml_valido().replace("mu_brake_paved = 0.40", "mu_brake_paved = 0.02");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("performance.mu_brake_paved"), "{err}");
    }

    #[test]
    fn rejeita_mu_brake_grass_fora_da_faixa() {
        let toml = aircraft_toml_valido().replace("mu_brake_grass = 0.30", "mu_brake_grass = 0.90");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("performance.mu_brake_grass"), "{err}");
    }

    #[test]
    fn rejeita_static_thrust_factor_nao_positivo() {
        let toml = aircraft_toml_valido()
            .replace("static_thrust_factor = 0.75", "static_thrust_factor = 0.0");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("performance.static_thrust_factor"), "{err}");
    }

    /// `static_thrust_factor` é uma correção que reduz a tração estática
    /// IDEAL de Rankine-Froude — fisicamente nunca pode ultrapassar 1.0. Sem
    /// este teto, um erro de digitação (ex.: 7.5 em vez de 0.75) passaria
    /// silenciosamente e SUBESTIMARIA as distâncias de decolagem (tração
    /// maior que a ideal é fisicamente impossível para este modelo).
    #[test]
    fn rejeita_static_thrust_factor_acima_de_1() {
        let toml = aircraft_toml_valido()
            .replace("static_thrust_factor = 0.75", "static_thrust_factor = 7.5");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("performance.static_thrust_factor"), "{err}");
    }

    #[test]
    fn rejeita_rotation_time_s_nao_positivo() {
        let toml = aircraft_toml_valido().replace("rotation_time_s = 1.0", "rotation_time_s = -1.0");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("performance.rotation_time_s"), "{err}");
    }

    #[test]
    fn rejeita_flare_time_s_nao_positivo() {
        let toml = aircraft_toml_valido().replace("flare_time_s = 1.5", "flare_time_s = 0.0");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("performance.flare_time_s"), "{err}");
    }

    #[test]
    fn rejeita_approach_angle_deg_fora_da_faixa() {
        let toml = aircraft_toml_valido()
            .replace("approach_angle_deg = 3.0", "approach_angle_deg = 0.5");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("performance.approach_angle_deg"), "{err}");
    }

    #[test]
    fn rejeita_approach_angle_deg_nao_finito() {
        let toml = aircraft_toml_valido()
            .replace("approach_angle_deg = 3.0", "approach_angle_deg = nan");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("finito"), "{err}");
    }

    // ─── [drag] cooling_drag_fraction (Task 5.2) ────────────────────────────

    #[test]
    fn rejeita_cooling_drag_fraction_zero() {
        let toml = aircraft_toml_valido()
            .replace("cooling_drag_fraction = 0.04", "cooling_drag_fraction = 0.0");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("drag.cooling_drag_fraction"), "{err}");
    }

    #[test]
    fn rejeita_cooling_drag_fraction_acima_de_0_10() {
        let toml = aircraft_toml_valido()
            .replace("cooling_drag_fraction = 0.04", "cooling_drag_fraction = 0.15");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("drag.cooling_drag_fraction"), "{err}");
    }

    #[test]
    fn rejeita_cooling_drag_fraction_nao_finito() {
        let toml = aircraft_toml_valido()
            .replace("cooling_drag_fraction = 0.04", "cooling_drag_fraction = nan");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("finito"), "{err}");
    }

    #[test]
    fn aceita_cooling_drag_fraction_no_limite_superior() {
        let toml = aircraft_toml_valido()
            .replace("cooling_drag_fraction = 0.04", "cooling_drag_fraction = 0.10");
        parse_aircraft(&toml).expect("cooling_drag_fraction = 0.10 (limite superior) deveria ser válido");
    }

    // ─── [electrical] (Task 5.2) ─────────────────────────────────────────────

    #[test]
    fn rejeita_bus_voltage_fora_dos_padroes() {
        let toml = aircraft_toml_valido().replace("bus_voltage_v = 28.0", "bus_voltage_v = 19.0");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("electrical.bus_voltage_v"), "{err}");
    }

    #[test]
    fn aceita_bus_voltage_padrao_12v() {
        let toml = aircraft_toml_valido().replace("bus_voltage_v = 28.0", "bus_voltage_v = 12.0");
        parse_aircraft(&toml).expect("bus_voltage_v = 12.0 deveria ser um barramento padrão válido");
    }

    #[test]
    fn rejeita_alternator_w_nao_positivo() {
        let toml = aircraft_toml_valido().replace("alternator_w = 900.0", "alternator_w = 0.0");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("electrical.alternator_w"), "{err}");
    }

    #[test]
    fn rejeita_electrical_loads_vazio() {
        let base = aircraft_toml_valido();
        let head = base.split("[electrical]").next().unwrap();
        let toml = format!(
            "{head}\n[electrical]\nbus_voltage_v = 28.0\nalternator_w = 900.0\nloads = []\n"
        );
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("electrical.loads"), "{err}");
    }

    #[test]
    fn rejeita_electrical_load_continuous_w_negativo() {
        let toml = aircraft_toml_valido()
            .replace("continuous_w = 180.0", "continuous_w = -5.0");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("continuous_w"), "{err}");
    }

    #[test]
    fn rejeita_electrical_load_peak_w_negativo() {
        let toml = aircraft_toml_valido()
            .replace("peak_w = 220.0", "peak_w = -1.0");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("peak_w"), "{err}");
    }

    #[test]
    fn rejeita_electrical_load_nome_duplicado() {
        let toml = aircraft_toml_valido()
            .replace(r#"name = "flaps""#, r#"name = "avionicos""#);
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("nome duplicado"), "{err}");
        assert!(err.to_string().contains("avionicos"), "{err}");
    }

    /// Guarda de consistência (Task 5.2): peak_w da carga 'trem_retratil'
    /// deve ser >= à potência mecânica do atuador calculada por
    /// `agents::landing_gear::actuator_power_w` a partir de
    /// `gear.mass_main_leg_kg`/`gear.retraction_time_s` — mesmo padrão da
    /// guarda 'trem_principal' × `mass_main_leg_kg` já existente.
    #[test]
    fn rejeita_peak_trem_retratil_abaixo_do_atuador_calculado() {
        // Atuador calculado para esta fixture (mass_main_leg_kg=25.0,
        // retraction_time_s=7.0) ≈ 16.8 W — 1.0 W fica abaixo disso.
        let toml = aircraft_toml_valido().replace("peak_w = 520.0", "peak_w = 1.0");
        let err = parse_aircraft(&toml).unwrap_err();
        assert!(err.to_string().contains("trem_retratil"), "{err}");
        assert!(err.to_string().contains("atuador"), "{err}");
    }

    #[test]
    fn aceita_peak_trem_retratil_acima_do_atuador_calculado() {
        // Atuador calculado para esta fixture ≈ 16.812 W — 17.0 W fica
        // acima (não testamos o limite exato de igualdade em ponto
        // flutuante, que seria frágil a arredondamento entre o literal TOML
        // e o resultado da mesma conta em Rust).
        let toml = aircraft_toml_valido().replace("peak_w = 520.0", "peak_w = 17.0");
        parse_aircraft(&toml).expect("peak_w acima do atuador calculado deveria ser aceito");
    }

    // ─── MISSÃO (REQUISITOS) ────────────────────────────────────────────────

    /// TOML de missão mínimo porém válido, usado como base para os testes de
    /// validação abaixo (cada teste sobrescreve um trecho para violar
    /// exatamente uma invariante).
    fn mission_toml_valido() -> String {
        r#"
            passengers = 4
            pax_mass_kg = 90.0
            baggage_kg = 80.0
            cruise_speed_min_kmh = 280.0
            endurance_min_h = 8.0
            fuel_reserve_fraction = 0.10
            cruise_altitude_m = 2500.0
            airfield_altitude_m = 0.0
            isa_delta_c = 0.0

            [analysis]
            taxi_fuel_l = 3.0
            descent_rate_ms = 4.0
            descent_power_fraction = 0.20
            climb_speed_policy = "vy"
        "#
        .to_string()
    }

    #[test]
    fn payload_kg_respeita_pax_mass_kg_configurado() {
        let req = parse_mission(&mission_toml_valido()).unwrap();
        // 4 pax × 90 kg + 80 kg bagagem = 440 kg
        assert_eq!(req.payload_kg(), 440.0);

        let req_leve = parse_mission(&mission_toml_valido().replace("pax_mass_kg = 90.0", "pax_mass_kg = 70.0"))
            .unwrap();
        // 4 pax × 70 kg + 80 kg bagagem = 360 kg — muda com pax_mass_kg
        assert_eq!(req_leve.payload_kg(), 360.0);
    }

    #[test]
    fn mission_toml_valido_carrega_sem_erro() {
        parse_mission(&mission_toml_valido()).expect("TOML de missão de teste deveria ser válido");
    }

    #[test]
    fn rejeita_zero_passageiros() {
        let toml = mission_toml_valido().replace("passengers = 4", "passengers = 0");
        let err = parse_mission(&toml).unwrap_err();
        assert!(err.to_string().contains("passengers"), "{err}");
    }

    #[test]
    fn rejeita_pax_mass_kg_nao_finito() {
        let toml = mission_toml_valido().replace("pax_mass_kg = 90.0", "pax_mass_kg = nan");
        let err = parse_mission(&toml).unwrap_err();
        assert!(err.to_string().contains("pax_mass_kg"), "{err}");
        assert!(err.to_string().contains("finito"), "{err}");
    }

    #[test]
    fn rejeita_reserva_de_combustivel_acima_de_0_5() {
        let toml = mission_toml_valido().replace("fuel_reserve_fraction = 0.10", "fuel_reserve_fraction = 0.9");
        let err = parse_mission(&toml).unwrap_err();
        assert!(err.to_string().contains("fuel_reserve_fraction"), "{err}");
    }

    #[test]
    fn rejeita_altitude_de_aerodromo_acima_da_altitude_de_cruzeiro() {
        let toml = mission_toml_valido().replace("airfield_altitude_m = 0.0", "airfield_altitude_m = 3000.0");
        let err = parse_mission(&toml).unwrap_err();
        assert!(err.to_string().contains("airfield_altitude_m"), "{err}");
        assert!(err.to_string().contains("cruise_altitude_m"), "{err}");
    }

    #[test]
    fn rejeita_isa_delta_fora_de_40() {
        let toml = mission_toml_valido().replace("isa_delta_c = 0.0", "isa_delta_c = 55.0");
        let err = parse_mission(&toml).unwrap_err();
        assert!(err.to_string().contains("isa_delta_c"), "{err}");
    }

    #[test]
    fn rejeita_baggage_kg_negativo() {
        let toml = mission_toml_valido().replace("baggage_kg = 80.0", "baggage_kg = -10.0");
        let err = parse_mission(&toml).unwrap_err();
        assert!(err.to_string().contains("baggage_kg"), "{err}");
    }

    #[test]
    fn rejeita_cruise_speed_nao_positivo() {
        let toml = mission_toml_valido().replace("cruise_speed_min_kmh = 280.0", "cruise_speed_min_kmh = 0.0");
        let err = parse_mission(&toml).unwrap_err();
        assert!(err.to_string().contains("cruise_speed_min_kmh"), "{err}");
    }

    #[test]
    fn rejeita_endurance_nao_positivo() {
        let toml = mission_toml_valido().replace("endurance_min_h = 8.0", "endurance_min_h = 0.0");
        let err = parse_mission(&toml).unwrap_err();
        assert!(err.to_string().contains("endurance_min_h"), "{err}");
    }

    #[test]
    fn rejeita_cruise_altitude_acima_de_10000() {
        let toml = mission_toml_valido().replace("cruise_altitude_m = 2500.0", "cruise_altitude_m = 12000.0");
        let err = parse_mission(&toml).unwrap_err();
        assert!(err.to_string().contains("cruise_altitude_m"), "{err}");
    }

    // ─── [analysis] (Task 5.1) ──────────────────────────────────────────────

    #[test]
    fn rejeita_taxi_fuel_l_nao_positivo() {
        let toml = mission_toml_valido().replace("taxi_fuel_l = 3.0", "taxi_fuel_l = 0.0");
        let err = parse_mission(&toml).unwrap_err();
        assert!(err.to_string().contains("taxi_fuel_l"), "{err}");
    }

    #[test]
    fn rejeita_taxi_fuel_l_nao_finito() {
        let toml = mission_toml_valido().replace("taxi_fuel_l = 3.0", "taxi_fuel_l = nan");
        let err = parse_mission(&toml).unwrap_err();
        assert!(err.to_string().contains("taxi_fuel_l"), "{err}");
        assert!(err.to_string().contains("finito"), "{err}");
    }

    #[test]
    fn rejeita_descent_rate_ms_nao_positivo() {
        let toml = mission_toml_valido().replace("descent_rate_ms = 4.0", "descent_rate_ms = -1.0");
        let err = parse_mission(&toml).unwrap_err();
        assert!(err.to_string().contains("descent_rate_ms"), "{err}");
    }

    #[test]
    fn rejeita_descent_power_fraction_abaixo_de_0_05() {
        let toml = mission_toml_valido()
            .replace("descent_power_fraction = 0.20", "descent_power_fraction = 0.01");
        let err = parse_mission(&toml).unwrap_err();
        assert!(err.to_string().contains("descent_power_fraction"), "{err}");
    }

    #[test]
    fn rejeita_descent_power_fraction_acima_de_0_5() {
        let toml = mission_toml_valido()
            .replace("descent_power_fraction = 0.20", "descent_power_fraction = 0.6");
        let err = parse_mission(&toml).unwrap_err();
        assert!(err.to_string().contains("descent_power_fraction"), "{err}");
    }

    #[test]
    fn rejeita_climb_speed_policy_diferente_de_vy() {
        let toml = mission_toml_valido()
            .replace(r#"climb_speed_policy = "vy""#, r#"climb_speed_policy = "vx""#);
        let err = parse_mission(&toml).unwrap_err();
        assert!(err.to_string().contains("climb_speed_policy"), "{err}");
    }

    #[test]
    fn aceita_variante_sintetica_de_analysis_taxi_e_descida() {
        // Fixture usada em `requirements::test_fixtures::requisitos_teste`
        // (taxi 2.5 L, descida 3.5 m/s @ 25% potência) — garante que a
        // validação aceita valores diferentes do baseline real, não só os
        // do `default.toml`.
        let toml = mission_toml_valido()
            .replace("taxi_fuel_l = 3.0", "taxi_fuel_l = 2.5")
            .replace("descent_rate_ms = 4.0", "descent_rate_ms = 3.5")
            .replace("descent_power_fraction = 0.20", "descent_power_fraction = 0.25");
        let req = parse_mission(&toml).expect("variante sintética deveria ser válida");
        assert_eq!(req.analysis.taxi_fuel_l, 2.5);
        assert_eq!(req.analysis.descent_rate_ms, 3.5);
        assert_eq!(req.analysis.descent_power_fraction, 0.25);
    }
}
