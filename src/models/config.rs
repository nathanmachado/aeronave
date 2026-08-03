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
    let cfg: AircraftConfig = toml::from_str(toml_str)?;
    validate_aircraft(&cfg)?;
    Ok(cfg)
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

/// Valida as invariantes físicas e de consistência de uma `AircraftConfig`
/// recém-carregada.
fn validate_aircraft(cfg: &AircraftConfig) -> Result<(), ConfigError> {
    require_positive("mtow_guess_kg", cfg.mtow_guess_kg)?;

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

    // [fuselage]
    require_positive("fuselage.length_m", cfg.fuselage.length_m)?;
    require_positive("fuselage.cabin_width_m", cfg.fuselage.cabin_width_m)?;
    require_positive("fuselage.cabin_height_m", cfg.fuselage.cabin_height_m)?;
    require_non_negative("fuselage.cd0", cfg.fuselage.cd0)?;

    // [empennage]
    require_non_negative("empennage.cd0", cfg.empennage.cd0)?;
    require_positive("empennage.tail_arm_m", cfg.empennage.tail_arm_m)?;

    // [propeller]
    require_positive("propeller.diameter_m", cfg.propeller.diameter_m)?;
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
            mtow_guess_kg = 1000.0
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
            [fuselage]
            length_m = 7.5
            cabin_width_m = 1.1
            cabin_height_m = 1.1
            cd0 = 0.01
            [empennage]
            cd0 = 0.004
            tail_arm_m = 4.5
            [propeller]
            diameter_m = 1.8
            blades = 2
            psru_ratio = 1.5
            psru_efficiency = 0.95
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
        "#
        .to_string()
    }

    #[test]
    fn aircraft_toml_valido_carrega_sem_erro() {
        parse_aircraft(&aircraft_toml_valido()).expect("TOML de teste deveria ser válido");
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
        let toml = format!("{head}\n[masses]\nitems = []\n");
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
}
