//! Carregamento e validação de especificações de motor a partir de arquivos TOML.

use std::path::Path;

use super::engine::EngineSpec;

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
            ConfigError::Io(e) => write!(f, "erro ao ler arquivo de configuração do motor: {e}"),
            ConfigError::Parse(e) => write!(f, "TOML de motor inválido: {e}"),
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn config_path(rel: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(rel)
    }

    #[test]
    fn carrega_os_dois_motores_do_disco() {
        let toyota = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
        let rotax = load_engine(&config_path("config/engines/rotax_915is.toml")).unwrap();
        assert!((toyota.torque_nm(2_400.0) - 500.0).abs() < 1.0);
        assert!((rotax.power_kw(5_800.0) - 67.4).abs() < 3.0); // 111 Nm @ 5800 ≈ 67 kW
        assert!(toyota.fuel.density_kg_per_l < rotax.fuel.density_kg_per_l + 1.0);
    }

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
}
