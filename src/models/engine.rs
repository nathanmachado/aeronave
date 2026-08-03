use serde::{Deserialize, Serialize};

/// Especificação genérica de motor — todos os dados vêm de config TOML.
/// A física (interpolação, P=Tω, correção de altitude) é genérica.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EngineSpec {
    pub name: String,
    pub mass_kg: f64,
    pub rpm_idle: f64,
    pub rpm_rated: f64,
    pub rpm_redline: f64,
    /// RPM máximo de uso contínuo (cruzeiro/subida prolongada)
    pub rpm_max_continuous: f64,
    /// Pontos (rpm, Nm) — interpolação linear entre pontos; 0 fora da faixa
    pub torque_curve: Vec<[f64; 2]>,
    pub bsfc: BsfcModel,
    pub induction: Induction,
    pub fuel: FuelSpec,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Induction {
    /// Aspirado: perda de potência com altitude por Gagg-Ferrar
    NaturallyAspirated,
    /// Turbo: potência plena até a altitude crítica, perda linear acima
    Turbocharged { critical_altitude_m: f64, power_loss_per_1000m: f64 },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FuelSpec {
    pub name: String,
    pub density_kg_per_l: f64,
    /// Poder calorífico inferior (MJ/kg) — para Breguet e validações de BSFC
    pub lhv_mj_per_kg: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BsfcModel {
    pub bsfc_min_gkwh: f64,
    pub rpm_optimal: f64,
    pub load_optimal: f64,
    /// Penalidade: g/kWh por (Δrpm/1000)²
    pub rpm_penalty_gkwh: f64,
    /// Penalidade: g/kWh por (Δload/0.30)²
    pub load_penalty_gkwh: f64,
    pub bsfc_max_gkwh: f64,
}

impl BsfcModel {
    pub fn default_diesel() -> Self {
        Self { bsfc_min_gkwh: 200.0, rpm_optimal: 2_200.0, load_optimal: 0.70,
               rpm_penalty_gkwh: 18.0, load_penalty_gkwh: 22.0, bsfc_max_gkwh: 380.0 }
    }
    pub fn bsfc_gkwh(&self, rpm: f64, load_fraction: f64) -> f64 {
        let rp = ((rpm - self.rpm_optimal) / 1_000.0).powi(2) * self.rpm_penalty_gkwh;
        let lp = ((load_fraction - self.load_optimal) / 0.30).powi(2) * self.load_penalty_gkwh;
        (self.bsfc_min_gkwh + rp + lp).clamp(self.bsfc_min_gkwh * 0.975, self.bsfc_max_gkwh)
    }
}

impl EngineSpec {
    /// Torque por interpolação linear na curva; 0 fora de [primeiro, último] rpm.
    pub fn torque_nm(&self, rpm: f64) -> f64 {
        let pts = &self.torque_curve;
        if pts.len() < 2 { return 0.0; }
        if rpm < pts[0][0] || rpm > pts[pts.len() - 1][0] { return 0.0; }
        for w in pts.windows(2) {
            let ([r0, t0], [r1, t1]) = (w[0], w[1]);
            if rpm >= r0 && rpm <= r1 {
                let f = if r1 > r0 { (rpm - r0) / (r1 - r0) } else { 0.0 };
                return t0 + (t1 - t0) * f;
            }
        }
        0.0
    }

    pub fn power_kw(&self, rpm: f64) -> f64 {
        self.torque_nm(rpm) * rpm * 2.0 * std::f64::consts::PI / 60_000.0
    }

    /// Potência máxima varrendo a curva (para relatório; não usar como referência de carga)
    pub fn power_kw_max(&self) -> f64 {
        (0..=(self.rpm_redline as u32)).step_by(50)
            .map(|r| self.power_kw(r as f64))
            .fold(0.0, f64::max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn engine_teste() -> EngineSpec {
        EngineSpec {
            name: "Motor Genérico Teste".into(),
            mass_kg: 100.0,
            rpm_idle: 700.0, rpm_rated: 3_400.0,
            rpm_redline: 3_800.0, rpm_max_continuous: 3_000.0,
            torque_curve: vec![[700.0, 200.0], [1_600.0, 500.0],
                               [2_800.0, 500.0], [3_400.0, 420.0], [3_800.0, 0.0]],
            bsfc: BsfcModel::default_diesel(),
            induction: Induction::Turbocharged { critical_altitude_m: 2_000.0,
                                                 power_loss_per_1000m: 0.10 },
            fuel: FuelSpec { name: "Diesel S-10".into(),
                             density_kg_per_l: 0.840, lhv_mj_per_kg: 42.5 },
        }
    }

    #[test]
    fn torque_interpola_linearmente() {
        let e = engine_teste();
        assert!((e.torque_nm(2_000.0) - 500.0).abs() < 1.0);   // banda plana
        assert!((e.torque_nm(1_150.0) - 350.0).abs() < 1.0);   // meio da rampa
        assert_eq!(e.torque_nm(500.0), 0.0);                    // abaixo do idle
        assert_eq!(e.torque_nm(4_000.0), 0.0);                  // acima do redline
    }

    #[test]
    fn potencia_de_torque_e_rpm() {
        let e = engine_teste();
        // P = T·2πN/60 → 420 Nm @ 3400 rpm ≈ 149.5 kW
        assert!((e.power_kw(3_400.0) - 149.5).abs() < 1.0);
    }
}
