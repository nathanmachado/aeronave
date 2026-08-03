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
    /// Pontos (rpm, Nm) — interpolação linear entre pontos; 0 fora da faixa.
    /// Invariante: rpm points must be strictly increasing for correct interpolation.
    /// Validation of this invariant is performed when loading from TOML config.
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

    /// Torque máximo (Nm) ao longo de toda a curva — usado para relatório
    /// (não é o torque no rpm de cruzeiro/operação, apenas o pico da curva).
    pub fn torque_max_nm(&self) -> f64 {
        self.torque_curve.iter()
            .map(|p| p[1])
            .fold(0.0_f64, f64::max)
    }

    /// Potência máxima varrendo a curva (para relatório; não usar como referência de carga)
    pub fn power_kw_max(&self) -> f64 {
        (0..=(self.rpm_redline as u32)).step_by(50)
            .map(|r| self.power_kw(r as f64))
            .fold(0.0, f64::max)
    }

    pub fn altitude_factor(&self, altitude_m: f64) -> f64 {
        match self.induction {
            Induction::NaturallyAspirated => {
                let sigma = crate::agents::aerodynamics::isa_density(altitude_m) / 1.225;
                (1.132 * sigma - 0.132).clamp(0.0, 1.0) // Gagg-Ferrar
            }
            Induction::Turbocharged { critical_altitude_m, power_loss_per_1000m } => {
                if altitude_m <= critical_altitude_m { 1.0 }
                else {
                    (1.0 - power_loss_per_1000m
                         * (altitude_m - critical_altitude_m) / 1_000.0).max(0.0)
                }
            }
        }
    }

    pub fn power_kw_at(&self, rpm: f64, altitude_m: f64) -> f64 {
        self.power_kw(rpm) * self.altitude_factor(altitude_m)
    }
}

/// Fixtures de motor sintético compartilhadas por todos os módulos de teste
/// deste crate (`engine.rs`, `propulsion.rs`, `performance.rs`,
/// `constraint_checker.rs`). `pub(crate)` + `#[cfg(test)]`: só existe em
/// builds de teste, mas é visível a qualquer módulo do crate nesse modo —
/// isso elimina a necessidade de cada módulo manter sua própria cópia local
/// dos mesmos números (o que, antes desta refatoração, tinha degenerado em
/// três cópias byte-a-byte dos dados de motores reais dentro de `src/`,
/// reintroduzindo dados de motor real onde não deveriam estar).
///
/// Os valores abaixo são deliberadamente sintéticos — não correspondem a
/// nenhum motor real em `config/engines/`. Motores reais só aparecem em
/// `tests/generic_engine.rs`, carregados dos TOMLs de verdade.
#[cfg(test)]
pub(crate) mod test_fixtures {
    use super::*;

    /// Motor sintético "forte" — usado como motor genérico padrão nos testes
    /// de `propulsion.rs`/`performance.rs`/`constraint_checker.rs` e nos
    /// testes deste próprio módulo.
    ///
    /// Nota sobre os valores: a banda de torque plano (520 Nm) e o fator de
    /// perda do turbo (0.10/1.000m) foram calibrados — via medição empírica,
    /// não estimativa — para que este motor sustente o cruzeiro de 280 km/h
    /// exigido pelo projeto com a célula/hélice/PSRU padrão
    /// (`AircraftState::initial()`) com margem folgada (`p_shaft_cruise_kw`
    /// bem acima de `p_req_cruise_kw`, não empatado por uma casa decimal),
    /// já que o rpm de cruzeiro fica restrito a `[rpm_optimal·0.8,
    /// rpm_optimal·1.2]` = [1.600, 2.400] rpm — abaixo do pico de potência
    /// real da curva (perto de `rpm_rated`). Nenhum destes números coincide
    /// com os TOMLs reais em `config/engines/`.
    pub(crate) fn motor_generico_teste() -> EngineSpec {
        EngineSpec {
            name: "Motor Sintético de Teste".into(),
            mass_kg: 150.0,
            rpm_idle: 800.0,
            rpm_rated: 3_200.0,
            rpm_redline: 3_600.0,
            rpm_max_continuous: 2_900.0,
            torque_curve: vec![
                [800.0, 270.0],
                [1_800.0, 520.0],
                [2_600.0, 520.0],
                [3_200.0, 440.0],
                [3_600.0, 0.0],
            ],
            bsfc: BsfcModel {
                bsfc_min_gkwh: 210.0,
                rpm_optimal: 2_000.0,
                load_optimal: 0.65,
                rpm_penalty_gkwh: 20.0,
                load_penalty_gkwh: 25.0,
                bsfc_max_gkwh: 400.0,
            },
            induction: Induction::Turbocharged {
                critical_altitude_m: 1_800.0,
                power_loss_per_1000m: 0.10,
            },
            fuel: FuelSpec {
                name: "Combustível Teste".into(),
                density_kg_per_l: 0.80,
                lhv_mj_per_kg: 43.0,
            },
        }
    }

    /// Motor sintético "fraco" — potência de pico muito abaixo da necessária
    /// para sustentar o cruzeiro exigido pelo projeto com a célula/hélice/
    /// PSRU padrão (`AircraftState::initial()`), usado para exercitar o
    /// caminho de inviabilidade de `search_cruise_rpm`/`cruise_feasible`.
    pub(crate) fn motor_generico_fraco_teste() -> EngineSpec {
        EngineSpec {
            name: "Motor Sintético Fraco de Teste".into(),
            mass_kg: 65.0,
            rpm_idle: 1_500.0,
            rpm_rated: 5_200.0,
            rpm_redline: 5_200.0,
            rpm_max_continuous: 4_900.0,
            torque_curve: vec![
                [1_500.0, 70.0],
                [3_200.0, 118.0],
                [4_000.0, 122.0],
                [5_200.0, 95.0],
            ],
            bsfc: BsfcModel {
                bsfc_min_gkwh: 265.0,
                rpm_optimal: 4_200.0,
                load_optimal: 0.72,
                rpm_penalty_gkwh: 22.0,
                load_penalty_gkwh: 28.0,
                bsfc_max_gkwh: 410.0,
            },
            induction: Induction::Turbocharged {
                critical_altitude_m: 3_800.0,
                power_loss_per_1000m: 0.08,
            },
            fuel: FuelSpec {
                name: "Combustível Teste Leve".into(),
                density_kg_per_l: 0.75,
                lhv_mj_per_kg: 44.0,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::test_fixtures::motor_generico_teste as engine_teste;

    #[test]
    fn torque_interpola_linearmente() {
        let e = engine_teste();
        assert!((e.torque_nm(2_200.0) - 520.0).abs() < 1.0);   // banda plana (1.800–2.600 rpm)
        assert!((e.torque_nm(1_300.0) - 395.0).abs() < 1.0);   // meio da rampa (800→1.800 rpm)
        assert_eq!(e.torque_nm(500.0), 0.0);                    // abaixo do idle (800 rpm)
        assert_eq!(e.torque_nm(4_000.0), 0.0);                  // acima do redline (3.600 rpm)
    }

    #[test]
    fn potencia_de_torque_e_rpm() {
        let e = engine_teste();
        // P = T·2πN/60 → 440 Nm @ 3.200 rpm (rpm_rated) = 147.45 kW
        let p = e.power_kw(3_200.0);
        assert!((p - 147.45).abs() < 0.1, "P(3200rpm) = {p:.2} kW (esperado ~147.45)");
    }

    #[test]
    fn bsfc_gkwh_penalidades() {
        let e = engine_teste();
        let bsfc_mdl = &e.bsfc;

        // No ponto ótimo (2.000 rpm, 0.65 carga): BSFC ≈ bsfc_min_gkwh
        let bsfc_optimal = bsfc_mdl.bsfc_gkwh(2_000.0, 0.65);
        assert!((bsfc_optimal - 210.0).abs() < 1.0, "optimal BSFC should be ~210 g/kWh");

        // Longe do ótimo: penalidades se aplicam; BSFC aumenta
        let bsfc_far_off = bsfc_mdl.bsfc_gkwh(3_500.0, 0.30);
        assert!(bsfc_far_off > bsfc_optimal, "BSFC far from optimal should be higher");
        assert!(bsfc_far_off <= bsfc_mdl.bsfc_max_gkwh, "BSFC should not exceed max");
    }

    #[test]
    fn torque_maximo_da_curva() {
        let e = engine_teste();
        // Pico da curva de teste é 520 Nm (banda plana 1.800–2.600 rpm)
        assert!((e.torque_max_nm() - 520.0).abs() < 1e-9);
    }

    #[test]
    fn potencia_maxima_da_curva() {
        let e = engine_teste();
        let p_max = e.power_kw_max();
        println!("potencia_maxima_da_curva: p_max = {p_max:.2} kW");

        // Pico de potência ocorre em 3.200 rpm (440 Nm) — apesar do torque
        // cair de 520→440 Nm entre 2.600 e 3.200 rpm, o produto T·rpm ainda
        // cresce nesse trecho, e cai rapidamente só depois de 3.200 rpm
        // (curva vai a 0 em 3.600 rpm). Valor observado empiricamente:
        // 147.45 kW.
        assert!((p_max - 147.45).abs() < 0.1,
            "peak power {p_max:.2} kW divergiu do valor observado (~147.45 kW)");
    }

    #[test]
    fn turbo_mantem_potencia_ate_altitude_critica() {
        let e = engine_teste(); // turbo, crítica 1.800 m, 10%/1.000 m acima
        assert!((e.altitude_factor(0.0) - 1.0).abs() < 1e-9);
        assert!((e.altitude_factor(1_800.0) - 1.0).abs() < 1e-9);
        // a 3.000 m: Δ=1.200m acima da crítica → fator = 1 - 0.10*(1200/1000) = 0.88
        assert!((e.altitude_factor(3_000.0) - 0.88).abs() < 0.01);
    }

    #[test]
    fn aspirado_perde_potencia_por_gagg_ferrar() {
        let mut e = engine_teste();
        e.induction = Induction::NaturallyAspirated;
        // Gagg-Ferrar: P/P0 = 1.132σ − 0.132; a 2.500 m σ≈0.781 → fator ≈ 0.752
        // (não depende dos parâmetros do turbo — mesmo valor de antes)
        let f = e.altitude_factor(2_500.0);
        assert!((f - 0.752).abs() < 0.02, "fator aspirado a 2.500 m = {f:.3}");
    }
}
