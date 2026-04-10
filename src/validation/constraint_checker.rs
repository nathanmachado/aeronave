/// ConstraintChecker — verifica se os resultados dos agentes satisfazem
/// os requisitos do projeto e reporta violações com detalhamento.

use crate::models::{requirements::Requirements, specs::{WingSpec, PropulsionSpec}};

#[derive(Debug)]
pub struct ConstraintReport {
    pub violations: Vec<String>,
    pub warnings: Vec<String>,
}

impl ConstraintReport {
    pub fn all_satisfied(&self) -> bool {
        self.violations.is_empty()
    }
}

pub struct ConstraintChecker;

impl ConstraintChecker {
    pub fn verify(
        req: &Requirements,
        wing: &WingSpec,
        prop: &PropulsionSpec,
        mtow_kg: f64,
    ) -> ConstraintReport {
        let mut violations = Vec::new();
        let mut warnings   = Vec::new();

        // 1. Velocidade de cruzeiro
        // (verificado indiretamente: se P_avail > P_req ao calcular propulsão
        //  com V = V_min_cruise, o requisito está satisfeito)
        // Aqui verificamos se a razão L/D está dentro do esperado
        if wing.ld_ratio_cruise < 10.0 {
            violations.push(format!(
                "L/D cruzeiro {:.1} abaixo do mínimo de 10 (eficiência insuficiente)",
                wing.ld_ratio_cruise
            ));
        }

        // 2. Velocidade de stall: CS-23 exige V_s ≤ V_cruise / 1.8
        let v_stall_limit = req.cruise_speed_min_kmh / 1.8;
        if wing.stall_speed_kmh > v_stall_limit {
            violations.push(format!(
                "V_stall {:.1} km/h excede limite CS-23 de {:.1} km/h (= V_cruise/1.8)",
                wing.stall_speed_kmh, v_stall_limit
            ));
        }

        // 3. Autonomia
        if prop.endurance_h < req.endurance_min_h {
            violations.push(format!(
                "Autonomia {:.2} h abaixo do requisito de {:.1} h",
                prop.endurance_h, req.endurance_min_h
            ));
        }

        // 4. Razão de aspecto: AR > 8 para eficiência em cruzeiro de longa distância
        if wing.aspect_ratio < 8.0 {
            warnings.push(format!(
                "AR {:.1} abaixo de 8 — considerar aumentar envergadura para melhor eficiência",
                wing.aspect_ratio
            ));
        }

        // 5. Stall em boa margem: V_stall deve ser < 115 km/h para operação em grama/terra
        if wing.stall_speed_kmh > 115.0 {
            warnings.push(format!(
                "V_stall {:.1} km/h acima de 115 km/h — distâncias de pouso em grama aumentam",
                wing.stall_speed_kmh
            ));
        }

        // 6. Consumo: alerta se acima de 35 L/h (degrada autonomia)
        if prop.fc_cruise_lph > 35.0 {
            violations.push(format!(
                "Consumo cruzeiro {:.1} L/h acima do limite de 35 L/h para autonomia de 8h",
                prop.fc_cruise_lph
            ));
        }

        // 7. Alcance mínimo
        let range_req = req.cruise_speed_min_kmh * req.endurance_min_h;
        if prop.range_km < range_req {
            violations.push(format!(
                "Alcance {:.0} km abaixo do requisito de {:.0} km",
                prop.range_km, range_req
            ));
        }

        // 8. MTOW razoável para 204 hp (fator de carga de potência)
        let hp_per_tonne = 204.0 / (mtow_kg / 1_000.0);
        if hp_per_tonne < 100.0 {
            warnings.push(format!(
                "Potência específica {:.0} hp/t abaixo de 100 hp/t — razão de subida pode ser limitada",
                hp_per_tonne
            ));
        }

        ConstraintReport { violations, warnings }
    }
}
