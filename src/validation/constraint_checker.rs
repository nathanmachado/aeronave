/// ConstraintChecker — verifica se os resultados dos agentes satisfazem
/// os requisitos do projeto e reporta violações com detalhamento.

use crate::models::{engine::EngineSpec, requirements::Requirements, specs::{WingSpec, PropulsionSpec}};

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
        engine: &EngineSpec,
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

        // 2. Velocidade de stall: CS-23 exige V_s0 (pouso, com flap) ≤ V_cruise / 1.8
        let v_stall_limit = req.cruise_speed_min_kmh / 1.8;
        if wing.stall_speed_flaps_kmh > v_stall_limit {
            violations.push(format!(
                "V_stall (VS0) {:.1} km/h excede limite CS-23 de {:.1} km/h (= V_cruise/1.8)",
                wing.stall_speed_flaps_kmh, v_stall_limit
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

        // 5. Stall em boa margem: V_s0 (pouso, com flap) deve ser < 115 km/h
        //    para operação em grama/terra
        if wing.stall_speed_flaps_kmh > 115.0 {
            warnings.push(format!(
                "V_stall (VS0) {:.1} km/h acima de 115 km/h — distâncias de pouso em grama aumentam",
                wing.stall_speed_flaps_kmh
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

        // 8. MTOW razoável para a potência do motor instalado (fator de carga de potência)
        let power_hp = engine.power_kw_max() / 0.7457;
        let hp_per_tonne = power_hp / (mtow_kg / 1_000.0);
        if hp_per_tonne < 100.0 {
            warnings.push(format!(
                "Potência específica {:.0} hp/t abaixo de 100 hp/t — razão de subida pode ser limitada",
                hp_per_tonne
            ));
        }

        // 9. Viabilidade de cruzeiro: o rpm de cruzeiro escolhido pela busca
        // (PropulsionAgent) precisa entregar potência de eixo >= potência
        // requerida em voo nivelado. Se não entregar (motor genuinamente
        // incapaz de sustentar a velocidade de cruzeiro exigida com esta
        // célula/hélice/PSRU), isto é uma violação de requisito, não um
        // panic — o motor pode não ser adequado a este projeto.
        if !prop.cruise_feasible {
            violations.push(format!(
                "Cruzeiro inviável: potência requerida {:.1} kW > disponível {:.1} kW \
                 no rpm de cruzeiro escolhido (motor {})",
                prop.p_req_cruise_kw, prop.p_shaft_cruise_kw, engine.name
            ));
        }

        ConstraintReport { violations, warnings }
    }
}
