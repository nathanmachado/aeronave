/// ConstraintChecker — verifica se os resultados dos agentes satisfazem
/// os requisitos do projeto e reporta violações com detalhamento.

use crate::agents::weight_balance::WeightBalanceOutput;
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
        wb: &WeightBalanceOutput,
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

        // 10. Envelope de CG admissível (Task 4.4): critério de aceite
        // substitui a antiga checagem isolada `sm > 0.03` — agora TODO
        // cenário de carga precisa ter o CG dentro de
        // [cg_limit_fwd_pct_mac, cg_limit_aft_pct_mac], os limites vindos
        // dos critérios de estabilidade de `[stability]` (sm_min/sm_max),
        // não apenas os extremos observados entre os cenários.
        // Limites físicos (m do nariz), reconstruídos a partir de
        // `cg_limit_fwd/aft_pct_mac` — inverso de `weight_balance::cg_pct_mac`
        // (%MAC = (x−x_mac_le)/MAC×100) contra `wb.mac_le_x_m`/`wb.mac_m`.
        let x_limit_fwd_m = wb.mac_le_x_m + wb.spec.cg_limit_fwd_pct_mac / 100.0 * wb.mac_m;
        let x_limit_aft_m = wb.mac_le_x_m + wb.spec.cg_limit_aft_pct_mac / 100.0 * wb.mac_m;
        for sc in &wb.scenarios {
            if !sc.inside_envelope {
                violations.push(format!(
                    "Cenário '{}': CG {:.1}% MAC fora do envelope de CG admissível \
                     [{:.1}%–{:.1}%] (SM={:.3}; x_cg={:.3}m, limites em x: \
                     [{:.3}m, {:.3}m])",
                    sc.name, sc.cg_pct_mac,
                    wb.spec.cg_limit_fwd_pct_mac, wb.spec.cg_limit_aft_pct_mac,
                    sc.static_margin, sc.x_cg_m,
                    x_limit_fwd_m, x_limit_aft_m,
                ));
            }
        }

        ConstraintReport { violations, warnings }
    }
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::agents::aerodynamics::AerodynamicsAgent;
    use crate::agents::empennage::EmpennageAgent;
    use crate::agents::propulsion::PropulsionAgent;
    use crate::agents::weight_balance::WeightBalanceAgent;
    use crate::models::aircraft_state::AircraftState;
    use crate::models::engine::test_fixtures::motor_generico_teste;

    /// Monta um `(Requirements, WingSpec, PropulsionSpec, EngineSpec,
    /// WeightBalanceOutput)` coerente via os agentes reais (motor sintético
    /// de teste — não um motor real), para servir de base às asserções de
    /// violação isolada abaixo. Os testes sobrescrevem apenas os campos de
    /// `PropulsionSpec`/`WeightBalanceOutput` relevantes à violação
    /// isolada em questão, para não depender do resultado real dos demais
    /// agentes (já testados em seus próprios módulos).
    fn setup() -> (Requirements, WingSpec, PropulsionSpec, EngineSpec, WeightBalanceOutput) {
        let cfg    = crate::models::aircraft_config::test_fixtures::config_teste();
        let state  = AircraftState::from_config(&cfg);
        let req    = crate::models::requirements::test_fixtures::requisitos_teste();
        let wing   = AerodynamicsAgent::run(&state, &req);
        let engine = motor_generico_teste();
        let prop   = PropulsionAgent::run(&state, &req, &wing, &engine);
        let emp    = EmpennageAgent::run(&wing, &cfg);
        let wb     = WeightBalanceAgent::run(&state, &wing, &engine, &cfg, &req, &emp);
        (req, wing, prop, engine, wb)
    }

    #[test]
    fn violacao_cruzeiro_inviavel_aparece_quando_infeasible() {
        let (req, wing, mut prop, engine, wb) = setup();
        // Força a inviabilidade independentemente do resultado real da busca
        // de rpm, para testar isoladamente a violação #9.
        prop.cruise_feasible = false;
        prop.p_req_cruise_kw = 150.0;
        prop.p_shaft_cruise_kw = 100.0;

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb);

        assert!(report.violations.iter().any(|v| v.contains("Cruzeiro inviável")),
            "esperava violação de cruzeiro inviável, obteve: {:?}", report.violations);
        // A mensagem deve carregar os números reais, não só um rótulo.
        assert!(report.violations.iter().any(|v| v.contains("150.0") && v.contains("100.0")),
            "violação deveria citar P_req/P_shaft: {:?}", report.violations);
    }

    #[test]
    fn sem_violacao_cruzeiro_quando_feasible() {
        let (req, wing, mut prop, engine, wb) = setup();
        prop.cruise_feasible = true;
        prop.p_req_cruise_kw = 90.0;
        prop.p_shaft_cruise_kw = 100.0;

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb);

        assert!(!report.violations.iter().any(|v| v.contains("Cruzeiro inviável")),
            "não deveria haver violação de cruzeiro inviável, obteve: {:?}", report.violations);
    }

    // ─── Task 4.4: envelope de CG admissível ─────────────────────────────

    /// Com a fixture sintética `config_teste()` (mesmo achado honesto do
    /// baseline real — ver `weight_balance::tests` e task-4.4-report.md), os
    /// cenários de carga ficam à frente do limite dianteiro do envelope
    /// (modelo de NP resulta em SM observada bem acima de `sm_max`).
    /// `ConstraintChecker::verify` deve reportar uma violação por cenário
    /// fora do envelope, citando o nome do cenário e os limites em %MAC.
    #[test]
    fn violacao_de_envelope_aparece_quando_cenario_esta_fora() {
        let (req, wing, prop, engine, wb) = setup();
        assert!(wb.scenarios.iter().any(|s| !s.inside_envelope),
            "pré-condição do teste: fixture sintética deveria ter ao menos um \
             cenário fora do envelope (achado honesto, replicado do baseline real)");

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb);

        assert!(report.violations.iter().any(|v| v.contains("fora do envelope de CG admissível")),
            "esperava violação de envelope de CG, obteve: {:?}", report.violations);
        // A mensagem deve citar os limites do envelope em %MAC, não só um rótulo.
        assert!(report.violations.iter().any(|v|
                v.contains(&format!("{:.1}%", wb.spec.cg_limit_fwd_pct_mac))),
            "violação deveria citar o limite dianteiro do envelope: {:?}", report.violations);
    }

    /// Sanidade inversa: se TODOS os cenários estiverem artificialmente
    /// dentro do envelope, nenhuma violação de envelope deve aparecer.
    #[test]
    fn sem_violacao_de_envelope_quando_todos_os_cenarios_estao_dentro() {
        let (req, wing, prop, engine, mut wb) = setup();
        for sc in &mut wb.scenarios {
            sc.inside_envelope = true;
        }

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb);

        assert!(!report.violations.iter().any(|v| v.contains("fora do envelope de CG admissível")),
            "não deveria haver violação de envelope de CG, obteve: {:?}", report.violations);
    }
}
