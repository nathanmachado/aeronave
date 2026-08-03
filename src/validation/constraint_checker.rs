/// ConstraintChecker — verifica se os resultados dos agentes satisfazem
/// os requisitos do projeto e reporta violações com detalhamento.

use crate::agents::weight_balance::WeightBalanceOutput;
use crate::models::{engine::EngineSpec, requirements::Requirements, specs::{WingSpec, PropulsionSpec, PropellerSpec, PerformanceSpec}};

/// Gradiente de subida mínimo exigido pela CS 23.65 para esta categoria (%).
const CLIMB_GRADIENT_MIN_PCT: f64 = 8.3;

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
        propeller: &PropellerSpec,
        perf: &PerformanceSpec,
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

        // 11. Mach de ponta de pá e folga de solo (Task 4.5) — estático,
        // cruzeiro e folga, cada um reportado com o diâmetro (config ou
        // derivado) que produziu o resultado.
        if !propeller.ok_mach_static {
            violations.push(format!(
                "Hélice: Mach de ponta ESTÁTICO {:.3} acima do admissível \
                 (diâmetro {:.2}m, fonte: {})",
                propeller.tip_mach_static, propeller.diameter_m, propeller.source
            ));
        }
        if !propeller.ok_mach_cruise {
            violations.push(format!(
                "Hélice: Mach de ponta em CRUZEIRO (helicoidal) {:.3} acima do admissível \
                 (diâmetro {:.2}m, fonte: {})",
                propeller.tip_mach_cruise_helical, propeller.diameter_m, propeller.source
            ));
        }
        if !propeller.ok_clearance {
            violations.push(format!(
                "Hélice: folga de solo {:.3}m abaixo do mínimo exigido \
                 (diâmetro {:.2}m, fonte: {})",
                propeller.ground_clearance_m, propeller.diameter_m, propeller.source
            ));
        }

        // 12. Diâmetro derivado × provisório (mitigação pós-revisão da Task
        // 4.5): quando `[propeller].diameter_m` está omitido, o resultado de
        // propulsão foi calculado com um diâmetro PROVISÓRIO (só folga de
        // solo — ver `agents::propeller::diameter_mismatch_warning`), que
        // pode divergir do diâmetro AUTORITATIVO acima quando o Mach de
        // ponta (não a folga) é a restrição mais apertada. Isto é um AVISO,
        // não uma violação — o resultado continua fisicamente válido, só
        // potencialmente inconsistente entre a hélice recomendada e o
        // rpm/BSFC/consumo de cruzeiro reportados.
        if let Some(aviso) = crate::agents::propeller::diameter_mismatch_warning(propeller, prop) {
            warnings.push(aviso);
        }

        // 13. Gradiente de subida (Task 4.7, CS 23.65): a categoria exige um
        // gradiente mínimo de 8.3% (avaliado em Vx, ao nível do mar, MTOW —
        // ver `agents::performance::best_climb_angle_ms`).
        if perf.climb_gradient_pct < CLIMB_GRADIENT_MIN_PCT {
            violations.push(format!(
                "Gradiente de subida {:.1}% abaixo do mínimo de {:.1}% exigido pela CS 23.65 \
                 (Vx={:.1}km/h)",
                perf.climb_gradient_pct, CLIMB_GRADIENT_MIN_PCT, perf.vx_kmh
            ));
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
    use crate::agents::performance::PerformanceAgent;
    use crate::agents::propeller::PropellerAgent;
    use crate::agents::propulsion::PropulsionAgent;
    use crate::agents::weight_balance::WeightBalanceAgent;
    use crate::models::aircraft_state::AircraftState;
    use crate::models::engine::test_fixtures::motor_generico_teste;

    /// Monta um `(Requirements, WingSpec, PropulsionSpec, EngineSpec,
    /// WeightBalanceOutput, PropellerSpec, PerformanceSpec)` coerente via os
    /// agentes reais (motor sintético de teste — não um motor real) a partir
    /// de uma `AircraftConfig` fornecida — usada por `setup()` (fixture
    /// padrão) e pelos testes de aviso de divergência de diâmetro abaixo, que
    /// precisam mutar `cfg.propeller` ANTES do pipeline rodar (o aviso
    /// depende do diâmetro PROVISÓRIO real calculado por
    /// `AircraftState::from_config`, não de um valor sobrescrito depois).
    fn setup_with_cfg(cfg: crate::models::aircraft_config::AircraftConfig)
        -> (Requirements, WingSpec, PropulsionSpec, EngineSpec, WeightBalanceOutput, PropellerSpec, PerformanceSpec)
    {
        let state  = AircraftState::from_config(&cfg);
        let req    = crate::models::requirements::test_fixtures::requisitos_teste();
        let wing   = AerodynamicsAgent::run(&state, &req);
        let engine = motor_generico_teste();
        let prop   = PropulsionAgent::run(&state, &req, &wing, &engine);
        let emp    = EmpennageAgent::run(&wing, &cfg);
        let wb     = WeightBalanceAgent::run(&state, &wing, &engine, &cfg, &req, &emp);
        let propeller = PropellerAgent::run(&cfg, &engine, &prop, &req);
        let perf   = PerformanceAgent::run(&state, &wing, &prop, state.mtow_kg, &engine, &req,
                                            &cfg.performance);
        (req, wing, prop, engine, wb, propeller, perf)
    }

    /// Base de todas as asserções de violação isolada abaixo — usa a
    /// fixture sintética padrão (`config_teste()`). Os testes sobrescrevem
    /// apenas os campos relevantes à violação isolada em questão, para não
    /// depender do resultado real dos demais agentes (já testados em seus
    /// próprios módulos).
    fn setup() -> (Requirements, WingSpec, PropulsionSpec, EngineSpec, WeightBalanceOutput, PropellerSpec, PerformanceSpec) {
        setup_with_cfg(crate::models::aircraft_config::test_fixtures::config_teste())
    }

    #[test]
    fn violacao_cruzeiro_inviavel_aparece_quando_infeasible() {
        let (req, wing, mut prop, engine, wb, propeller, perf) = setup();
        // Força a inviabilidade independentemente do resultado real da busca
        // de rpm, para testar isoladamente a violação #9.
        prop.cruise_feasible = false;
        prop.p_req_cruise_kw = 150.0;
        prop.p_shaft_cruise_kw = 100.0;

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf);

        assert!(report.violations.iter().any(|v| v.contains("Cruzeiro inviável")),
            "esperava violação de cruzeiro inviável, obteve: {:?}", report.violations);
        // A mensagem deve carregar os números reais, não só um rótulo.
        assert!(report.violations.iter().any(|v| v.contains("150.0") && v.contains("100.0")),
            "violação deveria citar P_req/P_shaft: {:?}", report.violations);
    }

    #[test]
    fn sem_violacao_cruzeiro_quando_feasible() {
        let (req, wing, mut prop, engine, wb, propeller, perf) = setup();
        prop.cruise_feasible = true;
        prop.p_req_cruise_kw = 90.0;
        prop.p_shaft_cruise_kw = 100.0;

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf);

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
        let (req, wing, prop, engine, wb, propeller, perf) = setup();
        assert!(wb.scenarios.iter().any(|s| !s.inside_envelope),
            "pré-condição do teste: fixture sintética deveria ter ao menos um \
             cenário fora do envelope (achado honesto, replicado do baseline real)");

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf);

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
        let (req, wing, prop, engine, mut wb, propeller, perf) = setup();
        for sc in &mut wb.scenarios {
            sc.inside_envelope = true;
        }

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf);

        assert!(!report.violations.iter().any(|v| v.contains("fora do envelope de CG admissível")),
            "não deveria haver violação de envelope de CG, obteve: {:?}", report.violations);
    }

    // ─── Task 4.5: hélice (Mach de ponta / folga de solo) ──────────────────

    #[test]
    fn violacao_de_helice_aparece_quando_algum_ok_e_falso() {
        let (req, wing, prop, engine, wb, mut propeller, perf) = setup();
        propeller.ok_mach_static = false;
        propeller.tip_mach_static = 0.99;

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf);

        assert!(report.violations.iter().any(|v| v.contains("Mach de ponta ESTÁTICO")),
            "esperava violação de Mach de ponta estático, obteve: {:?}", report.violations);
        assert!(report.violations.iter().any(|v| v.contains("0.990")),
            "violação deveria citar o Mach observado: {:?}", report.violations);
    }

    /// Achado honesto da fixture sintética (mesma filosofia da fixture de
    /// envelope de CG, Task 4.4): `config_teste()` usa `shaft_height_m=1.15`,
    /// `diameter_m=Some(1.90)`, `ground_clearance_min_m=0.25` — folga real =
    /// 1,15 − 0,95 = 0,20 m < 0,25 m, então a checagem de folga de solo falha
    /// naturalmente (não precisa de override) para esta fixture.
    #[test]
    fn violacao_de_folga_de_solo_aparece_naturalmente_na_fixture_sintetica() {
        let (req, wing, prop, engine, wb, propeller, perf) = setup();
        assert!(!propeller.ok_clearance,
            "pré-condição do teste: fixture sintética (shaft_height_m=1.15, diameter=1.90, \
             ground_clearance_min_m=0.25) deveria falhar na folga de solo — obtido \
             ground_clearance_m={:.3}", propeller.ground_clearance_m);

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf);

        assert!(report.violations.iter().any(|v| v.contains("folga de solo")),
            "esperava violação de folga de solo, obteve: {:?}", report.violations);
    }

    /// Sanidade inversa: se TODOS os `ok_*` estiverem artificialmente
    /// verdadeiros (mesmo padrão de `sem_violacao_de_envelope_...` acima),
    /// nenhuma violação de hélice deve aparecer.
    #[test]
    fn sem_violacao_de_helice_quando_todos_ok_forcado() {
        let (req, wing, prop, engine, wb, mut propeller, perf) = setup();
        propeller.ok_mach_static = true;
        propeller.ok_mach_cruise = true;
        propeller.ok_clearance = true;

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf);

        assert!(!report.violations.iter().any(|v| v.contains("Hélice:")),
            "não deveria haver violação de hélice, obteve: {:?}", report.violations);
    }

    // ─── Aviso de divergência: diâmetro derivado × provisório (mitigação) ──
    //
    // Testes de integração de ponta a ponta (via `AircraftState::from_config`
    // real, não specs construídas à mão) — complementam os testes unitários
    // de `agents::propeller::diameter_mismatch_warning` exercitando a
    // fiação real: `AircraftState` calcula o diâmetro PROVISÓRIO,
    // `PropulsionAgent` o usa na busca de cruzeiro, `PropellerAgent` deriva
    // o AUTORITATIVO, e `ConstraintChecker::verify` compara os dois.

    /// PSRU 1:1 (em vez do ~2.0 da fixture padrão) faz o Mach de ponta — não
    /// a folga de solo — governar o diâmetro derivado, divergindo do
    /// provisório usado pela busca de cruzeiro.
    #[test]
    fn aviso_de_diametro_aparece_quando_mach_governa_com_pipeline_real() {
        let mut cfg = crate::models::aircraft_config::test_fixtures::config_teste();
        cfg.propeller.diameter_m = None;
        cfg.propeller.psru_ratio = 1.0;

        let (req, wing, prop, engine, wb, propeller, perf) = setup_with_cfg(cfg);
        assert_eq!(propeller.source, "derivado");
        println!(
            "pipeline real (Mach governa): D_autoritativo={:.4} D_provisorio(prop.prop_diameter_m)={:.4}",
            propeller.diameter_m, prop.prop_diameter_m
        );
        assert!((propeller.diameter_m - prop.prop_diameter_m).abs() > 0.01,
            "pré-condição do teste: com PSRU 1:1 o diâmetro autoritativo ({:.4}) deveria \
             divergir do provisório ({:.4}) usado pela busca de cruzeiro",
            propeller.diameter_m, prop.prop_diameter_m);

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf);

        assert!(report.warnings.iter().any(|w| w.contains("Diâmetro de hélice derivado")),
            "esperava aviso de divergência de diâmetro, obteve: {:?}", report.warnings);
        // É AVISO, não violação — o resultado continua fisicamente válido.
        assert!(!report.violations.iter().any(|v| v.contains("Diâmetro de hélice derivado")),
            "divergência de diâmetro deveria ser aviso, não violação: {:?}", report.violations);
    }

    /// Fixture padrão (`config_teste()`, PSRU~2.0) com diâmetro omitido: a
    /// folga de solo governa o diâmetro derivado (ver
    /// `agents::propeller::tests::diametro_derivado_respeita_ambos_os_maximos_com_margem`),
    /// então provisório e autoritativo coincidem e nenhum aviso deve
    /// disparar.
    #[test]
    fn sem_aviso_de_diametro_quando_folga_governa_com_pipeline_real() {
        let mut cfg = crate::models::aircraft_config::test_fixtures::config_teste();
        cfg.propeller.diameter_m = None;

        let (req, wing, prop, engine, wb, propeller, perf) = setup_with_cfg(cfg);
        assert_eq!(propeller.source, "derivado");
        println!(
            "pipeline real (folga governa): D_autoritativo={:.4} D_provisorio(prop.prop_diameter_m)={:.4}",
            propeller.diameter_m, prop.prop_diameter_m
        );

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf);

        assert!(!report.warnings.iter().any(|w| w.contains("Diâmetro de hélice derivado")),
            "não deveria haver aviso de divergência de diâmetro quando a folga governa, \
             obteve: {:?}", report.warnings);
    }

    // ─── Task 4.7: gradiente de subida (CS 23.65) ───────────────────────────

    /// `PerformanceSpec` sintética mínima para os testes de violação de
    /// gradiente abaixo — literal construída à mão (não via `PerformanceAgent`)
    /// para isolar exatamente a checagem #13 de `ConstraintChecker::verify`,
    /// sem depender do resultado real de nenhum outro agente. Só
    /// `climb_gradient_pct`/`vx_kmh` importam para essa checagem; os demais
    /// campos recebem valores plausíveis arbitrários.
    fn performance_spec_com_gradiente(climb_gradient_pct: f64) -> PerformanceSpec {
        PerformanceSpec {
            v_cruise_kmh: 300.0,
            v_stall_kmh: 100.0,
            rc_sl_ms: 5.0,
            rc_cruise_alt_ms: 4.0,
            service_ceiling_m: 5_000.0,
            to_distance_paved_m: 300.0,
            to_distance_grass_m: 360.0,
            landing_distance_m: 400.0,
            range_km: 2_000.0,
            endurance_h: 8.0,
            vx_kmh: 120.0,
            vy_kmh: 150.0,
            best_glide_kmh: 170.0,
            glide_ratio: 15.0,
            climb_gradient_pct,
            to_50ft_paved_m: 400.0,
            to_50ft_grass_m: 450.0,
            ldg_50ft_m: 550.0,
        }
    }

    #[test]
    fn violacao_de_gradiente_aparece_quando_abaixo_de_8_3_por_cento() {
        let (req, wing, prop, engine, wb, propeller, _perf) = setup();
        let perf = performance_spec_com_gradiente(6.0);

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf);

        assert!(report.violations.iter().any(|v| v.contains("Gradiente de subida")),
            "esperava violação de gradiente de subida, obteve: {:?}", report.violations);
        // A mensagem deve citar o gradiente observado e o piso da CS 23.65.
        assert!(report.violations.iter().any(|v| v.contains("6.0") && v.contains("8.3")),
            "violação deveria citar o gradiente observado (6.0%) e o mínimo CS 23.65 \
             (8.3%): {:?}", report.violations);
    }

    #[test]
    fn sem_violacao_de_gradiente_quando_maior_ou_igual_a_8_3_por_cento() {
        let (req, wing, prop, engine, wb, propeller, _perf) = setup();
        let perf = performance_spec_com_gradiente(9.0);

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf);

        assert!(!report.violations.iter().any(|v| v.contains("Gradiente de subida")),
            "não deveria haver violação de gradiente quando climb_gradient_pct (9.0%) >= \
             8.3%, obteve: {:?}", report.violations);
    }
}
