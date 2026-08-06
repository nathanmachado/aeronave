/// ConstraintChecker — verifica se os resultados dos agentes satisfazem
/// os requisitos do projeto e reporta violações com detalhamento.

use crate::agents::weight_balance::WeightBalanceOutput;
use crate::models::{
    aircraft_config::GearCfg, engine::EngineSpec, requirements::Requirements,
    specs::{WingSpec, PropulsionSpec, PropellerSpec, PerformanceSpec, MissionSpec, ElectricalSpec, GearSpec},
};

/// Gradiente de subida mínimo exigido pela CS 23.65 para esta categoria (%).
const CLIMB_GRADIENT_MIN_PCT: f64 = 8.3;

/// Fração da capacidade do alternador reservada como margem — a carga
/// CONTÍNUA não pode ultrapassar 80% da capacidade nominal (regra de
/// projeto elétrico comum em aviação geral: reserva 20% para degradação
/// do alternador ao longo da vida útil, temperatura alta e cargas
/// transientes não capturadas no orçamento contínuo). Task 5.2.
const ELECTRICAL_CONTINUOUS_MARGIN_FRAC: f64 = 0.80;

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
        mission: &MissionSpec,
        electrical: &ElectricalSpec,
        gear: &GearSpec,
        gear_cfg: &GearCfg,
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

        // (Finding 4 da revisão final — checagens VAZIAS removidas do array
        // de aceite/reprovação, `report.violations`)
        //
        // Duas checagens antigas deste bloco eram VAZIAS por construção —
        // garantidas pela própria matemática de `MissionAgent::run`, não
        // por nenhuma propriedade física da célula/motor/missão candidata —
        // e portanto nunca conseguiam pegar uma regressão real:
        //
        //   - "block_time_h ≥ endurance_min_h" (antiga checagem #3): dado
        //     `MissionAgent::run` retornando `Ok`, `block_time_h` SEMPRE
        //     atende (a subida/descida usam velocidades ≤ V_cruzeiro, então
        //     o tempo de bloco nunca fica abaixo do tempo que a mesma
        //     distância levaria inteira em cruzeiro — ver dedução completa
        //     no comentário de `mission_block_time_h_atende_autonomia_
        //     minima_no_mtow_convergido`, `tests/generic_engine.rs`).
        //   - "range_no_wind_km ≥ range_req" (antiga checagem #7):
        //     `range_no_wind_km` é DEFINIDO em `MissionAgent::run` como a
        //     soma dos três segmentos de distância que FECHA exatamente
        //     `cruise_speed_min_kmh · endurance_min_h` (ver docstring de
        //     `MissionSpec::range_no_wind_km`) — uma identidade algébrica,
        //     não uma checagem.
        //
        // As duas invariantes continuam guardadas por teste (não removidas
        // silenciosamente — só tiradas do array de aceite/reprovação):
        // `tests/generic_engine.rs::mission_block_time_h_atende_autonomia_
        // minima_no_mtow_convergido` (block_time_h) e
        // `tests/generic_engine.rs::margem_de_combustivel_no_mtow_
        // convergido` (identidade de `range_no_wind_km`, tolerância 1e-6).
        // `block_time_h`/`range_no_wind_km` continuam impressos no
        // relatório (`main.rs`) como informação de missão.

        // 3. Razão de aspecto: AR > 8 para eficiência em cruzeiro de longa distância
        if wing.aspect_ratio < 8.0 {
            warnings.push(format!(
                "AR {:.1} abaixo de 8 — considerar aumentar envergadura para melhor eficiência",
                wing.aspect_ratio
            ));
        }

        // 4. Stall em boa margem: V_s0 (pouso, com flap) deve ser < 115 km/h
        //    para operação em grama/terra
        if wing.stall_speed_flaps_kmh > 115.0 {
            warnings.push(format!(
                "V_stall (VS0) {:.1} km/h acima de 115 km/h — distâncias de pouso em grama aumentam",
                wing.stall_speed_flaps_kmh
            ));
        }

        // 5. Consumo: alerta se acima de 35 L/h (degrada autonomia)
        if prop.fc_cruise_lph > 35.0 {
            violations.push(format!(
                "Consumo cruzeiro {:.1} L/h acima do limite de 35 L/h para autonomia de 8h",
                prop.fc_cruise_lph
            ));
        }

        // 6. Alcance Breguet com TANQUE CHEIO (Finding 4 da revisão final —
        // substitui a antiga checagem #7, vazia por construção, ver
        // comentário acima). Checa uma quantidade GENUINAMENTE FALSEÁVEL:
        // `mission.breguet_range_full_tank_km` (Breguet puro de cruzeiro,
        // ZFW → ZFW+tanque cheio, INDEPENDENTE da distância de
        // subida/descida da missão real — ver `MissionAgent::run`) precisa
        // cobrir o alcance mínimo exigido pela missão. Ao contrário da
        // checagem antiga, isto NÃO é garantido pela álgebra de
        // `MissionAgent::run`: a orquestração só garante que o combustível
        // da missão MÍNIMA cabe no tanque (`SizingError::
        // CombustivelInsuficiente`), não que o alcance Breguet do tanque
        // CHEIO cubra `range_req` — uma célula/motor pode, em tese,
        // convergir com folga de combustível pequena o bastante para que
        // esta checagem falhe mesmo com `Ok(sized)`.
        let range_req = req.cruise_speed_min_kmh * req.endurance_min_h;
        if mission.breguet_range_full_tank_km < range_req {
            violations.push(format!(
                "Alcance Breguet com tanque cheio {:.0} km abaixo do requisito de {:.0} km \
                 (a missão mínima cabe no tanque, mas o alcance Breguet do tanque cheio não \
                 cobre a distância exigida)",
                mission.breguet_range_full_tank_km, range_req
            ));
        }

        // 7. MTOW razoável para a potência do motor instalado (fator de carga de potência)
        let power_hp = engine.power_kw_max() / 0.7457;
        let hp_per_tonne = power_hp / (mtow_kg / 1_000.0);
        if hp_per_tonne < 100.0 {
            warnings.push(format!(
                "Potência específica {:.0} hp/t abaixo de 100 hp/t — razão de subida pode ser limitada",
                hp_per_tonne
            ));
        }

        // 8. Viabilidade de cruzeiro: o rpm de cruzeiro escolhido pela busca
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

        // 9. Envelope de CG admissível (Task 4.4 + task trim-authority):
        // critério de aceite substitui a antiga checagem isolada
        // `sm > 0.03` — agora TODO cenário de carga precisa ter o CG
        // dentro de [cg_limit_fwd_pct_mac (flare/rotação,
        // TrimAuthorityAgent — número ÚNICO, o MESMO para todos os
        // cenários desde o fix de revisão do cancelamento de peso na
        // rotação), cg_limit_aft_pct_mac (sm_min)], não apenas os
        // extremos observados entre os cenários. `sc.inside_envelope` já
        // reflete o veredito por cenário (finalizado por
        // `WeightBalanceOutput::apply_trim`); `cg_limit_fwd_pct_mac`
        // citado na mensagem abaixo é o limite que de fato SE APLICA a
        // este (e a todos os outros) cenário.
        // Limites físicos (m do nariz), reconstruídos a partir de
        // `cg_limit_fwd/aft_pct_mac` — inverso de `weight_balance::cg_pct_mac`
        // (%MAC = (x−x_mac_le)/MAC×100) contra `wb.mac_le_x_m`/`wb.mac_m`.
        let x_limit_fwd_m = wb.mac_le_x_m + wb.spec.cg_limit_fwd_pct_mac / 100.0 * wb.mac_m;
        let x_limit_aft_m = wb.mac_le_x_m + wb.spec.cg_limit_aft_pct_mac / 100.0 * wb.mac_m;

        // 9a. ENVELOPE VAZIO (fix de revisão, FIX4): quando o limite
        // dianteiro fica À FRENTE do traseiro (`cg_limit_fwd_pct_mac >
        // cg_limit_aft_pct_mac`), os dois critérios físicos (autoridade de
        // rotação vs. margem estática mínima) são mutuamente
        // incompatíveis com esta célula/trem — NENHUM CG é admissível,
        // independentemente dos cenários de carga observados. Violação
        // DEDICADA (distinta das violações por cenário abaixo) para não
        // deixar essa causa raiz implícita em N mensagens repetidas.
        let envelope_vazio = wb.spec.cg_limit_fwd_pct_mac > wb.spec.cg_limit_aft_pct_mac;
        if envelope_vazio {
            violations.push(format!(
                "Envelope de CG VAZIO: limite dianteiro por rotação ({:.1}% MAC) fica ATRÁS \
                 do limite traseiro de estabilidade ({:.1}% MAC) — nenhum CG é admissível; \
                 causa: trem principal muito atrás do CG (gear.x_main_m). Revisar posição do \
                 trem.",
                wb.spec.cg_limit_fwd_pct_mac, wb.spec.cg_limit_aft_pct_mac
            ));
        }

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

        // 10. Mach de ponta de pá e folga de solo (Task 4.5) — estático,
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

        // 11. Diâmetro derivado × provisório (mitigação pós-revisão da Task
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

        // 12. Gradiente de subida (Task 4.7, CS 23.65): a categoria exige um
        // gradiente mínimo de 8.3% (avaliado em Vx, ao nível do mar, MTOW —
        // ver `agents::performance::best_climb_angle_ms`).
        if perf.climb_gradient_pct < CLIMB_GRADIENT_MIN_PCT {
            violations.push(format!(
                "Gradiente de subida {:.1}% abaixo do mínimo de {:.1}% exigido pela CS 23.65 \
                 (Vx={:.1}km/h)",
                perf.climb_gradient_pct, CLIMB_GRADIENT_MIN_PCT, perf.vx_kmh
            ));
        }

        // 13. Orçamento elétrico — carga CONTÍNUA (Task 5.2): não pode
        // ultrapassar 80% da capacidade do alternador (regra de margem de
        // projeto — reserva para degradação/temperatura/transientes não
        // capturados no orçamento contínuo).
        let limite_continuo_w = electrical.alternator_w * ELECTRICAL_CONTINUOUS_MARGIN_FRAC;
        if electrical.continuous_load_w > limite_continuo_w {
            violations.push(format!(
                "Orçamento elétrico: carga contínua {:.1} W excede 80% da capacidade do \
                 alternador ({:.1} W de {:.1} W nominais)",
                electrical.continuous_load_w, limite_continuo_w, electrical.alternator_w
            ));
        }

        // 14. Orçamento elétrico — carga de PICO (Task 5.2): se o pico
        // "pior caso, tudo ligado" excede a capacidade do alternador, isto é
        // um AVISO, não violação — o banco de baterias da aeronave cobre
        // transientes que o alternador sozinho não sustenta (situação
        // normal em aviação geral: o alternador dimensiona a carga
        // CONTÍNUA, a bateria absorve os picos curtos de retração de
        // trem/flap).
        if electrical.peak_load_w > electrical.alternator_w {
            warnings.push(format!(
                "Orçamento elétrico: carga de pico (pior caso, todas as cargas simultâneas) \
                 {:.1} W excede a capacidade do alternador ({:.1} W) — banco de baterias \
                 deve cobrir o transiente",
                electrical.peak_load_w, electrical.alternator_w
            ));
        }

        // 15. Tipback (Task 2, refino-ciclo2, Raymer cap. 11): o ângulo do
        // trem principal ao CG mais TRASEIRO real dos cenários de carga
        // precisa ficar >= `[gear].tipback_min_deg`, senão a aeronave pode
        // tombar sobre a cauda (carregamento traseiro, empurrão de solo).
        // ACHADO HONESTO conhecido do baseline real: o trem principal foi
        // recuado (Task 1/campanha E1–E6) para abrir o envelope de CG via
        // autoridade de rotação — o preço é um tipback abaixo do piso
        // (~10,1° < 15°). Não é mascarado: é reportado como violação.
        if gear.tipback_angle_deg < gear_cfg.tipback_min_deg {
            violations.push(format!(
                "Tipback: ângulo {:.1}° abaixo do piso de {:.1}° (Raymer cap. 11) — risco de \
                 tombar sobre a cauda com o CG mais traseiro real dos cenários de carga \
                 (trem principal em x_main={:.2}m)",
                gear.tipback_angle_deg, gear_cfg.tipback_min_deg, gear_cfg.x_main_m
            ));
        }

        // 16. Tail-strike (Task 2, refino-ciclo2): a folga angular entre o
        // trem principal e o ponto mais baixo do cone de cauda precisa ser
        // >= `[gear].rotation_attitude_deg` (a atitude de picada nominal na
        // rotação/flare), senão a cauda toca o solo antes da rotação.
        if gear.tail_strike_margin_deg < gear_cfg.rotation_attitude_deg {
            violations.push(format!(
                "Tail-strike: folga angular {:.1}° abaixo da atitude de rotação de {:.1}° — \
                 risco de a cauda tocar o solo na rotação/flare (tail_cone_x_m={:.2}m, \
                 x_main={:.2}m)",
                gear.tail_strike_margin_deg, gear_cfg.rotation_attitude_deg,
                gear_cfg.tail_cone_x_m, gear_cfg.x_main_m
            ));
        }

        // 17. Carga de nariz nos DOIS extremos reais dos cenários de carga
        // (Task 2, refino-ciclo2) — substitui a antiga checagem única (só
        // no CG traseiro). Teto de 25% no CG mais DIANTEIRO; piso de 8% no
        // CG mais TRASEIRO (tração/direção em solo insuficiente abaixo
        // disso).
        const NOSE_LOAD_MAX_CEILING_PCT: f64 = 25.0;
        const NOSE_LOAD_MIN_FLOOR_PCT: f64 = 8.0;
        if gear.nose_load_max_pct > NOSE_LOAD_MAX_CEILING_PCT {
            violations.push(format!(
                "Carga de nariz: {:.1}% no CG mais DIANTEIRO real excede o teto de {:.1}% \
                 (risco de sobrecarga estrutural/pneu do trem de nariz)",
                gear.nose_load_max_pct, NOSE_LOAD_MAX_CEILING_PCT
            ));
        }
        if gear.nose_load_min_pct < NOSE_LOAD_MIN_FLOOR_PCT {
            violations.push(format!(
                "Carga de nariz: {:.1}% no CG mais TRASEIRO real abaixo do piso de {:.1}% \
                 (tração/direção em solo insuficiente)",
                gear.nose_load_min_pct, NOSE_LOAD_MIN_FLOOR_PCT
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
    use crate::agents::electrical::ElectricalAgent;
    use crate::agents::empennage::EmpennageAgent;
    use crate::agents::mission::MissionAgent;
    use crate::agents::performance::PerformanceAgent;
    use crate::agents::propeller::PropellerAgent;
    use crate::agents::propulsion::PropulsionAgent;
    use crate::agents::trim_authority::TrimAuthorityAgent;
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
        -> (Requirements, WingSpec, PropulsionSpec, EngineSpec, WeightBalanceOutput, PropellerSpec, PerformanceSpec, MissionSpec, ElectricalSpec, GearSpec, GearCfg)
    {
        let state  = AircraftState::from_config(&cfg);
        let req    = crate::models::requirements::test_fixtures::requisitos_teste();
        let wing   = AerodynamicsAgent::run(&state, &req);
        let engine = motor_generico_teste();
        let prop   = PropulsionAgent::run(&state, &req, &wing, &engine);
        let emp    = EmpennageAgent::run(&wing, &cfg);
        let mut wb = WeightBalanceAgent::run(&state, &wing, &engine, &cfg, &req, &emp);
        // task trim-authority: finaliza o envelope (inside_envelope/
        // cg_limit_fwd_pct_mac) com o limite dianteiro físico — mesma
        // sequência de `orchestrator::size_aircraft`/`main.rs`, necessária
        // para os testes de envelope de CG abaixo exercitarem o pipeline
        // real (não o placeholder NaN de `WeightBalanceAgent::run` sozinho).
        let trim = TrimAuthorityAgent::run(&cfg, &wing, &emp, &wb);
        wb.apply_trim(&trim);
        let propeller = PropellerAgent::run(&cfg, &engine, &prop, &req);
        let perf   = PerformanceAgent::run(&state, &wing, &prop, state.mtow_kg, &engine, &req,
                                            &cfg.performance);
        let mission = MissionAgent::run(&state, &wing, &prop, &engine, &req, state.mtow_kg)
            .expect("fixture sintética deveria produzir uma missão viável (ver agents::mission)");
        let electrical = ElectricalAgent::run(&cfg);
        // Task 2 (refino-ciclo2): CG mais dianteiro/traseiro REAIS dos
        // cenários de carga (não o limite admissível) — mesma fórmula de
        // `main.rs`.
        let x_cg_fwd = cfg.wing.le_root_x_m + wb.spec.cg_mac_fwd_pct / 100.0 * wb.mac_m;
        let x_cg_aft = cfg.wing.le_root_x_m + wb.spec.cg_mac_aft_pct / 100.0 * wb.mac_m;
        let mass_main_total = cfg.masses.item_mass("trem_principal")
            .expect("item de massa 'trem_principal' ausente na fixture");
        let mass_nose = cfg.masses.item_mass("trem_nariz")
            .expect("item de massa 'trem_nariz' ausente na fixture");
        let gear = crate::agents::landing_gear::LandingGearAgent::run(
            state.mtow_kg, x_cg_fwd, x_cg_aft, &cfg.gear, mass_main_total, mass_nose,
        );
        let gear_cfg = cfg.gear.clone();
        (req, wing, prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg)
    }

    /// Base de todas as asserções de violação isolada abaixo — usa a
    /// fixture sintética padrão (`config_teste()`). Os testes sobrescrevem
    /// apenas os campos relevantes à violação isolada em questão, para não
    /// depender do resultado real dos demais agentes (já testados em seus
    /// próprios módulos).
    fn setup() -> (Requirements, WingSpec, PropulsionSpec, EngineSpec, WeightBalanceOutput, PropellerSpec, PerformanceSpec, MissionSpec, ElectricalSpec, GearSpec, GearCfg) {
        setup_with_cfg(crate::models::aircraft_config::test_fixtures::config_teste())
    }

    #[test]
    fn violacao_cruzeiro_inviavel_aparece_quando_infeasible() {
        let (req, wing, mut prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg) = setup();
        // Força a inviabilidade independentemente do resultado real da busca
        // de rpm, para testar isoladamente a violação #9.
        prop.cruise_feasible = false;
        prop.p_req_cruise_kw = 150.0;
        prop.p_shaft_cruise_kw = 100.0;

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf, &mission, &electrical, &gear, &gear_cfg);

        assert!(report.violations.iter().any(|v| v.contains("Cruzeiro inviável")),
            "esperava violação de cruzeiro inviável, obteve: {:?}", report.violations);
        // A mensagem deve carregar os números reais, não só um rótulo.
        assert!(report.violations.iter().any(|v| v.contains("150.0") && v.contains("100.0")),
            "violação deveria citar P_req/P_shaft: {:?}", report.violations);
    }

    #[test]
    fn sem_violacao_cruzeiro_quando_feasible() {
        let (req, wing, mut prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg) = setup();
        prop.cruise_feasible = true;
        prop.p_req_cruise_kw = 90.0;
        prop.p_shaft_cruise_kw = 100.0;

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf, &mission, &electrical, &gear, &gear_cfg);

        assert!(!report.violations.iter().any(|v| v.contains("Cruzeiro inviável")),
            "não deveria haver violação de cruzeiro inviável, obteve: {:?}", report.violations);
    }

    // ─── Task 4.4: envelope de CG admissível ─────────────────────────────

    /// Com a fixture sintética `config_teste()` (mesmo achado honesto do
    /// baseline real — ver `weight_balance::tests`, `trim_authority::tests`
    /// e task-4.4-report.md/task-1-report.md), os cenários de carga ficam à
    /// frente do limite dianteiro FÍSICO do envelope (TrimAuthorityAgent —
    /// flare/rotação, task trim-authority).
    /// `ConstraintChecker::verify` deve reportar uma violação por cenário
    /// fora do envelope, citando o nome do cenário e os limites em %MAC.
    #[test]
    fn violacao_de_envelope_aparece_quando_cenario_esta_fora() {
        let (req, wing, prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg) = setup();
        assert!(wb.scenarios.iter().any(|s| !s.inside_envelope),
            "pré-condição do teste: fixture sintética deveria ter ao menos um \
             cenário fora do envelope (achado honesto, replicado do baseline real)");

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf, &mission, &electrical, &gear, &gear_cfg);

        assert!(report.violations.iter().any(|v| v.contains("fora do envelope de CG admissível")),
            "esperava violação de envelope de CG, obteve: {:?}", report.violations);
        // A mensagem deve citar os limites do envelope em %MAC, não só um rótulo.
        assert!(report.violations.iter().any(|v|
                v.contains(&format!("{:.1}%", wb.spec.cg_limit_fwd_pct_mac))),
            "violação deveria citar o limite dianteiro do envelope: {:?}", report.violations);
    }

    /// Fix de revisão (FIX4): o baseline real tem envelope de CG VAZIO —
    /// o limite de rotação (invariante ao peso, ≈39,9% MAC) fica À FRENTE
    /// do limite traseiro de estabilidade (≈36,6% MAC), então os dois
    /// critérios físicos são mutuamente incompatíveis com esta
    /// Campanha E1–E6 (2026-08-05): o baseline real fecha o envelope de CG
    /// (trem principal recuado, EH maior, bateria/bagageiro realocados —
    /// ver comentários em `config/aircraft/baseline_4seat.toml`). Achado
    /// honesto ANTERIOR (pré-E6): `violacao_de_envelope_vazio_aparece_no_
    /// baseline_real` — o baseline tinha o limite dianteiro (rotação,
    /// ≈39,9% MAC) À FRENTE do limite traseiro (≈36,6% MAC), envelope
    /// vazio. Após a E6, `ConstraintChecker::verify` NÃO deve reportar
    /// nenhuma violação de envelope — nem a dedicada "VAZIO", nem por
    /// cenário. O caminho de erro (envelope vazio) continua coberto por
    /// `violacao_de_envelope_vazio_aparece_com_baseline_mutado_x_main_antigo`
    /// logo abaixo, que reproduz o `gear.x_main_m` pré-E6 (3.85m, causa
    /// raiz original) em uma cópia mutada da config real.
    #[test]
    fn envelope_de_cg_fechado_sem_violacao_no_baseline_real() {
        let toml = std::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("config/aircraft/baseline_4seat.toml"),
        ).expect("falha ao ler baseline_4seat.toml do disco");
        let cfg = crate::models::config::parse_aircraft(&toml)
            .expect("baseline real deveria ser uma configuração válida");
        let (req, wing, prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg) = setup_with_cfg(cfg);
        assert!(wb.spec.cg_limit_fwd_pct_mac <= wb.spec.cg_limit_aft_pct_mac,
            "pré-condição do teste: baseline real (pós E1–E6) deveria ter envelope de CG \
             FECHADO (fwd={:.2}% <= aft={:.2}%)", wb.spec.cg_limit_fwd_pct_mac,
            wb.spec.cg_limit_aft_pct_mac);

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf, &mission, &electrical, &gear, &gear_cfg);

        assert!(!report.violations.iter().any(|v| v.contains("Envelope de CG VAZIO")),
            "não deveria haver violação dedicada de envelope vazio no baseline pós-E6: {:?}",
            report.violations);
        assert!(!report.violations.iter().any(|v| v.contains("fora do envelope de CG admissível")),
            "não deveria haver violações de envelope de CG por cenário no baseline pós-E6: {:?}",
            report.violations);
    }

    /// Caminho de erro preservado (achado histórico pré-E6, Task 4.4/
    /// trim-authority): parte da config REAL do disco (já com todas as
    /// demais mudanças da E6) e reverte, em uma cópia mutada em código, os
    /// TRÊS parâmetros que juntos fecham o envelope (`gear.x_main_m`,
    /// `empennage.v_h`, `control_surfaces.elevator_chord_frac` — desde a
    /// task refino-ciclo2, `[stability].cl_h_max_down` não existe mais
    /// como campo direto; a corda de profundor reduzida reproduz uma
    /// autoridade de download equivalente ao antigo palpite pré-E6) ao
    /// valor ANTIGO pré-E6. Reverter só `gear.x_main_m` (a causa raiz citada na
    /// violação) NÃO basta mais para reproduzir o envelope vazio — os
    /// outros ganhos de autoridade/estabilidade da E6 (EH maior, mais
    /// download do profundor) compensam sozinhos o trem principal antigo
    /// (checado experimentalmente: com só x_main_m revertido, fwd=29.4% <
    /// aft=43.5%, envelope ainda fechado; com x_main_m+v_h revertidos,
    /// fwd=36.5% < aft=36.6%, quase fechado mas ainda dentro). Com os três
    /// parâmetros revertidos juntos, confirma-se que
    /// `ConstraintChecker::verify` ainda detecta e reporta corretamente um
    /// envelope de CG vazio quando ele ocorre — a violação DEDICADA
    /// "Envelope de CG VAZIO", citando os dois limites e a causa raiz
    /// (`gear.x_main_m`), ALÉM das violações por cenário.
    #[test]
    fn violacao_de_envelope_vazio_aparece_com_baseline_mutado_parametros_pre_e6() {
        let toml = std::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("config/aircraft/baseline_4seat.toml"),
        ).expect("falha ao ler baseline_4seat.toml do disco");
        let mut cfg = crate::models::config::parse_aircraft(&toml)
            .expect("baseline real deveria ser uma configuração válida");
        cfg.gear.x_main_m = 3.85;                       // valor pré-E6 — causa raiz original
        cfg.empennage.v_h = 0.70;                       // valor pré-E6
        // Task refino-ciclo2: `[stability].cl_h_max_down` foi REMOVIDO — o
        // equivalente é reduzir `elevator_chord_frac` (0.40→0.30, τ menor)
        // para reproduzir uma autoridade de download reduzida (≈0.88, perto
        // do palpite antigo 0.85) — ver comentário equivalente em
        // `agents::trim_authority::tests::trim_authority_agent_run_hand_
        // check_baseline_mutado_parametros_pre_e6`.
        cfg.control_surfaces.elevator_chord_frac = 0.30;
        let (req, wing, prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg) = setup_with_cfg(cfg);
        assert!(wb.spec.cg_limit_fwd_pct_mac > wb.spec.cg_limit_aft_pct_mac,
            "pré-condição do teste: parâmetros pré-E6 (x_main_m=3.85, v_h=0.70, \
             elevator_chord_frac=0.30, config real mutada) deveriam reproduzir o envelope de \
             CG vazio original (fwd={:.2}% > aft={:.2}%)",
            wb.spec.cg_limit_fwd_pct_mac, wb.spec.cg_limit_aft_pct_mac);

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf, &mission, &electrical, &gear, &gear_cfg);

        assert!(report.violations.iter().any(|v| v.contains("Envelope de CG VAZIO")),
            "esperava violação dedicada de envelope vazio, obteve: {:?}", report.violations);
        assert!(report.violations.iter().any(|v| v.contains("gear.x_main_m")),
            "violação de envelope vazio deveria citar a causa raiz (gear.x_main_m): {:?}",
            report.violations);
        // Ainda deve haver violações POR CENÁRIO também (não substitui,
        // complementa).
        assert!(report.violations.iter().any(|v| v.contains("fora do envelope de CG admissível")),
            "violações por cenário deveriam continuar presentes ao lado da dedicada: {:?}",
            report.violations);
    }

    /// Sanidade inversa: se TODOS os cenários estiverem artificialmente
    /// dentro do envelope, nenhuma violação POR CENÁRIO deve aparecer —
    /// mas a violação DEDICADA de envelope vazio (se o `wb.spec` sintético
    /// da fixture também tiver fwd>aft) é independente do override de
    /// `inside_envelope` por cenário, então não é coberta por este teste
    /// (ver `violacao_de_envelope_vazio_aparece_no_baseline_real` acima).
    #[test]
    fn sem_violacao_de_envelope_quando_todos_os_cenarios_estao_dentro() {
        let (req, wing, prop, engine, mut wb, propeller, perf, mission, electrical, gear, gear_cfg) = setup();
        for sc in &mut wb.scenarios {
            sc.inside_envelope = true;
        }

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf, &mission, &electrical, &gear, &gear_cfg);

        assert!(!report.violations.iter().any(|v| v.contains("fora do envelope de CG admissível")),
            "não deveria haver violação de envelope de CG, obteve: {:?}", report.violations);
    }

    // ─── Task 4.5: hélice (Mach de ponta / folga de solo) ──────────────────

    #[test]
    fn violacao_de_helice_aparece_quando_algum_ok_e_falso() {
        let (req, wing, prop, engine, wb, mut propeller, perf, mission, electrical, gear, gear_cfg) = setup();
        propeller.ok_mach_static = false;
        propeller.tip_mach_static = 0.99;

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf, &mission, &electrical, &gear, &gear_cfg);

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
        let (req, wing, prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg) = setup();
        assert!(!propeller.ok_clearance,
            "pré-condição do teste: fixture sintética (shaft_height_m=1.15, diameter=1.90, \
             ground_clearance_min_m=0.25) deveria falhar na folga de solo — obtido \
             ground_clearance_m={:.3}", propeller.ground_clearance_m);

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf, &mission, &electrical, &gear, &gear_cfg);

        assert!(report.violations.iter().any(|v| v.contains("folga de solo")),
            "esperava violação de folga de solo, obteve: {:?}", report.violations);
    }

    /// Sanidade inversa: se TODOS os `ok_*` estiverem artificialmente
    /// verdadeiros (mesmo padrão de `sem_violacao_de_envelope_...` acima),
    /// nenhuma violação de hélice deve aparecer.
    #[test]
    fn sem_violacao_de_helice_quando_todos_ok_forcado() {
        let (req, wing, prop, engine, wb, mut propeller, perf, mission, electrical, gear, gear_cfg) = setup();
        propeller.ok_mach_static = true;
        propeller.ok_mach_cruise = true;
        propeller.ok_clearance = true;

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf, &mission, &electrical, &gear, &gear_cfg);

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

        let (req, wing, prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg) = setup_with_cfg(cfg);
        assert_eq!(propeller.source, "derivado");
        println!(
            "pipeline real (Mach governa): D_autoritativo={:.4} D_provisorio(prop.prop_diameter_m)={:.4}",
            propeller.diameter_m, prop.prop_diameter_m
        );
        assert!((propeller.diameter_m - prop.prop_diameter_m).abs() > 0.01,
            "pré-condição do teste: com PSRU 1:1 o diâmetro autoritativo ({:.4}) deveria \
             divergir do provisório ({:.4}) usado pela busca de cruzeiro",
            propeller.diameter_m, prop.prop_diameter_m);

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf, &mission, &electrical, &gear, &gear_cfg);

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

        let (req, wing, prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg) = setup_with_cfg(cfg);
        assert_eq!(propeller.source, "derivado");
        println!(
            "pipeline real (folga governa): D_autoritativo={:.4} D_provisorio(prop.prop_diameter_m)={:.4}",
            propeller.diameter_m, prop.prop_diameter_m
        );

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf, &mission, &electrical, &gear, &gear_cfg);

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
        let (req, wing, prop, engine, wb, propeller, _perf, mission, electrical, gear, gear_cfg) = setup();
        let perf = performance_spec_com_gradiente(6.0);

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear, &gear_cfg);

        assert!(report.violations.iter().any(|v| v.contains("Gradiente de subida")),
            "esperava violação de gradiente de subida, obteve: {:?}", report.violations);
        // A mensagem deve citar o gradiente observado e o piso da CS 23.65.
        assert!(report.violations.iter().any(|v| v.contains("6.0") && v.contains("8.3")),
            "violação deveria citar o gradiente observado (6.0%) e o mínimo CS 23.65 \
             (8.3%): {:?}", report.violations);
    }

    #[test]
    fn sem_violacao_de_gradiente_quando_maior_ou_igual_a_8_3_por_cento() {
        let (req, wing, prop, engine, wb, propeller, _perf, mission, electrical, gear, gear_cfg) = setup();
        let perf = performance_spec_com_gradiente(9.0);

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear, &gear_cfg);

        assert!(!report.violations.iter().any(|v| v.contains("Gradiente de subida")),
            "não deveria haver violação de gradiente quando climb_gradient_pct (9.0%) >= \
             8.3%, obteve: {:?}", report.violations);
    }

    // ─── Task 5.2: orçamento elétrico ────────────────────────────────────

    /// `ElectricalSpec` sintética mínima para os testes de violação/aviso
    /// elétrico abaixo — construída à mão (não via `ElectricalAgent`) para
    /// isolar exatamente as checagens #14/#15 de `ConstraintChecker::verify`.
    fn electrical_spec(alternator_w: f64, continuous_load_w: f64, peak_load_w: f64) -> ElectricalSpec {
        ElectricalSpec {
            bus_voltage_v: 28.0,
            alternator_w,
            continuous_load_w,
            peak_load_w,
            margin_continuous_pct: (alternator_w - continuous_load_w) / alternator_w * 100.0,
        }
    }

    #[test]
    fn violacao_eletrica_aparece_quando_carga_continua_excede_80pct_do_alternador() {
        let (req, wing, prop, engine, wb, propeller, perf, mission, _electrical, gear, gear_cfg) = setup();
        // 900 W de alternador, 80% = 720 W — 750 W de carga contínua excede.
        let electrical = electrical_spec(900.0, 750.0, 750.0);

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear, &gear_cfg);

        assert!(report.violations.iter().any(|v| v.contains("carga contínua")),
            "esperava violação de carga elétrica contínua, obteve: {:?}", report.violations);
        assert!(report.violations.iter().any(|v| v.contains("750.0") && v.contains("720.0")),
            "violação deveria citar a carga observada (750.0 W) e o limite de 80% (720.0 W): \
             {:?}", report.violations);
    }

    #[test]
    fn sem_violacao_eletrica_quando_carga_continua_dentro_de_80pct() {
        let (req, wing, prop, engine, wb, propeller, perf, mission, _electrical, gear, gear_cfg) = setup();
        // 900 W de alternador, 80% = 720 W — 430 W (baseline real) fica bem dentro.
        let electrical = electrical_spec(900.0, 430.0, 1_260.0);

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear, &gear_cfg);

        assert!(!report.violations.iter().any(|v| v.contains("carga contínua")),
            "não deveria haver violação de carga contínua, obteve: {:?}", report.violations);
    }

    #[test]
    fn aviso_eletrico_aparece_quando_pico_excede_o_alternador() {
        let (req, wing, prop, engine, wb, propeller, perf, mission, _electrical, gear, gear_cfg) = setup();
        // Pico pior-caso do baseline real (1.260 W) > alternador (900 W).
        let electrical = electrical_spec(900.0, 430.0, 1_260.0);

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear, &gear_cfg);

        assert!(report.warnings.iter().any(|w| w.contains("carga de pico")),
            "esperava aviso de pico elétrico, obteve: {:?}", report.warnings);
        // É AVISO, não violação — banco de baterias cobre o transiente.
        assert!(!report.violations.iter().any(|v| v.contains("pico")),
            "excesso de pico deveria ser aviso, não violação: {:?}", report.violations);
    }

    #[test]
    fn sem_aviso_eletrico_quando_pico_dentro_da_capacidade_do_alternador() {
        let (req, wing, prop, engine, wb, propeller, perf, mission, _electrical, gear, gear_cfg) = setup();
        let electrical = electrical_spec(900.0, 430.0, 800.0);

        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear, &gear_cfg);

        assert!(!report.warnings.iter().any(|w| w.contains("carga de pico")),
            "não deveria haver aviso de pico elétrico quando peak_load_w (800 W) <= \
             alternator_w (900 W), obteve: {:?}", report.warnings);
    }

    /// Regressão contra o baseline REAL (via `ElectricalAgent::run`, não a
    /// spec sintética acima): confirma que os números hand-checados do
    /// controller (contínuo 430 W, margem ~52,2%, pico 1.260 W > 900 W)
    /// disparam exatamente o padrão esperado — sem violação de carga
    /// contínua, com aviso de pico.
    #[test]
    fn orcamento_eletrico_do_baseline_real_dispara_so_o_aviso_de_pico() {
        let toml = std::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("config/aircraft/baseline_4seat.toml"),
        ).expect("falha ao ler baseline_4seat.toml do disco");
        let cfg = crate::models::config::parse_aircraft(&toml)
            .expect("baseline real deveria ser uma configuração válida");
        let electrical_real = crate::agents::electrical::ElectricalAgent::run(&cfg);

        let (req, wing, prop, engine, wb, propeller, perf, mission, _electrical, gear, gear_cfg) = setup();
        let report = ConstraintChecker::verify(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical_real, &gear, &gear_cfg);

        assert!(!report.violations.iter().any(|v| v.contains("carga contínua")),
            "baseline real (430 W contínuo / 900 W alternador, ~52,2% de margem) não deveria \
             violar o limite de 80%: {:?}", report.violations);
        assert!(report.warnings.iter().any(|w| w.contains("carga de pico")),
            "baseline real (pico 1.260 W > alternador 900 W) deveria disparar o aviso de \
             pico: {:?}", report.warnings);
    }
}
