/// ConstraintChecker — verifica se os resultados dos agentes satisfazem
/// os requisitos do projeto e reporta violações com detalhamento.

use crate::agents::weight_balance::WeightBalanceOutput;
use crate::models::{
    aircraft_config::{GearCfg, PropellerCfg}, engine::EngineSpec, requirements::Requirements,
    specs::{WingSpec, PropulsionSpec, PropellerSpec, PerformanceSpec, MissionSpec, ElectricalSpec, GearSpec, RobustnessSpec},
};

/// Gradiente de subida mínimo exigido pela CS 23.65 para esta categoria (%).
const CLIMB_GRADIENT_MIN_PCT: f64 = 8.3;

/// Fração da capacidade do alternador reservada como margem — a carga
/// CONTÍNUA não pode ultrapassar 80% da capacidade nominal (regra de
/// projeto elétrico comum em aviação geral: reserva 20% para degradação
/// do alternador ao longo da vida útil, temperatura alta e cargas
/// transientes não capturadas no orçamento contínuo). Task 5.2.
const ELECTRICAL_CONTINUOUS_MARGIN_FRAC: f64 = 0.80;

/// Teto de carga de nariz no CG mais DIANTEIRO real (%) — checagem #17.
/// `pub` (ciclo 4, task robustez) para fonte única: `validation::robustness`
/// avalia os conjuntos adversariais contra o MESMO teto usado aqui.
pub const NOSE_LOAD_MAX_CEILING_PCT: f64 = 25.0;
/// Piso de carga de nariz no CG mais TRASEIRO real (%) — checagem #17.
/// `pub` pelo mesmo motivo de `NOSE_LOAD_MAX_CEILING_PCT`.
pub const NOSE_LOAD_MIN_FLOOR_PCT: f64 = 8.0;

/// Razão de subida mínima ao nível do mar, MTOW (m/s) — checagem #21.
/// `pub` (achado de review, ciclo 5): antes deste fix, este piso era
/// hardcoded independentemente em DOIS lugares que podiam divergir —
/// `main.rs` (só IMPRIMIA o gate `rc_ok`, sem alimentar
/// `ConstraintChecker::verify`) e `validation::robustness` (caso
/// "massa-total", comparando o mundo +σ contra o MESMO número por
/// coincidência de literal, não por fonte única). Fonte única agora.
pub const RC_SL_MIN_MS: f64 = 1.5;
/// Teto de serviço mínimo, MTOW (m) — checagem #22. `pub` pelo mesmo motivo
/// de `RC_SL_MIN_MS`.
pub const SERVICE_CEILING_MIN_M: f64 = 3_000.0;

/// Nome da carga elétrica declarada em `[[electrical.loads]]` para o
/// atuador de retração do trem — checagem #20. `pub` para fonte única entre
/// a checagem e os testes que a exercitam (achado de review, ciclo 5: antes
/// era um literal `"trem_retratil"` repetido em produção e em testes).
pub const GEAR_ACTUATOR_LOAD_NAME: &str = "trem_retratil";

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

/// Entradas do veredito global (ciclo 6) — struct de parâmetros no lugar
/// dos 15 posicionais que três ciclos seguidos incharam. Todas as
/// referências vêm do pipeline convergido (mesmos valores de antes).
pub struct VerifyInputs<'a> {
    pub req: &'a Requirements,
    pub wing: &'a WingSpec,
    pub prop: &'a PropulsionSpec,
    pub mtow_kg: f64,
    pub engine: &'a EngineSpec,
    pub wb: &'a WeightBalanceOutput,
    pub propeller: &'a PropellerSpec,
    pub perf: &'a PerformanceSpec,
    pub mission: &'a MissionSpec,
    pub electrical: &'a ElectricalSpec,
    pub gear: &'a GearSpec,
    pub gear_cfg: &'a GearCfg,
    pub fuel_capacity_l: f64,
    pub robustness: &'a RobustnessSpec,
    /// `[propeller]` (ciclo 9, transferência de atitude do #25) — só
    /// `prop_plane_x_m` é consumido aqui, para o `debug_assert!`/mensagem
    /// de violação do check #25 recomputarem a MESMA fórmula fechada de
    /// `PropellerSpec::fill_critical_clearance` (fonte única preservada).
    pub prop_cfg: &'a PropellerCfg,
}

pub struct ConstraintChecker;

impl ConstraintChecker {
    pub fn verify(inputs: &VerifyInputs) -> ConstraintReport {
        let req = inputs.req;
        let wing = inputs.wing;
        let prop = inputs.prop;
        let mtow_kg = inputs.mtow_kg;
        let engine = inputs.engine;
        let wb = inputs.wb;
        let propeller = inputs.propeller;
        let perf = inputs.perf;
        let mission = inputs.mission;
        let electrical = inputs.electrical;
        let gear = inputs.gear;
        let gear_cfg = inputs.gear_cfg;
        let fuel_capacity_l = inputs.fuel_capacity_l;
        let robustness = inputs.robustness;
        let prop_cfg = inputs.prop_cfg;

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
        // cenários; desde o ciclo 10 task 2 é a ENVOLTÓRIA conservadora
        // avaliada no cenário mais leve, não mais uma invariância
        // algébrica — ver `agents::trim_authority::rotation_fwd_limit_m`),
        // cg_limit_aft_pct_mac (sm_min)], não apenas os
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
        // HISTÓRICO: no baseline E6 o trem principal recuado (Task
        // 1/campanha E1–E6) para abrir o envelope de CG via autoridade de
        // rotação deixava um tipback abaixo do piso (~10,1° < 15°) —
        // reportado honestamente como violação, não mascarado. RESOLVIDO na
        // campanha E7 (2026-08-06): `[gear].x_main_m` 3,55→3,66m fecha o
        // tipback (~15,6° ≥ 15°) mantendo a carga de nariz dentro do teto
        // (ver `config/aircraft/baseline_4seat.toml`). Gate continua ativo —
        // cobertura do caminho de violação preservada por config sintética
        // mutada (ver `violacao_de_tipback_aparece_quando_abaixo_do_piso`
        // abaixo, mesmo padrão da checagem #18).
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
        // disso). Constantes promovidas a `pub const` de módulo (ciclo 4,
        // task robustez) — `validation::robustness` reexporta as MESMAS
        // constantes para avaliar os conjuntos adversariais contra os
        // mesmos tetos/pisos, fonte única.
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

        // 18. Margem mínima de combustível (Task 3, refino-ciclo2): a folga
        // entre a capacidade do tanque e o combustível exigido pela missão
        // no ponto de MTOW convergido precisa ser >=
        // `req.min_fuel_margin_fraction`. CONVENÇÃO (padronizada nesta
        // task): a margem aqui é fração/percentual da CAPACIDADE do tanque
        // — a MESMA convenção de `sizing.fuel_margin_pct`
        // (`fuel_margin_l / fuel_capacity_l · 100`, ver `main.rs`), NÃO a
        // convenção de %-do-combustível-NECESSÁRIO usada por
        // `tests/generic_engine.rs::margem_de_combustivel_no_mtow_
        // convergido` (que mede a mesma folga com outro denominador — ver a
        // nota de convenção nesse teste). Achado honesto conhecido do
        // baseline real (missão de projeto completa): margem ≈1,82% da
        // capacidade — abaixo do piso de 5% — não mascarado por ajuste de
        // tanque/missão (decisão de projeto fica para revisão humana).
        let fuel_margin_pct = (fuel_capacity_l - mission.fuel_total_l) / fuel_capacity_l * 100.0;
        let fuel_margin_min_pct = req.min_fuel_margin_fraction * 100.0;
        if fuel_margin_pct < fuel_margin_min_pct {
            violations.push(format!(
                "Margem de combustível: {:.2}% da capacidade do tanque abaixo do mínimo de \
                 {:.1}% (combustível exigido pela missão {:.1} L, capacidade do tanque {:.1} L, \
                 margem {:.1} L)",
                fuel_margin_pct, fuel_margin_min_pct,
                mission.fuel_total_l, fuel_capacity_l, fuel_capacity_l - mission.fuel_total_l
            ));
        }

        // 19. Robustez à incerteza do modelo de massas (ciclo 4, task
        // robustez — `validation::robustness::RobustnessAgent`): as 7
        // massas estruturais são equações empíricas de componente (Raymer
        // cap. 15.2) ajustadas a uma FROTA histórica, com incerteza típica
        // de projeto conceitual de ±10-20% (Raymer/Roskam Classe II) — um
        // check que passa no NOMINAL mas reprova sob um dos dois conjuntos
        // adversariais determinísticos (±σ, `RobustnessSpec::flips`) é uma
        // violação nomeada, não um aviso: o veredito nominal sozinho não é
        // confiável o bastante para essas 7 massas. Um `violations.push`
        // por flip (zero flips ⇒ zero violações desta checagem).
        for flip in &robustness.flips {
            violations.push(format!(
                "Robustez: {} passa no nominal mas reprova com massas estruturais ±{:.1}% \
                 (pior caso {}): {:.2} vs {:.2}",
                flip.check, robustness.sigma_mass_fraction * 100.0, flip.caso,
                flip.valor, flip.limite
            ));
        }

        // 20 — atuador de retração vs orçamento elétrico (ciclo 5): o pico
        // DECLARADO da carga 'trem_retratil' deve cobrir a potência
        // COMPUTADA do atuador (LandingGearAgent). Substitui a guarda de
        // parse removida no ciclo 3 (`models::config::validate_aircraft` —
        // a massa da perna virou computada) — a checagem só é possível
        // PÓS-convergência, aqui.
        //
        // Gate de retrátil (achado de review, ciclo 5, Minor 5): uma
        // aeronave de trem FIXO não tem atuador de retração elétrico
        // (`gear.actuator_power_w` não se aplica) — exigir a carga
        // 'trem_retratil' nesse caso seria um falso positivo. A checagem só
        // se aplica quando `[gear].retractable = true`.
        if gear_cfg.retractable {
            match electrical.loads.iter().find(|l| l.name == GEAR_ACTUATOR_LOAD_NAME) {
                None => violations.push(format!(
                    "Carga '{GEAR_ACTUATOR_LOAD_NAME}' ausente do orçamento elétrico — aeronave \
                     de trem retrátil precisa declarar o pico do atuador em \
                     [[electrical.loads]]"
                )),
                Some(l) if l.peak_w < gear.actuator_power_w => violations.push(format!(
                    "Atuador de retração: pico declarado em [[electrical.loads]] \
                     '{GEAR_ACTUATOR_LOAD_NAME}' ({:.1} W) menor que a potência computada do \
                     atuador ({:.1} W) — orçamento elétrico subdimensionado",
                    l.peak_w, gear.actuator_power_w
                )),
                Some(_) => {}
            }
        }

        // 21. Razão de subida ao nível do mar, MTOW (achado de review, ciclo
        // 5, Important 1): `main.rs` já IMPRIMIA este piso (`rc_ok`) desde
        // antes deste fix, mas `ConstraintChecker::verify` nunca o CHECAVA
        // nominalmente — só a checagem #19 (robustez) comparava um mundo
        // perturbado contra ele, indiretamente. Isso produzia um veredito
        // NÃO-MONOTÔNICO: uma aeronave nominal rc=1.6 m/s cujo perturbado
        // caísse a 1.4 m/s REPROVARIA (#19, "passa no nominal mas reprova
        // perturbado"), enquanto uma aeronave nominal rc=1.4 m/s (JÁ abaixo
        // do piso) PASSARIA (nenhuma checagem de `report.violations` olhava
        // para `rc_sl_ms` diretamente) — pior nominal, melhor veredito.
        // Fecha essa lacuna: o piso nominal agora é verificado aqui.
        if perf.rc_sl_ms < RC_SL_MIN_MS {
            violations.push(format!(
                "Razão de subida ao nível do mar (MTOW) {:.2} m/s abaixo do mínimo de {:.1} m/s",
                perf.rc_sl_ms, RC_SL_MIN_MS
            ));
        }

        // 22. Teto de serviço, MTOW (achado de review, ciclo 5, Important
        // 1) — mesmo raciocínio de monotonicidade da checagem #21 acima.
        if perf.service_ceiling_m < SERVICE_CEILING_MIN_M {
            violations.push(format!(
                "Teto de serviço (MTOW) {:.0} m abaixo do mínimo de {:.0} m",
                perf.service_ceiling_m, SERVICE_CEILING_MIN_M
            ));
        }

        // 23. Decolagem na GRAMA sobre obstáculo de 15 m dentro da pista
        // disponível (Ciclo 6, task 2) — premissa fundadora do projeto:
        // operação em pista de terra/grama (`req.runway_available_m`,
        // `config/missions/default.toml`), não pavimentada.
        //
        // Semântica INCLUSIVA deliberada nos dois checks (#23/#24): o
        // comparador é `>`, logo distância EXATAMENTE igual à pista
        // disponível PASSA. Consistente com `ok_clearance` da hélice
        // (`agents::propeller`, `>=`) e com os demais pisos/tetos deste
        // arquivo — o limite pertence à região aceitável, e a margem
        // operacional real é responsabilidade do valor configurado em
        // `runway_available_m`, não de uma folga implícita no operador.
        if perf.to_50ft_grass_m > req.runway_available_m {
            violations.push(format!(
                "Decolagem (grama, 15 m): {:.0} m excede a pista disponível de {:.0} m",
                perf.to_50ft_grass_m, req.runway_available_m
            ));
        }
        // 24. Pouso na GRAMA sobre obstáculo de 15 m dentro da pista
        // disponível (Ciclo 6, task 2; superfície corrigida na revisão
        // final do mesmo ciclo) — mesmo raciocínio da checagem #23 acima,
        // inclusive a semântica inclusiva. Usa `ldg_50ft_grass_m`
        // (`mu_brake_grass`), NÃO `ldg_50ft_m` (pavimentado): gatear uma
        // pista de grama com a distância de pouso pavimentada era otimista
        // por construção — a frenagem pior da grama alonga a rolagem, e é
        // esse o caso dimensionante da premissa de pista. O pavimentado
        // permanece no spec como informativo.
        if perf.ldg_50ft_grass_m > req.runway_available_m {
            violations.push(format!(
                "Pouso (grama, 15 m): {:.0} m excede a pista disponível de {:.0} m",
                perf.ldg_50ft_grass_m, req.runway_available_m
            ));
        }

        // 25. Folga de hélice em condição CRÍTICA (CS 23.925) — ciclo 8,
        // task 2: até esta task, a única folga de hélice checada era a
        // ESTÁTICA (`ok_clearance` acima, check #10) — amortecedores
        // estendidos, pneus cheios. CS 23.925 também exige folga positiva
        // na condição CRÍTICA: amortecedor TOTALMENTE COMPRIMIDO (batente)
        // + pneu MURCHO/estourado. Hélice TRATORA: é o trem de NARIZ que
        // governa o TERMO AMPLIFICADO (fica sob o eixo da hélice,
        // dianteiro) — daí `gear.nose_oleo_stroke_mm` × `fator`, não
        // `main_oleo_stroke_mm`, alimentando esse termo. O trem PRINCIPAL
        // NUNCA precisou de termo aditivo aqui — leitura da norma pela
        // LETRA (ciclo 10, task 1): a condição crítica de CS 23.925 coloca
        // só o trem CRÍTICO (nariz, hélice tratora) no batente; os DEMAIS
        // (aqui, o principal) permanecem na deflexão ESTÁTICA normal, já
        // embutida em `gear_cfg.h_cg_ground_m`/`propeller.ground_clearance_m`
        // (a aeronave é sempre modelada CARREGADA — ver docstring de
        // `GearCfg::h_cg_ground_m`). CAVEAT DOS MAINS RÍGIDOS do ciclo 9
        // (deflexão do amortecedor/pneu principal precisaria entrar como
        // termo aditivo, condição COMPOSTA de CS 23.925) MORREU nesta task
        // — não faltava termo nenhum, ver `docs/backlog.md` (item 6,
        // RESOLVIDO). Lê `propeller.prop_clearance_critical_m` já
        // PRECOMPUTADO (ver `specs::PropellerSpec::fill_critical_clearance`,
        // chamado em `main.rs`/nas fixtures de teste logo após o trem de
        // pouso) — os termos abaixo (`ground_clearance_m`/
        // `nose_oleo_stroke_mm`/`static_sag_fraction`/
        // `tire_deflation_delta_m`/`fator`) só narram a mensagem, não
        // recalculam o resultado (fonte única).
        //
        // Ciclo 9 (transferência de atitude do #25): `fator` modela o PIVÔ
        // da célula sobre o trem PRINCIPAL (não mais uma translação
        // vertical 1:1 do nariz) — a hélice, à frente do trem de nariz,
        // mergulha um braço amplificado por
        // `(x_main−prop_plane_x_m)/(x_main−x_nose_m)` sobre o curso do
        // nariz/deflexão de pneu.
        //
        // Ciclo 10, task 1 (deflexão estática): o curso do nariz que entra
        // no termo amplificado é o curso RESTANTE até o batente, não o
        // curso TOTAL — `nose_oleo_stroke_mm × (1 − static_sag_fraction)`,
        // porque o amortecedor de nariz também PARTE da deflexão estática
        // (a mesma que `h_cg_ground_m` já modela para os mains), não
        // estendido. A fórmula do ciclo 9 contava essa compressão estática
        // do nariz DUAS VEZES; a correção reduz `Δ_prop` e AUMENTA a folga
        // crítica (honestamente ANTI-conservadora frente ao número antigo,
        // mas fiel à letra da norma). Ver docstring de
        // `PropellerSpec::prop_clearance_critical_m` para a física completa
        // e o old→new.
        //
        // Guarda de build debug (achado de review, ciclo 8): se algum
        // caminho novo esquecer de chamar `fill_critical_clearance` depois
        // do trem de pouso, `prop_clearance_critical_m` fica preso no
        // placeholder `0.0` (ver docstring do campo) — o que a checagem
        // acima trataria como VIOLAÇÃO (`0.0 <= 0.0`), não como omissão
        // silenciosa. Ainda assim, uma omissão pode por acaso produzir um
        // `prop_clearance_critical_m` positivo herdado de um `PropellerSpec`
        // reaproveitado de outra chamada — o `debug_assert!` abaixo fecha
        // essa lacuna comparando o campo PRECOMPUTADO contra a mesma fórmula
        // fechada que a mensagem já narra, para que a omissão GRITE em teste
        // (`panic!` em debug) em vez de mascarar silenciosamente atrás de um
        // valor coincidentemente plausível. Não recalcula o resultado usado
        // pelo gate (fonte única preservada) — só valida a invariante.
        let fator = (gear_cfg.x_main_m - prop_cfg.prop_plane_x_m)
            / (gear_cfg.x_main_m - gear_cfg.x_nose_m);
        let curso_restante_nariz_m = (gear.nose_oleo_stroke_mm / 1_000.0)
            * (1.0 - gear_cfg.static_sag_fraction);
        let delta_prop = (curso_restante_nariz_m + gear_cfg.tire_deflation_delta_m) * fator;
        debug_assert!(
            (propeller.prop_clearance_critical_m - (propeller.ground_clearance_m - delta_prop))
                .abs() < 1e-9,
            "propeller.prop_clearance_critical_m ({:.6}) não bate com a fórmula fechada \
             ground_clearance_m − (nose_oleo_stroke_mm/1000×(1−static_sag_fraction) + \
             tire_deflation_delta_m)×fator (fator={:.5}) ({:.6}) — \
             `PropellerSpec::fill_critical_clearance` foi chamado?",
            propeller.prop_clearance_critical_m, fator,
            propeller.ground_clearance_m - delta_prop
        );
        if propeller.prop_clearance_critical_m <= 0.0 {
            violations.push(format!(
                "Hélice (condição crítica CS 23.925, pivô nos mains): folga estática {:.3} m − \
                 Δ_prop {:.3} m (fator {:.4}× sobre curso RESTANTE do nariz {:.3} m + pneu murcho \
                 {:.3} m) = {:.3} m ≤ 0",
                propeller.ground_clearance_m, delta_prop, fator,
                curso_restante_nariz_m, gear_cfg.tire_deflation_delta_m,
                propeller.prop_clearance_critical_m
            ));
        }

        ConstraintReport { violations, warnings }
    }
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::agents::electrical::ElectricalAgent;
    use crate::agents::performance::PerformanceAgent;
    use crate::agents::propeller::PropellerAgent;
    use crate::models::engine::test_fixtures::motor_generico_teste;
    use crate::models::specs::{ElectricalLoadSpec, RobustnessFlip};

    /// Helper mecânico (ciclo 6, task 1): monta `VerifyInputs` a partir dos
    /// mesmos 14 parâmetros posicionais que `ConstraintChecker::verify`
    /// recebia antes desta refatoração — evita repetir a construção da
    /// struct em ~24 chamadas de teste, mantendo cada teste livre para
    /// sobrescrever campos individuais (basta mutar a variável
    /// correspondente antes de chamar este helper).
    fn inputs<'a>(
        req: &'a Requirements,
        wing: &'a WingSpec,
        prop: &'a PropulsionSpec,
        mtow_kg: f64,
        engine: &'a EngineSpec,
        wb: &'a WeightBalanceOutput,
        propeller: &'a PropellerSpec,
        perf: &'a PerformanceSpec,
        mission: &'a MissionSpec,
        electrical: &'a ElectricalSpec,
        gear: &'a GearSpec,
        gear_cfg: &'a GearCfg,
        fuel_capacity_l: f64,
        robustness: &'a RobustnessSpec,
        prop_cfg: &'a PropellerCfg,
    ) -> VerifyInputs<'a> {
        VerifyInputs {
            req, wing, prop, mtow_kg, engine, wb, propeller, perf, mission,
            electrical, gear, gear_cfg, fuel_capacity_l, robustness, prop_cfg,
        }
    }

    /// Monta um `(Requirements, WingSpec, PropulsionSpec, EngineSpec,
    /// WeightBalanceOutput, PropellerSpec, PerformanceSpec)` coerente via os
    /// agentes reais (motor sintético de teste — não um motor real) a partir
    /// de uma `AircraftConfig` fornecida — usada por `setup()` (fixture
    /// padrão) e pelos testes de aviso de divergência de diâmetro abaixo, que
    /// precisam mutar `cfg.propeller` ANTES do pipeline rodar (o aviso
    /// depende do diâmetro PROVISÓRIO real calculado por
    /// `AircraftState::from_config`, dentro do laço de
    /// `orchestrator::size_aircraft`, não de um valor sobrescrito depois).
    fn setup_with_cfg(cfg: crate::models::aircraft_config::AircraftConfig)
        -> (Requirements, WingSpec, PropulsionSpec, EngineSpec, WeightBalanceOutput, PropellerSpec, PerformanceSpec, MissionSpec, ElectricalSpec, GearSpec, GearCfg, PropellerCfg, RobustnessSpec)
    {
        let req = crate::models::requirements::test_fixtures::requisitos_teste();
        setup_with_cfg_and_req(cfg, req)
    }

    /// Mesmo pipeline de `setup_with_cfg`, mas recebe `req` explícito.
    ///
    /// DOCSTRING CORRIGIDA (revisão final, campanha E10): a versão anterior
    /// desta nota dizia que esta função era "usada por
    /// `envelope_de_cg_fechado_sem_violacao_no_baseline_real` ... para
    /// exercitar o baseline real (config E TAMBÉM missão de
    /// `config/missions/default.toml`, não a fixture sintética mais leve
    /// `requisitos_teste()`)" — FALSO desde que esse teste passou a chamar
    /// `setup_with_cfg_req_engine` DIRETAMENTE (campanha E10, para também
    /// controlar a massa do motor — ver docstring dessa função), sem passar
    /// por aqui. Hoje esta função é, na prática, um PASS-THROUGH interno: o
    /// único chamador é `setup_with_cfg` (logo abaixo), que sempre passa a
    /// MESMA fixture sintética (`requisitos_teste()`) — nenhum chamador
    /// atual usa o parâmetro `req` para passar algo diferente do fixture
    /// padrão. Mantida como função separada pela simetria com
    /// `setup_with_cfg_req_engine` (mesmo padrão de "MESMO pipeline, um
    /// parâmetro a mais explícito"), não por necessidade funcional atual.
    ///
    /// Ciclo 5 (task massa-total, fix de review): trocado de um pipeline
    /// MANUAL com MTOW FIXO (`state.mtow_kg` = palpite inicial de
    /// `[sizing]`, sem iterar o laço) para `orchestrator::size_aircraft`
    /// de verdade — mesma correção aplicada à fixture interna de
    /// `validation::robustness::tests::nominal_pipeline`. Motivo: o 3º
    /// caso adversarial de `RobustnessAgent::run` ("massa-total") sempre
    /// re-converge o laço COMPLETO para o mundo perturbado (+σ); se o
    /// nominal passado aqui (`mission`/`perf`, entre outros) viesse de um
    /// MTOW não convergido, a comparação do caso massa-total ficaria
    /// enviesada por bases diferentes (nominal não convergido vs.
    /// perturbado sempre convergido) — não pelo efeito físico de σ.
    /// `wb`/`gear`/`mission`/`perf` saem TODOS do MESMO `SizedAircraft`
    /// convergido, como em produção. Consequência aceita: os números
    /// pinados de vários testes abaixo mudam de "MTOW inicial de
    /// [sizing]" para "MTOW de missão convergido" — ver comentários
    /// `old→new` nos testes afetados.
    fn setup_with_cfg_and_req(cfg: crate::models::aircraft_config::AircraftConfig, req: Requirements)
        -> (Requirements, WingSpec, PropulsionSpec, EngineSpec, WeightBalanceOutput, PropellerSpec, PerformanceSpec, MissionSpec, ElectricalSpec, GearSpec, GearCfg, PropellerCfg, RobustnessSpec)
    {
        setup_with_cfg_req_engine(cfg, req, motor_generico_teste())
    }

    /// Mesmo pipeline de `setup_with_cfg_and_req`, mas com o MOTOR também
    /// explícito (campanha E10, 2026-08-08).
    ///
    /// Motivo: `motor_generico_teste()` pesa 150 kg — bem abaixo da classe de
    /// motor que ESTA célula assume no seu layout de braços (~195 kg, o valor
    /// concreto vive em `config/engines/*.toml`, fora de `src/`). Como o
    /// motor está no braço MAIS DIANTEIRO de toda a aeronave
    /// (`[arms].engine_cg_m` = 0,65 m, ≈2,8 m à frente do CG), 45 kg a menos
    /// ali valem ≈+6,7 pp de MAC no CG de TODOS os cenários — um viés de
    /// fixture grande o bastante para dominar qualquer conclusão sobre
    /// ENVELOPE de CG. Medido, com a config real de E10 e o resto da fixture
    /// sintética idêntico:
    ///   motor 150 kg → CG dos cenários [24,39%, 44,32%] MAC, SM mín 4,14%
    ///   motor 170 kg → CG dos cenários [20,86%, 41,30%] MAC, SM mín 7,16%
    ///   motor 195 kg → CG dos cenários [16,63%, 37,65%] MAC, SM mín 10,81%
    /// (limite traseiro do envelope: 43,46% nos três — não depende do motor).
    /// Ou seja: com o motor sintético leve o baseline "violaria" o envelope
    /// traseiro; com um motor da classe real, não — e é o pipeline real
    /// (`cargo run`, `tests/gear_tipback.rs`, `aircraft_spec.json`) que dá a
    /// resposta certa: CG [17,9%, 38,8%], zero violações.
    ///
    /// Este helper existe para que `envelope_de_cg_fechado_sem_violacao_no_
    /// baseline_real` — o único teste deste módulo cuja PERGUNTA é sobre o
    /// baseline real e não sobre uma violação isolada — possa fazer essa
    /// pergunta com uma massa de motor representativa, sem tocar em
    /// `motor_generico_teste()` (usada por dezenas de outros testes, todos
    /// sobre violações ISOLADAS, onde o viés de CG é irrelevante).
    fn setup_with_cfg_req_engine(cfg: crate::models::aircraft_config::AircraftConfig,
                                 req: Requirements, engine: EngineSpec)
        -> (Requirements, WingSpec, PropulsionSpec, EngineSpec, WeightBalanceOutput, PropellerSpec, PerformanceSpec, MissionSpec, ElectricalSpec, GearSpec, GearCfg, PropellerCfg, RobustnessSpec)
    {
        let sized = crate::orchestrator::size_aircraft(&cfg, &engine, &req)
            .expect("fixture sintética deveria convergir (ver orchestrator::size_aircraft)");
        let state = sized.state;
        let wing = sized.wing;
        let prop = sized.prop;
        let emp = sized.emp;
        // Massas estruturais COMPUTADAS na iteração CONVERGIDA (ciclo 3,
        // `agents::mass_model`, via `SizedAircraft::structural_masses`) —
        // as MESMAS que alimentaram o OEW (`wb`) dentro do laço. Fonte
        // única, como em produção.
        let masses = sized.structural_masses;
        // `wb` já sai do laço com `apply_trim` aplicado (ver docstring de
        // `orchestrator::SizedAircraft::wb`) — não precisa reaplicar aqui.
        let wb = sized.wb;
        let mut propeller = PropellerAgent::run(&cfg, &engine, &prop, &req);
        let perf = PerformanceAgent::run(&state, &wing, &prop, state.mtow_kg, &engine, &req,
                                          &cfg.performance);
        let mission = sized.mission;
        let electrical = ElectricalAgent::run(&cfg);
        // Task 2 (refino-ciclo2): CG mais dianteiro/traseiro REAIS dos
        // cenários de carga (não o limite admissível) — mesma fórmula de
        // `main.rs`. Trem de pouso dimensiona pelo MTOW de ENVELOPE
        // (`wb.spec.mtow_kg`, pior caso legal "4 pax + bagagem + cheio"),
        // não pelo MTOW de missão (`state.mtow_kg`) — mesma convenção de
        // `main.rs`/`orchestrator` para `LandingGearAgent`.
        let x_cg_fwd = cfg.wing.le_root_x_m + wb.spec.cg_mac_fwd_pct / 100.0 * wb.mac_m;
        let x_cg_aft = cfg.wing.le_root_x_m + wb.spec.cg_mac_aft_pct / 100.0 * wb.mac_m;
        let gear = crate::agents::landing_gear::LandingGearAgent::run(
            wb.spec.mtow_kg, x_cg_fwd, x_cg_aft, &cfg.gear,
            masses.trem_principal_kg, masses.trem_nariz_kg,
        );
        let gear_cfg = cfg.gear.clone();
        let prop_cfg = cfg.propeller.clone();
        // Ciclo 8 (task 2): preenche `prop_clearance_critical_m` (check
        // #25) no MESMO caminho de `main.rs` — depois que `gear` existe.
        // Ciclo 9: `prop_cfg` novo (fator de amplificação do pivô).
        propeller.fill_critical_clearance(&gear, &gear_cfg, &prop_cfg);
        // Ciclo 4 (task robustez, wiring): `RobustnessSpec` na MESMA
        // sequência de `main.rs` — os dois conjuntos adversariais avaliados
        // contra os limites NOMINAIS já calculados acima (`wb`/`gear`, já
        // com o trim aplicado); ciclo 5, `mission`/`perf` NOMINAIS (do MESMO
        // `SizedAircraft` convergido) para o 3º caso (massa-total).
        let robustness = crate::validation::robustness::RobustnessAgent::run(
            &cfg, &engine, &req, &state, &wing, &emp, &masses, &wb, &gear, &propeller, &mission, &perf,
        );
        (req, wing, prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness)
    }

    /// Base de todas as asserções de violação isolada abaixo — usa a
    /// fixture sintética padrão (`config_teste()`). Os testes sobrescrevem
    /// apenas os campos relevantes à violação isolada em questão, para não
    /// depender do resultado real dos demais agentes (já testados em seus
    /// próprios módulos).
    fn setup() -> (Requirements, WingSpec, PropulsionSpec, EngineSpec, WeightBalanceOutput, PropellerSpec, PerformanceSpec, MissionSpec, ElectricalSpec, GearSpec, GearCfg, PropellerCfg, RobustnessSpec) {
        setup_with_cfg(crate::models::aircraft_config::test_fixtures::config_teste())
    }

    #[test]
    fn violacao_cruzeiro_inviavel_aparece_quando_infeasible() {
        let (req, wing, mut prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        // Força a inviabilidade independentemente do resultado real da busca
        // de rpm, para testar isoladamente a violação #9.
        prop.cruise_feasible = false;
        prop.p_req_cruise_kw = 150.0;
        prop.p_shaft_cruise_kw = 100.0;

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(report.violations.iter().any(|v| v.contains("Cruzeiro inviável")),
            "esperava violação de cruzeiro inviável, obteve: {:?}", report.violations);
        // A mensagem deve carregar os números reais, não só um rótulo.
        assert!(report.violations.iter().any(|v| v.contains("150.0") && v.contains("100.0")),
            "violação deveria citar P_req/P_shaft: {:?}", report.violations);
    }

    #[test]
    fn sem_violacao_cruzeiro_quando_feasible() {
        let (req, wing, mut prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        prop.cruise_feasible = true;
        prop.p_req_cruise_kw = 90.0;
        prop.p_shaft_cruise_kw = 100.0;

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

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
        let (req, wing, prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        assert!(wb.scenarios.iter().any(|s| !s.inside_envelope),
            "pré-condição do teste: fixture sintética deveria ter ao menos um \
             cenário fora do envelope (achado honesto, replicado do baseline real)");

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(report.violations.iter().any(|v| v.contains("fora do envelope de CG admissível")),
            "esperava violação de envelope de CG, obteve: {:?}", report.violations);
        // A mensagem deve citar os limites do envelope em %MAC, não só um rótulo.
        assert!(report.violations.iter().any(|v|
                v.contains(&format!("{:.1}%", wb.spec.cg_limit_fwd_pct_mac))),
            "violação deveria citar o limite dianteiro do envelope: {:?}", report.violations);
    }

    /// Fix de revisão (FIX4): o baseline real tem envelope de CG VAZIO —
    /// o limite de rotação (à época invariante ao peso, ≈39,9% MAC) fica À FRENTE
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
    ///
    /// ATUALIZAÇÃO (campanha E7, 2026-08-06): `gear.x_main_m` 3,55→3,66m
    /// (fecha o tipback do baseline real, ver
    /// `config/aircraft/baseline_4seat.toml`) desloca o braço do item de
    /// massa `trem_principal` (arm_ref="gear_main") ~0,11m para trás,
    /// puxando o CG de TODOS os cenários um pouco para trás. No baseline
    /// real (pipeline COMPLETO via `orchestrator::size_aircraft`, missão
    /// real de `config/missions/default.toml`, pax 90kg/bagagem 80kg, MTOW
    /// reconvergido) a margem estática mínima segue folgada (~11,0%, ver
    /// `cargo run`/`aircraft_spec.json`, `tests/gear_tipback.rs`) — ZERO
    /// violações, confirmado pelo pipeline real. Na época (E7), este teste
    /// em particular usava `setup_with_cfg`/`requisitos_teste()` com MTOW
    /// FIXO (fixture não iterava o laço de convergência — ver histórico
    /// abaixo) e cruzava o piso de 5% por coincidência de fixture
    /// (SM≈4,97% no cenário "4 pax + bagagem + cheio").
    ///
    /// RE-PIN (ciclo 5, task massa-total, fix de review): `setup_with_cfg`/
    /// `setup_with_cfg_and_req` passaram a rodar `orchestrator::
    /// size_aircraft` de verdade (ver docstring de `setup_with_cfg_and_req`
    /// — necessário para o 3º caso do `RobustnessAgent` comparar contra um
    /// nominal genuinamente convergido). Com o MTOW agora reconvergido
    /// (mesmo com a missão sintética mais leve, `requisitos_teste()`), o
    /// cenário "4 pax + bagagem + cheio" sobe de SM≈4,97% para **SM≈11,07%**
    /// (old→new) — a coincidência de fixture desaparece; ZERO violações de
    /// envelope, coerente com o pipeline real. Não mascarado: a asserção
    /// abaixo agora exige zero violações de envelope (qualquer violação,
    /// marginal ou não, reprova o teste).
    ///
    /// ATUALIZAÇÃO (campanha E10, 2026-08-08): a bateria híbrida de 53 kg a
    /// 7,80 m recua o CG de todos os cenários ≈+6,5 pp MAC. No pipeline REAL
    /// isso é folgado (CG [17,9%, 38,8%] contra um limite traseiro de 43,5%,
    /// SM mín 9,68%, ZERO violações — ver `cargo run`/`aircraft_spec.json`/
    /// `tests/gear_tipback.rs`), mas somado ao VIÉS DE FIXTURE do motor
    /// sintético leve (150 kg no braço mais dianteiro, ≈+6,7 pp de CG — ver
    /// a tabela medida na docstring de `setup_with_cfg_req_engine`) o CG
    /// desta fixture ia a 44,3% e produzia 6 violações inexistentes no
    /// projeto real (2 de envelope, 1 de tipback, 3 de robustez).
    ///
    /// Correção: este teste passa a usar `setup_with_cfg_req_engine` com uma
    /// massa de motor da CLASSE que esta célula assume (~195 kg), em vez de
    /// `motor_generico_teste()` (150 kg). Não é afrouxamento — é o oposto:
    /// antes de E10 o viés de 6,7 pp existia igual, só não era grande o
    /// bastante para cruzar o limite, e o teste vinha "passando por sorte"
    /// com uma geometria de massas que não era a do baseline real. As
    /// asserções seguem exigindo ZERO violações de envelope, agora sobre uma
    /// fixture que de fato representa a pergunta do nome do teste. Os
    /// DEMAIS testes deste módulo (violações ISOLADAS, com campos
    /// sobrescritos à mão) continuam com `motor_generico_teste()` intocado.
    #[test]
    fn envelope_de_cg_fechado_sem_violacao_no_baseline_real() {
        let toml = std::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("config/aircraft/baseline_4seat.toml"),
        ).expect("falha ao ler baseline_4seat.toml do disco");
        let cfg = crate::models::config::parse_aircraft(&toml)
            .expect("baseline real deveria ser uma configuração válida");
        let fuel_capacity_l = cfg.fuel_system.capacity_l;
        // Campanha E10: massa de motor representativa da classe desta célula
        // (~195 kg) — ver docstring de `setup_with_cfg_req_engine` para a
        // sensibilidade medida (±45 kg no braço de 0,65 m ⟹ ∓6,7 pp de MAC).
        // O resto do motor segue sintético/genérico (curva de torque, BSFC,
        // combustível) — `src/` não conhece motores concretos.
        let mut engine_classe_real = motor_generico_teste();
        // Revisão final: constante compartilhada com o hand-check gêmeo de
        // `agents::trim_authority` (mesma massa de motor, mesmo motivo) —
        // ver `models::engine::test_fixtures::MASSA_MOTOR_CLASSE_KG`.
        engine_classe_real.mass_kg = crate::models::engine::test_fixtures::MASSA_MOTOR_CLASSE_KG;
        // Nome honesto (revisão final — era `req_real`, o que sugeria "a
        // missão real do projeto"; é o MESMO fixture sintético leve
        // (`requisitos_teste()`, 85kg/60kg pax/bagagem) que `setup_with_cfg`
        // usa em todo o resto do módulo — só o MOTOR e a CONFIG (`cfg`,
        // lida do TOML real acima) são "reais" aqui, não os requisitos).
        let req_sintetico = crate::models::requirements::test_fixtures::requisitos_teste();
        let (req, wing, prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness) =
            setup_with_cfg_req_engine(cfg, req_sintetico, engine_classe_real);
        assert!(wb.spec.cg_limit_fwd_pct_mac <= wb.spec.cg_limit_aft_pct_mac,
            "pré-condição do teste: baseline real (pós E1–E6) deveria ter envelope de CG \
             FECHADO (fwd={:.2}% <= aft={:.2}%)", wb.spec.cg_limit_fwd_pct_mac,
            wb.spec.cg_limit_aft_pct_mac);

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, fuel_capacity_l, &robustness, &prop_cfg));

        assert!(!report.violations.iter().any(|v| v.contains("Envelope de CG VAZIO")),
            "não deveria haver violação dedicada de envelope vazio no baseline pós-E6: {:?}",
            report.violations);
        // RE-PIN (ciclo 5, ver docstring acima): com a fixture agora
        // convergida de verdade, a violação marginal de fixture (SM≈4,97%,
        // achado da campanha E7) não ocorre mais — o cenário "4 pax +
        // bagagem + cheio" sobe para SM≈11,07%. Zero violações de envelope
        // esperadas, sem exceção.
        let violacoes_de_envelope: Vec<&String> = report.violations.iter()
            .filter(|v| v.contains("fora do envelope de CG admissível"))
            .collect();
        println!("violações de envelope = {violacoes_de_envelope:?}");
        // RE-CONFIRMADO (ciclo 10, task 2 — momento da linha de tração):
        // ZERO violações de envelope CONTINUA sendo o veredito. O limite
        // dianteiro de rotação recua de 8,533% para ≈13,4% MAC (o custo
        // físico da linha de tração, `T(Vr)·prop_axis_above_cg_m`), mas o
        // CG mais dianteiro do baseline está em 17,9% MAC — ainda 4,5 pp
        // atrás do novo limite. Uma versão intermediária desta task usava o
        // braço ERRADO (altura sobre o SOLO, 1,12 m em vez do offset
        // eixo↔CG de 0,20 m, sem o cancelamento inercial de d'Alembert) e
        // punha o limite em 35,5%, reabrindo 3 cenários; o ERRATUM da spec
        // §2 corrigiu o braço e a reabertura desapareceu com ele. Ver
        // `agents::trim_authority::rotation_available_moment_nm`.
        assert!(violacoes_de_envelope.is_empty(),
            "achado honesto (ciclo 5, RE-CONFIRMADO no ciclo 10 task 2): com a fixture \
             reconvergida, NÃO deveria haver nenhuma violação de envelope — o recuo do limite \
             de rotação pela linha de tração (≈+5 pp) não alcança o CG mais dianteiro: {:?}",
            report.violations);
        assert!(!report.violations.iter().any(|v| v.contains("Envelope de CG VAZIO")),
            "o envelope continua FECHADO (fwd ≈13,4% < aft ≈43,5%): {:?}", report.violations);

        let cheio = wb.scenarios.iter().find(|s| s.name == "4 pax + bagagem + cheio")
            .expect("cenário '4 pax + bagagem + cheio' deveria existir nos scenarios");
        // Pin de banda (achado de review, ciclo 5, Minor 6): `> 0.05` sozinho
        // é um pin honesto FRACO — passaria para qualquer SM acima do piso
        // de 5%, mascarando uma regressão que deslocasse o SM medido
        // (~11,07%) para, digamos, 6% sem quebrar o teste. A banda abaixo
        // (10,5%–11,5%) é centrada no valor medido e detecta esse tipo de
        // deriva silenciosa, mantendo a folga de arredondamento de ponto
        // flutuante entre runs.
        //
        // Campanha E10 (2026-08-08): a BANDA fica INALTERADA — o valor
        // medido se move dentro dela, de ≈11,07% para **≈10,81%**, por dois
        // efeitos que quase se cancelam: a bateria de 53 kg a 7,80 m recua o
        // CG (−SM) e a massa de motor da fixture sobe de 150 para 195 kg
        // (+SM, ver `setup_with_cfg_req_engine`). Não é coincidência de
        // fixture: no pipeline REAL a SM mínima é 9,68% (ver
        // `tests/empennage.rs`), o resíduo vem da missão sintética mais
        // leve (`requisitos_teste()`, 85 kg/60 kg pax/bagagem vs 90/80).
        assert!((0.105..0.115).contains(&cheio.static_margin),
            "SM do cenário '4 pax + bagagem + cheio' ({:.4}) deveria ficar na banda [10.5%, \
             11.5%) em torno do valor medido (~11,07%) agora que a fixture reconverge — pin \
             honesto do ciclo 5 (old≈0.0497→new≈0.1107)",
            cheio.static_margin);
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
        // para reproduzir uma autoridade de download reduzida, perto do
        // palpite antigo de 0.85. Ciclo 7 (task 1): 0.30→0.28 porque a
        // rotação passou a usar o CLmax de DECOLAGEM (`cl_max_to`), o que
        // avança TODO limite de rotação ~4 pp e fazia esta mutação deixar
        // de reproduzir o envelope vazio (35,7% < 36,6%) — ver comentário
        // completo em `agents::trim_authority::tests::trim_authority_agent_
        // run_hand_check_baseline_mutado_parametros_pre_e6`.
        //
        // Campanha E10 (2026-08-08): 0.28→**0.26** (`cl_h_max_down_calc`
        // ≈0.800), pelo MESMO mecanismo do ciclo 7 — `cl_max_to` e `Cm_TO`,
        // NÃO o recuo de CG da bateria: à época `rotation_fwd_limit_m`
        // não recebia CG, massa nem `x_nose_m` (era invariante ao peso —
        // invariância que MORREU no ciclo 10 task 2 com o momento da linha
        // de tração; a função passou a receber `weight_n`, e o dial desta
        // mutação continua valendo porque o achado que ele guarda —
        // envelope VAZIO — ficou ainda mais folgado com o limite recuado).
        // Com
        // o dial em 0.28 esta mutação passava a dar rot 36,09% < aft
        // 36,61%, ou seja, o envelope voltava a FECHAR e o achado sumia;
        // 0.26 dá rot 37,53%, restaurando-o com 0,91 pp de folga. Derivação
        // completa e varredura no comentário do teste gêmeo citado acima.
        cfg.control_surfaces.elevator_chord_frac = 0.26;
        let fuel_capacity_l = cfg.fuel_system.capacity_l;
        let (req, wing, prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness) = setup_with_cfg(cfg);
        assert!(wb.spec.cg_limit_fwd_pct_mac > wb.spec.cg_limit_aft_pct_mac,
            "pré-condição do teste: parâmetros pré-E6 (x_main_m=3.85, v_h=0.70, \
             elevator_chord_frac=0.26, config real mutada) deveriam reproduzir o envelope de \
             CG vazio original (fwd={:.2}% > aft={:.2}%)",
            wb.spec.cg_limit_fwd_pct_mac, wb.spec.cg_limit_aft_pct_mac);

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, fuel_capacity_l, &robustness, &prop_cfg));

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
        let (req, wing, prop, engine, mut wb, propeller, perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        for sc in &mut wb.scenarios {
            sc.inside_envelope = true;
        }

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(!report.violations.iter().any(|v| v.contains("fora do envelope de CG admissível")),
            "não deveria haver violação de envelope de CG, obteve: {:?}", report.violations);
    }

    // ─── Task 4.5: hélice (Mach de ponta / folga de solo) ──────────────────

    #[test]
    fn violacao_de_helice_aparece_quando_algum_ok_e_falso() {
        let (req, wing, prop, engine, wb, mut propeller, perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        propeller.ok_mach_static = false;
        propeller.tip_mach_static = 0.99;

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(report.violations.iter().any(|v| v.contains("Mach de ponta ESTÁTICO")),
            "esperava violação de Mach de ponta estático, obteve: {:?}", report.violations);
        assert!(report.violations.iter().any(|v| v.contains("0.990")),
            "violação deveria citar o Mach observado: {:?}", report.violations);
    }

    /// Achado honesto da fixture sintética (mesma filosofia da fixture de
    /// envelope de CG, Task 4.4): `config_teste()` deriva shaft_height =
    /// `h_cg_ground_m(1.03) + prop_axis_above_cg_m(0.12)` = 1.15 (idêntico ao
    /// `shaft_height_m` pré-ciclo-5), `diameter_m=Some(1.90)`,
    /// `ground_clearance_min_m=0.25` — folga real = 1,15 − 0,95 = 0,20 m <
    /// 0,25 m, então a checagem de folga de solo falha naturalmente (não
    /// precisa de override) para esta fixture.
    #[test]
    fn violacao_de_folga_de_solo_aparece_naturalmente_na_fixture_sintetica() {
        let (req, wing, prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        assert!(!propeller.ok_clearance,
            "pré-condição do teste: fixture sintética (shaft_height derivado=1.15, diameter=1.90, \
             ground_clearance_min_m=0.25) deveria falhar na folga de solo — obtido \
             ground_clearance_m={:.3}", propeller.ground_clearance_m);

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(report.violations.iter().any(|v| v.contains("folga de solo")),
            "esperava violação de folga de solo, obteve: {:?}", report.violations);
    }

    /// Sanidade inversa: se TODOS os `ok_*` estiverem artificialmente
    /// verdadeiros (mesmo padrão de `sem_violacao_de_envelope_...` acima),
    /// nenhuma violação de hélice deve aparecer.
    #[test]
    fn sem_violacao_de_helice_quando_todos_ok_forcado() {
        let (req, wing, prop, engine, wb, mut propeller, perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        propeller.ok_mach_static = true;
        propeller.ok_mach_cruise = true;
        propeller.ok_clearance = true;

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

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

        let (req, wing, prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness) = setup_with_cfg(cfg);
        assert_eq!(propeller.source, "derivado");
        println!(
            "pipeline real (Mach governa): D_autoritativo={:.4} D_provisorio(prop.prop_diameter_m)={:.4}",
            propeller.diameter_m, prop.prop_diameter_m
        );
        assert!((propeller.diameter_m - prop.prop_diameter_m).abs() > 0.01,
            "pré-condição do teste: com PSRU 1:1 o diâmetro autoritativo ({:.4}) deveria \
             divergir do provisório ({:.4}) usado pela busca de cruzeiro",
            propeller.diameter_m, prop.prop_diameter_m);

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

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

        let (req, wing, prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness) = setup_with_cfg(cfg);
        assert_eq!(propeller.source, "derivado");
        println!(
            "pipeline real (folga governa): D_autoritativo={:.4} D_provisorio(prop.prop_diameter_m)={:.4}",
            propeller.diameter_m, prop.prop_diameter_m
        );

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb, &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

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
            ldg_50ft_grass_m: 620.0,
        }
    }

    #[test]
    fn violacao_de_gradiente_aparece_quando_abaixo_de_8_3_por_cento() {
        let (req, wing, prop, engine, wb, propeller, _perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        let perf = performance_spec_com_gradiente(6.0);

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(report.violations.iter().any(|v| v.contains("Gradiente de subida")),
            "esperava violação de gradiente de subida, obteve: {:?}", report.violations);
        // A mensagem deve citar o gradiente observado e o piso da CS 23.65.
        assert!(report.violations.iter().any(|v| v.contains("6.0") && v.contains("8.3")),
            "violação deveria citar o gradiente observado (6.0%) e o mínimo CS 23.65 \
             (8.3%): {:?}", report.violations);
    }

    #[test]
    fn sem_violacao_de_gradiente_quando_maior_ou_igual_a_8_3_por_cento() {
        let (req, wing, prop, engine, wb, propeller, _perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        let perf = performance_spec_com_gradiente(9.0);

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(!report.violations.iter().any(|v| v.contains("Gradiente de subida")),
            "não deveria haver violação de gradiente quando climb_gradient_pct (9.0%) >= \
             8.3%, obteve: {:?}", report.violations);
    }

    // ─── Ciclo 5 (fix de review): checagens #21/#22 — RC ao nível do mar e
    // teto de serviço, nominais ─────────────────────────────────────────
    //
    // Antes deste fix, `main.rs` computava `rc_ok`/`ceil_ok` localmente
    // (hardcoded 1.5/3_000.0) só para IMPRIMIR o gate — `ConstraintChecker::
    // verify` nunca os checava, então nenhum teste desta suíte cobria o
    // caminho de violação. Mesmo padrão de `performance_spec_com_gradiente`
    // acima: `PerformanceSpec` sintética mínima construída à mão, variando
    // só os dois campos relevantes.

    fn performance_spec_com_rc_e_teto(rc_sl_ms: f64, service_ceiling_m: f64) -> PerformanceSpec {
        PerformanceSpec {
            v_cruise_kmh: 300.0,
            v_stall_kmh: 100.0,
            rc_sl_ms,
            rc_cruise_alt_ms: 4.0,
            service_ceiling_m,
            to_distance_paved_m: 300.0,
            to_distance_grass_m: 360.0,
            landing_distance_m: 400.0,
            range_km: 2_000.0,
            endurance_h: 8.0,
            vx_kmh: 120.0,
            vy_kmh: 150.0,
            best_glide_kmh: 170.0,
            glide_ratio: 15.0,
            climb_gradient_pct: 10.0,
            to_50ft_paved_m: 400.0,
            to_50ft_grass_m: 450.0,
            ldg_50ft_m: 550.0,
            ldg_50ft_grass_m: 620.0,
        }
    }

    #[test]
    fn violacao_de_rc_sl_aparece_quando_abaixo_do_minimo() {
        let (req, wing, prop, engine, wb, propeller, _perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        let perf = performance_spec_com_rc_e_teto(1.4, 5_000.0); // RC abaixo de RC_SL_MIN_MS (1.5)

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(report.violations.iter().any(|v| v.contains("Razão de subida ao nível do mar")),
            "esperava violação de RC ao nível do mar, obteve: {:?}", report.violations);
        assert!(report.violations.iter().any(|v| v.contains("1.40") && v.contains("1.5")),
            "violação deveria citar o RC observado (1.40) e o piso (1.5): {:?}", report.violations);
    }

    #[test]
    fn sem_violacao_de_rc_sl_quando_acima_do_minimo() {
        let (req, wing, prop, engine, wb, propeller, _perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        let perf = performance_spec_com_rc_e_teto(5.0, 5_000.0);

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(!report.violations.iter().any(|v| v.contains("Razão de subida ao nível do mar")),
            "não deveria haver violação de RC ao nível do mar quando rc_sl_ms (5.0) >= \
             RC_SL_MIN_MS (1.5), obteve: {:?}", report.violations);
    }

    #[test]
    fn violacao_de_teto_de_servico_aparece_quando_abaixo_do_minimo() {
        let (req, wing, prop, engine, wb, propeller, _perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        let perf = performance_spec_com_rc_e_teto(5.0, 2_500.0); // teto abaixo de SERVICE_CEILING_MIN_M (3_000)

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(report.violations.iter().any(|v| v.contains("Teto de serviço")),
            "esperava violação de teto de serviço, obteve: {:?}", report.violations);
        assert!(report.violations.iter().any(|v| v.contains("2500") && v.contains("3000")),
            "violação deveria citar o teto observado (2500) e o mínimo (3000): {:?}",
            report.violations);
    }

    #[test]
    fn sem_violacao_de_teto_de_servico_quando_acima_do_minimo() {
        let (req, wing, prop, engine, wb, propeller, _perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        let perf = performance_spec_com_rc_e_teto(5.0, 5_000.0);

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(!report.violations.iter().any(|v| v.contains("Teto de serviço")),
            "não deveria haver violação de teto de serviço quando service_ceiling_m (5000) >= \
             SERVICE_CEILING_MIN_M (3000), obteve: {:?}", report.violations);
    }

    // ─── Ciclo 6 (task 2): checagens #23/#24 — pista disponível ────────────
    //
    // #23: decolagem na grama sobre 15 m não pode exceder a pista disponível
    // (`req.runway_available_m`). #24: idem para o pouso. Ao contrário das
    // checagens de gradiente/RC/teto acima, aqui usamos o `perf` REAL
    // calculado pelo pipeline de `setup()` (não uma `PerformanceSpec`
    // sintética à mão) — basta sobrescrever `req.runway_available_m` para
    // ficar 1 m abaixo do valor JÁ calculado, isolando exatamente a
    // checagem sob teste sem depender de nenhum número mágico externo.

    #[test]
    fn check_23_reprova_decolagem_grama_maior_que_pista() {
        let (mut req, wing, prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        req.runway_available_m = perf.to_50ft_grass_m - 1.0;

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(report.violations.iter().any(|v| v.contains("grama") && v.contains("pista disponível")),
            "esperava violação de decolagem na grama excedendo a pista disponível \
             ({:.1} m > {:.1} m), obteve: {:?}", perf.to_50ft_grass_m, req.runway_available_m,
             report.violations);
    }

    #[test]
    fn check_24_reprova_pouso_maior_que_pista() {
        let (mut req, wing, prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        // Revisão final do ciclo 6: o gate é o pouso na GRAMA
        // (`ldg_50ft_grass_m`), não o pavimentado.
        req.runway_available_m = perf.ldg_50ft_grass_m - 1.0;

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(report.violations.iter().any(|v| v.contains("Pouso (grama") && v.contains("pista disponível")),
            "esperava violação de pouso na grama excedendo a pista disponível \
             ({:.1} m > {:.1} m), obteve: {:?}", perf.ldg_50ft_grass_m, req.runway_available_m,
             report.violations);
    }

    /// O gate #24 é o pouso na GRAMA, não o pavimentado: com a pista
    /// disponível ajustada para ficar ENTRE as duas distâncias (acima do
    /// pavimentado, abaixo da grama), o check DEVE reprovar. Antes da
    /// revisão final do ciclo 6 (quando #24 comparava `ldg_50ft_m`), esta
    /// faixa passava limpo — é exatamente a janela de otimismo que a
    /// correção fecha.
    #[test]
    fn check_24_usa_a_grama_e_nao_o_pavimentado() {
        let (mut req, wing, prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        assert!(perf.ldg_50ft_grass_m > perf.ldg_50ft_m,
            "pré-condição física: frenagem pior na grama (mu_brake_grass < mu_brake_paved) deveria \
             ALONGAR o pouso — grama {:.1} m vs pavimentado {:.1} m",
             perf.ldg_50ft_grass_m, perf.ldg_50ft_m);
        // Pista entre as duas distâncias: o pavimentado caberia, a grama não.
        req.runway_available_m = (perf.ldg_50ft_m + perf.ldg_50ft_grass_m) / 2.0;

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(report.violations.iter().any(|v| v.contains("Pouso (grama")),
            "pista de {:.1} m acomoda o pouso pavimentado ({:.1} m) mas NÃO o de grama ({:.1} m) — \
             #24 deveria reprovar: {:?}", req.runway_available_m, perf.ldg_50ft_m,
             perf.ldg_50ft_grass_m, report.violations);
    }

    #[test]
    fn checks_de_pista_passam_na_fixture_intacta() {
        let (req, wing, prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness) = setup();

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(!report.violations.iter().any(|v| v.contains("pista disponível")),
            "fixture intacta (pista de {:.0} m sintéticos) não deveria violar #23/#24 — \
             decolagem grama {:.1} m, pouso grama {:.1} m: {:?}",
             req.runway_available_m, perf.to_50ft_grass_m, perf.ldg_50ft_grass_m,
             report.violations);
    }

    // ─── Task 5.2: orçamento elétrico ────────────────────────────────────

    /// `ElectricalSpec` sintética mínima para os testes de violação/aviso
    /// elétrico abaixo — construída à mão (não via `ElectricalAgent`) para
    /// isolar exatamente as checagens #14/#15 de `ConstraintChecker::verify`.
    ///
    /// Inclui uma carga 'trem_retratil' sintética com `peak_w` MUITO acima
    /// de qualquer `gear.actuator_power_w` plausível (fixture ou baseline
    /// real) — deliberado, para que estes testes de #14/#15 não disparem
    /// acidentalmente a checagem #20 (ciclo 5), que tem seus próprios
    /// testes dedicados (`check_20_*`, abaixo).
    fn electrical_spec(alternator_w: f64, continuous_load_w: f64, peak_load_w: f64) -> ElectricalSpec {
        ElectricalSpec {
            bus_voltage_v: 28.0,
            alternator_w,
            continuous_load_w,
            peak_load_w,
            margin_continuous_pct: (alternator_w - continuous_load_w) / alternator_w * 100.0,
            loads: vec![ElectricalLoadSpec {
                name: GEAR_ACTUATOR_LOAD_NAME.to_string(),
                continuous_w: 0.0,
                peak_w: 1.0e6,
            }],
        }
    }

    #[test]
    fn violacao_eletrica_aparece_quando_carga_continua_excede_80pct_do_alternador() {
        let (req, wing, prop, engine, wb, propeller, perf, mission, _electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        // 900 W de alternador, 80% = 720 W — 750 W de carga contínua excede.
        let electrical = electrical_spec(900.0, 750.0, 750.0);

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(report.violations.iter().any(|v| v.contains("carga contínua")),
            "esperava violação de carga elétrica contínua, obteve: {:?}", report.violations);
        assert!(report.violations.iter().any(|v| v.contains("750.0") && v.contains("720.0")),
            "violação deveria citar a carga observada (750.0 W) e o limite de 80% (720.0 W): \
             {:?}", report.violations);
    }

    #[test]
    fn sem_violacao_eletrica_quando_carga_continua_dentro_de_80pct() {
        let (req, wing, prop, engine, wb, propeller, perf, mission, _electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        // 900 W de alternador, 80% = 720 W — 430 W (baseline real) fica bem dentro.
        let electrical = electrical_spec(900.0, 430.0, 1_260.0);

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(!report.violations.iter().any(|v| v.contains("carga contínua")),
            "não deveria haver violação de carga contínua, obteve: {:?}", report.violations);
    }

    #[test]
    fn aviso_eletrico_aparece_quando_pico_excede_o_alternador() {
        let (req, wing, prop, engine, wb, propeller, perf, mission, _electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        // Pico pior-caso do baseline real (1.260 W) > alternador (900 W).
        let electrical = electrical_spec(900.0, 430.0, 1_260.0);

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(report.warnings.iter().any(|w| w.contains("carga de pico")),
            "esperava aviso de pico elétrico, obteve: {:?}", report.warnings);
        // É AVISO, não violação — banco de baterias cobre o transiente.
        assert!(!report.violations.iter().any(|v| v.contains("pico")),
            "excesso de pico deveria ser aviso, não violação: {:?}", report.violations);
    }

    #[test]
    fn sem_aviso_eletrico_quando_pico_dentro_da_capacidade_do_alternador() {
        let (req, wing, prop, engine, wb, propeller, perf, mission, _electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        let electrical = electrical_spec(900.0, 430.0, 800.0);

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

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

        let (req, wing, prop, engine, wb, propeller, perf, mission, _electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical_real, &gear, &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(!report.violations.iter().any(|v| v.contains("carga contínua")),
            "baseline real (430 W contínuo / 900 W alternador, ~52,2% de margem) não deveria \
             violar o limite de 80%: {:?}", report.violations);
        assert!(report.warnings.iter().any(|w| w.contains("carga de pico")),
            "baseline real (pico 1.260 W > alternador 900 W) deveria disparar o aviso de \
             pico: {:?}", report.warnings);
    }

    // ─── Task 2 (refino-ciclo2) / campanha E7: tipback ──────────────────────
    //
    // Até a campanha E7 (2026-08-06), o baseline REAL violava a checagem #15
    // (tipback ~10,1° < 15°, ver `tests/gear_tipback.rs` antes de E7) — o
    // caminho de violação era exercitado pelo pipeline real, sem precisar de
    // fixture sintética dedicada aqui. E7 fechou o tipback do baseline
    // (`[gear].x_main_m` 3,55→3,66m, ver `config/aircraft/baseline_4seat.toml`),
    // então o teste abaixo assume o papel de preservar a cobertura do
    // caminho de violação — mesmo padrão de `violacao_de_margem_de_
    // combustivel_aparece_quando_abaixo_do_minimo` (checagem #18, mais
    // abaixo): sobrescreve só o campo relevante da fixture sintética
    // (`gear.tipback_angle_deg`), sem depender do resultado real dos demais
    // agentes.

    /// `gear_cfg_teste()`/`config_teste()` fixam `tipback_min_deg = 15.0`
    /// (ver `LandingGearAgent::run` nos testes de `agents::landing_gear` e
    /// `AircraftConfig` de teste). Sobrescreve `gear.tipback_angle_deg` para
    /// um valor sintético abaixo do piso — isola exatamente a checagem #15,
    /// sem depender de nenhum `x_main_m` real.
    #[test]
    fn violacao_de_tipback_aparece_quando_abaixo_do_piso() {
        let (req, wing, prop, engine, wb, propeller, perf, mission, electrical, mut gear, gear_cfg, prop_cfg, robustness) = setup();
        assert!(gear_cfg.tipback_min_deg > 0.0, "fixture deveria ter um piso de tipback positivo");
        gear.tipback_angle_deg = gear_cfg.tipback_min_deg - 2.0; // sintético, abaixo do piso

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear,
                                                 &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(report.violations.iter().any(|v| v.starts_with("Tipback:")),
            "esperava violação de tipback com gear.tipback_angle_deg sintético abaixo do piso, \
             obteve: {:?}", report.violations);
    }

    /// Sanidade inversa: `gear.tipback_angle_deg` acima do piso não deveria
    /// violar — confirma que a checagem #15 não dispara em falso.
    #[test]
    fn sem_violacao_de_tipback_quando_acima_do_piso() {
        let (req, wing, prop, engine, wb, propeller, perf, mission, electrical, mut gear, gear_cfg, prop_cfg, robustness) = setup();
        gear.tipback_angle_deg = gear_cfg.tipback_min_deg + 2.0; // sintético, acima do piso

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear,
                                                 &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(!report.violations.iter().any(|v| v.starts_with("Tipback:")),
            "não deveria haver violação de tipback com gear.tipback_angle_deg sintético acima \
             do piso, obteve: {:?}", report.violations);
    }

    // ─── Revisão final (baseline E10, 2026-08-08): checagem #17 (carga de
    // nariz nos dois extremos) — cobertura sintética dedicada ───────────────
    //
    // ACHADO DE REVISÃO: até esta rodada, nenhum teste deste módulo cobria
    // os dois ramos de #17 (:398 teto de 25%, :405 piso de 8%) diretamente —
    // a única cobertura viva era o pin honesto do baseline REAL em
    // `tests/gear_tipback.rs`, que passou a ficar do lado PASS (dentro dos
    // dois limites) desde que a campanha E10 fechou a violação de carga de
    // nariz. Um mutante `if false &&` em qualquer um dos dois ramos de #17
    // não quebrava NENHUM teste — buraco confirmado por mutação manual (ver
    // processo abaixo). Mesmo padrão de
    // `violacao_de_tipback_aparece_quando_abaixo_do_piso` (checagem #15,
    // acima): sobrescreve só o campo relevante da fixture sintética
    // (`gear.nose_load_max_pct`/`gear.nose_load_min_pct`), sem depender de
    // nenhum `x_nose_m`/`x_main_m` real.

    /// Ramo do TETO (:398): `gear.nose_load_max_pct` acima de
    /// `NOSE_LOAD_MAX_CEILING_PCT` (25%) deve violar — isola exatamente o
    /// ramo do teto, sem tocar no ramo do piso (`nose_load_min_pct` segue
    /// dentro da faixa da fixture padrão).
    #[test]
    fn violacao_de_carga_de_nariz_aparece_quando_max_acima_do_teto() {
        let (req, wing, prop, engine, wb, propeller, perf, mission, electrical, mut gear, gear_cfg, prop_cfg, robustness) = setup();
        gear.nose_load_max_pct = NOSE_LOAD_MAX_CEILING_PCT + 2.0; // sintético, acima do teto

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear,
                                                 &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(report.violations.iter().any(|v| v.starts_with("Carga de nariz:") && v.contains("DIANTEIRO")),
            "esperava violação de carga de nariz (ramo do TETO) com gear.nose_load_max_pct \
             sintético acima de {:.1}%, obteve: {:?}", NOSE_LOAD_MAX_CEILING_PCT, report.violations);
    }

    /// Sanidade inversa do ramo do TETO: `gear.nose_load_max_pct` no teto ou
    /// abaixo não deveria violar — confirma que o ramo não dispara em falso.
    #[test]
    fn sem_violacao_de_carga_de_nariz_quando_max_no_teto_ou_abaixo() {
        let (req, wing, prop, engine, wb, propeller, perf, mission, electrical, mut gear, gear_cfg, prop_cfg, robustness) = setup();
        gear.nose_load_max_pct = NOSE_LOAD_MAX_CEILING_PCT - 2.0; // sintético, abaixo do teto

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear,
                                                 &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(!report.violations.iter().any(|v| v.starts_with("Carga de nariz:") && v.contains("DIANTEIRO")),
            "não deveria haver violação de carga de nariz (ramo do TETO) com gear.nose_load_max_pct \
             sintético abaixo de {:.1}%, obteve: {:?}", NOSE_LOAD_MAX_CEILING_PCT, report.violations);
    }

    /// Ramo do PISO (:405): `gear.nose_load_min_pct` abaixo de
    /// `NOSE_LOAD_MIN_FLOOR_PCT` (8%) deve violar — isola exatamente o ramo
    /// do piso, sem tocar no ramo do teto (`nose_load_max_pct` segue dentro
    /// da faixa da fixture padrão).
    #[test]
    fn violacao_de_carga_de_nariz_aparece_quando_min_abaixo_do_piso() {
        let (req, wing, prop, engine, wb, propeller, perf, mission, electrical, mut gear, gear_cfg, prop_cfg, robustness) = setup();
        gear.nose_load_min_pct = NOSE_LOAD_MIN_FLOOR_PCT - 2.0; // sintético, abaixo do piso

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear,
                                                 &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(report.violations.iter().any(|v| v.starts_with("Carga de nariz:") && v.contains("TRASEIRO")),
            "esperava violação de carga de nariz (ramo do PISO) com gear.nose_load_min_pct \
             sintético abaixo de {:.1}%, obteve: {:?}", NOSE_LOAD_MIN_FLOOR_PCT, report.violations);
    }

    /// Sanidade inversa do ramo do PISO: `gear.nose_load_min_pct` no piso ou
    /// acima não deveria violar — confirma que o ramo não dispara em falso.
    #[test]
    fn sem_violacao_de_carga_de_nariz_quando_min_no_piso_ou_acima() {
        let (req, wing, prop, engine, wb, propeller, perf, mission, electrical, mut gear, gear_cfg, prop_cfg, robustness) = setup();
        gear.nose_load_min_pct = NOSE_LOAD_MIN_FLOOR_PCT + 2.0; // sintético, acima do piso

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear,
                                                 &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(!report.violations.iter().any(|v| v.starts_with("Carga de nariz:") && v.contains("TRASEIRO")),
            "não deveria haver violação de carga de nariz (ramo do PISO) com gear.nose_load_min_pct \
             sintético acima de {:.1}%, obteve: {:?}", NOSE_LOAD_MIN_FLOOR_PCT, report.violations);
    }

    // ─── Task 3 (refino-ciclo2): margem mínima de combustível ──────────────

    /// `requisitos_teste()` fixa `min_fuel_margin_fraction = 0.08` (8%).
    /// Sobrescreve `mission.fuel_total_l` e passa uma capacidade de tanque
    /// sintética diretamente (não a de `config_teste()`) para isolar
    /// exatamente a checagem #18 — mesmo padrão de `electrical_spec()`
    /// acima, que constrói valores sintéticos em vez de depender do
    /// resultado real dos demais agentes. 100 L de capacidade − 95 L
    /// exigidos = 5 L de margem = 5% da capacidade, abaixo do piso de 8%.
    #[test]
    fn violacao_de_margem_de_combustivel_aparece_quando_abaixo_do_minimo() {
        let (req, wing, prop, engine, wb, propeller, perf, mut mission, electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        mission.fuel_total_l = 95.0;
        let fuel_capacity_l = 100.0;

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear,
                                                 &gear_cfg, fuel_capacity_l, &robustness, &prop_cfg));

        assert!(report.violations.iter().any(|v| v.contains("Margem de combustível")),
            "esperava violação de margem de combustível, obteve: {:?}", report.violations);
        // A mensagem deve citar a margem observada (5.00%) e o piso da missão (8.0%).
        assert!(report.violations.iter().any(|v| v.contains("5.00%") && v.contains("8.0%")),
            "violação deveria citar a margem observada (5.00%) e o piso (8.0%): {:?}",
            report.violations);
    }

    /// Sanidade inversa: 100 L de capacidade − 70 L exigidos = 30 L de
    /// margem = 30% da capacidade, acima do piso de 8% — nenhuma violação.
    #[test]
    fn sem_violacao_de_margem_de_combustivel_quando_acima_do_minimo() {
        let (req, wing, prop, engine, wb, propeller, perf, mut mission, electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        mission.fuel_total_l = 70.0;
        let fuel_capacity_l = 100.0;

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear,
                                                 &gear_cfg, fuel_capacity_l, &robustness, &prop_cfg));

        assert!(!report.violations.iter().any(|v| v.contains("Margem de combustível")),
            "não deveria haver violação de margem de combustível, obteve: {:?}", report.violations);
    }

    // Regressão contra o baseline REAL (motor real + missão de projeto
    // completa) — deliberadamente NÃO fica aqui: um teste que carrega um
    // TOML de motor de catálogo por caminho literal citaria o nome do
    // motor dentro de `src/` e quebraria
    // `tests/acceptance.rs::src_nao_contem_nomes_de_motor_especificos`
    // (genericidade motor-agnóstica). Ver
    // `tests/gear_tipback.rs::margem_de_combustivel_do_baseline_real_fica_
    // acima_do_piso_pin_honesto` (pós-campanha E7, 2026-08-06 — antes disso
    // a margem real ficava ABAIXO do piso, achado honesto FAIL do baseline
    // E6, ver histórico no bloco `min_fuel_margin_fraction` de
    // `config/missions/default.toml`).

    // ─── Ciclo 4 (task robustez): checagem #19 — robustez à incerteza ──────

    /// #19: `RobustnessSpec` SINTÉTICO com um único flip injetado gera
    /// EXATAMENTE uma violação começando com "Robustez:" e citando o check,
    /// o caso (dianteiro/traseiro) e os números (valor/limite) do flip;
    /// `RobustnessSpec` com `flips` vazio não gera nenhuma violação desta
    /// checagem — isola a lógica de `verify` da lógica de
    /// `RobustnessAgent::run` (já coberta por `validation::robustness::
    /// tests`), sem depender do pipeline real convergir de um jeito
    /// específico.
    #[test]
    fn check_19_transforma_flips_em_violacoes_nomeadas() {
        let (req, wing, prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness_nominal) = setup();

        // Caso 1: flips vazio (RobustnessSpec nominal já vem assim para a
        // fixture sintética íntegra, mas explicitamos por clareza/robustez
        // do teste a mudanças futuras na fixture) — zero violações #19.
        let robustness_sem_flips = RobustnessSpec {
            flips: Vec::new(),
            ..robustness_nominal.clone()
        };
        let report_sem_flips = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                           &propeller, &perf, &mission, &electrical, &gear,
                                                           &gear_cfg, 220.0, &robustness_sem_flips, &prop_cfg));
        assert!(!report_sem_flips.violations.iter().any(|v| v.starts_with("Robustez:")),
            "RobustnessSpec com flips vazio não deveria gerar nenhuma violação #19, obteve: {:?}",
            report_sem_flips.violations);

        // Caso 2: um único flip sintético injetado — exatamente uma
        // violação "Robustez:", citando check/caso/valor/limite.
        let robustness_com_flip = RobustnessSpec {
            sigma_mass_fraction: 0.15,
            flips: vec![RobustnessFlip {
                check: "Tipback".to_string(),
                caso: "traseiro".to_string(),
                valor: 12.34,
                limite: 15.0,
                // Régua de config, invariante à perturbação (ciclo 10,
                // task 2 — ver `RobustnessFlip::limite_nominal`).
                limite_nominal: 15.0,
            }],
            ..robustness_nominal
        };
        let report_com_flip = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                          &propeller, &perf, &mission, &electrical, &gear,
                                                          &gear_cfg, 220.0, &robustness_com_flip, &prop_cfg));

        let violacoes_robustez: Vec<&String> = report_com_flip.violations.iter()
            .filter(|v| v.starts_with("Robustez:"))
            .collect();
        assert_eq!(violacoes_robustez.len(), 1,
            "esperava EXATAMENTE uma violação #19 para um único flip injetado: {:?}",
            report_com_flip.violations);
        let v = violacoes_robustez[0];
        assert!(v.contains("Tipback"), "violação deveria citar o check do flip ('Tipback'): {v}");
        assert!(v.contains("traseiro"), "violação deveria citar o caso do flip ('traseiro'): {v}");
        assert!(v.contains("12.34"), "violação deveria citar o valor do flip (12.34): {v}");
        assert!(v.contains("15.00") || v.contains("15.0"),
            "violação deveria citar o limite do flip (15.0): {v}");
        assert!(v.contains("15.0%"), "violação deveria citar σ formatado sem arredondamento \
            enganoso (±15.0%, {{:.1}} em vez de {{:.0}} — 0.125 arredondaria para \"13%\" com \
            {{:.0}}): {v}");
    }

    // ─── Ciclo 5 (task robustez-total-e-solo): checagem #20 — atuador de
    // retração vs orçamento elétrico ──────────────────────────────────────
    //
    // Substitui a guarda de parse-time removida no ciclo 3
    // (`models::config::validate_aircraft` — "carga elétrica 'trem_retratil'
    // peak_w >= potência mecânica do atuador de retração"): a massa da
    // perna do trem virou COMPUTADA (`agents::mass_model`), então a
    // comparação só é possível PÓS-convergência — aqui, comparando o pico
    // DECLARADO em `[[electrical.loads]]` (`ElectricalSpec::loads`, novo
    // nesta task) contra a potência COMPUTADA (`GearSpec::actuator_power_w`).

    /// #20: peak_w declarado da carga 'trem_retratil' menor que a potência
    /// COMPUTADA do atuador → violação nomeando os dois valores.
    #[test]
    fn check_20_reprova_peak_w_declarado_menor_que_atuador_computado() {
        let (req, wing, prop, engine, wb, propeller, perf, mission, mut electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        let carga = electrical.loads.iter_mut().find(|l| l.name == GEAR_ACTUATOR_LOAD_NAME)
            .expect("pré-condição do teste: fixture sintética deveria ter a carga 'trem_retratil'");
        carga.peak_w = 1.0;
        assert!(gear.actuator_power_w > 1.0,
            "pré-condição do teste: potência computada do atuador ({:.4} W) deveria ser \
             maior que o peak_w declarado sintético (1.0 W) para exercitar a violação",
            gear.actuator_power_w);

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear,
                                                 &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(report.violations.iter().any(|v|
                v.contains(GEAR_ACTUATOR_LOAD_NAME) && v.contains("1.0")
                && v.contains(&format!("{:.1}", gear.actuator_power_w))),
            "esperava violação citando 'trem_retratil', o peak_w declarado (1.0) e a potência \
             computada do atuador ({:.1}): {:?}", gear.actuator_power_w, report.violations);
    }

    /// #20: aeronave de trem retrátil SEM carga 'trem_retratil' declarada →
    /// violação (cobertura que morreu no ciclo 3 volta, agora
    /// pós-convergência).
    #[test]
    fn check_20_reprova_carga_trem_retratil_ausente() {
        let (req, wing, prop, engine, wb, propeller, perf, mission, mut electrical, gear, gear_cfg, prop_cfg, robustness) = setup();
        let tinha_a_carga = electrical.loads.iter().any(|l| l.name == GEAR_ACTUATOR_LOAD_NAME);
        assert!(tinha_a_carga,
            "pré-condição do teste: fixture sintética deveria ter a carga 'trem_retratil' antes \
             da remoção");
        electrical.loads.retain(|l| l.name != GEAR_ACTUATOR_LOAD_NAME);

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear,
                                                 &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(report.violations.iter().any(|v|
                v.contains(GEAR_ACTUATOR_LOAD_NAME) && v.contains("ausente")),
            "esperava violação citando a ausência da carga 'trem_retratil', obteve: {:?}",
            report.violations);
    }

    /// Gate de retrátil (achado de review, ciclo 5, Minor 5): com
    /// `gear_cfg.retractable = false`, a checagem #20 NÃO deve disparar
    /// mesmo sem nenhuma carga 'trem_retratil' declarada — uma aeronave de
    /// trem FIXO não tem atuador elétrico de retração, então exigir essa
    /// carga seria um falso positivo.
    #[test]
    fn check_20_nao_dispara_com_trem_fixo_mesmo_sem_carga_declarada() {
        let (req, wing, prop, engine, wb, propeller, perf, mission, mut electrical, gear, mut gear_cfg, prop_cfg, robustness) = setup();
        gear_cfg.retractable = false;
        electrical.loads.retain(|l| l.name != GEAR_ACTUATOR_LOAD_NAME);

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear,
                                                 &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(!report.violations.iter().any(|v| v.contains(GEAR_ACTUATOR_LOAD_NAME)),
            "trem FIXO (retractable=false) não deveria exigir a carga 'trem_retratil', mesmo \
             ausente: {:?}", report.violations);
    }

    /// Caminho PASS: fixture intacta (peak_w sintético 480 W do
    /// `config_teste()` fica bem acima da potência mecânica computada do
    /// atuador, dezenas de W) — nenhuma violação da checagem #20.
    #[test]
    fn check_20_passa_na_fixture_intacta() {
        let (req, wing, prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg, prop_cfg, robustness) = setup();

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear,
                                                 &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(!report.violations.iter().any(|v| v.contains("atuador")),
            "fixture intacta (peak_w declarado bem acima do computado) não deveria gerar \
             nenhuma violação da checagem #20, obteve: {:?}", report.violations);
    }

    // ─── Ciclo 8, task 2: folga de hélice em condição CRÍTICA (CS 23.925,
    // checagem #25) ───────────────────────────────────────────────────────

    /// Ramo de VIOLAÇÃO — override SINTÉTICO direto de
    /// `propeller.prop_clearance_critical_m` (mesmo padrão de
    /// `violacao_de_helice_aparece_quando_algum_ok_e_falso` acima, que
    /// sobrescreve `ok_mach_static`/`tip_mach_static` diretamente): a
    /// checagem lê o campo PRECOMPUTADO, não recalcula a partir de
    /// `ground_clearance_m`/`nose_oleo_stroke_mm`/`static_sag_fraction`/
    /// `tire_deflation_delta_m`/`fator` — o `debug_assert!` novo do check
    /// #25 (achado de review, ciclo 8; fórmula com fator desde o ciclo 9;
    /// curso RESTANTE do nariz desde o ciclo 10) exige que o campo bata
    /// com a fórmula fechada desses termos (guarda contra
    /// `fill_critical_clearance` esquecido), então também sobrescrevemos
    /// `ground_clearance_m` para manter a fixture internamente consistente
    /// — `nose_oleo_stroke_mm`/`static_sag_fraction`/`tire_deflation_delta_m`
    /// (vindos de `gear`/`gear_cfg` da fixture) e `fator` (de
    /// `gear_cfg`/`prop_cfg` da fixture) continuam intocados, só narram a
    /// mensagem.
    #[test]
    fn check_25_violacao_de_folga_critica_aparece_com_override_sintetico() {
        let (req, wing, prop, engine, wb, mut propeller, perf, mission, electrical, gear,
             gear_cfg, prop_cfg, robustness) = setup();
        let fator = (gear_cfg.x_main_m - prop_cfg.prop_plane_x_m)
            / (gear_cfg.x_main_m - gear_cfg.x_nose_m);
        let curso_restante_nariz_m = (gear.nose_oleo_stroke_mm / 1_000.0)
            * (1.0 - gear_cfg.static_sag_fraction);
        let folga_critica_alvo = -0.012;
        propeller.ground_clearance_m = folga_critica_alvo
            + (curso_restante_nariz_m + gear_cfg.tire_deflation_delta_m) * fator;
        propeller.prop_clearance_critical_m = folga_critica_alvo;

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear,
                                                 &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(report.violations.iter().any(|v| v.contains("condição crítica CS 23.925")),
            "esperava violação de folga crítica CS 23.925, obteve: {:?}", report.violations);
        assert!(report.violations.iter().any(|v| v.contains("-0.012")),
            "violação deveria citar a folga crítica observada: {:?}", report.violations);
    }

    /// Ramo PASS — a fixture sintética padrão (`config_teste()`, folga
    /// estática 0,200 m, curso de nariz do baseline sintético ≈127,46 mm,
    /// pneu murcho 0,035 m, fator ≈1,16667 — ver `fill_critical_clearance`)
    /// já produz `prop_clearance_critical_m > 0` NATURALMENTE (não
    /// forçado) — mesmo espírito do achado natural de
    /// `violacao_de_folga_de_solo_aparece_naturalmente_na_fixture_sintetica`
    /// acima, mas no sentido inverso (aqui a fixture passa, não falha).
    /// Ciclo 9 (old→new): margem antiga (fator implícito 1) era ≈+0,0225 m
    /// — `prop_plane_x_m` (0,95) e `tire_deflation_delta_m` (0,05→0,035) da
    /// fixture foram ajustados para preservar uma margem POSITIVA sob o
    /// fator novo, sem tocar em `diameter_m`/`h_cg_ground_m`/
    /// `prop_axis_above_cg_m`/`x_main_m`/`x_nose_m` (load-bearing para
    /// outros testes deste arquivo, ex. a checagem #10 acima). Ciclo 10,
    /// task 1: campo novo `static_sag_fraction` (0,40, DISTINTO do
    /// baseline real) só ENCOLHE o termo do nariz (curso RESTANTE < curso
    /// TOTAL) — a margem positiva desta fixture só cresce, não foi preciso
    /// reajustar nenhum outro valor.
    #[test]
    fn check_25_sem_violacao_na_fixture_padrao() {
        let (req, wing, prop, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg,
             prop_cfg, robustness) = setup();
        assert!(propeller.prop_clearance_critical_m > 0.0,
            "pré-condição do teste: fixture sintética deveria passar a folga crítica \
             NATURALMENTE — obtido prop_clearance_critical_m={:.4}",
            propeller.prop_clearance_critical_m);

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear,
                                                 &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(!report.violations.iter().any(|v| v.contains("condição crítica CS 23.925")),
            "não deveria haver violação de folga crítica CS 23.925, obteve: {:?}",
            report.violations);
    }

    /// Ramo PASS explícito no limite: `prop_clearance_critical_m` positivo
    /// mas bem pequeno não deveria violar — só `<= 0.0` viola (semântica
    /// consistente com os demais pisos/tetos deste arquivo). Também
    /// sobrescreve `ground_clearance_m` para satisfazer o `debug_assert!`
    /// novo do check #25 (ver docstring do teste de violação acima).
    #[test]
    fn check_25_sem_violacao_quando_folga_critica_positiva_forcada() {
        let (req, wing, prop, engine, wb, mut propeller, perf, mission, electrical, gear,
             gear_cfg, prop_cfg, robustness) = setup();
        let fator = (gear_cfg.x_main_m - prop_cfg.prop_plane_x_m)
            / (gear_cfg.x_main_m - gear_cfg.x_nose_m);
        let curso_restante_nariz_m = (gear.nose_oleo_stroke_mm / 1_000.0)
            * (1.0 - gear_cfg.static_sag_fraction);
        let folga_critica_alvo = 0.001;
        propeller.ground_clearance_m = folga_critica_alvo
            + (curso_restante_nariz_m + gear_cfg.tire_deflation_delta_m) * fator;
        propeller.prop_clearance_critical_m = folga_critica_alvo;

        let report = ConstraintChecker::verify(&inputs(&req, &wing, &prop, 1_500.0, &engine, &wb,
                                                 &propeller, &perf, &mission, &electrical, &gear,
                                                 &gear_cfg, 220.0, &robustness, &prop_cfg));

        assert!(!report.violations.iter().any(|v| v.contains("condição crítica CS 23.925")),
            "folga crítica positiva (0.001 m) não deveria violar: {:?}", report.violations);
    }
}
