/// PropellerAgent — Dimensionamento/Validação da Hélice (Task 4.5)
///
/// Verifica duas restrições físicas clássicas de hélice de passo variável:
///
///   1. **Mach de ponta de pá** — a velocidade resultante na ponta da pá
///      (tangencial, ou tangencial+avanço em cruzeiro) não pode se aproximar
///      demais da velocidade do som local, sob pena de perda severa de
///      eficiência/ruído por efeitos de compressibilidade. Verificado em DUAS
///      condições:
///        - ESTÁTICA: rpm nominal do motor via PSRU, V=0 (pior caso de rpm,
///          tipicamente o mais restritivo).
///        - CRUZEIRO: rpm de cruzeiro (já reduzido pela busca de BSFC do
///          `PropulsionAgent`) combinado vetorialmente (helicoidal) com a
///          velocidade de avanço.
///   2. **Folga de solo** — distância entre a ponta da pá e o solo (CS
///      23.925), com a hélice/trem na configuração estendida.
///
/// Quando `[propeller].diameter_m` está OMITIDO do TOML, o modelo deriva o
/// maior diâmetro que respeita SIMULTANEAMENTE os dois limites de Mach e a
/// folga mínima de solo, com uma margem de segurança de 2 cm (arredondada
/// para baixo ao cm mais próximo) — ver `derive_diameter_m`.
///
/// Referências:
///   - CS 23.925 — Folga de hélice
///   - McCormick, B. "Aerodynamics, Aeronautics, and Flight Mechanics" —
///     Mach de ponta de pá e perda de eficiência por compressibilidade

use crate::agents::aerodynamics::mach_tip;
use crate::models::{
    aircraft_config::AircraftConfig,
    atmosphere::Isa,
    engine::EngineSpec,
    requirements::Requirements,
    specs::{PropellerSpec, PropulsionSpec},
};

/// RPM da hélice na condição ESTÁTICA (rpm nominal do motor via PSRU, sem
/// depender do rpm de cruzeiro escolhido pela busca de BSFC).
pub fn rpm_static(engine_rpm_rated: f64, psru_ratio: f64) -> f64 {
    engine_rpm_rated / psru_ratio
}

/// Diâmetro máximo (m) que respeita o Mach de ponta ESTÁTICO (V=0):
///   M_max = (π·D·n_rps) / a  →  D = M_max·a / (π·n_rps)
///
/// `rpm_static_v`: rotações da hélice por MINUTO (não rps).
pub fn diameter_max_by_mach_static_m(a_ms: f64, mach_max: f64, rpm_static_v: f64) -> f64 {
    let n_rps = rpm_static_v / 60.0;
    mach_max * a_ms / (std::f64::consts::PI * n_rps)
}

/// Diâmetro máximo (m) que respeita o Mach de ponta em CRUZEIRO (composição
/// helicoidal tangencial + avanço):
///   M_max² = (tip² + V²) / a²  →  tip = √((a·M_max)² − V²)  →  D = 60·tip/(π·n_rpm)
///
/// Retorna 0.0 se `V` já excede `a·M_max` sozinha (a aeronave voa mais
/// rápido que o limite de Mach permitiria mesmo com hélice de diâmetro
/// nulo) — evita `sqrt` de negativo.
pub fn diameter_max_by_mach_cruise_m(a_ms: f64, mach_max: f64, v_cruise_ms: f64, rpm_cruise: f64) -> f64 {
    let limit_speed = a_ms * mach_max;
    let radicand = limit_speed * limit_speed - v_cruise_ms * v_cruise_ms;
    if radicand <= 0.0 || rpm_cruise <= 0.0 {
        return 0.0;
    }
    let tip_speed_max = radicand.sqrt();
    60.0 * tip_speed_max / (std::f64::consts::PI * rpm_cruise)
}

/// Diâmetro máximo (m) que respeita a folga mínima de solo — puramente
/// geométrico, não depende de motor/rpm/atmosfera.
pub fn diameter_max_by_clearance_m(shaft_height_m: f64, ground_clearance_min_m: f64) -> f64 {
    2.0 * (shaft_height_m - ground_clearance_min_m)
}

/// Arredonda `d` PARA BAIXO ao centímetro mais próximo (0.01 m) — diâmetros
/// derivados nunca "arredondam para cima" de volta a um valor que viole os
/// limites que motivaram a derivação.
pub fn round_down_cm(d: f64) -> f64 {
    (d * 100.0).floor() / 100.0
}

/// Margem de segurança subtraída dos máximos teóricos antes de arredondar —
/// evita que o diâmetro derivado fique exatamente na borda de um dos limites
/// (folga zero de projeto).
const DERIVE_MARGIN_M: f64 = 0.02;

pub struct PropellerAgent;

impl PropellerAgent {
    /// Executa a verificação/dimensionamento da hélice.
    ///
    /// `prop_spec` é a saída do `PropulsionAgent` já calculada para esta
    /// iteração (fornece `psru_ratio` e `prop_rpm_cruise` — o rpm de
    /// cruzeiro escolhido pela busca de BSFC, necessário para o Mach de
    /// ponta em cruzeiro).
    pub fn run(
        cfg: &AircraftConfig,
        engine: &EngineSpec,
        prop_spec: &PropulsionSpec,
        req: &Requirements,
    ) -> PropellerSpec {
        let pcfg = &cfg.propeller;

        // ── Condição ESTÁTICA (rpm nominal do motor, V=0, no aeródromo) ────
        let n_static_rpm = rpm_static(engine.rpm_rated, prop_spec.psru_ratio);
        let a_static = Isa::speed_of_sound_ms(req.airfield_altitude_m, req.isa_delta_c);
        let d_max_mach_static =
            diameter_max_by_mach_static_m(a_static, pcfg.tip_mach_max_static, n_static_rpm);

        // ── Condição de CRUZEIRO (rpm da busca de BSFC, helicoidal) ────────
        let n_cruise_rpm = prop_spec.prop_rpm_cruise;
        let a_cruise = Isa::speed_of_sound_ms(req.cruise_altitude_m, req.isa_delta_c);
        let v_cruise_ms = req.cruise_speed_min_kmh / 3.6;
        let d_max_mach_cruise = diameter_max_by_mach_cruise_m(
            a_cruise, pcfg.tip_mach_max_cruise, v_cruise_ms, n_cruise_rpm,
        );

        // Diâmetro máximo que respeita AMBOS os limites de Mach.
        let d_max_mach = d_max_mach_static.min(d_max_mach_cruise);
        let d_max_clearance = diameter_max_by_clearance_m(pcfg.shaft_height_m, pcfg.ground_clearance_min_m);

        let (diameter_m, source) = match pcfg.diameter_m {
            Some(d) => (d, "config".to_string()),
            None => {
                let derived = round_down_cm((d_max_mach.min(d_max_clearance) - DERIVE_MARGIN_M).max(0.0));
                (derived, "derivado".to_string())
            }
        };

        let tip_mach_static = mach_tip(diameter_m, n_static_rpm, 0.0, a_static);
        let tip_mach_cruise_helical = mach_tip(diameter_m, n_cruise_rpm, v_cruise_ms, a_cruise);
        let ground_clearance_m = pcfg.shaft_height_m - diameter_m / 2.0;

        PropellerSpec {
            diameter_m,
            blades: pcfg.blades,
            source,
            tip_mach_static,
            tip_mach_cruise_helical,
            ground_clearance_m,
            diameter_max_by_mach_m: d_max_mach,
            diameter_max_by_clearance_m: d_max_clearance,
            ok_mach_static: tip_mach_static <= pcfg.tip_mach_max_static,
            ok_mach_cruise: tip_mach_cruise_helical <= pcfg.tip_mach_max_cruise,
            ok_clearance: ground_clearance_m >= pcfg.ground_clearance_min_m,
        }
    }
}

/// Tolerância (m) acima da qual a divergência entre o diâmetro derivado
/// AUTORITATIVO (`PropellerSpec::diameter_m`, calculado com o `prop_rpm_cruise`
/// REAL da busca de BSFC) e o diâmetro PROVISÓRIO usado para inicializar essa
/// mesma busca (`AircraftState::prop_diameter_m`, só folga de solo — ver
/// `models::aircraft_state::AircraftState::from_config`) passa a ser
/// reportada como aviso — ver `diameter_mismatch_warning`.
pub const DIAMETER_MISMATCH_TOLERANCE_M: f64 = 0.01;

/// Quando `[propeller].diameter_m` está omitido, `AircraftState` usa um
/// diâmetro PROVISÓRIO (só a restrição de folga de solo, calculável sem
/// `EngineSpec`/`Requirements`) para inicializar a busca de rpm de cruzeiro
/// do `PropulsionAgent` — ver a nota de projeto no docstring do módulo.
/// Quando o Mach de ponta (não a folga de solo) é a restrição mais apertada,
/// o diâmetro AUTORITATIVO calculado por `PropellerAgent::run` (que já usa o
/// `prop_rpm_cruise` real) pode divergir desse provisório — e, nesse caso, o
/// rpm/BSFC/consumo escolhidos pela busca (calculados com o diâmetro
/// provisório) não refletem mais o diâmetro finalmente recomendado.
///
/// Retorna `Some(mensagem)` em português quando essa divergência excede
/// `DIAMETER_MISMATCH_TOLERANCE_M` — usada tanto por
/// `ConstraintChecker::verify` (como AVISO, não violação — o resultado
/// continua fisicamente válido, só potencialmente inconsistente entre si)
/// quanto por `main.rs` (impressa na seção do Agente 9), para que as duas
/// saídas fiquem alinhadas. Retorna `None` quando `source == "config"`
/// (não há provisório envolvido) ou quando a divergência está dentro da
/// tolerância (tipicamente o caso em que a folga de solo — não o Mach —
/// governa: o provisório já usa exatamente a mesma fórmula de margem/
/// arredondamento do caminho autoritativo nesse cenário, então os dois
/// valores coincidem bit-a-bit).
pub fn diameter_mismatch_warning(propeller: &PropellerSpec, prop: &PropulsionSpec) -> Option<String> {
    if propeller.source != "derivado" {
        return None;
    }
    let delta = (propeller.diameter_m - prop.prop_diameter_m).abs();
    if delta <= DIAMETER_MISMATCH_TOLERANCE_M {
        return None;
    }
    Some(format!(
        "Diâmetro de hélice derivado ({:.2} m) difere do provisório usado na busca de \
         cruzeiro ({:.2} m) — fixe [propeller] diameter_m = {:.2} e re-rode para \
         resultados consistentes",
        propeller.diameter_m, prop.prop_diameter_m, propeller.diameter_m
    ))
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::aircraft_config::test_fixtures::config_teste;
    use crate::models::engine::test_fixtures::motor_generico_teste as engine_teste;
    use crate::models::requirements::test_fixtures::requisitos_teste;

    /// `PropulsionSpec` sintética mínima — só os campos que `PropellerAgent`
    /// lê (`psru_ratio`, `prop_rpm_cruise`); os demais campos preenchidos
    /// com valores plausíveis, não usados pelo agente sob teste.
    fn prop_spec_teste(psru_ratio: f64, prop_rpm_cruise: f64) -> PropulsionSpec {
        PropulsionSpec {
            engine_model: "Motor Teste".to_string(),
            power_hp: 200.0,
            power_kw: 149.0,
            max_torque_nm: 440.0,
            rated_rpm: 3_200.0,
            engine_mass_kg: 150.0,
            psru_ratio,
            engine_rpm_cruise: 2_000.0,
            prop_rpm_cruise,
            prop_diameter_m: 1.90,
            fuel_type: "Diesel".to_string(),
            fuel_capacity_l: 220.0,
            fc_cruise_lph: 25.0,
            bsfc_cruise_gkwh: 220.0,
            endurance_h: 8.0,
            range_km: 2_200.0,
            prop_efficiency: 0.82,
            thrust_cruise_n: 1_800.0,
            p_req_cruise_kw: 90.0,
            p_shaft_cruise_kw: 100.0,
            cruise_feasible: true,
        }
    }

    // ─── Hand-check baseline (Task 4.5, controller) ─────────────────────────
    //
    // D=1.95m, PSRU=1.867, motor rpm_rated=3.400 (Toyota real):
    //   rpm_static = 3.400/1.867 = 1.821,1 rpm → n_rps=30,35 → tip=185,9 m/s
    //   a(0,0)=340,3 m/s → M_static = 185,9/340,3 = 0,546
    //   prop_rpm_cruise = 2.640/1.867 = 1.414,0 rpm → n_rps=23,57 → tip=144,4 m/s
    //   V=77,78 m/s → helical=164,0 m/s; a(2500,0)=330,6 → M_cruise=0,496
    //   clearance = 1,25 − 0,975 = 0,275 ≥ 0,23 ✓
    #[test]
    fn hand_check_baseline_toyota() {
        let mut cfg = config_teste();
        cfg.propeller.diameter_m = Some(1.95);
        cfg.propeller.psru_ratio = 1.867;
        cfg.propeller.shaft_height_m = 1.25;
        cfg.propeller.tip_mach_max_static = 0.85;
        cfg.propeller.tip_mach_max_cruise = 0.80;
        cfg.propeller.ground_clearance_min_m = 0.23;

        let mut engine = engine_teste();
        engine.rpm_rated = 3_400.0;

        let mut req = requisitos_teste();
        req.cruise_speed_min_kmh = 280.0;
        req.cruise_altitude_m = 2_500.0;
        req.airfield_altitude_m = 0.0;
        req.isa_delta_c = 0.0;

        let prop_spec = prop_spec_teste(1.867, 2_640.0 / 1.867);

        let spec = PropellerAgent::run(&cfg, &engine, &prop_spec, &req);
        println!(
            "M_static={:.4} M_cruise={:.4} clearance={:.4}",
            spec.tip_mach_static, spec.tip_mach_cruise_helical, spec.ground_clearance_m
        );

        assert!((spec.tip_mach_static - 0.546).abs() < 0.005,
            "M_static = {:.4} (esperado 0.546 ±0.005)", spec.tip_mach_static);
        assert!((spec.tip_mach_cruise_helical - 0.496).abs() < 0.005,
            "M_cruise = {:.4} (esperado 0.496 ±0.005)", spec.tip_mach_cruise_helical);
        assert!((spec.ground_clearance_m - 0.275).abs() < 1e-9,
            "clearance = {:.6} (esperado 0.275 exato)", spec.ground_clearance_m);
        assert_eq!(spec.source, "config");
        assert!(spec.ok_mach_static);
        assert!(spec.ok_mach_cruise);
        assert!(spec.ok_clearance);
    }

    // ─── Negativo: PSRU 1:1 estoura o Mach de ponta estático ────────────────
    #[test]
    fn psru_1_para_1_estoura_mach_estatico() {
        let mut cfg = config_teste();
        cfg.propeller.diameter_m = Some(1.90);
        cfg.propeller.psru_ratio = 1.0;
        cfg.propeller.tip_mach_max_static = 0.83;

        let mut engine = engine_teste();
        engine.rpm_rated = 3_200.0;

        let req = requisitos_teste();
        let prop_spec = prop_spec_teste(1.0, 1_500.0);

        let spec = PropellerAgent::run(&cfg, &engine, &prop_spec, &req);
        println!("M_static (PSRU 1:1) = {:.4}", spec.tip_mach_static);

        assert!(spec.tip_mach_static > 0.83,
            "M_static {:.4} deveria exceder o limite 0.83 com PSRU 1:1", spec.tip_mach_static);
        assert!(!spec.ok_mach_static);
    }

    // ─── Diâmetro derivado quando config omite ──────────────────────────────
    #[test]
    fn diametro_derivado_respeita_ambos_os_maximos_com_margem() {
        let mut cfg = config_teste();
        cfg.propeller.diameter_m = None; // omitido — deriva

        let engine = engine_teste();
        let req = requisitos_teste();
        let prop_spec = prop_spec_teste(cfg.propeller.psru_ratio, 1_200.0);

        let spec = PropellerAgent::run(&cfg, &engine, &prop_spec, &req);
        println!(
            "derivado: D={:.4} d_max_mach={:.4} d_max_clearance={:.4}",
            spec.diameter_m, spec.diameter_max_by_mach_m, spec.diameter_max_by_clearance_m
        );

        assert_eq!(spec.source, "derivado");
        let menor_maximo = spec.diameter_max_by_mach_m.min(spec.diameter_max_by_clearance_m);
        assert!(spec.diameter_m <= spec.diameter_max_by_mach_m + 1e-9);
        assert!(spec.diameter_m <= spec.diameter_max_by_clearance_m + 1e-9);
        assert!(spec.diameter_m >= menor_maximo - 0.04,
            "diâmetro derivado {:.4} deveria estar dentro de 4 cm do menor máximo {:.4}",
            spec.diameter_m, menor_maximo);
        // Autoconsistente: com o diâmetro derivado (que já embute a margem),
        // as checagens de Mach/folga devem passar.
        assert!(spec.ok_mach_static);
        assert!(spec.ok_mach_cruise);
        assert!(spec.ok_clearance);
    }

    /// Reproduz a mesma fórmula que `AircraftState::from_config` usa para o
    /// diâmetro PROVISÓRIO quando `[propeller].diameter_m` está omitido —
    /// duplicada aqui deliberadamente (não importada de `models::aircraft_state`)
    /// para que este teste também sirva de sentinela: se a fórmula de lá
    /// divergir desta cópia, os testes de `diameter_mismatch_warning` abaixo
    /// deixam de ser representativos do comportamento real.
    fn bootstrap_diameter_m(shaft_height_m: f64, ground_clearance_min_m: f64) -> f64 {
        round_down_cm(diameter_max_by_clearance_m(shaft_height_m, ground_clearance_min_m) - DERIVE_MARGIN_M)
    }

    // ─── Aviso de divergência: diâmetro derivado × provisório (mitigação) ──

    /// Quando o Mach de ponta (não a folga de solo) é a restrição mais
    /// apertada, o diâmetro AUTORITATIVO (calculado com o `prop_rpm_cruise`
    /// real) fica bem menor que o PROVISÓRIO (só folga de solo, usado para
    /// inicializar a busca de cruzeiro) — a divergência deve disparar aviso.
    #[test]
    fn aviso_dispara_quando_mach_governa_o_diametro_derivado() {
        let mut cfg = config_teste();
        cfg.propeller.diameter_m = None; // omitido — deriva
        cfg.propeller.psru_ratio = 1.0;  // PSRU 1:1 — rpm de hélice bem mais alto

        let mut engine = engine_teste();
        engine.rpm_rated = 3_200.0;

        let req = requisitos_teste();
        // rpm de cruzeiro também bem mais alto que o caso psru=2.0 normal —
        // reforça que o Mach (não a folga) governa em ambas as condições.
        let prop_spec = prop_spec_teste(1.0, 2_000.0);

        let spec = PropellerAgent::run(&cfg, &engine, &prop_spec, &req);
        let bootstrap = bootstrap_diameter_m(cfg.propeller.shaft_height_m, cfg.propeller.ground_clearance_min_m);
        println!(
            "mach governa: D_autoritativo={:.4} D_provisorio={:.4} (Δ={:.4})",
            spec.diameter_m, bootstrap, (spec.diameter_m - bootstrap).abs()
        );

        assert_eq!(spec.source, "derivado");
        assert!(spec.diameter_m < bootstrap - DIAMETER_MISMATCH_TOLERANCE_M,
            "pré-condição do teste: diâmetro autoritativo ({:.4}) deveria ficar bem abaixo do \
             provisório ({:.4}) quando o Mach governa", spec.diameter_m, bootstrap);

        // `prop_spec` simula o `PropulsionSpec` real: `prop_diameter_m` é o
        // que `AircraftState`/`PropulsionAgent` de fato usaram (o provisório).
        let mut prop_spec_real = prop_spec.clone();
        prop_spec_real.prop_diameter_m = bootstrap;

        let aviso = diameter_mismatch_warning(&spec, &prop_spec_real);
        assert!(aviso.is_some(), "esperava aviso de divergência de diâmetro, obteve None");
        let msg = aviso.unwrap();
        assert!(msg.contains("Diâmetro de hélice derivado"), "{msg}");
        assert!(msg.contains(&format!("{:.2}", spec.diameter_m)), "{msg}");
        assert!(msg.contains(&format!("{:.2}", bootstrap)), "{msg}");
    }

    /// Quando a folga de solo (não o Mach) governa, o provisório usa
    /// EXATAMENTE a mesma fórmula (folga − margem, arredondada para baixo)
    /// que o caminho autoritativo aplica nesse cenário — os dois valores
    /// devem coincidir e nenhum aviso deve disparar.
    #[test]
    fn sem_aviso_quando_folga_governa_o_diametro_derivado() {
        let mut cfg = config_teste();
        cfg.propeller.diameter_m = None; // omitido — deriva (folga governa,
                                          // ver `diametro_derivado_respeita_ambos_os_maximos_com_margem`)

        let engine = engine_teste();
        let req = requisitos_teste();
        let prop_spec = prop_spec_teste(cfg.propeller.psru_ratio, 1_200.0);

        let spec = PropellerAgent::run(&cfg, &engine, &prop_spec, &req);
        let bootstrap = bootstrap_diameter_m(cfg.propeller.shaft_height_m, cfg.propeller.ground_clearance_min_m);
        println!(
            "folga governa: D_autoritativo={:.4} D_provisorio={:.4}",
            spec.diameter_m, bootstrap
        );

        assert_eq!(spec.source, "derivado");
        assert!((spec.diameter_m - bootstrap).abs() < 1e-9,
            "quando a folga governa, autoritativo ({:.4}) e provisório ({:.4}) deveriam \
             coincidir exatamente (mesma fórmula)", spec.diameter_m, bootstrap);

        let mut prop_spec_real = prop_spec.clone();
        prop_spec_real.prop_diameter_m = bootstrap;

        let aviso = diameter_mismatch_warning(&spec, &prop_spec_real);
        assert!(aviso.is_none(), "não deveria haver aviso quando a folga governa, obteve: {aviso:?}");
    }

    #[test]
    fn sem_aviso_quando_diametro_vem_da_config() {
        let cfg = config_teste(); // diameter_m = Some(1.90) — source "config"
        let engine = engine_teste();
        let req = requisitos_teste();
        let prop_spec = prop_spec_teste(cfg.propeller.psru_ratio, 1_200.0);

        let spec = PropellerAgent::run(&cfg, &engine, &prop_spec, &req);
        assert_eq!(spec.source, "config");

        let aviso = diameter_mismatch_warning(&spec, &prop_spec);
        assert!(aviso.is_none(), "não deveria haver aviso quando diameter_m vem da config, obteve: {aviso:?}");
    }

    // ─── Propriedade: mais altura de eixo → mais folga de solo ──────────────
    #[test]
    fn folga_de_solo_cresce_com_altura_do_eixo() {
        let engine = engine_teste();
        let req = requisitos_teste();
        let prop_spec = prop_spec_teste(2.0, 1_200.0);

        let mut cfg_baixo = config_teste();
        cfg_baixo.propeller.diameter_m = Some(1.90);
        cfg_baixo.propeller.shaft_height_m = 1.10;

        let mut cfg_alto = config_teste();
        cfg_alto.propeller.diameter_m = Some(1.90);
        cfg_alto.propeller.shaft_height_m = 1.30;

        let spec_baixo = PropellerAgent::run(&cfg_baixo, &engine, &prop_spec, &req);
        let spec_alto = PropellerAgent::run(&cfg_alto, &engine, &prop_spec, &req);

        assert!(spec_alto.ground_clearance_m > spec_baixo.ground_clearance_m,
            "folga com eixo mais alto ({:.3}) deveria exceder a de eixo mais baixo ({:.3})",
            spec_alto.ground_clearance_m, spec_baixo.ground_clearance_m);
    }

    #[test]
    fn round_down_cm_nunca_arredonda_para_cima() {
        assert_eq!(round_down_cm(1.9649), 1.96);
        assert_eq!(round_down_cm(1.96), 1.96);
        assert_eq!(round_down_cm(0.0), 0.0);
    }

    #[test]
    fn diameter_max_by_mach_cruise_retorna_zero_se_v_excede_limite() {
        // a=330, M_max=0.5 → limite=165 m/s; V=200 m/s > limite → sem solução real.
        let d = diameter_max_by_mach_cruise_m(330.0, 0.5, 200.0, 1_500.0);
        assert_eq!(d, 0.0);
    }
}
