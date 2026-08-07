//! RobustnessAgent — conjuntos adversariais ±σ (ciclo 4, spec
//! 2026-08-07-ciclo4-fidelidade-massas).
//!
//! As 7 massas estruturais (`agents::mass_model::StructuralMasses`) vêm de
//! equações de componente empíricas (Raymer, "Aircraft Design: A
//! Conceptual Approach", cap. 15.2 — GA) ajustadas a uma FROTA histórica,
//! não a ESTA aeronave: Raymer cap. 15/Roskam ("Airplane Design Part V",
//! Classe II) citam incerteza típica de ±10–20% em projeto conceitual.
//! Este módulo quantifica se checks que PASSAM com as massas NOMINAIS
//! (ponto central das equações) continuariam passando sob essa incerteza
//! — não uma análise probabilística (sem RNG, sem distribuição), mas um
//! PIOR-CASO DETERMINÍSTICO DIRECIONAL: dois conjuntos adversariais, um
//! que empurra o CG vazio o mais para a FRENTE possível (todo componente
//! dianteiro do CG nominal fica ×(1+σ), todo componente traseiro fica
//! ×(1−σ)) e outro que o empurra o mais para TRÁS possível (o oposto) —
//! ver `adversarial_masses`.
//!
//! A classificação dianteiro/traseiro usa o CG VAZIO (`x_cg_oew`) como
//! pivô, não o CG carregado de cada cenário — isso só é um PIOR-CASO EXATO
//! enquanto nenhum braço estrutural cair DENTRO da banda de CG carregado
//! dos cenários (verdadeiro no baseline atual: os 7 braços estruturais vão
//! de 1,40 a 7,40 m, fora da banda de CG carregado de 3,01–3,30 m); se um
//! braço estrutural algum dia cair dentro dessa banda, a classificação por
//! `x_cg_oew` deixa de garantir o pior caso para os cenários cujo CG fica
//! do lado oposto do CG vazio.
//!
//! Consumido por `main.rs` (chamado logo após o `LandingGearAgent`) e por
//! `validation::constraint_checker::ConstraintChecker::verify` (checagem
//! #19 — um `flip` gera uma violação nomeada) desde a Task 4 do ciclo
//! (wiring, schema v4.6) — antes disso o módulo era isolado do pipeline.
//!
//! Os limites contra os quais os conjuntos perturbados são avaliados são
//! os NOMINAIS (`wb_nominal.spec.cg_limit_{fwd,aft}_pct_mac`,
//! `gear_cfg.tipback_min_deg`, os tetos/pisos de carga de nariz de
//! `validation::constraint_checker`) — esses limites são derivados de
//! geometria/margem de estabilidade/autoridade de profundor, não da massa
//! estrutural em si (a massa entra no CG e nas cargas, não no limite),
//! logo são invariantes à perturbação: reavaliá-los para cada conjunto
//! adversarial recalcularia o MESMO número (o `TrimAuthorityAgent`
//! dianteiro/`sm_min` traseiro não dependem de `StructuralMasses`) a um
//! custo de mais uma chamada de agente — por isso as massas perturbadas
//! são comparadas numericamente contra os limites NOMINAIS já calculados,
//! sem re-rodar `TrimAuthorityAgent`.

use crate::agents::landing_gear::LandingGearAgent;
use crate::agents::mass_model::StructuralMasses;
use crate::agents::weight_balance::{
    cg_from_items, oew_items, structural_arms, WeightBalanceAgent, WeightBalanceOutput,
};
use crate::models::aircraft_config::AircraftConfig;
use crate::models::aircraft_state::AircraftState;
use crate::models::engine::EngineSpec;
use crate::models::requirements::Requirements;
use crate::models::specs::{EmpennageSpec, GearSpec, RobustnessFlip, RobustnessSpec, WingSpec};
use crate::validation::constraint_checker::{NOSE_LOAD_MAX_CEILING_PCT, NOSE_LOAD_MIN_FLOOR_PCT};

/// Constrói os 2 conjuntos adversariais de massas estruturais (±σ).
/// Determinístico: classifica cada um dos 7 componentes comparando seu
/// braço de momento (MESMO mapeamento estático componente→braço de
/// `agents::weight_balance::oew_items`) com o CG VAZIO nominal
/// (`cg_from_items(oew_items(...))`) — componentes com braço À FRENTE do
/// CG vazio (`arm <= x_cg_oew`) são "dianteiros"; os demais, "traseiros".
/// Empates (`arm == x_cg_oew`, improvável na prática) vão para o lado
/// dianteiro, por convenção de `<=` (documentado aqui, não coincidência).
///
/// Devolve `(conjunto_cg_mais_dianteiro, conjunto_cg_mais_traseiro)`: no
/// primeiro, todo componente dianteiro fica MAIS pesado (×(1+σ)) e todo
/// componente traseiro fica MAIS leve (×(1−σ)) — o CG vazio resultante se
/// desloca o mais possível para a frente dado σ. O segundo é o espelho
/// exato (dianteiros mais leves, traseiros mais pesados) — desloca o CG
/// vazio o mais possível para trás.
pub fn adversarial_masses(
    cfg: &AircraftConfig,
    engine: &EngineSpec,
    masses: &StructuralMasses,
    sigma: f64,
) -> (StructuralMasses, StructuralMasses) {
    let items = oew_items(cfg, engine, masses);
    let (_, x_cg_oew) = cg_from_items(&items);
    // FONTE ÚNICA do mapeamento componente→braço (ver docstring de
    // `structural_arms`) — MESMA usada por `oew_items` para montar os 7
    // itens estruturais, evitando divergência silenciosa entre os dois.
    let [(_, asa_arm), (_, fuselagem_arm), (_, emp_h_arm), (_, emp_v_arm),
         (_, trem_principal_arm), (_, trem_nariz_arm), (_, tanques_arm)] = structural_arms(cfg);

    // `fwd_heavier`: true monta o conjunto CG-mais-DIANTEIRO (componentes
    // dianteiros ficam mais pesados); false monta o conjunto
    // CG-mais-TRASEIRO (componentes dianteiros ficam mais leves).
    let scale = |mass: f64, arm: f64, fwd_heavier: bool| -> f64 {
        let dianteiro = arm <= x_cg_oew;
        if dianteiro == fwd_heavier { mass * (1.0 + sigma) } else { mass * (1.0 - sigma) }
    };

    let fwd = StructuralMasses {
        asa_kg:            scale(masses.asa_kg,            asa_arm,            true),
        fuselagem_kg:      scale(masses.fuselagem_kg,      fuselagem_arm,      true),
        emp_h_kg:          scale(masses.emp_h_kg,           emp_h_arm,         true),
        emp_v_kg:          scale(masses.emp_v_kg,           emp_v_arm,         true),
        trem_principal_kg: scale(masses.trem_principal_kg,  trem_principal_arm, true),
        trem_nariz_kg:     scale(masses.trem_nariz_kg,      trem_nariz_arm,    true),
        tanques_kg:        scale(masses.tanques_kg,         tanques_arm,       true),
    };
    let aft = StructuralMasses {
        asa_kg:            scale(masses.asa_kg,            asa_arm,            false),
        fuselagem_kg:      scale(masses.fuselagem_kg,      fuselagem_arm,      false),
        emp_h_kg:          scale(masses.emp_h_kg,           emp_h_arm,         false),
        emp_v_kg:          scale(masses.emp_v_kg,           emp_v_arm,         false),
        trem_principal_kg: scale(masses.trem_principal_kg,  trem_principal_arm, false),
        trem_nariz_kg:     scale(masses.trem_nariz_kg,      trem_nariz_arm,    false),
        tanques_kg:        scale(masses.tanques_kg,         tanques_arm,       false),
    };
    (fwd, aft)
}

pub struct RobustnessAgent;

impl RobustnessAgent {
    /// Avalia os dois conjuntos adversariais (`adversarial_masses`, com
    /// `cfg.mass_model.sigma_mass_fraction`) contra os limites NOMINAIS já
    /// calculados (`wb_nominal`/`gear_nominal` — ver docstring do módulo
    /// para o porquê de não reavaliar `TrimAuthorityAgent`). Um `flip` é
    /// registrado por (check, caso) sempre que o conjunto perturbado
    /// REPROVA um check que o NOMINAL passava.
    pub fn run(
        cfg: &AircraftConfig,
        engine: &EngineSpec,
        req: &Requirements,
        state: &AircraftState,
        wing: &WingSpec,
        emp: &EmpennageSpec,
        masses: &StructuralMasses,
        wb_nominal: &WeightBalanceOutput,
        gear_nominal: &GearSpec,
    ) -> RobustnessSpec {
        debug_assert!(wb_nominal.spec.cg_limit_fwd_pct_mac.is_finite(),
            "RobustnessAgent exige um wb NOMINAL já com apply_trim (cg_limit_fwd_pct_mac = NaN)");
        debug_assert!(wb_nominal.spec.cg_limit_aft_pct_mac.is_finite(),
            "RobustnessAgent exige um wb NOMINAL já com apply_trim (cg_limit_aft_pct_mac = NaN)");
        let sigma = cfg.mass_model.sigma_mass_fraction;
        let (m_fwd, m_aft) = adversarial_masses(cfg, engine, masses, sigma);

        // Avalia um conjunto adversarial (`caso` = "dianteiro"/"traseiro")
        // contra os limites nominais: cenários de CG (envelope
        // dianteiro/traseiro) + trem de pouso (tipback, carga de nariz
        // máx/mín) — devolve a faixa de CG observada e os flips
        // encontrados.
        let evaluate_case = |caso: &str, m_p: &StructuralMasses| -> ([f64; 2], Vec<RobustnessFlip>) {
            let mut flips = Vec::new();
            let wb_p = WeightBalanceAgent::run(state, wing, engine, cfg, req, emp, m_p);

            let mut range = [f64::INFINITY, f64::NEG_INFINITY];
            for (sc_nom, sc_p) in wb_nominal.scenarios.iter().zip(wb_p.scenarios.iter()) {
                range[0] = range[0].min(sc_p.cg_pct_mac);
                range[1] = range[1].max(sc_p.cg_pct_mac);

                let dentro_do_envelope_nominal = sc_p.cg_pct_mac >= wb_nominal.spec.cg_limit_fwd_pct_mac
                    && sc_p.cg_pct_mac <= wb_nominal.spec.cg_limit_aft_pct_mac;
                if !dentro_do_envelope_nominal && sc_nom.inside_envelope {
                    let limite = if sc_p.cg_pct_mac < wb_nominal.spec.cg_limit_fwd_pct_mac {
                        wb_nominal.spec.cg_limit_fwd_pct_mac
                    } else {
                        wb_nominal.spec.cg_limit_aft_pct_mac
                    };
                    flips.push(RobustnessFlip {
                        check: format!("Cenário '{}'", sc_nom.name),
                        caso: caso.to_string(),
                        valor: sc_p.cg_pct_mac,
                        limite,
                    });
                }
            }

            let x_fwd_p = cfg.wing.le_root_x_m + wb_p.spec.cg_mac_fwd_pct / 100.0 * wb_p.mac_m;
            let x_aft_p = cfg.wing.le_root_x_m + wb_p.spec.cg_mac_aft_pct / 100.0 * wb_p.mac_m;
            let gear_p = LandingGearAgent::run(
                wb_p.spec.mtow_kg, x_fwd_p, x_aft_p, &cfg.gear,
                m_p.trem_principal_kg, m_p.trem_nariz_kg,
            );

            if gear_p.tipback_angle_deg < cfg.gear.tipback_min_deg
                && gear_nominal.tipback_angle_deg >= cfg.gear.tipback_min_deg
            {
                flips.push(RobustnessFlip {
                    check: "Tipback".to_string(),
                    caso: caso.to_string(),
                    valor: gear_p.tipback_angle_deg,
                    limite: cfg.gear.tipback_min_deg,
                });
            }
            if gear_p.nose_load_max_pct > NOSE_LOAD_MAX_CEILING_PCT
                && gear_nominal.nose_load_max_pct <= NOSE_LOAD_MAX_CEILING_PCT
            {
                flips.push(RobustnessFlip {
                    check: "Carga de nariz máx".to_string(),
                    caso: caso.to_string(),
                    valor: gear_p.nose_load_max_pct,
                    limite: NOSE_LOAD_MAX_CEILING_PCT,
                });
            }
            if gear_p.nose_load_min_pct < NOSE_LOAD_MIN_FLOOR_PCT
                && gear_nominal.nose_load_min_pct >= NOSE_LOAD_MIN_FLOOR_PCT
            {
                flips.push(RobustnessFlip {
                    check: "Carga de nariz mín".to_string(),
                    caso: caso.to_string(),
                    valor: gear_p.nose_load_min_pct,
                    limite: NOSE_LOAD_MIN_FLOOR_PCT,
                });
            }

            (range, flips)
        };

        let (cg_fwd_case_pct_mac, mut flips) = evaluate_case("dianteiro", &m_fwd);
        let (cg_aft_case_pct_mac, flips_traseiro) = evaluate_case("traseiro", &m_aft);
        flips.extend(flips_traseiro);

        RobustnessSpec { sigma_mass_fraction: sigma, cg_fwd_case_pct_mac, cg_aft_case_pct_mac, flips }
    }
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::agents::aerodynamics::AerodynamicsAgent;
    use crate::agents::empennage::EmpennageAgent;
    use crate::agents::mass_model::MassModelAgent;
    use crate::agents::trim_authority::TrimAuthorityAgent;
    use crate::models::aircraft_config::test_fixtures::config_teste;
    use crate::models::engine::test_fixtures::motor_generico_teste;
    use crate::models::requirements::test_fixtures::requisitos_teste;

    /// Pipeline nominal COMPLETO, mesma sequência de agentes de
    /// `validation::constraint_checker` (fixture `setup_with_cfg_and_req`):
    /// Aerodynamics → Empennage → MassModel → WeightBalance →
    /// TrimAuthority/`apply_trim` → LandingGear. MTOW/n_design FIXOS
    /// (1450.0/4.0 — não itera o ponto fixo do orchestrator, este teste
    /// exercita `RobustnessAgent` isoladamente, não a convergência).
    struct Nominal {
        cfg: AircraftConfig,
        engine: EngineSpec,
        req: Requirements,
        state: AircraftState,
        wing: WingSpec,
        emp: EmpennageSpec,
        masses: StructuralMasses,
        wb: WeightBalanceOutput,
        gear: GearSpec,
    }

    fn nominal_pipeline(cfg: AircraftConfig) -> Nominal {
        let req = requisitos_teste();
        let state = AircraftState::from_config(&cfg);
        let wing = AerodynamicsAgent::run(&state, &req);
        let engine = motor_generico_teste();
        let emp = EmpennageAgent::run(&wing, &cfg);
        let masses = MassModelAgent::run(&cfg, &engine, &req, &wing, &emp, 1450.0, 4.0);
        let mut wb = WeightBalanceAgent::run(&state, &wing, &engine, &cfg, &req, &emp, &masses);
        let trim = TrimAuthorityAgent::run(&cfg, &wing, &emp, &wb);
        wb.apply_trim(&trim);
        let x_cg_fwd = cfg.wing.le_root_x_m + wb.spec.cg_mac_fwd_pct / 100.0 * wb.mac_m;
        let x_cg_aft = cfg.wing.le_root_x_m + wb.spec.cg_mac_aft_pct / 100.0 * wb.mac_m;
        let gear = LandingGearAgent::run(
            wb.spec.mtow_kg, x_cg_fwd, x_cg_aft, &cfg.gear,
            masses.trem_principal_kg, masses.trem_nariz_kg,
        );
        Nominal { cfg, engine, req, state, wing, emp, masses, wb, gear }
    }

    /// Classificação direcional: cada um dos 7 componentes entra no lado
    /// certo do conjunto adversarial comparando o braço REAL — achado por
    /// nome na saída de `oew_items`, não uma cópia manual do mapeamento
    /// componente→braço — com o CG VAZIO nominal
    /// (`cg_from_items(oew_items(...))`).
    #[test]
    fn conjuntos_adversariais_perturbam_na_direcao_certa() {
        let n = nominal_pipeline(config_teste());
        let sigma = 0.20;

        let items = oew_items(&n.cfg, &n.engine, &n.masses);
        let (_, x_cg_oew) = cg_from_items(&items);
        println!("x_cg_oew = {x_cg_oew:.4}");

        let (fwd, aft) = adversarial_masses(&n.cfg, &n.engine, &n.masses, sigma);

        // Braço de cada componente vindo da saída REAL de `oew_items`
        // (achado pelo `MassItem::name`) — evita uma 3ª cópia manual do
        // mapeamento componente→braço (fonte única em
        // `agents::weight_balance::structural_arms`, consumida tanto por
        // `oew_items` quanto por `adversarial_masses`); se as duas
        // divergirem, este teste detecta.
        let braco = |nome_item: &str| items.iter()
            .find(|i| i.name == nome_item)
            .unwrap_or_else(|| panic!("oew_items deveria conter o item '{nome_item}'"))
            .arm_m;

        let componentes: [(&str, f64, f64, f64, f64); 7] = [
            ("asa",            n.masses.asa_kg,            braco("asa"),            fwd.asa_kg,            aft.asa_kg),
            ("fuselagem",      n.masses.fuselagem_kg,      braco("fuselagem"),      fwd.fuselagem_kg,      aft.fuselagem_kg),
            ("emp_h",          n.masses.emp_h_kg,          braco("emp_horizontal"), fwd.emp_h_kg,          aft.emp_h_kg),
            ("emp_v",          n.masses.emp_v_kg,          braco("emp_vertical"),   fwd.emp_v_kg,          aft.emp_v_kg),
            ("trem_principal", n.masses.trem_principal_kg, braco("trem_principal"), fwd.trem_principal_kg, aft.trem_principal_kg),
            ("trem_nariz",     n.masses.trem_nariz_kg,     braco("trem_nariz"),     fwd.trem_nariz_kg,     aft.trem_nariz_kg),
            ("tanques",        n.masses.tanques_kg,        braco("tanques"),        fwd.tanques_kg,        aft.tanques_kg),
        ];

        for (nome, massa_nominal, braco, massa_fwd, massa_aft) in componentes {
            let dianteiro = braco <= x_cg_oew;
            println!(
                "{nome}: braço={braco:.4} dianteiro={dianteiro} nominal={massa_nominal:.4} \
                 fwd={massa_fwd:.4} aft={massa_aft:.4}"
            );
            if dianteiro {
                // componente dianteiro: no conjunto CG-mais-dianteiro fica
                // MAIS pesado; no CG-mais-traseiro fica MAIS leve.
                assert!((massa_fwd - massa_nominal * (1.0 + sigma)).abs() < 1e-9,
                    "{nome} (dianteiro) deveria ficar ×(1+σ) no conjunto dianteiro");
                assert!((massa_aft - massa_nominal * (1.0 - sigma)).abs() < 1e-9,
                    "{nome} (dianteiro) deveria ficar ×(1−σ) no conjunto traseiro");
            } else {
                assert!((massa_fwd - massa_nominal * (1.0 - sigma)).abs() < 1e-9,
                    "{nome} (traseiro) deveria ficar ×(1−σ) no conjunto dianteiro");
                assert!((massa_aft - massa_nominal * (1.0 + sigma)).abs() < 1e-9,
                    "{nome} (traseiro) deveria ficar ×(1+σ) no conjunto traseiro");
            }
        }
    }

    /// σ→0 degenera no nominal: flips vazio e faixas de CG iguais às
    /// nominais (tolerância 1e-9) — construção (massas idênticas produzem
    /// `WeightBalanceOutput`/`GearSpec` bit-a-bit idênticos), não
    /// coincidência.
    #[test]
    fn sigma_zero_nao_produz_flips() {
        let mut cfg = config_teste();
        cfg.mass_model.sigma_mass_fraction = 1e-12; // σ efetivamente nulo — construído em memória, portanto não passa pela faixa validada (0.05, 0.30) de parse_aircraft
        let n = nominal_pipeline(cfg);

        let out = RobustnessAgent::run(
            &n.cfg, &n.engine, &n.req, &n.state, &n.wing, &n.emp, &n.masses, &n.wb, &n.gear,
        );

        println!("flips={:?}", out.flips);
        assert!(out.flips.is_empty(), "σ≈0 não deveria produzir flips: {:?}", out.flips);
        assert!((out.cg_fwd_case_pct_mac[0] - n.wb.spec.cg_mac_fwd_pct).abs() < 1e-6,
            "faixa dianteira[0] deveria bater com o nominal: {} vs {}",
            out.cg_fwd_case_pct_mac[0], n.wb.spec.cg_mac_fwd_pct);
        assert!((out.cg_fwd_case_pct_mac[1] - n.wb.spec.cg_mac_aft_pct).abs() < 1e-6,
            "faixa dianteira[1] deveria bater com o nominal: {} vs {}",
            out.cg_fwd_case_pct_mac[1], n.wb.spec.cg_mac_aft_pct);
        assert!((out.cg_aft_case_pct_mac[0] - n.wb.spec.cg_mac_fwd_pct).abs() < 1e-6,
            "faixa traseira[0] deveria bater com o nominal: {} vs {}",
            out.cg_aft_case_pct_mac[0], n.wb.spec.cg_mac_fwd_pct);
        assert!((out.cg_aft_case_pct_mac[1] - n.wb.spec.cg_mac_aft_pct).abs() < 1e-6,
            "faixa traseira[1] deveria bater com o nominal: {} vs {}",
            out.cg_aft_case_pct_mac[1], n.wb.spec.cg_mac_aft_pct);
    }

    /// Config sintética MARGINAL: aperta `gear.tipback_min_deg` até ~0,5°
    /// ABAIXO do tipback nominal (nominal passa por pouco) — com σ=0.20 o
    /// conjunto CG-TRASEIRO empurra o CG mais para trás, reduzindo o
    /// tipback (`θ = atan((x_main−x_cg_aft)/h_cg)`, x_cg_aft maior ⇒ θ
    /// menor) o suficiente para derrubar o check.
    #[test]
    fn config_marginal_gera_flip_nomeado() {
        let n0 = nominal_pipeline(config_teste());
        let theta_nominal = n0.gear.tipback_angle_deg;
        println!("theta_nominal = {theta_nominal:.3}");

        let mut cfg = config_teste();
        cfg.gear.tipback_min_deg = theta_nominal - 0.5;
        let n = nominal_pipeline(cfg);
        assert!(n.gear.tipback_angle_deg >= n.cfg.gear.tipback_min_deg,
            "pré-condição do teste: tipback nominal ({:.3}) deveria passar por pouco o piso \
             marginal ({:.3})", n.gear.tipback_angle_deg, n.cfg.gear.tipback_min_deg);

        let out = RobustnessAgent::run(
            &n.cfg, &n.engine, &n.req, &n.state, &n.wing, &n.emp, &n.masses, &n.wb, &n.gear,
        );
        println!("flips={:?}", out.flips);

        assert_eq!(out.flips.len(), 1,
            "esperava exatamente 1 flip (Tipback/traseiro): {:?}", out.flips);
        let flip = &out.flips[0];
        assert_eq!(flip.check, "Tipback");
        assert_eq!(flip.caso, "traseiro");
        assert!(flip.valor < flip.limite,
            "valor ({}) deveria estar abaixo do limite ({}) — é isso que caracteriza o flip",
            flip.valor, flip.limite);
        assert!((flip.limite - n.cfg.gear.tipback_min_deg).abs() < 1e-9);
    }

    /// Fixture intacta (`config_teste()`, σ=0.20 da fixture): saída
    /// bem-formada — faixas de CG do caso dianteiro À FRENTE das nominais
    /// e do caso traseiro ATRÁS (desigualdade estrita: perturbar 7
    /// componentes em ±20% desloca o CG vazio o suficiente para mover
    /// TODOS os cenários), flips só contêm checks que passam no nominal.
    #[test]
    fn casos_adversariais_movem_o_cg_nas_duas_direcoes() {
        let n = nominal_pipeline(config_teste());
        assert!((n.cfg.mass_model.sigma_mass_fraction - 0.20).abs() < 1e-9,
            "pré-condição: fixture deveria ter sigma_mass_fraction=0.20");

        let out = RobustnessAgent::run(
            &n.cfg, &n.engine, &n.req, &n.state, &n.wing, &n.emp, &n.masses, &n.wb, &n.gear,
        );
        println!("nominal fwd/aft = {:.3}/{:.3}  caso dianteiro = {:?}  caso traseiro = {:?}",
            n.wb.spec.cg_mac_fwd_pct, n.wb.spec.cg_mac_aft_pct,
            out.cg_fwd_case_pct_mac, out.cg_aft_case_pct_mac);

        // Caso dianteiro: os dois extremos ficam estritamente À FRENTE
        // (menor %MAC) dos extremos nominais correspondentes.
        assert!(out.cg_fwd_case_pct_mac[0] < n.wb.spec.cg_mac_fwd_pct,
            "extremo dianteiro do caso dianteiro deveria ficar à frente do nominal");
        assert!(out.cg_fwd_case_pct_mac[1] < n.wb.spec.cg_mac_aft_pct,
            "extremo traseiro do caso dianteiro deveria ficar à frente do nominal");

        // Caso traseiro: os dois extremos ficam estritamente ATRÁS (maior
        // %MAC) dos extremos nominais correspondentes.
        assert!(out.cg_aft_case_pct_mac[0] > n.wb.spec.cg_mac_fwd_pct,
            "extremo dianteiro do caso traseiro deveria ficar atrás do nominal");
        assert!(out.cg_aft_case_pct_mac[1] > n.wb.spec.cg_mac_aft_pct,
            "extremo traseiro do caso traseiro deveria ficar atrás do nominal");

        // Todo flip reportado corresponde a um check que passava no
        // nominal — reconstrói o veredito nominal de cada tipo de check
        // citado em `flip.check` e confirma que ele passava.
        for flip in &out.flips {
            if flip.check == "Tipback" {
                assert!(n.gear.tipback_angle_deg >= n.cfg.gear.tipback_min_deg,
                    "flip de Tipback só deveria existir se o nominal passava");
            } else if flip.check == "Carga de nariz máx" {
                assert!(n.gear.nose_load_max_pct <= NOSE_LOAD_MAX_CEILING_PCT);
            } else if flip.check == "Carga de nariz mín" {
                assert!(n.gear.nose_load_min_pct >= NOSE_LOAD_MIN_FLOOR_PCT);
            } else if let Some(nome_cenario) = flip.check.strip_prefix("Cenário '").and_then(|s| s.strip_suffix('\'')) {
                let sc = n.wb.scenarios.iter().find(|s| s.name == nome_cenario)
                    .unwrap_or_else(|| panic!("cenário '{nome_cenario}' do flip não existe no nominal"));
                assert!(sc.inside_envelope,
                    "flip do cenário '{nome_cenario}' só deveria existir se ele passava no nominal");
            } else {
                panic!("check de flip desconhecido: {}", flip.check);
            }
        }
    }
}
