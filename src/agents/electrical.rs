/// ElectricalAgent — Orçamento Elétrico (Task 5.2)
///
/// Agente pequeno e puro: soma as cargas configuradas em
/// `[electrical].loads` (`AircraftConfig`) e compara contra a capacidade do
/// alternador (`[electrical].alternator_w`). Não depende de MTOW nem de
/// nenhum outro agente — pode rodar isoladamente a qualquer momento.
///
/// Modelo de pico ADOTADO (documentado aqui de propósito, é uma escolha de
/// modelagem, não a única possível): `peak_load_w = Σ peak_w` de TODAS as
/// cargas — "pior caso, tudo ligado ao mesmo tempo". Isto é conservador
/// (superestima o pico real simultâneo — nem toda carga pica ao mesmo
/// tempo: o trem retrátil só pica durante a retração, não durante cruzeiro
/// com pitot aquecido ligado), de propósito: numa aeronave de baixa
/// complexidade sem um sequenciador de cargas modelado, assumir simultaneidade
/// total é a hipótese mais segura para dimensionar o banco de baterias que
/// cobre os transientes (ver `ConstraintChecker::verify`, que trata excesso
/// de `peak_load_w` sobre `alternator_w` como AVISO, não violação — o banco
/// de baterias existe exatamente para isto).
use crate::models::aircraft_config::AircraftConfig;
use crate::models::specs::ElectricalSpec;

pub struct ElectricalAgent;

impl ElectricalAgent {
    /// Executa o agente: soma as cargas de `cfg.electrical.loads` e deriva
    /// a margem sobre a capacidade contínua do alternador.
    ///
    /// Pressupõe uma `AircraftConfig` já validada (`models::config::
    /// validate_aircraft` — `electrical.loads` não vazio, nomes únicos,
    /// valores não-negativos/finitos) — este agente não revalida, apenas
    /// soma.
    pub fn run(cfg: &AircraftConfig) -> ElectricalSpec {
        let elec = &cfg.electrical;

        let continuous_load_w: f64 = elec.loads.iter().map(|l| l.continuous_w).sum();
        let peak_load_w: f64 = elec.loads.iter().map(|l| l.peak_w).sum();

        let margin_continuous_pct =
            (elec.alternator_w - continuous_load_w) / elec.alternator_w * 100.0;

        ElectricalSpec {
            bus_voltage_v: elec.bus_voltage_v,
            alternator_w: elec.alternator_w,
            continuous_load_w,
            peak_load_w,
            margin_continuous_pct,
        }
    }
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::aircraft_config::test_fixtures::config_teste;

    /// Regressão à mão contra o baseline real (`config/aircraft/
    /// baseline_4seat.toml`, não a fixture sintética): aviônicos 180 +
    /// luzes 45 + bomba 60 + trem 0 + flaps 0 + pitot 90 + rádio 55 =
    /// 430 W contínuo; margem = (900−430)/900×100 = 52,222...%.
    #[test]
    fn soma_exata_e_formula_de_margem_contra_baseline_real() {
        let toml = std::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("config/aircraft/baseline_4seat.toml"),
        )
        .expect("falha ao ler baseline_4seat.toml do disco");
        let cfg = crate::models::config::parse_aircraft(&toml)
            .expect("baseline real deveria ser uma configuração válida");

        let spec = ElectricalAgent::run(&cfg);
        println!(
            "continuous_load_w={:.3} peak_load_w={:.3} margin_continuous_pct={:.4}%",
            spec.continuous_load_w, spec.peak_load_w, spec.margin_continuous_pct
        );

        assert!((spec.continuous_load_w - 430.0).abs() < 1e-9,
            "continuous_load_w esperado 430.0 W, obtido {:.6}", spec.continuous_load_w);

        let margem_esperada = (900.0 - 430.0) / 900.0 * 100.0; // ≈ 52.2222%
        assert!((spec.margin_continuous_pct - margem_esperada).abs() < 1e-9,
            "margin_continuous_pct esperado {margem_esperada:.6}%, obtido {:.6}%",
            spec.margin_continuous_pct);

        assert_eq!(spec.bus_voltage_v, 28.0);
        assert_eq!(spec.alternator_w, 900.0);
    }

    /// Pico "pior caso" (Σ peak_w) contra o baseline real: 220+90+120+520+
    /// 150+90+70 = 1.260 W — acima do alternador (900 W), o que deve
    /// disparar o AVISO (não violação) de `ConstraintChecker::verify`.
    #[test]
    fn soma_de_pico_pior_caso_contra_baseline_real() {
        let toml = std::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("config/aircraft/baseline_4seat.toml"),
        )
        .expect("falha ao ler baseline_4seat.toml do disco");
        let cfg = crate::models::config::parse_aircraft(&toml)
            .expect("baseline real deveria ser uma configuração válida");

        let spec = ElectricalAgent::run(&cfg);
        assert!((spec.peak_load_w - 1_260.0).abs() < 1e-9,
            "peak_load_w esperado 1260.0 W, obtido {:.6}", spec.peak_load_w);
        assert!(spec.peak_load_w > spec.alternator_w,
            "pré-condição: pico pior-caso deveria exceder o alternador (banco de baterias \
             cobre o transiente) — peak={:.1} W, alternator={:.1} W",
            spec.peak_load_w, spec.alternator_w);
    }

    /// Soma exata contra a fixture sintética (`config_teste()`), independente
    /// do baseline real — mesma filosofia de "nenhum destes números
    /// coincide com o baseline real" usada nas demais fixtures do crate.
    #[test]
    fn soma_exata_contra_fixture_sintetica() {
        let cfg = config_teste();
        let spec = ElectricalAgent::run(&cfg);

        // 170+40+55+0+0+85+50 = 400 W
        let esperado: f64 = cfg.electrical.loads.iter().map(|l| l.continuous_w).sum();
        assert!((esperado - 400.0).abs() < 1e-9, "pré-condição da fixture mudou: {esperado}");
        assert!((spec.continuous_load_w - 400.0).abs() < 1e-9);

        // 210+85+110+480+140+85+65 = 1175 W
        let esperado_peak: f64 = cfg.electrical.loads.iter().map(|l| l.peak_w).sum();
        assert!((esperado_peak - 1_175.0).abs() < 1e-9, "pré-condição da fixture mudou: {esperado_peak}");
        assert!((spec.peak_load_w - 1_175.0).abs() < 1e-9);
    }

    /// `electrical.loads` vazio é rejeitado na VALIDAÇÃO de configuração
    /// (`models::config::validate_aircraft`), antes de `ElectricalAgent`
    /// sequer rodar — o agente em si é soma pura e não tem como retornar
    /// erro (assinatura infalível). Este teste prova que o caminho de
    /// entrada (parse+validação) já barra a configuração inválida, então
    /// `ElectricalAgent::run` nunca é chamado com `loads` vazio em uso
    /// normal.
    #[test]
    fn loads_vazio_e_rejeitado_na_validacao_antes_do_agente_rodar() {
        let toml_base = std::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("config/aircraft/baseline_4seat.toml"),
        )
        .expect("falha ao ler baseline_4seat.toml do disco");

        // Substitui a seção [electrical] inteira por uma com loads vazio.
        let head = toml_base.split("[electrical]").next().unwrap();
        let toml_mutado = format!(
            "{head}\n[electrical]\nbus_voltage_v = 28.0\nalternator_w = 900.0\nloads = []\n\
             [mass_model]\n\
             composite_factor_wing = 0.85\n\
             composite_factor_tail = 0.83\n\
             composite_factor_fuselage = 0.90\n\
             composite_factor_gear = 0.95\n\
             composite_factor_fuel_system = 1.00\n\
             d_fus_equiv_m = 1.30\n\
             fuselage_wetted_coeff = 0.75\n\
             landing_load_factor_ult = 4.5\n\
             main_strut_length_m = 0.67\n\
             nose_strut_length_m = 0.53\n\
             sigma_mass_fraction = 0.15\n"
        );

        let err = crate::models::config::parse_aircraft(&toml_mutado)
            .expect_err("electrical.loads vazio deveria ser rejeitado na validação");
        println!("erro esperado: {err}");
        assert!(err.to_string().contains("electrical.loads"), "{err}");
    }
}
