use super::aircraft_config::AircraftConfig;

/// Estado mutável da aeronave durante o loop de otimização.
/// O Orchestrator ajusta estes parâmetros entre iterações até
/// todos os requisitos serem satisfeitos.
///
/// Construído a partir de uma `AircraftConfig` (ver `from_config`) — a
/// aeronave inteira (geometria, perfil aerodinâmico, propulsão, trem) é
/// dado de configuração TOML, não constante Rust.
#[derive(Debug, Clone)]
pub struct AircraftState {
    // --- Geometria da asa ---
    pub wing_span_m: f64,
    pub wing_area_m2: f64,
    pub taper_ratio: f64,
    pub airfoil: String,
    /// Espessura relativa do perfil (t/c) — usada no dimensionamento
    /// estrutural da longarina.
    pub thickness_ratio: f64,

    // --- Perfil aerodinâmico e build-up de arrasto ---
    /// CL_max em configuração limpa (cruzeiro, sem flap) — usado para VS1.
    pub cl_max_clean: f64,
    /// CL_max com flap CHEIO (POUSO) — usado para VS0.
    pub cl_max_flaps: f64,
    /// Fração de deployment do flap no setting de DECOLAGEM — eco de
    /// `[stability].to_flap_fraction`. Vive aqui (e não só em
    /// `AircraftConfig`) porque o `AerodynamicsAgent` recebe o ESTADO, não
    /// a config, e é ele quem deriva `WingSpec::cl_max_to` a partir dela
    /// (ciclo 7, task 1).
    pub to_flap_fraction: f64,
    pub cd0_wing: f64,
    pub cd0_fuselage: f64,
    /// CD0 da empenagem — task refino-ciclo2 (1b): deixou de ser um eco
    /// direto de `[empennage].cd0` (campo REMOVIDO da config) e passou a
    /// ser DERIVADO de `[empennage].cd0_area_factor·(S_h+S_v)/S_w` — ver
    /// `AircraftState::from_config` para o cálculo (usa
    /// `agents::empennage::tail_areas_m2`, que não depende de
    /// `AerodynamicsAgent` já ter rodado).
    pub cd0_empennage: f64,
    /// CD0 residual (antenas, juntas, imperfeições).
    pub cd0_misc: f64,
    /// Incremento de CD0 do trem FIXO — só se soma quando `!gear_retractable`.
    pub cd0_gear_fixed_increment: f64,
    /// Fração do arrasto parasita total atribuída à refrigeração do motor
    /// (Task 5.2) — aplicada como multiplicador `(1 + fração)` sobre o CD0
    /// já somado em `agents::aerodynamics::cd0_total`.
    pub cooling_drag_fraction: f64,

    // --- Peso (estimativa inicial para o laço iterativo de projeto) ---
    pub mtow_kg: f64,

    // --- Propulsão ---
    pub psru_ratio: f64,
    /// Eficiência mecânica do PSRU (correia/engrenagens, `[propeller]
    /// psru_efficiency` do TOML) — Finding 1 da revisão final: antes vinha de
    /// um `const PSRU_EFFICIENCY = 0.97` hardcoded em `agents::propulsion`
    /// que IGNORAVA este campo (validado por `models::config` mas nunca
    /// lido pela física). Agora é dado de configuração de ponta a ponta —
    /// todo cálculo que envolve perdas mecânicas do PSRU (busca de rpm de
    /// cruzeiro, consumo, potência de eixo em `agents::performance`, Breguet
    /// em `agents::mission`, P/W do diagrama de restrições) lê este campo.
    pub psru_efficiency: f64,
    pub prop_diameter_m: f64,
    pub fuel_capacity_l: f64,

    // --- Trem de pouso ---
    pub gear_retractable: bool,
}

impl AircraftState {
    /// Constrói o estado inicial da aeronave a partir de uma configuração
    /// carregada de TOML (ver `models::config::load_aircraft`).
    pub fn from_config(cfg: &AircraftConfig) -> Self {
        // CD0 da empenagem (task refino-ciclo2, 1b) — derivado da área
        // REALMENTE dimensionada (S_h+S_v, `agents::empennage::
        // tail_areas_m2`), não mais um valor fixo de config. `tail_areas_m2`
        // usa só geometria de `[wing]`/`[empennage]` (span/área/afilamento
        // — sem nenhuma dependência de MTOW/cd0), então pode ser calculada
        // aqui, ANTES do `AerodynamicsAgent` rodar, sem reordenar o laço de
        // convergência em `orchestrator::size_aircraft` — ver docstring de
        // `tail_areas_m2` para a dedução completa.
        let (s_h, s_v) = crate::agents::empennage::tail_areas_m2(cfg);
        let cd0_empennage = cfg.empennage.cd0_area_factor * (s_h + s_v) / cfg.wing.area_m2;

        Self {
            wing_span_m: cfg.wing.span_m,
            wing_area_m2: cfg.wing.area_m2,
            taper_ratio: cfg.wing.taper_ratio,
            airfoil: cfg.wing.airfoil.clone(),
            thickness_ratio: cfg.wing.thickness_ratio,

            cl_max_clean: cfg.wing.cl_max_clean,
            cl_max_flaps: cfg.wing.cl_max_flaps,
            to_flap_fraction: cfg.stability.to_flap_fraction,
            cd0_wing: cfg.wing.cd0_wing,
            cd0_fuselage: cfg.fuselage.cd0,
            cd0_empennage,
            cd0_misc: cfg.drag.cd0_misc,
            cd0_gear_fixed_increment: cfg.gear.cd0_fixed_increment,
            cooling_drag_fraction: cfg.drag.cooling_drag_fraction,

            mtow_kg: cfg.sizing.mtow_initial_guess_kg,

            psru_ratio: cfg.propeller.psru_ratio,
            psru_efficiency: cfg.propeller.psru_efficiency,
            // Quando `[propeller].diameter_m` está presente, usa-o
            // diretamente (comportamento inalterado). Quando OMITIDO, usa
            // como palpite provisório o maior diâmetro que respeita a folga
            // de solo (`agents::propeller::diameter_max_by_clearance_m`,
            // única restrição calculável aqui sem `EngineSpec`/
            // `Requirements`, que `from_config` não recebe) — o valor
            // AUTORITATIVO, que também respeita os limites de Mach de ponta
            // (estático e cruzeiro, usando o rpm de cruzeiro real da busca
            // de BSFC), é calculado por `agents::propeller::PropellerAgent`,
            // rodado após o laço de convergência de MTOW (ver `main.rs`).
            prop_diameter_m: cfg.propeller.diameter_m.unwrap_or_else(|| {
                crate::agents::propeller::round_down_cm(
                    (crate::agents::propeller::diameter_max_by_clearance_m(
                        // Ciclo 5 (task 1): shaft_height agora DERIVA do trem —
                        // ver `agents::propeller::PropellerAgent::run`.
                        cfg.gear.h_cg_ground_m + cfg.propeller.prop_axis_above_cg_m,
                        cfg.propeller.ground_clearance_min_m,
                    ) - 0.02)
                        // Defesa em profundidade (achado de review, ciclo 5,
                        // Important 3): `models::config::validate_aircraft`
                        // já rejeita configs em que o shaft_height DERIVADO
                        // (gear.h_cg_ground_m + propeller.prop_axis_above_cg_m)
                        // não excede propeller.ground_clearance_min_m — para
                        // configs carregadas de TOML (`parse_aircraft`), este
                        // `.max(0.0)` nunca deveria disparar. Mas configs
                        // montadas em memória (what-ifs, testes que não
                        // passam por `parse_aircraft`) não têm essa garantia
                        // — sem o clamp, um shaft_height <=
                        // ground_clearance_min_m produziria um diâmetro
                        // provisório NEGATIVO aqui.
                        .max(0.0),
                )
            }),
            fuel_capacity_l: cfg.fuel_system.capacity_l,

            gear_retractable: cfg.gear.retractable,
        }
    }

    pub fn aspect_ratio(&self) -> f64 {
        self.wing_span_m.powi(2) / self.wing_area_m2
    }
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────
//
// Task refino-ciclo2 (1b), achado da revisão (Finding 1 — "estrito nos dois
// sentidos"): o brief pedia "mudar v_h em config mutada → massa E arrasto
// acompanham, estrito nos dois sentidos". A cobertura de MASSA já existia
// (`agents::weight_balance::tests::massa_emp_horizontal_aumenta_
// estritamente_quando_v_h_aumenta`) — faltava o lado do ARRASTO
// (`cd0_empennage`, derivado aqui em `AircraftState::from_config`). Este
// módulo não tinha `mod tests` antes desta adição — é o lar natural do teste
// porque é aqui que `cd0_empennage` é DERIVADO (ver comentário de
// `from_config` acima), não em `agents::aerodynamics` (que só CONSOME o
// valor já pronto via `cd0_total`, sem recalculá-lo).
#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::aircraft_config::test_fixtures::config_teste;

    /// Propriedade (Finding 1 da revisão, lado do ARRASTO — a contraparte de
    /// `weight_balance::tests::massa_emp_horizontal_aumenta_estritamente_
    /// quando_v_h_aumenta`, que já cobre o lado da MASSA): `v_h` maior → S_h
    /// maior (`agents::empennage::tail_areas_m2`, S_h = v_h·S_w·MAC/l_h) →
    /// `cd0_empennage = cd0_area_factor·(S_h+S_v)/S_w` maior, ESTRITAMENTE,
    /// nos DOIS sentidos (aumentar E diminuir `v_h`) — não só "aumentar
    /// aumenta", mas também "diminuir diminui".
    ///
    /// Verificação de que o teste de fato pega uma regressão (nota da
    /// revisão, "RED first"): rodei manualmente esta mesma asserção contra
    /// uma cópia local de `tail_areas_m2_from_wing_geometry` com a fórmula
    /// de `s_h` trocada para usar `emp_cfg.v_v` em vez de `emp_cfg.v_h` (um
    /// bug plausível — trocar os dois coeficientes de volume) — com essa
    /// troca, `s_h` deixa de responder a `v_h` (`cfg_maior.empennage.v_h`
    /// mutado não muda mais `s_h`, logo não muda `cd0_empennage`), e a
    /// asserção `cd0_maior > cd0_base` falha (`cd0_maior == cd0_base`,
    /// diferença 0.0). Revertido antes de qualquer commit — não faz parte
    /// do histórico do repositório, só documentado aqui como evidência de
    /// que o teste é sensível ao bug que ele existe para prevenir.
    #[test]
    fn cd0_empennage_acompanha_v_h_estritamente_nos_dois_sentidos() {
        let cfg_base = config_teste();
        let state_base = AircraftState::from_config(&cfg_base);

        let mut cfg_maior = cfg_base.clone();
        cfg_maior.empennage.v_h *= 1.3;
        let state_maior = AircraftState::from_config(&cfg_maior);

        let mut cfg_menor = cfg_base.clone();
        cfg_menor.empennage.v_h *= 0.7;
        let state_menor = AircraftState::from_config(&cfg_menor);

        println!(
            "cd0_empennage: menor(v_h={:.4})={:.8}  base(v_h={:.4})={:.8}  \
             maior(v_h={:.4})={:.8}",
            cfg_menor.empennage.v_h, state_menor.cd0_empennage,
            cfg_base.empennage.v_h, state_base.cd0_empennage,
            cfg_maior.empennage.v_h, state_maior.cd0_empennage,
        );

        assert!(state_maior.cd0_empennage > state_base.cd0_empennage,
            "cd0_empennage deveria AUMENTAR estritamente quando v_h aumenta: \
             base={:.8} maior={:.8}", state_base.cd0_empennage, state_maior.cd0_empennage);
        assert!(state_menor.cd0_empennage < state_base.cd0_empennage,
            "cd0_empennage deveria DIMINUIR estritamente quando v_h diminui: \
             base={:.8} menor={:.8}", state_base.cd0_empennage, state_menor.cd0_empennage);
    }
}
