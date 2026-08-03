/// WeightBalanceAgent — Peso, Balanceamento e Estabilidade Longitudinal
///
/// Calcula:
///   - Geometria da asa (MAC, corda raiz/ponta, posição do CA)
///   - Orçamento de peso por componente
///   - CG para cada cenário de carga (voo solo, 2 pax, carga máxima)
///   - Ponto neutro e margem estática
///   - Verificação da estabilidade longitudinal (SM > 0 em todos os cenários)
///
/// Datum (origem do eixo x): ponta do cone do nariz da aeronave.
/// Todos os braços de momento são medidos positivamente para a cauda.
///
/// Referências:
///   - Raymer, D. "Aircraft Design: A Conceptual Approach", Cap. 6 e 10
///   - Roskam, J. "Airplane Design Part II", estabilidade estática

use crate::models::{
    aircraft_config::AircraftConfig,
    aircraft_state::AircraftState,
    engine::EngineSpec,
    requirements::Requirements,
    specs::{EmpennageSpec, WingSpec},
};

const G: f64 = 9.807; // m/s²

// ─── GEOMETRIA DA ASA ─────────────────────────────────────────────────────────

/// Corda na raiz da asa trapezoidal:
/// c_r = 2·S / (b·(1 + λ))
pub fn chord_root(wing_area_m2: f64, span_m: f64, taper: f64) -> f64 {
    2.0 * wing_area_m2 / (span_m * (1.0 + taper))
}

/// Corda na ponta da asa:
/// c_t = λ · c_r
pub fn chord_tip(chord_root: f64, taper: f64) -> f64 {
    chord_root * taper
}

/// Corda Aerodinâmica Média (MAC) — asa trapezoidal:
/// MAC = (2/3) · c_r · (1 + λ + λ²) / (1 + λ)
pub fn mean_aerodynamic_chord(chord_root: f64, taper: f64) -> f64 {
    (2.0 / 3.0) * chord_root * (1.0 + taper + taper * taper) / (1.0 + taper)
}

/// Distância da raiz à seção do MAC (envergadura de semi-asa):
/// y_MAC = (b/6) · (1 + 2λ) / (1 + λ)
pub fn mac_spanwise_pos(span_m: f64, taper: f64) -> f64 {
    (span_m / 6.0) * (1.0 + 2.0 * taper) / (1.0 + taper)
}

/// Corda local de uma superfície trapezoidal numa estação intermediária
/// qualquer da semi-envergadura — generaliza `chord_root`/`chord_tip` (que
/// são os casos particulares η=0 e η=1) para qualquer η ∈ [0,1]:
///
///   c(η) = c_raiz · (1 − (1−λ)·η),   η = y / (b/2)
///
/// Usada por `agents::control_surfaces` para a corda local nas bordas de
/// aileron/flap/profundor/leme (Task 4.2). Note que, seguindo a mesma
/// convenção de `chord_root`/`chord_tip`, `η` é medido contra QUALQUER
/// referência de "meia-envergadura" que o chamador tenha usado para chegar
/// em `chord_root` — para a asa (mirrored, duas semi-asas) isso é
/// `wing.span_m/2`; para a deriva (painel único, sem espelhamento) é a
/// própria `span_v_m` inteira (ver docstring de `control_surfaces.rs`).
pub fn chord_at(eta: f64, chord_root: f64, taper: f64) -> f64 {
    chord_root * (1.0 - (1.0 - taper) * eta)
}

// ─── COMPONENTES DE PESO E BRAÇOS DE MOMENTO ─────────────────────────────────

/// Componente de peso com braço de momento (distância do datum ao CG do item)
#[derive(Debug, Clone)]
pub struct MassItem {
    pub name: String,
    pub mass_kg: f64,
    pub arm_m: f64, // distância do nariz (datum)
}

impl MassItem {
    pub fn moment(&self) -> f64 {
        self.mass_kg * self.arm_m
    }
}

/// Cenário de carga (quem e o que embarcou)
#[derive(Debug, Clone)]
pub struct LoadScenario {
    pub name: &'static str,
    pub pax_front: u32,   // passageiros na fila da frente
    pub pax_rear: u32,    // passageiros na fila traseira
    pub baggage_kg: f64,
    pub fuel_fraction: f64, // fração do tanque cheio (0.0 a 1.0)
}

/// Geometria longitudinal da aeronave (braços a partir do nariz).
/// Construída a partir de `AircraftConfig` (`[arms]`, mais `wing.le_root_x_m`
/// e `gear.x_nose_m`/`x_main_m`, que também são braços de momento mas vivem
/// em suas seções próprias por serem, cada um, fonte única de outro dado —
/// posição do bordo de ataque e geometria do trem, respectivamente).
pub struct ArmConfig {
    pub engine_cg_m:       f64, // CG do motor + PSRU (trator, nariz)
    pub wing_le_root_m:    f64, // bordo de ataque da raiz da asa
    pub fuel_cg_m:         f64, // CG dos tanques (asas integrais, ~MAC)
    pub pax_front_m:       f64, // CG da fila dianteira (piloto + copiloto)
    pub pax_rear_m:        f64, // CG da fila traseira
    pub baggage_m:         f64, // CG do compartimento de bagagem
    pub empenagem_cg_m:    f64, // CG da empenagem (horizontal + vertical + leme)
    pub gear_main_m:       f64, // CG do trem principal
    pub gear_nose_m:       f64, // CG do trem de nariz
    pub avionics_m:        f64, // CG dos aviônicos (painel dianteiro)
    pub fuselage_struct_m: f64, // CG da estrutura da fuselagem
    pub wing_struct_m:     f64, // CG da asa (raiz + ponta)
}

impl ArmConfig {
    pub fn from_config(cfg: &AircraftConfig) -> Self {
        Self {
            engine_cg_m:       cfg.arms.engine_cg_m,
            avionics_m:        cfg.arms.avionics_m,
            gear_nose_m:       cfg.gear.x_nose_m,
            pax_front_m:       cfg.arms.pax_front_m,
            fuel_cg_m:         cfg.arms.fuel_cg_m,
            wing_le_root_m:    cfg.wing.le_root_x_m,
            wing_struct_m:     cfg.arms.wing_struct_m,
            gear_main_m:       cfg.gear.x_main_m,
            pax_rear_m:        cfg.arms.pax_rear_m,
            fuselage_struct_m: cfg.arms.fuselage_struct_m,
            baggage_m:         cfg.arms.baggage_m,
            empenagem_cg_m:    cfg.arms.empennage_cg_m,
        }
    }

    /// Resolve um braço de momento pelo nome usado em `arm_ref` nos itens de
    /// `[[masses.items]]` do TOML de aeronave. `None` = nome desconhecido
    /// (rejeitado na validação de `models::config::load_aircraft`).
    pub fn by_name(&self, name: &str) -> Option<f64> {
        match name {
            "engine_cg"       => Some(self.engine_cg_m),
            "wing_le_root"    => Some(self.wing_le_root_m),
            "fuel_cg"         => Some(self.fuel_cg_m),
            "pax_front"       => Some(self.pax_front_m),
            "pax_rear"        => Some(self.pax_rear_m),
            "baggage"         => Some(self.baggage_m),
            "empennage_cg"    => Some(self.empenagem_cg_m),
            "gear_main"       => Some(self.gear_main_m),
            "gear_nose"       => Some(self.gear_nose_m),
            "avionics"        => Some(self.avionics_m),
            "fuselage_struct" => Some(self.fuselage_struct_m),
            "wing_struct"     => Some(self.wing_struct_m),
            _ => None,
        }
    }
}

// ─── PESOS POR COMPONENTE (OEW) ───────────────────────────────────────────────

/// Retorna os itens de peso do avião vazio operacional (OEW): o motor (de
/// `EngineSpec`) mais todos os itens de `[[masses.items]]` da configuração,
/// com o braço de cada item resolvido via `arm_ref` (+ `arm_offset_m`)
/// contra `ArmConfig`.
pub fn oew_items(cfg: &AircraftConfig, engine: &EngineSpec) -> Vec<MassItem> {
    let arms = ArmConfig::from_config(cfg);
    let mut items = vec![MassItem {
        name: "Motor + acessórios".to_string(),
        mass_kg: engine.mass_kg,
        arm_m: arms.engine_cg_m,
    }];
    for item in &cfg.masses.items {
        let base_arm = arms.by_name(&item.arm_ref).unwrap_or_else(|| {
            panic!(
                "arm_ref desconhecido '{}' no item de massa '{}' — deveria ter sido \
                 rejeitado por models::config::load_aircraft",
                item.arm_ref, item.name
            )
        });
        items.push(MassItem {
            name: item.name.clone(),
            mass_kg: item.mass_kg,
            arm_m: base_arm + item.arm_offset_m,
        });
    }
    items
}

// ─── CÁLCULO DE CG ───────────────────────────────────────────────────────────

/// Centro de Gravidade composto: x_cg = Σ(m_i·x_i) / Σ(m_i)
pub fn cg_from_items(items: &[MassItem]) -> (f64, f64) {
    let total_mass: f64 = items.iter().map(|i| i.mass_kg).sum();
    let total_moment: f64 = items.iter().map(|i| i.moment()).sum();
    (total_mass, total_moment / total_mass)
}

// ─── PONTO NEUTRO E ESTABILIDADE ─────────────────────────────────────────────

/// Inclinação de sustentação (lift-curve slope) de uma superfície de asa
/// finita — teoria de Prandtl/DATCOM simplificada (Anderson, "Fundamentals
/// of Aerodynamics", cap. 5; Raymer, cap. 12):
///
///   a = 2πAR / (2 + √(4 + AR²))         [1/rad]
///
/// Válida para asa/empenagem sem enflechamento; usada tanto para a asa
/// (a_w) quanto para a empenagem horizontal (a_t) em `neutral_point_m`.
pub fn lift_curve_slope(aspect_ratio: f64) -> f64 {
    2.0 * std::f64::consts::PI * aspect_ratio / (2.0 + (4.0 + aspect_ratio * aspect_ratio).sqrt())
}

/// Posição do Ponto Neutro (NP) da aeronave — método da área de cauda de
/// Raymer, com a empenagem REALMENTE dimensionada (`EmpennageAgent`, Task
/// 4.1) em vez dos antigos `s_ratio`/`a_t/a_w` hardcoded.
///
/// x_np = x_ac_wing + Δ_stab
///   onde Δ_stab = (a_t/a_w)·(S_h/S_w)·(l_h/MAC)·η_h · MAC
///
/// a_w, a_t vêm de `lift_curve_slope` aplicada ao AR da asa e ao AR da
/// empenagem horizontal (`emp.ar_h`), respectivamente. S_h, l_h, η_h vêm de
/// `emp` (saída do `EmpennageAgent`, dimensionada por coeficiente de
/// volume) — nenhum dos quatro é mais uma constante hardcoded aqui.
pub fn neutral_point_m(
    wing_le_root_m: f64,
    mac_m: f64,
    wing_ar: f64,
    emp: &EmpennageSpec,
    wing_area_m2: f64,
) -> f64 {
    // CA da asa: bordo de ataque do MAC + 25% MAC (asa sem enflechamento —
    // y_MAC não desloca x_ac longitudinalmente nesse caso).
    let x_ac_wing = wing_le_root_m + 0.25 * mac_m;

    let a_w = lift_curve_slope(wing_ar);
    let a_t = lift_curve_slope(emp.ar_h);

    let s_ratio = emp.s_horizontal_m2 / wing_area_m2; // S_h / S_w
    let delta_stab = (a_t / a_w) * s_ratio * (emp.arm_h_m / mac_m) * emp.eta_h * mac_m;

    x_ac_wing + delta_stab
}

/// Margem Estática (Static Margin):
/// SM = (x_np - x_cg) / MAC
/// SM > 0  →  estável  (NP atrás do CG)
/// SM = 0,05–0,15  →  faixa típica para aeronave manual leve
pub fn static_margin(x_np: f64, x_cg: f64, mac: f64) -> f64 {
    (x_np - x_cg) / mac
}

/// Posição do CG em % do MAC (forma de apresentação padrão):
/// %MAC = (x_cg - x_mac_le) / MAC × 100
pub fn cg_pct_mac(x_cg: f64, x_mac_le: f64, mac: f64) -> f64 {
    (x_cg - x_mac_le) / mac * 100.0
}

// ─── AGENTE PRINCIPAL ────────────────────────────────────────────────────────

pub struct WeightBalanceAgent;

/// Saída detalhada do agente, além da WeightSpec pública
#[derive(Debug)]
pub struct WeightBalanceOutput {
    pub spec: crate::models::specs::WeightSpec,
    pub oew_kg: f64,
    pub chord_root_m: f64,
    pub chord_tip_m: f64,
    pub mac_m: f64,
    pub mac_le_x_m: f64,    // posição do bordo de ataque do MAC
    pub x_np_m: f64,        // ponto neutro
    pub scenarios: Vec<ScenarioResult>,
}

#[derive(Debug)]
pub struct ScenarioResult {
    pub name: &'static str,
    pub total_mass_kg: f64,
    pub x_cg_m: f64,
    pub cg_pct_mac: f64,
    pub static_margin: f64,
    pub stable: bool,
}

impl WeightBalanceAgent {
    pub fn run(
        state: &AircraftState,
        wing: &WingSpec,
        engine: &EngineSpec,
        cfg: &AircraftConfig,
        req: &Requirements,
        emp: &EmpennageSpec,
    ) -> WeightBalanceOutput {
        let arms = ArmConfig::from_config(cfg);

        // Geometria da asa
        let c_r = chord_root(wing.area_m2, wing.span_m, wing.taper_ratio);
        let c_t = chord_tip(c_r, wing.taper_ratio);
        let mac = mean_aerodynamic_chord(c_r, wing.taper_ratio);
        let y_mac = mac_spanwise_pos(wing.span_m, wing.taper_ratio);

        // Bordo de ataque do MAC (estimativa para asa sem enflechamento)
        let x_mac_le = arms.wing_le_root_m + y_mac * 0.0; // sem sweep = constante

        // Ponto neutro — usa a empenagem REALMENTE dimensionada (Task 4.1),
        // não mais `s_ratio`/`l_tail`/`eta_t`/`at_aw` hardcoded.
        let x_np = neutral_point_m(
            arms.wing_le_root_m, mac, wing.aspect_ratio, emp, wing.area_m2,
        );

        // Peso vazio operacional
        let oew_items = oew_items(cfg, engine);
        let (oew_kg, x_cg_oew) = cg_from_items(&oew_items);

        // Peso dos passageiros — massa por passageiro vem de `req.pax_mass_kg`
        // (mission.toml), não mais hardcoded.
        let pax_mass = req.pax_mass_kg;
        // Combustível total (densidade do combustível do motor instalado —
        // não hardcoded, para não divergir silenciosamente ao trocar motor)
        let fuel_mass_full = state.fuel_capacity_l * engine.fuel.density_kg_per_l;

        // Cenários de carga
        let scenarios_def = vec![
            LoadScenario { name: "Solo (piloto)",           pax_front: 1, pax_rear: 0, baggage_kg:  0.0, fuel_fraction: 1.0 },
            LoadScenario { name: "2 pax dianteiros",        pax_front: 2, pax_rear: 0, baggage_kg: 20.0, fuel_fraction: 1.0 },
            LoadScenario { name: "4 pax sem bagagem",       pax_front: 2, pax_rear: 2, baggage_kg:  0.0, fuel_fraction: 1.0 },
            LoadScenario { name: "4 pax + bagagem + cheio", pax_front: 2, pax_rear: 2, baggage_kg: 80.0, fuel_fraction: 1.0 },
            LoadScenario { name: "4 pax + bagagem + meia",  pax_front: 2, pax_rear: 2, baggage_kg: 80.0, fuel_fraction: 0.5 },
            LoadScenario { name: "4 pax + bagagem vazio",   pax_front: 2, pax_rear: 2, baggage_kg: 80.0, fuel_fraction: 0.1 },
        ];

        let mut scenario_results = Vec::new();
        let mut mtow_max: f64 = 0.0;

        for sc in &scenarios_def {
            let mut items = oew_items.clone();

            // Passageiros da frente
            if sc.pax_front > 0 {
                items.push(MassItem {
                    name: "Pax frente".to_string(),
                    mass_kg: pax_mass * sc.pax_front as f64,
                    arm_m: arms.pax_front_m,
                });
            }
            // Passageiros traseiros
            if sc.pax_rear > 0 {
                items.push(MassItem {
                    name: "Pax traseiro".to_string(),
                    mass_kg: pax_mass * sc.pax_rear as f64,
                    arm_m: arms.pax_rear_m,
                });
            }
            // Bagagem
            if sc.baggage_kg > 0.0 {
                items.push(MassItem {
                    name: "Bagagem".to_string(),
                    mass_kg: sc.baggage_kg,
                    arm_m: arms.baggage_m,
                });
            }
            // Combustível
            items.push(MassItem {
                name: "Combustível".to_string(),
                mass_kg: fuel_mass_full * sc.fuel_fraction,
                arm_m: arms.fuel_cg_m,
            });

            let (total_mass, x_cg) = cg_from_items(&items);
            let sm = static_margin(x_np, x_cg, mac);
            let cg_pct = cg_pct_mac(x_cg, x_mac_le, mac);

            if total_mass > mtow_max { mtow_max = total_mass; }

            scenario_results.push(ScenarioResult {
                name:          sc.name,
                total_mass_kg: total_mass,
                x_cg_m:        x_cg,
                cg_pct_mac:    cg_pct,
                static_margin: sm,
                // Estável = NP atrás do CG (SM > 0). CG muito à frente (SM > 30%)
            // é problema de autoridade de profundor, não de instabilidade.
            stable:        sm > 0.03,
            });
        }

        // CG mais à frente e mais atrás observados
        let cg_fwd = scenario_results.iter().map(|s| s.cg_pct_mac).fold(f64::INFINITY,  f64::min);
        let cg_aft = scenario_results.iter().map(|s| s.cg_pct_mac).fold(f64::NEG_INFINITY, f64::max);
        let sm_min = scenario_results.iter().map(|s| s.static_margin).fold(f64::INFINITY, f64::min);

        WeightBalanceOutput {
            spec: crate::models::specs::WeightSpec {
                oew_kg,
                mtow_kg:          mtow_max,
                payload_kg:       req.payload_kg(),
                fuel_mass_kg:     fuel_mass_full,
                cg_mac_fwd_pct:   cg_fwd,
                cg_mac_aft_pct:   cg_aft,
                static_margin_pct: sm_min * 100.0,
            },
            oew_kg,
            chord_root_m:  c_r,
            chord_tip_m:   c_t,
            mac_m:         mac,
            mac_le_x_m:    x_mac_le,
            x_np_m:        x_np,
            scenarios:     scenario_results,
        }
    }
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::aircraft_state::AircraftState;
    use crate::models::aircraft_config::test_fixtures::config_teste;
    use crate::agents::aerodynamics::AerodynamicsAgent;
    use crate::agents::empennage::EmpennageAgent;
    use crate::models::engine::test_fixtures::motor_generico_teste as engine_teste;

    #[test]
    fn chord_root_coerente() {
        // S=14.2m², b=11.94m, λ=0.45 → c_r ≈ 1.64m
        let cr = chord_root(14.2, 11.94, 0.45);
        assert!((cr - 1.64).abs() < 0.05, "c_r = {cr:.3} m (esperado ~1.64 m)");
    }

    /// Hand-check (mesmos valores usados no hand-check do aileron em
    /// `agents::control_surfaces`, b=11.94m, c_r=1.6403m, λ=0.45):
    ///   c(0.55) = c_r·(1−0.55·0.55) = c_r·0.6975 ≈ 1.1441 m
    ///   c(0.90) = c_r·(1−0.55·0.90) = c_r·0.5050 ≈ 0.8284 m
    #[test]
    fn chord_at_hand_check() {
        let cr = chord_root(14.2, 11.94, 0.45);
        let c55 = chord_at(0.55, cr, 0.45);
        let c90 = chord_at(0.90, cr, 0.45);
        println!("c_r={cr:.4}  c(0.55)={c55:.4}  c(0.90)={c90:.4}");
        assert!((c55 - 1.1441).abs() < 0.001, "c(0.55) = {c55:.4} (esperado ≈1.1441)");
        assert!((c90 - 0.8284).abs() < 0.001, "c(0.90) = {c90:.4} (esperado ≈0.8284)");
    }

    #[test]
    fn chord_at_extremos_batem_com_chord_root_e_chord_tip() {
        let cr = chord_root(14.2, 11.94, 0.45);
        let ct = chord_tip(cr, 0.45);
        assert!((chord_at(0.0, cr, 0.45) - cr).abs() < 1e-9,
            "chord_at(0, ...) deveria ser exatamente chord_root");
        assert!((chord_at(1.0, cr, 0.45) - ct).abs() < 1e-9,
            "chord_at(1, ...) deveria ser exatamente chord_tip");
    }

    #[test]
    fn mac_maior_que_tip_menor_que_root() {
        let cr = chord_root(14.2, 11.94, 0.45);
        let ct = chord_tip(cr, 0.45);
        let mac = mean_aerodynamic_chord(cr, 0.45);
        assert!(mac > ct && mac < cr,
            "MAC {mac:.3} deve estar entre c_t={ct:.3} e c_r={cr:.3}");
    }

    #[test]
    fn oew_dentro_do_orcamento() {
        let cfg    = config_teste();
        let engine = engine_teste();
        let items  = oew_items(&cfg, &engine);
        let (oew, _) = cg_from_items(&items);
        println!("OEW (fixture sintética) = {oew:.1} kg");
        // Soma dos itens sintéticos de config_teste() (649 kg) + motor
        // sintético (150 kg) = 799 kg — faixa com folga ao redor deste valor,
        // testando o pipeline de resolução de arm_ref, não um número mágico.
        assert!(oew > 750.0 && oew < 850.0,
            "OEW = {oew:.1} kg fora do intervalo esperado (750–850 kg)");
    }

    #[test]
    fn todos_os_cenarios_estaveis() {
        let cfg    = config_teste();
        let state  = AircraftState::from_config(&cfg);
        let req    = crate::models::requirements::test_fixtures::requisitos_teste();
        let wing   = AerodynamicsAgent::run(&state, &req);
        let emp    = EmpennageAgent::run(&wing, &cfg);
        let engine = engine_teste();
        let wb     = WeightBalanceAgent::run(&state, &wing, &engine, &cfg, &req, &emp);

        for sc in &wb.scenarios {
            assert!(sc.stable,
                "Cenário '{}': CG {:.1}% MAC, SM={:.3} — INSTÁVEL",
                sc.name, sc.cg_pct_mac, sc.static_margin);
        }
    }

    #[test]
    fn mtow_dentro_do_limite_projeto() {
        let cfg    = config_teste();
        let state  = AircraftState::from_config(&cfg);
        let req    = crate::models::requirements::test_fixtures::requisitos_teste();
        let wing   = AerodynamicsAgent::run(&state, &req);
        let emp    = EmpennageAgent::run(&wing, &cfg);
        let engine = engine_teste();
        let wb     = WeightBalanceAgent::run(&state, &wing, &engine, &cfg, &req, &emp);

        println!("MTOW (fixture sintética) = {:.1} kg", wb.spec.mtow_kg);
        // Faixa ampla ao redor do MTOW observado empiricamente (~1.415 kg)
        // para a fixture sintética (célula + motor de teste) —
        // suficientemente apertada para pegar regressões reais no somatório
        // de peso, mas sem acoplar o teste a um valor exato.
        let mtow = wb.spec.mtow_kg;
        assert!(mtow > 1_300.0 && mtow < 1_500.0,
            "MTOW = {mtow:.1} kg fora do intervalo (1.300–1.500 kg)");
    }

    /// Hand-check: a = 2πAR/(2+√(4+AR²))
    ///   AR=10 → a = 2π·10/(2+√104)      = 62.832/12.198  ≈ 5.151
    ///   AR=4  → a = 2π·4/(2+√20)        = 25.133/6.472   ≈ 3.883
    #[test]
    fn lift_curve_slope_hand_check() {
        let a10 = lift_curve_slope(10.0);
        let a4  = lift_curve_slope(4.0);
        println!("a(AR=10) = {a10:.4}  a(AR=4) = {a4:.4}");
        assert!((a10 - 5.15).abs() < 0.05, "a(10) = {a10:.4} (esperado 5.15 ±0.05)");
        assert!((a4  - 3.88).abs() < 0.05, "a(4)  = {a4:.4} (esperado 3.88 ±0.05)");
    }

    /// Propriedade: aumentar V_h (coeficiente de volume da empenagem
    /// horizontal) aumenta S_h, que por sua vez AUMENTA a contribuição
    /// estabilizadora da empenagem (Δ_stab) — o NP deve recuar (mover para
    /// trás/aft) estritamente. Constrói uma config modificada em vez de usar
    /// um valor mágico, para testar a relação física, não um número fixo.
    #[test]
    fn np_recua_quando_v_h_aumenta() {
        let cfg    = config_teste();
        let state  = AircraftState::from_config(&cfg);
        let req    = crate::models::requirements::test_fixtures::requisitos_teste();
        let wing   = AerodynamicsAgent::run(&state, &req);

        let emp_base = EmpennageAgent::run(&wing, &cfg);
        let c_r = chord_root(wing.area_m2, wing.span_m, wing.taper_ratio);
        let mac = mean_aerodynamic_chord(c_r, wing.taper_ratio);
        let x_np_base = neutral_point_m(
            cfg.wing.le_root_x_m, mac, wing.aspect_ratio, &emp_base, wing.area_m2,
        );

        let mut cfg_maior = cfg.clone();
        cfg_maior.empennage.v_h *= 1.3;
        let emp_maior = EmpennageAgent::run(&wing, &cfg_maior);
        let x_np_maior = neutral_point_m(
            cfg_maior.wing.le_root_x_m, mac, wing.aspect_ratio, &emp_maior, wing.area_m2,
        );

        println!("x_np base (v_h={:.3}) = {x_np_base:.4}m | x_np maior (v_h={:.3}) = {x_np_maior:.4}m",
            cfg.empennage.v_h, cfg_maior.empennage.v_h);
        assert!(x_np_maior > x_np_base,
            "NP deveria recuar (aumentar x) quando v_h aumenta: base={x_np_base:.4}m \
             maior={x_np_maior:.4}m");
    }

    #[test]
    fn arm_config_by_name_resolve_todos_os_nomes_usados_em_masses() {
        let cfg = config_teste();
        let arms = ArmConfig::from_config(&cfg);
        for item in &cfg.masses.items {
            assert!(arms.by_name(&item.arm_ref).is_some(),
                "arm_ref '{}' do item '{}' não resolveu — teste e validação de \
                 config devem concordar sobre nomes válidos", item.arm_ref, item.name);
        }
        assert!(arms.by_name("nome_que_nao_existe").is_none());
    }
}
