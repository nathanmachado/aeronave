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
    aircraft_state::AircraftState,
    specs::{WingSpec, WeightSpec},
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

// ─── COMPONENTES DE PESO E BRAÇOS DE MOMENTO ─────────────────────────────────

/// Componente de peso com braço de momento (distância do datum ao CG do item)
#[derive(Debug, Clone)]
pub struct MassItem {
    pub name: &'static str,
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

/// Geometria longitudinal padrão da aeronave (braços a partir do nariz).
/// Baseado em aeronave de asa baixa tipo Cirrus SR22 / Lancair IV, 4 lugares.
/// Comprimento total da fuselagem: 8,2 m
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
    pub fn default_layout() -> Self {
        Self {
            engine_cg_m:       0.65,  // motor à frente da antepara corta-fogo
            avionics_m:        1.10,  // painel + computadores
            gear_nose_m:       1.40,  // trem de nariz recolhido
            pax_front_m:       3.20,  // fila da frente (piloto + copiloto)
            fuel_cg_m:         3.55,  // tanques nas asas — perto do CA
            wing_le_root_m:    2.90,  // bordo de ataque na raiz
            wing_struct_m:     3.70,  // CG estrutural da asa
            gear_main_m:       3.85,  // trem principal (após o CA)
            pax_rear_m:        4.55,  // fila traseira
            fuselage_struct_m: 4.20,  // CG da fuselagem
            baggage_m:         5.60,  // compartimento de bagagem
            empenagem_cg_m:    7.40,  // empenagem (última seção)
        }
    }
}

// ─── PESOS POR COMPONENTE (OEW) ───────────────────────────────────────────────

/// Retorna os itens de peso do avião vazio operacional (OEW).
/// Baseado em estimativas de Raymer (Tabela 6.1) e dados de aeronaves similares
/// em construção composta (fiberglass + reforço carbono nas longarinas).
pub fn oew_items(arms: &ArmConfig, fuel_capacity_l: f64) -> Vec<MassItem> {
    let _ = fuel_capacity_l; // reservado para variante com tanques maiores
    vec![
        // Motor + acessórios: massa ainda hardcoded (195 kg) — integração
        // com EngineSpec::mass_kg é escopo da Task 1.5.
        MassItem { name: "Motor + acessórios",     mass_kg: 195.0, arm_m: arms.engine_cg_m },
        MassItem { name: "PSRU + hélice + capô",  mass_kg:  65.0, arm_m: arms.engine_cg_m + 0.3 },
        MassItem { name: "Sistema de resfriamento",mass_kg: 18.0, arm_m: arms.engine_cg_m + 0.5 },
        MassItem { name: "Aviônicos + elétrica",   mass_kg: 60.0, arm_m: arms.avionics_m },
        MassItem { name: "Painel + comandos",       mass_kg: 25.0, arm_m: arms.pax_front_m - 0.3 },
        MassItem { name: "Fuselagem (composto)",    mass_kg:160.0, arm_m: arms.fuselage_struct_m },
        MassItem { name: "Asa (fiberglass+carbono)",mass_kg:130.0, arm_m: arms.wing_struct_m },
        MassItem { name: "Empenagem horizontal",    mass_kg: 22.0, arm_m: arms.empenagem_cg_m },
        MassItem { name: "Empenagem vertical",      mass_kg: 16.0, arm_m: arms.empenagem_cg_m - 0.2 },
        MassItem { name: "Trem retrátil principal", mass_kg: 55.0, arm_m: arms.gear_main_m },
        MassItem { name: "Trem de nariz",           mass_kg: 22.0, arm_m: arms.gear_nose_m },
        MassItem { name: "Mobiliário + acabamento", mass_kg: 45.0, arm_m: arms.pax_front_m + 0.5 },
        MassItem { name: "Tanques (estrutura)",     mass_kg: 12.0, arm_m: arms.fuel_cg_m },
        MassItem { name: "Cabos + hidráulico",      mass_kg: 20.0, arm_m: arms.fuselage_struct_m },
        MassItem { name: "Portas + vidros",         mass_kg: 28.0, arm_m: arms.pax_front_m },
        MassItem { name: "Antepara + firewall",     mass_kg: 12.0, arm_m: arms.engine_cg_m + 0.9 },
    ]
}

// ─── CÁLCULO DE CG ───────────────────────────────────────────────────────────

/// Centro de Gravidade composto: x_cg = Σ(m_i·x_i) / Σ(m_i)
pub fn cg_from_items(items: &[MassItem]) -> (f64, f64) {
    let total_mass: f64 = items.iter().map(|i| i.mass_kg).sum();
    let total_moment: f64 = items.iter().map(|i| i.moment()).sum();
    (total_mass, total_moment / total_mass)
}

// ─── PONTO NEUTRO E ESTABILIDADE ─────────────────────────────────────────────

/// Posição do Ponto Neutro (NP) da aeronave — método simplificado de Raymer.
/// Considera contribuição da asa e da empenagem horizontal.
///
/// x_np = x_ac_wing + (CL_α_tail / CL_α_wing) · (S_tail/S_wing) · l_tail · η_tail
///
/// Para estimativa preliminar, usa-se:
///   x_np ≈ x_ac_wing + 0.10·MAC  (margem típica para configuração convencional)
///
/// Refined: NP = x_ac_wing + Δ_stab
///   onde Δ_stab = (a_t/a_w)·(S_t/S_w)·(l_t/MAC)·η_t · MAC
pub fn neutral_point_m(
    wing_le_root_m: f64,
    mac_m: f64,
    span_m: f64,
    fuselage_length_m: f64,
) -> f64 {
    // CA da asa: bordo de ataque do MAC + 25% MAC
    let x_ac_wing = wing_le_root_m
        + (span_m / 6.0 * (1.0 + 2.0 * 0.45) / (1.0 + 0.45)).tan_estimate()
        + 0.25 * mac_m;

    // Contribuição estabilizadora da empenagem horizontal (método da área de cauda)
    // Parâmetros típicos para aeronave leve: η_t = 0.90, a_t/a_w = 0.85
    // S_t/S_w = 0.22 (área da empenagem horizontal / asa)
    // l_t = distância CA asa → CA empenagem ≈ 4,8 m
    let s_ratio = 0.22_f64;   // S_tail / S_wing
    let l_tail  = 4.80_f64;   // m — braço da empenagem
    let eta_t   = 0.90_f64;   // eficiência dinâmica da empenagem
    let at_aw   = 0.85_f64;   // razão de inclinações de CL

    let delta_stab = at_aw * s_ratio * (l_tail / mac_m) * eta_t * mac_m;

    let _ = fuselage_length_m; // usado em revisão futura (efeito de corpo)
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

// ─── TRAIT AUXILIAR ──────────────────────────────────────────────────────────
trait TanEstimate {
    fn tan_estimate(self) -> f64;
}
impl TanEstimate for f64 {
    // Para taper ratio 0.45 sem enflechamento: contribuição ≈ 0
    fn tan_estimate(self) -> f64 { 0.0 }
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
    pub fn run(state: &AircraftState, wing: &WingSpec) -> WeightBalanceOutput {
        let arms = ArmConfig::default_layout();

        // Geometria da asa
        let c_r = chord_root(wing.area_m2, wing.span_m, wing.taper_ratio);
        let c_t = chord_tip(c_r, wing.taper_ratio);
        let mac = mean_aerodynamic_chord(c_r, wing.taper_ratio);
        let y_mac = mac_spanwise_pos(wing.span_m, wing.taper_ratio);

        // Bordo de ataque do MAC (estimativa para asa sem enflechamento)
        let x_mac_le = arms.wing_le_root_m + y_mac * 0.0; // sem sweep = constante

        // Ponto neutro
        let x_np = neutral_point_m(arms.wing_le_root_m, mac, wing.span_m, 8.2);

        // Peso vazio operacional
        let oew_items = oew_items(&arms, state.fuel_capacity_l);
        let (oew_kg, x_cg_oew) = cg_from_items(&oew_items);

        // Peso dos passageiros (90 kg cada)
        let pax_mass = 90.0_f64;
        // Bagagem total
        let fuel_mass_full = state.fuel_capacity_l * 0.840;

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
                    name: "Pax frente",
                    mass_kg: pax_mass * sc.pax_front as f64,
                    arm_m: arms.pax_front_m,
                });
            }
            // Passageiros traseiros
            if sc.pax_rear > 0 {
                items.push(MassItem {
                    name: "Pax traseiro",
                    mass_kg: pax_mass * sc.pax_rear as f64,
                    arm_m: arms.pax_rear_m,
                });
            }
            // Bagagem
            if sc.baggage_kg > 0.0 {
                items.push(MassItem {
                    name: "Bagagem",
                    mass_kg: sc.baggage_kg,
                    arm_m: arms.baggage_m,
                });
            }
            // Combustível
            items.push(MassItem {
                name: "Combustível",
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
                payload_kg:       (4.0 * pax_mass) + 80.0,
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
    use crate::agents::aerodynamics::AerodynamicsAgent;
    use crate::models::requirements::Requirements;

    #[test]
    fn chord_root_coerente() {
        // S=14.2m², b=11.94m, λ=0.45 → c_r ≈ 1.64m
        let cr = chord_root(14.2, 11.94, 0.45);
        assert!((cr - 1.64).abs() < 0.05, "c_r = {cr:.3} m (esperado ~1.64 m)");
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
        let state = AircraftState::initial();
        let arms  = ArmConfig::default_layout();
        let items = oew_items(&arms, state.fuel_capacity_l);
        let (oew, _) = cg_from_items(&items);
        // OEW deve ficar entre 830 e 1.050 kg (aeronave 4 lugares composta, motor diesel)
        assert!(oew > 830.0 && oew < 1_050.0,
            "OEW = {oew:.1} kg fora do intervalo esperado (830–1.050 kg)");
    }

    #[test]
    fn todos_os_cenarios_estaveis() {
        let state = AircraftState::initial();
        let req   = Requirements::project_default();
        let wing  = AerodynamicsAgent::run(&state, &req);
        let wb    = WeightBalanceAgent::run(&state, &wing);

        for sc in &wb.scenarios {
            assert!(sc.stable,
                "Cenário '{}': CG {:.1}% MAC, SM={:.3} — INSTÁVEL",
                sc.name, sc.cg_pct_mac, sc.static_margin);
        }
    }

    #[test]
    fn mtow_dentro_do_limite_projeto() {
        let state = AircraftState::initial();
        let req   = Requirements::project_default();
        let wing  = AerodynamicsAgent::run(&state, &req);
        let wb    = WeightBalanceAgent::run(&state, &wing);

        // MTOW calculado deve ficar entre 1.350 e 1.600 kg
        let mtow = wb.spec.mtow_kg;
        assert!(mtow > 1_350.0 && mtow < 1_600.0,
            "MTOW = {mtow:.1} kg fora do intervalo (1.350–1.600 kg)");
    }
}
