//! `AircraftConfig` — estrutura desserializável que espelha `aircraft.toml`.
//!
//! Mesma filosofia de `EngineSpec`/`config::load_engine`: a célula inteira
//! (geometria, braços de momento, massas, material estrutural, trem de
//! pouso, hélice) é dado de configuração, não constante Rust. Trocar de
//! aeronave-base é trocar este arquivo, não o código.
//!
//! O parsing/validação vivem em `models::config` (`parse_aircraft`,
//! `load_aircraft`), ao lado do loader de motor, reaproveitando
//! `ConfigError`. Este módulo só contém os tipos de dado e (em teste) uma
//! fixture sintética.

use serde::{Deserialize, Serialize};

/// Configuração completa da célula — espelha `config/aircraft/*.toml`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AircraftConfig {
    pub sizing: SizingCfg,
    pub wing: WingCfg,
    pub fuselage: FuselageCfg,
    pub empennage: EmpennageCfg,
    pub propeller: PropellerCfg,
    pub fuel_system: FuelSystemCfg,
    pub gear: GearCfg,
    pub arms: ArmsCfg,
    pub structure: StructureCfg,
    /// Limites de estabilidade estática que definem o envelope de CG
    /// admissível (Task 4.4) — consumidos por
    /// `weight_balance::cg_limit_fwd_m`/`cg_limit_aft_m`.
    pub stability: StabilityCfg,
    /// CD0 residual (antenas, juntas, imperfeições) — não pertence a nenhum
    /// componente específico da aeronave.
    pub drag: DragCfg,
    pub masses: MassesCfg,
    /// Frações históricas (Raymer Tab. 6.5) que dimensionam aileron, flap,
    /// profundor e leme — consumidas por `agents::control_surfaces`.
    pub control_surfaces: ControlSurfacesCfg,
    /// Parâmetros de desempenho (Task 4.7) — atrito de frenagem por
    /// superfície, fator empírico de tração estática, e tempos/ângulos dos
    /// segmentos de decolagem/pouso sobre obstáculo de 15m (50 ft).
    pub performance: PerformanceCfg,
    /// Orçamento elétrico (Task 5.2) — barramento, alternador e cargas
    /// individuais, consumidos por `agents::electrical::ElectricalAgent`.
    pub electrical: ElectricalCfg,
}

/// Parâmetros do laço de convergência de MTOW (`orchestrator::size_aircraft`,
/// Task 3.1) — substitui o antigo `mtow_guess_kg` de topo, que era apenas um
/// palpite inicial nunca realimentado pelo `WeightBalanceAgent` (bug B5: o
/// `AerodynamicsAgent` calculava CL/CD de cruzeiro com o palpite, enquanto
/// `PerformanceAgent`/`StructuralAgent`/`LandingGearAgent` usavam o MTOW real
/// de `wb.spec.mtow_kg` — dois MTOWs diferentes no mesmo relatório).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SizingCfg {
    /// Estimativa inicial de MTOW (kg) — ponto de partida do laço de ponto
    /// fixo em `orchestrator::size_aircraft`; não é um requisito.
    pub mtow_initial_guess_kg: f64,
    /// Limite superior de MTOW aceito pelo laço de convergência — se o MTOW
    /// convergido ultrapassar este valor, `size_aircraft` retorna
    /// `SizingError::MtowExcedido` em vez de aceitar uma aeronave fora do
    /// envelope estrutural/operacional pretendido.
    pub mtow_max_kg: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WingCfg {
    pub span_m: f64,
    pub area_m2: f64,
    pub taper_ratio: f64,
    pub airfoil: String,
    /// Espessura relativa do perfil (t/c) — usada em `structural.rs` para a
    /// altura da longarina na raiz.
    pub thickness_ratio: f64,
    /// CL_max em configuração limpa (cruzeiro, sem flap) — usado para VS1.
    pub cl_max_clean: f64,
    /// CL_max com flap/slat (pouso/decolagem) — usado para VS0.
    pub cl_max_flaps: f64,
    pub cd0_wing: f64,
    /// Posição do bordo de ataque da raiz da asa (m do datum no nariz) —
    /// única fonte desta posição; `ArmConfig::wing_le_root_m` e o cálculo do
    /// CG mais traseiro em `main.rs` derivam dela.
    pub le_root_x_m: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FuselageCfg {
    pub length_m: f64,
    pub cabin_width_m: f64,
    pub cabin_height_m: f64,
    pub cd0: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmpennageCfg {
    pub cd0: f64,
    /// Braço da empenagem (CA asa → CA empenagem, m) — usado tanto no
    /// dimensionamento por coeficiente de volume (`agents::empennage`)
    /// quanto em `weight_balance::neutral_point_m`.
    pub tail_arm_m: f64,
    /// Coeficiente de volume da empenagem horizontal V_h = S_h·l_h/(S_w·MAC)
    /// — Raymer Tab. 6.4, monomotor GA (típico 0.5–0.9). Fonte única do
    /// dimensionamento de S_h em `agents::empennage::EmpennageAgent`.
    pub v_h: f64,
    /// Coeficiente de volume da empenagem vertical V_v = S_v·l_v/(S_w·b) —
    /// Raymer Tab. 6.4, monomotor GA (típico 0.02–0.05).
    pub v_v: f64,
    /// Alongamento (aspect ratio) da empenagem horizontal.
    pub ar_h: f64,
    /// Alongamento (aspect ratio) da empenagem vertical.
    pub ar_v: f64,
    /// Afilamento (taper ratio) da empenagem horizontal.
    pub taper_h: f64,
    /// Afilamento (taper ratio) da empenagem vertical.
    pub taper_v: f64,
    /// Eficiência de pressão dinâmica na empenagem horizontal (q_t/q_∞) —
    /// usada em `weight_balance::neutral_point_m`.
    pub eta_h: f64,
}

/// Configuração da hélice — dimensionamento/validação em
/// `agents::propeller::PropellerAgent` (Task 4.5).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PropellerCfg {
    /// Diâmetro da hélice (m). Quando OMITIDO do TOML (`None`), o modelo
    /// deriva o maior diâmetro que respeita simultaneamente os limites de
    /// Mach de ponta (estático e cruzeiro) e a folga mínima de solo — ver
    /// `agents::propeller::PropellerAgent::run` (`PropellerSpec::source`
    /// reporta `"config"` ou `"derivado"` conforme o caso).
    #[serde(default)]
    pub diameter_m: Option<f64>,
    pub blades: u32,
    pub psru_ratio: f64,
    pub psru_efficiency: f64,
    /// Altura do eixo da hélice ao solo (m, trem estendido) — usada na
    /// checagem de folga de solo (CS 23.925).
    pub shaft_height_m: f64,
    /// Mach de ponta de pá máximo admissível em condição ESTÁTICA (rpm
    /// nominal do motor via PSRU, V=0) — tipicamente mais restritivo que o
    /// limite de cruzeiro por não ter o alívio da velocidade de avanço na
    /// composição vetorial helicoidal.
    pub tip_mach_max_static: f64,
    /// Mach de ponta de pá máximo admissível em CRUZEIRO (composição
    /// helicoidal: velocidade tangencial da ponta + velocidade de avanço).
    pub tip_mach_max_cruise: f64,
    /// Folga mínima entre a ponta da pá e o solo (m). CS 23.925 exige
    /// ≥ 0,18 m (7 pol); o baseline usa 0,23 m (9 pol) como margem de
    /// projeto.
    pub ground_clearance_min_m: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FuelSystemCfg {
    pub capacity_l: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GearCfg {
    pub retractable: bool,
    /// Incremento de CD0 do trem FIXO (0 quando retrátil e recolhido).
    pub cd0_fixed_increment: f64,
    pub h_cg_ground_m: f64,
    pub x_nose_m: f64,
    pub x_main_m: f64,
    /// Massa de UMA perna do trem principal (kg) — usada no dimensionamento
    /// do atuador de retração. Note: a massa TOTAL do trem principal (ambas
    /// as pernas) vive em `[[masses.items]]` (`trem_principal`).
    pub mass_main_leg_kg: f64,
    /// Massa do trem de nariz (kg) — perna única; mantido aqui como o dado
    /// de engenharia "de perna", ainda que hoje coincida com o item de
    /// `[[masses.items]]` (`trem_nariz`), que é a massa total já que o
    /// nariz tem apenas uma perna.
    pub mass_nose_kg: f64,
    pub retraction_time_s: f64,
    /// Massa dos atuadores elétricos + portas do trem (kg) — soma ao peso
    /// total do sistema junto com as massas das pernas de `[masses]`.
    pub actuators_doors_mass_kg: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ArmsCfg {
    pub engine_cg_m: f64,
    pub avionics_m: f64,
    pub pax_front_m: f64,
    pub fuel_cg_m: f64,
    pub wing_struct_m: f64,
    pub pax_rear_m: f64,
    pub fuselage_struct_m: f64,
    pub baggage_m: f64,
    pub empennage_cg_m: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StructureCfg {
    /// Nome do material da longarina, resolvido em
    /// `structural::material_by_name` (ex.: "AA7075-T6", "AA6061-T6").
    pub spar_material: String,
    pub frame_spacing_mm: f64,
    /// Categoria de projeto CS-23: "normal" (n_lim 3.8g) | "utility" (4.4g)
    /// | "acrobatic" (6.0g).
    pub design_category: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DragCfg {
    pub cd0_misc: f64,
    /// Fração do arrasto parasita TOTAL (soma asa+fuselagem+empenagem+trem+
    /// misc) atribuída à refrigeração do motor (entrada de ar do radiador/
    /// intercooler, dutos, saída de ar quente) — Task 5.2. Típico 3–5% para
    /// uma instalação a pistão bem carenada (Raymer cap. 12 / Hoerner
    /// "Fluid-Dynamic Drag", arrasto de resfriamento de motores
    /// refrigerados a líquido/ar). Aplicado como multiplicador sobre o CD0
    /// já somado (`cd0_total`), não como uma parcela aditiva independente —
    /// ver `agents::aerodynamics::cd0_total`.
    pub cooling_drag_fraction: f64,
}

/// Limites de estabilidade estática (Task 4.4) que definem o envelope de CG
/// ADMISSÍVEL da aeronave — em contraste com `WeightSpec::cg_mac_fwd_pct`/
/// `cg_mac_aft_pct`, que são os extremos OBSERVADOS entre os cenários de
/// carga. O envelope vem de dois critérios físicos independentes:
///
///   - `sm_min`: margem estática mínima aceitável — abaixo dela a aeronave
///     fica perigosamente próxima da instabilidade estática longitudinal.
///     Define o limite TRASEIRO do CG (CG mais atrás permitido): SM cai
///     quando o CG recua, então SM = sm_min é o pior caso traseiro.
///   - `sm_max`: proxy de autoridade de profundor em flare/pouso — margem
///     estática ALTA demais (CG muito à frente) exige mais deflexão de
///     profundor do que a superfície consegue entregar. Define o limite
///     DIANTEIRO do CG: SM = sm_max é o pior caso dianteiro.
///
/// Ver `weight_balance::cg_limit_fwd_m`/`cg_limit_aft_m` para a conversão em
/// posição física (metros do datum no nariz).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StabilityCfg {
    /// Margem estática mínima admissível — define o limite TRASEIRO do CG.
    pub sm_min: f64,
    /// Margem estática máxima admissível (autoridade de profundor) — define
    /// o limite DIANTEIRO do CG.
    pub sm_max: f64,
}

/// Frações históricas (Raymer Tab. 6.5, monomotor GA) que dimensionam
/// aileron, flap, profundor (elevator) e leme (rudder) — ver
/// `agents::control_surfaces::ControlSurfacesAgent` para a geometria
/// derivada a partir destes valores.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControlSurfacesCfg {
    /// Início do aileron, fração da semi-envergadura da asa (η = y/(b/2)).
    pub aileron_span_start_frac: f64,
    /// Fim do aileron, fração da semi-envergadura da asa.
    pub aileron_span_end_frac: f64,
    /// Corda do aileron, fração da corda local da asa.
    pub aileron_chord_frac: f64,
    /// Início do flap (da raiz para fora), fração da semi-envergadura.
    pub flap_span_start_frac: f64,
    /// Fim do flap, fração da semi-envergadura — deve ser ≤
    /// `aileron_span_start_frac` (flap e aileron não podem se sobrepor).
    pub flap_span_end_frac: f64,
    /// Corda do flap, fração da corda local da asa.
    pub flap_chord_frac: f64,
    /// Envergadura do profundor, fração da envergadura TOTAL do
    /// estabilizador horizontal (`EmpennageSpec::span_h_m`).
    pub elevator_span_frac: f64,
    /// Corda do profundor, fração da corda local do estabilizador
    /// horizontal.
    pub elevator_chord_frac: f64,
    /// Envergadura do leme, fração da envergadura da deriva
    /// (`EmpennageSpec::span_v_m`).
    pub rudder_span_frac: f64,
    /// Corda do leme, fração da corda local da deriva.
    pub rudder_chord_frac: f64,
}

/// Parâmetros de desempenho (Task 4.7) — substituem os fatores ad hoc do
/// M5 (`/surface_factor`, `·√surface_factor`, tração estática sem correção,
/// distância aérea de pouso fixa em 200m) por dados de configuração
/// explícitos, consumidos por `agents::performance`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceCfg {
    /// Coeficiente de atrito de frenagem em pista pavimentada seca —
    /// substitui o antigo `0.40` literal em `landing_distance_m`.
    pub mu_brake_paved: f64,
    /// Coeficiente de atrito de frenagem em grama firme/terra compactada.
    /// As funções de pouso (`agents::performance::landing_distance_m`/
    /// `landing_distance_50ft_m`) recebem `mu_brake` genericamente — este
    /// valor está pronto para uso, mas `PerformanceSpec` hoje só modela
    /// pouso em pista pavimentada (mesma assimetria pré-existente: só
    /// decolagem tinha variantes pavimentada/grama antes da Task 4.7).
    pub mu_brake_grass: f64,
    /// Fator empírico (McCormick) aplicado sobre a tração estática IDEAL de
    /// Rankine-Froude (disco atuador) — a teoria de disco atuador
    /// superestima a tração real por não modelar perdas de ponta de pá,
    /// rotação de esteira e não-uniformidade da distribuição de carga.
    pub static_thrust_factor: f64,
    /// Tempo de rotação — do início da rotação até V_LOF, a V_LOF
    /// aproximadamente constante (s).
    pub rotation_time_s: f64,
    /// Tempo de flare/arredondamento no pouso, a V_ref aproximadamente
    /// constante (s).
    pub flare_time_s: f64,
    /// Ângulo de aproximação padrão de pouso (graus) — define a distância
    /// aérea até a altura de 15m (50 ft) na aproximação final.
    pub approach_angle_deg: f64,
}

/// Uma carga elétrica individual do orçamento (Task 5.2) — consumida por
/// `agents::electrical::ElectricalAgent`, que soma `continuous_w`/`peak_w`
/// de todos os itens de `[electrical].loads`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ElectricalLoadCfg {
    pub name: String,
    /// Potência contínua/média em operação normal (W).
    pub continuous_w: f64,
    /// Potência de PICO — o maior consumo instantâneo do item (ex.: motor
    /// de atuador em partida, transmissor de rádio transmitindo), não uma
    /// média. Ver `ElectricalAgent::run` para como `peak_load_w` do
    /// orçamento total é derivado destes picos individuais (modelo
    /// conservador: soma de todos os picos, não Σcontínuo + maior pico).
    pub peak_w: f64,
}

/// Orçamento elétrico da aeronave (Task 5.2) — barramento, capacidade do
/// alternador e cargas individuais. Antes desta task não existia um dono
/// único deste orçamento (potência de atuadores calculada isoladamente em
/// `landing_gear.rs`, sem comparação contra a capacidade de geração).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ElectricalCfg {
    /// Tensão nominal do barramento (V) — deve ser um padrão aeronáutico
    /// reconhecido: 12, 14, 24 ou 28 V (±0.1 V de tolerância numérica; ver
    /// `models::config::validate_aircraft`).
    pub bus_voltage_v: f64,
    /// Capacidade do alternador/gerador (W) — ex.: alternador automotivo
    /// típico de 32 A @ 28 V ≈ 900 W (derateado da placa por margem de
    /// projeto/temperatura), acoplado ao motor a pistão via correia/PSRU.
    pub alternator_w: f64,
    pub loads: Vec<ElectricalLoadCfg>,
}

/// Um item de massa do orçamento de peso vazio (OEW), com o braço de
/// momento expresso por REFERÊNCIA a uma entrada de `[arms]` (ou de
/// `[wing]`/`[gear]`, ver `weight_balance::ArmConfig::by_name`) mais um
/// deslocamento opcional — assim os braços continuam com fonte única em
/// `[arms]`/`[wing]`/`[gear]`, nunca duplicados dentro de `[masses]`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MassItemCfg {
    pub name: String,
    pub mass_kg: f64,
    pub arm_ref: String,
    #[serde(default)]
    pub arm_offset_m: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MassesCfg {
    pub items: Vec<MassItemCfg>,
}

impl MassesCfg {
    /// Massa (kg) de um item por nome — usado por `structural.rs` para obter
    /// a massa estrutural da asa (item `"asa"`) sem duplicar o valor.
    pub fn item_mass(&self, name: &str) -> Option<f64> {
        self.items.iter().find(|i| i.name == name).map(|i| i.mass_kg)
    }
}

/// Fixtures de configuração sintética compartilhadas pelos módulos de teste
/// deste crate. Valores deliberadamente perturbados em relação à aeronave
/// real de `config/aircraft/baseline_4seat.toml` — nenhum destes números
/// coincide com o baseline real (ver mesma justificativa em
/// `engine::test_fixtures`).
#[cfg(test)]
pub mod test_fixtures {
    use super::*;

    /// Configuração de célula sintética para testes de `src/`. O baseline
    /// real só aparece em `tests/`, carregado do TOML de verdade.
    pub fn config_teste() -> AircraftConfig {
        AircraftConfig {
            sizing: SizingCfg {
                mtow_initial_guess_kg: 1_400.0,
                mtow_max_kg: 1_900.0,
            },
            wing: WingCfg {
                span_m: 11.0,
                area_m2: 13.5,
                taper_ratio: 0.5,
                airfoil: "Perfil Sintético de Teste".to_string(),
                thickness_ratio: 0.14,
                cl_max_clean: 1.40,
                cl_max_flaps: 1.65,
                cd0_wing: 0.0052,
                le_root_x_m: 2.80,
            },
            fuselage: FuselageCfg {
                length_m: 8.0,
                cabin_width_m: 1.20,
                cabin_height_m: 1.18,
                cd0: 0.0105,
            },
            empennage: EmpennageCfg {
                cd0: 0.0042,
                tail_arm_m: 4.70,
                v_h: 0.65,
                v_v: 0.045,
                ar_h: 4.5,
                ar_v: 1.6,
                taper_h: 0.45,
                taper_v: 0.45,
                eta_h: 0.92,
            },
            propeller: PropellerCfg {
                diameter_m: Some(1.90),
                blades: 2,
                psru_ratio: 2.0,
                psru_efficiency: 0.965,
                shaft_height_m: 1.15,
                tip_mach_max_static: 0.83,
                tip_mach_max_cruise: 0.78,
                ground_clearance_min_m: 0.25,
            },
            fuel_system: FuelSystemCfg { capacity_l: 220.0 },
            gear: GearCfg {
                retractable: true,
                cd0_fixed_increment: 0.0082,
                h_cg_ground_m: 1.03,
                x_nose_m: 1.35,
                x_main_m: 3.75,
                mass_main_leg_kg: 26.0,
                mass_nose_kg: 21.0,
                retraction_time_s: 7.5,
                actuators_doors_mass_kg: 19.0,
            },
            arms: ArmsCfg {
                engine_cg_m: 0.60,
                avionics_m: 1.05,
                pax_front_m: 3.10,
                fuel_cg_m: 3.45,
                wing_struct_m: 3.60,
                pax_rear_m: 4.45,
                fuselage_struct_m: 4.10,
                baggage_m: 5.50,
                empennage_cg_m: 7.25,
            },
            structure: StructureCfg {
                spar_material: "AA6061-T6".to_string(),
                frame_spacing_mm: 310.0,
                design_category: "normal".to_string(),
            },
            // cooling_drag_fraction levemente diferente do baseline real
            // (0.04) — mesma justificativa de "nenhum destes números
            // coincide com o baseline real" usada nas demais seções desta
            // fixture.
            drag: DragCfg { cd0_misc: 0.0032, cooling_drag_fraction: 0.035 },
            // Levemente diferente do baseline real (0.05/0.25) — mesma
            // justificativa de "nenhum destes números coincide com o
            // baseline real" usada nas demais seções desta fixture.
            stability: StabilityCfg { sm_min: 0.06, sm_max: 0.28 },
            masses: MassesCfg {
                items: vec![
                    MassItemCfg { name: "psru_helice_capo".into(),   mass_kg: 62.0,  arm_ref: "engine_cg".into(),       arm_offset_m: 0.3 },
                    MassItemCfg { name: "resfriamento".into(),       mass_kg: 17.0,  arm_ref: "engine_cg".into(),       arm_offset_m: 0.5 },
                    MassItemCfg { name: "avionicos".into(),          mass_kg: 58.0,  arm_ref: "avionics".into(),        arm_offset_m: 0.0 },
                    MassItemCfg { name: "painel_comandos".into(),    mass_kg: 24.0,  arm_ref: "pax_front".into(),       arm_offset_m: -0.3 },
                    MassItemCfg { name: "fuselagem".into(),          mass_kg: 150.0, arm_ref: "fuselage_struct".into(), arm_offset_m: 0.0 },
                    MassItemCfg { name: "asa".into(),                mass_kg: 120.0, arm_ref: "wing_struct".into(),     arm_offset_m: 0.0 },
                    MassItemCfg { name: "emp_horizontal".into(),     mass_kg: 21.0,  arm_ref: "empennage_cg".into(),    arm_offset_m: 0.0 },
                    MassItemCfg { name: "emp_vertical".into(),       mass_kg: 15.0,  arm_ref: "empennage_cg".into(),    arm_offset_m: -0.2 },
                    MassItemCfg { name: "trem_principal".into(),     mass_kg: 52.0,  arm_ref: "gear_main".into(),       arm_offset_m: 0.0 },
                    MassItemCfg { name: "trem_nariz".into(),         mass_kg: 21.0,  arm_ref: "gear_nose".into(),       arm_offset_m: 0.0 },
                    MassItemCfg { name: "mobiliario".into(),         mass_kg: 42.0,  arm_ref: "pax_front".into(),       arm_offset_m: 0.5 },
                    MassItemCfg { name: "tanques".into(),            mass_kg: 11.0,  arm_ref: "fuel_cg".into(),         arm_offset_m: 0.0 },
                    MassItemCfg { name: "cabos_hidraulico".into(),   mass_kg: 19.0,  arm_ref: "fuselage_struct".into(), arm_offset_m: 0.0 },
                    MassItemCfg { name: "portas_vidros".into(),      mass_kg: 26.0,  arm_ref: "pax_front".into(),       arm_offset_m: 0.0 },
                    MassItemCfg { name: "antepara_firewall".into(),  mass_kg: 11.0,  arm_ref: "engine_cg".into(),       arm_offset_m: 0.9 },
                ],
            },
            // Frações levemente diferentes do baseline real
            // (config/aircraft/baseline_4seat.toml) — mesma justificativa de
            // "nenhum destes números coincide com o baseline real" usada nas
            // demais seções desta fixture.
            control_surfaces: ControlSurfacesCfg {
                aileron_span_start_frac: 0.58,
                aileron_span_end_frac: 0.92,
                aileron_chord_frac: 0.24,
                flap_span_start_frac: 0.12,
                flap_span_end_frac: 0.48,
                flap_chord_frac: 0.28,
                elevator_span_frac: 0.88,
                elevator_chord_frac: 0.33,
                rudder_span_frac: 0.85,
                rudder_chord_frac: 0.32,
            },
            // Levemente diferente do baseline real (0.40/0.30/0.75/1.0/1.5/3.0)
            // — mesma justificativa de "nenhum destes números coincide com o
            // baseline real" usada nas demais seções desta fixture.
            performance: PerformanceCfg {
                mu_brake_paved: 0.38,
                mu_brake_grass: 0.28,
                static_thrust_factor: 0.72,
                rotation_time_s: 1.2,
                flare_time_s: 1.4,
                approach_angle_deg: 3.2,
            },
            // Orçamento elétrico sintético (Task 5.2) — valores levemente
            // diferentes do baseline real (mesma justificativa de "nenhum
            // destes números coincide com o baseline real"). trem_retratil
            // peak_w (480 W) respeita a guarda de consistência: atuador
            // mecânico calculado para esta fixture (mass_main_leg_kg=26.0,
            // retraction_time_s=7.5) ≈ 16.3 W, bem abaixo de 480 W.
            electrical: ElectricalCfg {
                bus_voltage_v: 28.0,
                alternator_w: 850.0,
                loads: vec![
                    ElectricalLoadCfg { name: "avionicos".into(),          continuous_w: 170.0, peak_w: 210.0 },
                    ElectricalLoadCfg { name: "luzes_nav_strobe".into(),   continuous_w: 40.0,  peak_w: 85.0 },
                    ElectricalLoadCfg { name: "bomba_combustivel".into(),  continuous_w: 55.0,  peak_w: 110.0 },
                    ElectricalLoadCfg { name: "trem_retratil".into(),      continuous_w: 0.0,   peak_w: 480.0 },
                    ElectricalLoadCfg { name: "flaps".into(),              continuous_w: 0.0,   peak_w: 140.0 },
                    ElectricalLoadCfg { name: "pitot_aquecido".into(),     continuous_w: 85.0,  peak_w: 85.0 },
                    ElectricalLoadCfg { name: "radio_transponder".into(),  continuous_w: 50.0,  peak_w: 65.0 },
                ],
            },
        }
    }
}
