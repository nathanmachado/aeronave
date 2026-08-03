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
    /// CD0 residual (antenas, juntas, imperfeições) — não pertence a nenhum
    /// componente específico da aeronave.
    pub drag: DragCfg,
    pub masses: MassesCfg,
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PropellerCfg {
    pub diameter_m: f64,
    pub blades: u32,
    pub psru_ratio: f64,
    pub psru_efficiency: f64,
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
                diameter_m: 1.90,
                blades: 2,
                psru_ratio: 2.0,
                psru_efficiency: 0.965,
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
            drag: DragCfg { cd0_misc: 0.0032 },
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
        }
    }
}
