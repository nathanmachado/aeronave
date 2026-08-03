use serde::{Deserialize, Serialize};

/// Saída do AerodynamicsAgent
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WingSpec {
    pub span_m: f64,
    pub area_m2: f64,
    pub aspect_ratio: f64,
    pub airfoil: String,
    pub taper_ratio: f64,
    /// Espessura relativa do perfil (t/c) — usada no dimensionamento da
    /// longarina em `structural.rs`.
    pub thickness_ratio: f64,
    pub oswald_efficiency: f64,
    pub cd0: f64,
    pub cl_cruise: f64,
    pub cd_cruise: f64,
    /// CL_max com flap/slat (configuração de pouso/decolagem) — usado nas
    /// distâncias de decolagem/pouso (performance.rs).
    pub cl_max: f64,
    /// CL_max em configuração limpa (cruzeiro, sem flap).
    pub cl_max_clean: f64,
    /// VS0 — velocidade de stall com flap (configuração de pouso), km/h.
    pub stall_speed_flaps_kmh: f64,
    /// VS1 — velocidade de stall em configuração limpa, km/h.
    pub stall_speed_clean_kmh: f64,
    pub ld_ratio_cruise: f64,
}

/// Saída do PropulsionAgent
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PropulsionSpec {
    pub engine_model: String,
    pub power_hp: f64,
    pub power_kw: f64,
    pub max_torque_nm: f64,
    pub rated_rpm: f64,
    /// Massa do motor (kg) — vem de `EngineSpec::mass_kg` (consumido pela Task 1.5).
    pub engine_mass_kg: f64,
    pub psru_ratio: f64,
    /// RPM do motor no ponto de cruzeiro escolhido pela busca (ver `search_cruise_rpm`).
    pub engine_rpm_cruise: f64,
    pub prop_rpm_cruise: f64,
    pub prop_diameter_m: f64,
    pub fuel_type: String,
    pub fuel_capacity_l: f64,
    pub fc_cruise_lph: f64,
    pub bsfc_cruise_gkwh: f64,
    pub endurance_h: f64,
    pub range_km: f64,
    pub prop_efficiency: f64,
    pub thrust_cruise_n: f64,
    /// Potência requerida em voo nivelado no rpm/altitude de cruzeiro escolhido (kW).
    pub p_req_cruise_kw: f64,
    /// Potência de eixo disponível no rpm/altitude de cruzeiro escolhido (kW).
    pub p_shaft_cruise_kw: f64,
    /// true se `p_req_cruise_kw <= p_shaft_cruise_kw` no rpm de cruzeiro escolhido
    /// pela busca — ou seja, se o motor sustenta a velocidade de cruzeiro exigida.
    pub cruise_feasible: bool,
}

/// Saída do EmpennageAgent — dimensionamento de S_h/S_v por coeficiente de
/// volume (Raymer Tab. 6.4). Consumida por `weight_balance::neutral_point_m`
/// (Task 4.1) e ecoada no relatório final.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmpennageSpec {
    pub s_horizontal_m2: f64,
    pub s_vertical_m2: f64,
    /// Braço da empenagem horizontal (CA asa → CA empenagem, m).
    pub arm_h_m: f64,
    /// Braço da empenagem vertical (CA asa → CA empenagem, m).
    pub arm_v_m: f64,
    pub span_h_m: f64,
    pub span_v_m: f64,
    pub chord_h_root_m: f64,
    pub chord_h_tip_m: f64,
    pub chord_v_root_m: f64,
    pub chord_v_tip_m: f64,
    pub ar_h: f64,
    pub ar_v: f64,
    pub taper_h: f64,
    pub taper_v: f64,
    /// Coeficiente de volume horizontal usado no dimensionamento — ecoa
    /// `[empennage].v_h` da configuração, para o relatório.
    pub volume_h: f64,
    /// Coeficiente de volume vertical usado no dimensionamento — ecoa
    /// `[empennage].v_v` da configuração.
    pub volume_v: f64,
    /// Eficiência de pressão dinâmica na empenagem horizontal (q_t/q_∞) —
    /// ecoa `[empennage].eta_h`; usada por `weight_balance::neutral_point_m`
    /// sem que essa função precise acessar `AircraftConfig` diretamente.
    pub eta_h: f64,
}

/// Geometria física de UMA superfície de controle (m/m²) — saída de
/// `agents::control_surfaces::ControlSurfacesAgent`.
///
/// Convenção UNIFICADA de `span_m`/`start_m`/`end_m`/`area_m2` (ver
/// docstring do módulo do agente para a dedução algébrica completa):
///
///   - **Superfícies ESPELHADAS** (aileron, flap — asa; elevator/profundor
///     — EH): `span_m`, `start_m` e `end_m` são medidos POR LADO (a
///     superfície existe idêntica nos dois lados, esquerdo e direito, por
///     simetria) — `start_m`/`end_m` a partir da LINHA DE CENTRO da
///     superfície-mãe (0 = linha de centro; `end_m` nunca ultrapassa a
///     SEMI-envergadura da superfície-mãe: `wing.span_m/2` para
///     aileron/flap, `emp.span_h_m/2` para o profundor). `area_m2` é a área
///     TOTAL dos dois lados somados (2 × área de um lado).
///   - **Superfície ÚNICA** (rudder/leme — EV, painel não-espelhado):
///     `span_m`/`start_m`/`end_m` medidos a partir da RAIZ (base da deriva,
///     0 = raiz; `end_m` até `rudder_span_frac · span_v_m`). `area_m2` já é
///     a área total (não há segundo lado a somar).
///
/// Um consumidor de CAD deve tratar `start_m`/`end_m` como distância a
/// partir da linha de centro (superfícies espelhadas) OU da raiz (leme) —
/// NUNCA como a largura ponta-a-ponta da superfície (achado da revisão da
/// Task 4.2: a versão original reportava `elevator.end_m` como a largura
/// ponta-a-ponta do profundor, ~1.8× a semi-envergadura real do EH,
/// posicionando a superfície fora do estabilizador se lida como
/// distância-da-linha-de-centro).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SurfaceGeom {
    pub span_m: f64,
    pub area_m2: f64,
    pub chord_mean_m: f64,
    pub start_m: f64,
    pub end_m: f64,
}

/// Saída do ControlSurfacesAgent (Task 4.2) — dimensionamento de aileron,
/// flap, profundor (elevator) e leme (rudder) por razões históricas
/// (Raymer Tab. 6.5), parametrizadas em `[control_surfaces]` no TOML de
/// aeronave. Puramente geométrico (não depende de peso/MTOW).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ControlSurfacesSpec {
    pub aileron: SurfaceGeom,
    pub flap: SurfaceGeom,
    pub elevator: SurfaceGeom,
    pub rudder: SurfaceGeom,
}

/// Saída do WeightBalanceAgent (preenchida na Fase seguinte)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WeightSpec {
    pub oew_kg: f64,
    pub mtow_kg: f64,
    pub payload_kg: f64,
    pub fuel_mass_kg: f64,
    pub cg_mac_fwd_pct: f64,
    pub cg_mac_aft_pct: f64,
    pub static_margin_pct: f64,
}

/// Saída do PerformanceAgent (preenchida na Fase seguinte)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceSpec {
    pub v_cruise_kmh: f64,
    pub v_stall_kmh: f64,
    pub rc_sl_ms: f64,
    pub rc_cruise_alt_ms: f64,
    pub service_ceiling_m: f64,
    pub to_distance_paved_m: f64,
    pub to_distance_grass_m: f64,
    pub landing_distance_m: f64,
    pub range_km: f64,
    pub endurance_h: f64,
}

/// Saída do StructuralAgent
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StructuralSpec {
    /// Fator de carga de PROJETO usado para dimensionar a estrutura —
    /// `VnDiagramSpec::n_design` (Task 4.3): `max(n_lim_pos, n_gust_vc,
    /// n_gust_vc_light)`. Pode SUPERAR o fator de manobra da categoria
    /// CS-23 (Normal = 3.8g) quando a condição de rajada em carga alar
    /// baixa (CS 23.341) governa — ver `agents::vn_diagram`.
    pub design_load_factor_g: f64,
    /// Fator último = 1.5 × `design_load_factor_g`
    pub ultimate_load_factor_g: f64,
    /// Momento fletor na raiz da asa — carga limite (N·m)
    pub wing_root_bending_limit_nm: f64,
    /// Momento fletor na raiz da asa — carga última (N·m)
    pub wing_root_bending_ult_nm: f64,
    /// Material das longarinas
    pub spar_material: String,
    /// Altura da longarina na raiz (m)
    pub spar_height_root_m: f64,
    /// Área de mesa da longarina necessária (cm²)
    pub spar_flange_area_cm2: f64,
    /// Espessura da alma da longarina (mm)
    pub spar_web_thickness_mm: f64,
    /// Espessura mínima da pele (composto — mm)
    pub skin_min_thickness_mm: f64,
    /// Espaçamento de cavernas da fuselagem (mm)
    pub frame_spacing_mm: f64,
    /// Velocidade de flutter estimada (km/h) — deve ser > 1.20 × VD
    pub flutter_speed_kmh: f64,
    /// Velocidade de mergulho de projeto VD (km/h)
    pub design_dive_speed_kmh: f64,
    /// Velocidade de manobra VA (km/h) — CS 23.335, calculada com VS1 (limpa)
    pub va_kmh: f64,
    /// Vida em fadiga estimada (ciclos de voo)
    pub fatigue_life_cycles: f64,
    /// Verificação: flutter OK?
    pub flutter_ok: bool,
}

/// Saída do VnDiagramAgent (Task 4.3) — diagrama V-n completo com rajadas
/// (CS 23.333/.335/.337/.341). `n_design` é o fator de carga que governa o
/// dimensionamento estrutural: `max(n_lim_pos, n_gust_vc, n_gust_vc_light)`
/// — pode exceder o fator de manobra quando a condição de rajada em carga
/// alar baixa (cenário mais leve) governa (ver docstring do módulo
/// `agents::vn_diagram`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VnDiagramSpec {
    /// Velocidade de manobra VA (km/h) — CS 23.335, VS1×√n_lim_pos.
    pub va_kmh: f64,
    /// Velocidade de rajada de projeto VB (km/h) — simplificação de projeto
    /// preliminar, ver docstring de `VnDiagramAgent::run`.
    pub vb_kmh: f64,
    /// Velocidade de cruzeiro de projeto VC (km/h) — do requisito de missão.
    pub vc_kmh: f64,
    /// Velocidade de mergulho de projeto VD (km/h) — 1.25×VC.
    pub vd_kmh: f64,
    /// Fator de carga limite de manobra positivo (CS 23.337).
    pub n_lim_pos: f64,
    /// Fator de carga limite de manobra negativo (CS 23.337).
    pub n_lim_neg: f64,
    /// Fator de carga de rajada em VC, massa de envelope (CS 23.341).
    pub n_gust_vc: f64,
    /// Fator de carga de rajada em VD, massa de envelope (CS 23.341).
    pub n_gust_vd: f64,
    /// Fator de carga de rajada em VC, massa do cenário MAIS LEVE — carga
    /// alar baixa pode fazer a rajada governar (CS 23.341).
    pub n_gust_vc_light: f64,
    /// Fator de carga de PROJETO — o que efetivamente dimensiona a
    /// estrutura: `max(n_lim_pos, n_gust_vc, n_gust_vc_light)`.
    pub n_design: f64,
    /// Polígono do envelope [V_kmh, n] para plotagem/CAD — ver docstring de
    /// `envelope_polygon` em `agents::vn_diagram` para a convenção exata.
    pub points: Vec<[f64; 2]>,
}

/// Saída do LandingGearAgent
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GearSpec {
    /// Tipo de trem
    pub gear_type: String,
    /// Bitola do trem principal (distância entre rodas, m)
    pub track_width_m: f64,
    /// Distância entre eixos (empeno, m)
    pub wheelbase_m: f64,
    /// Ângulo anti-tombamento lateral (< 55°)
    pub tipover_angle_deg: f64,
    /// Fração de carga no trem de nariz (ideal: 8–20%)
    pub nose_load_fraction_pct: f64,
    /// Carga máxima no trem principal (N) — por perna
    pub main_gear_load_n: f64,
    /// Carga máxima no trem de nariz (N)
    pub nose_gear_load_n: f64,
    /// Curso do amortecedor principal (mm)
    pub main_oleo_stroke_mm: f64,
    /// Curso do amortecedor de nariz (mm)
    pub nose_oleo_stroke_mm: f64,
    /// Pneu do trem principal
    pub main_tire: String,
    /// Pneu do trem de nariz
    pub nose_tire: String,
    /// Pressão dos pneus (psi)
    pub tire_pressure_psi: f64,
    /// Taxa de afundamento máxima de projeto (m/s)
    pub max_sink_rate_ms: f64,
    /// Tempo de retração/extensão (s)
    pub retraction_time_s: f64,
    /// Potência do atuador elétrico (W)
    pub actuator_power_w: f64,
    /// Peso total do sistema de trem (kg)
    pub total_weight_kg: f64,
}

/// Relatório completo de validação — saída do Orchestrator
#[derive(Debug, Serialize, Deserialize)]
pub struct AircraftReport {
    pub revision: String,
    pub validation_status: String,
    pub wing: WingSpec,
    pub propulsion: PropulsionSpec,
    pub empennage: Option<EmpennageSpec>,
    pub control_surfaces: Option<ControlSurfacesSpec>,
    pub weight: Option<WeightSpec>,
    pub performance: Option<PerformanceSpec>,
    pub vn_diagram: Option<VnDiagramSpec>,
    pub structure: Option<StructuralSpec>,
    pub landing_gear: Option<GearSpec>,
    pub violations: Vec<String>,
}
