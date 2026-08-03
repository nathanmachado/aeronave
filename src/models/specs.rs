use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::agents::constraint_diagram::WingLoadingReport;

/// (De)serialização de `StructuralSpec::fatigue_life_cycles` (Task 6.1,
/// achado da própria checagem de round-trip deste schema).
///
/// `agents::structural::fatigue_life_cycles` retorna legitimamente
/// `f64::INFINITY` quando a tensão equivalente fica abaixo do limite de
/// fadiga (Se) — "vida infinita" é o resultado físico correto do modelo de
/// Goodman, não um erro. Mas o serializador padrão de `serde_json`
/// (RFC 8259 não tem representação de infinito/NaN em JSON) converte
/// `Infinity` silenciosamente para `null`, o que quebra a desserialização
/// de volta em `f64` (`null` não é um `f64` válido) — um consumidor de CAD
/// batendo `serde_json::from_str::<AircraftReport>` no schema oficial
/// falharia sempre que a longarina caísse abaixo do limite de fadiga.
/// Este módulo serializa o caso infinito explicitamente como a string
/// `"infinita"` (documentado em `docs/aircraft_spec.schema.md`) em vez de
/// deixar o valor virar `null` sem aviso.
mod fatigue_life_serde {
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    pub fn serialize<S>(value: &f64, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        if value.is_infinite() && *value > 0.0 {
            serializer.serialize_str("infinita")
        } else {
            value.serialize(serializer)
        }
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<f64, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        #[serde(untagged)]
        enum NumOrInfinita {
            Num(f64),
            Str(String),
        }
        match NumOrInfinita::deserialize(deserializer)? {
            NumOrInfinita::Num(n) => Ok(n),
            NumOrInfinita::Str(s) if s == "infinita" => Ok(f64::INFINITY),
            NumOrInfinita::Str(s) => Err(serde::de::Error::custom(format!(
                "valor inesperado para fatigue_life_cycles: '{s}' (esperado um número ou \"infinita\")"
            ))),
        }
    }
}

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
    /// INFORMATIVO (a tanque cheio, consumo constante no ponto de cruzeiro
    /// escolhido) — não é mais a fonte dos gates de autonomia/alcance do
    /// projeto desde a Task 5.1 (achado da revisão dessa task, Finding 1):
    /// esses gates agora usam `MissionSpec::block_time_h` (análise por
    /// segmentos, `ConstraintChecker::verify`). Mantido aqui só para
    /// referência ("quanto tempo o tanque cheio dura neste ponto de
    /// cruzeiro", não "a missão cumpre o requisito").
    pub endurance_h: f64,
    /// INFORMATIVO (a tanque cheio, consumo constante) — mesma ressalva de
    /// `endurance_h` acima; o gate de alcance do projeto usa
    /// `MissionSpec::range_no_wind_km` desde a Task 5.1.
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
    /// CG mais dianteiro OBSERVADO entre os cenários de carga (%MAC) — não
    /// confundir com `cg_limit_fwd_pct_mac`, que é o limite ADMISSÍVEL
    /// (Task 4.4).
    pub cg_mac_fwd_pct: f64,
    /// CG mais traseiro OBSERVADO entre os cenários de carga (%MAC) — não
    /// confundir com `cg_limit_aft_pct_mac`, que é o limite ADMISSÍVEL.
    pub cg_mac_aft_pct: f64,
    pub static_margin_pct: f64,
    /// Limite DIANTEIRO do envelope de CG admissível (%MAC) — vem de
    /// `stability.sm_max` (proxy de autoridade de profundor). CG à frente
    /// deste limite excede a margem estática máxima aceitável.
    pub cg_limit_fwd_pct_mac: f64,
    /// Limite TRASEIRO do envelope de CG admissível (%MAC) — vem de
    /// `stability.sm_min` (piso de estabilidade estática). CG atrás deste
    /// limite fica abaixo da margem estática mínima aceitável.
    pub cg_limit_aft_pct_mac: f64,
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
    /// INFORMATIVO — eco de `PropulsionSpec::range_km` (a tanque cheio,
    /// consumo constante). Não é a fonte dos gates de alcance do projeto
    /// desde a Task 5.1 — ver `PropulsionSpec::range_km`.
    pub range_km: f64,
    /// INFORMATIVO — eco de `PropulsionSpec::endurance_h`. Mesma ressalva
    /// de `range_km` acima.
    pub endurance_h: f64,
    // ─── Task 4.7: Vx/Vy, planeio, gradiente CS 23.65, distâncias sobre 15m ──
    /// Velocidade de MELHOR ÂNGULO de subida (km/h) — maximiza RC(V)/V, não
    /// RC(V) absoluto (isso é `vy_kmh`). Sempre < `vy_kmh`.
    pub vx_kmh: f64,
    /// Velocidade de MELHOR RAZÃO de subida (km/h) — maximiza RC(V).
    pub vy_kmh: f64,
    /// Velocidade de melhor planeio (km/h) — `V_bg = √(2W/ρS)·(K/CD0)^0.25`.
    pub best_glide_kmh: f64,
    /// Razão L/D máxima (planeio) — `1/(2√(K·CD0))`, K = 1/(π·AR·e).
    pub glide_ratio: f64,
    /// Gradiente de subida máximo (%) — `100·RC(Vx)/Vx`, avaliado no solo
    /// (MTOW). CS 23.65 exige ≥ 8.3% para esta categoria.
    pub climb_gradient_pct: f64,
    /// Distância de decolagem sobre obstáculo de 15m/50ft (pista pavimentada,
    /// m) — soma de segmentos: ground roll + rotação + subida até 15m.
    pub to_50ft_paved_m: f64,
    /// Distância de decolagem sobre obstáculo de 15m/50ft (grama/terra, m).
    pub to_50ft_grass_m: f64,
    /// Distância de pouso sobre obstáculo de 15m/50ft (pista pavimentada, m)
    /// — soma de segmentos: aproximação (γ padrão) + flare + ground roll.
    pub ldg_50ft_m: f64,
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
    /// Vida em fadiga estimada (ciclos de voo) — pode ser infinita (abaixo
    /// do limite de fadiga do material); serializada como a string
    /// `"infinita"` nesse caso, não `null` — ver `fatigue_life_serde`.
    #[serde(with = "fatigue_life_serde")]
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

/// Saída do PropellerAgent (Task 4.5) — dimensionamento/validação da hélice
/// por Mach de ponta de pá (estático e cruzeiro) e folga de solo (CS
/// 23.925). Quando `[propeller].diameter_m` está presente na configuração,
/// `diameter_m` ecoa esse valor (`source = "config"`); quando omitido, é o
/// maior diâmetro que respeita simultaneamente os dois limites de Mach e a
/// folga mínima de solo, com margem de segurança (`source = "derivado"`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PropellerSpec {
    pub diameter_m: f64,
    pub blades: u32,
    /// `"config"` quando `diameter_m` vem direto do TOML, `"derivado"`
    /// quando calculado pelo `PropellerAgent`.
    pub source: String,
    /// Mach de ponta de pá em condição ESTÁTICA (rpm nominal do motor via
    /// PSRU, V=0, no aeródromo).
    pub tip_mach_static: f64,
    /// Mach de ponta de pá em CRUZEIRO (composição helicoidal: velocidade
    /// tangencial da ponta + velocidade de avanço).
    pub tip_mach_cruise_helical: f64,
    /// Folga entre a ponta da pá e o solo (m) — `shaft_height_m − diameter_m/2`.
    pub ground_clearance_m: f64,
    /// Maior diâmetro (m) que respeita AMBOS os limites de Mach de ponta
    /// (estático e cruzeiro) — o menor dos dois máximos individuais.
    pub diameter_max_by_mach_m: f64,
    /// Maior diâmetro (m) que respeita a folga mínima de solo.
    pub diameter_max_by_clearance_m: f64,
    pub ok_mach_static: bool,
    pub ok_mach_cruise: bool,
    pub ok_clearance: bool,
}

/// Saída do MissionAgent (Task 5.1) — análise de missão por segmentos
/// (táxi, subida integrada, cruzeiro Breguet, descida, reserva), que
/// substitui o modelo antigo de consumo constante
/// (`fc_cruise_lph · endurance_min_h`) na determinação do combustível de
/// missão consumido pelo laço de convergência de MTOW
/// (`orchestrator::size_aircraft`). Ver `agents::mission` para a dedução
/// completa de cada segmento.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MissionSpec {
    /// Combustível de táxi + run-up (kg) — `analysis.taxi_fuel_l` × densidade.
    pub fuel_taxi_kg: f64,
    /// Combustível queimado durante a subida integrada (kg) — soma dos
    /// passos de 100m entre `airfield_altitude_m` e `cruise_altitude_m`, a
    /// potência de rpm_max_continuous (carga plena).
    pub fuel_climb_kg: f64,
    /// Combustível queimado em cruzeiro (kg) — equação de Breguet, massa
    /// decrescente ao longo da distância de cruzeiro (não consumo
    /// constante × tempo).
    pub fuel_cruise_kg: f64,
    /// Combustível queimado na descida (kg) — potência parcial
    /// (`analysis.descent_power_fraction` × vazão de cruzeiro) × tempo de
    /// descida.
    pub fuel_descent_kg: f64,
    /// Reserva (kg) — `req.fuel_reserve_fraction` × (táxi+subida+cruzeiro+
    /// descida), fração sobre o consumo da missão (não sobre o total com
    /// reserva incluída).
    pub fuel_reserve_kg: f64,
    /// Combustível total da missão (kg) — soma de todos os segmentos acima
    /// + reserva. É este valor (convertido para litros) que o laço de
    /// convergência de MTOW usa como `fuel_req_l`.
    pub fuel_total_kg: f64,
    /// `fuel_total_kg` convertido para litros pela densidade do combustível
    /// do motor — comparado contra `[fuel_system].capacity_l` no ponto
    /// convergido (`SizingError::CombustivelInsuficiente`).
    pub fuel_total_l: f64,
    /// Duração da subida integrada (minutos).
    pub climb_time_min: f64,
    /// Distância horizontal percorrida durante a subida (km) — aproximação
    /// de pequeno ângulo (`d ≈ V_y·t`, ignora o cosseno do ângulo de
    /// subida).
    pub climb_distance_km: f64,
    /// Distância horizontal percorrida durante a descida (km) — mesma
    /// aproximação de pequeno ângulo, à velocidade de cruzeiro.
    pub descent_distance_km: f64,
    /// Distância de cruzeiro (km) — `alcance_total_exigido − subida − descida`,
    /// consumida pela equação de Breguet para determinar `fuel_cruise_kg`.
    pub cruise_distance_km: f64,
    /// Tempo total de voo (subida + cruzeiro + descida, horas) — NÃO inclui
    /// o táxi (modelado só como combustível fixo, sem duração explícita).
    pub block_time_h: f64,
    /// Alcance sem vento (km) — soma dos três segmentos de distância
    /// (subida + cruzeiro + descida), recomputado a partir dos segmentos
    /// (não um eco direto de `cruise_speed_min_kmh · endurance_min_h`) como
    /// checagem de consistência interna; por construção, igual ao alcance
    /// exigido dentro de tolerância de ponto flutuante, já que
    /// `cruise_distance_km` é justamente o que falta para fechar essa soma.
    pub range_no_wind_km: f64,
    /// Informativo: alcance Breguet SE o tanque cheio inteiro fosse
    /// queimado em cruzeiro (não a missão real, que reserva parte do
    /// tanque para táxi/subida/descida/reserva) — mostra o alcance máximo
    /// deste modelo. Endpoints coerentes (Finding 3 da revisão da Task
    /// 5.1): `w0 = ZFW + tanque cheio`, `w1 = ZFW` (peso vazio de
    /// combustível — OEW + payload), não o MTOW da missão real (que só
    /// carrega o combustível da missão, não o tanque cheio) menos o peso
    /// do tanque cheio, que produzia `w1 < ZFW` — fisicamente incoerente
    /// (queimaria mais combustível do que a aeronave tem capacidade de
    /// carregar).
    pub breguet_range_full_tank_km: f64,
}

/// Saída do ElectricalAgent (Task 5.2) — orçamento elétrico: soma das
/// cargas configuradas (`[electrical].loads`) contra a capacidade do
/// alternador (`[electrical].alternator_w`). Pura soma/derivação — não
/// depende de MTOW nem de nenhum outro agente.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ElectricalSpec {
    pub bus_voltage_v: f64,
    pub alternator_w: f64,
    /// Soma das potências CONTÍNUAS de todas as cargas configuradas (W).
    pub continuous_load_w: f64,
    /// Soma das potências de PICO de todas as cargas configuradas (W) —
    /// modelo conservador de "pior caso, tudo ligado ao mesmo tempo"
    /// (`Σ peak_w`), não Σcontínuo + maior pico individual. Superestima o
    /// pico real simultâneo (nem toda carga atinge seu pico ao mesmo
    /// tempo — ex.: trem retrátil só pica durante a retração, não durante
    /// cruzeiro com pitot aquecido ligado), de propósito: é uma checagem
    /// de margem conservadora, não uma previsão de carga instantânea real.
    pub peak_load_w: f64,
    /// Margem sobre a capacidade CONTÍNUA do alternador (%):
    /// `(alternator_w − continuous_load_w) / alternator_w × 100`.
    pub margin_continuous_pct: f64,
}

/// Versão do schema JSON (`AircraftReport`) — contrato com o time de CAD
/// (`docs/aircraft_spec.schema.md`). Política de versionamento:
///   - Bump de MINOR (ex.: 4.0 → 4.1): mudança aditiva (novo campo opcional,
///     novo bloco) — consumidores existentes continuam funcionando sem
///     alteração.
///   - Bump de MAJOR (ex.: 4.0 → 5.0): mudança que quebra compatibilidade
///     (renomeia/remove campo, muda tipo ou unidade de um campo existente)
///     — consumidores precisam ser atualizados.
///
/// v4.0 (Task 6.1): adiciona `schema_version`, `geometry`, `sizing`,
/// `fidelity`, `warnings` ao relatório v3 (que só tinha `revision` como
/// string de versão livre, sem política declarada).
pub const SCHEMA_VERSION: &str = "4.0";

/// Geometria consolidada para consumo do CAD paramétrico — todas as
/// posições em metros do DATUM (ponta do nariz, x positivo para trás — ver
/// `docs/aircraft_spec.schema.md` para a convenção de eixos completa).
/// Campos que já existiam internamente (`WeightBalanceOutput`,
/// `AircraftConfig`) mas não eram ecoados no JSON antes da Task 6.1.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeometrySpec {
    /// Posição do bordo de ataque da raiz da asa (m do datum) — única fonte
    /// desta posição na configuração (`[wing].le_root_x_m`).
    pub wing_le_root_x_m: f64,
    /// Corda na raiz da asa (m).
    pub chord_root_m: f64,
    /// Corda na ponta da asa (m).
    pub chord_tip_m: f64,
    /// Corda Aerodinâmica Média — MAC (m).
    pub mac_m: f64,
    /// Posição do bordo de ataque do MAC (m do datum).
    pub mac_le_x_m: f64,
    /// Distância da raiz à seção do MAC, medida na envergadura (m) —
    /// `y_MAC = (b/6)·(1+2λ)/(1+λ)` (ver `agents::weight_balance::
    /// mac_spanwise_pos`).
    pub y_mac_m: f64,
    /// Comprimento total da fuselagem (m).
    pub fuselage_length_m: f64,
    /// Largura interna da cabine (m).
    pub cabin_width_m: f64,
    /// Altura interna da cabine (m).
    pub cabin_height_m: f64,
}

/// Relatório de dimensionamento (Task 6.1) — MTOWs convergido/envelope,
/// histórico de convergência, margem de combustível e o diagrama de
/// restrições clássico (`WingLoadingReport`), até aqui calculados por
/// `orchestrator::size_aircraft` mas não serializados no JSON final.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SizingReport {
    /// MTOW de missão (kg) — peso convergido levando exatamente o
    /// combustível da missão mínima (`SizedAircraft::state.mtow_kg`).
    pub mtow_mission_kg: f64,
    /// MTOW de envelope (kg) — pior caso legal de carregamento ("4 pax +
    /// bagagem + tanque cheio", `SizedAircraft::wb.spec.mtow_kg`); tipicamente
    /// ≥ `mtow_mission_kg`, dimensiona Estrutura/Trem de Pouso.
    pub mtow_envelope_kg: f64,
    /// Trajetória de MTOW do laço de ponto fixo (primeiro palpite → valor
    /// final convergido) — `SizedAircraft::iterations`.
    pub iterations: Vec<f64>,
    /// `true` quando o laço de ponto fixo convergiu dentro do limite de
    /// iterações (sempre `true` quando este `SizingReport` existe — se o
    /// laço não convergisse, `orchestrator::size_aircraft` teria retornado
    /// `SizingError::NaoConvergiu` e `main.rs` nunca chegaria a montar o
    /// relatório final). Mantido explícito para o consumidor de CAD não
    /// precisar inferir isso a partir de `iterations`.
    pub converged: bool,
    /// Combustível exigido pela missão (L) — `MissionSpec::fuel_total_l`.
    pub fuel_required_l: f64,
    /// Capacidade física do tanque configurado (L) — `[fuel_system].capacity_l`.
    pub fuel_capacity_l: f64,
    /// Margem absoluta de combustível no ponto convergido (L):
    /// `fuel_capacity_l − fuel_required_l`.
    pub fuel_margin_l: f64,
    /// Margem de combustível (%): `fuel_margin_l / fuel_capacity_l × 100`.
    pub fuel_margin_pct: f64,
    /// Diagrama de restrições clássico W/S × P/W (Task 3.2) no ponto
    /// convergido — puramente informativo, não redimensiona a aeronave
    /// automaticamente.
    pub constraints: WingLoadingReport,
}

/// Relatório completo de validação — saída do Orchestrator
#[derive(Debug, Serialize, Deserialize)]
pub struct AircraftReport {
    /// Versão do schema — ver `SCHEMA_VERSION` para a política de bump.
    pub schema_version: String,
    /// DEPRECATED (mantido só por compatibilidade com consumidores
    /// anteriores à v4, que liam uma string de revisão livre): mesmo valor
    /// de `schema_version` — novos consumidores devem usar `schema_version`.
    pub revision: String,
    pub validation_status: String,
    pub wing: WingSpec,
    pub propulsion: PropulsionSpec,
    /// Geometria consolidada para o CAD paramétrico (Task 6.1) — ver
    /// `GeometrySpec`.
    pub geometry: Option<GeometrySpec>,
    pub empennage: Option<EmpennageSpec>,
    pub control_surfaces: Option<ControlSurfacesSpec>,
    pub weight: Option<WeightSpec>,
    pub performance: Option<PerformanceSpec>,
    pub vn_diagram: Option<VnDiagramSpec>,
    pub structure: Option<StructuralSpec>,
    pub landing_gear: Option<GearSpec>,
    pub propeller: Option<PropellerSpec>,
    /// Análise de missão por segmentos (Task 5.1) — táxi, subida, cruzeiro
    /// Breguet, descida e reserva. `Option` só por simetria com os demais
    /// campos do relatório (`main.rs` sempre o preenche — o laço de
    /// convergência de MTOW já exige um `MissionSpec` válido para sequer
    /// convergir).
    pub mission: Option<MissionSpec>,
    /// Orçamento elétrico (Task 5.2) — `Option` só por simetria com os
    /// demais campos do relatório; `main.rs` sempre o preenche.
    pub electrical: Option<ElectricalSpec>,
    /// Dimensionamento/convergência de MTOW (Task 6.1) — ver `SizingReport`.
    pub sizing: Option<SizingReport>,
    /// Nível de confiança por bloco do relatório — chave = nome do bloco
    /// (ex.: "wing", "structure"), valor = uma de "preliminary" (estimativa
    /// simplificada, exige análise posterior — FEM, GVT, VLM/CFD conforme o
    /// bloco), "semi-empirical" (curvas/correlações de catálogo ou
    /// literatura, não first-principles puro) ou "computed" (equações
    /// fechadas/segmentadas, sem correlação empírica externa). O time de
    /// CAD deve tratar blocos "preliminary" como precisando de análise
    /// posterior antes de fabricação — ver `docs/aircraft_spec.schema.md`.
    pub fidelity: BTreeMap<String, String>,
    pub violations: Vec<String>,
    /// Avisos do `ConstraintChecker` (Task 6.1) — condições que não violam
    /// nenhum requisito do projeto, mas merecem atenção (ex.: pico elétrico
    /// acima da capacidade do alternador, coberto pela bateria). Antes desta
    /// task só `violations` era serializado — `warnings` existia em
    /// `ConstraintReport` mas era descartado ao montar o JSON final.
    pub warnings: Vec<String>,
}
