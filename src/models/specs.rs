use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};

use crate::agents::constraint_diagram::WingLoadingReport;

/// (De)serialização de `StructuralSpec::fatigue_life_cycles` (Task 6.1,
/// achado da própria checagem de round-trip deste schema). Fix wave ciclo 11
/// (2026-08-10): reutilizado também por `PerformanceSpec::to_50ft_paved_m`/
/// `to_50ft_grass_m` (ciclo 11 task 3, `docs/backlog.md` item 5) — mesmo
/// problema de round-trip, gatilho físico diferente (obstáculo de 15 m
/// inatingível, não limite de fadiga). Ciclo 12 (2026-08-15): estendido
/// também a `PerformanceSpec::to_distance_paved_m`, `to_distance_grass_m`,
/// e `landing_distance_m` (rolagem integrada devolve infinito quando tração
/// ou frenagem insuficientes).
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
    /// CL_max com flap/slat em configuração de POUSO (flap cheio) — usado
    /// nas distâncias de POUSO e no VS0 de referência (`performance.rs`).
    /// Ciclo 7 (task 1): NÃO é mais o CL_max das distâncias de DECOLAGEM
    /// nem da Vr da rotação — esses migraram para `cl_max_to`.
    pub cl_max: f64,
    /// CL_max em configuração limpa (cruzeiro, sem flap).
    pub cl_max_clean: f64,
    /// CL_max em configuração de DECOLAGEM (flap PARCIAL) — DERIVADO
    /// (ciclo 7, task 1) por interpolação linear de deployment de flap:
    ///
    ///   `cl_max_to = cl_max_clean + to_flap_fraction·(cl_max_flaps − cl_max_clean)`
    ///
    /// com a MESMA `[stability].to_flap_fraction` que o `trim_authority` já
    /// aplica ao ΔCm de flap na rotação — uma única fração de deployment
    /// governando os dois efeitos do flap (ver `aircraft_config::
    /// StabilityCfg::to_flap_fraction`). Consumido pela Vr/Vs0 da ROTAÇÃO
    /// (`agents::trim_authority`) e pelas distâncias de DECOLAGEM
    /// (`agents::performance`: `takeoff_ground_roll_m`,
    /// `takeoff_distance_m`, `takeoff_distance_50ft_m`).
    ///
    /// Motivação (campanha E10): antes, a rotação derivava Vr do CLmax de
    /// POUSO enquanto usava o Cm do flap PARCIAL de decolagem — incoerente
    /// —, e as distâncias de decolagem também usavam o CLmax de pouso
    /// (otimistas). Com flap slotted (CLmax 2,2) a Vr modelada ficava 13%
    /// lenta demais (q_r −24%) e o limite dianteiro de rotação explodia:
    /// artefato de modelagem, não física.
    pub cl_max_to: f64,
    /// ΔCD0 do flap PARCIAL de decolagem — DERIVADO (ciclo 8, task 1) pela
    /// MESMA interpolação de deployment de `cl_max_to`:
    ///
    ///   `cd0_flap_to_extra = to_flap_fraction · cd0_flap_delta`
    ///
    /// Consumido por `agents::performance::excess_power_kw` (parâmetro
    /// `cd0_extra`) no segmento de SUBIDA da decolagem
    /// (`takeoff_distance_50ft_m`), no gradiente CS 23.65 em configuração
    /// de decolagem (`best_climb_angle_ms`) e — desde o ciclo 12 — na
    /// própria rolagem de decolagem via `agents::performance::
    /// cd_gear_extended` — fecha a lacuna declarada desde o ciclo 7 ("não
    /// existe modelo de flap na polar deste crate").
    ///
    /// HISTÓRICO `old→new` (ciclo 8 → ciclo 12): até o ciclo 11 esta
    /// docstring afirmava que não havia um campo `cd0_flap_ldg_extra`
    /// equivalente para o pouso, porque a auditoria de call sites do ciclo
    /// 8 (task 1) não encontrou nenhum segmento de pouso que consumisse a
    /// polar de arrasto — a rolagem de pouso era frenagem pura (`S_G =
    /// V_ref²/(2gμ)`, sem termo de arrasto) e a aproximação sobre 15 m usava
    /// um ângulo de aproximação FIXO, não uma razão L/D. Essa conclusão
    /// valia então: nenhuma fórmula fechada daquele momento tinha onde
    /// receber um incremento de CD0 de pouso. **Ela morre no ciclo 12**: a
    /// rolagem de pouso passa a integrar a equação de movimento
    /// (`landing_ground_roll_m`, spec `2026-08-15-ciclo12-solo-honesto`
    /// §5), que consome a polar completa — a sustentação residual do flap
    /// de pouso ALIVIA o peso sobre as rodas e PIORA a frenagem, e o
    /// arrasto do flap CHEIO passa a ter, pela primeira vez, um call site
    /// que o consome: `WingSpec::cd0_flap_ldg_extra` (abaixo). Ver
    /// `WingCfg::cd0_flap_delta` para a mesma história do lado da
    /// configuração.
    pub cd0_flap_to_extra: f64,
    /// ΔCD0 do flap CHEIO (configuração de POUSO) — CAMPO NOVO (ciclo 12,
    /// spec §5.3a). Diferente de `cd0_flap_to_extra` (fração PARCIAL,
    /// `to_flap_fraction · cd0_flap_delta`): este é o delta CHEIO,
    /// `[wing].cd0_flap_delta` sem fração nenhuma — a aeronave rola no pouso
    /// com o flap TOTALMENTE deflexionado (decisão de projeto, spec §5.1:
    /// modelar o flap MANTIDO durante toda a rolagem de frenagem, a
    /// configuração em que CS 23 mede a distância de pouso desta classe).
    ///
    /// Consumido por `agents::performance::cd_gear_extended` (via
    /// `landing_ground_roll_m`/`landing_distance_m`/
    /// `landing_distance_50ft_m`) — primeiro consumidor do delta CHEIO
    /// desde que `cd0_flap_delta` existe (ver docstring `old→new` de
    /// `cd0_flap_to_extra` acima e de `WingCfg::cd0_flap_delta`).
    pub cd0_flap_ldg_extra: f64,
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
    /// Limite DIANTEIRO do envelope de CG admissível (%MAC) — desde a task
    /// trim-authority, vem do `TrimAuthorityAgent` (autoridade FÍSICA de
    /// profundor em flare + rotação de decolagem), não mais do proxy
    /// `stability.sm_max` (removido): `max(TrimSpec::flare_limit_pct_mac,
    /// TrimSpec::rotation_limit_pct_mac)` — AMBOS são números ÚNICOS
    /// aplicados IGUALMENTE a todos os cenários. A flare porque de fato
    /// não depende do peso; a ROTAÇÃO, desde o ciclo 10 (task 2), porque é
    /// avaliada no cenário MAIS LEVE (o mais restritivo) e usada como
    /// envoltória conservadora — o momento da linha de tração matou a
    /// invariância ao peso que valia até o ciclo 9 (ver docstring de
    /// `TrimSpec::rotation_limit_pct_mac` e `agents::trim_authority::
    /// rotation_fwd_limit_m`).
    /// PODE ficar À FRENTE de `cg_limit_aft_pct_mac` — ver essa doc-comment
    /// para o significado de ENVELOPE VAZIO nesse caso.
    pub cg_limit_fwd_pct_mac: f64,
    /// Limite TRASEIRO do envelope de CG admissível (%MAC) — vem de
    /// `stability.sm_min` (piso de estabilidade estática). CG atrás deste
    /// limite fica abaixo da margem estática mínima aceitável.
    ///
    /// **ENVELOPE VAZIO**: quando `cg_limit_fwd_pct_mac > cg_limit_aft_pct_mac`
    /// (baseline real: ~39,9% > ~36,6%), NENHUM CG é admissível — os dois
    /// critérios físicos (autoridade de rotação vs. margem estática mínima)
    /// são mutuamente incompatíveis com esta célula/trem, não apenas com
    /// os cenários de carga observados. `violations` sempre contém um item
    /// dedicado "Envelope de CG VAZIO" nesse caso (`ConstraintChecker::verify`),
    /// distinto das violações por cenário. Causa raiz típica: o trem
    /// principal (`[gear].x_main_m`) fica longe demais do CG — decisão de
    /// layout do trem, não corrigida automaticamente por este pipeline.
    pub cg_limit_aft_pct_mac: f64,
    /// Massas estruturais computadas + fatores de composto usados (Schema
    /// 4.5, Task 5, oew-parametrico) — ver `StructuralMassesSpec`.
    pub structural_masses: StructuralMassesSpec,
}

/// Massas estruturais computadas (ciclo 3, `agents::mass_model` — Raymer
/// 15.2 GA × fatores de composto Tab. 15.4) + os fatores usados
/// (rastreabilidade). Schema 4.5.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StructuralMassesSpec {
    pub asa_kg: f64,
    pub fuselagem_kg: f64,
    pub emp_h_kg: f64,
    pub emp_v_kg: f64,
    pub trem_principal_kg: f64,
    pub trem_nariz_kg: f64,
    pub tanques_kg: f64,
    pub composite_factor_wing: f64,
    pub composite_factor_tail: f64,
    pub composite_factor_fuselage: f64,
    pub composite_factor_gear: f64,
    pub composite_factor_fuel_system: f64,
}

/// Margem de autoridade de ROTAÇÃO na CG e no peso REAIS de UM cenário do
/// `WeightBalanceAgent` (task trim-authority, fix de revisão) — em
/// contraste com `TrimSpec::rotation_limit_pct_mac` (o CG MÍNIMO
/// admissível, número ÚNICO avaliado no cenário mais leve — ver sua
/// docstring), esta margem é avaliada na CG e no `Vr(W)` REAIS de CADA
/// cenário, não no limite: é a checagem EXATA por cenário, enquanto o
/// limite único é a envoltória conservadora. Desde o ciclo 10 (task 2)
/// inclui também a tração `T(Vr(W))` do próprio cenário. Ver
/// `agents::trim_authority::rotation_available_moment_nm`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScenarioTrimLimit {
    /// Nome do cenário — mesmo `ScenarioResult::name` do `WeightBalanceAgent`.
    pub scenario: String,
    /// `(momento nariz-acima DISPONÍVEL − momento nariz-acima NECESSÁRIO) /
    /// NECESSÁRIO × 100`, avaliados na CG e no peso reais deste cenário
    /// (`Vr(W)` desse peso). Negativo = autoridade de profundor
    /// INSUFICIENTE para rotacionar nesta CG/peso — quanto mais negativo,
    /// maior o déficit. Zero exatamente na CG do limite avaliado NO PESO
    /// DESTE cenário; desde o ciclo 10 (task 2) isso já NÃO coincide mais
    /// com `rotation_limit_pct_mac` (o MÁXIMO sobre os cenários), exceto
    /// para o cenário que produz esse máximo.
    ///
    /// CUSTO MEDIDO (ciclo 10, task 2): o momento da linha de tração come
    /// uma fatia real destas margens no baseline — o cenário mais apertado,
    /// "Solo (piloto)", cai de +21,6% para +10,5% — mas NENHUMA fica
    /// negativa: a aeronave continua rotacionando em todos os cenários de
    /// carga.
    pub rotation_authority_margin_pct: f64,
}

/// Sensibilidade do limite de flare a `cl_h_max_down` (±0.05) — a
/// autoridade de download da empenagem com o profundor no batente é um
/// parâmetro semi-empírico (faixa típica 0.5–1.2, Gudmundsson/Roskam) sem
/// medição direta neste projeto; pequenas variações deslocam o limite de
/// flare de forma NÃO desprezível (baseline real: 0,80→~10,7%,
/// 0,85→~7,9%, 0,90→~5,1% MAC). Reportado explicitamente para o
/// consumidor de CAD não tratar `flare_limit_pct_mac` como um número exato
/// — acompanha `fidelity["trim"] == "preliminary"`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrimSensitivity {
    /// `cl_h_max_down − 0.05` usado neste recálculo (perturbação direta do
    /// valor OPERACIONAL/capado, sem recalcular τ/δe).
    pub cl_h_max_down_minus: f64,
    /// Limite de flare (%MAC) recomputado com `cl_h_max_down_minus`.
    pub flare_limit_pct_mac_minus: f64,
    /// `cl_h_max_down + 0.05` usado neste recálculo.
    pub cl_h_max_down_plus: f64,
    /// Limite de flare (%MAC) recomputado com `cl_h_max_down_plus`.
    pub flare_limit_pct_mac_plus: f64,
    /// `[control_surfaces].elevator_deflection_max_deg − 2°` (task
    /// refino-ciclo2, 1a) — segunda dimensão de sensibilidade, agora que a
    /// autoridade é calculada por geometria: recalcula `cl_h_max_down_calc`
    /// (τ/a_t fixos, só δe muda) e o limite de flare resultante.
    pub elevator_deflection_max_deg_minus: f64,
    /// Limite de flare (%MAC) recomputado com `elevator_deflection_max_deg_minus`.
    pub flare_limit_pct_mac_deflection_minus: f64,
    /// `[control_surfaces].elevator_deflection_max_deg + 2°`.
    pub elevator_deflection_max_deg_plus: f64,
    /// Limite de flare (%MAC) recomputado com `elevator_deflection_max_deg_plus`.
    pub flare_limit_pct_mac_deflection_plus: f64,
}

/// Saída do TrimAuthorityAgent (task trim-authority) — limite dianteiro
/// FÍSICO do envelope de CG, derivado da autoridade de profundor nas duas
/// manobras críticas de arfagem nariz-para-cima: flare no pouso
/// (V_ref=1,3·Vs0, flap de pouso, balanço de momentos em torno do CG,
/// FECHADO pela contribuição de sustentação da própria empenagem — ver
/// `agents::trim_authority::cl_h_required_flare`) e rotação na decolagem
/// (Vr=1,1·Vs0(W), flap de decolagem, balanço de momentos em torno do TREM
/// PRINCIPAL). Substitui o antigo proxy `stability.sm_max` (margem
/// estática máxima, sem base física direta em autoridade de controle) —
/// ver `agents::trim_authority` para a dedução completa.
///
/// **Ambos os limites (`flare_limit_pct_mac`/`rotation_limit_pct_mac`) são
/// NÚMEROS ÚNICOS** — mas por motivos DIFERENTES desde o ciclo 10 (task 2).
/// A flare simplesmente não depende do peso. A rotação DEPENDIA e deixou de
/// depender... e voltou a depender: até o ciclo 9 valia a prova de que `W`
/// cancelava exatamente (`q_r(W) ∝ W`, todos os termos de momento
/// proporcionais a `W`); o momento da LINHA DE TRAÇÃO (`T(Vr(W))·z_eixo`)
/// entrou no balanço e NÃO escala com `W`, matando a prova. O número único
/// reportado passou a ser o MÁXIMO dos limites por cenário (que neste
/// modelo cai no mais leve, porque `T/W` cresce quando o peso cai) — uma
/// ENVOLTÓRIA conservadora, não uma identidade algébrica. Variação medida
/// entre os extremos de peso do baseline real: 1,4621 pp de MAC. Ver a re-derivação completa (em português) na
/// docstring de `agents::trim_authority::rotation_fwd_limit_m`.
/// `rotation_margin_per_scenario` carrega uma quantidade DIFERENTE (e
/// exata por cenário): a margem de autoridade avaliada na CG/peso REAIS de
/// cada cenário (ver `ScenarioTrimLimit`).
///
/// `governing`: `"flare"` ou `"rotacao"` — qual dos dois limites ÚNICOS é
/// maior (mais restritivo). ACHADO DE PROJETO honesto (baseline real): a
/// rotação governa (≈39,9% MAC ≫ flare ≈7,9% MAC) e fica À FRENTE do
/// limite traseiro (≈36,6% MAC) — **envelope de CG VAZIO**, ver docstring
/// de `WeightSpec::cg_limit_aft_pct_mac`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrimSpec {
    /// Limite dianteiro de flare (%MAC) — número único, independe do peso.
    pub flare_limit_pct_mac: f64,
    /// Limite dianteiro de rotação (%MAC) — número único, MÁXIMO dos
    /// limites por cenário (o mais restritivo). Depende do peso desde o
    /// ciclo 10 (task 2): o momento da linha de tração `T(Vr(W))·z_eixo`
    /// não escala com `W` e matou a invariância que valia até o ciclo 9
    /// (ver docstring da struct/`agents::trim_authority::
    /// rotation_fwd_limit_m`). Para a margem de autoridade REAL por
    /// cenário (exata, na CG/peso de cada um), ver
    /// `rotation_margin_per_scenario`.
    ///
    /// `old→new` (ciclo 13, spec §7 — fecha o backlog #16). Antes deste
    /// ciclo este campo era calculado com `[performance].mu_roll_paved`,
    /// enquanto as checagens #23/#24 (`ConstraintChecker`) já reprovavam a
    /// decolagem/pouso em GRAMA — o mesmo JSON afirmava duas superfícies
    /// para a MESMA decolagem. Agora as duas superfícies são calculadas
    /// (`rotation_limit_pct_mac_paved`/`rotation_limit_pct_mac_grass`
    /// abaixo) e este campo passa a valer o da superfície de OPERAÇÃO —
    /// GRAMA, a mesma que #23/#24 medem e que o TOML de missão descreve
    /// ("grama/terra, pista de fazenda típica"). Mudança de VALOR e de
    /// SIGNIFICADO do campo (spec §9.1).
    pub rotation_limit_pct_mac: f64,
    /// Limite dianteiro de rotação (%MAC) avaliado com
    /// `[performance].mu_roll_paved` — pista pavimentada. Campo NOVO do
    /// ciclo 13 (spec §7, fecha o backlog #16): antes só a superfície
    /// pavimentada era calculada (e publicada, em silêncio, no campo
    /// `rotation_limit_pct_mac`); agora as DUAS superfícies são publicadas
    /// explicitamente, e o consumidor decide qual olhar. Atrito menor ⟹
    /// menos momento nariz-abaixo de solo ⟹ limite MENOS restritivo
    /// (%MAC menor) que `rotation_limit_pct_mac_grass`.
    pub rotation_limit_pct_mac_paved: f64,
    /// Limite dianteiro de rotação (%MAC) avaliado com
    /// `[performance].mu_roll_grass` — grama/terra, a superfície de
    /// OPERAÇÃO desta aeronave (spec §7). É o valor que
    /// `rotation_limit_pct_mac` publica desde este ciclo. Atrito maior ⟹
    /// mais momento nariz-abaixo de solo ⟹ limite MAIS restritivo (%MAC
    /// maior) que `rotation_limit_pct_mac_paved`.
    pub rotation_limit_pct_mac_grass: f64,
    /// Margem de autoridade de rotação avaliada na CG/peso reais de cada
    /// cenário do `WeightBalanceAgent` — ver `ScenarioTrimLimit`.
    /// Diagnóstico informativo/falseável, NÃO usado para calcular
    /// `rotation_limit_pct_mac` nem `inside_envelope` (esses usam o limite
    /// único acima).
    pub rotation_margin_per_scenario: Vec<ScenarioTrimLimit>,
    /// Qual manobra governa (limite MAIOR, mais restritivo) — `"flare"` ou
    /// `"rotacao"`. Ver docstring da struct.
    pub governing: String,
    /// CL_h disponível — `-cl_h_max_down·(1 − trim_margin)` (download
    /// máximo da empenagem com o profundor no batente, com a margem de
    /// trim reservada para efeito solo/certificação).
    pub cl_h_available: f64,
    /// Sensibilidade do limite de flare a `cl_h_max_down` (±0.05) E a
    /// `elevator_deflection_max_deg` (±2°) — ver `TrimSensitivity`.
    pub sensitivity: TrimSensitivity,
    // ─── Parâmetros ecoados (rastreabilidade sem reabrir o TOML) ────────
    pub cm_ac: f64,
    pub cm_flap_delta: f64,
    /// `cl_h_max_down` OPERACIONAL — valor efetivamente usado no balanço de
    /// momentos, já truncado no teto de stall quando `capped_by_stall`
    /// (`min(cl_h_max_down_calc, [stability].cl_h_stall_limit)`). Task
    /// refino-ciclo2 (1a): deixou de ser um eco direto de
    /// `[stability].cl_h_max_down` (campo REMOVIDO da config) — agora é
    /// CALCULADO por geometria DATCOM/Nelson, ver `agents::trim_authority::
    /// cl_h_max_down_calc`.
    pub cl_h_max_down: f64,
    /// `cl_h_max_down` BRUTO — `a_t·τ·δe_max_rad`, ANTES de qualquer
    /// truncamento pelo teto de stall (`[stability].cl_h_stall_limit`).
    /// Igual a `cl_h_max_down` quando `capped_by_stall == false`; maior
    /// quando `capped_by_stall == true` (a geometria "pediria" mais
    /// download do que a empenagem consegue entregar antes de estolar).
    /// Task refino-ciclo2 (1a).
    pub cl_h_max_down_calc: f64,
    /// Eficácia de superfície do profundor τ(c_e/c) — ajuste de Nelson
    /// (`agents::trim_authority::tau_elevator`), calculada a partir de
    /// `[control_surfaces].elevator_chord_frac`. Task refino-ciclo2 (1a).
    pub tau_elevator: f64,
    /// `true` quando `cl_h_max_down_calc` (bruto) excede
    /// `[stability].cl_h_stall_limit` — o teto de stall da empenagem, não a
    /// geometria do profundor, é o fator limitante de `cl_h_max_down`
    /// neste caso. Task refino-ciclo2 (1a).
    pub capped_by_stall: bool,
    pub trim_margin: f64,
    pub cl_ground_rotation: f64,
    /// Fração de deployment do flap de decolagem — eco de
    /// `[stability].to_flap_fraction` (ciclo 7, task 1: renomeada de
    /// `to_flap_cm_fraction`; PAPEL DUPLO, governa o ΔCm da rotação E o
    /// `cl_max_to` de `WingSpec`).
    pub to_flap_fraction: f64,
    // ─── Arrasto de trim em cruzeiro (Task 4, refino-ciclo2) ─────────────
    /// `CL_h_trim` — sustentação/download que a empenagem horizontal precisa
    /// gerar em cruzeiro (sem flap), no CG de referência da missão
    /// (`cg_reference_scenario`), para equilibrar o momento de arfagem em
    /// voo nivelado 1g. Positivo = upload (CG atrás do CA da asa, caso
    /// típico deste baseline); negativo = download (CG à frente do CA). Ver
    /// `agents::trim_authority::cl_h_trim_cruise`. Calculado aqui com o CG
    /// JÁ CONVERGIDO (`wb.scenarios`, não o valor lag-1 usado dentro do
    /// laço de convergência — ver `orchestrator::size_aircraft` para a
    /// distinção entre os dois usos).
    pub cl_h_trim_cruise: f64,
    /// `ΔCD_trim` — arrasto INDUZIDO da empenagem ao gerar `cl_h_trim_cruise`.
    /// O delta somado a `WingSpec::cd_cruise` no polar de cruzeiro usa o CG
    /// LAG-1 do laço de convergência do MTOW; este campo aqui é
    /// RECALCULADO com o CG JÁ CONVERGIDO (mesma distinção de
    /// `cl_h_trim_cruise` acima) — na prática os dois coincidem a um
    /// resíduo de convergência (~1e-9), não são estritamente o mesmo
    /// número ecoado. Ver `agents::trim_authority::cd_trim_cruise`/
    /// `agents::aerodynamics::apply_cruise_trim_drag`.
    pub cd_trim: f64,
    /// Nome do cenário de carga (`agents::weight_balance::LoadScenario::name`)
    /// usado como CG de referência da missão para o cálculo acima —
    /// "4 pax + bagagem + meia" (meia-missão, ver docstring de
    /// `agents::trim_authority::cl_h_trim_cruise` para a justificativa da
    /// escolha).
    pub cg_reference_scenario: String,
    /// CG de referência (%MAC) do cenário acima, JÁ CONVERGIDO — o valor
    /// efetivamente usado para calcular `cl_h_trim_cruise`/`cd_trim` neste
    /// `TrimSpec` final (distinto do valor lag-1 usado dentro do laço, ver
    /// `cl_h_trim_cruise` acima).
    pub cg_reference_pct_mac: f64,
}

/// Saída do PerformanceAgent (preenchida na Fase seguinte)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceSpec {
    pub v_cruise_kmh: f64,
    pub v_stall_kmh: f64,
    pub rc_sl_ms: f64,
    pub rc_cruise_alt_ms: f64,
    pub service_ceiling_m: f64,
    /// Distância de decolagem sem obstáculo (pista pavimentada, m) —
    /// estimativa simplificada (rolagem × 1,5). Pode ser `f64::INFINITY`
    /// quando a rolagem integrada não consegue acelerar até V_LOF (tração
    /// insuficiente). Serializado como a string `"infinita"` nesse caso, não
    /// `null` — ver `fatigue_life_serde`.
    #[serde(with = "fatigue_life_serde")]
    pub to_distance_paved_m: f64,
    /// Distância de decolagem sem obstáculo (grama/terra, m) — estimativa
    /// simplificada (rolagem × 1,5). Pode ser `f64::INFINITY` quando a
    /// rolagem integrada não consegue acelerar até V_LOF. Serializado como a
    /// string `"infinita"` nesse caso, não `null` — ver `fatigue_life_serde`.
    #[serde(with = "fatigue_life_serde")]
    pub to_distance_grass_m: f64,
    /// Distância de pouso sem obstáculo (m) — estimativa simplificada
    /// (rolagem + 200 m). Pode ser `f64::INFINITY` quando a rolagem
    /// integrada não consegue desacelerar (arrasto e frenagem insuficientes).
    /// Serializado como a string `"infinita"` nesse caso, não `null` — ver
    /// `fatigue_life_serde`.
    #[serde(with = "fatigue_life_serde")]
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
    /// Pode ser `f64::INFINITY` quando o obstáculo é inatingível (razão de
    /// subida negativa ou nula no segmento de subida de 15m — ver
    /// `agents::performance::takeoff_distance_50ft_m`). Serializado como a
    /// string `"infinita"` nesse caso, não `null` — ver `fatigue_life_serde`.
    #[serde(with = "fatigue_life_serde")]
    pub to_50ft_paved_m: f64,
    /// Distância de decolagem sobre obstáculo de 15m/50ft (grama/terra, m).
    /// Pode ser `f64::INFINITY` quando o obstáculo é inatingível. Serializado
    /// como a string `"infinita"` nesse caso, não `null` — ver
    /// `fatigue_life_serde`.
    #[serde(with = "fatigue_life_serde")]
    pub to_50ft_grass_m: f64,
    /// Distância de pouso sobre obstáculo de 15m/50ft (pista pavimentada, m)
    /// — soma de segmentos: aproximação (γ padrão) + flare + ground roll
    /// com `mu_brake_paved`. INFORMATIVO desde o ciclo 6 (revisão final):
    /// o gate de pista (#24) usa `ldg_50ft_grass_m`, não este campo.
    pub ldg_50ft_m: f64,
    /// Distância de pouso sobre obstáculo de 15 m/50 ft em GRAMA (m) —
    /// mesmos segmentos de `ldg_50ft_m`, mas a rolagem de solo usa
    /// `mu_brake_grass` (menor que `mu_brake_paved`): frenagem pior
    /// ALONGA a rolagem, logo esta distância é sempre MAIOR que a
    /// pavimentada. É o caso DIMENSIONANTE da premissa de pista do
    /// projeto (operação em pista de terra/grama, não pavimentada) e a
    /// grandeza comparada contra `runway_available_m` na checagem #24
    /// (`ConstraintChecker::verify`) — simétrico ao par
    /// `to_50ft_paved_m`/`to_50ft_grass_m` da decolagem.
    pub ldg_50ft_grass_m: f64,
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
    /// Fração de carga no nariz no CG mais DIANTEIRO real dos cenários de
    /// carga (`WeightBalanceAgent` — não o limite admissível) — pior caso
    /// para o TETO de 25% e para a carga estrutural máxima real do trem de
    /// nariz (Task 2, refino-ciclo2). **v4.3**: substitui o antigo campo
    /// único `nose_load_fraction_pct` (renomeado, não é mais o único
    /// extremo avaliado — ver `nose_load_min_pct` para o piso).
    pub nose_load_max_pct: f64,
    /// Fração de carga no nariz no CG mais TRASEIRO real dos cenários de
    /// carga — pior caso para o PISO de 8% (tração/direção em solo).
    /// **v4.3** (novo).
    pub nose_load_min_pct: f64,
    /// Ângulo de tipback — trem principal → CG mais TRASEIRO real, medido
    /// a partir da altura do CG acima do solo (Raymer cap. 11). Deve ser >=
    /// `[gear].tipback_min_deg` para a aeronave não tombar sobre a cauda em
    /// solo/carregamento traseiro. **v4.3** (novo, Task 2 refino-ciclo2).
    pub tipback_angle_deg: f64,
    /// Folga angular de tail-strike — geometria simplificada do cone de
    /// cauda (ver `agents::landing_gear::tail_strike_margin_deg`). Deve ser
    /// >= `[gear].rotation_attitude_deg`. **v4.3** (novo).
    pub tail_strike_margin_deg: f64,
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

/// Um check que PASSA no nominal mas REPROVA sob um conjunto adversarial
/// de massas estruturais (±σ) — ciclo 4, check #19 (ver
/// `validation::robustness`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobustnessFlip {
    /// Nome do check que flipou (ex.: "Cenário 'Solo (piloto)'", "Tipback",
    /// "Carga de nariz máx").
    pub check: String,
    /// Conjunto adversarial que o derrubou: "dianteiro" | "traseiro".
    pub caso: String,
    /// Valor sob perturbação e limite violado. SEMPRE finitos (achado de
    /// review, ciclo 5): antes deste fix, o caso "massa-total" podia emitir
    /// `f64::NAN` aqui para 2 das 4 variantes de `SizingError` (serializa
    /// como `null` no JSON) — `validation::robustness::RobustnessAgent::run`
    /// agora escolhe um par valor/limite finito e informativo para toda
    /// variante de erro (ver comentário no local da construção).
    pub valor: f64,
    /// Limite EFETIVAMENTE aplicado ao mundo perturbado. Desde o ciclo 10
    /// (task 2) isto é a régua DO PRÓPRIO mundo perturbado para os limites
    /// de CG — antes era sempre a régua nominal, porque os limites eram
    /// invariantes às massas (ver `validation::robustness`, docstring do
    /// módulo).
    pub limite: f64,
    /// Limite NOMINAL do mesmo check — o que a régua valia antes da
    /// perturbação (ciclo 10, task 2). Existe para que o leitor consiga
    /// separar as DUAS causas possíveis de um flip, que `valor`/`limite`
    /// sozinhos confundem:
    ///   - **"o CG andou"**: `limite_nominal == limite` e `valor` cruzou;
    ///   - **"a régua andou"**: `limite != limite_nominal` — o mundo +σ
    ///     tem um limite dianteiro próprio (a linha de tração fez o limite
    ///     de rotação depender do peso, que responde às massas
    ///     perturbadas).
    /// Para os checks cuja régua É invariante à perturbação (tipback,
    /// carga de nariz, gates de desempenho/pista), `limite_nominal` é
    /// IGUAL a `limite` por construção.
    pub limite_nominal: f64,
}

/// Análise de robustez à incerteza do modelo de massas (ciclo 4) —
/// pior-caso determinístico direcional, ver `validation::robustness`.
/// Consumida por `main`/`AircraftReport::robustness`/`ConstraintChecker::
/// verify` (checagem #19, schema v4.6 — task de wiring, ciclo 4).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobustnessSpec {
    pub sigma_mass_fraction: f64,
    /// Faixa de CG dos cenários sob o conjunto CG-mais-DIANTEIRO (%MAC).
    pub cg_fwd_case_pct_mac: [f64; 2],
    /// Idem sob o conjunto CG-mais-TRASEIRO.
    pub cg_aft_case_pct_mac: [f64; 2],
    /// Checks que passam no nominal mas reprovam perturbados (vazio = robusto).
    pub flips: Vec<RobustnessFlip>,
    /// MTOW (kg) re-convergido pelo laço COMPLETO de `orchestrator::
    /// size_aircraft` no 3º caso adversarial (ciclo 5, check #19) — todas as
    /// 5 massas estruturais compostas ×(1+σ), não só ±σ direcional como os
    /// dois casos de CG acima. `0.0` quando o sizing perturbado FALHOU
    /// (`SizingError`) — nesse caso o flip de "Dimensionamento" acompanha e
    /// documenta a causa, este campo não carrega significado físico.
    pub mtow_masstotal_kg: f64,
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
    /// Folga entre a ponta da pá e o solo (m) — `shaft_height − diameter_m/2`,
    /// onde `shaft_height = gear.h_cg_ground_m + propeller.prop_axis_above_cg_m`
    /// (datum derivado do trem — ciclo 5).
    pub ground_clearance_m: f64,
    /// Maior diâmetro (m) que respeita AMBOS os limites de Mach de ponta
    /// (estático e cruzeiro) — o menor dos dois máximos individuais.
    pub diameter_max_by_mach_m: f64,
    /// Maior diâmetro (m) que respeita a folga mínima de solo.
    pub diameter_max_by_clearance_m: f64,
    pub ok_mach_static: bool,
    pub ok_mach_cruise: bool,
    pub ok_clearance: bool,
    /// Folga entre a ponta da pá e o solo na condição CRÍTICA de CS 23.925
    /// (ciclo 8, task 2) — amortecedor do trem de NARIZ TOTALMENTE
    /// COMPRIMIDO (batente) + pneu MURCHO/estourado, não a condição
    /// estática de `ground_clearance_m`. Hélice TRATORA: o trem de NARIZ
    /// governa (fica sob o eixo da hélice), não o principal.
    ///
    /// FÓRMULA (ciclo 10, task 1, deflexão estática — old→new):
    /// `ground_clearance_m − Δ_prop`, com `Δ_prop = (gear.nose_oleo_stroke_mm
    /// /1000 × (1 − gear_cfg.static_sag_fraction) +
    /// gear_cfg.tire_deflation_delta_m) × fator` e
    /// `fator = (gear_cfg.x_main_m − prop_cfg.prop_plane_x_m)/(gear_cfg.
    /// x_main_m − gear_cfg.x_nose_m)` — ver `fill_critical_clearance`.
    /// ANTES desta task (ciclo 9): `Δ_prop = (gear.nose_oleo_stroke_mm/1000
    /// + gear_cfg.tire_deflation_delta_m) × fator` — o termo de nariz usava
    /// o curso TOTAL do amortecedor, não o curso RESTANTE até o batente.
    /// FÍSICA CORRIGIDA (CS 23.925 pela LETRA — leitura da norma, não uma
    /// mudança de opinião): o caso crítico da norma coloca APENAS o trem
    /// CRÍTICO (aqui, o de nariz — hélice TRATORA) no batente; os DEMAIS
    /// trens (aqui, o principal) permanecem na deflexão ESTÁTICA normal.
    /// `[gear].h_cg_ground_m` já é medido com a aeronave CARREGADA, em
    /// deflexão estática (ver docstring desse campo) — os mains JÁ estão
    /// nessa deflexão dentro de `h_cg_ground_m`/`ground_clearance_m`, daí
    /// NUNCA precisarem de termo aditivo aqui (mata o caveat dos mains
    /// rígidos abaixo). Pelo MESMO motivo, o amortecedor de NARIZ também
    /// PARTE da deflexão estática, não estendido — na condição crítica ele
    /// só percorre o curso RESTANTE até o batente
    /// (`nose_oleo_stroke_mm × (1 − static_sag_fraction)`), não o curso
    /// TOTAL. A fórmula do ciclo 9 contava a compressão estática do nariz
    /// DUAS VEZES (implícita em `h_cg_ground_m`, explícita no curso total
    /// do batente) — corrigir para o curso restante reduz `Δ_prop` e
    /// AUMENTA a folga crítica: honestamente ANTI-conservadora frente ao
    /// número antigo, mas fiel à letra da norma (sem dupla contagem). No
    /// baseline E10 real (`static_sag_fraction` 0,33, mesmos `x_nose_m`
    /// 1,30 m/`x_main_m` 3,66 m/`prop_plane_x_m` 0,20 m do ciclo 9): fator
    /// ≈ 1,46610 (inalterado — não depende de `static_sag_fraction`),
    /// `prop_clearance_critical_m` **≈ −0,06416 m (FAIL, ciclo 9) →
    /// ≈ −0,00249 m (FAIL, ciclo 10)** — o veredito da checagem #25 NÃO
    /// MUDA (continua FAIL), só o NÚMERO da violação. Ver o histórico
    /// completo old→new na docstring de `SCHEMA_VERSION`.
    ///
    /// CAVEAT DOS MAINS RÍGIDOS (ciclo 9) — RESOLVIDO nesta task: a
    /// preocupação nomeada no ciclo 9 (deflexão do amortecedor/pneu
    /// PRINCIPAL, `gear.main_oleo_stroke_mm` ≈ 212,4 mm no baseline real,
    /// não entrar na fórmula, sendo ADITIVA ao termo do nariz sob uma
    /// condição COMPOSTA) partia da premissa de que o modelo tratava os
    /// mains como "estendidos" — mas `h_cg_ground_m` NUNCA foi "trem
    /// estendido"; é a altura CARREGADA (deflexão estática), como a
    /// docstring desse campo agora documenta explicitamente. Lida pela
    /// LETRA de CS 23.925, a condição crítica não exige os mains no
    /// batente simultaneamente ao nariz — só o trem CRÍTICO vai ao
    /// batente, os demais ficam na deflexão estática que `h_cg_ground_m`
    /// já modela. Não há termo faltando: os mains nunca precisaram de um.
    /// Ver `docs/backlog.md` (item 6, RESOLVIDO ciclo 10).
    ///
    /// Nota relacionada, sinal OPOSTO e pequeno, NÃO resolvida por esta
    /// task (item independente): o disco da hélice também não é modelado
    /// como INCLINADO junto com o pitch da célula — tratar o disco como
    /// permanecendo vertical (ponta mais baixa sempre à distância do raio
    /// abaixo do cubo) é CONSERVADOR em ≈+3,4 mm (`raio × (1 − cos θ)`,
    /// θ ≈ 5,04° no baseline real) frente a uma modelagem exata do disco
    /// tombado — o tombamento ERGUE o ponto mais baixo varrido em relação
    /// ao cubo, então ignorá-lo empurra a folga calculada para o lado
    /// SEGURO. Ver `docs/backlog.md` (item 6).
    ///
    /// PREENCHIDO EM DOIS PASSOS: `PropellerAgent::run` (este arquivo não
    /// tem acesso a `GearSpec`, e a hélice roda ANTES do trem de pouso no
    /// pipeline real — `main.rs`, "Agente 9" precede "Agente 6" na ordem de
    /// EXECUÇÃO apesar da numeração conceitual) inicializa este campo em
    /// `0.0` — PLACEHOLDER explícito, nunca `NaN` (lição do ciclo 5, ver
    /// `RobustnessFlip::valor`) — e `fill_critical_clearance` o preenche de
    /// verdade depois que `LandingGearAgent::run` produz `GearSpec`. Todo
    /// caminho que constrói um `PropellerSpec` para consumo real (`main.rs`,
    /// a fixture de `validation::constraint_checker`, `tests/schema_v4.rs`,
    /// `tests/gear_tipback.rs`) precisa chamar `fill_critical_clearance`
    /// logo após o trem de pouso — senão este campo fica preso no
    /// placeholder `0.0`, que a checagem #25 interpretaria como violação
    /// (`0.0 <= 0.0`).
    pub prop_clearance_critical_m: f64,
}

impl PropellerSpec {
    /// Preenche `prop_clearance_critical_m` — condição CRÍTICA de CS 23.925
    /// (ciclo 8, task 2): batente do amortecedor de nariz + pneu murcho,
    /// hélice TRATORA (trem de NARIZ governa), sob o pivô da célula sobre o
    /// trem PRINCIPAL (ciclo 9, transferência de atitude do #25), com o
    /// curso do nariz medido a partir da deflexão ESTÁTICA já embutida em
    /// `h_cg_ground_m` — curso RESTANTE, não curso total (ciclo 10, task 1
    /// — ver docstring do campo `prop_clearance_critical_m` para a física e
    /// o old→new completos). Chamado DEPOIS que `LandingGearAgent::run`
    /// produz `gear` — ver docstring do campo para o porquê da ordem (a
    /// hélice roda antes do trem no pipeline real). `prop_cfg` (ciclo 9) é
    /// `[propeller]` — só `prop_plane_x_m` é consumido aqui; `gear_cfg`
    /// (ciclo 10) também contribui `static_sag_fraction`, o resto do
    /// struct não participa da fórmula.
    pub fn fill_critical_clearance(
        &mut self,
        gear: &GearSpec,
        gear_cfg: &crate::models::aircraft_config::GearCfg,
        prop_cfg: &crate::models::aircraft_config::PropellerCfg,
    ) {
        let fator = (gear_cfg.x_main_m - prop_cfg.prop_plane_x_m)
            / (gear_cfg.x_main_m - gear_cfg.x_nose_m);
        let curso_restante_nariz_m = (gear.nose_oleo_stroke_mm / 1_000.0)
            * (1.0 - gear_cfg.static_sag_fraction);
        let delta_prop = (curso_restante_nariz_m + gear_cfg.tire_deflation_delta_m) * fator;
        self.prop_clearance_critical_m = self.ground_clearance_m - delta_prop;
    }
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

/// Eco de uma carga elétrica configurada (ciclo 5, check #20) — espelho de
/// `ElectricalLoadCfg` no relatório, para rastreabilidade e para o checker
/// comparar o pico DECLARADO do atuador de retração com a potência
/// COMPUTADA (`GearSpec::actuator_power_w`).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ElectricalLoadSpec {
    pub name: String,
    pub continuous_w: f64,
    pub peak_w: f64,
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
    /// Eco das cargas individuais configuradas (ciclo 5, check #20) — cada
    /// item espelha um `ElectricalLoadCfg` de `[electrical].loads`. Existe
    /// para que `ConstraintChecker::verify` possa comparar o pico
    /// DECLARADO da carga 'trem_retratil' contra a potência do atuador de
    /// retração COMPUTADA por `LandingGearAgent` — checagem que só é
    /// possível PÓS-convergência (ver nota histórica do ciclo 3 em
    /// `models::config::validate_aircraft`: a guarda equivalente de
    /// parse-time foi removida porque a massa da perna do trem virou
    /// computada).
    pub loads: Vec<ElectricalLoadSpec>,
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
///
/// v4.1 (task trim-authority): adiciona o bloco `trim` (`TrimSpec` —
/// TrimAuthorityAgent) — mudança ADITIVA (novo bloco opcional), consumidores
/// v4.0 continuam funcionando sem alteração. Acompanha, do lado da
/// CONFIGURAÇÃO de entrada (não deste schema JSON): `[stability].sm_max`
/// foi REMOVIDO de `aircraft.toml` (proxy de autoridade de profundor sem
/// base física direta) e substituído por `[stability].cl_h_max_down`/
/// `trim_margin`/`cl_ground_rotation`/`to_flap_fraction` +
/// `[wing].cm_ac`/`cm_flap_delta` — ver `docs/aircraft_spec.schema.md` §1 e
/// `models::config::parse_aircraft` (erro de migração claro se `sm_max`
/// ainda estiver presente no TOML).
///
/// v4.2 (task refino-ciclo2): `TrimSpec` ganha três campos NOVOS
/// (`cl_h_max_down_calc`, `tau_elevator`, `capped_by_stall`) e `TrimSensitivity`
/// ganha quatro campos NOVOS (par `elevator_deflection_max_deg_minus/plus` +
/// `flare_limit_pct_mac_deflection_minus/plus`) — mudança ADITIVA (campos
/// novos em blocos já existentes; nenhum campo existente foi removido nem
/// mudou de tipo/unidade), consumidores v4.1 continuam funcionando sem
/// alteração. `TrimSpec::cl_h_max_down` PERMANECE presente com o MESMO
/// significado (valor operacional usado no balanço de momentos) — só a
/// FONTE mudou (antes ecoava `[stability].cl_h_max_down` da config, agora é
/// CALCULADO por geometria DATCOM/Nelson, ver `agents::trim_authority::
/// cl_h_max_down_calc`). Acompanha, do lado da CONFIGURAÇÃO de entrada (não
/// deste schema JSON): `[stability].cl_h_max_down` foi REMOVIDO (substituído
/// por `[control_surfaces].elevator_deflection_max_deg` +
/// `[stability].cl_h_stall_limit`); `[empennage].cd0` e os itens
/// `emp_horizontal`/`emp_vertical` de `[[masses.items]]` também foram
/// REMOVIDOS (substituídos, na época, por
/// `[empennage].mass_per_area_{h,v}_kg_m2` + `cd0_area_factor`, aplicados
/// sobre a área da empenagem REALMENTE dimensionada) — todos com erro de
/// migração claro em `models::config::parse_aircraft` se ainda presentes
/// no TOML. No ciclo 3 (oew-parametrico) os próprios
/// `mass_per_area_{h,v}_kg_m2` foram removidos, junto com mais 5 nomes de
/// `[[masses.items]]` e `[gear].mass_main_leg_kg`: as 7 massas
/// estruturais do OEW passaram a ser COMPUTADAS por `agents::mass_model`
/// (Raymer cap. 15.2 × `[mass_model]`). Ver
/// `docs/aircraft_spec.schema.md` §1 e §4.
///
/// v4.3 (Task 2, refino-ciclo2): `GearSpec::nose_load_fraction_pct`
/// (único, calculado só no CG mais traseiro) foi RENOMEADO/SUBSTITUÍDO por
/// dois campos, `nose_load_max_pct` (CG mais dianteiro real, teto de 25%) e
/// `nose_load_min_pct` (CG mais traseiro real, piso de 8%) — mudança que
/// QUEBRA compatibilidade estrita (campo removido), mas versionada como
/// bump MINOR por diretriz explícita desta task (o novo par de campos
/// substitui integralmente o antigo com semântica equivalente para o
/// extremo traseiro — `nose_load_min_pct` é numericamente o antigo
/// `nose_load_fraction_pct`; consumidores devem atualizar a leitura do
/// campo, mas nenhum outro bloco muda). `GearSpec` também ganha dois campos
/// NOVOS, `tipback_angle_deg` e `tail_strike_margin_deg` (checagens de
/// Raymer cap. 11, ver `agents::landing_gear`). Acompanha, do lado da
/// CONFIGURAÇÃO de entrada (não deste schema JSON): `[gear]` ganha quatro
/// campos NOVOS obrigatórios — `tipback_min_deg`, `rotation_attitude_deg`,
/// `tail_cone_x_m`, `tail_cone_height_m` (sem valor padrão implícito; TOMLs
/// antigos sem esses campos falham o parse do `toml` crate por campo
/// ausente, não um erro de migração dedicado). Ver
/// `docs/aircraft_spec.schema.md` §1 e §4.
/// v4.4 (Task 4, refino-ciclo2): `TrimSpec` ganha QUATRO campos NOVOS —
/// `cl_h_trim_cruise`, `cd_trim`, `cg_reference_scenario`,
/// `cg_reference_pct_mac` (arrasto de trim em cruzeiro, ver
/// `agents::trim_authority::cl_h_trim_cruise`/`cd_trim_cruise`) — mudança
/// ADITIVA (campos novos num bloco já existente; nenhum campo removido nem
/// mudou de tipo/unidade), consumidores v4.3 continuam funcionando sem
/// alteração. `WingSpec::cd_cruise`/`ld_ratio_cruise` PERMANECEM com o
/// MESMO significado e forma — só o VALOR muda (agora inclui o arrasto de
/// trim, `agents::aerodynamics::apply_cruise_trim_drag`). Acompanha, do lado
/// da CONFIGURAÇÃO de entrada (não deste schema JSON): `[empennage]` ganha
/// um campo NOVO obrigatório, `e_h` (eficiência de Oswald da empenagem
/// horizontal, faixa 0,5–0,95) — sem valor padrão implícito; TOMLs antigos
/// sem esse campo falham o parse do `toml` crate por campo ausente, não um
/// erro de migração dedicado. Ver `docs/aircraft_spec.schema.md` §1 e §4.
/// v4.5 (Task 5, oew-parametrico): `WeightSpec` ganha UM campo NOVO,
/// `structural_masses` (`StructuralMassesSpec`) — as 7 massas estruturais
/// COMPUTADAS (`agents::mass_model`, já usadas internamente desde o Ciclo 3
/// mas nunca ecoadas no JSON) + os 5 fatores de composto de `[mass_model]`
/// usados para calculá-las, para rastreabilidade no consumidor de CAD —
/// mudança ADITIVA (campo novo num bloco já existente; nenhum campo
/// removido nem mudou de tipo/unidade), consumidores v4.4 continuam
/// funcionando sem alteração. Ver `docs/aircraft_spec.schema.md` §1 e §4.
///
/// v4.6 (Task 4, ciclo4-fidelidade-massas): `AircraftReport` ganha UM
/// bloco NOVO, `robustness` (`RobustnessSpec` — `validation::robustness::
/// RobustnessAgent`, já existente internamente desde a Task 3 do ciclo mas
/// isolado do pipeline até aqui) — checagem #19 NOVA em `ConstraintChecker
/// ::verify`: um check que PASSA no nominal mas REPROVA sob um dos dois
/// conjuntos adversariais de massas estruturais (±σ, `RobustnessSpec::
/// flips`) gera uma violação nomeada. Mudança ADITIVA (bloco novo opcional
/// + checagem nova que só pode ADICIONAR violações, nunca remover as
/// existentes), consumidores v4.5 continuam funcionando sem alteração. Ver
/// `docs/aircraft_spec.schema.md` §1 e §4.
///
/// v4.7 (Task 4, ciclo5-robustez-total-e-solo): dois campos NOVOS em
/// blocos já existentes. `ElectricalSpec` ganha `loads`
/// (`Vec<ElectricalLoadSpec>` — check #20) — eco individual de cada
/// `[electrical].loads` configurada (nome, potência contínua, potência de
/// pico), para que `ConstraintChecker::verify` compare o pico DECLARADO
/// da carga 'trem_retratil' contra `landing_gear.actuator_power_w`
/// COMPUTADO (checagem só possível pós-convergência). `RobustnessSpec`
/// ganha `mtow_masstotal_kg` — MTOW re-convergido pelo laço COMPLETO de
/// `orchestrator::size_aircraft` sob um TERCEIRO conjunto adversarial (as
/// 5 massas estruturais compostas ×(1+σ), não só ±σ direcional dos dois
/// casos de CG da v4.6) — checagem #19 ganha o caso "massa-total": um
/// re-sizing INTEIRO sob incerteza de massa, não apenas a reavaliação
/// posterior de CG/trem contra limites nominais invariantes. Mudança
/// ADITIVA (campos novos em blocos já existentes; nenhum campo removido
/// nem mudou de tipo/unidade), consumidores v4.6 continuam funcionando
/// sem alteração. Acompanha, do lado da CONFIGURAÇÃO de entrada (não
/// deste schema JSON — já implementado na Task 1 do mesmo ciclo):
/// `[propeller].shaft_height_m` (datum ABSOLUTO de altura do eixo) foi
/// REMOVIDO com erro de migração, substituído por `[propeller].
/// prop_axis_above_cg_m` (offset vertical FIXO entre eixo e CG) —
/// `propeller.ground_clearance_m` agora deriva de `gear.h_cg_ground_m +
/// propeller.prop_axis_above_cg_m`, acoplando a folga de hélice ao
/// comprimento do trem (encurtar o trem consome folga automaticamente,
/// em vez de dessincronizar como o datum absoluto antigo). Ver
/// `docs/aircraft_spec.schema.md` §1 e §4.
///
/// v4.8 (Task 4, ciclo6-pista-e-robustez-final; entrada EMENDADA na
/// revisão final do mesmo ciclo — a 4.8 ainda não havia shipado quando o
/// achado abaixo apareceu, então foi corrigida aqui em vez de virar uma
/// 4.9): UM campo NOVO no JSON, `performance.ldg_50ft_grass_m`, mais o
/// contrato de comportamento das checagens de pista. Quatro mudanças:
/// (1) `Requirements` (config de missão, não schema JSON) ganha um campo
/// NOVO obrigatório, `runway_available_m` (faixa válida (300, 2000) m) —
/// TOMLs de missão antigos sem esse campo falham o parse do crate `toml`
/// por campo ausente, mesmo padrão de migração sem valor-padrão implícito
/// já usado no `e_h` da v4.4; (2) `PerformanceSpec` ganha
/// `ldg_50ft_grass_m` (f64, m) — distância de pouso sobre 15 m em GRAMA,
/// mesmos segmentos de `ldg_50ft_m` mas com `mu_brake_grass` na rolagem
/// de frenagem. Campo ADITIVO (nenhum existente muda de nome/tipo/
/// unidade); `ldg_50ft_m` (pavimentado) permanece, agora INFORMATIVO;
/// (3) `ConstraintChecker::verify` ganha as checagens #23 (decolagem na
/// grama sobre obstáculo de 15 m > pista disponível) e #24 (pouso na
/// GRAMA sobre 15 m > pista disponível — usa `ldg_50ft_grass_m`, não o
/// pavimentado: gatear uma pista de grama com a distância pavimentada era
/// otimista por construção, e `mu_brake_grass` era validado na config
/// desde a Task 4.7 sem NUNCA ser consumido) — só podem ADICIONAR strings
/// a `violations`, nunca remover as existentes; (4) o caso "massa-total"
/// do check #19 (`RobustnessAgent`, `robustness.flips`) deixa de
/// descartar `sized_p.wb` e passa a avaliar TAMBÉM pista (#23/#24, ambas
/// as grandezas de grama) e envelope/nariz/tipback sob o mundo
/// re-convergido ×(1+σ) — antes só os gates de desempenho (margem, VS0,
/// rc, v_cruise, teto) eram checados nesse mundo; os casos direcionais
/// (±σ) já cobriam CG desde a v4.6. Mudança ADITIVA em forma e em
/// comportamento (um campo a mais; mais violações/flips POSSÍVEIS, nunca
/// menos), consumidores v4.7 continuam funcionando sem alteração.
/// ACHADO HONESTO do baseline real (σ=15%, pista 600 m): `validation_
/// status` continua `"FAIL"`, mas com **4** violações, não 3 — a QUARTA é
/// `"Pouso (grama, 15 m): 605 m excede a pista disponível de 600 m"`. O
/// pouso pavimentado (539,97 m) cabia nos 600 m; o de grama (604,99 m)
/// não cabe, e nunca coubera — o modelo é que não estava olhando. A
/// decolagem na grama (#23, 428,2 m) passa limpa, e o caso massa-total
/// ampliado não produz nenhum flip novo. Ver `docs/aircraft_spec.schema.md`
/// §1 e `tests/cli.rs`.
///
/// v5.0 (Task 2, ciclo7-clmax-decolagem; bump **MAJOR**, não MINOR — a
/// própria política acima é explícita: "renomeia/remove campo... muda o
/// TIPO ou a UNIDADE de um campo existente" é MAJOR): a Task 1 do mesmo
/// ciclo RENOMEOU um campo JÁ SERIALIZADO, `[stability].to_flap_cm_fraction`
/// → `to_flap_fraction`, ecoado em `TrimSpec::to_flap_fraction` — pela
/// própria régua deste crate isso não é uma mudança aditiva, é quebra de
/// contrato (um consumidor lendo `trim.to_flap_cm_fraction` do JSON v4.8
/// simplesmente não encontra mais a chave no v5.0). Duas mudanças de
/// conteúdo:
///   1. `WingSpec` ganha UM campo NOVO, `cl_max_to` — CL_max em
///      configuração de DECOLAGEM (flap PARCIAL), derivado por
///      interpolação linear entre `cl_max_clean` e `cl_max_flaps` pela
///      MESMA `to_flap_fraction` que já sinalizava o ΔCm da rotação (ver
///      docstring de `WingSpec::cl_max_to` acima). Isoladamente seria
///      ADITIVA (campo novo); é o renome acima que força o MAJOR.
///   2. `TrimSpec::to_flap_fraction` (RENOMEADO, ver acima) — mesmo
///      significado físico e VALOR do antigo `to_flap_cm_fraction`, agora
///      com papel DUPLO: além do ΔCm de rotação, governa também
///      `wing.cl_max_to`.
/// ACHADO HONESTO do baseline real (consequência da Task 1, física
/// corrigida — rotação e distâncias de decolagem passam a usar o CL_max de
/// DECOLAGEM em vez do CL_max de POUSO, que ninguém usa para decolar):
/// `rotation_limit_pct_mac` (limite dianteiro de rotação) recua de 12,995%
/// para **8,908% MAC** (mais autoridade, era pessimista com o CLmax
/// errado). `validation_status` PERMANECE `"FAIL"` com as MESMAS **4**
/// violações em CONTAGEM, mas DUAS trocam de natureza: os cenários nominais
/// 'Solo (piloto)' e '2 pax dianteiros', que antes violavam o limite
/// dianteiro de rotação DIRETAMENTE (envelope de CG nominal), agora ficam
/// DENTRO do envelope nominal (o limite recuou o bastante) e passam a
/// disparar a checagem #19 de ROBUSTEZ (`robustness.flips`) — reprovam sob
/// o caso adversarial dianteiro (±15% de massa estrutural) contra o mesmo
/// limite de 8,908% MAC. O achado físico não some, muda de categoria
/// (nominal → robustez). As outras duas violações (carga de nariz máxima e
/// pouso na grama, v4.8) ficam bit a bit INALTERADAS. Ver
/// `docs/aircraft_spec.schema.md` §1 e `tests/schema_v4.rs`.
///
/// v5.1 (Task 3, ciclo8-flap-e-solo — bump MINOR: formaliza dois campos
/// ADITIVOS introduzidos nas Tasks 1/2 do mesmo ciclo, que já estavam
/// serializados desde antes deste bump mas documentados como "pendentes"
/// — ver `docs/aircraft_spec.schema.md` §1 v5.0/nota de estado E10; nenhum
/// campo existente foi renomeado/removido nem mudou de tipo/unidade,
/// consumidores v5.0 continuam funcionando sem alteração):
///   1. `WingSpec` ganha `cd0_flap_to_extra` (f64, Task 1) — ΔCD0 do flap
///      PARCIAL de decolagem (`to_flap_fraction · [wing].cd0_flap_delta`),
///      consumido por `agents::performance::excess_power_kw` no segmento
///      de SUBIDA da decolagem e no gradiente CS 23.65 (`best_climb_angle_ms`,
///      avaliado em Vx). Fecha a lacuna "sem modelo de flap na polar",
///      declarada desde o ciclo 7.
///   2. `PropellerSpec` ganha `prop_clearance_critical_m` (f64, m, Task 2)
///      — folga ponta de pá ↔ solo na condição CRÍTICA de CS 23.925
///      (amortecedor do trem de nariz totalmente comprimido + pneu
///      murcho), distinta de `ground_clearance_m` (folga ESTÁTICA).
///      Checagem NOVA #25 em `ConstraintChecker::verify` reprova quando
///      `<= 0.0` — mudança ADITIVA em comportamento (só pode ADICIONAR
///      violações, nunca remover as existentes).
/// ACHADO HONESTO consolidado do baseline real E10 (consequência FÍSICA
/// das Tasks 1-2, não deste bump em si):
///   - §1-§2 (arrasto de flap + gradiente CS 23.65 honesto, Task 1):
///     `climb_gradient_pct` recua de 15,129850% para **13,896713%**
///     (−1,233137 p.p.), decomposto em dois efeitos isolados por medição
///     direta (não estimativa): ~72% (−0,888093 p.p.) vem do deslocamento
///     do PONTO de avaliação (referência de estol `wing.cl_max` de pouso →
///     `wing.cl_max_to` de decolagem parcial, CL_max menor → Vx maior);
///     ~28% (−0,345045 p.p.) vem do arrasto extra do flap
///     (`cd0_flap_to_extra`) somado à polar nesse ponto. `vx_kmh` sobe
///     **+11,89%** (108,609→121,520 km/h — reflexo direto do CL_max_to
///     menor). `to_50ft_paved_m`/`to_50ft_grass_m` alongam **~+1%**
///     (420,47/473,58 m — só o segmento de SUBIDA da decolagem consome a
///     polar nova; rolagem de solo/aproximação de pouso permanecem
///     bit-a-bit inalteradas, método energético/ângulo fixo sem termo de
///     CD0). `climb_gradient_pct` continua acima do piso CS 23.65 (8,3%),
///     folga ~5,6 p.p. — `validation_status` PASS mantido.
///   - **Viés remanescente NOMEADO** (achado da revisão, PRÉ-EXISTENTE ao
///     ciclo 8, não introduzido por ele — permanece registrado aqui para
///     rastreabilidade do bump): `best_climb_angle_ms` devolve o PISO da
///     varredura de velocidade (1,05·V_s_to), não um máximo interior — RC/V
///     é monotonicamente DECRESCENTE na faixa modelada para esta célula. A
///     CS 23.65 tipicamente avalia a ≥1,2·V_s; nessa referência o gradiente
///     do baseline real seria ≈12,4486%, não os 13,896713% retornados
///     (~1,45 p.p. de viés OTIMISTA remanescente). Documentado na docstring
///     de `agents::performance::best_climb_angle_ms` e em `fidelity.
///     performance` — item de ciclo futuro, não corrigido nesta task por
///     instrução explícita do brief (Task 1 já havia isolado e nomeado o
///     achado; reavaliar a velocidade de referência fica fora de escopo).
///   - §3-§4 (folga crítica CS 23.925 + pin de rotação, Task 2):
///     `prop_clearance_critical_m` ≈ **+0,0325 m** (checagem #25 PASS —
///     folga positiva, sem necessidade de PARAR). `rotation_limit_pct_mac`
///     recentrado em `8,533% ± 0,05%` (era `8,908% ± 1,5%` desde o ciclo 7,
///     dívida de cobertura reaper­tada nesta task).
///   - **CAVEAT NOMEADO** (achado de review, não corrigido nesta task): a
///     fórmula de `prop_clearance_critical_m` trata o colapso do trem de
///     nariz como TRANSLAÇÃO VERTICAL 1:1, mas a célula na realidade
///     PIVOTA sobre o trem principal — a hélice (à frente do nariz)
///     mergulha um braço amplificado (`≈1,4–1,55×` o curso vertical do
///     nariz para esta geometria). Sob a transferência de atitude real, a
///     folga crítica plausivelmente vira NEGATIVA (≈ −0,05 a −0,08 m), não
///     os +0,0325 m publicados pela simplificação — checagem #25 pode
///     estar mascarando um FAIL honesto do E10. Ver docstring de
///     `PropellerSpec::prop_clearance_critical_m` e `docs/backlog.md`
///     ("transferência de atitude do #25") — item de ciclo futuro.
/// `validation_status` do baseline real PERMANECE `"PASS"` com `violations`
/// VAZIO e `robustness.flips` VAZIO — mesmo veredito da campanha E10 (v5.0),
/// sem nenhum flip novo introduzido pelas Tasks 1-2. Ver
/// `docs/aircraft_spec.schema.md` §1 e `tests/schema_v4.rs`/
/// `tests/generic_engine.rs`.
///
/// v5.2 (Task 2, ciclo9-transferencia-atitude — bump **MINOR**, exceção
/// registrada): nenhum campo do JSON de saída foi
/// renomeado/removido/mudou de tipo/unidade — consumidores v5.1 continuam
/// funcionando sem alteração (mesmo TIPO, mesmo nome, o parser não
/// muda). O bump é sobre SEMÂNTICA: `propeller.prop_clearance_critical_m`
/// MANTÉM o nome, mas a FÓRMULA que o preenche mudou (Task 1 do mesmo
/// ciclo, `48a2ed4`) e o veredito honesto do baseline real virou de PASS
/// para FAIL. Pela LETRA da política de `docs/aircraft_spec.schema.md` §1
/// ("muda a semântica de um campo sem mudar seu nome" é gatilho de MAJOR),
/// isto seria MAJOR — tratado como MINOR por decisão de projeto aprovada
/// pelo usuário: é correção de um BUG de modelagem física (simplificação
/// otimista → fórmula honesta), não mudança de CONTRATO de tipo/estrutura.
/// Divergência entre a letra da política e esta decisão registrada
/// explicitamente em `docs/aircraft_spec.schema.md` §1 ("Exceção
/// registrada (v5.2)") — não escondida.
///
/// **CAVEAT NOMEADO na v5.1 acima RESOLVIDO** — não corrigido antes por
/// instrução explícita do brief do ciclo 8 (item de ciclo futuro nomeado em
/// `docs/backlog.md`, item 1). Campo de CONFIGURAÇÃO NOVO
/// `[propeller].prop_plane_x_m` (posição do plano da hélice, m do datum no
/// nariz — input, NÃO ecoado no JSON de saída, ver `PropellerSpec`)
/// alimenta o fator de amplificação do pivô descrito acima;
/// `PropellerSpec::fill_critical_clearance` ganha um terceiro parâmetro
/// (`prop_cfg: &PropellerCfg`) para lê-lo. Nenhuma tolerância de teste foi
/// afrouxada — só a fórmula mudou, old→new (fator implícito 1 → fator
/// explícito `(x_main−prop_plane_x_m)/(x_main−x_nose_m)`).
///
/// **ACHADO HONESTO**: no baseline E10 real (`prop_plane_x_m` 0,20 m,
/// `x_nose_m` 1,30 m, `x_main_m` 3,66 m) o fator vale ≈1,46610 —
/// `prop_clearance_critical_m` vai de **+0,0325 m (checagem #25 PASS) para
/// ≈ −0,06416 m (checagem #25 FAIL)**. `validation_status` do baseline real
/// vira `"FAIL"` com **exatamente 1 violação nomeada** (checagem #25,
/// hélice em condição crítica) — a simplificação 1:1 do ciclo 8 realmente
/// mascarava este achado, como o próprio caveat previu. Nenhuma outra
/// checagem muda: tipback/tail-strike/carga de nariz/margem de
/// combustível/pista/robustez continuam PASSANDO com os MESMOS números da
/// campanha E10 (nenhum deles depende de `prop_clearance_critical_m`). O
/// caminho PASS deste check continua coberto pelas fixtures sintéticas de
/// `models::specs::tests`/`validation::constraint_checker::tests` (que
/// mantêm folga crítica positiva por construção). Ver
/// `docs/aircraft_spec.schema.md` §1, `docs/backlog.md` (item 1, marcado
/// RESOLVIDO) e `tests/cli.rs`/`tests/gear_tipback.rs`/`tests/schema_v4.rs`
/// para os pins honestos completos.
///
/// v5.3 (Task 3, ciclo10-sag-e-linha-de-tracao — bump **MINOR**, exceção
/// registrada, MESMO padrão da v5.2): formaliza o bump que as Tasks 1 e 2
/// do mesmo ciclo já haviam anunciado como "ainda dentro da v5.2" (ver as
/// duas notas de exceção acima, "segunda aplicação" e "terceira
/// aplicação"). Nenhum campo do JSON de saída foi
/// renomeado/removido/mudou de tipo/unidade nas Tasks 1-2 — consumidores
/// v5.2 continuam funcionando sem alteração para esses dois pontos; só
/// `RobustnessFlip` abaixo ganha um campo genuinamente NOVO (aditivo). Três
/// mudanças de conteúdo, nenhuma nova nesta task — só formalizadas:
///   1. **`propeller.prop_clearance_critical_m` mudou de fórmula DE NOVO**
///      (Task 1, `6c34f8f`): o curso do amortecedor de nariz usado no
///      cálculo deixa de ser o curso TOTAL do batente e passa a ser o curso
///      RESTANTE até o batente — `Δ_prop = (nose_oleo_stroke_mm/1000 ×
///      (1 − [gear].static_sag_fraction) + tire_deflation_delta_m) ×
///      fator`. Corrige uma dupla contagem: a compressão estática do nariz
///      já está embutida em `[gear].h_cg_ground_m` (a aeronave é sempre
///      modelada CARREGADA), e a fórmula anterior (ciclo 9) somava essa
///      mesma compressão de novo ao usar o curso TOTAL. Campo de
///      CONFIGURAÇÃO NOVO `[gear].static_sag_fraction` (faixa validada
///      (0,15, 0,55), baseline 0,33 — sem valor padrão implícito, TOMLs
///      pré-5.3 sem esse campo falham o parse por campo ausente, mesmo
///      padrão de migração sem erro dedicado já usado em `e_h`/
///      `runway_available_m`). Mesma
///      exceção MINOR da v5.2 aplicada de novo: é correção de bug de
///      modelagem física (dupla contagem), não mudança de contrato — nome/
///      tipo/unidade do campo JSON são idênticos. Baseline real E10:
///      `prop_clearance_critical_m` **≈−0,06416 m → ≈−0,00249 m** — MESMO
///      veredito (checagem #25 continua `FAIL`, por 2,5 mm em vez de 6,4
///      cm).
///   2. **Física nova do momento da linha de tração** (Task 2, `79b2263` +
///      erratum `713e846` + `f9231ea`): o balanço de momentos da rotação
///      (`agents::trim_authority::rotation_available_moment_nm`) ganha o
///      termo `−T(Vr)·prop_axis_above_cg_m` (braço sobre o CG, não sobre o
///      solo — ver erratum §2 de
///      `docs/superpowers/specs/2026-08-09-ciclo10-sag-e-linha-de-tracao-design.md`,
///      termo de d'Alembert cancela a porção `h_cg` porque a corrida de
///      decolagem é ACELERADA); o trim de cruzeiro
///      (`agents::trim_authority::cl_h_trim_cruise`) ganha `cm_thrust =
///      −T_cruzeiro·prop_axis_above_cg_m/(q·S_w·MAC)` somado ao `cm_ac`.
///      Nenhum campo novo — `trim.rotation_limit_pct_mac`,
///      `trim.cl_h_trim_cruise`, `trim.cd_trim` e
///      `weight.cg_limit_fwd_pct_mac` MANTÊM nome/tipo/unidade, só o VALOR
///      muda (e `rotation_limit_pct_mac` deixa de ser invariante ao peso —
///      passa a ser a envoltória MÁXIMA sobre os cenários, ver
///      `agents::trim_authority::rotation_fwd_limit_m`). Mesma exceção
///      MINOR: correção física (termo de momento que faltava), não mudança
///      de contrato. Baseline real: `rotation_limit_pct_mac` **8,533% →
///      13,355% MAC** (+4,82 pp); `validation_status` continua `"FAIL"` com
///      a MESMA 1 violação (#25, hélice, inalterada por esta mudança) e
///      ZERO flips de robustez — o que encolhe é a FOLGA de rotação do
///      cenário mais apertado ("Solo (piloto)", de +21,6% para +10,5%).
///   3. **`RobustnessFlip` ganha `limite_nominal` (f64) — este SIM
///      genuinamente ADITIVO** (Task 2, campo já serializado desde a v5.2
///      mas com documentação de schema adiada para esta task): o limite
///      NOMINAL do mesmo check, ao lado de `limite` (o limite efetivamente
///      aplicado ao mundo perturbado). Necessário porque a mudança #2 acima
///      fez o limite dianteiro de CG (rotação) deixar de ser invariante à
///      massa — dois mundos adversariais diferentes agora podem ter
///      `limite` diferentes entre si E diferentes do nominal.
///      `limite_nominal == limite` para checks cuja régua é invariante à
///      perturbação (tipback, carga de nariz, gates de desempenho/pista);
///      `limite_nominal != limite` é o sinal de que "a régua andou" (não
///      só o CG do mundo perturbado). Ver `docs/aircraft_spec.schema.md`
///      §4.
/// Nenhuma tolerância de teste foi afrouxada em nenhuma das três mudanças —
/// só pins re-centrados old→new com a MESMA tolerância. Ver
/// `docs/aircraft_spec.schema.md` §1/§4 e `tests/cli.rs`/
/// `tests/gear_tipback.rs`/`tests/schema_v4.rs`/`tests/generic_engine.rs`
/// para os pins honestos completos.
///
/// v5.4 (Task 3, ciclo11-subida-honesta — bump **MINOR**, exceção
/// registrada, MESMO padrão da v5.2/v5.3): nenhum campo do JSON de saída foi
/// renomeado/removido/mudou de tipo/unidade — consumidores v5.3 continuam
/// funcionando sem alteração. O bump é sobre serialização de um caso extremo:
/// `PerformanceSpec.to_50ft_paved_m` e `PerformanceSpec.to_50ft_grass_m`
/// podem receber legitimamente `f64::INFINITY` quando o obstáculo de 15m é
/// inatingível (razão de subida ≤ 0 no segmento de subida — ver
/// `agents::performance::takeoff_distance_50ft_m`, ramo `rc <= 0.0`). Antes
/// desta task, serde_json convertiria silenciosamente para `null` (RFC 8259
/// não tem representação de infinito), quebrando o round-trip — um consumidor
/// desserializando `null` falharia em conversão `f64`. Ambos os campos agora
/// usam `#[serde(with = "fatigue_life_serde")]` (módulo existente desde Task
/// 6.1 para tratar `StructuralSpec::fatigue_life_cycles`), que serializa o
/// infinito como a string `"infinita"` (documentado em
/// `docs/aircraft_spec.schema.md` §5). Política de bump: é correção de
/// SEMÂNTICA de serialização (mesmos nomes/tipos, só o efeito colateral da
/// conversão muda — de `null` silencioso para `"infinita"` explícita), não
/// mudança de CONTRATO de tipo/estrutura, aplicada como exceção MINOR
/// (mesmo padrão aprovado em v5.2 para `prop_clearance_critical_m` e v5.3
/// para `RobustnessFlip`).
/// Campanha ciclo 11 (2026-08-10): itens 2/3/5/7 do backlog (ciclo 11 task 1,
/// task 2, e esta task + ciclo 10 fix wave) formalizados junto. Nenhuma
/// tolerância de teste foi afrouxada — só pins re-centrados old→new com a
/// MESMA tolerância. Ver `docs/aircraft_spec.schema.md` §5 e
/// `tests/schema_v4.rs`/`tests/generic_engine.rs` para os pins honestos
/// completos.
pub const SCHEMA_VERSION: &str = "5.6";

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
    /// Limite dianteiro FÍSICO do envelope de CG (task trim-authority) —
    /// ver `TrimSpec`. Roda depois de `weight` (consome
    /// `WeightBalanceOutput::scenarios`) e antes da finalização de
    /// `weight.cg_limit_fwd_pct_mac`/`ScenarioResult::inside_envelope`
    /// (`WeightBalanceOutput::apply_trim`).
    pub trim: Option<TrimSpec>,
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
    /// Análise de robustez à incerteza do modelo de massas (Task 4, ciclo4
    /// -fidelidade-massas, schema v4.6) — pior-caso determinístico ±σ sobre
    /// as 7 massas estruturais, ver `RobustnessSpec`/`validation::
    /// robustness`. `Option` só por simetria com os demais campos do
    /// relatório; `main.rs` sempre o preenche.
    pub robustness: Option<RobustnessSpec>,
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

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// `GearSpec` sintético mínimo — só `nose_oleo_stroke_mm` importa para
    /// `PropellerSpec::fill_critical_clearance`; os demais campos recebem
    /// valores plausíveis arbitrários (não usados pelo método sob teste).
    fn gear_spec_teste(nose_oleo_stroke_mm: f64) -> GearSpec {
        GearSpec {
            gear_type: "triciclo retrátil".to_string(),
            track_width_m: 2.2,
            wheelbase_m: 2.3,
            tipover_angle_deg: 30.0,
            nose_load_max_pct: 15.0,
            nose_load_min_pct: 10.0,
            tipback_angle_deg: 16.0,
            tail_strike_margin_deg: 13.0,
            main_gear_load_n: 6_000.0,
            nose_gear_load_n: 2_000.0,
            main_oleo_stroke_mm: 200.0,
            nose_oleo_stroke_mm,
            main_tire: "6.00-6".to_string(),
            nose_tire: "5.00-5".to_string(),
            tire_pressure_psi: 30.0,
            max_sink_rate_ms: 3.0,
            retraction_time_s: 7.0,
            actuator_power_w: 500.0,
            total_weight_kg: 60.0,
        }
    }

    fn propeller_spec_teste(ground_clearance_m: f64) -> PropellerSpec {
        PropellerSpec {
            diameter_m: 1.80,
            blades: 2,
            source: "config".to_string(),
            tip_mach_static: 0.5,
            tip_mach_cruise_helical: 0.5,
            ground_clearance_m,
            diameter_max_by_mach_m: 2.0,
            diameter_max_by_clearance_m: 2.0,
            ok_mach_static: true,
            ok_mach_cruise: true,
            ok_clearance: true,
            prop_clearance_critical_m: 0.0, // placeholder — preenchido pelo método sob teste
        }
    }

    fn gear_cfg_teste_com_deflacao(tire_deflation_delta_m: f64) -> crate::models::aircraft_config::GearCfg {
        let mut cfg = crate::models::aircraft_config::test_fixtures::config_teste().gear;
        cfg.tire_deflation_delta_m = tire_deflation_delta_m;
        cfg
    }

    /// `GearCfg` sintético — só `static_sag_fraction` importa para a
    /// property nova (ciclo 10, task 1); os demais campos vêm intactos da
    /// fixture padrão (não usados pelo método sob teste).
    fn gear_cfg_teste_com_sag(static_sag_fraction: f64) -> crate::models::aircraft_config::GearCfg {
        let mut cfg = crate::models::aircraft_config::test_fixtures::config_teste().gear;
        cfg.static_sag_fraction = static_sag_fraction;
        cfg
    }

    /// `PropellerCfg` sintético — só `prop_plane_x_m` importa para
    /// `PropellerSpec::fill_critical_clearance` (ciclo 9); os demais campos
    /// vêm intactos da fixture padrão (não usados pelo método sob teste).
    fn prop_cfg_teste_com_prop_plane_x_m(prop_plane_x_m: f64) -> crate::models::aircraft_config::PropellerCfg {
        let mut cfg = crate::models::aircraft_config::test_fixtures::config_teste().propeller;
        cfg.prop_plane_x_m = prop_plane_x_m;
        cfg
    }

    /// Hand-check congelado (ciclo 10, task 1, deflexão estática no #25 —
    /// RED-first, números do brief da task, ±0,001 no resultado final):
    /// geometria do baseline E10 real À ÉPOCA (`x_nose_m` 1,30 m, `x_main_m`
    /// 3,66 m, `prop_plane_x_m` 0,20 m, `static_sag_fraction` 0,33) ⟹
    /// campanha E12 "nariz-only" (2026-08-10) recuou `x_nose_m` para 1,20 no
    /// baseline REAL (`config/aircraft/baseline_4seat.toml`) — este
    /// hand-check FICA CONGELADO nos literais antigos de propósito (verifica
    /// a FÓRMULA fechada, não o baseline atual; o baseline atual é coberto
    /// por `tests/schema_v4.rs::propeller_prop_clearance_critical_m_
    /// presente_e_numerico_proximo_do_esperado`), então NÃO muda com a
    /// adoção de E12.
    /// fator = (3,66−0,20)/(3,66−1,30) = 3,46/2,36 = 1,46610 (INALTERADO
    /// frente ao ciclo 9 — não depende de `static_sag_fraction`). Curso
    /// RESTANTE do nariz = 0,12746 × (1 − 0,33) = 0,12746 × 0,67 =
    /// 0,0853982. Δ_prop = (0,0853982 + 0,08) × 1,46610 = 0,242491 (curso
    /// restante + pneu murcho 0,08 m). Folga crítica = 0,24000 − 0,242491 =
    /// **−0,00249 m** — SUBSTITUI o hand-check do ciclo 9 (curso TOTAL do
    /// nariz, não restante: −0,06416 m). old→new: o corte de
    /// `static_sag_fraction` é a mudança sob teste, não a magnitude bruta
    /// dos outros termos — todos os demais literais (127,46 mm, 0,08 m,
    /// geometria) são os MESMOS do hand-check do ciclo 9.
    #[test]
    fn fill_critical_clearance_bate_com_a_formula_fechada() {
        let mut propeller = propeller_spec_teste(0.24000);
        let gear = gear_spec_teste(127.46);
        let mut gear_cfg = gear_cfg_teste_com_deflacao(0.08);
        gear_cfg.x_nose_m = 1.30;
        gear_cfg.x_main_m = 3.66;
        gear_cfg.static_sag_fraction = 0.33;
        let prop_cfg = prop_cfg_teste_com_prop_plane_x_m(0.20);

        propeller.fill_critical_clearance(&gear, &gear_cfg, &prop_cfg);

        // Pin EXATO (fórmula fechada dos literais acima, fator =
        // (3.66−0.20)/(3.66−1.30) = 1.466101694915254..., curso restante =
        // 0.12746 × 0.67 = 0.0853982 exato) — não ±0,001: uma tolerância
        // larga deixaria passar um erro de ~1,5% no fator, ou um esquecimento
        // do fator (1 − static_sag_fraction), sem quebrar o teste. Mesmo
        // padrão de precisão do hand-check anterior (ciclo 9). ≈−0,00249 m
        // no brief da task era a estimativa arredondada (±0,001) a
        // verificar no run — confirmada aqui com 9 casas.
        assert!((propeller.prop_clearance_critical_m - (-0.002490581355932)).abs() < 1e-9,
            "prop_clearance_critical_m = {:.15} (esperado exatamente -0.002490581355932 — \
             fator = 1.466101694915254..., curso restante = 0.0853982 m)",
            propeller.prop_clearance_critical_m);
    }

    /// Property NOVA (ciclo 9, RED-first): quanto MENOR `prop_plane_x_m`
    /// (hélice mais à frente do trem de nariz, braço maior até o pivô no
    /// trem principal), MENOR a folga crítica resultante — o fator
    /// `(x_main−prop_plane_x_m)/(x_main−x_nose_m)` CRESCE quando
    /// `prop_plane_x_m` diminui (numerador cresce, denominador fixo),
    /// amplificando `Δ_prop` e reduzindo `ground_clearance_m − Δ_prop`.
    /// Estritamente monotônico (não só não-crescente): os dois valores de
    /// `prop_plane_x_m` abaixo produzem fatores distintos por construção.
    #[test]
    fn folga_critica_diminui_quando_prop_plane_x_m_diminui() {
        let gear = gear_spec_teste(120.0);
        let gear_cfg = gear_cfg_teste_com_deflacao(0.05);

        let mut propeller_prop_plane_longe_do_nariz = propeller_spec_teste(0.300);
        propeller_prop_plane_longe_do_nariz.fill_critical_clearance(
            &gear, &gear_cfg, &prop_cfg_teste_com_prop_plane_x_m(0.30));

        let mut propeller_prop_plane_perto_do_nariz = propeller_spec_teste(0.300);
        propeller_prop_plane_perto_do_nariz.fill_critical_clearance(
            &gear, &gear_cfg, &prop_cfg_teste_com_prop_plane_x_m(0.10));

        assert!(propeller_prop_plane_perto_do_nariz.prop_clearance_critical_m
                < propeller_prop_plane_longe_do_nariz.prop_clearance_critical_m,
            "folga crítica com prop_plane_x_m MENOR (0.10 ⟹ {:.4}) deveria ficar ABAIXO da folga \
             com prop_plane_x_m MAIOR (0.30 ⟹ {:.4})",
            propeller_prop_plane_perto_do_nariz.prop_clearance_critical_m,
            propeller_prop_plane_longe_do_nariz.prop_clearance_critical_m);
    }

    /// Property (ciclo 8, task 2, RED-first): quanto MAIOR a deflexão do
    /// pneu configurada (`gear_cfg.tire_deflation_delta_m`), MENOR a folga
    /// crítica resultante — os dois termos subtraídos de `ground_clearance_m`
    /// são independentes, e este é estritamente monotônico decrescente no
    /// segundo. Ciclo 9: `prop_cfg` FIXO nos dois ramos (mesmo
    /// `prop_plane_x_m`, isolando o efeito da deflação de pneu do efeito do
    /// fator de braço, coberto separadamente acima).
    #[test]
    fn folga_critica_diminui_quando_deflacao_de_pneu_aumenta() {
        let prop_cfg = crate::models::aircraft_config::test_fixtures::config_teste().propeller;
        let gear = gear_spec_teste(120.0);

        let mut propeller_pouca_deflacao = propeller_spec_teste(0.300);
        propeller_pouca_deflacao.fill_critical_clearance(
            &gear, &gear_cfg_teste_com_deflacao(0.04), &prop_cfg);

        let mut propeller_muita_deflacao = propeller_spec_teste(0.300);
        propeller_muita_deflacao.fill_critical_clearance(
            &gear, &gear_cfg_teste_com_deflacao(0.10), &prop_cfg);

        assert!(propeller_muita_deflacao.prop_clearance_critical_m
                < propeller_pouca_deflacao.prop_clearance_critical_m,
            "folga crítica com deflação MAIOR ({:.4}) deveria ficar ABAIXO da folga com \
             deflação MENOR ({:.4})",
            propeller_muita_deflacao.prop_clearance_critical_m,
            propeller_pouca_deflacao.prop_clearance_critical_m);
    }

    /// Property NOVA (ciclo 10, task 1, RED-first): quanto MAIOR
    /// `gear_cfg.static_sag_fraction`, MAIOR a folga crítica resultante —
    /// o curso RESTANTE do nariz (`nose_oleo_stroke_mm × (1 −
    /// static_sag_fraction)`) ENCOLHE quando `static_sag_fraction` cresce
    /// (mais compressão estática já consumida ⟹ menos curso restando até
    /// o batente), reduzindo `Δ_prop` e aumentando
    /// `ground_clearance_m − Δ_prop`. Estritamente monotônico: os dois
    /// valores de `static_sag_fraction` abaixo produzem cursos restantes
    /// distintos por construção. `prop_cfg`/`tire_deflation_delta_m` FIXOS
    /// nos dois ramos, isolando o efeito do sag do efeito do fator de
    /// braço/deflação de pneu (cobertos separadamente acima).
    #[test]
    fn folga_critica_aumenta_quando_static_sag_fraction_aumenta() {
        let prop_cfg = crate::models::aircraft_config::test_fixtures::config_teste().propeller;
        let gear = gear_spec_teste(120.0);

        let mut propeller_pouco_sag = propeller_spec_teste(0.300);
        propeller_pouco_sag.fill_critical_clearance(
            &gear, &gear_cfg_teste_com_sag(0.20), &prop_cfg);

        let mut propeller_muito_sag = propeller_spec_teste(0.300);
        propeller_muito_sag.fill_critical_clearance(
            &gear, &gear_cfg_teste_com_sag(0.50), &prop_cfg);

        assert!(propeller_muito_sag.prop_clearance_critical_m
                > propeller_pouco_sag.prop_clearance_critical_m,
            "folga crítica com static_sag_fraction MAIOR (0.50 ⟹ {:.4}) deveria ficar ACIMA da \
             folga com static_sag_fraction MENOR (0.20 ⟹ {:.4})",
            propeller_muito_sag.prop_clearance_critical_m,
            propeller_pouco_sag.prop_clearance_critical_m);
    }

    /// Round-trip serde (Campanha ciclo 11, 2026-08-10, Task 3): campos
    /// `to_50ft_paved_m` e `to_50ft_grass_m` podem receber `f64::INFINITY`
    /// (quando `takeoff_distance_50ft_m` em `src/agents/performance.rs`
    /// devolve `s_ground + s_rotation + f64::INFINITY` no ramo
    /// `rc <= 0.0` — obstáculo inatingível). Antes desta task, serde_json
    /// convertiria silenciosamente para `null`, quebrando o round-trip.
    /// Este teste verifica que ambos os campos serializam/desserializam
    /// corretamente com valores infinitos E finitos.
    #[test]
    fn performance_spec_roundtrip_serde_com_infinito() {
        use serde_json;

        // Caso 1: `to_50ft_paved_m = f64::INFINITY` (obstáculo inatingível)
        let perf_infinite_paved = PerformanceSpec {
            v_cruise_kmh: 150.0,
            v_stall_kmh: 45.0,
            rc_sl_ms: 2.5,
            rc_cruise_alt_ms: 1.0,
            service_ceiling_m: 3500.0,
            to_distance_paved_m: 800.0,
            to_distance_grass_m: 1200.0,
            landing_distance_m: 600.0,
            range_km: 1500.0,
            endurance_h: 8.0,
            vx_kmh: 110.0,
            vy_kmh: 130.0,
            best_glide_kmh: 120.0,
            glide_ratio: 8.5,
            climb_gradient_pct: 12.5,
            to_50ft_paved_m: f64::INFINITY,
            to_50ft_grass_m: 1500.0,
            ldg_50ft_m: 700.0,
            ldg_50ft_grass_m: 850.0,
        };

        // Serializar e verificar que INFINITY vira "infinita"
        let json = serde_json::to_string(&perf_infinite_paved)
            .expect("serialização deveria funcionar");
        assert!(json.contains("\"infinita\""),
            "to_50ft_paved_m = INFINITY deveria serializar como string \"infinita\", \
             mas o JSON contém: {}", json);

        // Desserializar de volta e verificar que é INFINITY novamente
        let perf_deserialized: PerformanceSpec = serde_json::from_str(&json)
            .expect("desserialização deveria funcionar");
        assert!(perf_deserialized.to_50ft_paved_m.is_infinite() &&
                perf_deserialized.to_50ft_paved_m > 0.0,
            "to_50ft_paved_m desserializado deveria ser +INFINITY, recebido: {}",
            perf_deserialized.to_50ft_paved_m);

        // Caso 2: ambos os campos com INFINITY
        let perf_both_infinite = PerformanceSpec {
            to_50ft_paved_m: f64::INFINITY,
            to_50ft_grass_m: f64::INFINITY,
            ..perf_infinite_paved.clone()
        };

        let json2 = serde_json::to_string(&perf_both_infinite)
            .expect("serialização deveria funcionar");
        let perf_deserialized2: PerformanceSpec = serde_json::from_str(&json2)
            .expect("desserialização deveria funcionar");
        assert!(perf_deserialized2.to_50ft_grass_m.is_infinite() &&
                perf_deserialized2.to_50ft_grass_m > 0.0,
            "to_50ft_grass_m desserializado deveria ser +INFINITY, recebido: {}",
            perf_deserialized2.to_50ft_grass_m);

        // Caso 3: valores finitos continuam sendo números normais
        let perf_finite = PerformanceSpec {
            to_50ft_paved_m: 1800.0,
            to_50ft_grass_m: 2200.0,
            ..perf_infinite_paved.clone()
        };

        let json3 = serde_json::to_string(&perf_finite)
            .expect("serialização deveria funcionar");
        assert!(!json3.contains("\"infinita\""),
            "valores finitos não deveriam conter \"infinita\" no JSON: {}", json3);
        assert!(json3.contains("1800") || json3.contains("2200"),
            "JSON deveria conter os valores numéricos, mas contém: {}", json3);

        let perf_deserialized3: PerformanceSpec = serde_json::from_str(&json3)
            .expect("desserialização deveria funcionar");
        assert_eq!(perf_deserialized3.to_50ft_paved_m, 1800.0,
            "to_50ft_paved_m finito deveria roundtrip corretamente");
        assert_eq!(perf_deserialized3.to_50ft_grass_m, 2200.0,
            "to_50ft_grass_m finito deveria roundtrip corretamente");
    }

    /// Ciclo 12: as três distâncias LEGADO (`to_distance_paved_m`,
    /// `to_distance_grass_m`, `landing_distance_m`) passam a poder valer
    /// `+INFINITY` — a rolagem integrada devolve infinito quando a tração
    /// não basta para acelerar. Sem `fatigue_life_serde` elas virariam
    /// `null` no JSON (RFC 8259 não representa infinito), quebrando
    /// round-trip. Mesmo defeito que o ciclo 11 corrigiu em `to_50ft_*`.
    #[test]
    fn performance_spec_roundtrip_serde_com_infinito_nas_distancias_legado() {
        use serde_json;

        let mut p = PerformanceSpec {
            v_cruise_kmh: 150.0,
            v_stall_kmh: 45.0,
            rc_sl_ms: 2.5,
            rc_cruise_alt_ms: 1.0,
            service_ceiling_m: 3500.0,
            to_distance_paved_m: 800.0,
            to_distance_grass_m: 1200.0,
            landing_distance_m: 600.0,
            range_km: 1500.0,
            endurance_h: 8.0,
            vx_kmh: 110.0,
            vy_kmh: 130.0,
            best_glide_kmh: 120.0,
            glide_ratio: 8.5,
            climb_gradient_pct: 12.5,
            to_50ft_paved_m: 900.0,
            to_50ft_grass_m: 1500.0,
            ldg_50ft_m: 700.0,
            ldg_50ft_grass_m: 850.0,
        };

        p.to_distance_paved_m = f64::INFINITY;
        p.to_distance_grass_m = f64::INFINITY;
        p.landing_distance_m = f64::INFINITY;

        let json = serde_json::to_string(&p).expect("serializa");
        assert!(json.contains("\"to_distance_paved_m\":\"infinita\""), "{json}");
        assert!(json.contains("\"to_distance_grass_m\":\"infinita\""), "{json}");
        assert!(json.contains("\"landing_distance_m\":\"infinita\""), "{json}");
        assert!(!json.contains("null"), "nenhum campo pode virar null: {json}");

        let volta: PerformanceSpec = serde_json::from_str(&json).expect("desserializa");
        assert!(volta.to_distance_paved_m.is_infinite());
        assert!(volta.to_distance_grass_m.is_infinite());
        assert!(volta.landing_distance_m.is_infinite());
    }
}
