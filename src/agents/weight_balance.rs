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

use crate::agents::mass_model::StructuralMasses;
use crate::models::{
    aircraft_config::AircraftConfig,
    aircraft_state::AircraftState,
    engine::EngineSpec,
    requirements::Requirements,
    specs::{EmpennageSpec, WingSpec},
};

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

/// Nome do cenário de carga usado como CG de REFERÊNCIA DA MISSÃO para o
/// arrasto de trim de cruzeiro (Task 4, refino-ciclo2) — "meia-missão"
/// (tanque pela metade), ver `agents::trim_authority::cl_h_trim_cruise` para
/// a justificativa da escolha. Única fonte deste nome — usado tanto em
/// `scenarios_def` (abaixo) quanto em `agents::trim_authority`/
/// `orchestrator::size_aircraft` para localizar o cenário certo em
/// `WeightBalanceOutput::scenarios` sem duplicar o literal.
pub const MID_MISSION_SCENARIO_NAME: &str = "4 pax + bagagem + meia";

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

/// Deslocamento do braço da empenagem VERTICAL em relação a
/// `[arms].empennage_cg_m` (m) — a deriva fica um pouco à frente do CG de
/// referência da empenagem horizontal neste layout de cone de cauda
/// convencional. Antes da task refino-ciclo2 este valor vinha do
/// `arm_offset_m` do item fixo `emp_vertical` de `[[masses.items]]`
/// (idêntico, −0,2 m, tanto no baseline real quanto na fixture sintética);
/// agora que a massa é COMPUTADA (`agents::mass_model`, não mais um item de
/// config), o deslocamento vira uma constante de engenharia documentada
/// aqui em vez de um campo de TOML.
pub const EMP_VERTICAL_ARM_OFFSET_M: f64 = -0.2;

/// Retorna os itens de peso do avião vazio operacional (OEW):
///   - o motor (massa de `EngineSpec`);
///   - os itens NÃO-estruturais de `[[masses.items]]` da configuração
///     (equipamentos/instalação: PSRU+hélice+capô, resfriamento, aviônicos,
///     bateria, painel, mobiliário, cabos, portas/vidros, antepara — braço
///     resolvido via `arm_ref` + `arm_offset_m` contra `ArmConfig`);
///   - as 7 massas ESTRUTURAIS COMPUTADAS (`masses`, ciclo 3 —
///     `agents::mass_model`, equações de componente Raymer cap. 15.2 ×
///     fatores de composto de `[mass_model]`), com mapeamento ESTÁTICO
///     componente→braço.
///
/// Os braços dos 7 itens computados são EXATAMENTE os mesmos `arm_ref` que
/// os antigos itens fixos de `[[masses.items]]` usavam (`wing_struct`,
/// `fuselage_struct`, `empennage_cg` — com `EMP_VERTICAL_ARM_OFFSET_M`
/// para a deriva —, `gear_main`, `gear_nose`, `fuel_cg`): só a MASSA passou
/// a ser paramétrica, não o braço. Os 7 nomes correspondentes são
/// PROIBIDOS em `[[masses.items]]` (erro de migração claro, ver
/// `models::config::check_structural_mass_items_migration`) para que a
/// mesma massa não possa ser contada duas vezes.
pub fn oew_items(
    cfg: &AircraftConfig,
    engine: &EngineSpec,
    masses: &StructuralMasses,
) -> Vec<MassItem> {
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
    // Itens estruturais COMPUTADOS (ciclo 3, agents::mass_model) — mapeamento
    // estático componente→braço, MESMOS arm_refs dos antigos itens de
    // [[masses.items]] (removidos; erro de migração se presentes):
    items.push(MassItem { name: "asa".into(), mass_kg: masses.asa_kg, arm_m: arms.wing_struct_m });
    items.push(MassItem { name: "fuselagem".into(), mass_kg: masses.fuselagem_kg, arm_m: arms.fuselage_struct_m });
    items.push(MassItem { name: "emp_horizontal".into(), mass_kg: masses.emp_h_kg, arm_m: arms.empenagem_cg_m });
    items.push(MassItem { name: "emp_vertical".into(), mass_kg: masses.emp_v_kg, arm_m: arms.empenagem_cg_m + EMP_VERTICAL_ARM_OFFSET_M });
    items.push(MassItem { name: "trem_principal".into(), mass_kg: masses.trem_principal_kg, arm_m: arms.gear_main_m });
    items.push(MassItem { name: "trem_nariz".into(), mass_kg: masses.trem_nariz_kg, arm_m: arms.gear_nose_m });
    items.push(MassItem { name: "tanques".into(), mass_kg: masses.tanques_kg, arm_m: arms.fuel_cg_m });
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

/// Gradiente de downwash na empenagem horizontal (dε/dα) — aproximação
/// clássica de asa elíptica (Raymer, "Aircraft Design: A Conceptual
/// Approach", cap. 16):
///
///   dε/dα = 2·a_w / (π·AR_w)
///
/// O escoamento descendente (downwash) induzido pela asa reduz o ângulo de
/// ataque efetivo — e portanto a contribuição estabilizadora — da
/// empenagem horizontal: a contribuição de `Δ_stab` no NP é multiplicada
/// por `(1 − dε/dα)` em `neutral_point_m`. `a_w` é `lift_curve_slope` da
/// asa [1/rad]; `ar_w` é o alongamento da asa.
pub fn downwash_gradient(a_w: f64, ar_w: f64) -> f64 {
    2.0 * a_w / (std::f64::consts::PI * ar_w)
}

/// Contribuição da fuselagem no ponto neutro — método de Multhopp
/// simplificado (Raymer, "Aircraft Design: A Conceptual Approach", cap. 16,
/// eq. 16.25). O escoamento sobre a fuselagem à frente do CA da asa produz
/// um momento de arfagem desestabilizador (Cm_α positivo), que AVANÇA o NP
/// (desloca para a frente/nariz):
///
///   Cm_α_fus = K_f·W_f²·L_f / (MAC·S_w)   [1/grau]
///   Δx_np_fus/MAC = −Cm_α_fus·(180/π) / a_w
///
/// `K_f` (`fuselage_kf`, config `[stability]`) vem da fig. 16.14 de Raymer
/// (faixa típica 0.01–0.03 conforme a posição vertical da asa na
/// fuselagem); `W_f` é a largura da cabine (`fuselage.cabin_width_m`), `L_f`
/// o comprimento da fuselagem (`fuselage.length_m`). `Cm_α_fus` sai em
/// 1/grau (convenção da fig. 16.14 de Raymer) e precisa ser convertida para
/// 1/rad (×180/π) antes de dividir por `a_w` [1/rad]. Retorna a fração
/// ADIMENSIONAL (já dividida por MAC) do deslocamento do NP — negativa
/// (avanço do NP), a ser multiplicada por `mac_m` pelo chamador.
pub fn fuselage_np_shift_mac(
    fuselage_kf: f64,
    cabin_width_m: f64,
    fuselage_length_m: f64,
    mac_m: f64,
    wing_area_m2: f64,
    a_w: f64,
) -> f64 {
    let cm_alpha_fus_per_deg =
        fuselage_kf * cabin_width_m * cabin_width_m * fuselage_length_m / (mac_m * wing_area_m2);
    let cm_alpha_fus_per_rad = cm_alpha_fus_per_deg * (180.0 / std::f64::consts::PI);
    -cm_alpha_fus_per_rad / a_w
}

/// Posição do Ponto Neutro (NP) da aeronave — método da área de cauda de
/// Raymer, com a empenagem REALMENTE dimensionada (`EmpennageAgent`, Task
/// 4.1) em vez dos antigos `s_ratio`/`a_t/a_w` hardcoded, e CORRIGIDA por
/// downwash e pela contribuição da fuselagem (Multhopp simplificado —
/// fidelidade do envelope de CG, ver `downwash_gradient`/
/// `fuselage_np_shift_mac` acima).
///
/// x_np = x_ac_wing + Δ_stab·(1 − dε/dα) + Δx_np_fus/MAC·MAC
///   onde Δ_stab = (a_t/a_w)·(S_h/S_w)·(l_h/MAC)·η_h · MAC
///
/// a_w, a_t vêm de `lift_curve_slope` aplicada ao AR da asa e ao AR da
/// empenagem horizontal (`emp.ar_h`), respectivamente. S_h, l_h, η_h vêm de
/// `emp` (saída do `EmpennageAgent`, dimensionada por coeficiente de
/// volume). Sem estas duas correções o NP fica artificialmente atrás
/// (ignora que parte da eficácia da empenagem é cancelada pelo downwash da
/// asa, e que a fuselagem por si só já é desestabilizadora) — achado
/// honesto documentado no relatório da task de downwash/fuselagem.
pub fn neutral_point_m(
    wing_le_root_m: f64,
    mac_m: f64,
    wing_ar: f64,
    emp: &EmpennageSpec,
    wing_area_m2: f64,
    fuselage_cabin_width_m: f64,
    fuselage_length_m: f64,
    fuselage_kf: f64,
) -> f64 {
    // CA da asa: bordo de ataque do MAC + 25% MAC (asa sem enflechamento —
    // y_MAC não desloca x_ac longitudinalmente nesse caso).
    let x_ac_wing = wing_le_root_m + 0.25 * mac_m;

    let a_w = lift_curve_slope(wing_ar);
    let a_t = lift_curve_slope(emp.ar_h);

    let s_ratio = emp.s_horizontal_m2 / wing_area_m2; // S_h / S_w
    let delta_stab = (a_t / a_w) * s_ratio * (emp.arm_h_m / mac_m) * emp.eta_h * mac_m;

    let deps_dalpha = downwash_gradient(a_w, wing_ar);
    let delta_stab_com_downwash = delta_stab * (1.0 - deps_dalpha);

    let dx_fus_mac = fuselage_np_shift_mac(
        fuselage_kf, fuselage_cabin_width_m, fuselage_length_m, mac_m, wing_area_m2, a_w,
    );

    x_ac_wing + delta_stab_com_downwash + dx_fus_mac * mac_m
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

// ─── ENVELOPE DE CG ADMISSÍVEL (Task 4.4 + task trim-authority) ──────────────
//
// Em contraste com `cg_fwd`/`cg_aft` (WeightSpec::cg_mac_fwd_pct/aft_pct),
// que são os extremos apenas OBSERVADOS entre os cenários de carga, o
// envelope ADMISSÍVEL vem de dois critérios físicos INDEPENDENTES — um por
// extremo, calculados em DUAS FASES por dois agentes diferentes (a
// dependência é circular: o limite de rotação por cenário precisa dos
// CENÁRIOS do WeightBalanceAgent, que por sua vez precisariam do limite
// para decidir `inside_envelope` — resolvida rodando primeiro os
// cenários/limite traseiro aqui, depois o `TrimAuthorityAgent`, depois
// `apply_trim` abaixo finaliza o veredito):
//
//   - Limite TRASEIRO (`cg_limit_aft_m` abaixo): invertendo `static_margin`
//     — SM = (x_np−x_cg)/MAC, logo x_cg = x_np−SM·MAC; como SM é
//     DECRESCENTE em x_cg, o SM MÍNIMO aceitável (`sm_min`) corresponde ao
//     CG MAIS ATRÁS aceitável. Não depende de peso — calculado aqui,
//     direto, dentro de `WeightBalanceAgent::run`.
//   - Limite DIANTEIRO: FÍSICO, não mais um proxy de margem estática — vem
//     da autoridade de profundor disponível nas manobras de flare/rotação
//     (`agents::trim_authority::TrimAuthorityAgent`), calculado POR
//     CENÁRIO (a rotação depende do peso). `WeightBalanceAgent::run`
//     calcula só um veredito PARCIAL de `inside_envelope` (só o critério
//     traseiro, que já é conhecido); `apply_trim` (abaixo) finaliza com o
//     critério dianteiro assim que o `TrimAuthorityAgent` rodar — ver
//     `orchestrator::size_aircraft`.

/// Limite TRASEIRO do envelope de CG — posição do CG mais atrás admissível
/// antes de violar a margem estática mínima (`sm_min`, piso de estabilidade
/// longitudinal, tipicamente 0.05):
///
///   x_cg_aft = x_np − sm_min·MAC
pub fn cg_limit_aft_m(x_np: f64, sm_min: f64, mac: f64) -> f64 {
    x_np - sm_min * mac
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
    /// Estável = NP atrás do CG (SM > 0.03, piso puramente de sinal/robustez
    /// numérica). Mantido por referência/regressão — NÃO é mais o critério
    /// de aceite do projeto: use `inside_envelope`, que verifica o CG
    /// contra os limites de `[stability]` (Task 4.4).
    pub stable: bool,
    /// Verdadeiro quando o CG do cenário está DENTRO do envelope admissível
    /// — critério de aceite do projeto (Task 4.4 + task trim-authority),
    /// substitui o antigo `sm > 0.03` isolado. `WeightBalanceAgent::run`
    /// só consegue avaliar o critério TRASEIRO (`x_cg_m ≤ cg_limit_aft_m`,
    /// via `sm_min` — não depende de peso); o critério DIANTEIRO (físico,
    /// por cenário — flare/rotação, `agents::trim_authority`) é aplicado
    /// depois por `WeightBalanceOutput::apply_trim`, que faz o AND final.
    /// NÃO leia este campo como veredito final antes de `apply_trim` ter
    /// rodado (`orchestrator::size_aircraft` sempre chama antes de
    /// devolver `SizedAircraft`).
    pub inside_envelope: bool,
}

impl WeightBalanceOutput {
    /// Finaliza o envelope de CG ADMISSÍVEL aplicando o limite dianteiro
    /// FÍSICO calculado pelo `TrimAuthorityAgent` (flare + rotação, task
    /// trim-authority) — segunda metade da dependência circular descrita
    /// no comentário da seção "ENVELOPE DE CG ADMISSÍVEL" mais acima em
    /// `weight_balance.rs`: `WeightBalanceAgent::run` já calculou os
    /// CENÁRIOS e o limite TRASEIRO; `TrimAuthorityAgent::run` consumiu
    /// esses cenários (peso) + a geometria para calcular o limite
    /// DIANTEIRO; esta função combina os dois em
    /// `inside_envelope`/`spec.cg_limit_fwd_pct_mac`.
    ///
    /// Chamado por `orchestrator::size_aircraft` logo após
    /// `TrimAuthorityAgent::run`, ANTES de devolver `SizedAircraft` — nunca
    /// leia `spec.cg_limit_fwd_pct_mac` (fica `NaN`) nem confie em
    /// `inside_envelope` refletir o critério dianteiro antes desta chamada.
    ///
    /// `spec.cg_limit_fwd_pct_mac` recebe `max(trim.flare_limit_pct_mac,
    /// trim.rotation_limit_pct_mac)` — desde o fix de revisão da task
    /// trim-authority (cancelamento de peso na rotação, ver
    /// `agents::trim_authority::rotation_fwd_limit_m`), OS DOIS são
    /// números ÚNICOS (não variam por cenário) — este limite se aplica
    /// IGUALMENTE a TODOS os cenários, iterados abaixo um a um por
    /// clareza estrutural (não porque o valor mude entre eles).
    pub fn apply_trim(&mut self, trim: &crate::models::specs::TrimSpec) {
        let fwd_limit_pct = trim.flare_limit_pct_mac.max(trim.rotation_limit_pct_mac);
        self.spec.cg_limit_fwd_pct_mac = fwd_limit_pct;

        for sc in &mut self.scenarios {
            sc.inside_envelope = sc.inside_envelope && sc.cg_pct_mac >= fwd_limit_pct;
        }
    }
}

impl WeightBalanceAgent {
    pub fn run(
        state: &AircraftState,
        wing: &WingSpec,
        engine: &EngineSpec,
        cfg: &AircraftConfig,
        req: &Requirements,
        emp: &EmpennageSpec,
        masses: &StructuralMasses,
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
            cfg.fuselage.cabin_width_m, cfg.fuselage.length_m, cfg.stability.fuselage_kf,
        );

        // Peso vazio operacional
        let oew_items = oew_items(cfg, engine, masses);
        // `x_cg_oew` (CG do OEW isolado) não é consumido por este agente —
        // só a lista de itens (`oew_items`) importa daqui pra frente, cada
        // cenário de carga recalcula seu próprio CG a partir dela.
        let (oew_kg, _x_cg_oew) = cg_from_items(&oew_items);

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
            LoadScenario { name: MID_MISSION_SCENARIO_NAME,  pax_front: 2, pax_rear: 2, baggage_kg: 80.0, fuel_fraction: 0.5 },
            LoadScenario { name: "4 pax + bagagem vazio",   pax_front: 2, pax_rear: 2, baggage_kg: 80.0, fuel_fraction: 0.1 },
        ];

        // Envelope de CG admissível (Task 4.4 + task trim-authority) —
        // limite TRASEIRO físico (m do nariz), derivado de `[stability].
        // sm_min`. O limite DIANTEIRO não é conhecido aqui ainda (depende
        // do `TrimAuthorityAgent`, que por sua vez depende dos cenários
        // calculados abaixo) — ver comentário da seção "ENVELOPE DE CG
        // ADMISSÍVEL" mais acima e `WeightBalanceOutput::apply_trim`.
        let x_cg_limit_aft = cg_limit_aft_m(x_np, cfg.stability.sm_min, mac);

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

            // Veredito PARCIAL (só o critério traseiro, via sm_min) — o
            // critério dianteiro (TrimAuthorityAgent) é AND'ado depois por
            // `apply_trim`. Ver docstring de `ScenarioResult::inside_envelope`.
            let inside_envelope = x_cg <= x_cg_limit_aft;

            scenario_results.push(ScenarioResult {
                name:          sc.name,
                total_mass_kg: total_mass,
                x_cg_m:        x_cg,
                cg_pct_mac:    cg_pct,
                static_margin: sm,
                // Estável = NP atrás do CG (SM > 0). CG muito à frente (SM > 30%)
                // é problema de autoridade de profundor, não de instabilidade.
                // Mantido por referência — critério de aceite é
                // `inside_envelope` (Task 4.4).
                stable:        sm > 0.03,
                inside_envelope,
            });
        }

        // CG mais à frente e mais atrás observados (extremos dos cenários —
        // distinto do envelope ADMISSÍVEL calculado acima)
        let cg_fwd = scenario_results.iter().map(|s| s.cg_pct_mac).fold(f64::INFINITY,  f64::min);
        let cg_aft = scenario_results.iter().map(|s| s.cg_pct_mac).fold(f64::NEG_INFINITY, f64::max);
        let sm_min_observado = scenario_results.iter().map(|s| s.static_margin).fold(f64::INFINITY, f64::min);

        // Envelope TRASEIRO em %MAC — mesma conversão usada para os
        // cenários (`cg_pct_mac`), aplicada ao limite físico traseiro. O
        // limite DIANTEIRO ainda não é conhecido aqui — `f64::NAN` é um
        // placeholder PROPOSITAL (não 0.0 nem qualquer valor "plausível")
        // até `WeightBalanceOutput::apply_trim` rodar: comparações com NaN
        // resolvem sempre `false`, então um chamador que esqueça de chamar
        // `apply_trim` antes de ler `cg_limit_fwd_pct_mac` falha ALTO
        // (NaN visível em qualquer print/serialização), não silenciosamente
        // com um número plausível mas errado.
        let cg_limit_aft_pct = cg_pct_mac(x_cg_limit_aft, x_mac_le, mac);

        WeightBalanceOutput {
            spec: crate::models::specs::WeightSpec {
                oew_kg,
                mtow_kg:          mtow_max,
                payload_kg:       req.payload_kg(),
                fuel_mass_kg:     fuel_mass_full,
                cg_mac_fwd_pct:   cg_fwd,
                cg_mac_aft_pct:   cg_aft,
                static_margin_pct: sm_min_observado * 100.0,
                cg_limit_fwd_pct_mac: f64::NAN,
                cg_limit_aft_pct_mac: cg_limit_aft_pct,
                structural_masses: crate::models::specs::StructuralMassesSpec {
                    asa_kg:            masses.asa_kg,
                    fuselagem_kg:      masses.fuselagem_kg,
                    emp_h_kg:          masses.emp_h_kg,
                    emp_v_kg:          masses.emp_v_kg,
                    trem_principal_kg: masses.trem_principal_kg,
                    trem_nariz_kg:     masses.trem_nariz_kg,
                    tanques_kg:        masses.tanques_kg,
                    composite_factor_wing:        cfg.mass_model.composite_factor_wing,
                    composite_factor_tail:        cfg.mass_model.composite_factor_tail,
                    composite_factor_fuselage:    cfg.mass_model.composite_factor_fuselage,
                    composite_factor_gear:        cfg.mass_model.composite_factor_gear,
                    composite_factor_fuel_system: cfg.mass_model.composite_factor_fuel_system,
                },
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

    /// Constrói a `EmpennageSpec` real da fixture sintética (mesma sequência
    /// de agentes de `wing_teste`/pipeline completo).
    fn emp_teste(cfg: &AircraftConfig) -> EmpennageSpec {
        let state = AircraftState::from_config(cfg);
        let req = crate::models::requirements::test_fixtures::requisitos_teste();
        let wing = AerodynamicsAgent::run(&state, &req);
        EmpennageAgent::run(&wing, cfg)
    }

    /// Massas estruturais COMPUTADAS da fixture sintética (ciclo 3,
    /// `agents::mass_model`) — mesma sequência de agentes do orchestrator,
    /// com `mtow` = palpite inicial de `[sizing]` e `n_design` = seed 3,8 do
    /// lag-1 (ver `orchestrator::size_aircraft_with_max_iters`). Estes
    /// testes de unidade não iteram o MTOW: exercitam `oew_items`/
    /// `WeightBalanceAgent::run` com um par (MTOW, n_design) fixo e
    /// documentado, não o ponto fixo convergido.
    fn masses_teste(cfg: &AircraftConfig, engine: &EngineSpec, emp: &EmpennageSpec)
        -> crate::agents::mass_model::StructuralMasses
    {
        let state = AircraftState::from_config(cfg);
        let req = crate::models::requirements::test_fixtures::requisitos_teste();
        let wing = AerodynamicsAgent::run(&state, &req);
        crate::agents::mass_model::MassModelAgent::run(
            cfg, engine, &req, &wing, emp, cfg.sizing.mtow_initial_guess_kg, 3.8,
        )
    }

    #[test]
    fn oew_dentro_do_orcamento() {
        let cfg    = config_teste();
        let engine = engine_teste();
        let emp    = emp_teste(&cfg);
        let masses = masses_teste(&cfg, &engine, &emp);
        let items  = oew_items(&cfg, &engine, &masses);
        let (oew, _) = cg_from_items(&items);
        println!("OEW (fixture sintética) = {oew:.1} kg");
        // Itens NÃO-estruturais sintéticos de config_teste() (259 kg, já sem
        // as 5 massas estruturais fixas — ciclo 3) + as 7 massas
        // COMPUTADAS (agents::mass_model) + motor sintético (150 kg) —
        // faixa com folga ao redor do valor observado, testando o pipeline
        // de resolução de arm_ref + o acoplamento com o modelo de massas,
        // não um número mágico.
        assert!(oew > 700.0 && oew < 850.0,
            "OEW = {oew:.1} kg fora do intervalo esperado (700–850 kg)");
    }

    // ─── oew_items: as 7 massas ESTRUTURAIS computadas (ciclo 3) ──────────

    /// `oew_items` deve conter os 7 itens estruturais com massa EXATAMENTE
    /// igual à computada por `agents::mass_model` (nenhuma reconta, nenhum
    /// fator aplicado no caminho) e com os braços do mapeamento estático
    /// componente→braço (MESMOS `arm_ref` dos antigos itens fixos).
    #[test]
    fn oew_items_usa_as_massas_computadas_com_o_mapeamento_estatico_de_bracos() {
        let cfg    = config_teste();
        let engine = engine_teste();
        let emp    = emp_teste(&cfg);
        let masses = masses_teste(&cfg, &engine, &emp);
        let items  = oew_items(&cfg, &engine, &masses);
        let arms   = ArmConfig::from_config(&cfg);

        let esperado: [(&str, f64, f64); 7] = [
            ("asa",            masses.asa_kg,             arms.wing_struct_m),
            ("fuselagem",      masses.fuselagem_kg,       arms.fuselage_struct_m),
            ("emp_horizontal", masses.emp_h_kg,           arms.empenagem_cg_m),
            ("emp_vertical",   masses.emp_v_kg,           arms.empenagem_cg_m + EMP_VERTICAL_ARM_OFFSET_M),
            ("trem_principal", masses.trem_principal_kg,  arms.gear_main_m),
            ("trem_nariz",     masses.trem_nariz_kg,      arms.gear_nose_m),
            ("tanques",        masses.tanques_kg,         arms.fuel_cg_m),
        ];
        for (nome, massa_esperada, braco_esperado) in esperado {
            let item = items.iter().find(|i| i.name == nome)
                .unwrap_or_else(|| panic!("oew_items deveria conter o item computado '{nome}'"));
            println!("{nome}: mass_kg={:.6} (esperado {massa_esperada:.6})  \
                      arm_m={:.6} (esperado {braco_esperado:.6})", item.mass_kg, item.arm_m);
            assert!((item.mass_kg - massa_esperada).abs() < 1e-12,
                "massa de '{nome}' ({:.9}) deveria bater EXATAMENTE com a computada por \
                 agents::mass_model ({massa_esperada:.9})", item.mass_kg);
            assert!((item.arm_m - braco_esperado).abs() < 1e-12,
                "braço de '{nome}' ({:.9}) deveria bater com o mapeamento estático \
                 ({braco_esperado:.9})", item.arm_m);
        }
    }

    /// Propriedade: aumentar `v_h` aumenta S_h (`agents::empennage`), que
    /// por sua vez deve aumentar ESTRITAMENTE a massa COMPUTADA do item
    /// `emp_horizontal` (Raymer 15.2, expoente 0.896 em S_ht) — o item
    /// `emp_vertical`, que não depende de `v_h`, permanece INALTERADO.
    #[test]
    fn massa_emp_horizontal_aumenta_estritamente_quando_v_h_aumenta() {
        let cfg_base = config_teste();
        let engine = engine_teste();
        let emp_base = emp_teste(&cfg_base);
        let masses_base = masses_teste(&cfg_base, &engine, &emp_base);
        let items_base = oew_items(&cfg_base, &engine, &masses_base);
        let mass_h_base = items_base.iter().find(|i| i.name == "emp_horizontal").unwrap().mass_kg;
        let mass_v_base = items_base.iter().find(|i| i.name == "emp_vertical").unwrap().mass_kg;

        let mut cfg_maior = cfg_base.clone();
        cfg_maior.empennage.v_h *= 1.3;
        let emp_maior = emp_teste(&cfg_maior);
        let masses_maior = masses_teste(&cfg_maior, &engine, &emp_maior);
        let items_maior = oew_items(&cfg_maior, &engine, &masses_maior);
        let mass_h_maior = items_maior.iter().find(|i| i.name == "emp_horizontal").unwrap().mass_kg;
        let mass_v_maior = items_maior.iter().find(|i| i.name == "emp_vertical").unwrap().mass_kg;

        println!("mass_h: base={mass_h_base:.4} maior={mass_h_maior:.4} | \
                   mass_v: base={mass_v_base:.4} maior={mass_v_maior:.4}");
        assert!(mass_h_maior > mass_h_base,
            "massa de emp_horizontal deveria aumentar estritamente com v_h: \
             base={mass_h_base:.4} maior={mass_h_maior:.4}");
        assert!((mass_v_maior - mass_v_base).abs() < 1e-9,
            "massa de emp_vertical não deveria mudar com v_h (S_v não depende de v_h): \
             base={mass_v_base:.9} maior={mass_v_maior:.9}");
    }

    #[test]
    fn todos_os_cenarios_estaveis() {
        let cfg    = config_teste();
        let state  = AircraftState::from_config(&cfg);
        let req    = crate::models::requirements::test_fixtures::requisitos_teste();
        let wing   = AerodynamicsAgent::run(&state, &req);
        let emp    = EmpennageAgent::run(&wing, &cfg);
        let engine = engine_teste();
        let masses = masses_teste(&cfg, &engine, &emp);
        let wb     = WeightBalanceAgent::run(&state, &wing, &engine, &cfg, &req, &emp, &masses);

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
        let masses = masses_teste(&cfg, &engine, &emp);
        let wb     = WeightBalanceAgent::run(&state, &wing, &engine, &cfg, &req, &emp, &masses);

        println!("MTOW (fixture sintética) = {:.1} kg", wb.spec.mtow_kg);
        // Faixa ampla ao redor do MTOW observado empiricamente para a
        // fixture sintética (célula + motor de teste) — suficientemente
        // apertada para pegar regressões reais no somatório de peso, mas sem
        // acoplar o teste a um valor exato.
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

    // ─── Downwash + Fuselagem (Multhopp) — fidelidade do NP ─────────────────

    /// Hand-check (Raymer cap. 16, aproximação clássica de asa elíptica):
    /// dε/dα = 2·a_w/(π·AR_w). Com a_w=5.1554 (lift_curve_slope(10.0394),
    /// AR baseline real) e AR=10.0394:
    ///   dε/dα = 2·5.1554/(π·10.0394) = 10.3108/31.5464 = 0.3268
    #[test]
    fn downwash_gradient_hand_check() {
        let deps = downwash_gradient(5.1554, 10.0394);
        println!("dε/dα = {deps:.4}");
        assert!((deps - 0.3268).abs() < 0.001, "dε/dα = {deps:.4} (esperado ≈0.3268)");
    }

    /// Propriedade: aumentar o AR da asa (a_w fixo) reduz dε/dα estritamente
    /// — asas de maior alongamento espalham a esteira de vórtice de ponta
    /// mais fina, reduzindo o downwash médio na cauda.
    #[test]
    fn downwash_gradient_diminui_quando_ar_aumenta() {
        let baixo = downwash_gradient(5.1554, 8.0);
        let alto  = downwash_gradient(5.1554, 14.0);
        assert!(alto < baixo,
            "dε/dα com AR maior ({alto:.4}) deveria ser MENOR que com AR menor ({baixo:.4})");
    }

    /// Hand-check (Multhopp simplificado, Raymer eq. 16.25): Cm_α_fus =
    /// K_f·W_f²·L_f/(MAC·S_w) [1/grau], convertido para 1/rad (×180/π) antes
    /// de dividir por a_w [1/rad]. Valores do baseline real: K_f=0.02,
    /// W_f=1.22 (cabin_width_m), L_f=8.2 (length_m), MAC=1.2463, S_w=14.2,
    /// a_w=5.1554:
    ///   Cm_α_fus = 0.02·1.4884·8.2/(1.2463·14.2) = 0.24410/17.6975
    ///            = 0.013793 /grau = 0.79027 /rad
    ///   Δx_np_fus/MAC = −0.79027/5.1554 = −0.15329
    #[test]
    fn fuselage_np_shift_mac_hand_check() {
        let dx = fuselage_np_shift_mac(0.02, 1.22, 8.2, 1.2463, 14.2, 5.1554);
        println!("Δx_np_fus/MAC = {dx:.5}");
        assert!((dx - (-0.15329)).abs() < 0.001, "Δx_np_fus/MAC = {dx:.5} (esperado ≈-0.15329)");
    }

    /// Propriedade: aumentar `fuselage_kf` aumenta o momento desestabilizador
    /// da fuselagem — o deslocamento do NP (negativo, avança o NP) fica MAIS
    /// negativo estritamente.
    #[test]
    fn fuselage_np_shift_mac_fica_mais_negativo_quando_kf_aumenta() {
        let baixo = fuselage_np_shift_mac(0.015, 1.22, 8.2, 1.2463, 14.2, 5.1554);
        let alto  = fuselage_np_shift_mac(0.030, 1.22, 8.2, 1.2463, 14.2, 5.1554);
        assert!(alto < baixo,
            "Δx_np_fus/MAC com kf maior ({alto:.5}) deveria ser MAIS NEGATIVO que com kf \
             menor ({baixo:.5})");
    }

    /// Propriedade: aumentar `fuselage_kf` move o NP integrado (`neutral_point_m`)
    /// para a FRENTE (x menor), estritamente — mais momento desestabilizador
    /// de fuselagem exige mais estabilizador atrás, empurrando o NP para
    /// perto do CA da asa.
    #[test]
    fn np_avanca_quando_fuselage_kf_aumenta() {
        let cfg    = config_teste();
        let state  = AircraftState::from_config(&cfg);
        let req    = crate::models::requirements::test_fixtures::requisitos_teste();
        let wing   = AerodynamicsAgent::run(&state, &req);
        let emp    = EmpennageAgent::run(&wing, &cfg);
        let c_r = chord_root(wing.area_m2, wing.span_m, wing.taper_ratio);
        let mac = mean_aerodynamic_chord(c_r, wing.taper_ratio);

        let x_np_baixo = neutral_point_m(
            cfg.wing.le_root_x_m, mac, wing.aspect_ratio, &emp, wing.area_m2,
            cfg.fuselage.cabin_width_m, cfg.fuselage.length_m, 0.012,
        );
        let x_np_alto = neutral_point_m(
            cfg.wing.le_root_x_m, mac, wing.aspect_ratio, &emp, wing.area_m2,
            cfg.fuselage.cabin_width_m, cfg.fuselage.length_m, 0.030,
        );
        println!("x_np(kf=0.012)={x_np_baixo:.4}m  x_np(kf=0.030)={x_np_alto:.4}m");
        assert!(x_np_alto < x_np_baixo,
            "NP deveria avançar (x menor) quando fuselage_kf aumenta: kf_baixo={x_np_baixo:.4}m \
             kf_alto={x_np_alto:.4}m");
    }

    /// Hand-check integrado do NP (baseline real, ver task-brief): x_ac_wing
    /// = 2.90+0.25·1.2463 = 3.211575m; delta_stab (empenagem, sem correção)
    /// = 0.47456·MAC; com downwash: ×(1−0.3268)=×0.6732 → 0.31947·MAC;
    /// contribuição da fuselagem: −0.15329·MAC. NP%MAC = 0.25+0.31947−0.15329
    /// = 0.41618 → x_np = 2.90+0.41618·1.2463 = 3.4187m.
    ///
    /// Usa a EmpennageSpec real do baseline (via EmpennageAgent) para não
    /// depender de uma spec sintética à mão — a_t/a_w e S_h/S_w vêm do
    /// dimensionamento real (V_h=0.70, AR_h=4.0).
    #[test]
    fn neutral_point_m_hand_check_downwash_e_fuselagem() {
        // Reconstrói a config do baseline real (não a fixture sintética —
        // este hand-check usa os números do TOML real, ver task brief).
        let toml = std::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("config/aircraft/baseline_4seat.toml"),
        ).expect("falha ao ler baseline_4seat.toml do disco");
        let cfg = crate::models::config::parse_aircraft(&toml)
            .expect("baseline real deveria ser uma configuração válida");
        let state = AircraftState::from_config(&cfg);
        let req = crate::models::requirements::test_fixtures::requisitos_teste();
        let wing = AerodynamicsAgent::run(&state, &req);
        let emp = EmpennageAgent::run(&wing, &cfg);
        let c_r = chord_root(wing.area_m2, wing.span_m, wing.taper_ratio);
        let mac = mean_aerodynamic_chord(c_r, wing.taper_ratio);

        let x_np = neutral_point_m(
            cfg.wing.le_root_x_m, mac, wing.aspect_ratio, &emp, wing.area_m2,
            cfg.fuselage.cabin_width_m, cfg.fuselage.length_m, cfg.stability.fuselage_kf,
        );
        println!("x_np (baseline, downwash+fuselagem) = {x_np:.4}m  MAC={mac:.4}m");
        // Campanha E1–E6 (2026-08-05): v_h 0.70→0.85 (S_h 2.58→3.13 m²) recua
        // o NP — mais estabilizador. Pin antigo: 3.4187m. Novo: 3.5040m.
        assert!((x_np - 3.5040).abs() < 0.005, "x_np = {x_np:.4}m (esperado ≈3.5040m ±0.005)");
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
            cfg.fuselage.cabin_width_m, cfg.fuselage.length_m, cfg.stability.fuselage_kf,
        );

        let mut cfg_maior = cfg.clone();
        cfg_maior.empennage.v_h *= 1.3;
        let emp_maior = EmpennageAgent::run(&wing, &cfg_maior);
        let x_np_maior = neutral_point_m(
            cfg_maior.wing.le_root_x_m, mac, wing.aspect_ratio, &emp_maior, wing.area_m2,
            cfg_maior.fuselage.cabin_width_m, cfg_maior.fuselage.length_m, cfg_maior.stability.fuselage_kf,
        );

        println!("x_np base (v_h={:.3}) = {x_np_base:.4}m | x_np maior (v_h={:.3}) = {x_np_maior:.4}m",
            cfg.empennage.v_h, cfg_maior.empennage.v_h);
        assert!(x_np_maior > x_np_base,
            "NP deveria recuar (aumentar x) quando v_h aumenta: base={x_np_base:.4}m \
             maior={x_np_maior:.4}m");
    }

    /// Hand-check (mesmos valores do baseline real, ver task-4.1-report.md):
    ///   x_np=3.803m, MAC=1.2463m
    ///   sm_min=0.05 → x_cg_aft = 3.803 − 0.05·1.2463 = 3.803 − 0.062315 = 3.740685 m
    #[test]
    fn cg_limit_aft_hand_check() {
        let aft = cg_limit_aft_m(3.803, 0.05, 1.2463);
        println!("cg_limit_aft_m = {aft:.6}");
        assert!((aft - 3.740685).abs() < 1e-4, "aft = {aft:.6} (esperado ≈3.740685)");
    }

    /// Propriedade: aumentar `sm_min` torna o critério de estabilidade
    /// mínima mais exigente — o limite TRASEIRO deve avançar (mover para a
    /// frente, x menor), reduzindo o envelope admissível por trás.
    #[test]
    fn aumentar_sm_min_move_limite_traseiro_para_frente() {
        let aft_base  = cg_limit_aft_m(3.803, 0.05, 1.2463);
        let aft_maior = cg_limit_aft_m(3.803, 0.10, 1.2463);
        assert!(aft_maior < aft_base,
            "aft com sm_min maior ({aft_maior:.4}) deveria ser MENOR (mais à frente) que \
             aft base ({aft_base:.4})");
    }

    // ─── WeightBalanceOutput::apply_trim (task trim-authority) ──────────────
    //
    // O limite dianteiro FÍSICO (flare/rotação) é calculado pelo
    // `TrimAuthorityAgent` — testado a fundo em `agents::trim_authority::
    // tests`. Os testes abaixo cobrem só a COMPOSIÇÃO em
    // `WeightBalanceOutput::apply_trim`: o AND entre o critério traseiro
    // (já calculado por `WeightBalanceAgent::run`) e o dianteiro (vindo de
    // um `TrimSpec` sintético), e a agregação do pior caso em
    // `spec.cg_limit_fwd_pct_mac`.

    /// Constrói um `WeightBalanceOutput` sintético mínimo (2 cenários) para
    /// testar `apply_trim` isoladamente, sem depender do pipeline completo
    /// de agentes.
    fn wb_sintetico_para_apply_trim() -> WeightBalanceOutput {
        let mac = 1.25;
        let mac_le = 2.90;
        let x_np = 3.80;
        let sm_min = 0.05;
        let x_cg_limit_aft = cg_limit_aft_m(x_np, sm_min, mac);

        let make_scenario = |name: &'static str, cg_pct: f64| {
            let x_cg = mac_le + cg_pct / 100.0 * mac;
            ScenarioResult {
                name,
                total_mass_kg: 1_200.0,
                x_cg_m: x_cg,
                cg_pct_mac: cg_pct,
                static_margin: static_margin(x_np, x_cg, mac),
                stable: true,
                inside_envelope: x_cg <= x_cg_limit_aft, // mesmo critério parcial de `run()`
            }
        };

        WeightBalanceOutput {
            spec: crate::models::specs::WeightSpec {
                oew_kg: 900.0,
                mtow_kg: 1_400.0,
                payload_kg: 400.0,
                fuel_mass_kg: 150.0,
                cg_mac_fwd_pct: 10.0,
                cg_mac_aft_pct: 30.0,
                static_margin_pct: 12.0,
                cg_limit_fwd_pct_mac: f64::NAN,
                cg_limit_aft_pct_mac: cg_pct_mac(x_cg_limit_aft, mac_le, mac),
                structural_masses: crate::models::specs::StructuralMassesSpec {
                    asa_kg: 148.0,
                    fuselagem_kg: 110.6,
                    emp_h_kg: 14.0,
                    emp_v_kg: 5.6,
                    trem_principal_kg: 110.5,
                    trem_nariz_kg: 22.0,
                    tanques_kg: 22.4,
                    composite_factor_wing: 0.90,
                    composite_factor_tail: 0.80,
                    composite_factor_fuselage: 0.95,
                    composite_factor_gear: 1.00,
                    composite_factor_fuel_system: 1.05,
                },
            },
            oew_kg: 900.0,
            chord_root_m: 1.6,
            chord_tip_m: 0.7,
            mac_m: mac,
            mac_le_x_m: mac_le,
            x_np_m: x_np,
            scenarios: vec![
                make_scenario("Leve (dianteiro)", 10.0),
                make_scenario("Pesado (traseiro)", 25.0),
            ],
        }
    }

    /// `TrimSpec` sintético — desde o fix de revisão (cancelamento de peso
    /// na rotação), `flare_limit_pct_mac`/`rotation_limit_pct_mac` são
    /// NÚMEROS ÚNICOS (não variam por cenário) — `rotation_margin_per_scenario`
    /// é só um diagnóstico informativo, não consumido por `apply_trim`.
    fn trim_sintetico(flare_pct: f64, rotation_pct: f64) -> crate::models::specs::TrimSpec {
        use crate::models::specs::{ScenarioTrimLimit, TrimSensitivity, TrimSpec};
        TrimSpec {
            flare_limit_pct_mac: flare_pct,
            rotation_limit_pct_mac: rotation_pct,
            rotation_margin_per_scenario: vec![
                ScenarioTrimLimit { scenario: "Leve (dianteiro)".to_string(), rotation_authority_margin_pct: -10.0 },
                ScenarioTrimLimit { scenario: "Pesado (traseiro)".to_string(), rotation_authority_margin_pct: 5.0 },
            ],
            governing: if rotation_pct >= flare_pct { "rotacao".to_string() } else { "flare".to_string() },
            cl_h_available: -0.7,
            sensitivity: TrimSensitivity {
                cl_h_max_down_minus: 0.80,
                flare_limit_pct_mac_minus: flare_pct + 3.0,
                cl_h_max_down_plus: 0.90,
                flare_limit_pct_mac_plus: flare_pct - 3.0,
                elevator_deflection_max_deg_minus: 23.0,
                flare_limit_pct_mac_deflection_minus: flare_pct + 2.0,
                elevator_deflection_max_deg_plus: 27.0,
                flare_limit_pct_mac_deflection_plus: flare_pct - 2.0,
            },
            cm_ac: -0.008,
            cm_flap_delta: -0.30,
            cl_h_max_down: 0.85,
            cl_h_max_down_calc: 0.85,
            tau_elevator: 0.6,
            capped_by_stall: false,
            trim_margin: 0.10,
            cl_ground_rotation: 0.5,
            to_flap_cm_fraction: 0.5,
            cl_h_trim_cruise: 0.04,
            cd_trim: 5.0e-5,
            cg_reference_scenario: MID_MISSION_SCENARIO_NAME.to_string(),
            cg_reference_pct_mac: 35.0,
        }
    }

    /// Limite dianteiro efetivo (rotação, 15% MAC) fica À FRENTE do
    /// cenário "Leve" (CG a 10% MAC) → `inside_envelope` deve virar `false`
    /// depois de `apply_trim` (estava `true`, critério parcial traseiro
    /// apenas) — e ATRÁS do cenário "Pesado" (CG a 25% MAC) → continua
    /// `true`. O MESMO limite (15%) se aplica aos dois (número único).
    #[test]
    fn apply_trim_marca_fora_quando_cg_fica_a_frente_do_limite_de_rotacao() {
        let mut wb = wb_sintetico_para_apply_trim();
        assert!(wb.scenarios[0].inside_envelope, "pré-condição: parcial (só traseiro) é true");
        assert!(wb.scenarios[1].inside_envelope, "pré-condição: parcial (só traseiro) é true");

        let trim = trim_sintetico(5.0, 15.0);
        wb.apply_trim(&trim);

        assert!(!wb.scenarios[0].inside_envelope,
            "cenário 'Leve' (CG=10%) deveria ficar FORA — limite de rotação (15%) está à frente");
        assert!(wb.scenarios[1].inside_envelope,
            "cenário 'Pesado' (CG=25%) deveria continuar DENTRO — limite de rotação (12%) < CG");
    }

    /// `spec.cg_limit_fwd_pct_mac` finalizado deve ser `max(flare, rotação)`
    /// — aqui, rotação (15%) > flare (5%).
    #[test]
    fn apply_trim_cg_limit_fwd_pct_mac_e_o_maior_entre_flare_e_rotacao() {
        let mut wb = wb_sintetico_para_apply_trim();
        assert!(wb.spec.cg_limit_fwd_pct_mac.is_nan(), "placeholder antes de apply_trim deveria ser NaN");

        let trim = trim_sintetico(5.0, 15.0);
        wb.apply_trim(&trim);

        assert!((wb.spec.cg_limit_fwd_pct_mac - 15.0).abs() < 1e-9,
            "cg_limit_fwd_pct_mac = {} (esperado 15.0, max(flare=5.0, rotação=15.0))",
            wb.spec.cg_limit_fwd_pct_mac);
    }

    /// Quando a FLARE é mais restritiva que a rotação, `apply_trim` usa a
    /// flare (constante, `max(flare, rotação)`), não a rotação — mesmo
    /// limite único aplicado a TODOS os cenários (config
    /// flare-governada continua correta, não hardcoded para "rotação
    /// sempre governa").
    #[test]
    fn apply_trim_usa_flare_quando_flare_e_mais_restritiva_que_rotacao() {
        let mut wb = wb_sintetico_para_apply_trim();
        // flare=20% > rotação (6%) — flare governa.
        let trim = trim_sintetico(20.0, 6.0);
        wb.apply_trim(&trim);

        assert!(!wb.scenarios[0].inside_envelope,
            "cenário 'Leve' (CG=10%) deveria ficar FORA — flare (20%) está à frente do CG");
        assert!(wb.scenarios[1].inside_envelope,
            "cenário 'Pesado' (CG=25%) deveria continuar DENTRO — flare (20%) < CG (25%)");
        assert!((wb.spec.cg_limit_fwd_pct_mac - 20.0).abs() < 1e-9,
            "cg_limit_fwd_pct_mac deveria ser a flare (20.0), obtido {}",
            wb.spec.cg_limit_fwd_pct_mac);
    }

    /// `apply_trim` preserva um veredito `false` já vindo do critério
    /// traseiro (AND, não OR) — CG artificialmente atrás do limite traseiro
    /// deve continuar `false` mesmo com um limite dianteiro folgado.
    #[test]
    fn apply_trim_preserva_falso_do_criterio_traseiro() {
        let mut wb = wb_sintetico_para_apply_trim();
        wb.scenarios[1].inside_envelope = false; // simula violação traseira

        let trim = trim_sintetico(1.0, 1.0); // dianteiro folgadíssimo
        wb.apply_trim(&trim);

        assert!(!wb.scenarios[1].inside_envelope,
            "AND com critério traseiro false deveria permanecer false mesmo com dianteiro \
             folgado");
    }

    /// Fix de revisão (envelope vazio, FIX4): quando o limite dianteiro
    /// (rotação, aqui 40%) fica À FRENTE do limite traseiro
    /// (`cg_limit_aft_pct_mac`, calculado a partir de x_np=3.80/sm_min=0.05
    /// na fixture — ≈32%), NENHUM cenário pode ficar dentro do envelope,
    /// mesmo um cenário com CG bem atrás (aqui, 25%, ainda à FRENTE de
    /// 40%) — `apply_trim` não precisa de lógica especial para isso, o AND
    /// simples já produz `false` para todos.
    #[test]
    fn apply_trim_com_limite_dianteiro_maior_que_traseiro_marca_todos_fora() {
        let mut wb = wb_sintetico_para_apply_trim();
        let aft_pct = wb.spec.cg_limit_aft_pct_mac;
        let trim = trim_sintetico(5.0, aft_pct + 5.0); // dianteiro além do traseiro
        wb.apply_trim(&trim);

        assert!(wb.spec.cg_limit_fwd_pct_mac > wb.spec.cg_limit_aft_pct_mac,
            "pré-condição: fwd ({}) deveria ficar à frente (maior) do aft ({})",
            wb.spec.cg_limit_fwd_pct_mac, wb.spec.cg_limit_aft_pct_mac);
        assert!(wb.scenarios.iter().all(|s| !s.inside_envelope),
            "com envelope vazio (fwd > aft), NENHUM cenário deveria ficar dentro — obtido: {:?}",
            wb.scenarios.iter().map(|s| (s.name, s.inside_envelope)).collect::<Vec<_>>());
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
