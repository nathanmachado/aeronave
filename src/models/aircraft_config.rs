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
    /// Limites de estabilidade/controle que definem o envelope de CG
    /// admissível (Task 4.4 + task trim-authority) — o limite traseiro vem
    /// de `weight_balance::cg_limit_aft_m`; o dianteiro, de
    /// `agents::trim_authority::TrimAuthorityAgent`.
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
    /// Parâmetros do modelo de massas estruturais (ciclo 3, spec
    /// 2026-08-06-oew-parametrico-design.md) — fatores de composto e
    /// geometria auxiliar consumidos por `agents::mass_model`.
    pub mass_model: MassModelCfg,
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
    /// Cm_ac do perfil em torno do seu próprio centro aerodinâmico,
    /// configuração LIMPA (adimensional, tipicamente negativo para perfis
    /// com câmber positivo) — quase nulo para a série NACA 230 (Abbott &
    /// von Doenhoff, "Theory of Wing Sections"). Consumido por
    /// `agents::trim_authority` no balanço de momentos de flare/rotação
    /// (task trim-authority) — substitui, junto com `cm_flap_delta`, o
    /// antigo proxy `stability.sm_max` (removido) como fonte do limite
    /// DIANTEIRO físico do envelope de CG.
    pub cm_ac: f64,
    /// ΔCm de flap de POUSO (incremento nariz-para-baixo do momento de
    /// arfagem com o flap totalmente estendido, adimensional, negativo) —
    /// semi-empírico (Raymer cap. 16, faixa típica −0,20 a −0,45).
    /// Consumido por `agents::trim_authority` (flare usa o valor cheio;
    /// rotação usa `stability.to_flap_fraction · cm_flap_delta`, flap de
    /// decolagem parcial).
    pub cm_flap_delta: f64,
    /// ΔCD₀ do flap CHEIO (configuração de POUSO, adimensional, positivo) —
    /// incremento de arrasto parasita por deployment do flap (fenda/batente,
    /// separação de escoamento sobre a superfície defletida), semi-empírico
    /// (Raymer cap. 12 / Hoerner "Fluid-Dynamic Drag"), faixa típica de flap
    /// slotted moderado (ciclo 8, task 1 — fecha a lacuna declarada desde o
    /// ciclo 7: "não existe modelo de flap na polar deste crate").
    ///
    /// Consumido por `agents::performance::excess_power_kw` (parâmetro
    /// `cd0_extra`), via `WingSpec::cd0_flap_to_extra` = `to_flap_fraction ·
    /// cd0_flap_delta` — a MESMA fração parcial que já governa `cl_max_to`
    /// (`StabilityCfg::to_flap_fraction`) e `cm_flap_delta` na rotação, agora
    /// também o arrasto: um único dial de deployment para os três efeitos do
    /// flap PARCIAL de decolagem.
    ///
    /// HISTÓRICO `old→new` (ciclo 8 → ciclo 12). Até o ciclo 11, o delta
    /// CHEIO (pouso) NÃO tinha um call site que o consumisse — auditoria de
    /// `agents::performance` (ciclo 8, task 1): a rolagem de solo de
    /// decolagem era o método ENERGÉTICO de Raymer (sem termo de arrasto,
    /// por construção); a rolagem de pouso era dominada por frenagem
    /// (`S_G = V_ref²/(2gμ)`, sem polar); e a aproximação de pouso
    /// (`landing_distance_50ft_m`) usava um ângulo FIXO
    /// (`[performance].approach_angle_deg`), não uma razão L/D — nenhuma
    /// delas tinha onde receber um incremento de CD0. Essa conclusão valia
    /// para o modelo daquele momento.
    ///
    /// **Ela morre no ciclo 12** (spec `2026-08-15-ciclo12-solo-honesto`
    /// §5.3a): a rolagem de pouso passa a integrar a equação de movimento
    /// (`agents::performance::landing_ground_roll_m`), consumindo a polar
    /// completa. O delta CHEIO ganha o seu primeiro consumidor real —
    /// `WingSpec::cd0_flap_ldg_extra` = `cfg.wing.cd0_flap_delta` (SEM
    /// fração, ao contrário de `cd0_flap_to_extra`: a rolagem de pouso é
    /// modelada com o flap de pouso TOTALMENTE deflexionado do início ao
    /// fim da frenagem, spec §5.1) — via `agents::performance::
    /// cd_gear_extended`, a mesma fonte única de CD de solo que a rolagem de
    /// decolagem (ciclo 12, task 2) e o balanço de rotação (ciclo 12, task
    /// 4) também consomem.
    ///
    /// Faixa validada (0.005, 0.05) — `models::config::validate_aircraft`.
    pub cd0_flap_delta: f64,
    /// Offset vertical entre o CENTRO DE ARRASTO (resultante de arrasto da
    /// aeronave) e o CG (m, positivo = centro de arrasto ACIMA do CG) —
    /// ciclo 12, spec §6.2. Default `0.0` (braço CHEIO `h_cg`, caso
    /// CONSERVADOR: `h_D > 0` encolheria o braço líquido `h_cg − h_D` do
    /// termo de arrasto de solo). Faixa plausível numa célula convencional
    /// (asa baixa/média, motor no nariz): 0–0,10 m — o centro de arrasto
    /// fica alguns centímetros ACIMA do CG. Faixa validada `[0.0, 0.30]`
    /// (`models::config::validate_aircraft`).
    ///
    /// Consumido por `agents::trim_authority::rotation_available_moment_nm`
    /// (braço líquido `h_cg − z_drag_above_cg_m` do termo de arrasto de
    /// solo no balanço de rotação). A docstring de `cm_thrust_cruise` já
    /// registrava a necessidade deste campo desde o ciclo 10 (aproximação
    /// `z_D = 0` do arrasto de CRUZEIRO) — este ciclo cria o campo, mas
    /// **NÃO** o pluga em `cm_thrust_cruise`: fazê-lo mudaria `CL_h_trim`,
    /// `cd_trim`, cruzeiro/alcance/autonomia — cascata fora de escopo,
    /// registrada no backlog como consumidor pendente.
    pub z_drag_above_cg_m: f64,
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
    // NOTA (ciclo 3, oew-parametrico): `mass_per_area_h_kg_m2`/
    // `mass_per_area_v_kg_m2` foram REMOVIDOS — a massa das empenagens
    // agora é COMPUTADA por `agents::mass_model` (Raymer cap. 15.2,
    // `htail_mass_raymer_kg`/`vtail_mass_raymer_kg` × `[mass_model].
    // composite_factor_tail`), que já responde a S_h/S_v, N_z, q e
    // alongamento/afilamento em vez de uma densidade de área calibrada à
    // mão. Erro de migração claro se ainda presentes no TOML — ver
    // `models::config::check_mass_per_area_migration`.
    /// Fator de área para o CD0 da empenagem (adimensional) — task
    /// refino-ciclo2 (1b), substitui o antigo `[empennage].cd0` fixo (erro
    /// de migração se presente). `agents::aerodynamics::cd0_total` recebe
    /// `cd0_area_factor·(S_h+S_v)/S_w` como `cd0_empennage` — mesma lógica
    /// de "component build-up" de `cd0_wing`/`cd0_fuselage` (Raymer cap.
    /// 12), mas escalado pela área MOLHADA real da empenagem em vez de um
    /// valor fixo desacoplado da geometria. Fisicamente ≈ `2·Cf·FF`
    /// (coeficiente de atrito de placa plana turbulenta × fator de forma
    /// do perfil da empenagem) referenciado à área da ASA (não à área
    /// molhada da própria empenagem) — daí o fator ~0,014, bem acima de um
    /// `2·Cf·FF` típico (~0,006–0,010) referenciado à própria área.
    /// Faixa 0,008–0,025. CALIBRADO a partir do `cd0=0,0046` fixo do
    /// baseline E6: `0,0046·S_w/(S_h+S_v) ≈ 0,014366` (ver
    /// task-1-report.md).
    pub cd0_area_factor: f64,
    /// Eficiência de Oswald da empenagem horizontal (adimensional) — task
    /// refino-ciclo2 (Task 4): usada em `agents::trim_authority::
    /// cd_trim_cruise` para o arrasto INDUZIDO da própria empenagem ao gerar
    /// `cl_h_trim` (download/upload de trim em cruzeiro,
    /// `ΔCD_trim = CL_h_trim²/(π·ar_h·e_h)·(S_h/S_w)`, mesma forma de
    /// `agents::aerodynamics::cd_induced` aplicada à empenagem). Distinto de
    /// `oswald_efficiency` (asa, calculado por `agents::aerodynamics::
    /// oswald_efficiency` a partir do AR — a EH não tem esse cálculo
    /// dedicado, e seu Oswald tende a ser um pouco mais baixo que o da asa
    /// por causa da razão de aspecto menor e da interferência da fuselagem/
    /// esteira da asa). Faixa 0,5–0,95 (Raymer/Gudmundsson, superfícies de
    /// cauda de baixo alongamento).
    pub e_h: f64,
    /// Espessura relativa (t/c) dos perfis da empenagem (ciclo 4) — perfis
    /// simétricos finos típicos de empenagem (NACA 0009–0012). Consumido por
    /// `agents::mass_model` (expoentes de (100·t/c): EH −0.12, EV −0.49 —
    /// empenagem mais fina é mais PESADA). Antes do ciclo 4 usava-se o t/c da
    /// ASA como aproximação (subestimava EV ~21%).
    pub thickness_ratio: f64,
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
    /// Offset vertical FIXO da célula entre o eixo da hélice e o CG (m) —
    /// ciclo 5. A altura do eixo ao solo DERIVA do trem:
    /// shaft_height = gear.h_cg_ground_m + prop_axis_above_cg_m — encurtar o
    /// trem consome folga de hélice 1:1 automaticamente (a campanha E9
    /// encurtou h_cg 13 cm e o datum absoluto antigo manteve a folga parada).
    /// Valor do baseline derivado da geometria atual (1,25 − 1,05) — validar
    /// no CAD (Fase 3). Pode ser negativo (eixo abaixo do CG).
    pub prop_axis_above_cg_m: f64,
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
    /// Posição longitudinal do PLANO DA HÉLICE (m do datum no nariz —
    /// ciclo 9, transferência de atitude do #25). Hélice TRATORA: no
    /// baseline REAL o spinner fica à frente do CG do motor
    /// (`prop_plane_x_m` 0,20 m < `[arms].engine_cg_m` 0,65 m) — ordem
    /// física esperada para esta geometria, mas NÃO imposta por validação
    /// nenhuma (só a composta abaixo é checada; a fixture sintética de
    /// teste, `test_fixtures::config_teste()`, deliberadamente NÃO segue
    /// essa ordem — ver comentário no valor da fixture). Único consumidor:
    /// `PropellerSpec::fill_critical_clearance` — o fator de amplificação
    /// do pivô sobre o trem principal, `(gear.x_main_m −
    /// prop_plane_x_m)/(gear.x_main_m − gear.x_nose_m)`, que substitui a
    /// translação vertical 1:1 (achado de review, ciclo 8; corrigido no
    /// ciclo 9 — ver docstring do campo
    /// `PropellerSpec::prop_clearance_critical_m`). Validação COMPOSTA
    /// (`models::config::validate_aircraft_config`): deve ficar
    /// ESTRITAMENTE à frente do trem de nariz (`prop_plane_x_m <
    /// gear.x_nose_m`) — a hélice tratora não pode ficar atrás do próprio
    /// trem que ela sobrevoa. Valor do baseline (0,20 m) é uma ESTIMATIVA
    /// de geometria — validar no CAD (Fase 3).
    pub prop_plane_x_m: f64,
    /// Figura de mérito da hélice em J=0 (ciclo 13, spec §3.1) — fator
    /// empírico de McCormick sobre a tração estática ideal de disco atuador.
    /// MIGRADO de `[performance].static_thrust_factor`: mudou de lugar porque
    /// é propriedade da HÉLICE, não da política de performance, e porque
    /// deixou de ser um multiplicador plano para virar a âncora J=0 de uma
    /// curva (`FigureOfMerit`).
    pub fom_static: f64,
    /// Figura de mérito na razão de avanço de projeto da hélice (ciclo 13,
    /// spec §3.2). Retro-derivada uma vez do polinômio JavaProp no ponto de
    /// cruzeiro do baseline E12 — ver a premissa calibrada declarada em
    /// `agents::propulsion::FigureOfMerit`.
    pub fom_design: f64,
    /// Razão de avanço de projeto da hélice (ciclo 13, spec §3.2). Acima
    /// dela `FigureOfMerit::at` satura.
    pub j_design: f64,
    /// Tolerância epistêmica declarada sobre `fom_static` (%) — ciclo 16.
    ///
    /// NÃO é medição de hélice. É a declaração de quanto o projeto confia
    /// numa entrada que NUNCA foi calibrada: `fom_design` e `j_design` foram
    /// retro-derivados por ponto fixo até resíduo 0,000e0, `fom_static` é um
    /// fator de McCormick herdado com dois algarismos significativos.
    /// Obrigatório de propósito — ver spec do ciclo 16, §5.1.
    pub fom_static_tol_pct: f64,
}

/// Banda de incerteza EFETIVA de `fom_static` (ciclo 16, spec §5.1).
#[derive(Debug, Clone, PartialEq)]
pub struct Banda {
    pub nominal: f64,
    pub lo: f64,
    pub hi: f64,
    pub lo_declarado: f64,
    pub hi_declarado: f64,
    pub truncada: bool,
    pub motivo_truncagem: Option<String>,
}

impl PropellerCfg {
    /// Figura de mérito desta hélice a partir das três âncoras do TOML —
    /// fonte única de verdade da curva `FoM(J)` (ciclo 13, spec §3). Todo
    /// consumidor de tração chama isto em vez de ler os campos soltos.
    pub fn figure_of_merit(&self) -> crate::agents::propulsion::FigureOfMerit {
        crate::agents::propulsion::FigureOfMerit {
            fom_static: self.fom_static,
            fom_design: self.fom_design,
            j_design:   self.j_design,
        }
    }

    /// Banda efetiva. O topo é truncado em `fom_design` (acima dele FoM(J)
    /// seria DECRESCENTE em J) e em 1,0 (teto de quantidade de movimento).
    /// O topo efetivo é, portanto, a maior tração estática que se pode alegar
    /// sem contradizer a única âncora que FOI calibrada.
    pub fn banda(&self) -> Banda {
        let lo_declarado = self.fom_static * (1.0 - self.fom_static_tol_pct / 100.0);
        let hi_declarado = self.fom_static * (1.0 + self.fom_static_tol_pct / 100.0);
        let mut hi = hi_declarado;
        // Razões ACUMULADAS, nunca sobrescritas. A spec §5.1 promete que a
        // truncagem é publicada com a razão; um `motivo = Some(...)` que
        // sobrescreve o anterior descarta metade da explicação exatamente no
        // formato que a promessa proíbe.
        let mut motivos: Vec<String> = Vec::new();
        if self.fom_design < hi {
            hi = self.fom_design;
            motivos.push(format!(
                "topo truncado em propeller.fom_design ({}): acima dele FoM(J) seria \
                 DECRESCENTE em J", self.fom_design));
        }
        // RAMO INATINGÍVEL HOJE, mantido como defesa em profundidade. Prova:
        // `validate_aircraft_config` rejeita `fom_design > 1.0` (config.rs:858-868).
        // Logo, ou o primeiro ramo truncou e `hi = fom_design ≤ 1,0`,
        // ou não truncou e `hi = hi_declarado ≤ fom_design ≤ 1,0`. Nos dois casos
        // `hi ≤ 1,0` e este ramo não dispara.
        //
        // A guarda de monotonicidade (`fom_design >= fom_static`) NÃO participa
        // desta prova. Se alguém um dia relaxar o teto de `fom_design`, este ramo
        // passa a valer — e acumula a razão em vez de apagar a anterior.
        if hi > 1.0 {
            hi = 1.0;
            motivos.push("topo truncado em 1,0 — teto de quantidade de movimento".to_string());
        }
        Banda { nominal: self.fom_static, lo: lo_declarado, hi,
                lo_declarado, hi_declarado, truncada: !motivos.is_empty(),
                motivo_truncagem: if motivos.is_empty() { None }
                                  else { Some(motivos.join(" ; ")) } }
    }
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
    /// Altura do CG ao solo (m) — CONTRATO (ciclo 10, task 1, deflexão
    /// estática no #25): esta é a altura da aeronave CARREGADA, em atitude
    /// ESTÁTICA de solo — trem PARCIALMENTE comprimido pelo peso da
    /// aeronave (mains e nariz ambos em compressão típica de amortecedor
    /// oleo-pneumático, não trem "totalmente estendido" sem carga). É por
    /// isso que a checagem #25 (`specs::PropellerSpec::
    /// fill_critical_clearance`, condição CRÍTICA de CS 23.925) NÃO soma
    /// termo aditivo nenhum para os MAINS: eles já estão na deflexão
    /// estática típica embutida neste valor, tanto na condição normal
    /// quanto na condição crítica (só o trem CRÍTICO — o de NARIZ, hélice
    /// TRATORA — colapsa até o batente nessa condição; CS 23.925 pela
    /// letra pede o trem crítico no limite com os DEMAIS em deflexão
    /// estática, não os três simultaneamente no batente). Pelo mesmo
    /// motivo, o trem de NARIZ na condição crítica só percorre o curso
    /// RESTANTE até o batente — `nose_oleo_stroke_mm × (1 −
    /// [gear].static_sag_fraction)`, não o curso TOTAL — porque a fração
    /// `static_sag_fraction` do curso já foi consumida pela compressão
    /// estática que `h_cg_ground_m` já reflete. Ver docstring de
    /// `static_sag_fraction` abaixo e `docs/backlog.md` (item 6, RESOLVIDO
    /// ciclo 10).
    pub h_cg_ground_m: f64,
    pub x_nose_m: f64,
    pub x_main_m: f64,
    // NOTA (ciclo 3, oew-parametrico): `mass_main_leg_kg` foi REMOVIDO — a
    // massa TOTAL do trem principal agora é COMPUTADA por
    // `agents::mass_model::main_gear_mass_raymer_kg` (× `[mass_model].
    // composite_factor_gear`), e a massa de UMA perna usada no
    // dimensionamento do atuador de retração é essa total ÷ 2 (ver
    // `agents::landing_gear::LandingGearAgent::run`). Erro de migração
    // claro se ainda presente no TOML — ver
    // `models::config::check_mass_main_leg_migration`.
    // NOTA (revisão final, oew-parametrico): `mass_nose_kg` também foi
    // REMOVIDO, mesmo tratamento de `mass_main_leg_kg` acima — a massa do
    // trem de nariz que entra no peso vazio é COMPUTADA por
    // `agents::mass_model::nose_gear_mass_raymer_kg` (× `[mass_model].
    // composite_factor_gear`); não havia código de produção lendo o campo,
    // só a validação de `require_positive` e um TOML baseline que já
    // divergia do valor computado. Erro de migração claro se ainda presente
    // no TOML — ver `models::config::check_mass_nose_migration`.
    pub retraction_time_s: f64,
    /// Massa dos atuadores elétricos + portas do trem (kg) — soma ao peso
    /// total do sistema junto com as massas das pernas de `[masses]`.
    pub actuators_doors_mass_kg: f64,
    /// Ângulo MÍNIMO de tipback (Task 2, refino-ciclo2) — Raymer, "Aircraft
    /// Design", cap. 11: `θ = atan((x_main − x_cg_aft)/h_cg)` medido do
    /// trem principal ao CG mais TRASEIRO real (`agents::landing_gear::
    /// tipback_angle_deg`) precisa ser >= este piso para a aeronave não
    /// tombar sobre a cauda em solo/carregamento traseiro. Típico 15°.
    /// Faixa validada (8, 25).
    pub tipback_min_deg: f64,
    /// Atitude de rotação na decolagem (ângulo de picada, nariz para cima,
    /// graus) — usada como piso na checagem de folga de tail-strike
    /// (`agents::landing_gear::tail_strike_margin_deg`). Típico 11°. Faixa
    /// validada (5, 18).
    pub rotation_attitude_deg: f64,
    /// Posição longitudinal do ponto mais baixo do cone de cauda (m do
    /// nariz) — geometria SIMPLIFICADA para a checagem de tail-strike (ver
    /// docstring de `agents::landing_gear::tail_strike_margin_deg`). Deve
    /// ser maior que `x_main_m` (o cone fica atrás do trem principal).
    /// Faixa validada (3.0, 12.0).
    pub tail_cone_x_m: f64,
    /// Altura do ponto mais baixo do cone de cauda ao solo, em atitude
    /// ESTÁTICA (aeronave CARREGADA, em deflexão estática — mesmo contrato
    /// de `h_cg_ground_m` acima, não trem estendido sem carga, m) —
    /// aproximação:
    /// tratada como a altura já disponível ao rotacionar (não subtrai raio
    /// de pneu/geometria do trem, ver docstring do agente). Faixa validada
    /// (0.3, 2.5).
    pub tail_cone_height_m: f64,
    /// Deflexão TOTAL do pneu de nariz (m) na condição crítica de CS 23.925
    /// — batente do amortecedor de nariz TOTALMENTE COMPRIMIDO + pneu
    /// MURCHO/estourado. Típica de um pneu 5.00-5 (deflexão de projeto
    /// total, não só a seção de borracha) — consumida por
    /// `specs::PropellerSpec::fill_critical_clearance` junto com
    /// `GearSpec::nose_oleo_stroke_mm` (hélice TRATORA: é o trem de NARIZ
    /// que governa a folga crítica, não o principal — a hélice fica à
    /// frente, sobre o eixo de nariz). Ciclo 8, task 2. Faixa validada
    /// (0.03, 0.15).
    pub tire_deflation_delta_m: f64,
    /// Fração do curso do amortecedor de NARIZ já consumida pela
    /// compressão ESTÁTICA (aeronave carregada, parada no solo) — ciclo 10,
    /// task 1 (CS 23.925 pela letra). Compressão estática típica de um
    /// amortecedor oleo-pneumático é ~1/3 do curso total (baseline 0,33).
    /// Consumida por `specs::PropellerSpec::fill_critical_clearance`: como
    /// `[gear].h_cg_ground_m` já mede a aeronave em deflexão estática (ver
    /// docstring desse campo), o trem de NARIZ na condição CRÍTICA de CS
    /// 23.925 (batente do amortecedor) só percorre o curso RESTANTE —
    /// `nose_oleo_stroke_mm × (1 − static_sag_fraction)` — não o curso
    /// TOTAL. ANTES desta task (ciclo 9), a fórmula usava o curso TOTAL do
    /// nariz, contando DUAS VEZES a compressão estática: uma vez
    /// implicitamente (embutida em `h_cg_ground_m`, que já é a altura
    /// CARREGADA) e outra vez explicitamente (o curso total do batente,
    /// como se o amortecedor partisse ESTENDIDO em vez de já parcialmente
    /// comprimido) — isso SUPERESTIMAVA `Δ_prop` e por isso SUBESTIMAVA
    /// (deixava mais NEGATIVA) a folga crítica. A correção honesta
    /// (RESOLVIDO nesta task) troca o curso total pelo curso RESTANTE, o
    /// que reduz `Δ_prop` e AUMENTA a folga crítica calculada — mais
    /// otimista, portanto ANTI-conservadora frente ao número antigo
    /// (−0,0642 m → −0,0025 m no baseline E10), mas fisicamente CORRETA
    /// pela letra da norma: não há dupla contagem da mesma compressão. Ver
    /// `docs/backlog.md` (item 6, RESOLVIDO) e a docstring de
    /// `PropellerSpec::prop_clearance_critical_m` para a física completa
    /// old→new. Faixa validada (0.15, 0.55) — compressão estática
    /// plausível de um amortecedor oleo-pneumático (nunca quase zero,
    /// nunca quase o curso inteiro).
    pub static_sag_fraction: f64,
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

/// Limites de estabilidade/controle (Task 4.4 + task trim-authority) que
/// definem o envelope de CG ADMISSÍVEL da aeronave — em contraste com
/// `WeightSpec::cg_mac_fwd_pct`/`cg_mac_aft_pct`, que são os extremos
/// OBSERVADOS entre os cenários de carga. O envelope vem de DOIS critérios
/// físicos INDEPENDENTES, um por extremo:
///
///   - `sm_min`: margem estática mínima aceitável — abaixo dela a aeronave
///     fica perigosamente próxima da instabilidade estática longitudinal.
///     Define o limite TRASEIRO do CG (CG mais atrás permitido): SM cai
///     quando o CG recua, então SM = sm_min é o pior caso traseiro (ver
///     `weight_balance::cg_limit_aft_m`).
///   - O limite DIANTEIRO do CG (CG mais à frente permitido) NÃO usa mais
///     um proxy de margem estática (`sm_max`, REMOVIDO nesta task — config
///     antiga com `sm_max` produz um erro de migração claro em
///     `models::config::parse_aircraft`). Em vez disso é calculado
///     FISICAMENTE pelo `TrimAuthorityAgent` a partir da autoridade de
///     profundor disponível (`cl_h_max_down`/`trim_margin` abaixo) nas duas
///     manobras críticas nariz-para-cima (flare no pouso + rotação na
///     decolagem) — ver `agents::trim_authority` e `models::specs::TrimSpec`.
///     `cm_ac`/`cm_flap_delta` (agora em `[wing]`) completam o balanço de
///     momentos usado por esse cálculo.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StabilityCfg {
    /// Margem estática mínima admissível — define o limite TRASEIRO do CG.
    pub sm_min: f64,
    /// Teto de |CL| de DOWNLOAD da empenagem horizontal por STALL da
    /// própria empenagem — task refino-ciclo2 (1a), substitui o antigo
    /// parâmetro livre `[stability].cl_h_max_down` (palpite semi-empírico
    /// sem base geométrica direta; erro de migração claro se presente, ver
    /// `models::config::parse_aircraft`). A autoridade bruta disponível
    /// agora é CALCULADA por geometria DATCOM/Nelson
    /// (`agents::trim_authority::cl_h_max_down_calc`, a partir de
    /// `[control_surfaces].elevator_chord_frac`/`elevator_deflection_max_deg`
    /// e `EmpennageSpec::ar_h`) — este campo só entra como TETO físico
    /// (download de profundor acima do CL_max da própria empenagem não é
    /// alcançável, a superfície estola antes). Faixa típica 0,8–1,4
    /// (Gudmundsson/Roskam, CL_max de superfícies de cauda finas).
    pub cl_h_stall_limit: f64,
    /// Fração da autoridade de profundor RESERVADA como margem (efeito solo
    /// na aproximação/flare + margem de certificação) — não consumida no
    /// balanço de momentos nominal, reduz o `cl_h_max_down` calculado
    /// efetivo.
    pub trim_margin: f64,
    /// CL da asa na corrida de decolagem, ANTES da rotação (ângulo de
    /// ataque de solo, trem no chão) — faixa típica 0–1. Usado no momento
    /// de sustentação nariz-para-cima em `agents::trim_authority`
    /// (`rotation_fwd_limit_m`, termo `L_g`).
    pub cl_ground_rotation: f64,
    /// Fração de DEPLOYMENT do flap no setting de DECOLAGEM (tipicamente
    /// parcial, menos extensão que o flap de pouso usado na flare) —
    /// faixa 0–1. **PAPEL DUPLO** (ciclo 7, task 1 — antes chamado
    /// `to_flap_cm_fraction`, ver `models::config::
    /// check_to_flap_cm_fraction_migration`): a MESMA fração governa os
    /// DOIS efeitos do flap parcial de decolagem —
    ///
    ///   - ΔCm: `Cm_TO = cm_ac + to_flap_fraction·cm_flap_delta`
    ///     (`agents::trim_authority::rotation_available_moment_nm`);
    ///   - ΔCL: `cl_max_to = cl_max_clean + to_flap_fraction·
    ///     (cl_max_flaps − cl_max_clean)` (`agents::aerodynamics::
    ///     AerodynamicsAgent::run` → `specs::WingSpec::cl_max_to`),
    ///     consumido pela Vr da rotação (`trim_authority`) e pelas
    ///     distâncias de DECOLAGEM (`agents::performance`).
    ///
    /// Antes do ciclo 7 a fração governava só o ΔCm, enquanto a Vr da
    /// rotação e as distâncias de decolagem usavam o `cl_max_flaps` de
    /// POUSO — inconsistência exposta pela campanha E10 (flap slotted):
    /// rotação pessimista (Vr 13% lenta demais, q_r −24%) e decolagem
    /// otimista, os dois lados do MESMO erro. Ninguém decola com flap de
    /// pouso.
    pub to_flap_fraction: f64,
    /// Coeficiente empírico de momento de arfagem da fuselagem (Multhopp
    /// simplificado, Raymer eq. 16.25, fig. 16.14) — usado por
    /// `weight_balance::fuselage_np_shift_mac` para corrigir o ponto neutro
    /// pela contribuição desestabilizadora da fuselagem (o escoamento sobre
    /// a fuselagem à frente do CA da asa produz um Cm_α positivo/
    /// desestabilizante, que avança o NP). Faixa típica 0.01–0.03 conforme a
    /// posição vertical da asa na fuselagem (Raymer fig. 16.14): asa baixa
    /// tende ao topo da faixa, asa alta ao fundo. Vive em `[stability]`
    /// (não em `[fuselage]`) porque é um coeficiente de estabilidade, não
    /// uma medida geométrica — a geometria em si (`length_m`/
    /// `cabin_width_m`) já vive em `[fuselage]` e é reaproveitada daqui.
    pub fuselage_kf: f64,
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
    /// Corda do profundor, fração da corda LOCAL do estabilizador
    /// horizontal — esta é, ao mesmo tempo, a razão `c_e/c` usada por
    /// `agents::trim_authority::tau_elevator` (a razão corda-do-profundor/
    /// corda-local é constante ao longo da envergadura neste modelo
    /// trapezoidal, ver `agents::control_surfaces`), então nenhum campo
    /// adicional é necessário para a eficácia de superfície do profundor.
    pub elevator_chord_frac: f64,
    /// Deflexão máxima do profundor no batente (graus) — task
    /// refino-ciclo2 (1a), usada por `agents::trim_authority::
    /// cl_h_max_down_calc` (`a_t·τ·δe_max_rad`). Faixa típica 10–35°
    /// (Gudmundsson/Roskam, batentes mecânicos de profundor em GA leve).
    pub elevator_deflection_max_deg: f64,
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
    /// `landing_distance_50ft_m`) recebem `mu_brake` genericamente; desde
    /// a revisão final do ciclo 6 este valor é de fato CONSUMIDO, em
    /// `PerformanceSpec::ldg_50ft_grass_m` — o pouso na grama, que é o
    /// caso dimensionante da premissa de pista do projeto e a grandeza
    /// gateada pela checagem #24. Antes disso o campo era só validado
    /// (`config.rs`) e nunca usado: `PerformanceSpec` modelava apenas
    /// pouso pavimentado (assimetria pré-existente — só a decolagem tinha
    /// variantes pavimentada/grama desde a Task 4.7).
    pub mu_brake_grass: f64,
    /// Coeficiente de atrito de ROLAGEM (rolagem LIVRE, sem frenagem) em
    /// pista pavimentada seca — ciclo 12, spec §4 (Raymer Tab. 17.1,
    /// faixa 0,03–0,05). Consumido por `agents::performance::
    /// takeoff_distance_m`/`takeoff_distance_50ft_m` — substitui
    /// `surface_factor` (1,00/1,15–1,20/1,25), que multiplicava a rolagem
    /// de solo INTEIRA por fora. Com `mu_roll` explícito na integração da
    /// rolagem, o antigo fator de superfície contaria a grama duas vezes
    /// (o 1,20 antigo ERA o atrito ausente que a fórmula energética não
    /// tinha) — por isso `surface_factor` SAI do caminho de decolagem, não
    /// fica ao lado de `mu_roll`.
    pub mu_roll_paved: f64,
    /// Coeficiente de atrito de ROLAGEM em grama firme de fazenda — ciclo
    /// 12, spec §4 (Gudmundsson cap. 17, faixa curta 0,05 / alta 0,10).
    pub mu_roll_grass: f64,
    // `old→new` (ciclo 13, spec §9.2): `pub static_thrust_factor: f64` foi
    // REMOVIDO daqui — MIGROU para `[propeller].fom_static`. Deixou de ser
    // um multiplicador plano aplicado só sobre a tração estática ideal (ou,
    // no ciclo 12, esticado como fator PLANO sobre toda a rolagem via
    // `thrust_ground_roll_n`, também apagada) e virou a âncora `J=0` de uma
    // figura de mérito `FoM(J)` que governa a lei de tração única em
    // qualquer velocidade (ver `agents::propulsion::FigureOfMerit`,
    // `agents::performance::thrust_available_n`). Um TOML com
    // `[performance].static_thrust_factor` é rejeitado por
    // `check_static_thrust_factor_migration` (`models::config`), não
    // silenciosamente ignorado.
    /// Tempo de rotação — do início da rotação até V_LOF, a V_LOF
    /// aproximadamente constante (s).
    pub rotation_time_s: f64,
    /// Fator de carga do FLARE de pouso (ciclo 14, spec §2.2) — substitui
    /// `flare_time_s`. O flare é um arco de recolhimento de raio
    /// `R = V_ref²/(g·(n−1))`, não uma cronometragem: com `n` maior o arco é
    /// mais fechado, consome menos altura e termina antes.
    ///
    /// Estritamente > 1,0 — em `n = 1` o raio DIVERGE (voo reto, o flare
    /// nunca termina). Teto de 2,0: fator de carga de flare acima disso não é
    /// pilotagem de pouso.
    ///
    /// `old→new` (ciclo 14, spec §2.2/§6.2): substitui `flare_time_s`
    /// (flare cronometrado, cinemática pura, `s_flare = V_ref × tempo`, sem
    /// consumir altura — REMOVIDO, erro de migração em
    /// `models::config::check_flare_time_migration`).
    pub flare_load_factor: f64,
    // `old→new` (ciclo 14, spec §2.1/§6.2): `pub approach_angle_deg: f64`
    // foi REMOVIDO daqui — o ângulo de aproximação deixou de ser config e
    // passou a ser DERIVADO da polar de pouso (`atan(CD_ref/CL_ref)` a
    // V_ref, planeio power-off — ver `agents::performance::
    // landing_air_segment`). Um TOML com `[performance].approach_angle_deg`
    // é rejeitado por `check_approach_angle_migration` (`models::config`),
    // não silenciosamente ignorado.
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

/// Parâmetros do modelo de massas estruturais (ciclo 3, spec
/// 2026-08-06-oew-parametrico-design.md) — fatores de composto (Raymer
/// Tab. 15.4) e geometria auxiliar consumidos por `agents::mass_model`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MassModelCfg {
    pub composite_factor_wing: f64,
    pub composite_factor_tail: f64,
    pub composite_factor_fuselage: f64,
    pub composite_factor_gear: f64,
    pub composite_factor_fuel_system: f64,
    /// Diâmetro equivalente da fuselagem (m) — cabine + estrutura.
    pub d_fus_equiv_m: f64,
    /// S_molhada = coeff × π × d_equiv × comprimento (corpo afilado < cilindro).
    pub fuselage_wetted_coeff: f64,
    /// Fator de carga de POUSO ultimate N_l (= N_pouso × 1.5, Raymer 15.2).
    pub landing_load_factor_ult: f64,
    pub main_strut_length_m: f64,
    pub nose_strut_length_m: f64,
    /// Fração ±σ da estatística de frota das equações de peso — Raymer
    /// cap. 15/Roskam Classe II citam ±10–20% em projeto conceitual
    /// (equações empíricas ajustadas a uma frota histórica, não a ESTA
    /// aeronave). Consumida por `validation::robustness::adversarial_masses`
    /// (ciclo 4) para construir os dois conjuntos de massas estruturais
    /// ±σ e quantificar se checks que PASSAM no nominal sobrevivem à
    /// incerteza do modelo de massas. Faixa validada (0.05, 0.30).
    pub sigma_mass_fraction: f64,
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
                // Levemente diferentes do baseline real (−0.008/−0.30) —
                // mesma justificativa de "nenhum destes números coincide
                // com o baseline real" usada nas demais seções desta
                // fixture (task trim-authority).
                cm_ac: -0.010,
                cm_flap_delta: -0.28,
                // Distinto do baseline real (0.015) — mesma justificativa de
                // "nenhum destes números coincide com o baseline real" usada
                // nas demais seções desta fixture (ciclo 8, task 1).
                cd0_flap_delta: 0.020,
                // Distinto do baseline real (0.0) — ciclo 12, task 4. Valor
                // não-nulo DELIBERADO nesta fixture (dentro da faixa
                // plausível 0–0.10m) para que os testes que exercitam o
                // termo de arrasto de solo do balanço de rotação, se algum
                // dia usarem `config_teste()` em vez de literais próprios,
                // não fiquem mascarados pelo caso degenerado `h_D=0`.
                z_drag_above_cg_m: 0.05,
            },
            fuselage: FuselageCfg {
                length_m: 8.0,
                cabin_width_m: 1.20,
                cabin_height_m: 1.18,
                cd0: 0.0105,
            },
            empennage: EmpennageCfg {
                tail_arm_m: 4.70,
                v_h: 0.65,
                v_v: 0.045,
                ar_h: 4.5,
                ar_v: 1.6,
                taper_h: 0.45,
                taper_v: 0.45,
                eta_h: 0.92,
                // Levemente diferente da calibração do baseline real
                // (0.014366) — mesma justificativa de "nenhum destes
                // números coincide com o baseline real" usada nas demais
                // seções desta fixture (task refino-ciclo2, 1b).
                // `mass_per_area_{h,v}_kg_m2` morreram no ciclo 3 (massa da
                // empenagem computada por `agents::mass_model`).
                cd0_area_factor: 0.0135,
                // Distinto do baseline real (0.70, task refino-ciclo2 Task 4)
                // — mesma justificativa de "nenhum destes números coincide
                // com o baseline real" usada nas demais seções desta fixture.
                e_h: 0.80,
                // Ciclo 4 (t/c dedicado): DISTINTO do baseline real (0.10) —
                // mesma justificativa de "nenhum destes números coincide com
                // o baseline real" usada nas demais seções desta fixture.
                thickness_ratio: 0.12,
            },
            propeller: PropellerCfg {
                diameter_m: Some(1.90),
                blades: 2,
                psru_ratio: 2.0,
                psru_efficiency: 0.965,
                prop_axis_above_cg_m: 0.12,
                tip_mach_max_static: 0.83,
                tip_mach_max_cruise: 0.78,
                ground_clearance_min_m: 0.25,
                // Distinto do baseline real (0.20, ciclo 9) — mesma
                // justificativa de "nenhum destes números coincide com o
                // baseline real" usada nas demais seções desta fixture. <
                // x_nose_m (1.35) desta fixture, como a validação exige.
                // Valor DELIBERADAMENTE perto de x_nose_m (fator de
                // amplificação do pivô, `fill_critical_clearance`, próximo
                // de 1×): esta fixture mantém `ground_clearance_m` = 0,20 m
                // FIXO (`diameter_m`/`h_cg_ground_m`/`prop_axis_above_cg_m`
                // acima são load-bearing para
                // `violacao_de_folga_de_solo_aparece_naturalmente_na_
                // fixture_sintetica`, checagem #10 — não mexer), então não
                // sobra margem estática para um fator grande como o do
                // baseline real (≈1,466, que FALHA #25) — ver
                // `tire_deflation_delta_m` abaixo pelo mesmo motivo. A
                // checagem #25 desta fixture continua PASSANDO por
                // construção (ver `check_25_sem_violacao_na_fixture_
                // padrao`).
                prop_plane_x_m: 0.95,
                // Figura de mérito da hélice (ciclo 13, spec §3). DELIBERADAMENTE
                // DISTINTOS dos valores do baseline real (0.75/0.8237/1.8751) —
                // mesma justificativa de "nenhum destes números coincide com o
                // baseline real" usada nas demais seções desta fixture (spec §10
                // item 4): um literal do baseline real plantado em teste sintético
                // produz teste verde que não prova nada.
                fom_static: 0.72,
                fom_static_tol_pct: 10.0,
                fom_design: 0.80,
                j_design:   1.60,
            },
            fuel_system: FuelSystemCfg { capacity_l: 220.0 },
            gear: GearCfg {
                retractable: true,
                cd0_fixed_increment: 0.0082,
                h_cg_ground_m: 1.03,
                x_nose_m: 1.35,
                x_main_m: 3.75,
                retraction_time_s: 7.5,
                actuators_doors_mass_kg: 19.0,
                // Levemente diferente do baseline real (15.0/11.0/7.80/1.10,
                // Task 2 refino-ciclo2) — mesma justificativa de "nenhum
                // destes números coincide com o baseline real" usada nas
                // demais seções desta fixture. tail_cone_x_m (7.20) > x_main_m
                // (3.75), como a validação exige.
                tipback_min_deg: 14.0,
                rotation_attitude_deg: 10.0,
                tail_cone_x_m: 7.20,
                tail_cone_height_m: 1.00,
                // Distinto do baseline real (0.08, ciclo 8 task 2) — mesma
                // justificativa de "nenhum destes números coincide com o
                // baseline real" usada nas demais seções desta fixture.
                // Ciclo 9: reduzido de 0.05 para 0.035 (ainda > 0.03, piso
                // de validação) para preservar margem positiva de #25 sob
                // o fator de amplificação novo — ver comentário de
                // `propeller.prop_plane_x_m` acima.
                tire_deflation_delta_m: 0.035,
                // Ciclo 10 (task 1, deflexão estática no #25): campo novo,
                // DISTINTO do valor do baseline real (0.33) — mesma
                // justificativa de "nenhum destes números coincide com o
                // baseline real" usada nas demais seções desta fixture.
                // Dentro da faixa validada (0.15, 0.55). O termo novo só
                // ENCOLHE `Δ_prop` frente à fórmula antiga (curso restante
                // < curso total), então não ameaça a margem positiva de
                // #25 que esta fixture já preservava desde o ciclo 9.
                static_sag_fraction: 0.40,
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
            // Levemente diferente do baseline real (0.05/1.10/0.10/0.5/0.5/
            // 0.02) — mesma justificativa de "nenhum destes números
            // coincide com o baseline real" usada nas demais seções desta
            // fixture (task trim-authority: sm_max REMOVIDO, ver
            // `StabilityCfg`; task refino-ciclo2: cl_h_max_down REMOVIDO
            // — autoridade agora calculada por geometria — cl_h_stall_limit
            // fica só como teto).
            stability: StabilityCfg {
                sm_min: 0.06,
                cl_h_stall_limit: 1.05,
                trim_margin: 0.12,
                cl_ground_rotation: 0.55,
                to_flap_fraction: 0.5,
                fuselage_kf: 0.018,
            },
            // Só itens NÃO-estruturais (equipamentos/instalação). As 7
            // massas ESTRUTURAIS (asa, fuselagem, emp_horizontal,
            // emp_vertical, trem_principal, trem_nariz, tanques) saíram
            // daqui no ciclo 3 (oew-parametrico) — agora COMPUTADAS por
            // `agents::mass_model` e injetadas em
            // `weight_balance::oew_items` com mapeamento estático de
            // braços; os 7 nomes são PROIBIDOS em `[[masses.items]]` (erro
            // de migração, ver `models::config::
            // check_structural_mass_items_migration`).
            masses: MassesCfg {
                items: vec![
                    MassItemCfg { name: "psru_helice_capo".into(),   mass_kg: 62.0,  arm_ref: "engine_cg".into(),       arm_offset_m: 0.3 },
                    MassItemCfg { name: "resfriamento".into(),       mass_kg: 17.0,  arm_ref: "engine_cg".into(),       arm_offset_m: 0.5 },
                    MassItemCfg { name: "avionicos".into(),          mass_kg: 58.0,  arm_ref: "avionics".into(),        arm_offset_m: 0.0 },
                    MassItemCfg { name: "painel_comandos".into(),    mass_kg: 24.0,  arm_ref: "pax_front".into(),       arm_offset_m: -0.3 },
                    MassItemCfg { name: "mobiliario".into(),         mass_kg: 42.0,  arm_ref: "pax_front".into(),       arm_offset_m: 0.5 },
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
                // Levemente diferente da baseline real (25.0) — mesma
                // justificativa de "nenhum destes números coincide com o
                // baseline real" (task refino-ciclo2, 1a).
                elevator_deflection_max_deg: 24.0,
                rudder_span_frac: 0.85,
                rudder_chord_frac: 0.32,
            },
            // Levemente diferente do baseline real (0.40/0.30/0.04/0.08/1.0)
            // — mesma justificativa de "nenhum destes números coincide com o
            // baseline real" usada nas demais seções desta fixture.
            // `old→new` (ciclo 14): `approach_angle_deg`/`flare_time_s`
            // saíram (campos removidos de `PerformanceCfg`, spec §2/§6.2);
            // `flare_load_factor: 1.25` já existia desde o ciclo 14 Task 1,
            // deliberadamente distinto do baseline real (1.20).
            performance: PerformanceCfg {
                mu_brake_paved: 0.38,
                mu_brake_grass: 0.28,
                // Ciclo 12 (task 2): valores sintéticos, deliberadamente
                // distintos do baseline real (mesmo padrão dos demais
                // campos desta fixture).
                mu_roll_paved: 0.045,
                mu_roll_grass: 0.085,
                rotation_time_s: 1.2,
                flare_load_factor: 1.25,
            },
            // Orçamento elétrico sintético (Task 5.2) — valores levemente
            // diferentes do baseline real (mesma justificativa de "nenhum
            // destes números coincide com o baseline real"). trem_retratil
            // peak_w (480 W) fica com folga larga sobre a potência mecânica
            // do atuador de retração (dezenas de W — ver
            // `agents::landing_gear::actuator_power_w`, que desde o ciclo 3
            // usa a massa COMPUTADA de uma perna, `trem_principal_kg/2`).
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
            mass_model: MassModelCfg {
                composite_factor_wing: 0.90,
                composite_factor_tail: 0.80,
                composite_factor_fuselage: 0.95,
                composite_factor_gear: 1.00,
                composite_factor_fuel_system: 1.05,
                d_fus_equiv_m: 1.10,
                fuselage_wetted_coeff: 0.70,
                landing_load_factor_ult: 4.0,
                main_strut_length_m: 0.50,
                nose_strut_length_m: 0.40,
                // Fixture do ciclo 4 (task robustez) — dentro da faixa
                // validada (0.05, 0.30), distinto do baseline real (0.15)
                // por conveniência de teste (perturbação visível nos hand-
                // checks sem coincidir com o valor de projeto).
                sigma_mass_fraction: 0.20,
            },
        }
    }
}
