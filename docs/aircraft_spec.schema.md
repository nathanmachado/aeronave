# `aircraft_spec.json` — contrato do schema v5.6

Este documento é o **contrato formal** entre o pipeline de modelagem
matemática (`aeronave`, este repositório) e qualquer consumidor a jusante —
hoje, principalmente o time/agente de CAD paramétrico (FreeCAD). Descreve
cada bloco de topo do JSON gerado por `aeronave` (`src/main.rs`, tipo Rust
`aeronave::models::specs::AircraftReport`), campo a campo: nome, tipo,
unidade, agente que o produz, nível de confiança (fidelidade) e qualquer
convenção especial de interpretação.

Gerado a partir de (e deve ser mantido em sincronia com)
`src/models/specs.rs`. Em caso de divergência entre este documento e o
código, **o código é a fonte da verdade** — mas divergência é um bug de
documentação a ser corrigido, não um comportamento aceitável.

## 1. Versionamento do schema

- `schema_version` (string, ex.: `"4.0"`) é o campo autoritativo de versão
  — lido pela constante Rust `aeronave::models::specs::SCHEMA_VERSION`.
- `revision`: **DEPRECATED**, mantido só por compatibilidade com
  consumidores anteriores à v4 que liam uma string de revisão livre sem
  política declarada (`"3.0"`, sem semântica de bump). Desde a v4, tem
  sempre o MESMO valor de `schema_version`. Novos consumidores devem ler
  `schema_version`, não `revision`.
- **Política de bump** (`MAJOR.MINOR`):
  - **MINOR** (ex.: 4.0 → 4.1): mudança aditiva — novo campo opcional, novo
    bloco de topo, novo par chave/valor em `fidelity`. Consumidores
    existentes continuam funcionando sem alteração (campos desconhecidos
    devem ser ignorados pelo parser do consumidor).
  - **MAJOR** (ex.: 4.0 → 5.0): mudança que quebra compatibilidade —
    renomeia ou remove um campo existente, muda o TIPO ou a UNIDADE de um
    campo existente, ou muda a semântica de um campo sem mudar seu nome.
    Consumidores precisam ser atualizados antes de consumir a nova versão.
  - **Exceção registrada (v5.2)**: `propeller.prop_clearance_critical_m`
    mudou de fórmula (semântica) sem mudar nome/tipo/unidade — pela letra
    da regra acima isso seria gatilho de MAJOR. Decisão de projeto
    (aprovada pelo usuário, ciclo9-transferencia-atitude): tratado como
    MINOR porque é uma correção de CORREÇÃO FÍSICA (bug de modelagem —
    simplificação otimista virando fórmula honesta), não uma mudança de
    CONTRATO — o TIPO/nome/unidade do campo no JSON são idênticos, e o
    consumidor v5.1 que já lia esse campo numericamente continua lendo o
    mesmo tipo, só com um valor mais correto. Registrado aqui para não
    esconder a divergência entre esta política e a decisão real tomada.
    **Mesma exceção reaplicada** (ciclo10-sag-e-linha-de-tracao, task 1,
    2026-08-09, ainda dentro da v5.2 — bump formal fica para a Task 3 do
    mesmo ciclo): a fórmula de `prop_clearance_critical_m` mudou DE NOVO
    (curso TOTAL do amortecedor de nariz → curso RESTANTE até o batente,
    corrigindo uma dupla contagem da compressão estática — ver §1 bloco
    `propeller` e `docs/backlog.md` item 6, RESOLVIDO), pelo MESMO
    raciocínio: correção física, não mudança de contrato. Baseline real
    E10: **≈−0,06416 m (ciclo 9) → ≈−0,00249 m (ciclo 10)** — MESMO
    veredito (checagem #25 continua FAIL), só o número muda.
    **Terceira aplicação da mesma exceção** (ciclo10-sag-e-linha-de-tracao,
    task 2, 2026-08-09, ainda dentro da v5.2): o momento da LINHA DE TRAÇÃO
    entra no balanço de rotação e no trim de cruzeiro. Nenhum campo novo,
    nenhum rename, nenhuma mudança de tipo/unidade — mas a SEMÂNTICA de
    `trim.rotation_limit_pct_mac` muda (deixa de ser invariante ao peso e
    passa a ser a envoltória avaliada no cenário mais leve; ver §4) e os
    VALORES de `trim.rotation_limit_pct_mac`, `trim.cl_h_trim_cruise`,
    `trim.cd_trim` e `weight.cg_limit_fwd_pct_mac` mudam. Mesmo raciocínio
    das duas anteriores: é CORREÇÃO FÍSICA (um termo de momento que
    faltava), não mudança de contrato. Baseline real:
    `rotation_limit_pct_mac` **8,533% → 13,355% MAC** (+4,82 pp);
    `validation_status` continua `"FAIL"` com a MESMA 1 violação (a de
    hélice, #25, inalterada) e ZERO flips de robustez — o que encolhe é a
    FOLGA (margem de rotação do cenário mais apertado, "Solo (piloto)",
    de +21,6% para +10,5%). `robustness.flips[]` ganha o campo
    `limite_nominal` (aditivo; ver §4) — o bump formal para 5.3 fica para
    a Task 3 do ciclo.
- Histórico: v3.x usava `revision` como string livre, sem os blocos
  `geometry`/`sizing`/`fidelity`/`warnings` (que existiam calculados
  internamente, mas não eram serializados) e sem política de bump
  declarada. v4.0 (Task 6.1) formaliza o contrato.
- **v4.1** (task trim-authority): adiciona o bloco `trim` (`TrimSpec` —
  ver §4 abaixo) — mudança ADITIVA (novo bloco opcional), consumidores v4.0
  continuam funcionando sem alteração.
  - **Migração de CONFIGURAÇÃO** (`aircraft.toml`, não deste schema JSON):
    `[stability].sm_max` (proxy de margem estática máxima usado como limite
    dianteiro do envelope de CG, sem base física direta em autoridade de
    controle) foi **REMOVIDO**. O limite dianteiro do bloco `weight`
    (`weight.cg_limit_fwd_pct_mac`) agora é calculado FISICAMENTE pelo
    `TrimAuthorityAgent` a partir de autoridade de profundor real — o
    NOME/TIPO/UNIDADE do campo JSON `weight.cg_limit_fwd_pct_mac` não
    mudou (ainda é `f64`, %MAC), só a fonte de cálculo, por isso a mudança
    não quebra consumidores existentes. Configs `aircraft.toml` antigas com
    `sm_max` presente são REJEITADAS com um erro de migração claro por
    `models::config::parse_aircraft` — substitua `[stability].sm_max` por
    `[stability].trim_margin`/`cl_ground_rotation`/`to_flap_fraction` +
    `[wing].cm_ac`/`cm_flap_delta` (ver `config/aircraft/baseline_4seat.toml`
    para valores de referência).
- **v4.2** (task refino-ciclo2): `TrimSpec` ganha três campos NOVOS
  (`cl_h_max_down_calc`, `tau_elevator`, `capped_by_stall`) e
  `TrimSensitivity` ganha quatro campos NOVOS (par
  `elevator_deflection_max_deg_minus`/`plus` +
  `flare_limit_pct_mac_deflection_minus`/`plus`) — mudança ADITIVA (campos
  novos em blocos já existentes; nenhum campo existente foi removido nem
  mudou de tipo/unidade), consumidores v4.1 continuam funcionando sem
  alteração. `trim.cl_h_max_down` PERMANECE presente com o MESMO
  significado (valor operacional usado no balanço de momentos) — só a
  FONTE mudou (antes ecoava `[stability].cl_h_max_down` da config, agora é
  CALCULADO por geometria DATCOM/Nelson a partir de
  `[control_surfaces].elevator_chord_frac`/`elevator_deflection_max_deg` e
  `EmpennageSpec.ar_h`).
  - **Migração de CONFIGURAÇÃO** (`aircraft.toml`, não deste schema JSON),
    TRÊS remoções, todas com erro de migração claro em
    `models::config::parse_aircraft`:
    1. `[stability].cl_h_max_down` (parâmetro livre) foi **REMOVIDO** —
       substitua por `[control_surfaces].elevator_deflection_max_deg`
       (faixa 10–35°) + `[stability].cl_h_stall_limit` (faixa 0.8–1.4, teto
       por stall da empenagem).
    2. `[empennage].cd0` (valor fixo) foi **REMOVIDO** — substitua por
       `[empennage].cd0_area_factor` (faixa 0.008–0.025), aplicado sobre
       `(S_h+S_v)/S_w` — a área da empenagem REALMENTE dimensionada, não
       mais uma parcela de arrasto desacoplada da geometria.
    3. Os itens `emp_horizontal`/`emp_vertical` de `[[masses.items]]`
       (massas fixas) foram **REMOVIDOS** — substitua por
       `[empennage].mass_per_area_h_kg_m2`/`mass_per_area_v_kg_m2` (faixa
       4–20 kg/m²), multiplicados por `EmpennageSpec::s_horizontal_m2`/
       `s_vertical_m2` (`weight_balance::oew_items`) em vez de um valor
       fixo na lista de massas — isto valia NA ÉPOCA (v4.2). Desde a
       **v4.5** (ciclo 3, oew-parametrico) os próprios
       `mass_per_area_h_kg_m2`/`mass_per_area_v_kg_m2` foram removidos: a
       substituição atual é `[mass_model]` (fatores de composto
       `composite_factor_tail` etc., Raymer Tab. 15.4, aplicados às
       equações de componente de `agents::mass_model`) — ver a entrada v4.5
       abaixo e `models::specs::TrimSpec` (docstring) para o mesmo hedge do
       lado Rust.
    Ver `config/aircraft/baseline_4seat.toml` para valores de referência
    calibrados (reproduzem os antigos valores fixos na área/autoridade
    runtime desta config, dentro de resíduo de arredondamento
    desprezível — ver task-1-report.md).
- **v4.3** (Task 2, refino-ciclo2): `GearSpec::nose_load_fraction_pct`
  (campo único, calculado só no CG mais TRASEIRO real) foi RENOMEADO por
  DOIS campos — `nose_load_max_pct` (CG mais DIANTEIRO real, teto de 25%) e
  `nose_load_min_pct` (CG mais TRASEIRO real, piso de 8% — numericamente o
  antigo `nose_load_fraction_pct`). Esta é, estritamente, uma mudança que
  QUEBRA compatibilidade (campo removido) — versionada como bump MINOR por
  diretriz explícita desta task, não como exceção à política de §1 acima
  (consumidores que leem `nose_load_fraction_pct` devem migrar para
  `nose_load_min_pct`, equivalente). `GearSpec` também ganha dois campos
  NOVOS (aditivos): `tipback_angle_deg` e `tail_strike_margin_deg` — ver
  `agents::landing_gear` (Raymer, "Aircraft Design", cap. 11).
  - **Migração de CONFIGURAÇÃO** (`aircraft.toml`, não deste schema JSON):
    `[gear]` ganha QUATRO campos NOVOS **obrigatórios** (sem default
    implícito) — `tipback_min_deg` (faixa (8, 25)°, típico 15°),
    `rotation_attitude_deg` (faixa (5, 18)°, típico 11°), `tail_cone_x_m`
    (faixa (3.0, 12.0) m, deve ser maior que `x_main_m`) e
    `tail_cone_height_m` (faixa (0.3, 2.5) m). TOMLs antigos sem esses
    campos falham o parse (`missing field`) — não há um erro de migração
    dedicado (diferente das remoções da v4.1/v4.2), porque não há um
    campo antigo equivalente para detectar e redirecionar; a mensagem de
    erro do parser TOML já nomeia o campo ausente. Ver
    `config/aircraft/baseline_4seat.toml` §`[gear]` para valores de
    referência.
  - **Achado honesto NOVO** (não uma mudança de contrato, mas relevante
    para consumidores): a aeronave-base real reporta `tipback_angle_deg
    ≈ 10.1°`, abaixo do piso de 15° — `validation_status` do baseline real
    volta a `"FAIL"` (era `"PASS"` desde a v4.1/campanha E1–E6). Ver
    `ConstraintChecker::verify` (checks #15-17) e
    `tests/gear_tipback.rs`/`tests/cli.rs`.
- **Task 3, refino-ciclo2 (ainda v4.3 — SEM mudança de forma do JSON)**:
  margem mínima de combustível como requisito de projeto — check NOVO #18
  de `ConstraintChecker::verify`. Não altera nenhum campo/bloco deste
  schema (`violations` já era `array de string` genérico; `sizing.
  fuel_margin_pct`, já existente desde a v4.0, já usava a convenção
  %-da-CAPACIDADE do tanque — ver a tabela do bloco `sizing` abaixo — o
  gate novo só passou a comparar esse número já existente contra um piso
  configurável).
  - **Migração de CONFIGURAÇÃO** (`mission.toml`, não deste schema JSON):
    ganha um campo NOVO **obrigatório** — `min_fuel_margin_fraction`
    (faixa [0, 0.3], fração da CAPACIDADE do tanque, não do combustível
    exigido pela missão). TOMLs de missão antigos sem esse campo falham o
    parse (`missing field`). Ver `config/missions/default.toml`/
    `rotax_ferry.toml` para valores de referência.
  - **Achado honesto NOVO**: a aeronave-base real (missão de projeto
    completa) reportava `sizing.fuel_margin_pct ≈ 1,82%` na v4.3; ver v4.4
    — abaixo do piso de 5% (`min_fuel_margin_fraction` do `default.toml`)
    — mais uma entrada em `violations` (`validation_status` já era
    `"FAIL"` por causa do tipback). Ver `tests/gear_tipback.rs`/
    `tests/cli.rs`.
- **v4.4** (Task 4, refino-ciclo2): `TrimSpec` ganha QUATRO campos NOVOS —
  `cl_h_trim_cruise`, `cd_trim`, `cg_reference_scenario`,
  `cg_reference_pct_mac` (arrasto de trim em cruzeiro — ver §4 abaixo) —
  mudança ADITIVA (campos novos num bloco já existente; nenhum campo
  removido nem mudou de tipo/unidade), consumidores v4.3 continuam
  funcionando sem alteração. `wing.cd_cruise`/`wing.ld_ratio_cruise`
  PERMANECEM com o MESMO nome/tipo/unidade — só o VALOR muda (agora inclui
  o arrasto de trim de cruzeiro, `ΔCD_trim`, somado por
  `agents::aerodynamics::apply_cruise_trim_drag`).
  - **Física**: em cruzeiro (sem flap), a empenagem horizontal precisa
    gerar `CL_h_trim` (upload OU download) para equilibrar o momento de
    arfagem no CG de REFERÊNCIA da missão (cenário "4 pax + bagagem +
    meia" — meia-missão, escolhido por representar o CG médio ao longo do
    voo de cruzeiro, não um extremo — ver `agents::trim_authority::
    cl_h_trim_cruise`):
    ```
    CL_h_trim = [cm_ac + cm_thrust + CL_cruise·(x̄_cg−0,25)]
                / [η_h·(S_h/S_w)·(l_h/MAC+0,25−x̄_cg)]
    ΔCD_trim  = (CL_h_trim²/(π·ar_h·e_h))·(S_h/S_w)
    ```
    O termo `cm_thrust` entrou no **ciclo 10 (task 2)** — momento da linha
    de tração em torno do CG em voo:
    ```
    cm_thrust = − T_cruzeiro · [propeller].prop_axis_above_cg_m / (q·S_w·MAC)
    ```
    Braço sobre o **CG** (não sobre o solo, ao contrário da rotação — o
    pivô é diferente). Sinal: eixo ACIMA do CG + tração para a frente ⟹
    nariz-abaixo ⟹ `Cm` NEGATIVO, o que empurra `CL_h_trim` na direção
    NEGATIVA (mais download / menos upload). No baseline real vale ≈−0,0054
    contra um `cm_ac` de −0,008 — não é um termo desprezível.
    **Aproximação documentada** (`z_D = 0`): em cruzeiro nivelado `T = D`,
    horizontais e opostas, formando um binário de braço `(z_T − z_D)`. Este
    modelo assume a resultante de ARRASTO passando pelo CG, o que deixa o
    braço líquido igual a `prop_axis_above_cg_m`. Num centro de arrasto
    tipicamente 0–0,10 m acima do CG, isso SUPERESTIMA o `cm_thrust`
    nariz-abaixo em até ~50% no pior caso plausível — direção
    CONSERVADORA para o trim. Refinar exigiria um campo de config novo
    (`z_drag_above_cg_m`) sem base no CAD atual.
    `cm_ac` é o coeficiente de momento do perfil ISOLADO (sem
    `cm_flap_delta` — cruzeiro é sem flap, ao contrário do balanço de
    flare/rotação). `e_h` é a eficiência de Oswald da empenagem horizontal
    (config nova, ver abaixo). Aproximação documentada: a contribuição de
    sustentação extra que a asa precisaria gerar para compensar o
    upload/download da cauda é DESPREZADA (efeito de 2ª ordem, ver
    docstring da função).
  - **Acoplamento no laço de convergência de MTOW** (`orchestrator::
    size_aircraft`): `CL_h_trim` depende do CG (`WeightBalanceAgent`), que
    só fica disponível DEPOIS da aerodinâmica rodar na MESMA iteração — em
    vez de resolver por bisseção, usa-se o CG da iteração ANTERIOR do
    próprio laço de MTOW (lag-1; seed inicial 0,0). `TrimSpec.
    cl_h_trim_cruise`/`cd_trim` no relatório final, por outro lado, são
    recalculados com o CG JÁ CONVERGIDO (não o lag) — ver docstring de
    `TrimSpec::cl_h_trim_cruise` em `src/models/specs.rs`.
  - **Migração de CONFIGURAÇÃO** (`aircraft.toml`, não deste schema JSON):
    `[empennage]` ganha um campo NOVO **obrigatório** — `e_h` (eficiência
    de Oswald da empenagem horizontal, faixa (0.5, 0.95)). TOMLs antigos
    sem esse campo falham o parse (`missing field`) — não há um erro de
    migração dedicado (mesmo padrão da v4.3, não há campo antigo
    equivalente). Ver `config/aircraft/baseline_4seat.toml` §`[empennage]`
    para valor de referência (0,70).
  - **Achado honesto NOVO**: no baseline real (motor padrão), o CG de
    meia-missão fica bem mais atrás do CA da asa (upload de trim,
    `CL_h_trim ≈ +0,044`), gerando `ΔCD_trim ≈ 4,9e-5` (~0,17% do CD0) —
    pequeno, quase neutro. Com o Rotax 915 iS (motor bem mais leve,
    montado no nariz), o CG de meia-missão fica proporcionalmente mais
    atrás ainda (sem o peso do motor no nariz), exigindo `CL_h_trim ≈
    +0,18` e `ΔCD_trim ≈ 8,0e-4` (~3,4% do CD0 — NÃO desprezível) —
    penalidade de arrasto de trim mais visível para motores mais leves que
    o Toyota para o qual esta célula foi dimensionada. Nos dois casos, o
    MTOW convergido sobe levemente e a margem de combustível cai um pouco
    (~0,1–0,7 pontos percentuais, dependendo da missão) — ver
    `tests/gear_tipback.rs`/`tests/generic_engine.rs`/`tests/acceptance.rs`
    para os pins atualizados.
- **Ciclo 3 — oew-parametrico (ainda v4.4 — SEM mudança de forma do
  JSON)**: as 7 massas ESTRUTURAIS do OEW (asa, fuselagem, empenagem
  horizontal/vertical, trem principal/nariz, sistema de combustível)
  deixaram de ser DADOS de configuração e passaram a ser COMPUTADAS pelas
  equações de componente de Raymer ("Aircraft Design: A Conceptual
  Approach", cap. 15.2, equações GA) × fatores de composto (Tab. 15.4) —
  `agents::mass_model`. Nenhum campo/bloco deste schema JSON muda de
  nome, tipo ou unidade; `weight.oew_kg`/`weight.mtow_kg`/`weight.cg_*`
  e tudo que deriva deles mudam de VALOR.
  - **Migração de CONFIGURAÇÃO** (`aircraft.toml`, não deste schema JSON),
    TRÊS remoções, todas com erro de migração claro em
    `models::config::parse_aircraft`:
    1. Os SETE nomes `asa`, `fuselagem`, `emp_horizontal`, `emp_vertical`,
       `trem_principal`, `trem_nariz` e `tanques` são agora **PROIBIDOS**
       em `[[masses.items]]` (`check_structural_mass_items_migration`) —
       mantê-los contaria a mesma massa duas vezes. Sobram na seção só os
       itens NÃO-estruturais (equipamentos/instalação). Os braços
       continuam vindo de `[arms]`/`[wing]`/`[gear]`: o mapeamento
       componente→braço é ESTÁTICO em `weight_balance::oew_items` e usa
       exatamente os mesmos `arm_ref` que os itens removidos usavam.
    2. `[empennage].mass_per_area_h_kg_m2`/`mass_per_area_v_kg_m2` (que na
       v4.2 tinham substituído os itens fixos `emp_horizontal`/
       `emp_vertical`) foram **REMOVIDOS**
       (`check_mass_per_area_migration`) — a massa das empenagens agora sai
       de `htail_mass_raymer_kg`/`vtail_mass_raymer_kg` ×
       `[mass_model].composite_factor_tail`, funções de S_h/S_v, N_z, q,
       alongamento e afilamento.
    3. `[gear].mass_main_leg_kg` foi **REMOVIDO**
       (`check_mass_main_leg_migration`) — a massa total do trem principal
       é computada e a massa de UMA perna usada no dimensionamento do
       atuador de retração passou a ser essa total ÷ 2
       (`agents::landing_gear`). Com isso morrem também duas guardas de
       consistência que existiam só para amarrar campos de config
       redundantes: `'trem_principal' == 2× mass_main_leg_kg` e
       `'trem_retratil'.peak_w >= potência mecânica do atuador` (esta
       última porque sua entrada só existe DEPOIS do MTOW convergir, muito
       além do parse do TOML — a potência do atuador continua calculada e
       reportada em `landing_gear.actuator_power_w`).
    `[mass_model]` (10 campos obrigatórios: `composite_factor_wing/tail/
    fuselage/gear/fuel_system`, `d_fus_equiv_m`, `fuselage_wetted_coeff`,
    `landing_load_factor_ult`, `main_strut_length_m`,
    `nose_strut_length_m`) é a nova fonte de calibração. Ver
    `config/aircraft/baseline_4seat.toml` para valores de referência.
  - **Achado honesto NOVO** (não uma mudança de contrato, mas relevante
    para consumidores): o total estrutural mal se move (422,0 → 411,0 kg;
    `weight.oew_kg` 890,0 → 879,0 kg), mas a DISTRIBUIÇÃO muda muito —
    fuselagem 160,0→110,6 kg e empenagens 43,0→19,6 kg (braços traseiros)
    contra trem 77,0→110,5 kg, asa 130,0→148,0 kg e tanques 12,0→22,4 kg
    (braços dianteiros). O CG VAZIO AVANÇA e com ele o de todos os
    cenários: `weight.cg_mac_fwd_pct`/`cg_mac_aft_pct` vão de 16,0/37,5%
    para **8,3/31,7% MAC**. Consequência: `validation_status` do baseline
    real volta a `"FAIL"` com TRÊS violações — dois cenários leves à frente
    do limite dianteiro de rotação (13,0% MAC) e
    `landing_gear.nose_load_max_pct` 24,8→**29,0%**, acima do teto de 25%.
    Tipback FOLGA (15,6→19,2°) e a margem de combustível SOBE
    (13,97→14,56%). Ver `tests/cli.rs`/`tests/gear_tipback.rs`.
- **v4.5** (Task 5, oew-parametrico): `weight` (`WeightSpec`) ganha UM
  campo NOVO, `structural_masses` (`StructuralMassesSpec`) — as 7 massas
  ESTRUTURAIS computadas descritas na entrada "Ciclo 3" acima (asa,
  fuselagem, empenagem horizontal/vertical, trem principal/nariz, sistema
  de combustível), já usadas internamente desde aquele ciclo mas nunca
  ecoadas no JSON, agora rastreáveis pelo consumidor de CAD — mais os 5
  fatores de composto de `[mass_model]` usados para calculá-las
  (`composite_factor_wing/tail/fuselage/gear/fuel_system`). Mudança
  ADITIVA (campo novo num bloco já existente; nenhum campo removido nem
  mudou de tipo/unidade), consumidores v4.4 continuam funcionando sem
  alteração. `fidelity.weight` muda de texto — de `"preliminary"` (soma de
  itens de massa configurados não pesados) para `"semi-empirical"`
  (estruturas: Raymer 15.2 GA × fatores de composto Tab. 15.4; hardware:
  itens configurados ainda não pesados) — reflete que a MAIOR parte do OEW
  (as 7 massas estruturais) agora vem de equações semi-empíricas, não mais
  de valores de catálogo/projeto direto; o hardware (aviônicos, bateria,
  cabos etc.) continua `preliminary` internamente, mas o rótulo agregado do
  bloco passa a refletir o método DOMINANTE. Ver §3 e §4 abaixo.

- **v4.6** (Task 4, ciclo4-fidelidade-massas): `AircraftReport` ganha UM
  bloco NOVO, `robustness` (`RobustnessSpec`) — checagem #19 NOVA em
  `ConstraintChecker::verify`. As 7 massas estruturais (`weight.
  structural_masses`, desde a v4.5) vêm de equações empíricas de
  componente (Raymer cap. 15.2) ajustadas a uma FROTA histórica, não a
  ESTA aeronave — incerteza típica de projeto conceitual de ±10-20%
  (Raymer/Roskam Classe II). `validation::robustness::RobustnessAgent`
  (módulo já existente desde a Task 3 do ciclo, isolado do pipeline até
  aqui) constrói dois conjuntos adversariais DETERMINÍSTICOS (±σ
  direcional: um empurra o CG vazio o mais para a FRENTE possível, o outro
  o mais para TRÁS) e reavalia os cenários de CG e o trem de pouso contra
  os MESMOS limites NOMINAIS já calculados (esses limites — autoridade de
  profundor, tetos/pisos de carga de nariz — são invariantes à massa
  estrutural). Um check que PASSA no nominal mas REPROVA sob um dos dois
  conjuntos é um `flip` (`RobustnessSpec::flips`) — a checagem #19 gera uma
  violação nomeada por flip: `"Robustez: {check} passa no nominal mas
  reprova com massas estruturais ±{σ}% (pior caso {caso}): {valor} vs
  {limite}"`. Mudança ADITIVA (bloco novo opcional + checagem nova que só
  pode ADICIONAR violações, nunca remover as existentes), consumidores
  v4.5 continuam funcionando sem alteração. Achado honesto do baseline
  real (`config/aircraft/baseline_4seat.toml`, σ=15%): ZERO flips — as
  margens nominais dos checks que passam (tipback, carga de nariz mínima,
  os 4 cenários de CG dentro do envelope) são folgadas o bastante para
  absorver a perturbação; as 3 violações nominais já existentes (2
  cenários de envelope + carga de nariz máxima, ver entrada "Ciclo 3"
  acima) continuam as ÚNICAS. Ver §3 e §4 abaixo, e `validation::
  robustness` para a dedução completa.
- **v4.7** (Task 4, ciclo5-robustez-total-e-solo): dois campos NOVOS em
  blocos já existentes.
  - `electrical` (`ElectricalSpec`) ganha `loads`
    (array de `ElectricalLoadSpec` — checagem #20 NOVA em
    `ConstraintChecker::verify`): eco individual de cada `[electrical].
    loads` configurada (`name`, `continuous_w`, `peak_w`), para que o
    checker compare o pico DECLARADO da carga `'trem_retratil'` contra
    `landing_gear.actuator_power_w` COMPUTADO pelo `LandingGearAgent` —
    checagem só possível PÓS-convergência (a guarda equivalente de
    parse-time do ciclo 3 foi removida quando a massa da perna do trem
    virou computada).
  - `robustness` (`RobustnessSpec`) ganha `mtow_masstotal_kg` — MTOW (kg)
    re-convergido pelo laço COMPLETO de `orchestrator::size_aircraft` sob
    um TERCEIRO conjunto adversarial, "massa-total": as 5 massas
    estruturais COMPOSTAS (asa, empenagens, fuselagem, trem, tanques) vão
    todas ×(1+σ) simultaneamente — não ±σ direcional como os dois casos de
    CG já existentes na v4.6 (que só reavaliam limites nominais contra CG
    perturbado, sem re-rodar o laço de convergência). A checagem #19 ganha
    o caso "massa-total": um re-sizing INTEIRO sob incerteza de massa
    (MTOW/combustível podem mudar, não só o CG). `0.0` quando o sizing
    perturbado FALHA (`SizingError`) — nesse caso o flip de
    "Dimensionamento" acompanha e documenta a causa.
  - Mudança ADITIVA (campos novos em blocos já existentes; nenhum campo
    removido nem mudou de tipo/unidade), consumidores v4.6 continuam
    funcionando sem alteração. Achado honesto do baseline real
    (`config/aircraft/baseline_4seat.toml`): `mtow_masstotal_kg` ≈
    1585,9 kg, ACIMA do MTOW de missão nominal (`sizing.mtow_mission_kg`
    ≈ 1512,4 kg) — sizing perturbado converge normalmente, zero flip de
    Dimensionamento; as MESMAS 3 violações nominais continuam as únicas.
  - Acompanha, do lado da CONFIGURAÇÃO de entrada (não deste schema JSON
    — já implementado na Task 1 do mesmo ciclo): `[propeller].
    shaft_height_m` (datum ABSOLUTO e desacoplado do trem — encurtar o
    trem NÃO afetava a folga reportada) foi REMOVIDO com erro de
    migração, substituído por `[propeller].prop_axis_above_cg_m` (offset
    vertical FIXO entre o eixo da hélice e o CG). `propeller.
    ground_clearance_m` (campo já existente, sem mudança de nome/tipo)
    agora deriva de `gear.h_cg_ground_m + propeller.
    prop_axis_above_cg_m` em vez do datum absoluto antigo — acopla a
    folga de hélice ao comprimento do trem, para que encurtar o trem
    consuma folga de hélice automaticamente. Ver §3 e §4 abaixo.
- **v4.8** (Task 4, ciclo6-pista-e-robustez-final; entrada **EMENDADA**
  na revisão final do mesmo ciclo — a v4.8 ainda NÃO havia shipado
  quando o achado do pouso na grama apareceu, então a entrada foi
  corrigida no lugar em vez de virar uma v4.9): **UM campo NOVO** neste
  schema JSON (`performance.ldg_50ft_grass_m`), mais contrato de
  COMPORTAMENTO (`violations`/`flips` podem conter textos NOVOS; nenhum
  bloco/campo existente muda de nome, tipo ou unidade). Quatro mudanças:
  1. **Requisito de pista** — `ConstraintChecker::verify` ganha as
     checagens #23 (decolagem na GRAMA sobre obstáculo de 15 m,
     `performance.to_50ft_grass_m`, excede a pista disponível) e #24
     (pouso na GRAMA sobre 15 m, `performance.ldg_50ft_grass_m`, idem) —
     comparadas contra `runway_available_m`, um requisito NOVO
     **obrigatório** de missão (não deste schema JSON — ver migração de
     CONFIGURAÇÃO abaixo). Textos de violação NOVOS possíveis:
     `"Decolagem (grama, 15 m): {d} m excede a pista disponível de
     {p} m"` e `"Pouso (grama, 15 m): {d} m excede a pista disponível de
     {p} m"`. Os dois comparadores são `>`, isto é, semântica
     **INCLUSIVA deliberada**: distância EXATAMENTE igual à pista
     disponível PASSA — consistente com `propeller.ok_clearance` (`>=`)
     e com os demais pisos/tetos do checker; a margem operacional é
     responsabilidade do valor configurado em `runway_available_m`, não
     de uma folga implícita no operador.
  1b. **Campo NOVO `performance.ldg_50ft_grass_m`** (f64, m) — distância
     de pouso sobre 15 m em GRAMA: mesmos segmentos de `ldg_50ft_m`
     (aproximação + flare + rolagem de frenagem), mas a rolagem usa
     `[performance].mu_brake_grass` em vez de `mu_brake_paved`. Frenagem
     pior ALONGA a rolagem, logo é sempre MAIOR que a pavimentada, e é o
     caso DIMENSIONANTE da premissa de pista do projeto. Motivo do
     campo: `mu_brake_grass` existia na config desde a Task 4.7,
     validado e **nunca consumido** — o check #24 gateava uma pista de
     grama com a distância de pouso PAVIMENTADA, otimista por
     construção. `ldg_50ft_m` permanece no JSON, agora INFORMATIVO
     (simétrico ao par `to_50ft_paved_m`/`to_50ft_grass_m` da
     decolagem). Campo ADITIVO — consumidores v4.7 que ignoram campos
     desconhecidos seguem funcionando.
  2. **Gates do caso "massa-total" ampliados** (checagem #19,
     `robustness.flips`) — até a v4.7, o mundo "massa-total"
     (`RobustnessAgent`, MTOW re-convergido com as 5 massas estruturais
     compostas ×(1+σ)) só avaliava os gates de DESEMPENHO (margem de
     combustível, VS0, razão de subida, v_cruise, teto de serviço);
     `sized_p.wb` (o CG re-convergido desse mundo) era descartado. Desde
     esta task, o mundo massa-total avalia TAMBÉM pista (#23/#24 acima,
     as duas grandezas de GRAMA, iguais às do checker) e
     envelope de CG/carga de nariz/tipback — a MESMA avaliação já aplicada
     aos dois casos direcionais (±σ) desde a v4.6, agora reutilizada
     (função helper compartilhada) sobre `sized_p.wb`/`LandingGearAgent`
     re-computado desse mundo. Mais nomes de check POSSÍVEIS em
     `robustness.flips` com `caso == "massa-total"` (os mesmos nomes já
     usados pelos casos direcionais: cenários de CG, carga de nariz
     máx/mín, tipback) — nenhum flip existente muda de forma.
  3. **Refactor mecânico** (`ConstraintChecker::verify` → recebe
     `VerifyInputs<'a>`, uma struct, no lugar de 15 parâmetros
     posicionais) — mudança de assinatura Rust interna, ZERO efeito no
     JSON gerado (mesmos valores, mesma ordem de checagem, mesmas
     mensagens para os checks #1-22 já existentes).
  - Mudança ADITIVA em forma e em comportamento (um campo novo; mais
    violações/flips POSSÍVEIS, nunca menos; nenhum texto/campo existente
    removido ou alterado), consumidores v4.7 continuam funcionando sem
    alteração.
  - **Migração de CONFIGURAÇÃO** (`mission.toml`, não deste schema
    JSON): ganha um campo NOVO **obrigatório** — `runway_available_m`
    (faixa válida (300, 2000) m). TOMLs de missão antigos sem esse campo
    falham o parse (`missing field`) — mesmo padrão sem erro de migração
    dedicado das v4.3/v4.4 (não há campo antigo equivalente para
    redirecionar). Ver `config/missions/default.toml` (600 m — premissa
    de pista de fazenda, deliberadamente apertada) e `rotax_ferry.toml`
    (800 m — ferry entre aeródromos) para valores de referência.
  - **Achado honesto do baseline real** (`config/aircraft/
    baseline_4seat.toml`, missão `default.toml`, pista 600 m) —
    CORRIGIDO na revisão final do ciclo; a redação anterior desta
    entrada dizia que "os checks #23/#24 passam LIMPOS", o que era
    verdade apenas porque #24 media a superfície ERRADA:
    - **#23 (decolagem na grama) PASSA** — `to_50ft_grass_m` ≈ 428,2 m
      contra 600 m, folga de ≈172 m.
    - **#24 (pouso na grama) REPROVA** — `ldg_50ft_grass_m` ≈ 605,0 m
      contra 600 m. Com o μ pavimentado (o que o check media antes da
      correção) a distância era 540,0 m e cabia; com `mu_brake_grass`
      = 0,30 a rolagem de frenagem alonga ≈65 m e a distância estoura a
      pista por ≈5 m. Não é uma regressão de projeto: o pouso na grama
      nunca coubera nos 600 m — o modelo é que não estava olhando.
    - O caso massa-total ampliado não produz nenhum flip novo (as
      margens nominais que já absorviam a perturbação ±σ direcional
      desde a v4.6 também absorvem a perturbação ×(1+σ) do massa-total,
      no mundo de desempenho JÁ verificado desde a v4.7).
    - `validation_status` continua `"FAIL"`, agora com **4** violações:
      as 3 nominais de sempre (dois cenários de envelope + carga de
      nariz máxima, ver entrada "Ciclo 3" acima, inalteradas em texto e
      valor) MAIS `"Pouso (grama, 15 m): 605 m excede a pista disponível
      de 600 m"`.
    Ver `tests/cli.rs`, `src/validation/constraint_checker.rs` (mod
    tests) e `src/validation/robustness.rs` (mod tests).
- **v5.0** (Task 2, ciclo7-clmax-decolagem — bump **MAJOR**, não MINOR):
  a Task 1 do mesmo ciclo RENOMEOU um campo já serializado,
  `[stability].to_flap_cm_fraction` → `to_flap_fraction`, ecoado em
  `trim.to_flap_fraction` (ver bloco `trim` §4 abaixo). Pela própria
  política de bump declarada acima (§1: "renomeia ou remove um campo
  existente" é MAJOR), isso não é uma mudança aditiva — um consumidor lendo
  `trim.to_flap_cm_fraction` do JSON v4.8 não encontra mais essa chave no
  v5.0. Duas mudanças de conteúdo:
  1. **Campo NOVO `wing.cl_max_to`** (f64) — CL_max em configuração de
     DECOLAGEM (flap PARCIAL), derivado por interpolação linear entre
     `cl_max_clean` e `cl_max_flaps` (não ecoado diretamente; `wing.cl_max`
     é o mesmo valor de pouso) pela MESMA `trim.to_flap_fraction` que já
     sinalizava o ΔCm da rotação: `cl_max_to = cl_max_clean +
     to_flap_fraction·(cl_max_flaps − cl_max_clean)`. Consumido pela Vr/VS0
     da ROTAÇÃO (`trim`) e pelas quatro distâncias de DECOLAGEM
     (`performance.to_distance_paved_m`/`to_distance_grass_m`/
     `to_50ft_paved_m`/`to_50ft_grass_m`, calculadas internamente por
     `agents::performance::takeoff_distance_m`/`takeoff_distance_50ft_m`) —
     antes essas grandezas derivavam do
     `cl_max` de POUSO (flap CHEIO), fisicamente incoerente (ninguém decola
     com flap de pouso) e otimista. Isoladamente esta mudança seria
     ADITIVA; é o renome abaixo que força o MAJOR.
  2. **`trim.to_flap_fraction`** (RENOMEADO de `to_flap_cm_fraction`) —
     mesmo significado físico e VALOR, agora com papel DUPLO: além do ΔCm
     de rotação (como antes), também governa `wing.cl_max_to` acima.
  - **Migração de CONFIGURAÇÃO** (`aircraft.toml`, não deste schema JSON):
    `[stability].to_flap_cm_fraction` foi RENOMEADO para
    `[stability].to_flap_fraction` — TOMLs antigos com o nome antigo falham
    o parse (`missing field: to_flap_fraction`, mesmo padrão sem erro de
    migração dedicado das v4.3/v4.4/v4.8). Ver
    `config/aircraft/baseline_4seat.toml` para o valor de referência.
  - **Achado honesto do baseline real** (consequência FÍSICA da Task 1 —
    rotação e distâncias de decolagem usando o CL_max de DECOLAGEM correto
    em vez do de POUSO): `trim.rotation_limit_pct_mac` (limite dianteiro de
    rotação, número único então invariante ao peso) recua de **12,995% para
    8,908% MAC** — a Vr correta é MAIOR (menos CL_max disponível na
    rotação), logo há MAIS autoridade de profundor disponível; o modelo
    anterior era pessimista, não o contrário. O espelho honesto desse
    ganho: as distâncias de DECOLAGEM alongam (`to_50ft_grass_m` 428,2 →
    457,7 m; `to_50ft_paved_m` 381,4 → 406,9 m) — o modelo anterior era
    otimista na decolagem, não o contrário; a decolagem na grama continua
    PASSANDO nos 600 m de pista disponível, com folga de 142 m.
    `validation_status`
    PERMANECE `"FAIL"` com as MESMAS **4** violações em CONTAGEM
    (inalterado desde a v4.8), mas DUAS trocam de NATUREZA: os cenários
    'Solo (piloto)' e '2 pax dianteiros', que na v4.8 violavam o limite
    dianteiro de rotação DIRETAMENTE como parte do envelope de CG NOMINAL,
    agora ficam DENTRO do envelope nominal (o limite recuou o bastante para
    os dois) e passam a disparar a checagem #19 de ROBUSTEZ
    (`robustness.flips`) — reprovam sob o caso adversarial dianteiro (±15%
    de massa estrutural) contra o MESMO limite de 8,908% MAC. O achado
    físico não desaparece, muda de categoria (violação nominal → flip de
    robustez). As outras duas violações da v4.8 (carga de nariz máxima
    28,6%/25,0% e pouso na grama 605 m/600 m) permanecem bit a bit
    INALTERADAS — a Task 1 não toca pouso, VS0/VS1 nem subida. Ver
    `tests/schema_v4.rs`, `tests/cli.rs` e `src/agents/trim_authority.rs`.
  - **NOTA DE ESTADO (campanha E10, 2026-08-08) — o schema NÃO muda, o
    baseline sim.** As entradas v4.9 e v5.0 acima descrevem o
    `validation_status: "FAIL"` do baseline VIGENTE NAQUELAS versões de
    schema; ficam como registro histórico e não são reescritas. A campanha
    E10 é uma mudança puramente de DADOS (`config/aircraft/
    baseline_4seat.toml`: bateria híbrida 28→53 kg a 7,80 m, `x_nose_m`
    1,40→1,30, `h_cg_ground_m` 1,05→0,92, pernas 0,67/0,53→0,54/0,40,
    hélice Ø1,95→1,76 m, `cl_max_flaps` 1,72→2,1, `to_flap_fraction`
    0,5→0,35) — nenhum campo do JSON foi adicionado, removido ou renomeado,
    logo **`schema_version` permanece `"5.0"`**. O que muda é o VALOR de
    `validation_status`, que passa a **`"PASS"` com `violations` VAZIO e
    `robustness.flips` VAZIO** — o primeiro PASS completo do projeto sob os
    24 checks mais os 3 mundos de robustez. As quatro violações citadas
    acima fecharam assim: carga de nariz 28,6%→22,77% (teto 25%); pouso na
    grama 605,0→556,7 m (pista 600 m); e os dois flips de robustez ('Solo
    (piloto)', '2 pax dianteiros') sumiram porque o CG desses cenários
    recuou de 9,1%/12,5% para 17,9%/20,5% MAC contra um limite de rotação
    praticamente parado (8,908%→8,533% MAC). O aviso ELÉTRICO de pico
    (1.260 W > 900 W) permanece — é aviso, não violação. Custos honestos
    registrados nos pins: SM mínima 16,25%→9,68% (piso 5%), margem de
    combustível 14,33%→9,14% da capacidade (piso 5%), cruzeiro
    302,1→300,2 km/h, autonomia informativa 7,71→7,23 h, decolagem na grama
    457,7→469,3 m. Ver `aircraft_spec.json`, `tests/cli.rs` e
    `tests/gear_tipback.rs`.
- **v5.1** (Task 3, ciclo8-flap-e-solo — bump **MINOR**): formaliza dois
  campos ADITIVOS já serializados desde as Tasks 1/2 do mesmo ciclo (a
  entrada v5.0/NOTA DE ESTADO acima é sobre a campanha E10, um bump
  ANTERIOR e não relacionado — este é o bump que fecha o ciclo 8).
  Nenhum campo existente foi renomeado/removido nem mudou de tipo/unidade
  — consumidores v5.0 continuam funcionando sem alteração.
  1. **Campo NOVO `wing.cd0_flap_to_extra`** (f64, Task 1, §1 do ciclo) —
     ver tabela do bloco `wing` abaixo.
  2. **Campo NOVO `propeller.prop_clearance_critical_m`** (f64, m, Task 2,
     §3 do ciclo) — ver tabela do bloco `propeller` abaixo. Acompanhado da
     **checagem NOVA #25** em `ConstraintChecker::verify` (reprova quando
     `prop_clearance_critical_m <= 0.0`) — mudança ADITIVA em
     comportamento (só pode ADICIONAR violações, nunca remover as
     existentes).
  - **Migração de CONFIGURAÇÃO** (`aircraft.toml`, não deste schema
    JSON, Task 1/2): `[wing].cd0_flap_delta` (faixa (0.005, 0.05),
    baseline 0,015) e `[gear].tire_deflation_delta_m` (faixa
    (0.03, 0.15) m, baseline 0,08) são campos NOVOS **obrigatórios** —
    TOMLs antigos sem esses campos falham o parse (`missing field`),
    mesmo padrão sem erro de migração dedicado das v4.3/v4.4/v4.8. Ver
    `config/aircraft/baseline_4seat.toml` para os valores de referência.
  - **Achado honesto consolidado do baseline real E10** (consequência
    FÍSICA das Tasks 1-2, não deste bump em si):
    - §1-§2 (arrasto de flap + gradiente CS 23.65 honesto):
      `performance.climb_gradient_pct` recua de **15,129850% para
      13,896713%** (−1,233137 p.p.), decomposto em dois efeitos isolados
      por medição direta: **~72%** (−0,888093 p.p.) vem do deslocamento
      do PONTO de avaliação (referência de estol `wing.cl_max` de pouso →
      `wing.cl_max_to` de decolagem parcial); **~28%** (−0,345045 p.p.)
      vem do arrasto extra do flap (`cd0_flap_to_extra`) somado à polar
      nesse ponto. `performance.vx_kmh` sobe **+11,89%**
      (108,609→121,520 km/h). `performance.to_50ft_paved_m`/
      `to_50ft_grass_m` alongam **~+1%** (420,47/473,58 m — só o segmento
      de SUBIDA da decolagem consome a polar nova; rolagem de solo e
      aproximação de pouso permanecem bit-a-bit INALTERADAS). O gradiente
      continua acima do piso CS 23.65 (8,3%), folga ~5,6 p.p.
    - **Viés remanescente NOMEADO** (achado da revisão da Task 1,
      PRÉ-EXISTENTE ao ciclo 8, não introduzido por ele —
      `best_climb_angle_ms` devolve o PISO da varredura de velocidade
      (1,05·V_s_to), não um máximo interior; à referência típica da CS
      23.65 (≥1,2·V_s) o gradiente do baseline real seria ≈12,4486%, não
      os 13,896713% retornados — ~1,45 p.p. de viés OTIMISTA
      remanescente). Não corrigido nesta task (fora de escopo por
      instrução explícita do brief); ver `fidelity.performance` e a
      docstring de `agents::performance::best_climb_angle_ms`.
    - §3-§4 (folga crítica CS 23.925 + pin de rotação):
      `propeller.prop_clearance_critical_m` ≈ **+0,0325 m** — checagem
      #25 **PASSA** (folga positiva). `trim.rotation_limit_pct_mac`
      recentrado em `8,533% ± 0,05%` (era `8,908% ± 1,5%` desde o ciclo
      7, dívida de cobertura reapertada nesta task).
    - **CAVEAT NOMEADO** (achado de review desta revisão final, NÃO
      corrigido — fora de escopo): `prop_clearance_critical_m` modela o
      colapso do trem de nariz como TRANSLAÇÃO VERTICAL 1:1 — simplificação
      OTIMISTA, pois a célula real PIVOTA sobre o trem PRINCIPAL nesse
      evento, e a hélice (à frente do trem de nariz) mergulha um braço
      amplificado por `(x_main − x_prop)/(x_main − x_nose)` ≈ 1,4–1,55×,
      não 1:1. Sob a transferência de atitude correta, a folga crítica real
      do E10 é plausivelmente **NEGATIVA** (≈ −0,05 a −0,08 m, não os
      +0,0325 m acima) — a checagem #25 pode estar aprovando um FAIL
      honesto. Ver docstring de `PropellerSpec::prop_clearance_critical_m`
      e `docs/backlog.md` ("transferência de atitude do #25") — nomeado
      como item de ciclo futuro.
    - `validation_status` do baseline real **PERMANECE `"PASS"`** com
      `violations` VAZIO e `robustness.flips` VAZIO — mesmo veredito da
      campanha E10 (v5.0), sem nenhum flip novo introduzido pelas
      Tasks 1-2 deste ciclo. Ver `aircraft_spec.json`, `tests/schema_v4.rs`
      e `tests/generic_engine.rs`.
- **v5.2** (Task 2, ciclo9-transferencia-atitude, 2026-08-09 — bump
  **MINOR**): nenhum campo do JSON de saída foi renomeado/removido/mudou de
  tipo — consumidores v5.1 continuam funcionando sem alteração. O bump é
  sobre SEMÂNTICA: `propeller.prop_clearance_critical_m` **mantém o nome**,
  mas a FÓRMULA que o preenche mudou (Task 1 do mesmo ciclo, `48a2ed4`) e o
  veredito honesto do baseline real virou de PASS para FAIL — grande o
  bastante para merecer o bump MINOR mesmo sem quebra de contrato
  estrutural.
  - **CAVEAT NOMEADO na v5.1 RESOLVIDO**: campo de CONFIGURAÇÃO NOVO
    `[propeller].prop_plane_x_m` (posição do plano da hélice, m do datum no
    nariz — ESTIMATIVA, validar no CAD; input, **não ecoado** no JSON de
    saída) alimenta o fator de amplificação do pivô descrito no caveat.
    `PropellerSpec::fill_critical_clearance` ganha um terceiro parâmetro
    (`prop_cfg: &PropellerCfg`) para lê-lo.
  - **Migração de CONFIGURAÇÃO** (`aircraft.toml`, não deste schema JSON,
    Task 1): `[propeller].prop_plane_x_m` (faixa (0,0, 1,0) m, baseline
    0,20) é campo NOVO **obrigatório**, SEM default — TOMLs pré-5.2 (sem
    esse campo) falham o parse (`missing field`), mesmo padrão sem erro de
    migração dedicado das v4.3/v4.4/v4.8/v5.1. Ver
    `config/aircraft/baseline_4seat.toml` para o valor de referência.
  - **Achado confirmado**: no baseline E10 real, fator =
    (3,66−0,20)/(3,66−1,30) ≈ **1,46610**; `prop_clearance_critical_m` vai
    de **+0,0325 m (PASS) para ≈−0,06416 m (FAIL)** — exatamente a faixa
    que o caveat previu. `validation_status` do baseline real vira
    **`"FAIL"`** com **exatamente 1 violação nomeada** (checagem #25) —
    tipback/tail-strike/carga de nariz/margem de combustível/pista/
    robustez continuam PASSANDO com os MESMOS números da campanha E10
    (nenhum deles depende de `prop_clearance_critical_m`). Ver
    `docs/backlog.md` (item 1, marcado RESOLVIDO),
    `tests/cli.rs`/`tests/gear_tipback.rs`/`tests/schema_v4.rs` para os
    pins honestos completos.
  - **Task 1 de ciclo10-sag-e-linha-de-tracao (2026-08-09, ainda dentro da
    v5.2 — bump formal fica para a Task 3 do mesmo ciclo)**: CAVEAT dos
    mains rígidos nomeado logo acima **RESOLVIDO** — `[gear].
    h_cg_ground_m` sempre foi a altura do CG com a aeronave CARREGADA, em
    deflexão ESTÁTICA (não "trem estendido sem carga"), então os mains
    JÁ estão nessa deflexão dentro de `ground_clearance_m`; CS 23.925 pela
    LETRA só exige o trem CRÍTICO (nariz) no batente, não os mains
    simultaneamente. Não havia condição composta não modelada. Campo de
    CONFIGURAÇÃO NOVO `[gear].static_sag_fraction` (faixa (0,15, 0,55) m,
    baseline 0,33) é campo NOVO **obrigatório**, SEM default — TOMLs
    pré-task falham o parse (`missing field`). Corrige o curso do nariz
    usado na fórmula de TOTAL para RESTANTE
    (`nose_oleo_stroke_mm × (1 − static_sag_fraction)`), já que o
    amortecedor de nariz também PARTE da deflexão estática — a fórmula
    anterior contava essa compressão DUAS VEZES. Fator geométrico
    inalterado (≈1,46610); `prop_clearance_critical_m` vai de
    **≈−0,06416 m para ≈−0,00249 m** — MESMO veredito (`validation_status`
    continua `"FAIL"`, mesma 1 violação nomeada), honestamente
    ANTI-conservador (a correção AUMENTA a folga calculada), mas fiel à
    letra da norma. Ver `docs/backlog.md` (item 6, RESOLVIDO),
    `tests/cli.rs`/`tests/gear_tipback.rs`/`tests/schema_v4.rs` para os
    pins honestos completos.
- **v5.3** (Task 3, ciclo10-sag-e-linha-de-tracao, 2026-08-09 — bump
  **MINOR**, exceção registrada, mesmo padrão da v5.2): formaliza o bump
  que as Tasks 1/2 do mesmo ciclo já haviam anunciado como "ainda dentro
  da v5.2" (ver notas de exceção em §1 acima e o sub-item "Task 1 de
  ciclo10..." logo acima, sob v5.2). Três mudanças de conteúdo, nenhuma
  nova nesta task — só formalizadas e documentadas:
  1. **`propeller.prop_clearance_critical_m` mudou de fórmula DE NOVO**
     (Task 1, `6c34f8f` — já detalhado no sub-item "Task 1 de
     ciclo10-sag-e-linha-de-tracao" sob v5.2 acima): curso do amortecedor
     de nariz de TOTAL para RESTANTE, campo de CONFIGURAÇÃO NOVO
     `[gear].static_sag_fraction`. Baseline real E10: **≈−0,06416 m →
     ≈−0,00249 m** — MESMO veredito (checagem #25 continua `FAIL`).
  2. **Física nova do momento da linha de tração** (Task 2, `79b2263` +
     erratum `713e846` + `f9231ea`): o balanço de momentos da rotação
     ganha o termo `−T(Vr)·prop_axis_above_cg_m` (braço sobre o CG — ver
     ERRATUM da spec, `docs/superpowers/specs/
     2026-08-09-ciclo10-sag-e-linha-de-tracao-design.md` §2: a corrida de
     decolagem é ACELERADA, o termo de d'Alembert cancela a porção
     `h_cg`, termos de solo `μN·h_cg`/`D·(h_cg−h_D)` permanecem
     DESPREZADOS e documentados, ≲2 pp, direção anti-conservadora —
     **`old→new`, ciclo 12 task 4**: essa estimativa de magnitude estava
     ERRADA, a medição real deu ≈4,40 pp, mais que o dobro; os termos foram
     IMPLEMENTADOS nesta task, ver v5.5 abaixo e §3/§4 (bloco `trim`)); o trim
     de cruzeiro ganha `cm_thrust = −T_cruzeiro·prop_axis_above_cg_m/
     (q·S_w·MAC)` somado ao `cm_ac` (aproximação `z_D = 0` documentada,
     conservadora em até ~50% no pior caso plausível). Nenhum campo novo
     — `trim.rotation_limit_pct_mac`, `trim.cl_h_trim_cruise`,
     `trim.cd_trim` e `weight.cg_limit_fwd_pct_mac` mantêm nome/tipo/
     unidade, só o VALOR muda. **Mudança de contrato adicional**:
     `trim.rotation_limit_pct_mac` deixa de ser invariante ao peso —
     agora é a envoltória MÁXIMA sobre os cenários (ver §4, bloco
     `trim`). Baseline real: `rotation_limit_pct_mac` **8,533% →
     13,355% MAC** (+4,82 pp); `validation_status` continua `"FAIL"` com
     a MESMA 1 violação (#25, inalterada por esta mudança) e ZERO flips
     de robustez — o que encolhe é a FOLGA de rotação do cenário mais
     apertado ("Solo (piloto)", de +21,6% para +10,5%; todos os cenários
     permanecem positivos).
  3. **Campo NOVO `robustness.flips[].limite_nominal`** (f64) — este SIM
     genuinamente ADITIVO (ver tabela `RobustnessFlip` em §4 abaixo). O
     limite NOMINAL do mesmo check, ao lado de `limite` (o limite
     efetivamente aplicado ao mundo perturbado) — necessário porque a
     mudança #2 acima fez o limite dianteiro de rotação deixar de ser
     invariante à massa, então dois mundos adversariais podem ter
     `limite` diferentes entre si e do nominal. `limite_nominal ==
     limite` para checks de régua invariante (tipback, carga de nariz,
     gates de desempenho/pista); `limite_nominal != limite` sinaliza "a
     régua andou", não só o CG do mundo perturbado.
  - Mesma exceção MINOR da v5.2 aplicada aos itens 1-2: correção de bug de
    modelagem física (dupla contagem / termo de momento faltante), não
    mudança de CONTRATO de tipo/estrutura — nome/tipo/unidade dos campos
    afetados são idênticos aos da v5.2. Registrado aqui para não esconder
    a divergência entre a letra da política (§1 acima) e a decisão real.
  - **Migração de CONFIGURAÇÃO** (`aircraft.toml`, não deste schema JSON,
    Task 1): `[gear].static_sag_fraction` (faixa (0,15, 0,55), baseline
    0,33) é campo NOVO **obrigatório**, SEM default — TOMLs pré-5.3 sem
    esse campo falham o parse (`missing field`), mesmo padrão sem erro de
    migração dedicado das versões anteriores. Ver
    `config/aircraft/baseline_4seat.toml` para o valor de referência.
  - Nenhuma tolerância de teste foi afrouxada em nenhuma das três
    mudanças — só pins re-centrados old→new com a MESMA tolerância. Ver
    `tests/cli.rs`/`tests/gear_tipback.rs`/`tests/schema_v4.rs`/
    `tests/generic_engine.rs` para os pins honestos completos.
- **v5.4** (Task 3, ciclo11-subida-honesta, 2026-08-10 — bump **MINOR**,
  exceção registrada, mesmo padrão da v5.2/v5.3): nenhum campo do JSON de
  saída foi renomeado/removido/mudou de tipo/unidade — consumidores v5.3
  continuam funcionando sem alteração. O bump é sobre serialização de um caso
  extremo já possível em v5.3 mas nunca acionado no baseline real:
  `performance.to_50ft_paved_m` e `performance.to_50ft_grass_m` podem receber
  legitimamente `f64::INFINITY` quando o obstáculo de 15m é inatingível (razão
  de subida ≤ 0 no segmento de subida — ver `agents::performance::
  takeoff_distance_50ft_m`, ramo `rc <= 0.0`). Antes desta task, serde_json
  convertiria silenciosamente para `null` (RFC 8259 não tem representação de
  infinito), quebrando o round-trip — um consumidor desserializando `null`
  falharia em conversão para `f64`. Ambos os campos agora usam
  `#[serde(with = "fatigue_life_serde")]` (módulo existente desde Task 6.1 do
  ciclo 8 para tratar `StructuralSpec::fatigue_life_cycles`), que serializa o
  infinito como a string `"infinita"` (documentado em §5 abaixo). Política de
  bump: é correção de SEMÂNTICA de serialização (mesmos nomes/tipos, só o
  efeito colateral da conversão muda — de `null` silencioso para `"infinita"`
  explícita), não mudança de CONTRATO de tipo/estrutura, aplicada como
  exceção MINOR (mesmo padrão aprovado em v5.2 para `prop_clearance_critical_m`
  e v5.3 para `RobustnessFlip`).
  - Campanha ciclo 11 (2026-08-10): formaliza o bump que as Tasks 1/2 do mesmo
    ciclo já haviam anunciado como pendente (backlog itens 2/3/5/7 — ver
    `docs/backlog.md`). Nenhuma tolerância de teste foi afrouxada — só pins
    re-centrados old→new com a MESMA tolerância (ciclo 11 task 1: piso da
    varredura de gradiente sobe de 1,05·Vs — que RC/V decrescente devolvia
    como viés OTIMISTA — para 1,20·Vs, referência típica da CS 23.65
    (≥1,2·Vs1) — `climb_gradient_pct` **13,896713% → 12,451842%**; `vx_kmh`
    **121,519501 → 138,871480** (+14,28%); ciclo 11 task 2: Vy referência
    `cl_max_clean` com janela de busca [1,05;2,00]·Vs — `vy_kmh`
    **147,915721 → 148,435393**, `rc_sl_ms` **4,999902 → 4,999905**,
    `service_ceiling_m` 5.200 inalterado; ciclo 10 fix wave: textos de linha de
    tração, item 7). Ver `tests/schema_v4.rs`/`tests/generic_engine.rs` para os
    pins honestos completos.
- **v5.5** (Task 1, ciclo12-solo-honesto, 2026-08-15 — bump **MINOR**,
  exceção registrada, mesmo padrão da v5.2/v5.3/v5.4): nenhum campo do JSON
  de saída foi renomeado/removido/mudou de tipo/unidade — consumidores v5.4
  continuam funcionando sem alteração. O bump é sobre serialização de um
  caso extremo que passou a ser ATINGÍVEL nesta task: `performance.
  to_distance_paved_m`, `performance.to_distance_grass_m` e
  `performance.landing_distance_m` podem receber `f64::INFINITY` quando a
  rolagem integrada (ver Task 2/3 abaixo) não consegue acelerar/desacelerar
  dentro da distância (tração ou frenagem insuficientes). Mesmo tratamento
  de `to_50ft_paved_m`/`to_50ft_grass_m` desde a v5.4:
  `#[serde(with = "fatigue_life_serde")]` serializa o infinito como a
  string `"infinita"`, nunca `null` (ver §5 abaixo, agora com SEIS campos).
  Política de bump: mesma exceção MINOR das versões anteriores (correção de
  semântica de serialização, não de contrato de tipo/estrutura).
  - **Campanha ciclo 12 (tasks 2/3, 2026-08-15, fecha `docs/backlog.md`
    item 4)**: a mudança de CONTEÚDO que torna o caso `+INFINITY` acima
    genuinamente alcançável — `takeoff_ground_roll_m`/`landing_ground_roll_m`
    passam do método ENERGÉTICO fechado de Raymer (`V_ref²/2gμ`, sem termo
    de arrasto por construção) para integração numérica da equação de
    movimento em V (Simpson composto, 200 intervalos), consumindo a polar
    completa (CD0 + trem estendido + incremento de flap + induzido)
    segmento a segmento. Valores MEDIDOS old→new no baseline real
    (`aircraft_spec.json`, commit pré-ciclo-12 `1e11998` vs HEAD):
    `to_50ft_paved_m` **420,372451 → 651,258408 m** (+54,92%),
    `to_50ft_grass_m` **473,469470 → 819,110978 m** (+73,00%),
    `to_distance_paved_m` **398,227641 → 744,556577 m** (+86,97%),
    `to_distance_grass_m` **477,873169 → 996,335432 m** (+108,49%),
    `ldg_50ft_m` **502,458299 → 582,341118 m** (+15,90%),
    `ldg_50ft_grass_m` **556,677173 → 646,437301 m** (+16,12%),
    `landing_distance_m` **362,656622 → 442,539441 m** (+22,03%).
    Consequência de gate: `to_50ft_grass_m`/`ldg_50ft_grass_m` passam a
    EXCEDER a pista de 600 m das checagens #23/#24 —
    `validation_status` do baseline real vira `"FAIL"` com 4 violações
    (2 de robustez de massa estrutural + #23 + #24). Não é regressão — é o
    modelo pagando arrasto que sempre esteve fisicamente presente.
    `old→new` (proveniência corrigida, fix wave ciclo 12): esta entrada
    dizia que as 2 violações de robustez já eram "nomeadas antes deste
    ciclo" — FALSO, medido. `git show 1e11998:aircraft_spec.json`
    (commit pré-ciclo-12) tem `validation_status: "PASS"`, `violations: []`,
    `robustness.flips: []` — nenhuma violação de robustez existia antes
    deste ciclo. As 2 violações de robustez (cenários 'Solo (piloto)' e
    '2 pax dianteiros') são causadas pela TASK 4 (termos de solo do
    balanço de rotação — ver task 4 abaixo), são NOVAS deste ciclo, e não
    têm relação com o método de rolagem por integração (tasks 2/3, que
    causam exclusivamente #23/#24). Tolerâncias de teste INALTERADAS — só
    pins re-centrados old→new. Ver `docs/backlog.md` item 4 (RESOLVIDO) e
    `docs/superpowers/specs/2026-08-15-ciclo12-solo-honesto-design.md`
    (§2, §3, §5, §9 tabela congelada, §10 veredito).
  - **Campanha ciclo 12 (task 4)**: o balanço de momentos da ROTAÇÃO
    (`trim.rotation_limit_pct_mac`) ganha os termos de SOLO
    (`−μ_roll·N·h_cg − D·(h_cg−z_drag_above_cg_m)`) que a v5.3 havia
    deixado deliberadamente de fora com uma estimativa de magnitude
    (`≲2 pp de MAC`) que a medição real desta task DESMENTIU — ver
    correção `old→new` em §3 e §4 (bloco `trim`) abaixo. Nenhum campo novo
    de schema — mesmo padrão de bump MINOR por correção de física da v5.3.
    Campos de CONFIGURAÇÃO novos (`aircraft.toml`, não deste schema JSON,
    ambos introduzidos na task 2 deste ciclo): `[performance].mu_roll_paved`/
    `mu_roll_grass` (atrito de rolagem, consumidos pela rolagem integrada E,
    desde a task 4, pelo termo de solo da rotação) e `[wing].
    z_drag_above_cg_m` (default 0,0, task 4) — ver
    `config/aircraft/baseline_4seat.toml`.
  - Nenhuma tolerância de teste foi afrouxada em nenhuma das mudanças
    acima — só pins re-centrados old→new com a MESMA tolerância. Ver
    `tests/generic_engine.rs` para os pins honestos completos.
- **v5.6** (ciclo13-tracao-unificada, 2026-08-15 — bump **MINOR**, DUAS
  exceções registradas, mesmo padrão da v5.2/v5.3/v5.4/v5.5). Fecha
  `docs/backlog.md` #8 (unificar o modelo de tração), #9 (`prop_efficiency`
  com η(0)=0,58 e janela nula), #15 (PRIORIDADE ALTA — inconsistência de
  tração no balanço de rotação) e #16 (assimetria de superfície da
  rotação).
  - **Campos ADICIONADOS (MINOR puro)**: `trim.rotation_limit_pct_mac_paved`
    e `trim.rotation_limit_pct_mac_grass` (ambos f64, %MAC) — o limite
    dianteiro de rotação calculado nas DUAS superfícies (ver §4 abaixo).
  - **Exceção registrada 1 — mudança de ORIGEM sem mudança de tipo**:
    `propulsion.prop_efficiency` mantém nome, tipo (f64) e faixa, mas deixa
    de vir do polinômio JavaProp `η = −0,15·J²+0,39·J+0,58` (APAGADO — via
    `agents::propulsion::prop_efficiency`) e passa a ser DERIVADO por
    inversão em forma fechada da lei única de tração `T(V) =
    FoM(J)·T_ideal_momentum(V, P_eixo)` (`η = FoM(J)·V/u`, spec ciclo13
    §5). No baseline real o valor é **idêntico por construção da âncora**
    (`fom_design` foi retro-derivada exatamente para reproduzir
    `η_poly(j_design)` no ponto de cruzeiro) — `prop_efficiency`
    **0,7838814965676598 → 0,7838814965676598** (inalterado, guarda de
    regressão `eficiencia_de_cruzeiro_reproduz_a_ancora_do_polinomio_
    apagado` em `tests/generic_engine.rs`), então NENHUM consumidor de JSON
    quebra. A mudança é de PROVENIÊNCIA, não de contrato.
  - **Exceção registrada 2 — mudança de VALOR e de SIGNIFICADO**:
    `trim.rotation_limit_pct_mac` (campo legado, nome/tipo/unidade
    inalterados) passa a valer a superfície de OPERAÇÃO — **GRAMA**, a
    mesma premissa que as checagens #23/#24 (decolagem/pouso) já usam —
    em vez de PAVIMENTADO. Medido no baseline real: pavimentado
    **16,380458137686837%** → grama **18,268251143882534%** MAC
    (+1,887793 pp). Ver §4 (bloco `trim`) para a derivação completa.
  - **Mecanismo físico (spec ciclo13 §1/§2)**: os dois modelos de tração
    que o projeto mantinha desde sempre — `agents::performance::
    thrust_ground_roll_n` (quantidade de movimento com velocidade de
    avanço, usado só na rolagem) e `agents::performance::
    thrust_available_n` (disco atuador estático + polinômio
    `prop_efficiency(J)`, usado em cruzeiro/subida/teto/rotação) —
    divergiam **27,69%** em `V_LOF` (backlog #8). O polinômio apagado
    violava o teto físico de conservação de quantidade de movimento
    (`T_real ≤ T_ideal`) em **4 dos 8** pontos de operação medidos do
    baseline, **2** deles alimentando gates que PASSAVAM (`Vx` → gradiente
    CS 23.65; `V_LOF` → balanço de rotação — backlog #9). Os dois modelos
    foram fundidos numa lei única `T(V) = FoM(J)·T_ideal_momentum(V,
    P_eixo)`, ancorada na tração estática de McCormick (`fom_static=0,75`,
    J=0) e na eficiência de cruzeiro do polinômio apagado
    (`fom_design=0,81597699924588796`, convergida por ponto fixo — ver
    ERRATUM da spec §3.2.1 —, J=`j_design=1,87514348025711675`). O
    resíduo de d'Alembert do balanço de rotação (backlog #15,
    PRIORIDADE ALTA) — `(T_solo − T_momento)·z_eixo`, que chegava a
    **−1.005,97 N·m (−6,816 pp de MAC)** no cenário governante — vai a
    **ZERO por construção**, medido a **1e-12 relativo** nos 6 cenários de
    CG.
  - **Migração de CONFIGURAÇÃO** (`aircraft.toml`, não deste schema JSON):
    `[performance].static_thrust_factor` foi REMOVIDO e substituído por
    `[propeller].fom_static`/`fom_design`/`j_design` — carregar um TOML com
    o campo antigo produz erro de migração NOMEADO (`check_static_thrust_
    factor_migration`), não default silencioso.
  - **Consequência de veredito** (não é regressão do modelo — é o
    polinômio deixando de mascarar tração fisicamente impossível):
    `climb_gradient_pct` cai de **12,451842% para 7,913277%** e FLIPA de
    PASS para FAIL contra o mínimo de 8,3% (CS 23.65); a superfície de
    grama na rotação faz o cenário 'Solo (piloto)' cruzar para violação
    NOMINAL de envelope de CG. `validation_status` continua `"FAIL"`, com
    **5** violações (era 4 antes deste ciclo). Ver `docs/backlog.md` #8/#9/
    #15/#16 (RESOLVIDOS, com a medição completa) e spec
    `docs/superpowers/specs/2026-08-15-ciclo13-tracao-unificada-design.md`
    §1/§2/§3/§6/§7/§9/§11.1.
  - Nenhuma tolerância de teste foi afrouxada — só pins re-centrados
    old→new com a MESMA tolerância. Ver `tests/generic_engine.rs` para os
    pins honestos completos.

## 2. Convenção de eixos e unidades

- **Sistema de unidades**: SI em todo o documento, com DUAS exceções
  historicamente mantidas por legibilidade doméstica (não SI):
  velocidades em **km/h** (não m/s) e algumas dimensões de trem de pouso em
  **mm**/**psi** (ver bloco `landing_gear`). Cada campo abaixo declara sua
  unidade explicitamente — não assuma SI cegamente.
- **Datum (origem)**: ponta do nariz da aeronave (x = 0).
- **Eixo x**: positivo para TRÁS (cauda). Todas as posições longitudinais
  do relatório (`wing_le_root_x_m`, `mac_le_x_m`, braços de empenagem
  somados ao CA da asa, etc.) são medidas nesta convenção.
- **Eixo y**: lateral, usado só implicitamente em grandezas de envergadura
  (`span_m`, `y_mac_m`) — medido a partir da linha de centro da aeronave
  (semi-envergadura) para superfícies espelhadas, ou da raiz para
  superfícies únicas (ver nota especial no bloco `control_surfaces`).
- **Alturas** (`spar_height_root_m`, folgas de solo do bloco `propeller`,
  etc.): medidas a partir do solo/plano de referência do trem de pouso
  estendido — não do datum do nariz.
- **Ângulos**: graus (`deg`), não radianos, em todos os campos de ângulo do
  JSON (o código Rust interno usa radianos em vários pontos de cálculo,
  mas converte para graus antes de popular o relatório).

## 3. Nível de confiança (`fidelity`)

`fidelity` é um mapa `{ "nome_do_bloco": "descrição de fidelidade" }` —
chave = mesmo nome do bloco de topo do JSON (`"wing"`, `"structure"`,
etc.). Cada valor começa com um destes três rótulos:

| Rótulo | Significado |
|---|---|
| `computed` | Equações fechadas ou por segmentos, sem correlação empírica externa (dado o modelo físico assumido, o resultado é determinístico e rastreável até a fórmula). |
| `semi-empirical` | Combina equações com curvas/correlações de catálogo ou de literatura aeronáutica (Raymer, Gudmundsson) — não é first-principles puro. |
| `preliminary` | Estimativa simplificada de projeto conceitual/preliminar — **requer análise posterior** antes de fabricação (FEM, GVT, VLM/CFD, ensaio, conforme o bloco). |

**O time de CAD deve tratar todo bloco marcado `preliminary` como
NÃO liberado para fabricação sem uma etapa de validação adicional** — a
tabela abaixo lista o tipo de análise esperada por bloco. Simplificações
de modelo conhecidas e conscientemente NÃO corrigidas (achados de review
nomeados, não silenciados) estão consolidadas em `docs/backlog.md`, com
ponteiro para a docstring/campo onde cada uma está documentada em detalhe.

| Bloco | Fidelidade típica (Task 6.1) | Análise posterior recomendada se `preliminary` |
|---|---|---|
| `wing` | semi-empirical (polar por build-up: CD0 por componente + Oswald empírico) | — |
| `propulsion` | semi-empirical (curvas de catálogo do motor + BSFC paramétrico); **ciclo 13 (`old→new`, fecha `docs/backlog.md` #8/#9)**: `prop_efficiency` deixa de ser lido de um polinômio JavaProp calibrado só em J de cruzeiro (`η(0)=0,58` fisicamente errado, salto de 84.843,5 N em V=1,0 m/s) e passa a ser DERIVADO por inversão fechada da lei única de tração `T(V)=FoM(J)·T_ideal_momentum(V,P_eixo)` — mesmo valor no baseline por construção da âncora, mas a origem muda (ver v5.6 em §1) | — |
| `geometry` | computed (derivado da configuração + `WeightBalanceAgent`) | — |
| `empennage` | preliminary (coeficiente de volume, Raymer Tab. 6.4) | VLM/CFD para eficiência real de downwash/sidewash |
| `control_surfaces` | preliminary (frações históricas, Raymer Tab. 6.5) | Análise de autoridade/eficiência de controle |
| `weight` | **v4.5**: semi-empirical (estruturas: Raymer 15.2 GA × fatores de composto Tab. 15.4; hardware: itens configurados NÃO pesados) | Pesagem em balança de cada item antes da fabricação — as 7 massas estruturais (`weight.structural_masses`) vêm de equações semi-empíricas de componente, mas o hardware/instalação (aviônicos, bateria, cabos etc.) ainda é estimativa de catálogo/projeto, não massa medida; erros aqui se propagam para MTOW/estrutura/trem de pouso |
| `trim` | preliminary (semi-empírico — Cm_ac/Cm_flap de literatura NACA 230/Raymer cap. 16; `cl_h_max_down_calc` CALCULADO por geometria DATCOM/Nelson (`τ(c_e/c)`, ajuste empírico de Nelson — válido em c_e/c ∈ [0.1, 0.6]); rotação CONSIDERA o binário da linha de TRAÇÃO (braço `prop_axis_above_cg_m`, pós-cancelamento do termo inercial de d'Alembert — ciclo 10 task 2) **e**, desde o ciclo 12 task 4, os binários de atrito de rolamento (`μ_roll·N·h_cg`) e de arrasto de solo (`D·(h_cg−z_drag_above_cg_m)`) — `old→new`: a v5.3 os desprezava com uma estimativa de "≲2 pp de %MAC" que a medição real desta task DESMENTIU (≈4,40 pp medido, mais que o dobro, os dois termos somados são 29% MAIORES que o termo de tração já no balanço); ambos IMPLEMENTADOS neste ciclo, limite dianteiro de rotação recua de 13,3546% para ≈17,76% MAC no baseline real); **ciclo 13 (`old→new`, fecha `docs/backlog.md` #15 PRIORIDADE ALTA e #16)**: o termo de MOMENTO da tração e os termos de SOLO passam a vir da MESMA lei única de tração (antes divergiam ≈27,69% em `Vr≡V_LOF`, indeterminação de ≈8,3 pp de %MAC) — resíduo de d'Alembert ZERO por construção, medido a 1e-12 relativo nos 6 cenários; o limite é calculado nas DUAS superfícies (`rotation_limit_pct_mac_paved`/`_grass`, campos novos) e o campo legado `rotation_limit_pct_mac` passa a valer GRAMA (superfície de operação, mesma premissa de #23/#24) — pavimentado 16,380458% → grama 18,268251% MAC (+1,888 pp) | Ensaio de voo (flare + rotação de decolagem) — resultado SENSÍVEL a `elevator_deflection_max_deg` (±2°) e a `cl_h_max_down` (±0.05 residual) (ver `trim.sensitivity` e §4 abaixo), não tratar como definitivo |
| `performance` | computed (equações fechadas, atmosfera ISA padrão); **ciclo 8 task 1**: polar de subida/gradiente inclui arrasto de flap parcial (`wing.cd0_flap_to_extra`); **ciclo 12 tasks 2/3 (`old→new`, fecha `docs/backlog.md` item 4)**: rolagem de solo de decolagem E de pouso passam do método ENERGÉTICO fechado (Raymer, `V_ref²/2gμ`, sem termo de arrasto por construção) para integração numérica consumindo a polar completa segmento a segmento — medido: `to_50ft_paved_m` +54,92%, `to_50ft_grass_m` +73,00%, `ldg_50ft_m` +15,90%, `ldg_50ft_grass_m` +16,12% no baseline real (ver v5.5 em §1); a aproximação de pouso (segmento antes do toque, ângulo fixo) SEGUE sem termo de arrasto por construção, não afetada; **ciclo 11 task 1**: `climb_gradient_pct` avaliado no piso LEGAL da norma (1,2·Vs_to, referência típica da CS 23.65 ≥1,2·Vs1), não mais no piso de varredura 1,05·Vs_to (viés otimista removido, `docs/backlog.md` item 2); **ciclo 11 task 2**: Vy/teto de serviço usam referência de estol LIMPA (`wing.cl_max_clean`) e polar limpa, janela de busca [1,05;2,00]·Vs com guarda de argmax interior (`docs/backlog.md` item 3); **ciclo 13 (`old→new`, fecha `docs/backlog.md` #8/#9)**: a tração de rolagem (`thrust_ground_roll_n`) e a de cruzeiro/subida/teto (`thrust_available_n`, polinômio `prop_efficiency`) — que divergiam ≈27,69% em `V_LOF` — foram fundidas na lei única `T(V)=FoM(J)·T_ideal_momentum(V,P_eixo)`; o polinômio apagado violava o teto de conservação de quantidade de movimento em 4 dos 8 pontos de operação medidos, 2 alimentando gates que PASSAVAM (`Vx`→gradiente CS 23.65; `V_LOF`→rotação) — consequência medida: `climb_gradient_pct` **12,451842% → 7,913277%**, FLIPA de PASS para FAIL contra o mínimo de 8,3%; `to_50ft_grass_m` **819,110978 → 858,593425 m** (segue REPROVANDO #23) | Mapa de desempenho de hélice real do fabricante para refinar a polar de subida |
| `vn_diagram` | computed (CS 23.333/.335/.337/.341, fórmulas fechadas) | — |
| `structure` | preliminary (vigas simplificadas — viga I equivalente); flutter: preliminary (estimativa analítica) | FEM (estrutura); GVT — ensaio de vibração em solo (flutter) |
| `landing_gear` | preliminary (dimensionamento estático de cargas) | Análise dinâmica de pouso/afundamento |
| `propeller` | semi-empirical (Mach de ponta; folga de solo ESTÁTICA e, desde o ciclo 8, folga em condição CRÍTICA de CS 23.925 — checagem #25 — modelando desde o ciclo 9 o PIVÔ da célula sobre o trem principal (não mais uma translação vertical 1:1 do nariz), fator `(x_main−prop_plane_x_m)/(x_main−x_nose_m)`; achado honesto: no baseline E10 real esse fator (≈1,466) reprovava a checagem #25 — `old→new` (ciclo 10 → ciclo 11): desde o baseline E12 (`x_nose_m` 1,30→1,20, trem de nariz recuado) a checagem #25 **PASSA** (`prop_clearance_critical_m = +0,007367 m`, HOJE); ver `docs/backlog.md` item 1) | Mapa de desempenho de hélice real do fabricante; validar `[propeller].prop_plane_x_m` no CAD (Fase 3) |
| `mission` | computed (segmentos + equação de Breguet, L/D constante em cruzeiro) | — |
| `electrical` | preliminary (soma de cargas nominais configuradas) | Análise transiente/térmica real |
| `sizing` | computed (laço de convergência de ponto fixo) | — |
| `robustness` | **v4.7** (atualizado v5.3): computed (pior-caso determinístico ±σ direcional sobre as 7 massas estruturais; limites de envelope avaliados na régua do PRÓPRIO mundo perturbado desde o ciclo 10 task 2 — o limite dianteiro deixou de ser invariante a massa (linha de tração); `flips[].limite_nominal` carrega a régua nominal para contraste; caso massa-total: re-sizing completo com fatores ×(1+σ)) | — (o próprio bloco É a análise posterior de sensibilidade das 7 massas estruturais `semi-empirical`/`preliminary`; nenhuma análise adicional recomendada) |

O texto exato de cada entrada (em português, como gerado pelo pipeline)
pode variar ligeiramente entre execuções — a tabela acima é a referência
canônica de INTERPRETAÇÃO, o JSON em si é a fonte do texto exibido.

## 4. Blocos de topo

| Campo | Tipo | Presença |
|---|---|---|
| `schema_version` | string | sempre |
| `revision` | string (DEPRECATED, = `schema_version`) | sempre |
| `validation_status` | string (`"PASS"` \| `"FAIL"`) | sempre |
| `wing` | objeto (`WingSpec`) | sempre |
| `propulsion` | objeto (`PropulsionSpec`) | sempre |
| `geometry` | objeto (`GeometrySpec`) ou `null` | sempre preenchido por `main.rs` |
| `empennage` | objeto (`EmpennageSpec`) ou `null` | sempre preenchido |
| `control_surfaces` | objeto (`ControlSurfacesSpec`) ou `null` | sempre preenchido |
| `weight` | objeto (`WeightSpec`) ou `null` | sempre preenchido |
| `trim` | objeto (`TrimSpec`) ou `null` | sempre preenchido (novo na v4.1) |
| `performance` | objeto (`PerformanceSpec`) ou `null` | sempre preenchido |
| `vn_diagram` | objeto (`VnDiagramSpec`) ou `null` | sempre preenchido |
| `structure` | objeto (`StructuralSpec`) ou `null` | sempre preenchido |
| `landing_gear` | objeto (`GearSpec`) ou `null` | sempre preenchido |
| `propeller` | objeto (`PropellerSpec`) ou `null` | sempre preenchido |
| `mission` | objeto (`MissionSpec`) ou `null` | sempre preenchido |
| `electrical` | objeto (`ElectricalSpec`) ou `null` | sempre preenchido |
| `sizing` | objeto (`SizingReport`) ou `null` | sempre preenchido |
| `robustness` | objeto (`RobustnessSpec`) ou `null` | sempre preenchido (novo na v4.6) |
| `fidelity` | objeto `{string: string}` | sempre, não-vazio |
| `violations` | array de string | sempre (vazio se `validation_status == "PASS"`) |
| `warnings` | array de string | sempre (pode ser vazio) |

Os blocos são tipados `Option<T>` no Rust (podem em tese ser `null`) por
simetria estrutural entre si — na prática, `main.rs` sempre os preenche
antes de escrever o arquivo; um consumidor rigoroso pode tratar todos os
blocos acima (exceto os dois primeiros e `wing`/`propulsion`, que nunca são
`Option`) como potencialmente ausentes, mas **um `null` real nunca é
esperado na saída atual do pipeline**.

---

### `wing` — `WingSpec` (AerodynamicsAgent)

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `span_m` | f64 | m | Envergadura total |
| `area_m2` | f64 | m² | Área de referência da asa |
| `aspect_ratio` | f64 | — | Alongamento (AR = b²/S) |
| `airfoil` | string | — | Designação do perfil (ex.: `"NACA 23015"`) |
| `taper_ratio` | f64 | — | Afilamento λ = corda_ponta/corda_raiz |
| `thickness_ratio` | f64 | — | Espessura relativa t/c do perfil |
| `oswald_efficiency` | f64 | — | Fator de eficiência de Oswald (e) |
| `cd0` | f64 | — | Arrasto parasita da asa em cruzeiro |
| `cl_cruise` | f64 | — | CL de cruzeiro |
| `cd_cruise` | f64 | — | CD de cruzeiro. **v4.4**: inclui o arrasto de trim de cruzeiro (`ΔCD_trim`, ver bloco `trim` §4/`cd_trim`) somado ao `cd0+cdi` do build-up — nome/tipo/unidade inalterados, só o VALOR mudou |
| `cl_max` | f64 | — | CL_max com flap/slat em configuração de POUSO (flap cheio) — usado nas distâncias de POUSO e no VS0. **Desde a v5.0**: NÃO é mais o CL_max das distâncias de DECOLAGEM nem da Vr da rotação — ver `cl_max_to` abaixo |
| `cl_max_clean` | f64 | — | CL_max em configuração limpa (cruzeiro) |
| `cl_max_to` | f64 (**novo v5.0**) | — | CL_max em configuração de DECOLAGEM (flap PARCIAL) — DERIVADO por interpolação linear entre `cl_max_clean` e `cl_max_flaps` (não ecoado; `cl_max` é o valor de pouso) pela mesma `trim.to_flap_fraction`: `cl_max_to = cl_max_clean + to_flap_fraction·(cl_max_flaps − cl_max_clean)`. Consumido pela Vr/VS0 da ROTAÇÃO (bloco `trim`) e pelas distâncias de DECOLAGEM (`performance.to_distance_paved_m`/`to_distance_grass_m`/`to_50ft_paved_m`/`to_50ft_grass_m`, calculadas internamente por `agents::performance::takeoff_distance_m`/`takeoff_distance_50ft_m`) — ver §1 (v5.0) para o motivo da mudança |
| `cd0_flap_to_extra` | f64 (**novo, ciclo 8 task 1 — formalizado na v5.1**) | — | ΔCD0 do flap PARCIAL de decolagem = `to_flap_fraction · [wing].cd0_flap_delta` — mesma fração de `cl_max_to` acima, agora aplicada ao arrasto. Consumido por `agents::performance::excess_power_kw` no segmento de SUBIDA da decolagem (`to_50ft_paved_m`/`to_50ft_grass_m`) e no gradiente CS 23.65 (`performance.climb_gradient_pct`, avaliado em Vx). Fecha PARCIALMENTE a lacuna "não existe modelo de flap na polar deste crate" declarada desde o ciclo 7 — a rolagem de solo (segmento DOMINANTE da distância de decolagem, método energético de Raymer) e a aproximação de pouso (ângulo fixo) seguem sem nenhum termo de arrasto de flap, por construção; ver `fidelity.performance` para o detalhe de quais segmentos consomem/não consomem a polar |
| `stall_speed_flaps_kmh` | f64 | km/h | VS0 — stall com flap |
| `stall_speed_clean_kmh` | f64 | km/h | VS1 — stall configuração limpa |
| `ld_ratio_cruise` | f64 | — | L/D em cruzeiro. **v4.4**: recalculado com `cd_cruise` já incluindo o arrasto de trim (ver acima) |

### `propulsion` — `PropulsionSpec` (PropulsionAgent)

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `engine_model` | string | — | Nome/modelo do motor |
| `power_hp` / `power_kw` | f64 | hp / kW | Potência nominal do motor |
| `max_torque_nm` | f64 | N·m | Torque máximo |
| `rated_rpm` | f64 | rpm | RPM nominal do motor |
| `engine_mass_kg` | f64 | kg | Massa do motor |
| `psru_ratio` | f64 | — | Redução do PSRU (motor:hélice) |
| `engine_rpm_cruise` | f64 | rpm | RPM do motor no ponto de cruzeiro escolhido |
| `prop_rpm_cruise` | f64 | rpm | RPM da hélice em cruzeiro |
| `prop_diameter_m` | f64 | m | Diâmetro da hélice usado na busca de cruzeiro (ver nota no bloco `propeller`) |
| `fuel_type` | string | — | Tipo de combustível |
| `fuel_capacity_l` | f64 | L | Capacidade do tanque |
| `fc_cruise_lph` | f64 | L/h | Consumo em cruzeiro |
| `bsfc_cruise_gkwh` | f64 | g/kWh | Consumo específico em cruzeiro |
| `endurance_h` | f64 | h | **INFORMATIVO** — autonomia a tanque cheio/consumo constante; NÃO é o gate de autonomia do projeto (ver `mission.block_time_h`) |
| `range_km` | f64 | km | **INFORMATIVO** — mesma ressalva; ver `mission.range_no_wind_km` |
| `prop_efficiency` | f64 | — | Eficiência de hélice em cruzeiro. **`old→new` (v5.6, ciclo 13, exceção de schema registrada em §1)**: mesmo nome/tipo/faixa, mas muda de ORIGEM — antes lido do polinômio JavaProp `η=−0,15·J²+0,39·J+0,58` (APAGADO), agora DERIVADO por inversão fechada de `T(V)=FoM(J)·T_ideal_momentum(V,P_eixo)` (`η=FoM(J)·V/u`). No baseline real o valor é IDÊNTICO por construção da âncora (`0,7838814965676598`, inalterado) |
| `thrust_cruise_n` | f64 | N | Tração em cruzeiro |
| `p_req_cruise_kw` / `p_shaft_cruise_kw` | f64 | kW | Potência requerida / disponível em cruzeiro |
| `cruise_feasible` | bool | — | `p_req ≤ p_shaft` no ponto de cruzeiro escolhido |

### `geometry` — `GeometrySpec` (Task 6.1 — consolidado de config + WeightBalanceAgent)

Bloco novo na v4.0 — consolida geometria que já existia internamente mas
não era serializada antes.

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `wing_le_root_x_m` | f64 | m (datum) | Bordo de ataque da raiz da asa |
| `chord_root_m` / `chord_tip_m` | f64 | m | Corda na raiz / na ponta da asa |
| `mac_m` | f64 | m | Corda Aerodinâmica Média (MAC) |
| `mac_le_x_m` | f64 | m (datum) | Bordo de ataque do MAC |
| `y_mac_m` | f64 | m | Posição do MAC na envergadura (da raiz) |
| `fuselage_length_m` | f64 | m | Comprimento total da fuselagem |
| `cabin_width_m` / `cabin_height_m` | f64 | m | Dimensões internas da cabine |

### `empennage` — `EmpennageSpec` (EmpennageAgent)

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `s_horizontal_m2` / `s_vertical_m2` | f64 | m² | Área EH / EV |
| `arm_h_m` / `arm_v_m` | f64 | m | Braço CA-asa → CA-empenagem (horizontal/vertical) |
| `span_h_m` / `span_v_m` | f64 | m | Envergadura EH / EV |
| `chord_h_root_m` / `chord_h_tip_m` | f64 | m | Cordas raiz/ponta EH |
| `chord_v_root_m` / `chord_v_tip_m` | f64 | m | Cordas raiz/ponta EV |
| `ar_h` / `ar_v` | f64 | — | Alongamento EH / EV |
| `taper_h` / `taper_v` | f64 | — | Afilamento EH / EV |
| `volume_h` / `volume_v` | f64 | — | Coeficiente de volume EH / EV |
| `eta_h` | f64 | — | Eficiência de pressão dinâmica na EH (q_t/q_∞) |

### `control_surfaces` — `ControlSurfacesSpec` (ControlSurfacesAgent)

Objeto com quatro sub-blocos, todos do tipo `SurfaceGeom`: `aileron`,
`flap`, `elevator`, `rudder`.

| Campo (`SurfaceGeom`) | Tipo | Unidade | Descrição |
|---|---|---|---|
| `span_m` | f64 | m | Envergadura da superfície |
| `area_m2` | f64 | m² | Área da superfície |
| `chord_mean_m` | f64 | m | Corda média |
| `start_m` / `end_m` | f64 | m | Início/fim ao longo da envergadura da superfície-mãe |

**Convenção crítica de `start_m`/`end_m`** (achado da revisão da Task
4.2 — não interprete errado):
- **Aileron, flap** (asa) e **elevator/profundor** (EH) são superfícies
  ESPELHADAS: `span_m`/`start_m`/`end_m` são medidos POR LADO, a partir da
  **linha de centro** (0 = linha de centro; `end_m` nunca ultrapassa a
  SEMI-envergadura da superfície-mãe). `area_m2` é a área TOTAL dos dois
  lados somados.
- **Rudder/leme** (EV) é uma superfície ÚNICA, não espelhada:
  `span_m`/`start_m`/`end_m` são medidos a partir da **raiz** (base da
  deriva). `area_m2` já é a área total (não há segundo lado a somar).

Um consumidor de CAD nunca deve tratar `end_m` como a largura ponta-a-ponta
da superfície espelhada.

### `weight` — `WeightSpec` (WeightBalanceAgent)

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `oew_kg` | f64 | kg | Peso vazio operacional |
| `mtow_kg` | f64 | kg | MTOW do cenário de ENVELOPE (4 pax + bagagem + tanque cheio) — ver `sizing.mtow_envelope_kg` para o par explícito com o MTOW de missão |
| `payload_kg` | f64 | kg | Payload de projeto |
| `fuel_mass_kg` | f64 | kg | Massa de combustível no cenário de peso |
| `cg_mac_fwd_pct` / `cg_mac_aft_pct` | f64 | %MAC | CG mais dianteiro/traseiro OBSERVADO entre os cenários de carga |
| `static_margin_pct` | f64 | % | Margem estática |
| `cg_limit_fwd_pct_mac` | f64 | %MAC | Limite DIANTEIRO admissível — `max(trim.flare_limit_pct_mac, trim.rotation_limit_pct_mac)` (bloco `trim` abaixo), não mais o proxy `[stability].sm_max`. Ambos os termos do `max` são NÚMEROS ÚNICOS (não variam por cenário de carga — ver bloco `trim`), então este limite se aplica IGUALMENTE a todos os cenários. Não confundir com `cg_mac_fwd_pct` acima (valor OBSERVADO). **PODE ficar À FRENTE de `cg_limit_aft_pct_mac`** — ver "Envelope vazio" abaixo. |
| `cg_limit_aft_pct_mac` | f64 | %MAC | Limite TRASEIRO admissível — de `[stability].sm_min` |
| `structural_masses` | objeto (`StructuralMassesSpec`, **novo v4.5**) | — | As 7 massas estruturais COMPUTADAS + os 5 fatores de composto usados — ver tabela abaixo |

Sub-bloco `StructuralMassesSpec` (**novo v4.5**) — massas estruturais
computadas por `agents::mass_model` (Raymer, "Aircraft Design: A
Conceptual Approach", cap. 15.2, equações GA) × fatores de composto
(Tab. 15.4, `[mass_model]` da configuração). São EXATAMENTE as mesmas
massas que entraram no OEW (`weight.oew_kg`) e nos blocos `structure`/
`landing_gear` — não uma cópia recomputada independentemente:

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `asa_kg` | f64 | kg | Massa da asa (Raymer eq. 15.2, GA) |
| `fuselagem_kg` | f64 | kg | Massa da fuselagem |
| `emp_h_kg` | f64 | kg | Massa da empenagem horizontal |
| `emp_v_kg` | f64 | kg | Massa da empenagem vertical |
| `trem_principal_kg` | f64 | kg | Massa TOTAL do trem principal (as duas pernas) |
| `trem_nariz_kg` | f64 | kg | Massa do trem de nariz |
| `tanques_kg` | f64 | kg | Massa do sistema de combustível (tanques integrais) |
| `composite_factor_wing` | f64 | — | Fator de composto aplicado à asa (`[mass_model].composite_factor_wing`, Tab. 15.4) |
| `composite_factor_tail` | f64 | — | Fator de composto aplicado às empenagens (`composite_factor_tail`) |
| `composite_factor_fuselage` | f64 | — | Fator de composto aplicado à fuselagem (`composite_factor_fuselage`) |
| `composite_factor_gear` | f64 | — | Fator de composto aplicado ao trem de pouso (`composite_factor_gear`) |
| `composite_factor_fuel_system` | f64 | — | Fator de composto aplicado ao sistema de combustível (`composite_factor_fuel_system`) |

**Envelope vazio**: quando `cg_limit_fwd_pct_mac > cg_limit_aft_pct_mac`
(achado histórico pré-E6 do baseline real: ≈39,9% > ≈36,6% — o baseline
atual tem o envelope FECHADO, `old→new` (números desatualizados, fix wave
ciclo 12; era "≈6,1% < ≈43,5%") **17,757974% < 43,460036%** HOJE, ver
bloco `trim` abaixo), NENHUM CG é admissível — os dois
critérios físicos (autoridade de rotação de decolagem vs. margem estática
mínima) são mutuamente incompatíveis com esta célula/trem, não apenas com
os cenários de carga observados. Nesse caso `violations` sempre contém um
item DEDICADO começando com `"Envelope de CG VAZIO:"` (distinto das
violações por cenário, que também continuam presentes) —
`ConstraintChecker::verify`, seção 9a. Um consumidor de CAD deve tratar
`cg_limit_fwd_pct_mac > cg_limit_aft_pct_mac` como sinal explícito dessa
condição (não como um intervalo "invertido" a ser corrigido/normalizado).

### `trim` — `TrimSpec` (TrimAuthorityAgent — novo na v4.1, autoridade calculada por geometria desde a v4.2)

Limite dianteiro FÍSICO do envelope de CG, derivado da autoridade de
profundor disponível nas duas manobras críticas de arfagem
nariz-para-cima: **flare no pouso** (V_ref = 1,3·Vs0, flap de pouso,
balanço de momentos em torno do CG, fechado pela contribuição de
sustentação da própria empenagem) e **rotação na decolagem** (Vr =
1,1·Vs0(W), flap de decolagem, balanço de momentos em torno do TREM
PRINCIPAL). Substitui o proxy `[stability].sm_max` (removido — ver §1).

Desde a v4.2 (task refino-ciclo2), a autoridade bruta (`cl_h_max_down`)
deixou de ser um parâmetro livre de config e passou a ser CALCULADA por
geometria DATCOM/Nelson:

```text
τ(c_e/c) = 1,24·√(c_e/c) − 0,16                    [Nelson, fig. 2.21; válido c_e/c ∈ [0.1, 0.6]]
a_t = lift_curve_slope(AR_h)                       [weight_balance::lift_curve_slope]
cl_h_max_down_calc = a_t · τ · δe_max_rad
cl_h_max_down = min(cl_h_max_down_calc, cl_h_stall_limit)
```

onde `c_e/c` é `[control_surfaces].elevator_chord_frac` (a razão
corda-do-profundor/corda-local do EH é constante ao longo da envergadura
neste modelo trapezoidal — nenhum campo adicional é necessário), `AR_h` é
`EmpennageSpec.ar_h`, e `δe_max_rad` é `[control_surfaces].
elevator_deflection_max_deg` convertido para radianos.

**`flare_limit_pct_mac` e `rotation_limit_pct_mac` são NÚMEROS ÚNICOS,
NÃO variam por cenário de carga** — mas por motivos diferentes.

A **flare** simplesmente não depende do peso.

A **rotação** dependia, deixou de depender (prova de cancelamento
algébrico), e voltou a depender:

- Até o ciclo 9 valia o resultado NÃO ÓBVIO de que `W` cancelava: sob a
  política `Vr = 1,1·Vs0(W)`, a pressão dinâmica de rotação `q_r(W)` é
  PROPORCIONAL a `W`, logo todos os termos de momento em jogo (download da
  empenagem, sustentação da asa, momento de perfil+flap) também são, e o
  `W` CANCELA EXATAMENTE em `x_cg_rot = x_main − M_disponível(W)/W`.
- **Ciclo 10 (task 2)** acrescenta o momento da **linha de tração**:
  `−T(Vr)·z_eixo` (nariz-abaixo), com `z_eixo =
  [propeller].prop_axis_above_cg_m` — o offset EIXO↔CG, **não** a altura
  sobre o solo: a corrida de decolagem é ACELERADA, e o termo inercial de
  d'Alembert (`+m·aₓ·h_cg`) cancela exatamente a porção `h_cg` do braço da
  tração no somatório sobre o contato do trem principal (Gudmundsson/
  Roskam carregam `T·(z_T−z_mg)` e `m·aₓ·(z_cg−z_mg)` juntos). `old→new`
  (ciclo 10 → ciclo 12): esta task deixava os termos de solo remanescentes
  (`μN·h_cg`, `D·(h_cg−h_D)`, estimados então em "≲2 pp") DESPREZADOS — a
  frase abaixo descreve esse estado ANTIGO, já não é o modelo ATUAL (ver
  bullet do ciclo 12 logo adiante): "Termos de solo remanescentes
  (`μN·h_cg`, `D·(h_cg−h_D)`, ≲2 pp) são desprezados e documentados no
  código." `T` é tração de
  hélice a `Vr` e **não** escala com `W`: como `Vr ∝ √W` e a potência de
  eixo é fixa, `T/W ∝ η(J)·W^(−3/2)`. O termo sobrevive à divisão por `W`
  e a invariância morre.
- **Ciclo 12 (task 4)** acrescenta os termos de SOLO que o ciclo 10 havia
  deixado de fora: `−μ_roll·N·h_cg − D·(h_cg−z_drag_above_cg_m)`, ambos
  nariz-ABAIXO (mesmo sentido do termo de tração). A estimativa de
  magnitude do ciclo 10 ("≲2 pp de MAC") estava ERRADA — medição real deu
  **≈4,40 pp** (mais que o dobro), os dois termos somados sendo 29%
  MAIORES que o termo de tração já no balanço. Fórmula ATUAL (ciclo 12,
  substitui a do ciclo 10 acima):
  `x_cg_rot(W) = x_main − k_aero + [T(Vr(W))·z_eixo + μ_roll·N·h_cg +
  D·(h_cg−z_drag_above_cg_m)]/W`. Ver docstring de
  `agents::trim_authority::rotation_available_moment_nm` (seção "TERMOS DE
  SOLO") para a derivação completa.
- Consequência falseável: **aeronave mais LEVE ⟹ limite mais RECUADO**.
  Variação MEDIDA no baseline real entre os extremos de peso dos cenários:
  **1,4621 pp de MAC**. O número único publicado neste JSON é portanto o
  **MÁXIMO** dos limites por cenário (que neste modelo cai no mais leve),
  usado como ENVOLTÓRIA conservadora — não é mais uma identidade
  algébrica. A checagem exata por cenário continua em
  `rotation_margin_per_scenario`.

Ver a re-derivação completa (em português) na docstring de
`agents::trim_authority::rotation_fwd_limit_m`.

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `flare_limit_pct_mac` | f64 | %MAC | Limite dianteiro de flare — número único, independe do peso |
| `rotation_limit_pct_mac` | f64 | %MAC | Limite dianteiro de rotação — número único, MÁXIMO dos limites por cenário (envoltória conservadora; DEPENDE do peso desde o ciclo 10 task 2, momento da linha de tração — ver acima). **`old→new` (v5.6, ciclo 13, exceção de schema registrada em §1)**: mesmo nome/tipo/unidade, mas muda de VALOR **e** de SIGNIFICADO — passa de valer a superfície PAVIMENTADA para valer a de OPERAÇÃO (**GRAMA**, mesma premissa das checagens #23/#24). É idêntico a `rotation_limit_pct_mac_grass` |
| `rotation_limit_pct_mac_paved` | f64 (**novo v5.6**) | %MAC | Limite dianteiro de rotação avaliado com `[performance].mu_roll_paved` — publicado a título informativo/comparativo, NÃO é o que o gate de envelope de CG usa |
| `rotation_limit_pct_mac_grass` | f64 (**novo v5.6**) | %MAC | Limite dianteiro de rotação avaliado com `[performance].mu_roll_grass` — a superfície de OPERAÇÃO (spec ciclo13 §7); idêntico a `rotation_limit_pct_mac` |
| `rotation_margin_per_scenario` | array de objeto (`ScenarioTrimLimit`) | — | Diagnóstico informativo POR CENÁRIO — margem de autoridade de rotação avaliada na CG/peso REAIS de cada cenário (essa sim varia por cenário), agora sobre a superfície de GRAMA (ciclo 13) — NÃO usado para calcular `rotation_limit_pct_mac`/`inside_envelope` |
| `governing` | string (`"flare"` \| `"rotacao"`) | — | Qual dos dois limites ÚNICOS é maior (mais restritivo) |
| `cl_h_available` | f64 | — | CL_h disponível — `-cl_h_max_down·(1−trim_margin)` |
| `sensitivity` | objeto (`TrimSensitivity`) | — | Limites de flare recomputados a `cl_h_max_down ± 0,05` E a `elevator_deflection_max_deg ± 2°` |
| `cm_ac` / `cm_flap_delta` | f64 | — | Parâmetros ecoados de `[wing]` |
| `cl_h_max_down` | f64 | — | `cl_h_max_down` OPERACIONAL (`min(cl_h_max_down_calc, cl_h_stall_limit)`) — o valor efetivamente usado no balanço de momentos. **v4.2**: deixou de ser um eco de `[stability].cl_h_max_down` (campo removido); agora é CALCULADO — ver fórmula acima |
| `cl_h_max_down_calc` | f64 (**novo v4.2**) | — | `cl_h_max_down` BRUTO (`a_t·τ·δe_max_rad`), ANTES do truncamento pelo teto de stall — igual a `cl_h_max_down` quando `capped_by_stall == false` |
| `tau_elevator` | f64 (**novo v4.2**) | — | Eficácia de superfície do profundor τ(c_e/c), ajuste de Nelson |
| `capped_by_stall` | bool (**novo v4.2**) | — | `true` quando `cl_h_max_down_calc` excede `[stability].cl_h_stall_limit` — o teto de stall, não a geometria do profundor, é o fator limitante |
| `trim_margin` / `cl_ground_rotation` / `to_flap_fraction` | f64 | — | Parâmetros ecoados de `[stability]` (`to_flap_fraction`: renomeado de `to_flap_cm_fraction` no ciclo 7 — papel duplo, ΔCm da rotação **e** `wing.cl_max_to`) |
| `cl_h_trim_cruise` | f64 (**novo v4.4**) | — | CL_h de TRIM em cruzeiro (sem flap) — upload (positivo, CG atrás do CA da asa) ou download (negativo, CG à frente), calculado no CG de REFERÊNCIA da missão (`cg_reference_scenario`, JÁ CONVERGIDO — não o valor lag-1 usado dentro do laço de MTOW). Ver fórmula/dedução em §1 (v4.4) |
| `cd_trim` | f64 (**novo v4.4**) | — | ΔCD_trim — arrasto INDUZIDO da empenagem ao gerar `cl_h_trim_cruise`. O delta somado a `wing.cd_cruise`/refletido em `wing.ld_ratio_cruise` usa o CG LAG-1 do laço de convergência do MTOW; este campo é RECALCULADO no CG JÁ CONVERGIDO (mesma distinção de `cl_h_trim_cruise` acima) — na prática os dois coincidem a um resíduo de convergência (~1e-9), não são estritamente o mesmo número ecoado |
| `cg_reference_scenario` | string (**novo v4.4**) | — | Nome do cenário de carga (`weight`) usado como CG de referência da missão — sempre `"4 pax + bagagem + meia"` neste modelo (meia-missão) |
| `cg_reference_pct_mac` | f64 (**novo v4.4**) | %MAC | CG do cenário acima, JÁ CONVERGIDO — o valor efetivamente usado para calcular `cl_h_trim_cruise`/`cd_trim` |

Sub-bloco `ScenarioTrimLimit` (um por cenário de `weight`) — `rotation_margin_per_scenario`:

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `scenario` | string | — | Nome do cenário (mesmo do bloco `weight`) |
| `rotation_authority_margin_pct` | f64 | % | `(momento nariz-acima DISPONÍVEL − NECESSÁRIO)/NECESSÁRIO × 100`, avaliados na CG e no `Vr(W)` REAIS deste cenário. Negativo = autoridade insuficiente para rotacionar nesta CG/peso (quanto mais negativo, maior o déficit). Zero exatamente na CG de `rotation_limit_pct_mac`, para qualquer peso. |

Sub-bloco `sensitivity` (`TrimSensitivity`):

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `cl_h_max_down_minus` / `cl_h_max_down_plus` | f64 | — | `cl_h_max_down ∓ 0,05` (perturbação direta do valor OPERACIONAL, sem recalcular τ/δe) |
| `flare_limit_pct_mac_minus` / `flare_limit_pct_mac_plus` | f64 | %MAC | Limite de flare recomputado com o parâmetro acima |
| `elevator_deflection_max_deg_minus` / `elevator_deflection_max_deg_plus` | f64 (**novo v4.2**) | ° | `[control_surfaces].elevator_deflection_max_deg ∓ 2°` |
| `flare_limit_pct_mac_deflection_minus` / `flare_limit_pct_mac_deflection_plus` | f64 (**novo v4.2**) | %MAC | Limite de flare recomputado com `cl_h_max_down_calc` recalculado (τ/a_t fixos, só δe muda) |

**Baseline real** (`config/aircraft/baseline_4seat.toml`) — `old→new`,
PARÁGRAFO REESCRITO POR INTEIRO na fix wave do ciclo 12 (terceira vez que
esta seção era remendada por baixo com um parágrafo `old→new` anexado sem
reescrever o texto original desatualizado; esta reescrita consolida tudo
num único texto vigente, com o histórico preservado como `old→new`
explícito, não mais espalhado por parágrafos empilhados):
`cl_h_max_down_calc ≈ 1,0577` (c_e/c=0,40, AR_h=4,0, δe_max=25° — abaixo
do teto de stall 1,10, `capped_by_stall=false`), +11,3% sobre o antigo
palpite de config (0,95, inalterado desde a task refino-ciclo2).

`rotation_limit_pct_mac` mudou repetidamente desde a task refino-ciclo2:
**≈6,10% MAC** (refino-ciclo2, `cl_max_to` ainda inconsistente com o
`Cm_TO`) → **8,533% MAC** (ciclo 7, task 1, `cl_max_to` consistente) →
**13,354637% MAC** (ciclo 10, task 2, momento da linha de tração,
+4,82 pp) → **17,757974% MAC** (ciclo 12, task 4, termos de solo,
+4,403 pp — mais que o dobro da estimativa "≲2 pp" que o ciclo 10
registrava) → **18,268251% MAC** (ciclo 13, superfície de GRAMA na
rotação — o valor pavimentado do ciclo 12 era 16,380458% MAC, +1,888 pp
ao trocar de superfície; ver `rotation_limit_pct_mac_paved`/`_grass`
acima) — o valor publicado HOJE no baseline real. A ROTAÇÃO CONTINUA
governando o limite dianteiro (mais restritiva que a FLARE) — não mais a
≈10,95% MAC intermediária de quando só a autoridade calculada (sem a
linha de tração) estava em vigor — e continua ATRÁS do limite traseiro
(**43,460036% MAC**, antes citado como "≈43,46%"): **envelope de CG
FECHADO** (o limite dianteiro segue atrás do traseiro). `old→new`
(ciclo 12 → ciclo 13): o limite dianteiro subiu (recuou) o bastante para
que o CG NOMINAL do cenário 'Solo (piloto)' (17,758487% MAC, invariante a
esta mudança) fique À FRENTE dele — o cenário cruza para **violação
NOMINAL** de envelope de CG (não mais só sob perturbação de robustez, ver
`robustness.flips` abaixo). Não é regressão: é a MESMA superfície física
da decolagem, agora avaliada de forma consistente nos dois lugares que a
medem (rotação e checagens #23/#24).

As margens de autoridade de rotação por cenário
(`trim.rotation_margin_per_scenario` no JSON) também mudaram:
**≈+26% a +207%** (refino-ciclo2, todos os 6 cenários folgados) →
**+0,0011863088529595282% a +100,57072320379189%** (ciclo 12) →
**−1,179429677656323% a +95,86217358293354%** (ciclo 13) — a margem do
cenário mais apertado ('Solo (piloto)') cruzou de essencialmente zero
(+0,0011863088529595282%, ciclo 12) para NEGATIVA (−1,179429677656323%,
ciclo 13): déficit de autoridade de rotação nominal, consistente com a
violação de envelope de CG acima. Ver `docs/backlog.md` (entrada "Achados
da task 4 — recuo do limite de rotação e margem residual", ciclo 12, e a
entrada #15/#16 RESOLVIDAS, ciclo 13) para o registro completo, incluindo
a checagem de estabilidade numérica do ciclo 12 (tolerância de
convergência 0,5 kg → 1e-9 kg muda o resultado só no 9º/10º dígito).

A **flare** também mudou desde refino-ciclo2: **≈−16,29% MAC** (então) →
**−8,818504% MAC** (HOJE) — segue NEGATIVA (fisicamente "antes do bordo
de ataque"), nunca governa.

Achado de projeto que PERSISTE (não corrigido em nenhuma das tasks acima,
decisão de layout humana): o trem principal (`[gear].x_main_m`) continua
sendo a causa raiz de a ROTAÇÃO (não a flare) governar o limite
dianteiro — ver `agents::trim_authority` (docstring do módulo) para a
dedução completa.

### `performance` — `PerformanceSpec` (PerformanceAgent)

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `v_cruise_kmh` / `v_stall_kmh` | f64 | km/h | Velocidades de cruzeiro / stall |
| `rc_sl_ms` / `rc_cruise_alt_ms` | f64 | m/s | Razão de subida ao nível do mar / na altitude de cruzeiro. **Campanha ciclo 11 (task 2, 2026-08-10)**: Vy referência mudou para `cl_max_clean` (estol limpo), janela de busca refinada de [1,3;1,8]·Vs para [1,05;2,00]·Vs com guarda de argmax interior (ver `docs/superpowers/specs/2026-08-10-ciclo11-subida-honesta-design.md` ERRATUM). Baseline real: **4,999902 → 4,999905 m/s** (~0,00003 m/s de mudança, efeito líquido ≈zero: Vy genuinamente não depende de CL_max). |
| `service_ceiling_m` | f64 | m | Teto de serviço |
| `to_distance_paved_m` / `to_distance_grass_m` | **f64 ou a string `"infinita"`** | m | Distância de decolagem — estimativa SIMPLIFICADA legada, rolagem × fator ad hoc 1,5 (aproximação de transição de Raymer). **`old→new`, ciclo 12 (backlog item 4, RESOLVIDO)**: a rolagem que este fator multiplica passou do método energético fechado para integração numérica com arrasto — medido: **398,227641 → 744,556577 m** (pavimentada, +86,97%), **477,873169 → 996,335432 m** (grama, +108,49%). **Caso especial (v5.5)**: pode ser `f64::INFINITY` (tração insuficiente para acelerar até V_LOF), serializado como `"infinita"` — ver §5. **Achado companheiro (backlog item 11, fora de escopo)**: o fator 1,5 foi calibrado para o método ANTIGO; a razão real medida hoje é `to_50ft_paved_m/rolagem_pavimentada ≈ 1,312` — abaixo de 1,5 — e `to_distance_paved_m` (744,56 m) passa a EXCEDER `to_50ft_paved_m` (651,26 m), inconsistência entre a estimativa legada e o campo FÍSICO do mesmo JSON. `to_50ft_paved_m`/`to_50ft_grass_m` (abaixo) são a referência física, não estes dois campos. |
| `landing_distance_m` | **f64 ou a string `"infinita"`** | m | Distância de pouso — estimativa SIMPLIFICADA legada, rolagem + 200 m fixos de aproximação. **`old→new`, ciclo 12 (backlog item 4, RESOLVIDO)**: a rolagem que este campo soma passou do método energético fechado para integração numérica com arrasto e alívio de sustentação — medido: **362,656622 → 442,539441 m** (+22,03%). **Caso especial (v5.5)**: pode ser `f64::INFINITY` (arrasto e frenagem insuficientes para desacelerar), serializado como `"infinita"` — ver §5. `ldg_50ft_m`/`ldg_50ft_grass_m` (abaixo) são a referência física, não este campo (backlog item 11, fora de escopo — remoção num MAJOR futuro). |
| `range_km` / `endurance_h` | f64 | km / h | **INFORMATIVO** — eco de `propulsion.range_km`/`endurance_h`; não é o gate do projeto |
| `vx_kmh` / `vy_kmh` | f64 | km/h | Velocidade de melhor ângulo / melhor razão de subida. **Viés RESOLVIDO em `vx_kmh`** (ciclo 11, task 1, 2026-08-10): `best_climb_angle_ms` varria a partir do piso 1,05·V_s_to; como RC/V é monotonicamente DECRESCENTE na faixa modelada, a função devolvia esse piso como resultado — avaliar mais cedo (mais devagar) dá gradiente MAIOR, então 1,05·V_s_to era um viés OTIMISTA. O piso da varredura subiu para 1,20·V_s_to, referência típica da CS 23.65 (≥1,2·Vs1). Baseline real: **121,519501 → 138,871480 km/h** (+14,28%, razão exata 1,20/1,05 = 1,142857). **Viés RESOLVIDO em `vy_kmh`** (ciclo 11, task 2, 2026-08-10): referência de estol mudou de `wing.cl_max` (flap cheio) para `wing.cl_max_clean` (estol limpo — ver bloco `wing`); janela de busca refinada de [1,3;1,8]·Vs para [1,05;2,00]·Vs com guarda de argmax interior (evita artefato de piso da janela anterior que ocorria quando o pico real de RC ficava fora). Baseline real: **147,915721 → 148,435393 km/h** (+0,35%, efeito líquido ≈zero: Vy genuinamente não depende de CL_max, o pico de RC é quase insensível à mudança). Ver `docs/superpowers/specs/2026-08-10-ciclo11-subida-honesta-design.md` (DISCOVERY e ERRATUM da spec). |
| `best_glide_kmh` / `glide_ratio` | f64 | km/h / — | Velocidade e razão L/D de melhor planeio |
| `climb_gradient_pct` | f64 | % | Gradiente de subida em Vx, solo, MTOW (CS 23.65 exige ≥ 8,3%). **Viés RESOLVIDO** (ciclo 11, task 1, 2026-08-10): a varredura de `best_climb_angle_ms` partia do piso 1,05·V_s_to; como RC/V é monotonicamente DECRESCENTE nessa faixa, a função devolvia esse piso — avaliar mais cedo (mais devagar) sempre dá gradiente MAIOR, então 1,05·V_s_to era um viés OTIMISTA, não uma leitura conservadora. O piso subiu para 1,20·V_s_to, referência típica da CS 23.65 (≥1,2·Vs1). Baseline real E10: **13,896713% (antigo, a 1,05·Vs, otimista) → 12,451842%** (novo, a 1,20·Vs, honesto), -1,444871 pp. Gate PASSA (≥ 8,3%), folga intacta. Ver `docs/backlog.md` item 2 (RESOLVIDO). |
| `to_50ft_paved_m` / `to_50ft_grass_m` | **f64 ou a string `"infinita"`** | m | Distância de decolagem sobre obstáculo de 15 m/50 ft — referência FÍSICA (segmentada: rolagem + rotação + subida até 15 m). **Caso especial (v5.4)**: quando o obstáculo é inatingível (razão de subida ≤ 0 no segmento de subida até 15m — ver `agents::performance::takeoff_distance_50ft_m`, ramo `rc <= 0.0`), o resultado é `f64::INFINITY`, serializado como a string literal `"infinita"`, NUNCA como `null` nem como um número (ver §5). Um parser JSON genérico deve tratar estes campos como `number \| "infinita"`, não como `number` puro. **`old→new`, ciclo 12 (tasks 2/3, backlog item 4, RESOLVIDO)**: a rolagem que alimenta este campo passou do método energético fechado para integração numérica com arrasto — medido no baseline real: `to_50ft_paved_m` **420,372451 → 651,258408 m** (+54,92%), `to_50ft_grass_m` **473,469470 → 819,110978 m** (+73,00%). **Consequência de gate**: `to_50ft_grass_m` (819,11 m) passa a EXCEDER a pista de grama de 600 m — checagem #23 REPROVA no baseline real (antes PASSAVA). |
| `ldg_50ft_m` | f64 | m | Distância de pouso sobre obstáculo de 15 m/50 ft, pista PAVIMENTADA (`mu_brake_paved`) — **INFORMATIVO** desde a v4.8: não é o gate de pista. Diferente de `to_50ft_*`/`to_distance_*`/`landing_distance_m`, este campo NÃO tem tratamento especial de infinito (sempre `number`, nunca `"infinita"`) — ver §5. **`old→new`, ciclo 12 (task 3, backlog item 4, RESOLVIDO)**: rolagem por integração com arrasto e alívio de sustentação — medido: **502,458299 → 582,341118 m** (+15,90%). |
| `ldg_50ft_grass_m` | f64 | m | Distância de pouso sobre obstáculo de 15 m/50 ft em GRAMA (`mu_brake_grass`) — sempre > `ldg_50ft_m`; é a grandeza gateada pela checagem #24 contra `runway_available_m`. NÃO tem tratamento especial de infinito (sempre `number`) — ver §5. **`old→new`, ciclo 12 (task 3, backlog item 4, RESOLVIDO)**: medido **556,677173 → 646,437301 m** (+16,12%). **Consequência de gate**: 646,44 m passa a EXCEDER a pista de grama de 600 m — checagem #24 REPROVA no baseline real (antes PASSAVA, por `old→new` (número corrigido, fix wave ciclo 12; era "≈46 m") **600 − 556,677173 = 43,3 m**). |

### `vn_diagram` — `VnDiagramSpec` (VnDiagramAgent, CS 23.333/.335/.337/.341)

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `va_kmh` / `vb_kmh` / `vc_kmh` / `vd_kmh` | f64 | km/h | Velocidades de manobra / rajada de projeto / cruzeiro de projeto / mergulho de projeto |
| `n_lim_pos` / `n_lim_neg` | f64 | g | Fatores de carga limite de manobra (CS 23.337) |
| `n_gust_vc` / `n_gust_vd` | f64 | g | Fatores de carga de rajada em VC/VD, massa de envelope (CS 23.341) |
| `n_gust_vc_light` | f64 | g | Fator de carga de rajada em VC, massa do cenário mais leve |
| `n_design` | f64 | g | Fator de carga de PROJETO — `max(n_lim_pos, n_gust_vc, n_gust_vc_light)`; dimensiona a estrutura |
| `points` | array de `[V_kmh, n]` | km/h, g | Polígono do envelope V-n, para plotagem/CAD |

### `structure` — `StructuralSpec` (StructuralAgent)

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `design_load_factor_g` | f64 | g | Fator de carga de projeto = `vn_diagram.n_design` |
| `ultimate_load_factor_g` | f64 | g | Fator último = 1,5 × `design_load_factor_g` |
| `wing_root_bending_limit_nm` / `_ult_nm` | f64 | N·m | Momento fletor na raiz — carga limite / última |
| `spar_material` | string | — | Material das longarinas |
| `spar_height_root_m` | f64 | m | Altura da longarina na raiz |
| `spar_flange_area_cm2` | f64 | cm² | Área de mesa da longarina |
| `spar_web_thickness_mm` | f64 | mm | Espessura da alma da longarina |
| `skin_min_thickness_mm` | f64 | mm | Espessura mínima da pele |
| `frame_spacing_mm` | f64 | mm | Espaçamento de cavernas da fuselagem |
| `flutter_speed_kmh` | f64 | km/h | Velocidade de flutter estimada (deve ser ≥ 1,20 × VD) |
| `design_dive_speed_kmh` | f64 | km/h | VD |
| `va_kmh` | f64 | km/h | VA (CS 23.335, com VS1 limpa) |
| `fatigue_life_cycles` | **f64 ou a string `"infinita"`** | ciclos de voo | Vida em fadiga estimada. **Caso especial**: quando a tensão equivalente fica abaixo do limite de fadiga do material, o resultado físico é vida infinita — serializado como a string literal `"infinita"`, NUNCA como `null` nem como um número (ver §5 abaixo). Um parser JSON genérico deve tratar este campo como `number \| "infinita"`, não como `number` puro. |
| `flutter_ok` | bool | — | `flutter_speed_kmh ≥ 1,20 × design_dive_speed_kmh` |

### `landing_gear` — `GearSpec` (LandingGearAgent)

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `gear_type` | string | — | Tipo de trem |
| `track_width_m` | f64 | m | Bitola do trem principal |
| `wheelbase_m` | f64 | m | Distância entre eixos |
| `tipover_angle_deg` | f64 | deg | Ângulo anti-tombamento LATERAL (deve ser < 55°) — distinto de `tipback_angle_deg` (fore/aft) |
| `nose_load_max_pct` | f64 | % | **v4.3**: fração de carga no nariz no CG mais DIANTEIRO real dos cenários de carga — teto 25% (substitui `nose_load_fraction_pct`) |
| `nose_load_min_pct` | f64 | % | **v4.3**: fração de carga no nariz no CG mais TRASEIRO real dos cenários de carga — piso 8% |
| `tipback_angle_deg` | f64 (**novo v4.3**) | deg | Ângulo de tipback (trem principal → CG mais TRASEIRO real, Raymer cap. 11) — deve ser >= `[gear].tipback_min_deg` |
| `tail_strike_margin_deg` | f64 (**novo v4.3**) | deg | Folga angular de tail-strike (geometria simplificada do cone de cauda) — deve ser >= `[gear].rotation_attitude_deg` |
| `main_gear_load_n` | f64 | N | Carga máxima no trem principal (por perna) |
| `nose_gear_load_n` | f64 | N | Carga máxima no trem de nariz. **v4.3**: agora dimensionada no CG mais dianteiro real (antes: CG traseiro, que subestimava — 3.296→8.038 N) |
| `main_oleo_stroke_mm` / `nose_oleo_stroke_mm` | f64 | mm | Curso do amortecedor |
| `main_tire` / `nose_tire` | string | — | Designação do pneu |
| `tire_pressure_psi` | f64 | **psi** (não SI) | Pressão dos pneus |
| `max_sink_rate_ms` | f64 | m/s | Taxa de afundamento máxima de projeto |
| `retraction_time_s` | f64 | s | Tempo de retração/extensão |
| `actuator_power_w` | f64 | W | Potência do atuador elétrico |
| `total_weight_kg` | f64 | kg | Peso total do sistema de trem |

### `propeller` — `PropellerSpec` (PropellerAgent, CS 23.925)

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `diameter_m` | f64 | m | Diâmetro autoritativo da hélice |
| `blades` | u32 | — | Número de pás |
| `source` | string | — | `"config"` (veio de `[propeller].diameter_m`) ou `"derivado"` (calculado pelo agente) |
| `tip_mach_static` | f64 | — | Mach de ponta de pá, condição estática |
| `tip_mach_cruise_helical` | f64 | — | Mach de ponta de pá, cruzeiro (composição helicoidal) |
| `ground_clearance_m` | f64 | m | Folga ponta de pá ↔ solo — `shaft_height − diameter_m/2`, onde `shaft_height = gear.h_cg_ground_m + propeller.prop_axis_above_cg_m` (**datum derivado do trem desde a Task 1 do ciclo 5** — antes, `[propeller].shaft_height_m` era um valor ABSOLUTO independente, e encurtar o trem não afetava a folga reportada; ver §1 histórico v4.7) |
| `diameter_max_by_mach_m` | f64 | m | Maior diâmetro que respeita ambos os limites de Mach |
| `diameter_max_by_clearance_m` | f64 | m | Maior diâmetro que respeita a folga mínima de solo |
| `ok_mach_static` / `ok_mach_cruise` / `ok_clearance` | bool | — | Checagens individuais |
| `prop_clearance_critical_m` | f64 (**novo, ciclo 8 task 2 — formalizado na v5.1**) | m | Folga ponta de pá ↔ solo na condição CRÍTICA de CS 23.925 (amortecedor do trem de NARIZ TOTALMENTE COMPRIMIDO/batente + pneu MURCHO), distinta de `ground_clearance_m` (folga ESTÁTICA, aeronave CARREGADA em deflexão estática — mains e nariz comprimidos pelo peso, pneu cheio — CONTRATO de `[gear].h_cg_ground_m`, não "trem estendido"). Na condição CRÍTICA, o trem de NARIZ some da deflexão estática e vai ao curso RESTANTE + pneu murcho; os mains permanecem na mesma deflexão estática de `ground_clearance_m`. Hélice TRATORA: o trem de NARIZ governa, não o principal. Preenchido em DOIS PASSOS pelo pipeline (`specs::PropellerSpec::fill_critical_clearance`, chamado DEPOIS do `LandingGearAgent` — a hélice roda antes do trem na ordem de execução real) — nunca `NaN`, placeholder `0.0` até essa chamada. **Checagem #25** de `ConstraintChecker::verify` reprova quando `<= 0.0`. **FÓRMULA (ciclo 10, task 1, deflexão estática — old→new)**: `ground_clearance_m − Δ_prop`, `Δ_prop = (landing_gear.nose_oleo_stroke_mm/1000 × (1 − [gear].static_sag_fraction) + [gear].tire_deflation_delta_m) × fator`, `fator = ([gear].x_main_m − [propeller].prop_plane_x_m)/([gear].x_main_m − [gear].x_nose_m)`. ANTES do ciclo 10 (ciclo 9, curso TOTAL do nariz não RESTANTE): `Δ_prop = (nose_oleo_stroke_mm/1000 + tire_deflation_delta_m) × fator` — contava a compressão estática do nariz DUAS VEZES (implícita em `[gear].h_cg_ground_m`, que já é a altura CARREGADA/em deflexão estática — ver docstring desse campo —, e explícita no curso TOTAL do batente). ANTES do ciclo 9 (translação vertical 1:1, ciclo 8, CAVEAT RESOLVIDO): `Δ_prop = nose_oleo_stroke_mm/1000 + tire_deflation_delta_m` (fator implícito 1) — simplificação otimista, pois a célula na realidade PIVOTA sobre o trem principal e a hélice (à frente do nariz) mergulha um braço AMPLIFICADO pelo fator acima (sempre > 1, invariante garantido pela validação composta `prop_plane_x_m < x_nose_m`). Baseline real E10: **+0,033 m (PASS, ciclo 8) → ≈−0,06416 m (FAIL, ciclo 9) → ≈−0,00249 m (FAIL, ciclo 10)** — MESMO veredito do ciclo 9 ao ciclo 10 (checagem #25 reprovava), fator ≈1,46610 (inalterado desde o ciclo 9). `old→new` (ciclo 10 → ciclo 11): desde o baseline E12 (`x_nose_m` 1,30→1,20, trem de nariz recuado) `prop_clearance_critical_m = +0,007367 m` e a checagem #25 **PASSA** — não reprova mais o baseline real HOJE. Ver `docs/backlog.md` ("transferência de atitude do #25", RESOLVIDO; item 6, RESOLVIDO ciclo 10). **CAVEAT DOS MAINS RÍGIDOS do ciclo 9 — RESOLVIDO no ciclo 10**: a fórmula pivota sobre os MAINS, mas NUNCA precisou de termo aditivo para eles — CS 23.925 pela LETRA só exige o trem CRÍTICO (nariz) no batente, os DEMAIS (mains) permanecem na deflexão ESTÁTICA já embutida em `[gear].h_cg_ground_m`/`ground_clearance_m` (a aeronave é sempre modelada CARREGADA). Não havia condição COMPOSTA não modelada — havia uma leitura imprecisa do que `h_cg_ground_m` representa. Ver `docs/backlog.md` (item 6, RESOLVIDO). **Nota independente, sinal OPOSTO e pequena, NÃO resolvida**: o disco da hélice também não é modelado como INCLINADO junto com o pitch da célula — CONSERVADOR em ≈+3,4 mm, ver `docs/backlog.md` (item 6) |

**Nota de consistência**: quando `source == "derivado"`, o diâmetro aqui
(autoritativo) pode divergir do `propulsion.prop_diameter_m` (provisório,
usado internamente para calcular rpm/BSFC/consumo de cruzeiro) — quando
isso ocorre, `warnings` contém um aviso explícito ("Diâmetro de hélice
derivado..."). Um consumidor de CAD deve preferir sempre `propeller.
diameter_m`, não `propulsion.prop_diameter_m`.

### `mission` — `MissionSpec` (MissionAgent — análise por segmentos)

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `fuel_taxi_kg` / `fuel_climb_kg` / `fuel_cruise_kg` / `fuel_descent_kg` / `fuel_reserve_kg` | f64 | kg | Combustível por segmento de missão |
| `fuel_total_kg` / `fuel_total_l` | f64 | kg / L | Combustível total da missão (soma dos segmentos + reserva) |
| `climb_time_min` | f64 | min | Duração da subida integrada |
| `climb_distance_km` / `descent_distance_km` / `cruise_distance_km` | f64 | km | Distâncias horizontais por segmento |
| `block_time_h` | f64 | h | Tempo total de voo (subida+cruzeiro+descida; NÃO inclui táxi) — **gate de autonomia do projeto** |
| `range_no_wind_km` | f64 | km | Alcance sem vento (soma dos segmentos) — **gate de alcance do projeto** |
| `breguet_range_full_tank_km` | f64 | km | **INFORMATIVO** — alcance Breguet se o tanque cheio inteiro fosse queimado em cruzeiro (não a missão real) |

### `electrical` — `ElectricalSpec` (ElectricalAgent)

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `bus_voltage_v` | f64 | V | Tensão do barramento |
| `alternator_w` | f64 | W | Capacidade nominal do alternador |
| `continuous_load_w` | f64 | W | Soma das cargas CONTÍNUAS configuradas |
| `peak_load_w` | f64 | W | Soma das cargas de PICO (pior caso, todas simultâneas — conservador) |
| `margin_continuous_pct` | f64 | % | Margem sobre a capacidade contínua do alternador |
| `loads` | array de objetos (`ElectricalLoadSpec`, **novo v4.7**) | — | Eco individual de cada `[electrical].loads` configurada — ver sub-tabela abaixo |

Sub-bloco `electrical.loads[]` (`ElectricalLoadSpec`, **novo v4.7** —
check #20): permite ao consumidor (e ao `ConstraintChecker::verify`)
comparar o pico DECLARADO de uma carga específica — em especial
`'trem_retratil'` — contra a potência COMPUTADA pelo agente responsável
(`landing_gear.actuator_power_w`), checagem só possível PÓS-convergência.

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `name` | string | — | Nome da carga (chave usada por `ConstraintChecker::verify` para localizar `'trem_retratil'`) |
| `continuous_w` | f64 | W | Potência CONTÍNUA declarada da carga |
| `peak_w` | f64 | W | Potência de PICO declarada da carga |

### `sizing` — `SizingReport` (Task 6.1 — `orchestrator::size_aircraft`)

Bloco novo na v4.0 — o laço de convergência de MTOW e o diagrama de
restrições já existiam desde as Tasks 3.1/3.2, mas não eram serializados.

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `mtow_mission_kg` | f64 | kg | MTOW de missão — peso convergido levando exatamente o combustível da missão mínima |
| `mtow_envelope_kg` | f64 | kg | MTOW de envelope — pior caso legal ("4 pax + bagagem + tanque cheio"); tipicamente ≥ `mtow_mission_kg`; dimensiona Estrutura/Trem de Pouso |
| `iterations` | array de f64 | kg | Trajetória de MTOW do laço de ponto fixo (primeiro palpite → valor final convergido) |
| `converged` | bool | — | Sempre `true` quando este bloco existe (se o laço não tivesse convergido, o pipeline teria abortado antes de gerar o relatório) |
| `fuel_required_l` | f64 | L | Combustível exigido pela missão = `mission.fuel_total_l` |
| `fuel_capacity_l` | f64 | L | Capacidade física do tanque configurado |
| `fuel_margin_l` | f64 | L | `fuel_capacity_l − fuel_required_l` |
| `fuel_margin_pct` | f64 | % | `fuel_margin_l / fuel_capacity_l × 100` |
| `constraints` | objeto (`WingLoadingReport`) | — | Diagrama de restrições clássico W/S × P/W (ver sub-tabela abaixo) |

Sub-bloco `sizing.constraints` (`WingLoadingReport`, Raymer cap. 5 /
Gudmundsson cap. 3):

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `ws_max_stall_n_m2` | f64 | N/m² | W/S máximo para atender V_stall com flap |
| `v_stall_ref_ms` | f64 | m/s | V_stall de referência usada (derivada de V_cruise/1,8) |
| `ws_optimal_cruise_n_m2` | f64 | N/m² | W/S ótimo de cruzeiro (mínimo arrasto) |
| `ws_actual_n_m2` | f64 | N/m² | W/S atual do projeto |
| `pw_min_climb_w_n` | f64 | W/N | P/W mínimo para a razão de subida requerida ao nível do mar |
| `pw_actual_w_n` | f64 | W/N | P/W atual (potência máxima contínua, nível do mar) |
| `recommended_wing_area_m2` | f64 | m² | Área de asa recomendada no W/S escolhido — **puramente informativo, não redimensiona a asa automaticamente** |
| `ws_chosen_n_m2` | f64 | N/m² | W/S escolhido para a recomendação = `min(ws_max_stall, ws_optimal_cruise)` |

### `robustness` — `RobustnessSpec` (RobustnessAgent — novo na v4.6)

Análise de robustez à incerteza do modelo de massas: as 7 massas
estruturais (`weight.structural_masses`, desde a v4.5) vêm de equações
empíricas de componente (Raymer cap. 15.2) ajustadas a uma FROTA
histórica, não a esta aeronave — incerteza típica de projeto conceitual de
±10-20% (Raymer/Roskam Classe II). Este bloco quantifica se checks que
PASSAM com as massas NOMINAIS continuariam passando sob essa incerteza —
não uma análise probabilística (sem RNG, sem distribuição): um **pior-caso
determinístico direcional**. Dois conjuntos adversariais de massa são
construídos (`sigma_mass_fraction` = σ): um empurra o CG vazio o mais para
a FRENTE possível (todo componente estrutural cujo braço fica à frente do
CG vazio nominal fica ×(1+σ), os demais ×(1−σ)), o outro o mais para TRÁS
(o espelho exato). Os dois conjuntos são reavaliados contra os limites
NOMINAIS já calculados nos blocos `weight`/`landing_gear` como ponto de
partida — esses limites (autoridade de profundor do bloco `trim`,
tetos/pisos de carga de nariz) são derivados de geometria/estabilidade,
não da massa estrutural em si. `old→new` (correção, fix wave ciclo 12):
esta frase terminava afirmando que os limites "são invariantes à
perturbação" — FALSO desde o ciclo 10 (task 2) para o limite dianteiro de
ROTAÇÃO especificamente, que depende do PESO (`x_cg_rot(W)`, ver bloco
`trim` acima) — o mundo perturbado tem pesos diferentes do nominal, logo
sua régua de rotação também difere. Medido HOJE (flips do baseline real):
`limite = 18,094655% MAC` sob perturbação contra `limite_nominal =
17,757974% MAC` no nominal — a régua ANDA. Os demais limites (tipback,
carga de nariz, gates de desempenho/pista, dimensionamento, hélice) SÃO
invariantes à perturbação, como o campo `robustness.flips[].
limite_nominal` documenta explicitamente (ver sub-bloco `RobustnessFlip`
abaixo, que já registrava esta exceção corretamente — só este parágrafo
introdutório estava desatualizado).

**Terceiro caso, "massa-total"** (**novo v4.7**): distinto dos dois
conjuntos direcionais acima (que só reavaliam CG/trem contra os limites
NOMINAIS já calculados, sem re-rodar o laço de convergência), o caso
massa-total multiplica as 5 massas estruturais COMPOSTAS (asa,
empenagens, fuselagem, trem, tanques) todas ×(1+σ) simultaneamente e
RE-RODA o laço COMPLETO de `orchestrator::size_aircraft` — MTOW e
combustível de missão podem mudar, não só o CG. O resultado fica em
`mtow_masstotal_kg`. Se o sizing perturbado FALHAR (`SizingError` — ex.:
combustível insuficiente sob o peso inflado), a checagem #19 registra um
flip nomeado "Dimensionamento" e `mtow_masstotal_kg` fica `0.0` (sem
significado físico nesse caso).

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `sigma_mass_fraction` | f64 | fração (0–1) | σ usado — eco de `[mass_model].sigma_mass_fraction` da configuração |
| `cg_fwd_case_pct_mac` | array [f64; 2] | %MAC | Faixa de CG observada nos 6 cenários de carga sob o conjunto CG-mais-DIANTEIRO (`[mínimo, máximo]`) |
| `cg_aft_case_pct_mac` | array [f64; 2] | %MAC | Idem sob o conjunto CG-mais-TRASEIRO |
| `flips` | array de objetos (`RobustnessFlip`) | — | Checks que PASSAM no nominal mas REPROVAM sob um dos TRÊS conjuntos adversariais (dois direcionais de CG + massa-total). **Array vazio = robusto** (nenhum check descoberto flipa) — não é ausência de dado, é o resultado positivo |
| `mtow_masstotal_kg` | f64 (**novo v4.7**) | kg | MTOW re-convergido pelo laço completo sob o caso massa-total (ver acima) — `0.0` se o sizing perturbado falhou (ver flip "Dimensionamento") |

Sub-bloco `robustness.flips[]` (`RobustnessFlip`):

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `check` | string | — | Nome do check que flipou. `old→new` (enumeração completada, fix wave ciclo 12 — a lista anterior listava só `"Cenário '<nome>'"`, `"Tipback"`, `"Carga de nariz máx"`/`"Carga de nariz mín"`, faltando os valores só alcançáveis pelo caso "massa-total"): `"Cenário '<nome>'"` (envelope de CG, casos dianteiro/traseiro), `"Tipback"`, `"Carga de nariz máx"`, `"Carga de nariz mín"` (casos dianteiro/traseiro), e — só sob o caso `"massa-total"` (`src/validation/robustness.rs`) — `"Dimensionamento"` (e a variante `"Dimensionamento (<detalhe do SizingError>)"`), `"Margem de combustível"`, `"VS0"`, `"Hélice (condição crítica CS 23.925)"`, além dos gates de desempenho/pista re-testados sob massa-total (`"Razão de subida"`, `"Velocidade de cruzeiro"`, `"Teto de serviço"`, `"Decolagem (grama, 15 m)"`, `"Pouso (grama, 15 m)"`) — lista não necessariamente exaustiva, ver o código-fonte para a enumeração autoritativa |
| `caso` | string | — | Qual conjunto adversarial derrubou o check. `old→new` (enumeração completada, fix wave ciclo 12): `"dianteiro"` \| `"traseiro"` (os dois casos direcionais de CG) \| `"massa-total"` (o terceiro caso, §`robustness.mtow_masstotal_kg` acima — faltava nesta enumeração) |
| `valor` | f64 | %MAC ou ° (conforme `check`) | Valor observado SOB perturbação |
| `limite` | f64 | mesma unidade de `valor` | Limite EFETIVAMENTE aplicado ao mundo perturbado. Até a v5.2, sempre igual ao limite NOMINAL (todos os limites de CG eram invariantes à massa). Desde o ciclo 10 (task 2, momento da linha de tração) o limite dianteiro de rotação passou a depender do peso — para os dois casos direcionais de CG este campo é a régua do PRÓPRIO mundo perturbado, que pode diferir da régua nominal |
| `limite_nominal` | f64 (**novo, v5.3**) | mesma unidade de `valor` | Limite NOMINAL do mesmo check — o que a régua valia ANTES da perturbação. Existe para separar as duas causas possíveis de um flip que `valor`/`limite` sozinhos confundem: **"o CG andou"** (`limite_nominal == limite`, o mundo perturbado moveu o CG do cenário através da MESMA régua) vs. **"a régua andou"** (`limite_nominal != limite`, possível desde que o limite dianteiro de rotação passou a responder à massa perturbada — ver item 2 da entrada v5.3 em §1). Para checks cuja régua é invariante à perturbação (tipback, carga de nariz, gates de desempenho/pista, dimensionamento, hélice), `limite_nominal` é IGUAL a `limite` por construção |

Cada flip em `robustness.flips` gera exatamente uma entrada em
`violations` (checagem #19 de `ConstraintChecker::verify`, ver §4
"`fidelity`, `violations`, `warnings`" abaixo): `"Robustez: {check} passa
no nominal mas reprova com massas estruturais ±{σ}% (pior caso {caso}):
{valor} vs {limite}"`.

### `fidelity`, `violations`, `warnings`

- `fidelity`: ver §3 acima.
- `violations`: array de strings, uma por requisito de projeto NÃO
  satisfeito (`ConstraintChecker::verify`). Vazio se e somente se
  `validation_status == "PASS"`.
- `warnings`: array de strings — condições que NÃO violam nenhum requisito
  do projeto, mas merecem atenção do time de CAD (ex.: diâmetro de hélice
  derivado divergente do provisório; pico elétrico acima da capacidade do
  alternador, coberto pela bateria). Pode ser vazio mesmo com
  `validation_status == "PASS"`; pode ser não-vazio mesmo com `"FAIL"`.

## 5. Nota especial: infinito em campos `f64` — `fatigue_life_cycles`, `to_50ft_paved_m`, `to_50ft_grass_m`, `to_distance_paved_m`, `to_distance_grass_m`, `landing_distance_m`

JSON (RFC 8259) não tem representação nativa de `Infinity`/`NaN`. A
biblioteca de serialização usada pelo pipeline (`serde_json`), por padrão,
converteria um `f64::INFINITY` silenciosamente para `null` — o que quebra
a desserialização de volta em `f64` (achado do próprio teste de round-trip
deste schema, `tests/schema_v4.rs`). O pipeline trata **SEIS** campos
especificamente com este comportamento, serializando `f64::INFINITY` como a
string `"infinita"` em vez de `null` (`old→new`: eram TRÊS até a v5.4;
subiu para seis na v5.5, ciclo 12 task 1):

1. **`structure.fatigue_life_cycles`** (v4.0+): "vida em fadiga infinita" é
   um resultado FISICAMENTE VÁLIDO do modelo de Goodman modificado — a
   longarina opera abaixo do limite de fadiga do material, não há ciclo
   limite de ruptura. Comportamento: infinito serializa como `"infinita"`.

2. **`performance.to_50ft_paved_m` e `performance.to_50ft_grass_m`** (v5.4+,
   ciclo 11, task 3): distância de decolagem sobre obstáculo de 15m pode ser
   infinita quando o obstáculo é inatingível — razão de subida ≤ 0 no
   segmento de subida até 15m (ver `agents::performance::
   takeoff_distance_50ft_m`, ramo `rc <= 0.0`). Isso ocorre quando a célula
   não consegue manter a altitude mínima para transpor o obstáculo com a
   carga/configuração testada — um resultado de FÍSICA válido, não um erro.
   Comportamento: infinito serializa como `"infinita"`.

3. **`performance.to_distance_paved_m`, `performance.to_distance_grass_m` e
   `performance.landing_distance_m`** (v5.5+, ciclo 12, task 1): com a
   rolagem de solo agora integrada numericamente (tasks 2/3 do mesmo ciclo,
   backlog item 4), o integrando pode divergir para `+INFINITY` quando a
   tração (decolagem) ou o arrasto+frenagem (pouso) não bastam para
   acelerar/desacelerar dentro do domínio de integração — ver
   `agents::performance::integra_rolagem_decolagem_com_passos`/
   `integra_rolagem_pouso_com_passos` (guarda: `F_net(V) ≤ 0` ⟹ integrando
   `+INFINITY` naquele nó de Simpson, nunca `NaN`). Resultado de FÍSICA
   válido ("decolagem/pouso impossível nesta condição"), não um erro.
   Comportamento: infinito serializa como `"infinita"`. **NÃO afeta**
   `ldg_50ft_m`/`ldg_50ft_grass_m` — estes dois continuam sempre `number`,
   sem tratamento especial (não têm `#[serde(with = "fatigue_life_serde")]`
   em `src/models/specs.rs`).

**Um parser de `aircraft_spec.json` deve tratar estes SEIS campos como
`number | "infinita"`, nunca assumir que são sempre números:**
- `structure.fatigue_life_cycles`
- `performance.to_50ft_paved_m`
- `performance.to_50ft_grass_m`
- `performance.to_distance_paved_m`
- `performance.to_distance_grass_m`
- `performance.landing_distance_m`

Nenhum outro campo do schema tem esse comportamento especial — em
particular, `performance.ldg_50ft_m`/`ldg_50ft_grass_m` são sempre
`number`, mesmo após o ciclo 12.

## 6. Manutenção deste documento

Ao adicionar/remover/renomear um campo em `src/models/specs.rs`:
1. Atualize a tabela do bloco correspondente aqui.
2. Se a mudança for aditiva (novo campo opcional/novo bloco), bump MINOR de
   `SCHEMA_VERSION` em `specs.rs` e registre em §1 acima.
3. Se a mudança quebrar compatibilidade (renomeia/remove/muda tipo ou
   unidade), bump MAJOR e registre a mudança em §1.
4. Rode `cargo test` (inclui `tests/schema_v4.rs`, que verifica a presença
   de todos os blocos de topo esperados e faz round-trip serde) e
   regenere `aircraft_spec.json` (`cargo run`) antes de commitar.
