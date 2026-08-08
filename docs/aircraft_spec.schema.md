# `aircraft_spec.json` — contrato do schema v5.0

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
    CL_h_trim = [cm_ac + CL_cruise·(x̄_cg−0,25)] / [η_h·(S_h/S_w)·(l_h/MAC+0,25−x̄_cg)]
    ΔCD_trim = (CL_h_trim²/(π·ar_h·e_h))·(S_h/S_w)
    ```
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
    rotação, número único invariante ao peso) recua de **12,995% para
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
tabela abaixo lista o tipo de análise esperada por bloco.

| Bloco | Fidelidade típica (Task 6.1) | Análise posterior recomendada se `preliminary` |
|---|---|---|
| `wing` | semi-empirical (polar por build-up: CD0 por componente + Oswald empírico) | — |
| `propulsion` | semi-empirical (curvas de catálogo do motor + BSFC paramétrico) | — |
| `geometry` | computed (derivado da configuração + `WeightBalanceAgent`) | — |
| `empennage` | preliminary (coeficiente de volume, Raymer Tab. 6.4) | VLM/CFD para eficiência real de downwash/sidewash |
| `control_surfaces` | preliminary (frações históricas, Raymer Tab. 6.5) | Análise de autoridade/eficiência de controle |
| `weight` | **v4.5**: semi-empirical (estruturas: Raymer 15.2 GA × fatores de composto Tab. 15.4; hardware: itens configurados NÃO pesados) | Pesagem em balança de cada item antes da fabricação — as 7 massas estruturais (`weight.structural_masses`) vêm de equações semi-empíricas de componente, mas o hardware/instalação (aviônicos, bateria, cabos etc.) ainda é estimativa de catálogo/projeto, não massa medida; erros aqui se propagam para MTOW/estrutura/trem de pouso |
| `trim` | preliminary (semi-empírico — Cm_ac/Cm_flap de literatura NACA 230/Raymer cap. 16; `cl_h_max_down_calc` CALCULADO por geometria DATCOM/Nelson (`τ(c_e/c)`, ajuste empírico de Nelson — válido em c_e/c ∈ [0.1, 0.6]); rotação DESCONSIDERA o binário tração/arrasto/inércia, resíduo estimado ≈ μ_roll·(W−L_g)·h_cg) | Ensaio de voo (flare + rotação de decolagem) — resultado SENSÍVEL a `elevator_deflection_max_deg` (±2°) e a `cl_h_max_down` (±0.05 residual) (ver `trim.sensitivity` e §4 abaixo), não tratar como definitivo |
| `performance` | computed (equações fechadas, atmosfera ISA padrão); **ciclo 8 task 1**: polar de subida/gradiente inclui arrasto de flap parcial (`wing.cd0_flap_to_extra`); rolagem de solo (energético) e aproximação de pouso (ângulo fixo) seguem sem termo de arrasto por construção; `climb_gradient_pct` AINDA tem viés otimista remanescente (avaliado no piso da varredura, 1,05·Vs, abaixo do ≥1,2·Vs típico da CS 23.65 — achado da revisão, pré-existente) | Reavaliar a velocidade de referência de `best_climb_angle_ms` (item de ciclo futuro) |
| `vn_diagram` | computed (CS 23.333/.335/.337/.341, fórmulas fechadas) | — |
| `structure` | preliminary (vigas simplificadas — viga I equivalente); flutter: preliminary (estimativa analítica) | FEM (estrutura); GVT — ensaio de vibração em solo (flutter) |
| `landing_gear` | preliminary (dimensionamento estático de cargas) | Análise dinâmica de pouso/afundamento |
| `propeller` | semi-empirical (Mach de ponta + folga de solo) | Mapa de desempenho de hélice real do fabricante |
| `mission` | computed (segmentos + equação de Breguet, L/D constante em cruzeiro) | — |
| `electrical` | preliminary (soma de cargas nominais configuradas) | Análise transiente/térmica real |
| `sizing` | computed (laço de convergência de ponto fixo) | — |
| `robustness` | **v4.7**: computed (pior-caso determinístico ±σ direcional sobre as 7 massas estruturais; limites de envelope nominais — invariantes a massa; caso massa-total: re-sizing completo com fatores ×(1+σ)) | — (o próprio bloco É a análise posterior de sensibilidade das 7 massas estruturais `semi-empirical`/`preliminary`; nenhuma análise adicional recomendada) |

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
| `cd0_flap_to_extra` | f64 (**novo, ciclo 8 task 1 — ainda dentro de v5.0, bump formal para v5.1 pendente de §3/§4 do mesmo ciclo**) | — | ΔCD0 do flap PARCIAL de decolagem = `to_flap_fraction · [wing].cd0_flap_delta` — mesma fração de `cl_max_to` acima, agora aplicada ao arrasto. Consumido por `agents::performance::excess_power_kw` no segmento de SUBIDA da decolagem (`to_50ft_paved_m`/`to_50ft_grass_m`) e no gradiente CS 23.65 (`performance.climb_gradient_pct`, avaliado em Vx). Fecha a lacuna "não existe modelo de flap na polar deste crate" declarada desde o ciclo 7 — ver `fidelity.performance` para o detalhe de quais segmentos consomem/não consomem a polar |
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
| `prop_efficiency` | f64 | — | Eficiência de hélice em cruzeiro |
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
atual tem o envelope FECHADO, ≈6,1% < ≈43,5%, ver bloco `trim` abaixo),
NENHUM CG é admissível — os dois
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
NÃO variam por cenário de carga.** Isto é um resultado NÃO ÓBVIO para a
rotação especificamente: apesar do balanço de momentos físico depender do
peso do cenário (`W`), sob a política de velocidade `Vr = 1,1·Vs0(W)`
usada por este modelo, a pressão dinâmica de rotação `q_r(W)` é
PROPORCIONAL a `W` — logo TODOS os termos de momento em jogo (download da
empenagem, sustentação da asa, momento de perfil+flap) também são
proporcionais a `W`, e o `W` CANCELA EXATAMENTE ao calcular a posição do
CG-limite (`x_cg_rot = x_main − M_disponível(W)/W`). Ver a dedução
completa (em português) na docstring de
`agents::trim_authority::rotation_fwd_limit_m`.

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `flare_limit_pct_mac` | f64 | %MAC | Limite dianteiro de flare — número único, independe do peso |
| `rotation_limit_pct_mac` | f64 | %MAC | Limite dianteiro de rotação — número único, INVARIANTE ao peso (ver acima) |
| `rotation_margin_per_scenario` | array de objeto (`ScenarioTrimLimit`) | — | Diagnóstico informativo POR CENÁRIO — margem de autoridade de rotação avaliada na CG/peso REAIS de cada cenário (essa sim varia por cenário) — NÃO usado para calcular `rotation_limit_pct_mac`/`inside_envelope` |
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

**Baseline real** (`config/aircraft/baseline_4seat.toml`, pós task
refino-ciclo2): `cl_h_max_down_calc ≈ 1,0577` (c_e/c=0,40, AR_h=4,0,
δe_max=25° — abaixo do teto de stall 1,10, `capped_by_stall=false`),
+11,3% sobre o antigo palpite de config (0,95). A ROTAÇÃO ainda governa
(≈6,10% MAC), mas agora bem mais à FRENTE do que antes de a autoridade
ser calculada (≈10,95% MAC) — e continua ATRÁS do limite traseiro
(≈43,46% MAC): **envelope de CG FECHADO**, com margens de autoridade de
rotação POSITIVAS em todos os 6 cenários reais (≈+26% a +207%). A flare
fica NEGATIVA (≈-16,29% MAC — fisicamente "antes do bordo de ataque",
nunca governa). Achado de projeto que PERSISTE (não corrigido por esta
task, decisão de layout humana): o trem principal (`[gear].x_main_m`)
continua sendo a causa raiz de a ROTAÇÃO (não a flare) governar o limite
dianteiro — ver `agents::trim_authority` (docstring do módulo) para a
dedução completa.

### `performance` — `PerformanceSpec` (PerformanceAgent)

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `v_cruise_kmh` / `v_stall_kmh` | f64 | km/h | Velocidades de cruzeiro / stall |
| `rc_sl_ms` / `rc_cruise_alt_ms` | f64 | m/s | Razão de subida ao nível do mar / na altitude de cruzeiro |
| `service_ceiling_m` | f64 | m | Teto de serviço |
| `to_distance_paved_m` / `to_distance_grass_m` | f64 | m | Distância de decolagem (rolagem simples, pista pavimentada/grama) |
| `landing_distance_m` | f64 | m | Distância de pouso (rolagem simples) |
| `range_km` / `endurance_h` | f64 | km / h | **INFORMATIVO** — eco de `propulsion.range_km`/`endurance_h`; não é o gate do projeto |
| `vx_kmh` / `vy_kmh` | f64 | km/h | Velocidade de melhor ângulo / melhor razão de subida |
| `best_glide_kmh` / `glide_ratio` | f64 | km/h / — | Velocidade e razão L/D de melhor planeio |
| `climb_gradient_pct` | f64 | % | Gradiente de subida em Vx, solo, MTOW (CS 23.65 exige ≥ 8,3%) |
| `to_50ft_paved_m` / `to_50ft_grass_m` | f64 | m | Distância de decolagem sobre obstáculo de 15 m/50 ft |
| `ldg_50ft_m` | f64 | m | Distância de pouso sobre obstáculo de 15 m/50 ft, pista PAVIMENTADA (`mu_brake_paved`) — **INFORMATIVO** desde a v4.8: não é o gate de pista |
| `ldg_50ft_grass_m` | f64 | m | Distância de pouso sobre obstáculo de 15 m/50 ft em GRAMA (`mu_brake_grass`) — sempre > `ldg_50ft_m`; é a grandeza gateada pela checagem #24 contra `runway_available_m` |

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
(o espelho exato). Os dois conjuntos são reavaliados contra os MESMOS
limites NOMINAIS já calculados nos blocos `weight`/`landing_gear` — esses
limites (autoridade de profundor do bloco `trim`, tetos/pisos de carga de
nariz) são derivados de geometria/estabilidade, não da massa estrutural em
si, logo são invariantes à perturbação.

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
| `check` | string | — | Nome do check que flipou — `"Cenário '<nome>'"` (envelope de CG), `"Tipback"`, `"Carga de nariz máx"` ou `"Carga de nariz mín"` |
| `caso` | string | — | Qual conjunto adversarial derrubou o check: `"dianteiro"` \| `"traseiro"` |
| `valor` | f64 | %MAC ou ° (conforme `check`) | Valor observado SOB perturbação |
| `limite` | f64 | mesma unidade de `valor` | Limite NOMINAL violado |

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

## 5. Nota especial: `fatigue_life_cycles` e infinito

JSON (RFC 8259) não tem representação nativa de `Infinity`/`NaN`. A
biblioteca de serialização usada pelo pipeline (`serde_json`), por padrão,
converteria um `f64::INFINITY` silenciosamente para `null` — o que quebra
a desserialização de volta em `f64` (achado do próprio teste de round-trip
deste schema, `tests/schema_v4.rs`). Como "vida em fadiga infinita" é um
resultado FISICAMENTE VÁLIDO do modelo de Goodman modificado (a longarina
opera abaixo do limite de fadiga do material), o pipeline serializa esse
caso especificamente como a string `"infinita"` em vez de `null` ou de um
número.

**Um parser de `aircraft_spec.json` deve tratar `structure.
fatigue_life_cycles` como `number | "infinita"`, nunca assumir que é
sempre um número.** Nenhum outro campo do schema tem esse comportamento
especial — é específico deste campo.

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
