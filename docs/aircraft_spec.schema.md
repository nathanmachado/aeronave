# `aircraft_spec.json` — contrato do schema v4.4

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
    `[stability].trim_margin`/`cl_ground_rotation`/`to_flap_cm_fraction` +
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
       fixo na lista de massas.
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
| `weight` | preliminary (soma de itens de massa configurados NÃO pesados) | Pesagem em balança de cada item antes da fabricação — a aritmética é exata, mas os valores de entrada são estimativas de catálogo/projeto, não massas medidas; erros aqui se propagam para MTOW/estrutura/trem de pouso |
| `trim` | preliminary (semi-empírico — Cm_ac/Cm_flap de literatura NACA 230/Raymer cap. 16; `cl_h_max_down_calc` CALCULADO por geometria DATCOM/Nelson (`τ(c_e/c)`, ajuste empírico de Nelson — válido em c_e/c ∈ [0.1, 0.6]); rotação DESCONSIDERA o binário tração/arrasto/inércia, resíduo estimado ≈ μ_roll·(W−L_g)·h_cg) | Ensaio de voo (flare + rotação de decolagem) — resultado SENSÍVEL a `elevator_deflection_max_deg` (±2°) e a `cl_h_max_down` (±0.05 residual) (ver `trim.sensitivity` e §4 abaixo), não tratar como definitivo |
| `performance` | computed (equações fechadas, atmosfera ISA padrão) | — |
| `vn_diagram` | computed (CS 23.333/.335/.337/.341, fórmulas fechadas) | — |
| `structure` | preliminary (vigas simplificadas — viga I equivalente); flutter: preliminary (estimativa analítica) | FEM (estrutura); GVT — ensaio de vibração em solo (flutter) |
| `landing_gear` | preliminary (dimensionamento estático de cargas) | Análise dinâmica de pouso/afundamento |
| `propeller` | semi-empirical (Mach de ponta + folga de solo) | Mapa de desempenho de hélice real do fabricante |
| `mission` | computed (segmentos + equação de Breguet, L/D constante em cruzeiro) | — |
| `electrical` | preliminary (soma de cargas nominais configuradas) | Análise transiente/térmica real |
| `sizing` | computed (laço de convergência de ponto fixo) | — |

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
| `cl_max` | f64 | — | CL_max com flap/slat (configuração pouso/decolagem) |
| `cl_max_clean` | f64 | — | CL_max em configuração limpa (cruzeiro) |
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
| `trim_margin` / `cl_ground_rotation` / `to_flap_cm_fraction` | f64 | — | Parâmetros ecoados de `[stability]` |
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
| `ldg_50ft_m` | f64 | m | Distância de pouso sobre obstáculo de 15 m/50 ft |

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
| `ground_clearance_m` | f64 | m | Folga ponta de pá ↔ solo |
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
