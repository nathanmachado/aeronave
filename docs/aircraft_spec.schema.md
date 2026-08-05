# `aircraft_spec.json` — contrato do schema v4.1

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
    `[stability].cl_h_max_down`/`trim_margin`/`cl_ground_rotation`/
    `to_flap_cm_fraction` + `[wing].cm_ac`/`cm_flap_delta` (ver
    `config/aircraft/baseline_4seat.toml` para valores de referência).

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
| `trim` | preliminary (semi-empírico — Cm_ac/Cm_flap de literatura NACA 230/Raymer cap. 16; `cl_h_max_down` semi-empírico, Gudmundsson/Roskam) | Ensaio de voo (flare + rotação de decolagem) — resultado SENSÍVEL a `cl_h_max_down` (ver `trim.sensitivity` e §4 abaixo), não tratar como definitivo |
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
| `cd_cruise` | f64 | — | CD de cruzeiro |
| `cl_max` | f64 | — | CL_max com flap/slat (configuração pouso/decolagem) |
| `cl_max_clean` | f64 | — | CL_max em configuração limpa (cruzeiro) |
| `stall_speed_flaps_kmh` | f64 | km/h | VS0 — stall com flap |
| `stall_speed_clean_kmh` | f64 | km/h | VS1 — stall configuração limpa |
| `ld_ratio_cruise` | f64 | — | L/D em cruzeiro |

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
| `cg_limit_fwd_pct_mac` | f64 | %MAC | Limite DIANTEIRO admissível — desde a v4.1, o PIOR CASO (maior %MAC) entre os limites por cenário calculados pelo `TrimAuthorityAgent` (bloco `trim` abaixo), não mais o proxy `[stability].sm_max`. Não confundir com `cg_mac_fwd_pct` acima (valor OBSERVADO). O veredito de aceite por cenário fica em `trim.rotation_limit_pct_mac_per_scenario`/`violations`, não neste agregado isolado. |
| `cg_limit_aft_pct_mac` | f64 | %MAC | Limite TRASEIRO admissível — de `[stability].sm_min` |

### `trim` — `TrimSpec` (TrimAuthorityAgent — novo na v4.1)

Limite dianteiro FÍSICO do envelope de CG, derivado da autoridade de
profundor disponível nas duas manobras críticas de arfagem
nariz-para-cima: **flare no pouso** (V_ref = 1,3·Vs0, flap de pouso,
balanço de momentos em torno do CG) e **rotação na decolagem** (Vr =
1,1·Vs0, flap de decolagem, balanço de momentos em torno do TREM
PRINCIPAL). Substitui o proxy `[stability].sm_max` (removido — ver §1).

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `flare_limit_pct_mac` | f64 | %MAC | Limite dianteiro de flare — independe do peso do cenário |
| `rotation_limit_pct_mac_per_scenario` | array de objeto (`ScenarioTrimLimit`) | — | Um limite de rotação por cenário de carga (`weight`'s cenários) — depende do peso |
| `governing` | string (`"flare"` \| `"rotacao"` \| `"misto"`) | — | Qual manobra governa, agregado sobre todos os cenários |
| `cl_h_required_at_fwd_limit` | f64 | — | CL_h requerido no limite de flare resolvido (checagem de sanidade — coincide com `cl_h_available` por construção) |
| `cl_h_available` | f64 | — | CL_h disponível — `-cl_h_max_down·(1−trim_margin)` |
| `sensitivity` | objeto (`TrimSensitivity`) | — | Limite de flare recomputado a `cl_h_max_down ± 0,05` |
| `cm_ac` / `cm_flap_delta` | f64 | — | Parâmetros ecoados de `[wing]` |
| `cl_h_max_down` / `trim_margin` / `cl_ground_rotation` / `to_flap_cm_fraction` | f64 | — | Parâmetros ecoados de `[stability]` |

Sub-bloco `ScenarioTrimLimit` (um por cenário de `weight`):

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `scenario` | string | — | Nome do cenário (mesmo do bloco `weight`) |
| `rotation_limit_pct_mac` | f64 | %MAC | Limite dianteiro de rotação NESTE cenário |
| `governing_limit_pct_mac` | f64 | %MAC | `max(flare_limit_pct_mac, rotation_limit_pct_mac)` — limite EFETIVO deste cenário |
| `governing` | string (`"flare"` \| `"rotacao"`) | — | Qual manobra governa NESTE cenário |

Sub-bloco `sensitivity` (`TrimSensitivity`):

| Campo | Tipo | Unidade | Descrição |
|---|---|---|---|
| `cl_h_max_down_minus` / `cl_h_max_down_plus` | f64 | — | `cl_h_max_down ∓ 0,05` |
| `flare_limit_pct_mac_minus` / `flare_limit_pct_mac_plus` | f64 | %MAC | Limite de flare recomputado com o parâmetro acima |

**ACHADO DE PROJETO honesto** (baseline real, não um bug deste código): a
ROTAÇÃO governa em TODOS os cenários (≈29,6%–40,2% MAC conforme o peso),
MUITO mais restritiva que a flare (≈5,5% MAC) e que o antigo proxy
`sm_max` (16,6% MAC) — causa física: o trem principal
(`[gear].x_main_m`) fica muito atrás do CG desta célula. Ver
`agents::trim_authority` (docstring do módulo) para a dedução completa.

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
| `tipover_angle_deg` | f64 | deg | Ângulo anti-tombamento lateral (deve ser < 55°) |
| `nose_load_fraction_pct` | f64 | % | Fração de carga no trem de nariz (ideal 8–20%) |
| `main_gear_load_n` | f64 | N | Carga máxima no trem principal (por perna) |
| `nose_gear_load_n` | f64 | N | Carga máxima no trem de nariz |
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
