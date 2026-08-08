# Ciclo 5 — Robustez de Massa Total e Segurança de Solo — Design

**Data:** 2026-08-07 · **Baseline de partida:** pós-ciclo-4 (`a1ad5dd`, FAIL honesto com 3 violações nominais, schema 4.6)

## Problema

Três lacunas expostas pelo ciclo 4 e pela campanha E9 (108 execuções, 2026-08-07):

1. **O check #19 só cobre deslocamento de CG.** Os dois conjuntos adversariais (±σ direcional) maximizam viagem de CG; nenhum conjunto maximiza a MASSA TOTAL — margem de combustível, VS0, razão de subida, cruzeiro e autonomia (todos sensíveis ao MTOW) não têm cobertura de robustez. Limitação de plano documentada na revisão final do ciclo 4: o princípio "o modelo falha no ponto de perigo" está meio-entregue.
2. **A folga de hélice usa um datum desacoplado do trem.** `PropellerAgent` já computa `ground_clearance_m` e o checker já reprova folga insuficiente — mas a partir de `[propeller].shaft_height_m` ABSOLUTO (1,25 m). A campanha E9 encurtou o trem 13 cm (`h_cg_ground_m` 1,05→0,92) e a folga reportada não se moveu: com o datum físico correto, o eixo desce para 1,12 m e a folga do E9 é **0,145 m < 0,23 m** — o "PASS robusto" do E9 não viu isso. É exatamente a dessincronia silenciosa que os ciclos 3–4 eliminaram nas massas. **Adoção do E9 está suspensa até este item existir** (decisão do usuário, 2026-08-07).
3. **Cobertura perdida no ciclo 3:** a guarda `electrical.loads['trem_retratil'].peak_w ≥ potência do atuador` foi removida (a entrada morreu no parse); a potência computada (`GearSpec.actuator_power_w`, 31,1 W no baseline) não é comparada a nada.

Mais um débito de teste: `flutter_speed_kmh` caiu 6,3% no ciclo 3 sem nenhum pin.

## Decisões (do usuário, 2026-08-07)

- **§1 por re-sizing completo:** o caso "massa-total" roda `size_aircraft` inteiro com fatores perturbados — física completa, sem aproximação nova para auditar. (Descartado: aproximação algébrica no ponto convergido — subestima porque o combustível de missão cresce com o MTOW.)
- **§3 por offset eixo−CG:** `prop_axis_above_cg_m` fixo da célula; altura do eixo deriva de `h_cg_ground_m` — encurtar trem come folga 1:1 automaticamente. (Descartado: manter altura absoluta — recria a dessincronia.)

## Design

### §1 — Caso "massa-total" no check #19 (re-sizing completo)

- `RobustnessAgent` ganha o 3º caso adversarial: clona o `AircraftConfig` em memória multiplicando os 5 `composite_factor_*` por `(1+σ)` (σ = `sigma_mass_fraction` existente; SEM re-passar pelas faixas de parse — é um what-if físico e o produto pode exceder a faixa de config, documentado no código) e roda `orchestrator::size_aircraft` completo com esse clone.
- **Se o sizing falhar** (`SizingError`): flip único `{"check": "Dimensionamento", "caso": "massa-total", valor/limite: MTOW ou combustível do erro}`.
- **Se convergir** (`sized_p`): reavalia no mundo +σ, flipando o que passa no nominal e reprova perturbado:
  - margem de combustível ≥ `min_fuel_margin_fraction` (mesma fórmula do check #18);
  - `VS0 ≤ cruise_speed_min_kmh/1.8` (CS-23, mesma fórmula do check #2);
  - `rc_sl_ms ≥ 1.5` e `v_cruise ≥ cruise_speed_min_kmh` e `endurance ≥ endurance_min_h` (via `PerformanceAgent`/`MissionSpec` do mundo perturbado — mesmos gates do pipeline nominal).
- Flips do caso novo entram na MESMA lista `robustness.flips` (campo `caso: "massa-total"`), viram violações #19 pelo mecanismo existente. `RobustnessSpec` ganha `mtow_masstotal_kg: f64` (rastreabilidade do MTOW perturbado; `None`/ausente não existe — se o sizing falhar, ecoa o MTOW do erro ou 0.0 com flip de Dimensionamento presente, documentado).
- Custo: 1 laço de sizing extra por rodada (~ms), determinístico.
- Expectativa no baseline (verificar, não forçar): estrutural +~62 kg → MTOW +~75 kg, margem de combustível 14,3→~11%, sem flip provável.

### §2 — Check #20: atuador de retração vs orçamento elétrico

- `ElectricalSpec` passa a ecoar as cargas configuradas: `pub loads: Vec<ElectricalLoadSpec>` com `{name: String, continuous_w: f64, peak_w: f64}` (espelho do config, preenchido pelo `ElectricalAgent`).
- Check #20 no `ConstraintChecker::verify`: localizar a carga `"trem_retratil"` em `electrical.loads`; se ausente → violação (aeronave de trem retrátil sem carga declarada); se presente e `peak_w < gear.actuator_power_w` → violação nomeando os dois valores. Fecha a cobertura perdida no ciclo 3 — agora no lugar certo (pós-convergência, com a potência COMPUTADA).

### §3 — Folga de hélice recoplada ao trem (gate do E9)

- `[propeller].shaft_height_m` **removido com erro de migração** citando o substituto (padrão dos ciclos 3–4).
- Campo novo `[propeller].prop_axis_above_cg_m` — offset vertical FIXO da célula entre o eixo da hélice e o CG: baseline **0,20** (= 1,25 − 1,05 de hoje: baseline numericamente IDÊNTICO após a troca), faixa **(−0,3, 0,8)**. Fixture sintética: **0,12** (= 1,15 − 1,03 dos valores atuais da fixture — `shaft_height_m` 1,15 e `h_cg_ground_m` 1,03 — folga sintética idêntica após a troca, e valor distinto do baseline). String TOML de teste do config.rs: idem (shaft 1,20 vira offset coerente com o h_cg daquela string). Comentário no TOML real: valor derivado da geometria atual; validar no CAD (Fase 3).
- `PropellerAgent::run`: `shaft_height = cfg.gear.h_cg_ground_m + cfg.propeller.prop_axis_above_cg_m` — única mudança de fórmula; `ground_clearance_m`, `diameter_max_by_clearance_m` e `ok_clearance` seguem como estão, agora com o datum acoplado.
- Consequência esperada: baseline INALTERADO (0,275 m de folga). Validação E9: folga 0,92+0,20−1,95/2 = **0,145 < 0,23 → FAIL honesto** — o relatório do ciclo apresenta as saídas físicas (hélice ≤1,78 m com custo de Mach/tração; trem menos curto com perda de robustez de tipback; ou solução no plano da asa).

### §4 — Pin de flutter

- `flutter_speed_kmh` pinado honesto no teste de integração que roda o baseline real (tests/generic_engine.rs ou vn_diagram.rs, onde couber melhor — valor atual ~702,6 km/h, tolerância no padrão dos pins vizinhos, old→new comentado citando a queda de 6,3% do ciclo 3).

### Schema 4.6 → 4.7

- `electrical.loads` (lista nova); `robustness.mtow_masstotal_kg` + flips com `caso: "massa-total"`. `docs/aircraft_spec.schema.md`: histórico 4.7. `fidelity`: entrada `robustness` atualizada mencionando o caso massa-total.

## Tratamento de erros

- Campo novo fora de faixa → erro de validação padrão; `shaft_height_m` presente → erro de migração.
- Falha de sizing no mundo +σ NÃO é erro do pipeline: vira flip de "Dimensionamento" (o mundo nominal continua válido).
- Panics de invariante seguem o padrão existente.

## Testes (TDD)

1. **§1:** flip de Dimensionamento com config sintética que quebra no +σ (ex.: tanque apertado que estoura com MTOW perturbado); config sintética marginal em margem de combustível flipa com caso "massa-total"; σ pequeno ≈ nominal (sem flips novos); baseline real assere o resultado honesto (o que for).
2. **§2:** violação quando `peak_w` declarado < atuador computado (config sintética mutada); violação quando a carga `trem_retratil` não existe; caminho PASS na fixture.
3. **§3:** migração de `shaft_height_m`; rejeição de `prop_axis_above_cg_m` fora de faixa; hand-check da folga com h_cg + offset; property: reduzir `h_cg_ground_m` reduz `ground_clearance_m` na mesma medida (o acoplamento que o ciclo garante).
4. **§4:** pin honesto de flutter.
5. **Pins honestos** em toda a suite; genericidade verde.

## Sequência do ciclo e critério de conclusão

1. Implementação (plano SDD em worktree).
2. Rodada do baseline: `aircraft_spec.json` regenerado (schema 4.7); esperado FAIL com as mesmas 3 violações nominais (folga baseline inalterada; #20 provavelmente PASS com 520 W declarados vs 31 W computados; massa-total sem flip provável) — o que o modelo disser.
3. **Validação E9** (célula bateria 53 kg / x_nose 1,30 / h_cg 0,92 / pernas 0,54/0,40): rodar com os checks novos; esperado FAIL honesto na folga de hélice (0,145 < 0,23). Relatório apresenta o veredito e as alternativas físicas quantificadas (d_max por folga na altura E9; h_cg mínimo para manter d=1,95 = 1,205−0,20 = 1,005 m).
4. Decisão humana posterior: destino do E9 (hélice menor × trem menos curto × outra alavanca) e eventual campanha E10.

## Fora de escopo

- Adoção do E9 (suspensa; decisão do usuário após o item 3 da sequência).
- Robustez em grandezas não-massa (aerodinâmica, propulsão).
- Modelagem de pneu murcho/amortecedor comprimido na folga (CS 23.925 completo) — o piso 0,23 m (Raymer 9 in) segue como proxy único.
- `verify` com 14+ params → struct (registrado no backlog, momento calmo).
