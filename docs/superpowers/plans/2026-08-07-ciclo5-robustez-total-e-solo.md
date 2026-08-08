# Ciclo 5 — Robustez de Massa Total e Segurança de Solo — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Caso adversarial "massa-total" no check #19 (re-sizing completo ×(1+σ)), check #20 (atuador vs orçamento elétrico), folga de hélice recoplada ao trem (`prop_axis_above_cg_m`, gate do E9) e pin de flutter — schema 4.6→4.7.

**Architecture:** Spec: `docs/superpowers/specs/2026-08-07-ciclo5-robustez-total-e-solo-design.md`. Extensões cirúrgicas: `validation/robustness.rs` ganha o 3º caso (clone do cfg + `size_aircraft` + `PerformanceAgent`); `ElectricalSpec` ecoa as cargas; `PropellerAgent` deriva `shaft_height` de `h_cg_ground_m + prop_axis_above_cg_m` (remoção com migração de `shaft_height_m`). O ciclo termina na validação honesta da célula E9.

**Tech Stack:** Rust, serde/TOML, sem dependências novas.

## Global Constraints

- Pins honestos: old→new comentado, tolerâncias INALTERADAS; baseline esperado: FAIL com as MESMAS 3 violações nominais (folga baseline numericamente idêntica: 1,05+0,20 = 1,25); surpresa >5% ou flip de veredito → investigar antes de pinar.
- Dados só em TOML (faixa + rejection test + fixture distinta); campos removidos → erro de migração citando o substituto.
- Determinismo total; Português; referências citadas; `cargo test` verde ao fim de cada task; genericidade (grep) verde.
- NUNCA mascarar: o veredito E9 da Task 5 é o que o modelo disser.

## Fatos do código atual (verificados em `8998b56`)

- `PropellerAgent::run(cfg: &AircraftConfig, engine, prop_spec, req) -> PropellerSpec` (`src/agents/propeller.rs:96`); usa `pcfg.shaft_height_m` nas linhas ~120 (d_max por folga) e ~132 (`ground_clearance_m = shaft_height − D/2`); `ok_clearance` na ~145; checker já reprova `!propeller.ok_clearance` (constraint_checker.rs:254).
- `PropellerCfg.shaft_height_m` (aircraft_config.rs:198), fixture `1.15` (:607) com `h_cg_ground_m: 1.03` (:616); string TOML de teste `shaft_height_m = 1.20` (config.rs:1395) — conferir o `h_cg_ground_m` da MESMA string para derivar o offset equivalente; `require_positive("propeller.shaft_height_m", ...)` (config.rs:653); rejection test `rejeita_shaft_height_m_nao_positivo` (config.rs:1564).
- `ElectricalLoadCfg {name: String, continuous_w: f64, peak_w: f64}` (aircraft_config.rs:453); `ElectricalAgent::run(cfg) -> ElectricalSpec` (electrical.rs:32); `ElectricalSpec {bus_voltage_v, alternator_w, continuous_load_w, peak_load_w, margin_continuous_pct}` (specs.rs:754) — SEM as cargas individuais; carga `"trem_retratil"` existe no baseline TOML (linha ~421).
- `GearSpec.actuator_power_w` existe (31,1 W no baseline).
- `RobustnessAgent::run(cfg, engine, req, state, wing, emp, masses, wb_nominal, gear_nominal) -> RobustnessSpec`; `RobustnessSpec {sigma_mass_fraction, cg_fwd_case_pct_mac: [f64;2], cg_aft_case_pct_mac: [f64;2], flips: Vec<RobustnessFlip>}`; `RobustnessFlip {check, caso, valor, limite}`; chamada em `src/main.rs:~520`, na fixture de teste do constraint_checker (~516-522), em `tests/schema_v4.rs` e `tests/gear_tipback.rs` (×2).
- `orchestrator::size_aircraft(cfg, engine, req) -> Result<SizedAircraft, SizingError>`; `SizedAircraft {state, wing, prop, wb, trim, emp, mission_fuel_kg, mission, constraints, iterations, ...}`; `MissionSpec.fuel_total_l`.
- `PerformanceAgent::run(state, wing, prop, mtow_kg, engine, req, &cfg.performance) -> PerformanceSpec` (`rc_sl_ms`, `v_cruise_kmh`, `service_ceiling_m`, `endurance_h`); `wing.stall_speed_flaps_kmh`.
- Check #18 (margem): fração da CAPACIDADE — `(capacity_l − fuel_required_l)/capacity_l ≥ req.min_fuel_margin_fraction`. Check #2 (VS0): `stall_speed_flaps_kmh ≤ cruise_speed_min_kmh/1.8`.
- `MassModelCfg.composite_factor_{wing,tail,fuselage,gear,fuel_system}`.
- `SCHEMA_VERSION = "4.6"` (specs.rs); flutter: `structure.flutter_speed_kmh` (702,596 km/h no aircraft_spec.json atual); `tests/cli.rs` roda o binário e assere campos do JSON.
- **Nota de fidelidade à spec:** a spec §1 lista "autonomia ≥ endurance_min_h" entre os checks reavaliados; a autonomia é garantida POR CONSTRUÇÃO pelo `MissionAgent` (dimensiona a missão para a autonomia mínima ou o sizing retorna `CombustivelInsuficiente`) — mesma razão pela qual o checker removeu o antigo check vazio de `block_time`. O flip "Dimensionamento" cobre exatamente esse modo de falha; documentar no código em vez de criar check vazio.

---

### Task 1: Folga de hélice recoplada (`prop_axis_above_cg_m`)

**Files:**
- Modify: `src/models/aircraft_config.rs` (troca do campo em `PropellerCfg` + fixture)
- Modify: `src/models/config.rs` (migração + faixa + rejection tests + string TOML de teste)
- Modify: `config/aircraft/baseline_4seat.toml` (`[propeller]`)
- Modify: `src/agents/propeller.rs` (derivação do shaft_height + testes)
- Test: pins — NENHUM valor do baseline muda (0,275 m idêntico); qualquer golden que mudar é bug.

**Interfaces:**
- Produces: `cfg.propeller.prop_axis_above_cg_m: f64`; `PropellerAgent` deriva `shaft_height = cfg.gear.h_cg_ground_m + cfg.propeller.prop_axis_above_cg_m`.

- [ ] **Step 1: Testes (RED)**

```rust
// propeller.rs (mod tests) — property do ACOPLAMENTO (o motivo do ciclo):
/// Encurtar o trem reduz a folga NA MESMA MEDIDA — o datum recoplado
/// garante que h_cg e folga andam juntos (falha do E9 que motivou o ciclo 5).
#[test]
fn encurtar_o_trem_reduz_a_folga_na_mesma_medida() {
    let mut cfg = config_teste();
    // fixa o diâmetro p/ isolar o efeito (senão a derivação re-escolhe D):
    // usar a mesma técnica do teste existente de folga (diameter_m fixo).
    let spec_alto = /* PropellerAgent::run com cfg intacta */;
    cfg.gear.h_cg_ground_m -= 0.10;
    let spec_baixo = /* PropellerAgent::run com trem 10 cm mais curto */;
    assert!(((spec_alto.ground_clearance_m - spec_baixo.ground_clearance_m) - 0.10).abs() < 1e-9,
        "folga deveria cair EXATAMENTE 0,10 m: {:.4} → {:.4}",
        spec_alto.ground_clearance_m, spec_baixo.ground_clearance_m);
}

// hand-check do datum novo na fixture: shaft = 1.03 + 0.12 = 1.15 (idêntico
// ao antigo) — atualizar o hand-check existente de folga para a soma.
```

`config.rs`: teste de migração (`shaft_height_m = 1.20` presente → erro citando `prop_axis_above_cg_m` e a derivação `h_cg_ground_m + offset`); rejection tests de `prop_axis_above_cg_m` fora de (−0,3, 0,8) — ex.: `1.0` e `-0.5`.

- [ ] **Step 2: Confirmar RED.**

- [ ] **Step 3: Implementar.**
1. `PropellerCfg`: remover `shaft_height_m`; adicionar:

```rust
/// Offset vertical FIXO da célula entre o eixo da hélice e o CG (m) —
/// ciclo 5. A altura do eixo ao solo DERIVA do trem:
/// shaft_height = gear.h_cg_ground_m + prop_axis_above_cg_m — encurtar o
/// trem consome folga de hélice 1:1 automaticamente (a campanha E9
/// encurtou h_cg 13 cm e o datum absoluto antigo manteve a folga parada).
/// Valor do baseline derivado da geometria atual (1,25 − 1,05) — validar
/// no CAD (Fase 3). Pode ser negativo (eixo abaixo do CG).
pub prop_axis_above_cg_m: f64,
```

2. Fixture: `prop_axis_above_cg_m: 0.12` (= 1,15 − 1,03 — folga sintética idêntica). String TOML de teste: substituir `shaft_height_m = 1.20` por `prop_axis_above_cg_m` = (1,20 − `h_cg_ground_m` daquela string — CALCULAR ao ler a string; o resultado deve reproduzir o shaft 1,20 exato).
3. Baseline TOML: `shaft_height_m` (valor 1,25) → `prop_axis_above_cg_m = 0.20` com o comentário do docstring (validar no CAD).
4. `config.rs`: `check_shaft_height_migration` ANTES do parse (padrão dos ciclos 3–4), mensagem citando o substituto e a derivação; faixa (−0,3, 0,8) no padrão `require_finite` + range (mensagem `"(−0.3, 0.8) (valor: {})"` — o campo pode ser negativo, NÃO usar require_positive); remover o `require_positive` antigo e o rejection test dele (vira o de faixa).
5. `propeller.rs`: `let shaft_height = cfg.gear.h_cg_ground_m + pcfg.prop_axis_above_cg_m;` substituindo os DOIS usos de `pcfg.shaft_height_m` (~120 e ~132); docstrings atualizadas.

- [ ] **Step 4: `cargo test` completo** — expectativa: ZERO pins mudam (folga idêntica por construção); se algo mudar, é transcrição errada — investigar.

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat(propeller): folga de hélice recoplada ao trem — prop_axis_above_cg_m substitui shaft_height_m (migração)"
```

---

### Task 2: Check #20 — atuador de retração vs orçamento elétrico

**Files:**
- Modify: `src/models/specs.rs` (`ElectricalLoadSpec` + campo em `ElectricalSpec`)
- Modify: `src/agents/electrical.rs` (ecoa as cargas)
- Modify: `src/validation/constraint_checker.rs` (check #20 + testes)
- Test: `src/validation/constraint_checker.rs` mod tests

**Interfaces:**
- Consumes: `GearSpec.actuator_power_w`; `cfg.electrical.loads`.
- Produces: `ElectricalSpec.loads: Vec<ElectricalLoadSpec>` com `ElectricalLoadSpec {name: String, continuous_w: f64, peak_w: f64}`; violação #20.

- [ ] **Step 1: Testes (RED)**

```rust
// constraint_checker.rs (mod tests):
/// #20: peak_w declarado da carga 'trem_retratil' menor que a potência
/// COMPUTADA do atuador → violação nomeando os dois valores.
#[test]
fn check_20_reprova_peak_w_declarado_menor_que_atuador_computado() {
    // fixture-base; mutar a carga 'trem_retratil' do ElectricalSpec (ou da
    // cfg antes do ElectricalAgent) para peak_w = 1.0 W; verify → violação
    // contendo "trem_retratil", "1.0" e o actuator_power_w do gear.
}

/// #20: aeronave de trem retrátil SEM carga 'trem_retratil' declarada →
/// violação (cobertura que morreu no ciclo 3 volta, agora pós-convergência).
#[test]
fn check_20_reprova_carga_trem_retratil_ausente() {
    // remover a carga da lista; verify → violação citando a ausência.
}

/// Caminho PASS: fixture intacta (peak_w sintético ≥ atuador computado).
#[test]
fn check_20_passa_na_fixture_intacta() { /* nenhuma violação com "atuador" */ }
```

- [ ] **Step 2: Confirmar RED.**

- [ ] **Step 3: Implementar.**
1. `specs.rs`:

```rust
/// Eco de uma carga elétrica configurada (ciclo 5, check #20) — espelho de
/// `ElectricalLoadCfg` no relatório, para rastreabilidade e para o checker
/// comparar o pico DECLARADO do atuador com a potência COMPUTADA.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ElectricalLoadSpec {
    pub name: String,
    pub continuous_w: f64,
    pub peak_w: f64,
}
```

+ `pub loads: Vec<ElectricalLoadSpec>` em `ElectricalSpec` (docstring: ciclo 5). `electrical.rs`: preencher no `run` (map direto de `cfg.electrical.loads`).
2. Checker, após o #19 (comentário numera #20 e cita a spec do ciclo 5 + a guarda removida no ciclo 3):

```rust
// #20 — atuador de retração vs orçamento elétrico (ciclo 5): o pico
// DECLARADO da carga 'trem_retratil' deve cobrir a potência COMPUTADA do
// atuador (LandingGearAgent). Substitui a guarda de parse removida no
// ciclo 3 (a massa da perna virou computada — a checagem só é possível
// PÓS-convergência, aqui).
match electrical.loads.iter().find(|l| l.name == "trem_retratil") {
    None => violations.push(
        "Carga 'trem_retratil' ausente do orçamento elétrico — aeronave de \
         trem retrátil precisa declarar o pico do atuador em [[electrical.loads]]"
            .to_string(),
    ),
    Some(l) if l.peak_w < gear.actuator_power_w => violations.push(format!(
        "Atuador de retração: pico declarado em [[electrical.loads]] \
         'trem_retratil' ({:.1} W) menor que a potência computada do atuador \
         ({:.1} W) — orçamento elétrico subdimensionado",
        l.peak_w, gear.actuator_power_w
    )),
    Some(_) => {}
}
```

- [ ] **Step 4: `cargo test` completo** — baseline: 520 W declarados ≥ 31,1 W computados → sem violação nova; pins intactos.

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat(validation): check #20 — pico declarado do atuador vs potência computada (fecha cobertura do ciclo 3)"
```

---

### Task 3: Caso "massa-total" no check #19 (re-sizing completo)

**Files:**
- Modify: `src/validation/robustness.rs` (3º caso + assinatura)
- Modify: `src/models/specs.rs` (`RobustnessSpec.mtow_masstotal_kg`)
- Modify: `src/main.rs`, fixture do `constraint_checker.rs`, `tests/schema_v4.rs`, `tests/gear_tipback.rs` (call sites da assinatura nova)
- Test: `src/validation/robustness.rs` mod tests

**Interfaces:**
- Consumes: `orchestrator::size_aircraft`, `PerformanceAgent::run(state, wing, prop, mtow, engine, req, &cfg.performance)`, `MissionSpec.fuel_total_l`, fórmulas dos checks #18/#2.
- Produces: `RobustnessAgent::run(cfg, engine, req, state, wing, emp, masses, wb_nominal, gear_nominal, mission_nominal: &MissionSpec, perf_nominal: &PerformanceSpec) -> RobustnessSpec` (2 params novos no FIM); `RobustnessSpec.mtow_masstotal_kg: f64`; flips com `caso: "massa-total"`.

- [ ] **Step 1: Testes (RED)**

```rust
// robustness.rs (mod tests):
/// Sizing quebra no mundo +σ → flip único "Dimensionamento(massa-total)".
/// Construção: fixture com tanque apertado — reduzir fuel_system.capacity_l
/// até o nominal convergir com margem pequena e o +σ (MTOW maior → mais
/// combustível de missão) estourar a capacidade.
#[test]
fn sizing_inviavel_no_mundo_mais_sigma_gera_flip_de_dimensionamento() { /* ... */ }

/// Margem de combustível marginal no nominal flipa com caso "massa-total":
/// fixture com min_fuel_margin_fraction apertado logo abaixo da margem
/// nominal; +σ (MTOW ↑ → fuel_required ↑ → margem ↓) cruza o piso.
#[test]
fn margem_de_combustivel_marginal_flipa_no_caso_massa_total() { /* ... */ }

/// σ mínimo (0.05 na fixture clonada) com margens folgadas: nenhum flip
/// massa-total; mtow_masstotal_kg > MTOW nominal (perturbação para CIMA).
#[test]
fn caso_massa_total_bem_formado_sem_flips_na_fixture_folgada() { /* ... */ }
```

- [ ] **Step 2: Confirmar RED.**

- [ ] **Step 3: Implementar** — em `RobustnessAgent::run`, após os 2 casos de CG:

```rust
// ── Caso 3: MASSA-TOTAL (ciclo 5) — todas as massas estruturais +σ via
// re-sizing COMPLETO: clona o config multiplicando os 5 fatores de
// composto por (1+σ) e re-converge o laço inteiro. What-if físico em
// memória — deliberadamente NÃO re-passa pelas faixas de parse (o produto
// pode exceder a faixa de config; a faixa protege dados de entrada, não
// experimentos adversariais). Autonomia não é reavaliada aqui: o
// MissionAgent a garante por construção ou o sizing falha
// (CombustivelInsuficiente) — coberto pelo flip de Dimensionamento.
let sigma = cfg.mass_model.sigma_mass_fraction;
let mut cfg_p = cfg.clone();
cfg_p.mass_model.composite_factor_wing *= 1.0 + sigma;
cfg_p.mass_model.composite_factor_tail *= 1.0 + sigma;
cfg_p.mass_model.composite_factor_fuselage *= 1.0 + sigma;
cfg_p.mass_model.composite_factor_gear *= 1.0 + sigma;
cfg_p.mass_model.composite_factor_fuel_system *= 1.0 + sigma;

let mtow_masstotal_kg;
match crate::orchestrator::size_aircraft(&cfg_p, engine, req) {
    Err(e) => {
        mtow_masstotal_kg = 0.0; // sem ponto convergido; flip documenta
        flips.push(RobustnessFlip {
            check: "Dimensionamento".to_string(),
            caso: "massa-total".to_string(),
            valor: match &e {
                SizingError::CombustivelInsuficiente { necessario_l, .. } => *necessario_l,
                SizingError::MtowExcedido { mtow, .. } => *mtow,
                _ => f64::NAN,
            },
            limite: match &e {
                SizingError::CombustivelInsuficiente { capacidade_l, .. } => *capacidade_l,
                SizingError::MtowExcedido { limite, .. } => *limite,
                _ => f64::NAN,
            },
        });
    }
    Ok(sized_p) => {
        mtow_masstotal_kg = sized_p.state.mtow_kg;
        let cap = cfg.fuel_system.capacity_l;
        // margem de combustível (fórmula do check #18):
        let margem_p = (cap - sized_p.mission.fuel_total_l) / cap;
        let margem_nom = (cap - mission_nominal.fuel_total_l) / cap;
        if margem_nom >= req.min_fuel_margin_fraction
            && margem_p < req.min_fuel_margin_fraction {
            flips.push(RobustnessFlip { check: "Margem de combustível".into(),
                caso: "massa-total".into(), valor: margem_p * 100.0,
                limite: req.min_fuel_margin_fraction * 100.0 });
        }
        // VS0 (fórmula do check #2):
        let vs0_lim = req.cruise_speed_min_kmh / 1.8;
        if wing.stall_speed_flaps_kmh <= vs0_lim
            && sized_p.wing.stall_speed_flaps_kmh > vs0_lim {
            flips.push(RobustnessFlip { check: "VS0".into(),
                caso: "massa-total".into(),
                valor: sized_p.wing.stall_speed_flaps_kmh, limite: vs0_lim });
        }
        // desempenho no mundo +σ (mesmos gates do pipeline nominal):
        let perf_p = crate::agents::performance::PerformanceAgent::run(
            &sized_p.state, &sized_p.wing, &sized_p.prop,
            sized_p.state.mtow_kg, engine, req, &cfg.performance);
        for (nome, nom, p, lim, maior_melhor) in [
            ("Razão de subida", perf_nominal.rc_sl_ms, perf_p.rc_sl_ms, 1.5, true),
            ("Velocidade de cruzeiro", perf_nominal.v_cruise_kmh, perf_p.v_cruise_kmh,
             req.cruise_speed_min_kmh, true),
            ("Teto de serviço", perf_nominal.service_ceiling_m,
             perf_p.service_ceiling_m, 3_000.0, true),
        ] {
            let nom_ok = if maior_melhor { nom >= lim } else { nom <= lim };
            let p_ok = if maior_melhor { p >= lim } else { p <= lim };
            if nom_ok && !p_ok {
                flips.push(RobustnessFlip { check: nome.into(),
                    caso: "massa-total".into(), valor: p, limite: lim });
            }
        }
    }
}
```

`RobustnessSpec` ganha `pub mtow_masstotal_kg: f64` (docstring: MTOW re-convergido do caso massa-total; 0.0 quando o sizing falhou — o flip de Dimensionamento acompanha). Assinatura: 2 params novos no fim; atualizar os 5 call sites (main.rs, fixture do checker, schema_v4, gear_tipback ×2) — main tem `mission`/`perf` locais; a fixture do checker já constrói ambos.

- [ ] **Step 4: `cargo test` completo** — baseline: sem flip massa-total esperado (margem 14,3→~11% > 5%); registrar o `mtow_masstotal_kg` real no report; pins de schema ainda 4.6 (bump é Task 4).

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat(robustness): caso massa-total no check #19 — re-sizing completo com fatores ×(1+sigma)"
```

---

### Task 4: Schema 4.7 + pin de flutter + baseline regenerado

**Files:**
- Modify: `src/models/specs.rs` (`SCHEMA_VERSION = "4.7"` + histórico)
- Modify: `src/main.rs` (fidelity `robustness` menciona o caso massa-total)
- Modify: `docs/aircraft_spec.schema.md` (histórico 4.7: `electrical.loads`, `robustness.mtow_masstotal_kg`, caso "massa-total", troca do datum da folga)
- Modify: `tests/schema_v4.rs` (4.7 + campos novos), `tests/acceptance.rs` (pins 4.6→4.7), `tests/cli.rs` (pin de flutter)
- Modify: `aircraft_spec.json` (regenerado)

**Interfaces:**
- Consumes: Tasks 1–3.

- [ ] **Step 1: Testes (RED):** `tests/schema_v4.rs` assere `schema_version == "4.7"`, `electrical.loads` (array não-vazio com `name`/`peak_w`), `robustness.mtow_masstotal_kg` numérico > 0. `tests/cli.rs`: pin novo de flutter:

```rust
// Pin honesto de flutter (revisão final do ciclo 4: caiu 6,3% no ciclo 3
// — 749,55 → 702,60 km/h com a asa computada mais pesada — sem nenhum pin).
// Piso regulatório 1,2×VD = 420 km/h fica LONGE; o pin pega regressão de
// modelo, não proximidade de limite.
let flutter = json["structure"]["flutter_speed_kmh"].as_f64().unwrap();
assert!((flutter - 702.6).abs() < 7.0, // ±1%, padrão dos pins de performance
    "flutter_speed_kmh = {flutter:.1} divergiu do pin honesto ≈702,6 (±1%)");
```

- [ ] **Step 2: Confirmar RED** (schema ainda 4.6).

- [ ] **Step 3: Implementar:** bump + histórico em specs.rs; fidelity `robustness` → acrescentar "; caso massa-total: re-sizing completo com fatores ×(1+σ)"; schema doc (4 itens do histórico); regenerar `aircraft_spec.json` com o comando padrão (`cargo run --release -- --engine config/engines/toyota_1gd_ftv.toml --aircraft config/aircraft/baseline_4seat.toml --mission config/missions/default.toml --out aircraft_spec.json`).

- [ ] **Step 4: `cargo test` completo** — baseline esperado: FAIL, mesmas 3 violações nominais, 0 flips (CG e massa-total), folga 0,275 idêntica, #20 sem violação; investigar QUALQUER desvio.

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat(schema): v4.7 — electrical.loads, caso massa-total e pin de flutter; aircraft_spec.json regenerado"
```

---

### Task 5: Validação E9 e relatório do achado (sem commit de código)

**Files:**
- Create: report da task (workspace SDD)

**Interfaces:**
- Consumes: binário final; célula E9 (mutações sobre o baseline ATUAL).

- [ ] **Step 1: Construir o TOML da célula E9** a partir do `config/aircraft/baseline_4seat.toml` atual (que já tem `prop_axis_above_cg_m`), em /tmp (NUNCA no repo): item `bateria_recolocada` 28,0→53,0 kg + `arm_offset_m = 0.4`; `x_nose_m` 1,40→1,30; `h_cg_ground_m` 1,05→0,92; `main_strut_length_m` 0,67→0,54; `nose_strut_length_m` 0,53→0,40.

- [ ] **Step 2: Rodar** com `--out /tmp/e9_ciclo5.json` e registrar: `validation_status`, TODAS as violações (esperado: folga de hélice `ground_clearance_m` = 0,92+0,20−1,95/2 = **0,145 < 0,23** → violação; conferir também #19 CG/massa-total e #20), flips, e os derivados físicos:
  - `diameter_max_by_clearance_m` na altura E9 (= 2×(1,12−0,23) = 1,78 m);
  - `h_cg` mínimo para manter D=1,95 (= 0,23+0,975−0,20 = 1,005 m).

- [ ] **Step 3: Relatório** no workspace: veredito E9 honesto + tabela das alternativas físicas (hélice ≤1,78 m: consultar `tip_mach_*` do run E9 para o custo de Mach; `h_cg` ≥1,005 m: fora da região robusta de tipback encontrada na E9f — citar os números da campanha), SEM recomendação unilateral de adoção — a decisão é humana, com o quadro completo.

- [ ] **Step 4: `cargo test` completo** (nada mudou — confirmação final) e encerrar o ciclo.
