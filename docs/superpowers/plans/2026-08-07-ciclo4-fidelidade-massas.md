# Ciclo 4 — Fidelidade de Massas (t/c Empenagem, W_dg Envelope, Check #19 Robustez) — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Três refinos de fidelidade do modelo de massas: t/c dedicado da empenagem nas equações Raymer, W_dg = MTOW de envelope (lag-1), e check #19 de robustez à incerteza (±σ pior-caso determinístico) — para que um PASS marginal reprove sozinho.

**Architecture:** Spec: `docs/superpowers/specs/2026-08-07-ciclo4-fidelidade-massas-design.md`. Campo novo `[empennage].thickness_ratio`; terceiro lag-1 no orchestrator (`mtow_envelope_prev`); módulo novo `src/validation/robustness.rs` (2 conjuntos adversariais de massas → re-roda WB e LandingGear → lista de checks que flipam) consumido por `ConstraintChecker::verify` como violação #19 e pelo JSON como bloco `robustness` (schema 4.5→4.6).

**Tech Stack:** Rust, serde/TOML, sem dependências novas.

## Global Constraints

- Pins honestos: old→new comentado, tolerâncias INALTERADAS; baseline esperado continua FAIL (3 violações; #19 pode adicionar — o que o modelo disser); surpresa >5% vs expectativa → investigar antes de pinar.
- Dados só em TOML: campo novo com faixa validada + rejection test + valor DISTINTO na fixture sintética.
- Determinismo total no #19 (sem RNG); perturbação avaliada só no ponto convergido.
- Português; referências citadas; `cargo test` verde ao fim de CADA task; aceitação de genericidade (grep) verde.
- Padrão lag-1: teste de convergência exercita campo REAL do `SizedAircraft` (nunca duplicar o corpo do laço); residual medido e pinado com folga 2× e comentário.

## Fatos do código atual (verificados em `7defd1b`)

- `MassModelAgent::run(cfg, engine, req, wing, emp, mtow_kg, n_design)` em `src/agents/mass_model.rs`; hoje `t_c = cfg.wing.thickness_ratio` com comentário de aproximação; docstring do módulo tem o bloco "aproximação t/c" da fix wave do ciclo 3.
- Orchestrator (`src/orchestrator.rs`): `let mut n_design_prev: f64 = 3.8;` (linha ~277); laço: `masses = MassModelAgent::run(..., mtow, n_design_prev)` (~301) com `n_design_iterations.push` (~302); `wb = WeightBalanceAgent::run(&state, &wing, engine, cfg, req, &emp, &masses)` (~342); `vn = VnDiagramAgent::run(...)` (~372) e `n_design_prev = vn.n_design` (~375).
- `ConstraintChecker::verify(req, wing, prop, mtow_kg, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg, fuel_capacity_l) -> ConstraintReport` (`src/validation/constraint_checker.rs:35-49`); call site `src/main.rs:561`.
- `LandingGearAgent::run(mtow_kg, x_cg_fwd, x_cg_aft, &cfg.gear, mass_main_total, mass_nose) -> GearSpec` (tipback/nose loads dentro do `GearSpec`).
- `weight_balance::oew_items(cfg, engine, masses) -> Vec<MassItem>` e `cg_from_items(&[MassItem]) -> (massa, x_cg)` são `pub`; `WeightBalanceOutput` tem `spec` (WeightSpec), `oew_kg`, `mac_m`, `scenarios: Vec<ScenarioResult>` (com `cg_pct_mac`, `total_mass_kg`); limites em `wb.spec.cg_limit_fwd_pct_mac`/`cg_limit_aft_pct_mac`.
- Braços dos 7 itens: asa→`wing_struct_m`, fuselagem→`fuselage_struct_m`, emp_h→`empenagem_cg_m`, emp_v→`empenagem_cg_m − 0.2`, trem_principal→`gear_main_m` (= `gear.x_main_m`), trem_nariz→`gear_nose_m` (= `gear.x_nose_m`), tanques→`fuel_cg_m` (via `ArmConfig`).
- `SCHEMA_VERSION = "4.5"` em `src/models/specs.rs:~797`; `AircraftReport` (specs.rs ~904) com blocos `Option<...>`; fidelity map em `src/main.rs` ~653.
- Checks hoje: #15 tipback (`gear.tipback_angle_deg < gear_cfg.tipback_min_deg`), #16 nariz máx (`NOSE_LOAD_MAX_CEILING_PCT` = 25.0), #17 nariz mín (`NOSE_LOAD_MIN_FLOOR_PCT` = 8.0), envelope por cenário via `sc.inside_envelope` + "Envelope de CG VAZIO".

## Pins congelados (entradas E7 do ciclo 3; só muda t/c → 0.10 nas empenagens)

Entradas: `n_z_ult` 6.286149, `w_dg_kg` 1548.4, `q_pa` 3366.1331, `s_ht_m2` 3.133966, `s_vt_m2` 1.412900, `ar_h` 4.0, `ar_v` 1.5, `taper_h` 0.5, `taper_v` 0.5.

| Componente (t/c = 0.10) | raw | ×0.83 |
|---|---:|---:|
| EH | 17.596 | 14.605 |
| EV | 9.270 | 7.694 |

(Com t/c 0.15 os pins do ciclo 3 continuam válidos — as funções puras recebem t/c como parâmetro e seus testes não mudam.)

---

### Task 1: t/c dedicado da empenagem (`[empennage].thickness_ratio`)

**Files:**
- Modify: `src/models/aircraft_config.rs` (campo em `EmpennageCfg` + fixture)
- Modify: `src/models/config.rs` (faixa + rejection test + string TOML de teste)
- Modify: `config/aircraft/baseline_4seat.toml` (campo com comentário)
- Modify: `src/agents/mass_model.rs` (agente usa o t/c da empenagem; docstring vira nota histórica; hand-checks novos)
- Modify: pins quebrados em `src/` e `tests/` (golden update honesto — as massas de EH/EV do baseline sobem)

**Interfaces:**
- Produces: `cfg.empennage.thickness_ratio: f64` — consumido por `MassModelAgent::run`.

- [ ] **Step 1: Testes (RED)** — hand-checks com t/c de empenagem em `mass_model.rs` (mod tests) e rejection test em `config.rs`:

```rust
// mass_model.rs — pins congelados do plano (t/c 0.10, demais entradas E7):
#[test]
fn hand_check_empenagens_com_t_c_dedicado_0_10() {
    let eh = htail_mass_raymer_kg(N_Z_ULT, W_DG_KG, Q_PA, 3.133966, 0.10, 4.0, 0.5);
    assert!((eh - 17.596).abs() < 0.1, "EH raw t/c=0.10 = {eh:.3} (esperado 17.596 ±0.1)");
    assert!((eh * 0.83 - 14.605).abs() < 0.1);
    let ev = vtail_mass_raymer_kg(N_Z_ULT, W_DG_KG, Q_PA, 1.412900, 0.10, 1.5, 0.5);
    assert!((ev - 9.270).abs() < 0.1, "EV raw t/c=0.10 = {ev:.3} (esperado 9.270 ±0.1)");
    assert!((ev * 0.83 - 7.694).abs() < 0.1);
}

// property: empenagem mais FINA é mais PESADA (expoentes negativos de t/c)
#[test]
fn empenagem_mais_fina_e_mais_pesada() {
    let ev_grosso = vtail_mass_raymer_kg(N_Z_ULT, W_DG_KG, Q_PA, 1.4129, 0.15, 1.5, 0.5);
    let ev_fino   = vtail_mass_raymer_kg(N_Z_ULT, W_DG_KG, Q_PA, 1.4129, 0.10, 1.5, 0.5);
    assert!(ev_fino > ev_grosso, "EV t/c=0.10 ({ev_fino:.2}) deveria pesar mais que t/c=0.15 ({ev_grosso:.2})");
}

// agente usa cfg.empennage.thickness_ratio (relacional — ver teste existente
// `agente_aplica_fatores_de_composto_sobre_as_funcoes_puras`, que reconstrói
// as entradas derivadas: atualizar sua expectativa de emp_h/emp_v para usar
// cfg.empennage.thickness_ratio no lugar de cfg.wing.thickness_ratio)
```

Rejection test em `config.rs` (padrão dos existentes; âncora da string TOML de teste: seção `[empennage]`): `empennage.thickness_ratio = 0.30` → erro contendo `"empennage.thickness_ratio"` e a faixa `(0.06, 0.18)`.

- [ ] **Step 2: Confirmar RED** (`cargo test --lib mass_model config` — campo não existe).

- [ ] **Step 3: Implementar.**

`aircraft_config.rs` — em `EmpennageCfg`:

```rust
/// Espessura relativa (t/c) dos perfis da empenagem (ciclo 4) — perfis
/// simétricos finos típicos de empenagem (NACA 0009–0012). Consumido por
/// `agents::mass_model` (expoentes de (100·t/c): EH −0.12, EV −0.49 —
/// empenagem mais fina é mais PESADA). Antes do ciclo 4 usava-se o t/c da
/// ASA como aproximação (subestimava EV ~21%).
pub thickness_ratio: f64,
```

Fixture `config_teste()`: `thickness_ratio: 0.12` (DISTINTO do baseline 0.10). String TOML de teste em `config.rs`: `thickness_ratio = 0.12` na seção `[empennage]`. Validação em `config.rs` (padrão `require_finite` + faixa): erro `"configuração de aeronave inválida: empennage.thickness_ratio deve estar em (0.06, 0.18) — valor: {v}"`.

`config/aircraft/baseline_4seat.toml`, seção `[empennage]`:

```toml
# Espessura relativa dos perfis da empenagem (ciclo 4) — NACA 0009–0012
# típicos; alimenta as equações de massa Raymer (EH^-0.12, EV^-0.49).
thickness_ratio = 0.10
```

`mass_model.rs` — no `MassModelAgent::run`, substituir o `t_c` único:

```rust
let t_c_asa = cfg.wing.thickness_ratio;
let t_c_emp = cfg.empennage.thickness_ratio; // ciclo 4: campo dedicado
```

asa usa `t_c_asa`; `htail_mass_raymer_kg`/`vtail_mass_raymer_kg` usam `t_c_emp`. Docstring do módulo: o bloco "Aproximação t/c" da fix wave vira nota histórica ("resolvido no ciclo 4: `[empennage].thickness_ratio`"), mantendo a derivação dos ~21%/~5% como registro do viés que existia.

- [ ] **Step 4: Rodar o baseline real e atualizar pins (golden update honesto).**

Run: `cargo run --release -- --engine config/engines/toyota_1gd_ftv.toml --aircraft config/aircraft/baseline_4seat.toml --mission config/missions/default.toml --out /tmp/c4_t1.json`
Expectativa (verificar, não forçar): EH ~13.4→~14.1 kg, EV ~6.1→~7.5 kg (cauda +~2,3 kg), OEW +~2,3 kg, CG recua ~0,2–0,3 pp MAC, as 3 violações continuam (margens ligeiramente menores). Atualizar TODOS os pins quebrados old→new comentado, tolerâncias iguais; investigar surpresa >5%.

- [ ] **Step 5: `cargo test` completo + commit**

```bash
git add -A
git commit -m "feat(mass_model): t/c dedicado da empenagem ([empennage].thickness_ratio) nas equações Raymer"
```

---

### Task 2: W_dg = MTOW de envelope (lag-1 `mtow_envelope_prev`)

**Files:**
- Modify: `src/agents/mass_model.rs` (parâmetro `mtow_kg` → `w_dg_envelope_kg`, docstring)
- Modify: `src/orchestrator.rs` (lag novo + campo diagnóstico `mtow_envelope_iterations`)
- Modify: pins quebrados (golden update honesto)

**Interfaces:**
- Consumes: `wb.spec.mtow_kg` (MTOW de envelope da iteração).
- Produces: `SizedAircraft.mtow_envelope_iterations: Vec<f64>` (primeira entrada = seed `cfg.sizing.mtow_initial_guess_kg`); `MassModelAgent::run(cfg, engine, req, wing, emp, w_dg_envelope_kg, n_design)`.

- [ ] **Step 1: Testes (RED)** — em `orchestrator.rs` (mod tests):

```rust
/// Ciclo 4: W_dg do modelo de massas é o MTOW de ENVELOPE com lag-1.
/// Testa o campo REAL: as massas do SizedAircraft devem ser EXATAMENTE
/// as que MassModelAgent::run produz com o penúltimo envelope e o
/// penúltimo n_design do histórico (os valores lag-1 da iteração final).
#[test]
fn massas_do_sized_vem_do_envelope_lag_1() {
    let cfg = config_teste();
    let engine = engine_teste();
    let req = requisitos_teste();
    let sized = size_aircraft(&cfg, &engine, &req).expect("fixture converge");

    let env = &sized.mtow_envelope_iterations;
    let nd = &sized.n_design_iterations;
    assert!(env.len() >= 2, "histórico do envelope: {env:?}");
    // seed na primeira entrada (mesmo padrão de n_design_iterations):
    assert!((env[0] - cfg.sizing.mtow_initial_guess_kg).abs() < 1e-12,
        "seed do lag deveria ser mtow_initial_guess_kg, obtido {}", env[0]);

    let w_dg_lag = env[env.len() - 2];
    let n_design_lag = nd[nd.len() - 2];
    let esperado = MassModelAgent::run(&cfg, &engine, &req, &sized.wing,
                                       &sized.emp, w_dg_lag, n_design_lag);
    assert!((sized.structural_masses.asa_kg - esperado.asa_kg).abs() < 1e-9);
    assert!((sized.structural_masses.trem_principal_kg - esperado.trem_principal_kg).abs() < 1e-9);

    // convergência: delta final do envelope pequeno e pinado honesto
    let d = (env[env.len() - 1] - env[env.len() - 2]).abs();
    // PIN HONESTO: rodar, imprimir o histórico, medir e pinar com folga 2×
    // (padrão n_design_iterations_do_campo_real_converge). Placeholder até medir:
    assert!(d < 5.0, "residual do lag de envelope = {d:.4} kg — MEDIR E PINAR");
}
```

(O histórico `n_design_iterations` tem uma entrada por iteração com push ANTES do consumo — `env` deve seguir o MESMO protocolo de push para os índices casarem; ver Step 3.)

- [ ] **Step 2: Confirmar RED** (campo/parâmetro não existem).

- [ ] **Step 3: Implementar.**

`mass_model.rs`: renomear o parâmetro para `w_dg_envelope_kg` (aridade igual); docstring: "W_dg/W_l de Raymer são o peso máximo de projeto — o MTOW de ENVELOPE (`wb.spec.mtow_kg`), com LAG-1 no laço (seed = `sizing.mtow_initial_guess_kg`); o ciclo 3 usava o candidato de MISSÃO (inconsistente com `StructuralAgent`/`LandingGearAgent`, ~−3,5 kg dianteiros)". `assert!` de positividade mantido.

`orchestrator.rs`, espelhando o lag de `n_design`:
1. Junto de `n_design_prev` (~277): `let mut mtow_envelope_prev: f64 = cfg.sizing.mtow_initial_guess_kg;` + `let mut mtow_envelope_iterations: Vec<f64> = Vec::new();` (comentário: seed simples — o envelope estabiliza em poucas iterações; terceiro uso do padrão lag-1).
2. Na chamada (~301): `MassModelAgent::run(cfg, engine, req, &wing, &emp, mtow_envelope_prev, n_design_prev)` + `mtow_envelope_iterations.push(mtow_envelope_prev);` (mesma posição relativa do push de `n_design`).
3. Após `wb` rodar (~342, antes do V-n): `mtow_envelope_prev = wb.spec.mtow_kg;`.
4. `SizedAircraft` ganha `pub mtow_envelope_iterations: Vec<f64>` com docstring no padrão de `n_design_iterations`.

- [ ] **Step 4: Rodar baseline, medir/pinar o residual, golden update.**

Run: mesmo comando da Task 1 (`--out /tmp/c4_t2.json`).
Expectativa: massas +~1–2% (asa ~+1 kg, trem ~+2 kg), CG avança de volta ~0,1–0,3 pp, FAIL com 3 violações continua. Medir o residual do lag de envelope no teste (--nocapture), substituir o placeholder `< 5.0` pelo pin honesto (valor medido, folga 2×, comentário). Re-medir também o pin do residual de `n_design` se quebrar (dois lags agora interagem). Pins honestos em toda a suite.

- [ ] **Step 5: `cargo test` completo + commit**

```bash
git add -A
git commit -m "feat(mass_model): W_dg = MTOW de envelope com lag-1 (consistência com Structural/LandingGear)"
```

---

### Task 3: `RobustnessAgent` — conjuntos adversariais ±σ (módulo isolado, ainda sem check)

**Files:**
- Create: `src/validation/robustness.rs`
- Modify: `src/validation/mod.rs` (`pub mod robustness;`)
- Modify: `src/models/aircraft_config.rs` (`sigma_mass_fraction` em `MassModelCfg` + fixture 0.20)
- Modify: `src/models/config.rs` (faixa (0.05, 0.30) + rejection test + string TOML de teste)
- Modify: `config/aircraft/baseline_4seat.toml` (`sigma_mass_fraction = 0.15` comentado)
- Modify: `src/models/specs.rs` (structs `RobustnessSpec`/`RobustnessFlip`, serializáveis — SEM bump de versão ainda)

**Interfaces:**
- Consumes: `oew_items(cfg, engine, masses)`, `cg_from_items`, `WeightBalanceAgent::run(state, wing, engine, cfg, req, emp, masses)`, `LandingGearAgent::run(mtow, x_cg_fwd, x_cg_aft, gear_cfg, mass_main, mass_nose)`, limites nominais `wb.spec.cg_limit_{fwd,aft}_pct_mac`.
- Produces (Task 4 consome):

```rust
// specs.rs
/// Um check que PASSA no nominal mas REPROVA sob um conjunto adversarial
/// de massas estruturais (±σ) — ciclo 4, check #19.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobustnessFlip {
    /// Nome do check que flipou (ex.: "Cenário 'Solo (piloto)'", "Tipback",
    /// "Carga de nariz máx").
    pub check: String,
    /// Conjunto adversarial que o derrubou: "dianteiro" | "traseiro".
    pub caso: String,
    /// Valor sob perturbação e limite violado.
    pub valor: f64,
    pub limite: f64,
}

/// Análise de robustez à incerteza do modelo de massas (ciclo 4) —
/// pior-caso determinístico direcional, ver `validation::robustness`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RobustnessSpec {
    pub sigma_mass_fraction: f64,
    /// Faixa de CG dos cenários sob o conjunto CG-mais-DIANTEIRO (%MAC).
    pub cg_fwd_case_pct_mac: [f64; 2],
    /// Idem sob o conjunto CG-mais-TRASEIRO.
    pub cg_aft_case_pct_mac: [f64; 2],
    /// Checks que passam no nominal mas reprovam perturbados (vazio = robusto).
    pub flips: Vec<RobustnessFlip>,
}
```

```rust
// robustness.rs
pub struct RobustnessAgent;
impl RobustnessAgent {
    pub fn run(
        cfg: &AircraftConfig, engine: &EngineSpec, req: &Requirements,
        state: &AircraftState, wing: &WingSpec, emp: &EmpennageSpec,
        masses: &StructuralMasses, wb_nominal: &WeightBalanceOutput,
        gear_nominal: &GearSpec,
    ) -> RobustnessSpec
}
```

- [ ] **Step 1: Testes (RED)** — em `robustness.rs` (mod tests) + rejection em `config.rs`:

```rust
// σ = 0 (via clone da fixture com sigma_mass_fraction = 0.05 mínimo? NÃO —
// usar helper interno com σ explícito): perturbação nula ≈ nominal.
// Estrutura dos testes (fixture sintética completa, mesma sequência do
// orchestrator para obter wing/emp/masses/wb/gear nominais):

/// Classificação direcional: cada um dos 7 componentes entra no lado certo
/// comparando o braço (ArmConfig) com o CG VAZIO nominal — teste da função
/// auxiliar pública do módulo `adversarial_masses(cfg, engine, masses, sigma)
/// -> (StructuralMasses /*cg fwd*/, StructuralMasses /*cg aft*/)`.
#[test]
fn conjuntos_adversariais_perturbam_na_direcao_certa() {
    // (fixture) x_cg_oew nominal via cg_from_items(oew_items(...));
    // para o conjunto CG-DIANTEIRO: todo componente com braço < x_cg_oew
    // deve sair ×(1+σ) e todo componente com braço > x_cg_oew ×(1−σ);
    // asserar componente a componente contra o braço real do ArmConfig.
}

/// σ→0 degenera no nominal: flips vazio e faixas de CG iguais às nominais
/// (tolerância 1e-9) — construção, não coincidência.
#[test]
fn sigma_zero_nao_produz_flips() { /* RobustnessAgent com helper σ=0.0 */ }

/// Config sintética MARGINAL: partir da fixture e apertar UM limite para
/// deixar um check passando por pouco no nominal (ex.: subir
/// `gear.tipback_min_deg` até ~0,5° abaixo do tipback nominal) → com
/// σ=0.20 o conjunto CG-TRASEIRO derruba o tipback → flips contém
/// exatamente esse check, com caso "traseiro" e valor < limite.
#[test]
fn config_marginal_gera_flip_nomeado() { /* ... */ }

/// Fixture intacta com σ da fixture (0.20): saída bem-formada — faixas de
/// CG do caso dianteiro À FRENTE das nominais e do traseiro ATRÁS
/// (desigualdade estrita), flips só contém checks que passam no nominal.
#[test]
fn casos_adversariais_movem_o_cg_nas_duas_direcoes() { /* ... */ }
```

Rejection test: `sigma_mass_fraction = 0.5` → erro com `"mass_model.sigma_mass_fraction"` e faixa `(0.05, 0.30)`.

- [ ] **Step 2: Confirmar RED.**

- [ ] **Step 3: Implementar `robustness.rs`** (docstring do módulo cita a spec do ciclo 4 e a base ±10–20% de Raymer cap. 15/Roskam Classe II):

```rust
/// Constrói os 2 conjuntos adversariais. Determinístico: classifica cada
/// componente comparando seu braço (mesmo mapeamento de oew_items) com o
/// CG VAZIO nominal; empates (braço == CG, improvável) vão para o lado
/// dianteiro (documentado).
pub fn adversarial_masses(
    cfg: &AircraftConfig, engine: &EngineSpec,
    masses: &StructuralMasses, sigma: f64,
) -> (StructuralMasses, StructuralMasses) {
    let items = crate::agents::weight_balance::oew_items(cfg, engine, masses);
    let (_, x_cg_oew) = crate::agents::weight_balance::cg_from_items(&items);
    let arms = /* ArmConfig::from_config(cfg) — braços dos 7 componentes,
                  emp_v com o offset −0.2 (usar a MESMA constante pública
                  EMP_VERTICAL_ARM_OFFSET_M) */;
    let scale = |mass: f64, arm: f64, fwd_heavier: bool| -> f64 {
        let dianteiro = arm <= x_cg_oew;
        if dianteiro == fwd_heavier { mass * (1.0 + sigma) } else { mass * (1.0 - sigma) }
    };
    let fwd = StructuralMasses { asa_kg: scale(masses.asa_kg, arms.wing_struct_m, true), /* ...7 campos... */ };
    let aft = StructuralMasses { asa_kg: scale(masses.asa_kg, arms.wing_struct_m, false), /* ... */ };
    (fwd, aft)
}
```

`RobustnessAgent::run`:
1. `(m_fwd, m_aft) = adversarial_masses(cfg, engine, masses, cfg.mass_model.sigma_mass_fraction)`.
2. Para cada conjunto: `wb_p = WeightBalanceAgent::run(state, wing, engine, cfg, req, emp, &m_p)`; extremos de CG em x: `x = cfg.wing.le_root_x_m + pct/100 * wb_p.mac_m` sobre `wb_p.scenarios`; `gear_p = LandingGearAgent::run(wb_p.spec.mtow_kg, x_fwd_p, x_aft_p, &cfg.gear, m_p.trem_principal_kg, m_p.trem_nariz_kg)`.
3. Avaliar contra limites NOMINAIS (`wb_nominal.spec.cg_limit_*` — invariantes a massa, documentado; ver spec §3) e pisos/tetos dos checks #15–#17 (`gear_cfg.tipback_min_deg`, constantes de nariz — reexportar `NOSE_LOAD_MAX_CEILING_PCT`/`NOSE_LOAD_MIN_FLOOR_PCT` como `pub` de `constraint_checker` para fonte única):
   - cada cenário: `cg_pct_mac` dentro de `[lim_fwd, lim_aft]` nominal;
   - `gear_p.tipback_angle_deg >= tipback_min_deg`;
   - `gear_p.nose_load_max_pct <= 25.0`; `gear_p.nose_load_min_pct >= 8.0`.
4. Flip = reprova sob o conjunto E passa no nominal (nominal: `sc.inside_envelope` do `wb_nominal`, `gear_nominal.tipback_angle_deg`, `gear_nominal.nose_load_*`). Um flip por (check, caso), `valor`/`limite` preenchidos.
5. `RobustnessSpec` com σ, faixas `[min,max]` de `cg_pct_mac` por caso, flips.

Config: `sigma_mass_fraction` em `MassModelCfg` (doc: "±σ da estatística de frota das equações de peso — Raymer cap. 15/Roskam Classe II citam ±10–20% em projeto conceitual"), fixture `0.20`, baseline TOML `0.15`, faixa `(0.05, 0.30)` no padrão de validação existente.

- [ ] **Step 4: `cargo test` completo** (nenhum comportamento do pipeline muda — módulo ainda não é chamado por `main`/`verify`; zero pins quebrados esperados; warning de código morto não pode existir — os testes do módulo o consomem, e `RobustnessSpec` é serializável mas ainda fora do `AircraftReport`... se `dead_code` reclamar do agente, o teste já o usa; conferir saída limpa).

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat(robustness): conjuntos adversariais ±sigma e RobustnessAgent (módulo isolado)"
```

---

### Task 4: Check #19 + bloco `robustness` no JSON (schema 4.5 → 4.6)

**Files:**
- Modify: `src/validation/constraint_checker.rs` (`verify` ganha `robustness: &RobustnessSpec`; #19; fixture de teste)
- Modify: `src/main.rs` (chama `RobustnessAgent::run` após gear, passa ao `verify` e ao `AircraftReport`; fidelity; print)
- Modify: `src/models/specs.rs` (`AircraftReport.robustness: Option<RobustnessSpec>`, `SCHEMA_VERSION = "4.6"`)
- Modify: `docs/aircraft_spec.schema.md` (histórico 4.6 + bloco novo)
- Modify: `tests/schema_v4.rs`, `tests/cli.rs`, `tests/acceptance.rs` e demais pins (golden update honesto)

**Interfaces:**
- Consumes: `RobustnessAgent::run(...) -> RobustnessSpec` (Task 3); assinatura atual de `verify` (fatos do código).
- Produces: `ConstraintChecker::verify(..., robustness: &RobustnessSpec)` — violação #19 por flip: `"Robustez: {check} passa no nominal mas reprova com massas estruturais ±{σ:.0%} (pior caso {caso}): {valor:.2} vs {limite:.2}"`.

- [ ] **Step 1: Testes (RED):**

```rust
// constraint_checker.rs (mod tests): a fixture-base do checker passa a
// construir RobustnessSpec via RobustnessAgent::run na MESMA sequência do
// main; teste novo:
/// #19: um flip injetado (RobustnessSpec sintético com 1 flip) gera
/// exatamente uma violação começando com "Robustez:" citando check, caso,
/// valor e limite; RobustnessSpec com flips vazio não gera nenhuma.
#[test]
fn check_19_transforma_flips_em_violacoes_nomeadas() { /* ... */ }
```

`tests/schema_v4.rs`: `schema_version == "4.6"` + presença de `robustness.sigma_mass_fraction` e `robustness.flips` (array) no JSON do binário.

- [ ] **Step 2: Confirmar RED.**

- [ ] **Step 3: Implementar.**
1. `verify(..., robustness: &RobustnessSpec)` — bloco #19 no final (comentário numera o check e cita a spec do ciclo 4); um `violations.push` por flip.
2. `main.rs`: após o `LandingGearAgent` (~460): `let robustness = RobustnessAgent::run(&cfg, &engine, &req, &state_final, wing, &sized.emp, &sized.structural_masses, wb, &gear);` (usar as MESMAS referências já disponíveis no fluxo — conferir nomes locais reais); print curto `[ ROBUSTEZ ] σ=15%: N flips` (listar flips se houver); passar ao `verify` e `robustness: Some(robustness.clone())` no `AircraftReport`; `fidelity.insert("robustness", "computed (pior-caso determinístico ±σ direcional sobre as 7 massas estruturais; limites de envelope nominais — invariantes a massa)")`.
3. `specs.rs`: campo + `SCHEMA_VERSION = "4.6"` (docstring de versionamento ganha a entrada).
4. `docs/aircraft_spec.schema.md`: histórico 4.6, bloco `robustness` documentado (campos, semântica de flips vazio, reuso dos limites nominais).

- [ ] **Step 4: Rodar baseline + golden update honesto.**

Run: mesmo comando (`--out /tmp/c4_t4.json`).
Expectativa (verificar, não forçar): FAIL continua; as 3 violações nominais seguem; #19 pode adicionar flips de checks hoje folgados (tipback 19°+ provavelmente NÃO flipa com σ=15%; nariz mín/cenários intermediários — o que o modelo disser). `tests/cli.rs` atualiza a contagem/lista de violações honestamente (nomeando as novas, se houver). Pins de "4.5" → "4.6" old→new.

- [ ] **Step 5: `cargo test` completo + commit**

```bash
git add -A
git commit -m "feat(validation): check #19 robustez à incerteza + bloco robustness — schema 4.6"
```

---

### Task 5: Rodada final, validação E8 e relatório do achado

**Files:**
- Modify: `aircraft_spec.json` (regenerado/commitado)
- Create: report da task (workspace SDD) com tabela old→new e o veredito E8

**Interfaces:**
- Consumes: tudo acima; TOML da célula E8 recomendada (recriar a partir do baseline atual com: `x_main_m = 3.58`, `x_nose_m = 1.25`, item `bateria_recolocada` com `arm_offset_m = 0.3` — NÃO reutilizar os TOMLs do scratchpad da campanha, que não têm os campos novos do ciclo 4).

- [ ] **Step 1: Regenerar o JSON commitado**

Run: `cargo run --release -- --engine config/engines/toyota_1gd_ftv.toml --aircraft config/aircraft/baseline_4seat.toml --mission config/missions/default.toml --out aircraft_spec.json`

- [ ] **Step 2: Tabela old→new no report** — baseline pós-ciclo-3 (`f209886`) vs pós-ciclo-4: as 7 massas, OEW, MTOW missão/envelope, CG por cenário, violações nominais + flips do #19, schema 4.5→4.6.

- [ ] **Step 3: Validação E8** — construir o TOML da célula E8 (mutações acima sobre o baseline ATUAL), rodar com `--out /tmp/e8_ciclo4.json`, registrar no report: status, violações nominais e flips #19 com números. Esperado da spec (§Testes item 5): #19 reprova a célula (margens ~0,2 pp < σ) — se os refinos §1–§2 mudarem o quadro (nominal já reprova, ou abre margem real), reportar o que for, sem mascarar.

- [ ] **Step 4: `cargo test` completo + commit**

```bash
git add aircraft_spec.json
git commit -m "feat(spec): aircraft_spec.json regenerado — ciclo 4 (t/c empenagem, W_dg envelope, robustez #19), schema 4.6"
```

O relatório do achado (baseline + veredito E8) encerra o ciclo; re-campanha E8 é decisão humana posterior.
