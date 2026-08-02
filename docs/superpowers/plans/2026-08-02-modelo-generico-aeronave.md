# Plano de Incremento — Modelo Genérico de Aeronave

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Transformar o modelo atual (hardcoded para o Toyota 1GD-FTV) em um modelo paramétrico genérico, dirigido por arquivos de configuração, que calcule **todos** os parâmetros de projeto de uma aeronave leve — prontos para consumo por ferramentas de CAD paramétrico.

**Architecture:** Separação estrita entre **dados** (TOML: motor, aeronave, missão) e **física** (Rust: equações genéricas). Um loop de dimensionamento (sizing loop) itera MTOW/área de asa até convergência. A saída é um JSON versionado com a geometria completa da aeronave.

**Tech Stack:** Rust 2021, `serde`/`serde_json` (já presentes), `toml` (novo), `clap` (novo, Fase 2).

## Global Constraints

- Unidades SI internamente (m, kg, N, W, Pa, m/s); conversões só na apresentação.
- Nenhum nome de motor, valor de torque, BSFC ou massa de motor pode aparecer em código `.rs` — só em arquivos `config/*.toml`.
- Nenhuma verificação de requisito pode ser vácua (deve poder falhar com entradas ruins — cada teste de verificação precisa de um caso negativo).
- Todos os testes existentes que continuarem válidos devem seguir passando após cada task (`cargo test`).
- Commits pequenos e frequentes, um por task no mínimo.
- Textos de saída e comentários em português (padrão atual do projeto).

---

## Diagnóstico do modelo atual (v3.0)

### Bugs críticos (invalidam resultados ou verificações)

| # | Local | Problema |
|---|-------|----------|
| B1 | `src/agents/structural.rs:192` | `flutter_speed_ms` termina com `vf.max(1.20 * vd_ms * 1.15)` — o piso garante que `flutter_check` **sempre passa**. A verificação de flutter é vácua. |
| B2 | `src/agents/structural.rs:224-227` | `fatigue_life_cycles` invertida: quando `sigma_equiv >= SE_MPA` (tensão **acima** do limite de fadiga) retorna `INFINITY` (vida infinita). É exatamente o contrário da física. |
| B3 | `src/agents/performance.rs:250` | `v_cruise_kmh = range_km / endurance_h`, mas `range_km` foi definido como `cruise_speed_min × endurance` em `propulsion.rs:218` — a "velocidade calculada" é o próprio requisito de entrada (circular). O check `perf.v_cruise_kmh >= 280.0` em `main.rs:129` sempre passa. O modelo nunca resolve o equilíbrio tração = arrasto. |
| B4 | `src/agents/propulsion.rs:208` | `load_fraction = p_req_kw / POWER_KW_MAX` usa a potência máxima ao nível do mar como referência. O correto é a potência disponível **naquele rpm e altitude** — o BSFC calculado fica sistematicamente errado. |
| B5 | Arquitetura | Não existe loop de convergência. `AircraftState::initial()` fixa `mtow_kg = 1461`, a aerodinâmica usa esse valor, mas o `WeightBalanceAgent` calcula outro MTOW (máximo dos cenários) que **não realimenta** os demais agentes. O comentário em `aircraft_state.rs:2-3` promete um Orchestrator que não existe. |
| B6 | `src/agents/propulsion.rs` (todo o arquivo) | Motor Toyota 1GD-FTV hardcoded: `Engine1GdFtv`, `torque_1gd_ftv()`, `bsfc_gkwh()`, densidade do diesel, strings. Vazamentos do acoplamento: `performance.rs:19-21` importa `torque_1gd_ftv`/`Engine1GdFtv`; `constraint_checker.rs:92` hardcoda `204.0` hp; `weight_balance.rs:121` hardcoda `mass_kg: 195.0` do motor; `main.rs:21` imprime o nome do motor. **Este é o problema central apontado pelo usuário.** |

### Problemas moderados (modelagem imprecisa ou frágil)

| # | Local | Problema |
|---|-------|----------|
| M1 | `weight_balance.rs:169,202-208` | `neutral_point_m` usa um trait `TanEstimate` que retorna sempre `0.0` (código morto disfarçado); parâmetros da empenagem (`s_ratio=0.22`, `l_tail=4.8`, `eta_t`, `at_aw`) hardcoded em vez de derivados do dimensionamento da empenagem. |
| M2 | `aerodynamics.rs:20-21,124` | `stall_speed_ms` usa `CL_MAX_23015 = 1.72` (com flap) mas o comentário diz "configuração limpa"; `CL_MAX_CLEAN = 1.45` nunca é usado. V_stall limpa e com flap precisam ser parâmetros distintos (VS1 vs VS0 — afetam VA e distâncias). |
| M3 | `main.rs:102` | Posição do bordo de ataque da asa (`2.90`) duplicada e hardcoded fora do `ArmConfig`. |
| M4 | `structural.rs:262` | `VC` hardcoded como `280.0` em vez de vir de `Requirements`. Massa da asa `130.0` hardcoded em `main.rs:81` e `structural.rs:184`. |
| M5 | `performance.rs:45-48` | Tração estática por Rankine-Froude ideal, sem fator de correção real (~0,75); superestima a decolagem. Distância aérea de pouso fixa em `200.0` m; `mu_brk = 0.40/surface_factor` com `*sqrt(surface_factor)` são correções ad hoc não referenciadas. |
| M6 | `propulsion.rs:81-85` | Comentário diz perda de 3%/300 m; código usa 5%/300 m. `TURBO_ALTITUDE_FACTOR` declarado e nunca usado. |
| M7 | `aerodynamics.rs:29-31,97` | Atmosfera só modela densidade; velocidade do som hardcoded (`340.0`). Precisa de ISA completa (T, p, ρ, a) para Mach de ponta de hélice em qualquer altitude. |
| M8 | `main.rs:156-160` | Preços de combustível hardcoded no código. |
| M9 | `requirements.rs:33`, `weight_balance.rs:336` | Massa por passageiro (90 kg) e payload hardcoded em dois lugares distintos. |
| M10 | `landing_gear.rs:163,200` | Altura do CG (`1.05`) e peso total do trem hardcoded; o peso do trem deveria realimentar o orçamento de peso. |

### Parâmetros ausentes (necessários para desenhar e construir a aeronave)

1. **Empenagem** — áreas S_h e S_v, braços, cordas, alongamento (hoje só existem razões hardcoded dentro do cálculo de ponto neutro).
2. **Superfícies de controle** — dimensões de aileron, profundor, leme e flap.
3. **Fuselagem** — comprimento, largura/altura de cabine, estações (hoje só um comprimento `8.2` implícito).
4. **Diagrama V-n completo** — VA, VB, VC, VD, fatores negativos, linhas de rajada (CS 23.341) — hoje só n_lim/n_ult e VD.
5. **Envelope de CG permitido** — limites dianteiro/traseiro *admissíveis* (por autoridade de profundor e SM mínima), não apenas o CG *observado* nos cenários.
6. **Hélice** — diâmetro derivado (limite de Mach de ponta), número de pás, corda de pá (hoje diâmetro é entrada fixa).
7. **Desempenho completo** — Vx, Vy, melhor planeio, gradientes de subida (CS 23.65), velocidade máxima nivelada, alcance com vento.
8. **Análise de missão** — combustível de táxi/subida/descida/reserva por segmentos (hoje endurance = tanque/consumo, otimista).
9. **Cargas do trem** realimentadas na estrutura da fuselagem; **carga elétrica** total; **arrasto de refrigeração** do motor.

---

## Estrutura de arquivos alvo

```
config/
  engines/
    toyota_1gd_ftv.toml      # dados do motor atual (vira DADO, não código)
    rotax_915is.toml         # segundo motor — prova da genericidade
  aircraft/
    baseline_4seat.toml      # geometria, braços, materiais, trem, hélice
  missions/
    default.toml             # requisitos: pax, cruzeiro, autonomia, reservas
src/
  models/
    engine.rs                # EngineSpec genérico + interpolação (NOVO)
    atmosphere.rs            # ISA completa: T, p, ρ, a (NOVO)
    config.rs                # carregamento TOML de todos os inputs (NOVO)
    requirements.rs          # passa a ser desserializável de TOML
    aircraft_state.rs        # estado mutável do sizing loop (sem defaults mágicos)
    specs.rs                 # +EmpennageSpec, +ControlSurfaceSpec, +VnDiagram, +MissionSpec
  agents/                    # física genérica — consome EngineSpec/config
  orchestrator.rs            # sizing loop com convergência de MTOW (NOVO)
  validation/
main.rs                      # CLI: --engine --aircraft --mission --out
```

---

# FASE 0 — Correção dos bugs críticos de física

*Objetivo: nenhuma verificação vácua, nenhum resultado fisicamente invertido. Pode ser executada antes de qualquer refatoração.*

### Task 0.1: Corrigir a velocidade de flutter (B1)

**Files:**
- Modify: `src/agents/structural.rs:172-198`
- Test: `src/agents/structural.rs` (mod tests)

**Interfaces:**
- Produces: `flutter_speed_ms(vd_ms, wing_area_m2, span_m, chord_root_m, spar_height_m, wing_mass_kg) -> f64` — retorna a estimativa física **sem piso artificial**; novo parâmetro `wing_mass_kg` substitui o `130.0` interno.

- [ ] **Step 1: Escrever teste que falha** — um caso com longarina subdimensionada DEVE reprovar no flutter:

```rust
#[test]
fn flutter_reprova_com_longarina_fraca() {
    // Longarina de 20 mm de altura em asa de 12 m: rigidez torsional GJ ~ h⁴
    // despenca e V_flutter deve cair abaixo de 1.2×VD.
    let vd = vd_ms(vc_ms(280.0));
    let vf = flutter_speed_ms(vd, 14.2, 11.94, 1.64, 0.020, 130.0);
    assert!(!flutter_check(vf, vd),
        "Flutter check passou com longarina de 20 mm — verificação vácua");
}
```

- [ ] **Step 2: Rodar e confirmar falha** — `cargo test flutter_reprova` → FALHA (hoje o `.max()` garante aprovação).

- [ ] **Step 3: Corrigir a implementação** — remover o piso e usar a massa da asa recebida:

```rust
pub fn flutter_speed_ms(vd_ms: f64, wing_area_m2: f64, span_m: f64,
                        chord_root_m: f64, spar_height_m: f64,
                        wing_mass_kg: f64) -> f64 {
    let _ = vd_ms; // não participa da estimativa — apenas do critério
    let g_al = 27.6e9_f64;
    let j_eff = 0.02 * spar_height_m.powi(4);
    let gj = g_al * j_eff;
    let m_per_m = wing_mass_kg / span_m;
    let r_alpha = chord_root_m / 4.0;
    let i_alpha_per_m = m_per_m * r_alpha * r_alpha;
    0.60 * (gj / (i_alpha_per_m * wing_area_m2 / span_m)).sqrt()
}
```

- [ ] **Step 4: Rodar todos os testes** — `cargo test`. O teste `flutter_acima_de_1_2_vd` existente deve continuar passando (se a longarina real reprova, isso é um **resultado de engenharia legítimo** que deve ser reportado como violação, não mascarado — nesse caso ajustar a asserção do teste antigo para refletir o valor físico e registrar a pendência no relatório de validação).

- [ ] **Step 5: Commit** — `git commit -m "fix(structural): flutter check deixa de ser vácuo"`

### Task 0.2: Corrigir a direção da vida em fadiga (B2)

**Files:**
- Modify: `src/agents/structural.rs:209-228`
- Test: `src/agents/structural.rs` (mod tests)

- [ ] **Step 1: Teste que falha:**

```rust
#[test]
fn fadiga_alta_tensao_vida_curta() {
    // σ_a equivalente acima do limite de fadiga → vida FINITA e menor que 10⁷
    let vida_alta_tensao = fatigue_life_cycles(300.0, 50.0);
    // σ_a equivalente abaixo do limite → vida "infinita" (≥ 10⁹ por convenção)
    let vida_baixa_tensao = fatigue_life_cycles(80.0, 40.0);
    assert!(vida_alta_tensao < 1e7, "alta tensão deveria dar vida finita < 10⁷");
    assert!(vida_baixa_tensao >= 1e9, "baixa tensão deveria dar vida quase infinita");
}
```

- [ ] **Step 2: Rodar e confirmar falha** (hoje alta tensão devolve `INFINITY`).

- [ ] **Step 3: Corrigir** — inverter a condição:

```rust
    if sigma_equiv <= SE_MPA {
        return f64::INFINITY; // abaixo do limite de fadiga → vida infinita
    }
    N_BASE * (SE_MPA / sigma_equiv).powf(B) // < N_BASE quando σ > Se
```

- [ ] **Step 4: `cargo test`** — reavaliar `fadiga_acima_de_10000_voos` com o resultado agora correto; se a longarina real ficar abaixo de 10.000 ciclos, isso vira violação reportada (não ajustar o modelo para "passar").

- [ ] **Step 5: Commit** — `git commit -m "fix(structural): direcao da curva S-N corrigida"`

### Task 0.3: Corrigir a fração de carga do BSFC (B4)

**Files:**
- Modify: `src/agents/propulsion.rs:207-209`

- [ ] **Step 1: Teste que falha:**

```rust
#[test]
fn load_fraction_relativa_a_potencia_disponivel_no_rpm() {
    // A 2.400 rpm o 1GD-FTV entrega ~125 kW (500 Nm), não 150 kW.
    // Com P_req = 100 kW, a carga real é ~0.80, não 0.67.
    let p_avail = power_kw_altitude(2_400.0, 2_500.0) * PSRU_EFFICIENCY;
    let load = 100.0 / p_avail;
    assert!(load > 0.78, "fração de carga {load:.2} deveria referenciar P_disponível no rpm");
}
```

- [ ] **Step 2: Corrigir em `PropulsionAgent::run`:**

```rust
        let load_fraction = (p_req_kw / p_shaft_kw).min(1.0);
```

(`p_shaft_kw` já existe na função — linha 195 — e hoje está morto, prefixado com `_`.)

- [ ] **Step 3: Adicionar verificação de viabilidade** que hoje não existe — se a potência exigida excede a disponível no rpm de cruzeiro, o cruzeiro é inviável:

```rust
        assert!(p_req_kw <= p_shaft_kw * 1.0,
            "Cruzeiro inviável: P_req {p_req_kw:.0} kW > P_disp {p_shaft_kw:.0} kW");
```

(Na Fase 3 este assert vira violação do ConstraintChecker; por ora falha ruidosamente.)

- [ ] **Step 4: `cargo test`** — consumo e autonomia mudarão; atualizar as asserções numéricas dos testes de consumo/autonomia para os novos valores **após conferir que são fisicamente plausíveis** (BSFC deve subir um pouco, consumo idem).

- [ ] **Step 5: Commit** — `git commit -m "fix(propulsion): BSFC usa carga relativa a potencia disponivel no rpm"`

### Task 0.4: Calcular a velocidade máxima de cruzeiro real (B3)

**Files:**
- Modify: `src/agents/performance.rs` (nova função + uso em `run`)
- Modify: `src/main.rs:129` (check passa a usar o valor resolvido)

**Interfaces:**
- Produces: `pub fn max_level_speed_ms(mass_kg, altitude_m, wing, state, engine_rpm) -> f64` — resolve T(V) = D(V) por bissecção.

- [ ] **Step 1: Teste que falha:**

```rust
#[test]
fn velocidade_maxima_resolvida_do_equilibrio() {
    let (state, wing, _) = setup();
    let v_max = max_level_speed_ms(1_461.0, 2_500.0, &wing, &state, 3_400.0);
    let v_max_kmh = v_max * 3.6;
    // Deve ser um número resolvido (não o requisito ecoado) e > requisito
    assert!(v_max_kmh > 280.0 && v_max_kmh < 400.0,
        "V_max nivelada {v_max_kmh:.0} km/h implausível");
}
```

- [ ] **Step 2: Implementar por bissecção sobre o excesso de potência:**

```rust
/// Velocidade máxima em voo nivelado: maior V com P_disp(V) ≥ P_req(V).
/// Bissecção entre 1.2·Vs e 200 m/s (720 km/h) sobre f(V) = P_excesso(V).
pub fn max_level_speed_ms(mass_kg: f64, altitude_m: f64, wing: &WingSpec,
                          state: &AircraftState, engine_rpm: f64) -> f64 {
    let rho = isa_density(altitude_m);
    let v_stall = ((2.0 * mass_kg * G) / (rho * wing.area_m2 * wing.cl_max)).sqrt();
    let (mut lo, mut hi) = (1.2 * v_stall, 200.0);
    let pex = |v: f64| excess_power_kw(v, mass_kg, rho, wing, engine_rpm,
                                       state.psru_ratio, state.prop_diameter_m, altitude_m);
    if pex(hi) > 0.0 { return hi; }        // limitado pelo teto do modelo
    if pex(lo) <= 0.0 { return lo; }       // não sustenta nem o mínimo — inviável
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        if pex(mid) > 0.0 { lo = mid; } else { hi = mid; }
    }
    0.5 * (lo + hi)
}
```

- [ ] **Step 3: Usar em `PerformanceAgent::run`** — `v_cruise_kmh: max_level_speed_ms(mtow_kg, 2_500.0, wing, state, rpm_max_continuo) * 3.6` (rpm máximo contínuo = rated por enquanto; parametrizado na Fase 1). O requisito em `main.rs` passa a comparar um valor resolvido.

- [ ] **Step 4: `cargo test`** — todos passando; `velocidade_cruzeiro_acima_do_requisito` agora testa algo real.

- [ ] **Step 5: Commit** — `git commit -m "fix(performance): V_cruise resolvida por equilibrio tracao-arrasto"`

### Task 0.5: Separar V_stall limpa (VS1) e com flap (VS0)

**Files:**
- Modify: `src/agents/aerodynamics.rs`, `src/models/specs.rs` (campos `stall_speed_clean_kmh` e `stall_speed_flaps_kmh` em `WingSpec`)
- Modify: `src/agents/structural.rs` (VA usa VS1 limpa), `src/main.rs` (impressão)

- [ ] **Step 1: Teste:** `vs0 < vs1` e ambos nas faixas esperadas (VS0 ~98–105, VS1 ~107–115 km/h para o baseline).
- [ ] **Step 2: Implementar** — `WingSpec` ganha os dois campos; `cl_max` (flap) e `cl_max_clean` ambos preenchidos; `va_ms` passa a receber VS1.
- [ ] **Step 3: `cargo test` e commit** — `git commit -m "feat(aero): VS0 e VS1 distintas"`

---

# FASE 1 — Motor genérico dirigido por dados (núcleo do pedido)

*Objetivo: trocar de motor = trocar um arquivo TOML. Nenhuma recompilação com dados novos, nenhum código específico de motor.*

### Task 1.1: Tipo `EngineSpec` com curva de torque interpolada

**Files:**
- Create: `src/models/engine.rs`
- Modify: `src/models/mod.rs` (adicionar `pub mod engine;`)
- Test: `src/models/engine.rs` (mod tests)

**Interfaces:**
- Produces:
  - `pub struct EngineSpec { name, mass_kg, rpm_idle, rpm_rated, rpm_redline, rpm_max_continuous, torque_curve: Vec<[f64;2]>, bsfc: BsfcModel, induction: Induction, fuel: FuelSpec }`
  - `impl EngineSpec { pub fn torque_nm(&self, rpm: f64) -> f64; pub fn power_kw(&self, rpm: f64) -> f64; pub fn power_kw_max(&self) -> f64; }`

- [ ] **Step 1: Teste que falha (arquivo novo):**

```rust
#[cfg(test)]
mod tests {
    use super::*;

    fn engine_teste() -> EngineSpec {
        EngineSpec {
            name: "Motor Genérico Teste".into(),
            mass_kg: 100.0,
            rpm_idle: 700.0, rpm_rated: 3_400.0,
            rpm_redline: 3_800.0, rpm_max_continuous: 3_000.0,
            torque_curve: vec![[700.0, 200.0], [1_600.0, 500.0],
                               [2_800.0, 500.0], [3_400.0, 420.0], [3_800.0, 0.0]],
            bsfc: BsfcModel::default_diesel(),
            induction: Induction::Turbocharged { critical_altitude_m: 2_000.0,
                                                 power_loss_per_1000m: 0.10 },
            fuel: FuelSpec { name: "Diesel S-10".into(),
                             density_kg_per_l: 0.840, lhv_mj_per_kg: 42.5 },
        }
    }

    #[test]
    fn torque_interpola_linearmente() {
        let e = engine_teste();
        assert!((e.torque_nm(2_000.0) - 500.0).abs() < 1.0);   // banda plana
        assert!((e.torque_nm(1_150.0) - 350.0).abs() < 1.0);   // meio da rampa
        assert_eq!(e.torque_nm(500.0), 0.0);                    // abaixo do idle
        assert_eq!(e.torque_nm(4_000.0), 0.0);                  // acima do redline
    }

    #[test]
    fn potencia_de_torque_e_rpm() {
        let e = engine_teste();
        // P = T·2πN/60 → 420 Nm @ 3400 rpm ≈ 149.5 kW
        assert!((e.power_kw(3_400.0) - 149.5).abs() < 1.0);
    }
}
```

- [ ] **Step 2: Rodar e confirmar falha de compilação** (tipo não existe).

- [ ] **Step 3: Implementar:**

```rust
use serde::{Deserialize, Serialize};

/// Especificação genérica de motor — todos os dados vêm de config TOML.
/// A física (interpolação, P=Tω, correção de altitude) é genérica.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EngineSpec {
    pub name: String,
    pub mass_kg: f64,
    pub rpm_idle: f64,
    pub rpm_rated: f64,
    pub rpm_redline: f64,
    /// RPM máximo de uso contínuo (cruzeiro/subida prolongada)
    pub rpm_max_continuous: f64,
    /// Pontos (rpm, Nm) — interpolação linear entre pontos; 0 fora da faixa
    pub torque_curve: Vec<[f64; 2]>,
    pub bsfc: BsfcModel,
    pub induction: Induction,
    pub fuel: FuelSpec,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Induction {
    /// Aspirado: perda de potência com altitude por Gagg-Ferrar
    NaturallyAspirated,
    /// Turbo: potência plena até a altitude crítica, perda linear acima
    Turbocharged { critical_altitude_m: f64, power_loss_per_1000m: f64 },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FuelSpec {
    pub name: String,
    pub density_kg_per_l: f64,
    /// Poder calorífico inferior (MJ/kg) — para Breguet e validações de BSFC
    pub lhv_mj_per_kg: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BsfcModel {
    pub bsfc_min_gkwh: f64,
    pub rpm_optimal: f64,
    pub load_optimal: f64,
    /// Penalidade: g/kWh por (Δrpm/1000)²
    pub rpm_penalty_gkwh: f64,
    /// Penalidade: g/kWh por (Δload/0.30)²
    pub load_penalty_gkwh: f64,
    pub bsfc_max_gkwh: f64,
}

impl BsfcModel {
    pub fn default_diesel() -> Self {
        Self { bsfc_min_gkwh: 200.0, rpm_optimal: 2_200.0, load_optimal: 0.70,
               rpm_penalty_gkwh: 18.0, load_penalty_gkwh: 22.0, bsfc_max_gkwh: 380.0 }
    }
    pub fn bsfc_gkwh(&self, rpm: f64, load_fraction: f64) -> f64 {
        let rp = ((rpm - self.rpm_optimal) / 1_000.0).powi(2) * self.rpm_penalty_gkwh;
        let lp = ((load_fraction - self.load_optimal) / 0.30).powi(2) * self.load_penalty_gkwh;
        (self.bsfc_min_gkwh + rp + lp).clamp(self.bsfc_min_gkwh * 0.975, self.bsfc_max_gkwh)
    }
}

impl EngineSpec {
    /// Torque por interpolação linear na curva; 0 fora de [primeiro, último] rpm.
    pub fn torque_nm(&self, rpm: f64) -> f64 {
        let pts = &self.torque_curve;
        if pts.len() < 2 { return 0.0; }
        if rpm < pts[0][0] || rpm > pts[pts.len() - 1][0] { return 0.0; }
        for w in pts.windows(2) {
            let ([r0, t0], [r1, t1]) = (w[0], w[1]);
            if rpm >= r0 && rpm <= r1 {
                let f = if r1 > r0 { (rpm - r0) / (r1 - r0) } else { 0.0 };
                return t0 + (t1 - t0) * f;
            }
        }
        0.0
    }

    pub fn power_kw(&self, rpm: f64) -> f64 {
        self.torque_nm(rpm) * rpm * 2.0 * std::f64::consts::PI / 60_000.0
    }

    /// Potência máxima varrendo a curva (para relatório; não usar como referência de carga)
    pub fn power_kw_max(&self) -> f64 {
        (0..=(self.rpm_redline as u32)).step_by(50)
            .map(|r| self.power_kw(r as f64))
            .fold(0.0, f64::max)
    }
}
```

- [ ] **Step 4: `cargo test models::engine`** — PASS.
- [ ] **Step 5: Commit** — `git commit -m "feat(engine): EngineSpec generico com curva de torque interpolada"`

### Task 1.2: Correção de altitude genérica (turbo E aspirado)

**Files:**
- Modify: `src/models/engine.rs`
- Test: `src/models/engine.rs` (mod tests)

**Interfaces:**
- Produces: `impl EngineSpec { pub fn altitude_factor(&self, altitude_m: f64) -> f64; pub fn power_kw_at(&self, rpm: f64, altitude_m: f64) -> f64 }`

- [ ] **Step 1: Testes que falham:**

```rust
    #[test]
    fn turbo_mantem_potencia_ate_altitude_critica() {
        let e = engine_teste(); // turbo, crítica 2.000 m, 10%/1.000 m acima
        assert!((e.altitude_factor(0.0) - 1.0).abs() < 1e-9);
        assert!((e.altitude_factor(2_000.0) - 1.0).abs() < 1e-9);
        assert!((e.altitude_factor(3_000.0) - 0.90).abs() < 0.01);
    }

    #[test]
    fn aspirado_perde_potencia_por_gagg_ferrar() {
        let mut e = engine_teste();
        e.induction = Induction::NaturallyAspirated;
        // Gagg-Ferrar: P/P0 = 1.132σ − 0.132; a 2.500 m σ≈0.781 → fator ≈ 0.752
        let f = e.altitude_factor(2_500.0);
        assert!((f - 0.752).abs() < 0.02, "fator aspirado a 2.500 m = {f:.3}");
    }
```

- [ ] **Step 2: Implementar** (usa `isa_density` — mover a chamada para o módulo de atmosfera na Task 4.6; por ora importar de `aerodynamics`):

```rust
    pub fn altitude_factor(&self, altitude_m: f64) -> f64 {
        match self.induction {
            Induction::NaturallyAspirated => {
                let sigma = crate::agents::aerodynamics::isa_density(altitude_m) / 1.225;
                (1.132 * sigma - 0.132).clamp(0.0, 1.0) // Gagg-Ferrar
            }
            Induction::Turbocharged { critical_altitude_m, power_loss_per_1000m } => {
                if altitude_m <= critical_altitude_m { 1.0 }
                else {
                    (1.0 - power_loss_per_1000m
                         * (altitude_m - critical_altitude_m) / 1_000.0).max(0.0)
                }
            }
        }
    }

    pub fn power_kw_at(&self, rpm: f64, altitude_m: f64) -> f64 {
        self.power_kw(rpm) * self.altitude_factor(altitude_m)
    }
```

- [ ] **Step 3: `cargo test` e commit** — `git commit -m "feat(engine): correcao de altitude turbo e aspirado (Gagg-Ferrar)"`

### Task 1.3: Carregamento TOML + arquivos de motor

**Files:**
- Modify: `Cargo.toml` (adicionar `toml = "0.8"`)
- Create: `src/models/config.rs` (loader), `config/engines/toyota_1gd_ftv.toml`, `config/engines/rotax_915is.toml`
- Modify: `src/models/mod.rs`
- Test: `src/models/config.rs` (mod tests)

**Interfaces:**
- Produces: `pub fn load_engine(path: &Path) -> Result<EngineSpec, ConfigError>` com erros descritivos (arquivo ausente, TOML inválido, curva com <2 pontos, rpm fora de ordem).

- [ ] **Step 1: Escrever `config/engines/toyota_1gd_ftv.toml`** — os dados atuais do código viram dados:

```toml
name = "Toyota 1GD-FTV 2.8 Turbo Diesel"
mass_kg = 195.0
rpm_idle = 700.0
rpm_rated = 3400.0
rpm_redline = 3800.0
rpm_max_continuous = 3000.0
torque_curve = [
  [700.0, 200.0],
  [1600.0, 500.0],
  [2800.0, 500.0],
  [3400.0, 420.0],
  [3800.0, 0.0],
]

[bsfc]
bsfc_min_gkwh = 200.0
rpm_optimal = 2200.0
load_optimal = 0.70
rpm_penalty_gkwh = 18.0
load_penalty_gkwh = 22.0
bsfc_max_gkwh = 380.0

[induction.turbocharged]
critical_altitude_m = 2000.0
power_loss_per_1000m = 0.167   # equivale aos 5%/300 m do código atual

[fuel]
name = "Diesel S-10"
density_kg_per_l = 0.840
lhv_mj_per_kg = 42.5
```

- [ ] **Step 2: Escrever `config/engines/rotax_915is.toml`** — segundo motor real para provar a genericidade (dados públicos de catálogo):

```toml
name = "Rotax 915 iS"
mass_kg = 84.0
rpm_idle = 1400.0
rpm_rated = 5800.0
rpm_redline = 5800.0
rpm_max_continuous = 5500.0
torque_curve = [
  [1400.0, 80.0],
  [4300.0, 132.0],
  [4900.0, 137.0],
  [5800.0, 111.0],
]

[bsfc]
bsfc_min_gkwh = 285.0
rpm_optimal = 4800.0
load_optimal = 0.75
rpm_penalty_gkwh = 25.0
load_penalty_gkwh = 30.0
bsfc_max_gkwh = 420.0

[induction.turbocharged]
critical_altitude_m = 4600.0
power_loss_per_1000m = 0.09

[fuel]
name = "Mogas E0 / AVGAS 100LL"
density_kg_per_l = 0.72
lhv_mj_per_kg = 43.5
```

*(Nota: o 915 iS tem redutor integrado 2.54:1 — o PSRU no `aircraft.toml` da Fase 2 deve ser configurável por motor; até lá, documentar no próprio TOML via comentário.)*

- [ ] **Step 3: Teste que falha:**

```rust
#[test]
fn carrega_os_dois_motores_do_disco() {
    let toyota = load_engine(Path::new("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let rotax  = load_engine(Path::new("config/engines/rotax_915is.toml")).unwrap();
    assert!((toyota.torque_nm(2_400.0) - 500.0).abs() < 1.0);
    assert!((rotax.power_kw(5_800.0) - 67.4).abs() < 3.0); // 111 Nm @ 5800 ≈ 67 kW
    assert!(toyota.fuel.density_kg_per_l < rotax.fuel.density_kg_per_l + 1.0);
}

#[test]
fn erro_claro_para_curva_invalida() {
    let toml_ruim = r#"
        name = "X"
        mass_kg = 1.0
        rpm_idle = 700.0
        rpm_rated = 3000.0
        rpm_redline = 3500.0
        rpm_max_continuous = 2800.0
        torque_curve = [[700.0, 200.0]]
        [bsfc]
        bsfc_min_gkwh = 200.0
        rpm_optimal = 2200.0
        load_optimal = 0.7
        rpm_penalty_gkwh = 18.0
        load_penalty_gkwh = 22.0
        bsfc_max_gkwh = 380.0
        [induction.naturally_aspirated]
        [fuel]
        name = "d"
        density_kg_per_l = 0.84
        lhv_mj_per_kg = 42.5
    "#;
    let err = parse_engine(toml_ruim).unwrap_err();
    assert!(err.to_string().contains("pelo menos 2 pontos"));
}
```

- [ ] **Step 4: Implementar `config.rs`** com `parse_engine(&str)` + `load_engine(&Path)` e validação (`torque_curve.len() >= 2`, rpm estritamente crescentes, valores ≥ 0, `rpm_max_continuous <= rpm_redline`).

- [ ] **Step 5: `cargo test` e commit** — `git commit -m "feat(config): motores carregados de TOML com validacao"`

### Task 1.4: Refatorar `PropulsionAgent` para consumir `EngineSpec`

**Files:**
- Modify: `src/agents/propulsion.rs` (REMOVER `Engine1GdFtv`, `torque_1gd_ftv`, `bsfc_gkwh`, `power_kw`, `power_kw_altitude`, `fuel_consumption_lph` com densidade fixa)
- Modify: `src/agents/performance.rs` (remover imports `torque_1gd_ftv`, `Engine1GdFtv`; funções recebem `&EngineSpec`)
- Modify: `src/validation/constraint_checker.rs:91-98` (potência específica calculada de `engine.power_kw_max()`, não `204.0`)
- Modify: `src/main.rs` (carrega o motor do TOML, imprime `engine.name`)
- Test: todos os mods de teste afetados

**Interfaces:**
- Consumes: `EngineSpec` (Task 1.1–1.3), `max_level_speed_ms` (Task 0.4)
- Produces: `PropulsionAgent::run(state, req, wing, engine: &EngineSpec) -> PropulsionSpec`; `thrust_available_n(v_ms, engine: &EngineSpec, rpm, psru_ratio, prop_diam_m, altitude_m)`; `PropulsionSpec` ganha `engine_mass_kg: f64`.

- [ ] **Step 1: Teste de genericidade (o teste central do pedido do usuário):**

```rust
#[test]
fn trocar_motor_muda_resultado_sem_mudar_codigo() {
    let state = AircraftState::initial();
    let req   = Requirements::project_default();
    let wing  = AerodynamicsAgent::run(&state, &req);
    let toyota = crate::models::config::load_engine(
        Path::new("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let rotax = crate::models::config::load_engine(
        Path::new("config/engines/rotax_915is.toml")).unwrap();

    let p_toyota = PropulsionAgent::run(&state, &req, &wing, &toyota);
    let p_rotax  = PropulsionAgent::run(&state, &req, &wing, &rotax);

    // Mesmo código, dados diferentes → resultados diferentes e coerentes
    assert!(p_toyota.power_kw > p_rotax.power_kw);
    assert!(p_toyota.fc_cruise_lph != p_rotax.fc_cruise_lph);
    assert_eq!(p_toyota.engine_model, "Toyota 1GD-FTV 2.8 Turbo Diesel");
    assert_eq!(p_rotax.engine_model, "Rotax 915 iS");
}
```

- [ ] **Step 2: Refatorar `PropulsionAgent::run`** — trocar cada uso:
  - `torque_1gd_ftv(rpm)` → `engine.torque_nm(rpm)`
  - `power_kw_altitude(rpm, alt)` → `engine.power_kw_at(rpm, alt)`
  - `bsfc_gkwh(rpm, load)` → `engine.bsfc.bsfc_gkwh(rpm, load)`
  - `0.840` → `engine.fuel.density_kg_per_l`
  - `"Toyota 1GD-FTV..."` → `engine.name.clone()`
  - `Engine1GdFtv::POWER_KW_MAX` → `p_shaft_kw` (já corrigido na Task 0.3)
  - RPM de cruzeiro `2_400.0` → busca simples: menor BSFC entre `rpm_optimal ± 20%` que entregue `p_req` (varredura de 50 em 50 rpm) — remove o número mágico.

- [ ] **Step 3: Refatorar `performance.rs`** — `thrust_available_n` e `climb_rate_ms` recebem `&EngineSpec`; rpm de subida = `engine.rpm_max_continuous` (não `2_800.0`).

- [ ] **Step 4: Grep de regressão** — `grep -rn "1GD\|1gd\|Toyota\|204\.0\|500\.0.*Nm\|0\.840" src/` deve retornar **zero** ocorrências em `src/` (dados só em `config/`).

- [ ] **Step 5: `cargo test`** — atualizar testes antigos de propulsão para construírem o motor via `load_engine` (ou o `engine_teste()` de fixture). Manter os intervalos físicos das asserções.

- [ ] **Step 6: Commit** — `git commit -m "refactor(propulsion): motor 100% generico via EngineSpec"`

### Task 1.5: Massa do motor no orçamento de peso

**Files:**
- Modify: `src/agents/weight_balance.rs:118-138` (`oew_items` recebe `engine: &EngineSpec`; item vira `MassItem { name: "Motor + acessórios", mass_kg: engine.mass_kg, ... }`)
- Modify: `src/main.rs` (passa o engine ao WeightBalanceAgent)

- [ ] **Step 1: Teste:** trocar o motor Toyota (195 kg) pelo Rotax (84 kg) desloca o CG para trás e reduz OEW em ~111 kg:

```rust
#[test]
fn massa_do_motor_afeta_oew_e_cg() {
    // fixtures como na Task 1.4 …
    let wb_toyota = WeightBalanceAgent::run(&state, &wing, &toyota);
    let wb_rotax  = WeightBalanceAgent::run(&state, &wing, &rotax);
    assert!((wb_toyota.oew_kg - wb_rotax.oew_kg - 111.0).abs() < 5.0);
    assert!(wb_rotax.scenarios[0].x_cg_m > wb_toyota.scenarios[0].x_cg_m,
        "motor mais leve no nariz → CG recua");
}
```

- [ ] **Step 2: Implementar, `cargo test`, commit** — `git commit -m "feat(weight): massa do motor vem do EngineSpec"`

---

# FASE 2 — Toda a aeronave e a missão viram configuração

*Objetivo: `aeronave --engine E.toml --aircraft A.toml --mission M.toml` produz o relatório completo. Trocar qualquer entrada = nova rodada de cálculo, zero código.*

### Task 2.1: `aircraft.toml` — geometria, braços, materiais, trem, hélice

**Files:**
- Create: `config/aircraft/baseline_4seat.toml`, `src/models/aircraft_config.rs`
- Modify: `src/models/aircraft_state.rs` (deixa de ter `initial()` mágico; é construído a partir do config), `weight_balance.rs` (`ArmConfig` desserializável), `landing_gear.rs` (`h_cg`, posições, massa por perna do config), `structural.rs` (material selecionável por nome, espaçamento de cavernas do config), `aerodynamics.rs` (perfil: `cl_max_clean`, `cl_max_flaps`, `cd0_wing`, `t/c` do config em vez de constantes NACA 23015)

**Conteúdo do TOML (estrutura completa):**

```toml
[wing]
span_m = 11.94
area_m2 = 14.2
taper_ratio = 0.45
airfoil = "NACA 23015"
thickness_ratio = 0.15
cl_max_clean = 1.45
cl_max_flaps = 1.72
cd0_wing = 0.0050
le_root_x_m = 2.90          # única fonte da posição do bordo de ataque (remove M3)

[fuselage]
length_m = 8.2
cabin_width_m = 1.22
cabin_height_m = 1.20
cd0 = 0.010

[empennage]
cd0 = 0.004
# Fase 4 dimensiona S_h/S_v por coeficiente de volume; braços iniciais:
tail_arm_m = 4.80

[propeller]
diameter_m = 1.95
blades = 2
psru_ratio = 1.867
psru_efficiency = 0.97

[fuel_system]
capacity_l = 240.0

[gear]
retractable = true
cd0_fixed_increment = 0.008
h_cg_ground_m = 1.05
x_nose_m = 1.40
x_main_m = 3.85
mass_main_leg_kg = 27.5
mass_nose_kg = 22.0
retraction_time_s = 7.0

[arms]      # braços de momento (m do datum no nariz) — hoje ArmConfig::default_layout()
engine_cg_m = 0.65
avionics_m = 1.10
pax_front_m = 3.20
fuel_cg_m = 3.55
wing_struct_m = 3.70
pax_rear_m = 4.55
fuselage_struct_m = 4.20
baggage_m = 5.60
empennage_cg_m = 7.40

[structure]
spar_material = "AA7075-T6"   # resolvido numa tabela de materiais em structural.rs
frame_spacing_mm = 300.0
design_category = "normal"    # normal | utility | acrobatic → n_lim 3.8/4.4/6.0

[masses]    # itens de OEW hoje hardcoded em oew_items()
fuselage_kg = 160.0
wing_kg = 130.0
htail_kg = 22.0
vtail_kg = 16.0
avionics_kg = 60.0
furnishings_kg = 45.0
# … (transcrever TODOS os itens da lista atual; motor vem do EngineSpec)
```

- [ ] **Step 1: Teste** — carregar o TOML e verificar campo a campo contra os valores atuais de `AircraftState::initial()` + `ArmConfig::default_layout()`; rodar o pipeline completo e comparar `aircraft_spec.json` com o da versão anterior (regressão de ouro: diferenças só onde bugs foram corrigidos).
- [ ] **Step 2: Implementar structs `AircraftConfig` desserializáveis, espelhando o TOML; `AircraftState::from_config(&AircraftConfig)`.**
- [ ] **Step 3: Propagar** — cada agente troca constantes internas pelos campos do config (a lista "Modify" acima enumera cada arquivo e o quê).
- [ ] **Step 4: `cargo test`, commit** — `git commit -m "feat(config): aeronave inteira dirigida por aircraft.toml"`

### Task 2.2: `mission.toml` — requisitos desserializáveis

**Files:**
- Create: `config/missions/default.toml`
- Modify: `src/models/requirements.rs` (derive `Deserialize`; adicionar `pax_mass_kg`, `cruise_speed_target_kmh` separado do mínimo, `reserve_minutes`, `airfield_altitude_m`, `isa_delta_c`)
- Modify: `weight_balance.rs:257,336` (massa de pax e payload passam a vir de `req` — remove M9)

```toml
passengers = 4
pax_mass_kg = 90.0
baggage_kg = 80.0
cruise_speed_min_kmh = 280.0
endurance_min_h = 8.0
fuel_reserve_fraction = 0.10
cruise_altitude_m = 2500.0
airfield_altitude_m = 0.0
isa_delta_c = 0.0
```

- [ ] **Step 1: Teste** de carregamento + teste de que `payload_kg()` respeita `pax_mass_kg` configurado.
- [ ] **Step 2: Implementar, propagar, `cargo test`, commit.**

### Task 2.3: CLI com `clap`

**Files:**
- Modify: `Cargo.toml` (`clap = { version = "4", features = ["derive"] }`), `src/main.rs`

**Interfaces:**
- Produces: `aeronave --engine <path> --aircraft <path> --mission <path> [--out aircraft_spec.json]`; defaults apontam para os arquivos `baseline`/`default`.

- [ ] **Step 1: Teste de integração** (`tests/cli.rs`): rodar o binário com `--engine config/engines/rotax_915is.toml` e verificar que o JSON de saída contém `"Rotax 915 iS"`.
- [ ] **Step 2: Implementar `struct Cli` com `#[derive(Parser)]`; mover a economia (preços de combustível) para flags opcionais `--fuel-price-brl` (remove M8).**
- [ ] **Step 3: `cargo test`, commit** — `git commit -m "feat(cli): selecao de motor/aeronave/missao por argumentos"`

---

# FASE 3 — Loop de dimensionamento (Orchestrator)

*Objetivo: o modelo passa a **convergir** os parâmetros em vez de usar chutes fixos — resolve B5.*

### Task 3.1: Convergência de MTOW

**Files:**
- Create: `src/orchestrator.rs`
- Modify: `src/main.rs` (delega ao orchestrator)

**Interfaces:**
- Produces: `pub fn size_aircraft(cfg: &AircraftConfig, engine: &EngineSpec, req: &Requirements) -> Result<SizedAircraft, SizingError>` onde `SizedAircraft` agrega todas as specs convergidas + histórico de iterações.

**Algoritmo (ponto fixo com relaxação):**

```rust
// mtow_{k+1} = 0.5·mtow_k + 0.5·(OEW(cfg, engine) + payload(req) + fuel(req, consumo_k))
// convergido quando |Δmtow| < 0.5 kg; erro se > 50 iterações ou mtow > limite do config
let mut mtow = cfg.initial_mtow_guess_kg;
for iter in 0..50 {
    let wing  = AerodynamicsAgent::run(...mtow...);
    let prop  = PropulsionAgent::run(...);         // consumo p/ a missão
    let fuel_kg = fuel_required_kg(req, &prop);    // Task 5.2 refina p/ segmentos
    let wb    = WeightBalanceAgent::run(...);      // OEW + cenários
    let novo  = wb.oew_kg + req.payload_kg() + fuel_kg;
    if (novo - mtow).abs() < 0.5 { return Ok(...); }
    mtow = 0.5 * mtow + 0.5 * novo;
}
Err(SizingError::NaoConvergiu { ultimo_mtow: mtow })
```

- [ ] **Step 1: Teste:** convergência em < 50 iterações para o baseline; teste negativo: missão impossível (autonomia 30 h com tanque de 240 L) retorna `Err`, não loop infinito.
- [ ] **Step 2: Implementar; o MTOW convergido substitui `state.mtow_kg` em TODOS os agentes (fim da inconsistência 1461 vs máximo dos cenários).**
- [ ] **Step 3: `cargo test`, commit.**

### Task 3.2: Diagrama de restrições (W/S × P/W)

**Files:**
- Create: `src/agents/constraint_diagram.rs`
- Test: mod tests no arquivo

**Interfaces:**
- Produces: `pub fn wing_loading_limits(req, cfg, engine) -> WingLoadingReport` com W/S máximo por stall (`W/S ≤ ½ρ·Vs²·CLmax`), W/S de cruzeiro ótimo (`W/S = q·√(π·AR·e·CD0)`), e P/W mínimo para RC e para decolagem (equações de Raymer cap. 5 / Gudmundsson cap. 3, escritas por extenso no código).
- Uso: o orchestrator **recomenda** área de asa (`S = MTOW·g / (W/S escolhido)`) e valida a combinação motor+asa; reporta no JSON como `sizing.constraints`.

- [ ] **Step 1: Testes com os limites conhecidos do baseline (stall 113 km/h flap → W/S ≤ ~938 N/m²).**
- [ ] **Step 2: Implementar, integrar ao relatório, commit.**

---

# FASE 4 — Parâmetros ausentes para a aeronave completa

*Cada task produz specs novas no JSON final — os inputs do CAD paramétrico.*

### Task 4.1: Dimensionamento da empenagem (resolve M1)

**Files:**
- Create: `src/agents/empennage.rs`
- Modify: `src/models/specs.rs` (+`EmpennageSpec`), `weight_balance.rs` (NP usa a empenagem dimensionada, remove `s_ratio`/`l_tail` hardcoded e o trait `TanEstimate`), `main.rs`

**Interfaces:**
- Produces: `EmpennageSpec { s_horizontal_m2, s_vertical_m2, arm_h_m, arm_v_m, chord_h_root_m, chord_h_tip_m, chord_v_root_m, chord_v_tip_m, ar_h, ar_v, volume_h, volume_v }`
- Fórmulas: `S_h = V_h · S_w · MAC / l_h` e `S_v = V_v · S_w · b / l_v` com `V_h = 0.70`, `V_v = 0.04` (Raymer Tab. 6.4, monomotor GA — valores no `aircraft.toml`, não no código). NP recalculado com `a_t/a_w` de `a = 2πAR/(2+√(4+AR²))` por superfície.

- [ ] Testes: S_h ≈ 0.70·14.2·1.246/4.8 ≈ 2.58 m²; NP recua quando S_h aumenta; commit.

### Task 4.2: Superfícies de controle e flaps

**Files:**
- Create: `src/agents/control_surfaces.rs`; Modify: `specs.rs` (+`ControlSurfacesSpec`)

**Interfaces:**
- Produces: dimensões por razões históricas (Raymer Tab. 6.5, parametrizadas no TOML): aileron 50–90% da semi-envergadura, corda 25% da corda local; flap da raiz a 50% da semi-envergadura, corda 30%; profundor 90% da envergadura do EH, corda 35%; leme corda 35% do EV. Saída: envergadura, cordas e posições iniciais/finais de cada superfície em metros.

- [ ] Testes de coerência geométrica (aileron não sobrepõe flap; somas ≤ semi-envergadura); commit.

### Task 4.3: Diagrama V-n completo com rajadas (CS 23.333/.341)

**Files:**
- Create: `src/agents/vn_diagram.rs`; Modify: `specs.rs` (+`VnDiagramSpec`), `structural.rs` (usa o n de rajada se exceder o de manobra)

**Interfaces:**
- Produces: `VnDiagramSpec { va_kmh, vb_kmh, vc_kmh, vd_kmh, n_lim_pos, n_lim_neg, n_gust_vc, n_gust_vd, points: Vec<[f64;2]> }` (polígono para plotagem/CAD).
- Fórmulas: `n_neg = -0.4·n_pos` (Normal); rajada `n = 1 ± (½ρ₀·V·a·Kg·Ude·S)/(m·g)` com `Ude = 15.24 m/s` em VC e `7.62 m/s` em VD, `Kg = 0.88μ/(5.3+μ)`, `μ = 2(m/S)/(ρ·MAC·a)`; `a` = inclinação CLα da asa 3D.

- [ ] Testes: n_gust > n_manobra para carga alar baixa (asas leves são sensíveis a rajada); dimensionamento estrutural usa `max(n_lim, n_gust)`; commit.

### Task 4.4: Envelope de CG admissível (limites, não observações)

**Files:**
- Modify: `src/agents/weight_balance.rs`; `specs.rs` (`WeightSpec` + `cg_limit_fwd_pct_mac`, `cg_limit_aft_pct_mac`)

**Interfaces:**
- Limite traseiro: `x_cg_aft = x_np − SM_min·MAC` com `SM_min = 0.05` (config). Limite dianteiro: SM ≤ 0.25 como proxy de autoridade de profundor em flare (config). Validação passa a ser: todos os cenários DENTRO do envelope (hoje só verifica SM > 0.03).

- [ ] Testes: cenário com bagagem no limite + tanque cheio dentro do envelope; caso sintético com lastro no nariz viola o limite dianteiro (teste negativo); commit.

### Task 4.5: Dimensionamento da hélice

**Files:**
- Create: `src/agents/propeller.rs`; Modify: `specs.rs` (+`PropellerSpec`), propulsion usa o diâmetro derivado quando `[propeller] diameter_m` for omitido no TOML

**Interfaces:**
- Produces: `PropellerSpec { diameter_m, blades, tip_mach_static, tip_mach_cruise, activity_factor_est }`.
- Diâmetro máximo por Mach de ponta: `D ≤ (M_tip_max·a_som/ (π·n_rps))` com `M_tip_max = 0.85` estático e `0.80` cruzeiro (config); também limite de solo (`clearance`: raio ≤ h_eixo − 0.23 m, CS 23.925).

- [ ] Testes com o baseline (1.95 m @ 1285 rpm → M_tip ~0.4, folga OK); teste negativo com PSRU 1:1 (M_tip > limite); commit.

### Task 4.6: Atmosfera ISA completa (resolve M7)

**Files:**
- Create: `src/models/atmosphere.rs`; Modify: todos os usos de `isa_density` e do `340.0`

**Interfaces:**
- Produces: `pub struct Isa; impl Isa { pub fn temperature_k(h_m, isa_delta_c) -> f64; pub fn pressure_pa(h_m) -> f64; pub fn density_kgm3(h_m, isa_delta_c) -> f64; pub fn speed_of_sound_ms(h_m, isa_delta_c) -> f64 }` — `T = 288.15 − 0.0065h`; `p = 101325·(T/288.15)^5.2561`; `ρ = p/(287.05·T)`; `a = √(1.4·287.05·T)`.

- [ ] Testes contra a tabela ISA (2.500 m: T=271.9 K, ρ=0.957 kg/m³, a=330.6 m/s); `isa_delta_c` afeta ρ (dia quente degrada decolagem); commit.

### Task 4.7: Desempenho completo (Vx, gradientes, planeio, decolagem 50 ft)

**Files:**
- Modify: `src/agents/performance.rs`; `specs.rs` (`PerformanceSpec` + `vx_kmh, vy_kmh, best_glide_kmh, glide_ratio, climb_gradient_pct, to_50ft_m, ldg_50ft_m`)

**Interfaces:**
- Vx: máximo de `(T−D)/W` sobre V; melhor planeio: `V_bg = √(2W/ρS)·(K/CD0)^0.25`, `L/D_max = 1/(2√(K·CD0))` com `K = 1/(π·AR·e)`; gradiente CS 23.65 ≥ 8.3% (validação); decolagem/pouso sobre obstáculo de 15 m via segmentos (ground roll + rotação + transição γ) substituindo os fatores ad hoc de M5; tração estática com fator empírico 0.75 sobre Rankine-Froude.

- [ ] Testes: `Vx < Vy < V_bg` ordenados; `L/D_max` ≈ valor de `1/(2√(K·CD0))` cruzado com a polar; gradiente ≥ 8.3% vira check do validador; commit.

---

# FASE 5 — Análise de missão e consumo realista

### Task 5.1: Autonomia/alcance por segmentos + Breguet

**Files:**
- Create: `src/agents/mission.rs`; Modify: `specs.rs` (+`MissionSpec { fuel_taxi_kg, fuel_climb_kg, fuel_cruise_kg, fuel_descent_kg, fuel_reserve_kg, fuel_total_kg, block_time_h, range_no_wind_km, range_with_reserve_km }`), propulsion (endurance sai daqui)

**Interfaces:**
- Segmentos: táxi (fração fixa config, default 1%), subida (integração de RC × consumo a rpm contínuo, passo 100 m até `cruise_altitude_m`), cruzeiro (Breguet: `R = (η_p/c)·(L/D)·ln(W0/W1)` com `c` de BSFC em kg/W·s), descida (idle, distância creditada), reserva (`reserve_minutes` a consumo de espera).
- O `fuel_required_kg` do orchestrator (Task 3.1) passa a chamar este módulo.

- [ ] Testes: soma dos segmentos ≤ capacidade; Breguet vs. método atual (deve dar alcance MAIOR — massa cai ao queimar combustível); autonomia de 8 h re-validada com o modelo honesto; commit.

### Task 5.2: Cargas elétrica e de refrigeração

**Files:**
- Modify: `src/agents/propulsion.rs` (+`cooling_drag_n` estimado como 3–5% do arrasto total, config), `landing_gear.rs` (potência de atuador soma ao budget elétrico), `specs.rs` (+`ElectricalSpec { bus_voltage_v, continuous_load_w, peak_load_w }` com itens do TOML)

- [ ] Testes de soma de budget; arrasto de refrigeração degrada V_max em ~2-4 km/h (verificar direção); commit.

---

# FASE 6 — Saída para CAD paramétrico

### Task 6.1: Schema JSON versionado e completo

**Files:**
- Modify: `src/models/specs.rs` (`AircraftReport` v4: adiciona `EmpennageSpec`, `ControlSurfacesSpec`, `VnDiagramSpec`, `PropellerSpec`, `MissionSpec`, `ElectricalSpec`, `sizing: SizingReport` com histórico de convergência e o diagrama de restrições; campo `schema_version: "4.0"`; TODAS as posições geométricas em metros do datum)
- Create: `docs/aircraft_spec.schema.md` (documento descrevendo cada campo, unidade e a convenção de eixos — o contrato com o time de CAD)

- [ ] Testes: serialização round-trip; `jq` no CI simples via teste de integração validando presença dos blocos; commit.

### Task 6.2: Verificação final de genericidade (teste de aceitação)

- [ ] **Rodar as duas configurações completas:**

```bash
cargo run -- --engine config/engines/toyota_1gd_ftv.toml --out spec_toyota.json
cargo run -- --engine config/engines/rotax_915is.toml  --out spec_rotax.json
```

- [ ] **Critério de aceitação:** ambos executam sem tocar em código; specs diferem em propulsão, pesos, CG, desempenho e estrutura de forma fisicamente coerente (motor mais leve → OEW menor, CG mais traseiro, MTOW convergido menor); `grep -rn "Toyota\|Rotax\|1GD\|915" src/` retorna zero.
- [ ] **Commit final + tag** — `git tag v4.0-generic`

---

## Ordem de execução e dependências

```
Fase 0 (bugs)  ──►  Fase 1 (motor genérico)  ──►  Fase 2 (config total + CLI)
                                                        │
                              Fase 4 (4.1–4.7) ◄──  Fase 3 (sizing loop)
                                     │
                              Fase 5 (missão)  ──►  Fase 6 (saída CAD)
```

- Fases 0 e 1 são o núcleo do pedido e devem vir primeiro; cada task é commitável isolada.
- Tasks 4.1–4.7 são independentes entre si (paralelizáveis), exceto 4.6 (atmosfera) que convém adiantar por ser dependência de 4.3/4.5/4.7.
- Sugestão da skill de planejamento: ao iniciar as Fases 3–6, gerar um plano detalhado por fase (este documento fixa escopo, interfaces e fórmulas; os passos TDD finos das fases 3+ seguem o padrão exibido nas Fases 0–1).

## Riscos e observações de engenharia

1. **Correções da Fase 0 podem revelar reprovações reais** (flutter, fadiga, autonomia com consumo corrigido). Isso é o modelo funcionando — as violações devem aparecer no relatório e orientar mudanças de *projeto* (longarina, tanque), nunca serem mascaradas no código.
2. **Modelos semi-empíricos** (BSFC paramétrico, flutter simplificado, Oswald de Raymer) são adequados para dimensionamento preliminar, mas o JSON final deve marcar cada bloco com o nível de fidelidade (`"fidelity": "preliminary"`) — o time de CAD/estrutura precisa saber o que exige análise posterior (FEM, ensaio de solo GVT para flutter, célula de carga para a curva real do motor).
3. **Este modelo não substitui certificação**: CS-23/RBAC-23 são usados como *guias de projeto*; a construção experimental (ANAC RBAC 21, categoria experimental) exigirá documentação própria — fora do escopo deste código.
