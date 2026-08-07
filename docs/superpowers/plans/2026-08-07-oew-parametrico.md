# OEW Paramétrico por Equações de Componente (Raymer) — Plano de Implementação

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Substituir as 7 massas estruturais fixas/calibradas do OEW (asa, fuselagem, EH, EV, trem principal, trem de nariz, tanques) por equações de componente Raymer cap. 15.2 (GA) × fatores de composto (Tab. 15.4), computadas a cada iteração do laço de convergência de MTOW — e reportar honestamente o que o baseline E7 diz depois disso.

**Architecture:** Módulo novo `src/agents/mass_model.rs` (funções puras SI→kg + `MassModelAgent`), seção nova `[mass_model]` no TOML de aeronave, acoplamento lag-1 de `n_design` (V-n roda DENTRO do laço, mesmo padrão do trim de cruzeiro do ciclo 2), `WeightBalanceAgent` consome as massas computadas com os mesmos braços dos itens removidos. Spec: `docs/superpowers/specs/2026-08-06-oew-parametrico-design.md`.

**Tech Stack:** Rust (edition existente), serde/TOML, sem dependências novas.

## Global Constraints

- SI em toda interface pública; unidades imperiais SÓ dentro de `mass_model.rs`, com constantes de conversão nomeadas e documentadas (fidelidade à fonte Raymer — expoentes não-dimensionalizáveis).
- Português em nomes/docs/mensagens, referências citadas (Raymer cap. 15.2, Tab. 15.4).
- Parâmetros/dados SÓ em config TOML: cada campo novo com faixa validada + rejection test + valor DISTINTO na fixture sintética (`config_teste()` e a string TOML dos testes de `models/config.rs`).
- Campos/itens removidos → erro de migração claro citando o substituto (padrão `check_sm_max_migration`).
- Pins honestos: old→new comentado, tolerâncias INALTERADAS. Se o baseline reprovar qualquer check após a mudança, os testes asseram o **FAIL honesto** com as violações nomeadas — NUNCA mascarar (nem afrouxar tolerância, nem "calibrar" fator para passar).
- Genericidade: teste de aceitação (grep de `src/` sem nomes de motor) continua verde.
- `cargo test` verde ao fim de CADA task.

## Entradas E7 congeladas (fonte única dos hand-checks das Tasks 1 e 3)

Extraídas do `aircraft_spec.json` E7 (`da078cb`) + valores da spec em 2026-08-07. Os testes congelam ESTES números — não releem o TOML real:

| Entrada | Valor | Origem |
|---|---|---|
| `w_dg_kg` | 1548.4 | weight.mtow_kg |
| `n_z_ult` | 6.286149 | 1.5 × n_design 4.190766 |
| `s_w_m2` | 14.2 · `ar` 10.03969014084507 · `taper` 0.45 · `t_c` 0.15 | wing |
| `q_pa` | 3366.1331 | 0.5·0.9570·(301.9/3.6)² (ISA 2500 m, V de cruzeiro E7) |
| `w_fw_kg` | 218.4 | weight.fuel_mass_kg |
| `s_ht_m2` 3.133966 · `s_vt_m2` 1.412900 · `ar_h` 4.0 · `ar_v` 1.5 · `taper_h` 0.5 · `taper_v` 0.5 | | empennage |
| `s_f_m2` | 25.117033 | 0.75·π·1.30·8.2 |
| `l_t_m` | 4.8 · `l_over_d` 6.307692 | empennage.arm_h_m; 8.2/1.30 |
| `n_l_ult` | 4.5 · `w_l_kg` 1548.4 | fator de pouso ultimate; W_l = MTOW (conservador) |
| `strut` | 0.67 m (principal) / 0.53 m (nariz) | [mass_model] |
| `capacity_l` | 260.0 | fuel_system |

Pins esperados (kg, ±0.1) — **raw Raymer** e **× fator de composto**:

| Componente | raw | fator | ×comp |
|---|---|---|---|
| asa | 176.17 | 0.85 | 149.74 |
| EH | 16.76 | 0.83 | 13.91 |
| EV | 7.60 | 0.83 | 6.31 |
| fuselagem | 127.91 | 0.90 | 115.12 |
| trem principal | 97.60 | 0.95 | 92.72 |
| trem nariz | 21.19 | 0.95 | 20.13 |
| tanques | 22.39 | 1.00 | 22.39 |

---

### Task 1: Funções puras Raymer em `src/agents/mass_model.rs`

**Files:**
- Create: `src/agents/mass_model.rs`
- Modify: `src/agents/mod.rs` (adicionar `pub mod mass_model;`)

**Interfaces:**
- Produces: `StructuralMasses` (struct pública), 7 funções puras `*_raymer_kg` (assinaturas abaixo) e constantes de conversão — consumidas pela Task 3.

- [ ] **Step 1: Escrever os testes que falham (RED)** — em `#[cfg(test)] mod tests` do próprio `mass_model.rs`:

```rust
// Entradas E7 congeladas (ver tabela do plano — NÃO ler TOML real aqui).
const W_DG_KG: f64 = 1548.4;
const N_Z_ULT: f64 = 6.286149;
const Q_PA: f64 = 3366.1331;

#[test]
fn hand_check_asa_raw_e_com_fator_de_composto() {
    let raw = wing_mass_raymer_kg(14.2, 218.4, 10.03969014084507, Q_PA, 0.45, 0.15, N_Z_ULT, W_DG_KG);
    assert!((raw - 176.17).abs() < 0.1, "asa raw = {raw:.2} kg (esperado 176.17 ±0.1)");
    let comp = raw * 0.85;
    assert!((comp - 149.74).abs() < 0.1, "asa ×0.85 = {comp:.2} kg (esperado 149.74 ±0.1)");
}

#[test]
fn hand_check_empenagem_horizontal() {
    let raw = htail_mass_raymer_kg(N_Z_ULT, W_DG_KG, Q_PA, 3.133966, 0.15, 4.0, 0.5);
    assert!((raw - 16.76).abs() < 0.1, "EH raw = {raw:.2} kg (esperado 16.76 ±0.1)");
    assert!((raw * 0.83 - 13.91).abs() < 0.1);
}

#[test]
fn hand_check_empenagem_vertical() {
    let raw = vtail_mass_raymer_kg(N_Z_ULT, W_DG_KG, Q_PA, 1.412900, 0.15, 1.5, 0.5);
    assert!((raw - 7.60).abs() < 0.1, "EV raw = {raw:.2} kg (esperado 7.60 ±0.1)");
    assert!((raw * 0.83 - 6.31).abs() < 0.1);
}

#[test]
fn hand_check_fuselagem() {
    let raw = fuselage_mass_raymer_kg(25.117033, N_Z_ULT, W_DG_KG, 4.8, 6.307692, Q_PA);
    assert!((raw - 127.91).abs() < 0.1, "fuselagem raw = {raw:.2} kg (esperado 127.91 ±0.1)");
    assert!((raw * 0.90 - 115.12).abs() < 0.1);
}

#[test]
fn hand_check_trem_principal() {
    let raw = main_gear_mass_raymer_kg(4.5, W_DG_KG, 0.67);
    assert!((raw - 97.60).abs() < 0.1, "trem principal raw = {raw:.2} kg (esperado 97.60 ±0.1)");
    assert!((raw * 0.95 - 92.72).abs() < 0.1);
}

#[test]
fn hand_check_trem_nariz() {
    let raw = nose_gear_mass_raymer_kg(4.5, W_DG_KG, 0.53);
    assert!((raw - 21.19).abs() < 0.1, "trem nariz raw = {raw:.2} kg (esperado 21.19 ±0.1)");
    assert!((raw * 0.95 - 20.13).abs() < 0.1);
}

#[test]
fn hand_check_sistema_de_combustivel() {
    let raw = fuel_system_mass_raymer_kg(260.0);
    assert!((raw - 22.39).abs() < 0.1, "tanques raw = {raw:.2} kg (esperado 22.39 ±0.1)");
}

// ─── Propriedades ESTRITAS de direção (spec, seção Testes item 2) ───
#[test]
fn massa_da_asa_cresce_com_area_e_com_n_z() {
    let base = wing_mass_raymer_kg(14.2, 218.4, 10.04, Q_PA, 0.45, 0.15, N_Z_ULT, W_DG_KG);
    let s_maior = wing_mass_raymer_kg(14.2 * 1.2, 218.4, 10.04, Q_PA, 0.45, 0.15, N_Z_ULT, W_DG_KG);
    let nz_maior = wing_mass_raymer_kg(14.2, 218.4, 10.04, Q_PA, 0.45, 0.15, N_Z_ULT * 1.2, W_DG_KG);
    assert!(s_maior > base, "∂m_asa/∂S > 0: base={base:.2} s_maior={s_maior:.2}");
    assert!(nz_maior > base, "∂m_asa/∂N_z > 0: base={base:.2} nz_maior={nz_maior:.2}");
}

#[test]
fn massa_do_trem_cresce_com_peso_e_tanques_com_capacidade() {
    let mg_base = main_gear_mass_raymer_kg(4.5, W_DG_KG, 0.67);
    let mg_pesado = main_gear_mass_raymer_kg(4.5, W_DG_KG * 1.2, 0.67);
    assert!(mg_pesado > mg_base, "∂m_trem/∂MTOW > 0");
    let fs_base = fuel_system_mass_raymer_kg(260.0);
    let fs_maior = fuel_system_mass_raymer_kg(320.0);
    assert!(fs_maior > fs_base, "∂m_tanques/∂capacidade > 0");
}
```

- [ ] **Step 2: Rodar e confirmar que falha**

Run: `cargo test --lib mass_model`
Expected: FAIL (funções não existem — erro de compilação conta como RED aqui).

- [ ] **Step 3: Implementar o módulo** — docstring do módulo cita Raymer cap. 15.2 (GA) e Tab. 15.4, explica a política SI-fora/imperial-dentro e a aproximação documentada de usar `t_c` da ASA também nas empenagens (t/c dedicado da empenagem não existe no config — degrau deste ciclo):

```rust
//! MassModelAgent — massas estruturais por equações de componente
//! (Raymer, "Aircraft Design: A Conceptual Approach", cap. 15.2, equações
//! GA; fatores de composto da Tab. 15.4 vêm de [mass_model] no TOML).
//! Interface em SI; internamente as equações usam unidades imperiais
//! (fidelidade à fonte — expoentes empíricos não-dimensionalizáveis).

/// lb por kg (NIST)
pub const LB_PER_KG: f64 = 2.20462;
/// ft² por m²
pub const FT2_PER_M2: f64 = 10.7639;
/// psf por Pa
pub const PSF_PER_PA: f64 = 0.020885;
/// gal US por litro
pub const GAL_PER_L: f64 = 0.264172;
/// polegadas por metro
pub const IN_PER_M: f64 = 39.3701;

/// Nº de tanques (asas integrais, um por semi-asa) — constante de layout
/// desta configuração, não dado de projeto variável (Raymer eq. 15.2,
/// expoente fraco 0.242).
const N_TANKS: f64 = 2.0;
/// Nº de motores (monomotor — expoente 0.157).
const N_ENGINES: f64 = 1.0;

/// As 7 massas estruturais computadas (kg) — spec ciclo 3.
#[derive(Debug, Clone)]
pub struct StructuralMasses {
    pub asa_kg: f64,
    pub fuselagem_kg: f64,
    pub emp_h_kg: f64,
    pub emp_v_kg: f64,
    pub trem_principal_kg: f64,
    pub trem_nariz_kg: f64,
    pub tanques_kg: f64,
}

/// Raymer 15.2 (GA), asa sem enflechamento (cos Λ = 1):
/// W = 0.036·S^0.758·W_fw^0.0035·A^0.6·q^0.006·λ^0.04·(100 t/c)^-0.3·(N_z·W_dg)^0.49
pub fn wing_mass_raymer_kg(
    s_w_m2: f64, w_fw_kg: f64, ar: f64, q_pa: f64,
    taper: f64, t_c: f64, n_z_ult: f64, w_dg_kg: f64,
) -> f64 {
    let s_w = s_w_m2 * FT2_PER_M2;
    let w_fw = w_fw_kg * LB_PER_KG;
    let q = q_pa * PSF_PER_PA;
    let w_dg = w_dg_kg * LB_PER_KG;
    let w_lb = 0.036 * s_w.powf(0.758) * w_fw.powf(0.0035) * ar.powf(0.6)
        * q.powf(0.006) * taper.powf(0.04) * (100.0 * t_c).powf(-0.3)
        * (n_z_ult * w_dg).powf(0.49);
    w_lb / LB_PER_KG
}

/// W_ht = 0.016·(N_z·W_dg)^0.414·q^0.168·S_ht^0.896·(100 t/c)^-0.12·A_h^0.043·λ_h^-0.02
pub fn htail_mass_raymer_kg(
    n_z_ult: f64, w_dg_kg: f64, q_pa: f64, s_ht_m2: f64,
    t_c: f64, ar_h: f64, taper_h: f64,
) -> f64 {
    let w_dg = w_dg_kg * LB_PER_KG;
    let q = q_pa * PSF_PER_PA;
    let s_ht = s_ht_m2 * FT2_PER_M2;
    let w_lb = 0.016 * (n_z_ult * w_dg).powf(0.414) * q.powf(0.168)
        * s_ht.powf(0.896) * (100.0 * t_c).powf(-0.12) * ar_h.powf(0.043)
        * taper_h.powf(-0.02);
    w_lb / LB_PER_KG
}

/// W_vt = 0.073·(1+0.2·H_t/H_v)·(N_z·W_dg)^0.376·q^0.122·S_vt^0.873·
///        (100 t/c)^-0.49·A_v^0.357·λ_v^0.039 — cauda convencional:
///        H_t/H_v = 0 (estabilizador na fuselagem), fator = 1.0.
pub fn vtail_mass_raymer_kg(
    n_z_ult: f64, w_dg_kg: f64, q_pa: f64, s_vt_m2: f64,
    t_c: f64, ar_v: f64, taper_v: f64,
) -> f64 {
    let w_dg = w_dg_kg * LB_PER_KG;
    let q = q_pa * PSF_PER_PA;
    let s_vt = s_vt_m2 * FT2_PER_M2;
    let w_lb = 0.073 * 1.0 * (n_z_ult * w_dg).powf(0.376) * q.powf(0.122)
        * s_vt.powf(0.873) * (100.0 * t_c).powf(-0.49) * ar_v.powf(0.357)
        * taper_v.powf(0.039);
    w_lb / LB_PER_KG
}

/// W_fus = 0.052·S_f^1.086·(N_z·W_dg)^0.177·L_t^-0.051·(L/D)^-0.072·q^0.241
/// — SEM o termo de pressurização (cabine não pressurizada, spec).
pub fn fuselage_mass_raymer_kg(
    s_f_m2: f64, n_z_ult: f64, w_dg_kg: f64, l_t_m: f64,
    l_over_d: f64, q_pa: f64,
) -> f64 {
    let s_f = s_f_m2 * FT2_PER_M2;
    let w_dg = w_dg_kg * LB_PER_KG;
    let l_t = l_t_m * 3.28084;
    let q = q_pa * PSF_PER_PA;
    let w_lb = 0.052 * s_f.powf(1.086) * (n_z_ult * w_dg).powf(0.177)
        * l_t.powf(-0.051) * l_over_d.powf(-0.072) * q.powf(0.241);
    w_lb / LB_PER_KG
}

/// W_mg = 0.095·(N_l·W_l)^0.768·(L_m/12)^0.409 — L_m em polegadas.
pub fn main_gear_mass_raymer_kg(n_l_ult: f64, w_l_kg: f64, strut_len_m: f64) -> f64 {
    let w_l = w_l_kg * LB_PER_KG;
    let l_m_in = strut_len_m * IN_PER_M;
    let w_lb = 0.095 * (n_l_ult * w_l).powf(0.768) * (l_m_in / 12.0).powf(0.409);
    w_lb / LB_PER_KG
}

/// W_ng = 0.125·(N_l·W_l)^0.566·(L_n/12)^0.845 — L_n em polegadas.
pub fn nose_gear_mass_raymer_kg(n_l_ult: f64, w_l_kg: f64, strut_len_m: f64) -> f64 {
    let w_l = w_l_kg * LB_PER_KG;
    let l_n_in = strut_len_m * IN_PER_M;
    let w_lb = 0.125 * (n_l_ult * w_l).powf(0.566) * (l_n_in / 12.0).powf(0.845);
    w_lb / LB_PER_KG
}

/// W_fs = 2.49·V_t^0.726·(1/(1+V_i/V_t))^0.363·N_t^0.242·N_en^0.157 —
/// V_t em galões US; tanques INTEGRAIS (V_i/V_t = 1, spec: "tanques
/// integrais compostos ≈ metálicos").
pub fn fuel_system_mass_raymer_kg(capacity_l: f64) -> f64 {
    let v_t = capacity_l * GAL_PER_L;
    let w_lb = 2.49 * v_t.powf(0.726) * (1.0_f64 / (1.0 + 1.0)).powf(0.363)
        * N_TANKS.powf(0.242) * N_ENGINES.powf(0.157);
    w_lb / LB_PER_KG
}
```

Nota (fuselagem): o literal `3.28084` (ft por m) deve virar a constante nomeada `FT_PER_M` — declarar `pub const FT_PER_M: f64 = 3.28084;` junto às demais e usá-la.

- [ ] **Step 4: Rodar e confirmar que passa**

Run: `cargo test --lib mass_model`
Expected: PASS (9 testes).

- [ ] **Step 5: Suite completa + commit**

Run: `cargo test`
```bash
git add src/agents/mass_model.rs src/agents/mod.rs
git commit -m "feat(mass_model): equações de componente Raymer 15.2 GA (funções puras SI)"
```

---

### Task 2: Seção `[mass_model]` no config (struct + faixas + fixtures + TOMLs)

**Files:**
- Modify: `src/models/aircraft_config.rs` (struct `MassModelCfg`, campo em `AircraftConfig`, fixture `config_teste()`)
- Modify: `src/models/config.rs` (validação de faixas + rejection tests + string TOML de teste — localizar pela âncora `mass_per_area_h_kg_m2 = 9.0`)
- Modify: `config/aircraft/baseline_4seat.toml` (nova seção com comentários citando Raymer Tab. 15.4)
- Test: rejection tests em `src/models/config.rs`; `tests/config_files.rs` (carrega os TOMLs reais) deve continuar verde

**Interfaces:**
- Produces: `cfg.mass_model` com os campos abaixo (todos `f64`), consumido pela Task 3.

- [ ] **Step 1: Rejection tests (RED)** — em `models/config.rs`, seguir o padrão exato de `rejeita_mass_per_area_h_fora_da_faixa` (replace na string TOML de teste + assert na mensagem). Um teste por campo, 10 campos, cada um mutado para fora da faixa (ex.: `composite_factor_wing = 1.5`, `d_fus_equiv_m = 0.5`, `landing_load_factor_ult = 9.0`, `main_strut_length_m = 2.0`…). Mensagem de erro no padrão existente: `"configuração de aeronave inválida: mass_model.<campo> deve estar em (<lo>, <hi>) — valor: <v>"`.

- [ ] **Step 2: Rodar e confirmar RED** (`cargo test --lib config` — falha de compilação/parse esperada).

- [ ] **Step 3: Struct + validação + fixtures + TOML real.**

Em `aircraft_config.rs` (docstrings citando a base de cada valor, como nas demais seções):

```rust
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
}
```

+ campo `pub mass_model: MassModelCfg,` em `AircraftConfig`.

Faixas (em `config.rs`, padrão `require_finite` + range): fatores de composto (0.6, 1.1) — exceto `composite_factor_fuel_system` (0.6, 1.2); `d_fus_equiv_m` (0.9, 2.0); `fuselage_wetted_coeff` (0.5, 0.95); `landing_load_factor_ult` (3.0, 7.0); `main_strut_length_m`/`nose_strut_length_m` (0.3, 1.5).

Fixture sintética `config_teste()` — valores DISTINTOS do baseline real:

```rust
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
},
```

String TOML de teste em `config.rs`: mesma seção com os MESMOS valores da fixture sintética (é a base dos replaces dos rejection tests).

`config/aircraft/baseline_4seat.toml` — valores da spec (comentar base de cada um):

```toml
# Modelo de massas estruturais (ciclo 3) — equações de componente Raymer
# cap. 15.2 (GA) × fatores de composto (Tab. 15.4). As 7 massas estruturais
# do OEW são COMPUTADAS por agents::mass_model — os antigos itens fixos de
# [[masses.items]] (asa, fuselagem, trem_principal, trem_nariz, tanques) e
# [empennage].mass_per_area_* foram removidos (erro de migração se presentes).
[mass_model]
composite_factor_wing        = 0.85  # Raymer Tab. 15.4 (asa composta)
composite_factor_tail        = 0.83  # idem (empenagens)
composite_factor_fuselage    = 0.90  # idem (fuselagem)
composite_factor_gear        = 0.95  # idem (trem)
composite_factor_fuel_system = 1.00  # tanques integrais compostos ≈ metálicos
d_fus_equiv_m                = 1.30  # cabine 1.22 m + estrutura
fuselage_wetted_coeff        = 0.75  # corpo afilado vs cilindro pleno
landing_load_factor_ult      = 4.5   # N_l = N_pouso×1.5 (Raymer 15.2)
main_strut_length_m          = 0.67  # curso oleo E7 (212 mm) + roda ≈ 26.3 in
nose_strut_length_m          = 0.53  # idem
```

- [ ] **Step 4: Rodar tudo**

Run: `cargo test`
Expected: PASS (rejection tests verdes; TOMLs reais carregam).

- [ ] **Step 5: Commit**

```bash
git add src/models/aircraft_config.rs src/models/config.rs config/aircraft/baseline_4seat.toml
git commit -m "feat(config): seção [mass_model] — fatores de composto e geometria auxiliar (Raymer Tab. 15.4)"
```

---

### Task 3: `MassModelAgent` + lag-1 de `n_design` no orchestrator (computa e carrega — ainda NÃO consome)

**Files:**
- Modify: `src/agents/mass_model.rs` (agente)
- Modify: `src/orchestrator.rs` (lag-1 + V-n no laço + campos novos em `SizedAircraft`)
- Modify: `src/main.rs` (usar `sized.vn` em vez de recomputar o V-n)

**Interfaces:**
- Consumes: funções puras da Task 1, `cfg.mass_model` da Task 2, `EmpennageSpec`, `WingSpec`, `EngineSpec::density_kg_per_l`, `Isa::density_kgm3`, `VnDiagramAgent::run` (assinatura existente: `(wing, mtow_envelope_kg, mass_light_kg, req, category) -> VnDiagramSpec`).
- Produces: `MassModelAgent::run(cfg, engine, req, wing, emp, mtow_kg, n_design) -> StructuralMasses`; `SizedAircraft` ganha `pub structural_masses: StructuralMasses`, `pub vn: VnDiagramSpec`, `pub n_design_iterations: Vec<f64>` — consumidos pelas Tasks 4–6.

- [ ] **Step 1: Testes (RED)** — no mod tests de `mass_model.rs` e `orchestrator.rs`:

```rust
// mass_model.rs — o agente aplica q/W_fw/S_f derivados + fatores de composto
// sobre as funções puras (teste RELACIONAL, sem números mágicos novos).
#[test]
fn agente_aplica_fatores_de_composto_sobre_as_funcoes_puras() {
    let cfg = config_teste();
    let engine = motor_generico_teste();
    let req = requisitos_teste();
    let state = AircraftState::from_config(&cfg);
    let wing = AerodynamicsAgent::run(&state, &req);
    let emp = EmpennageAgent::run(&wing, &cfg);
    let mtow = 1_400.0;
    let n_design = 4.0;
    let m = MassModelAgent::run(&cfg, &engine, &req, &wing, &emp, mtow, n_design);

    // Reconstrói as MESMAS entradas derivadas que o agente deve usar:
    let rho = Isa::density_kgm3(req.cruise_altitude_m, req.isa_delta_c);
    let v_ms = req.cruise_speed_min_kmh / 3.6;
    let q_pa = 0.5 * rho * v_ms * v_ms;
    let w_fw = cfg.fuel_system.capacity_l * engine.density_kg_per_l;
    let esperado_asa = wing_mass_raymer_kg(
        wing.area_m2, w_fw, wing.aspect_ratio, q_pa, wing.taper_ratio,
        cfg.wing.thickness_ratio, 1.5 * n_design, mtow,
    ) * cfg.mass_model.composite_factor_wing;
    assert!((m.asa_kg - esperado_asa).abs() < 1e-9,
        "asa_kg = {:.4} (esperado {esperado_asa:.4})", m.asa_kg);
    // ...idem para os outros 6 campos (fuselagem usa S_f =
    // fuselage_wetted_coeff·π·d_fus_equiv·fuselage.length_m e
    // l_over_d = length_m/d_fus_equiv; trens usam landing_load_factor_ult,
    // strut lengths e W_l = mtow; tanques usam capacity_l).
    assert!(m.asa_kg > 0.0 && m.tanques_kg > 0.0);
}

// Empenagem responde a v_h NOS DOIS SENTIDOS (spec, Testes item 2) —
// substitui a property de mass_per_area do ciclo 2.
#[test]
fn massa_da_empenagem_horizontal_responde_a_v_h_nos_dois_sentidos() {
    // cfg base → emp base → m base; cfg.empennage.v_h *= 1.2 → m_maior;
    // *= 0.8 → m_menor. assert m_maior.emp_h_kg > base > m_menor.emp_h_kg.
}
```

```rust
// orchestrator.rs — convergência do lag-1 de n_design no CAMPO REAL
// (lição do ciclo 2: nunca duplicar o corpo do laço em teste).
#[test]
fn n_design_iterations_do_campo_real_converge() {
    let sized = size_aircraft(&config_teste(), &engine_teste(), &requisitos_teste())
        .expect("baseline sintético deveria convergir");
    let h = &sized.n_design_iterations;
    assert!(h.len() >= 2);
    // seed 3.8 na primeira entrada (N_z = 1.5×3.8 = 5.70, spec):
    assert!((h[0] - 3.8).abs() < 1e-12, "seed do lag deveria ser 3.8, obtido {}", h[0]);
    let delta_final = (h[h.len() - 1] - h[h.len() - 2]).abs();
    // PIN HONESTO: rodar, medir o residual real e pinar aqui com 2× de
    // folga (mesmo padrão de cl_h_trim_iterations_do_campo_real_...).
    // O implementador DEVE imprimir o histórico e registrar o valor medido.
    assert!(delta_final < 0.05, "residual do lag de n_design = {delta_final:.3e}");
    // + structural_masses do SizedAircraft finitas e positivas:
    for (nome, v) in [("asa", sized.structural_masses.asa_kg),
                      ("fuselagem", sized.structural_masses.fuselagem_kg),
                      ("tanques", sized.structural_masses.tanques_kg)] {
        assert!(v.is_finite() && v > 0.0, "{nome} = {v}");
    }
}
```

- [ ] **Step 2: Confirmar RED** (`cargo test --lib`).

- [ ] **Step 3: Implementar.**

`MassModelAgent::run` em `mass_model.rs`:

```rust
pub struct MassModelAgent;

impl MassModelAgent {
    /// `n_design`: fator de carga LIMITE (o agente aplica ×1.5 para o
    /// ultimate N_z). No laço do orchestrator vem com LAG-1 (iteração
    /// anterior; seed 3.8 → N_z 5.70) — ver `orchestrator::size_aircraft`.
    /// q de cruzeiro vem do REQUISITO (cruise_speed_min_kmh + ISA na
    /// altitude de missão), não da velocidade real da iteração — expoentes
    /// de q são fracos (0.006–0.241), erro ≤3% (spec).
    pub fn run(
        cfg: &AircraftConfig, engine: &EngineSpec, req: &Requirements,
        wing: &WingSpec, emp: &EmpennageSpec, mtow_kg: f64, n_design: f64,
    ) -> StructuralMasses {
        assert!(mtow_kg > 0.0, "MTOW deve ser positivo, obtido {mtow_kg}");
        assert!(n_design > 0.0, "n_design deve ser positivo, obtido {n_design}");
        let mm = &cfg.mass_model;
        let rho = Isa::density_kgm3(req.cruise_altitude_m, req.isa_delta_c);
        let v_ms = req.cruise_speed_min_kmh / 3.6;
        let q_pa = 0.5 * rho * v_ms * v_ms;
        let w_fw_kg = cfg.fuel_system.capacity_l * engine.density_kg_per_l;
        let n_z_ult = 1.5 * n_design;
        let t_c = cfg.wing.thickness_ratio; // aproximação: mesmo t/c nas empenagens
        let s_f_m2 = mm.fuselage_wetted_coeff * std::f64::consts::PI
            * mm.d_fus_equiv_m * cfg.fuselage.length_m;
        let l_over_d = cfg.fuselage.length_m / mm.d_fus_equiv_m;
        StructuralMasses {
            asa_kg: wing_mass_raymer_kg(wing.area_m2, w_fw_kg, wing.aspect_ratio,
                q_pa, wing.taper_ratio, t_c, n_z_ult, mtow_kg)
                * mm.composite_factor_wing,
            emp_h_kg: htail_mass_raymer_kg(n_z_ult, mtow_kg, q_pa,
                emp.s_horizontal_m2, t_c, emp.ar_h, emp.taper_h)
                * mm.composite_factor_tail,
            emp_v_kg: vtail_mass_raymer_kg(n_z_ult, mtow_kg, q_pa,
                emp.s_vertical_m2, t_c, emp.ar_v, emp.taper_v)
                * mm.composite_factor_tail,
            fuselagem_kg: fuselage_mass_raymer_kg(s_f_m2, n_z_ult, mtow_kg,
                emp.arm_h_m, l_over_d, q_pa)
                * mm.composite_factor_fuselage,
            trem_principal_kg: main_gear_mass_raymer_kg(
                mm.landing_load_factor_ult, mtow_kg, mm.main_strut_length_m)
                * mm.composite_factor_gear,
            trem_nariz_kg: nose_gear_mass_raymer_kg(
                mm.landing_load_factor_ult, mtow_kg, mm.nose_strut_length_m)
                * mm.composite_factor_gear,
            tanques_kg: fuel_system_mass_raymer_kg(cfg.fuel_system.capacity_l)
                * mm.composite_factor_fuel_system,
        }
    }
}
```

`orchestrator.rs` — dentro de `size_aircraft_with_max_iters`:
1. Antes do laço: `let mut n_design_prev: f64 = 3.8;` (comentário: seed = fator de manobra normal, N_z = 5.70, spec) e `let mut n_design_iterations: Vec<f64> = Vec::new();`.
2. Logo após `let emp = EmpennageAgent::run(...)`: `let masses = MassModelAgent::run(cfg, engine, req, &wing, &emp, mtow, n_design_prev);` (nesta task `masses` ainda não alimenta o WB — anotar com comentário "consumido pelo WeightBalanceAgent a partir da Task 4").
3. Logo após o bloco que atualiza `x_cg_trim_ref_prev` (o V-n precisa do `wb` desta iteração): calcular `mass_light` (mesma expressão de `main.rs`: mínimo de `wb.scenarios[].total_mass_kg`), rodar `let vn = VnDiagramAgent::run(&wing, wb.spec.mtow_kg, mass_light, req, &cfg.structure.design_category);`, `n_design_prev = vn.n_design; n_design_iterations.push(vn.n_design);`.
4. No retorno convergido: incluir `structural_masses: masses`, `vn`, `n_design_iterations` no `SizedAircraft` (campos novos com docstrings no padrão dos existentes, explicando o lag-1 e o residual honesto).
5. Imports novos no topo.

`main.rs`: apagar o cálculo local do V-n (bloco da linha ~378: `mass_light_kg` + `VnDiagramAgent::run`) e usar `let vn = &sized.vn;` — valores idênticos (mesmas entradas da iteração convergida); manter os `println!` existentes (o `mass_light_kg` impresso pode ser recalculado localmente da mesma expressão, só para o print).

- [ ] **Step 4: Rodar, medir o residual real do lag, pinar honesto** (substituir o placeholder 0.05 do teste pelo pin medido com folga 2×, comentado).

Run: `cargo test`
Expected: PASS; `cargo run -- --engine config/engines/*.toml ...` NÃO precisa rodar aqui (comportamento do baseline inalterado — massas ainda não consumidas).

- [ ] **Step 5: Commit**

```bash
git add src/agents/mass_model.rs src/orchestrator.rs src/main.rs
git commit -m "feat(mass_model): MassModelAgent no laço com lag-1 de n_design (V-n por iteração)"
```

---

### Task 4: O corte — WB consome massas computadas; itens legados removidos com migração; pins honestos

Esta é a task de mudança de comportamento (padrão "golden update honesto" dos ciclos E6/E7). O baseline real E7 MUDA de números aqui; o resultado que sair é o achado do ciclo — reportar, nunca mascarar.

**Files:**
- Modify: `src/agents/weight_balance.rs` (`oew_items` + `WeightBalanceAgent::run` ganham `&StructuralMasses`; itens computados)
- Modify: `src/orchestrator.rs` (passa `&masses` ao WB)
- Modify: `src/models/aircraft_config.rs` (remove `mass_per_area_h_kg_m2`/`mass_per_area_v_kg_m2` de `EmpennageCfg` e `mass_main_leg_kg` de `GearCfg`; fixtures)
- Modify: `src/models/config.rs` (migrações novas; remove checks obsoletos)
- Modify: `src/agents/landing_gear.rs` (atuador usa `mass_main_total/2.0`)
- Modify: `src/main.rs` (massas p/ StructuralAgent e LandingGearAgent vêm de `sized.structural_masses`)
- Modify: `src/validation/constraint_checker.rs` (fixture de teste usa massas computadas)
- Modify: `config/aircraft/baseline_4seat.toml` (remove 5 itens + 2 campos + `mass_main_leg_kg`, com comentários)
- Modify: pins em `src/` e `tests/` (generic_engine.rs, cli.rs, schema_v4.rs, gear_tipback.rs, acceptance.rs, empennage.rs, performance.rs etc. — todos que quebrarem)

**Interfaces:**
- Consumes: `StructuralMasses`/`SizedAircraft.structural_masses` (Task 3).
- Produces: `oew_items(cfg, engine, emp, masses: &StructuralMasses) -> Vec<MassItem>`; `WeightBalanceAgent::run(state, wing, engine, cfg, req, emp, masses)`.

- [ ] **Step 1: Testes de migração (RED)** — em `config.rs`, padrão de `rejeita_config_antiga_com_sm_max_com_erro_de_migracao_claro`:
  - TOML com item `name = "asa"` em `[[masses.items]]` → erro citando `[mass_model]` (idem para fuselagem/trem_principal/trem_nariz/tanques — um teste cobrindo pelo menos 2 nomes + a lista completa na checagem).
  - TOML com `mass_per_area_h_kg_m2` em `[empennage]` → erro citando `[mass_model].composite_factor_tail`/equações Raymer (idem `_v_`).
  - TOML com `mass_main_leg_kg` em `[gear]` → erro citando que a massa do trem agora é computada (`agents::mass_model`) e o atuador usa a massa computada da perna.

- [ ] **Step 2: Confirmar RED.**

- [ ] **Step 3: Implementar o corte, nesta ordem (um `cargo build` entre cada grupo):**

1. **`weight_balance.rs`**: `oew_items(cfg, engine, emp, masses)` — remove os dois `items.push` de `mass_per_area` e adiciona os 7 computados com o mapeamento estático componente→braço (MESMOS braços dos itens removidos — spec, seção Braços de CG):

```rust
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
```

`WeightBalanceAgent::run(..., masses: &StructuralMasses)` repassa para `oew_items`. Atualizar docstrings e os testes do módulo (o teste `oew_items_deriva_massa_da_empenagem_...` muda de sentido: agora verifica `masses.emp_h_kg`).

2. **`orchestrator.rs`**: `WeightBalanceAgent::run(&state, &wing, engine, cfg, req, &emp, &masses)`.
3. **`aircraft_config.rs`**: remover os 2 campos de `EmpennageCfg` e `mass_main_leg_kg` de `GearCfg` (+ fixtures `config_teste()`); `constraint_checker.rs`/`landing_gear.rs` fixtures idem.
4. **`config.rs`**: adicionar as 3 guardas de migração (Step 1); ESTENDER `check_emp_mass_items_migration` (ou substituí-la por `check_structural_mass_items_migration`) para rejeitar os 7 nomes `{asa, fuselagem, trem_principal, trem_nariz, tanques, emp_horizontal, emp_vertical}`; REMOVER: as faixas de `mass_per_area_*`, a exigência dos itens obrigatórios `["asa","trem_principal","trem_nariz"]` (agora proibidos) e a checagem `trem_principal == 2× mass_main_leg_kg` (o campo morre).
5. **`landing_gear.rs`**: `actuator_power_w(mass_main_total / 2.0, ...)` — a massa de UMA perna agora deriva da massa computada total (comentário citando o ciclo 3); fixture local sem `mass_main_leg_kg`.
6. **`main.rs`**: `let wing_mass_kg = sized.structural_masses.asa_kg;`, `mass_main_total = sized.structural_masses.trem_principal_kg;`, `mass_nose = sized.structural_masses.trem_nariz_kg;` (os `item_mass(...)`/`expect` morrem).
7. **`constraint_checker.rs`** (fixture de teste): obter as massas via `MassModelAgent::run` na mesma sequência do orchestrator (ou preferencialmente construir a tupla a partir de `size_aircraft`, se o fixture permitir sem reescrita grande).
8. **`baseline_4seat.toml`**: remover os 5 `[[masses.items]]` estruturais, os 2 `mass_per_area_*` e `mass_main_leg_kg`, com comentário no lugar (padrão do comentário existente sobre emp_horizontal/emp_vertical) apontando `[mass_model]`.

- [ ] **Step 4: Rodar o baseline real e INVESTIGAR o resultado**

Run: `cargo run --release -- --engine config/engines/toyota_1gd_ftv.toml --aircraft config/aircraft/baseline_4seat.toml --mission config/missions/default.toml --out /tmp/e7_ciclo3.json`
Registrar no report: MTOW, OEW, as 7 massas, CG por cenário, status/violações dos 18 checks. Surpresa >5% vs a expectativa da spec (OEW ~888 kg, CG vazio avança) → investigar antes de prosseguir.

- [ ] **Step 5: Atualizar TODOS os pins quebrados (honestos: old→new comentado, tolerâncias iguais).** Se o envelope de CG (ou outro check) reprovar no baseline real: os testes de integração passam a asserar o **FAIL honesto com as violações nomeadas** (padrão do ciclo 2 pré-E7 — ver histórico de `tests/cli.rs`/`tests/gear_tipback.rs`), e a cobertura do caminho PASS fica nas configs sintéticas. A fixture sintética também muda de números — mesma regra.

- [ ] **Step 6: Suite completa + commit**

Run: `cargo test`
```bash
git add -A
git commit -m "feat(weight): OEW estrutural computado por agents::mass_model — itens fixos e mass_per_area removidos (migração)"
```

---

### Task 5: Schema 4.4 → 4.5 — bloco `structural_masses`, fidelity, CLI e doc

**Files:**
- Modify: `src/models/specs.rs` (`StructuralMassesSpec`, campo em `WeightSpec`, `SCHEMA_VERSION = "4.5"`)
- Modify: `src/agents/weight_balance.rs` (preenche o bloco)
- Modify: `src/main.rs` (fidelity.weight + tabela CLI)
- Modify: `docs/aircraft_spec.schema.md` (histórico 4.5 + linhas novas)
- Test: `tests/schema_v4.rs`

**Interfaces:**
- Consumes: `StructuralMasses` (Task 1) via `WeightBalanceAgent::run` (Task 4).
- Produces: JSON `weight.structural_masses` com as 7 massas + os 5 fatores de composto usados.

- [ ] **Step 1: Teste (RED)** em `tests/schema_v4.rs`: rodar o binário e asserar `schema_version == "4.5"` e a presença/positividade de `weight.structural_masses.asa_kg`, `.fuselagem_kg`, `.emp_h_kg`, `.emp_v_kg`, `.trem_principal_kg`, `.trem_nariz_kg`, `.tanques_kg`, `.composite_factor_wing` (padrão dos testes existentes do arquivo).

- [ ] **Step 2: Confirmar RED.**

- [ ] **Step 3: Implementar.**

`specs.rs`:

```rust
/// Massas estruturais computadas (ciclo 3, `agents::mass_model` — Raymer
/// 15.2 GA × fatores de composto Tab. 15.4) + os fatores usados
/// (rastreabilidade). Schema 4.5.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StructuralMassesSpec {
    pub asa_kg: f64,
    pub fuselagem_kg: f64,
    pub emp_h_kg: f64,
    pub emp_v_kg: f64,
    pub trem_principal_kg: f64,
    pub trem_nariz_kg: f64,
    pub tanques_kg: f64,
    pub composite_factor_wing: f64,
    pub composite_factor_tail: f64,
    pub composite_factor_fuselage: f64,
    pub composite_factor_gear: f64,
    pub composite_factor_fuel_system: f64,
}
```

+ `pub structural_masses: StructuralMassesSpec,` em `WeightSpec` (preenchido em `WeightBalanceAgent::run` a partir de `masses` + `cfg.mass_model`); `SCHEMA_VERSION` → `"4.5"` (docstring de versionamento ganha a entrada 4.5).

`main.rs`:
- `fidelity.weight` → `"semi-empirical (estruturas: Raymer 15.2 GA × fatores de composto Tab. 15.4; hardware: itens configurados não pesados — validar na balança)"`.
- Tabela nova no relatório CLI, após o bloco de peso existente:

```text
[ MASSAS ESTRUTURAIS ]  (Raymer 15.2 GA × composto)
  asa             149.7 kg  @ 3.95 m
  fuselagem       115.1 kg  @ 4.20 m
  ...
```
(componente, massa, braço — braços da mesma `ArmConfig` usada pelos itens; imprimir os 7.)

`docs/aircraft_spec.schema.md`: entrada 4.5 no histórico (o que entrou, o que mudou em `fidelity.weight`) + documentação das linhas novas do bloco `weight`.

- [ ] **Step 4: `cargo test` completo** (pins de schema_v4/cli que citem "4.4" atualizam honesto).

- [ ] **Step 5: Commit**

```bash
git add src/models/specs.rs src/agents/weight_balance.rs src/main.rs docs/aircraft_spec.schema.md tests/schema_v4.rs
git commit -m "feat(schema): v4.5 — weight.structural_masses + fidelity semi-empirical de estruturas"
```

---

### Task 6: Rodada final do baseline E7 + `aircraft_spec.json` + achado honesto

**Files:**
- Modify: `aircraft_spec.json` (regenerado)
- Create: report da task (workspace SDD) com a tabela comparativa

**Interfaces:**
- Consumes: tudo acima.

- [ ] **Step 1: Rodar o pipeline completo** gravando por cima do JSON commitado:

Run: `cargo run --release -- --engine config/engines/toyota_1gd_ftv.toml --aircraft config/aircraft/baseline_4seat.toml --mission config/missions/default.toml --out aircraft_spec.json`

- [ ] **Step 2: Tabela comparativa old→new no report** — E7 pré-ciclo-3 vs pós: MTOW, OEW, as 7 massas estruturais (TOML antigo vs computado), CG por cenário (%MAC), `validation_status`, e CADA violação nomeada com número (se houver). Conferir consistência com o run da Task 4.

- [ ] **Step 3: `cargo test` completo** (verde — os pins da Task 4/5 já refletem este estado).

- [ ] **Step 4: Commit**

```bash
git add aircraft_spec.json
git commit -m "feat(spec): aircraft_spec.json regenerado — OEW paramétrico (ciclo 3), schema 4.5"
```

O relatório do achado (violações ou PASS) é o entregável final do ciclo — o controlador o apresenta ao usuário; a decisão de campanha E8 é humana e fica FORA deste plano.
