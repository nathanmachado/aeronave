# Refinamento do Ponto Neutro — Downwash + Fuselagem (Multhopp) — Plano

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Corrigir o viés do modelo de ponto neutro (NP hoje a 72,5% MAC, irrealisticamente traseiro) adicionando os dois efeitos omitidos — downwash na empenagem e contribuição desestabilizadora da fuselagem — para que o envelope de CG admissível seja fisicamente crível.

**Architecture:** Ambas as correções entram em `neutral_point_m` (src/agents/weight_balance.rs), alimentadas por dados já existentes (AR da asa, geometria da fuselagem) + um fator empírico configurável. Nenhuma mudança de schema além dos valores.

## Global Constraints

- SI; português; fórmulas com referências; dados de projeto só em config/*.toml (fatores empíricos de física com faixa documentada podem ter default, mas o valor usado vem do TOML).
- Nenhuma verificação vácua; pins atualizados honestamente (old→new documentado); requisitos intocados.
- `cargo test` verde ao final; aircraft_spec.json regenerado e commitado.

---

### Task 1: Downwash + Multhopp no ponto neutro

**Files:**
- Modify: `src/agents/weight_balance.rs` (neutral_point_m + nova fn downwash_gradient + nova fn fuselage_np_shift_mac)
- Modify: `src/models/aircraft_config.rs` + `src/models/config.rs` (campo `[stability] fuselage_kf` com validação) + `config/aircraft/baseline_4seat.toml` + fixture sintética
- Modify: pins afetados (tests/generic_engine.rs, tests/cli.rs, unit tests de weight_balance/constraint_checker) + aircraft_spec.json
- Test: unit hand-checks + property tests

**Fórmulas (documentar em português com referência):**

1. **Gradiente de downwash** (aproximação clássica de asa elíptica, Raymer cap. 16):
   `dε/dα = 2·a_w/(π·AR_w)` com `a_w = lift_curve_slope(AR_w)` [1/rad].
   A contribuição da empenagem no NP é multiplicada por `(1 − dε/dα)`.
   Hand-check baseline: a_w = 5.1554, AR = 10.0394 → dε/dα = 2·5.1554/(π·10.0394) = 0.3268; (1−dε/dα) = 0.6732.

2. **Contribuição da fuselagem** (Multhopp simplificado via Raymer eq. 16.25):
   `Cm_α_fus = K_f·W_f²·L_f/(MAC·S_w)` [1/grau, com K_f da fig. 16.14 de Raymer]
   `Δx_np_fus/MAC = −Cm_α_fus·(180/π)/a_w`  (converter 1/grau → 1/rad antes de dividir por a_w [1/rad])
   Config: `[stability] fuselage_kf = 0.02` (faixa típica 0.01–0.03 conforme posição da asa na fuselagem — Raymer fig. 16.14; validação em (0.005, 0.05) + finite + rejection test; fixture sintética 0.018).
   Hand-check baseline: W_f = 1.22 (cabin_width_m), L_f = 8.2, MAC = 1.2463, S_w = 14.2 →
   Cm_α_fus = 0.02·1.4884·8.2/(1.2463·14.2) = 0.24410/17.6975 = 0.013793 /grau = 0.79027 /rad
   Δx_np_fus = −0.79027/5.1554 = −0.15329 MAC (NP avança 15,3% MAC).

3. **NP corrigido:**
   `x_np = x_ac_wing + delta_stab·(1−dε/dα) + Δx_np_fus·MAC`
   Baseline: delta_stab era 0.47456·MAC → ×0.6732 = 0.31947·MAC; NP%MAC = 0.25 + 0.31947 − 0.15329 = 0.41618 → x_np = 2.90 + 0.41618·1.2463 = 3.4187 m (≈ 41,6% MAC — na faixa realista 35–50% de aeronaves leves convencionais).

**Consequências esperadas (honestas):**
- Envelope de CG: dianteiro = 41,6−25 = 16,6% MAC; traseiro = 41,6−5 = 36,6% MAC.
- Cenários (1,7–29,4% MAC): os traseiros (4 pax: ~25–29,4%) devem PASSAR; os dianteiros (solo ~1,7%, 2 pax dianteiros ~11%) continuam à frente do limite → violação residual honesta (menor que antes — antes eram 6/6 fora; esperar ~3-4 fora). Reportar a tabela por cenário.
- SM min (cenário mais traseiro 29,4%): 41,6−29,4 = 12,2% — dentro de [5%, 25%] ✓.
- validation_status: continua FAIL enquanto houver cenário fora (honesto).

**Steps (TDD):**
- [ ] Testes unitários que falham: `downwash_gradient(5.1554, 10.0394) ≈ 0.3268 ±0.001`; `fuselage_np_shift_mac` hand-check ≈ −0.15329 ±0.001; NP integrado ≈ 3.4187 m ±0.005; property: aumentar fuselage_kf move NP para FRENTE (estrito); aumentar AR da asa reduz dε/dα (estrito).
- [ ] Rejection test para fuselage_kf fora de faixa/NaN.
- [ ] Implementar; propagar (WeightBalanceAgent já recebe cfg? verificar como neutral_point_m obtém os dados — a fn ganha parâmetros explícitos; o Agent os extrai de cfg/emp/wing).
- [ ] Atualizar pins (old→new em comentário) incl. envelope %MAC, SM por cenário, contagem de violações do checker, aircraft_spec.json.
- [ ] `cargo test` completo; `cargo run` (JSON regenerado, commit).
- [ ] Commit: `feat(stability): NP com downwash e fuselagem (Multhopp) — fidelidade do envelope de CG`
