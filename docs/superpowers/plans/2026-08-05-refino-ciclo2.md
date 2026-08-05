# Refino do Modelo — Ciclo 2 (pós-E6) — Plano

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development.

**Goal:** Eliminar as fragilidades expostas pela campanha E6: (1) autoridade do profundor vira resultado da geometria, não parâmetro livre; (2) massa/arrasto da empenagem escalam com a geometria; (3) tipback/tail-strike verificados; (4) carga de nariz nos dois extremos; (5) margem de combustível como requisito; (6) arrasto de trim em cruzeiro.

## Global Constraints

- SI; português; referências; parâmetros/dados só em config com faixas; nenhuma verificação vácua; pins honestos (old→new, tolerâncias iguais); achados novos NÃO mascarados (se um check novo reprovar o baseline, é resultado — reportar).
- `cargo test` verde ao final de CADA task; aircraft_spec.json regenerado/commitado por task; schema doc atualizado quando o JSON mudar.

---

### Task 1: Autoridade do profundor por geometria (DATCOM) + massa/arrasto da empenagem derivados

**Files:** Modify: src/agents/trim_authority.rs, src/agents/empennage.rs ou weight_balance.rs (itens de massa derivados), src/agents/aerodynamics.rs (cd0_emp derivado), src/models/{aircraft_config,config,specs}.rs, config/aircraft/baseline_4seat.toml, docs/aircraft_spec.schema.md, pins.

**1a — Autoridade calculada:**
- Config: REMOVER `[stability] cl_h_max_down` (erro de migração claro, padrão sm_max). NOVOS: `[control_surfaces] elevator_deflection_max_deg = 25.0` (faixa 10–35) e `[stability] cl_h_stall_limit = 1.10` (faixa 0.8–1.4; teto por stall da empenagem). Fixture sintética: 24.0 / 1.05.
- Física: eficácia de superfície τ(c_e/c) pelo ajuste `τ = 1.24·√(c_e/c) − 0.16` (curva de Nelson, Flight Stability fig. 2.21; documentar validade 0.1–0.6). `cl_h_max_down_calc = min(a_t·τ·δe_max_rad, cl_h_stall_limit)` com `a_t = lift_curve_slope(ar_h)`.
- Hand-check baseline E6 (c_e/c 0.40, AR_h 4.0 → a_t 3.8836, δ 25° = 0.43633 rad): τ = 1.24·0.63246−0.16 = 0.62425; cl = 3.8836·0.62425·0.43633 = **1.0578** (< teto 1.10) — ±0.01. Sanity: com a corda antiga 0.35 → τ 0.5735, cl 0.9720 ≈ o 0.95 que se assumia (o palpite da E6 era consistente).
- TrimSpec ganha: `cl_h_max_down_calc`, `tau_elevator`, `capped_by_stall: bool`; sensibilidade agora sobre ±2° de δe_max (recalcular limites) além de ±0.05 no cl.
- Consequência esperada: autoridade 0.95→1.058 (+11%) → limite de rotação avança (~10.95%→**~7–8% MAC**; calcular real) → envelope alarga; PASS mantido; margens de CG melhoram.

**1b — Massa e arrasto derivados:**
- Config: REMOVER itens `emp_horizontal`/`emp_vertical` de `[[masses.items]]` (erro de migração se presentes) e o campo `[empennage] cd0` (idem). NOVOS em `[empennage]`: `mass_per_area_h_kg_m2` e `mass_per_area_v_kg_m2` (faixa 4–20; CALIBRAR: 27.0/S_h e 16.0/S_v do runtime E6, ~8.6 e ~11.3 — comentar a base de calibração e a faixa típica de Raymer p/ caudas compostas) e `cd0_area_factor = ` (calibrar: cd0_emp_atual·S_w/(S_h+S_v) ≈ 0.0144 ≈ 2·Cf·FF — comentar; faixa 0.008–0.025).
- `oew_items` constrói os dois itens da empenagem de `EmpennageSpec × densidade` (braços: mesmos arm_refs de hoje); `cd0_total` usa `cd0_area_factor·(S_h+S_v)/S_w`.
- Calibração DEVE deixar o golden ~intacto (OEW 890.0 ±0.1, cd0 idem) — qualquer resíduo de arredondamento documentado nos pins. Testes: mudar v_h em config mutada → massa E arrasto acompanham (property, estrito nos dois sentidos).

### Task 2: Tipback/tail-strike + carga de nariz nos dois extremos

**Files:** Modify: src/agents/landing_gear.rs, config (novos campos [gear]), constraint_checker, specs (GearSpec + campos), main, pins.

- Config `[gear]`: `tipback_min_deg = 15.0` (faixa 8–25; Raymer cap. 11: 15° típico), `rotation_attitude_deg = 11.0` (faixa 5–18), `tail_cone_x_m = 7.80`, `tail_cone_height_m = 1.10` (geometria do cone de cauda; faixas sanas; fixture distinta).
- Tipback: `θ = atan((x_main − x_cg_aft_real)/h_cg)` com x_cg_aft_real = CG mais traseiro DOS CENÁRIOS (documentar escolha: envelope de carregamento real, não o limite admissível); violação se θ < tipback_min. Hand-check E6: x_cg aft ≈ 3.35 m (36.1% MAC), x_main 3.55, h_cg 1.05 → θ = atan(0.20/1.05) = **10.8° < 15° → VIOLAÇÃO ESPERADA** (achado honesto novo: rotação empurrou o trem p/ frente; tipback é o preço — tensão fundamental do triciclo; com a autoridade da Task 1 o trem PODE recuar um pouco: reportar no relatório a posição de trem que equilibra os dois, ex. varredura rápida informativa no print — SEM alterar o baseline).
- Tail-strike: ângulo disponível `atan((tail_cone_height − raio_pneu?)/(tail_cone_x − x_main))` — simplificar: folga angular = `atan(tail_cone_height_m/(tail_cone_x_m − x_main))` ≥ rotation_attitude_deg; documentar aproximação (altura já é do fundo do cone ao solo em atitude estática). Hand-check: atan(1.10/4.25) = 14.5° ≥ 11° ✓.
- Carga de nariz: avaliar em AMBOS extremos reais dos cenários: `nose_max` no CG mais dianteiro (vs teto 25%) e `nose_min` no mais traseiro (vs piso 8%). GearSpec: `nose_load_max_pct`, `nose_load_min_pct` (substituem o único; manter campo antigo como alias deprecado? NÃO — renomear com schema minor bump e nota). Hand-check E6: fwd CG 14.2% MAC = 3.077 m → nose = (3.55−3.077)/2.15 = 22.0% ≤ 25 ✓; aft 3.35 → 9.3%… conferir com o runtime (o 8.7% reportado usava outro CG — investigar e documentar qual).
- ConstraintChecker: 3 checks novos (tipback, tail-strike, nariz dois lados substitui o atual).

### Task 3: Margem mínima de combustível como requisito

**Files:** Modify: config/missions/default.toml (+ ferry), requirements.rs, config.rs, constraint_checker, main, pins, schema doc.

- `mission.toml`: `min_fuel_margin_fraction = 0.05` (faixa [0, 0.3]; fração da CAPACIDADE — PADRONIZAR a convenção: gate e campo `sizing.fuel_margin_pct` ambos %-da-capacidade; o teste que usa %-do-necessário ganha nota/ajuste).
- Violação se `fuel_margin < min`: baseline E6 tem 1.82% da capacidade → **VIOLAÇÃO ESPERADA (PASS→FAIL honesto)** — é o requisito funcionando; opções de projeto (tanque, missão) ficam para o humano; rotax_ferry: verificar e pinar.
- Nota: a Task 1 pode mexer nas margens (cd0 recalibrado ~igual) — usar os números pós-Task-2 reais.

### Task 4: Arrasto de trim em cruzeiro

**Files:** Modify: src/agents/trim_authority.rs (ou novo módulo de trim de cruzeiro), aerodynamics.rs (polar), orchestrator.rs (acoplamento no loop), specs (TrimSpec + campos), config ([empennage] e_h = 0.70, faixa 0.5–0.95), pins, schema doc.

- Física: em cruzeiro (sem flap), CL_h_trim resolve o balanço de momentos no CG de referência da missão (cenário "4 pax + bagagem + meia" — documentar escolha) com `cm_ac` (sem flap): `CL_h_trim = [cm_ac + CL_cruise·(x̄_cg − 0.25)]/[η_h·(S_h/S)·(l_h/MAC + 0.25 − x̄_cg)]`. Arrasto de trim: `ΔCD_trim = (CL_h_trim²/(π·ar_h·e_h))·(S_h/S_w)` somado ao polar de cruzeiro (documentar: contribuição de sustentação extra da asa desprezada, 2ª ordem).
- Acoplamento no loop do orchestrator: CL_h_trim depende do CG (WB) que vem DEPOIS da aero na iteração — usar o valor da iteração ANTERIOR (lag-1; inicial 0; converge com o loop — documentar; teste: valor final estável |Δ| < 1e-6 entre as duas últimas iterações).
- Hand-check E6 (x̄_cg meia ≈ 0.355, CL_cruise ≈ 0.36, cm_ac −0.008): num = −0.008+0.36·0.105 = 0.0298 → CL_h_trim ≈ 0.0298/[0.9·0.2207·(3.8514+0.25−0.355)] = 0.0298/0.7442 = **+0.0400** (upload! CG atrás do CA → cauda sustenta) → ΔCD = (0.0016/(π·4·0.7))·0.2207 = **4.0e-5** (~0.14% do CD — pequeno, honesto: reportar que no CG de meia-missão o trim é quase neutro; no CG dianteiro (solo 14.2% → x̄ 0.142) CL_h = [−0.008+0.36·(−0.108)]/0.7592 = −0.0620 → ΔCD ≈ 9.6e-5). TrimSpec: cl_h_trim_cruise, cd_trim, cg de referência. Consequência: fc/margens mudam ~0.1% — pins.
- S_h/S da E6: 0.2207 (3.134/14.2) — usar valores do runtime.

**Steps por task (TDD):** hand-checks como testes; propriedades estritas; rejection/migração; suite completa; cargo run; commit por task (`feat(trim): autoridade do profundor derivada da geometria DATCOM + empenagem paramétrica`, `feat(gear): tipback, tail-strike e carga de nariz nos dois extremos`, `feat(mission): margem mínima de combustível como requisito`, `feat(trim): arrasto de trim em cruzeiro no polar`).
