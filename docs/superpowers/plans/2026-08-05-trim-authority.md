# TrimAuthorityAgent — Limite Dianteiro de CG por Autoridade de Profundor — Plano

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development.

**Goal:** Substituir o proxy `sm_max` por um limite dianteiro de CG **físico**, calculado do balanço de momentos nas duas manobras críticas de nariz-para-cima: flare do pouso e rotação na decolagem.

**Architecture:** Novo agente `src/agents/trim_authority.rs` roda após WeightBalance/Empennage; produz `TrimSpec`; o envelope dianteiro em weight_balance passa a vir dele; `sm_max` é REMOVIDO da config (breaking → schema 4.1). Parâmetros semi-empíricos novos são dados de config com faixas documentadas.

## Global Constraints

- SI; português; fórmulas com referências (Raymer cap. 16 / Gudmundsson estabilidade e controle); dados/parâmetros empíricos só em config/*.toml.
- Nenhuma verificação vácua; pins honestos (old→new); requisitos intocados; violações reveladas ficam visíveis (não mascarar).
- `cargo test` verde ao final; aircraft_spec.json regenerado/commitado; schema doc atualizado (4.0→4.1: bloco `trim` adicionado, semântica de `cg_limit_fwd_pct_mac` agora derivada de controle; sm_max removido — documentar a migração).

---

### Task 1: TrimAuthorityAgent (flare + rotação)

**Files:**
- Create: `src/agents/trim_authority.rs` (+ mod)
- Modify: `src/models/aircraft_config.rs` + `src/models/config.rs` (novos campos + validação + remoção de sm_max), `config/aircraft/baseline_4seat.toml`, fixture sintética, `src/agents/weight_balance.rs` (envelope dianteiro vem do TrimSpec), `src/models/specs.rs` (+TrimSpec; SCHEMA_VERSION 4.1), `src/main.rs` (print + ordem dos agentes + fidelity "preliminary — sensível a cl_h_max_down/cl_ground; validar em ensaio"), `docs/aircraft_spec.schema.md`, pins.

**Config nova (dados, não código):**
```toml
[wing]
cm_ac = -0.008          # Cm_ac do perfil NACA 23015 (quase nulo — característica da série 230; Abbott & von Doenhoff)
cm_flap_delta = -0.30   # ΔCm de flap de pouso, semi-empírico (Raymer cap. 16; faixa típica −0.20 a −0.45)

[stability]
sm_min = 0.05           # (mantido — limite traseiro)
# sm_max REMOVIDO — limite dianteiro agora é físico (TrimAuthorityAgent)
cl_h_max_down = 0.85    # |CL| máximo de download da empenagem c/ profundor no batente (semi-empírico; faixa 0.5–1.2; Gudmundsson/Roskam)
trim_margin = 0.10      # fração da autoridade reservada como margem (efeito solo + certificação)
cl_ground_rotation = 0.5 # CL da asa na corrida de decolagem antes da rotação (α de solo; faixa 0–1)
to_flap_cm_fraction = 0.5 # fração de cm_flap_delta aplicável em flap de decolagem
```
Validação: cm_ac in (−0.15, 0.05); cm_flap_delta in (−0.6, 0.0); cl_h_max_down in (0.5, 1.2); trim_margin in [0, 0.3]; cl_ground in (0,1); fração in [0,1]; finitos; rejection tests; fixture sintética com valores levemente distintos (ex.: −0.010, −0.28, 0.80, 0.12, 0.55, 0.5).

**Física — flare (caso crítico de pouso), voo 1g a V_ref = 1.3·Vs0, flap de pouso:**
- CL de equilíbrio na flare independe do peso: `CL_flare = cl_max_flaps/1.69`.
- Balanço de momentos em torno do CG (adimensional, x̄ = posição em fração da MAC a partir do LE da MAC; x̄_ac = 0.25):
  `CL_h(x̄) = [cm_ac_total + CL_flare·(x̄ − 0.25)] / [η_h·(S_h/S_w)·(l_cg/MAC)]`
  com `cm_ac_total = cm_ac + cm_flap_delta` e `l_cg/MAC = l_h/MAC + 0.25 − x̄` (l_h = braço CA-asa→CA-emp).
- Autoridade disponível: `CL_h_avail = −cl_h_max_down·(1 − trim_margin)`.
- **Limite dianteiro de flare** = x̄ que resolve `CL_h(x̄) = CL_h_avail` (bissecção; CL_h_req fica mais negativo com x̄ menor).
- Hand-check baseline (MAC 1.2463, l_h 4.80 → l_h/MAC 3.8514, S_h/S 0.18173 = 2.5809/14.2, η_h 0.90, CL_flare 1.72/1.69 = 1.01775, cm_ac_total = −0.308, avail = −0.85·0.90 = −0.765):
  resolver −0.765·0.16356·(4.1014−x̄) = −0.308 − 0.25·1.01775 + 1.01775·x̄ → **x̄_flare ≈ 0.0551 (5,5% MAC) ±0.5%**.
  Sanity direcional: CL_h_req(16.6%) ≈ −0.611 (dentro da autoridade) — o limite antigo era conservador p/ flare.
- ATENÇÃO (documentar): resultado é SENSÍVEL a cl_h_max_down (0.80 → limite ~14% MAC; 0.90 → ~2% MAC). Reportar a sensibilidade no TrimSpec (limites recomputados a ±0.05 do parâmetro) e marcar fidelidade "preliminary".

**Física — rotação na decolagem (Vr = 1.1·Vs0, flap TO), momentos em torno do trem principal:**
- q_r = ½·ρ_SL·Vr²; download disponível da emp.: `F_h = q_r·S_h·η_h·cl_h_max_down·(1−trim_margin)` com braço `(x_ac_tail − x_main)`, x_ac_tail = x_mac_le + 0.25·MAC + l_h.
- Nariz-abaixo: peso `W·(x_main − x_cg)`; Cm de perfil+flap TO: `(cm_ac + to_flap_cm_fraction·cm_flap_delta)·q_r·S·MAC` (sinal nariz-abaixo);
- Nariz-acima: sustentação da asa na corrida `L_g = q_r·S·cl_ground_rotation` com braço `(x_main − x_ac_wing)`.
- **Limite de rotação**: maior x_cg... CUIDADO com a direção: CG mais dianteiro → momento de peso maior → mais difícil rotacionar; limite dianteiro = x_cg mínimo que ainda equilibra: `x_cg_rot = x_main − [F_h·(x_ac_tail−x_main) + L_g·(x_main−x_ac_wing) − |Cm_TO|·q_r·S·MAC] / W`. Depende do PESO do cenário → calcular por cenário (o agente recebe a lista de cenários do WB) e reportar o pior por cenário.
- Hand-check baseline (solo, W = 1193.4·9.807 N, Vs0 31.69 m/s, Vr 34.86, q_r ≈ 744.5 Pa, x_main 3.85, x_ac_wing 3.2116, x_ac_tail 8.0116):
  F_h = 744.5·2.5809·0.90·0.85·0.90 ≈ 1323 N; momento emp. ≈ 1323·4.1616 ≈ 5506 N·m; L_g = 744.5·14.2·0.5 ≈ 5286 N → +5286·0.6384 ≈ 3375 N·m; Cm_TO = −0.008−0.15 = −0.158 → |M| = 0.158·744.5·14.2·1.2463 ≈ 2082 N·m;
  x_cg_rot = 3.85 − (5506+3375−2082)/11703.6 = 3.85 − 0.5810 = 3.2690 m → **(3.2690−2.90)/1.2463 = 29.6% MAC ±1%** — VERIFICAR no código com os valores exatos do runtime; se divergir >2% investigar antes de pinar.
- CONSEQUÊNCIA ESPERADA (honesta): a rotação GOVERNA o limite dianteiro (≈29–30% MAC ≫ flare 5,5%) e é MAIS restritiva que o antigo proxy (16,6%) — vai reprovar mais cenários. Causa física: trem principal muito atrás do CG (x_main 3.85; carga no nariz já em 20–24%, perto do teto de 25%). Este é um ACHADO DO MODELO sobre o layout do trem — reportar com destaque; NÃO ajustar o trem (decisão de projeto é do humano).

**Envelope:** `cg_limit_fwd = max(x̄_flare, x̄_rot_do_cenário)` (o mais restritivo por cenário — rotação varia por peso); aft segue `NP − sm_min·MAC`. `inside_envelope` por cenário usa o limite do próprio cenário. TrimSpec (Serialize): flare_limit_pct_mac, rotation_limit_pct_mac_per_scenario (Vec com nome+limite), governing ("flare"|"rotacao"), cl_h_required_at_fwd_limit, cl_h_available, sensitivity (limites a cl_h_max_down ±0.05), parâmetros ecoados. JSON: `pub trim: Option<TrimSpec>`; SCHEMA_VERSION "4.1"; schema doc atualizado (novo bloco + migração sm_max).

**Steps (TDD):** testes unitários com os hand-checks acima (±0.5% flare, ±1% rotação); propriedade: cl_h_max_down↑ → limite de flare avança (x̄ menor, estrito); trem mais à frente (x_main↓) → limite de rotação avança (estrito); rejection tests dos novos campos + remoção de sm_max (config antiga com sm_max → erro claro de migração "sm_max foi substituído..."); re-avaliação dos cenários com pins honestos; suite completa; cargo run; commit `feat(trim): limite dianteiro de CG por autoridade de profundor (flare + rotacao)`.
