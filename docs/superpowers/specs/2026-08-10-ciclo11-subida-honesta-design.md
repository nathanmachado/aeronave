# Ciclo 11 — Subida Honesta (CS 23.65 a 1,2·Vs; Vy limpo) e Robustez do JSON — Design

**Data:** 2026-08-10 · **Baseline de partida:** E12 PASS (`20192b2`, schema 5.3, 466 testes, 0 violações, 0 flips)

## Problema

1. **Backlog item 2** — `agents::performance::best_climb_angle_ms` varre `[1,05·Vs_to, 1,80·Vs_to]`; como RC/V é monotonicamente decrescente na faixa (célula/motor reais), devolve o PISO da varredura. A CS 23.65 avalia o gradiente a velocidade de subida ≥ **1,2·Vs1** — avaliar a 1,05·Vs é ~1,45 p.p. OTIMISTA (13,900318% hoje vs ≈12,45% na referência correta).
2. **Backlog item 3** — `agents::performance::climb_rate_ms` (Vy/`rc_sl_ms`/`service_ceiling_m`) mistura estol FLAPADO (`wing.cl_max` = CL_max de pouso) com polar LIMPA (`cd0_extra = 0`) — híbrido fisicamente inconsistente para uma referência EN-ROUTE.
3. **Backlog item 5** — `takeoff_distance_50ft_m` devolve `+INFINITY` quando rc ≤ 0; `serde_json` converte para `null` silenciosamente em `to_50ft_paved_m`/`to_50ft_grass_m`, quebrando o round-trip (o precedente `fatigue_life_cycles` serializa `"infinita"` explicitamente).

## Design (aprovado pelo usuário, 2026-08-10)

### §1 — Gradiente CS 23.65 a 1,2·Vs

- `best_climb_angle_ms`: piso da varredura `1,05·Vs_to` → **`1,20·Vs_to`** (CS 23.65; teto 1,80·Vs_to inalterado). Docstring old→new (o "AINDA NÃO CORRIGIDO" do ciclo 8 morre aqui).
- Estimativas (verificar, não forçar): `climb_gradient_pct` 13,900318 → **≈12,45%** (±0,2 p.p.); `vx_kmh` sobe para ≈1,2·Vs_to (121,5 → ≈139 km/h, razão 1,2/1,05 ≈ 1,143 se o piso continuar sendo o argmax). Check de gradiente mínimo (8,3%) segue **PASS** com margem menor e honesta.
- Property nova (estrita na célula real): piso de varredura MAIOR ⟹ gradiente devolvido NÃO aumenta.

### §2 — Vy consistente (referência limpa)

- `climb_rate_ms`: referência de estol passa de `wing.cl_max` (flap CHEIO, 2,1) para **`wing.cl_max_clean`** (1,45) — estol limpo + polar limpa, configuração EN-ROUTE consistente. Faixa de varredura continua `[1,3·Vs, 1,8·Vs]`, agora sobre `Vs_clean`.
- Efeito: `Vs_clean/Vs_flap = √(2,1/1,45) ≈ 1,203` — o piso sobe ~20%. ALERTA de honestidade: o Vy atual (147,9 km/h) pode JÁ ser o piso da varredura antiga (RC monotônico decrescente); nesse caso Vy sobe ~20%, `rc_sl_ms` (4,9999) CAI e `service_ceiling_m` (5.200 m, resolução de 100 m da busca) pode cair. Gates: `RC_SL_MIN_MS = 1,5` (folga ampla), `SERVICE_CEILING_MIN_M` (verificar no run) — se algum flipar, é achado honesto, NÃO mascarar. Cascata: missão/autonomia consomem RC? — verificar consumidores e atualizar pins em cascata.
- Property nova (estrita): referência de estol limpa (CL_max menor) ⟹ Vy devolvido NÃO diminui (piso mais alto em curva RC decrescente ⟹ Vy sobe ou fica).

### §3 — `+INF` → `"infinita"` no JSON

- `to_50ft_paved_m`/`to_50ft_grass_m` (`PerformanceSpec`) ganham `#[serde(with = "fatigue_life_serde")]` — REUSO do módulo existente (`src/models/specs.rs`, string `"infinita"`, round-trip já testado para `fatigue_life_cycles`). Sem string nova, sem módulo novo (renomear o módulo para nome genérico é permitido se o implementador julgar mais limpo — decisão documentada).
- Teste de round-trip: fixture sintética com rc ≤ 0 (obstáculo inatingível) serializa `"infinita"` e desserializa de volta `f64::INFINITY`; baseline real (finito) serializa número normal — diff do JSON real: NENHUM.
- `docs/aircraft_spec.schema.md` §5: estender o precedente aos dois campos.

### §4 — Schema 5.3 → 5.4 + housekeeping

- Bump MINOR pelo padrão de exceção registrada (5.2/5.3): números de performance movem (§1/§2) + serialização condicional nova em dois campos (§3). Pins 5.3→5.4; histórico no schema doc.
- `docs/backlog.md`: item 7 → RESOLVIDO (fix wave do ciclo 10, commits `a7b561a`/`2d4fff7`/`a465e7b` — só faltava o registro); itens 2, 3, 5 → RESOLVIDO por este ciclo, com números old→new.

## Testes / Erros

- TDD RED-first; hand-checks congelados (§1: ≈12,45 ±0,2 p.p.; §2: razão de pisos ≈1,203 com literais); properties estritas (§1 e §2 acima); round-trip §3; pins honestos em cascata com tolerâncias INALTERADAS; genericidade verde (`cargo test` inteiro por task); investigar surpresa >5% vs estimativas; NUNCA mascarar — se um gate flipar, o veredito muda e o achado é documentado.

## Fora de escopo

- Item 4 (rolagem com arrasto — integração numérica) e termos de solo da rotação (μN·h_cg, D·(h_cg−h_D), `z_drag_above_cg_m`) — ciclo 12.
- Disco de hélice inclinado no #25 (conservador ≈+3,4 mm, permanece nomeado no backlog item 6).

## ERRATUM (2026-08-10, revisão da Task 2 — escalação ao principal)

O §2 prescrevia trocar a referência de estol de `climb_rate_ms` (`wing.cl_max` →
`wing.cl_max_clean`) tratando isso como a correção completa do híbrido. **Incompleto,
e o resultado isolado é ERRADO.** O CL_max NÃO entra no cálculo de RC(V) — só define
os limites da varredura `[1,3·Vs, 1,8·Vs]`, e essa heurística de 1,3–1,8 foi calibrada
contra o estol FLAPADO. Com `cl_max_clean` (Vs 20,3% maior), a janela desloca para
`[161,8; 224,0]` km/h enquanto o pico real de RC permanece em ≈148 km/h — **fora da
janela**. A função passa a devolver o PISO da janela, não o máximo: Vy 147,9→161,8 km/h,
`rc_sl_ms` 5,0010→4,9533, `service_ceiling_m` 5200→5100 m são ARTEFATOS DE BUSCA, não
física. Verificado por sondagem numérica direta na revisão (RC(V) é idêntica antes e
depois da troca de referência).

Correção que governa (a intenção do §2 — "Vy consistente" — permanece):
1. A referência de estol limpa está CERTA e fica (`cl_max_clean`).
2. A janela passa a `[1,05·Vs_clean, 2,00·Vs_clean]` com `steps` 50→100 — larga o
   bastante para conter o ótimo em vez de depender de uma heurística calibrada para
   outra referência. Vy é, por definição, o argmax de RC(V); a janela é ferramenta de
   busca, não modelagem.
3. **Guarda falseável obrigatória** (a lição do erro): teste que exige o argmax
   ESTRITAMENTE INTERIOR à janela no baseline real (`best_v > v_min && best_v < v_max`).
   Argmax na fronteira de uma busca por ótimo é defeito de modelo, não resultado.
   (Distinto de `best_climb_angle_ms`, onde avaliar em 1,2·Vs é PRESCRIÇÃO da CS 23.65,
   não busca por ótimo — lá a fronteira é legítima e a guarda não se aplica.)
4. Consequência esperada: Vy/RC/teto voltam a ≈148 km/h / ≈5,001 / 5200 m, e o efeito
   LÍQUIDO da Task 2 sobre os números fica ≈zero — que é o resultado honesto, porque Vy
   genuinamente não depende do CL_max. O entregável real da task passa a ser a
   consistência da referência + a guarda contra ótimo de fronteira.
