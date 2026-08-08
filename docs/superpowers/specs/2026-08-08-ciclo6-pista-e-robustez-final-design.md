# Ciclo 6 — Requisito de Pista, Robustez Completa do Massa-Total e Refactor do Verify — Design

**Data:** 2026-08-08 · **Baseline de partida:** pós-ciclo-5 (`45e0846`, FAIL honesto com 3 violações nominais, schema 4.7, 22 checks)

## Problema

1. **Pista de grama é premissa fundadora do projeto sem requisito no modelo.** O `PerformanceAgent` computa as distâncias sobre obstáculo de 15 m desde o Task 4.7 (`to_50ft_paved_m` 381 m, `to_50ft_grass_m` 428 m, `ldg_50ft_m` 540 m no baseline) — mas nenhum check as compara a nada. Consequência prática: a alternativa "hélice ≤ 1,78 m" do veredito E9 (ciclo 5) não tem gate para ser julgada — o custo de tração de um diâmetro menor aparece nas distâncias, e as distâncias não reprovam nunca.
2. **O caso massa-total do #19 descarta o `sized_p.wb`** (recomendação da revisão final do ciclo 5): no mundo +σ re-convergido, OEW e MTOW mudam — envelope de CG, carga de nariz e tipback mereciam reavaliação com dados que já estão computados.
3. **`ConstraintChecker::verify` chegou a 15 parâmetros posicionais** — backlog de dois ciclos.

Decisão de requisito (do usuário, 2026-08-08): **pista disponível = 600 m** (grama/terra) — deliberadamente apertada; se o modelo reprovar algo, é achado.

## Design

### §1 — Requisito de pista + checks #23/#24 + sensibilidade ao diâmetro

- **Config de missão:** `runway_available_m = 600.0` em `config/missions/*.toml` / `Requirements` — faixa **(300, 2000)**, fixture sintética distinta (700.0), comentário citando a premissa de pista de fazenda. Campo obrigatório (missões antigas falham no parse por campo ausente — mesmo padrão dos ciclos anteriores; `rotax_ferry.toml` também ganha o campo).
- **Check #23:** `to_50ft_grass_m ≤ runway_available_m` (decolagem na GRAMA sobre 15 m — o caso dimensionante da premissa; a pavimentada é informativa). **Check #24:** `ldg_50ft_m ≤ runway_available_m`. Violações nomeando distância, pista e superfície. Baseline esperado: ambos PASSAM (428/540 vs 600 — pouso com folga de 10%, o mais apertado).
- **Gates massa-total (#19):** decolagem grama 50 ft e pouso 50 ft entram na lista de reavaliações do mundo +σ (distâncias crescem com MTOW; flip se nominal passa e perturbado cruza a pista). Via `PerformanceAgent` do mundo perturbado, que já é rodado.
- **Property tests de sensibilidade ao diâmetro** (a garantia que o veredito da hélice menor precisa): com tudo o mais fixo, `prop_diameter` menor ⟹ tração estática menor (`static_thrust_ideal_n`) ⟹ `takeoff_distance_50ft_m` maior. Dois testes de direção estrita no `performance.rs` (não existem hoje; a cadeia está ligada mas nenhum teste a protege).
- **Fora de escopo:** refinar a curva η(J) (fator de atividade, nº de pás) — o efeito de 1ª ordem do diâmetro está capturado por disco atuador + J = V/(nD), documentado; fidelidade de hélice é ciclo futuro.

### §2 — Envelope/nariz/tipback no mundo massa-total

- No caso massa-total do `RobustnessAgent` (braço `Ok(sized_p)`): usar `sized_p.wb` (hoje descartado) — cenários `cg_pct_mac` vs limites NOMINAIS (invariantes a massa, racional do ciclo 4 documentado); `LandingGearAgent::run` com MTOW de envelope perturbado e extremos de CG perturbados → tipback e carga de nariz máx/mín vs pisos/tetos; flips caso `"massa-total"` gated em "passa no nominal". Mesma mecânica dos casos direcionais — sem código novo de avaliação, só a aplicação ao terceiro mundo.
- Expectativa no baseline: sem flips novos (massa +σ é quase simétrica no CG; os casos direcionais continuam sendo o pior caso de CG) — verificar, não forçar.

### §3 — Refactor `verify` → struct de parâmetros

- `pub struct VerifyInputs<'a>` em `constraint_checker.rs` agrupando os parâmetros atuais (req, wing, prop, mtow_kg, engine, wb, propeller, perf, mission, electrical, gear, gear_cfg, fuel_capacity_l, robustness); `verify(inputs: &VerifyInputs)` (ou consumir por valor de refs — escolha do plano, documentada). Mudança MECÂNICA: zero comportamento, zero mensagens; todos os ~28 call sites migrados. Motivação registrada: 3 ciclos seguidos adicionaram parâmetros; a struct permite crescer sem tocar em 28 lugares.

### Schema 4.7 → 4.8

- Sem campos novos no JSON de saída (as distâncias 50 ft já existem); o bump documenta os checks #23/#24 (mudança de contrato do veredito: `violations` pode conter os novos textos) e o requisito novo de missão. Histórico no `docs/aircraft_spec.schema.md` + `SCHEMA_VERSION`.

## Tratamento de erros

- `runway_available_m` fora de faixa → erro de validação padrão; ausente → erro de parse de missão (campo obrigatório).
- Sem migrações (nenhum campo removido).

## Testes (TDD)

1. **§1:** rejection test da faixa; violação isolada #23 e #24 com config sintética mutada (pista curta artificial); baseline real assere o resultado honesto (esperado: sem violação nova); property tests de direção do diâmetro (2); flip massa-total de decolagem com fixture marginal sintética (pista apertada logo acima da distância nominal).
2. **§2:** flip de cenário de CG / carga de nariz no caso massa-total com fixture construída (σ alto na fixture clonada + limites apertados); baseline sem flip novo.
3. **§3:** compilação + suite inteira verde SEM nenhuma mudança de mensagem/pin (refactor puro); nenhum teste novo além da migração dos existentes.
4. Pins honestos; genericidade verde.

## Sequência do ciclo e critério de conclusão

1. Implementação (plano SDD em worktree): §3 (refactor primeiro — diff mecânico isolado antes das mudanças de comportamento) → §1 → §2 → schema 4.8.
2. Rodada do baseline: `aircraft_spec.json` regenerado; esperado FAIL com as MESMAS 3 violações (checks #23/#24 passam com 428/540 vs 600) — o que o modelo disser.
3. **Avaliação da hélice 1,78 m** (a pergunta que motivou o ciclo): rodar a célula E9 (bateria 53/x_nose 1,30/h_cg 0,92/pernas curtas) COM `diameter_m` fixado em 1,78 no TOML de teste — registrar folga de hélice (esperado: 1,12−0,89 = 0,23 m — passa por construção no limite exato; conferir contra `ground_clearance_min_m` com o ≥), distâncias #23/#24, Mach de ponta, robustez completa. Se 1,78 cravar exatamente no limite da folga, testar também 1,76 m (folga 0,24) — o relatório apresenta o quadro para a decisão E10.
4. Relatório do ciclo. A correção do projeto (adoção E9/E10) é o passo seguinte, por decisão do usuário já registrada.

## Fora de escopo

- Adoção de configuração (E9/E10) — vem imediatamente após, com o quadro deste ciclo.
- Fidelidade da curva η(J) da hélice; efeito de solo na decolagem; vento.
- Robustez direcional (±σ CG) nas distâncias de pista (efeito de CG em decolagem/pouso é 2ª ordem no modelo atual — não modelado; documentado).
