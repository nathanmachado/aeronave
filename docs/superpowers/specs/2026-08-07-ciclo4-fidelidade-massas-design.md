# Ciclo 4 — Fidelidade do Modelo de Massas: t/c da Empenagem, W_dg de Envelope e Robustez à Incerteza — Design

**Data:** 2026-08-07 · **Baseline de partida:** pós-ciclo-3 (`f209886`, FAIL honesto com 3 violações, schema 4.5)

## Problema

A campanha E8 (2026-08-07, 57 execuções em scratchpad) encontrou uma janela de PASS — `x_main` 3,58 m, `x_nose` 1,25 m, bateria +0,3 m — mas com margens de +0,25° (tipback), +0,21 pp (carga de nariz) e +0,23 pp MAC (rotação). Essas margens são da MESMA ordem que dois vieses conhecidos do modelo (revisão final do ciclo 3) e uma ordem de grandeza menores que a incerteza da estatística de frota das equações de Raymer (±10–20%). O modelo atual não tem como dizer isso: os 18 checks são binários contra pisos exatos, e um PASS por 0,007 pp (célula E8 `b0.4/n1.28`) sai idêntico a um PASS com 5 pp de folga.

Decisão de princípio do usuário (2026-08-07): *o foco é o modelo, não um projeto definitivo; se uma decisão é perigosa, o modelo deve ser ajustado para FALHAR no ponto de perigo* — não cabe a mim avisar em prosa o que o veredito deveria dizer sozinho.

Os três itens são refino natural do modelo (valem para qualquer configuração), não ajuste para o caso E8:

1. **t/c da empenagem**: as equações de massa usam o t/c da ASA (0,15) nas empenagens por falta do campo; empenagens reais usam ~0,09–0,12 e o expoente do EV é −0,49 → EV subestimado ~21%, EH ~5% (≈ +2,3 kg de cauda faltando, no braço mais sensível do CG).
2. **W_dg de envelope**: Raymer define W_dg como o peso máximo de projeto — no modelo, o MTOW de ENVELOPE (`wb.spec.mtow_kg`, 1.537 kg), não o de missão (1.506 kg) que a spec do ciclo 3 escolheu. `StructuralAgent`/`LandingGearAgent` já dimensionam para envelope; o modelo de massas está inconsistente com eles (~−3,5 kg dianteiros).
3. **Robustez**: nenhum check considera a incerteza das massas que o alimentam.

## Decisões (do usuário, 2026-08-07)

- **Veredito marginal = FAIL via check novo (#19).** `validation_status` continua binário; um check que só passa dentro da incerteza gera violação nomeada. (Alternativas descartadas: terceiro status `PASS_MARGINAL` — mudança de contrato para todo consumidor; warning — contradiz o princípio de reprovar no ponto de perigo.)
- **Propagação por pior-caso determinístico direcional** (abordagem A). Descartados: Monte Carlo (não-determinístico, quebra pins/CI, não melhora um gate binário) e margens fixas por check (folgas arbitrárias no config sem base física — o σ deriva a folga da causa real).

## Design

### §1 — t/c dedicado da empenagem

- Campo novo `[empennage].thickness_ratio` (adimensional): baseline **0,10** (perfis simétricos finos típicos de empenagem, NACA 0009–0012), faixa validada **(0,06, 0,18)**, fixture sintética **0,12** (distinta). Comentário no TOML cita a base.
- `MassModelAgent::run` passa `cfg.empennage.thickness_ratio` para `htail_mass_raymer_kg`/`vtail_mass_raymer_kg`; a asa continua com `cfg.wing.thickness_ratio`. As funções puras não mudam (t/c já é parâmetro).
- Docstring do módulo `mass_model.rs`: o bloco da aproximação (adicionado na fix wave do ciclo 3) vira nota histórica — "resolvido no ciclo 4 com campo dedicado".
- Campo novo obrigatório: TOMLs antigos falham no parse por campo ausente (serde) — mesmo comportamento de `[mass_model]` no ciclo 3; sem guarda de migração dedicada (não é remoção).
- Consequência esperada no baseline (verificar no run real, não forçar): EV +~21%, EH +~5% (cauda +~2,3 kg), CG vazio recua ligeiramente.

### §2 — W_dg = MTOW de envelope (lag-1)

- `MassModelAgent::run` passa a receber `w_dg_kg` = MTOW de ENVELOPE. Como `wb.spec.mtow_kg` desta iteração só existe DEPOIS do WB rodar, o orchestrator usa **lag-1** — terceiro uso do padrão (trim ciclo 2, n_design ciclo 3): `mtow_envelope_prev` seed = `cfg.sizing.mtow_initial_guess_kg` (seed simples; o envelope estabiliza em poucas iterações), atualizado após cada `WeightBalanceAgent::run` com `wb.spec.mtow_kg`.
- `W_l` (peso de pouso) permanece = W_dg (conservador, inalterado).
- A assinatura mantém a aridade; muda a semântica do parâmetro (renomear `mtow_kg` → `w_dg_envelope_kg` + docstring). O lag de `n_design` não muda.
- Teste de convergência: residual do lag re-medido no campo real e re-pinado honesto (agora são dois insumos com lag).
- Consequência esperada: massas +~1–2% (todas), levemente para frente (trem/asa dominam).

### §3 — Check #19: robustez à incerteza do modelo de massas

- Config: `[mass_model].sigma_mass_fraction` — baseline **0,15**, faixa **(0,05, 0,30)**, fixture **0,20**. Base: precisão típica ±10–20% de equações estatísticas de peso em projeto conceitual (Raymer cap. 15; Roskam Classe II).
- **Conjuntos adversariais (2, determinísticos):** classifica cada uma das 7 massas estruturais como dianteira/traseira comparando o braço do item com o CG VAZIO nominal:
  - *CG-mais-dianteiro:* dianteiras ×(1+σ), traseiras ×(1−σ);
  - *CG-mais-traseiro:* o inverso.
- Para cada conjunto: re-roda `WeightBalanceAgent::run` com as massas perturbadas (mesmos braços) e reavalia SOMENTE os checks sensíveis a massa: envelope de CG por cenário (6), carga de nariz máx/mín, tipback. Limites do envelope NÃO são recalculados (limite de rotação é invariante a peso — ciclo 2 — e `sm_min` define o traseiro pela geometria/NP, que não depende das massas): re-usa os limites do WB nominal, documentado. Tail-strike fica fora (só geometria de trem). Margem de combustível fica fora (efeito de 2ª ordem via MTOW; documentado como fora de escopo).
- **Veredito:** check que passa no nominal mas reprova sob qualquer conjunto adversarial → violação #19 nomeada: `"Robustez: <check> passa no nominal mas reprova com massas estruturais ±15% (pior caso <dianteiro|traseiro>): <valor perturbado> vs <limite>"`. Checks já reprovados no nominal não são duplicados.
- **Saída (schema 4.5 → 4.6):** bloco novo `robustness`: `sigma_mass_fraction` usado, faixa de CG dos dois casos perturbados (%MAC), lista dos checks que flipam (vazia quando robusto). `fidelity` ganha entrada `robustness`. `docs/aircraft_spec.schema.md`: histórico 4.6.
- Implementação: função dedicada em `validation/` (ou submódulo) chamada pelo `ConstraintChecker::verify`, recebendo o que já existe no pipeline (cfg, engine, req, state, wing, emp, masses nominais + wb nominal). Sem RNG, sem laço extra de convergência (perturbação avaliada no ponto convergido).

## Tratamento de erros

- Campos novos fora de faixa → erro de validação nomeando campo e faixa (padrão existente).
- σ que produza massa não-positiva (impossível com faixa ≤0,30) → invariante interno, panic com mensagem (padrão orchestrator).

## Testes (TDD)

1. **§1:** rejection tests do campo novo; hand-check do agente com t/c da empenagem ≠ asa (pins exatos congelados no plano, recalculados pelo método do diagnóstico do ciclo 3); property: EV cresce quando `thickness_ratio` da empenagem diminui (expoente negativo).
2. **§2:** teste do agente confirma W_dg = envelope lag-1 (relacional, contra `wb.spec.mtow_kg` da iteração anterior via campo real do `SizedAircraft`); residual re-pinado honesto.
3. **§3:** rejection test de `sigma_mass_fraction`; teste unitário da classificação dianteira/traseira e da construção dos 2 conjuntos; teste de veredito com config sintética construída para ser marginal (um check flipa) e outra robusta (nenhum flipa); σ=0 ≈ nominal (nenhum flip por construção).
4. **Pins honestos** em toda a suite (old→new, tolerâncias iguais); baseline esperado: FAIL continua, violações ≥3 (o #19 pode adicionar — o que o modelo disser).
5. **Validação do achado E8** (passo do ciclo, não teste de CI): rodar a célula E8 recomendada (`x_main` 3,58 / `x_nose` 1,25 / bateria +0,3) com o modelo novo — esperado que o #19 a reprove (margens < σ); se os §1–§2 mudarem os números a ponto de o nominal já reprovar (ou abrir margem real), reportar o que for.
6. **Genericidade:** aceitação (grep) verde.

## Sequência do ciclo e critério de conclusão

1. Implementação (plano SDD em worktree): §1 → §2 → §3 → schema 4.6.
2. Rodada do baseline com o modelo novo: `aircraft_spec.json` regenerado/commitado; relatório old→new.
3. Validação E8 (item 5 acima) e relatório do achado — o ciclo termina aqui, FAIL ou PASS, o que o modelo disser.
4. Decisão humana posterior: re-campanha E8 (ou outra alavanca) com o modelo refinado.

## Fora de escopo

- Re-campanha E8 completa (decisão humana após o ciclo).
- Demais itens do backlog do ciclo 3: check de `peak_w` do atuador (#20 futuro), pin de flutter, acoplamento da massa da asa à longarina real.
- Incerteza em grandezas não-massa (aerodinâmica, propulsão) — degrau futuro.
