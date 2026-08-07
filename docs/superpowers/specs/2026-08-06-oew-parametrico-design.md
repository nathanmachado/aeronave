# Ciclo 3 — OEW Paramétrico por Equações de Componente (Raymer) — Design

**Data:** 2026-08-06 · **Baseline de partida:** E7 (`da078cb`, PASS, 0 violações, schema 4.4)

## Problema

As 7 massas estruturais do OEW são dados fixos em `config/aircraft/baseline_4seat.toml` (asa 130 kg, fuselagem 160, trem principal 55, trem nariz 22, tanques 12) ou calibradas num único ponto (empenagens via `mass_per_area_*`, ciclo 2). Nenhuma responde a mudanças de geometria, MTOW, fator de carga ou capacidade de tanque — qualquer campanha futura que mexa nessas variáveis obtém resultados silenciosamente inconsistentes. `fidelity.weight` admite: *"soma de itens de massa configurados não pesados"*.

Diagnóstico (2026-08-06, scratchpad `diagnostico_massas.py`, Raymer 15.2 GA × fatores de composto Tab. 15.4, ponto E7): **total estrutural concorda em 0,4%** (420,4 vs 422 kg), mas a distribuição não — empenagens 1,9–2,5× conservadoras, trem principal 41% otimista (55 vs 92,7 kg), tanques 54%, fuselagem 39% conservadora. Erros opostos cancelam no OEW, mas **não cancelam no CG**: ~23 kg de excesso de palpite a ~4,8 m atrás do CG.

## Decisão de filosofia (do usuário, 2026-08-06)

**Equações puras.** O nível absoluto das massas estruturais vem da estatística da frota (Raymer × composto), não dos palpites do TOML. Consequências esperadas — OEW ~888 kg (−2 kg), CG vazio avança, envelope de CG provavelmente reprova — são **achado honesto do ciclo, não mascarado**. A campanha de refechamento (E8: bateria, x_main etc.) é decisão humana posterior, fora deste escopo.

Alternativas descartadas: calibração auditada no ponto E7 (preserva viés dos palpites no nível absoluto); estimador offline que escreve TOML (reintroduz a dessincronia que este ciclo mata).

## Arquitetura

### Módulo novo: `src/agents/mass_model.rs`

Uma responsabilidade: **geometria + cargas → massas estruturais**. Sem CG, sem envelope (isso é do `WeightBalanceAgent`).

```
pub struct StructuralMasses {
    pub asa_kg, fuselagem_kg, emp_h_kg, emp_v_kg,
    pub trem_principal_kg, trem_nariz_kg, tanques_kg: f64,
}
MassModelAgent::run(cfg, mtow_kg, n_design, /* geometria via cfg/spec */) -> StructuralMasses
```

Equações Raymer cap. 15.2 (GA): asa, EH, EV, fuselagem (sem termo de pressurização), trem principal, trem de nariz, sistema de combustível. Internamente em unidades imperiais (fidelidade à fonte, expoentes não-dimensionalizáveis), interface em SI; conversões nomeadas em constantes documentadas. Cada equação × fator de composto do config.

**Entradas e acoplamento** (escolhidas para minimizar dependências dentro da iteração):

| Entrada | Fonte | Justificativa |
|---|---|---|
| q de cruzeiro | `cruise_speed_min_kmh` do requisito + ISA na altitude de missão | estável (não depende do loop); expoentes de q são fracos (0,006–0,241) — erro ≤3% vs velocidade real; documentado |
| W_fw (combustível na asa) | capacidade × densidade | estável |
| MTOW | valor corrente da iteração do loop | acoplamento direto de ponto fixo |
| N_z = 1,5 × n_design | **lag-1** (iteração anterior; seed 1,5×3,8 = 5,70) | n_design vem do V-n, que depende de W/S da iteração — mesmo padrão do trim de cruzeiro (ciclo 2) |
| S_molhada fuselagem | `fuselage_wetted_coeff × π × d_fus_equiv × comprimento` | geometria derivada; coeficiente em config |
| Comprimentos de perna | config `[mass_model]` | dado geométrico com faixa |

### Config: nova seção `[mass_model]` (tudo com faixa validada + rejection test + valor distinto na fixture sintética)

| Campo | Valor | Faixa | Base |
|---|---|---|---|
| `composite_factor_wing` | 0.85 | [0.6, 1.1] | Raymer Tab. 15.4 |
| `composite_factor_tail` | 0.83 | [0.6, 1.1] | idem |
| `composite_factor_fuselage` | 0.90 | [0.6, 1.1] | idem |
| `composite_factor_gear` | 0.95 | [0.6, 1.1] | idem |
| `composite_factor_fuel_system` | 1.00 | [0.6, 1.2] | tanques integrais compostos ≈ metálicos |
| `d_fus_equiv_m` | 1.30 | [0.9, 2.0] | cabine 1,22 m + estrutura |
| `fuselage_wetted_coeff` | 0.75 | [0.5, 0.95] | corpo afilado vs cilindro pleno |
| `landing_load_factor_ult` | 4.5 | [3.0, 7.0] | N_l = N_pouso×1,5, Raymer |
| `main_strut_length_m` | 0.67 | [0.3, 1.5] | curso oleo E7 (212 mm) + roda, ≈26,3 in |
| `nose_strut_length_m` | 0.53 | [0.3, 1.5] | idem |

**Removidos com erro de migração claro** (padrão `sm_max`/ciclos anteriores):
- `[[masses.items]]` com `name` ∈ {asa, fuselagem, trem_principal, trem_nariz, tanques} — erro cita `[mass_model]`.
- `[empennage] mass_per_area_h_kg_m2` / `mass_per_area_v_kg_m2` (degrau intermediário do ciclo 2; viveu um ciclo).

**Braços de CG:** `WeightBalanceAgent` constrói os 7 itens computados com os mesmos `arm_ref`/`arm_offset` que os itens removidos usam hoje (mapeamento estático componente→braço em código, padrão que a empenagem já usa desde o ciclo 2; valores de braço continuam no config `[arms]` quando referenciados).

### Loop do orchestrator

`MassModelAgent` roda **antes** do `WeightBalanceAgent` em cada iteração. Ponto fixo: massas → OEW → MTOW → (V-n → n_design, lag-1) → massas. Teste de convergência no **campo real** do `SizedAircraft` (lição do fix-round do ciclo 2: nada de duplicar o corpo do loop em teste), com residual honesto pinado.

### Saída (schema 4.4 → 4.5)

- Bloco `weight` ganha `structural_masses`: as 7 massas computadas + os fatores de composto usados (rastreabilidade).
- `fidelity.weight`: *preliminary* → *"semi-empirical (estruturas: Raymer 15.2 GA × fatores de composto Tab. 15.4; hardware: itens configurados não pesados — validar na balança)"*.
- Print do CLI: tabela `[ MASSAS ESTRUTURAIS ]` (componente, massa, braço).
- `docs/aircraft_spec.schema.md`: histórico 4.5 + linhas novas.

## Tratamento de erros

- Config fora de faixa → erro de validação nomeando campo e faixa (padrão existente).
- Campos removidos presentes → erro de migração citando o substituto.
- Cenário/entrada inconsistente (MTOW ≤ 0, n_design fora do V-n) → panic com mensagem — invariantes internos, mesmo padrão do orchestrator.

## Testes (TDD)

1. **Hand-checks por equação** (RED primeiro): valores do diagnóstico recalculados como pins — asa 149,7 kg, fuselagem 115,1, EH 13,9, EV 6,3, trem principal 92,7, trem nariz 20,3, tanques 22,4 (±0,1; entradas do ponto E7 congeladas no teste).
2. **Propriedades estritas de direção:** ∂m_asa/∂S > 0, ∂m_asa/∂n_design > 0, ∂m_trem/∂MTOW > 0, ∂m_tanques/∂capacidade > 0; empenagem responde a `v_h` nos dois sentidos (substitui a property de `mass_per_area` do ciclo 2, que morre com o campo).
3. **Rejeição/migração:** cada campo novo fora de faixa; cada campo removido presente.
4. **Convergência:** loop completo converge; residual do acoplamento lag-1 pinado do campo real.
5. **Pins honestos** (old→new, tolerâncias iguais) em toda a suite afetada; se o baseline reprovar o envelope, os testes asseram o **FAIL honesto** com as violações nomeadas (padrão do ciclo 2 pré-E7), e a cobertura do caminho PASS fica em config sintética.
6. **Genericidade:** fatores/dados só em config; equações (física estatística publicada) em código — aceitação `src` sem nomes de motor continua verde.

## Fora de escopo

- Campanha E8 de refechamento do envelope (decisão humana posterior).
- Massa da asa acoplada ao dimensionamento real da longarina do `StructuralAgent` (fidelidade futura; a equação estatística é o degrau deste ciclo).
- Pressurização, cargas de flutter na massa, pesos de hardware (continuam itens de config).
