#!/usr/bin/env bash

# Porteiro de verificação de todo ciclo de desenvolvimento
# Nenhum report de task é aceito sem a saída deste script

set -uo pipefail

# Descubra a raiz do repo a partir do diretório do script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && git rev-parse --show-toplevel 2>/dev/null || echo "$SCRIPT_DIR/..")
cd "$REPO_ROOT"

# Arquivos temporários
TMPFILE=$(mktemp)
TEST_OUTPUT_FILE=$(mktemp)
trap "rm -f $TMPFILE $TEST_OUTPUT_FILE" EXIT

# Contadores e flags para o resumo final
TESTES_PASSARAM=true
GENERICIDADE_PASSARAM=true

# ============================================================================
# SEÇÃO 1 — SUÍTE DE TESTES
# ============================================================================
echo
echo "SEÇÃO 1 — SUÍTE DE TESTES"
echo "─────────────────────────"

cargo test --release 2>&1 | tee "$TEST_OUTPUT_FILE"

# Extrai todas as linhas "test result:" de cada target
test_results=$(grep "test result:" "$TEST_OUTPUT_FILE")
total_passed=0

if [ -n "$test_results" ]; then
    echo
    echo "Resultados por target:"
    echo "$test_results" | while read line; do
        echo "  $line"
        # Extrai o número de testes passados
        if echo "$line" | grep -q "ok"; then
            passed=$(echo "$line" | grep -oE 'test result: ok\. [0-9]+ passed' | grep -oE '[0-9]+ passed' | grep -oE '[0-9]+')
            total_passed=$((total_passed + passed))
        else
            TESTES_PASSARAM=false
        fi
    done

    # Conta total de testes passados de uma forma mais confiável
    total_from_grep=$(echo "$test_results" | grep -oE '[0-9]+ passed' | awk '{sum+=$1} END {print sum}')
    if [ -z "$total_from_grep" ]; then
        total_from_grep=0
    fi
    echo
    echo "Total de testes que passaram: $total_from_grep"
else
    echo "AVISO: Nenhuma linha 'test result:' encontrada"
    TESTES_PASSARAM=false
fi

# Verifica se houve falhas
if grep -q "test result:.*FAILED" "$TEST_OUTPUT_FILE"; then
    echo
    echo "TESTES FALHADOS:"
    grep "FAILED" "$TEST_OUTPUT_FILE" || true
    TESTES_PASSARAM=false
fi

if [ "$TESTES_PASSARAM" = true ]; then
    RESULTADO_TESTES="APROVADA"
else
    RESULTADO_TESTES="REPROVADA"
fi

# ============================================================================
# SEÇÃO 2 — REGENERAÇÃO DO JSON
# ============================================================================
echo
echo "SEÇÃO 2 — REGENERAÇÃO DO JSON"
echo "──────────────────────────────"

cargo run --release -- \
    --engine config/engines/toyota_1gd_ftv.toml \
    --aircraft config/aircraft/baseline_4seat.toml \
    --mission config/missions/default.toml \
    --out "$TMPFILE" > /dev/null 2>&1

if [ -f "$TMPFILE" ]; then
    if diff -u aircraft_spec.json "$TMPFILE" > /dev/null 2>&1; then
        echo "Resultado: sem diferenças"
    else
        echo "Diferenças encontradas:"
        echo
        diff -u aircraft_spec.json "$TMPFILE" || true
    fi
else
    echo "Erro: Falha ao gerar JSON temporário"
fi

# ============================================================================
# SEÇÃO 3 — VEREDITO DO MODELO
# ============================================================================
echo
echo "SEÇÃO 3 — VEREDITO DO MODELO"
echo "─────────────────────────────"

python3 << 'PYTHON_SCRIPT'
import json

with open('aircraft_spec.json', 'r') as f:
    data = json.load(f)

schema_version = data.get('schema_version', 'AUSENTE')
validation_status = data.get('validation_status', 'AUSENTE')
violations = data.get('violations', [])
flips = data.get('robustness', {}).get('flips', [])

print(f"schema_version: {schema_version}")
print(f"validation_status: {validation_status}")
print(f"Contagem de violations: {len(violations)}")

if violations:
    print("Lista de violations:")
    for v in violations:
        print(f"  - {v}")

print(f"Contagem de robustness.flips: {len(flips)}")
if flips:
    print("Nomes de flips:")
    for flip in flips:
        if isinstance(flip, dict):
            print(f"  - {flip.get('nome', flip)}")
        else:
            print(f"  - {flip}")
PYTHON_SCRIPT

# ============================================================================
# SEÇÃO 4 — GENERICIDADE
# ============================================================================
echo
echo "SEÇÃO 4 — GENERICIDADE"
echo "──────────────────────"

resultado_grep=$(grep -rniE 'toyota|1gd|rotax|915is' src/ --include='*.rs' 2>/dev/null || true)

if [ -z "$resultado_grep" ]; then
    echo "Resultado: OK — nenhum nome de motor em src/"
    RESULTADO_GENERICIDADE="APROVADA"
else
    echo "REPROVADA — encontrados nomes de motor em src/:"
    echo
    echo "$resultado_grep"
    RESULTADO_GENERICIDADE="REPROVADA"
fi

# ============================================================================
# SEÇÃO 5 — NÚMEROS QUE CASCATEIAM
# ============================================================================
echo
echo "SEÇÃO 5 — NÚMEROS QUE CASCATEIAM"
echo "─────────────────────────────────"
echo

python3 << 'PYTHON_SCRIPT'
import json

with open('aircraft_spec.json', 'r') as f:
    data = json.load(f)

# Função auxiliar para extrair valores com fallback
def get_value(data, path, default='AUSENTE'):
    keys = path.split('.')
    value = data
    for key in keys:
        if isinstance(value, dict):
            value = value.get(key)
        else:
            return default
    return value if value is not None else default

# Mapeamento de campos: (rótulo, caminho no JSON)
fields = [
    ('mtow_kg', 'weight.mtow_kg'),
    ('oew_kg', 'weight.oew_kg'),
    ('cruise_speed_kmh', 'performance.v_cruise_kmh'),
    ('endurance_h', 'propulsion.endurance_h'),
    ('range_km', 'propulsion.range_km'),
    ('margem de combustível (%)', 'sizing.fuel_margin_pct'),
    ('climb_gradient_pct', 'performance.climb_gradient_pct'),
    ('vx_kmh', 'performance.vx_kmh'),
    ('vy_kmh', 'performance.vy_kmh'),
    ('rc_sl_ms', 'performance.rc_sl_ms'),
    ('service_ceiling_m', 'performance.service_ceiling_m'),
    ('to_50ft_paved_m', 'performance.to_50ft_paved_m'),
    ('to_50ft_grass_m', 'performance.to_50ft_grass_m'),
    ('ldg_50ft_m', 'performance.ldg_50ft_m'),
    ('prop_clearance_critical_m', 'propeller.prop_clearance_critical_m'),
]

# Calcula a largura máxima do rótulo para alinhamento
max_label_width = max(len(label) for label, _ in fields)

print(f"{'Campo':<{max_label_width}}  |  {'Valor':<20}")
print(f"{'-' * max_label_width}--+--{'-' * 20}")

for label, path in fields:
    value = get_value(data, path)

    if isinstance(value, str):
        formatted_value = value
    elif isinstance(value, (int, float)):
        if isinstance(value, float):
            formatted_value = f"{value:.6f}"
        else:
            formatted_value = str(value)
    else:
        formatted_value = str(value)

    print(f"{label:<{max_label_width}}  |  {formatted_value:<20}")
PYTHON_SCRIPT

# ============================================================================
# SEÇÃO 6 — RESUMO FINAL
# ============================================================================
echo
echo "SEÇÃO 6 — RESUMO FINAL"
echo "─────────────────────"
echo

echo "SEÇÃO 1 — SUÍTE DE TESTES:        $RESULTADO_TESTES"
echo "SEÇÃO 2 — REGENERAÇÃO DO JSON:   INFORMATIVA"
echo "SEÇÃO 3 — VEREDITO DO MODELO:    INFORMATIVA"
echo "SEÇÃO 4 — GENERICIDADE:          $RESULTADO_GENERICIDADE"
echo "SEÇÃO 5 — NÚMEROS:               INFORMATIVA"

echo

if [ "$RESULTADO_TESTES" = "APROVADA" ] && [ "$RESULTADO_GENERICIDADE" = "APROVADA" ]; then
    echo "Status geral: APROVADO"
    exit 0
else
    echo "Status geral: REPROVADO"
    exit 1
fi
