//! Teste de integração do binário: `aeronave --engine <path> --aircraft <path>
//! --mission <path> --out <path>`.
//!
//! Roda o binário compilado via `Command` (não chama funções internas) para
//! validar o comportamento real da CLI: qualquer combinação de motor/
//! aeronave/missão deve rodar sem recompilar, e erros de carregamento devem
//! sair com mensagem em português e código de saída != 0 (sem panic/
//! backtrace).

use std::path::PathBuf;
use std::process::Command;

fn bin() -> Command {
    Command::new(env!("CARGO_BIN_EXE_aeronave"))
}

fn manifest_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

/// Task 2.3, Step 1: trocar de motor via `--engine` sem recompilar deve
/// refletir no relatório gerado.
///
/// NOTA (Task 3.1): antes desta task, este teste checava sucesso (exit 0) e
/// o nome do motor no JSON final. Com `orchestrator::size_aircraft`
/// convergindo o MTOW honestamente (fecha o bug B5 — ver
/// `tests/generic_engine.rs`,
/// `orchestrator_baseline_rotax_tambem_revela_inviabilidade_maior`), o Rotax
/// 915iS — motor fraco demais para sustentar 280 km/h com esta célula —
/// agora falha por combustível insuficiente ANTES de gerar o JSON (que só é
/// escrito após o sizing convergir). O teste passa a verificar o que ainda é
/// verdade e é o ponto real da Task 2.3: que `--engine` troca qual motor é
/// usado (visível no cabeçalho impresso em stdout antes do erro) — não mais
/// que o pipeline completo com o Rotax "funciona" (ele nunca funcionou de
/// verdade contra o requisito de 280 km/h; o bug B5 só escondia isso do
/// código de saída).
#[test]
fn engine_flag_troca_motor_no_cabecalho() {
    let out_path = std::env::temp_dir().join(format!(
        "aeronave_cli_test_engine_flag_{}.json",
        std::process::id()
    ));

    let output = bin()
        .current_dir(manifest_dir())
        .arg("--engine")
        .arg("config/engines/rotax_915is.toml")
        .arg("--out")
        .arg(&out_path)
        .output()
        .expect("falha ao executar o binário aeronave");

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("Rotax 915 iS"),
        "stdout deveria conter 'Rotax 915 iS' (prova de que --engine trocou o motor \
         usado, mesmo que o sizing falhe depois — ver comentário acima):\n{stdout}");
    assert!(!stdout.contains("panicked") && !String::from_utf8_lossy(&output.stderr).contains("panicked"),
        "erro de sizing não deveria gerar panic");

    let _ = std::fs::remove_file(&out_path);
}

/// Motor inexistente: deve sair com código de erro e mensagem em português
/// no stderr — sem panic/backtrace do Rust.
#[test]
fn engine_inexistente_gera_erro_amigavel() {
    let out_path = std::env::temp_dir().join(format!(
        "aeronave_cli_test_engine_inexistente_{}.json",
        std::process::id()
    ));

    let output = bin()
        .current_dir(manifest_dir())
        .arg("--engine")
        .arg("nonexistent.toml")
        .arg("--out")
        .arg(&out_path)
        .output()
        .expect("falha ao executar o binário aeronave");

    assert!(!output.status.success(), "binário deveria sair com erro para motor inexistente");
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        stderr.contains("motor") || stderr.contains("Motor"),
        "stderr deveria conter uma mensagem em português sobre o motor: {stderr}"
    );
    assert!(!stderr.contains("panicked"), "erro de carregamento não deveria gerar panic: {stderr}");

    let _ = std::fs::remove_file(&out_path);
}

/// Sem argumentos, os defaults devem apontar para o motor Toyota (via
/// `config/engines/default.toml`, um symlink) — comportamento idêntico ao
/// hardcoded anterior, mas agora parametrizável.
///
/// NOTA (Task 3.1): antes desta task, este teste checava sucesso (exit 0) e
/// o nome do motor no JSON final, calculado a um MTOW fixo (o palpite
/// `sizing.mtow_initial_guess_kg`, nunca realimentado — bug B5). Ao fechar o
/// laço de convergência honestamente (`orchestrator::size_aircraft`), o
/// MTOW real da aeronave-base + Toyota converge para ~1.530 kg — e a essa
/// massa a aeronave-base precisa de 240,73 L de combustível para a missão
/// contra um tanque de 240,0 L (achado documentado em detalhe em
/// `tests/generic_engine.rs`,
/// `orchestrator_baseline_toyota_revela_tanque_insuficiente_apos_convergencia`).
/// O binário agora corretamente sai com erro (não mais silenciosamente com
/// um MTOW de projeto internamente inconsistente). Este teste passa a
/// verificar exatamente isso: falha limpa, mensagem em português, sem
/// panic — mesmo padrão dos demais testes de erro deste arquivo — em vez de
/// reafirmar um "sucesso" que dependia do bug que esta task corrigiu.
#[test]
fn sem_argumentos_falha_com_combustivel_insuficiente_apos_convergencia_de_mtow() {
    let out_path = std::env::temp_dir().join(format!(
        "aeronave_cli_test_default_{}.json",
        std::process::id()
    ));

    let output = bin()
        .current_dir(manifest_dir())
        .arg("--out")
        .arg(&out_path)
        .output()
        .expect("falha ao executar o binário aeronave");

    assert!(!output.status.success(),
        "binário deveria sair com erro: o MTOW convergido da aeronave-base padrão (Toyota) \
         exige mais combustível (240.73 L) do que a capacidade do tanque (240.0 L) — achado \
         da Task 3.1, ver tests/generic_engine.rs");

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("Toyota 1GD-FTV"),
        "stdout deveria conter 'Toyota 1GD-FTV' (o motor padrão, impresso antes do erro de \
         sizing):\n{stdout}");

    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(stderr.contains("combustível") || stderr.contains("Combustível"),
        "stderr deveria conter uma mensagem em português sobre o combustível insuficiente: \
         {stderr}");
    assert!(!stderr.contains("panicked"), "erro de sizing não deveria gerar panic: {stderr}");

    assert!(!out_path.exists(),
        "JSON de saída não deveria ser escrito quando o sizing falha antes dos demais agentes");
}
