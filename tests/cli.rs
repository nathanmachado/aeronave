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
/// `orchestrator_baseline_rotax_ainda_inviavel_com_tanque_260l`), o Rotax
/// 915iS — motor fraco demais para sustentar 280 km/h com esta célula —
/// falha por combustível insuficiente (precisa de ~404 L, o tanque tem
/// 260 L — não é uma borda de alguns litros como o caso Toyota original,
/// é quase o dobro) ANTES de gerar o JSON (que só é escrito após o sizing
/// convergir). O teste verifica os dois pontos honestos: (1) `--engine`
/// trocou qual motor é usado (visível no cabeçalho impresso em stdout antes
/// do erro) — ponto real da Task 2.3; (2) o binário sai com erro e uma
/// mensagem em português sobre combustível — essa É a resposta correta do
/// modelo para o Rotax nesta célula, não um bug (ele nunca sustentou
/// 280 km/h de verdade; o bug B5 só escondia isso do código de saída).
#[test]
fn engine_flag_troca_motor_e_rotax_falha_honestamente_por_combustivel() {
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

    assert!(!output.status.success(),
        "binário deveria sair com erro: o Rotax 915iS precisa de ~404 L contra os 260 L do \
         tanque configurado — inviável por uma margem grande, não um caso de borda");

    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(stderr.contains("combustível") || stderr.contains("Combustível"),
        "stderr deveria conter uma mensagem em português sobre combustível insuficiente: \
         {stderr}");
    assert!(!stderr.contains("panicked"), "erro de sizing não deveria gerar panic: {stderr}");
    assert!(!out_path.exists(),
        "JSON de saída não deveria ser escrito quando o sizing falha antes dos demais agentes");

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
/// NOTA (Task 3.1): a primeira rodada desta task descobriu que, ao fechar o
/// laço de convergência honestamente (`orchestrator::size_aircraft`), o MTOW
/// real da aeronave-base + Toyota (~1.530 kg) exigia mais combustível
/// (240,73 L) do que o tanque original de 240,0 L — este teste chegou a
/// esperar falha (achado NEEDS_CONTEXT, ver `task-3.1-report.md`). O
/// controller decidiu a remediação de projeto: `fuel_system.capacity_l`
/// 240 → 260 L (`config/aircraft/baseline_4seat.toml`), dando ~8% de
/// margem. Com essa correção, o binário volta a rodar com sucesso sem
/// argumentos — o teste reverte à expectativa original (exit 0, JSON com o
/// motor padrão). A cobertura de regressão do achado original (tanque de
/// 240 L, mutação sintética) continua em
/// `tests/generic_engine.rs::orchestrator_toyota_240l_insuficiente_regressao_sintetica`.
#[test]
fn sem_argumentos_usa_motor_padrao_toyota() {
    let out_path = std::env::temp_dir().join(format!(
        "aeronave_cli_test_default_{}.json",
        std::process::id()
    ));

    let status = bin()
        .current_dir(manifest_dir())
        .arg("--out")
        .arg(&out_path)
        .status()
        .expect("falha ao executar o binário aeronave");

    assert!(status.success(),
        "binário deveria rodar com sucesso sem argumentos — MTOW convergido da aeronave-base \
         padrão (Toyota) deve caber no tanque de 260 L (ver comentário acima)");

    let json = std::fs::read_to_string(&out_path)
        .unwrap_or_else(|e| panic!("falha ao ler '{}': {e}", out_path.display()));
    assert!(json.contains("Toyota 1GD-FTV"), "JSON de saída deveria conter 'Toyota 1GD-FTV':\n{json}");

    let _ = std::fs::remove_file(&out_path);
}

/// Caminho feliz explícito (Task 3.1): roda o motor padrão via `--engine`
/// explícito (não apenas o default do `clap`) com `--out` apontando para um
/// arquivo temporário, e confirma que o sizing converge e o JSON é gerado
/// com sucesso — uma rede de segurança independente do teste "sem
/// argumentos" acima, para o caso de algum default do `clap` mudar sem que
/// o pipeline de sizing em si esteja quebrado.
#[test]
fn engine_padrao_explicito_com_out_tempfile_converge_com_sucesso() {
    let out_path = std::env::temp_dir().join(format!(
        "aeronave_cli_test_engine_padrao_explicito_{}.json",
        std::process::id()
    ));

    let output = bin()
        .current_dir(manifest_dir())
        .arg("--engine")
        .arg("config/engines/toyota_1gd_ftv.toml")
        .arg("--aircraft")
        .arg("config/aircraft/baseline_4seat.toml")
        .arg("--mission")
        .arg("config/missions/default.toml")
        .arg("--out")
        .arg(&out_path)
        .output()
        .expect("falha ao executar o binário aeronave");

    assert!(output.status.success(),
        "binário deveria convergir e sair com sucesso com o motor/aeronave/missão reais \
         passados explicitamente: stderr={}",
        String::from_utf8_lossy(&output.stderr));

    let json = std::fs::read_to_string(&out_path)
        .unwrap_or_else(|e| panic!("falha ao ler '{}': {e}", out_path.display()));
    assert!(json.contains("Toyota 1GD-FTV"), "JSON de saída deveria conter 'Toyota 1GD-FTV':\n{json}");
    assert!(json.contains("\"validation_status\": \"PASS\""),
        "JSON de saída deveria reportar validation_status PASS:\n{json}");

    let _ = std::fs::remove_file(&out_path);
}
