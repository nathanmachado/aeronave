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
/// falha por combustível insuficiente (precisa de ~393,3 L, o tanque tem
/// 260 L — não é uma borda de alguns litros como o caso Toyota original,
/// é ~1,51×; valor pinado em
/// `tests/generic_engine.rs::orchestrator_baseline_rotax_ainda_inviavel_com_tanque_260l`)
/// ANTES de gerar o JSON (que só é escrito após o sizing
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
        "binário deveria sair com erro: o Rotax 915iS precisa de ~401,8 L (pós campanha E1–E6, \
         2026-08-05 — era ~393,3 L antes) contra os 260 L do tanque configurado — inviável por \
         uma margem grande, não um caso de borda");

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
/// real da aeronave-base + Toyota (~1.529,9 kg) exigia mais combustível
/// (243,92 L no ponto convergido) do que o tanque original de 240,0 L —
/// este teste chegou a esperar falha (achado NEEDS_CONTEXT, ver
/// `task-3.1-report.md`). O controller decidiu a remediação de projeto:
/// `fuel_system.capacity_l` 240 → 260 L
/// (`config/aircraft/baseline_4seat.toml`), dando 16,08 L (~6,6%) de
/// margem. Com essa correção, o binário volta a rodar com sucesso sem
/// argumentos — o teste reverte à expectativa original (exit 0, JSON com o
/// motor padrão). A cobertura de regressão do achado original (tanque de
/// 240 L, mutação sintética) continua em
/// `tests/generic_engine.rs::orchestrator_toyota_240l_insuficiente_regressao_sintetica`.
/// (Nota da revisão: o número originalmente reportado aqui, 240,73 L / 0,3%,
/// era um transiente de a checagem de aceite rodar a cada iteração —
/// corrigido para o valor no ponto convergido, 243,92 L / 1,6%.)
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
/// — uma rede de segurança independente do teste "sem argumentos" acima,
/// para o caso de algum default do `clap` mudar sem que o pipeline de
/// sizing em si esteja quebrado.
///
/// NOTA (Task 4.4 — achado honesto de projeto, NÃO um bug deste código): com
/// o envelope de CG ADMISSÍVEL (limite traseiro de `[stability].sm_min`,
/// não mais apenas `SM > 0.03` isolado), a aeronave-base real ficava (até a
/// campanha E1–E6, ver abaixo) com `validation_status: FAIL`.
///
/// ATUALIZAÇÃO (task de downwash + fuselagem/Multhopp): o ponto neutro
/// agora conta com o downwash na empenagem (dε/dα≈0.327) e a contribuição
/// desestabilizadora da fuselagem (Multhopp simplificado, `fuselage_kf`),
/// além do modelo de área de cauda de Raymer (Task 4.1). NP≈3,4187m
/// (≈41,6% MAC, MAC≈1,2463m) — bem mais à frente que os ~3,803m
/// (≈72,5% MAC) do modelo anterior, que ignorava downwash e fuselagem.
///
/// ATUALIZAÇÃO (task trim-authority): o limite DIANTEIRO deixou de ser o
/// proxy `sm_max` (16,6% MAC) e passou a ser calculado fisicamente pelo
/// `TrimAuthorityAgent` a partir da autoridade de profundor em flare
/// (≈5,5% MAC — GENEROSO, o Cm_ac quase nulo do NACA 230015 ajuda) e
/// rotação de decolagem (≈29,6%–40,2% MAC conforme o peso do cenário —
/// GOVERNA, mais restritiva). Achado honesto: a ROTAÇÃO governa em TODOS os
/// cenários, e o limite resultante (pior caso ≈40,2% MAC) fica À FRENTE de
/// TODOS os 6 cenários reais (CG observado ~1,7%–29,4% MAC) — os 6/6
/// cenários ficam FORA do envelope (eram 2/6 fora com o proxy `sm_max`
/// antigo). Causa física: o trem principal (`[gear].x_main_m=3,85m`) fica
/// muito atrás do CG desta célula (carga de nariz já em 20–24%, perto do
/// teto de 25% da Task 4.5) — o braço de peso em torno do trem na rotação é
/// grande demais para a autoridade de profundor disponível. Isso é uma
/// descoberta de engenharia real sobre o layout atual do trem (não um erro
/// de implementação) — decisão de projeto para revisão humana futura: mover
/// o trem principal mais para a frente (`gear.x_main_m`), revisar os braços
/// de `[arms]`/CG, ou aumentar a autoridade de profundor (`cl_h_max_down`,
/// maior EH). O binário ainda sai com código 0 (não há `process::exit`
/// condicionado a `validation_status`) — apenas o conteúdo do relatório
/// refletia a falha, honestamente.
///
/// ATUALIZAÇÃO (campanha E1–E6, 2026-08-05): a decisão de projeto pedida na
/// nota acima foi tomada — `gear.x_main_m` 3,85→3,55m (trem recuado),
/// `empennage.v_h` 0,70→0,85 (EH maior, mais autoridade de profundor E mais
/// estabilizador) e `stability.cl_h_max_down` 0,85→0,95 (mais download do
/// profundor), validados experimentalmente (envelope [10,9%, 43,5%] MAC,
/// 6/6 cenários dentro). A aeronave-base real convergia então com
/// `validation_status: PASS` e ZERO violações de envelope de CG — primeiro
/// PASS honesto do projeto. O caminho de erro (envelope vazio/cenário fora
/// do envelope) continua coberto por testes unitários com config mutada em
/// código (ver `src/validation/constraint_checker.rs::tests::violacao_de_
/// envelope_vazio_aparece_com_baseline_mutado_parametros_pre_e6` e
/// `violacao_de_envelope_aparece_quando_cenario_esta_fora`).
///
/// ATUALIZAÇÃO (Task 2, refino-ciclo2, 2026-08-06): NOVO achado honesto —
/// exatamente o preço antecipado pela nota da campanha E1–E6 acima ("mover
/// o trem principal mais para a frente" para abrir o envelope de CG via
/// autoridade de rotação). Recuar `gear.x_main_m` de 3,85 para 3,55m
/// reduziu a distância horizontal ao CG mais TRASEIRO real dos cenários de
/// carga, e portanto o ângulo de TIPBACK (Raymer cap. 11): θ ≈ 10,1° (CG
/// aft real ≈3,363m/37,2% MAC, x_main=3,55m, h_cg=1,05m) — ABAIXO do piso
/// de 15° (`[gear].tipback_min_deg`, ver checagem nova #15 de
/// `ConstraintChecker::verify`). `validation_status` volta a `FAIL` — não
/// é uma regressão desta task, é a checagem NOVA pegando uma tensão física
/// real do triciclo que já existia, mas nunca tinha sido verificada. O
/// envelope de CG continua FECHADO (zero violações de cenário/vazio) e a
/// checagem de tail-strike/carga de nariz nos dois extremos (também novas
/// desta task) PASSAM — só o tipback falha.
///
/// ATUALIZAÇÃO (Task 3, refino-ciclo2, 2026-08-06): SEGUNDO achado honesto
/// NOVO, independente do tipback — a margem de combustível
/// (`config/missions/default.toml`, `min_fuel_margin_fraction = 0.05`,
/// checagem nova #18 de `ConstraintChecker::verify`) também falha: a
/// missão exige ≈255,3 L contra 260 L de capacidade, margem ≈1,82% da
/// capacidade — abaixo do piso de 5%. `validation_status` continua `FAIL`
/// (já era, por causa do tipback) — agora por DOIS motivos independentes.
/// Não mascarado tunando o tanque/missão; a decisão de projeto (tanque
/// maior, missão menor, ou aceitar o risco) fica para revisão humana. Ver
/// a varredura informativa de `x_main` impressa por `main.rs` para o
/// trade-off tipback×rotação; tail-strike e carga de nariz continuam
/// PASSANDO — só tipback e margem de combustível falham.
#[test]
fn engine_padrao_explicito_com_out_tempfile_converge_e_reporta_fail_honesto_de_tipback() {
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
        "binário deveria convergir e sair com sucesso (código 0) com o motor/aeronave/missão \
         reais passados explicitamente — o achado de tipback (ver nota acima) aparece no \
         CONTEÚDO do relatório, não no código de saída: stderr={}",
        String::from_utf8_lossy(&output.stderr));

    let json = std::fs::read_to_string(&out_path)
        .unwrap_or_else(|e| panic!("falha ao ler '{}': {e}", out_path.display()));
    assert!(json.contains("Toyota 1GD-FTV"), "JSON de saída deveria conter 'Toyota 1GD-FTV':\n{json}");
    // Achado honesto NOVO (Task 2, ver nota acima): validation_status É
    // FAIL — tipback abaixo do piso de 15°. Não mascarar tunando config.
    assert!(json.contains("\"validation_status\": \"FAIL\""),
        "JSON de saída deveria reportar validation_status FAIL (tipback abaixo do piso de \
         15° — achado honesto da Task 2, refino-ciclo2 — ver comentário acima):\n{json}");
    assert!(json.contains("Tipback:") && json.contains("abaixo do piso"),
        "violações deveriam citar a checagem de tipback nova:\n{json}");
    // Achado honesto NOVO (Task 3, ver nota acima): margem de combustível
    // (checagem #18) também viola — segundo motivo independente do FAIL.
    assert!(json.contains("Margem de combustível:") && json.contains("abaixo do mínimo"),
        "violações deveriam citar a checagem nova de margem de combustível \
         (Task 3, refino-ciclo2, achado honesto — não mascarar):\n{json}");
    // O envelope de CG (Task 4.4/E1–E6) continua FECHADO — tipback e margem
    // de combustível (achados NOVOS) falham, não uma regressão do envelope.
    assert!(!json.contains("fora do envelope de CG admissível"),
        "não deveria haver violações de cenário fora do envelope de CG admissível:\n{json}");
    assert!(!json.contains("Envelope de CG VAZIO"),
        "não deveria haver violação dedicada de envelope de CG vazio:\n{json}");
    // Tail-strike e carga de nariz nos dois extremos (também novas desde a
    // Task 2) PASSAM no baseline real — só tipback e margem de combustível
    // falham.
    assert!(!json.contains("Tail-strike:"),
        "não deveria haver violação de tail-strike no baseline real:\n{json}");
    assert!(!json.contains("Carga de nariz:"),
        "não deveria haver violação de carga de nariz (dois extremos) no baseline real:\n{json}");

    let _ = std::fs::remove_file(&out_path);
}
