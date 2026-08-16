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
/// missão exige ≈255,9 L contra 260 L de capacidade, margem ≈1,58% da
/// capacidade — abaixo do piso de 5%. `validation_status` continua `FAIL`
/// (já era, por causa do tipback) — agora por DOIS motivos independentes.
/// Não mascarado tunando o tanque/missão; a decisão de projeto (tanque
/// maior, missão menor, ou aceitar o risco) fica para revisão humana. Ver
/// a varredura informativa de `x_main` impressa por `main.rs` para o
/// trade-off tipback×rotação; tail-strike e carga de nariz continuam
/// PASSANDO — só tipback e margem de combustível falham.
///
/// ATUALIZAÇÃO (campanha E7, 2026-08-06): AMBOS os achados acima foram
/// RESOLVIDOS por decisão de projeto/cliente — dados, não física:
/// `mission.endurance_min_h` 8,0→7,0h (autonomia 7h + reserva, decisão do
/// cliente) fecha a margem de combustível (1,58%→≈13,97%, bem acima do
/// piso de 5%) e reduz o MTOW de missão convergido; `gear.x_main_m`
/// 3,55→3,66m fecha o tipback (10,1°→≈15,58° ≥ piso de 15°), viável porque
/// a autoridade de rotação DATCOM (ciclo 2) alargou o limite dianteiro do
/// envelope de CG para ≈13,0% MAC nesta posição. `validation_status` vira
/// `PASS` — primeiro PASS honesto com os 18 checks do ciclo 2 todos
/// ativos, zero violações (só o aviso elétrico de pico permanece, não é
/// violação).
///
/// ATUALIZAÇÃO (ciclo 3 — oew-parametrico, Task 4, 2026-08-07): TERCEIRO
/// achado honesto, e o mais importante do ciclo. As 7 massas estruturais
/// do OEW deixaram de ser itens FIXOS de `[[masses.items]]`/`mass_per_area`
/// e passaram a ser COMPUTADAS pelas equações de componente de Raymer
/// (cap. 15.2, `agents::mass_model`). O total estrutural mal se move
/// (422,0 → 411,0 kg; OEW 890,0 → 879,0 kg), mas a DISTRIBUIÇÃO muda muito:
/// fuselagem 160,0→110,6 kg e empenagens 43,0→19,6 kg (massas de braço
/// TRASEIRO, encolhem) contra trem 77,0→110,5 kg, asa 130,0→148,0 kg e
/// tanques 12,0→22,4 kg (braços à frente do CG, crescem). O CG vazio
/// AVANÇA, e com ele o de todos os cenários: a faixa observada vai de
/// 16,0–37,5% MAC para **8,3–31,7% MAC**. Consequência direta, NÃO
/// mascarada:
///   - 2 dos 6 cenários caem À FRENTE do limite dianteiro de rotação
///     (13,0% MAC): "Solo (piloto)" (8,3%, RE-MEDIDO ciclo 4 Task 2 — W_dg
///     de envelope com lag-1 — em 9,1%) e "2 pax dianteiros" (11,8%,
///     RE-MEDIDO em 12,5%);
///   - a carga no trem de NARIZ no CG mais dianteiro sobe de 24,8% para
///     **29,0%** (RE-MEDIDO em **28,6%**), acima do teto de 25% (checagem
///     #16).
/// `validation_status` volta a `FAIL`, com 3 violações nomeadas. Tipback
/// (19,2° ≥ 15°, RE-MEDIDO em 18,85°), tail-strike, margem de combustível
/// (14,56%, RE-MEDIDO em 14,33%) e hélice
/// continuam PASSANDO — o CG mais traseiro também avançou, o que na
/// verdade FOLGOU o tipback. A decisão de projeto (recuar de novo o
/// bagageiro/bateria, mover a asa, ou aceitar) fica para revisão humana:
/// este ciclo mede, não tuna. O caminho PASS continua coberto pelas
/// configs sintéticas (`validation::constraint_checker::tests`).
/// Renomeado de novo (o nome anterior dizia "pass_honesto").
///
/// ATUALIZAÇÃO (ciclo 4, Task 4 — checagem #19, robustez à incerteza do
/// modelo de massas): INVESTIGADO, não forçado. Com σ=15%
/// (`[mass_model].sigma_mass_fraction`), os dois conjuntos adversariais
/// (±σ direcional sobre as 7 massas estruturais) NÃO derrubam nenhum check
/// que passa no nominal — tipback (≈18,85° nominal vs. piso 15°: cai para
/// ≈17,36° no pior caso, ainda acima), carga de nariz MÍNIMA (≈15,86%
/// nominal vs. piso 8%: pior caso ≈14,52%, ainda acima) e os 4 cenários de
/// CG dentro do envelope (o mais próximo do limite, "4 pax sem bagagem" a
/// 25,2%, cai para ≈22,15% no pior caso — ainda bem acima do piso de
/// 13,0%) têm folga nominal grande o bastante para absorver a perturbação.
/// `validation_status` continua `FAIL`, com as MESMAS 3 violações — zero
/// violações NOVAS de robustez. Ver `tests/gear_tipback.rs::
/// constraint_checker_sem_violacoes_de_trem_nem_de_robustez_no_baseline_
/// real` (renomeado na campanha E10, quando este achado histórico deixou
/// de ocorrer) para os números completos.
///
/// ATUALIZAÇÃO (ciclo 6, revisão final — QUARTA violação honesta,
/// 3 → 4): o ciclo 6 introduziu o requisito de pista (600 m,
/// `config/missions/default.toml`, checks #23/#24) e, na revisão final do
/// mesmo ciclo, descobriu-se que o check #24 gateava a pista de GRAMA com
/// a distância de pouso PAVIMENTADA (`ldg_50ft_m`, μ de frenagem 0,40) —
/// `mu_brake_grass` (0,30) era validado na config e NUNCA consumido. Com
/// o pouso na grama de fato computado (`ldg_50ft_grass_m`), a rolagem de
/// frenagem alonga ~65 m e a distância vai de **539,97 m (pavimentado,
/// passava) para 604,99 m (grama, NÃO passa)** contra os 600 m
/// disponíveis. Violação NOVA e honesta:
/// `"Pouso (grama, 15 m): 605 m excede a pista disponível de 600 m"`.
/// Ou seja: o pouso na grama nunca coube na pista de fazenda de 600 m —
/// o modelo é que não estava olhando. Contagem esperada 3 → **4**;
/// nenhuma tolerância foi afrouxada e nenhuma das 3 violações anteriores
/// mudou de texto ou valor. A decolagem na grama (#23, 428,2 m) continua
/// passando com folga. Ver `.superpowers/sdd/2026-08-08-ciclo6-pista-e-
/// robustez-final/task-5-report.md` (seção "Correção pós-revisão final").
///
/// ATUALIZAÇÃO (ciclo 7, task 1 — `cl_max_to`): MESMA contagem (4), mas
/// DUAS violações TROCADAS de natureza. A rotação passou a derivar Vr do
/// CLmax de DECOLAGEM (`cl_max_to` = 1,585 = 1,45 + 0,5·(1,72−1,45)) em
/// vez do CLmax de POUSO (1,72), coerente com o `Cm_TO` de flap parcial
/// que o mesmo balanço já usava; e as distâncias de DECOLAGEM passaram a
/// usar o mesmo `cl_max_to`. Consequências, TODAS verificadas:
///
///   - **Limite dianteiro de rotação 12,995% → 8,908% MAC** (−4,087 pp).
///     Explicação fechada: `q_r ∝ 1/CL_max_TO` ⟹ todo o momento
///     disponível ×(1,72/1,585) = +8,52%; `x_cg_rot = x_main − M/W` ⟹
///     Δx = −0,08517·(3,660 − 3,0620) = −0,0509 m = −4,087 pp de MAC
///     (MAC 1,2463 m). Bate com o observado a 0,0e0 m. A Vr correta é
///     MAIOR (+4,21%), logo há MAIS pressão dinâmica e MAIS autoridade de
///     profundor na rotação — o modelo antigo era pessimista, não
///     conservador por escolha.
///   - **As DUAS violações NOMINAIS de envelope FECHARAM**: 'Solo
///     (piloto)' (9,1% MAC) e '2 pax dianteiros' (12,5%) agora ficam
///     ATRÁS do limite dianteiro de 8,91% — dentro do envelope
///     [8,9%–43,5%]. As margens de autoridade de rotação por cenário
///     saíram do vermelho: Solo −7,46% → +0,42%, 2 pax −1,07% → +7,36%.
///   - **Duas violações NOVAS de ROBUSTEZ** tomaram o lugar delas: os
///     MESMOS dois cenários passam no NOMINAL mas reprovam com massas
///     estruturais ±15% (checagem #19): Solo 4,55 vs 8,91 e 2 pax 8,46 vs
///     8,91 (%MAC no pior caso dianteiro). O achado não sumiu, mudou de
///     categoria — deixou de ser "fora do envelope no nominal" e virou
///     "sem margem para a incerteza de massa estrutural". É exatamente
///     para isso que a checagem #19 existe; ela não disparava antes
///     porque o nominal já violava.
///   - **Decolagem mais LONGA** (o espelho honesto do ganho na rotação —
///     o modelo antigo era otimista na decolagem): grama 15 m 428,2 →
///     457,7 m (+6,88%), pavimentada 381,4 → 406,9 m; estimativas
///     simplificadas (×1,5) grama 421,3 → 457,1 m e pavimentada 351,1 →
///     381,0 m (+8,52% exatos, `S_G ∝ 1/CL_TO`). A decolagem na grama
///     continua PASSANDO nos 600 m (folga 142 m).
///   - Pouso (grama e pavimentado), VS0, VS1 e o gradiente de subida:
///     INALTERADOS — nenhum deles descreve decolagem.
///
/// Contagem 4 → **4**, `validation_status` continua `FAIL`, e NENHUMA
/// tolerância foi afrouxada.
///
/// ─── ATUALIZAÇÃO (campanha E10, 2026-08-08) — O TESTE INVERTE ───────────
///
/// Este teste existia para asserir, POR NOME e POR CONTAGEM, os FAILs
/// honestos que o baseline real ainda carregava. A campanha E10 resolve os
/// quatro por PROJETO (dados, nada em `src/` mudou) e o baseline real passa
/// a reportar `validation_status: PASS` com **0 violações e 0 flips de
/// robustez** — o primeiro PASS completo do projeto sob o modelo inteiro
/// (24 checks + robustez a 3 mundos). O teste é reescrito para asserir o
/// PASS; a cobertura dos CAMINHOS DE ERRO não é perdida: ela vive nas
/// configs sintéticas mutadas de `src/validation/constraint_checker.rs`
/// (violações nomeadas de envelope, tipback, carga de nariz, pista, margem
/// de combustível, hélice, …) e de `src/validation/robustness.rs` (flips
/// nomeados nos três mundos adversariais), todas verificadas e intactas.
///
/// As quatro violações, uma a uma, e o que as fechou:
///
///   1. **Carga de nariz 28,6% > teto de 25%** → **22,77%**. Duas mudanças:
///      `[gear].x_nose_m` 1,40→1,30 (alonga `x_main − x_nose`, o
///      denominador da fração de carga) e a bateria híbrida de 53 kg
///      (28→53) a 7,80 m (`arm_offset_m` 0,4 sobre `empennage_cg`), que
///      recua o CG mais DIANTEIRO real de 9,1% para 17,9% MAC.
///   2/3. **Robustez (#19) 'Solo (piloto)' e '2 pax dianteiros'** (4,55 e
///      8,46 vs limite 8,91 %MAC no pior mundo dianteiro) → **0 flips**.
///      Mesmo recuo de CG do item 1: as margens de autoridade de rotação
///      desses cenários saem de 0,4% e 7,4% para 21,6% e 29,4%, folga
///      suficiente para sobreviver a ±15% de massa estrutural nos dois
///      conjuntos adversariais. O que fecha os flips é o CG dos CENÁRIOS
///      recuar, não o LIMITE se mover: o limite de rotação é invariante ao
///      peso e ao CG (não recebe nem um nem outro — ver
///      `agents::trim_authority::rotation_fwd_limit_m`) e praticamente não
///      anda, 8,908% → 8,533% MAC. Esses 0,375 pp são o saldo de dois
///      efeitos OPOSTOS, os únicos canais por onde E10 alcança o limite:
///      `cl_max_to` 1,585→1,6775 o recuaria sozinho para 11,78% (menos
///      `q_r`), e `Cm_TO` −0,158→−0,113 o avançaria sozinho para 5,47%
///      (menos flap de decolagem, menos nariz-para-baixo) — ambos governados
///      por `to_flap_fraction` 0,5→0,35 (o primeiro também por
///      `cl_max_flaps`).
///   4. **Pouso (grama, 15 m) 605 m > pista de 600 m** → **556,7 m**.
///      `[wing].cl_max_flaps` 1,72→2,1 (flap SIMPLES → SLOTTED) derruba VS0
///      de 113,3 para 103,4 km/h; a distância de pouso escala com `V_ref²`.
///
/// Custos honestos da campanha, todos pinados nos testes citados:
///   - Hélice Ø1,95→1,76 m (obrigatória: o trem curto `h_cg_ground_m`
///     1,05→0,92 baixa o eixo 1:1 e a folga de solo cairia para 0,145 m <
///     0,23 m) ⟹ η_p 81,0%→78,4% ⟹ cruzeiro 302,1→300,2 km/h, consumo
///     30,4→32,4 L/h, autonomia informativa 7,71→7,23 h.
///   - +13,8 kg de OEW e +25,1 kg de MTOW convergido ⟹ margem de
///     combustível 14,33%→9,14% da capacidade (piso 5%).
///   - Margem estática mínima 16,25%→9,68% (piso de projeto 5%) — o recuo
///     de CG que fechou o lado DIANTEIRO consome o lado TRASEIRO. Ver
///     `tests/empennage.rs`.
///   - Decolagem 2,3–2,6% mais longa (grama 457,7→469,3 m sobre 15 m,
///     pista de 600 m). ATUALIZAÇÃO (ciclo 8, task 1 — arrasto de flap na
///     polar): 469,3→473,6 m (o segmento de SUBIDA cobra o arrasto de flap
///     parcial agora, `cd0_flap_to_extra`), folga sobre a pista cai de
///     ≈131 m para ≈126 m, ainda folgada — ver `tests/generic_engine.rs`.
/// O aviso elétrico de pico (1.260 W > alternador 900 W) PERMANECE — é
/// aviso, não violação, e agora é coberto pelo banco de baterias de 53 kg
/// que a própria E10 instalou (ver comentário do item de massa no TOML).
///
/// ─── ATUALIZAÇÃO (ciclo 9, transferência de atitude do #25) — O TESTE
/// INVERTE DE NOVO ───────────────────────────────────────────────────────
///
/// `PropellerSpec::fill_critical_clearance` corrige a simplificação
/// conhecida (ciclo 8, `docs/backlog.md` item 1): o colapso do amortecedor
/// de nariz + pneu murcho não translada a célula 1:1 — ela PIVOTA sobre o
/// trem PRINCIPAL, e a hélice (à frente do trem de nariz) mergulha um
/// braço amplificado por `fator =
/// (gear.x_main_m−propeller.prop_plane_x_m)/(gear.x_main_m−gear.x_nose_m)`.
/// Campo novo `[propeller].prop_plane_x_m = 0,20` (posição do plano da
/// hélice, m do datum no nariz — ESTIMATIVA, validar no CAD). No baseline
/// real: fator = (3,66−0,20)/(3,66−1,30) ≈ **1,46610**; `prop_clearance_
/// critical_m` vai de **+0,0325 m (PASS) para ≈−0,06416 m (FAIL)** —
/// checagem #25 reprova. Física corrigida, NÃO uma regressão: a
/// simplificação 1:1 do ciclo 8 subestimava o mergulho da hélice em
/// ~47% e mascarava este achado, exatamente como o próprio caveat previu.
///
/// Nenhuma outra checagem muda — todos os achados honestos da E10 acima
/// (carga de nariz, robustez, pista, tipback/tail-strike/margem de
/// combustível) continuam com os MESMOS números, verificados pelos
/// asserts abaixo. `validation_status` vira `"FAIL"` com **exatamente 1
/// violação nomeada** (checagem #25, hélice) — o primeiro FAIL honesto
/// desde a campanha E10, movido por física corrigida, não por dados de
/// projeto. O caminho PASS de #25 continua coberto pela fixture sintética
/// (`validation::constraint_checker::tests::check_25_sem_violacao_na_
/// fixture_padrao`).
///
/// ─── ATUALIZAÇÃO (ciclo 10, task 1, deflexão estática) — MESMO VEREDITO,
/// NÚMERO CORRIGIDO ─────────────────────────────────────────────────────
///
/// Campo novo `[gear].static_sag_fraction = 0,33` (fração do curso do
/// nariz já consumida pela compressão ESTÁTICA — ver docstring de
/// `GearCfg::static_sag_fraction`). CS 23.925 pela LETRA: só o trem
/// CRÍTICO (nariz) vai ao batente; os mains ficam na deflexão estática já
/// embutida em `h_cg_ground_m`, e o próprio nariz PARTE dessa mesma
/// deflexão — na condição crítica ele só percorre o curso RESTANTE
/// (`nose_oleo_stroke_mm × (1 − static_sag_fraction)`), não o curso TOTAL
/// que o ciclo 9 usava (dupla contagem da compressão estática). `fator`
/// permanece **1,46610** (não depende de `static_sag_fraction`);
/// `prop_clearance_critical_m` vai de **≈−0,06416 m (ciclo 9) para
/// ≈−0,00249 m (ciclo 10)** — honestamente ANTI-conservador (folga
/// MAIOR), mas fiel à norma. `validation_status` PERMANECE `"FAIL"` com a
/// MESMA 1 violação nomeada (checagem #25) — só o NÚMERO da violação
/// muda, não o veredito. Ver `docs/backlog.md` (item 6, RESOLVIDO ciclo
/// 10).
///
/// ─── ATUALIZAÇÃO (ciclo 10, task 2, LINHA DE TRAÇÃO) — MESMO VEREDITO,
/// MENOS FOLGA ─────────────────────────────────────────────────────────
///
/// O momento da linha de tração entra no balanço de rotação
/// (`−T(Vr)·z_eixo`, nariz-abaixo). `z_eixo` é o offset EIXO↔CG
/// (`[propeller].prop_axis_above_cg_m` = 0,20 m), **não** a altura sobre o
/// solo: na corrida ACELERADA o termo inercial de d'Alembert cancela a
/// porção `h_cg` do braço (derivação em `agents::trim_authority::
/// rotation_available_moment_nm`; ERRATUM da spec §2).
///
/// Efeito medido no baseline real: `rotation_limit_pct_mac` **8,533% →
/// 13,355% MAC** (+4,82 pp) — a tração a Vr do cenário mais leve vale
/// ≈3,5 kN e o binário nariz-abaixo consome ~18% do momento nariz-acima
/// disponível. O envelope continua FECHADO (13,4% < 43,5%) e o CG mais
/// dianteiro do baseline (17,9% MAC) continua ATRÁS do limite, com 4,5 pp
/// de folga (eram 9,3 pp).
///
/// **Veredito INALTERADO**: `validation_status` continua `"FAIL"` com
/// EXATAMENTE 1 violação nomeada — a de hélice (#25), com o MESMO número
/// (−0,002 m), que a task 2 não toca. ZERO flips de robustez, ZERO
/// cenários fora do envelope. O que mudou é a FOLGA: as margens de
/// autoridade de rotação caem (o cenário mais apertado, "Solo (piloto)",
/// de +21,6% para +10,5%). Custo honesto de um termo de momento que
/// faltava, agora cobrado.
///
/// ─── ATUALIZAÇÃO (campanha E12 "nariz-only", 2026-08-10, adoção pós-
/// ciclo-10) — O TESTE INVERTE DE NOVO, `validation_status` VIRA `PASS`
/// ───────────────────────────────────────────────────────────────────────
///
/// A célula E11 do ciclo 9 (`docs/superpowers/specs/2026-08-09-ciclo9-
/// transferencia-atitude-design.md` §4) combinava DUAS mudanças para
/// fechar a checagem #25: `[propeller].prop_axis_above_cg_m` 0,20→0,32
/// (eixo da hélice mais alto) e `[gear].x_nose_m` 1,30→1,20 (nariz
/// avançado). A re-avaliação do ciclo 10 com o modelo completo (sag
/// estático + linha de tração) mostrou que a metade BARATA — só o nariz,
/// sem mexer no eixo — já fecha o envelope sozinha: janela viável medida
/// x_nose ∈ (1,16075, 1,27550) m. Decisão de adoção do usuário: `x_nose_m`
/// 1,30→1,20, `prop_axis_above_cg_m` mantido em 0,20.
///
/// O fator de amplificação do #25 (`(x_main−prop_plane_x_m)/
/// (x_main−x_nose_m)`) cai de ≈1,46610 para ≈1,40650 (denominador
/// alonga) — `prop_clearance_critical_m` vai de **≈−0,00249 m (ciclo 10)
/// para +0,007367 m (E12)**, a checagem #25 FECHA. `rotation_limit_pct_mac`
/// fica **INALTERADO por `x_nose_m`** (13,354637% MAC NAQUELE momento —
/// `x_nose_m` não entra na régua de `TrimAuthorityAgent`; ciclo 12, task 4,
/// muda esse valor por um mecanismo TOTALMENTE diferente — ver a
/// ATUALIZAÇÃO mais abaixo); o que se move é o CG dos cenários mais
/// dianteiros (o item de massa `trem_nariz` avança com `x_nose_m`), que
/// consome uma fração pequena da margem de rotação de cada cenário — a do
/// cenário mais apertado, "Solo (piloto)", cai de +10,4595% para
/// +10,1891% (ainda com folga folgada sobre o limite).
///
/// `validation_status` vira `"PASS"` com **ZERO violações** — o primeiro
/// PASS do baseline com o MODELO COMPLETO (sag estático correto + linha de
/// tração + transferência de atitude do #25, todos ativos). Todos os
/// demais achados honestos (carga de nariz, robustez, pista, tipback/
/// tail-strike/margem de combustível) continuam com os MESMOS números —
/// `x_nose_m` só afeta o #25 e o CG dos cenários mais dianteiros de forma
/// mensurável, verificados pelos asserts abaixo.
///
/// ─── ATUALIZAÇÃO (ciclo 12, task 2, 2026-08-15) — O TESTE VOLTA A `FAIL`,
/// DE PROPÓSITO ───────────────────────────────────────────────────────────
///
/// A rolagem de decolagem passa de método energético fechado de Raymer
/// (sem termo de arrasto nem de atrito explícitos — ver docstring `old→new`
/// de `agents::performance::takeoff_ground_roll_m`) para integração
/// numérica da equação de movimento consumindo a polar completa (spec
/// `2026-08-15-ciclo12-solo-honesto`). O segmento DOMINANTE da distância de
/// decolagem finalmente paga arrasto (`cd_gear_extended`) e atrito de
/// rolagem explícito (`mu_roll_grass=0,08`, substituindo o antigo
/// `surface_factor=1,20` que contava a grama sem separar atrito de
/// arrasto). Medido: `to_50ft_grass_m` 473,469470 m → **819,110978 m**
/// (+73,0%), estourando `req.runway_available_m` (600 m) por ≈219 m —
/// checagem #23 REPROVA pela primeira vez. `validation_status` volta a
/// `"FAIL"` com, na task 2, **EXATAMENTE 1 violação** — a de decolagem na
/// grama. Nenhum outro achado muda (hélice/#25, carga de nariz, robustez,
/// tipback/tail-strike/margem de combustível, pouso na grama): a rolagem de
/// pouso e o balanço de rotação eram as Tasks 3/4 deste ciclo, fora do
/// escopo da task 2. **Isto não é regressão** — é o modelo passando a
/// dizer a verdade sobre operar esta célula numa pista de fazenda de
/// 600 m (diretriz permanente do usuário: "se uma decisão é perigosa, o
/// modelo deve FALHAR no ponto de perigo").
///
/// ─── ATUALIZAÇÃO (ciclo 12, task 3, 2026-08-15) — SEGUNDA VIOLAÇÃO,
/// TAMBÉM O RESULTADO ESPERADO ─────────────────────────────────────────────
///
/// A rolagem de pouso passa pela MESMA transformação (método fechado
/// `S_G=V_ref²/(2gμ)` → integração numérica, spec §5) — com o flap de
/// pouso mantido deflexionado durante toda a frenagem (decisão do usuário),
/// a sustentação residual ALIVIA o peso sobre as rodas e PIORA a frenagem;
/// o arrasto ajuda, mas o saldo é uma rolagem MAIOR. Medido:
/// `ldg_50ft_grass_m` 556,677173 m → **646,437301 m** (+16,1%), estourando
/// os 600 m de pista por ≈46 m — checagem #24 REPROVA também.
/// `validation_status` continua `"FAIL"`, agora com EXATAMENTE 2
/// violações — decolagem E pouso na grama, NAQUELE momento. Nenhum outro
/// achado mudava (hélice/#25, carga de nariz, robustez, tipback/
/// tail-strike/margem de combustível): o balanço de rotação era a Task 4
/// deste ciclo, ainda não implementada. **Isto também não é regressão** —
/// mesma diretriz permanente do usuário citada acima.
///
/// ─── ATUALIZAÇÃO (ciclo 12, task 4, 2026-08-15) — TERCEIRA E QUARTA
/// VIOLAÇÃO (ROBUSTEZ), `old→new` ────────────────────────────────────────
///
/// Os termos de SOLO do balanço de rotação (atrito de rolagem + arrasto,
/// spec §6) somam-se ao termo de linha de tração já existente e recuam o
/// limite dianteiro **≈13,3546% → ≈17,7580% MAC** (+4,40 pp) —
/// `rotation_limit_pct_mac`/`cg_limit_fwd_pct_mac` medidos no pipeline
/// real. Nenhum cenário cruza esse limite NO NOMINAL (`validation_status`
/// continua sem violação DEDICADA de envelope — a margem de rotação do
/// cenário mais apertado, "Solo (piloto)", cai para
/// **≈0,0012%**, essencialmente ZERO, mas ainda tecnicamente positiva).
/// Só que a margem quase-zero deixa dois cenários — "Solo (piloto)" e
/// "2 pax dianteiros" — vulneráveis ao mundo de ROBUSTEZ `dianteiro`
/// (massas estruturais ±15%, checagem #19): a régua do mundo perturbado
/// sobe para ≈18,09% MAC (o momento da linha de tração e dos termos de
/// solo também respondem ao peso do mundo perturbado), e os dois cenários,
/// que passavam no nominal, REPROVAM no mundo dianteiro — **2 flips
/// NOVOS**, `validation_status` continua `"FAIL"`, agora com
/// **EXATAMENTE 4 violações**: decolagem na grama, pouso na grama (Tasks
/// 2/3, inalteradas) E os dois flips de robustez novos (Task 4). Nenhum
/// outro achado muda (hélice/#25, carga de nariz, tipback/tail-strike/
/// margem de combustível). **Isto também não é regressão** — mesma
/// diretriz permanente do usuário citada acima: os termos de solo
/// "deliberadamente desprezados" pelo ciclo 10 (estimativa "≲2 pp de MAC",
/// hoje sabida errada — medição real ≈4,40 pp, ver docstring `old→new` de
/// `agents::trim_authority::rotation_available_moment_nm`) finalmente
/// cobram o preço físico que já deveriam cobrar. O ciclo 13 decide o que
/// fazer a respeito (mais potência, mais asa, pista maior, ou aceitar
/// operação só pavimentada) — não esta task.
///
/// ─── ATUALIZAÇÃO (ciclo 13, task 2, 2026-08-15) — LEI ÚNICA DE TRAÇÃO,
/// COMPOSIÇÃO MUDA, CONTAGEM NÃO ────────────────────────────────────────
///
/// A lei única `T(V) = FoM(J)·T_ideal(V, P_eixo)` (spec §2) substitui os
/// dois modelos que divergiam 27,69% em `Vr≡V_LOF` (backlog #15). Medido
/// no baseline real, DUAS coisas acontecem ao mesmo tempo, em direções
/// OPOSTAS:
///
/// (a) **O balanço de rotação AFROUXA** (spec §6): `thrust_at_rotation_n`
///     agora chama a MESMA lei que a rolagem — o polinômio apagado violava
///     o teto de quantidade de movimento em `Vr` por 1,0372× (spec §1.1);
///     com a lei nova (mais fraca nesse ponto), `rotation_limit_pct_mac`
///     recua **17,757974% → 16,392661% MAC** (−1,365 pp, −7,69% —
///     bate com a projeção da spec §11, "≈16,4%"). A margem nominal de
///     'Solo (piloto)' sobe 0,001186%→3,160081% (+3,16 pp — a spec §11
///     projetava "≈+1,4 pp", subestimou por ~2,3×) e a de '2 pax
///     dianteiros' sobe 7,776175%→10,611581% — o bastante para o flip de
///     robustez desse cenário **DESAPARECER** (spec §11: "provavelmente
///     resolve", confirmado). O de 'Solo (piloto)' PERSISTE (13,74 vs
///     16,60 no mundo dianteiro — spec §11: "persiste", confirmado).
///
/// (b) **O gradiente CS 23.65 e a decolagem em grama PIORAM**: em Vx
///     (J≈0,82) e ao longo do segmento de SUBIDA da decolagem (V_climb =
///     1,2·Vs_to ≈ 38,6 m/s, J similar), o polinômio apagado também
///     violava o teto físico — a tração cai ≈21% nesse regime (spec §11).
///     `climb_gradient_pct` cai **12,451842% → 8,015811%** — ABAIXO do
///     piso de 8,3% da CS 23.65 (bate com a projeção "≈7,9%", gate FLIPA
///     PASS→FAIL como a spec §11 avisou que podia acontecer — não é
///     regressão, é o polinômio deixando de mascarar o teto físico).
///     `to_50ft_grass_m` **AUMENTA** 819,110978→**848,927019 m** (+3,64%)
///     — na direção OPOSTA da projetada pela spec §3.4 (≈784,5 m, uma
///     REDUÇÃO): a rolagem pura de fato encolhe (mais tração que o modelo
///     0,75-constante em toda a faixa V∈[0,V_LOF], FoM(J)≥0,75 sempre),
///     mas o segmento de SUBIDA (que usa V_climb, MUITO além de V_LOF, no
///     regime onde a tração cai ≈21%) cresce mais que o suficiente para
///     inverter o sinal do total. **A projeção da spec §3.4 errou** — a
///     tabela de sensibilidade de forma de FoM(J) mediu só a ROLAGEM
///     isoladamente contra `to_50ft_grass_m`, sem isolar que o segmento de
///     SUBIDA (V muito maior que V_LOF) domina o resultado quando o
///     gradiente já está perto do piso. Achado NOVO deste ciclo, registrar
///     no backlog (Task 5).
///
/// Resultado líquido (task 2): **mesma CONTAGEM** de violações (4),
/// COMPOSIÇÃO diferente — sai o flip de robustez '2 pax dianteiros', entra
/// o gradiente CS 23.65. `validation_status` continua `"FAIL"`.
///
/// ─── ATUALIZAÇÃO (ciclo 13, task 3, 2026-08-15) — ERRATUM §3.2.1,
/// NÚMEROS SE MOVEM, CONTAGEM NÃO ────────────────────────────────────────
///
/// `fom_design` recalibrado por ponto fixo (a âncora de cruzeiro do §3.2
/// usava o `u` da POTÊNCIA disponível; o correto é o `u` da TRAÇÃO
/// requerida — ver o erratum na spec). `0,823706...→0,815977...` (−0,94%)
/// gira a curva `FoM(J)` inteira um pouco para baixo: `climb_gradient_pct`
/// 8,015811%→**7,913277%**, `to_50ft_grass_m` 848,927019→**≈859 m**,
/// `ldg_50ft_grass_m` 646,437301→**≈647 m**. Contagem PERMANECE **4**.
///
/// ─── ATUALIZAÇÃO (ciclo 13, task 4, 2026-08-15) — DUAS SUPERFÍCIES DA
/// ROTAÇÃO (spec §7, fecha o backlog #16), CONTAGEM SOBE PARA 5 ──────────
///
/// Até aqui `rotation_limit_pct_mac` era calculado com `mu_roll_paved`
/// enquanto as checagens #23/#24 (decolagem/pouso) já reprovavam a GRAMA —
/// o mesmo JSON afirmava duas superfícies para a MESMA decolagem. Esta
/// task calcula o limite NAS DUAS superfícies
/// (`rotation_limit_pct_mac_paved`/`_grass`, campos NOVOS) e publica
/// `rotation_limit_pct_mac` na superfície de OPERAÇÃO — GRAMA, a mesma que
/// #23/#24 medem.
///
/// Medido no baseline real: `rotation_limit_pct_mac` **16,392661% (pavimentado,
/// ciclo 13 task 2/3) → 18,268251% (grama)**, +1,875590 pp — mesma ordem da
/// projeção da spec §11 ("grama: +~1,9 pp") e quase idêntica à medição do
/// ciclo 12 com o modelo ANTIGO de tração (+1,888 pp, spec §7). Efeito em
/// CASCATA, nomeado pela spec §11.1 ("Interação §7×§6") e NÃO consertado:
///
/// - **'Solo (piloto)' (CG nominal ≈17,8% MAC) cai À FRENTE do novo limite
///   (≈18,3%)** — deixa de ser um FLIP de robustez e vira **VIOLAÇÃO
///   NOMINAL de envelope**, EXATAMENTE o cenário que a spec §11.1 nomeou
///   como possível ("pode reabrir 'Solo (piloto)' NOMINALMENTE"). Sai da
///   lista de robustez, entra na de envelope.
/// - **'2 pax dianteiros' reabre em robustez**: a régua do mundo
///   perturbado (±15% de massa estrutural) recua junto com o limite
///   nominal, e a margem volta a ficar negativa (16,80 vs 18,47).
///
/// Contagem de violações: **4 → 5** — a de envelope é NOVA (soma 1); a de
/// robustez continua em 1 (só troca de nome, 'Solo (piloto)' → '2 pax
/// dianteiros'). `validation_status` continua `"FAIL"`. Nenhuma config foi
/// ajustada para evitar este resultado — é a medição que a spec §11.1
/// pediu, não uma regressão.
///
/// Resultado líquido (task 4): **5 violações** — gradiente CS 23.65,
/// decolagem em grama, pouso em grama (as três INTOCADAS por esta task),
/// a violação NOMINAL de envelope de 'Solo (piloto)' (NOVA), e o flip de
/// robustez de '2 pax dianteiros' (reaberto). `validation_status`
/// continua `"FAIL"`.
///
/// ─── ATUALIZAÇÃO (ciclo 14, task 2, 2026-08-15) — CHECAGEM #24 FLIPA
/// FAIL→PASS, CONTAGEM CAI PARA 4 ───────────────────────────────────────
///
/// O segmento AÉREO do pouso (`agents::performance::landing_air_segment`)
/// corrige DOIS defeitos independentes (spec do ciclo 14, §1):
///
/// 1. GEOMÉTRICO: até aqui `s_air = 15/tan(γ_app)` descia os 15 m
///    INTEIROS até o solo e o flare (`s_flare = V_ref × flare_time_s`) era
///    somado com altura ZERO — a aeronave "pousava" duas vezes. Agora o
///    flare é um arco de raio `R = V_ref²/(g·(n−1))` que CONSOME altura
///    (`h_flare = R(1−cos γ)`), e a rampa desce só `15 − h_flare`.
/// 2. DE PREMISSA: `[performance].approach_angle_deg = 3,0°` (removido)
///    era o *glideslope* de ILS — aproximação COM POTÊNCIA de aeroporto
///    pavimentado, mais RASA do que esta célula desce com o motor
///    cortado. Agora γ_app é DERIVADO da polar de pouso a V_ref
///    (`atan(CD_ref/CL_ref)`, `cd_gear_extended` — mesma função que a
///    rolagem usa): **5,1181°**, procedimento de campo curto padrão
///    (motor em marcha lenta sobre o obstáculo).
///
/// Os dois juntos encolhem o segmento aéreo 339,82 m → **196,57 m**
/// (−42,1%). Medido no baseline real: `ldg_50ft_grass_m`
/// 646,660942 m → **503,414253 m** (−22,2%), agora ABAIXO da pista de
/// 600 m — a checagem #24 FLIPA FAIL→PASS, o PRIMEIRO ciclo desde o 11 que
/// REMOVE uma violação. `ldg_50ft_m` (pavimentado) 582,521767 m →
/// **439,275078 m** (−24,6%). Contagem de violações: **5 → 4** — só a de
/// pouso na grama sai; as outras 4 (gradiente CS 23.65, decolagem em
/// grama, envelope NOMINAL de 'Solo (piloto)', robustez de '2 pax
/// dianteiros') são de FORA do pouso e permanecem INALTERADAS por
/// construção (spec §3 do ciclo 14 as declara isoladas). `validation_status`
/// continua `"FAIL"` (ainda restam 4 violações, incluindo a decolagem em
/// grama a ≈859 m). **Não é afrouxamento de premissa**: a pista de 600 m
/// em grama permanece INTACTA — a violação era sustentada por um erro
/// geométrico e uma premissa de aeroporto pavimentado, ambos nomeados e
/// medidos, não por uma config relaxada.
#[test]
fn engine_padrao_explicito_com_out_tempfile_reporta_fail_honesto_ciclo12_decolagem_e_pouso_grama() {
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
         reais passados explicitamente: stderr={}",
        String::from_utf8_lossy(&output.stderr));

    let json = std::fs::read_to_string(&out_path)
        .unwrap_or_else(|e| panic!("falha ao ler '{}': {e}", out_path.display()));
    assert!(json.contains("Toyota 1GD-FTV"), "JSON de saída deveria conter 'Toyota 1GD-FTV':\n{json}");
    // Ciclo 12 (task 2, `old→new`, ver a ATUALIZAÇÃO na docstring):
    // validation_status VOLTA a FAIL — a rolagem de decolagem integrada
    // (arrasto+atrito explícitos) estoura a pista de grama de 600 m
    // (checagem #23). Não é regressão: é o achado que este ciclo existe
    // para produzir.
    assert!(json.contains("\"validation_status\": \"FAIL\""),
        "JSON de saída deveria reportar validation_status FAIL (ciclo 12, task 2 — rolagem de \
         decolagem integrada estoura a pista de grama, ver comentário acima):\n{json}");
    let spec: serde_json::Value = serde_json::from_str(&json)
        .expect("saída deveria ser JSON válido");
    let violations: Vec<String> = spec["violations"].as_array()
        .expect("violations deveria ser um array presente")
        .iter().map(|v| v.as_str().unwrap_or_default().to_string()).collect();
    // `old→new` (ciclo 13, task 4 — ver ATUALIZAÇÃO na docstring acima):
    // contagem SOBE de 4 para 5 — o limite de rotação passa a valer a
    // superfície de OPERAÇÃO (grama, spec §7), que é MAIS restritiva que a
    // pavimentada usada até a task 3. Isso empurra 'Solo (piloto)' para
    // violação NOMINAL de envelope (soma 1) e reabre o flip de robustez de
    // '2 pax dianteiros' (troca de nome, não soma). Assert de contagem
    // PRIMEIRO, com a lista inteira na mensagem: qualquer violação nova
    // aparece por nome no output do teste, sem precisar adivinhar qual foi.
    // `old→new` (ciclo 14, spec §2/§7): contagem **5 → 4** — a checagem
    // #24 (pouso na grama sobre 15 m) FLIPA FAIL→PASS. Causa: os dois
    // defeitos do segmento aéreo do pouso corrigidos juntos (flare que não
    // consumia altura + ângulo de aproximação de ILS/3° substituído pelo
    // planeio power-off derivado da polar, γ_app=5,1181°) —
    // `ldg_50ft_grass_m` 646,660942 m → **503,414253 m** (−22,2%), abaixo
    // da pista de 600 m. As outras 4 violações são de FORA do pouso (spec
    // §3 as declara ISOLADAS desta mudança) e permanecem INALTERADAS.
    assert_eq!(violations.len(), 4,
        "ciclo 14 (spec §2/§7): esperava EXATAMENTE 4 violações no baseline real — gradiente \
         CS 23.65 abaixo de 8,3%, decolagem na grama sobre 15 m (≈859 m > 600 m), a violação \
         NOMINAL de envelope de 'Solo (piloto)' (superfície de grama no balanço de rotação — \
         spec §7/§11.1 do ciclo 13), E o flip de robustez de '2 pax dianteiros'. O pouso na \
         grama sobre 15 m SAIU da lista (checagem #24 FLIPOU FAIL→PASS, ldg_50ft_grass_m \
         646,660942→503,414253 m): {violations:#?}");
    // Asserts NOMEADOS por checagem — redundantes com a contagem acima de
    // propósito: se um refactor um dia reabrir/fechar uma violação, a
    // contagem sozinha não diria QUAL mudou.
    //
    // (#25) Folga crítica de hélice: continua FECHADA desde a E12
    // nariz-only (≈+0,007367 m) — INTOCADA pelo ciclo 13 (não consome
    // tração).
    assert!(!violations.iter().any(|v| v.contains("condição crítica CS 23.925")),
        "folga crítica de hélice (≈+0,007367 m, checagem #25) deveria continuar FECHADA (ciclo \
         13 não a toca): {violations:#?}");
    // (1) Carga de nariz: continua abaixo do teto de 25% (≈21,90%) —
    // INTOCADA pelo ciclo 13 (não consome tração).
    assert!(!violations.iter().any(|v| v.contains("Carga de nariz:")),
        "carga de nariz (≈21,90%) deveria continuar abaixo do teto de 25% (ciclo 13 não a \
         toca): {violations:#?}");
    // (2) Robustez (#19). `old→new` (ciclo 12 → ciclo 13 task 2): eram DOIS
    // flips ('Solo (piloto)' e '2 pax dianteiros'). A lei única afrouxa o
    // balanço de rotação (spec §6 — resíduo de d'Alembert zerado, spec
    // §1.1: o polinômio apagado violava o teto físico em `Vr` por
    // 1,0372×), subindo a margem nominal de '2 pax dianteiros' o
    // suficiente (7,776%→10,612%) para o flip dele DESAPARECER — spec §11
    // projetava "provavelmente resolve", confirmado. 'Solo (piloto)'
    // PERSISTIA (margem nominal 0,0012%→3,160%, ainda insuficiente contra
    // o mundo dianteiro ±15%) — spec §11 projetava "persiste", confirmado.
    //
    // `old→new` (ciclo 13, task 4 — spec §7/§11.1): o limite de rotação
    // passa a valer a superfície de OPERAÇÃO (grama), MAIS restritiva
    // (+1,876 pp). A régua nominal de 'Solo (piloto)' sobe junto e o CG
    // dele fica À FRENTE dela — deixa de ser um flip de ROBUSTEZ (que exige
    // passar no nominal) e vira violação NOMINAL de envelope (checada mais
    // abaixo). '2 pax dianteiros' REABRE: a régua do mundo dianteiro
    // (±15% de massa) recua junto e a margem volta a negativa (16,80 vs
    // 18,47). Contagem de robustez CONTINUA 1 — só troca de nome.
    assert_eq!(violations.iter().filter(|v| v.starts_with("Robustez:")).count(), 1,
        "ciclo 13 (task 4): esperava EXATAMENTE 1 violação de robustez (σ=15%, mundo \
         dianteiro), na superfície de operação (grama) — '2 pax dianteiros' reabriu; 'Solo \
         (piloto)' saiu desta lista porque virou violação NOMINAL de envelope: \
         {violations:#?}");
    assert!(violations.iter().any(|v| v.contains("Robustez")
        && v.contains("2 pax dianteiros")),
        "esperava o flip de robustez nomeado do cenário '2 pax dianteiros' (reaberto pela \
         superfície de grama, spec §11.1): {violations:#?}");
    assert!(!violations.iter().any(|v| v.contains("Robustez")
        && v.contains("Solo (piloto)")),
        "'Solo (piloto)' não deveria mais aparecer como flip de ROBUSTEZ — ele virou violação \
         NOMINAL de envelope (a régua o alcançou até no mundo nominal): {violations:#?}");
    // (3) `old→new` (ciclo 13, spec §11 — RISCO CENTRAL DO CICLO): o
    // polinômio apagado também violava o teto físico em Vx/no segmento de
    // subida da decolagem (≈21% de tração a menos com a lei nova nesse
    // regime). `climb_gradient_pct` cai 12,451842%→8,015811%, ABAIXO do
    // piso de 8,3% da CS 23.65 — gate FLIPA PASS→FAIL. Não é regressão de
    // código: é o polinômio deixando de mascarar o teto de quantidade de
    // movimento exatamente onde a spec §1.1 media a violação mais grave.
    assert!(violations.iter().any(|v| v.contains("Gradiente de subida")),
        "gradiente CS 23.65 (≈8,02%, abaixo do piso de 8,3%) deveria aparecer como violação \
         NOVA (lei única de tração, spec §11 — risco central do ciclo): {violations:#?}");
    // (4) Pouso na grama — ASSERÇÃO RELACIONAL QUE DEIXOU DE VALER,
    // `old→new` (ciclo 14, spec §2/§7): até o ciclo 13 esta violação
    // aparecia (`ldg_50ft_grass_m` ≈647 m > 600 m de pista). Ciclo 14
    // corrige os dois defeitos do segmento aéreo do pouso (flare sem
    // altura + ângulo de aproximação de ILS/3° em vez do planeio
    // power-off, γ_app=5,1181° derivado da polar): `ldg_50ft_grass_m`
    // 646,660942 m → **503,414253 m** (−22,2%), agora ABAIXO dos 600 m —
    // a checagem #24 FLIPA FAIL→PASS. A relação nova e verdadeira, viva,
    // no lugar: essa violação NÃO pode mais aparecer.
    assert!(!violations.iter().any(|v| v.contains("Pouso (grama, 15 m)")),
        "ciclo 14: a checagem #24 (pouso na grama sobre 15 m) deveria estar em PASS — \
         ldg_50ft_grass_m ≈503,4 m < 600 m de pista (γ_app derivado da polar + flare com \
         altura, spec §2): {violations:#?}");
    // (5) `old→new` (ciclo 13, task 2, spec §3.4/§11 — achado NOVO, a
    // projeção da spec ERROU a direção): `to_50ft_grass_m` AUMENTA
    // (819,110978→848,927019 m, +3,64%), não diminui como a spec §3.4
    // projetava (≈784,5 m). A rolagem pura de fato encolhe (FoM(J)≥0,75
    // sempre > constante 0,75 antigo em V∈[0,V_LOF]), mas o segmento de
    // SUBIDA usa V_climb≈38,6 m/s — MUITO além de V_LOF≈35,4 m/s, no regime
    // onde a tração cai ≈21% (mesmo efeito do gradiente CS 23.65 acima) — e
    // esse efeito domina o total. Registrar como achado de projeção errada
    // no relatório da task, não silenciar.
    // `old→new` (ciclo 13, task 3, ERRATUM §3.2.1): 848,927019→**≈859 m**
    // (mesma causa do pouso acima — `fom_design` recalibrado).
    assert!(violations.iter().any(|v| v.contains("Decolagem (grama")),
        "decolagem na grama (≈859 m, ciclo 13 — segmento de SUBIDA mais caro compensa a \
         rolagem mais barata, projeção da spec §3.4 errou a direção) deveria exceder os 600 m \
         de pista disponível: {violations:#?}");
    // Envelope de CG NOMINAL por cenário, `old→new` (ciclo 10 → ciclo 12,
    // task 4): `rotation_limit_pct_mac` era 13,354637% MAC (INALTERADO
    // pelas tasks 2/3) — os termos de solo do balanço de rotação (task 4)
    // recuam esse limite para 17,757974% MAC (+4,40 pp).
    //
    // `old→new` (ciclo 13, task 2): a lei única AFROUXA o limite —
    // `rotation_limit_pct_mac` (então PAVIMENTADO) 17,757974%→16,392661%
    // MAC (−1,365 pp). Naquele momento NENHUM dos 6 cenários cruzava o
    // limite NOMINAL.
    //
    // `old→new` (ciclo 13, task 4, spec §7/§11.1 — fecha o backlog #16):
    // `rotation_limit_pct_mac` passa a valer a superfície de OPERAÇÃO
    // (GRAMA) — 16,392661%→**18,268251% MAC** (+1,876 pp). 'Solo (piloto)'
    // (CG nominal ≈17,8% MAC) fica À FRENTE do novo limite: **1 dos 6
    // cenários agora cruza o envelope NOMINAL** — exatamente o cenário que
    // a spec §11.1 nomeou como possível. Os outros 5 seguem dentro.
    for cenario in ["2 pax dianteiros", "4 pax sem bagagem",
                    "4 pax + bagagem + cheio", "4 pax + bagagem + meia",
                    "4 pax + bagagem vazio"] {
        assert!(!violations.iter().any(|v|
            v.contains(cenario) && v.contains("fora do envelope de CG admissível")),
            "cenário '{cenario}' deveria estar DENTRO do envelope NOMINAL: {violations:#?}");
    }
    assert!(violations.iter().any(|v|
        v.contains("Solo (piloto)") && v.contains("fora do envelope de CG admissível")),
        "cenário 'Solo (piloto)' deveria estar FORA do envelope NOMINAL desde a task 4 \
         (superfície de grama no balanço de rotação, spec §7/§11.1): {violations:#?}");
    assert!(!json.contains("Envelope de CG VAZIO"),
        "não deveria haver violação dedicada de envelope de CG vazio:\n{json}");
    // Tipback, tail-strike e margem de combustível continuam PASSANDO,
    // com os MESMOS números do ciclo 10 (x_nose_m não afeta nenhum dos
    // três diretamente): tipback ≈16,79° (piso 15°), tail-strike ≈13,19°
    // (piso 11°), margem de combustível 9,14% da capacidade (piso 5%).
    assert!(!json.contains("Tipback:") && !json.contains("Tail-strike:")
        && !json.contains("Margem de combustível:"),
        "tipback/tail-strike/margem de combustível deveriam continuar sem violação:\n{json}");
    // O aviso elétrico de pico (não é violação) continua presente — só
    // confirma que o pipeline real ainda reporta avisos quando aplicável.
    assert!(json.contains("Orçamento elétrico:") && json.contains("banco de baterias"),
        "aviso elétrico de pico esperado (pico 1.260 W > alternador 900 W, não é violação):\n{json}");

    // Pin honesto de flutter — histórico em três passos, nenhum deles pinado
    // antes desta task: 749,55 (pré-ciclo-3) → 702,60 km/h no ciclo 3 (asa
    // computada mais pesada, −6,3%) → 698,82 km/h no ciclo 4 (t/c da
    // empenagem + W_dg de envelope, −0,54%). Centra no valor REAL medido
    // agora (698,82), não num valor histórico já superado. Piso regulatório
    // 1,2×VD = 420 km/h fica LONGE; o pin pega regressão de modelo, não
    // proximidade de limite.
    // Campanha E10 (2026-08-08): 698,82 → **698,50 km/h** (−0,05%, dentro
    // do ±1% do pin anterior; re-pinado mesmo assim). A asa fica um fio mais
    // pesada (W_dg de envelope maior) e o `n_design` cai de 4,201 para
    // 4,171 g — efeitos que quase se cancelam no flutter.
    let flutter = spec["structure"]["flutter_speed_kmh"].as_f64().unwrap();
    assert!((flutter - 698.5).abs() < 7.0, // ±1%, padrão dos pins de performance // PIN: structure.flutter_speed_kmh
        "flutter_speed_kmh = {flutter:.1} divergiu do pin honesto ≈698,5 (±1%)");

    let _ = std::fs::remove_file(&out_path);
}

/// Golden-file (recomendação adotada da revisão final, campanha E10): o
/// binário rodado com os TOMLs REAIS do repositório (mesma invocação do
/// teste PASS acima) deve produzir um JSON estruturalmente IDÊNTICO ao
/// `aircraft_spec.json` COMMITADO. Pega ARTEFATO STALE — o `aircraft_spec.
/// json` desatualizado em relação a `config/`/`src/` (aconteceu de verdade
/// entre os ciclos 7 e E10: o JSON commitado ficou uma rodada inteira sem
/// refletir os valores do baseline real, achado só na revisão).
///
/// ESCOLHA DE COMPARAÇÃO (documentada, achado do brief — "estruturalmente
/// ou bytes, escolha a mais robusta a float"): estrutural via
/// `serde_json::Value` (`PartialEq`), não byte-a-byte. Dois motivos:
///   1. `Value::Object`/`serde_json::Map` compara por conteúdo, não por
///      ORDEM de chave — não falsifica em uma mudança de formatação/ordem
///      inofensiva (o que uma comparação de bytes faria).
///   2. Ainda assim é EXATA para números: o crate depende de `serde_json`
///      com a feature `float_roundtrip` (ver `Cargo.toml`, comentário de
///      Task 6.1) especificamente para ida-e-volta f64 byte-exata através
///      do parser JSON. Como `size_aircraft`/os agentes são determinísticos
///      (mesma config, mesmo resultado), dois runs da MESMA config real
///      devem produzir f64s idênticos bit a bit — não há folga de
///      tolerância a definir aqui, ao contrário dos pins de
///      `tests/generic_engine.rs`/`tests/gear_tipback.rs` (que comparam o
///      pipeline real contra um valor ESPERADO escrito à mão).
#[test]
fn aircraft_spec_json_commitado_bate_com_o_pipeline_real() {
    let out_path = std::env::temp_dir().join(format!(
        "aeronave_cli_test_golden_file_{}.json",
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
        "binário deveria convergir com sucesso (mesma config real do teste PASS acima): \
         stderr={}", String::from_utf8_lossy(&output.stderr));

    let fresh_json = std::fs::read_to_string(&out_path)
        .unwrap_or_else(|e| panic!("falha ao ler '{}': {e}", out_path.display()));
    let fresh: serde_json::Value = serde_json::from_str(&fresh_json)
        .expect("JSON recém-gerado pelo binário deveria ser válido");
    let _ = std::fs::remove_file(&out_path);

    let committed_path = manifest_dir().join("aircraft_spec.json");
    let committed_json = std::fs::read_to_string(&committed_path)
        .unwrap_or_else(|e| panic!("falha ao ler '{}': {e}", committed_path.display()));
    let committed: serde_json::Value = serde_json::from_str(&committed_json)
        .expect("aircraft_spec.json commitado deveria ser JSON válido");

    if fresh != committed {
        let mut mismatches = Vec::new();
        diff_json(&fresh, &committed, "$", &mut mismatches);
        panic!(
            "aircraft_spec.json COMMITADO diverge do JSON que o pipeline real produz agora com \
             config/aircraft/baseline_4seat.toml + config/engines/toyota_1gd_ftv.toml + \
             config/missions/default.toml — artefato STALE (não regenerado após uma mudança de \
             config/src).\nRegenere com:\n  cargo run --release -- --engine \
             config/engines/toyota_1gd_ftv.toml --aircraft config/aircraft/baseline_4seat.toml \
             --mission config/missions/default.toml --out aircraft_spec.json\n\
             Divergências ({} no total, até 10 mostradas):\n{}",
            mismatches.len(),
            mismatches.iter().take(10).cloned().collect::<Vec<_>>().join("\n"),
        );
    }
}

/// Diff estrutural recursivo de dois `serde_json::Value` — só para produzir
/// a mensagem de erro legível do teste golden-file acima; a comparação que
/// decide PASS/FAIL é `fresh != committed` (structural `PartialEq`
/// completo, não este helper).
fn diff_json(fresh: &serde_json::Value, committed: &serde_json::Value, path: &str, out: &mut Vec<String>) {
    use serde_json::Value;
    match (fresh, committed) {
        (Value::Object(mf), Value::Object(mc)) => {
            let keys: std::collections::BTreeSet<&String> = mf.keys().chain(mc.keys()).collect();
            for key in keys {
                let child_path = format!("{path}.{key}");
                match (mf.get(key), mc.get(key)) {
                    (Some(vf), Some(vc)) => diff_json(vf, vc, &child_path, out),
                    (Some(_), None) => out.push(format!("{child_path}: presente só no JSON gerado agora")),
                    (None, Some(_)) => out.push(format!("{child_path}: presente só no JSON commitado")),
                    (None, None) => unreachable!(),
                }
            }
        }
        (Value::Array(af), Value::Array(ac)) if af.len() == ac.len() => {
            for (i, (vf, vc)) in af.iter().zip(ac.iter()).enumerate() {
                diff_json(vf, vc, &format!("{path}[{i}]"), out);
            }
        }
        _ if fresh != committed => out.push(format!("{path}: gerado agora={fresh} | commitado={committed}")),
        _ => {}
    }
}
