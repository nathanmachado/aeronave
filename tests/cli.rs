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
///      conjuntos adversariais. O limite de rotação praticamente não se
///      move (8,908% → 8,533%): `to_flap_fraction` 0,5→0,35 compensa quase
///      exatamente o `cl_max_flaps` 1,72→2,1 no `cl_max_to`
///      (1,585 → 1,6775).
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
///     pista de 600 m).
/// O aviso elétrico de pico (1.260 W > alternador 900 W) PERMANECE — é
/// aviso, não violação, e agora é coberto pelo banco de baterias de 53 kg
/// que a própria E10 instalou (ver comentário do item de massa no TOML).
#[test]
fn engine_padrao_explicito_com_out_tempfile_reporta_pass_sem_violacoes() {
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
    // Campanha E10 (ver a ATUALIZAÇÃO na docstring): validation_status é
    // PASS — primeiro PASS completo do projeto (24 checks + robustez a 3
    // mundos), sem nenhuma violação.
    assert!(json.contains("\"validation_status\": \"PASS\""),
        "JSON de saída deveria reportar validation_status PASS (campanha E10 — ver comentário \
         acima):\n{json}");
    let spec: serde_json::Value = serde_json::from_str(&json)
        .expect("saída deveria ser JSON válido");
    let violations: Vec<String> = spec["violations"].as_array()
        .expect("violations deveria ser um array presente")
        .iter().map(|v| v.as_str().unwrap_or_default().to_string()).collect();
    // Contagem 4 → **0** (campanha E10). Assert de contagem PRIMEIRO, com
    // a lista inteira na mensagem: qualquer violação nova aparece por
    // nome no output do teste, sem precisar adivinhar qual foi.
    assert!(violations.is_empty(),
        "campanha E10: esperava ZERO violações no baseline real (eram 4 no ciclo 7): \
         {violations:#?}");
    // Asserts NOMEADOS por violação fechada — redundantes com a contagem
    // acima de propósito: se um refactor um dia trocar o texto de uma
    // violação, a contagem sozinha não diria QUAL regrediu.
    //
    // (1) Carga de nariz: 28,6% → 22,77%, abaixo do teto de 25%.
    assert!(!violations.iter().any(|v| v.contains("Carga de nariz:")),
        "carga de nariz (≈22,77%) deveria estar abaixo do teto de 25% desde a E10 \
         (x_nose_m 1,40→1,30 + bateria de 53 kg a 7,80 m): {violations:#?}");
    // (2/3) Robustez (#19): 2 flips → 0. Os cenários 'Solo (piloto)' e
    // '2 pax dianteiros' recuaram de 9,1%/12,5% para 17,9%/20,5% MAC e
    // agora sobrevivem a ±15% de massa estrutural nos dois mundos.
    assert!(!violations.iter().any(|v| v.starts_with("Robustez:")),
        "campanha E10: esperava ZERO violações de robustez (σ=15%) — eram 2 ('Solo (piloto)' e \
         '2 pax dianteiros') no ciclo 7: {violations:#?}");
    // (4) Pouso na GRAMA sobre 15 m: 605,0 m → 556,7 m, dentro dos 600 m
    // da pista de fazenda, por `cl_max_flaps` 1,72→2,1 (flap SLOTTED:
    // VS0 113,3→103,4 km/h, distância ∝ V_ref²).
    assert!(!violations.iter().any(|v| v.contains("Pouso (grama, 15 m)")),
        "pouso na grama (≈556,7 m pós-E10, era ≈605,0 m) deveria caber nos 600 m: \
         {violations:#?}");
    // A decolagem na grama continua PASSANDO — E10 a alonga um pouco
    // (457,7 → 469,3 m sobre 15 m: hélice menor + MTOW maior superam o
    // `cl_max_to` maior), folga remanescente ≈131 m nos 600 m.
    assert!(!violations.iter().any(|v| v.contains("Decolagem (grama")),
        "decolagem na grama (≈469,3 m pós-E10, era ≈457,7 m) deveria continuar dentro dos \
         600 m: {violations:#?}");
    // Nenhum dos 6 cenários de carga fora do envelope admissível — nem os
    // dianteiros (que E10 recuou) nem os traseiros (que E10 aproximou do
    // limite: o pior, '4 pax + bagagem + cheio', vai a 38,8% MAC contra um
    // limite traseiro de 43,5%).
    for cenario in ["Solo (piloto)", "2 pax dianteiros", "4 pax sem bagagem",
                    "4 pax + bagagem + cheio", "4 pax + bagagem + meia",
                    "4 pax + bagagem vazio"] {
        assert!(!violations.iter().any(|v| v.contains(cenario)),
            "cenário '{cenario}' deveria estar DENTRO do envelope: {violations:#?}");
    }
    assert!(!json.contains("Envelope de CG VAZIO"),
        "não deveria haver violação dedicada de envelope de CG vazio:\n{json}");
    // Tipback, tail-strike e margem de combustível continuam PASSANDO,
    // agora com menos folga que no ciclo 4 (custo honesto de E10): tipback
    // 18,85°→16,74° (piso 15°) e margem de combustível 14,33%→9,14% da
    // capacidade (piso 5%).
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
    assert!((flutter - 698.5).abs() < 7.0, // ±1%, padrão dos pins de performance
        "flutter_speed_kmh = {flutter:.1} divergiu do pin honesto ≈698,5 (±1%)");

    let _ = std::fs::remove_file(&out_path);
}
