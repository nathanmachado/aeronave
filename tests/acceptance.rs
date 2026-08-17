//! Task 6.2 — Verificação final de genericidade (teste de aceitação).
//!
//! Fecha o pedido original do projeto: o modelo deve ser motor-agnóstico —
//! trocar de motor é trocar um arquivo TOML (`--engine`), nunca recompilar
//! ou editar `src/`. Este arquivo roda o binário real via `Command` (mesmo
//! padrão de `tests/cli.rs`, não chamadas internas) para provar isso
//! ponta-a-ponta com os dois motores de catálogo do projeto, e varre
//! `src/` em busca de qualquer nome de motor específico vazado no código.
//!
//! Desvio deliberado do texto literal do brief da Task 6.2 ("ambos
//! executam"): o Rotax 915 iS genuinely não sustenta a missão de projeto
//! completa (280 km/h / 8 h — precisa de ~393,3 L contra os 260 L do tanque;
//! ver `tests/cli.rs::engine_flag_troca_motor_e_rotax_falha_honestamente_por_combustivel`).
//! Essa é a resposta honesta do modelo, não um bug — então o critério de
//! aceitação vira: (a) Toyota roda a missão completa até um spec completo;
//! (b) Rotax produz um diagnóstico de inviabilidade limpo e correto em
//! português (saída 1, sem panic, sem JSON) na missão de projeto; (c) uma
//! SEGUNDA missão ("ferry", `config/missions/rotax_ferry.toml`) — dado
//! trocado, não código — prova que o Rotax GERA um spec completo quando a
//! missão está dentro do envelope físico do motor, demonstrando a
//! genericidade no espírito do pedido original (trocar dados → resultados
//! novos, sem tocar em código).

use std::path::{Path, PathBuf};
use std::process::Command;

fn bin() -> Command {
    Command::new(env!("CARGO_BIN_EXE_aeronave"))
}

fn manifest_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn tmp_out(tag: &str) -> PathBuf {
    std::env::temp_dir().join(format!(
        "aeronave_acceptance_{tag}_{}.json",
        std::process::id()
    ))
}

/// (a) Toyota + missão default: roda até o fim, produz um `AircraftReport`
/// v4 completo com o bloco de validação presente (`validation_status` +
/// `violations`/`warnings`) e o nome do motor correto no JSON.
#[test]
fn toyota_missao_default_gera_spec_completo_v4() {
    let out_path = tmp_out("toyota_default");

    let output = bin()
        .current_dir(manifest_dir())
        .arg("--engine").arg("config/engines/toyota_1gd_ftv.toml")
        .arg("--mission").arg("config/missions/default.toml")
        .arg("--out").arg(&out_path)
        .output()
        .expect("falha ao executar o binário aeronave");

    assert!(output.status.success(),
        "Toyota deveria convergir e sair com sucesso na missão de projeto completa: stderr={}",
        String::from_utf8_lossy(&output.stderr));

    let json_text = std::fs::read_to_string(&out_path)
        .unwrap_or_else(|e| panic!("falha ao ler '{}': {e}", out_path.display()));
    let json: serde_json::Value = serde_json::from_str(&json_text)
        .expect("saída deveria ser JSON válido");

    // `old→new` (ciclo 16, Task 5 — bump MAJOR): "5.7" → "6.0".
    assert_eq!(json["schema_version"], "6.0", "schema_version deveria ser 6.0:\n{json_text}");
    assert!(json["propulsion"]["engine_model"].as_str().unwrap_or_default().contains("Toyota"),
        "engine_model deveria conter 'Toyota':\n{json_text}");

    // Bloco de validação presente (independente do veredito ser PASS ou
    // FAIL — ver `tests/cli.rs::engine_padrao_explicito_com_out_tempfile_
    // converge_e_reporta_pass_honesto` para o veredito honesto atual do
    // baseline real, pós-campanha-E7 — "presente" aqui significa
    // estrutural, não um veredito específico).
    let status = json["validation_status"].as_str()
        .expect("validation_status deveria ser uma string presente no JSON");
    assert!(status == "PASS" || status == "FAIL",
        "validation_status deveria ser PASS ou FAIL, veio: {status:?}");
    assert!(json["violations"].is_array(), "bloco de violations deveria estar presente");
    assert!(json["warnings"].is_array(), "bloco de warnings deveria estar presente");

    let _ = std::fs::remove_file(&out_path);
}

/// (b) Rotax + missão default (projeto completo): diagnóstico de
/// inviabilidade honesto — saída != 0, mensagem em português sobre
/// combustível, sem panic/backtrace, sem JSON escrito.
#[test]
fn rotax_missao_default_falha_honestamente_sem_json() {
    let out_path = tmp_out("rotax_default");

    let output = bin()
        .current_dir(manifest_dir())
        .arg("--engine").arg("config/engines/rotax_915is.toml")
        .arg("--mission").arg("config/missions/default.toml")
        .arg("--out").arg(&out_path)
        .output()
        .expect("falha ao executar o binário aeronave");

    assert!(!output.status.success(),
        "Rotax na missão completa (280km/h/8h) deveria sair com erro — motor não sustenta a missão");

    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(stderr.contains("combustível") || stderr.contains("Combustível"),
        "stderr deveria conter uma mensagem em português sobre combustível insuficiente: {stderr}");
    assert!(!stderr.contains("panicked"), "erro de sizing não deveria gerar panic: {stderr}");
    assert!(!out_path.exists(),
        "JSON de saída não deveria ser escrito quando o sizing falha antes dos demais agentes");

    let _ = std::fs::remove_file(&out_path);
}

/// (c) Rotax + missão "ferry" (`config/missions/rotax_ferry.toml`): mesmo
/// motor problemático acima, missão reduzida (dado trocado, não código) —
/// gera um spec completo (saída 0), com o nome do motor correto e
/// MTOW/combustível fixados (±1%) contra o ponto de convergência real
/// verificado por execução antes deste teste ser escrito (ver comentário
/// no TOML da missão). Prova a genericidade: o MESMO binário, mudando só
/// arquivos de configuração, produz um segundo spec completo e coerente.
#[test]
fn rotax_missao_ferry_gera_spec_completo_data_driven() {
    let out_path = tmp_out("rotax_ferry");

    let output = bin()
        .current_dir(manifest_dir())
        .arg("--engine").arg("config/engines/rotax_915is.toml")
        .arg("--mission").arg("config/missions/rotax_ferry.toml")
        .arg("--out").arg(&out_path)
        .output()
        .expect("falha ao executar o binário aeronave");

    assert!(output.status.success(),
        "Rotax na missão ferry (200km/h/3h, 2 pax) deveria convergir e sair com sucesso: stderr={}",
        String::from_utf8_lossy(&output.stderr));

    let json_text = std::fs::read_to_string(&out_path)
        .unwrap_or_else(|e| panic!("falha ao ler '{}': {e}", out_path.display()));
    let json: serde_json::Value = serde_json::from_str(&json_text)
        .expect("saída deveria ser JSON válido");

    // `old→new` (ciclo 16, Task 5 — bump MAJOR): "5.7" → "6.0".
    assert_eq!(json["schema_version"], "6.0");
    assert_eq!(json["propulsion"]["engine_model"], "Rotax 915 iS",
        "engine_model deveria ser exatamente 'Rotax 915 iS':\n{json_text}");

    // Pins do ponto de convergência real (verificado por `cargo run` antes
    // de escrever este teste — ver comentário em
    // `config/missions/rotax_ferry.toml`). Tolerância de ±1% para
    // acomodar pequenas variações de ponto flutuante entre plataformas,
    // sem mascarar uma mudança de comportamento real do sizing.
    let mtow_mission = json["sizing"]["mtow_mission_kg"].as_f64()
        .expect("sizing.mtow_mission_kg deveria estar presente");
    let fuel_required = json["sizing"]["fuel_required_l"].as_f64()
        .expect("sizing.fuel_required_l deveria estar presente");

    // Campanha E1–E6 (2026-08-05): mesma config de aeronave (agora com
    // trem recuado, EH maior, bateria/bagageiro realocados — ver
    // `config/aircraft/baseline_4seat.toml`) desloca levemente estes pins.
    // mtow_mission_kg: 1.025,2 → 1.031,3 kg (+0,6%, dentro da tolerância
    // ±1% de antes, mas atualizado para refletir o valor real medido).
    // fuel_required_l: 71,1 → 72,7 L (+2,2%, mais CD0 do empennage).
    //
    // Task 4 (refino-ciclo2, arrasto de trim em cruzeiro): achado honesto
    // — o Rotax (84 kg) é muito mais leve que o Toyota (195 kg) para o
    // qual esta célula foi dimensionada; sem o motor pesado no nariz, o CG
    // de meia-missão fica bem mais ATRÁS, exigindo um CL_h_trim de cruzeiro
    // maior e portanto um ΔCD_trim maior (NÃO desprezível, ao contrário do
    // caso Toyota). Pins da época: mtow_mission_kg 1.031,3 → 1.032,6 kg;
    // fuel_required_l 72,7 → 74,4 L.
    //
    // Ciclo 3 (oew-parametrico, Task 4): as 7 massas estruturais do OEW
    // passaram a ser COMPUTADAS (`agents::mass_model`, Raymer cap. 15.2)
    // em função do MTOW candidato, em vez de itens FIXOS de
    // `[[masses.items]]`. Nesta missão o efeito é MUITO maior que no
    // baseline Toyota, e por um motivo físico claro: com o Rotax e só 2
    // pax, o MTOW de missão é bem menor (~915 kg vs ~1.033 kg), e as
    // equações de Raymer escalam a estrutura COM o peso de projeto — o que
    // antes era uma célula de massa estrutural congelada (422 kg,
    // dimensionada para o Toyota) agora encolhe junto com a aeronave. O
    // laço realimenta: estrutura mais leve → MTOW menor → estrutura ainda
    // mais leve. OEW 779,0 → **665,5 kg**; mtow_mission_kg 1.032,6 →
    // **914,9 kg**; fuel_required_l 74,4 → **68,5 L** (old→new,
    // TOLERÂNCIAS INALTERADAS).
    //
    // Ciclo 4 (Task 2, W_dg = MTOW de envelope com lag-1): `MassModelAgent
    // ::run` passou a receber o MTOW de ENVELOPE (`wb.spec.mtow_kg`,
    // cenário fixo "4 pax + bagagem + tanque cheio", ~1.351 kg nesta
    // missão) em vez do candidato de MISSÃO desta iteração (`mtow`, que
    // para a missão ferry — só 2 pax, 3h — convergia bem mais leve,
    // ~915 kg). INVESTIGADO (achado de revisão, salto >5%): esta missão é
    // exatamente o caso em que a diferença entre os dois MTOWs é maior —
    // a missão ferry usa metade dos passageiros e um terço da autonomia da
    // missão de projeto, então seu candidato de MTOW ficava bem abaixo do
    // envelope estrutural. Antes desta task a estrutura era dimensionada
    // (incorretamente) para o peso LEVE da missão específica sendo voada;
    // agora é dimensionada (corretamente, consistente com `StructuralAgent`
    // /`LandingGearAgent`) para o peso do PIOR CASO estrutural (4 pax +
    // bagagem + tanque cheio), que não muda com a missão. O salto grande
    // (+6,6% no MTOW de missão) é justamente a estrutura deixando de ser
    // subdimensionada para esta missão leve — não uma regressão. OEW não
    // exposto no JSON desta missão; mtow_mission_kg 914,9 → **975,2 kg**;
    // fuel_required_l 68,5 → **71,0 L** (old→new, TOLERÂNCIAS INALTERADAS).
    //
    // Campanha E10 (2026-08-08): esta missão compartilha a CÉLULA do
    // baseline (`config/aircraft/baseline_4seat.toml`), então herda os +25
    // kg de bateria híbrida e a hélice Ø1,95→1,76 m — mas NÃO o motor
    // (Rotax) nem a missão (ferry, 2 pax/3h). Efeito: OEW sobe (bateria
    // pesada, parcialmente compensada pelo trem mais curto) e o laço
    // realimenta com a hélice menos eficiente. mtow_mission_kg 975,2 →
    // **991,3 kg** (+1,65%); fuel_required_l 71,0 → **71,4 L** (+0,56% —
    // dentro do ±1% do pin antigo, re-pinado mesmo assim por honestidade).
    // TOLERÂNCIAS INALTERADAS (±1%).
    //
    // Ciclo 13 (task 2, lei única de tração — spec §2/§4): `prop_efficiency
    // (J)` (polinômio JavaProp) apagado, substituído por `FigureOfMerit`,
    // retro-derivada do ponto de CRUZEIRO DO TOYOTA (J≈1,875, spec §3.2) —
    // premissa calibrada declarada (spec §3.3). A missão ferry usa o MESMO
    // conjunto hélice/PSRU (config da CÉLULA, `baseline_4seat.toml`) mas o
    // Rotax cruza a J≈0,72, bem ABAIXO do ponto de calibração: medido,
    // eta_cruzeiro cai de ≈0,7836 (polinômio no mesmo J) para **0,74475**
    // (FoM(J=0,72)≈0,778, e eta=FoM·V/u < FoM por construção — spec §5) —
    // -4,95% de eficiência propulsiva, mais potência requerida, mais
    // combustível. `old→new`: mtow_mission_kg 991,3 → **994,067254 kg**
    // (+0,28%); fuel_required_l 71,4 → **75,218966 L** (+5,35% — o maior
    // deslocamento de pin do ciclo, fora do envelope de 5% da spec §11
    // porque este pin NÃO está na tabela da spec §11, que só projeta o
    // baseline Toyota; verificado por sonda direta — J baixo é a causa
    // única, não há anomalia). TOLERÂNCIAS INALTERADAS (±1%).
    let mtow_expected = 994.067254_f64; // PIN: NAO-PUBLICADO — cenário Rotax + missão ferry, não o par Toyota+default que gera o aircraft_spec.json commitado
    let fuel_expected = 75.218966_f64; // PIN: NAO-PUBLICADO — idem

    assert!((mtow_mission - mtow_expected).abs() / mtow_expected < 0.01,
        "MTOW de missão convergido ({mtow_mission:.1}kg) deveria estar a ±1% de {mtow_expected}kg \
         (mudou o comportamento do sizing? atualize o pin e o comentário no TOML da missão)");
    assert!((fuel_required - fuel_expected).abs() / fuel_expected < 0.01,
        "Combustível requerido ({fuel_required:.1}L) deveria estar a ±1% de {fuel_expected}L");

    // Margem de tanque folgada (missão escolhida deliberadamente com folga,
    // não um caso-limite) — guarda-corpo de sanidade, não um pin fino.
    let fuel_capacity = json["sizing"]["fuel_capacity_l"].as_f64().unwrap();
    assert!(fuel_required < fuel_capacity,
        "combustível requerido ({fuel_required:.1}L) deveria caber no tanque ({fuel_capacity:.1}L)");

    // Margem mínima de combustível (Task 3, refino-ciclo2): a missão ferry
    // (`config/missions/rotax_ferry.toml`, `min_fuel_margin_fraction =
    // 0.05`) foi escolhida deliberadamente com folga grande — verificado
    // por execução real antes de fixar este pin: margem ≈72,0% da
    // capacidade do tanque, MUITO acima do piso de 5%. Não deveria gerar a
    // violação nova de margem de combustível (checagem #18 de
    // `ConstraintChecker::verify`). Histórico: até a campanha E7
    // (2026-08-06), isso era diferente do que acontecia na missão de
    // projeto completa (`config/missions/default.toml`, margem ≈1,58%,
    // abaixo do piso de 5%) — desde E7 (`endurance_min_h` 8h→7h), a missão
    // de projeto também fica bem acima do piso (≈13,97%, ver
    // `tests/cli.rs::engine_padrao_explicito_com_out_tempfile_converge_e_
    // reporta_pass_honesto`) — este teste (missão ferry) continua sendo o
    // caso com a MAIOR folga das duas missões, não mais o único sem
    // violação.
    let fuel_margin_pct = json["sizing"]["fuel_margin_pct"].as_f64()
        .expect("sizing.fuel_margin_pct deveria estar presente");
    // Task 4 (refino-ciclo2, arrasto de trim em cruzeiro): 72,04% → 71,39%
    // — ver comentário acima sobre o ΔCD_trim maior do Rotax nesta célula.
    // Ciclo 3 (oew-parametrico, Task 4): 71,39% → 73,65% (old→new) —
    // aeronave mais leve exige menos combustível para a mesma missão, e a
    // capacidade do tanque não mudou. Continua MUITO acima do piso de 5%
    // — nenhuma violação nova.
    // Ciclo 4 (Task 2, W_dg de envelope): 73,65% → **72,71%** (old→new) —
    // estrutura mais pesada (dimensionada para o envelope, ver comentário
    // acima) consome um pouco mais de combustível para a mesma missão;
    // ainda MUITO acima do piso de 5% — nenhuma violação nova.
    // Ciclo 13 (task 2, lei única de tração): mais combustível requerido
    // (ver comentário `mtow_expected`/`fuel_expected` acima, +5,35%) come
    // parte da folga do tanque: 72,71% → **71,069629%** (old→new,
    // −1,64 pp). Continua MUITO acima do piso de 5% — nenhuma violação
    // nova.
    let fuel_margin_pct_expected = 71.069629_f64; // PIN: NAO-PUBLICADO — cenário Rotax + missão ferry, não o par Toyota+default que gera o aircraft_spec.json commitado
    assert!((fuel_margin_pct - fuel_margin_pct_expected).abs() < 1.0,
        "margem de combustível ({fuel_margin_pct:.2}%) deveria estar próxima do pin honesto \
         ~{fuel_margin_pct_expected}% (mudou o comportamento do sizing? atualize o pin)");
    let violations = json["violations"].as_array()
        .expect("violations deveria ser um array presente");
    assert!(!violations.iter().any(|v| v.as_str().unwrap_or_default().contains("Margem de combustível")),
        "não deveria haver violação de margem de combustível na missão ferry (folga ~72%): \
         {violations:?}");

    let _ = std::fs::remove_file(&out_path);
}

/// (d) `src/` não deve conter nomes de motor específicos em lugar nenhum —
/// nem em código de produção, nem em comentários, nem em nomes de testes.
/// Equivalente ao `grep -rniE "toyota|rotax|1gd|915" src/` do brief, mas
/// escrito em Rust para rodar como parte da suíte (e para tratar "915" com
/// cuidado, ver `contains_standalone_915_token`).
///
/// Motores concretos vivem só em `config/engines/*.toml` — o código em
/// `src/` deve enxergar apenas o `EngineSpec` genérico
/// (`models::engine::EngineSpec`), nunca um nome de catálogo.
#[test]
fn src_nao_contem_nomes_de_motor_especificos() {
    let src_dir = manifest_dir().join("src");
    let files = collect_rs_files(&src_dir);
    assert!(!files.is_empty(), "esperava encontrar arquivos .rs em {}", src_dir.display());

    let mut hits: Vec<String> = Vec::new();
    for path in &files {
        let content = std::fs::read_to_string(path)
            .unwrap_or_else(|e| panic!("falha ao ler '{}': {e}", path.display()));
        for (idx, line) in content.lines().enumerate() {
            let lower = line.to_lowercase();
            let mut terms: Vec<&str> = Vec::new();
            if lower.contains("toyota") { terms.push("toyota"); }
            if lower.contains("rotax") { terms.push("rotax"); }
            if lower.contains("1gd") { terms.push("1gd"); }
            if contains_standalone_915_token(line) { terms.push("915"); }
            if !terms.is_empty() {
                let rel = path.strip_prefix(&manifest_dir()).unwrap_or(path);
                hits.push(format!(
                    "{}:{}: [{}] {}",
                    rel.display(), idx + 1, terms.join(","), line.trim()
                ));
            }
        }
    }

    assert!(hits.is_empty(),
        "src/ não deveria conter nomes de motor específicos (toyota/rotax/1gd/915 como token) \
         — isso quebra a genericidade motor-agnóstica do modelo (motores concretos devem viver \
         só em config/engines/*.toml). Ocorrências encontradas:\n{}",
        hits.join("\n"));
}

/// "915" só conta como ocorrência quando aparece como TOKEN isolado — os
/// caracteres imediatamente antes/depois (se existirem) não podem ser
/// dígito ASCII nem '.'. Isso evita falsos positivos em números de ponto
/// flutuante (`0.915`, `2.9150`) ou outros literais numéricos de 4+ dígitos
/// que apenas contêm "915" como substring incidental, mas ainda pega
/// `915is`, `rotax-915`, `915 iS` etc.
fn contains_standalone_915_token(line: &str) -> bool {
    let bytes = line.as_bytes();
    let needle = b"915";
    if bytes.len() < needle.len() {
        return false;
    }
    for i in 0..=(bytes.len() - needle.len()) {
        if &bytes[i..i + needle.len()] != needle {
            continue;
        }
        let before_ok = i == 0 || {
            let c = bytes[i - 1];
            !(c.is_ascii_digit() || c == b'.')
        };
        let after_idx = i + needle.len();
        let after_ok = after_idx == bytes.len() || {
            let c = bytes[after_idx];
            !(c.is_ascii_digit() || c == b'.')
        };
        if before_ok && after_ok {
            return true;
        }
    }
    false
}

/// Varredura recursiva simples de `*.rs` sob `dir` (sem dependências
/// externas — só `std::fs`, mesmo padrão de robustez dos demais testes
/// deste projeto).
fn collect_rs_files(dir: &Path) -> Vec<PathBuf> {
    let mut out = Vec::new();
    let entries = match std::fs::read_dir(dir) {
        Ok(e) => e,
        Err(_) => return out,
    };
    for entry in entries.flatten() {
        let path = entry.path();
        if path.is_dir() {
            out.extend(collect_rs_files(&path));
        } else if path.extension().and_then(|e| e.to_str()) == Some("rs") {
            out.push(path);
        }
    }
    out
}

/// Fixtures do genericidade: os dois TOMLs de motor + o symlink `default`
/// devem existir (pré-condição óbvia para os testes acima, mas checada à
/// parte para um diagnóstico direto se algum arquivo sumir).
#[test]
fn fixtures_de_motor_existem() {
    let dir = manifest_dir().join("config/engines");
    assert!(dir.join("toyota_1gd_ftv.toml").is_file(), "config/engines/toyota_1gd_ftv.toml deveria existir");
    assert!(dir.join("rotax_915is.toml").is_file(), "config/engines/rotax_915is.toml deveria existir");
    let default_path = dir.join("default.toml");
    let meta = std::fs::symlink_metadata(&default_path)
        .unwrap_or_else(|e| panic!("config/engines/default.toml deveria existir: {e}"));
    assert!(meta.file_type().is_symlink(),
        "config/engines/default.toml deveria ser um symlink (não um arquivo comum)");
    assert!(manifest_dir().join("config/missions/rotax_ferry.toml").is_file(),
        "config/missions/rotax_ferry.toml (caso de aceitação de genericidade) deveria existir");
}
