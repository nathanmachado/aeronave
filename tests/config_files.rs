//! Testes de carregamento de arquivos de configuração reais do disco
//! (`config/engines/*.toml`, `config/aircraft/*.toml`, `config/missions/*.toml`).
//!
//! Vivem em `tests/` (crate de teste separada), não em `src/models/config.rs`
//! — `src/` deve permanecer livre de qualquer menção a um motor real
//! específico (ver grep de regressão no relatório da Task 1.4/2.3, e
//! `tests/generic_engine.rs`). Os testes de parse/validação com TOML inline
//! (sem tocar disco, sem nomear arquivos reais) continuam em
//! `src/models/config.rs`.

use std::path::PathBuf;

use aeronave::models::config::{load_aircraft, load_engine, load_mission};

fn config_path(rel: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(rel)
}

#[test]
fn carrega_os_dois_motores_do_disco() {
    let toyota = load_engine(&config_path("config/engines/toyota_1gd_ftv.toml")).unwrap();
    let rotax = load_engine(&config_path("config/engines/rotax_915is.toml")).unwrap();
    assert!((toyota.torque_nm(2_400.0) - 500.0).abs() < 1.0);
    assert!((rotax.power_kw(5_800.0) - 67.4).abs() < 3.0); // 111 Nm @ 5800 ≈ 67 kW
    assert!(toyota.fuel.density_kg_per_l < rotax.fuel.density_kg_per_l + 1.0);
}

#[test]
fn carrega_baseline_do_disco_campo_a_campo() {
    let cfg = load_aircraft(&config_path("config/aircraft/baseline_4seat.toml")).unwrap();
    assert_eq!(cfg.sizing.mtow_initial_guess_kg, 1461.0);
    assert_eq!(cfg.sizing.mtow_max_kg, 1800.0);
    assert_eq!(cfg.wing.span_m, 11.94);
    assert_eq!(cfg.wing.area_m2, 14.2);
    assert_eq!(cfg.wing.taper_ratio, 0.45);
    assert_eq!(cfg.wing.airfoil, "NACA 23015");
    assert_eq!(cfg.wing.cl_max_clean, 1.45);
    // Campanha E10 (2026-08-08): 1.72 → 2.1 — flap SLOTTED (ver comentário no
    // TOML); é o que tira o pouso na grama sobre 15 m da violação (605→557 m).
    assert_eq!(cfg.wing.cl_max_flaps, 2.1);
    assert_eq!(cfg.wing.le_root_x_m, 2.90);
    assert_eq!(cfg.propeller.psru_ratio, 1.867);
    // Campanha E10 (2026-08-08): 1.95 → 1.76 — consequência obrigatória do
    // trem curto (h_cg_ground_m 1.05→0.92 consome folga de hélice 1:1).
    assert_eq!(cfg.propeller.diameter_m, Some(1.76));
    // Ciclo 5 (task 1): shaft_height_m virou datum derivado — pin o offset
    // fixo (0.20). Campanha E10: shaft derivado = 0.92 + 0.20 = 1.12 (era
    // 1.05 + 0.20 = 1.25); o OFFSET em si não muda.
    assert_eq!(cfg.propeller.prop_axis_above_cg_m, 0.20);
    assert_eq!(cfg.propeller.tip_mach_max_static, 0.85);
    assert_eq!(cfg.propeller.tip_mach_max_cruise, 0.80);
    assert_eq!(cfg.propeller.ground_clearance_min_m, 0.23);
    // 260 L desde a correção pós-Task-3.1 (era 240 L; margem de 16,08 L
    // (~6,6%) sobre os 243,92 L exigidos pela missão no MTOW convergido
    // (~1.529,9 kg) — ver task-3.1-report.md).
    assert_eq!(cfg.fuel_system.capacity_l, 260.0);
    // Campanha E10 (2026-08-08): h_cg_ground_m 1.05 → 0.92 (trem curto,
    // tipback robusto) e x_nose_m 1.40 → 1.30 (dilui a carga de nariz, que
    // era 28,6% > teto de 25%). Ver comentários no TOML.
    assert_eq!(cfg.gear.h_cg_ground_m, 0.92);
    assert_eq!(cfg.gear.x_nose_m, 1.30);
    // Campanha E1–E6 (2026-08-05): 3.85 → 3.55 — trem principal recuado,
    // causa raiz do fechamento do envelope de CG (ver comentário no TOML).
    // Campanha E7 (2026-08-06): 3.55 → 3.66 — fecha o tipback do baseline
    // real (ver comentário no TOML).
    assert_eq!(cfg.gear.x_main_m, 3.66);
    assert_eq!(cfg.arms.engine_cg_m, 0.65);
    assert_eq!(cfg.arms.empennage_cg_m, 7.40);
    assert_eq!(cfg.structure.spar_material, "AA7075-T6");
    assert_eq!(cfg.structure.frame_spacing_mm, 300.0);
    // Campanha E1–E6: 15 → 16 itens — novo item "bateria_recolocada" (28kg,
    // bateria realocada do painel para o cone de cauda, mudança 2). Task
    // refino-ciclo2 (1b): 16 → 14 itens — "emp_horizontal"/"emp_vertical"
    // REMOVIDOS de [[masses.items]]. Ciclo 3 (oew-parametrico): 14 → 9
    // itens — as 5 massas estruturais restantes ("asa", "fuselagem",
    // "trem_principal", "trem_nariz", "tanques") também saíram, agora
    // COMPUTADAS por `agents::mass_model` (Raymer cap. 15.2) e injetadas
    // por `weight_balance::oew_items`. Sobraram só os NÃO-estruturais.
    assert_eq!(cfg.masses.items.len(), 9);
    // Campanha E1–E6: novos/alterados itens de massa — cobertura direta
    // dos valores do brief (mudanças 2 e 8).
    assert_eq!(cfg.masses.item_mass("avionicos"), Some(32.0));
    // Campanha E10 (2026-08-08): 28.0 → 53.0 kg — passa a ser o banco de
    // baterias da diretriz HÍBRIDA (~3 kWh úteis) e ganha arm_offset_m=0.4
    // (braço efetivo 7.40 → 7.80 m); os +25 kg no cone de cauda recuam o CG
    // vazio e é isso que tira a carga de nariz do teto de 25%.
    assert_eq!(cfg.masses.item_mass("bateria_recolocada"), Some(53.0));
    // Nenhum dos 7 nomes estruturais pode aparecer aqui (erro de migração
    // no parse — ver `models::config::check_structural_mass_items_
    // migration`; a cobertura da massa COMPUTADA está em
    // `agents::weight_balance::tests::oew_items_usa_as_massas_computadas_
    // com_o_mapeamento_estatico_de_bracos` e no baseline real).
    for nome in ["asa", "fuselagem", "emp_horizontal", "emp_vertical",
                 "trem_principal", "trem_nariz", "tanques"] {
        assert_eq!(cfg.masses.item_mass(nome), None,
            "item estrutural '{nome}' não deveria mais existir em [[masses.items]]");
    }
    assert_eq!(cfg.empennage.cd0_area_factor, 0.014366);
    // Task 4 (refino-ciclo2): eficiência de Oswald da empenagem horizontal
    // — usada em `agents::trim_authority::cd_trim_cruise` (arrasto de trim
    // em cruzeiro).
    assert_eq!(cfg.empennage.e_h, 0.70);
    // Campanha E10 (2026-08-08): pins novos para os campos restantes que a
    // campanha alterou e que este teste "campo a campo" ainda não cobria —
    // sem eles uma reversão silenciosa de E10 passaria despercebida aqui.
    //   to_flap_fraction 0.5 → 0.35 (decolagem com flap parcial: menos ΔCm de
    //     rotação com o flap slotted mais potente)
    //   main/nose_strut_length_m 0.67/0.53 → 0.54/0.40 (coerência dimensional
    //     com h_cg_ground_m −13 cm)
    //   cm_flap_delta permanece −0.30 (slotted moderado, INALTERADO em E10)
    assert_eq!(cfg.stability.to_flap_fraction, 0.35);
    assert_eq!(cfg.mass_model.main_strut_length_m, 0.54);
    assert_eq!(cfg.mass_model.nose_strut_length_m, 0.40);
    assert_eq!(cfg.wing.cm_flap_delta, -0.30);
}

#[test]
fn carrega_missao_default_do_disco_campo_a_campo() {
    let req = load_mission(&config_path("config/missions/default.toml")).unwrap();
    assert_eq!(req.passengers, 4);
    assert_eq!(req.pax_mass_kg, 90.0);
    assert_eq!(req.baggage_kg, 80.0);
    assert_eq!(req.cruise_speed_min_kmh, 280.0);
    // Campanha E7 (2026-08-06): 8.0 → 7.0 — decisão de requisito do cliente
    // (autonomia 7h + reserva, ver comentário no TOML).
    assert_eq!(req.endurance_min_h, 7.0);
    assert_eq!(req.fuel_reserve_fraction, 0.10);
    assert_eq!(req.cruise_altitude_m, 2500.0);
    assert_eq!(req.airfield_altitude_m, 0.0);
    assert_eq!(req.isa_delta_c, 0.0);
    assert_eq!(req.min_fuel_margin_fraction, 0.05);
    // Ciclo 6 (task 2): premissa de pista de fazenda — 600 m disponíveis,
    // gate dos checks #23 (decolagem na grama sobre 15 m) e #24 (pouso na
    // grama sobre 15 m).
    assert_eq!(req.runway_available_m, 600.0);
    assert_eq!(req.analysis.taxi_fuel_l, 3.0);
    assert_eq!(req.analysis.descent_rate_ms, 4.0);
    assert_eq!(req.analysis.descent_power_fraction, 0.20);
    assert_eq!(req.analysis.climb_speed_policy, "vy");
}
