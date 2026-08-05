/// EmpennageAgent — Dimensionamento da Empenagem (Task 4.1, resolve M1)
///
/// Dimensiona a área da empenagem horizontal (S_h) e vertical (S_v) pelo
/// método do coeficiente de volume de cauda (Raymer, "Aircraft Design: A
/// Conceptual Approach", cap. 6, Tab. 6.4 — valores típicos para monomotor
/// GA), a partir da asa já dimensionada (`AerodynamicsAgent`) e dos
/// coeficientes de `[empennage]` em `AircraftConfig`:
///
///   S_h = V_h · S_w · MAC / l_h
///   S_v = V_v · S_w · b   / l_v
///
/// onde V_h, V_v são os coeficientes de volume (adimensionais), S_w a área
/// da asa, MAC a corda aerodinâmica média da asa, b a envergadura da asa, e
/// l_h = l_v = `[empennage].tail_arm_m` o braço da empenagem (CA asa → CA
/// empenagem — mesmo braço para H e V nesta modelagem, configuração
/// convencional com deriva/estabilizador no mesmo cone de cauda).
///
/// Geometria de cada superfície (trapezoidal, mesmo modelo da asa):
///   b_surf   = √(AR_surf · S_surf)
///   c_raiz   = 2·S_surf / (b_surf·(1 + λ_surf))
///   c_ponta  = λ_surf · c_raiz
///
/// Este agente é puramente geométrico (não depende de peso/MTOW) — pode ser
/// calculado uma única vez por iteração do laço de convergência, logo após
/// `AerodynamicsAgent` (ver `orchestrator::size_aircraft`), e sua saída
/// (`EmpennageSpec`) alimenta `weight_balance::neutral_point_m`.
use crate::agents::weight_balance::{chord_root, mean_aerodynamic_chord};
use crate::models::{aircraft_config::AircraftConfig, specs::{EmpennageSpec, WingSpec}};

/// S_h/S_v pelo método do coeficiente de volume (Raymer Tab. 6.4) — mesma
/// fórmula usada por `EmpennageAgent::run`, mas calculável DIRETO de
/// `AircraftConfig`, sem precisar de uma `WingSpec` já computada pelo
/// `AerodynamicsAgent`. Existe porque `span_m`/`area_m2`/`taper_ratio` de
/// `WingSpec` são cópias literais de `[wing]` (`AircraftState::from_config`
/// → `AerodynamicsAgent::run`, ver `WingSpec` em `agents::aerodynamics`) —
/// SEM nenhuma dependência de MTOW/cd0 — então essa geometria já está
/// disponível direto do TOML, antes de qualquer agente rodar.
///
/// Usada por `AircraftState::from_config` (task refino-ciclo2, 1b) para
/// derivar `cd0_empennage` a partir de `[empennage].cd0_area_factor` ANTES
/// do `AerodynamicsAgent` calcular `cd0_total` — resolve a dependência
/// circular (cd0 precisa de S_h+S_v; S_h+S_v antes só vinha de uma
/// `WingSpec` já computada) SEM reordenar `EmpennageAgent`/
/// `AerodynamicsAgent` no laço de convergência (`orchestrator::
/// size_aircraft` continua chamando os dois agentes na mesma ordem de
/// antes — a `EmpennageSpec` completa, com corda/envergadura/etc., ainda é
/// produzida por `EmpennageAgent::run` logo após a asa, como sempre).
pub fn tail_areas_m2(cfg: &AircraftConfig) -> (f64, f64) {
    tail_areas_m2_from_wing_geometry(cfg.wing.area_m2, cfg.wing.span_m, cfg.wing.taper_ratio, cfg)
}

/// Núcleo de `tail_areas_m2`/`EmpennageAgent::run` — separado para as duas
/// chamadas (uma via `AircraftConfig` puro, outra via os campos já
/// presentes numa `WingSpec`) compartilharem a MESMA fórmula sem divergir
/// silenciosamente.
fn tail_areas_m2_from_wing_geometry(
    wing_area_m2: f64,
    wing_span_m: f64,
    wing_taper_ratio: f64,
    cfg: &AircraftConfig,
) -> (f64, f64) {
    let emp_cfg = &cfg.empennage;
    let c_r = chord_root(wing_area_m2, wing_span_m, wing_taper_ratio);
    let mac = mean_aerodynamic_chord(c_r, wing_taper_ratio);
    let l_h = emp_cfg.tail_arm_m;
    let l_v = emp_cfg.tail_arm_m;
    let s_h = emp_cfg.v_h * wing_area_m2 * mac / l_h;
    let s_v = emp_cfg.v_v * wing_area_m2 * wing_span_m / l_v;
    (s_h, s_v)
}

pub struct EmpennageAgent;

impl EmpennageAgent {
    pub fn run(wing: &WingSpec, cfg: &AircraftConfig) -> EmpennageSpec {
        let emp_cfg = &cfg.empennage;
        let l_h = emp_cfg.tail_arm_m;
        let l_v = emp_cfg.tail_arm_m;

        let (s_h, s_v) =
            tail_areas_m2_from_wing_geometry(wing.area_m2, wing.span_m, wing.taper_ratio, cfg);

        let span_h = (emp_cfg.ar_h * s_h).sqrt();
        let span_v = (emp_cfg.ar_v * s_v).sqrt();

        let chord_h_root = 2.0 * s_h / (span_h * (1.0 + emp_cfg.taper_h));
        let chord_h_tip = chord_h_root * emp_cfg.taper_h;
        let chord_v_root = 2.0 * s_v / (span_v * (1.0 + emp_cfg.taper_v));
        let chord_v_tip = chord_v_root * emp_cfg.taper_v;

        EmpennageSpec {
            s_horizontal_m2: s_h,
            s_vertical_m2: s_v,
            arm_h_m: l_h,
            arm_v_m: l_v,
            span_h_m: span_h,
            span_v_m: span_v,
            chord_h_root_m: chord_h_root,
            chord_h_tip_m: chord_h_tip,
            chord_v_root_m: chord_v_root,
            chord_v_tip_m: chord_v_tip,
            ar_h: emp_cfg.ar_h,
            ar_v: emp_cfg.ar_v,
            taper_h: emp_cfg.taper_h,
            taper_v: emp_cfg.taper_v,
            volume_h: emp_cfg.v_h,
            volume_v: emp_cfg.v_v,
            eta_h: emp_cfg.eta_h,
        }
    }
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::agents::aerodynamics::AerodynamicsAgent;
    use crate::models::aircraft_config::test_fixtures::config_teste;
    use crate::models::aircraft_state::AircraftState;

    fn wing_teste() -> (WingSpec, AircraftConfig) {
        let cfg = config_teste();
        let state = AircraftState::from_config(&cfg);
        let req = crate::models::requirements::test_fixtures::requisitos_teste();
        let wing = AerodynamicsAgent::run(&state, &req);
        (wing, cfg)
    }

    /// S_h/S_v calculados à mão a partir dos valores da fixture sintética
    /// (config_teste(): span=11.0m, area=13.5m², taper=0.5, tail_arm=4.70m,
    /// v_h=0.65, v_v=0.045):
    ///   c_r  = 2·13.5/(11.0·1.5)          = 1.63636 m
    ///   MAC  = (2/3)·c_r·(1+0.5+0.25)/1.5 = 1.27273 m
    ///   S_h  = 0.65·13.5·1.27273/4.70     ≈ 2.3762 m²
    ///   S_v  = 0.045·13.5·11.0/4.70       ≈ 1.4218 m²
    #[test]
    fn s_h_s_v_batem_com_calculo_manual_na_fixture_sintetica() {
        let (wing, cfg) = wing_teste();
        let emp = EmpennageAgent::run(&wing, &cfg);

        let esperado_s_h = 2.3762_f64;
        let esperado_s_v = 1.4218_f64;

        assert!(
            (emp.s_horizontal_m2 - esperado_s_h).abs() / esperado_s_h < 0.01,
            "S_h = {:.4} m² (esperado ≈{esperado_s_h:.4} m², ±1%)",
            emp.s_horizontal_m2
        );
        assert!(
            (emp.s_vertical_m2 - esperado_s_v).abs() / esperado_s_v < 0.01,
            "S_v = {:.4} m² (esperado ≈{esperado_s_v:.4} m², ±1%)",
            emp.s_vertical_m2
        );
    }

    #[test]
    fn chord_tip_menor_que_chord_root_para_ambas_superficies() {
        let (wing, cfg) = wing_teste();
        let emp = EmpennageAgent::run(&wing, &cfg);

        assert!(emp.chord_h_tip_m < emp.chord_h_root_m,
            "corda de ponta H ({:.3}) deveria ser menor que a de raiz ({:.3})",
            emp.chord_h_tip_m, emp.chord_h_root_m);
        assert!(emp.chord_v_tip_m < emp.chord_v_root_m,
            "corda de ponta V ({:.3}) deveria ser menor que a de raiz ({:.3})",
            emp.chord_v_tip_m, emp.chord_v_root_m);
    }

    // ─── tail_areas_m2 (task refino-ciclo2, 1b) ──────────────────────────
    //
    // `AircraftState::from_config` precisa de S_h/S_v para derivar
    // `cd0_empennage` ANTES do `AerodynamicsAgent` rodar (cd0_empennage
    // alimenta `cd0_total`, consumido pelo build-up de arrasto que o
    // AerodynamicsAgent calcula) — mas `EmpennageAgent::run` pede uma
    // `WingSpec` já calculada, que só existe DEPOIS do AerodynamicsAgent
    // rodar. `tail_areas_m2` resolve isso: computa S_h/S_v direto de
    // `AircraftConfig` (span_m/area_m2/taper_ratio de `[wing]` — os MESMOS
    // valores que `WingSpec` apenas ecoa de `AircraftState`, sem nenhuma
    // dependência de MTOW/cd0), reaproveitando a mesma fórmula usada por
    // `EmpennageAgent::run`.

    /// `tail_areas_m2(cfg)` deve bater EXATAMENTE com `EmpennageAgent::run`
    /// (mesma fórmula, calculada por dois caminhos diferentes) — usa a
    /// fixture sintética.
    #[test]
    fn tail_areas_m2_bate_com_empennage_agent_run() {
        let (wing, cfg) = wing_teste();
        let emp = EmpennageAgent::run(&wing, &cfg);

        let (s_h, s_v) = tail_areas_m2(&cfg);
        println!("tail_areas_m2: S_h={s_h:.6}  S_v={s_v:.6}");
        assert!((s_h - emp.s_horizontal_m2).abs() < 1e-9,
            "S_h de tail_areas_m2 ({s_h:.9}) deveria bater exatamente com \
             EmpennageAgent::run ({:.9})", emp.s_horizontal_m2);
        assert!((s_v - emp.s_vertical_m2).abs() < 1e-9,
            "S_v de tail_areas_m2 ({s_v:.9}) deveria bater exatamente com \
             EmpennageAgent::run ({:.9})", emp.s_vertical_m2);
    }

    /// Hand-check direto (mesmos valores de `s_h_s_v_batem_com_calculo_
    /// manual_na_fixture_sintetica` acima) — confirma que `tail_areas_m2`
    /// não depende de rodar `AerodynamicsAgent`/`EmpennageAgent` primeiro.
    #[test]
    fn tail_areas_m2_hand_check_fixture_sintetica() {
        let cfg = config_teste();
        let (s_h, s_v) = tail_areas_m2(&cfg);
        println!("tail_areas_m2 (direto de cfg): S_h={s_h:.6}  S_v={s_v:.6}");
        assert!((s_h - 2.3762).abs() / 2.3762 < 0.01, "S_h = {s_h:.4} (esperado ≈2.3762, ±1%)");
        assert!((s_v - 1.4218).abs() / 1.4218 < 0.01, "S_v = {s_v:.4} (esperado ≈1.4218, ±1%)");
    }

    /// Área da empenagem horizontal cresce linearmente com V_h (a fórmula
    /// S_h = V_h·S_w·MAC/l_h é diretamente proporcional a V_h, com todo o
    /// resto fixo) — checagem de propriedade, não de valor mágico.
    #[test]
    fn s_h_cresce_com_v_h() {
        let (wing, mut cfg) = wing_teste();
        let emp_base = EmpennageAgent::run(&wing, &cfg);

        cfg.empennage.v_h *= 1.2;
        let emp_maior = EmpennageAgent::run(&wing, &cfg);

        assert!(emp_maior.s_horizontal_m2 > emp_base.s_horizontal_m2,
            "S_h deveria crescer com v_h: base={:.4} maior={:.4}",
            emp_base.s_horizontal_m2, emp_maior.s_horizontal_m2);
    }
}
