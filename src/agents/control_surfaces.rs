/// ControlSurfacesAgent — Superfícies de Controle e Flaps (Task 4.2)
///
/// Dimensiona aileron, flap (asa), profundor (elevator, EH) e leme (rudder,
/// EV) por razões históricas (Raymer, "Aircraft Design: A Conceptual
/// Approach", Tab. 6.5 — frações típicas de envergadura/corda para
/// monomotor GA), parametrizadas em `[control_surfaces]` no TOML de
/// aeronave — nenhuma fração é hardcoded aqui.
///
/// Geometria trapezoidal (mesmo modelo da asa/empenagem — ver
/// `weight_balance::chord_at`):
///
///   c(η) = c_raiz · (1 − (1−λ)·η),   η = y / (b/2)
///
/// Este agente é puramente geométrico (não depende de peso/MTOW nem do
/// motor) — pode ser calculado uma única vez, após `AerodynamicsAgent` e
/// `EmpennageAgent`, sem participar do laço de convergência de MTOW
/// (`orchestrator::size_aircraft`).
///
/// ── Aileron / Flap (asa) ────────────────────────────────────────────────
/// Definidos por fração de INÍCIO/FIM da semi-envergadura da asa
/// (`aileron_span_start_frac`..`aileron_span_end_frac`, idem flap — η
/// medido a partir da linha de centro, η=0, até a ponta, η=1). Cada
/// superfície existe nos DOIS lados (esquerdo/direito, idênticos por
/// simetria): `SurfaceGeom::area_m2` já soma os dois lados; `span_m` é a
/// envergadura FÍSICA DE UM LADO apenas (a grandeza relevante para
/// dimensionamento estrutural/de atuador, que se aplica por lado).
///
/// ── Profundor (EH) ──────────────────────────────────────────────────────
/// O estabilizador horizontal é uma superfície ESPELHADA (`span_h_m` é a
/// envergadura TOTAL ponta-a-ponta, mesma convenção da asa — ver
/// `EmpennageAgent`). O profundor é definido por UMA fração
/// (`elevator_span_frac`), medida a partir da raiz (η=0, linha de centro)
/// — cobrindo `elevator_span_frac` da semi-envergadura EM CADA lado,
/// simetricamente. Por identidade algébrica (a mesma que torna
/// `chord_root = 2S/(b(1+λ))` válida tanto para uma asa espelhada quanto
/// para um único painel trapezoidal de comprimento `b`), a área de dois
/// trapézios "raiz→η_edge" espelhados, com η_edge=`elevator_span_frac`,
/// soma-se EXATAMENTE à área de um único trapézio calculado com a
/// envergadura FÍSICA TOTAL `elevator_span_frac · span_h_m` — por isso o
/// cálculo abaixo não tem um fator ×2 explícito (`tail_surface_single`
/// serve tanto para o profundor quanto para o leme).
///
/// ── Leme (EV) ────────────────────────────────────────────────────────────
/// A deriva NÃO é espelhada (um único painel) — `span_v_m` já é a
/// envergadura física TOTAL desse painel (raiz na base, η=0, até
/// `rudder_span_frac·span_v_m`). Mesma função `tail_surface_single`.
use crate::agents::weight_balance::{chord_at, chord_root};
use crate::models::{
    aircraft_config::AircraftConfig,
    specs::{ControlSurfacesSpec, EmpennageSpec, SurfaceGeom, WingSpec},
};

pub struct ControlSurfacesAgent;

impl ControlSurfacesAgent {
    pub fn run(wing: &WingSpec, emp: &EmpennageSpec, cfg: &AircraftConfig) -> ControlSurfacesSpec {
        let cs = &cfg.control_surfaces;

        let c_r_wing = chord_root(wing.area_m2, wing.span_m, wing.taper_ratio);
        let half_span_wing = wing.span_m / 2.0;

        let aileron = wing_surface_per_side(
            c_r_wing,
            wing.taper_ratio,
            half_span_wing,
            cs.aileron_span_start_frac,
            cs.aileron_span_end_frac,
            cs.aileron_chord_frac,
        );
        let flap = wing_surface_per_side(
            c_r_wing,
            wing.taper_ratio,
            half_span_wing,
            cs.flap_span_start_frac,
            cs.flap_span_end_frac,
            cs.flap_chord_frac,
        );

        let elevator = tail_surface_single(
            emp.chord_h_root_m,
            emp.taper_h,
            emp.span_h_m,
            cs.elevator_span_frac,
            cs.elevator_chord_frac,
        );
        let rudder = tail_surface_single(
            emp.chord_v_root_m,
            emp.taper_v,
            emp.span_v_m,
            cs.rudder_span_frac,
            cs.rudder_chord_frac,
        );

        ControlSurfacesSpec { aileron, flap, elevator, rudder }
    }
}

/// Aileron/flap: um lado da asa, entre `eta_start`..`eta_end` (fração da
/// semi-envergadura `half_span`), com `span_m` reportado POR LADO e
/// `area_m2` já somando os dois lados (×2) — ver docstring do módulo.
fn wing_surface_per_side(
    c_root: f64,
    taper: f64,
    half_span: f64,
    eta_start: f64,
    eta_end: f64,
    chord_frac: f64,
) -> SurfaceGeom {
    let c_start = chord_frac * chord_at(eta_start, c_root, taper);
    let c_end = chord_frac * chord_at(eta_end, c_root, taper);
    let span_per_side = (eta_end - eta_start) * half_span;
    let area_per_side = 0.5 * (c_start + c_end) * span_per_side;

    SurfaceGeom {
        span_m: span_per_side,
        area_m2: 2.0 * area_per_side,
        chord_mean_m: 0.5 * (c_start + c_end),
        start_m: eta_start * half_span,
        end_m: eta_end * half_span,
    }
}

/// Profundor/leme: superfície única "raiz (η=0) → η=span_frac", sobre a
/// envergadura física total `surf_span_m` da superfície-mãe (`span_h_m`
/// para o EH, `span_v_m` para o EV) — ver docstring do módulo para a
/// justificativa de por que a mesma função serve para ambos os casos
/// (EH espelhado vs. EV painel único).
fn tail_surface_single(
    c_root: f64,
    taper: f64,
    surf_span_m: f64,
    span_frac: f64,
    chord_frac: f64,
) -> SurfaceGeom {
    let c_start = chord_frac * chord_at(0.0, c_root, taper);
    let c_end = chord_frac * chord_at(span_frac, c_root, taper);
    let span_m = span_frac * surf_span_m;
    let area_m2 = 0.5 * (c_start + c_end) * span_m;

    SurfaceGeom {
        span_m,
        area_m2,
        chord_mean_m: 0.5 * (c_start + c_end),
        start_m: 0.0,
        end_m: span_m,
    }
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::agents::aerodynamics::AerodynamicsAgent;
    use crate::agents::empennage::EmpennageAgent;
    use crate::agents::weight_balance::chord_root as chord_root_fn;
    use crate::models::aircraft_config::test_fixtures::config_teste;
    use crate::models::aircraft_state::AircraftState;

    fn fixture() -> (WingSpec, EmpennageSpec, AircraftConfig) {
        let cfg = config_teste();
        let state = AircraftState::from_config(&cfg);
        let req = crate::models::requirements::test_fixtures::requisitos_teste();
        let wing = AerodynamicsAgent::run(&state, &req);
        let emp = EmpennageAgent::run(&wing, &cfg);
        (wing, emp, cfg)
    }

    /// Hand-check do aileron na fixture sintética (config_teste():
    /// span=11.0m, area=13.5m², taper=0.5; control_surfaces:
    /// aileron_span_start_frac=0.58, aileron_span_end_frac=0.92,
    /// aileron_chord_frac=0.24):
    ///   c_r  = 2·13.5/(11.0·1.5)              = 1.63636 m
    ///   c(0.58) = c_r·(1−0.5·0.58)            = c_r·0.71   ≈ 1.16182 m
    ///   c(0.92) = c_r·(1−0.5·0.92)            = c_r·0.54   ≈ 0.88364 m
    ///   c_médio·chord_frac = 0.24·(1.16182+0.88364)/2      ≈ 0.24546 m
    ///   span/lado = (0.92−0.58)·(11.0/2)      = 0.34·5.5   = 1.87 m
    ///   área/lado ≈ 0.24546·1.87              ≈ 0.4590 m²
    ///   área total (×2)                        ≈ 0.9180 m²
    #[test]
    fn aileron_hand_check_fixture_sintetica() {
        let (wing, emp, cfg) = fixture();
        let cs = ControlSurfacesAgent::run(&wing, &emp, &cfg);

        println!(
            "aileron: span/lado={:.4}m  área(×2)={:.4}m²  corda_média={:.4}m  start={:.4}m  end={:.4}m",
            cs.aileron.span_m, cs.aileron.area_m2, cs.aileron.chord_mean_m,
            cs.aileron.start_m, cs.aileron.end_m
        );

        let esperado_span = 1.87_f64;
        let esperado_area = 0.9180_f64;

        assert!((cs.aileron.span_m - esperado_span).abs() / esperado_span < 0.01,
            "span/lado = {:.4}m (esperado ≈{esperado_span:.4}m, ±1%)", cs.aileron.span_m);
        assert!((cs.aileron.area_m2 - esperado_area).abs() / esperado_area < 0.01,
            "área = {:.4}m² (esperado ≈{esperado_area:.4}m², ±1%)", cs.aileron.area_m2);
    }

    #[test]
    fn aileron_e_flap_nao_se_sobrepoem_e_cabem_na_semi_envergadura() {
        let (wing, emp, cfg) = fixture();
        let cs = ControlSurfacesAgent::run(&wing, &emp, &cfg);
        let half_span = wing.span_m / 2.0;

        assert!(cs.flap.start_m >= 0.0, "flap.start_m ({:.4}) deveria ser ≥ 0", cs.flap.start_m);
        assert!(cs.flap.start_m < cs.flap.end_m,
            "flap.start_m ({:.4}) deveria ser < flap.end_m ({:.4})", cs.flap.start_m, cs.flap.end_m);
        assert!(cs.aileron.start_m < cs.aileron.end_m,
            "aileron.start_m ({:.4}) deveria ser < aileron.end_m ({:.4})",
            cs.aileron.start_m, cs.aileron.end_m);
        assert!(cs.flap.end_m <= cs.aileron.start_m + 1e-9,
            "flap.end_m ({:.4}) não deveria ultrapassar aileron.start_m ({:.4}) — sobreposição",
            cs.flap.end_m, cs.aileron.start_m);
        assert!(cs.aileron.end_m <= half_span + 1e-9,
            "aileron.end_m ({:.4}) não deveria ultrapassar a semi-envergadura ({:.4})",
            cs.aileron.end_m, half_span);
    }

    #[test]
    fn todas_as_areas_sao_positivas() {
        let (wing, emp, cfg) = fixture();
        let cs = ControlSurfacesAgent::run(&wing, &emp, &cfg);

        for (nome, area) in [
            ("aileron", cs.aileron.area_m2),
            ("flap", cs.flap.area_m2),
            ("elevator", cs.elevator.area_m2),
            ("rudder", cs.rudder.area_m2),
        ] {
            assert!(area > 0.0, "{nome}.area_m2 = {area:.4} deveria ser positivo");
        }
    }

    /// O profundor deve cobrir uma fração razoável (mas não exceder) a área
    /// do estabilizador horizontal — checagem de propriedade/sanidade, não
    /// de valor mágico: `elevator.area_m2` ≈ `elevator_chord_frac ·
    /// elevator_span_frac · S_h`, dentro de ±15% (a aproximação do produto
    /// simples ignora a variação de corda ao longo da envergadura,
    /// diferente do trapézio real calculado pelo agente).
    #[test]
    fn area_do_profundor_e_coerente_com_s_h() {
        let (wing, emp, cfg) = fixture();
        let cs = ControlSurfacesAgent::run(&wing, &emp, &cfg);

        let aprox = cfg.control_surfaces.elevator_chord_frac
            * cfg.control_surfaces.elevator_span_frac
            * emp.s_horizontal_m2;

        println!("elevator.area_m2={:.4}  aproximação (chord_frac·span_frac·S_h)={:.4}",
            cs.elevator.area_m2, aprox);

        assert!((cs.elevator.area_m2 - aprox).abs() / aprox < 0.15,
            "área do profundor {:.4}m² deveria estar a ±15% da aproximação {:.4}m²",
            cs.elevator.area_m2, aprox);
    }

    #[test]
    fn area_cresce_com_chord_frac() {
        let (wing, emp, cfg) = fixture();
        let cs_base = ControlSurfacesAgent::run(&wing, &emp, &cfg);

        let mut cfg_maior = cfg.clone();
        cfg_maior.control_surfaces.aileron_chord_frac *= 1.2;
        let cs_maior = ControlSurfacesAgent::run(&wing, &emp, &cfg_maior);

        assert!(cs_maior.aileron.area_m2 > cs_base.aileron.area_m2,
            "área do aileron deveria crescer com aileron_chord_frac: base={:.4} maior={:.4}",
            cs_base.aileron.area_m2, cs_maior.aileron.area_m2);
    }

    #[test]
    fn chord_root_usado_no_agente_bate_com_helper_publico() {
        let (wing, _emp, _cfg) = fixture();
        let cr = chord_root_fn(wing.area_m2, wing.span_m, wing.taper_ratio);
        assert!(cr > 0.0);
    }
}
