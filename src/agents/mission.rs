//! MissionAgent — Análise de Missão por Segmentos (Task 5.1)
//!
//! Substitui o modelo antigo de consumo constante do laço de convergência de
//! MTOW (`fc_cruise_lph · endurance_min_h / (1 − reserva)`, um único ponto de
//! operação de cruzeiro aplicado à missão inteira) por uma missão dividida em
//! segmentos fisicamente distintos:
//!
//!   1. **Táxi**: combustível fixo de projeto (`analysis.taxi_fuel_l`) — sem
//!      duração modelada (não há dado de tempo de táxi na configuração).
//!   2. **Subida**: integração numérica em passos de 100m de altitude, do
//!      aeródromo até `cruise_altitude_m`, à potência de `rpm_max_continuous`
//!      (carga plena) e à velocidade de melhor razão de subida (Vy —
//!      `agents::performance::climb_rate_ms`; única política suportada hoje,
//!      `analysis.climb_speed_policy = "vy"`). A massa cai a cada passo
//!      conforme o combustível queimado, então RC (que depende da massa)
//!      também varia passo a passo.
//!   3. **Cruzeiro**: equação de Breguet (dedução completa abaixo) — a massa
//!      cai continuamente ao longo da distância de cruzeiro, ao contrário do
//!      modelo antigo (consumo constante × tempo), que superestimava o
//!      combustível necessário por nunca refletir o alívio de arrasto
//!      induzido conforme a aeronave emagrece.
//!   4. **Descida**: potência parcial (`analysis.descent_power_fraction`,
//!      não motor cortado), taxa de descida de projeto
//!      (`analysis.descent_rate_ms`).
//!   5. **Reserva**: fração (`req.fuel_reserve_fraction`) sobre o consumo da
//!      missão SEM reserva (táxi+subida+cruzeiro+descida) — não sobre o
//!      total já incluindo a própria reserva (não é uma fração composta).
//!
//! `MissionAgent::run` é chamado pelo laço de convergência de MTOW
//! (`orchestrator::size_aircraft`) a cada iteração — o combustível total da
//! missão (`MissionSpec::fuel_total_kg`) substitui o antigo
//! `fc_cruise_lph · endurance_min_h / (1 − reserva)` como `fuel_kg` que
//! fecha o ponto fixo `mtow = OEW(mtow) + payload + fuel_kg(mtow)`.
//!
//! ## Dedução da equação de Breguet (segmento de cruzeiro)
//!
//! Para uma aeronave a hélice em voo nivelado, a taxa de queima de PESO de
//! combustível é proporcional à potência consumida no VIRABREQUIM (não à
//! potência de eixo entregue à hélice — ver "BSFC referencia o virabrequim"
//! abaixo, achado da revisão desta task, Finding 2):
//!
//! ```text
//!   -dW/dt = c_p · P_virabrequim
//! ```
//!
//! onde `c_p` é o consumo específico "baseado em peso" (unidade: 1/m — peso
//! de combustível por unidade de ENERGIA). A potência de eixo NA HÉLICE
//! (pós-PSRU) relaciona-se ao arrasto via a eficiência de hélice `η_p`:
//!
//! ```text
//!   P_hélice = D·V / η_p = (W / (L/D)) · V / η_p     [voo nivelado: T=D]
//! ```
//!
//! e a potência no VIRABREQUIM (pré-PSRU) é MAIOR que `P_hélice` pelas
//! perdas mecânicas do PSRU (correia/engrenagens, `η_PSRU` — dado de
//! configuração, `AircraftState::psru_efficiency` / `[propeller]
//! psru_efficiency` do TOML):
//!
//! ```text
//!   P_virabrequim = P_hélice / η_PSRU
//! ```
//!
//! Substituindo e usando `dR = V·dt` (R = distância percorrida):
//!
//! ```text
//!   -dW = (c_p/η_PSRU)/η_p · (W/(L/D)) · dR
//!   dR  = -η_p·η_PSRU·(L/D)/c_p · dW/W
//! ```
//!
//! Integrando de `W0` (início do cruzeiro) a `W1` (fim):
//!
//! ```text
//!   R = η_p·η_PSRU·(L/D)/c_p · ln(W0/W1)
//! ```
//!
//! Esta é a equação clássica de Breguet para propulsão a hélice (Anderson,
//! "Introduction to Flight", cap. 6; Raymer, "Aircraft Design", cap. 3),
//! generalizada com o fator `η_PSRU` (ausente na dedução original desta
//! task — Finding 2 da revisão). O BSFC do motor (`bsfc_gkwh`, g de
//! combustível por kWh de energia no VIRABREQUIM) é um consumo específico
//! baseado em MASSA, não em peso: `c_p = g·c`, onde `c` = BSFC convertido
//! para kg de combustível por Joule de energia no virabrequim (kg/(W·s) =
//! kg/J). Substituindo:
//!
//! ```text
//!   R = (η_p·η_PSRU/(g·c))·(L/D)·ln(W0/W1)
//! ```
//!
//! Isolando `W1` (a forma usada aqui — dada a distância de cruzeiro `R`,
//! quanto sobra de massa depois de queimar combustível):
//!
//! ```text
//!   W1 = W0 · exp(−R·g·c / (η_p·η_PSRU·(L/D)))
//! ```
//!
//! Como `W = m·g`, a razão `W1/W0 = m1/m0` — o fator `g` cancela na razão de
//! massas, então a fórmula acima é aplicada diretamente em massa (kg), sem
//! precisar converter para Newtons: `m1 = m0·exp(−expoente)`,
//! `expoente = R·g·c/(η_p·η_PSRU·(L/D))`.
//!
//! ### BSFC referencia o virabrequim, não o eixo pós-PSRU (Finding 2)
//!
//! `BsfcModel::bsfc_gkwh` modela o consumo específico do MOTOR — medido no
//! virabrequim, ANTES das perdas mecânicas do PSRU (correia/engrenagem,
//! `η_PSRU`, tipicamente ~0,97, dado de configuração —
//! `AircraftState::psru_efficiency`). `shaft_power_kw`/`p_req_cruise_kw` (usados na subida e
//! no cálculo de `fc_cruise_lph` em `agents::propulsion`) já são potências
//! PÓS-PSRU (potência de eixo entregue à hélice). Multiplicar BSFC
//! diretamente por uma potência pós-PSRU subestima o consumo de
//! combustível pela fração de perdas do PSRU (~3%, já que `η_PSRU=0,97`):
//! a correção — usada tanto na subida (`fuel_climb_kg`) quanto no cruzeiro
//! Breguet acima e em `agents::propulsion::PropulsionAgent::run`
//! (`fc_cruise_lph`) — é dividir a potência de eixo pós-PSRU por `η_PSRU`
//! antes de aplicar o BSFC, recuperando a potência de virabrequim.
//!
//! ### Conversão de unidades de `c` (BSFC → kg/(W·s))
//!
//! `bsfc_gkwh` está em g/(kW·h). Convertendo para kg/(W·s) = kg/J:
//!
//! ```text
//!   c [kg/J] = bsfc_gkwh [g/(kW·h)] ÷ 1000 [g→kg] ÷ 1000 [kW→W] ÷ 3600 [h→s]
//!            = bsfc_gkwh / 3,6×10⁹
//! ```
//!
//! ### Verificação dimensional do expoente
//!
//! `expoente = R·g·c`, com `R` em metros, `g` em m/s², `c` em kg/(W·s):
//!
//! ```text
//!   [m]·[m/s²]·[kg/(W·s)] = [m]·[m/s²]·[kg·s²/(kg·m²)]   (W = kg·m²/s³)
//!                          = [m]·[m/s²]·[s²/m²] = adimensional ✓
//! ```
//!
//! (`η_p`, `η_PSRU` e `L/D` já são adimensionais, então dividir por eles não
//! afeta a verificação.)
//!
//! ### Hand-check (documentado no brief do controller, ATUALIZADO na revisão
//! ### desta task — Finding 2 — para incluir `η_PSRU`, reproduzido em teste)
//!
//! `R = 2.000.000 m`, `bsfc = 210 g/kWh` → `c = 210/3,6×10⁹ = 5,8333×10⁻⁸
//! kg/(W·s)`, `η_p = 0,808`, `η_PSRU = 0,97`, `L/D = 13`:
//!
//! ```text
//!   expoente = 2.000.000 · 9,807 · 5,8333×10⁻⁸ / (0,808·0,97·13)
//!            = 1,1442 / 10,1889 = 0,11231
//!   W1/W0 = exp(−0,11231) = 0,89379 → combustível ≈ 10,62% de W0
//! ```
//!
//! (Valor original do brief, SEM `η_PSRU` — expoente 0,1089, combustível
//! ~10,32% — preservado no comentário do teste como "antes da correção da
//! revisão", não mais o valor autoritativo.)

use crate::agents::performance::{climb_rate_ms, shaft_power_kw};
use crate::models::aircraft_state::AircraftState;
use crate::models::engine::EngineSpec;
use crate::models::requirements::Requirements;
use crate::models::specs::{MissionSpec, PropulsionSpec, WingSpec};

const G: f64 = 9.807; // m/s²

/// RC (m/s) abaixo do qual a subida é considerada travada — não exatamente
/// zero, para não deixar o passo de integração explodir (`dt = passo/RC`)
/// por ruído numérico perto do teto de serviço.
const RC_MIN_MS: f64 = 0.1;

/// Passo vertical de integração da subida (m) — ver docstring do módulo.
const CLIMB_STEP_M: f64 = 100.0;

/// Fator de conversão BSFC (g/kWh) → consumo específico de massa por
/// energia (kg/(W·s) = kg/J) — ver "Conversão de unidades de `c`" acima.
const BSFC_GKWH_TO_KG_PER_J: f64 = 1000.0 * 1000.0 * 3600.0; // 3.6e9

/// Converte BSFC (g/kWh) para `c` (kg/(W·s) = kg/J) — ver dedução no
/// docstring do módulo.
fn bsfc_kg_per_j(bsfc_gkwh: f64) -> f64 {
    bsfc_gkwh / BSFC_GKWH_TO_KG_PER_J
}

/// Fração de massa restante `m1/m0` depois de percorrer `range_m` em
/// cruzeiro Breguet — ver dedução completa no docstring do módulo.
///
/// `eta_psru`: eficiência mecânica do PSRU (Finding 2 da revisão) — o BSFC
/// referencia potência de VIRABREQUIM, mas `eta_p` (eficiência de hélice)
/// só converte potência de EIXO (pós-PSRU) em tração; sem este fator, o
/// consumo é subestimado em `1/η_PSRU − 1 ≈ 3%`.
fn breguet_mass_ratio(range_m: f64, eta_p: f64, eta_psru: f64, ld: f64, bsfc_gkwh: f64) -> f64 {
    let c = bsfc_kg_per_j(bsfc_gkwh);
    let expoente = range_m * c * G / (eta_p * eta_psru * ld);
    (-expoente).exp()
}

/// Combustível queimado (kg) para percorrer `range_m` em cruzeiro Breguet,
/// partindo de massa `w0_kg`.
fn breguet_fuel_burn_kg(
    w0_kg: f64, range_m: f64, eta_p: f64, eta_psru: f64, ld: f64, bsfc_gkwh: f64,
) -> f64 {
    let ratio = breguet_mass_ratio(range_m, eta_p, eta_psru, ld, bsfc_gkwh);
    w0_kg * (1.0 - ratio)
}

/// Alcance Breguet (m) percorrido queimando de `w0_kg` até `w1_kg` — inverso
/// de `breguet_fuel_burn_kg`, usado só para o campo informativo
/// `breguet_range_full_tank_km`. Requer `w0_kg > w1_kg > 0`; fora disso
/// (configuração fisicamente degenerada — tanque cheio mais pesado que o
/// MTOW) retorna `0.0` em vez de produzir NaN/infinito.
fn breguet_range_m(
    w0_kg: f64, w1_kg: f64, eta_p: f64, eta_psru: f64, ld: f64, bsfc_gkwh: f64,
) -> f64 {
    if w1_kg <= 0.0 || w0_kg <= w1_kg {
        return 0.0;
    }
    let c = bsfc_kg_per_j(bsfc_gkwh);
    (eta_p * eta_psru / (G * c)) * ld * (w0_kg / w1_kg).ln()
}

/// Erros da análise de missão por segmentos — casos em que a missão pedida
/// não é fisicamente alcançável com a célula/motor/MTOW candidato desta
/// iteração, não um bug do cálculo.
#[derive(Debug, Clone, PartialEq)]
pub enum MissionError {
    /// A subida travou: RC caiu a `rc_ms` (≤ `RC_MIN_MS`) na altitude
    /// `altitude_m`, massa `massa_kg` — o motor não tem potência suficiente
    /// para continuar subindo até `cruise_altitude_m` com este MTOW.
    SubidaInviavel { altitude_m: f64, massa_kg: f64, rc_ms: f64 },
    /// A distância obrigatória de subida + descida já consome toda (ou mais
    /// que toda) a distância exigida pela autonomia mínima da missão —
    /// sobra zero ou menos para o segmento de cruzeiro Breguet.
    CruzeiroDistanciaNaoPositiva { climb_km: f64, descent_km: f64, exigido_km: f64 },
}

impl std::fmt::Display for MissionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MissionError::SubidaInviavel { altitude_m, massa_kg, rc_ms } => write!(
                f,
                "subida inviável: razão de subida caiu a {rc_ms:.3} m/s em {altitude_m:.0} m \
                 (massa {massa_kg:.1} kg) — motor insuficiente para alcançar a altitude de \
                 cruzeiro com este MTOW"
            ),
            MissionError::CruzeiroDistanciaNaoPositiva { climb_km, descent_km, exigido_km } => write!(
                f,
                "distância de cruzeiro não positiva: subida ({climb_km:.1} km) + descida \
                 ({descent_km:.1} km) já consomem toda a distância exigida pela missão \
                 ({exigido_km:.1} km) — autonomia mínima incompatível com a geometria de \
                 subida/descida desta missão"
            ),
        }
    }
}

impl std::error::Error for MissionError {}

pub struct MissionAgent;

impl MissionAgent {
    /// Executa a análise de missão por segmentos e retorna o `MissionSpec`
    /// completo — ou um `MissionError` quando a missão pedida não é
    /// fisicamente alcançável (subida travada ou distância de cruzeiro não
    /// positiva).
    ///
    /// `mtow_mission_kg`: massa no início da missão (MTOW candidato desta
    /// iteração do laço de convergência) — inclui o combustível total da
    /// missão (é exatamente esse combustível que este agente calcula).
    pub fn run(
        state: &AircraftState,
        wing: &WingSpec,
        prop: &PropulsionSpec,
        engine: &EngineSpec,
        req: &Requirements,
        mtow_mission_kg: f64,
    ) -> Result<MissionSpec, MissionError> {
        let density = engine.fuel.density_kg_per_l;

        // ── 1. Táxi ──────────────────────────────────────────────────────
        let fuel_taxi_kg = req.analysis.taxi_fuel_l * density;

        // ── 2. Subida (integração numérica, passos de 100m) ─────────────
        let mut mass_kg = mtow_mission_kg - fuel_taxi_kg;
        let mut alt_m = req.airfield_altitude_m;
        let mut climb_time_s = 0.0_f64;
        let mut climb_distance_m = 0.0_f64;
        let mut fuel_climb_kg = 0.0_f64;

        while alt_m < req.cruise_altitude_m - 1e-9 {
            let step_m = CLIMB_STEP_M.min(req.cruise_altitude_m - alt_m);

            // `static_thrust_factor=1.0`: parâmetro exigido pela assinatura
            // de `climb_rate_ms`, mas PROVADAMENTE inerte neste caminho —
            // ele só afeta o ramo de tração ESTÁTICA (V<0,5 m/s) de
            // `thrust_available_n`, e `climb_rate_ms` varre exclusivamente
            // V ∈ [1,05·Vs, 2,00·Vs] (ERRATUM ciclo 11 §2, era
            // [1,3·Vs, 1,8·Vs] — ver docstring de `climb_rate_ms`), sempre
            // ≫ 0,5 m/s para esta classe de aeronave. Qualquer valor
            // produziria o mesmo resultado; `1.0` evita threading
            // `PerformanceCfg` por uma assinatura que o controller fixou
            // sem esse parâmetro.
            let (rc_ms, vy_kmh) = climb_rate_ms(
                mass_kg, alt_m, req.isa_delta_c, wing, state, engine, 1.0,
            );
            if rc_ms <= RC_MIN_MS {
                return Err(MissionError::SubidaInviavel { altitude_m: alt_m, massa_kg: mass_kg, rc_ms });
            }

            let dt_s = step_m / rc_ms;
            let dt_h = dt_s / 3_600.0;

            // Potência de eixo (pós-PSRU, na hélice) à carga plena
            // (rpm_max_continuous) na altitude do passo, e BSFC nesse mesmo
            // ponto a carga ≈ 100% (subida é regime de potência plena, não
            // de cruzeiro).
            let p_shaft_kw = shaft_power_kw(engine, engine.rpm_max_continuous, alt_m,
                                             state.psru_efficiency);
            let bsfc_gkwh = engine.bsfc.bsfc_gkwh(engine.rpm_max_continuous, 1.0);
            // BSFC referencia o VIRABREQUIM (pré-PSRU) — ver "BSFC
            // referencia o virabrequim" no docstring do módulo (Finding 2
            // da revisão): recupera a potência de virabrequim dividindo a
            // potência de eixo pós-PSRU por η_PSRU antes de aplicar o BSFC.
            let p_crankshaft_kw = p_shaft_kw / state.psru_efficiency;
            // massa[g] = P[kW]·bsfc[g/kWh]·t[h]  →  massa[kg] = /1000
            let step_fuel_kg = p_crankshaft_kw * bsfc_gkwh * dt_h / 1_000.0;

            // Distância horizontal ≈ TAS·t (pequeno ângulo: TAS·cos γ ≈ TAS
            // — γ tipicamente < 10° para esta classe de aeronave, erro de
            // cosseno < 1,5%).
            let step_distance_m = (vy_kmh / 3.6) * dt_s;

            mass_kg -= step_fuel_kg;
            climb_time_s += dt_s;
            climb_distance_m += step_distance_m;
            fuel_climb_kg += step_fuel_kg;
            alt_m += step_m;
        }

        // ── 4. Descida (calculada antes do cruzeiro — a distância de
        //      descida é necessária para fechar a distância de cruzeiro) ──
        let delta_alt_m = req.cruise_altitude_m - req.airfield_altitude_m;
        let descent_time_s = delta_alt_m / req.analysis.descent_rate_ms;
        let descent_time_h = descent_time_s / 3_600.0;
        let v_cruise_ms = req.cruise_speed_min_kmh / 3.6;
        // Distância horizontal ≈ TAS·t — mesma aproximação de pequeno
        // ângulo do segmento de subida.
        let descent_distance_m = descent_time_s * v_cruise_ms;
        // Vazão de combustível de cruzeiro (kg/h) escalada pela fração de
        // potência parcial da descida — não recalcula BSFC a essa carga
        // reduzida (simplificação documentada em `AnalysisCfg::
        // descent_power_fraction`).
        let cruise_fuel_flow_kg_h = prop.fc_cruise_lph * density;
        let fuel_descent_kg =
            req.analysis.descent_power_fraction * cruise_fuel_flow_kg_h * descent_time_h;

        // ── 3. Cruzeiro (Breguet) ────────────────────────────────────────
        // Distância total exigida pela autonomia mínima da missão, à
        // velocidade de cruzeiro de projeto (km/h·h = km).
        let total_range_m = req.cruise_speed_min_kmh * req.endurance_min_h * 1_000.0;
        let cruise_distance_m = total_range_m - climb_distance_m - descent_distance_m;
        if cruise_distance_m <= 0.0 {
            return Err(MissionError::CruzeiroDistanciaNaoPositiva {
                climb_km: climb_distance_m / 1_000.0,
                descent_km: descent_distance_m / 1_000.0,
                exigido_km: total_range_m / 1_000.0,
            });
        }

        let mass_start_cruise_kg = mass_kg; // massa após táxi + subida
        let fuel_cruise_kg = breguet_fuel_burn_kg(
            mass_start_cruise_kg, cruise_distance_m, prop.prop_efficiency, state.psru_efficiency,
            wing.ld_ratio_cruise, prop.bsfc_cruise_gkwh,
        );

        // ── 5. Reserva ────────────────────────────────────────────────────
        // Fração sobre o consumo da missão SEM reserva (não uma fração
        // composta sobre o total já com reserva incluída).
        let subtotal_kg = fuel_taxi_kg + fuel_climb_kg + fuel_cruise_kg + fuel_descent_kg;
        let fuel_reserve_kg = req.fuel_reserve_fraction * subtotal_kg;
        let fuel_total_kg = subtotal_kg + fuel_reserve_kg;
        let fuel_total_l = fuel_total_kg / density;

        // ── Informativo: alcance Breguet queimando o TANQUE CHEIO inteiro
        //    (Finding 3 da revisão desta task — endpoints coerentes, NÃO
        //    `mtow_mission_kg`/`mtow_mission_kg − capacidade`, que combinava
        //    a massa da missão REAL — só ~229 L a bordo — com uma queima de
        //    260 L, produzindo `w1 < ZFW`, fisicamente incoerente).
        //
        //    Par coerente: parte-se do peso vazio de combustível (ZFW —
        //    OEW + payload, SEM nenhum combustível) com o TANQUE CHEIO
        //    (`w0`), e queima-se até `w1 = ZFW` (tanque vazio). Mostra o
        //    alcance MÁXIMO deste modelo (não a missão real, que reserva
        //    parte do tanque para táxi/subida/descida/reserva).
        //
        //    `MissionAgent::run` não recebe OEW diretamente (assinatura
        //    fixada pelo controller na Task 5.1: state/wing/prop/engine/
        //    req/mtow_mission_kg) — mas, por construção do laço de
        //    convergência (`orchestrator::size_aircraft`), no MTOW
        //    candidato `mtow_mission_kg = OEW + payload + fuel_total_kg`
        //    (exato no ponto convergido, ±`CONVERGENCE_TOL_KG` em
        //    iterações intermediárias) — então `mtow_mission_kg −
        //    fuel_total_kg` já é o ZFW, sem precisar de um parâmetro novo.
        let zfw_kg = mtow_mission_kg - fuel_total_kg;
        let full_tank_fuel_kg = state.fuel_capacity_l * density;
        let w0_full_tank_kg = zfw_kg + full_tank_fuel_kg;
        let breguet_range_full_tank_km = breguet_range_m(
            w0_full_tank_kg, zfw_kg, prop.prop_efficiency, state.psru_efficiency,
            wing.ld_ratio_cruise, prop.bsfc_cruise_gkwh,
        ) / 1_000.0;

        let cruise_time_h = (cruise_distance_m / 1_000.0) / req.cruise_speed_min_kmh;
        let block_time_h = (climb_time_s / 3_600.0) + cruise_time_h + descent_time_h;

        Ok(MissionSpec {
            fuel_taxi_kg,
            fuel_climb_kg,
            fuel_cruise_kg,
            fuel_descent_kg,
            fuel_reserve_kg,
            fuel_total_kg,
            fuel_total_l,
            climb_time_min: climb_time_s / 60.0,
            climb_distance_km: climb_distance_m / 1_000.0,
            descent_distance_km: descent_distance_m / 1_000.0,
            cruise_distance_km: cruise_distance_m / 1_000.0,
            block_time_h,
            // Recomputado a partir dos segmentos (não um eco de
            // `total_range_m`) — ver docstring de `MissionSpec::
            // range_no_wind_km`; por construção é igual a `total_range_m`
            // dentro de tolerância de ponto flutuante, já que
            // `cruise_distance_m` foi definido justamente para fechar essa
            // soma.
            range_no_wind_km:
                (climb_distance_m + cruise_distance_m + descent_distance_m) / 1_000.0,
            breguet_range_full_tank_km,
        })
    }
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::agents::aerodynamics::AerodynamicsAgent;
    use crate::agents::propulsion::PropulsionAgent;
    use crate::models::aircraft_config::test_fixtures::config_teste;
    use crate::models::engine::test_fixtures::{
        motor_generico_fraco_teste as engine_fraco_teste,
        motor_generico_teste as engine_teste,
    };
    use crate::models::requirements::test_fixtures::requisitos_teste;

    fn setup() -> (AircraftState, WingSpec, PropulsionSpec, EngineSpec, Requirements) {
        let cfg = config_teste();
        let state = AircraftState::from_config(&cfg);
        let req = requisitos_teste();
        let wing = AerodynamicsAgent::run(&state, &req);
        let engine = engine_teste();
        let prop = PropulsionAgent::run(&state, &req, &wing, &engine);
        (state, wing, prop, engine, req)
    }

    // ─── Breguet: sanidade dimensional ──────────────────────────────────

    /// η_PSRU de teste — valor típico de correia dentada (0,97, mesmo
    /// literal do antigo `[propeller] psru_efficiency` do baseline real),
    /// literal aqui para que os testes de sanidade abaixo não dependam
    /// silenciosamente de `state.psru_efficiency`/`config_teste()` (que usa
    /// 0,965, um valor sintético deliberadamente distinto — ver
    /// `aircraft_config::test_fixtures::config_teste`).
    const ETA_PSRU_TESTE: f64 = 0.97;

    #[test]
    fn breguet_fuel_burn_zero_para_distancia_zero() {
        let fuel = breguet_fuel_burn_kg(1_500.0, 0.0, 0.80, ETA_PSRU_TESTE, 13.0, 210.0);
        assert!(fuel.abs() < 1e-9, "combustível para 0 km deveria ser exatamente 0, obtido {fuel}");
    }

    #[test]
    fn breguet_fuel_burn_monotono_na_distancia() {
        let f1 = breguet_fuel_burn_kg(1_500.0, 500_000.0, 0.80, ETA_PSRU_TESTE, 13.0, 210.0);
        let f2 = breguet_fuel_burn_kg(1_500.0, 1_000_000.0, 0.80, ETA_PSRU_TESTE, 13.0, 210.0);
        let f3 = breguet_fuel_burn_kg(1_500.0, 2_000_000.0, 0.80, ETA_PSRU_TESTE, 13.0, 210.0);
        assert!(f1 < f2 && f2 < f3,
            "combustível Breguet deveria crescer estritamente com a distância: \
             {f1:.3} < {f2:.3} < {f3:.3}");
    }

    /// Combustível Breguet deve crescer conforme `η_PSRU` cai (mais perdas
    /// mecânicas → mais potência de virabrequim exigida para a mesma
    /// potência de eixo/tração) — sanidade direta do Finding 2 da revisão.
    #[test]
    fn breguet_fuel_burn_cresce_quando_eta_psru_cai() {
        let fuel_psru_alto = breguet_fuel_burn_kg(1_500.0, 1_000_000.0, 0.80, 0.99, 13.0, 210.0);
        let fuel_psru_baixo = breguet_fuel_burn_kg(1_500.0, 1_000_000.0, 0.80, 0.90, 13.0, 210.0);
        assert!(fuel_psru_baixo > fuel_psru_alto,
            "η_PSRU menor (mais perdas) deveria exigir MAIS combustível para o mesmo alcance: \
             η=0,90 → {fuel_psru_baixo:.3}kg, η=0,99 → {fuel_psru_alto:.3}kg");
    }

    /// Hand-check do docstring do módulo (dedução da equação de Breguet,
    /// ATUALIZADO na revisão desta task — Finding 2 — para incluir
    /// `η_PSRU`): R=2.000.000 m, bsfc=210 g/kWh, η_p=0,808, η_PSRU=0,97,
    /// L/D=13 → expoente≈0,11231 → W1/W0≈0,89379 → combustível ≈10,62% de
    /// W0. (Valor original do brief, sem η_PSRU: expoente≈0,1089,
    /// combustível ≈10,32% — não mais autoritativo, preservado aqui só
    /// como referência histórica do valor pré-correção.)
    #[test]
    fn breguet_hand_check_expoente_e_fracao_de_combustivel() {
        let w0 = 1_000.0; // massa arbitrária — a fração não depende de w0
        let ratio = breguet_mass_ratio(2_000_000.0, 0.808, ETA_PSRU_TESTE, 13.0, 210.0);
        let fuel = breguet_fuel_burn_kg(w0, 2_000_000.0, 0.808, ETA_PSRU_TESTE, 13.0, 210.0);
        let fuel_fraction = fuel / w0;

        println!("hand-check: w1/w0={ratio:.6}  fração de combustível={fuel_fraction:.6}");

        // expoente = 2e6 · 5,8333e-8 · 9,807 / (0,808·0,97·13) ≈ 0,11231
        let expoente_esperado =
            2_000_000.0 * (210.0 / 3.6e9) * G / (0.808 * ETA_PSRU_TESTE * 13.0);
        assert!((expoente_esperado - 0.11231).abs() < 0.001,
            "expoente hand-calculado {expoente_esperado:.6} diverge do valor da revisão \
             (~0,11231)");

        let ratio_esperado = (-expoente_esperado).exp();
        assert!((ratio - ratio_esperado).abs() / ratio_esperado < 0.005,
            "razão de massa {ratio:.6} do código diverge >0,5% do hand-check {ratio_esperado:.6}");
        assert!((ratio - 0.89379).abs() < 0.005,
            "razão de massa {ratio:.6} diverge >0,5% do valor da revisão (~0,89379)");
        assert!((fuel_fraction - 0.10621).abs() < 0.005,
            "fração de combustível {fuel_fraction:.6} diverge >0,5% do valor da revisão \
             (~0,10621)");
    }

    #[test]
    fn breguet_range_e_inverso_de_fuel_burn() {
        // Percorrendo R km e queimando `fuel`, o alcance Breguet calculado
        // a partir de w0/(w0-fuel) deve reproduzir R.
        let w0 = 1_500.0;
        let r_m = 1_800_000.0;
        let fuel = breguet_fuel_burn_kg(w0, r_m, 0.80, ETA_PSRU_TESTE, 13.0, 210.0);
        let w1 = w0 - fuel;
        let r_recuperado_m = breguet_range_m(w0, w1, 0.80, ETA_PSRU_TESTE, 13.0, 210.0);
        assert!((r_recuperado_m - r_m).abs() / r_m < 1e-6,
            "alcance recuperado {r_recuperado_m:.1} m diverge do original {r_m:.1} m");
    }

    #[test]
    fn breguet_range_degenerado_retorna_zero_sem_nan() {
        // w1 >= w0 (tanque mais pesado que o MTOW) é fisicamente degenerado
        // — deve retornar 0.0, não NaN/infinito.
        assert_eq!(breguet_range_m(1_000.0, 1_000.0, 0.80, ETA_PSRU_TESTE, 13.0, 210.0), 0.0);
        assert_eq!(breguet_range_m(1_000.0, 1_200.0, 0.80, ETA_PSRU_TESTE, 13.0, 210.0), 0.0);
        assert_eq!(breguet_range_m(1_000.0, 0.0, 0.80, ETA_PSRU_TESTE, 13.0, 210.0), 0.0);
        assert_eq!(breguet_range_m(1_000.0, -50.0, 0.80, ETA_PSRU_TESTE, 13.0, 210.0), 0.0);
    }

    // ─── Subida integrada ────────────────────────────────────────────────

    #[test]
    fn subida_queima_combustivel_positivo_e_tempo_plausivel() {
        let (state, wing, prop, engine, req) = setup();
        let mission = MissionAgent::run(&state, &wing, &prop, &engine, &req, state.mtow_kg)
            .expect("fixture sintética deveria produzir uma missão viável");

        println!(
            "climb: fuel={:.3}kg time={:.2}min dist={:.2}km",
            mission.fuel_climb_kg, mission.climb_time_min, mission.climb_distance_km
        );

        assert!(mission.fuel_climb_kg > 0.0, "combustível de subida deveria ser positivo");
        // Faixa plausível (5–15 min) para uma subida de aeronave leve desta
        // classe — valor observado empiricamente para a fixture sintética:
        // ~7.4 min.
        assert!(mission.climb_time_min > 5.0 && mission.climb_time_min < 15.0,
            "tempo de subida {:.2} min fora da faixa plausível (5, 15) min \
             para esta fixture sintética", mission.climb_time_min);
        assert!(mission.climb_distance_km > 0.0);
    }

    #[test]
    fn massa_decresce_estritamente_ao_longo_da_subida() {
        // Reimplementa a integração isoladamente (fora do agente) para
        // observar a massa passo a passo e confirmar que ela é
        // ESTRITAMENTE decrescente (nunca plana nem crescente).
        let (state, wing, _prop, engine, req) = setup();
        let fuel_taxi_kg = req.analysis.taxi_fuel_l * engine.fuel.density_kg_per_l;
        let mut mass_kg = state.mtow_kg - fuel_taxi_kg;
        let mut alt_m = req.airfield_altitude_m;
        let mut massas = vec![mass_kg];

        while alt_m < req.cruise_altitude_m - 1e-9 {
            let step_m = CLIMB_STEP_M.min(req.cruise_altitude_m - alt_m);
            let (rc_ms, _vy) = climb_rate_ms(mass_kg, alt_m, req.isa_delta_c, &wing, &state, &engine, 1.0);
            assert!(rc_ms > RC_MIN_MS, "fixture sintética deveria ter subida viável em todo o perfil");
            let dt_s = step_m / rc_ms;
            let p_shaft_kw = shaft_power_kw(&engine, engine.rpm_max_continuous, alt_m,
                                             state.psru_efficiency);
            let bsfc_gkwh = engine.bsfc.bsfc_gkwh(engine.rpm_max_continuous, 1.0);
            let p_crankshaft_kw = p_shaft_kw / state.psru_efficiency;
            let step_fuel_kg = p_crankshaft_kw * bsfc_gkwh * (dt_s / 3_600.0) / 1_000.0;
            mass_kg -= step_fuel_kg;
            massas.push(mass_kg);
            alt_m += step_m;
        }

        assert!(massas.len() > 1, "deveria haver mais de um passo de integração");
        for w in massas.windows(2) {
            assert!(w[1] < w[0],
                "massa deveria decrescer estritamente a cada passo: {:.6} → {:.6}", w[0], w[1]);
        }
    }

    #[test]
    fn subida_inviavel_com_motor_fraco_retorna_erro() {
        let cfg = config_teste();
        let state = AircraftState::from_config(&cfg);
        let req = requisitos_teste();
        let wing = AerodynamicsAgent::run(&state, &req);
        let engine = engine_fraco_teste();
        let prop = PropulsionAgent::run(&state, &req, &wing, &engine);

        let err = MissionAgent::run(&state, &wing, &prop, &engine, &req, state.mtow_kg)
            .expect_err("motor fraco (~52 kW de pico) não deveria sustentar a subida até \
                          a altitude de cruzeiro com este MTOW");
        println!("erro esperado: {err}");
        match err {
            MissionError::SubidaInviavel { rc_ms, .. } => {
                assert!(rc_ms <= RC_MIN_MS, "rc_ms do erro ({rc_ms}) deveria ser ≤ {RC_MIN_MS}");
            }
            other => panic!("esperava MissionError::SubidaInviavel, obtido: {other:?}"),
        }
    }

    // ─── Reserva ─────────────────────────────────────────────────────────

    #[test]
    fn reserva_e_fracao_exata_do_subtotal_sem_reserva() {
        let (state, wing, prop, engine, req) = setup();
        let mission = MissionAgent::run(&state, &wing, &prop, &engine, &req, state.mtow_kg)
            .expect("fixture sintética deveria produzir uma missão viável");

        let subtotal =
            mission.fuel_taxi_kg + mission.fuel_climb_kg + mission.fuel_cruise_kg + mission.fuel_descent_kg;
        let esperado = req.fuel_reserve_fraction * subtotal;
        assert!((mission.fuel_reserve_kg - esperado).abs() < 1e-9,
            "reserva {:.6} kg deveria ser EXATAMENTE {:.2}%·subtotal = {:.6} kg",
            mission.fuel_reserve_kg, req.fuel_reserve_fraction * 100.0, esperado);
        assert!((mission.fuel_total_kg - (subtotal + mission.fuel_reserve_kg)).abs() < 1e-9);
    }

    // ─── Sanidade geral do MissionSpec ──────────────────────────────────

    #[test]
    fn todos_os_segmentos_sao_positivos_e_total_e_a_soma() {
        let (state, wing, prop, engine, req) = setup();
        let mission = MissionAgent::run(&state, &wing, &prop, &engine, &req, state.mtow_kg)
            .expect("fixture sintética deveria produzir uma missão viável");

        for (nome, v) in [
            ("fuel_taxi_kg", mission.fuel_taxi_kg),
            ("fuel_climb_kg", mission.fuel_climb_kg),
            ("fuel_cruise_kg", mission.fuel_cruise_kg),
            ("fuel_descent_kg", mission.fuel_descent_kg),
            ("fuel_reserve_kg", mission.fuel_reserve_kg),
        ] {
            assert!(v > 0.0 && v.is_finite(), "{nome} = {v} deveria ser positivo e finito");
        }

        let soma = mission.fuel_taxi_kg + mission.fuel_climb_kg + mission.fuel_cruise_kg
            + mission.fuel_descent_kg + mission.fuel_reserve_kg;
        assert!((mission.fuel_total_kg - soma).abs() < 1e-9,
            "fuel_total_kg ({:.6}) deveria ser a soma exata dos segmentos ({:.6})",
            mission.fuel_total_kg, soma);
        assert!((mission.fuel_total_l - mission.fuel_total_kg / engine.fuel.density_kg_per_l).abs() < 1e-9);

        // Alcance recomputado a partir dos segmentos deve bater com o
        // alcance exigido (km/h·h) dentro de tolerância de ponto flutuante
        // — ver docstring de `MissionSpec::range_no_wind_km`.
        let exigido_km = req.cruise_speed_min_kmh * req.endurance_min_h;
        assert!((mission.range_no_wind_km - exigido_km).abs() < 1e-6,
            "range_no_wind_km ({:.6}) deveria bater o alcance exigido ({:.6}) por construção",
            mission.range_no_wind_km, exigido_km);

        assert!(mission.breguet_range_full_tank_km > 0.0);
        assert!(mission.block_time_h > 0.0);
    }

    #[test]
    fn cruzeiro_distancia_nao_positiva_retorna_erro() {
        // Autonomia mínima tão curta que subida+descida (que têm distância
        // fixa dada a geometria da missão) já excedem sozinhas o alcance
        // exigido — não sobra nada para o cruzeiro.
        let (state, wing, prop, engine, mut req) = setup();
        req.endurance_min_h = 0.01; // ~36s de voo "exigido" a 260km/h ≈ 2.6km

        let err = MissionAgent::run(&state, &wing, &prop, &engine, &req, state.mtow_kg)
            .expect_err("autonomia mínima absurdamente curta deveria estourar a distância \
                          obrigatória de subida+descida");
        println!("erro esperado: {err}");
        assert!(matches!(err, MissionError::CruzeiroDistanciaNaoPositiva { .. }));
    }
}
