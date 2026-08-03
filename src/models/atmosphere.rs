//! Atmosfera Padrão Internacional (ISA / ICAO Doc 7488) — camada troposférica.
//!
//! Substitui a aproximação exponencial de densidade
//! (`ρ = ρ_SL·(1 − 2,26e-5·h)^4,256`, usada até a Task 4.5) por um modelo ISA
//! completo: temperatura, pressão, densidade e velocidade do som, todos
//! derivados do mesmo perfil de temperatura linear da troposfera.
//!
//! Válido para 0–11.000 m (troposfera, até a tropopausa). Acima disso a ISA
//! muda para um perfil isotérmico (`T` constante) que este módulo NÃO
//! modela — os chamadores devem se manter dentro da faixa; nenhum clamp é
//! aplicado (decisão de projeto: altitudes fora da faixa são um erro de
//! configuração de missão/desempenho, não algo a mascarar silenciosamente).
//!
//! Referências:
//!   - ICAO Doc 7488 — "Manual of the ICAO Standard Atmosphere"
//!   - ISO 2533:1975 — "Standard Atmosphere"
//!   - Anderson, J. "Introduction to Flight", Cap. 3

/// Densidade do ar ISA ao nível do mar, dia padrão (kg/m³).
/// Fonte única (single source of truth) — outros módulos importam esta
/// constante em vez de repetir o literal `1.225`.
pub const RHO_SL: f64 = 1.225;

/// Temperatura ISA ao nível do mar, dia padrão (K) — 15°C.
const T_SL_K: f64 = 288.15;

/// Gradiente térmico da troposfera ISA (K/m) — queda de 6,5°C a cada 1.000 m.
const LAPSE_RATE_K_PER_M: f64 = 0.0065;

/// Pressão ISA ao nível do mar, dia padrão (Pa).
const P_SL_PA: f64 = 101_325.0;

/// Constante específica do ar seco (J/(kg·K)).
const R_AIR: f64 = 287.05;

/// Expoente barométrico da troposfera ISA: g/(R·L) = 9,80665/(287,05·0,0065).
const BAROMETRIC_EXPONENT: f64 = 5.2561;

/// Razão de calores específicos do ar seco (γ), usada na velocidade do som.
const GAMMA_AIR: f64 = 1.4;

/// Atmosfera Padrão Internacional (ISA) — funções puras, sem estado.
///
/// Válidas para `0.0 <= h_m <= 11_000.0` (troposfera). Nenhuma das funções
/// abaixo faz clamp ou valida a faixa — é responsabilidade do chamador
/// manter `h_m` dentro da troposfera (todas as altitudes de missão/
/// desempenho deste projeto ficam bem abaixo de 11.000 m).
pub struct Isa;

impl Isa {
    /// Temperatura do ar em altitude `h_m` (metros), incluindo o desvio ISA
    /// `isa_delta_c` (°C — ex.: ISA+20 → `isa_delta_c = 20.0`).
    ///
    /// T = 288,15 − 0,0065·h + ΔISA
    ///
    /// O desvio ISA desloca a temperatura real em relação ao dia padrão
    /// (dia quente/frio) SEM alterar o perfil de pressão (ver `pressure_pa`)
    /// — fisicamente, pressão é uma função de altitude geopotencial/pressão
    /// barométrica, não da temperatura local do dia.
    pub fn temperature_k(h_m: f64, isa_delta_c: f64) -> f64 {
        T_SL_K - LAPSE_RATE_K_PER_M * h_m + isa_delta_c
    }

    /// Pressão atmosférica em altitude `h_m` (metros).
    ///
    /// p = 101.325·(T_padrão(h)/288,15)^5,2561
    ///
    /// Deliberadamente NÃO recebe `isa_delta_c`: pressão é altitude de
    /// pressão pura (ISA padrão), independente do desvio de temperatura do
    /// dia. Um dia ISA+20 tem a MESMA pressão que um dia ISA padrão na mesma
    /// altitude geométrica — o que muda é a densidade (via T na equação de
    /// estado dos gases ideais, `density_kgm3`), não a pressão. Esta é a
    /// distinção clássica "altitude de pressão" vs. "desvio de temperatura"
    /// usada em performance de aeronaves (ICAO Doc 7488 §2).
    pub fn pressure_pa(h_m: f64) -> f64 {
        let t_padrao = T_SL_K - LAPSE_RATE_K_PER_M * h_m;
        P_SL_PA * (t_padrao / T_SL_K).powf(BAROMETRIC_EXPONENT)
    }

    /// Densidade do ar em altitude `h_m` (metros), com desvio ISA `isa_delta_c`.
    ///
    /// ρ = p / (R·T)   (equação de estado dos gases ideais)
    ///
    /// Nota: um dia mais quente (ΔISA > 0) reduz a densidade a uma dada
    /// pressão (T maior no denominador) — é assim que um "dia quente"
    /// degrada a decolagem/subida (menos massa de ar por volume → menos
    /// sustentação e tração disponíveis a uma dada velocidade indicada).
    pub fn density_kgm3(h_m: f64, isa_delta_c: f64) -> f64 {
        let p = Self::pressure_pa(h_m);
        let t = Self::temperature_k(h_m, isa_delta_c);
        p / (R_AIR * t)
    }

    /// Velocidade do som em altitude `h_m` (metros), com desvio ISA `isa_delta_c`.
    ///
    /// a = √(γ·R·T)
    pub fn speed_of_sound_ms(h_m: f64, isa_delta_c: f64) -> f64 {
        let t = Self::temperature_k(h_m, isa_delta_c);
        (GAMMA_AIR * R_AIR * t).sqrt()
    }
}

// ─── TESTES UNITÁRIOS ────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // Tabela ISA de referência (ICAO Doc 7488 / ISO 2533), h = 2.500 m:
    //   T = 288,15 − 16,25       = 271,90 K
    //   p = 101.325·(271,90/288,15)^5,2561 ≈ 74.691,8 Pa
    //   ρ = p/(287,05·271,90)    ≈ 0,95695 kg/m³
    //   a = √(1,4·287,05·271,90) ≈ 330,56 m/s

    #[test]
    fn temperatura_bate_tabela_isa_2500m() {
        let t = Isa::temperature_k(2_500.0, 0.0);
        assert!((t - 271.90).abs() < 0.1, "T(2500m) = {t:.3} K, esperado 271,90 K ±0,1");
    }

    #[test]
    fn pressao_bate_tabela_isa_2500m() {
        let p = Isa::pressure_pa(2_500.0);
        assert!((p - 74_691.8).abs() < 50.0, "p(2500m) = {p:.1} Pa, esperado 74.691,8 Pa ±50");
    }

    #[test]
    fn densidade_bate_tabela_isa_2500m() {
        let rho = Isa::density_kgm3(2_500.0, 0.0);
        assert!((rho - 0.957).abs() < 0.001, "ρ(2500m) = {rho:.5} kg/m³, esperado 0,957 ±0,001");
    }

    #[test]
    fn velocidade_do_som_bate_tabela_isa_2500m() {
        let a = Isa::speed_of_sound_ms(2_500.0, 0.0);
        assert!((a - 330.6).abs() < 0.5, "a(2500m) = {a:.2} m/s, esperado 330,6 m/s ±0,5");
    }

    #[test]
    fn densidade_ao_nivel_do_mar_dia_padrao() {
        let rho = Isa::density_kgm3(0.0, 0.0);
        assert!((rho - 1.225_00).abs() < 0.0005,
            "ρ(0m, ISA) = {rho:.5} kg/m³, esperado 1,22500 ±0,0005");
        // Consistente com a constante RHO_SL usada como fonte única em todo
        // o crate.
        assert!((rho - RHO_SL).abs() < 0.001);
    }

    #[test]
    fn temperatura_ao_nivel_do_mar_dia_padrao() {
        assert!((Isa::temperature_k(0.0, 0.0) - 288.15).abs() < 1e-9);
    }

    #[test]
    fn dia_quente_reduz_densidade_no_solo() {
        // ISA+20 no solo: T = 288,15+20 = 308,15 K; p inalterado (101.325 Pa);
        // ρ = 101.325/(287,05·308,15) ≈ 1,1455 kg/m³ — menor que o dia padrão,
        // degradando decolagem (menos sustentação/tração a uma dada V).
        let rho_padrao = Isa::density_kgm3(0.0, 0.0);
        let rho_quente = Isa::density_kgm3(0.0, 20.0);
        assert!(rho_quente < rho_padrao,
            "dia quente (ISA+20) deveria ter densidade menor que o dia padrão: \
             quente={rho_quente:.5}, padrão={rho_padrao:.5}");
        assert!((rho_quente - 1.1455).abs() < 0.001,
            "ρ(0m, ISA+20) = {rho_quente:.5} kg/m³, esperado 1,1455 ±0,001");
    }

    #[test]
    fn pressao_independe_do_desvio_isa() {
        // Pressão é altitude de pressão pura — dia quente ou frio na MESMA
        // altitude geométrica tem a MESMA pressão (só a temperatura/densidade
        // mudam). `pressure_pa` nem recebe `isa_delta_c` — este teste
        // documenta a intenção via `density_kgm3`, comparando a pressão
        // implícita (p = ρ·R·T) em dois desvios ISA diferentes.
        let p_via_padrao = Isa::density_kgm3(1_000.0, 0.0) * R_AIR * Isa::temperature_k(1_000.0, 0.0);
        let p_via_quente  = Isa::density_kgm3(1_000.0, 15.0) * R_AIR * Isa::temperature_k(1_000.0, 15.0);
        assert!((p_via_padrao - p_via_quente).abs() < 1e-6,
            "pressão implícita deveria ser idêntica independente do ΔISA: \
             padrão={p_via_padrao:.3} Pa, quente={p_via_quente:.3} Pa");
    }

    #[test]
    fn densidade_decresce_com_altitude() {
        assert!(Isa::density_kgm3(2_500.0, 0.0) < Isa::density_kgm3(0.0, 0.0));
        assert!(Isa::density_kgm3(5_000.0, 0.0) < Isa::density_kgm3(2_500.0, 0.0));
    }

    #[test]
    fn velocidade_do_som_decresce_com_altitude() {
        // a depende só de T, que cai com a altitude na troposfera.
        assert!(Isa::speed_of_sound_ms(5_000.0, 0.0) < Isa::speed_of_sound_ms(0.0, 0.0));
    }
}
