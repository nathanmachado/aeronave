//! `Requirements` — estrutura desserializável que espelha `mission.toml`.
//!
//! Mesma filosofia de `EngineSpec`/`AircraftConfig`: os requisitos de missão
//! (passageiros, autonomia, velocidade de cruzeiro, altitude, reservas) são
//! dado de configuração, não constante Rust. Trocar de missão agora é trocar
//! `config/missions/*.toml`, não o código.
//!
//! O parsing/validação vivem em `models::config` (`parse_mission`,
//! `load_mission`), ao lado dos loaders de motor e de célula, reaproveitando
//! `ConfigError`. Este módulo só contém o tipo de dado e (em teste) uma
//! fixture sintética.

use serde::{Deserialize, Serialize};

/// Requisitos imutáveis do projeto — originados das premissas do cliente.
/// Todos os agentes validam seus resultados contra esta estrutura.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Requirements {
    /// Número de passageiros adultos
    pub passengers: u32,
    /// Massa por passageiro em kg
    pub pax_mass_kg: f64,
    /// Bagagem total em kg
    pub baggage_kg: f64,
    /// Velocidade de cruzeiro mínima em km/h
    pub cruise_speed_min_kmh: f64,
    /// Autonomia mínima em horas (sem reservas)
    pub endurance_min_h: f64,
    /// Reserva de combustível (fração — ex: 0.10 = 10% extra)
    pub fuel_reserve_fraction: f64,
    /// Altitude de cruzeiro alvo em metros
    pub cruise_altitude_m: f64,
    /// Altitude do aeródromo de referência em metros
    pub airfield_altitude_m: f64,
    /// Desvio ISA em °C (ex.: ISA+20 → 20.0)
    pub isa_delta_c: f64,
    /// Margem mínima de combustível exigida (Task 3, refino-ciclo2) — fração
    /// da CAPACIDADE do tanque (`[fuel_system].capacity_l`), não do
    /// combustível exigido pela missão (ver nota de convenção em
    /// `validation::constraint_checker::ConstraintChecker::verify`, checagem
    /// #18, e `tests/generic_engine.rs::margem_de_combustivel_no_mtow_
    /// convergido`, que documenta as DUAS percentagens que coexistem no
    /// projeto). É um requisito de PROJETO (piso de segurança operacional —
    /// combustível não planejado para contingência além da reserva já
    /// embutida em `fuel_reserve_fraction`), não uma propriedade física
    /// calculada — por isso vive em `Requirements`/`mission.toml`, ao lado
    /// de `fuel_reserve_fraction`. Faixa válida: [0, 0.3] (acima de 30%
    /// seria um piso irrealisticamente alto, provavelmente um erro de
    /// digitação de fração vs. percentual).
    pub min_fuel_margin_fraction: f64,
    /// Parâmetros da análise de missão por segmentos (Task 5.1) — táxi,
    /// subida integrada e descida. Ver `AnalysisCfg`.
    pub analysis: AnalysisCfg,
}

impl Requirements {
    /// Payload total em kg (passageiros a `pax_mass_kg` cada + bagagem)
    pub fn payload_kg(&self) -> f64 {
        self.passengers as f64 * self.pax_mass_kg + self.baggage_kg
    }
}

/// Parâmetros da análise de missão por segmentos (Task 5.1,
/// `agents::mission::MissionAgent`) — substituem o modelo antigo de
/// consumo constante (`fc_cruise_lph · endurance`) por uma missão
/// segmentada: táxi (fração fixa), subida integrada (RC × consumo, passo
/// 100m), cruzeiro (Breguet, massa decrescente) e descida (potência
/// parcial). São parâmetros da MISSÃO (não da célula/motor), portanto
/// vivem em `config/missions/*.toml` ao lado de `Requirements`, não em
/// `AircraftConfig`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnalysisCfg {
    /// Combustível fixo de táxi + run-up antes da decolagem (litros) — não
    /// é modelado por tempo/potência, é um valor fixo de projeto (típico
    /// para esta classe de aeronave).
    pub taxi_fuel_l: f64,
    /// Taxa de descida de projeto (m/s) — define o tempo (e, por
    /// aproximação de pequeno ângulo, a distância horizontal) do segmento
    /// de descida.
    pub descent_rate_ms: f64,
    /// Fração da potência/vazão de combustível de CRUZEIRO usada durante a
    /// descida (potência parcial, não motor cortado) — ex.: 0.20 = 20% da
    /// vazão de combustível de cruzeiro. Faixa válida: [0.05, 0.5] (abaixo
    /// de 5% seria efetivamente motor cortado/idle, não modelado; acima de
    /// 50% não seria uma descida de baixa potência).
    pub descent_power_fraction: f64,
    /// Política de velocidade de subida. Hoje só `"vy"` (melhor razão de
    /// subida, `agents::performance::climb_rate_ms`) é suportada — reservado
    /// para uma futura política `"vx"` (melhor ângulo) ou velocidade fixa,
    /// daí ser uma string validada em vez de um booleano.
    pub climb_speed_policy: String,
}

/// Fixture sintética de `Requirements` para uso em testes de `src/` —
/// mesma filosofia de `aircraft_config::test_fixtures::config_teste` e
/// `engine::test_fixtures::motor_generico_teste`: valores plausíveis mas
/// deliberadamente distintos do baseline real (`config/missions/default.toml`),
/// para que os testes de `src/` não fiquem acoplados ao arquivo real e cada
/// teste ainda seja capaz de falhar (não são cópias do TOML de produção).
#[cfg(test)]
pub mod test_fixtures {
    use super::*;

    pub fn requisitos_teste() -> Requirements {
        Requirements {
            passengers: 4,
            pax_mass_kg: 85.0,
            baggage_kg: 60.0,
            cruise_speed_min_kmh: 260.0,
            endurance_min_h: 6.0,
            fuel_reserve_fraction: 0.10,
            cruise_altitude_m: 2_400.0,
            airfield_altitude_m: 0.0,
            isa_delta_c: 0.0,
            // Distinto do valor real de `config/missions/default.toml`
            // (0.05) — mesma filosofia de fixture sintética das demais
            // (ver docstring do módulo).
            min_fuel_margin_fraction: 0.08,
            analysis: AnalysisCfg {
                taxi_fuel_l: 2.5,
                descent_rate_ms: 3.5,
                descent_power_fraction: 0.25,
                climb_speed_policy: "vy".to_string(),
            },
        }
    }
}
