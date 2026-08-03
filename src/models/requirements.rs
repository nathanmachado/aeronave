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
}

impl Requirements {
    /// Payload total em kg (passageiros a `pax_mass_kg` cada + bagagem)
    pub fn payload_kg(&self) -> f64 {
        self.passengers as f64 * self.pax_mass_kg + self.baggage_kg
    }
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
        }
    }
}
