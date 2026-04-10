/// Requisitos imutáveis do projeto — originados das premissas do cliente.
/// Todos os agentes validam seus resultados contra esta estrutura.
#[derive(Debug, Clone)]
pub struct Requirements {
    /// Número de passageiros adultos
    pub passengers: u32,
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
}

impl Requirements {
    pub fn project_default() -> Self {
        Self {
            passengers: 4,
            baggage_kg: 80.0,
            cruise_speed_min_kmh: 280.0,
            endurance_min_h: 8.0,
            fuel_reserve_fraction: 0.10,  // 10% + 45 min embutidos no endurance
            cruise_altitude_m: 2_500.0,
        }
    }

    /// Payload total em kg (passageiros a 90 kg cada + bagagem)
    pub fn payload_kg(&self) -> f64 {
        self.passengers as f64 * 90.0 + self.baggage_kg
    }
}
