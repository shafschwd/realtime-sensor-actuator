use std::time::Instant;
use rand::{Rng, SeedableRng};
use rand::rngs::StdRng;

#[derive(Debug, Clone, Copy)]
pub struct SensorReading {
    pub timestamp: Instant,
    pub force: f32,
    pub position: f32,
    pub temperature: f32,
    #[allow(dead_code)]
    pub sequence_id: u64,
}

pub struct SensorGenerator {
    rng: StdRng,
    sequence_counter: u64,
    pub base_force: f32,
    pub base_position: f32,
    pub base_temp: f32,
    pub noise_amplitude: f32,
}

impl SensorGenerator {
    pub fn new(seed: u64) -> Self {
        Self {
            rng: StdRng::seed_from_u64(seed),
            sequence_counter: 0,
            base_force: 50.0,
            base_position: 100.0,
            base_temp: 25.0,
            noise_amplitude: 2.0,
        }
    }

    pub fn generate(&mut self) -> SensorReading {
        let mut noise = || self.rng.gen_range(-self.noise_amplitude..self.noise_amplitude);
        self.sequence_counter += 1;
        SensorReading {
            timestamp: Instant::now(),
            force: self.base_force + noise(),
            position: self.base_position + noise() * 0.5,
            temperature: self.base_temp + noise() * 0.1,
            sequence_id: self.sequence_counter,
        }
    }

    #[allow(dead_code)]
    pub fn inject_disturbance(&mut self, force_delta: f32, pos_delta: f32) {
        self.base_force += force_delta;
        self.base_position += pos_delta;
    }
}
