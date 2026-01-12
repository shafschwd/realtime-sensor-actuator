//! Sensor module - Data generation, filtering, and anomaly detection

use std::time::Instant;
use std::collections::VecDeque;
use rand::{Rng, SeedableRng};
use rand::rngs::StdRng;

// ============================================================================
// SENSOR READING
// ============================================================================

#[derive(Debug, Clone, Copy)]
pub struct SensorReading {
    pub timestamp: Instant,
    pub force: f32,
    pub position: f32,
    pub temperature: f32,
    pub sequence_id: u64,
}

// ============================================================================
// SENSOR GENERATOR - Simulates real-time sensor data
// ============================================================================

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
            base_position: 0.0,
            base_temp: 25.0,
            noise_amplitude: 10.0,
        }
    }

    pub fn generate(&mut self) -> SensorReading {
        self.sequence_counter += 1;
        let noise_force = self.rng.gen_range(-self.noise_amplitude..self.noise_amplitude);

        SensorReading {
            timestamp: Instant::now(),
            force: self.base_force + noise_force,
            position: self.base_position,
            temperature: self.base_temp,
            sequence_id: self.sequence_counter,
        }
    }
}

// ============================================================================
// MOVING AVERAGE FILTER - Noise reduction
// ============================================================================

pub struct MovingAverageFilter {
    window_size: usize,
    force_buffer: VecDeque<f32>,
    position_buffer: VecDeque<f32>,
    temp_buffer: VecDeque<f32>,
}

impl MovingAverageFilter {
    pub fn new(window_size: usize) -> Self {
        Self {
            window_size,
            force_buffer: VecDeque::with_capacity(window_size),
            position_buffer: VecDeque::with_capacity(window_size),
            temp_buffer: VecDeque::with_capacity(window_size),
        }
    }

    pub fn filter(&mut self, reading: &SensorReading) -> SensorReading {
        self.force_buffer.push_back(reading.force);
        self.position_buffer.push_back(reading.position);
        self.temp_buffer.push_back(reading.temperature);

        if self.force_buffer.len() > self.window_size {
            self.force_buffer.pop_front();
            self.position_buffer.pop_front();
            self.temp_buffer.pop_front();
        }

        let avg_force: f32 = self.force_buffer.iter().sum::<f32>() / self.force_buffer.len() as f32;
        let avg_pos: f32 = self.position_buffer.iter().sum::<f32>() / self.position_buffer.len() as f32;
        let avg_temp: f32 = self.temp_buffer.iter().sum::<f32>() / self.temp_buffer.len() as f32;

        SensorReading {
            force: avg_force,
            position: avg_pos,
            temperature: avg_temp,
            ..*reading
        }
    }
}

// ============================================================================
// ANOMALY DETECTOR - Detects abnormal sensor readings
// ============================================================================

pub struct AnomalyDetector {
    force_threshold: f32,
    position_threshold: f32,
    temp_threshold: f32,
}

impl AnomalyDetector {
    pub fn new(force_thresh: f32, pos_thresh: f32, temp_thresh: f32) -> Self {
        Self {
            force_threshold: force_thresh,
            position_threshold: pos_thresh,
            temp_threshold: temp_thresh,
        }
    }

    pub fn detect(&self, reading: &SensorReading, baseline: &SensorReading) -> Option<String> {
        if (reading.force - baseline.force).abs() > self.force_threshold {
            return Some(format!("Force anomaly: {:.2} N (baseline: {:.2} N)",
                                reading.force, baseline.force));
        }
        if (reading.position - baseline.position).abs() > self.position_threshold {
            return Some(format!("Position anomaly: {:.2} mm", reading.position));
        }
        if (reading.temperature - baseline.temperature).abs() > self.temp_threshold {
            return Some(format!("Temperature anomaly: {:.2} °C", reading.temperature));
        }
        None
    }
}

