//! IPC module - Inter-process communication (channels and shared resources)

use crossbeam::channel::{Sender, Receiver, bounded};
use parking_lot::{Mutex, RwLock};
use std::sync::Arc;
use std::collections::VecDeque;
use std::time::Instant;
use crate::sensor::SensorReading;
use crate::actuator::ActuatorStatus;

// ============================================================================
// SYSTEM CHANNELS - Communication between sensor and actuator threads
// ============================================================================

#[derive(Clone)]
pub struct SystemChannels {
    pub sensor_tx: Sender<SensorReading>,
    pub sensor_rx: Arc<Receiver<SensorReading>>,
    pub feedback_tx: Sender<ActuatorFeedback>,
    pub feedback_rx: Arc<Receiver<ActuatorFeedback>>,
}

impl SystemChannels {
    pub fn new(buffer_size: usize) -> Self {
        let (sensor_tx, sensor_rx) = bounded(buffer_size);
        let (feedback_tx, feedback_rx) = bounded(buffer_size);

        Self {
            sensor_tx,
            sensor_rx: Arc::new(sensor_rx),
            feedback_tx,
            feedback_rx: Arc::new(feedback_rx),
        }
    }
}

// ============================================================================
// ACTUATOR FEEDBACK - Feedback from actuators to sensor
// ============================================================================

#[derive(Clone, Debug)]
pub struct ActuatorFeedback {
    pub timestamp: Instant,
    pub actuator_name: String,
    pub error: f32,
    pub control: f32,
    pub status: ActuatorStatus,
    pub cycle_id: u64,
}

// ============================================================================
// DIAGNOSTIC LOG - Thread-safe logging with RwLock
// ============================================================================

#[derive(Clone)]
pub struct DiagnosticLog {
    entries: Arc<RwLock<VecDeque<String>>>,
    max_size: usize,
}

impl DiagnosticLog {
    pub fn new(max_size: usize) -> Self {
        Self {
            entries: Arc::new(RwLock::new(VecDeque::with_capacity(max_size))),
            max_size,
        }
    }

    pub fn write(&self, message: String) {
        let mut log = self.entries.write();
        log.push_back(message);
        if log.len() > self.max_size {
            log.pop_front();
        }
    }
}

// ============================================================================
// CONFIG BUFFER - Thread-safe configuration with Mutex
// ============================================================================

#[derive(Clone)]
pub struct ConfigBuffer {
    data: Arc<Mutex<SystemConfig>>,
}

#[derive(Clone, Debug)]
pub struct SystemConfig {
    pub sensor_interval_ms: u64,
    pub processing_deadline_ms: f64,
    pub transmission_deadline_ms: f64,
    pub fail_safe_enabled: bool,
}

impl ConfigBuffer {
    pub fn new() -> Self {
        Self {
            data: Arc::new(Mutex::new(SystemConfig {
                sensor_interval_ms: 10,
                processing_deadline_ms: 0.2,
                transmission_deadline_ms: 0.1,
                fail_safe_enabled: false,
            })),
        }
    }

    pub fn update<F>(&self, f: F) where F: FnOnce(&mut SystemConfig) {
        let mut config = self.data.lock();
        f(&mut *config);
    }

    pub fn get(&self) -> SystemConfig {
        self.data.lock().clone()
    }
}

// ============================================================================
// CONFIG FILE LOADING
// ============================================================================

use serde::Deserialize;

#[derive(Debug, Clone, Deserialize)]
pub struct RuntimeConfig {
    pub sensor_interval_ms: u64,
    pub processing_deadline_ms: f64,
    pub transmission_deadline_ms: f64,
    pub fail_safe_enabled: bool,
}

impl Default for RuntimeConfig {
    fn default() -> Self {
        Self {
            sensor_interval_ms: 10,
            processing_deadline_ms: 0.2,
            transmission_deadline_ms: 0.1,
            fail_safe_enabled: false,
        }
    }
}

pub fn load_config(path: &str) -> RuntimeConfig {
    match std::fs::read_to_string(path) {
        Ok(s) => toml::from_str::<RuntimeConfig>(&s).unwrap_or_default(),
        Err(_) => RuntimeConfig::default(),
    }
}

