use serde::Deserialize;
use std::fs;

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
            sensor_interval_ms: 5,
            processing_deadline_ms: 0.2,
            transmission_deadline_ms: 0.1,
            fail_safe_enabled: false,
        }
    }
}

pub fn load_config(path: &str) -> RuntimeConfig {
    match fs::read_to_string(path) {
        Ok(s) => toml::from_str::<RuntimeConfig>(&s).unwrap_or_default(),
        Err(_) => RuntimeConfig::default(),
    }
}
