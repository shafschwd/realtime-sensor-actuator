// Configuration loading/management (minimal stub)
#[derive(Debug, Clone)]
pub struct SystemConfig {
    pub sample_rate_hz: u32,
    pub window_size: usize,
    pub anomaly_threshold: f64,
}

impl Default for SystemConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 100,
            window_size: 5,
            anomaly_threshold: 2.5,
        }
    }
}

pub fn load_default() -> SystemConfig {
    SystemConfig::default()
}

