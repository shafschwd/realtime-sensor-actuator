use parking_lot::{Mutex, RwLock};
use std::sync::Arc;
use std::collections::VecDeque;

// Shared diagnostic log (demonstrates lock contention measurement)
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

    pub fn read_all(&self) -> Vec<String> {
        self.entries.read().iter().cloned().collect()
    }

    #[allow(dead_code)]
    pub fn clear(&self) {
        self.entries.write().clear();
    }
}

// Shared configuration buffer (demonstrates Mutex usage)
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
                sensor_interval_ms: 5,
                processing_deadline_ms: 0.2,
                transmission_deadline_ms: 0.1,
                fail_safe_enabled: false,
            })),
        }
    }

    pub fn update<F>(&self, f: F)
    where
        F: FnOnce(&mut SystemConfig),
    {
        let mut config = self.data.lock();
        f(&mut *config);
    }

    pub fn get(&self) -> SystemConfig {
        self.data.lock().clone()
    }
}
