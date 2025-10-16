// Shared diagnostic log/buffer (Arc<Mutex<Vec<String>>>)
use std::sync::{Arc, Mutex};

#[derive(Clone, Default)]
pub struct SharedLog(Arc<Mutex<Vec<String>>>);

impl SharedLog {
    pub fn new() -> Self { Self::default() }

    pub fn push(&self, line: impl Into<String>) {
        if let Ok(mut g) = self.0.lock() {
            g.push(line.into());
        }
    }

    pub fn snapshot(&self) -> Vec<String> {
        self.0.lock().map(|g| g.clone()).unwrap_or_default()
    }
}

