// Latency, jitter, throughput tracking (stub)
#[derive(Default, Debug, Clone)]
pub struct Metrics {
    pub count: u64,
    pub sum: f64,
}

impl Metrics {
    pub fn record(&mut self, v: f64) {
        self.count += 1;
        self.sum += v;
    }

    pub fn mean(&self) -> f64 {
        if self.count == 0 { 0.0 } else { self.sum / self.count as f64 }
    }
}

