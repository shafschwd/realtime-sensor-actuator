//! Metrics module - Performance tracking and statistics

use hdrhistogram::Histogram;
use std::time::Duration;
use parking_lot::Mutex;
use std::sync::Arc;

// ============================================================================
// TIMING METRICS - Thread-safe performance tracking
// ============================================================================

#[derive(Clone)]
pub struct TimingMetrics {
    generation_hist: Arc<Mutex<Histogram<u64>>>,
    processing_hist: Arc<Mutex<Histogram<u64>>>,
    transmission_hist: Arc<Mutex<Histogram<u64>>>,
    e2e_hist: Arc<Mutex<Histogram<u64>>>,
}

impl TimingMetrics {
    pub fn new() -> Self {
        Self {
            generation_hist: Arc::new(Mutex::new(Histogram::new(3).unwrap())),
            processing_hist: Arc::new(Mutex::new(Histogram::new(3).unwrap())),
            transmission_hist: Arc::new(Mutex::new(Histogram::new(3).unwrap())),
            e2e_hist: Arc::new(Mutex::new(Histogram::new(3).unwrap())),
        }
    }

    pub fn record_generation(&self, duration: Duration) {
        self.generation_hist.lock().record(duration.as_nanos() as u64).ok();
    }

    pub fn record_processing(&self, duration: Duration) {
        self.processing_hist.lock().record(duration.as_nanos() as u64).ok();
    }

    pub fn record_transmission(&self, duration: Duration) {
        self.transmission_hist.lock().record(duration.as_nanos() as u64).ok();
    }

    pub fn record_e2e(&self, duration: Duration) {
        self.e2e_hist.lock().record(duration.as_nanos() as u64).ok();
    }

    pub fn report(&self) -> MetricsReport {
        let gen = self.generation_hist.lock();
        let proc = self.processing_hist.lock();
        let e2e = self.e2e_hist.lock();

        MetricsReport {
            generation_p50: Duration::from_nanos(gen.value_at_quantile(0.5)),
            generation_p99: Duration::from_nanos(gen.value_at_quantile(0.99)),
            processing_p50: Duration::from_nanos(proc.value_at_quantile(0.5)),
            processing_p99: Duration::from_nanos(proc.value_at_quantile(0.99)),
            e2e_p50: Duration::from_nanos(e2e.value_at_quantile(0.5)),
            e2e_p99: Duration::from_nanos(e2e.value_at_quantile(0.99)),
        }
    }
}

// ============================================================================
// METRICS REPORT - Summary statistics
// ============================================================================

#[derive(Debug)]
pub struct MetricsReport {
    pub generation_p50: Duration,
    pub generation_p99: Duration,
    pub processing_p50: Duration,
    pub processing_p99: Duration,
    pub e2e_p50: Duration,
    pub e2e_p99: Duration,
}

