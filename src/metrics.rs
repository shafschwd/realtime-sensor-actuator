//! Metrics module - Performance tracking and statistics

use hdrhistogram::Histogram;
use std::time::Duration;
use parking_lot::Mutex;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

// ============================================================================
// TIMING METRICS - Thread-safe performance tracking
// ============================================================================

#[derive(Clone)]
pub struct TimingMetrics {
    generation_hist: Arc<Mutex<Histogram<u64>>>,
    processing_hist: Arc<Mutex<Histogram<u64>>>,
    transmission_hist: Arc<Mutex<Histogram<u64>>>,
    e2e_hist: Arc<Mutex<Histogram<u64>>>,
    // Jitter tracking (variance in timing)
    last_cycle_time_ns: Arc<AtomicU64>,
    jitter_hist: Arc<Mutex<Histogram<u64>>>,
    // Lock contention tracking
    lock_wait_hist: Arc<Mutex<Histogram<u64>>>,
    lock_contentions: Arc<AtomicU64>,
}

impl TimingMetrics {
    pub fn new() -> Self {
        Self {
            generation_hist: Arc::new(Mutex::new(Histogram::new(3).unwrap())),
            processing_hist: Arc::new(Mutex::new(Histogram::new(3).unwrap())),
            transmission_hist: Arc::new(Mutex::new(Histogram::new(3).unwrap())),
            e2e_hist: Arc::new(Mutex::new(Histogram::new(3).unwrap())),
            last_cycle_time_ns: Arc::new(AtomicU64::new(0)),
            jitter_hist: Arc::new(Mutex::new(Histogram::new(3).unwrap())),
            lock_wait_hist: Arc::new(Mutex::new(Histogram::new(3).unwrap())),
            lock_contentions: Arc::new(AtomicU64::new(0)),
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

    /// Record jitter (variation between consecutive cycle times)
    pub fn record_cycle_jitter(&self, cycle_duration_ns: u64) {
        let last = self.last_cycle_time_ns.swap(cycle_duration_ns, Ordering::Relaxed);
        if last > 0 {
            let jitter = if cycle_duration_ns > last {
                cycle_duration_ns - last
            } else {
                last - cycle_duration_ns
            };
            self.jitter_hist.lock().record(jitter).ok();
        }
    }

    /// Record lock wait time (for contention analysis)
    pub fn record_lock_wait(&self, duration: Duration) {
        self.lock_wait_hist.lock().record(duration.as_nanos() as u64).ok();
        self.lock_contentions.fetch_add(1, Ordering::Relaxed);
    }

    pub fn report(&self) -> MetricsReport {
        let gen = self.generation_hist.lock();
        let proc = self.processing_hist.lock();
        let e2e = self.e2e_hist.lock();
        let jitter = self.jitter_hist.lock();
        let lock_wait = self.lock_wait_hist.lock();

        MetricsReport {
            generation_p50: Duration::from_nanos(gen.value_at_quantile(0.5)),
            generation_p99: Duration::from_nanos(gen.value_at_quantile(0.99)),
            processing_p50: Duration::from_nanos(proc.value_at_quantile(0.5)),
            processing_p99: Duration::from_nanos(proc.value_at_quantile(0.99)),
            e2e_p50: Duration::from_nanos(e2e.value_at_quantile(0.5)),
            e2e_p99: Duration::from_nanos(e2e.value_at_quantile(0.99)),
            jitter_p50: Duration::from_nanos(jitter.value_at_quantile(0.5)),
            jitter_p99: Duration::from_nanos(jitter.value_at_quantile(0.99)),
            lock_wait_p50: Duration::from_nanos(lock_wait.value_at_quantile(0.5)),
            lock_wait_p99: Duration::from_nanos(lock_wait.value_at_quantile(0.99)),
            total_lock_contentions: self.lock_contentions.load(Ordering::Relaxed),
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
    pub jitter_p50: Duration,
    pub jitter_p99: Duration,
    pub lock_wait_p50: Duration,
    pub lock_wait_p99: Duration,
    pub total_lock_contentions: u64,
}
