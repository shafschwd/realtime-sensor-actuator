use hdrhistogram::Histogram;
use std::time::Duration;
use parking_lot::Mutex;
use std::sync::Arc;

#[derive(Clone)]
pub struct TimingMetrics {
    generation_hist: Arc<Mutex<Histogram<u64>>>,
    processing_hist: Arc<Mutex<Histogram<u64>>>,
    transmission_hist: Arc<Mutex<Histogram<u64>>>,
    e2e_hist: Arc<Mutex<Histogram<u64>>>,
    missed_deadlines: Arc<Mutex<u64>>,
}

impl TimingMetrics {
    pub fn new() -> Self {
        Self {
            generation_hist: Arc::new(Mutex::new(Histogram::new(3).unwrap())),
            processing_hist: Arc::new(Mutex::new(Histogram::new(3).unwrap())),
            transmission_hist: Arc::new(Mutex::new(Histogram::new(3).unwrap())),
            e2e_hist: Arc::new(Mutex::new(Histogram::new(3).unwrap())),
            missed_deadlines: Arc::new(Mutex::new(0)),
        }
    }

    pub fn record_generation(&self, duration: Duration) {
        self.generation_hist.lock().record(duration.as_nanos() as u64).ok();
    }

    pub fn record_processing(&self, duration: Duration, deadline_ns: u64) {
        let nanos = duration.as_nanos() as u64;
        self.processing_hist.lock().record(nanos).ok();

        if nanos > deadline_ns {
            *self.missed_deadlines.lock() += 1;
        }
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
        let trans = self.transmission_hist.lock();
        let e2e = self.e2e_hist.lock();

        MetricsReport {
            generation_p50: Duration::from_nanos(gen.value_at_quantile(0.5)),
            generation_p99: Duration::from_nanos(gen.value_at_quantile(0.99)),
            processing_p50: Duration::from_nanos(proc.value_at_quantile(0.5)),
            processing_p99: Duration::from_nanos(proc.value_at_quantile(0.99)),
            transmission_p50: Duration::from_nanos(trans.value_at_quantile(0.5)),
            e2e_p50: Duration::from_nanos(e2e.value_at_quantile(0.5)),
            e2e_p99: Duration::from_nanos(e2e.value_at_quantile(0.99)),
            missed_deadlines: *self.missed_deadlines.lock(),
        }
    }
}

#[derive(Debug)]
pub struct MetricsReport {
    pub generation_p50: Duration,
    pub generation_p99: Duration,
    pub processing_p50: Duration,
    pub processing_p99: Duration,
    #[allow(dead_code)]
    pub transmission_p50: Duration,
    pub e2e_p50: Duration,
    pub e2e_p99: Duration,
    pub missed_deadlines: u64,
}
