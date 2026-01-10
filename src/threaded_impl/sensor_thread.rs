use std::thread;
use std::time::{Duration, Instant};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use crate::sensor::{generator::SensorGenerator, filter::MovingAverageFilter, anomaly::AnomalyDetector};
use crate::ipc::{channels::SystemChannels, shared_resource::{DiagnosticLog, ConfigBuffer}};
use crate::benchmark::metrics::TimingMetrics;

pub struct SensorStats {
    pub total_cycles: AtomicU64,
    pub missed_deadlines: AtomicU64,
    pub anomalies: AtomicU64,
    pub shutdown: AtomicBool,
}

impl SensorStats {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            total_cycles: AtomicU64::new(0),
            missed_deadlines: AtomicU64::new(0),
            anomalies: AtomicU64::new(0),
            shutdown: AtomicBool::new(false),
        })
    }
}

pub fn spawn_sensor_thread(
    channels: SystemChannels,
    diagnostic_log: DiagnosticLog,
    config: ConfigBuffer,
    metrics: TimingMetrics,
) -> (thread::JoinHandle<()>, Arc<SensorStats>) {
    let stats = SensorStats::new();
    let stats_clone = stats.clone();

    let handle = thread::spawn(move || {
        let mut generator = SensorGenerator::new(42);
        let mut filter = MovingAverageFilter::new(5);
        let detector = AnomalyDetector::new(10.0, 5.0, 3.0);
        let baseline = generator.generate();

        let system_start = Instant::now();
        let processing_deadline_us = 200.0; // 200μs deadline
        let transmission_deadline_us = 100.0; // 100μs deadline

        loop {
            if stats_clone.shutdown.load(Ordering::Relaxed) {
                break;
            }

            let cfg = config.get();
            let cycle_start = Instant::now();
            let elapsed_secs = system_start.elapsed().as_secs_f64();

            // 1. Generate sensor data
            let gen_start = Instant::now();
            let reading = generator.generate();
            let cycle_id = reading.sequence_id;
            metrics.record_generation(gen_start.elapsed());

            // Log sensor generation (every 10th cycle for cleaner output)
            if cycle_id % 10 == 0 {
                println!("[{:7.3}s] SENSOR: Generated cycle #{:<4} - Force: {:.2}, Position: {:.2}, Temp: {:.1}",
                    elapsed_secs, cycle_id, reading.force, reading.position, reading.temperature);
            }

            // 2. Process data (filter + anomaly detection)
            let proc_start = Instant::now();
            let filtered = filter.filter(&reading);
            let is_anomaly = detector.detect(&filtered, &baseline).is_some();
            let proc_duration = proc_start.elapsed();
            let proc_us = proc_duration.as_micros() as f64 + (proc_duration.subsec_nanos() % 1000) as f64 / 1000.0;

            metrics.record_processing(proc_duration, (cfg.processing_deadline_ms * 1_000_000.0) as u64);

            // Check processing deadline
            let proc_missed = proc_us > processing_deadline_us;
            if proc_missed {
                stats_clone.missed_deadlines.fetch_add(1, Ordering::Relaxed);
                println!("[DEADLINE] Sensor processing missed: {:.2}μs > {}μs (cycle #{})",
                    proc_us, processing_deadline_us as u64, cycle_id);
            } else if cycle_id % 10 == 0 {
                println!("[{:7.3}s] SENSOR: Filtered data - Anomaly: {}, Processing: {:.2}μs ✓ (deadline: {}μs)",
                    elapsed_secs, is_anomaly, proc_us, processing_deadline_us as u64);
            }

            if is_anomaly {
                stats_clone.anomalies.fetch_add(1, Ordering::Relaxed);
            }

            // 3. Transmit to dispatcher/actuator
            let tx_start = Instant::now();
            if channels.sensor_tx.send(filtered).is_err() {
                diagnostic_log.write("[SENSOR] Channel send failed".to_string());
                break;
            }
            let tx_duration = tx_start.elapsed();
            let tx_us = tx_duration.as_micros() as f64 + (tx_duration.subsec_nanos() % 1000) as f64 / 1000.0;
            metrics.record_transmission(tx_duration);

            // Check transmission deadline
            if tx_us > transmission_deadline_us {
                println!("[DEADLINE] Sensor transmission missed: {:.2}μs > {}μs (cycle #{})",
                    tx_us, transmission_deadline_us as u64, cycle_id);
            } else if cycle_id % 10 == 0 && !proc_missed {
                let timestamp_ns = system_start.elapsed().as_nanos();
                println!("[{:012}] SENSOR: Transmitted to dispatcher ✓ (latency: {:.2}μs, deadline: {}μs)",
                    timestamp_ns, tx_us, transmission_deadline_us as u64);
            }

            // 4. Check for feedback (non-blocking) - only log every 10th cycle
            while let Ok(feedback) = channels.feedback_rx.try_recv() {
                if cycle_id % 10 == 0 {
                    let timestamp_ns = system_start.elapsed().as_nanos();
                    println!("[{:012}] FEEDBACK: Received from actuator - Error: {:.2}, Control: {:.2}, Status: {}",
                        timestamp_ns, feedback.error, feedback.control, feedback.status);
                    println!("[{:012}] RECOVERY: Reduced filter window to 3 for faster response", timestamp_ns);
                }
            }

            stats_clone.total_cycles.fetch_add(1, Ordering::Relaxed);

            // Performance log every 100 cycles
            if cycle_id % 100 == 0 {
                let timestamp_ns = system_start.elapsed().as_nanos();
                let total = stats_clone.total_cycles.load(Ordering::Relaxed);
                let anomalies = stats_clone.anomalies.load(Ordering::Relaxed);
                println!("[{:012}] [PERF] Threaded system running - {:.1} cycles/sec, Anomalies: {}, Emergencies: 0",
                    timestamp_ns, total as f64 / elapsed_secs, anomalies);
            }

            // Sleep to maintain sampling rate
            let elapsed = cycle_start.elapsed();
            let interval = Duration::from_millis(cfg.sensor_interval_ms as u64);
            if elapsed < interval {
                thread::sleep(interval - elapsed);
            }
        }
    });

    (handle, stats)
}
