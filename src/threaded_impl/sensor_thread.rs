use std::thread;
use std::time::{Duration, Instant};
use crate::sensor::{generator::SensorGenerator, filter::MovingAverageFilter, anomaly::AnomalyDetector};
use crate::ipc::{channels::SystemChannels, shared_resource::{DiagnosticLog, ConfigBuffer}};
use crate::benchmark::metrics::TimingMetrics;

pub fn spawn_sensor_thread(
    channels: SystemChannels,
    diagnostic_log: DiagnosticLog,
    config: ConfigBuffer,
    metrics: TimingMetrics,
) -> thread::JoinHandle<()> {
    thread::spawn(move || {
        let mut generator = SensorGenerator::new(42);
        let mut filter = MovingAverageFilter::new(5);
        let detector = AnomalyDetector::new(10.0, 5.0, 3.0);

        let baseline = generator.generate();
        let mut iteration = 0u64;

        loop {
            let cfg = config.get();
            let cycle_start = Instant::now();

            // 1. Generate sensor data
            let gen_start = Instant::now();
            let reading = generator.generate();
            metrics.record_generation(gen_start.elapsed());

            // 2. Process data (filter + anomaly detection)
            let proc_start = Instant::now();
            let filtered = filter.filter(&reading);

            if let Some(anomaly) = detector.detect(&filtered, &baseline) {
                diagnostic_log.write(format!("[SENSOR] {}", anomaly));
            }

            let proc_duration = proc_start.elapsed();
            metrics.record_processing(proc_duration, (cfg.processing_deadline_ms * 1_000_000.0) as u64);

            // 3. Transmit to actuator
            let tx_start = Instant::now();
            if channels.sensor_tx.send(filtered).is_err() {
                diagnostic_log.write("[SENSOR] Channel send failed".to_string());
                break;
            }
            metrics.record_transmission(tx_start.elapsed());

            // 4. Check for feedback (non-blocking)
            if let Ok(feedback) = channels.feedback_rx.try_recv() {
                diagnostic_log.write(format!("[SENSOR] Feedback: {:?}", feedback.status));
            }

            // Log to shared resource (demonstrates lock contention)
            if iteration % 100 == 0 {
                diagnostic_log.write(format!("[SENSOR] Iteration {}: cycle time {:?}",
                                             iteration, cycle_start.elapsed()));
            }

            iteration += 1;

            // Sleep to maintain sampling rate
            let elapsed = cycle_start.elapsed();
            let interval = Duration::from_millis(cfg.sensor_interval_ms);
            if elapsed < interval {
                thread::sleep(interval - elapsed);
            }
        }
    })
}
