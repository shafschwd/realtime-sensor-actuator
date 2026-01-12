//! Real-Time Sensor-Actuator System
//! A multi-threaded system demonstrating real-time control with deadline tracking

mod sensor;
mod actuator;
mod ipc;
mod metrics;

use std::thread;
use std::time::{Duration, Instant};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;

use sensor::{SensorGenerator, MovingAverageFilter, AnomalyDetector};
use actuator::{ActuatorConfig, ActuatorStatus};
use ipc::{SystemChannels, DiagnosticLog, ConfigBuffer, ActuatorFeedback, load_config};
use metrics::TimingMetrics;

// ============================================================================
// STATISTICS TRACKING
// ============================================================================

pub struct SystemStats {
    pub total_cycles: AtomicU64,
    pub missed_deadlines: AtomicU64,
    pub anomalies: AtomicU64,
    pub gripper_cycles: AtomicU64,
    pub gripper_missed: AtomicU64,
    pub motor_cycles: AtomicU64,
    pub motor_missed: AtomicU64,
    pub stabilizer_cycles: AtomicU64,
    pub stabilizer_missed: AtomicU64,
    pub shutdown: AtomicBool,
}

impl SystemStats {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            total_cycles: AtomicU64::new(0),
            missed_deadlines: AtomicU64::new(0),
            anomalies: AtomicU64::new(0),
            gripper_cycles: AtomicU64::new(0),
            gripper_missed: AtomicU64::new(0),
            motor_cycles: AtomicU64::new(0),
            motor_missed: AtomicU64::new(0),
            stabilizer_cycles: AtomicU64::new(0),
            stabilizer_missed: AtomicU64::new(0),
            shutdown: AtomicBool::new(false),
        })
    }
}

// ============================================================================
// SENSOR THREAD
// ============================================================================

fn spawn_sensor_thread(
    channels: SystemChannels,
    _diagnostic_log: DiagnosticLog,
    config: ConfigBuffer,
    metrics: TimingMetrics,
    stats: Arc<SystemStats>,
) -> thread::JoinHandle<()> {
    thread::spawn(move || {
        let mut generator = SensorGenerator::new(42);
        let mut filter = MovingAverageFilter::new(5);
        let detector = AnomalyDetector::new(10.0, 5.0, 3.0);
        let baseline = generator.generate();

        let system_start = Instant::now();
        let processing_deadline_us = 200.0;
        let transmission_deadline_us = 100.0;

        loop {
            if stats.shutdown.load(Ordering::Relaxed) {
                break;
            }

            let cfg = config.get();
            let cycle_start = Instant::now();
            let elapsed_secs = system_start.elapsed().as_secs_f64();

            // Generate sensor data
            let gen_start = Instant::now();
            let reading = generator.generate();
            let cycle_id = reading.sequence_id;
            metrics.record_generation(gen_start.elapsed());

            // Log every 10th cycle
            if cycle_id % 10 == 0 {
                println!("[{:7.3}s] SENSOR: Generated cycle #{:<4} - Force: {:.2}, Position: {:.2}, Temp: {:.1}",
                    elapsed_secs, cycle_id, reading.force, reading.position, reading.temperature);
            }

            // Process data (filter + anomaly detection)
            let proc_start = Instant::now();
            let filtered = filter.filter(&reading);
            let is_anomaly = detector.detect(&filtered, &baseline).is_some();
            let proc_duration = proc_start.elapsed();
            let proc_us = proc_duration.as_micros() as f64;

            metrics.record_processing(proc_duration);

            // Check processing deadline
            let proc_missed = proc_us > processing_deadline_us;
            if proc_missed {
                stats.missed_deadlines.fetch_add(1, Ordering::Relaxed);
                println!("[DEADLINE] Sensor processing missed: {:.2}μs > {}μs (cycle #{})",
                    proc_us, processing_deadline_us as u64, cycle_id);
            } else if cycle_id % 10 == 0 {
                println!("[{:7.3}s] SENSOR: Filtered data - Anomaly: {}, Processing: {:.2}μs ✓ (deadline: {}μs)",
                    elapsed_secs, is_anomaly, proc_us, processing_deadline_us as u64);
            }

            if is_anomaly {
                stats.anomalies.fetch_add(1, Ordering::Relaxed);
            }

            // Transmit to actuator
            let tx_start = Instant::now();
            if channels.sensor_tx.send(filtered).is_err() {
                break;
            }
            let tx_us = tx_start.elapsed().as_micros() as f64;
            metrics.record_transmission(tx_start.elapsed());

            // Check transmission deadline
            if tx_us > transmission_deadline_us {
                println!("[DEADLINE] Sensor transmission missed: {:.2}μs > {}μs (cycle #{})",
                    tx_us, transmission_deadline_us as u64, cycle_id);
            } else if cycle_id % 10 == 0 && !proc_missed {
                let timestamp_ns = system_start.elapsed().as_nanos();
                println!("[{:012}] SENSOR: Transmitted to dispatcher ✓ (latency: {:.2}μs, deadline: {}μs)",
                    timestamp_ns, tx_us, transmission_deadline_us as u64);
            }

            // Check for feedback
            while let Ok(feedback) = channels.feedback_rx.try_recv() {
                if cycle_id % 10 == 0 {
                    let timestamp_ns = system_start.elapsed().as_nanos();
                    println!("[{:012}] FEEDBACK: Received from {} - Error: {:.2}, Control: {:.2}, Status: {}",
                        timestamp_ns, feedback.actuator_name, feedback.error, feedback.control, feedback.status);
                }
            }

            stats.total_cycles.fetch_add(1, Ordering::Relaxed);

            // Performance log every 100 cycles
            if cycle_id % 100 == 0 {
                let timestamp_ns = system_start.elapsed().as_nanos();
                let total = stats.total_cycles.load(Ordering::Relaxed);
                let anomalies = stats.anomalies.load(Ordering::Relaxed);
                println!("[{:012}] [PERF] System running - {:.1} cycles/sec, Anomalies: {}",
                    timestamp_ns, total as f64 / elapsed_secs, anomalies);
            }

            // Maintain sampling rate
            let elapsed = cycle_start.elapsed();
            let interval = Duration::from_millis(cfg.sensor_interval_ms);
            if elapsed < interval {
                thread::sleep(interval - elapsed);
            }
        }
    })
}

// ============================================================================
// ACTUATOR THREAD
// ============================================================================

fn spawn_actuator_thread(
    channels: SystemChannels,
    _diagnostic_log: DiagnosticLog,
    metrics: TimingMetrics,
    stats: Arc<SystemStats>,
) -> thread::JoinHandle<()> {
    thread::spawn(move || {
        let mut actuators = ActuatorConfig::create_defaults();
        let system_start = Instant::now();
        let mut dispatch_counter = 0u64;

        loop {
            if stats.shutdown.load(Ordering::Relaxed) {
                println!("[SYSTEM] Dispatcher shutting down");
                break;
            }

            // Receive sensor data with timeout
            let reading = match channels.sensor_rx.recv_timeout(Duration::from_millis(100)) {
                Ok(r) => r,
                Err(crossbeam::channel::RecvTimeoutError::Timeout) => continue,
                Err(_) => break,
            };

            let cycle_id = reading.sequence_id;
            let elapsed_secs = system_start.elapsed().as_secs_f64();
            dispatch_counter += 1;

            // Log dispatcher routing
            if dispatch_counter % 5 == 0 {
                println!("[{:7.3}s] DISPATCHER: Routed cycle #{:<4} to actuators (G:true, M:true, S:true)",
                    elapsed_secs, cycle_id);
            }

            // Process each actuator
            let inputs = [reading.force, reading.position, reading.temperature];

            for (i, actuator) in actuators.iter_mut().enumerate() {
                let proc_start = Instant::now();
                let control = actuator.pid.compute(inputs[i]);
                let error = actuator.pid.get_error();
                let proc_ms = proc_start.elapsed().as_secs_f64() * 1000.0;

                // Update stats
                match i {
                    0 => {
                        stats.gripper_cycles.fetch_add(1, Ordering::Relaxed);
                        if proc_ms > actuator.deadline_ms {
                            stats.gripper_missed.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                    1 => {
                        stats.motor_cycles.fetch_add(1, Ordering::Relaxed);
                        if proc_ms > actuator.deadline_ms {
                            stats.motor_missed.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                    2 => {
                        stats.stabilizer_cycles.fetch_add(1, Ordering::Relaxed);
                        if proc_ms > actuator.deadline_ms {
                            stats.stabilizer_missed.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                    _ => {}
                }

                let status = if proc_ms > actuator.deadline_ms {
                    ActuatorStatus::Warning
                } else {
                    ActuatorStatus::Normal
                };

                // Log every 10th cycle
                if cycle_id % 10 == 0 {
                    println!("[{:7.3}s] {}: Processed cycle #{:<4} - Error: {:.2}, Control: {:.2} ({})",
                        elapsed_secs, actuator.name, cycle_id, error, control, status);

                    if proc_ms > actuator.deadline_ms {
                        println!("[{:7.3}s] [DEADLINE] {}: Processing missed - {:.2}ms > {:.1}ms ✗",
                            elapsed_secs, actuator.name, proc_ms, actuator.deadline_ms);
                    } else {
                        println!("[{:7.3}s] {}: Processed ✓ (latency: {:.2}ms, deadline: {:.1}ms)",
                            elapsed_secs, actuator.name, proc_ms, actuator.deadline_ms);
                    }
                }

                // Send feedback
                let feedback = ActuatorFeedback {
                    timestamp: Instant::now(),
                    actuator_name: actuator.name.to_string(),
                    error,
                    control,
                    status,
                    cycle_id,
                };
                let _ = channels.feedback_tx.send(feedback);
            }

            metrics.record_e2e(reading.timestamp.elapsed());
        }
    })
}

// ============================================================================
// MAIN ENTRY POINT
// ============================================================================

fn main() {
    println!("===========================================");
    println!("Real-Time Sensor-Actuator System");
    println!("===========================================\n");

    // Load configuration
    let file_cfg = load_config("config/system_config.toml");
    let cfg_buf = ConfigBuffer::new();
    cfg_buf.update(|config| {
        config.sensor_interval_ms = file_cfg.sensor_interval_ms;
        config.processing_deadline_ms = file_cfg.processing_deadline_ms;
        config.transmission_deadline_ms = file_cfg.transmission_deadline_ms;
        config.fail_safe_enabled = file_cfg.fail_safe_enabled;
    });

    // Initialize system components
    let channels = SystemChannels::new(256);
    let diagnostic_log = DiagnosticLog::new(2000);
    let metrics = TimingMetrics::new();
    let stats = SystemStats::new();

    // Spawn threads
    let sensor_handle = spawn_sensor_thread(
        channels.clone(),
        diagnostic_log.clone(),
        cfg_buf.clone(),
        metrics.clone(),
        stats.clone(),
    );
    let actuator_handle = spawn_actuator_thread(
        channels.clone(),
        diagnostic_log.clone(),
        metrics.clone(),
        stats.clone(),
    );

    println!("System running for 10 seconds...\n");
    thread::sleep(Duration::from_secs(10));

    // Shutdown
    println!("\n===========================================");
    println!("Experiment completed - initiating shutdown");
    stats.shutdown.store(true, Ordering::Relaxed);

    let _ = sensor_handle.join();
    let _ = actuator_handle.join();

    // Calculate and display statistics
    let total_cycles = stats.total_cycles.load(Ordering::Relaxed);
    let missed_deadlines = stats.missed_deadlines.load(Ordering::Relaxed);
    let compliance = if total_cycles > 0 {
        ((total_cycles - missed_deadlines) as f64 / total_cycles as f64) * 100.0
    } else { 100.0 };

    let gripper_cycles = stats.gripper_cycles.load(Ordering::Relaxed);
    let gripper_missed = stats.gripper_missed.load(Ordering::Relaxed);
    let motor_cycles = stats.motor_cycles.load(Ordering::Relaxed);
    let motor_missed = stats.motor_missed.load(Ordering::Relaxed);
    let stabilizer_cycles = stats.stabilizer_cycles.load(Ordering::Relaxed);
    let stabilizer_missed = stats.stabilizer_missed.load(Ordering::Relaxed);

    let gripper_compliance = if gripper_cycles > 0 {
        ((gripper_cycles - gripper_missed) as f64 / gripper_cycles as f64) * 100.0
    } else { 100.0 };
    let motor_compliance = if motor_cycles > 0 {
        ((motor_cycles - motor_missed) as f64 / motor_cycles as f64) * 100.0
    } else { 100.0 };
    let stabilizer_compliance = if stabilizer_cycles > 0 {
        ((stabilizer_cycles - stabilizer_missed) as f64 / stabilizer_cycles as f64) * 100.0
    } else { 100.0 };

    println!("\n===========================================");
    println!("FINAL SYSTEM RESULTS");
    println!("===========================================");
    println!("Total Cycles: {}", total_cycles);
    println!("Deadline Compliance: {:.2}% ({} missed)", compliance, missed_deadlines);
    println!("\nActuator Performance:");
    println!("  - Gripper:    {:.1}% compliance ({} cycles)", gripper_compliance, gripper_cycles);
    println!("  - Motor:      {:.1}% compliance ({} cycles)", motor_compliance, motor_cycles);
    println!("  - Stabilizer: {:.1}% compliance ({} cycles)", stabilizer_compliance, stabilizer_cycles);

    let report = metrics.report();
    println!("\nLatency Metrics:");
    println!("  Generation  - P50: {:?}, P99: {:?}", report.generation_p50, report.generation_p99);
    println!("  Processing  - P50: {:?}, P99: {:?}", report.processing_p50, report.processing_p99);
    println!("  End-to-End  - P50: {:?}, P99: {:?}", report.e2e_p50, report.e2e_p99);
    println!("===========================================");
}

