use std::thread;
use std::time::{Duration, Instant};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use crate::actuator::controller::PIDController;
use crate::ipc::channels::{SystemChannels, ActuatorFeedback, ActuatorStatus};
use crate::ipc::shared_resource::{DiagnosticLog, ConfigBuffer};
use crate::benchmark::metrics::TimingMetrics;

pub struct ActuatorStats {
    pub gripper_cycles: AtomicU64,
    pub gripper_missed: AtomicU64,
    pub motor_cycles: AtomicU64,
    pub motor_missed: AtomicU64,
    pub stabilizer_cycles: AtomicU64,
    pub stabilizer_missed: AtomicU64,
    pub shutdown: AtomicBool,
}

impl ActuatorStats {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
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

struct ActuatorConfig {
    name: &'static str,
    pid: PIDController,
    deadline_ms: f64,
    feedback_deadline_us: f64,
}

pub fn spawn_actuator_thread(
    channels: SystemChannels,
    diagnostic_log: DiagnosticLog,
    _config: ConfigBuffer,
    metrics: TimingMetrics,
) -> (thread::JoinHandle<()>, Arc<ActuatorStats>) {
    let stats = ActuatorStats::new();
    let stats_clone = stats.clone();

    let handle = thread::spawn(move || {
        // Create individual actuators with their own PIDs and deadlines
        let mut actuators = vec![
            ActuatorConfig {
                name: "Gripper",
                pid: PIDController::new(0.5, 0.1, 0.05, 50.0),
                deadline_ms: 1.0,
                feedback_deadline_us: 500.0,
            },
            ActuatorConfig {
                name: "Motor",
                pid: PIDController::new(0.8, 0.15, 0.1, 100.0),
                deadline_ms: 2.0,
                feedback_deadline_us: 500.0,
            },
            ActuatorConfig {
                name: "Stabilizer",
                pid: PIDController::new(0.3, 0.05, 0.02, 25.0),
                deadline_ms: 1.5,
                feedback_deadline_us: 500.0,
            },
        ];

        let system_start = Instant::now();
        let mut dispatch_counter = 0u64;

        loop {
            if stats_clone.shutdown.load(Ordering::Relaxed) {
                println!("[SYSTEM] Threaded dispatcher shutting down");
                break;
            }

            // Receive sensor data (with timeout to allow shutdown check)
            let reading = match channels.sensor_rx.recv_timeout(Duration::from_millis(100)) {
                Ok(r) => r,
                Err(crossbeam::channel::RecvTimeoutError::Timeout) => continue,
                Err(_) => {
                    diagnostic_log.write("[ACTUATOR] Channel closed".to_string());
                    break;
                }
            };

            let cycle_id = reading.sequence_id;
            let elapsed_secs = system_start.elapsed().as_secs_f64();
            dispatch_counter += 1;

            // Log dispatcher routing every 5 dispatches
            if dispatch_counter % 5 == 0 {
                println!("[{:7.3}s] DISPATCHER: Routed cycle #{:<4} to actuators (G:true, M:true, S:true)",
                    0.0, cycle_id);
            }

            // Process each actuator
            let inputs = [reading.force, reading.position, reading.temperature];

            for (i, actuator) in actuators.iter_mut().enumerate() {
                let proc_start = Instant::now();

                // Compute PID control
                let control = actuator.pid.compute(inputs[i]);
                let error = actuator.pid.get_error();

                let proc_duration = proc_start.elapsed();
                let proc_ms = proc_duration.as_secs_f64() * 1000.0;

                // Update stats
                match i {
                    0 => {
                        stats_clone.gripper_cycles.fetch_add(1, Ordering::Relaxed);
                        if proc_ms > actuator.deadline_ms {
                            stats_clone.gripper_missed.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                    1 => {
                        stats_clone.motor_cycles.fetch_add(1, Ordering::Relaxed);
                        if proc_ms > actuator.deadline_ms {
                            stats_clone.motor_missed.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                    2 => {
                        stats_clone.stabilizer_cycles.fetch_add(1, Ordering::Relaxed);
                        if proc_ms > actuator.deadline_ms {
                            stats_clone.stabilizer_missed.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                    _ => {}
                }

                let status = if proc_ms > actuator.deadline_ms {
                    ActuatorStatus::Warning
                } else {
                    ActuatorStatus::Normal
                };

                // Log actuator processing (every 10th cycle)
                if cycle_id % 10 == 0 {
                    println!("[{:7.3}s] {}: Processed cycle #{:<4} - Error: {:.2}, Control: {:.2} ({})",
                        elapsed_secs, actuator.name, cycle_id, error, control, status);

                    if proc_ms > actuator.deadline_ms {
                        println!("[{:7.3}s] {}: Processed ✗ (latency: {:.2}ms, deadline: {:.1}ms)",
                            elapsed_secs, actuator.name, proc_ms, actuator.deadline_ms);
                        println!("[{:7.3}s] [DEADLINE] {}: Processing missed - {:.2}ms > {:.1}ms (cycle #{}) ✗",
                            elapsed_secs, actuator.name, proc_ms, actuator.deadline_ms, cycle_id);
                    } else {
                        println!("[{:7.3}s] {}: Processed ✓ (latency: {:.2}ms, deadline: {:.1}ms)",
                            elapsed_secs, actuator.name, proc_ms, actuator.deadline_ms);
                    }
                }

                // Send feedback
                let fb_start = Instant::now();
                let feedback = ActuatorFeedback {
                    timestamp: Instant::now(),
                    actuator_name: actuator.name.to_string(),
                    error,
                    control,
                    status: status.clone(),
                    cycle_id,
                };

                if channels.feedback_tx.send(feedback).is_ok() {
                    let fb_duration = fb_start.elapsed();
                    let fb_us = fb_duration.as_nanos() as f64 / 1000.0;

                    if cycle_id % 10 == 0 {
                        let timestamp_ns = system_start.elapsed().as_nanos();
                        println!("[{:012}] {}: Feedback sent ✓ (latency: {:.2}μs, deadline: {}μs)",
                            timestamp_ns, actuator.name, fb_us, actuator.feedback_deadline_us as u64);
                    }
                }
            }

            // Record end-to-end latency
            metrics.record_e2e(reading.timestamp.elapsed());
        }
    });

    (handle, stats)
}
