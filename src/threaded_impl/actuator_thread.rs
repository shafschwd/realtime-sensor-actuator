use std::thread;
use std::time::Instant;
use crate::actuator::manager::ActuatorManager;
use crate::ipc::channels::{SystemChannels, ActuatorFeedback};
use crate::ipc::shared_resource::{DiagnosticLog, ConfigBuffer};
use crate::benchmark::metrics::TimingMetrics;

pub fn spawn_actuator_thread(
    channels: SystemChannels,
    diagnostic_log: DiagnosticLog,
    _config: ConfigBuffer,
    metrics: TimingMetrics,
) -> thread::JoinHandle<()> {
    thread::spawn(move || {
        let mut manager = ActuatorManager::new();

        loop {
            // Receive sensor data (blocking)
            let reading = match channels.sensor_rx.recv() {
                Ok(r) => r,
                Err(_) => {
                    diagnostic_log.write("[ACTUATOR] Channel closed".to_string());
                    break;
                }
            };

            let cycle_start = Instant::now();

            // Update actuators with PID control
            manager.update(&reading);
            let commands = manager.get_commands();

            // Send feedback
            let feedback = ActuatorFeedback {
                timestamp: Instant::now(),
                commands: commands.clone(),
                status: "OK".to_string(),
            };

            if channels.feedback_tx.send(feedback).is_err() {
                diagnostic_log.write("[ACTUATOR] Feedback send failed".to_string());
            }

            // Record end-to-end latency
            let e2e_latency = cycle_start.duration_since(reading.timestamp);
            metrics.record_e2e(e2e_latency);

            // Shared resource access
            diagnostic_log.write(format!("[ACTUATOR] Commands: {:?}", commands));
        }
    })
}
