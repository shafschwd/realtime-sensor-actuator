use crate::actuator::manager::ActuatorManager;
use crate::ipc::channels::{SystemChannels, ActuatorFeedback};
use crate::benchmark::metrics::TimingMetrics;
use std::time::Instant;

#[allow(dead_code)]
pub async fn actuator_task(channels: SystemChannels, metrics: TimingMetrics) {
    let mut manager = ActuatorManager::new();
    loop {
        let reading = match channels.sensor_rx.recv() {
            Ok(r) => r,
            Err(_) => break,
        };
        let cycle_start = Instant::now();

        manager.update(&reading);
        let commands = manager.get_commands();

        let feedback = ActuatorFeedback { timestamp: Instant::now(), commands: commands.clone(), status: "OK".to_string() };
        let _ = channels.feedback_tx.send(feedback);

        let e2e_latency = cycle_start.duration_since(reading.timestamp);
        metrics.record_e2e(e2e_latency);
    }
}
