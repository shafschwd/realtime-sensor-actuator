use std::time::Duration;
use tracing::{info, Level};
use tracing_subscriber;
mod sensor;
mod actuator;
mod ipc;
mod benchmark;
mod threaded_impl;
mod async_impl;
mod visualization;
mod config;

use ipc::{channels::SystemChannels, shared_resource::{DiagnosticLog, ConfigBuffer}};
use benchmark::metrics::TimingMetrics;
use threaded_impl::{sensor_thread, actuator_thread};
use config::load_config;

fn main() {
    tracing_subscriber::fmt().with_max_level(Level::INFO).init();
    info!("Starting Real-Time Sensor-Actuator System");

    // Load runtime config from file, mirror into shared buffer
    let file_cfg = load_config("config/system_config.toml");
    let cfg_buf = ConfigBuffer::new();

    // Update the config buffer with loaded values
    cfg_buf.update(|config| {
        config.sensor_interval_ms = file_cfg.sensor_interval_ms;
        config.processing_deadline_ms = file_cfg.processing_deadline_ms;
        config.transmission_deadline_ms = file_cfg.transmission_deadline_ms;
        config.fail_safe_enabled = file_cfg.fail_safe_enabled;
    });

    let channels = SystemChannels::new(256);
    let diagnostic_log = DiagnosticLog::new(2000);
    let metrics = TimingMetrics::new();

    // Spawn threaded implementation
    let sensor_handle = sensor_thread::spawn_sensor_thread(
        channels.clone(),
        diagnostic_log.clone(),
        cfg_buf.clone(),
        metrics.clone(),
    );
    let actuator_handle = actuator_thread::spawn_actuator_thread(
        channels.clone(),
        diagnostic_log.clone(),
        cfg_buf.clone(),
        metrics.clone(),
    );

    info!("System running for 10 seconds...");
    std::thread::sleep(Duration::from_secs(10));

    // Print metrics
    let report = metrics.report();
    info!("=== Performance Report ===");
    info!("Generation P50: {:?}, P99: {:?}", report.generation_p50, report.generation_p99);
    info!("Processing P50: {:?}, P99: {:?}", report.processing_p50, report.processing_p99);
    info!("E2E P50: {:?}, P99: {:?}", report.e2e_p50, report.e2e_p99);
    info!("Missed Deadlines: {}", report.missed_deadlines);

    // Optional: render chart
    visualization::dashboard::render_report_charts(&report);

    // Stop threads by dropping channels (or use a shutdown flag in advanced version)
    drop(sensor_handle);
    drop(actuator_handle);

    // Dump last diagnostics (optional)
    for line in diagnostic_log.read_all().into_iter().rev().take(10).rev() {
        info!("{}", line);
    }
}
