use std::time::Duration;
use std::sync::atomic::Ordering;

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
    println!("===========================================");
    println!("Starting Real-Time Sensor-Actuator System");
    println!("===========================================\n");

    // Load runtime config from file
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
    let (sensor_handle, sensor_stats) = sensor_thread::spawn_sensor_thread(
        channels.clone(),
        diagnostic_log.clone(),
        cfg_buf.clone(),
        metrics.clone(),
    );
    let (actuator_handle, actuator_stats) = actuator_thread::spawn_actuator_thread(
        channels.clone(),
        diagnostic_log.clone(),
        cfg_buf.clone(),
        metrics.clone(),
    );

    println!("System running for 10 seconds...\n");
    std::thread::sleep(Duration::from_secs(10));

    // Signal shutdown
    println!("\n===========================================");
    println!("Threaded experiment completed - initiating shutdown");
    sensor_stats.shutdown.store(true, Ordering::Relaxed);
    actuator_stats.shutdown.store(true, Ordering::Relaxed);

    // Wait for threads to finish
    let _ = sensor_handle.join();
    let _ = actuator_handle.join();

    // Calculate statistics
    let total_cycles = sensor_stats.total_cycles.load(Ordering::Relaxed);
    let missed_deadlines = sensor_stats.missed_deadlines.load(Ordering::Relaxed);
    let compliance = if total_cycles > 0 {
        ((total_cycles - missed_deadlines) as f64 / total_cycles as f64) * 100.0
    } else {
        100.0
    };

    // Actuator stats
    let gripper_cycles = actuator_stats.gripper_cycles.load(Ordering::Relaxed);
    let gripper_missed = actuator_stats.gripper_missed.load(Ordering::Relaxed);
    let motor_cycles = actuator_stats.motor_cycles.load(Ordering::Relaxed);
    let motor_missed = actuator_stats.motor_missed.load(Ordering::Relaxed);
    let stabilizer_cycles = actuator_stats.stabilizer_cycles.load(Ordering::Relaxed);
    let stabilizer_missed = actuator_stats.stabilizer_missed.load(Ordering::Relaxed);

    let gripper_compliance = if gripper_cycles > 0 {
        ((gripper_cycles - gripper_missed) as f64 / gripper_cycles as f64) * 100.0
    } else { 100.0 };
    let motor_compliance = if motor_cycles > 0 {
        ((motor_cycles - motor_missed) as f64 / motor_cycles as f64) * 100.0
    } else { 100.0 };
    let stabilizer_compliance = if stabilizer_cycles > 0 {
        ((stabilizer_cycles - stabilizer_missed) as f64 / stabilizer_cycles as f64) * 100.0
    } else { 100.0 };

    let recent_met = total_cycles.saturating_sub(missed_deadlines);
    println!("[PERF] Threaded throughput: {:.1} cycles/sec, Recent compliance: {:.1}% ({}/{} cycles met)",
        total_cycles as f64 / 10.0, compliance, recent_met, total_cycles);

    println!("===========================================");
    println!("FINAL THREADED SYSTEM RESULTS");
    println!("===========================================");
    println!("Total Cycles: {}", total_cycles);
    println!("Deadline Compliance: {:.2}% ({} missed)", compliance, missed_deadlines);
    println!("===========================================\n");

    println!("=== Experiment Results ===");
    println!("Total Cycles: {}", total_cycles);
    println!("Deadline Compliance: {:.2}% ({} missed)", compliance, missed_deadlines);
    println!("Actuator Performance:");
    println!("- Gripper: {:.1}% compliance ({} cycles)", gripper_compliance, gripper_cycles);
    println!("- Motor: {:.1}% compliance ({} cycles)", motor_compliance, motor_cycles);
    println!("- Stabilizer: {:.1}% compliance ({} cycles)", stabilizer_compliance, stabilizer_cycles);

    // Print metrics report
    let report = metrics.report();
    println!("\n=== Performance Metrics ===");
    println!("Generation P50: {:?}, P99: {:?}", report.generation_p50, report.generation_p99);
    println!("Processing P50: {:?}, P99: {:?}", report.processing_p50, report.processing_p99);
    println!("E2E P50: {:?}, P99: {:?}", report.e2e_p50, report.e2e_p99);
}
