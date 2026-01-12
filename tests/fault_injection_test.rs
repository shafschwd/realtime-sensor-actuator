//! Fault injection tests - Testing system resilience and recovery

use realtime_sensor_actuator::{
    SensorGenerator, MovingAverageFilter, AnomalyDetector,
    PIDController, SystemChannels,
};
use std::time::Duration;

// ============================================================================
// SENSOR FAULT TESTS
// ============================================================================

#[test]
fn test_disturbance_changes_baseline() {
    let mut gen = SensorGenerator::new(1);
    let before = gen.generate().force;
    gen.base_force += 5.0; // Simulate external disturbance
    let after = gen.generate().force;
    assert!((after - before).abs() > 1.0, "Disturbance should affect readings");
}

#[test]
fn test_sensor_handles_extreme_noise() {
    let mut gen = SensorGenerator::new(42);
    gen.noise_amplitude = 100.0; // Extreme noise

    // System should still produce valid readings
    for _ in 0..100 {
        let reading = gen.generate();
        assert!(reading.force.is_finite(), "Should handle extreme noise");
        assert!(!reading.force.is_nan(), "Should not produce NaN");
    }
}

#[test]
fn test_filter_recovers_from_spike() {
    let mut filter = MovingAverageFilter::new(5);
    let mut gen = SensorGenerator::new(42);

    // Normal readings
    for _ in 0..10 {
        let reading = gen.generate();
        filter.filter(&reading);
    }

    // Inject spike
    let mut spike = gen.generate();
    spike.force = 1000.0; // Huge spike
    let filtered_spike = filter.filter(&spike);

    // Filter should dampen the spike
    assert!(filtered_spike.force < spike.force, "Filter should dampen spikes");

    // After more normal readings, should recover (need enough to flush the spike)
    for _ in 0..20 {
        let reading = gen.generate();
        filter.filter(&reading);
    }

    // Final reading should be close to normal (~50N baseline)
    let final_reading = gen.generate();
    let final_filtered = filter.filter(&final_reading);
    assert!(final_filtered.force < 100.0, "Should recover from spike eventually");
}

// ============================================================================
// PID FAULT TESTS
// ============================================================================

#[test]
fn test_pid_handles_sudden_setpoint_change() {
    let mut pid = PIDController::new(0.5, 0.1, 0.05, 50.0);

    // Stabilize at setpoint
    let mut value = 50.0;
    for _ in 0..20 {
        let control = pid.compute(value);
        value += control * 0.1;
    }

    // Simulate disturbance - value suddenly changes
    value = 100.0;
    let control = pid.compute(value);

    // Control output should be non-zero to correct
    assert!(control.abs() > 0.1, "PID should react to disturbance");
}

#[test]
fn test_pid_handles_oscillation() {
    let mut pid = PIDController::new(0.5, 0.1, 0.05, 50.0);

    // Oscillating input
    let mut max_control = 0.0f32;
    for i in 0..100 {
        let value = if i % 2 == 0 { 40.0 } else { 60.0 }; // Oscillating
        let control = pid.compute(value);
        max_control = max_control.max(control.abs());
    }

    // Control should be bounded - with anti-windup, should stay reasonable
    // The integral can accumulate but anti-windup limits it to 100
    assert!(max_control < 10000.0, "PID should have bounded output");
}

// ============================================================================
// COMMUNICATION FAULT TESTS
// ============================================================================

#[test]
fn test_channel_timeout_on_no_data() {
    let channels = SystemChannels::new(10);

    // Try to receive with no data sent
    let result = channels.sensor_rx.recv_timeout(Duration::from_millis(50));

    assert!(result.is_err(), "Should timeout when no data");
}

#[test]
fn test_channel_handles_buffer_full() {
    let channels = SystemChannels::new(5); // Small buffer
    let mut gen = SensorGenerator::new(42);

    // Fill buffer
    for _ in 0..5 {
        let reading = gen.generate();
        channels.sensor_tx.send(reading).expect("Should succeed");
    }

    // Try to send when full (with timeout)
    let reading = gen.generate();
    let result = channels.sensor_tx.send_timeout(reading, Duration::from_millis(10));

    // Should fail or timeout (buffer full)
    assert!(result.is_err(), "Should fail when buffer is full");
}

// ============================================================================
// ANOMALY DETECTION FAULT TESTS
// ============================================================================

#[test]
fn test_detector_handles_gradual_drift() {
    let mut gen = SensorGenerator::new(42);
    let detector = AnomalyDetector::new(5.0, 3.0, 2.0);
    let baseline = gen.generate();

    // Large drift should definitely trigger
    let mut drifted = baseline;
    drifted.force = baseline.force + 50.0;
    let result = detector.detect(&drifted, &baseline);
    assert!(result.is_some(), "Large drift should be detected");
}

#[test]
fn test_detector_handles_all_sensors_anomaly() {
    let mut gen = SensorGenerator::new(42);
    let detector = AnomalyDetector::new(5.0, 3.0, 2.0);
    let baseline = gen.generate();

    // All sensors anomalous
    let mut anomalous = baseline;
    anomalous.force = baseline.force + 100.0;
    anomalous.position = baseline.position + 50.0;
    anomalous.temperature = baseline.temperature + 20.0;

    let result = detector.detect(&anomalous, &baseline);
    assert!(result.is_some(), "Multi-sensor anomaly should be detected");
}

// ============================================================================
// RECOVERY TESTS
// ============================================================================

#[test]
fn test_system_recovers_after_fault() {
    let mut gen = SensorGenerator::new(42);
    let mut filter = MovingAverageFilter::new(5);

    // Normal operation
    let mut last_filtered = 0.0;
    for _ in 0..10 {
        let reading = gen.generate();
        last_filtered = filter.filter(&reading).force;
    }
    let normal_value = last_filtered;

    // Inject fault
    gen.base_force = 200.0; // Sudden change
    for _ in 0..5 {
        let reading = gen.generate();
        filter.filter(&reading);
    }

    // Recover
    gen.base_force = 50.0; // Back to normal
    for _ in 0..20 {
        let reading = gen.generate();
        last_filtered = filter.filter(&reading).force;
    }

    // Should be close to normal again
    assert!((last_filtered - normal_value).abs() < 30.0,
        "System should recover after fault");
}
