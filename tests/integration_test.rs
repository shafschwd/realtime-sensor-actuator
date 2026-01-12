//! Integration tests for the real-time sensor-actuator system

use realtime_sensor_actuator::{
    SensorGenerator, MovingAverageFilter, AnomalyDetector,
    PIDController, SystemChannels,
};
use std::time::Duration;

// ============================================================================
// SENSOR TESTS
// ============================================================================

#[test]
fn test_sensor_generates_valid_data() {
    let mut gen = SensorGenerator::new(42);
    let reading = gen.generate();

    assert!(reading.force.is_finite(), "Force should be a valid number");
    assert!(reading.position.is_finite(), "Position should be a valid number");
    assert!(reading.temperature.is_finite(), "Temperature should be a valid number");
    assert!(reading.sequence_id == 1, "First reading should have sequence_id = 1");
}

#[test]
fn test_sensor_sequence_increments() {
    let mut gen = SensorGenerator::new(42);

    for expected_id in 1..=10 {
        let reading = gen.generate();
        assert_eq!(reading.sequence_id, expected_id, "Sequence should increment");
    }
}

#[test]
fn test_sensor_produces_varied_force() {
    let mut gen = SensorGenerator::new(42);
    let mut forces: Vec<f32> = Vec::new();

    for _ in 0..100 {
        forces.push(gen.generate().force);
    }

    let min = forces.iter().cloned().fold(f32::INFINITY, f32::min);
    let max = forces.iter().cloned().fold(f32::NEG_INFINITY, f32::max);

    assert!(max - min > 1.0, "Force readings should have variation (noise)");
}

// ============================================================================
// FILTER TESTS
// ============================================================================

#[test]
fn test_moving_average_smooths_data() {
    let mut filter = MovingAverageFilter::new(5);
    let mut gen = SensorGenerator::new(42);

    // Generate readings and filter them
    let mut raw_forces: Vec<f32> = Vec::new();
    let mut filtered_forces: Vec<f32> = Vec::new();

    for _ in 0..20 {
        let reading = gen.generate();
        raw_forces.push(reading.force);
        let filtered = filter.filter(&reading);
        filtered_forces.push(filtered.force);
    }

    // Calculate variance of raw vs filtered
    let raw_variance: f32 = raw_forces.iter()
        .map(|x| (x - 50.0).powi(2))
        .sum::<f32>() / raw_forces.len() as f32;

    let filtered_variance: f32 = filtered_forces.iter()
        .map(|x| (x - 50.0).powi(2))
        .sum::<f32>() / filtered_forces.len() as f32;

    // Filtered data should have less variance (smoother)
    assert!(filtered_variance <= raw_variance, "Filter should reduce variance");
}

// ============================================================================
// PID CONTROLLER TESTS
// ============================================================================

#[test]
fn test_pid_reduces_error_over_time() {
    let mut pid = PIDController::new(0.5, 0.1, 0.05, 50.0);

    let mut value = 30.0; // Start far from setpoint (50.0)

    for _ in 0..100 {
        let control = pid.compute(value);
        value += control * 0.1; // Apply control
    }

    let final_error = (value - 50.0).abs();
    assert!(final_error < 5.0, "PID should bring value close to setpoint");
}

#[test]
fn test_pid_handles_overshoot() {
    let mut pid = PIDController::new(0.8, 0.15, 0.1, 100.0);

    let mut value = 150.0; // Start above setpoint

    for _ in 0..50 {
        let control = pid.compute(value);
        value += control * 0.1;
    }

    let final_error = (value - 100.0).abs();
    assert!(final_error < 10.0, "PID should correct overshoot");
}

// ============================================================================
// ANOMALY DETECTION TESTS
// ============================================================================

#[test]
fn test_anomaly_detector_finds_outliers() {
    let mut gen = SensorGenerator::new(42);
    let detector = AnomalyDetector::new(5.0, 3.0, 2.0);
    let baseline = gen.generate();

    // Create an anomalous reading
    let mut anomalous = gen.generate();
    anomalous.force = baseline.force + 100.0; // Way outside normal range

    let result = detector.detect(&anomalous, &baseline);
    assert!(result.is_some(), "Detector should find the anomaly");
}

#[test]
fn test_anomaly_detector_accepts_normal() {
    let mut gen = SensorGenerator::new(42);
    let detector = AnomalyDetector::new(15.0, 10.0, 5.0); // Wide thresholds
    let baseline = gen.generate();

    // Generate several normal readings
    for _ in 0..10 {
        let reading = gen.generate();
        let result = detector.detect(&reading, &baseline);
        assert!(result.is_none(), "Normal readings should not trigger anomaly");
    }
}

// ============================================================================
// IPC CHANNEL TESTS
// ============================================================================

#[test]
fn test_channels_transmit_data() {
    let channels = SystemChannels::new(10);
    let mut gen = SensorGenerator::new(42);

    // Send data
    let reading = gen.generate();
    channels.sensor_tx.send(reading).expect("Send should succeed");

    // Receive data
    let received = channels.sensor_rx.recv_timeout(Duration::from_millis(100))
        .expect("Receive should succeed");

    assert_eq!(received.sequence_id, reading.sequence_id);
    assert_eq!(received.force, reading.force);
}

#[test]
fn test_channels_handle_multiple_messages() {
    let channels = SystemChannels::new(100);
    let mut gen = SensorGenerator::new(42);

    // Send 50 messages
    for _ in 0..50 {
        let reading = gen.generate();
        channels.sensor_tx.send(reading).expect("Send should succeed");
    }

    // Receive all 50
    for expected_id in 1..=50 {
        let received = channels.sensor_rx.recv_timeout(Duration::from_millis(100))
            .expect("Receive should succeed");
        assert_eq!(received.sequence_id, expected_id);
    }
}

// ============================================================================
// TIMING TESTS
// ============================================================================

#[test]
fn test_sensor_generation_is_fast() {
    let mut gen = SensorGenerator::new(42);

    let start = std::time::Instant::now();
    for _ in 0..1000 {
        let _ = gen.generate();
    }
    let elapsed = start.elapsed();

    // 1000 generations should take less than 10ms
    assert!(elapsed < Duration::from_millis(10),
        "Generation should be fast, took {:?}", elapsed);
}

#[test]
fn test_filter_processing_is_fast() {
    let mut gen = SensorGenerator::new(42);
    let mut filter = MovingAverageFilter::new(5);

    let start = std::time::Instant::now();
    for _ in 0..1000 {
        let reading = gen.generate();
        let _ = filter.filter(&reading);
    }
    let elapsed = start.elapsed();

    // 1000 filter operations should take less than 10ms
    assert!(elapsed < Duration::from_millis(10),
        "Filtering should be fast, took {:?}", elapsed);
}
