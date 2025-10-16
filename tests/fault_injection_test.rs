use realtime_sensor_actuator::sensor::generator::SensorGenerator;

#[test]
fn disturbance_changes_baseline() {
    let mut gen = SensorGenerator::new(1);
    let before = gen.generate().force;
    gen.inject_disturbance(5.0, 0.0);
    let after = gen.generate().force;
    assert!((after - before).abs() > 1.0);
}
