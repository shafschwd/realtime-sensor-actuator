use realtime_sensor_actuator::SensorGenerator;

#[test]
fn disturbance_changes_baseline() {
    let mut gen = SensorGenerator::new(1);
    let before = gen.generate().force;
    gen.base_force += 5.0;
    let after = gen.generate().force;
    assert!((after - before).abs() > 1.0);
}
