use realtime_sensor_actuator::SensorGenerator;

#[test]
fn generate_sensor_data() {
    let mut gen = SensorGenerator::new(1);
    let r = gen.generate();
    assert!(r.force.is_finite());
}
