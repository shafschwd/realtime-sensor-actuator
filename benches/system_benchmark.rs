use criterion::{criterion_group, criterion_main, Criterion};
use realtime_sensor_actuator::sensor::generator::SensorGenerator;
use realtime_sensor_actuator::actuator::controller::PIDController;

fn benchmark_sensor_generation(c: &mut Criterion) {
    let mut gen = SensorGenerator::new(42);
    c.bench_function("sensor_generate", |b| b.iter(|| gen.generate()));
}

fn benchmark_pid_control(c: &mut Criterion) {
    let mut pid = PIDController::new(0.5, 0.1, 0.05, 50.0);
    c.bench_function("pid_compute", |b| b.iter(|| pid.compute(48.0)));
}

criterion_group!(benches, benchmark_sensor_generation, benchmark_pid_control);
criterion_main!(benches);
