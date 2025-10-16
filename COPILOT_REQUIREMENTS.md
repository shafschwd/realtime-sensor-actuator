# Copilot Requirements Checklist

This document maps the CT087-3-3 Real-Time Systems assignment requirements to concrete code artifacts in this repository so tooling can verify compliance.

## Core
- [ ] System architecture implemented (README.md, src/main.rs, src/sensor/*, src/actuator/*, src/ipc/*)
- [ ] Modular Rust structure and crates configured (Cargo.toml, src/*)
- [ ] Concurrency and synchronization used safely (crossbeam channels, Arc<Mutex/RwLock>)
- [ ] Logging and benchmarking integrated (tracing, hdrhistogram, criterion)
- [ ] PID control implemented in actuator (src/actuator/controller.rs, manager.rs)
- [ ] Sensor simulator with filtering and anomaly detection (src/sensor/*)
- [ ] Actuator commander with deadlines and feedback (src/threaded_impl/actuator_thread.rs)
- [ ] Timing metrics with latency/jitter/throughput (src/benchmark/metrics.rs, main)
- [ ] Tests and benches present (tests/*, benches/*)

## Optional Advanced
- [ ] Async implementation (src/async_impl/*) and rationale in README
- [ ] Fault injection and fail-safe (recv_timeout path and sensor delays)
- [ ] CPU load simulation (rayon, optional load thread)
- [ ] Visualization chart output (latency.png via plotters)
- [ ] Message broker integration (lapin/rumqttc/zeromq feature-gated stubs)

## Commands
- Build: `cargo build --release`
- Run: `cargo run --release`
- Test: `cargo test --release`
- Bench: `cargo bench`

## Expected Artifacts
- Console performance report (P50/P99, missed deadlines)
- latency.png chart (if visualization enabled)
