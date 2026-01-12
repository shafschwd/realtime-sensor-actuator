# Real-Time Sensor-Actuator System

A multi-threaded real-time control system implemented in Rust, demonstrating sensor data processing, PID control, and deadline tracking.

## Project Structure

```
realtime-sensor-actuator/
├── Cargo.toml              # Project dependencies and metadata
├── Cargo.lock              # Locked dependency versions
├── README.md               # This file
├── config/
│   └── system_config.toml  # System configuration (intervals, deadlines)
├── src/
│   ├── main.rs             # Entry point - orchestrates the system
│   ├── sensor.rs           # Sensor data generation & processing
│   ├── actuator.rs         # PID controller & actuator management
│   ├── ipc.rs              # Inter-thread communication & shared resources
│   └── metrics.rs          # Performance tracking & statistics
├── benches/
│   └── system_benchmark.rs # Criterion benchmarks
├── tests/
│   ├── integration_test.rs # End-to-end tests
│   └── fault_injection_test.rs # Fault tolerance tests
└── logs/                   # Log output directory
```

## File Descriptions

### `src/main.rs` - System Entry Point
- **Purpose**: Orchestrates the entire real-time system
- **Key Components**:
  - `SystemStats`: Atomic counters for tracking cycles, deadlines, and per-actuator statistics
  - `spawn_sensor_thread()`: Creates the sensor thread that generates and processes data
  - `spawn_actuator_thread()`: Creates the actuator thread that runs PID control
  - `main()`: Initializes components, spawns threads, runs for 10 seconds, then prints results

### `src/sensor.rs` - Sensor Module
- **Purpose**: Simulates real-time sensor data generation and processing
- **Key Components**:
  - `SensorReading`: Data structure holding force, position, temperature, and timestamp
  - `SensorGenerator`: Generates simulated sensor data with configurable noise
  - `MovingAverageFilter`: Noise reduction using a sliding window average
  - `AnomalyDetector`: Detects abnormal readings based on thresholds

### `src/actuator.rs` - Actuator Module  
- **Purpose**: Implements PID control and manages multiple actuators
- **Key Components**:
  - `PIDController`: Proportional-Integral-Derivative controller with anti-windup
  - `ActuatorType`: Enum for different actuator types (Gripper, Motor, Stabilizer)
  - `ActuatorStatus`: Enum for actuator states (Normal, Warning, Emergency)
  - `ActuatorConfig`: Configuration for each actuator including deadlines and PID gains

### `src/ipc.rs` - Inter-Process Communication
- **Purpose**: Manages communication between threads and shared resources
- **Key Components**:
  - `SystemChannels`: Crossbeam channels for sensor→actuator and feedback communication
  - `ActuatorFeedback`: Feedback data structure from actuators back to sensor
  - `DiagnosticLog`: Thread-safe logging using RwLock
  - `ConfigBuffer`: Thread-safe configuration storage using Mutex
  - `load_config()`: Loads configuration from TOML file

### `src/metrics.rs` - Performance Metrics
- **Purpose**: Tracks and reports system performance statistics
- **Key Components**:
  - `TimingMetrics`: HDR histogram-based latency tracking
  - `MetricsReport`: Summary statistics (P50, P99 latencies)
  - Tracks: generation time, processing time, transmission time, end-to-end latency

### `config/system_config.toml` - Configuration File
- **Purpose**: External configuration for system parameters
- **Settings**:
  - `sensor_interval_ms`: How often sensor generates data (default: 10ms = 100Hz)
  - `processing_deadline_ms`: Max allowed processing time
  - `transmission_deadline_ms`: Max allowed transmission time
  - `fail_safe_enabled`: Enable/disable fail-safe mode

## How It Works

```
┌─────────────────┐     Channel      ┌─────────────────┐
│  Sensor Thread  │ ───────────────► │ Actuator Thread │
│                 │                  │                 │
│ • Generate data │                  │ • Receive data  │
│ • Filter noise  │                  │ • PID control   │
│ • Detect anomaly│ ◄─────────────── │ • Send feedback │
│ • Check deadline│     Feedback     │ • Check deadline│
└─────────────────┘                  └─────────────────┘
```

1. **Sensor Thread**: Generates sensor readings at configured intervals (100Hz default)
2. **Processing**: Applies moving average filter and anomaly detection
3. **Transmission**: Sends filtered data via crossbeam channel to actuator thread
4. **Actuator Thread**: Receives data and runs PID control for 3 actuators:
   - **Gripper**: 1.0ms deadline, controls grip force
   - **Motor**: 2.0ms deadline, controls position
   - **Stabilizer**: 1.5ms deadline, controls temperature
5. **Feedback**: Actuators send feedback back to sensor thread
6. **Deadline Tracking**: All operations are timed and checked against deadlines

## Building and Running

```bash
# Build the project
cargo build --release

# Run the system (runs for 10 seconds)
cargo run --release

# Run benchmarks
cargo bench

# Run tests
cargo test
```

## Output Explanation

The system outputs real-time logs showing:
- `[X.XXXs] SENSOR:` - Sensor data generation and processing
- `[X.XXXs] DISPATCHER:` - Data routing to actuators
- `[X.XXXs] Gripper/Motor/Stabilizer:` - Actuator processing results
- `[DEADLINE]` - Deadline violations (when processing exceeds allowed time)
- `[PERF]` - Periodic performance summaries

### Final Results Include:
- **Total Cycles**: Number of sensor readings processed
- **Deadline Compliance**: Percentage of operations meeting deadlines
- **Actuator Performance**: Per-actuator compliance statistics
- **Latency Metrics**: P50 and P99 latencies for all operations

## Dependencies

- `crossbeam`: Lock-free channels for thread communication
- `parking_lot`: Efficient Mutex and RwLock implementations
- `rand`: Random number generation for sensor simulation
- `hdrhistogram`: High Dynamic Range histogram for latency tracking
- `toml` / `serde`: Configuration file parsing

## Real-Time Characteristics

- **Bounded channels**: Prevents unbounded memory growth
- **Deadline tracking**: Every operation is timed and verified
- **Atomic statistics**: Lock-free counters for performance tracking
- **Configurable intervals**: Adjust timing via config file
