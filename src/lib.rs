//! Real-Time Sensor-Actuator System Library
//!
//! This library provides components for a multi-threaded real-time control system.

pub mod sensor;
pub mod actuator;
pub mod ipc;
pub mod metrics;

// Re-export commonly used types for convenience
pub use sensor::{SensorGenerator, SensorReading, MovingAverageFilter, AnomalyDetector};
pub use actuator::{PIDController, ActuatorType, ActuatorStatus, ActuatorConfig};
pub use ipc::{SystemChannels, DiagnosticLog, ConfigBuffer, ActuatorFeedback};
pub use metrics::TimingMetrics;

