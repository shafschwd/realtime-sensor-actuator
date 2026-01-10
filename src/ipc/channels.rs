use crossbeam::channel::{Sender, Receiver, bounded};
use std::sync::Arc;
use crate::sensor::generator::SensorReading;

#[derive(Clone)]
pub struct SystemChannels {
    // Sensor -> Dispatcher
    pub sensor_tx: Sender<SensorReading>,
    pub sensor_rx: Arc<Receiver<SensorReading>>,

    // Actuator -> Sensor (feedback)
    pub feedback_tx: Sender<ActuatorFeedback>,
    pub feedback_rx: Arc<Receiver<ActuatorFeedback>>,
}

#[derive(Clone, Debug)]
pub enum ActuatorStatus {
    Normal,
    Warning,
    Emergency,
}

impl std::fmt::Display for ActuatorStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ActuatorStatus::Normal => write!(f, "Normal"),
            ActuatorStatus::Warning => write!(f, "Warning"),
            ActuatorStatus::Emergency => write!(f, "Emergency"),
        }
    }
}

#[derive(Clone, Debug)]
pub struct ActuatorFeedback {
    pub timestamp: std::time::Instant,
    pub actuator_name: String,
    pub error: f32,
    pub control: f32,
    pub status: ActuatorStatus,
    pub cycle_id: u64,
}

impl SystemChannels {
    pub fn new(buffer_size: usize) -> Self {
        let (sensor_tx, sensor_rx) = bounded(buffer_size);
        let (feedback_tx, feedback_rx) = bounded(buffer_size);

        Self {
            sensor_tx,
            sensor_rx: Arc::new(sensor_rx),
            feedback_tx,
            feedback_rx: Arc::new(feedback_rx),
        }
    }
}
