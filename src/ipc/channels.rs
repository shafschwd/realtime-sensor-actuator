use crossbeam::channel::{Sender, Receiver, bounded};
use std::sync::Arc;
use crate::sensor::generator::SensorReading;

#[derive(Clone)]
pub struct SystemChannels {
    // Sensor -> Actuator
    pub sensor_tx: Sender<SensorReading>,
    pub sensor_rx: Arc<Receiver<SensorReading>>,

    // Actuator -> Sensor (feedback)
    pub feedback_tx: Sender<ActuatorFeedback>,
    pub feedback_rx: Arc<Receiver<ActuatorFeedback>>,
}

#[derive(Clone, Debug)]
pub struct ActuatorFeedback {
    #[allow(dead_code)]
    pub timestamp: std::time::Instant,
    #[allow(dead_code)]
    pub commands: Vec<f32>,
    pub status: String,
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
