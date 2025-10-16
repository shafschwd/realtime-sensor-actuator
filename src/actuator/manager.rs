use super::controller::PIDController;
use crate::sensor::generator::SensorReading;

pub enum ActuatorType {
    Gripper,
    Motor,
    Stabilizer,
}

pub struct Actuator {
    #[allow(dead_code)]
    pub actuator_type: ActuatorType,
    pub pid: PIDController,
    pub current_command: f32,
}

pub struct ActuatorManager {
    actuators: Vec<Actuator>,
}

impl ActuatorManager {
    pub fn new() -> Self {
        Self {
            actuators: vec![
                Actuator {
                    actuator_type: ActuatorType::Gripper,
                    pid: PIDController::new(0.5, 0.1, 0.05, 50.0), // Force control
                    current_command: 0.0,
                },
                Actuator {
                    actuator_type: ActuatorType::Motor,
                    pid: PIDController::new(0.8, 0.15, 0.1, 100.0), // Position control
                    current_command: 0.0,
                },
                Actuator {
                    actuator_type: ActuatorType::Stabilizer,
                    pid: PIDController::new(0.3, 0.05, 0.02, 25.0), // Temp control
                    current_command: 0.0,
                },
            ],
        }
    }

    pub fn update(&mut self, reading: &SensorReading) {
        // Update each actuator based on corresponding sensor value
        self.actuators[0].current_command = self.actuators[0].pid.compute(reading.force);
        self.actuators[1].current_command = self.actuators[1].pid.compute(reading.position);
        self.actuators[2].current_command = self.actuators[2].pid.compute(reading.temperature);
    }

    pub fn get_commands(&self) -> Vec<f32> {
        self.actuators.iter().map(|a| a.current_command).collect()
    }
}
