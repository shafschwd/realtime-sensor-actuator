//! Actuator module - PID control and multi-actuator management

use std::time::Instant;

// ============================================================================
// PID CONTROLLER - Proportional-Integral-Derivative control
// ============================================================================

pub struct PIDController {
    kp: f32,
    ki: f32,
    kd: f32,
    setpoint: f32,
    integral: f32,
    prev_error: f32,
    last_error: f32,
    prev_time: Option<Instant>,
    integral_max: f32,
}

impl PIDController {
    pub fn new(kp: f32, ki: f32, kd: f32, setpoint: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            setpoint,
            integral: 0.0,
            prev_error: 0.0,
            last_error: 0.0,
            prev_time: None,
            integral_max: 100.0,
        }
    }

    pub fn compute(&mut self, measured_value: f32) -> f32 {
        let now = Instant::now();
        let error = self.setpoint - measured_value;
        self.last_error = error;

        let dt = if let Some(prev) = self.prev_time {
            now.duration_since(prev).as_secs_f32()
        } else {
            0.001
        };

        // Proportional term
        let p = self.kp * error;

        // Integral term with anti-windup
        self.integral += error * dt;
        self.integral = self.integral.clamp(-self.integral_max, self.integral_max);
        let i = self.ki * self.integral;

        // Derivative term
        let derivative = if dt > 0.0 {
            (error - self.prev_error) / dt
        } else {
            0.0
        };
        let d = self.kd * derivative;

        self.prev_error = error;
        self.prev_time = Some(now);

        p + i + d
    }

    pub fn get_error(&self) -> f32 {
        self.last_error
    }
}

// ============================================================================
// ACTUATOR TYPES
// ============================================================================

#[derive(Clone, Debug)]
pub enum ActuatorType {
    Gripper,
    Motor,
    Stabilizer,
}

impl std::fmt::Display for ActuatorType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ActuatorType::Gripper => write!(f, "Gripper"),
            ActuatorType::Motor => write!(f, "Motor"),
            ActuatorType::Stabilizer => write!(f, "Stabilizer"),
        }
    }
}

// ============================================================================
// ACTUATOR STATUS
// ============================================================================

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

// ============================================================================
// ACTUATOR CONFIGURATION
// ============================================================================

pub struct ActuatorConfig {
    pub name: &'static str,
    pub actuator_type: ActuatorType,
    pub pid: PIDController,
    pub deadline_ms: f64,
    pub feedback_deadline_us: f64,
}

impl ActuatorConfig {
    pub fn create_defaults() -> Vec<Self> {
        vec![
            ActuatorConfig {
                name: "Gripper",
                actuator_type: ActuatorType::Gripper,
                pid: PIDController::new(0.5, 0.1, 0.05, 50.0),
                deadline_ms: 1.0,
                feedback_deadline_us: 500.0,
            },
            ActuatorConfig {
                name: "Motor",
                actuator_type: ActuatorType::Motor,
                pid: PIDController::new(0.8, 0.15, 0.1, 100.0),
                deadline_ms: 2.0,
                feedback_deadline_us: 500.0,
            },
            ActuatorConfig {
                name: "Stabilizer",
                actuator_type: ActuatorType::Stabilizer,
                pid: PIDController::new(0.3, 0.05, 0.02, 25.0),
                deadline_ms: 1.5,
                feedback_deadline_us: 500.0,
            },
        ]
    }
}

