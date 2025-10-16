// PID control implementation (minimal)
#[derive(Debug, Clone, Copy)]
pub struct PID {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    integral: f64,
    prev_error: f64,
}

impl PID {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self { kp, ki, kd, integral: 0.0, prev_error: 0.0 }
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
    }

    pub fn update(&mut self, setpoint: f64, measurement: f64, dt: f64) -> f64 {
        let error = setpoint - measurement;
        self.integral += error * dt;
        let derivative = (error - self.prev_error) / dt.max(1e-9);
        self.prev_error = error;
        self.kp * error + self.ki * self.integral + self.kd * derivative
    }
}

