use std::time::Instant;

pub struct PIDController {
    // Gains
    kp: f32,  // Proportional
    ki: f32,  // Integral
    kd: f32,  // Derivative

    // State
    setpoint: f32,
    integral: f32,
    prev_error: f32,
    last_error: f32,  // Store the last computed error
    prev_time: Option<Instant>,

    // Anti-windup
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

        // Calculate error
        let error = self.setpoint - measured_value;
        self.last_error = error;

        // Calculate dt
        let dt = if let Some(prev) = self.prev_time {
            now.duration_since(prev).as_secs_f32()
        } else {
            0.001 // First call default
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

        // Update state
        self.prev_error = error;
        self.prev_time = Some(now);

        // Output
        p + i + d
    }

    pub fn get_error(&self) -> f32 {
        self.last_error
    }

    #[allow(dead_code)]
    pub fn set_setpoint(&mut self, setpoint: f32) {
        self.setpoint = setpoint;
    }

    #[allow(dead_code)]
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.last_error = 0.0;
        self.prev_time = None;
    }
}
