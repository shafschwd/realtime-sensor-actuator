use super::generator::SensorReading;

pub struct AnomalyDetector {
    force_threshold: f32,
    position_threshold: f32,
    temp_threshold: f32,
}

impl AnomalyDetector {
    pub fn new(force_thresh: f32, pos_thresh: f32, temp_thresh: f32) -> Self {
        Self {
            force_threshold: force_thresh,
            position_threshold: pos_thresh,
            temp_threshold: temp_thresh,
        }
    }

    pub fn detect(&self, reading: &SensorReading, baseline: &SensorReading) -> Option<String> {
        if (reading.force - baseline.force).abs() > self.force_threshold {
            return Some(format!("Force anomaly: {} N (baseline: {} N)",
                                reading.force, baseline.force));
        }
        if (reading.position - baseline.position).abs() > self.position_threshold {
            return Some(format!("Position anomaly: {} mm", reading.position));
        }
        if (reading.temperature - baseline.temperature).abs() > self.temp_threshold {
            return Some(format!("Temperature anomaly: {} °C", reading.temperature));
        }
        None
    }
}
