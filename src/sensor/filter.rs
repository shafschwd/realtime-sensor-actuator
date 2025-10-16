use super::generator::SensorReading;
use std::collections::VecDeque;

pub struct MovingAverageFilter {
    window_size: usize,
    force_buffer: VecDeque<f32>,
    position_buffer: VecDeque<f32>,
    temp_buffer: VecDeque<f32>,
}

impl MovingAverageFilter {
    pub fn new(window_size: usize) -> Self {
        Self {
            window_size,
            force_buffer: VecDeque::with_capacity(window_size),
            position_buffer: VecDeque::with_capacity(window_size),
            temp_buffer: VecDeque::with_capacity(window_size),
        }
    }

    pub fn filter(&mut self, reading: &SensorReading) -> SensorReading {
        // Add new values
        self.force_buffer.push_back(reading.force);
        self.position_buffer.push_back(reading.position);
        self.temp_buffer.push_back(reading.temperature);

        // Remove oldest if exceeding window
        if self.force_buffer.len() > self.window_size {
            self.force_buffer.pop_front();
            self.position_buffer.pop_front();
            self.temp_buffer.pop_front();
        }

        // Calculate averages
        let avg_force: f32 = self.force_buffer.iter().sum::<f32>() / self.force_buffer.len() as f32;
        let avg_pos: f32 = self.position_buffer.iter().sum::<f32>() / self.position_buffer.len() as f32;
        let avg_temp: f32 = self.temp_buffer.iter().sum::<f32>() / self.temp_buffer.len() as f32;

        SensorReading {
            force: avg_force,
            position: avg_pos,
            temperature: avg_temp,
            ..*reading
        }
    }
}
