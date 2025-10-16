// Noise reduction (moving average) - minimal implementation
use std::collections::VecDeque;

pub struct MovingAverage {
    window: usize,
    buf: VecDeque<f64>,
    sum: f64,
}

impl MovingAverage {
    pub fn new(window: usize) -> Self {
        Self { window: window.max(1), buf: VecDeque::new(), sum: 0.0 }
    }

    pub fn push(&mut self, x: f64) -> f64 {
        self.buf.push_back(x);
        self.sum += x;
        if self.buf.len() > self.window {
            if let Some(old) = self.buf.pop_front() { self.sum -= old; }
        }
        self.sum / self.buf.len() as f64
    }
}

