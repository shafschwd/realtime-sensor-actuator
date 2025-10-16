// OS thread actuator (stub)
use std::thread::{self, JoinHandle};
use std::time::Duration;

pub fn start() -> JoinHandle<()> {
    thread::spawn(|| {
        for _ in 0..3 {
            thread::sleep(Duration::from_millis(1));
        }
    })
}

