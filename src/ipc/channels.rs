// Channel-based communication (std mpsc)
use std::sync::mpsc::{channel, Receiver, Sender};

pub type Tx<T> = Sender<T>;
pub type Rx<T> = Receiver<T>;

pub fn new_channel<T>() -> (Tx<T>, Rx<T>) {
    channel()
}

