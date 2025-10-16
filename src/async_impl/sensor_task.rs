use tokio::time::{interval, Duration, Instant};
use crate::sensor::{generator::SensorGenerator, filter::MovingAverageFilter};
use crate::ipc::channels::SystemChannels;
use crate::benchmark::metrics::TimingMetrics;

#[allow(dead_code)]
pub async fn sensor_task(
    channels: SystemChannels,
    metrics: TimingMetrics,
) {
    let mut generator = SensorGenerator::new(42);
    let mut filter = MovingAverageFilter::new(5);
    let mut interval_timer = interval(Duration::from_millis(5));

    loop {
        interval_timer.tick().await;

        let gen_start = Instant::now();
        let reading = generator.generate();
        metrics.record_generation(gen_start.elapsed());

        let proc_start = Instant::now();
        let filtered = filter.filter(&reading);
        metrics.record_processing(proc_start.elapsed(), 200_000);

        let tx_start = Instant::now();
        if channels.sensor_tx.send(filtered).is_err() {
            break;
        }
        metrics.record_transmission(tx_start.elapsed());
    }
}
