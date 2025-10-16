use super::metrics::MetricsReport;
use plotters::prelude::*;

pub fn generate_latency_chart(report: &MetricsReport, path: &str) -> Result<(), Box<dyn std::error::Error>> {
    let root = BitMapBackend::new(path, (800, 600)).into_drawing_area();
    root.fill(&WHITE)?;
    let max_ms = report.e2e_p99.as_secs_f64() * 1000.0 * 1.2;

    let mut chart = ChartBuilder::on(&root)
        .caption("End-to-End Latency (ms)", ("sans-serif", 24))
        .margin(10)
        .x_label_area_size(40)
        .y_label_area_size(50)
        .build_cartesian_2d(0.0..100.0, 0.0..max_ms)?;

    chart.configure_mesh().x_desc("Quantile").y_desc("Latency (ms)").draw()?;

    chart.draw_series(LineSeries::new(
        vec![(50.0, report.e2e_p50.as_secs_f64() * 1000.0), (99.0, report.e2e_p99.as_secs_f64() * 1000.0)],
        &RED,
    ))?;
    root.present()?;
    Ok(())
}
