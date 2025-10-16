use crate::benchmark::metrics::MetricsReport;
use crate::benchmark::analysis::generate_latency_chart;

pub fn render_report_charts(report: &MetricsReport) {
    let _ = generate_latency_chart(report, "latency.png");
}
