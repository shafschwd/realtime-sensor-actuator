#!/usr/bin/env bash
set -euo pipefail

# Resolve to repo root even if invoked from scripts/ or elsewhere
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

REQ=".copilot/requirements.json"
if [[ ! -f "$REQ" ]]; then
  echo "Missing $REQ"; exit 1
fi

echo "== Verify Requirements for realtime-sensor-actuator =="
echo "Repository root: $REPO_ROOT"
echo

# Optional: show top of Cargo.toml for debugging
echo "Top of Cargo.toml (debug):"
head -n 40 Cargo.toml || true
echo

# 1) Build
echo "[1/6] Build"
cargo build --release
echo

# 2) Tests
echo "[2/6] Tests"
cargo test --release
echo

# 3) Benches (smoke run). Allow non-zero exit if environment lacks perf counters, etc.
echo "[3/6] Benches (smoke)"
cargo bench --quiet || true
echo

# 4) Crates present (robust TOML checks)
echo "[4/6] Crates present"
missing=0
has_crate() {
  local name="$1"
  # Match `name = "x"` or `name = { ... }` or `[dependencies.name]` / `[dev-dependencies.name]`
  if grep -Eiq "^[[:space:]]*${name}[[:space:]]*=" Cargo.toml || \
     grep -Eiq "^\[dependencies\.${name}\]" Cargo.toml || \
     grep -Eiq "^\[dev-dependencies\.${name}\]" Cargo.toml; then
    echo "✓ found ${name}"
  else
    echo "Missing crate in Cargo.toml: ${name}"
    missing=1
  fi
}

has_crate tokio
has_crate crossbeam
has_crate parking_lot
has_crate hdrhistogram
has_crate criterion
# Optional crates (do not fail if absent)
for opt in rayon plotters lapin rumqttc zeromq tracing tracing-subscriber serde toml rand chrono; do
  if grep -Eiq "^[[:space:]]*${opt}[[:space:]]*=" Cargo.toml || \
     grep -Eiq "^\[dependencies\.${opt}\]" Cargo.toml || \
     grep -Eiq "^\[dev-dependencies\.${opt}\]" Cargo.toml; then
    echo "• optional present: ${opt}"
  fi
done

if [[ $missing -ne 0 ]]; then
  echo "One or more required crates are missing"; exit 1
fi
echo

# 5) Files present
echo "[5/6] Files present"
files=(
  "src/main.rs"
  "src/lib.rs"
  "src/config.rs"
  "src/sensor/mod.rs"
  "src/sensor/generator.rs"
  "src/sensor/filter.rs"
  "src/sensor/anomaly.rs"
  "src/actuator/mod.rs"
  "src/actuator/controller.rs"
  "src/actuator/manager.rs"
  "src/ipc/mod.rs"
  "src/ipc/channels.rs"
  "src/ipc/shared_resource.rs"
  "src/benchmark/mod.rs"
  "src/benchmark/metrics.rs"
  "src/benchmark/analysis.rs"
  "src/threaded_impl/mod.rs"
  "src/threaded_impl/sensor_thread.rs"
  "src/threaded_impl/actuator_thread.rs"
  "src/async_impl/mod.rs"
  "src/async_impl/sensor_task.rs"
  "src/async_impl/actuator_task.rs"
  "benches/system_benchmark.rs"
  "tests/integration_test.rs"
  "tests/fault_injection_test.rs"
  "config/system_config.toml"
)
for f in "${files[@]}"; do
  if [[ -f "$f" ]]; then
    echo "✓ found $f"
  else
    echo "Missing file: $f"; exit 1
  fi
done
echo

# 6) Run for metrics (10–12s)
echo "[6/6] Run for metrics (10s)"
if command -v timeout >/dev/null 2>&1; then
  timeout 12s cargo run --release || true
else
  cargo run --release &
  PID=$!
  sleep 12
  kill $PID >/dev/null 2>&1 || true
fi
echo

# Post-run artifacts
echo "Artifacts:"
[[ -f latency.png ]] && echo "✓ latency.png generated" || echo "• latency.png not found (visualization optional)"
echo

echo "All checklist checks completed successfully."
