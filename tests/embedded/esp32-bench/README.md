# cartan ESP32 on-device IK timing bench

An ESP-IDF project that runs cartan's native (dependency-free) inverse-kinematics
solvers on real silicon and reports their on-device convergence and worst-case
timing. Where the [`esp32-smoke`](../esp32-smoke) example proves the headers
compile and a single solve runs, this bench measures how the solvers actually
behave under a size-optimized (`-Os`), `float`, exceptions-off embedded build.

## What it does

`main/main.cpp` sweeps four native steppers — `projected_lm`, `dls`, `lm`
(builtin), and `lbfgsb` (builtin) — each `no_limits`, over a planar 3R and a 6R
chain. For every (robot, solver) cell it solves eight forward-kinematics-walked
targets from a zero seed, timing each solve with the CPU cycle counter and
independently FK-re-verifying the body-twist error of every converged solution
(never trusting the solver's self-report). It reports per cell: convergence
count, total iterations, min/avg/max cycles, and the worst re-verify error.

## Two output transports

- **Console UART (USB0)** — a human-readable table via `ESP_LOGI`.
- **Second UART (USB1)** — machine-parseable CSV, for piping into analysis. The
  telemetry TX/RX default to **GPIO17 / GPIO16** (`k_tele_tx` / `k_tele_rx` in
  `main/main.cpp`); wire your USB-serial adapter's RX to GPIO17 and its TX to
  GPIO16, or change the constants to match your board.

## Build and run

```sh
. $IDF_PATH/export.sh
cd tests/embedded/esp32-bench
idf.py set-target esp32
idf.py build                 # compile-only; this is what CI runs
idf.py -p /dev/ttyUSB0 flash monitor   # measure on a connected board
```

If the ESP Component Registry is unreachable, point at a local Eigen checkout:
`export CARTAN_EIGEN_DIR=/path/to/eigen` before `idf.py reconfigure build`.

## What the numbers show

A representative run on an ESP32-D0WD (Xtensa LX6, 160 MHz) illustrates why
worst-case matters more than best-case for a deadline-bound controller: `dls`
converges on every 6R target within a tight ~23 ms bound, while the fast-in-the-
best-case `projected_lm` can spike past 500 ms when a hard target triggers
repeated restarts. The CSV stream is intended to feed exactly this kind of
per-solver timing profile into a real-time-safety budget.

The measured latencies are hardware- and build-specific; treat the harness as
the deliverable and re-measure on your own target and toolchain.
