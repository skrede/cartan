# cartan ESP32 compile-only smoke

A minimal ESP-IDF project that verifies cartan headers compile cleanly under
the xtensa-esp32-elf and riscv32-esp-elf toolchains. The smoke proves the
language-level compatibility added in the v0.4.1 C++20 downgrade (PR migrating
`std::expected` to `cartan::expected`); it does NOT prove hardware-level
correctness, memory footprint, Eigen alignment safety, or any deployment
characteristic. Those land in a follow-on Embedded Targeting milestone.

## What this smoke does

`main/main.cpp` constructs a planar 3R `kinematic_chain<float, 3>`, runs
forward kinematics on it, and invokes Paden-Kahan subproblem 1 from
`cartan::analytical`. The `float` scalar type is the embedded default. Build
success at the link step is the only pass condition; `app_main` produces a
single `ESP_LOGI` line if booted but is not asserted from outside.

## Build (no hardware required)

Install ESP-IDF v5.1+ following the
[official guide](https://docs.espressif.com/projects/esp-idf/en/v5.1/esp32/get-started/index.html),
then:

```sh
. $IDF_PATH/export.sh
cd tests/embedded/esp32-smoke
idf.py set-target esp32s3       # or esp32, esp32c3, esp32c6, esp32h2
idf.py reconfigure build
```

If the ESP Component Registry is unreachable in your environment, point the
build at a local Eigen checkout instead:

```sh
export CARTAN_EIGEN_DIR=/path/to/eigen
idf.py reconfigure build
```

## Targets verified

The smoke is configured for ESP32-S3 by default; switching targets is a
single `idf.py set-target <target>` invocation. C++20 baseline + Eigen
header-only mean the same source compiles unchanged on:

- ESP32 (Xtensa LX6)
- ESP32-S2 (Xtensa LX7)
- ESP32-S3 (Xtensa LX7, dual-core)
- ESP32-C3 (RISC-V)
- ESP32-C6 (RISC-V)
- ESP32-H2 (RISC-V)

## Out of scope here (deferred to Embedded Targeting milestone)

- Flashing and running on real hardware
- Memory + flash footprint sweep across the cartan public surface
- Eigen alignment / `EIGEN_DONT_ALIGN_STATICALLY` investigation under Xtensa
  stack alignment constraints
- IK solver hot-loop benchmarking on a microcontroller (closed-form vs
  iterative trade analysis tightens at ~50 us / iteration on a 240 MHz core)
- A real robot-arm demo driving a small 6R/7R from `cartan.canonical.*`
