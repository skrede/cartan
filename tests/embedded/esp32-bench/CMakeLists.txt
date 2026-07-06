# Cartan ESP-IDF on-device IK timing bench: compiles cartan under the
# xtensa-esp32-elf toolchain and, when flashed, sweeps the native (dependency-
# free) iterative solvers over two robots with float scalars, reporting per-cell
# convergence and worst-case cycle/us timing. CI runs the compile leg only
# (no hardware); on a board `idf.py flash monitor` streams a human-readable table
# on the console UART (USB0) and machine-parseable CSV on a second UART (USB1).
#
# Wiring for the CSV telemetry link (see main/main.cpp k_tele_tx/k_tele_rx):
#   ESP32 GPIO17 (TX) -> external USB-serial RX
#   ESP32 GPIO16 (RX) <- external USB-serial TX
#
# To run:
#   1. Install ESP-IDF v5.1+ (https://docs.espressif.com/projects/esp-idf/).
#   2. `. $IDF_PATH/export.sh`
#   3. `cd tests/embedded/esp32-bench && idf.py set-target esp32 && idf.py build`
#   4. To measure on hardware: `idf.py -p <port> flash monitor`.
#
# Exercises cartan-lie (SE3/SO3, float), cartan-serial-chain (kinematic_chain,
# FK, projected_lm/dls/lm/lbfgsb), and the FK re-verify path. A multi-board
# flash/RAM sweep and a robot-arm demo remain future embedded-targeting work.

cmake_minimum_required(VERSION 3.16)

set(CARTAN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../..")

# Expose cartan's public include directories to all components in this
# project. Each header-only sublib's include/ root becomes a system include
# so we do not pollute the warning surface with cartan-internal noise.
set(EXTRA_COMPONENT_DIRS "")
list(APPEND CMAKE_INCLUDE_PATH
    "${CARTAN_ROOT}/lib/cartan-lie/include"
    "${CARTAN_ROOT}/lib/cartan-serial-chain/include"
    "${CARTAN_ROOT}/lib/cartan-analytical/include"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(cartan_esp32_bench)
