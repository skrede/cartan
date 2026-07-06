# Cartan ESP-IDF smoke: compiles cartan under the xtensa-esp32-elf and
# riscv32-esp-elf toolchains and, when flashed, runs a live numeric check on the
# chip. CI runs the compile leg only (no hardware); `idf.py build` proves the
# language-level compatibility. On a connected board `idf.py flash monitor` also
# runs app_main, which streams FK, a Paden-Kahan subproblem, and a full float IK
# solve with cycle/us timing over the console UART for comparison against host.
#
# To run:
#   1. Install ESP-IDF v5.1+ (https://docs.espressif.com/projects/esp-idf/).
#   2. `. $IDF_PATH/export.sh`
#   3. `cd tests/embedded/esp32-smoke && idf.py set-target esp32 && idf.py build`
#   4. To validate on hardware: `idf.py -p <port> flash monitor`.
#
# Exercises cartan-lie (SE3/SO3, float), cartan-serial-chain (kinematic_chain,
# FK, iterative IK), and cartan-analytical (paden_kahan). A multi-board flash/RAM
# sweep and a robot-arm demo remain future embedded-targeting work.

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
project(cartan_esp32_smoke)
