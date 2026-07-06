# Compile-only smoke test verifying cartan headers build under xtensa-esp32-elf
# and riscv32-esp-elf toolchains via ESP-IDF. No flash, no hardware required;
# `idf.py reconfigure build` exercises the language-level compatibility.
#
# To run:
#   1. Install ESP-IDF v5.1+ (https://docs.espressif.com/projects/esp-idf/).
#   2. `. $IDF_PATH/export.sh`
#   3. `cd tests/embedded/esp32-smoke && idf.py set-target esp32s3 && idf.py reconfigure build`
#
# The smoke confirms three propositions:
#   - cartan-lie compiles (SE3/SO3 with float scalar)
#   - cartan-serial-chain compiles (kinematic_chain<float, 3>, FK)
#   - cartan-analytical compiles (paden_kahan subproblems)
# Hardware deployment, Eigen alignment audit (Xtensa LX6/LX7 stack alignment
# vs Eigen's vectorization assumptions), memory + flash sweep, and an actual
# robot-arm demo are out of scope for this smoke; those land in a future
# embedded-targeting effort.

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
