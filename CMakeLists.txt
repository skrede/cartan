cmake_minimum_required(VERSION 3.28)
project(cartan VERSION 0.4.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_VERBOSE_MAKEFILE ON)

set(CARTAN_WARNING_FLAGS
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /permissive- /utf-8>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:
    -fPIC
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wconversion
    -Wsign-conversion
    -Wold-style-cast
    -Wcast-align
    -Woverloaded-virtual
    -Wnon-virtual-dtor
    -Wdouble-promotion
    -Wimplicit-fallthrough
    -Wformat=2
    >
)

option(CARTAN_BUILD_TESTS "Build unit and compile tests" OFF)
option(CARTAN_BUILD_EXAMPLES "Build examples" OFF)
option(CARTAN_BUILD_FUZZ_TESTS "Build fuzz test targets" OFF)
option(CARTAN_BUILD_PROPERTY_TESTS "Build property-based test targets (requires RapidCheck)" OFF)
option(CARTAN_BUILD_ARGMIN "Build with Argmin NLP solver backend" OFF)
option(CARTAN_BUILD_NLOPT "Build NLopt solver backend for IK" OFF)
option(CARTAN_BUILD_BENCHMARKS "Build benchmarks (requires Google Benchmark)" OFF)
option(CARTAN_BUILD_TRAC_IK "Build TRAC-IK comparison benchmarks (requires orocos-kdl, NLopt)" OFF)
option(CARTAN_BUILD_PINOCCHIO "Build Pinocchio comparison benchmarks (requires pinocchio)" OFF)
option(CARTAN_BUILD_PROFILING "Build profiling benchmarks (no external robot library dependencies)" OFF)
option(CARTAN_CMAKE_FETCH_DEPS "Use CMake FetchContent to download and install dependencies" OFF)

set(CARTAN_ARGMIN_SOURCE_DIR "" CACHE PATH "Path to local argmin source checkout")

include(FetchContent)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_subdirectory(lib)

if (CARTAN_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif ()

if (CARTAN_BUILD_TESTS OR CARTAN_BUILD_FUZZ_TESTS OR CARTAN_BUILD_PROPERTY_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif ()

if (CARTAN_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif ()

if (CARTAN_BUILD_PROFILING)
    add_subdirectory(profiling)
endif ()
