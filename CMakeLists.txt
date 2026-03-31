cmake_minimum_required(VERSION 3.28)
project(liepp VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_VERBOSE_MAKEFILE ON)

set(LIEPP_WARNING_FLAGS
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

option(LIEPP_BUILD_TESTS "Build unit and compile tests" OFF)
option(LIEPP_BUILD_EXAMPLES "Build examples" OFF)
option(LIEPP_BUILD_FUZZ_TESTS "Build fuzz test targets" OFF)
option(LIEPP_BUILD_PROPERTY_TESTS "Build property-based test targets (requires RapidCheck)" OFF)
option(LIEPP_BUILD_NLOPT "Build NLopt solver backend for IK" OFF)
option(LIEPP_BUILD_BENCHMARKS "Build benchmarks (requires Google Benchmark)" OFF)
option(LIEPP_BUILD_PROFILING "Build profiling benchmarks (no external robot library dependencies)" OFF)
option(LIEPP_CMAKE_FETCH_DEPS "Use CMake FetchContent to download and install dependencies" OFF)

include(FetchContent)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_subdirectory(lib)

if (LIEPP_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif ()

if (LIEPP_BUILD_TESTS OR LIEPP_BUILD_FUZZ_TESTS OR LIEPP_BUILD_PROPERTY_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif ()

if (LIEPP_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif ()

if (LIEPP_BUILD_PROFILING)
    add_subdirectory(profiling)
endif ()
