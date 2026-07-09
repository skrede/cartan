# The repository root serves two build systems. When ESP-IDF processes cartan
# as a managed component it defines ESP_PLATFORM; in that mode the root registers
# cartan as a header-only IDF component (public include trees + Eigen) and returns
# before reaching the host project below. A normal host configure leaves
# ESP_PLATFORM undefined and runs the full project unchanged.
if(ESP_PLATFORM)
    idf_component_register(
        INCLUDE_DIRS
            "lib/cartan-lie/include"
            "lib/cartan-serial-chain/include"
            "lib/cartan-analytical/include"
        REQUIRES eigen)
    return()
endif()

cmake_minimum_required(VERSION 3.28)
project(cartan VERSION 0.4.2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
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
option(CARTAN_BUILD_URDF "Build URDF loading module" OFF)
option(CARTAN_BUILD_URDF_EXTENDED_TESTS "Build extended URDF parity tests against vendored real-world URDFs" OFF)
option(CARTAN_BUILD_BENCHMARKS "Build the benchmark suite; third-party comparison dependencies are auto-detected and any benchmark whose dependency is missing is omitted with a warning" OFF)
option(CARTAN_FETCH_BENCHMARK_DEPS "Fetch missing benchmark dependencies via FetchContent instead of only using installed ones" OFF)
option(CARTAN_BUILD_PROFILING "Build profiling benchmarks (no external robot library dependencies)" OFF)
option(CARTAN_BUILD_PYTHON "Build Python bindings via nanobind" OFF)
option(CARTAN_BUILD_DOC_TESTS "Compile documentation C++ snippets against the headers" OFF)
option(CARTAN_CMAKE_FETCH_DEPS "Use CMake FetchContent to download and install dependencies" OFF)

set(CARTAN_ARGMIN_SOURCE_DIR "" CACHE PATH "Path to local argmin source checkout")

include(FetchContent)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# The install/export layer is only coherent when every exported target links
# imported dependencies that the generated package config can re-resolve. Under
# FetchContent the local eigen_headers/pugixml aliases are not installable, and
# the Argmin/NLopt backends drag in FetchContent-only link edges; any of these
# would poison install(EXPORT). Gate the whole install surface on a single
# variable so the module files and the root config block stay in lockstep.
if (NOT CARTAN_CMAKE_FETCH_DEPS AND NOT CARTAN_BUILD_ARGMIN AND NOT CARTAN_BUILD_NLOPT)
    set(CARTAN_ENABLE_INSTALL ON)
else ()
    set(CARTAN_ENABLE_INSTALL OFF)
endif ()

add_subdirectory(lib)

# The doc-snippet compile gate reuses the cartan targets defined by lib/, so it
# must be added after them. It is a compile-only target with no CTest entries,
# hence independent of the test/example gates below.
if (CARTAN_BUILD_DOC_TESTS)
    add_subdirectory(docs)
endif ()

if (CARTAN_ENABLE_INSTALL)
    set(CARTAN_INSTALL_CMAKEDIR ${CMAKE_INSTALL_LIBDIR}/cmake/cartan)

    install(EXPORT cartanTargets
        FILE cartanTargets.cmake
        NAMESPACE cartan::
        DESTINATION ${CARTAN_INSTALL_CMAKEDIR}
    )

    configure_package_config_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/cartanConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/cartanConfig.cmake
        INSTALL_DESTINATION ${CARTAN_INSTALL_CMAKEDIR}
    )

    write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/cartanConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMinorVersion
    )

    install(
        FILES
        ${CMAKE_CURRENT_BINARY_DIR}/cartanConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/cartanConfigVersion.cmake
        DESTINATION ${CARTAN_INSTALL_CMAKEDIR}
    )
endif ()

# enable_testing() must be invoked before any add_subdirectory() call that
# registers CTest entries, otherwise CMake will not generate the
# per-subdirectory CTestTestfile.cmake needed for ctest discovery. The gate is
# widened to include CARTAN_BUILD_EXAMPLES so the tutorial smoke tests
# registered under examples/tutorials/ are discoverable without also turning on
# CARTAN_BUILD_TESTS. The tests/ subdirectory itself stays gated on the test
# flags only.
if (CARTAN_BUILD_TESTS OR CARTAN_BUILD_FUZZ_TESTS OR CARTAN_BUILD_PROPERTY_TESTS OR CARTAN_BUILD_EXAMPLES)
    enable_testing()
endif ()

if (CARTAN_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif ()

if (CARTAN_BUILD_TESTS OR CARTAN_BUILD_FUZZ_TESTS OR CARTAN_BUILD_PROPERTY_TESTS)
    add_subdirectory(tests)
endif ()

if (CARTAN_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif ()

if (CARTAN_BUILD_PROFILING)
    add_subdirectory(profiling)
endif ()

if (CARTAN_BUILD_PYTHON)
    add_subdirectory(python)
endif ()
