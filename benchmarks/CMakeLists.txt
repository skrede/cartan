# Benchmark dependencies are detected here, not hard-required. Under
# CARTAN_BUILD_BENCHMARKS each third-party dependency is looked for (and, when
# CARTAN_FETCH_BENCHMARK_DEPS is ON, fetched via FetchContent if missing); any
# benchmark whose dependency cannot be satisfied is omitted with a warning
# rather than failing the configure. This is the only place third-party domain
# libraries are introduced, referenced, or linked -- tests/ stay cartan-only.

# --- Google Benchmark: the harness every benchmark target needs ---
if (CARTAN_FETCH_BENCHMARK_DEPS)
    FetchContent_Declare(
        benchmark
        GIT_REPOSITORY https://github.com/google/benchmark.git
        GIT_TAG v1.9.5
        EXCLUDE_FROM_ALL
    )
    set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
    set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(benchmark)
else ()
    find_package(benchmark CONFIG QUIET)
endif ()

if (NOT TARGET benchmark::benchmark)
    message(WARNING
        "cartan benchmarks: Google Benchmark not found -- skipping all benchmarks. "
        "Install it or configure with -DCARTAN_FETCH_BENCHMARK_DEPS=ON.")
    return ()
endif ()

# --- orocos-kdl: benchmark_utils.h ships KDL chain factories alongside the
# cartan PoE ones, so every benchmark target links it. Treat KDL as a
# suite-wide dependency: absent it, no benchmark can build. ---
find_package(orocos_kdl CONFIG QUIET)
if (NOT TARGET orocos-kdl)
    message(WARNING
        "cartan benchmarks: orocos-kdl not found -- skipping all benchmarks "
        "(benchmark_utils.h depends on KDL chain factories). Install orocos-kdl to enable them.")
    return ()
endif ()

function(cartan_add_benchmark BENCH_NAME BENCH_SOURCE)
    add_executable(${BENCH_NAME} ${BENCH_SOURCE})
    target_link_libraries(${BENCH_NAME}
        PRIVATE
        cartan::cartan
        benchmark::benchmark
        benchmark::benchmark_main
        orocos-kdl
    )
    target_compile_options(${BENCH_NAME} PRIVATE ${CARTAN_WARNING_FLAGS})
endfunction()

# Lie group benchmarks
cartan_add_benchmark(lie_group_benchmarks lie_group_benchmarks.cpp)

# FK and Jacobian benchmarks
cartan_add_benchmark(fk_benchmarks fk_benchmarks.cpp)
cartan_add_benchmark(jacobian_benchmarks jacobian_benchmarks.cpp)

# IK solve policy benchmarks (basic_ik_runner)
cartan_add_benchmark(basic_ik_dls_benchmarks basic_ik_dls_benchmarks.cpp)
cartan_add_benchmark(basic_ik_lm_benchmarks basic_ik_lm_benchmarks.cpp)

# Exhaustive IK solver benchmarks
cartan_add_benchmark(exhaustive_ik_full_benchmarks exhaustive_ik_full_benchmarks.cpp)

# BOBYQA/SLSQP benchmarks require NLopt (a library solver backend, not a
# benchmark-only comparison dependency).
if (CARTAN_BUILD_NLOPT)
    if (NOT TARGET NLopt::nlopt)
        find_package(NLopt CONFIG REQUIRED)
    endif()
    cartan_add_benchmark(basic_ik_sqp_benchmarks basic_ik_sqp_benchmarks.cpp)
    target_compile_definitions(basic_ik_sqp_benchmarks PRIVATE CARTAN_HAS_NLOPT)
    target_link_libraries(basic_ik_sqp_benchmarks PRIVATE NLopt::nlopt)
endif()

# --- TRAC-IK comparison benchmarks: trac_ik ships no find_package config, so it
# is obtained from source -- fetched when CARTAN_FETCH_BENCHMARK_DEPS is ON, or
# from a caller-provided CARTAN_TRAC_IK_SOURCE_DIR. Absent both, the comparison
# benchmarks are omitted with a warning. ---
set(CARTAN_HAVE_TRAC_IK FALSE)
if (CARTAN_TRAC_IK_SOURCE_DIR OR CARTAN_FETCH_BENCHMARK_DEPS)
    if (CARTAN_TRAC_IK_SOURCE_DIR)
        FetchContent_Declare(
            trac_ik_lib
            SOURCE_DIR "${CARTAN_TRAC_IK_SOURCE_DIR}"
            EXCLUDE_FROM_ALL
            SYSTEM
        )
    else ()
        FetchContent_Declare(
            trac_ik_lib
            GIT_REPOSITORY git@github.com:skrede/trac_ik_lib.git
            GIT_TAG master
            EXCLUDE_FROM_ALL
            SYSTEM
        )
    endif ()
    set(TRAC_IK_WITH_URDF OFF CACHE BOOL "" FORCE)
    find_package(Eigen3 CONFIG REQUIRED)
    set(EIGEN3_FOUND TRUE) # To satisfy orocos-kdl and their wrong capitalization requirement of Eigen3
    set(EIGEN3_INCLUDE_DIR "${EIGEN3_INCLUDE_DIR}")
    FetchContent_MakeAvailable(trac_ik_lib)
    set(CARTAN_HAVE_TRAC_IK TRUE)
else ()
    message(WARNING
        "cartan benchmarks: TRAC-IK comparison benchmarks need the trac_ik source -- skipping. "
        "Configure with -DCARTAN_FETCH_BENCHMARK_DEPS=ON or provide -DCARTAN_TRAC_IK_SOURCE_DIR=<path>.")
endif ()

if (CARTAN_HAVE_TRAC_IK)
    cartan_add_benchmark(ik_comparison_benchmarks ik_comparison_benchmarks.cpp)
    target_link_libraries(ik_comparison_benchmarks PRIVATE trac_ik)
    if(CARTAN_BUILD_NLOPT)
        target_compile_definitions(ik_comparison_benchmarks PRIVATE CARTAN_HAS_NLOPT)
        target_link_libraries(ik_comparison_benchmarks PRIVATE NLopt::nlopt)
    endif()
    if(CARTAN_BUILD_ARGMIN)
        target_compile_definitions(ik_comparison_benchmarks PRIVATE CARTAN_BUILD_ARGMIN)
        target_link_libraries(ik_comparison_benchmarks PRIVATE argmin::argmin)
    endif()

    cartan_add_benchmark(basic_ik_full_benchmarks basic_ik_full_benchmarks.cpp)
    target_link_libraries(basic_ik_full_benchmarks PRIVATE trac_ik)
    if(CARTAN_BUILD_ARGMIN)
        target_compile_definitions(basic_ik_full_benchmarks PRIVATE CARTAN_BUILD_ARGMIN)
        target_link_libraries(basic_ik_full_benchmarks PRIVATE argmin::argmin)
    endif()
    if(CARTAN_BUILD_NLOPT)
        target_compile_definitions(basic_ik_full_benchmarks PRIVATE CARTAN_HAS_NLOPT)
        target_link_libraries(basic_ik_full_benchmarks PRIVATE NLopt::nlopt)
    endif()

    if(CARTAN_BUILD_ARGMIN)
        add_executable(slsqp_per_pose_capture slsqp_per_pose_capture.cpp)
        target_link_libraries(slsqp_per_pose_capture
            PRIVATE
            cartan::cartan
            argmin::argmin)
        target_compile_options(slsqp_per_pose_capture PRIVATE ${CARTAN_WARNING_FLAGS})

        add_executable(slsqp_per_pose_trace slsqp_per_pose_trace.cpp)
        target_link_libraries(slsqp_per_pose_trace
            PRIVATE
            cartan::cartan
            argmin::argmin)
        target_compile_options(slsqp_per_pose_trace PRIVATE ${CARTAN_WARNING_FLAGS})
    endif()

    if(CARTAN_BUILD_NLOPT)
        add_executable(nlopt_slsqp_per_pose_capture nlopt_slsqp_per_pose_capture.cpp)
        target_compile_definitions(nlopt_slsqp_per_pose_capture PRIVATE CARTAN_HAS_NLOPT)
        target_link_libraries(nlopt_slsqp_per_pose_capture
            PRIVATE
            cartan::cartan
            NLopt::nlopt)
        target_compile_options(nlopt_slsqp_per_pose_capture PRIVATE ${CARTAN_WARNING_FLAGS})
    endif()

    cartan_add_benchmark(fk_comparison_benchmarks fk_comparison_benchmarks.cpp)
    target_link_libraries(fk_comparison_benchmarks PRIVATE trac_ik)

    cartan_add_benchmark(jacobian_comparison_benchmarks jacobian_comparison_benchmarks.cpp)
    target_link_libraries(jacobian_comparison_benchmarks PRIVATE trac_ik)
endif ()

# --- Pinocchio comparison benchmarks: pinocchio ships a find_package config, so
# it is find-only. Absent it, the comparison benchmarks are omitted with a
# warning. ---
find_package(pinocchio CONFIG QUIET)
if (TARGET pinocchio::pinocchio)
    cartan_add_benchmark(fk_pinocchio_benchmarks fk_pinocchio_benchmarks.cpp)
    target_link_libraries(fk_pinocchio_benchmarks PRIVATE pinocchio::pinocchio)

    add_executable(perf_fk_cartan_ur3e perf_fk_cartan_ur3e.cpp)
    target_link_libraries(perf_fk_cartan_ur3e PRIVATE cartan::cartan orocos-kdl)
    target_compile_options(perf_fk_cartan_ur3e PRIVATE ${CARTAN_WARNING_FLAGS})

    add_executable(perf_fk_pinocchio_ur3e perf_fk_pinocchio_ur3e.cpp)
    target_link_libraries(perf_fk_pinocchio_ur3e PRIVATE cartan::cartan pinocchio::pinocchio orocos-kdl)
    target_compile_options(perf_fk_pinocchio_ur3e PRIVATE ${CARTAN_WARNING_FLAGS})

    add_executable(perf_fk_cartan_matrix_ur3e perf_fk_cartan_matrix_ur3e.cpp)
    target_link_libraries(perf_fk_cartan_matrix_ur3e PRIVATE cartan::cartan orocos-kdl)
    target_compile_options(perf_fk_cartan_matrix_ur3e PRIVATE ${CARTAN_WARNING_FLAGS})

    cartan_add_benchmark(ik_comparison_pinocchio_benchmarks ik_comparison_pinocchio_benchmarks.cpp)
    target_link_libraries(ik_comparison_pinocchio_benchmarks PRIVATE pinocchio::pinocchio)
    if (CARTAN_HAVE_TRAC_IK)
        target_link_libraries(ik_comparison_pinocchio_benchmarks PRIVATE trac_ik)
        target_compile_definitions(ik_comparison_pinocchio_benchmarks PRIVATE CARTAN_HAS_TRAC_IK)
    endif ()
    if (CARTAN_BUILD_ARGMIN)
        target_compile_definitions(ik_comparison_pinocchio_benchmarks PRIVATE CARTAN_BUILD_ARGMIN)
        target_link_libraries(ik_comparison_pinocchio_benchmarks PRIVATE argmin::argmin)
    endif ()

    cartan_add_benchmark(closed_form_ik_benchmarks closed_form_ik_benchmarks.cpp)
    target_link_libraries(closed_form_ik_benchmarks PRIVATE pinocchio::pinocchio)
    if (CARTAN_HAVE_TRAC_IK)
        target_link_libraries(closed_form_ik_benchmarks PRIVATE trac_ik)
        target_compile_definitions(closed_form_ik_benchmarks PRIVATE CARTAN_HAS_TRAC_IK)
    endif ()
    if (CARTAN_BUILD_ARGMIN)
        target_compile_definitions(closed_form_ik_benchmarks PRIVATE CARTAN_BUILD_ARGMIN)
        target_link_libraries(closed_form_ik_benchmarks PRIVATE argmin::argmin)
    endif ()
else ()
    message(WARNING
        "cartan benchmarks: pinocchio not found -- skipping Pinocchio comparison benchmarks. "
        "Install pinocchio to enable them.")
endif ()

# --- OPW closed-form comparison benchmark: opw_kinematics (ROS-Industrial,
# Apache-2.0) is an external reference implementation for the ortho-parallel 6R
# closed-form solve. Its own CMake calls
# find_package(ros_industrial_cmake_boilerplate REQUIRED) in an add_subdirectory
# that is unavailable here, so the source is populated WITHOUT running that
# add_subdirectory (SOURCE_SUBDIR points at a nonexistent path) and a manual
# header-only INTERFACE target is built instead. Obtained from source -- fetched
# when CARTAN_FETCH_BENCHMARK_DEPS is ON, or from a caller-provided
# CARTAN_OPW_KINEMATICS_SOURCE_DIR. Absent both, the comparison is omitted with a
# warning. The reference target links ONLY into this one benchmark executable --
# never into cartan::cartan, the include root, or any tests/ target. ---
set(CARTAN_HAVE_OPW_KINEMATICS FALSE)
if (CARTAN_OPW_KINEMATICS_SOURCE_DIR OR CARTAN_FETCH_BENCHMARK_DEPS)
    if (CARTAN_OPW_KINEMATICS_SOURCE_DIR)
        FetchContent_Declare(
            opw_kinematics
            SOURCE_DIR "${CARTAN_OPW_KINEMATICS_SOURCE_DIR}"
            SOURCE_SUBDIR does-not-exist
            EXCLUDE_FROM_ALL
            SYSTEM
        )
    else ()
        # Literal SHA pin (v0.5.5), never a moving tag.
        FetchContent_Declare(
            opw_kinematics
            GIT_REPOSITORY https://github.com/Jmeyer1292/opw_kinematics.git
            GIT_TAG 8a32bda8197c50bd0d60dfe1d12ecb4c13111b72
            SOURCE_SUBDIR does-not-exist
            EXCLUDE_FROM_ALL
            SYSTEM
        )
    endif ()
    FetchContent_MakeAvailable(opw_kinematics)
    find_package(Eigen3 CONFIG REQUIRED)
    add_library(opw_kinematics_reference INTERFACE)
    target_include_directories(opw_kinematics_reference SYSTEM INTERFACE
        "${opw_kinematics_SOURCE_DIR}/include")
    target_link_libraries(opw_kinematics_reference INTERFACE Eigen3::Eigen)
    set(CARTAN_HAVE_OPW_KINEMATICS TRUE)
else ()
    message(WARNING
        "cartan benchmarks: OPW comparison benchmark needs the opw_kinematics source -- skipping. "
        "Configure with -DCARTAN_FETCH_BENCHMARK_DEPS=ON or provide -DCARTAN_OPW_KINEMATICS_SOURCE_DIR=<path>.")
endif ()

if (CARTAN_HAVE_OPW_KINEMATICS)
    # Custom main() drives the branch-for-branch parity receipt (and returns
    # nonzero on any mismatch), then hands off to google-benchmark for the
    # solve-timing cells -- so link benchmark::benchmark WITHOUT benchmark_main.
    # No orocos-kdl: this TU uses the cartan screw model directly, not
    # benchmark_utils.h.
    add_executable(opw_comparison_benchmarks opw_comparison_benchmarks.cpp)
    target_link_libraries(opw_comparison_benchmarks
        PRIVATE
        cartan::cartan
        benchmark::benchmark
        opw_kinematics_reference
    )
    target_compile_options(opw_comparison_benchmarks PRIVATE ${CARTAN_WARNING_FLAGS})
endif ()

# --- IKFast comparison benchmark: an OpenRAVE-generated closed-form solver for
# the KR6 R900 (Transform6D). Unlike opw_kinematics, IKFast derives the inverse
# per-robot by algebraic elimination -- an INDEPENDENT formulation, so it agrees
# with cartan at ~machine precision rather than bit-for-bit. The generated solver
# is a VENDORED artifact (benchmarks/third_party/ikfast_kr6r900/), since codegen
# is a one-shot offline OpenRAVE build; provenance is pinned in its README. It is
# self-contained: only ikfast.h + LAPACK (it calls dgeev_ for polynomial roots).
# Compiled into a static library WITHOUT cartan's warning flags (generated code)
# and linked ONLY into this one benchmark. ---
find_package(LAPACK QUIET)
if (TARGET LAPACK::LAPACK OR LAPACK_FOUND)
    add_library(ikfast_kr6r900_reference STATIC
        third_party/ikfast_kr6r900/ikfast_kr6r900_transform6d.cpp)
    # IKFAST_NO_MAIN drops the generated CLI main(). The entry points keep
    # default (C++) linkage, matching ikfast.h's declarations in the benchmark.
    target_compile_definitions(ikfast_kr6r900_reference PRIVATE IKFAST_NO_MAIN)
    target_include_directories(ikfast_kr6r900_reference SYSTEM PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ikfast_kr6r900")
    if (TARGET LAPACK::LAPACK)
        target_link_libraries(ikfast_kr6r900_reference PUBLIC LAPACK::LAPACK)
    else ()
        target_link_libraries(ikfast_kr6r900_reference PUBLIC ${LAPACK_LIBRARIES})
    endif ()

    add_executable(ikfast_comparison_benchmarks ikfast_comparison_benchmarks.cpp)
    target_link_libraries(ikfast_comparison_benchmarks
        PRIVATE
        cartan::cartan
        benchmark::benchmark
        ikfast_kr6r900_reference
    )
    target_compile_options(ikfast_comparison_benchmarks PRIVATE ${CARTAN_WARNING_FLAGS})
else ()
    message(WARNING
        "cartan benchmarks: LAPACK not found -- skipping the IKFast comparison benchmark "
        "(the generated solver calls LAPACK for polynomial roots).")
endif ()

# --- ik-geo comparison benchmark: ik-geo (Elias & Wen, RPI) is an independent
# geometric subproblem-decomposition IK solver. Its C++ port's spherical solver
# is broken for roll-pitch-roll wrists, so it is driven through the ik-geo Rust
# crate (vendored under third_party/ikgeo_ffi/, BSD-3-Clause) via a small C FFI
# shim, built with cargo and linked as a static library into this one benchmark.
# Provide -DCARTAN_IKGEO_FFI_LIB=<path> to use a prebuilt shim instead of cargo. ---
if (NOT CARTAN_IKGEO_FFI_LIB)
    find_program(CARTAN_CARGO_EXECUTABLE cargo)
    if (CARTAN_CARGO_EXECUTABLE)
        set(_ikgeo_ffi_dir "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ikgeo_ffi")
        set(CARTAN_IKGEO_FFI_LIB "${_ikgeo_ffi_dir}/target/release/libikgeo_ffi.a")
        add_custom_command(
            OUTPUT "${CARTAN_IKGEO_FFI_LIB}"
            COMMAND "${CARTAN_CARGO_EXECUTABLE}" build --release
            WORKING_DIRECTORY "${_ikgeo_ffi_dir}"
            COMMENT "Building the ik-geo Rust FFI shim (cargo build --release)"
            VERBATIM)
        add_custom_target(ikgeo_ffi_build DEPENDS "${CARTAN_IKGEO_FFI_LIB}")
    endif ()
endif ()

if (CARTAN_IKGEO_FFI_LIB)
    add_executable(ikgeo_comparison_benchmarks ikgeo_comparison_benchmarks.cpp)
    if (TARGET ikgeo_ffi_build)
        add_dependencies(ikgeo_comparison_benchmarks ikgeo_ffi_build)
    endif ()
    target_link_libraries(ikgeo_comparison_benchmarks
        PRIVATE
        cartan::cartan
        benchmark::benchmark
        "${CARTAN_IKGEO_FFI_LIB}"
        pthread dl m
    )
    target_compile_options(ikgeo_comparison_benchmarks PRIVATE ${CARTAN_WARNING_FLAGS})
else ()
    message(WARNING
        "cartan benchmarks: ik-geo comparison benchmark needs cargo (to build the "
        "vendored Rust FFI shim) or a prebuilt -DCARTAN_IKGEO_FFI_LIB=<path> -- skipping.")
endif ()
