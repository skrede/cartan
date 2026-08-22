cmake_language(DEFER CALL cartan_warn_directory_targets)

# Measured-region purity gate.
#
# A checked kinematics entry point inside a `for (auto _ : state)` block times
# the guard along with the kinematics and moves a published number under an
# unchanged benchmark name. Nothing downstream can see that: it compiles, it
# runs, and it reports. The check therefore runs here, at configure time, and is
# placed before the dependency detection below so a missing third-party library
# cannot take the whole file's early return past it. The fixture and support
# headers are scanned as well, because the leak can be indirect: a helper that
# calls a checked entry point puts the same guard inside every measured block
# that calls the helper, and no benchmark source names an entry point at all.
#
# Turning the benchmark suite on therefore now requires a Python 3 interpreter.
find_package(Python3 REQUIRED COMPONENTS Interpreter)

execute_process(
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/check_measured_regions.py
            --benchmarks-root ${CMAKE_CURRENT_SOURCE_DIR}
            --reach-root ${CMAKE_SOURCE_DIR}/tests/fixtures
            --reach-root ${CMAKE_SOURCE_DIR}/tests/support
            --min-sources 20
    RESULT_VARIABLE _measured_region_result
    OUTPUT_VARIABLE _measured_region_output
    ERROR_VARIABLE _measured_region_error)

# Exit 1 is the tool's finding about the sources; anything else nonzero means the
# tool did not run, which must not be reported as if it had found a violation.
if (_measured_region_result EQUAL 1)
    message(FATAL_ERROR
        "cartan benchmarks: a measured block reaches a checked kinematics entry "
        "point. Call the _unchecked sibling there and establish the precondition "
        "outside the block.\n${_measured_region_output}${_measured_region_error}")
elseif (NOT _measured_region_result EQUAL 0)
    message(FATAL_ERROR
        "cartan benchmarks: the measured-region check did not run to completion "
        "(exit ${_measured_region_result}). This is a failure of the check, not a "
        "finding about the sources.\n${_measured_region_output}${_measured_region_error}")
endif ()

message(STATUS "cartan benchmarks: ${_measured_region_output}")

file(GLOB _measured_region_sources CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/*.h
    ${CMAKE_SOURCE_DIR}/tests/fixtures/*.h ${CMAKE_SOURCE_DIR}/tests/support/*.h)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/tools/check_measured_regions.py ${_measured_region_sources})

# Manufacturer joint limits are the one dependency of this suite that is NOT
# find-then-fetch-then-omit-with-a-warning. Every other block below degrades to
# a skipped benchmark; this one refuses the configure, because a study whose
# feasible set silently fell back to a synthetic box is worse than no study --
# it publishes numbers measured on a different problem under the same names.
# The gate sits ahead of the dependency detection so a missing comparator cannot
# take an early return past it.
if (NOT CARTAN_BUILD_URDF)
    message(FATAL_ERROR
        "cartan benchmarks: CARTAN_BUILD_URDF is OFF, so no robot description can be read and "
        "the suite has no source for manufacturer joint limits. Configure with "
        "-DCARTAN_BUILD_URDF=ON.")
endif ()

include(${CMAKE_SOURCE_DIR}/cmake/CartanBenchRuntimePath.cmake)
cartan_bench_prefer_toolchain_runtime()

include(${CMAKE_SOURCE_DIR}/cmake/CartanBenchDescriptions.cmake)

# The limit source's own correctness gate. It measures nothing and needs neither
# a timing harness nor a comparator, so it is defined ahead of the detection
# below and survives a configure in which every comparison benchmark is omitted.
add_executable(study_description_gate study_description_gate.cpp)
target_link_libraries(study_description_gate PRIVATE cartan::cartan)
target_compile_definitions(study_description_gate PRIVATE
    CARTAN_BENCH_DESCRIPTION_ROOT="${CARTAN_BENCH_DESCRIPTION_ROOT}"
    CARTAN_URDF_EXTENDED_TESTS)
add_test(NAME study_description_gate COMMAND study_description_gate)

# The measurement harness is the one third-party dependency this build may still
# download: it produces the measurement rather than participating in it, and a
# developer who has it packaged is served by the package lookup that runs first.
# It gates nothing else. Comparators are never fetched -- see the acquisition
# module for why -- and the two owned libraries have their own find-then-fetch-at-
# an-immutable-commit acquisition, which this option has never covered.
option(CARTAN_FETCH_BENCHMARK_DEPS
    "Fetch the measurement harness (Google Benchmark) when it is not installed. It gates the harness and nothing else; no comparator is ever fetched"
    OFF)

# Every dependency this suite links is resolved by the acquisition module, which
# owns the policy: comparators are discovered on the machine and never fetched,
# the two owned libraries are fetched at a verified immutable commit, and the
# generated per-robot solver stays in the tree. This file decides which targets
# exist; the module decides where each dependency came from and records it. This
# is still the only place third-party domain libraries are introduced,
# referenced, or linked -- tests/ stay cartan-only.
include(${CMAKE_SOURCE_DIR}/cmake/CartanBenchComparators.cmake)

# The harness every benchmark target needs, and the shared chain factories every
# benchmark source reaches through benchmark_utils.h. Absent either, there is no
# benchmark to build.
if (NOT CARTAN_BENCH_BENCHMARK_FOUND OR NOT CARTAN_BENCH_OROCOS_KDL_FOUND)
    message(WARNING "cartan benchmarks: skipping all benchmarks -- see the status lines above.")
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

# BOBYQA/SLSQP benchmarks measure the NLopt solve policy, which ships as an
# example rather than as library surface; the target carrying it is defined only
# when that example's own option is on.
if (TARGET cartan_examples::nlopt_policy)
    cartan_add_benchmark(basic_ik_sqp_benchmarks basic_ik_sqp_benchmarks.cpp)
    target_link_libraries(basic_ik_sqp_benchmarks PRIVATE cartan_examples::nlopt_policy)
endif()

set(CARTAN_HAVE_TRAC_IK ${CARTAN_BENCH_TRAC_IK_FOUND})
if (CARTAN_HAVE_TRAC_IK)
    cartan_add_benchmark(basic_ik_full_benchmarks basic_ik_full_benchmarks.cpp)
    target_link_libraries(basic_ik_full_benchmarks PRIVATE TracIK::trac_ik)
    if(CARTAN_BUILD_ARGMIN)
        target_link_libraries(basic_ik_full_benchmarks PRIVATE cartan::argmin)
    endif()
    if(TARGET cartan_examples::nlopt_policy)
        target_link_libraries(basic_ik_full_benchmarks PRIVATE cartan_examples::nlopt_policy)
    endif()

    if(CARTAN_BUILD_ARGMIN)
        add_executable(slsqp_per_pose_capture slsqp_per_pose_capture.cpp)
        target_link_libraries(slsqp_per_pose_capture
            PRIVATE
            cartan::cartan
            cartan::argmin)

        add_executable(slsqp_per_pose_trace slsqp_per_pose_trace.cpp)
        target_link_libraries(slsqp_per_pose_trace
            PRIVATE
            cartan::cartan
            cartan::argmin)
    endif()

    if(TARGET cartan_examples::nlopt_policy)
        add_executable(nlopt_slsqp_per_pose_capture nlopt_slsqp_per_pose_capture.cpp)
        target_link_libraries(nlopt_slsqp_per_pose_capture
            PRIVATE
            cartan::cartan
            cartan_examples::nlopt_policy)
    endif()

    cartan_add_benchmark(fk_comparison_benchmarks fk_comparison_benchmarks.cpp)
    target_link_libraries(fk_comparison_benchmarks PRIVATE TracIK::trac_ik)

    cartan_add_benchmark(jacobian_comparison_benchmarks jacobian_comparison_benchmarks.cpp)
    target_link_libraries(jacobian_comparison_benchmarks PRIVATE TracIK::trac_ik)
endif ()

if (CARTAN_BENCH_PINOCCHIO_FOUND)
    cartan_add_benchmark(fk_pinocchio_benchmarks fk_pinocchio_benchmarks.cpp)
    target_link_libraries(fk_pinocchio_benchmarks PRIVATE pinocchio::pinocchio)

    add_executable(perf_fk_cartan_ur3e perf_fk_cartan_ur3e.cpp)
    target_link_libraries(perf_fk_cartan_ur3e PRIVATE cartan::cartan orocos-kdl)

    add_executable(perf_fk_pinocchio_ur3e perf_fk_pinocchio_ur3e.cpp)
    target_link_libraries(perf_fk_pinocchio_ur3e PRIVATE cartan::cartan pinocchio::pinocchio orocos-kdl)

    add_executable(perf_fk_cartan_matrix_ur3e perf_fk_cartan_matrix_ur3e.cpp)
    target_link_libraries(perf_fk_cartan_matrix_ur3e PRIVATE cartan::cartan orocos-kdl)

    cartan_add_benchmark(ik_comparison_pinocchio_benchmarks ik_comparison_pinocchio_benchmarks.cpp)
    target_link_libraries(ik_comparison_pinocchio_benchmarks PRIVATE pinocchio::pinocchio)
    if (CARTAN_HAVE_TRAC_IK)
        target_link_libraries(ik_comparison_pinocchio_benchmarks PRIVATE TracIK::trac_ik)
        target_compile_definitions(ik_comparison_pinocchio_benchmarks PRIVATE CARTAN_BENCH_HAS_TRAC_IK)
    endif ()
    if (CARTAN_BUILD_ARGMIN)
        target_link_libraries(ik_comparison_pinocchio_benchmarks PRIVATE cartan::argmin)
    endif ()

    cartan_add_benchmark(closed_form_ik_benchmarks closed_form_ik_benchmarks.cpp)
    target_link_libraries(closed_form_ik_benchmarks PRIVATE pinocchio::pinocchio)
    if (CARTAN_HAVE_TRAC_IK)
        target_link_libraries(closed_form_ik_benchmarks PRIVATE TracIK::trac_ik)
        target_compile_definitions(closed_form_ik_benchmarks PRIVATE CARTAN_BENCH_HAS_TRAC_IK)
    endif ()
    if (CARTAN_BUILD_ARGMIN)
        target_link_libraries(closed_form_ik_benchmarks PRIVATE cartan::argmin)
    endif ()
endif ()

# --- The study's two passes over one target pool. The timed program registers
# cells and links the harness's default main; the untimed capture program owns
# its own main and carries every counter, adjudication and file write, which is
# how those stay out of a measured block. Both are defined whether or not the
# peer comparator is present: a build that resolves fewer participants than the
# capture declares must say so at run start rather than emit a smaller table. ---
add_executable(ik_study_benchmarks ik_study_benchmarks.cpp)
target_link_libraries(ik_study_benchmarks PRIVATE benchmark::benchmark_main)

add_executable(ik_study_capture ik_study_capture.cpp)

# The accuracy mode's tolerances are measured before the study runs, by a third
# program with its own main and no timing harness at all: an offline search is
# re-runnable evidence a reader can inspect, and it keeps a search loop out of
# every pass that reports a number.
add_executable(ik_study_calibrate ik_study_calibrate.cpp)

cartan_bench_join("${CARTAN_BENCH_DEPENDENCY_RECORDS}" CARTAN_BENCH_DEPENDENCY_MANIFEST)
cartan_bench_join("${CARTAN_BENCH_ABSENT_COMPARATORS}" CARTAN_BENCH_ABSENCE_MANIFEST)
cartan_bench_join("${CARTAN_BENCH_DESCRIPTION_PINS}" CARTAN_BENCH_DESCRIPTION_MANIFEST)
cartan_bench_configure_command(CARTAN_BENCH_CONFIGURE_COMMAND)

foreach (_study_program ik_study_benchmarks ik_study_capture ik_study_calibrate)
    target_link_libraries(${_study_program}
        PRIVATE
        cartan::cartan
        benchmark::benchmark
        orocos-kdl
    )
    target_compile_definitions(${_study_program} PRIVATE
        CARTAN_BENCH_DESCRIPTION_ROOT="${CARTAN_BENCH_DESCRIPTION_ROOT}"
        CARTAN_URDF_EXTENDED_TESTS
        CARTAN_BENCH_DEPENDENCY_MANIFEST="${CARTAN_BENCH_DEPENDENCY_MANIFEST}"
        CARTAN_BENCH_ABSENCE_MANIFEST="${CARTAN_BENCH_ABSENCE_MANIFEST}"
        CARTAN_BENCH_DESCRIPTION_MANIFEST="${CARTAN_BENCH_DESCRIPTION_MANIFEST}"
        CARTAN_BENCH_CONFIGURE_COMMAND="${CARTAN_BENCH_CONFIGURE_COMMAND}"
        CARTAN_BENCH_COMPILER="${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
    if (CARTAN_BENCH_PINOCCHIO_FOUND)
        target_link_libraries(${_study_program} PRIVATE pinocchio::pinocchio)
        target_compile_definitions(${_study_program} PRIVATE
            CARTAN_BENCH_STUDY_HAS_PINOCCHIO
            CARTAN_BENCH_KERNEL_COUNTABLE_PINOCCHIO=1)
    endif ()
    if (CARTAN_BENCH_TRAC_IK_FOUND)
        target_link_libraries(${_study_program} PRIVATE TracIK::trac_ik)
        target_compile_definitions(${_study_program} PRIVATE
            CARTAN_BENCH_STUDY_HAS_TRAC_IK
            CARTAN_BENCH_KERNEL_COUNTABLE_TRAC_IK=0)
    endif ()
endforeach ()

# --- OPW closed-form comparison benchmark. The reference is consumed as headers
# only -- its own CMake calls find_package(ros_industrial_cmake_boilerplate) in an
# add_subdirectory that expects a ROS workspace -- and the interface target the
# acquisition module builds for it links ONLY into this one benchmark executable,
# never into cartan::cartan, the include root, or any tests/ target. ---
if (CARTAN_BENCH_OPW_KINEMATICS_FOUND)
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
endif ()

# --- IKFast comparison benchmark: an OpenRAVE-generated closed-form solver for
# the KR6 R900 (Transform6D). Unlike opw_kinematics, IKFast derives the inverse
# per-robot by algebraic elimination -- an INDEPENDENT formulation, so it agrees
# with cartan at ~machine precision rather than bit-for-bit. The generated solver
# is a VENDORED artifact (benchmarks/third_party/ikfast_kr6r900/), since codegen
# is a one-shot offline OpenRAVE build; provenance is pinned in its README. It is
# self-contained: only ikfast.h + LAPACK (it calls dgeev_ for polynomial roots).
# Compiled into a static library linked ONLY into this one benchmark. ---
if (CARTAN_BENCH_LAPACK_FOUND)
    add_library(ikfast_kr6r900_reference STATIC
        third_party/ikfast_kr6r900/ikfast_kr6r900_transform6d.cpp)
    # Machine-emitted source nobody edits, so a diagnostic cartan's warning set
    # raises in it has no author to act on.
    cartan_exclude_from_warnings(ikfast_kr6r900_reference)
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
endif ()

# --- ik-geo comparison benchmark: an independent geometric
# subproblem-decomposition solver. Its C++ port's spherical solver is broken for
# roll-pitch-roll wrists, so the benchmark drives the Rust crate through this
# project's own C foreign-function shim; the crate itself is an ordinary registry
# dependency the shim's Cargo.lock fixes, downloaded by cargo and not by this
# build. Provide -DCARTAN_IKGEO_FFI_LIB=<path> to link a prebuilt shim instead. ---
if (CARTAN_BENCH_IK_GEO_FOUND AND NOT CARTAN_IKGEO_FFI_LIB)
    set(_ikgeo_ffi_dir "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ikgeo_ffi")
    set(CARTAN_IKGEO_FFI_LIB "${_ikgeo_ffi_dir}/target/release/libikgeo_ffi.a")
    # The crate's own optimizer dependency vendors a CMake project whose declared
    # minimum predates 4.0, which refuses it outright. The floor is raised for
    # that build's environment only; no library is modified.
    add_custom_command(
        OUTPUT "${CARTAN_IKGEO_FFI_LIB}"
        COMMAND "${CMAKE_COMMAND}" -E env CMAKE_POLICY_VERSION_MINIMUM=3.5
                "${CARTAN_CARGO_EXECUTABLE}" build --release --locked
        WORKING_DIRECTORY "${_ikgeo_ffi_dir}"
        COMMENT "Building the ik-geo foreign-function shim (cargo build --release --locked)"
        VERBATIM)
    add_custom_target(ikgeo_ffi_build DEPENDS "${CARTAN_IKGEO_FFI_LIB}")
endif ()

if (CARTAN_BENCH_IK_GEO_FOUND)
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
endif ()
