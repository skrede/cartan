# Each header's declared entry-point count is asserted, not printed: a family
# that quietly shrinks is the same defect as one that quietly grows.
set(cartan_kinematics_headers "")
set(cartan_kinematics_entry_points "")
foreach (header IN ITEMS forward_kinematics:3 forward_kinematics_matrix:2
        jacobian:6 jacobian_matrix:2 velocity:1)
    string(REPLACE ":" ";" header_pair "${header}")
    list(GET header_pair 0 header_stem)
    list(GET header_pair 1 header_count)
    list(APPEND cartan_kinematics_headers
        "${CMAKE_SOURCE_DIR}/lib/cartan-serial-chain/include/cartan/serial/fk/${header_stem}.h")
    list(APPEND cartan_kinematics_entry_points "${header_count}")
endforeach ()

set(cartan_entry_point_roots "${CMAKE_SOURCE_DIR}/lib")
file(GLOB_RECURSE cartan_entry_point_watched "${cartan_entry_point_roots}/*.h")
set_property(DIRECTORY APPEND PROPERTY
    CMAKE_CONFIGURE_DEPENDS ${cartan_entry_point_watched})

execute_process(
    COMMAND "${CMAKE_COMMAND}"
            "-DHEADERS=${cartan_kinematics_headers}"
            "-DEXPECTED=${cartan_kinematics_entry_points}"
            "-DSEARCH_ROOTS=${cartan_entry_point_roots}"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/entry_point_gate.cmake"
    OUTPUT_VARIABLE cartan_entry_point_gate_stdout
    ERROR_VARIABLE cartan_entry_point_gate_stderr
    RESULT_VARIABLE cartan_entry_point_gate_result)

if (NOT cartan_entry_point_gate_result EQUAL 0)
    message(FATAL_ERROR
        "${cartan_entry_point_gate_stdout}${cartan_entry_point_gate_stderr}")
endif ()
string(REGEX REPLACE "^-- |[ \t\r\n]+$" ""
    cartan_entry_point_gate_summary "${cartan_entry_point_gate_stdout}")
message(STATUS "${cartan_entry_point_gate_summary}")

# The gate's own rejection is a test rather than a run recorded when it was
# written, so a scan that stops matching cannot report every header clean.
add_test(NAME entry_point_gate_rejects_an_unguarded_overload
    COMMAND "${CMAKE_COMMAND}"
        "-DGATE_SCRIPT=${CMAKE_CURRENT_SOURCE_DIR}/entry_point_gate.cmake"
        "-DSOURCE_HEADER=${CMAKE_SOURCE_DIR}/lib/cartan-serial-chain/include/cartan/serial/fk/jacobian.h"
        "-DSCRATCH_DIR=${CMAKE_CURRENT_BINARY_DIR}/entry_point_gate_scratch"
        -DEXPECTED=6
        -P "${CMAKE_CURRENT_SOURCE_DIR}/expect_gate_rejects.cmake")

function(cartan_add_boundary_test TEST_NAME TEST_SOURCE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_link_libraries(${TEST_NAME}
        PRIVATE
        cartan::cartan
        Catch2::Catch2WithMain
    )
    target_compile_options(${TEST_NAME} PRIVATE ${CARTAN_WARNING_FLAGS})
    if (CARTAN_HAS_WNO_C2Y_EXTENSIONS)
        target_compile_options(${TEST_NAME} PRIVATE -Wno-c2y-extensions)
    endif ()
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endfunction()

# These exercise the checked entry points only, so they hold in every
# configuration.
cartan_add_boundary_test(velocity_boundary_test velocity_boundary_test.cpp)
cartan_add_boundary_test(velocity_shape_boundary_test velocity_shape_boundary_test.cpp)
cartan_add_boundary_test(fk_boundary_test fk_boundary_test.cpp)
cartan_add_boundary_test(jacobian_boundary_test jacobian_boundary_test.cpp)
cartan_add_boundary_test(solver_setup_boundary_test solver_setup_boundary_test.cpp)
cartan_add_boundary_test(fixture_smoke_test fixture_smoke_test.cpp)

# One sweep of every checked factory against a NaN and both infinities. The
# boundaries it covers are each also covered by one of the targets above; what
# this one adds is that a boundary silently losing its guard is still caught if
# that target is later deleted or narrowed.
cartan_add_boundary_test(nonfinite_matrix_test nonfinite_matrix_test.cpp)

# The tag/axis and classification cells are wrong-value cells with no sanitizer
# signature, and their demonstrations construct no group element from nonfinite
# data, so nothing here depends on NDEBUG either.
cartan_add_boundary_test(axis_classification_test axis_classification_test.cpp)

# Capability probes rather than a search of CMAKE_CXX_FLAGS: a flag can arrive
# through CMAKE_CXX_FLAGS_<CONFIG>, a toolchain file, add_compile_options() or a
# different spelling of the sanitizer list, and a substring test that misses it
# would drop everything below while ctest still reported success.
include(CheckCXXSourceCompiles)
include(CheckCXXSourceRuns)

# try_compile does not apply CMAKE_CXX_FLAGS_<CONFIG>, and the sanitizer switches
# can live there, so the per-configuration flags and the directory-level options
# are both handed to the probe explicitly or the answer is about a configuration
# nobody builds.
get_directory_property(cartan_boundary_directory_options COMPILE_OPTIONS)
list(JOIN cartan_boundary_directory_options " " cartan_boundary_extra_flags)
string(TOUPPER "${CMAKE_BUILD_TYPE}" cartan_boundary_config)
set(CMAKE_REQUIRED_FLAGS
    "${cartan_boundary_extra_flags} ${CMAKE_CXX_FLAGS_${cartan_boundary_config}}")

check_cxx_source_runs("
#if defined(__SANITIZE_ADDRESS__)
#elif defined(__has_feature)
#  if !__has_feature(address_sanitizer)
#    error no address sanitizer
#  endif
#else
#  error no address sanitizer
#endif
int main() { volatile int probe = 0; return probe; }
" CARTAN_BOUNDARY_ADDRESS_SANITIZER)

unset(CMAKE_REQUIRED_FLAGS)

# Anything that calls an _unchecked entry point with a violated precondition
# needs assertions compiled out. Eigen asserts on the dimension mismatch and
# cartan asserts on a nonfinite quaternion, and either aborts the process before
# the behavior under demonstration can be observed.
#
# Each therefore compiles its own assertions out rather than depending on the
# surrounding build type, for the reason the sanitizer probes below do: a
# configure-time answer about the build type is not one a multi-configuration
# generator has, which left this whole group unregistered under Visual Studio
# while the guard was compiled with NDEBUG and accounted for it. Depending on the
# build type also silently dropped the group from every Debug build. The library
# is header-only, so no other translation unit in these binaries sees the
# definition and there is nothing for it to disagree with.
foreach (differential IN ITEMS velocity_unchecked_differential_test
        fk_unchecked_differential_test jacobian_unchecked_differential_test
        solver_setup_unchecked_differential_test)
    cartan_add_boundary_test(${differential} ${differential}.cpp)
    target_compile_definitions(${differential} PRIVATE NDEBUG _GLIBCXX_NO_ASSERTIONS)
    set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND
        PROPERTY cartan_boundary_differential "${differential}")
endforeach ()

if (CARTAN_BOUNDARY_ADDRESS_SANITIZER)
    # A probe carries no test framework and exists solely to reproduce what the
    # shipped configuration does, so it compiles its own assertions out rather
    # than depending on the surrounding build type. That keeps the sanitizer
    # evidence runnable in an assertions-on sanitizer configuration, where Eigen
    # would otherwise abort on the dimension mismatch first.
    #
    # The standard library's own container checks need the same treatment and are
    # a separate switch. libstdc++ turns them on for any unoptimized build
    # (c++config.h: `! defined(__OPTIMIZE__) && ! defined(_GLIBCXX_NO_ASSERTIONS)`)
    # regardless of NDEBUG, and a bounds assert inside vector::operator[] aborts
    # before the sanitizer sees the over-read: measured in a Debug ASan/UBSan
    # configure, where the cached-result over-read reported
    # `Assertion '__n < this->size()' failed` from stl_vector.h and named no
    # cartan file at all.
    function(cartan_add_sanitizer_probe PROBE_NAME PROBE_SOURCE)
        add_executable(${PROBE_NAME} ${PROBE_SOURCE})
        target_link_libraries(${PROBE_NAME} PRIVATE cartan::cartan)
        target_compile_options(${PROBE_NAME} PRIVATE ${CARTAN_WARNING_FLAGS})
        target_compile_definitions(${PROBE_NAME} PRIVATE NDEBUG _GLIBCXX_NO_ASSERTIONS)
    endfunction()

    cartan_add_sanitizer_probe(velocity_unchecked_empty_probe
        velocity_unchecked_empty_probe.cpp)
    cartan_add_sanitizer_probe(velocity_unchecked_short_probe
        velocity_unchecked_short_probe.cpp)
    cartan_add_sanitizer_probe(velocity_unchecked_dq_probe
        velocity_unchecked_dq_probe.cpp)
    cartan_add_sanitizer_probe(fk_unchecked_empty_probe
        fk_unchecked_empty_probe.cpp)
    cartan_add_sanitizer_probe(fk_unchecked_short_probe
        fk_unchecked_short_probe.cpp)
    cartan_add_sanitizer_probe(jacobian_unchecked_short_fk_probe
        jacobian_unchecked_short_fk_probe.cpp)
    cartan_add_sanitizer_probe(fk_unchecked_fixed_short_probe
        fk_unchecked_fixed_short_probe.cpp)

    function(cartan_add_sanitizer_tripwire TEST_NAME PROBE_NAME WANT_ERROR WANT_FILE)
        set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND
            PROPERTY cartan_boundary_evidence "${TEST_NAME}")
        add_test(NAME ${TEST_NAME}
            COMMAND "${CMAKE_COMMAND}"
                "-DPROBE=$<TARGET_FILE:${PROBE_NAME}>"
                "-DPROBE_ARGS=${ARGN}"
                -DEXPECT=report
                "-DWANT_ERROR=${WANT_ERROR}"
                "-DWANT_FILE=${WANT_FILE}"
                -P "${CMAKE_CURRENT_SOURCE_DIR}/expect_sanitizer.cmake")
    endfunction()

    # An empty joint vector binds a reference to null before it reaches an
    # out-of-bounds read, so where -fno-sanitize-recover=undefined is in force the
    # report is the UndefinedBehaviorSanitizer diagnostic rather than the
    # segmentation fault that follows when the violation is recoverable.
    cartan_add_sanitizer_tripwire(velocity_unchecked_empty_q_trips_sanitizer_in_double
        velocity_unchecked_empty_probe
        "reference binding to null pointer" "forward_kinematics.h" double)
    cartan_add_sanitizer_tripwire(velocity_unchecked_empty_q_trips_sanitizer_in_float
        velocity_unchecked_empty_probe
        "reference binding to null pointer" "forward_kinematics.h" float)

    cartan_add_sanitizer_tripwire(velocity_unchecked_short_q_trips_sanitizer_in_double
        velocity_unchecked_short_probe
        "heap-buffer-overflow" "forward_kinematics.h" 5 double)
    cartan_add_sanitizer_tripwire(velocity_unchecked_short_q_trips_sanitizer_in_float
        velocity_unchecked_short_probe
        "heap-buffer-overflow" "forward_kinematics.h" 5 float)

    # The joint-velocity vector is consumed by the Jacobian product, so its length
    # cases fault inside Eigen and the trace names velocity.h rather than the
    # accumulation loop's header. The over-long case faults in float only; in
    # double it is a silent truncation, demonstrated by assertion instead.
    cartan_add_sanitizer_tripwire(velocity_unchecked_empty_dq_trips_sanitizer_in_double
        velocity_unchecked_dq_probe
        "reference binding to null pointer" "velocity.h" 0 double)
    cartan_add_sanitizer_tripwire(velocity_unchecked_empty_dq_trips_sanitizer_in_float
        velocity_unchecked_dq_probe
        "reference binding to null pointer" "velocity.h" 0 float)

    cartan_add_sanitizer_tripwire(velocity_unchecked_short_dq_trips_sanitizer_in_double
        velocity_unchecked_dq_probe
        "heap-buffer-overflow" "velocity.h" 5 double)
    cartan_add_sanitizer_tripwire(velocity_unchecked_short_dq_trips_sanitizer_in_float
        velocity_unchecked_dq_probe
        "heap-buffer-overflow" "velocity.h" 5 float)

    cartan_add_sanitizer_tripwire(velocity_unchecked_long_dq_trips_sanitizer_in_float
        velocity_unchecked_dq_probe
        "heap-buffer-overflow" "velocity.h" 7 float)

    # The same two joint-vector cells reached through the forward-kinematics
    # entry point itself rather than through the velocity path above, so the
    # report is attributable to that boundary and survives a later change to
    # what the velocity path calls.
    cartan_add_sanitizer_tripwire(fk_unchecked_empty_q_trips_sanitizer_in_double
        fk_unchecked_empty_probe
        "reference binding to null pointer" "forward_kinematics.h" double)
    cartan_add_sanitizer_tripwire(fk_unchecked_empty_q_trips_sanitizer_in_float
        fk_unchecked_empty_probe
        "reference binding to null pointer" "forward_kinematics.h" float)

    cartan_add_sanitizer_tripwire(fk_unchecked_short_q_trips_sanitizer_in_double
        fk_unchecked_short_probe
        "heap-buffer-overflow" "forward_kinematics.h" 5 double)
    cartan_add_sanitizer_tripwire(fk_unchecked_short_q_trips_sanitizer_in_float
        fk_unchecked_short_probe
        "heap-buffer-overflow" "forward_kinematics.h" 5 float)

    # The Jacobian family's own hole, which no joint-vector case reaches: a
    # cached result holding fewer intermediates than the chain has joints is
    # read past its end while building the columns.
    cartan_add_sanitizer_tripwire(jacobian_unchecked_short_fk_trips_sanitizer_in_double
        jacobian_unchecked_short_fk_probe
        "heap-buffer-overflow" "jacobian.h" 3 double)
    cartan_add_sanitizer_tripwire(jacobian_unchecked_short_fk_trips_sanitizer_in_float
        jacobian_unchecked_short_fk_probe
        "heap-buffer-overflow" "jacobian.h" 3 float)

    # The unchecked siblings still spell their joint vector as the chain's
    # fixed-size position_type, so an ill-sized argument is converted in the
    # caller's frame and over-reads before the callee is entered. The report
    # therefore names the calling file, not a cartan header — which is exactly
    # why the checked entry points take the vector at the caller's own type.
    cartan_add_sanitizer_tripwire(fk_unchecked_fixed_short_q_trips_sanitizer_in_double
        fk_unchecked_fixed_short_probe
        "heap-buffer-overflow" "fk_unchecked_fixed_short_probe.cpp" 5 double)
    cartan_add_sanitizer_tripwire(fk_unchecked_fixed_short_q_trips_sanitizer_in_float
        fk_unchecked_fixed_short_probe
        "heap-buffer-overflow" "fk_unchecked_fixed_short_probe.cpp" 5 float)

    # An in-bounds invocation of the same binary must be clean and exit zero, and
    # the assertion above must reject that same run. The first is what makes a
    # tripwire attributable to the violated precondition; the second is what keeps
    # the assertion from being satisfiable by anything at all.
    set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND
        PROPERTY cartan_boundary_evidence "velocity_unchecked_in_bounds_is_clean")
    add_test(NAME velocity_unchecked_in_bounds_is_clean
        COMMAND "${CMAKE_COMMAND}"
            "-DPROBE=$<TARGET_FILE:velocity_unchecked_short_probe>"
            "-DPROBE_ARGS=6;double"
            -DEXPECT=clean
            -P "${CMAKE_CURRENT_SOURCE_DIR}/expect_sanitizer.cmake")

    set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND
        PROPERTY cartan_boundary_evidence "fk_unchecked_in_bounds_is_clean")
    add_test(NAME fk_unchecked_in_bounds_is_clean
        COMMAND "${CMAKE_COMMAND}"
            "-DPROBE=$<TARGET_FILE:fk_unchecked_short_probe>"
            "-DPROBE_ARGS=6;double"
            -DEXPECT=clean
            -P "${CMAKE_CURRENT_SOURCE_DIR}/expect_sanitizer.cmake")

    set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND
        PROPERTY cartan_boundary_evidence "jacobian_unchecked_matching_result_is_clean")
    add_test(NAME jacobian_unchecked_matching_result_is_clean
        COMMAND "${CMAKE_COMMAND}"
            "-DPROBE=$<TARGET_FILE:jacobian_unchecked_short_fk_probe>"
            "-DPROBE_ARGS=6;double"
            -DEXPECT=clean
            -P "${CMAKE_CURRENT_SOURCE_DIR}/expect_sanitizer.cmake")

    set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" APPEND
        PROPERTY cartan_boundary_evidence "velocity_unchecked_assertion_rejects_a_clean_run")
    add_test(NAME velocity_unchecked_assertion_rejects_a_clean_run
        COMMAND "${CMAKE_COMMAND}"
            "-DEXPECT_SCRIPT=${CMAKE_CURRENT_SOURCE_DIR}/expect_sanitizer.cmake"
            "-DPROBE=$<TARGET_FILE:velocity_unchecked_short_probe>"
            "-DPROBE_ARGS=6;double"
            "-DWANT_ERROR=heap-buffer-overflow"
            "-DWANT_FILE=forward_kinematics.h"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/expect_sanitizer_rejects.cmake")

endif ()

get_property(cartan_boundary_evidence_names
    DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY cartan_boundary_evidence)
list(LENGTH cartan_boundary_evidence_names cartan_boundary_evidence_tests)
get_property(cartan_boundary_differential_names
    DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY cartan_boundary_differential)
list(LENGTH cartan_boundary_differential_names cartan_boundary_differential_tests)

# Registering nothing must not be able to pass, and neither must registering
# some. Both counts are derived from the registrations themselves and compared
# against a declared expectation, so deleting one case is caught as well as
# deleting the group. The guard is compiled in this configuration and reads
# NDEBUG and the sanitizer for itself.
add_executable(boundary_evidence_guard boundary_evidence_guard.cpp)
target_compile_options(boundary_evidence_guard PRIVATE ${CARTAN_WARNING_FLAGS})
target_compile_definitions(boundary_evidence_guard PRIVATE
    CARTAN_REGISTERED_EVIDENCE_TESTS=${cartan_boundary_evidence_tests}
    CARTAN_EXPECTED_EVIDENCE_TESTS=21
    CARTAN_REGISTERED_DIFFERENTIAL_TESTS=${cartan_boundary_differential_tests}
    CARTAN_EXPECTED_DIFFERENTIAL_TESTS=4)
add_test(NAME boundary_evidence_registration COMMAND boundary_evidence_guard)
