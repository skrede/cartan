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

# These two exercise the checked entry point only, so they hold in every
# configuration.
cartan_add_boundary_test(velocity_boundary_test velocity_boundary_test.cpp)
cartan_add_boundary_test(velocity_shape_boundary_test velocity_shape_boundary_test.cpp)

# Capability probes rather than a search of CMAKE_CXX_FLAGS: a flag can arrive
# through CMAKE_CXX_FLAGS_<CONFIG>, a toolchain file, add_compile_options() or a
# different spelling of the sanitizer list, and a substring test that misses it
# would drop everything below while ctest still reported success.
include(CheckCXXSourceCompiles)
include(CheckCXXSourceRuns)

# try_compile does not apply CMAKE_CXX_FLAGS_<CONFIG>, and -DNDEBUG lives there,
# so the per-configuration flags and the directory-level options are both handed
# to the probes explicitly or the answer is about a configuration nobody builds.
get_directory_property(cartan_boundary_directory_options COMPILE_OPTIONS)
list(JOIN cartan_boundary_directory_options " " cartan_boundary_extra_flags)
string(TOUPPER "${CMAKE_BUILD_TYPE}" cartan_boundary_config)
set(CMAKE_REQUIRED_FLAGS
    "${cartan_boundary_extra_flags} ${CMAKE_CXX_FLAGS_${cartan_boundary_config}}")

check_cxx_source_compiles("
#ifndef NDEBUG
#error assertions are live
#endif
int main() { return 0; }
" CARTAN_BOUNDARY_ASSERTIONS_OFF)

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
if (CARTAN_BOUNDARY_ASSERTIONS_OFF)
    cartan_add_boundary_test(velocity_unchecked_differential_test
        velocity_unchecked_differential_test.cpp)
endif ()

set(cartan_boundary_evidence_tests 0)

if (CARTAN_BOUNDARY_ADDRESS_SANITIZER)
    # A probe carries no test framework and exists solely to reproduce what the
    # shipped configuration does, so it compiles its own assertions out rather
    # than depending on the surrounding build type. That keeps the sanitizer
    # evidence runnable in an assertions-on sanitizer configuration, where Eigen
    # would otherwise abort on the dimension mismatch first.
    function(cartan_add_sanitizer_probe PROBE_NAME PROBE_SOURCE)
        add_executable(${PROBE_NAME} ${PROBE_SOURCE})
        target_link_libraries(${PROBE_NAME} PRIVATE cartan::cartan)
        target_compile_options(${PROBE_NAME} PRIVATE ${CARTAN_WARNING_FLAGS})
        target_compile_definitions(${PROBE_NAME} PRIVATE NDEBUG)
    endfunction()

    cartan_add_sanitizer_probe(velocity_unchecked_empty_probe
        velocity_unchecked_empty_probe.cpp)
    cartan_add_sanitizer_probe(velocity_unchecked_short_probe
        velocity_unchecked_short_probe.cpp)
    cartan_add_sanitizer_probe(velocity_unchecked_dq_probe
        velocity_unchecked_dq_probe.cpp)

    function(cartan_add_sanitizer_tripwire TEST_NAME PROBE_NAME WANT_ERROR WANT_FILE)
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

    # An in-bounds invocation of the same binary must be clean and exit zero, and
    # the assertion above must reject that same run. The first is what makes a
    # tripwire attributable to the violated precondition; the second is what keeps
    # the assertion from being satisfiable by anything at all.
    add_test(NAME velocity_unchecked_in_bounds_is_clean
        COMMAND "${CMAKE_COMMAND}"
            "-DPROBE=$<TARGET_FILE:velocity_unchecked_short_probe>"
            "-DPROBE_ARGS=6;double"
            -DEXPECT=clean
            -P "${CMAKE_CURRENT_SOURCE_DIR}/expect_sanitizer.cmake")

    add_test(NAME velocity_unchecked_assertion_rejects_a_clean_run
        COMMAND "${CMAKE_COMMAND}"
            "-DEXPECT_SCRIPT=${CMAKE_CURRENT_SOURCE_DIR}/expect_sanitizer.cmake"
            "-DPROBE=$<TARGET_FILE:velocity_unchecked_short_probe>"
            "-DPROBE_ARGS=6;double"
            "-DWANT_ERROR=heap-buffer-overflow"
            "-DWANT_FILE=forward_kinematics.h"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/expect_sanitizer_rejects.cmake")

    set(cartan_boundary_evidence_tests 11)
endif ()

# Registering nothing must not be able to pass. The guard is compiled in this
# configuration and reads NDEBUG and the sanitizer for itself, so a configuration
# that ought to carry the evidence but registered none of it fails here.
add_executable(boundary_evidence_guard boundary_evidence_guard.cpp)
target_compile_options(boundary_evidence_guard PRIVATE ${CARTAN_WARNING_FLAGS})
target_compile_definitions(boundary_evidence_guard PRIVATE
    CARTAN_REGISTERED_EVIDENCE_TESTS=${cartan_boundary_evidence_tests}
    CARTAN_EXPECTED_EVIDENCE_TESTS=11)
add_test(NAME boundary_evidence_registration COMMAND boundary_evidence_guard)
