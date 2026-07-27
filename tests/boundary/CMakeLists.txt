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

cartan_add_boundary_test(velocity_boundary_test velocity_boundary_test.cpp)
cartan_add_boundary_test(velocity_shape_boundary_test velocity_shape_boundary_test.cpp)

# Only the under-length cases of this boundary are visible to a sanitizer; the
# over-length and nonfinite cases are silent wrong answers and are demonstrated
# by assertion in velocity_boundary_test instead. The probes below are built and
# registered only where the address sanitizer is actually configured, so the
# ordinary Release and Debug lanes are left untouched.
string(FIND "${CMAKE_CXX_FLAGS}" "fsanitize=address" cartan_asan_flag_pos)
if (NOT cartan_asan_flag_pos EQUAL -1)
    function(cartan_add_sanitizer_probe PROBE_NAME PROBE_SOURCE)
        add_executable(${PROBE_NAME} ${PROBE_SOURCE})
        target_link_libraries(${PROBE_NAME} PRIVATE cartan::cartan)
        target_compile_options(${PROBE_NAME} PRIVATE ${CARTAN_WARNING_FLAGS})
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
                "-DWANT_ERROR=${WANT_ERROR}"
                "-DWANT_FILE=${WANT_FILE}"
                -P "${CMAKE_CURRENT_SOURCE_DIR}/expect_sanitizer.cmake")
    endfunction()

    # An empty joint vector binds a reference to null before it reaches an
    # out-of-bounds read, and -fno-sanitize-recover=undefined halts the process
    # there, so the report is the UndefinedBehaviorSanitizer diagnostic rather
    # than the segmentation fault that follows when the violation is recoverable.
    cartan_add_sanitizer_tripwire(velocity_unchecked_empty_trips_sanitizer
        velocity_unchecked_empty_probe
        "reference binding to null pointer" "forward_kinematics.h")

    cartan_add_sanitizer_tripwire(velocity_unchecked_short_trips_sanitizer
        velocity_unchecked_short_probe
        "heap-buffer-overflow" "forward_kinematics.h")

    # The joint-velocity vector is consumed by the Jacobian product, so its
    # length cases fault inside Eigen and name velocity.h in the trace rather
    # than the accumulation loop's header. The over-long case is a fault only in
    # float: the double evaluator reads no further than the Jacobian's columns.
    cartan_add_sanitizer_tripwire(velocity_unchecked_empty_dq_trips_sanitizer
        velocity_unchecked_dq_probe
        "reference binding to null pointer" "velocity.h" 0)
    cartan_add_sanitizer_tripwire(velocity_unchecked_short_dq_trips_sanitizer
        velocity_unchecked_dq_probe
        "heap-buffer-overflow" "velocity.h" 5)
    cartan_add_sanitizer_tripwire(velocity_unchecked_long_dq_trips_sanitizer_in_float
        velocity_unchecked_dq_probe
        "heap-buffer-overflow" "velocity.h" 7 float)

    # Negative control: the same assertion against an in-bounds invocation of
    # the same probe must fail, so the wrapper cannot be vacuously green.
    cartan_add_sanitizer_tripwire(velocity_unchecked_wrapper_negative_control
        velocity_unchecked_short_probe
        "heap-buffer-overflow" "forward_kinematics.h" 6)
    set_tests_properties(velocity_unchecked_wrapper_negative_control
        PROPERTIES WILL_FAIL TRUE)
endif ()
