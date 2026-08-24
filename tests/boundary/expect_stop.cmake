# Runs a probe and asserts how it ended. With EXPECT=stop the process must not
# reach its return statement; with EXPECT=clean it must run to completion and
# exit zero. The stop emits no diagnostic at all, so the result code is the
# whole signal and there is nothing here to match output against. A CMake script
# rather than a shell wrapper so the mechanism runs on macOS and Windows as well
# as Linux, where the terminations differ but "did not exit zero" does not.

if (NOT DEFINED PROBE OR NOT DEFINED EXPECT)
    message(FATAL_ERROR "expect_stop.cmake requires -DPROBE and -DEXPECT")
endif ()

execute_process(
    COMMAND "${PROBE}" ${PROBE_ARGS}
    OUTPUT_VARIABLE probe_stdout
    ERROR_VARIABLE probe_stderr
    RESULT_VARIABLE probe_result)

set(report "${probe_stdout}${probe_stderr}")

if (EXPECT STREQUAL "stop")
    if (probe_result EQUAL 0)
        message(FATAL_ERROR "the probe exited 0 where it had to stop\n${report}")
    endif ()
    return ()
endif ()

if (EXPECT STREQUAL "clean")
    if (NOT probe_result EQUAL 0)
        message(FATAL_ERROR "a run that had to complete ended as ${probe_result}\n${report}")
    endif ()
    return ()
endif ()

message(FATAL_ERROR "expect_stop.cmake was given the unknown expectation '${EXPECT}'")
