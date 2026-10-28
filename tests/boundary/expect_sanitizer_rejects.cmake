# Control for expect_sanitizer.cmake. Pointed at a run that produces no report,
# the assertion must fail, and it must fail with its own missing-report
# diagnostic: a control that accepts any nonzero exit also accepts a probe binary
# that is not there at all, which is how a green tripwire suite can be measuring
# nothing.

if (NOT DEFINED EXPECT_SCRIPT OR NOT DEFINED PROBE)
    message(FATAL_ERROR
        "expect_sanitizer_rejects.cmake requires -DEXPECT_SCRIPT and -DPROBE")
endif ()

execute_process(
    COMMAND "${CMAKE_COMMAND}"
            "-DPROBE=${PROBE}" "-DPROBE_ARGS=${PROBE_ARGS}" -DEXPECT=report
            "-DWANT_ERROR=${WANT_ERROR}" "-DWANT_FILE=${WANT_FILE}"
            -P "${EXPECT_SCRIPT}"
    OUTPUT_VARIABLE assertion_stdout
    ERROR_VARIABLE assertion_stderr
    RESULT_VARIABLE assertion_result)

set(report "${assertion_stdout}${assertion_stderr}")

if (assertion_result EQUAL 0)
    message(FATAL_ERROR
        "the assertion accepted a run that produced no sanitizer report\n${report}")
endif ()

string(FIND "${report}" "no sanitizer report matching" diagnostic_position)
if (diagnostic_position EQUAL -1)
    message(FATAL_ERROR
        "the assertion failed for some reason other than a missing report\n${report}")
endif ()
