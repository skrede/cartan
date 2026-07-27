# Runs a probe and asserts what its output must be. With EXPECT=report the output
# must name the expected error class and the expected source file and the process
# must exit nonzero; with EXPECT=clean the output must carry no sanitizer
# diagnostic at all and the process must exit zero. A CMake script rather than a
# shell wrapper so the mechanism runs on macOS and Windows as well as Linux.
#
# print_stacktrace is not optional. Without it an UndefinedBehaviorSanitizer
# report is a single line naming the third-party header where the bad access
# lands, and no frame names the cartan source file the assertion is about.

if (NOT DEFINED PROBE OR NOT DEFINED EXPECT)
    message(FATAL_ERROR "expect_sanitizer.cmake requires -DPROBE and -DEXPECT")
endif ()

execute_process(
    COMMAND "${CMAKE_COMMAND}" -E env "UBSAN_OPTIONS=print_stacktrace=1"
            "${PROBE}" ${PROBE_ARGS}
    OUTPUT_VARIABLE probe_stdout
    ERROR_VARIABLE probe_stderr
    RESULT_VARIABLE probe_result)

set(report "${probe_stdout}${probe_stderr}")

if (EXPECT STREQUAL "clean")
    foreach (marker "AddressSanitizer" "runtime error" "UndefinedBehaviorSanitizer")
        string(FIND "${report}" "${marker}" marker_position)
        if (NOT marker_position EQUAL -1)
            message(FATAL_ERROR
                "an in-bounds run produced '${marker}'; exit ${probe_result}\n${report}")
        endif ()
    endforeach ()
    if (NOT probe_result EQUAL 0)
        message(FATAL_ERROR "an in-bounds run exited ${probe_result}\n${report}")
    endif ()
    return ()
endif ()

if (NOT DEFINED WANT_ERROR OR NOT DEFINED WANT_FILE)
    message(FATAL_ERROR "EXPECT=report requires -DWANT_ERROR and -DWANT_FILE")
endif ()

string(FIND "${report}" "${WANT_ERROR}" error_position)
if (error_position EQUAL -1)
    message(FATAL_ERROR
        "no sanitizer report matching '${WANT_ERROR}'; exit ${probe_result}\n${report}")
endif ()

# The file name rather than a frame name: frame names depend on the compiler and
# on what it chose to inline, whereas the file survives an inlining change.
string(FIND "${report}" "${WANT_FILE}" file_position)
if (file_position EQUAL -1)
    message(FATAL_ERROR
        "sanitizer report does not name '${WANT_FILE}'; exit ${probe_result}\n${report}")
endif ()

if (probe_result EQUAL 0)
    message(FATAL_ERROR "sanitizer reported but the process exited 0\n${report}")
endif ()
