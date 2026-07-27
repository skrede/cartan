# Bounded corpus replay: every stored input executed exactly once, no mutation.
#
# This is a regression check and it must never stand in for a fuzzing run: a
# replay is structurally unable to discover a defect that needs a mutation to
# reach, which is most of them. Its job is to guarantee that the targets are
# executed by something in the repository rather than only built.
#
# An empty or shrunken corpus fails loudly. A replay over no inputs is the exact
# shape of a check that passes because it tested nothing.

file(GLOB inputs "${CORPUS}/*")
list(LENGTH inputs available)
if (available LESS MINIMUM)
    message(FATAL_ERROR
        "corpus ${CORPUS} holds ${available} inputs, fewer than the ${MINIMUM} required")
endif ()

execute_process(
    COMMAND ${TARGET} -runs=0 -timeout=25 -rss_limit_mb=2048 ${CORPUS}
    RESULT_VARIABLE status
    OUTPUT_VARIABLE report
    ERROR_VARIABLE report
)

if (NOT status EQUAL 0)
    message(FATAL_ERROR "${TARGET} exited ${status}\n${report}")
endif ()

# libFuzzer prints its execution count on the final line. Its absence means the
# binary never reached its entry point, which a zero exit status alone would not
# distinguish from a clean replay.
if (NOT report MATCHES "Done ([0-9]+) runs")
    message(FATAL_ERROR "${TARGET} reported no execution count\n${report}")
endif ()
if (CMAKE_MATCH_1 LESS available)
    message(FATAL_ERROR
        "${TARGET} executed ${CMAKE_MATCH_1} inputs against a corpus of ${available}")
endif ()
