function(matrix_git_revision directory out)
    execute_process(COMMAND ${MATRIX_GIT} -C "${directory}" rev-parse HEAD
        TIMEOUT ${MATRIX_STEP_TIMEOUT}
        RESULT_VARIABLE result
        OUTPUT_VARIABLE revision
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
    if (NOT result EQUAL 0)
        set(revision "unknown")
    endif ()
    set(${out} "${revision}" PARENT_SCOPE)
endfunction()

function(matrix_require_revision cell directory expected)
    matrix_git_revision("${directory}" revision)
    execute_process(COMMAND ${MATRIX_GIT} -C "${directory}" rev-parse "${expected}^{commit}"
        TIMEOUT ${MATRIX_STEP_TIMEOUT}
        RESULT_VARIABLE result
        OUTPUT_VARIABLE resolved
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
    if (result EQUAL 0)
        set(expected "${resolved}")
    endif ()
    if (NOT revision STREQUAL "${expected}")
        message(FATAL_ERROR
            "${cell}: ${directory} is at revision '${revision}', not the pinned ${expected}")
    endif ()
endfunction()

# A supplied checkout is the escape hatch for developing against a churning sibling, so
# it is reported rather than rejected when it is not the pin; the revision the cell
# actually exercised then appears in the log instead of being assumed.
function(matrix_argmin_source cell out)
    if (MATRIX_ARGMIN_SOURCE_DIR)
        matrix_git_revision("${MATRIX_ARGMIN_SOURCE_DIR}" revision)
        if (revision STREQUAL "${CARTAN_ARGMIN_REVISION}")
            message(STATUS "matrix cell ${cell}: dependency source is the pinned revision")
        else ()
            message(WARNING "matrix cell ${cell}: the supplied dependency source is at "
                "'${revision}', not the pinned ${CARTAN_ARGMIN_REVISION}; this cell exercises "
                "that revision instead")
        endif ()
        set(${out} "${MATRIX_ARGMIN_SOURCE_DIR}" PARENT_SCOPE)
        return()
    endif ()
    set(clone "${MATRIX_WORK_DIR}/argmin-src")
    if (NOT IS_DIRECTORY "${clone}")
        matrix_execute("${cell}" "clone the dependency" FALSE
            ${MATRIX_GIT} clone --quiet "${CARTAN_ARGMIN_REPOSITORY}" "${clone}")
        matrix_execute("${cell}" "check out the pinned revision" FALSE
            ${MATRIX_GIT} -C "${clone}" checkout --quiet "${CARTAN_ARGMIN_REVISION}")
    endif ()
    matrix_require_revision("${cell}" "${clone}" "${CARTAN_ARGMIN_REVISION}")
    set(${out} "${clone}" PARENT_SCOPE)
endfunction()
