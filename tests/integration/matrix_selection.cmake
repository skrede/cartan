set(matrix_known_cells
    minimal argmin nlopt backend-fetch-fallback fetched-deps-refusal
    embedded-headers embedded-fetched)
set(matrix_cloning_cells backend-fetch-fallback fetched-deps-refusal embedded-fetched)
set(matrix_dependency_source_cells argmin embedded-headers)
set(matrix_host_nlopt_cells nlopt)

# Returns the prerequisite a cell is missing, empty when it can run. A cell the caller
# named is never skipped on one of these: an explicit request that cannot be honored is
# reported rather than quietly dropped.
function(matrix_cell_prerequisite cell out)
    set(${out} "" PARENT_SCOPE)
    if (NOT MATRIX_ALLOW_NETWORK)
        if (cell IN_LIST matrix_cloning_cells)
            set(${out} "it clones dependencies and MATRIX_ALLOW_NETWORK is off" PARENT_SCOPE)
            return()
        endif ()
        if (cell IN_LIST matrix_dependency_source_cells AND NOT MATRIX_ARGMIN_SOURCE_DIR)
            set(${out} "it needs MATRIX_ARGMIN_SOURCE_DIR or MATRIX_ALLOW_NETWORK" PARENT_SCOPE)
            return()
        endif ()
    endif ()
    if (cell IN_LIST matrix_host_nlopt_cells)
        matrix_host_provides("${cell}" nlopt available)
        if (NOT available)
            set(${out} "NLopt is not installed here and this cell resolves it find-first" PARENT_SCOPE)
        endif ()
    endif ()
endfunction()
