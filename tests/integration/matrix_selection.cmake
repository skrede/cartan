set(matrix_known_cells minimal fetched-deps-refusal embedded-headers)
set(matrix_cloning_cells fetched-deps-refusal)

# Returns the prerequisite a cell is missing, empty when it can run. A cell the caller
# named is never skipped on one of these: an explicit request that cannot be honored is
# reported rather than quietly dropped.
function(matrix_cell_prerequisite cell out)
    set(${out} "" PARENT_SCOPE)
    if (NOT MATRIX_ALLOW_NETWORK AND cell IN_LIST matrix_cloning_cells)
        set(${out} "it clones dependencies and MATRIX_ALLOW_NETWORK is off" PARENT_SCOPE)
    endif ()
endfunction()
