cmake_minimum_required(VERSION 3.28)

if (NOT IS_DIRECTORY "${CARTAN_SOURCE_DIR}")
    message(FATAL_ERROR "CARTAN_SOURCE_DIR must name the cartan source tree, got '${CARTAN_SOURCE_DIR}'")
endif ()
if (NOT MATRIX_WORK_DIR)
    message(FATAL_ERROR "MATRIX_WORK_DIR must name the scratch directory each cell builds in")
endif ()
if (NOT MATRIX_GENERATOR)
    message(FATAL_ERROR "MATRIX_GENERATOR must name the generator every cell is driven with")
endif ()
if (NOT MATRIX_PARALLEL)
    set(MATRIX_PARALLEL 2)
endif ()

# The pinned revision and the backend registry have one source of truth in the tree,
# so a backend added there is covered without editing this harness.
include("${CARTAN_SOURCE_DIR}/cmake/CartanOptionalBackends.cmake")

find_program(MATRIX_GIT NAMES git REQUIRED)
get_filename_component(matrix_command_dir "${CMAKE_COMMAND}" DIRECTORY)
find_program(MATRIX_CTEST NAMES ctest HINTS "${matrix_command_dir}" REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/matrix_assert.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/matrix_steps.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/matrix_install_cells.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/matrix_build_cells.cmake")

set(matrix_known_cells
    minimal argmin nlopt backend-fetch-fallback fetched-deps-refusal
    embedded-headers embedded-fetched)

# Only an absent selection means "every cell"; an empty one is a caller that asked
# for a named set and named none, which must not pass as a run that proved something.
if (NOT DEFINED MATRIX_CELLS)
    set(MATRIX_CELLS ${matrix_known_cells})
endif ()
list(LENGTH MATRIX_CELLS matrix_selected_count)
if (matrix_selected_count EQUAL 0)
    message(FATAL_ERROR "no matrix cells were selected, so this run would assert nothing")
endif ()

file(REMOVE_RECURSE "${MATRIX_WORK_DIR}")
file(MAKE_DIRECTORY "${MATRIX_WORK_DIR}")

foreach (cell IN LISTS MATRIX_CELLS)
    if (NOT cell IN_LIST matrix_known_cells)
        message(FATAL_ERROR "unknown matrix cell '${cell}'; known cells are ${matrix_known_cells}")
    endif ()
    string(REPLACE "-" "_" entry "${cell}")
    message(STATUS "matrix cell ${cell}: running")
    cmake_language(CALL matrix_cell_${entry})
    message(STATUS "matrix cell ${cell}: passed")
endforeach ()

file(REMOVE_RECURSE "${MATRIX_WORK_DIR}")
message(STATUS "matrix: ${matrix_selected_count} cell(s) passed")
