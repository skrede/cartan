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
if (NOT MATRIX_BUILD_TYPE)
    set(MATRIX_BUILD_TYPE Release)
endif ()
# Bounds a hung step: a network that drops rather than refuses would otherwise park the
# whole run against the registered test's own timeout.
if (NOT MATRIX_STEP_TIMEOUT)
    set(MATRIX_STEP_TIMEOUT 300)
endif ()

set(MATRIX_TOOLCHAIN_ARGUMENTS "")
foreach (setting IN ITEMS CMAKE_C_COMPILER CMAKE_CXX_COMPILER CMAKE_CXX_FLAGS
        CMAKE_TOOLCHAIN_FILE CMAKE_GENERATOR_PLATFORM CMAKE_GENERATOR_TOOLSET)
    if (MATRIX_${setting})
        list(APPEND MATRIX_TOOLCHAIN_ARGUMENTS "-D${setting}=${MATRIX_${setting}}")
    endif ()
endforeach ()

# The pinned revisions and the backend registry have one source of truth in the tree, so
# a backend added there is covered without editing this harness.
include("${CARTAN_SOURCE_DIR}/cmake/CartanOptionalBackends.cmake")

find_program(MATRIX_GIT NAMES git REQUIRED)
get_filename_component(matrix_command_dir "${CMAKE_COMMAND}" DIRECTORY)
find_program(MATRIX_CTEST NAMES ctest HINTS "${matrix_command_dir}" REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/matrix_assert.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/matrix_steps.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/matrix_dependency.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/matrix_selection.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/matrix_install_cells.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/matrix_build_cells.cmake")

# Only an absent selection means "every cell". An empty one is a caller that asked for a
# named set and named none.
if (DEFINED MATRIX_CELLS)
    set(matrix_selection_is_explicit TRUE)
else ()
    set(matrix_selection_is_explicit FALSE)
    set(MATRIX_CELLS ${matrix_known_cells})
endif ()

file(REMOVE_RECURSE "${MATRIX_WORK_DIR}")
file(MAKE_DIRECTORY "${MATRIX_WORK_DIR}")

set(matrix_ran "")
set(matrix_skipped "")
foreach (cell IN LISTS MATRIX_CELLS)
    if (NOT cell IN_LIST matrix_known_cells)
        message(FATAL_ERROR "unknown matrix cell '${cell}'; known cells are ${matrix_known_cells}")
    endif ()
    matrix_cell_prerequisite("${cell}" unmet)
    if (unmet)
        if (matrix_selection_is_explicit)
            message(FATAL_ERROR "matrix cell ${cell} was requested but cannot run: ${unmet}")
        endif ()
        message(STATUS "matrix cell ${cell}: skipped -- ${unmet}")
        list(APPEND matrix_skipped ${cell})
        continue ()
    endif ()
    string(REPLACE "-" "_" entry "${cell}")
    message(STATUS "matrix cell ${cell}: running")
    cmake_language(CALL matrix_cell_${entry})
    message(STATUS "matrix cell ${cell}: passed")
    list(APPEND matrix_ran ${cell})
endforeach ()

if (NOT matrix_ran)
    message(FATAL_ERROR "no matrix cell ran, so this run asserted nothing (skipped: ${matrix_skipped})")
endif ()

file(REMOVE_RECURSE "${MATRIX_WORK_DIR}")
message(STATUS "matrix: ran ${matrix_ran}")
message(STATUS "matrix: skipped ${matrix_skipped}")
