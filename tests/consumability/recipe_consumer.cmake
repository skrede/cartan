set(recipe_consumer_directory "${CMAKE_CURRENT_LIST_DIR}")

# A published fence shows the lines a reader pastes into a project they already
# have, so the surrounding project is not in the fence and the harness must
# supply it: the minimum-version and project lines CMake refuses to configure
# without, the standard the library requires, and the executable the fence's
# link line names.
set(recipe_preamble_text [==[
cmake_minimum_required(VERSION 3.28)
project(recipe_consumer CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(my_app main.cpp)

]==])

function(recipe_step label)
    execute_process(
        COMMAND ${ARGN}
        RESULT_VARIABLE status
        TIMEOUT ${RECIPE_TIMEOUT}
        COMMAND_ECHO STDOUT
    )
    if (NOT status EQUAL 0)
        message(FATAL_ERROR "consumability: the ${label} step failed (exit ${status})")
    endif ()
endfunction()

function(recipe_assert_verbatim written body)
    string(FIND "${written}" "${body}" position)
    string(LENGTH "${written}" written_length)
    string(LENGTH "${body}" body_length)
    math(EXPR tail "${written_length} - ${body_length}")
    if (position LESS 0 OR NOT position EQUAL tail)
        message(FATAL_ERROR "consumability: the consumer's build description does not end in the emitted recipe verbatim")
    endif ()
endfunction()

function(recipe_materialize label body_path out_project out_target)
    set(project_directory "${RECIPE_OUT}/consumer-${label}")
    file(MAKE_DIRECTORY "${project_directory}")
    file(READ "${body_path}" body)
    set(preamble "")
    if (NOT body MATCHES "cmake_minimum_required")
        set(preamble "${recipe_preamble_text}")
        message(STATUS "consumability: recipe '${label}' is a fragment; the harness adds the preamble\n${preamble}")
    else ()
        message(STATUS "consumability: recipe '${label}' carries its own project preamble; the harness adds none")
    endif ()
    file(WRITE "${project_directory}/CMakeLists.txt" "${preamble}${body}")
    file(READ "${project_directory}/CMakeLists.txt" written)
    recipe_assert_verbatim("${written}" "${body}")
    configure_file("${recipe_consumer_directory}/main.cpp" "${project_directory}/main.cpp" COPYONLY)
    if (NOT written MATCHES "add_executable *\\( *([A-Za-z0-9_]+)")
        message(FATAL_ERROR "consumability: recipe '${label}' declares no executable to run")
    endif ()
    set(${out_target} "${CMAKE_MATCH_1}" PARENT_SCOPE)
    set(${out_project} "${project_directory}" PARENT_SCOPE)
endfunction()

function(recipe_executable build_directory target out_var)
    foreach (candidate IN ITEMS "${target}" "${target}.exe" "Release/${target}" "Release/${target}.exe")
        if (EXISTS "${build_directory}/${candidate}")
            set(${out_var} "${build_directory}/${candidate}" PARENT_SCOPE)
            return ()
        endif ()
    endforeach ()
    message(FATAL_ERROR "consumability: the build produced no executable named '${target}' under ${build_directory}")
endfunction()

function(recipe_consume label body_path)
    recipe_materialize("${label}" "${body_path}" project_directory target)
    set(build_directory "${RECIPE_OUT}/build-${label}")
    # A user-level package registry entry satisfies the dependency lookup the
    # gate must find missing, which would turn the whole run green for a reason
    # no consumer of a clean machine enjoys.
    set(arguments -S "${project_directory}" -B "${build_directory}"
        -DCMAKE_BUILD_TYPE=Release
        -DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF
        -DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF
    )
    if (RECIPE_MODE STREQUAL "working-tree")
        list(APPEND arguments "-DFETCHCONTENT_SOURCE_DIR_CARTAN=${RECIPE_REPO_ROOT}")
    endif ()
    if (RECIPE_BLOCK_SYSTEM_DEPENDENCY)
        list(APPEND arguments "-DCMAKE_DISABLE_FIND_PACKAGE_Eigen3=ON")
    endif ()
    recipe_step("configure '${label}'" "${CMAKE_COMMAND}" ${arguments})
    recipe_step("build '${label}'" "${CMAKE_COMMAND}" --build "${build_directory}" --config Release --parallel 2)
    recipe_executable("${build_directory}" "${target}" executable)
    recipe_step("run '${label}'" "${executable}")
    file(REMOVE_RECURSE "${build_directory}" "${project_directory}")
endfunction()
