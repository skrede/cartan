function(matrix_execute cell step expect_failure)
    execute_process(COMMAND ${ARGN}
        RESULT_VARIABLE result
        OUTPUT_VARIABLE output
        ERROR_VARIABLE output)
    set(MATRIX_OUTPUT "${output}" PARENT_SCOPE)
    if (expect_failure AND result EQUAL 0)
        message(FATAL_ERROR "${cell}: ${step} succeeded, and this cell asserts that it must not\n${output}")
    endif ()
    if (NOT expect_failure AND NOT result EQUAL 0)
        message(FATAL_ERROR "${cell}: ${step} failed with '${result}'\n${output}")
    endif ()
endfunction()

# The single place a configure command is spelled, so no cell can reach a
# find_package without the package registries disabled.
function(matrix_cmake_configure cell build expect_failure source)
    matrix_execute("${cell}" "configure ${build}" "${expect_failure}"
        ${CMAKE_COMMAND} -S "${source}" -B "${build}" -G "${MATRIX_GENERATOR}"
        -DCMAKE_BUILD_TYPE=Release
        -DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF
        -DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF
        ${ARGN})
    set(MATRIX_OUTPUT "${MATRIX_OUTPUT}" PARENT_SCOPE)
endfunction()

function(matrix_configure cell source build)
    matrix_cmake_configure("${cell}" "${build}" FALSE "${source}" ${ARGN})
    matrix_require_cache("${cell}" "${build}" "^CMAKE_FIND_USE_PACKAGE_REGISTRY:[A-Z]+=OFF$")
    matrix_require_cache("${cell}" "${build}" "^CMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY:[A-Z]+=OFF$")
endfunction()

function(matrix_refuse_configure cell source build)
    matrix_cmake_configure("${cell}" "${build}" TRUE "${source}" ${ARGN})
    set(MATRIX_OUTPUT "${MATRIX_OUTPUT}" PARENT_SCOPE)
endfunction()

function(matrix_build cell build)
    matrix_execute("${cell}" "build ${build}" FALSE
        ${CMAKE_COMMAND} --build "${build}" --config Release --parallel ${MATRIX_PARALLEL})
endfunction()

function(matrix_install cell build)
    matrix_execute("${cell}" "install ${build}" FALSE
        ${CMAKE_COMMAND} --install "${build}" --config Release)
endfunction()

function(matrix_run_tests cell build)
    matrix_execute("${cell}" "ctest ${build}" FALSE
        ${MATRIX_CTEST} --test-dir "${build}" --build-config Release
        --output-on-failure --no-tests=error)
endfunction()

function(matrix_relocate cell staged moved)
    if (NOT IS_DIRECTORY "${staged}")
        message(FATAL_ERROR "${cell}: the install created no directory at ${staged}")
    endif ()
    file(RENAME "${staged}" "${moved}")
endfunction()

function(matrix_consume cell label prefix)
    set(build "${MATRIX_WORK_DIR}/${cell}-consume-${label}")
    matrix_configure("${cell}" "${CARTAN_SOURCE_DIR}/tests/integration/consumers/installed" "${build}"
        "-DCMAKE_PREFIX_PATH=${prefix}" ${ARGN})
    matrix_build("${cell}" "${build}")
    matrix_run_tests("${cell}" "${build}")
endfunction()

function(matrix_argmin_source cell out)
    if (MATRIX_ARGMIN_SOURCE_DIR)
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
    set(${out} "${clone}" PARENT_SCOPE)
endfunction()
