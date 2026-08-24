function(matrix_execute cell step expect_failure)
    execute_process(COMMAND ${ARGN}
        TIMEOUT ${MATRIX_STEP_TIMEOUT}
        RESULT_VARIABLE result
        OUTPUT_VARIABLE output
        ERROR_VARIABLE output)
    string(REGEX REPLACE "[ \t\r\n]+" " " normalized "${output}")
    set(MATRIX_OUTPUT "${normalized}" PARENT_SCOPE)
    set(MATRIX_RESULT "${result}" PARENT_SCOPE)
    if (expect_failure STREQUAL "ANY")
        return()
    endif ()
    if (expect_failure AND result EQUAL 0)
        message(FATAL_ERROR "${cell}: ${step} succeeded, and this cell asserts that it must not\n${output}")
    endif ()
    if (NOT expect_failure AND NOT result EQUAL 0)
        message(FATAL_ERROR "${cell}: ${step} failed with '${result}'\n${output}")
    endif ()
endfunction()

# The single place a configure command is spelled, so no cell can reach a find_package
# without the package registries disabled, and none can silently exercise a different
# toolchain than the build that registered the test.
function(matrix_cmake_configure cell build expect_failure source)
    matrix_execute("${cell}" "configure ${build}" "${expect_failure}"
        ${CMAKE_COMMAND} -S "${source}" -B "${build}" -G "${MATRIX_GENERATOR}"
        "-DCMAKE_BUILD_TYPE=${MATRIX_BUILD_TYPE}"
        -DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF
        -DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF
        ${MATRIX_TOOLCHAIN_ARGUMENTS} ${ARGN})
    set(MATRIX_OUTPUT "${MATRIX_OUTPUT}" PARENT_SCOPE)
    set(MATRIX_RESULT "${MATRIX_RESULT}" PARENT_SCOPE)
endfunction()

function(matrix_configure cell source build)
    matrix_cmake_configure("${cell}" "${build}" FALSE "${source}" ${ARGN})
    matrix_require_cache("${cell}" "${build}" "^CMAKE_FIND_USE_PACKAGE_REGISTRY:[A-Z]+=OFF$")
    matrix_require_cache("${cell}" "${build}" "^CMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY:[A-Z]+=OFF$")
    set(MATRIX_OUTPUT "${MATRIX_OUTPUT}" PARENT_SCOPE)
endfunction()

function(matrix_refuse_configure cell source build)
    matrix_cmake_configure("${cell}" "${build}" TRUE "${source}" ${ARGN})
    set(MATRIX_OUTPUT "${MATRIX_OUTPUT}" PARENT_SCOPE)
endfunction()

function(matrix_try_configure cell source build)
    matrix_cmake_configure("${cell}" "${build}" ANY "${source}" ${ARGN})
    set(MATRIX_OUTPUT "${MATRIX_OUTPUT}" PARENT_SCOPE)
    set(MATRIX_RESULT "${MATRIX_RESULT}" PARENT_SCOPE)
endfunction()

function(matrix_build cell build)
    matrix_execute("${cell}" "build ${build}" FALSE
        ${CMAKE_COMMAND} --build "${build}" --config "${MATRIX_BUILD_TYPE}"
        --parallel ${MATRIX_PARALLEL})
endfunction()

function(matrix_install cell build)
    matrix_execute("${cell}" "install ${build}" FALSE
        ${CMAKE_COMMAND} --install "${build}" --config "${MATRIX_BUILD_TYPE}")
endfunction()

function(matrix_run_tests cell build)
    matrix_execute("${cell}" "ctest ${build}" FALSE
        ${MATRIX_CTEST} --test-dir "${build}" --build-config "${MATRIX_BUILD_TYPE}"
        --output-on-failure --no-tests=error)
endfunction()

function(matrix_relocate cell staged moved)
    if (NOT IS_DIRECTORY "${staged}")
        message(FATAL_ERROR "${cell}: the install created no directory at ${staged}")
    endif ()
    file(RENAME "${staged}" "${moved}")
endfunction()

# A configure that fails for any reason other than an unpopulated dependency is a real
# breakage rather than a host without the package, so the probe insists on seeing the
# one diagnostic that only an unpopulated dependency produces.
function(matrix_host_provides cell backend out)
    string(TOUPPER "${backend}" upper)
    set(probe "${MATRIX_WORK_DIR}/${cell}-host-probe")
    matrix_try_configure("${cell}" "${CARTAN_SOURCE_DIR}" "${probe}"
        "-DCARTAN_BUILD_${upper}=ON" -DFETCHCONTENT_FULLY_DISCONNECTED=ON)
    file(REMOVE_RECURSE "${probe}")
    if (MATRIX_RESULT EQUAL 0)
        set(${out} TRUE PARENT_SCOPE)
        return()
    endif ()
    matrix_require_text("${cell}" "the host probe" "${MATRIX_OUTPUT}" "FETCHCONTENT_FULLY_DISCONNECTED")
    set(${out} FALSE PARENT_SCOPE)
endfunction()
