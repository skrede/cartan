function(matrix_cell_fetched_deps_refusal)
    set(cell fetched-deps-refusal)
    set(downloads "${MATRIX_WORK_DIR}/${cell}-downloads")
    set(refused "${MATRIX_WORK_DIR}/${cell}-install-requested")
    set(usable "${MATRIX_WORK_DIR}/${cell}-no-install-requested")
    # Both halves share one download area: the refusal fires after the dependencies are
    # populated, so the second half would otherwise clone them again.
    set(fetch_mode -DCARTAN_CMAKE_FETCH_DEPS=ON "-DFETCHCONTENT_BASE_DIR=${downloads}")
    matrix_refuse_configure("${cell}" "${CARTAN_SOURCE_DIR}" "${refused}"
        ${fetch_mode} -DCARTAN_ENABLE_INSTALL=ON)
    set(label "the refusal")
    matrix_require_text("${cell}" "${label}" "${MATRIX_OUTPUT}" "cannot generate install and export rules")
    matrix_require_text("${cell}" "${label}" "${MATRIX_OUTPUT}" "CARTAN_ENABLE_INSTALL=OFF to build without an install surface")
    matrix_require_text("${cell}" "${label}" "${MATRIX_OUTPUT}" "as findable packages via CMAKE_PREFIX_PATH")
    matrix_configure("${cell}" "${CARTAN_SOURCE_DIR}" "${usable}" ${fetch_mode} -DCARTAN_BUILD_EXAMPLES=ON)
    matrix_require_cache("${cell}" "${usable}" "^CARTAN_ENABLE_INSTALL:BOOL=OFF$")
    matrix_build("${cell}" "${usable}")
endfunction()

function(matrix_cell_embedded_headers)
    set(cell embedded-headers)
    set(build "${MATRIX_WORK_DIR}/${cell}-build")
    matrix_configure("${cell}" "${CARTAN_SOURCE_DIR}/tests/integration/consumers/embedded" "${build}"
        -DBUILD_SHARED_LIBS=ON "-DCARTAN_SOURCE_DIR=${CARTAN_SOURCE_DIR}"
        -DFETCHCONTENT_FULLY_DISCONNECTED=ON)
    matrix_require_cache("${cell}" "${build}" "^BUILD_SHARED_LIBS:[A-Z]+=ON$")
    matrix_require_cache("${cell}" "${build}" "^CARTAN_ENABLE_INSTALL:BOOL=OFF$")
    matrix_build("${cell}" "${build}")
    matrix_run_tests("${cell}" "${build}")
endfunction()
