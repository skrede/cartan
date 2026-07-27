function(matrix_cell_backend_fetch_fallback)
    set(cell backend-fetch-fallback)
    set(build "${MATRIX_WORK_DIR}/${cell}-build")
    matrix_configure("${cell}" "${CARTAN_SOURCE_DIR}" "${build}"
        -DCARTAN_BUILD_ARGMIN=ON -DCARTAN_BUILD_EXAMPLES=ON)
    matrix_require_file("${cell}" "${build}/_deps/argmin-src")
    matrix_require_cache("${cell}" "${build}" "^CARTAN_ENABLE_INSTALL:BOOL=OFF$")
    matrix_build("${cell}" "${build}")
endfunction()

function(matrix_cell_fetched_deps_refusal)
    set(cell fetched-deps-refusal)
    set(refused "${MATRIX_WORK_DIR}/${cell}-install-requested")
    set(usable "${MATRIX_WORK_DIR}/${cell}-no-install-requested")
    # Both halves share one download area: the refusal fires after the dependencies
    # are populated, so the second half would otherwise clone the same two repositories again.
    set(fetch_mode -DCARTAN_CMAKE_FETCH_DEPS=ON -DCARTAN_BUILD_ARGMIN=ON
        -DCMAKE_DISABLE_FIND_PACKAGE_argmin=ON
        "-DFETCHCONTENT_BASE_DIR=${MATRIX_WORK_DIR}/${cell}-downloads")
    matrix_refuse_configure("${cell}" "${CARTAN_SOURCE_DIR}" "${refused}"
        ${fetch_mode} -DCARTAN_ENABLE_INSTALL=ON)
    matrix_require_text("${cell}" "the refusal" "${MATRIX_OUTPUT}" "CARTAN_ENABLE_INSTALL=OFF")
    matrix_require_text("${cell}" "the refusal" "${MATRIX_OUTPUT}" "CMAKE_PREFIX_PATH")
    matrix_configure("${cell}" "${CARTAN_SOURCE_DIR}" "${usable}" ${fetch_mode} -DCARTAN_BUILD_EXAMPLES=ON)
    matrix_require_cache("${cell}" "${usable}" "^CARTAN_ENABLE_INSTALL:BOOL=OFF$")
    matrix_build("${cell}" "${usable}")
endfunction()

function(matrix_embedded_cell cell fetched_dependency)
    set(build "${MATRIX_WORK_DIR}/${cell}-build")
    matrix_configure("${cell}" "${CARTAN_SOURCE_DIR}/tests/integration/consumers/embedded" "${build}"
        -DBUILD_SHARED_LIBS=ON "-DCARTAN_SOURCE_DIR=${CARTAN_SOURCE_DIR}" ${ARGN})
    matrix_require_cache("${cell}" "${build}" "^BUILD_SHARED_LIBS:[A-Z]+=ON$")
    matrix_require_cache("${cell}" "${build}" "^CARTAN_ENABLE_INSTALL:BOOL=OFF$")
    if (fetched_dependency)
        matrix_require_file("${cell}" "${build}/_deps/${fetched_dependency}-src")
    endif ()
    matrix_build("${cell}" "${build}")
    matrix_run_tests("${cell}" "${build}")
endfunction()

function(matrix_cell_embedded_headers)
    matrix_argmin_source(embedded-headers source)
    matrix_embedded_cell(embedded-headers ""
        -DCARTAN_BUILD_ARGMIN=ON "-DCARTAN_ARGMIN_SOURCE_DIR=${source}")
endfunction()

# Ignoring the system prefixes would also hide Eigen, which cartan requires in every
# configuration; disabling the one lookup is the portable way to force the fetch path.
function(matrix_cell_embedded_fetched)
    matrix_embedded_cell(embedded-fetched nlopt
        -DCARTAN_BUILD_NLOPT=ON -DCMAKE_DISABLE_FIND_PACKAGE_NLopt=ON)
endfunction()
