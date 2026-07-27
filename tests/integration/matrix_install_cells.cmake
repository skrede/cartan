function(matrix_consume cell label prefix)
    set(build "${MATRIX_WORK_DIR}/${cell}-consume-${label}")
    matrix_configure("${cell}" "${CARTAN_SOURCE_DIR}/tests/integration/consumers/installed" "${build}"
        "-DCMAKE_PREFIX_PATH=${prefix}" ${ARGN})
    matrix_build("${cell}" "${build}")
    matrix_run_tests("${cell}" "${build}")
endfunction()

# Requesting a component the install was not built with is the only path through the
# generated config's component bookkeeping; without it that whole block can be deleted
# and every other assertion here still passes.
function(matrix_refuse_absent_components cell prefix enabled)
    foreach (backend IN LISTS CARTAN_OPTIONAL_BACKENDS)
        if (backend IN_LIST enabled)
            continue ()
        endif ()
        string(TOUPPER "${backend}" upper)
        matrix_refuse_configure("${cell}"
            "${CARTAN_SOURCE_DIR}/tests/integration/consumers/installed"
            "${MATRIX_WORK_DIR}/${cell}-requests-${backend}"
            "-DCMAKE_PREFIX_PATH=${prefix}" "-DCONSUMER_EXPECTS_${upper}=ON")
        matrix_require_text("${cell}" "the refusal for an absent component" "${MATRIX_OUTPUT}"
            "installed without the requested optional component\\(s\\): ${backend}")
    endforeach ()
endfunction()

function(matrix_cell_minimal)
    set(cell minimal)
    set(build "${MATRIX_WORK_DIR}/${cell}-build")
    set(staged "${MATRIX_WORK_DIR}/${cell}-stage")
    set(moved "${MATRIX_WORK_DIR}/${cell}-moved")
    matrix_configure("${cell}" "${CARTAN_SOURCE_DIR}" "${build}"
        "-DCMAKE_INSTALL_PREFIX=${staged}" -DFETCHCONTENT_FULLY_DISCONNECTED=ON)
    matrix_require_cache("${cell}" "${build}" "^CARTAN_ENABLE_INSTALL:BOOL=ON$")
    matrix_build("${cell}" "${build}")
    matrix_install("${cell}" "${build}")
    matrix_require_export("${cell}" "${staged}" "")
    matrix_relocate("${cell}" "${staged}" "${moved}")
    matrix_consume("${cell}" no-backend "${moved}")
    matrix_refuse_absent_components("${cell}" "${moved}" "")
endfunction()

# The dependency reaches a consumer as <package>_DIR rather than a second
# CMAKE_PREFIX_PATH entry: a semicolon-separated value would be split apart again by the
# unquoted argument forwarding below, silently dropping everything after the first
# prefix. CMAKE_PREFIX_PATH therefore names the relocated cartan prefix and nothing else.
function(matrix_backend_cell cell backend dependency_dir)
    string(TOUPPER "${backend}" upper)
    set(build "${MATRIX_WORK_DIR}/${cell}-build")
    set(staged "${MATRIX_WORK_DIR}/${cell}-stage")
    set(moved "${MATRIX_WORK_DIR}/${cell}-moved")
    matrix_configure("${cell}" "${CARTAN_SOURCE_DIR}" "${build}"
        "-DCMAKE_INSTALL_PREFIX=${staged}" "-DCARTAN_BUILD_${upper}=ON"
        -DFETCHCONTENT_FULLY_DISCONNECTED=ON ${ARGN})
    matrix_require_cache("${cell}" "${build}" "^CARTAN_ENABLE_INSTALL:BOOL=ON$")
    matrix_require_no_file("${cell}" "${build}/_deps/${backend}-src")
    matrix_build("${cell}" "${build}")
    matrix_install("${cell}" "${build}")
    matrix_require_export("${cell}" "${staged}" "${backend}")
    matrix_relocate("${cell}" "${staged}" "${moved}")
    set(linked_arguments "-DCONSUMER_EXPECTS_${upper}=ON")
    if (dependency_dir)
        list(APPEND linked_arguments "-D${CARTAN_${upper}_PACKAGE}_DIR=${dependency_dir}")
    endif ()
    matrix_consume("${cell}" linked "${moved}" ${linked_arguments})
    matrix_consume("${cell}" unlinked "${moved}")
    matrix_refuse_absent_components("${cell}" "${moved}" "${backend}")
endfunction()

function(matrix_cell_argmin)
    set(cell argmin)
    set(build "${MATRIX_WORK_DIR}/${cell}-dependency-build")
    set(staged "${MATRIX_WORK_DIR}/${cell}-dependency-stage")
    set(moved "${MATRIX_WORK_DIR}/${cell}-dependency-moved")
    matrix_argmin_source("${cell}" source)
    matrix_configure("${cell}" "${source}" "${build}" "-DCMAKE_INSTALL_PREFIX=${staged}")
    matrix_build("${cell}" "${build}")
    matrix_install("${cell}" "${build}")
    matrix_relocate("${cell}" "${staged}" "${moved}")
    matrix_package_dir("${cell}" "${moved}" "${CARTAN_ARGMIN_PACKAGE}" dependency_dir)
    matrix_backend_cell("${cell}" argmin "${dependency_dir}" "-DCMAKE_PREFIX_PATH=${moved}")
endfunction()

function(matrix_cell_nlopt)
    matrix_backend_cell(nlopt nlopt "")
endfunction()
