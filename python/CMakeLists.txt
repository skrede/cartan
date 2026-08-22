find_package(Python 3.10 COMPONENTS Interpreter Development.Module REQUIRED)

execute_process(
    COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
    OUTPUT_VARIABLE _nanobind_cmake_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _nanobind_locate_result
)
if (NOT _nanobind_locate_result EQUAL 0)
    message(FATAL_ERROR
        "Failed to locate nanobind via Python module. Ensure nanobind is "
        "installed in the Python interpreter at ${Python_EXECUTABLE}.")
endif ()

list(APPEND CMAKE_PREFIX_PATH "${_nanobind_cmake_dir}")
find_package(nanobind CONFIG REQUIRED)

option(CARTAN_PYTHON_STABLE_ABI "Build Python extension with nanobind stable ABI" OFF)

set(_cartan_python_module_options NB_STATIC)
if (CARTAN_PYTHON_STABLE_ABI)
    list(APPEND _cartan_python_module_options STABLE_ABI)
endif ()

set(_cartan_source_digest_header "${CMAKE_CURRENT_BINARY_DIR}/generated/cartan_source_digest.h")
set(_cartan_source_digest_script "${CMAKE_CURRENT_SOURCE_DIR}/cmake/source_digest.cmake")
set(_cartan_source_digest_args
    "-DCARTAN_DIGEST_ROOT=${PROJECT_SOURCE_DIR}"
    "-DCARTAN_DIGEST_OUT=${_cartan_source_digest_header}")

# Re-stamped on every build, not at configure time: a header edited under lib/
# recompiles the extension without reconfiguring, and a stamp that did not move
# with it would report a stale module as fresh.
execute_process(COMMAND "${CMAKE_COMMAND}" ${_cartan_source_digest_args} -P "${_cartan_source_digest_script}")
add_custom_target(_core_source_digest
    COMMAND "${CMAKE_COMMAND}" ${_cartan_source_digest_args} -P "${_cartan_source_digest_script}"
    BYPRODUCTS "${_cartan_source_digest_header}"
    COMMENT "Hashing the sources the Python extension is built from"
    VERBATIM)

nanobind_add_module(
    _core
    ${_cartan_python_module_options}
    src/_core.cpp
    src/bindings/lie_bindings.cpp
    src/bindings/chain_bindings.cpp
    src/bindings/fk_bindings.cpp
    src/bindings/urdf_bindings.cpp
    src/bindings/ik_bindings.cpp
    src/bindings/analytical_bindings.cpp
    src/bindings/exhaustive_bindings.cpp
)

# nanobind publishes its own headers and CPython's with -I rather than -isystem
# unless asked, and the keyword that asks it only exists from nanobind 2.5 while
# the declared floor is 2.0. Republishing them as system roots is what keeps a
# C-style cast in code cartan does not own from becoming an error under cartan's
# warning set, as the Eigen acquisition does for Eigen. Cartan's own targets are
# linked after this loop precisely so that their roots stay non-system.
get_target_property(_cartan_core_links _core LINK_LIBRARIES)
foreach (_cartan_core_link IN LISTS _cartan_core_links)
    if (NOT TARGET ${_cartan_core_link})
        continue ()
    endif ()
    get_target_property(_cartan_link_includes ${_cartan_core_link}
        INTERFACE_INCLUDE_DIRECTORIES)
    if (_cartan_link_includes)
        set_target_properties(${_cartan_core_link} PROPERTIES
            INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_cartan_link_includes}")
    endif ()
endforeach ()

# Named rather than swept: nanobind_add_module defines its own static library
# in this directory, and a sweep of the directory would put cartan's warning set
# on nanobind's sources.
cartan_target_warnings(_core)

add_dependencies(_core _core_source_digest)

target_include_directories(_core PRIVATE src/bindings "${CMAKE_CURRENT_BINARY_DIR}/generated")

target_link_libraries(_core
    PRIVATE
    cartan::cartan
)

target_compile_features(_core PRIVATE cxx_std_20)

set_target_properties(_core PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/cartan")

if (TARGET cartan::urdf)
    target_compile_definitions(_core PRIVATE CARTAN_PY_HAS_URDF=1)
endif ()

if (CARTAN_BUILD_ARGMIN)
    target_link_libraries(_core PRIVATE cartan::argmin)
endif ()

install(TARGETS _core LIBRARY DESTINATION cartan)

nanobind_add_stub(
    _core_stub
    MODULE _core
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/_core.pyi
    PYTHON_PATH $<TARGET_FILE_DIR:_core>
    DEPENDS _core
)

# Separate stub for the cartan.analytical submodule. nanobind_add_stub on the
# parent module emits `from . import analytical as analytical` in _core.pyi but
# does not recurse into the submodule's symbol table, so a dedicated invocation
# is required for pyright / mypy to resolve cartan.analytical.solve_pieper_6r
# and the other 11 names registered in analytical_bindings.cpp.
nanobind_add_stub(
    _analytical_stub
    MODULE _core.analytical
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/analytical.pyi
    PYTHON_PATH $<TARGET_FILE_DIR:_core>
    DEPENDS _core
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/_core.pyi
    ${CMAKE_CURRENT_BINARY_DIR}/analytical.pyi
    ${CMAKE_CURRENT_SOURCE_DIR}/cartan/py.typed
    DESTINATION cartan
)

set(_cartan_python_tutorial_env
    "PYTHONPATH=$<TARGET_FILE_DIR:_core>:${CMAKE_CURRENT_SOURCE_DIR}"
    "PYTHONNOUSERSITE=1")

function(cartan_add_python_tutorial_test test_name script_name)
    add_test(
        NAME ${test_name}
        COMMAND "${Python_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/tutorials/${script_name}")
    set_tests_properties(${test_name} PROPERTIES
        ENVIRONMENT "${_cartan_python_tutorial_env}")
endfunction()

if (TARGET cartan::urdf)
    cartan_add_python_tutorial_test(
        python_tutorial_01_urdf_walkthrough
        01_urdf_walkthrough.py)
endif ()

cartan_add_python_tutorial_test(
    python_tutorial_02_fk_and_jacobians
    02_fk_and_jacobians.py)
cartan_add_python_tutorial_test(
    python_tutorial_03_ik_composition
    03_ik_composition.py)
