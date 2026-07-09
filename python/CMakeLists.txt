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

target_include_directories(_core PRIVATE src/bindings)

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
    target_compile_definitions(_core PRIVATE CARTAN_BUILD_ARGMIN=1)
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
