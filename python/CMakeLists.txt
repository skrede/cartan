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

nanobind_add_module(
    _core
    NB_STATIC
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

if (TARGET cartan::urdf)
    target_compile_definitions(_core PRIVATE CARTAN_PY_HAS_URDF=1)
endif ()

install(TARGETS _core LIBRARY DESTINATION cartan)

nanobind_add_stub(
    _core_stub
    MODULE _core
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/_core.pyi
    PYTHON_PATH $<TARGET_FILE_DIR:_core>
    DEPENDS _core
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/_core.pyi
    ${CMAKE_CURRENT_SOURCE_DIR}/cartan/py.typed
    DESTINATION cartan
)
