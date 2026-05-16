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
    src/cartan_bindings.cpp
)

target_link_libraries(_core
    PRIVATE
    cartan::cartan
)

target_compile_features(_core PRIVATE cxx_std_23)

if (TARGET cartan::urdf)
    target_compile_definitions(_core PRIVATE CARTAN_PY_HAS_URDF=1)
endif ()

install(TARGETS _core LIBRARY DESTINATION cartan)
