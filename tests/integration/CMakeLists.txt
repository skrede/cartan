option(CARTAN_BUILD_NETWORK_TESTS "Also register the packaging cells that clone dependencies" OFF)

set(matrix_arguments
    "-DCARTAN_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
    "-DMATRIX_GENERATOR=${CMAKE_GENERATOR}"
    "-DMATRIX_BUILD_TYPE=${CMAKE_BUILD_TYPE}"
    "-DMATRIX_ARGMIN_SOURCE_DIR=${CARTAN_ARGMIN_SOURCE_DIR}"
    "-DMATRIX_CMAKE_C_COMPILER=${CMAKE_C_COMPILER}"
    "-DMATRIX_CMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}"
    "-DMATRIX_CMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}"
    "-DMATRIX_CMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}"
    "-DMATRIX_CMAKE_GENERATOR_PLATFORM=${CMAKE_GENERATOR_PLATFORM}"
    "-DMATRIX_CMAKE_GENERATOR_TOOLSET=${CMAKE_GENERATOR_TOOLSET}"
)

add_test(NAME install_matrix
    COMMAND ${CMAKE_COMMAND} ${matrix_arguments}
    "-DMATRIX_WORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/matrix"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/run_matrix.cmake"
)

set_tests_properties(install_matrix PROPERTIES TIMEOUT 900 LABELS packaging)

# The cells that clone a dependency are a separate, opt-in entry: the default run has to
# stay usable with no network at all, and it reports every cell it left out.
if (CARTAN_BUILD_NETWORK_TESTS)
    add_test(NAME install_matrix_network
        COMMAND ${CMAKE_COMMAND} ${matrix_arguments}
        "-DMATRIX_WORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/matrix-network"
        -DMATRIX_ALLOW_NETWORK=TRUE
        -P "${CMAKE_CURRENT_SOURCE_DIR}/run_matrix.cmake"
    )

    set_tests_properties(install_matrix_network PROPERTIES TIMEOUT 3600 LABELS "packaging;network")
endif ()
