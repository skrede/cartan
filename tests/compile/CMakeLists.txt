add_executable(include_test include_test.cpp)
target_link_libraries(include_test PRIVATE cartan::cartan)
target_compile_options(include_test PRIVATE ${CARTAN_WARNING_FLAGS})
add_test(NAME include_test COMMAND include_test)

# Compile-fail test infrastructure
# Each test should FAIL to compile. If it compiles, that's a test failure.
# Uses try_compile at configure time to verify type-safety errors.
function(cartan_add_compile_fail_test TEST_NAME TEST_SOURCE)
    try_compile(${TEST_NAME}_COMPILES
        PROJECT compile_fail_${TEST_NAME}
        SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/fail"
        BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/${TEST_NAME}_test"
        TARGET compile_fail_target
        CMAKE_FLAGS
            "-DCMAKE_CXX_STANDARD=23"
            "-DCARTAN_INCLUDE_DIR=${PROJECT_SOURCE_DIR}/lib/cartan-lie/include"
            "-DEIGEN_INCLUDE_DIR=${eigen3_SOURCE_DIR}"
            "-DTEST_SOURCE=${TEST_SOURCE}"
    )

    if(${TEST_NAME}_COMPILES)
        message(FATAL_ERROR "Compile-fail test '${TEST_NAME}' unexpectedly compiled!")
    endif()

    add_test(NAME compile_fail_${TEST_NAME}
        COMMAND ${CMAKE_COMMAND} -E echo "PASS: ${TEST_NAME} correctly fails to compile")
endfunction()

cartan_add_compile_fail_test(frame_mismatch_compose frame_mismatch_compose.cpp)
cartan_add_compile_fail_test(frame_mismatch_inverse_compose frame_mismatch_inverse_compose.cpp)
cartan_add_compile_fail_test(frame_mismatch_adjoint_twist frame_mismatch_adjoint_twist.cpp)
cartan_add_compile_fail_test(frame_mismatch_coadjoint_wrench frame_mismatch_coadjoint_wrench.cpp)
