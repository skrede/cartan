add_executable(include_test include_test.cpp)
target_link_libraries(include_test PRIVATE cartan::cartan)
target_compile_options(include_test PRIVATE ${CARTAN_WARNING_FLAGS})
add_test(NAME include_test COMMAND include_test)

# Two-mode proof that the validated Lie factories compile with C++ exceptions
# both disabled and enabled. The exceptions-off build mirrors the embedded
# posture (ESP-IDF defaults to -fno-exceptions -fno-rtti) and guards against a
# regression that reintroduces an ungated throw on a factory-reachable path.
# The parity build keeps the exceptions-on behavior honest. Building either
# target is the test: if it compiles and links, the mode is sound.
set(CARTAN_NO_EXCEPTIONS_FLAGS
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-fno-exceptions -fno-rtti>)

add_executable(no_exceptions_factory_test no_exceptions_factory_test.cpp)
target_link_libraries(no_exceptions_factory_test PRIVATE cartan::cartan)
target_compile_options(no_exceptions_factory_test
    PRIVATE ${CARTAN_WARNING_FLAGS} ${CARTAN_NO_EXCEPTIONS_FLAGS})
add_test(NAME no_exceptions_factory_test COMMAND no_exceptions_factory_test)

add_executable(exceptions_factory_parity_test no_exceptions_factory_test.cpp)
target_link_libraries(exceptions_factory_parity_test PRIVATE cartan::cartan)
target_compile_options(exceptions_factory_parity_test PRIVATE ${CARTAN_WARNING_FLAGS})
add_test(NAME exceptions_factory_parity_test COMMAND exceptions_factory_parity_test)

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
