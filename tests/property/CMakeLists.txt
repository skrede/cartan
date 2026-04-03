function(cartan_add_property_test TEST_NAME TEST_SOURCE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_link_libraries(${TEST_NAME}
        PRIVATE
        cartan::cartan
        Catch2::Catch2WithMain
        rapidcheck
        rapidcheck_catch
    )
    target_compile_options(${TEST_NAME} PRIVATE ${CARTAN_WARNING_FLAGS})
    if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(${TEST_NAME} PRIVATE -Wno-c2y-extensions)
    endif ()
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endfunction()

cartan_add_property_test(lie_group_axioms_test lie_group_axioms_test.cpp)
