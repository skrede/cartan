# Appended to this directory's copy of the flag list rather than applied to each
# target: the sweep below puts the list on the command line last, and a
# suppression that -Wpedantic follows suppresses nothing.
if (CARTAN_HAS_WNO_C2Y_EXTENSIONS)
    list(APPEND CARTAN_WARNING_FLAGS -Wno-c2y-extensions)
endif ()

cmake_language(DEFER CALL cartan_warn_directory_targets)

function(cartan_add_property_test TEST_NAME TEST_SOURCE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_link_libraries(${TEST_NAME}
        PRIVATE
        cartan::cartan
        Catch2::Catch2WithMain
        rapidcheck
        rapidcheck_catch
    )
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endfunction()

cartan_add_property_test(lie_group_axioms_test lie_group_axioms_test.cpp)
