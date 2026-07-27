# Two views of one public type, linked into one program. The only difference
# between odr_native and odr_backend is which component target they link, so a
# compile definition applied by hand here would prove nothing about what a
# consumer actually sees.

add_library(name_invariance_native OBJECT name_invariance.cpp)
target_link_libraries(name_invariance_native PRIVATE cartan::serial_chain)
target_compile_options(name_invariance_native PRIVATE ${CARTAN_WARNING_FLAGS})

if (TARGET cartan::argmin)
    add_library(name_invariance_backend OBJECT name_invariance.cpp)
    target_link_libraries(name_invariance_backend PRIVATE cartan::argmin)
    target_compile_options(name_invariance_backend PRIVATE ${CARTAN_WARNING_FLAGS})

    add_library(odr_native OBJECT tu_native.cpp)
    target_link_libraries(odr_native PRIVATE cartan::serial_chain)
    target_compile_options(odr_native PRIVATE ${CARTAN_WARNING_FLAGS})

    add_library(odr_backend OBJECT tu_backend.cpp)
    target_link_libraries(odr_backend PRIVATE cartan::argmin)
    target_compile_options(odr_backend PRIVATE ${CARTAN_WARNING_FLAGS})

    add_executable(mixed_configuration_test main.cpp)
    target_link_libraries(mixed_configuration_test
        PRIVATE cartan::serial_chain odr_native odr_backend)
    target_compile_options(mixed_configuration_test PRIVATE ${CARTAN_WARNING_FLAGS})
    add_test(NAME mixed_configuration_test COMMAND mixed_configuration_test)
endif ()
