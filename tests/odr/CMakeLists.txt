# Two views of one public type, linked into one program. The only difference
# between the native object library and a backend one is which component target
# it links, so a compile definition applied by hand here would prove nothing
# about what a consumer actually sees.

# No assertion inside a translation unit can see this. An alias keyed on a macro
# that no target defines still resolves to the native implementation in every
# configuration the build can produce, so the conditional is visible only in the
# header text -- which is exactly the state a reintroduction would restore.
set(cartan_odr_unconditional_headers
    "${PROJECT_SOURCE_DIR}/lib/cartan-serial-chain/include/cartan/serial/ik/solver/lm.h"
    "${PROJECT_SOURCE_DIR}/lib/cartan-serial-chain/include/cartan/serial/ik/solver/lbfgsb.h")
foreach (cartan_odr_header IN LISTS cartan_odr_unconditional_headers)
    file(STRINGS "${cartan_odr_header}" cartan_odr_conditionals
        REGEX "^[ ]*#[ ]*(if|ifdef|ifndef)")
    list(LENGTH cartan_odr_conditionals cartan_odr_count)
    if (NOT cartan_odr_count EQUAL 1)
        message(FATAL_ERROR
            "${cartan_odr_header} holds ${cartan_odr_count} preprocessor conditionals; only its "
            "include guard may be one. A public solver name that varies with build configuration "
            "is the defect this directory stands against.")
    endif ()
endforeach ()

add_library(name_invariance_native OBJECT name_invariance.cpp)
target_link_libraries(name_invariance_native PRIVATE cartan::serial_chain)
target_compile_options(name_invariance_native PRIVATE ${CARTAN_WARNING_FLAGS})

add_library(odr_native OBJECT tu_native.cpp)
target_link_libraries(odr_native PRIVATE cartan::serial_chain)
target_compile_options(odr_native PRIVATE ${CARTAN_WARNING_FLAGS})

set(cartan_odr_covered "")
foreach (cartan_odr_backend IN LISTS CARTAN_OPTIONAL_BACKENDS)
    if (NOT TARGET cartan::${cartan_odr_backend})
        continue ()
    endif ()
    list(APPEND cartan_odr_covered ${cartan_odr_backend})

    add_library(name_invariance_${cartan_odr_backend} OBJECT name_invariance.cpp)
    target_link_libraries(name_invariance_${cartan_odr_backend}
        PRIVATE cartan::${cartan_odr_backend})
    target_compile_options(name_invariance_${cartan_odr_backend}
        PRIVATE ${CARTAN_WARNING_FLAGS})

    add_library(odr_${cartan_odr_backend} OBJECT tu_backend.cpp)
    target_link_libraries(odr_${cartan_odr_backend} PRIVATE cartan::${cartan_odr_backend})
    target_compile_options(odr_${cartan_odr_backend} PRIVATE ${CARTAN_WARNING_FLAGS})

    add_executable(mixed_configuration_${cartan_odr_backend}_test main.cpp)
    target_link_libraries(mixed_configuration_${cartan_odr_backend}_test
        PRIVATE cartan::serial_chain odr_native odr_${cartan_odr_backend})
    target_compile_options(mixed_configuration_${cartan_odr_backend}_test
        PRIVATE ${CARTAN_WARNING_FLAGS})
    add_test(NAME mixed_configuration_${cartan_odr_backend}_test
        COMMAND mixed_configuration_${cartan_odr_backend}_test)
endforeach ()

if (cartan_odr_covered)
    message(STATUS "cartan: mixed-configuration link gate covers ${cartan_odr_covered}")
else ()
    message(STATUS
        "cartan: no optional backend component is built, so the mixed-configuration link gate "
        "registers no test and only the single-configuration name assertions are compiled")
endif ()
