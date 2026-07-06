if (CARTAN_BUILD_ARGMIN)
    if (CARTAN_ARGMIN_SOURCE_DIR)
        FetchContent_Declare(
            argmin
            SOURCE_DIR "${CARTAN_ARGMIN_SOURCE_DIR}"
            EXCLUDE_FROM_ALL
        )
    else ()
        FetchContent_Declare(
            argmin
            GIT_REPOSITORY https://github.com/skrede/argmin.git
            GIT_TAG master
            EXCLUDE_FROM_ALL
        )
    endif ()
    set(ARGMIN_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(ARGMIN_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(ARGMIN_BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(argmin)
endif ()

add_library(cartan_serial_chain INTERFACE)
add_library(cartan::serial_chain ALIAS cartan_serial_chain)

target_include_directories(cartan_serial_chain
    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(cartan_serial_chain
    INTERFACE
    cartan::lie
)

if (CARTAN_BUILD_ARGMIN)
    target_link_libraries(cartan_serial_chain
        INTERFACE
        argmin::argmin
    )
endif ()

target_compile_features(cartan_serial_chain INTERFACE cxx_std_20)

set_target_properties(cartan_serial_chain PROPERTIES EXPORT_NAME serial_chain)

if (CARTAN_ENABLE_INSTALL)
    install(TARGETS cartan_serial_chain EXPORT cartanTargets)
    install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
endif ()

# --- NLopt solver backend (optional) ---
if (CARTAN_BUILD_NLOPT)
    FetchContent_Declare(
        nlopt
        GIT_REPOSITORY https://github.com/stevengj/nlopt.git
        GIT_TAG v2.10.1
        EXCLUDE_FROM_ALL
        SYSTEM
    )
    set(NLOPT_PYTHON OFF CACHE BOOL "" FORCE)
    set(NLOPT_OCTAVE OFF CACHE BOOL "" FORCE)
    set(NLOPT_GUILE OFF CACHE BOOL "" FORCE)
    set(NLOPT_TESTS OFF CACHE BOOL "" FORCE)
    set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(nlopt)

    add_library(cartan_nlopt INTERFACE)
    add_library(cartan::nlopt ALIAS cartan_nlopt)

    target_link_libraries(cartan_nlopt
        INTERFACE
        cartan::serial_chain
        nlopt
    )

    target_compile_definitions(cartan_nlopt
        INTERFACE
        CARTAN_HAS_NLOPT=1
    )
endif ()
