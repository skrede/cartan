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
endif ()
