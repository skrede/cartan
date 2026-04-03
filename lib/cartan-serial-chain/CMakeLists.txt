option(CARTAN_BUILD_NABLAPP "Build with nablapp optimization library" ${CARTAN_CMAKE_FETCH_DEPS})

if (CARTAN_BUILD_NABLAPP)
    if (CARTAN_CMAKE_FETCH_DEPS)
        FetchContent_Declare(
            nablapp
            GIT_REPOSITORY git@github.com:skrede/nablapp.git
            GIT_TAG milestone/v0.2.0-11-intervention
            EXCLUDE_FROM_ALL
            SYSTEM
        )
        FetchContent_MakeAvailable(nablapp)
    else ()
        find_package(nablapp CONFIG REQUIRED)
    endif ()
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

if (CARTAN_BUILD_NABLAPP)
    target_link_libraries(cartan_serial_chain
        INTERFACE
        nablapp::nablapp
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
