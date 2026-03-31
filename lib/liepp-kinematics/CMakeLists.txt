if (LIEPP_CMAKE_FETCH_DEPS)
    FetchContent_Declare(
        nablapp
        GIT_REPOSITORY git@github.com:skrede/nablapp.git
        GIT_TAG master
    )
    FetchContent_GetProperties(nablapp)

    if (NOT nablapp_POPULATED)
        FetchContent_Populate(nablapp)
        add_library(nablapp INTERFACE)
        add_library(nablapp::nablapp ALIAS nablapp)
        target_include_directories(nablapp SYSTEM INTERFACE
            "${nablapp_SOURCE_DIR}/lib/nablapp/include"
        )
        target_link_libraries(nablapp INTERFACE Eigen3::Eigen)
    endif ()
else ()
    find_package(nablapp CONFIG REQUIRED)
endif ()

add_library(liepp_kinematics INTERFACE)
add_library(liepp::kinematics ALIAS liepp_kinematics)

target_include_directories(liepp_kinematics
    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(liepp_kinematics
    INTERFACE
    liepp::lie
    nablapp::nablapp
)

# --- NLopt solver backend (optional) ---
if (LIEPP_BUILD_NLOPT)
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

    add_library(liepp_nlopt INTERFACE)
    add_library(liepp::nlopt ALIAS liepp_nlopt)

    target_link_libraries(liepp_nlopt
        INTERFACE
        liepp::kinematics
        nlopt
    )

    target_compile_definitions(liepp_nlopt
        INTERFACE
        LIEPP_HAS_NLOPT=1
    )
endif ()
