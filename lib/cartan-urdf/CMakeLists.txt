if (CARTAN_CMAKE_FETCH_DEPS)
    FetchContent_Declare(
        pugixml
        GIT_REPOSITORY https://github.com/zeux/pugixml.git
        GIT_TAG v1.15
        GIT_SHALLOW TRUE
        EXCLUDE_FROM_ALL
        SYSTEM
    )
    set(PUGIXML_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(PUGIXML_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(pugixml)
else ()
    find_package(pugixml CONFIG REQUIRED)
    if (pugixml_VERSION VERSION_LESS "1.14")
        message(FATAL_ERROR "cartan-urdf requires pugixml >= 1.14, found ${pugixml_VERSION}")
    endif ()
endif ()

add_library(cartan_urdf INTERFACE)
add_library(cartan::urdf ALIAS cartan_urdf)

target_include_directories(cartan_urdf
    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(cartan_urdf
    INTERFACE
    cartan::serial_chain
    pugixml::pugixml
)

target_compile_features(cartan_urdf INTERFACE cxx_std_20)

set_target_properties(cartan_urdf PROPERTIES EXPORT_NAME urdf)

if (CARTAN_ENABLE_INSTALL)
    install(TARGETS cartan_urdf EXPORT cartanTargets)
    install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
endif ()
