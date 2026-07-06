add_library(cartan_analytical INTERFACE)
add_library(cartan::analytical ALIAS cartan_analytical)

target_include_directories(cartan_analytical
    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(cartan_analytical
    INTERFACE
    cartan::serial_chain
)

target_compile_features(cartan_analytical INTERFACE cxx_std_20)

set_target_properties(cartan_analytical PROPERTIES EXPORT_NAME analytical)

if (CARTAN_ENABLE_INSTALL)
    install(TARGETS cartan_analytical EXPORT cartanTargets)
    install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
endif ()
