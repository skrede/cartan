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
