add_library(liepp_analytical INTERFACE)
add_library(liepp::analytical ALIAS liepp_analytical)

target_include_directories(liepp_analytical
    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(liepp_analytical
    INTERFACE
    liepp::serial_chain
)
