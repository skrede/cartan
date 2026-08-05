cartan_acquire_meios()

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
    meios::urdf
)

# The optional evaluation component enters through this module and nowhere else:
# it is part of the same supplier boundary the module exists to hold, and a
# consumer that reaches it directly widens that boundary.
if (CARTAN_MEIOS_ENABLE_PYTHON_EVAL)
    target_link_libraries(cartan_urdf INTERFACE meios::eval-python)
endif ()

target_compile_features(cartan_urdf INTERFACE cxx_std_20)

set_target_properties(cartan_urdf PROPERTIES EXPORT_NAME urdf)

if (CARTAN_ENABLE_INSTALL)
    install(TARGETS cartan_urdf EXPORT cartanTargets)
    install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    )
endif ()
