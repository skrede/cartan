add_subdirectory(cartan-lie)
add_subdirectory(cartan-serial-chain)
add_subdirectory(cartan-analytical)

# Convenience target pulling all modules
add_library(cartan_all INTERFACE)
add_library(cartan::cartan ALIAS cartan_all)
target_link_libraries(cartan_all INTERFACE cartan::lie cartan::serial_chain cartan::analytical)
target_compile_features(cartan_all INTERFACE cxx_std_20)

if (CARTAN_BUILD_URDF)
    add_subdirectory(cartan-urdf)
    target_link_libraries(cartan_all INTERFACE cartan::urdf)
endif ()

# Alias targets do not survive install(EXPORT); EXPORT_NAME is the supported
# mechanism to keep the cartan::cartan imported name in the installed package.
set_target_properties(cartan_all PROPERTIES EXPORT_NAME cartan)

if (CARTAN_ENABLE_INSTALL)
    install(TARGETS cartan_all EXPORT cartanTargets)
endif ()
