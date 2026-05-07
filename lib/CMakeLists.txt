add_subdirectory(cartan-lie)
add_subdirectory(cartan-serial-chain)
add_subdirectory(cartan-analytical)

# Convenience target pulling all modules
add_library(cartan_all INTERFACE)
add_library(cartan::cartan ALIAS cartan_all)
target_link_libraries(cartan_all INTERFACE cartan::lie cartan::serial_chain cartan::analytical)
