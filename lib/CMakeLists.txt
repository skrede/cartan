add_subdirectory(liepp-lie)
add_subdirectory(liepp-serial-chain)
add_subdirectory(liepp-analytical)

# Convenience target pulling all modules
add_library(liepp_all INTERFACE)
add_library(liepp::liepp ALIAS liepp_all)
target_link_libraries(liepp_all INTERFACE liepp::lie liepp::serial_chain liepp::analytical)
