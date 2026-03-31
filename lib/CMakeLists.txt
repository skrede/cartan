add_subdirectory(liepp-lie)
add_subdirectory(liepp-kinematics)

# Convenience target pulling both modules
add_library(liepp_all INTERFACE)
add_library(liepp::liepp ALIAS liepp_all)
target_link_libraries(liepp_all INTERFACE liepp::lie liepp::kinematics)
