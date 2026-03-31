add_executable(lie_basics lie_basics.cpp)
target_link_libraries(lie_basics PRIVATE liepp::liepp)

add_executable(frame_safety frame_safety.cpp)
target_link_libraries(frame_safety PRIVATE liepp::liepp)

add_executable(fk_jacobian fk_jacobian.cpp)
target_link_libraries(fk_jacobian PRIVATE liepp::liepp)

add_executable(basic_ik basic_ik.cpp)
target_link_libraries(basic_ik PRIVATE liepp::liepp)

add_executable(ik_composition ik_composition.cpp)
target_link_libraries(ik_composition PRIVATE liepp::liepp)

add_executable(ik_service_single ik_service_single.cpp)
target_link_libraries(ik_service_single PRIVATE liepp::liepp)

add_executable(ik_service_multi ik_service_multi.cpp)
target_link_libraries(ik_service_multi PRIVATE liepp::liepp)

add_executable(nablapp_solvers nablapp_solvers.cpp)
target_link_libraries(nablapp_solvers PRIVATE liepp::liepp)
