add_executable(lie_basics lie_basics.cpp)
target_link_libraries(lie_basics PRIVATE cartan::cartan)

add_executable(frame_safety frame_safety.cpp)
target_link_libraries(frame_safety PRIVATE cartan::cartan)

add_executable(fk_jacobian fk_jacobian.cpp)
target_link_libraries(fk_jacobian PRIVATE cartan::cartan)

add_executable(basic_ik basic_ik.cpp)
target_link_libraries(basic_ik PRIVATE cartan::cartan)

add_executable(ik_composition ik_composition.cpp)
target_link_libraries(ik_composition PRIVATE cartan::cartan)

add_executable(ik_service_single ik_service_single.cpp)
target_link_libraries(ik_service_single PRIVATE cartan::cartan)

add_executable(ik_service_multi ik_service_multi.cpp)
target_link_libraries(ik_service_multi PRIVATE cartan::cartan)

add_executable(argmin_solvers argmin_solvers.cpp)
target_link_libraries(argmin_solvers PRIVATE cartan::cartan)
if (CARTAN_BUILD_ARGMIN)
    target_compile_definitions(argmin_solvers PRIVATE CARTAN_BUILD_ARGMIN)
    target_link_libraries(argmin_solvers PRIVATE argmin::argmin)
endif ()
