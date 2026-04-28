if (CARTAN_CMAKE_FETCH_DEPS)
    FetchContent_Declare(
        benchmark
        GIT_REPOSITORY https://github.com/google/benchmark.git
        GIT_TAG v1.9.5
        EXCLUDE_FROM_ALL
    )
    set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
    set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(benchmark)
else ()
    find_package(benchmark CONFIG REQUIRED)
endif ()

function(cartan_add_benchmark BENCH_NAME BENCH_SOURCE)
    add_executable(${BENCH_NAME} ${BENCH_SOURCE})
    target_link_libraries(${BENCH_NAME}
        PRIVATE
        cartan::cartan
        benchmark::benchmark
        benchmark::benchmark_main
    )
    target_compile_options(${BENCH_NAME} PRIVATE ${CARTAN_WARNING_FLAGS})
endfunction()

# Lie group benchmarks
cartan_add_benchmark(lie_group_benchmarks lie_group_benchmarks.cpp)

# FK and Jacobian benchmarks
cartan_add_benchmark(fk_benchmarks fk_benchmarks.cpp)
cartan_add_benchmark(jacobian_benchmarks jacobian_benchmarks.cpp)

# IK solve policy benchmarks (basic_ik_runner)
cartan_add_benchmark(basic_ik_dls_benchmarks basic_ik_dls_benchmarks.cpp)
cartan_add_benchmark(basic_ik_lm_benchmarks basic_ik_lm_benchmarks.cpp)

# Exhaustive IK solver benchmarks
cartan_add_benchmark(exhaustive_ik_full_benchmarks exhaustive_ik_full_benchmarks.cpp)

# BOBYQA/SLSQP benchmarks require NLopt
if (CARTAN_BUILD_NLOPT)
    if (NOT TARGET NLopt::nlopt)
        find_package(NLopt CONFIG REQUIRED)
    endif()
    cartan_add_benchmark(basic_ik_sqp_benchmarks basic_ik_sqp_benchmarks.cpp)
    target_compile_definitions(basic_ik_sqp_benchmarks PRIVATE CARTAN_HAS_NLOPT)
    target_link_libraries(basic_ik_sqp_benchmarks PRIVATE NLopt::nlopt)
endif()

# TRAC-IK comparison benchmarks (requires orocos-kdl + NLopt via trac_ik)
if (CARTAN_BUILD_TRAC_IK)
    FetchContent_Declare(
        trac_ik_lib
        GIT_REPOSITORY git@github.com:skrede/trac_ik_lib.git
        GIT_TAG master
        EXCLUDE_FROM_ALL
        SYSTEM
    )
    set(TRAC_IK_WITH_URDF OFF CACHE BOOL "" FORCE)
    find_package(Eigen3 CONFIG REQUIRED)
    set(EIGEN3_FOUND TRUE) # To satisfy orocos-kdl and their wrong capitalization requirement of Eigen3
    set(EIGEN3_INCLUDE_DIR "${EIGEN3_INCLUDE_DIR}")
    FetchContent_MakeAvailable(trac_ik_lib)

    cartan_add_benchmark(ik_comparison_benchmarks ik_comparison_benchmarks.cpp)
    target_link_libraries(ik_comparison_benchmarks PRIVATE trac_ik)
    if(CARTAN_BUILD_NLOPT)
        target_compile_definitions(ik_comparison_benchmarks PRIVATE CARTAN_HAS_NLOPT)
        target_link_libraries(ik_comparison_benchmarks PRIVATE NLopt::nlopt)
    endif()
    if(CARTAN_BUILD_ARGMIN)
        target_compile_definitions(ik_comparison_benchmarks PRIVATE CARTAN_BUILD_ARGMIN)
        target_link_libraries(ik_comparison_benchmarks PRIVATE nablapp::nablapp)
    endif()

    cartan_add_benchmark(basic_ik_full_benchmarks basic_ik_full_benchmarks.cpp)
    target_link_libraries(basic_ik_full_benchmarks PRIVATE trac_ik)
    if(CARTAN_BUILD_ARGMIN)
        target_compile_definitions(basic_ik_full_benchmarks PRIVATE CARTAN_BUILD_ARGMIN)
        target_link_libraries(basic_ik_full_benchmarks PRIVATE nablapp::nablapp)
    endif()
    if(CARTAN_BUILD_NLOPT)
        target_compile_definitions(basic_ik_full_benchmarks PRIVATE CARTAN_HAS_NLOPT)
        target_link_libraries(basic_ik_full_benchmarks PRIVATE NLopt::nlopt)
    endif()

    if(CARTAN_BUILD_ARGMIN)
        add_executable(slsqp_per_pose_capture slsqp_per_pose_capture.cpp)
        target_link_libraries(slsqp_per_pose_capture
            PRIVATE
            cartan::cartan
            nablapp::nablapp)
        target_compile_options(slsqp_per_pose_capture PRIVATE ${CARTAN_WARNING_FLAGS})

        add_executable(slsqp_per_pose_trace slsqp_per_pose_trace.cpp)
        target_link_libraries(slsqp_per_pose_trace
            PRIVATE
            cartan::cartan
            nablapp::nablapp)
        target_compile_options(slsqp_per_pose_trace PRIVATE ${CARTAN_WARNING_FLAGS})
    endif()

    cartan_add_benchmark(fk_comparison_benchmarks fk_comparison_benchmarks.cpp)
    target_link_libraries(fk_comparison_benchmarks PRIVATE trac_ik)

    cartan_add_benchmark(jacobian_comparison_benchmarks jacobian_comparison_benchmarks.cpp)
    target_link_libraries(jacobian_comparison_benchmarks PRIVATE trac_ik)
endif ()
