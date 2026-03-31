if (LIEPP_CMAKE_FETCH_DEPS)
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

function(liepp_add_benchmark BENCH_NAME BENCH_SOURCE)
    add_executable(${BENCH_NAME} ${BENCH_SOURCE})
    target_link_libraries(${BENCH_NAME}
        PRIVATE
        liepp::liepp
        benchmark::benchmark
        benchmark::benchmark_main
    )
    target_compile_options(${BENCH_NAME} PRIVATE ${LIEPP_WARNING_FLAGS})
endfunction()

# Lie group benchmarks
liepp_add_benchmark(lie_group_benchmarks lie_group_benchmarks.cpp)

# FK and Jacobian benchmarks
liepp_add_benchmark(fk_benchmarks fk_benchmarks.cpp)
liepp_add_benchmark(jacobian_benchmarks jacobian_benchmarks.cpp)

# IK solve policy benchmarks
liepp_add_benchmark(ik_dls_benchmarks ik_dls_benchmarks.cpp)
liepp_add_benchmark(ik_lm_benchmarks ik_lm_benchmarks.cpp)

# BOBYQA/SLSQP benchmarks require NLopt
if (LIEPP_BUILD_NLOPT)
    if (NOT TARGET NLopt::nlopt)
        find_package(NLopt CONFIG REQUIRED)
    endif()
    liepp_add_benchmark(ik_sqp_benchmarks ik_sqp_benchmarks.cpp)
    target_compile_definitions(ik_sqp_benchmarks PRIVATE LIEPP_HAS_NLOPT)
    target_link_libraries(ik_sqp_benchmarks PRIVATE NLopt::nlopt)
endif()

# TRAC-IK comparison benchmark (requires KDL + NLopt via trac_ik)
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/trac_ik_lib/CMakeLists.txt")
    add_subdirectory(trac_ik_lib)

    liepp_add_benchmark(ik_comparison_benchmarks ik_comparison_benchmarks.cpp)
    target_link_libraries(ik_comparison_benchmarks PRIVATE trac_ik orocos-kdl)

    liepp_add_benchmark(ik_native_benchmarks ik_native_benchmarks.cpp)
    target_link_libraries(ik_native_benchmarks PRIVATE trac_ik orocos-kdl)
    if(LIEPP_BUILD_NLOPT)
        target_compile_definitions(ik_native_benchmarks PRIVATE LIEPP_HAS_NLOPT)
        target_link_libraries(ik_native_benchmarks PRIVATE NLopt::nlopt)
    endif()
endif ()
