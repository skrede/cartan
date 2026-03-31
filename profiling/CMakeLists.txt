if (NOT TARGET benchmark::benchmark)
    include(FetchContent)
    FetchContent_Declare(
        benchmark
        GIT_REPOSITORY https://github.com/google/benchmark.git
        GIT_TAG v1.9.5
        EXCLUDE_FROM_ALL
    )
    set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
    set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(benchmark)
endif()

add_executable(ik_profiling ik_profiling.cpp)
target_link_libraries(ik_profiling
    PRIVATE
    liepp::kinematics
    benchmark::benchmark
    benchmark::benchmark_main
)
target_compile_options(ik_profiling PRIVATE ${LIEPP_WARNING_FLAGS})

# NLopt solvers are optional
if (LIEPP_BUILD_NLOPT)
    if (NOT TARGET NLopt::nlopt)
        find_package(NLopt CONFIG REQUIRED)
    endif()
    target_compile_definitions(ik_profiling PRIVATE LIEPP_HAS_NLOPT)
    target_link_libraries(ik_profiling PRIVATE NLopt::nlopt)
endif()
