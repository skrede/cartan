if (NOT TARGET benchmark::benchmark)
    include(FetchContent)
    FetchContent_Declare(
        benchmark
        GIT_REPOSITORY https://github.com/google/benchmark.git
        GIT_TAG 192ef10025eb2c4cdd392bc502f0c852196baa48 # v1.9.5
        EXCLUDE_FROM_ALL
    )
    set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
    set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(benchmark)
endif()

add_executable(ik_profiling ik_profiling.cpp)
target_link_libraries(ik_profiling
    PRIVATE
    cartan::serial_chain
    benchmark::benchmark
    benchmark::benchmark_main
)
target_compile_options(ik_profiling PRIVATE ${CARTAN_WARNING_FLAGS})

# Argmin (argmin) solvers are optional
if (CARTAN_BUILD_ARGMIN)
    target_link_libraries(ik_profiling PRIVATE cartan::argmin)
endif()

# The NLopt solve policy ships as an example rather than as library surface, so
# the target carrying it exists only when that example's own option is on.
if (TARGET cartan_examples::nlopt_policy)
    target_link_libraries(ik_profiling PRIVATE cartan_examples::nlopt_policy)
endif()
