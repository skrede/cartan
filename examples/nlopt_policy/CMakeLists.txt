# The policy published here is example code rather than library surface, so the
# dependency it needs is acquired in this directory instead of by a project-wide
# option: cartan itself builds against Eigen and nothing else.
#
# Benchmarks and the profiling target consume the same policy, so this directory
# is added whenever any of the three is built, not only with the examples.
option(CARTAN_EXAMPLE_NLOPT_POLICY "Build the NLopt custom solve-policy example" OFF)

if (NOT CARTAN_EXAMPLE_NLOPT_POLICY)
    return ()
endif ()

# Discovered, never downloaded. This is a comparator's optimizer as well as this
# example's, and a benchmark suite that provisions the libraries it measures
# against decides which revision of them a published number describes.
find_package(NLopt CONFIG QUIET)
if (NOT NLopt_FOUND AND NOT TARGET NLopt::nlopt)
    message(WARNING
        "cartan examples: NLopt is not installed -- the NLopt solve-policy example is omitted. "
        "Install it, or add its prefix to CMAKE_PREFIX_PATH.")
    return ()
endif ()

cmake_language(DEFER CALL cartan_warn_directory_targets)

add_library(cartan_example_nlopt_policy INTERFACE)
add_library(cartan_examples::nlopt_policy ALIAS cartan_example_nlopt_policy)
target_include_directories(cartan_example_nlopt_policy
    INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(cartan_example_nlopt_policy
    INTERFACE
    cartan::cartan
    NLopt::nlopt)
# Consumers that compile the policy in conditionally -- the benchmarks and the
# profiling target -- key on this rather than on a project-wide option, so the
# feature macro travels with the target that supplies the headers.
target_compile_definitions(cartan_example_nlopt_policy INTERFACE CARTAN_EXAMPLE_HAS_NLOPT=1)

add_executable(nlopt_policy_example nlopt_policy_example.cpp)
target_link_libraries(nlopt_policy_example PRIVATE cartan_examples::nlopt_policy)
