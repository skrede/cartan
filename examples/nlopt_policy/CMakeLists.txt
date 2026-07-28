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

include(FetchContent)

find_package(NLopt CONFIG QUIET)
if (NOT NLopt_FOUND AND NOT TARGET NLopt::nlopt)
    FetchContent_Declare(nlopt
        GIT_REPOSITORY https://github.com/stevengj/nlopt.git
        GIT_TAG v2.10.1
        EXCLUDE_FROM_ALL
        SYSTEM
    )
    # NLOPT_SWIG and NLOPT_JAVA default on, and leaving them so makes the
    # fallback probe for SWIG, Java and JNI at configure time for bindings
    # nothing here consumes.
    block()
        set(NLOPT_PYTHON OFF)
        set(NLOPT_OCTAVE OFF)
        set(NLOPT_GUILE OFF)
        set(NLOPT_JAVA OFF)
        set(NLOPT_SWIG OFF)
        set(NLOPT_TESTS OFF)
        set(BUILD_SHARED_LIBS OFF)
        FetchContent_MakeAvailable(nlopt)
    endblock()
    if (NOT TARGET NLopt::nlopt)
        add_library(NLopt::nlopt ALIAS nlopt)
    endif ()
endif ()

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
target_compile_options(nlopt_policy_example PRIVATE ${CARTAN_WARNING_FLAGS})
