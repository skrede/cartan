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

# benchmark_utils.h ships KDL chain factories alongside the cartan PoE ones,
# so every benchmark target that includes it needs orocos-kdl on the link
# line. KDL is therefore a benchmark-section-wide dependency rather than
# a CARTAN_BUILD_TRAC_IK-gated one (the trac_ik FetchContent transitively
# pulls KDL too, but we want the headers and link target available even
# without trac_ik).
find_package(orocos_kdl CONFIG REQUIRED)

function(cartan_add_benchmark BENCH_NAME BENCH_SOURCE)
    add_executable(${BENCH_NAME} ${BENCH_SOURCE})
    target_link_libraries(${BENCH_NAME}
        PRIVATE
        cartan::cartan
        benchmark::benchmark
        benchmark::benchmark_main
        orocos-kdl
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
    if (CARTAN_TRAC_IK_SOURCE_DIR)
        FetchContent_Declare(
            trac_ik_lib
            SOURCE_DIR "${CARTAN_TRAC_IK_SOURCE_DIR}"
            EXCLUDE_FROM_ALL
            SYSTEM
        )
    else ()
        FetchContent_Declare(
            trac_ik_lib
            GIT_REPOSITORY git@github.com:skrede/trac_ik_lib.git
            GIT_TAG master
            EXCLUDE_FROM_ALL
            SYSTEM
        )
    endif ()
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
        target_link_libraries(ik_comparison_benchmarks PRIVATE argmin::argmin)
    endif()

    cartan_add_benchmark(basic_ik_full_benchmarks basic_ik_full_benchmarks.cpp)
    target_link_libraries(basic_ik_full_benchmarks PRIVATE trac_ik)
    if(CARTAN_BUILD_ARGMIN)
        target_compile_definitions(basic_ik_full_benchmarks PRIVATE CARTAN_BUILD_ARGMIN)
        target_link_libraries(basic_ik_full_benchmarks PRIVATE argmin::argmin)
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
            argmin::argmin)
        target_compile_options(slsqp_per_pose_capture PRIVATE ${CARTAN_WARNING_FLAGS})

        add_executable(slsqp_per_pose_trace slsqp_per_pose_trace.cpp)
        target_link_libraries(slsqp_per_pose_trace
            PRIVATE
            cartan::cartan
            argmin::argmin)
        target_compile_options(slsqp_per_pose_trace PRIVATE ${CARTAN_WARNING_FLAGS})
    endif()

    if(CARTAN_BUILD_NLOPT)
        add_executable(nlopt_slsqp_per_pose_capture nlopt_slsqp_per_pose_capture.cpp)
        target_compile_definitions(nlopt_slsqp_per_pose_capture PRIVATE CARTAN_HAS_NLOPT)
        target_link_libraries(nlopt_slsqp_per_pose_capture
            PRIVATE
            cartan::cartan
            NLopt::nlopt)
        target_compile_options(nlopt_slsqp_per_pose_capture PRIVATE ${CARTAN_WARNING_FLAGS})
    endif()

    cartan_add_benchmark(fk_comparison_benchmarks fk_comparison_benchmarks.cpp)
    target_link_libraries(fk_comparison_benchmarks PRIVATE trac_ik)

    cartan_add_benchmark(jacobian_comparison_benchmarks jacobian_comparison_benchmarks.cpp)
    target_link_libraries(jacobian_comparison_benchmarks PRIVATE trac_ik)
endif ()

if (CARTAN_BUILD_PINOCCHIO)
    find_package(pinocchio CONFIG REQUIRED)
    cartan_add_benchmark(fk_pinocchio_benchmarks fk_pinocchio_benchmarks.cpp)
    target_link_libraries(fk_pinocchio_benchmarks PRIVATE pinocchio::pinocchio)

    add_executable(perf_fk_cartan_ur3e perf_fk_cartan_ur3e.cpp)
    target_link_libraries(perf_fk_cartan_ur3e PRIVATE cartan::cartan orocos-kdl)
    target_compile_options(perf_fk_cartan_ur3e PRIVATE ${CARTAN_WARNING_FLAGS})

    add_executable(perf_fk_pinocchio_ur3e perf_fk_pinocchio_ur3e.cpp)
    target_link_libraries(perf_fk_pinocchio_ur3e PRIVATE cartan::cartan pinocchio::pinocchio orocos-kdl)
    target_compile_options(perf_fk_pinocchio_ur3e PRIVATE ${CARTAN_WARNING_FLAGS})

    add_executable(perf_fk_cartan_matrix_ur3e perf_fk_cartan_matrix_ur3e.cpp)
    target_link_libraries(perf_fk_cartan_matrix_ur3e PRIVATE cartan::cartan orocos-kdl)
    target_compile_options(perf_fk_cartan_matrix_ur3e PRIVATE ${CARTAN_WARNING_FLAGS})

    cartan_add_benchmark(ik_comparison_pinocchio_benchmarks ik_comparison_pinocchio_benchmarks.cpp)
    target_link_libraries(ik_comparison_pinocchio_benchmarks PRIVATE pinocchio::pinocchio)
    if (CARTAN_BUILD_TRAC_IK)
        target_link_libraries(ik_comparison_pinocchio_benchmarks PRIVATE trac_ik)
        target_compile_definitions(ik_comparison_pinocchio_benchmarks PRIVATE CARTAN_HAS_TRAC_IK)
    endif ()
    if (CARTAN_BUILD_ARGMIN)
        target_compile_definitions(ik_comparison_pinocchio_benchmarks PRIVATE CARTAN_BUILD_ARGMIN)
        target_link_libraries(ik_comparison_pinocchio_benchmarks PRIVATE argmin::argmin)
    endif ()

    cartan_add_benchmark(closed_form_ik_benchmarks closed_form_ik_benchmarks.cpp)
    target_link_libraries(closed_form_ik_benchmarks PRIVATE pinocchio::pinocchio)
    if (CARTAN_BUILD_TRAC_IK)
        target_link_libraries(closed_form_ik_benchmarks PRIVATE trac_ik)
        target_compile_definitions(closed_form_ik_benchmarks PRIVATE CARTAN_HAS_TRAC_IK)
    endif ()
    if (CARTAN_BUILD_ARGMIN)
        target_compile_definitions(closed_form_ik_benchmarks PRIVATE CARTAN_BUILD_ARGMIN)
        target_link_libraries(closed_form_ik_benchmarks PRIVATE argmin::argmin)
    endif ()
endif ()
