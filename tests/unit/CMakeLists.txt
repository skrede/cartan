function(cartan_add_test TEST_NAME TEST_SOURCE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_link_libraries(${TEST_NAME}
        PRIVATE
        cartan::cartan
        Catch2::Catch2WithMain
    )
    target_compile_options(${TEST_NAME} PRIVATE ${CARTAN_WARNING_FLAGS})
    if (CARTAN_HAS_WNO_C2Y_EXTENSIONS)
        target_compile_options(${TEST_NAME} PRIVATE -Wno-c2y-extensions)
    endif ()
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endfunction()

cartan_add_test(version_test version_test.cpp)
cartan_add_test(expected_test expected_test.cpp)
cartan_add_test(types_test types_test.cpp)
cartan_add_test(hat_vee_test hat_vee_test.cpp)
cartan_add_test(so2_test so2_test.cpp)
cartan_add_test(se2_test se2_test.cpp)
cartan_add_test(so3_test so3_test.cpp)
cartan_add_test(se3_test se3_test.cpp)
cartan_add_test(se3_left_jacobian_test se3_left_jacobian_test.cpp)
cartan_add_test(se3_q_matrix_sweep_test se3_q_matrix_sweep_test.cpp)
cartan_add_test(analytical_gradient_test analytical_gradient_test.cpp)
cartan_add_test(quaternion_utils_test quaternion_utils_test.cpp)
cartan_add_test(twist_test twist_test.cpp)
cartan_add_test(rotation_test rotation_test.cpp)
cartan_add_test(transform_test transform_test.cpp)
cartan_add_test(framed_twist_test framed_twist_test.cpp)
cartan_add_test(framed_wrench_test framed_wrench_test.cpp)
cartan_add_test(screw_axis_test screw_axis_test.cpp)
cartan_add_test(kinematic_chain_test kinematic_chain_test.cpp)
cartan_add_test(joint_limits_test joint_limits_test.cpp)
cartan_add_test(limit_feasibility_test limit_feasibility_test.cpp)
cartan_add_test(forward_kinematics_test forward_kinematics_test.cpp)
cartan_add_test(jacobian_compile_test jacobian_compile_test.cpp)
cartan_add_test(entry_point_parity_test entry_point_parity_test.cpp)
cartan_add_test(jacobian_test jacobian_test.cpp)
cartan_add_test(velocity_test velocity_test.cpp)
cartan_add_test(ik_compile_test ik_compile_test.cpp)
cartan_add_test(dls_test dls_test.cpp)
cartan_add_test(lm_test lm_test.cpp)
cartan_add_test(lm_kernel_accounting_test lm_kernel_accounting_test.cpp)
cartan_add_test(projected_lm_test projected_lm_test.cpp)
cartan_add_test(projected_lm_metric_test projected_lm_metric_test.cpp)
cartan_add_test(projected_lm_bit_identity_test projected_lm_bit_identity_test.cpp)
target_compile_definitions(projected_lm_bit_identity_test
    PRIVATE CARTAN_PLM_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
cartan_add_test(lm_bit_identity_test lm_bit_identity_test.cpp)
target_compile_definitions(lm_bit_identity_test
    PRIVATE CARTAN_LM_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
option(CARTAN_RUN_BITWISE_GOLDEN
    "Run the lm and projected_lm bitwise golden comparisons — an opt-in single-build refactor guard, since floating-point results are not bit-portable across toolchains" OFF)
if (CARTAN_RUN_BITWISE_GOLDEN)
    target_compile_definitions(projected_lm_bit_identity_test PRIVATE CARTAN_RUN_BITWISE_GOLDEN)
    target_compile_definitions(lm_bit_identity_test PRIVATE CARTAN_RUN_BITWISE_GOLDEN)
endif ()
cartan_add_test(projected_lm_alloc_free_test projected_lm_alloc_free_test.cpp)
# The global operator new/delete counter and Eigen's process-global no-malloc
# flag are shared per process, so this test must not run concurrently with any
# other test that could perturb them.
set_tests_properties(projected_lm_alloc_free_test PROPERTIES RUN_SERIAL TRUE)
cartan_add_test(basic_ik_solver_compile_test basic_ik_solver_compile_test.cpp)
cartan_add_test(basic_ik_solver_test basic_ik_solver_test.cpp)
cartan_add_test(basic_ik_runner_test basic_ik_runner_test.cpp)
cartan_add_test(runner_work_accounting_test runner_work_accounting_test.cpp)
cartan_add_test(bobyqa_solve_policy_test bobyqa_solve_policy_test.cpp)
cartan_add_test(slsqp_solve_policy_test slsqp_solve_policy_test.cpp)
cartan_add_test(lbfgsb_test lbfgsb_test.cpp)
cartan_add_test(restart_wrapper_test restart_wrapper_test.cpp)
cartan_add_test(abort_semantics_test abort_semantics_test.cpp)
cartan_add_test(selection_objective_test selection_objective_test.cpp)
cartan_add_test(zero_dof_semantics_test zero_dof_semantics_test.cpp)
cartan_add_test(honest_diagnostics_test honest_diagnostics_test.cpp)
cartan_add_test(ik_status_test ik_status_test.cpp)
cartan_add_test(singularity_failure_test singularity_failure_test.cpp)
cartan_add_test(spectrum_resolution_floor_test spectrum_resolution_floor_test.cpp)
cartan_add_test(newton_raphson_solve_policy_test newton_raphson_solve_policy_test.cpp)
cartan_add_test(newton_raphson_line_search_test newton_raphson_line_search_test.cpp)
cartan_add_test(default_solvers_test default_solvers_test.cpp)
cartan_add_test(solver_semantics_test solver_semantics_test.cpp)

cartan_add_test(variadic_solver_test variadic_solver_test.cpp)
cartan_add_test(solver_factory_test solver_factory_test.cpp)
cartan_add_test(ik_integration_test ik_integration_test.cpp)
cartan_add_test(halton_seed_generator_test halton_seed_generator_test.cpp)
cartan_add_test(exhaustive_ik_runner_test exhaustive_ik_runner_test.cpp)
cartan_add_test(static_chain_test static_chain_test.cpp)
cartan_add_test(static_chain_ik_parity_test static_chain_ik_parity_test.cpp)
cartan_add_test(specialized_fk_jacobian_test specialized_fk_jacobian_test.cpp)

cartan_add_test(paden_kahan_test paden_kahan_test.cpp)
cartan_add_test(angle_unwrap_test angle_unwrap_test.cpp)
cartan_add_test(analytical_solver_2r_test analytical_solver_2r_test.cpp)
cartan_add_test(analytical_solver_3r_test analytical_solver_3r_test.cpp)
cartan_add_test(analytical_solver_6r_test analytical_solver_6r_test.cpp)
cartan_add_test(opw_forward_reconstruction_test opw_forward_reconstruction_test.cpp)
cartan_add_test(analytical_solver_opw_test analytical_solver_opw_test.cpp)
cartan_add_test(analytical_opw_diagnostic_test analytical_opw_diagnostic_test.cpp)
cartan_add_test(analytical_diagnostic_magnitude_test analytical_diagnostic_magnitude_test.cpp)
cartan_add_test(unwrapped_result_test unwrapped_result_test.cpp)
cartan_add_test(unwrapped_solver_test unwrapped_solver_test.cpp)
cartan_add_test(closest_to_seed_test closest_to_seed_test.cpp)
cartan_add_test(abb_irb120_reconciliation_test abb_irb120_reconciliation_test.cpp)

if(CARTAN_BUILD_ARGMIN)
    cartan_add_test(argmin_adapters_compile_test argmin_adapters_compile_test.cpp)
    cartan_add_test(argmin_policies_test argmin_policies_test.cpp)
    cartan_add_test(argmin_slsqp_restart_test argmin_slsqp_restart_test.cpp)
    cartan_add_test(argmin_projected_gn_test argmin_projected_gn_test.cpp)
    cartan_add_test(argmin_projected_gradient_gn_test argmin_projected_gradient_gn_test.cpp)
    cartan_add_test(solver_unit_count_test solver_unit_count_test.cpp)
    cartan_add_test(argmin_solver_instantiation_test argmin_solver_instantiation_test.cpp)
    cartan_add_test(unbounded_joint_feasibility_test unbounded_joint_feasibility_test.cpp)
    foreach (backend_test IN ITEMS
        argmin_adapters_compile_test
        argmin_policies_test
        argmin_slsqp_restart_test
        argmin_projected_gn_test
        argmin_projected_gradient_gn_test
        argmin_solver_instantiation_test
        unbounded_joint_feasibility_test
        solver_unit_count_test
        exhaustive_ik_runner_test
        solver_semantics_test)
        target_link_libraries(${backend_test} PRIVATE cartan::argmin)
    endforeach ()
endif()

if(CARTAN_BUILD_INSTANTIATION_TESTS)
    cartan_add_test(static_chain_instantiation_test static_chain_instantiation_test.cpp)
    cartan_add_test(native_policy_instantiation_test native_policy_instantiation_test.cpp)
    cartan_add_test(runner_instantiation_test runner_instantiation_test.cpp)
    # Nested rather than listed in the backend block's foreach above: that list
    # names targets which exist unconditionally once the backend is on, and this
    # one exists only when both options are.
    if(CARTAN_BUILD_ARGMIN)
        cartan_add_test(backend_policy_instantiation_test backend_policy_instantiation_test.cpp)
        target_link_libraries(backend_policy_instantiation_test PRIVATE cartan::argmin)
    endif()
endif()

cartan_add_test(fk_sweep_test fk_sweep_test.cpp)
cartan_add_test(jacobian_sweep_test jacobian_sweep_test.cpp)
cartan_add_test(ik_sweep_test ik_sweep_test.cpp)
cartan_add_test(ik_float_tolerance_sweep_test ik_float_tolerance_sweep_test.cpp)

# These two cover the NLopt solve policy, which ships as an example rather than
# as library surface; each compiles to an empty translation unit without it.
if(TARGET cartan_examples::nlopt_policy)
    target_link_libraries(bobyqa_solve_policy_test PRIVATE cartan_examples::nlopt_policy)
    target_link_libraries(slsqp_solve_policy_test PRIVATE cartan_examples::nlopt_policy)
endif()

# The cartan-vs-KDL FK comparison is an external-library comparison and lives in
# benchmarks/, not here: tests/ verify cartan using only cartan.

if (CARTAN_BUILD_URDF)
    cartan_add_test(urdf_sink_test urdf_sink_test.cpp)
    target_link_libraries(urdf_sink_test PRIVATE cartan::urdf)
    target_compile_definitions(urdf_sink_test PRIVATE
        CARTAN_TESTS_FIXTURE_DIR="${CMAKE_SOURCE_DIR}/tests/fixtures")

    cartan_add_test(urdf_document_test urdf_document_test.cpp)
    target_link_libraries(urdf_document_test PRIVATE cartan::urdf)
    target_compile_definitions(urdf_document_test PRIVATE
        CARTAN_TESTS_FIXTURE_DIR="${CMAKE_SOURCE_DIR}/tests/fixtures")

    cartan_add_test(urdf_adversarial_test urdf_adversarial_test.cpp)
    target_link_libraries(urdf_adversarial_test PRIVATE cartan::urdf)
    target_compile_definitions(urdf_adversarial_test PRIVATE
        CARTAN_TESTS_FIXTURE_DIR="${CMAKE_SOURCE_DIR}/tests/fixtures")

    cartan_add_test(urdf_evaluator_test urdf_evaluator_test.cpp)
    target_link_libraries(urdf_evaluator_test PRIVATE cartan::urdf)
    target_compile_definitions(urdf_evaluator_test PRIVATE
        CARTAN_TESTS_FIXTURE_DIR="${CMAKE_SOURCE_DIR}/tests/fixtures")

    cartan_add_test(urdf_diagnostic_test urdf_diagnostic_test.cpp)
    target_link_libraries(urdf_diagnostic_test PRIVATE cartan::urdf)
    target_compile_definitions(urdf_diagnostic_test PRIVATE
        CARTAN_TESTS_FIXTURE_DIR="${CMAKE_SOURCE_DIR}/tests/fixtures")
    if (CARTAN_BUILD_URDF_EXTENDED_TESTS)
        target_compile_definitions(urdf_diagnostic_test PRIVATE CARTAN_URDF_EXTENDED_TESTS)
    endif ()

    cartan_add_test(urdf_chain_extractor_test urdf_chain_extractor_test.cpp)
    target_link_libraries(urdf_chain_extractor_test PRIVATE cartan::urdf)
    target_compile_definitions(urdf_chain_extractor_test PRIVATE
        CARTAN_TESTS_FIXTURE_DIR="${CMAKE_SOURCE_DIR}/tests/fixtures")

    cartan_add_test(urdf_parity_test urdf_parity_test.cpp)
    target_link_libraries(urdf_parity_test PRIVATE cartan::urdf)
    target_compile_definitions(urdf_parity_test PRIVATE
        CARTAN_TESTS_FIXTURE_DIR="${CMAKE_SOURCE_DIR}/tests/fixtures")
    if (CARTAN_BUILD_URDF_EXTENDED_TESTS)
        target_compile_definitions(urdf_parity_test PRIVATE CARTAN_URDF_EXTENDED_TESTS)
    endif ()

    cartan_add_test(solver_continuous_joint_test solver_continuous_joint_test.cpp)
    target_link_libraries(solver_continuous_joint_test PRIVATE cartan::urdf)
    target_compile_definitions(solver_continuous_joint_test PRIVATE
        CARTAN_TESTS_FIXTURE_DIR="${CMAKE_SOURCE_DIR}/tests/fixtures")
    if (CARTAN_BUILD_ARGMIN)
        target_link_libraries(solver_continuous_joint_test PRIVATE cartan::argmin)

        cartan_add_test(continuous_joint_fallback_sweep continuous_joint_fallback_sweep.cpp)
        target_link_libraries(continuous_joint_fallback_sweep PRIVATE cartan::urdf cartan::argmin)
        target_compile_definitions(continuous_joint_fallback_sweep PRIVATE
            CARTAN_TESTS_FIXTURE_DIR="${CMAKE_SOURCE_DIR}/tests/fixtures")
        # Exclude from the default ctest run; invoke directly via
        #   ctest -R continuous_joint_fallback_sweep
        # or run the binary at tests/unit/continuous_joint_fallback_sweep.
        set_tests_properties(continuous_joint_fallback_sweep
            PROPERTIES LABELS "manual" DISABLED TRUE)
    endif ()
endif ()
