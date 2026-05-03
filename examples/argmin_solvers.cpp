/// @file argmin_solvers.cpp
/// @brief Demonstrates argmin-backed SLSQP and BOBYQA solve policies.
///
/// Shows: ik::argmin_slsqp, ik::argmin_bobyqa, restart wrapping,
/// and racing a argmin solver against a native cartan solver.

#include "cartan/serial_chain.h"

#include <iostream>
#include <numbers>

int main()
{
    using vec3 = cartan::vector3<double>;

    // UR3e 6-DOF chain: PoE screw axes in space frame
    auto s1 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(0, 0, 0));
    auto s2 = cartan::screw_axis<double>::revolute(vec3(0, 1, 0), vec3(0, 0, 0.15185));
    auto s3 = cartan::screw_axis<double>::revolute(vec3(0, 1, 0), vec3(-0.24355, 0, 0.15185));
    auto s4 = cartan::screw_axis<double>::revolute(vec3(0, 1, 0), vec3(-0.45675, 0, 0.15185));
    auto s5 = cartan::screw_axis<double>::revolute(vec3(0, 0, -1), vec3(-0.45675, 0.13105, 0));
    auto s6 = cartan::screw_axis<double>::revolute(vec3(0, 1, 0), vec3(-0.45675, 0, -0.08535));

    vec3 home_trans(-0.45675, 0.22315, 0.0665);
    auto home = cartan::se3<double>(cartan::so3<double>::identity(), home_trans);

    cartan::joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};
    cartan::kinematic_chain<double, 6> chain(
        home, {s1, s2, s3, s4, s5, s6},
        {lim, lim, lim, lim, lim, lim});

    // Compute a target by FK at a known configuration
    Eigen::Vector<double, 6> q_known{0.3, -0.5, 0.2, -0.4, 0.1, 0.3};
    auto target = cartan::forward_kinematics(chain, q_known).end_effector;

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    cartan::convergence_criteria<double> criteria{1e-5, 1e-5, 500};

    // --- Section 1: argmin SLSQP with restart wrapping ---

    cartan::basic_ik_runner slsqp_solver{
        cartan::ik::restart_wrapper{
            cartan::ik::argmin_slsqp<cartan::kinematic_chain<double, 6>>{}}};

    slsqp_solver.setup(chain, target, q0, criteria);
    auto slsqp_result = slsqp_solver.solve();

    std::cout << "=== argmin SLSQP + restart ===\n";
    if (slsqp_result.has_value())
    {
        auto& r = slsqp_result.value();
        std::cout << "  Converged in " << r.iterations << " iterations\n";
        std::cout << "  Error: " << r.final_error_norm << "\n";
    }
    else
    {
        std::cout << "  Failed\n";
    }

    // --- Section 2: argmin BOBYQA with restart wrapping ---

    cartan::basic_ik_runner bobyqa_solver{
        cartan::ik::restart_wrapper{
            cartan::ik::argmin_bobyqa<cartan::kinematic_chain<double, 6>>{}}};

    bobyqa_solver.setup(chain, target, q0, criteria);
    auto bobyqa_result = bobyqa_solver.solve();

    std::cout << "\n=== argmin BOBYQA + restart ===\n";
    if (bobyqa_result.has_value())
    {
        auto& r = bobyqa_result.value();
        std::cout << "  Converged in " << r.iterations << " iterations\n";
        std::cout << "  Error: " << r.final_error_norm << "\n";
    }
    else
    {
        std::cout << "  Failed\n";
    }

    // --- Section 3: Racing argmin SLSQP against native projected LM ---

    cartan::basic_ik_runner racing_solver{
        cartan::ik::restart_wrapper{cartan::ik::argmin_slsqp<cartan::kinematic_chain<double, 6>>{}},
        cartan::ik::restart_wrapper{cartan::ik::projected_lm<cartan::kinematic_chain<double, 6>>{}}};

    racing_solver.setup(chain, target, q0, criteria);
    auto racing_result = racing_solver.solve();

    std::cout << "\n=== SLSQP vs projected LM (racing) ===\n";
    if (racing_result.has_value())
    {
        auto& r = racing_result.value();
        std::cout << "  Converged in " << r.iterations << " iterations\n";
        std::cout << "  Winner: policy " << r.solver_index << "\n";
        std::cout << "  Error: " << r.final_error_norm << "\n";
    }
    else
    {
        std::cout << "  Failed\n";
    }
}
