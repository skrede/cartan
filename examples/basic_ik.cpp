/// @file basic_ik.cpp
/// @brief Single IK solve with Levenberg-Marquardt solver on a 3-DOF planar arm.
///
/// Shows: kinematic_chain construction, basic_ik_runner with ik::lm,
/// std::expected result handling, FK verification of the IK solution.

#include "cartan/serial_chain.h"

#include <iostream>
#include <numbers>

int main()
{
    using vec3 = cartan::vector3<double>;

    // 3-DOF planar arm: three revolute joints about z, unit link lengths
    auto s1 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(0, 0, 0));
    auto s2 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(1, 0, 0));
    auto s3 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(2, 0, 0));

    vec3 home_trans(3, 0, 0);
    auto home = cartan::se3<double>(cartan::so3<double>::identity(), home_trans);

    cartan::joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};
    cartan::kinematic_chain<double, 3> chain(home, {s1, s2, s3}, {lim, lim, lim});

    // Compute a target by FK at a known configuration
    Eigen::Vector3d q_known{0.3, -0.5, 0.2};
    auto fk_target = cartan::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    // Solve IK from a different initial guess
    Eigen::Vector3d q0{0.0, 0.0, 0.0};
    cartan::convergence_criteria<double> criteria{1e-6, 1e-6, 200};

    cartan::basic_ik_runner<cartan::ik::lm<cartan::kinematic_chain<double, 3>>> solver;
    solver.setup(chain, target, q0, criteria);
    auto result = solver.solve();

    if (result.has_value())
    {
        auto& sol = result.value();
        std::cout << "IK converged in " << sol.iterations << " iterations\n";
        std::cout << "Solution: " << sol.solution.position.transpose() << "\n";
        std::cout << "Error:    " << sol.final_error_norm << "\n";

        // Verify: FK of solution should match target
        auto fk_verify = cartan::forward_kinematics(chain, sol.solution.position);
        auto err = (fk_verify.end_effector.inverse() * target).log();
        std::cout << "FK verification error: " << err.norm() << "\n";
    }
    else
    {
        std::cout << "IK failed\n";
    }
}
