/// @file ik_composition.cpp
/// @brief Solver composition: multi-policy racing and factory functions.
///
/// Shows: variadic basic_ik_solver with two policies (cooperative racing),
/// preset factory functions with .build(), and the composable solver builder.

#include "cartan/serial_chain.h"

#include <iostream>
#include <numbers>

int main()
{
    using vec3 = cartan::vector3<double>;

    // LBR iiwa 7-DOF: PoE screw axes in space frame
    auto s1 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(0, 0, 0));
    auto s2 = cartan::screw_axis<double>::revolute(vec3(0, 1, 0), vec3(0, 0, 0.360));
    auto s3 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(0, 0, 0.360));
    auto s4 = cartan::screw_axis<double>::revolute(vec3(0, -1, 0), vec3(0, 0, 0.780));
    auto s5 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(0, 0, 0.780));
    auto s6 = cartan::screw_axis<double>::revolute(vec3(0, 1, 0), vec3(0, 0, 1.180));
    auto s7 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(0, 0, 1.180));

    vec3 home_trans(0, 0, 1.306);
    auto home = cartan::se3<double>(cartan::so3<double>::identity(), home_trans);

    cartan::joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};
    cartan::kinematic_chain<double, 7> chain(
        home, {s1, s2, s3, s4, s5, s6, s7},
        {lim, lim, lim, lim, lim, lim, lim});

    // Target via FK at known configuration
    Eigen::Vector<double, 7> q_known{0.2, -0.3, 0.1, -0.5, 0.4, -0.2, 0.3};
    auto target = cartan::forward_kinematics(chain, q_known).end_effector;

    cartan::convergence_criteria<double> criteria{1e-6, 1e-6, 200};
    Eigen::Vector<double, 7> q0 = Eigen::Vector<double, 7>::Zero();

    // --- Multi-policy solver: races speed + convergence ---
    {
        auto solver = cartan::basic_ik_solver{
            cartan::speed_solver<cartan::kinematic_chain<double, 7>>{},
            cartan::convergence_solver<cartan::kinematic_chain<double, 7>>{}
        };
        solver.setup(chain, target, q0, criteria);
        auto result = solver.solve();
        if (result.has_value())
        {
            std::cout << "Racing: converged in " << result->iterations
                      << " iterations (policy " << result->solver_index << " won)\n";
            std::cout << "  Solution: " << result->solution.position.transpose() << "\n";
            std::cout << "  Error:    " << result->final_error_norm << "\n";
        }
        else
        {
            std::cout << "Racing: failed\n";
        }
    }

    // --- Factory function: make_default_solver().build() ---
    {
        auto solver = cartan::make_default_solver<cartan::kinematic_chain<double, 7>>().build();
        solver.setup(chain, target, q0, criteria);
        auto result = solver.solve();
        if (result.has_value())
        {
            std::cout << "Factory (default): converged in " << result->iterations << " iterations\n";
        }
    }

    // --- Single-policy presets with .build() ---
    {
        auto solver = cartan::make_speed_solver<cartan::kinematic_chain<double, 7>>().build();
        solver.setup(chain, target, q0, criteria);
        auto result = solver.solve();
        if (result.has_value())
        {
            std::cout << "Factory (speed): converged in " << result->iterations << " iterations\n";
        }
    }
    {
        auto solver = cartan::make_convergence_solver<cartan::kinematic_chain<double, 7>>().build();
        solver.setup(chain, target, q0, criteria);
        auto result = solver.solve();
        if (result.has_value())
        {
            std::cout << "Factory (convergence): converged in " << result->iterations << " iterations\n";
        }
    }

    // --- Composable builder: make_solver ---
    {
        auto solver = cartan::make_solver<cartan::kinematic_chain<double, 7>>()
            .policy(cartan::speed_solver<cartan::kinematic_chain<double, 7>>{})
            .policy(cartan::convergence_solver<cartan::kinematic_chain<double, 7>>{})
            .build();
        solver.setup(chain, target, q0, criteria);
        auto result = solver.solve();
        if (result.has_value())
        {
            std::cout << "Builder: converged in " << result->iterations << " iterations\n";
        }
    }
}
