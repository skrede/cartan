/// @file nlopt_policy_example.cpp
/// @brief Driving a custom solve policy that wraps a third-party optimizer.
///
/// Shows: a solve policy living outside namespace cartan, satisfying
/// cartan::solve_policy, and being driven both by basic_ik_runner::solve() and
/// one work unit at a time through step().
///
/// The policy itself is in include/cartan_examples/nlopt/. What makes it worth
/// reading is that NLopt only offers run-to-completion optimization: the adapter
/// bounds each call with set_maxeval and re-enters from the previous iterate, so
/// a black-box optimizer still presents cartan's stepped, budgeted interface.

#include "cartan_examples/nlopt/nlopt_slsqp.h"

#include "cartan/serial_chain.h"

#include <iostream>
#include <numbers>

int main()
{
    using vec3 = cartan::vector3<double>;
    using chain_t = cartan::kinematic_chain<double, 3>;

    static_assert(cartan::solve_policy<cartan_examples::nlopt_slsqp<chain_t>>,
        "the example policy must satisfy the same concept the built-in ones do");

    auto s1 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(0, 0, 0));
    auto s2 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(1, 0, 0));
    auto s3 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(2, 0, 0));

    auto home = cartan::se3<double>(cartan::so3<double>::identity(), vec3(3, 0, 0));

    auto lim = cartan::joint_limits<double>::make(-std::numbers::pi, std::numbers::pi);
    if (!lim.has_value())
    {
        std::cerr << "joint limits rejected: " << cartan::message(lim.error()) << "\n";
        return 1;
    }

    chain_t chain(home, {s1, s2, s3}, {*lim, *lim, *lim});

    Eigen::Vector3d q_known{0.3, -0.5, 0.2};
    auto fk_target = cartan::forward_kinematics(chain, q_known);
    if (!fk_target.has_value())
    {
        std::cerr << "forward kinematics rejected q_known: "
                  << cartan::message(fk_target.error()) << "\n";
        return 1;
    }
    auto target = fk_target->end_effector;

    Eigen::Vector3d q0{0.0, 0.0, 0.0};
    cartan::convergence_criteria<double> criteria{1e-6, 1e-6, 200};

    cartan::basic_ik_runner<cartan_examples::nlopt_slsqp<chain_t>> solver;
    solver.setup(chain, target, q0, criteria);
    auto result = solver.solve();

    if (!result.has_value())
    {
        std::cerr << "IK refused: " << cartan::message(result.error().reason) << "\n";
        return 1;
    }

    std::cout << "solved in " << result->iterations << " work unit(s), error "
              << result->final_error_norm << "\n";
    std::cout << "verified: "
              << (cartan::verify_solution(chain, target, result->solution.position, criteria)
                      ? "yes"
                      : "no")
              << "\n";

    // The same policy driven one work unit at a time. A caller with a deadline
    // spends units until it runs out rather than handing control to the
    // optimizer and waiting for it to finish.
    cartan_examples::nlopt_slsqp<chain_t> stepped;
    stepped.setup(chain, target, q0, criteria);
    for (int unit = 1; unit <= 8 && !stepped.converged(); ++unit)
    {
        stepped.step(chain, 1);
        std::cout << "  unit " << unit << ": error " << stepped.error_norm() << "\n";
    }

    return 0;
}
