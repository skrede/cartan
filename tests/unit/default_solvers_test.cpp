#include <liepp/serial/ik/ik_types.h>
#include <liepp/serial/ik/limits_policy.h>
#include <liepp/serial/ik/basic_ik_solver.h>
#include <liepp/serial/ik/default_solvers.h>
#include <liepp/serial/ik/ik_solve_policy.h>
#include <liepp/serial/ik/lbfgsb_solve_policy.h>
#include <liepp/serial/ik/restart_solve_policy.h>
#include <liepp/serial/ik/projected_lm_solve_policy.h>

#include <liepp/types.h>

#include <liepp/lie/se3.h>
#include <liepp/lie/so3.h>

#include <liepp/serial/chain/screw_axis.h>
#include <liepp/serial/chain/joint_state.h>
#include <liepp/serial/chain/joint_limits.h>
#include <liepp/serial/chain/kinematic_chain.h>

#include <liepp/serial/fk/forward_kinematics.h>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>

namespace spp = liepp;

// Chain-parameterized aliases from default_solvers.h require using spp:: prefix directly.

// ============================================================================
// Helper: UR5-like 6R chain
// ============================================================================

static spp::kinematic_chain<double, 6> make_ur5_like_chain()
{
    auto s1 = spp::screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s2 = spp::screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0.089});
    auto s3 = spp::screw_axis<double>::revolute({0, 1, 0}, {0.425, 0, 0.089});
    auto s4 = spp::screw_axis<double>::revolute({0, 1, 0}, {0.817, 0, 0.089});
    auto s5 = spp::screw_axis<double>::revolute({0, 0, -1}, {0.817, 0.109, 0});
    auto s6 = spp::screw_axis<double>::revolute({0, 1, 0}, {0.817, 0, -0.006});

    spp::vector3<double> home_trans;
    home_trans << 0.817, 0.191, -0.006;
    auto home = spp::se3<double>(spp::so3<double>::identity(), home_trans);

    spp::joint_limits<double> lim{-2 * std::numbers::pi, 2 * std::numbers::pi};
    return spp::kinematic_chain<double, 6>(home, {s1, s2, s3, s4, s5, s6},
                                  {lim, lim, lim, lim, lim, lim});
}

// ============================================================================
// Helper: generate reachable FK target from known configuration
// ============================================================================

static spp::se3<double> reachable_target(
    const spp::kinematic_chain<double, 6>& chain,
    const Eigen::Vector<double, 6>& q)
{
    return spp::forward_kinematics(chain, q).end_effector;
}

// ============================================================================
// speed_solver: compile and convergence
// ============================================================================

TEST_CASE("speed_solver compiles and converges", "[ik][default_solvers]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;
    auto target = reachable_target(chain, q_known);

    spp::basic_ik_solver solver{spp::speed_solver<spp::kinematic_chain<double, 6>>{}};

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations = 200;

    solver.setup(chain, target, q0, criteria);
    auto result = solver.solve();

    REQUIRE(result.has_value());

    auto fk_sol = spp::forward_kinematics(chain, result->solution.position);
    auto err = (fk_sol.end_effector.inverse() * target).log();
    REQUIRE(err.norm() < 1e-4);
}

// ============================================================================
// convergence_solver: compile and convergence
// ============================================================================

TEST_CASE("convergence_solver compiles and converges", "[ik][default_solvers]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;
    auto target = reachable_target(chain, q_known);

    spp::basic_ik_solver solver{spp::convergence_solver<spp::kinematic_chain<double, 6>>{}};

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations = 200;

    solver.setup(chain, target, q0, criteria);
    auto result = solver.solve();

    REQUIRE(result.has_value());

    auto fk_sol = spp::forward_kinematics(chain, result->solution.position);
    auto err = (fk_sol.end_effector.inverse() * target).log();
    REQUIRE(err.norm() < 1e-4);
}

// ============================================================================
// default_solver: compile and convergence
// ============================================================================

TEST_CASE("default_solver compiles and converges", "[ik][default_solvers]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;
    auto target = reachable_target(chain, q_known);

    spp::default_solver<spp::kinematic_chain<double, 6>> solver;
    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations = 200;

    solver.setup(chain, target, q0, criteria);
    auto result = solver.solve();

    REQUIRE(result.has_value());

    auto fk_sol = spp::forward_kinematics(chain, result->solution.position);
    auto err = (fk_sol.end_effector.inverse() * target).log();
    REQUIRE(err.norm() < 1e-4);
}

// ============================================================================
// Concept satisfaction checks
// ============================================================================

TEST_CASE("speed_solver satisfies ik_solve_policy concept", "[ik][default_solvers]")
{
    static_assert(spp::ik_solve_policy<spp::speed_solver<spp::kinematic_chain<double, 6>>>);
}

TEST_CASE("convergence_solver satisfies ik_solve_policy concept", "[ik][default_solvers]")
{
    static_assert(spp::ik_solve_policy<spp::convergence_solver<spp::kinematic_chain<double, 6>>>);
}

// ============================================================================
// Harder target: tests restart behavior via default_solver racing
// ============================================================================

TEST_CASE("default_solver converges on harder target", "[ik][default_solvers]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 2.5, -1.8, 1.2, -2.0, 1.5, -1.0;
    auto target = reachable_target(chain, q_known);

    spp::default_solver<spp::kinematic_chain<double, 6>> solver;
    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations = 300;

    spp::solver_options<double> opts;
    opts.max_total_iterations = 600;

    solver.setup(chain, target, q0, criteria, opts);
    auto result = solver.solve();

    REQUIRE(result.has_value());

    auto fk_sol = spp::forward_kinematics(chain, result->solution.position);
    auto err = (fk_sol.end_effector.inverse() * target).log();
    REQUIRE(err.norm() < 1e-3);
}
