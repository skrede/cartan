#include <liepp/serial/ik/ik_types.h>
#include <liepp/serial/ik/limits_policy.h>
#include <liepp/serial/ik/basic_ik_solver.h>
#include <liepp/serial/ik/default_solvers.h>
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
#include <concepts>
#include <numbers>

namespace spp = liepp;

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
// make_speed_solver().build() compiles and converges
// ============================================================================

TEST_CASE("make_speed_solver().build() compiles and converges", "[ik][solver_factory]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;
    auto target = reachable_target(chain, q_known);

    auto solver = spp::make_speed_solver<spp::kinematic_chain<double, 6>>().build();

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 200};

    solver.setup(chain, target, q0, criteria);
    auto result = solver.solve();

    REQUIRE(result.has_value());

    auto fk_sol = spp::forward_kinematics(chain, result->solution.position);
    auto err = (fk_sol.end_effector.inverse() * target).log();
    REQUIRE(err.norm() < 1e-4);
}

// ============================================================================
// make_convergence_solver().build() compiles and converges
// ============================================================================

TEST_CASE("make_convergence_solver().build() compiles and converges", "[ik][solver_factory]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;
    auto target = reachable_target(chain, q_known);

    auto solver = spp::make_convergence_solver<spp::kinematic_chain<double, 6>>().build();

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 200};

    solver.setup(chain, target, q0, criteria);
    auto result = solver.solve();

    REQUIRE(result.has_value());

    auto fk_sol = spp::forward_kinematics(chain, result->solution.position);
    auto err = (fk_sol.end_effector.inverse() * target).log();
    REQUIRE(err.norm() < 1e-4);
}

// ============================================================================
// make_default_solver().build() compiles and converges
// ============================================================================

TEST_CASE("make_default_solver().build() compiles and converges", "[ik][solver_factory]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;
    auto target = reachable_target(chain, q_known);

    auto solver = spp::make_default_solver<spp::kinematic_chain<double, 6>>().build();

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 200};

    solver.setup(chain, target, q0, criteria);
    auto result = solver.solve();

    REQUIRE(result.has_value());

    auto fk_sol = spp::forward_kinematics(chain, result->solution.position);
    auto err = (fk_sol.end_effector.inverse() * target).log();
    REQUIRE(err.norm() < 1e-4);
}

// ============================================================================
// make_solver builder constructs single-policy solver
// ============================================================================

TEST_CASE("make_solver builder constructs single-policy solver", "[ik][solver_factory]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;
    auto target = reachable_target(chain, q_known);

    auto solver = spp::make_solver<spp::kinematic_chain<double, 6>>()
        .policy(spp::speed_solver<spp::kinematic_chain<double, 6>>{})
        .build();

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 200};

    solver.setup(chain, target, q0, criteria);
    auto result = solver.solve();

    REQUIRE(result.has_value());

    auto fk_sol = spp::forward_kinematics(chain, result->solution.position);
    auto err = (fk_sol.end_effector.inverse() * target).log();
    REQUIRE(err.norm() < 1e-4);
}

// ============================================================================
// make_solver builder constructs two-policy solver
// ============================================================================

TEST_CASE("make_solver builder constructs two-policy solver", "[ik][solver_factory]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;
    auto target = reachable_target(chain, q_known);

    auto solver = spp::make_solver<spp::kinematic_chain<double, 6>>()
        .policy(spp::speed_solver<spp::kinematic_chain<double, 6>>{})
        .policy(spp::convergence_solver<spp::kinematic_chain<double, 6>>{})
        .build();

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 200};

    solver.setup(chain, target, q0, criteria);
    auto result = solver.solve();

    REQUIRE(result.has_value());

    auto fk_sol = spp::forward_kinematics(chain, result->solution.position);
    auto err = (fk_sol.end_effector.inverse() * target).log();
    REQUIRE(err.norm() < 1e-4);
}

// ============================================================================
// default_solver alias matches make_default_solver().build() type
// ============================================================================

TEST_CASE("default_solver alias matches make_default_solver().build() type", "[ik][solver_factory]")
{
    static_assert(std::same_as<
        spp::default_solver<spp::kinematic_chain<double, 6>>,
        decltype(spp::make_default_solver<spp::kinematic_chain<double, 6>>().build())>);
}

// ============================================================================
// preset builders are not solvers -- .build() required
// ============================================================================

TEST_CASE("preset builders are not solvers -- .build() required", "[ik][solver_factory]")
{
    static_assert(!std::same_as<
        decltype(spp::make_speed_solver<spp::kinematic_chain<double, 6>>()),
        spp::basic_ik_solver<spp::speed_solver<spp::kinematic_chain<double, 6>>>>);
}
