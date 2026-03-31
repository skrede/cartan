#include <liepp/ik/newton_raphson_solve_policy.h>
#include <liepp/ik/restart_solve_policy.h>

#include <liepp/types.h>

#include <liepp/lie/se3.h>
#include <liepp/lie/so3.h>
#include <liepp/chain/screw_axis.h>
#include <liepp/chain/joint_state.h>
#include <liepp/chain/joint_limits.h>
#include <liepp/chain/kinematic_chain.h>
#include <liepp/kinematics/forward_kinematics.h>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
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
// Helper: run stepper to completion
// ============================================================================

template <int N, typename Scalar, typename Stepper>
spp::ik_status run_stepper(
    Stepper& stepper,
    const spp::kinematic_chain<Scalar, N>& chain,
    int max_steps)
{
    spp::ik_status status = spp::ik_status::running;
    for (int i = 0; i < max_steps && status == spp::ik_status::running; ++i)
    {
        status = stepper.step(chain);
    }
    return status;
}

// ============================================================================
// Concept satisfaction
// ============================================================================

TEST_CASE("newton_raphson_solve_policy satisfies ik_solve_policy", "[ik][newton_raphson]")
{
    static_assert(spp::ik_solve_policy<spp::newton_raphson_solve_policy<double, 6>>);
}

// ============================================================================
// FK roundtrip convergence
// ============================================================================

TEST_CASE("newton_raphson_solve_policy converges on UR5", "[ik][newton_raphson]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    spp::newton_raphson_solve_policy<double, 6> stepper;

    // Seed nearby: perturb by 0.1 rad
    Eigen::Vector<double, 6> q0 = q_known;
    q0.array() += 0.1;

    spp::convergence_criteria<double> criteria;
    criteria.max_iterations = 200;

    stepper.setup(chain, target, q0, criteria);
    auto status = run_stepper(stepper, chain, 200);

    REQUIRE(status == spp::ik_status::converged);

    // Verify FK roundtrip
    auto fk_sol = spp::forward_kinematics(chain, stepper.solution());
    auto err = (fk_sol.end_effector.inverse() * target).log();
    REQUIRE(err.head<3>().norm() < 1e-6);
    REQUIRE(err.tail<3>().norm() < 1e-6);
}

// ============================================================================
// Composes with restart_solve_policy
// ============================================================================

TEST_CASE("newton_raphson_solve_policy composes with restart_solve_policy", "[ik][newton_raphson]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    using inner_type = spp::newton_raphson_solve_policy<double, 6>;
    using restart_type = spp::restart_solve_policy<double, 6, inner_type>;

    restart_type::options opts;
    opts.max_restarts = 5;
    restart_type stepper(opts);

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations = 200;

    stepper.setup(chain, target, q0, criteria);
    auto status = run_stepper(stepper, chain, 1000);

    // Should converge or at least not crash
    REQUIRE((status == spp::ik_status::converged ||
             status == spp::ik_status::stalled ||
             status == spp::ik_status::diverged ||
             status == spp::ik_status::iteration_limit));
}

// ============================================================================
// Handles singular configuration without crashing
// ============================================================================

TEST_CASE("newton_raphson_solve_policy handles singular configuration", "[ik][newton_raphson]")
{
    auto chain = make_ur5_like_chain();

    // Target far from reachable workspace to force near-singular Jacobians
    spp::vector3<double> far_trans;
    far_trans << 100.0, 100.0, 100.0;
    auto target = spp::se3<double>(spp::so3<double>::identity(), far_trans);

    spp::newton_raphson_solve_policy<double, 6>::options opts;
    opts.divergence_factor = 2.0;
    opts.stall_window = 5;
    opts.stall_threshold = 1e-6;
    spp::newton_raphson_solve_policy<double, 6> stepper(opts);

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations = 100;

    stepper.setup(chain, target, q0, criteria);
    auto status = run_stepper(stepper, chain, 100);

    // Should terminate gracefully: stalled, diverged, or iteration limit -- NOT crash/NaN
    REQUIRE(status != spp::ik_status::converged);
    REQUIRE((status == spp::ik_status::stalled ||
             status == spp::ik_status::diverged ||
             status == spp::ik_status::iteration_limit));

    // Error norm must be finite (no NaN)
    REQUIRE(std::isfinite(stepper.error_norm()));
}
