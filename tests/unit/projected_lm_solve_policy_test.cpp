#include <liepp/serial/ik/projected_lm_solve_policy.h>

#include <liepp/types.h>

#include <liepp/lie/se3.h>
#include <liepp/lie/so3.h>
#include <liepp/serial/chain/screw_axis.h>
#include <liepp/serial/chain/joint_state.h>
#include <liepp/serial/chain/joint_limits.h>
#include <liepp/serial/chain/kinematic_chain.h>
#include <liepp/serial/fk/forward_kinematics.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>

namespace spp = liepp;
using Catch::Approx;

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
// Helper: UR5 with tight limits on joint 3
// ============================================================================

static spp::kinematic_chain<double, 6> make_ur5_tight_limits_chain()
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

    spp::joint_limits<double> wide{-2 * std::numbers::pi, 2 * std::numbers::pi};
    spp::joint_limits<double> tight{-0.5, 0.5};
    return spp::kinematic_chain<double, 6>(home, {s1, s2, s3, s4, s5, s6},
                                  {wide, wide, tight, wide, wide, wide});
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

TEST_CASE("projected_lm_solve_policy concept satisfaction", "[ik][projected_lm]")
{
    static_assert(spp::ik_solve_policy<spp::projected_lm_solve_policy<spp::kinematic_chain<double, 6>>>);
}

// ============================================================================
// FK roundtrip convergence
// ============================================================================

TEST_CASE("projected_lm_solve_policy FK roundtrip", "[ik][projected_lm]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    spp::projected_lm_solve_policy<spp::kinematic_chain<double, 6>> stepper;
    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
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
// Respects tight joint limits
// ============================================================================

TEST_CASE("projected_lm_solve_policy respects tight limits", "[ik][projected_lm]")
{
    auto chain = make_ur5_tight_limits_chain();

    // Use a configuration where joint 3 is within tight limits
    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.4, 0.1, -0.4, 0.7;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    spp::projected_lm_solve_policy<spp::kinematic_chain<double, 6>> stepper;
    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations = 300;

    stepper.setup(chain, target, q0, criteria);
    auto status = run_stepper(stepper, chain, 300);

    REQUIRE(status == spp::ik_status::converged);

    // Joint 3 must be within tight limits [-0.5, 0.5]
    auto sol = stepper.solution();
    REQUIRE(sol(2) >= -0.5 - 1e-10);
    REQUIRE(sol(2) <= 0.5 + 1e-10);
}

// ============================================================================
// Active set: joints at limits with gradient pushing outward are held fixed
// ============================================================================

TEST_CASE("projected_lm_solve_policy active set holds joints at limits", "[ik][projected_lm]")
{
    auto chain = make_ur5_tight_limits_chain();

    // Seed at exactly the limit on joint 3
    Eigen::Vector<double, 6> q_known;
    q_known << 0.2, -0.3, 0.3, 0.1, -0.2, 0.5;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    spp::projected_lm_solve_policy<spp::kinematic_chain<double, 6>> stepper;
    Eigen::Vector<double, 6> q0;
    q0 << 0.0, 0.0, 0.5, 0.0, 0.0, 0.0;  // joint 3 at upper limit

    spp::convergence_criteria<double> criteria;
    criteria.max_iterations = 300;

    stepper.setup(chain, target, q0, criteria);
    run_stepper(stepper, chain, 300);

    // Solution must respect joint 3 limits regardless of convergence
    auto sol = stepper.solution();
    REQUIRE(sol(2) >= -0.5 - 1e-10);
    REQUIRE(sol(2) <= 0.5 + 1e-10);
}

// ============================================================================
// Dogleg option converges
// ============================================================================

TEST_CASE("projected_lm_solve_policy dogleg converges", "[ik][projected_lm]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    spp::projected_lm_solve_policy<spp::kinematic_chain<double, 6>>::options opts;
    opts.use_dogleg = true;
    spp::projected_lm_solve_policy<spp::kinematic_chain<double, 6>> stepper(opts);

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations = 200;

    stepper.setup(chain, target, q0, criteria);
    auto status = run_stepper(stepper, chain, 200);

    REQUIRE(status == spp::ik_status::converged);

    auto fk_sol = spp::forward_kinematics(chain, stepper.solution());
    auto err = (fk_sol.end_effector.inverse() * target).log();
    REQUIRE(err.head<3>().norm() < 1e-6);
    REQUIRE(err.tail<3>().norm() < 1e-6);
}

// ============================================================================
// Error weight affects convergence
// ============================================================================

TEST_CASE("projected_lm_solve_policy with error weight", "[ik][projected_lm]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    // Weight emphasizing position (linear part) over orientation
    spp::error_weight<double> weight;
    weight.weights << 1.0, 1.0, 1.0, 100.0, 100.0, 100.0;

    spp::projected_lm_solve_policy<spp::kinematic_chain<double, 6>> stepper;
    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations = 200;

    stepper.setup(chain, target, q0, criteria, weight);
    auto status = run_stepper(stepper, chain, 200);

    REQUIRE(status == spp::ik_status::converged);

    auto fk_sol = spp::forward_kinematics(chain, stepper.solution());
    auto err = (fk_sol.end_effector.inverse() * target).log();
    REQUIRE(err.head<3>().norm() < 1e-6);
    REQUIRE(err.tail<3>().norm() < 1e-6);
}

// ============================================================================
// Stall detection
// ============================================================================

TEST_CASE("projected_lm_solve_policy stall detection", "[ik][projected_lm]")
{
    auto chain = make_ur5_like_chain();

    // Use unreachable target -- far away
    spp::vector3<double> far_trans;
    far_trans << 100.0, 100.0, 100.0;
    auto target = spp::se3<double>(spp::so3<double>::identity(), far_trans);

    spp::projected_lm_solve_policy<spp::kinematic_chain<double, 6>>::options opts;
    opts.stall_threshold = 1e-4;  // generous threshold to trigger stall
    opts.stall_window = 3;
    spp::projected_lm_solve_policy<spp::kinematic_chain<double, 6>> stepper(opts);

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations = 500;

    stepper.setup(chain, target, q0, criteria);
    auto status = run_stepper(stepper, chain, 500);

    // Should stall, diverge, or hit iteration limit -- NOT converge
    REQUIRE(status != spp::ik_status::converged);
    REQUIRE((status == spp::ik_status::stalled ||
             status == spp::ik_status::diverged ||
             status == spp::ik_status::iteration_limit));
}

// ============================================================================
// Divergence detection
// ============================================================================

TEST_CASE("projected_lm_solve_policy divergence detection", "[ik][projected_lm]")
{
    auto chain = make_ur5_like_chain();

    spp::vector3<double> far_trans;
    far_trans << 100.0, 100.0, 100.0;
    auto target = spp::se3<double>(spp::so3<double>::identity(), far_trans);

    spp::projected_lm_solve_policy<spp::kinematic_chain<double, 6>>::options opts;
    opts.divergence_factor = 1.01;  // very tight -- triggers divergence quickly
    spp::projected_lm_solve_policy<spp::kinematic_chain<double, 6>> stepper(opts);

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations = 200;

    stepper.setup(chain, target, q0, criteria);
    auto status = run_stepper(stepper, chain, 200);

    REQUIRE(status != spp::ik_status::converged);
}
