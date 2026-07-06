#include "../fixtures/redundant_chains.h"

#include <cartan/serial/ik/solver/projected_lm.h>

#include <cartan/types.h>

#include <cartan/lie/se3.h>
#include <cartan/lie/so3.h>
#include <cartan/serial/fk/jacobian.h>
#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/joint_state.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>
#include <cartan/serial/fk/forward_kinematics.h>
#include <cartan/serial/ik/policy/limits_policy.h>
#include <cartan/serial/ik/detail/limit_enforcement.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>

namespace spp = cartan;
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
        status = stepper.step(chain, 1).status;
    }
    return status;
}

// ============================================================================
// Null-space limit enforcement stays in the Jacobian kernel (redundant chain)
// ============================================================================

TEST_CASE("null-space limit step stays in the Jacobian kernel", "[ik][limits][nullspace][redundant]")
{
    // A 7-DOF chain has a 6x7 body Jacobian with a one-dimensional null space.
    // The null-space limit-enforcement step must move the joints only along
    // that kernel, so the induced first-order end-effector velocity J_b * dq
    // is at machine-noise level. A thin-V SVD hands back a row-space direction
    // instead, giving a per-unit residual ||J_b . v_hat|| ~ O(0.1).
    auto chain = spp::fixtures::make_redundant_7r_chain_dynamic<double>();
    const int n = chain.num_joints();
    REQUIRE(n == 7);

    // A configuration well inside the joint limits so the trailing safety
    // clamp in enforce_limits is a no-op and dq is purely the null-space step.
    Eigen::VectorXd q(n);
    q << 0.35, -0.55, 0.42, 0.90, -0.30, 0.65, -0.50;

    auto fk = spp::forward_kinematics(chain, q);
    auto J_b = spp::body_jacobian(chain, fk);

    Eigen::VectorXd q_before = q;
    spp::detail::enforce_limits<spp::null_space_limits>(q, chain);
    Eigen::VectorXd dq = q - q_before;

    // The off-center configuration yields a non-zero midpoint gradient, so the
    // projected step must be a real motion for the residual test to have teeth.
    REQUIRE(dq.norm() > 1e-6);

    // The step must lie in the Jacobian kernel: the per-unit task velocity is
    // the residual we gate on.
    const double residual = (J_b * dq).norm() / dq.norm();
    REQUIRE(residual < 1e-10);
}

// ============================================================================
// Concept satisfaction
// ============================================================================

TEST_CASE("projected_lm concept satisfaction", "[ik][projected_lm]")
{
    static_assert(spp::solve_policy<spp::projected_lm<spp::kinematic_chain<double, 6>>>);
}

// ============================================================================
// FK roundtrip convergence
// ============================================================================

TEST_CASE("projected_lm FK roundtrip", "[ik][projected_lm]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    spp::projected_lm<spp::kinematic_chain<double, 6>> stepper;
    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations_per_attempt = 200;
    criteria.max_total_work_units = 400;

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

TEST_CASE("projected_lm respects tight limits", "[ik][projected_lm]")
{
    auto chain = make_ur5_tight_limits_chain();

    // Use a configuration where joint 3 is within tight limits
    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.4, 0.1, -0.4, 0.7;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    spp::projected_lm<spp::kinematic_chain<double, 6>> stepper;
    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations_per_attempt = 300;
    criteria.max_total_work_units = 600;

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

TEST_CASE("projected_lm active set holds joints at limits", "[ik][projected_lm]")
{
    auto chain = make_ur5_tight_limits_chain();

    // Seed at exactly the limit on joint 3
    Eigen::Vector<double, 6> q_known;
    q_known << 0.2, -0.3, 0.3, 0.1, -0.2, 0.5;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    spp::projected_lm<spp::kinematic_chain<double, 6>> stepper;
    Eigen::Vector<double, 6> q0;
    q0 << 0.0, 0.0, 0.5, 0.0, 0.0, 0.0;  // joint 3 at upper limit

    spp::convergence_criteria<double> criteria;
    criteria.max_iterations_per_attempt = 300;
    criteria.max_total_work_units = 600;

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

TEST_CASE("projected_lm dogleg converges", "[ik][projected_lm]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    spp::projected_lm<spp::kinematic_chain<double, 6>>::options opts;
    opts.use_dogleg = true;
    spp::projected_lm<spp::kinematic_chain<double, 6>> stepper(opts);

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations_per_attempt = 200;
    criteria.max_total_work_units = 400;

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

TEST_CASE("projected_lm with error weight", "[ik][projected_lm]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    // Weight emphasizing position (linear part) over orientation
    spp::error_weight<double> weight;
    weight.weights << 1.0, 1.0, 1.0, 100.0, 100.0, 100.0;

    spp::projected_lm<spp::kinematic_chain<double, 6>> stepper;
    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations_per_attempt = 200;
    criteria.max_total_work_units = 400;

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

TEST_CASE("projected_lm stall detection", "[ik][projected_lm]")
{
    auto chain = make_ur5_like_chain();

    // Use unreachable target -- far away
    spp::vector3<double> far_trans;
    far_trans << 100.0, 100.0, 100.0;
    auto target = spp::se3<double>(spp::so3<double>::identity(), far_trans);

    spp::projected_lm<spp::kinematic_chain<double, 6>>::options opts;
    opts.stall_threshold = 1e-4;  // generous threshold to trigger stall
    opts.stall_window = 3;
    spp::projected_lm<spp::kinematic_chain<double, 6>> stepper(opts);

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations_per_attempt = 500;
    criteria.max_total_work_units = 1000;

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

TEST_CASE("projected_lm divergence detection", "[ik][projected_lm]")
{
    auto chain = make_ur5_like_chain();

    spp::vector3<double> far_trans;
    far_trans << 100.0, 100.0, 100.0;
    auto target = spp::se3<double>(spp::so3<double>::identity(), far_trans);

    spp::projected_lm<spp::kinematic_chain<double, 6>>::options opts;
    opts.divergence_factor = 1.01;  // very tight -- triggers divergence quickly
    spp::projected_lm<spp::kinematic_chain<double, 6>> stepper(opts);

    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations_per_attempt = 200;
    criteria.max_total_work_units = 400;

    stepper.setup(chain, target, q0, criteria);
    auto status = run_stepper(stepper, chain, 200);

    REQUIRE(status != spp::ik_status::converged);
}
