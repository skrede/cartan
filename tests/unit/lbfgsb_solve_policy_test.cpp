#include <cartan/serial/ik/solver/lbfgsb.h>

#include <cartan/types.h>

#include <cartan/lie/se3.h>
#include <cartan/lie/so3.h>
#include <cartan/serial/ik/ik_status.h>
#include <cartan/serial/ik/policy/error_weight.h>
#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/joint_state.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>
#include <cartan/serial/fk/forward_kinematics.h>

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

TEST_CASE("lbfgsb_solve_policy concept satisfaction", "[ik][lbfgsb]")
{
    static_assert(spp::ik::solve_policy<spp::ik::builtin_lbfgsb<spp::kinematic_chain<double, 6>>>);
}

// ============================================================================
// FK roundtrip convergence
// ============================================================================

TEST_CASE("lbfgsb_solve_policy FK roundtrip", "[ik][lbfgsb]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    spp::ik::builtin_lbfgsb<spp::kinematic_chain<double, 6>> stepper;
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
// Convergence with tight joint limits
// ============================================================================

TEST_CASE("lbfgsb_solve_policy with tight limits", "[ik][lbfgsb]")
{
    // UR5 chain but joint 3 has tight limits [-0.5, 0.5]
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
    auto chain = spp::kinematic_chain<double, 6>(home, {s1, s2, s3, s4, s5, s6},
                                         {wide, wide, tight, wide, wide, wide});

    // Use a known config within limits
    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.4, 0.1, -0.4, 0.7;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    spp::ik::builtin_lbfgsb<spp::kinematic_chain<double, 6>> stepper;
    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations = 300;

    stepper.setup(chain, target, q0, criteria);
    auto status = run_stepper(stepper, chain, 300);

    REQUIRE(status == spp::ik_status::converged);

    // Verify solution respects tight limits on joint 3
    auto sol = stepper.solution();
    REQUIRE(sol(2) >= -0.5);
    REQUIRE(sol(2) <= 0.5);
}

// ============================================================================
// Iterations count (one per step() call, per D-06)
// ============================================================================

TEST_CASE("lbfgsb_solve_policy iterations count", "[ik][lbfgsb]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;
    auto fk_target = spp::forward_kinematics(chain, q_known);

    spp::ik::builtin_lbfgsb<spp::kinematic_chain<double, 6>> stepper;
    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations = 200;

    stepper.setup(chain, fk_target.end_effector, q0, criteria);

    for (int i = 0; i < 5; ++i)
    {
        stepper.step(chain);
    }
    REQUIRE(stepper.iterations() == 5);
}

// ============================================================================
// ObjectivePolicy template works
// ============================================================================

TEST_CASE("lbfgsb_solve_policy ObjectivePolicy template", "[ik][lbfgsb]")
{
    // Verify default ObjectivePolicy is ik_se3_objective
    using chain6 = spp::kinematic_chain<double, 6>;
    static_assert(std::is_same_v<
        spp::ik::builtin_lbfgsb<chain6>,
        spp::ik::builtin_lbfgsb<chain6, spp::clamp_limits, spp::ik_se3_objective<chain6>>>);
}

// ============================================================================
// Convergence with error_weight
// ============================================================================

TEST_CASE("lbfgsb_solve_policy with error weight", "[ik][lbfgsb]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    spp::error_weight<double> weight;
    weight.weights << 1.0, 1.0, 1.0, 2.0, 2.0, 2.0;

    spp::ik::builtin_lbfgsb<spp::kinematic_chain<double, 6>> stepper;
    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations = 200;

    stepper.setup(chain, target, q0, criteria, weight);
    auto status = run_stepper(stepper, chain, 200);

    REQUIRE(status == spp::ik_status::converged);
}

// ============================================================================
// Stall detection
// ============================================================================

TEST_CASE("lbfgsb_solve_policy stall detection", "[ik][lbfgsb]")
{
    // UR5 chain with very tight limits on all joints -- unreachable target
    auto s1 = spp::screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s2 = spp::screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0.089});
    auto s3 = spp::screw_axis<double>::revolute({0, 1, 0}, {0.425, 0, 0.089});
    auto s4 = spp::screw_axis<double>::revolute({0, 1, 0}, {0.817, 0, 0.089});
    auto s5 = spp::screw_axis<double>::revolute({0, 0, -1}, {0.817, 0.109, 0});
    auto s6 = spp::screw_axis<double>::revolute({0, 1, 0}, {0.817, 0, -0.006});

    spp::vector3<double> home_trans;
    home_trans << 0.817, 0.191, -0.006;
    auto home = spp::se3<double>(spp::so3<double>::identity(), home_trans);

    spp::joint_limits<double> tight{-0.01, 0.01};
    auto chain = spp::kinematic_chain<double, 6>(home, {s1, s2, s3, s4, s5, s6},
                                         {tight, tight, tight, tight, tight, tight});

    // Target far from home config -- unreachable with tight limits
    Eigen::Vector<double, 6> q_far;
    q_far << 1.0, -1.0, 1.0, 0.5, -0.5, 0.5;
    auto fk_target_chain = make_ur5_like_chain();
    auto fk_target = spp::forward_kinematics(fk_target_chain, q_far);
    auto target = fk_target.end_effector;

    spp::ik::builtin_lbfgsb<spp::kinematic_chain<double, 6>> stepper;
    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations = 200;

    stepper.setup(chain, target, q0, criteria);
    auto status = run_stepper(stepper, chain, 200);

    // Should stall or hit iteration limit -- definitely not converge
    REQUIRE(status != spp::ik_status::converged);
    REQUIRE((status == spp::ik_status::stalled ||
             status == spp::ik_status::iteration_limit));
}
