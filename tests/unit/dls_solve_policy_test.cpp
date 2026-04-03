#include <cartan/serial/ik/dls_solve_policy.h>

#include <cartan/types.h>

#include <cartan/lie/se3.h>
#include <cartan/lie/so3.h>
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
// Helper: UR5-like 6R chain (per RESEARCH.md)
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
// Helper: 3R planar chain
// ============================================================================

static spp::kinematic_chain<double, 3> make_3r_planar_chain()
{
    auto s1 = spp::screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s2 = spp::screw_axis<double>::revolute({0, 0, 1}, {1, 0, 0});
    auto s3 = spp::screw_axis<double>::revolute({0, 0, 1}, {2, 0, 0});

    spp::vector3<double> home_trans;
    home_trans << 3, 0, 0;
    auto home = spp::se3<double>(spp::so3<double>::identity(), home_trans);

    spp::joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};
    return spp::kinematic_chain<double, 3>(home, {s1, s2, s3}, {lim, lim, lim});
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
// DLS converges on reachable 6R target
// ============================================================================

TEST_CASE("DLS converges on reachable 6R target", "[ik][dls]")
{
    auto chain = make_ur5_like_chain();

    // Known configuration -> FK -> use as IK target
    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    // Solve IK from zero seed
    spp::dls_solve_policy<spp::kinematic_chain<double, 6>> stepper;
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
// DLS converges on 3R planar target
// ============================================================================

TEST_CASE("DLS converges on 3R planar target", "[ik][dls]")
{
    auto chain = make_3r_planar_chain();

    Eigen::Vector3d q_known;
    q_known << 0.5, -0.3, 0.7;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    spp::dls_solve_policy<spp::kinematic_chain<double, 3>> stepper;
    Eigen::Vector3d q0 = Eigen::Vector3d::Zero();
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
// DLS returns iteration_limit on unreachable target
// ============================================================================

TEST_CASE("DLS returns iteration_limit on unreachable target", "[ik][dls]")
{
    auto chain = make_ur5_like_chain();

    // Target far outside workspace
    spp::vector3<double> far_trans;
    far_trans << 100, 100, 100;
    auto target = spp::se3<double>(spp::so3<double>::identity(), far_trans);

    spp::dls_solve_policy<spp::kinematic_chain<double, 6>> stepper;
    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations = 50;

    stepper.setup(chain, target, q0, criteria);
    auto status = run_stepper(stepper, chain, 50);

    REQUIRE(status != spp::ik_status::converged);
}

// ============================================================================
// DLS near-singular convergence
// ============================================================================

TEST_CASE("DLS near-singular convergence", "[ik][dls]")
{
    auto chain = make_ur5_like_chain();

    // Near-singular config: joints nearly stretched out
    Eigen::Vector<double, 6> q_known;
    q_known << 0.0, 0.01, 0.01, 0.0, 0.01, 0.0;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    spp::dls_solve_policy<spp::kinematic_chain<double, 6>> stepper;
    Eigen::Vector<double, 6> q0;
    q0 << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations = 300;

    stepper.setup(chain, target, q0, criteria);
    auto status = run_stepper(stepper, chain, 300);

    // Should converge or at least not diverge
    REQUIRE(status != spp::ik_status::diverged);
}

// ============================================================================
// DLS separate angular/linear convergence
// ============================================================================

TEST_CASE("DLS separate angular/linear convergence", "[ik][dls]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    // Loose position tolerance, tight orientation tolerance
    spp::dls_solve_policy<spp::kinematic_chain<double, 6>> stepper;
    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.position_tol = 1.0;
    criteria.orientation_tol = 1e-8;
    criteria.max_iterations = 200;

    stepper.setup(chain, target, q0, criteria);
    auto status = run_stepper(stepper, chain, 200);

    REQUIRE(status == spp::ik_status::converged);
}

// ============================================================================
// DLS condition_number returns positive value
// ============================================================================

TEST_CASE("DLS condition_number returns positive value", "[ik][dls]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;

    auto fk_target = spp::forward_kinematics(chain, q_known);

    spp::dls_solve_policy<spp::kinematic_chain<double, 6>> stepper;
    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;

    stepper.setup(chain, fk_target.end_effector, q0, criteria);
    stepper.step(chain);

    REQUIRE(stepper.condition_number() > 0);
}

// ============================================================================
// DLS iterations count
// ============================================================================

TEST_CASE("DLS iterations count", "[ik][dls]")
{
    auto chain = make_ur5_like_chain();

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;
    auto fk_target = spp::forward_kinematics(chain, q_known);

    spp::dls_solve_policy<spp::kinematic_chain<double, 6>> stepper;
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
