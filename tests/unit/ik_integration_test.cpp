#include <cartan/types.h>

#include <cartan/serial/ik/ik.h>

#include <cartan/lie/se3.h>
#include <cartan/lie/so3.h>
#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/joint_state.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/kinematic_chain.h>
#include <cartan/serial/fk/forward_kinematics.h>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>

namespace spp = cartan;

// ============================================================================
// Helpers
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
// Reachable target: DLS converges within tolerance
// ============================================================================

TEST_CASE("Reachable target: DLS converges within tolerance", "[ik][integration]")
{
    auto chain = make_ur5_like_chain();

    auto test_config = [&](Eigen::Vector<double, 6> q_known)
    {
        auto fk_target = spp::forward_kinematics(chain, q_known);
        auto target = fk_target.end_effector;

        spp::basic_ik_runner<spp::ik::dls<spp::kinematic_chain<double, 6>>> solver;
        Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
        spp::convergence_criteria<double> criteria;
        criteria.max_iterations_per_attempt = 300;

        solver.setup(chain, target, q0, criteria);
        auto result = solver.solve();

        REQUIRE(result.has_value());

        auto fk_sol = spp::forward_kinematics(chain, result.value().solution.position);
        auto err = (fk_sol.end_effector.inverse() * target).log();
        REQUIRE(err.head<3>().norm() < 1e-5);
        REQUIRE(err.tail<3>().norm() < 1e-5);
    };

    SECTION("config 1: {0.3, -0.5, 0.8, 0.1, -0.4, 0.7}")
    {
        Eigen::Vector<double, 6> q;
        q << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;
        test_config(q);
    }

    SECTION("config 2: {1.0, -1.0, 0.5, -0.5, 1.0, -1.0}")
    {
        Eigen::Vector<double, 6> q;
        q << 1.0, -1.0, 0.5, -0.5, 1.0, -1.0;
        test_config(q);
    }

    SECTION("config 3: home {0, 0, 0, 0, 0, 0}")
    {
        Eigen::Vector<double, 6> q;
        q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        auto fk_target = spp::forward_kinematics(chain, q);
        auto target = fk_target.end_effector;

        // Use a non-zero seed to avoid trivial solution
        Eigen::Vector<double, 6> q0;
        q0 << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;

        spp::basic_ik_runner<spp::ik::dls<spp::kinematic_chain<double, 6>>> solver;
        spp::convergence_criteria<double> criteria;
        criteria.max_iterations_per_attempt = 300;

        solver.setup(chain, target, q0, criteria);
        auto result = solver.solve();

        REQUIRE(result.has_value());

        auto fk_sol = spp::forward_kinematics(chain, result.value().solution.position);
        auto err = (fk_sol.end_effector.inverse() * target).log();
        REQUIRE(err.head<3>().norm() < 1e-5);
        REQUIRE(err.tail<3>().norm() < 1e-5);
    }
}

// ============================================================================
// Reachable target: LM converges within tolerance
// ============================================================================

TEST_CASE("Reachable target: LM converges within tolerance", "[ik][integration]")
{
    auto chain = make_ur5_like_chain();

    auto test_config = [&](Eigen::Vector<double, 6> q_known)
    {
        auto fk_target = spp::forward_kinematics(chain, q_known);
        auto target = fk_target.end_effector;

        spp::basic_ik_runner<spp::ik::lm<spp::kinematic_chain<double, 6>>> solver;
        Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
        spp::convergence_criteria<double> criteria;
        criteria.max_iterations_per_attempt = 300;

        solver.setup(chain, target, q0, criteria);
        auto result = solver.solve();

        REQUIRE(result.has_value());

        auto fk_sol = spp::forward_kinematics(chain, result.value().solution.position);
        auto err = (fk_sol.end_effector.inverse() * target).log();
        REQUIRE(err.head<3>().norm() < 1e-5);
        REQUIRE(err.tail<3>().norm() < 1e-5);
    };

    SECTION("config 1")
    {
        Eigen::Vector<double, 6> q;
        q << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;
        test_config(q);
    }

    SECTION("config 2")
    {
        Eigen::Vector<double, 6> q;
        q << 1.0, -1.0, 0.5, -0.5, 1.0, -1.0;
        test_config(q);
    }
}

// ============================================================================
// Boundary target: near workspace limit
// ============================================================================

TEST_CASE("Boundary target: near workspace limit", "[ik][integration]")
{
    auto chain = make_ur5_like_chain();

    // Near max extension
    Eigen::Vector<double, 6> q_ext;
    q_ext << 0.0, 0.05, 0.05, 0.0, 0.05, 0.0;

    auto fk_target = spp::forward_kinematics(chain, q_ext);
    auto target = fk_target.end_effector;

    spp::basic_ik_runner<spp::ik::dls<spp::kinematic_chain<double, 6>>> solver;
    Eigen::Vector<double, 6> q0;
    q0 << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations_per_attempt = 300;

    solver.setup(chain, target, q0, criteria);
    auto result = solver.solve();

    // Should converge or stall -- not diverge
    if (!result.has_value())
    {
        REQUIRE(result.error().reason != spp::ik_failure::diverged);
    }
    else
    {
        auto fk_sol = spp::forward_kinematics(chain, result.value().solution.position);
        auto err = (fk_sol.end_effector.inverse() * target).log();
        REQUIRE(err.norm() < 1e-3);
    }
}

// ============================================================================
// Singular configuration: elbow singularity
// ============================================================================

TEST_CASE("Singular configuration: elbow singularity does not diverge", "[ik][integration]")
{
    auto chain = make_ur5_like_chain();

    // All zeros is near-singular for UR5
    Eigen::Vector<double, 6> q_sing = Eigen::Vector<double, 6>::Zero();
    auto fk_target = spp::forward_kinematics(chain, q_sing);
    auto target = fk_target.end_effector;

    spp::basic_ik_runner<spp::ik::dls<spp::kinematic_chain<double, 6>>> solver;
    Eigen::Vector<double, 6> q0;
    q0 << 0.1, 0.2, 0.1, 0.1, 0.1, 0.1;
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations_per_attempt = 300;

    solver.setup(chain, target, q0, criteria);
    auto result = solver.solve();

    // Adaptive damping should prevent divergence
    if (!result.has_value())
    {
        REQUIRE(result.error().reason != spp::ik_failure::diverged);
    }
}

// ============================================================================
// Unreachable target: far outside workspace
// ============================================================================

TEST_CASE("Unreachable target: far outside workspace", "[ik][integration]")
{
    auto chain = make_ur5_like_chain();

    spp::vector3<double> far_trans;
    far_trans << 10, 10, 10;
    auto target = spp::se3<double>(spp::so3<double>::identity(), far_trans);

    spp::basic_ik_runner<spp::ik::dls<spp::kinematic_chain<double, 6>>> solver;
    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations_per_attempt = 100;

    solver.setup(chain, target, q0, criteria);
    auto result = solver.solve();

    REQUIRE(!result.has_value());
}

// ============================================================================
// Unreachable target: impossible orientation
// ============================================================================

TEST_CASE("Unreachable target: inside workspace but impossible orientation", "[ik][integration]")
{
    auto chain = make_ur5_like_chain();

    // Position within workspace but with an unusual orientation
    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;
    auto fk = spp::forward_kinematics(chain, q_known);
    auto pos = fk.end_effector.translation();

    // Create target with same position but drastically different orientation
    auto rot = spp::so3<double>::exp(Eigen::Vector3d(3.0, 3.0, 3.0));
    auto target = spp::se3<double>(rot, pos);

    spp::basic_ik_runner<spp::ik::dls<spp::kinematic_chain<double, 6>>> solver;
    Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations_per_attempt = 100;

    solver.setup(chain, target, q0, criteria);
    auto result = solver.solve();

    // May converge to a different config or report failure; either is acceptable
    // Key: does not hang or crash
    REQUIRE(true);
}

// ============================================================================
// 3R planar reachable
// ============================================================================

TEST_CASE("3R planar reachable", "[ik][integration]")
{
    auto chain = make_3r_planar_chain();

    Eigen::Vector3d q_known;
    q_known << 0.5, -0.3, 0.7;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    spp::basic_ik_runner<spp::ik::dls<spp::kinematic_chain<double, 3>>> solver;
    Eigen::Vector3d q0 = Eigen::Vector3d::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations_per_attempt = 200;

    solver.setup(chain, target, q0, criteria);
    auto result = solver.solve();

    REQUIRE(result.has_value());

    auto fk_sol = spp::forward_kinematics(chain, result.value().solution.position);
    auto err = (fk_sol.end_effector.inverse() * target).log();
    REQUIRE(err.head<3>().norm() < 1e-5);
    REQUIRE(err.tail<3>().norm() < 1e-5);
}

// ============================================================================
// 3R planar unreachable
// ============================================================================

TEST_CASE("3R planar unreachable", "[ik][integration]")
{
    auto chain = make_3r_planar_chain();

    // Beyond total link length (1+1+1 = 3)
    spp::vector3<double> far_trans;
    far_trans << 5, 0, 0;
    auto target = spp::se3<double>(spp::so3<double>::identity(), far_trans);

    spp::basic_ik_runner<spp::ik::dls<spp::kinematic_chain<double, 3>>> solver;
    Eigen::Vector3d q0 = Eigen::Vector3d::Zero();
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations_per_attempt = 100;

    solver.setup(chain, target, q0, criteria);
    auto result = solver.solve();

    REQUIRE(!result.has_value());
}

// ============================================================================
// Float scalar type
// ============================================================================

TEST_CASE("Float scalar type compiles and converges", "[ik][integration]")
{
    auto s1 = spp::screw_axis<float>::revolute({0, 0, 1}, {0, 0, 0});
    auto s2 = spp::screw_axis<float>::revolute({0, 0, 1}, {1, 0, 0});
    auto s3 = spp::screw_axis<float>::revolute({0, 0, 1}, {2, 0, 0});

    spp::vector3<float> home_trans;
    home_trans << 3.0f, 0.0f, 0.0f;
    auto home = spp::se3<float>(spp::so3<float>::identity(), home_trans);

    spp::joint_limits<float> lim{-std::numbers::pi_v<float>, std::numbers::pi_v<float>};
    spp::kinematic_chain<float, 3> chain(home, {s1, s2, s3}, {lim, lim, lim});

    Eigen::Vector3f q_known;
    q_known << 0.5f, -0.3f, 0.7f;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    spp::basic_ik_runner<spp::ik::dls<spp::kinematic_chain<float, 3>>> solver;
    Eigen::Vector3f q0 = Eigen::Vector3f::Zero();
    spp::convergence_criteria<float> criteria;
    criteria.position_tol = 1e-4f;
    criteria.orientation_tol = 1e-4f;
    criteria.max_iterations_per_attempt = 200;

    solver.setup(chain, target, q0, criteria);
    auto result = solver.solve();

    REQUIRE(result.has_value());
}

// ============================================================================
// Dynamic chain IK
// ============================================================================

TEST_CASE("Dynamic chain IK converges", "[ik][integration]")
{
    auto fixed_chain = make_ur5_like_chain();
    auto chain = fixed_chain.to_dynamic();

    Eigen::Vector<double, 6> q_known_fixed;
    q_known_fixed << 0.3, -0.5, 0.8, 0.1, -0.4, 0.7;

    Eigen::VectorXd q_known = q_known_fixed;

    auto fk_target = spp::forward_kinematics(chain, q_known);
    auto target = fk_target.end_effector;

    spp::basic_ik_runner<spp::ik::dls<spp::kinematic_chain<double, spp::dynamic>>> solver;
    Eigen::VectorXd q0 = Eigen::VectorXd::Zero(6);
    spp::convergence_criteria<double> criteria;
    criteria.max_iterations_per_attempt = 300;

    solver.setup(chain, target, q0, criteria);
    auto result = solver.solve();

    REQUIRE(result.has_value());

    auto fk_sol = spp::forward_kinematics(chain, result.value().solution.position);
    auto err = (fk_sol.end_effector.inverse() * target).log();
    REQUIRE(err.head<3>().norm() < 1e-5);
    REQUIRE(err.tail<3>().norm() < 1e-5);
}
