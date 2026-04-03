/// @file static_chain_ik_parity_test.cpp
/// @brief Verifies FK, Jacobian, and IK parity between kinematic_chain and
///        static_chain for all 9 benchmark robot geometries.

#include "../../profiling/chain_factories.h"

#include <cartan/serial/ik/default_solvers.h>

#include <cartan/lie/se3.h>
#include <cartan/serial/fk/jacobian.h>
#include <cartan/serial/chain/static_chain.h>
#include <cartan/serial/fk/forward_kinematics.h>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>

namespace spp = cartan;

// ============================================================================
// Static chain factory helpers
//
// Each mirrors the corresponding kinematic_chain factory from chain_factories.h
// with identical runtime geometry but compile-time joint type tags.
// ============================================================================

template <typename Scalar>
auto make_3r_planar_static()
{
    auto kc = spp::benchmarks::make_3r_planar_chain<Scalar>();
    return spp::static_chain<Scalar, spp::revolute_z, spp::revolute_z, spp::revolute_z>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_ur3e_static()
{
    auto kc = spp::benchmarks::make_ur3e_chain<Scalar>();
    return spp::static_chain<Scalar,
        spp::revolute_z, spp::revolute_y, spp::revolute_y,
        spp::revolute_y, spp::revolute_z, spp::revolute_y>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_lbr_med14_static()
{
    auto kc = spp::benchmarks::make_lbr_med14_chain<Scalar>();
    return spp::static_chain<Scalar,
        spp::revolute_z, spp::revolute_y, spp::revolute_z, spp::revolute_y,
        spp::revolute_z, spp::revolute_y, spp::revolute_z>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_kr6_sixx_static()
{
    auto kc = spp::benchmarks::make_kr6_sixx_chain<Scalar>();
    return spp::static_chain<Scalar,
        spp::revolute_z, spp::revolute_y, spp::revolute_y,
        spp::revolute_x, spp::revolute_y, spp::revolute_x>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_panda_static()
{
    auto kc = spp::benchmarks::make_panda_chain<Scalar>();
    return spp::static_chain<Scalar,
        spp::revolute_z, spp::revolute_y, spp::revolute_z, spp::revolute_y,
        spp::revolute_z, spp::revolute_y, spp::revolute_z>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_abb_irb120_static()
{
    auto kc = spp::benchmarks::make_abb_irb120_chain<Scalar>();
    return spp::static_chain<Scalar,
        spp::revolute_z, spp::revolute_y, spp::revolute_y,
        spp::revolute_x, spp::revolute_y, spp::revolute_x>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_jaco2_static()
{
    auto kc = spp::benchmarks::make_jaco2_chain<Scalar>();
    return spp::static_chain<Scalar,
        spp::revolute_z, spp::revolute_y, spp::revolute_y,
        spp::revolute_x, spp::revolute_y, spp::revolute_x>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_fetch_static()
{
    auto kc = spp::benchmarks::make_fetch_chain<Scalar>();
    return spp::static_chain<Scalar,
        spp::revolute_z, spp::revolute_y, spp::revolute_x, spp::revolute_y,
        spp::revolute_x, spp::revolute_y, spp::revolute_x>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_baxter_static()
{
    auto kc = spp::benchmarks::make_baxter_chain<Scalar>();
    return spp::static_chain<Scalar,
        spp::revolute_z, spp::revolute_y, spp::revolute_x, spp::revolute_y,
        spp::revolute_x, spp::revolute_y, spp::revolute_x>(
        kc.home(), kc.axes(), kc.limits());
}

// ============================================================================
// Parity verification helpers
// ============================================================================

/// Compare FK between kinematic_chain and static_chain at multiple configs.
template <typename KChain, typename SChain, int N>
void verify_fk_parity(
    const KChain& kc,
    const SChain& sc,
    const std::array<Eigen::Vector<double, N>, 5>& configs,
    double tol)
{
    for (const auto& q : configs)
    {
        auto fk_kc = spp::forward_kinematics(kc, q);
        auto fk_sc = spp::forward_kinematics(sc, q);

        auto diff = (fk_kc.end_effector.inverse() * fk_sc.end_effector).log();
        REQUIRE(diff.norm() < tol);
    }
}

/// Compare space and body Jacobians between chain types at multiple configs.
template <typename KChain, typename SChain, int N>
void verify_jacobian_parity(
    const KChain& kc,
    const SChain& sc,
    const std::array<Eigen::Vector<double, N>, 5>& configs,
    double tol)
{
    for (const auto& q : configs)
    {
        auto fk_kc = spp::forward_kinematics(kc, q);
        auto fk_sc = spp::forward_kinematics(sc, q);

        auto Js_kc = spp::space_jacobian(kc, fk_kc);
        auto Js_sc = spp::space_jacobian(sc, fk_sc);
        REQUIRE((Js_kc - Js_sc).norm() < tol);

        auto Jb_kc = spp::body_jacobian(kc, fk_kc);
        auto Jb_sc = spp::body_jacobian(sc, fk_sc);
        REQUIRE((Jb_kc - Jb_sc).norm() < tol);
    }
}

/// Solve IK on both chain types and verify both converge to the target.
template <typename KChain, typename SChain, int N>
void verify_ik_parity(
    const KChain& kc,
    const SChain& sc,
    const Eigen::Vector<double, N>& q_known)
{
    auto target = spp::forward_kinematics(kc, q_known).end_effector;
    Eigen::Vector<double, N> q0 = Eigen::Vector<double, N>::Zero();
    spp::convergence_criteria<double> criteria{1e-6, 1e-6, 200};

    spp::basic_ik_solver kc_solver{spp::speed_solver<KChain>{}};
    kc_solver.setup(kc, target, q0, criteria);
    auto kc_result = kc_solver.solve();
    REQUIRE(kc_result.has_value());

    spp::basic_ik_solver sc_solver{spp::speed_solver<SChain>{}};
    sc_solver.setup(sc, target, q0, criteria);
    auto sc_result = sc_solver.solve();
    REQUIRE(sc_result.has_value());

    // Verify both solutions reach the target
    auto fk_kc = spp::forward_kinematics(kc, kc_result->solution.position);
    auto fk_sc = spp::forward_kinematics(sc, sc_result->solution.position);

    auto err_kc = (fk_kc.end_effector.inverse() * target).log();
    auto err_sc = (fk_sc.end_effector.inverse() * target).log();
    REQUIRE(err_kc.norm() < 1e-4);
    REQUIRE(err_sc.norm() < 1e-4);
}

// ============================================================================
// 3R Planar
// ============================================================================

TEST_CASE("static_chain FK parity - 3R planar", "[static_chain][parity]")
{
    auto kc = spp::benchmarks::make_3r_planar_chain<double>();
    auto sc = make_3r_planar_static<double>();

    std::array<Eigen::Vector<double, 3>, 5> configs = {{
        {0.3, -0.5, 0.2}, {0.0, 0.0, 0.0}, {1.0, -1.0, 0.5},
        {-0.7, 0.4, -0.3}, {2.0, -1.5, 1.0}
    }};

    constexpr double tol = 100 * std::numeric_limits<double>::epsilon();
    verify_fk_parity<decltype(kc), decltype(sc), 3>(kc, sc, configs, tol);
    verify_jacobian_parity<decltype(kc), decltype(sc), 3>(kc, sc, configs, tol);
}

TEST_CASE("static_chain IK parity - 3R planar", "[static_chain][parity]")
{
    auto kc = spp::benchmarks::make_3r_planar_chain<double>();
    auto sc = make_3r_planar_static<double>();

    Eigen::Vector<double, 3> q_known{0.3, -0.5, 0.2};
    verify_ik_parity<decltype(kc), decltype(sc), 3>(kc, sc, q_known);
}

// ============================================================================
// UR3e
// ============================================================================

TEST_CASE("static_chain FK parity - UR3e", "[static_chain][parity]")
{
    auto kc = spp::benchmarks::make_ur3e_chain<double>();
    auto sc = make_ur3e_static<double>();

    std::array<Eigen::Vector<double, 6>, 5> configs = {{
        {0.3, -0.5, 0.8, -0.3, 0.6, -0.2}, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {1.0, -1.0, 0.5, -0.5, 1.0, -1.0}, {-0.7, 0.4, -0.3, 0.6, -0.2, 0.8},
        {0.1, -0.1, 0.1, -0.1, 0.1, -0.1}
    }};

    constexpr double tol = 100 * std::numeric_limits<double>::epsilon();
    verify_fk_parity<decltype(kc), decltype(sc), 6>(kc, sc, configs, tol);
    verify_jacobian_parity<decltype(kc), decltype(sc), 6>(kc, sc, configs, tol);
}

TEST_CASE("static_chain IK parity - UR3e", "[static_chain][parity]")
{
    auto kc = spp::benchmarks::make_ur3e_chain<double>();
    auto sc = make_ur3e_static<double>();

    Eigen::Vector<double, 6> q_known{0.3, -0.5, 0.8, -0.3, 0.6, -0.2};
    verify_ik_parity<decltype(kc), decltype(sc), 6>(kc, sc, q_known);
}

// ============================================================================
// LBR Med14
// ============================================================================

TEST_CASE("static_chain FK parity - LBR Med14", "[static_chain][parity]")
{
    auto kc = spp::benchmarks::make_lbr_med14_chain<double>();
    auto sc = make_lbr_med14_static<double>();

    std::array<Eigen::Vector<double, 7>, 5> configs = {{
        {0.2, -0.3, 0.1, -0.5, 0.4, -0.2, 0.3},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.5, -0.4, 0.3, -0.2, 0.1, 0.4, -0.3},
        {-0.3, 0.2, -0.4, 0.5, -0.1, 0.3, -0.2},
        {0.1, -0.1, 0.1, -0.1, 0.1, -0.1, 0.1}
    }};

    constexpr double tol = 100 * std::numeric_limits<double>::epsilon();
    verify_fk_parity<decltype(kc), decltype(sc), 7>(kc, sc, configs, tol);
    verify_jacobian_parity<decltype(kc), decltype(sc), 7>(kc, sc, configs, tol);
}

TEST_CASE("static_chain IK parity - LBR Med14", "[static_chain][parity]")
{
    auto kc = spp::benchmarks::make_lbr_med14_chain<double>();
    auto sc = make_lbr_med14_static<double>();

    Eigen::Vector<double, 7> q_known{0.2, -0.3, 0.1, -0.5, 0.4, -0.2, 0.3};
    verify_ik_parity<decltype(kc), decltype(sc), 7>(kc, sc, q_known);
}

// ============================================================================
// KR6 SIXX
// ============================================================================

TEST_CASE("static_chain FK parity - KR6 SIXX", "[static_chain][parity]")
{
    auto kc = spp::benchmarks::make_kr6_sixx_chain<double>();
    auto sc = make_kr6_sixx_static<double>();

    std::array<Eigen::Vector<double, 6>, 5> configs = {{
        {0.1, -0.05, 0.1, 0.05, -0.05, 0.05},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.3, -0.2, 0.4, 0.1, -0.3, 0.2},
        {-0.2, 0.1, -0.3, 0.2, -0.1, 0.3},
        {0.05, -0.05, 0.05, -0.05, 0.05, -0.05}
    }};

    constexpr double tol = 100 * std::numeric_limits<double>::epsilon();
    verify_fk_parity<decltype(kc), decltype(sc), 6>(kc, sc, configs, tol);
    verify_jacobian_parity<decltype(kc), decltype(sc), 6>(kc, sc, configs, tol);
}

TEST_CASE("static_chain IK parity - KR6 SIXX", "[static_chain][parity]")
{
    auto kc = spp::benchmarks::make_kr6_sixx_chain<double>();
    auto sc = make_kr6_sixx_static<double>();

    Eigen::Vector<double, 6> q_known{0.1, -0.05, 0.1, 0.05, -0.05, 0.05};
    verify_ik_parity<decltype(kc), decltype(sc), 6>(kc, sc, q_known);
}

// ============================================================================
// Franka Panda
// ============================================================================

TEST_CASE("static_chain FK parity - Franka Panda", "[static_chain][parity]")
{
    auto kc = spp::benchmarks::make_panda_chain<double>();
    auto sc = make_panda_static<double>();

    std::array<Eigen::Vector<double, 7>, 5> configs = {{
        {0.2, -0.3, 0.1, -0.5, 0.4, -0.2, 0.3},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.5, -0.4, 0.3, -0.2, 0.1, 0.4, -0.3},
        {-0.3, 0.2, -0.4, 0.5, -0.1, 0.3, -0.2},
        {0.1, -0.1, 0.1, -0.1, 0.1, -0.1, 0.1}
    }};

    constexpr double tol = 100 * std::numeric_limits<double>::epsilon();
    verify_fk_parity<decltype(kc), decltype(sc), 7>(kc, sc, configs, tol);
    verify_jacobian_parity<decltype(kc), decltype(sc), 7>(kc, sc, configs, tol);
}

TEST_CASE("static_chain IK parity - Franka Panda", "[static_chain][parity]")
{
    auto kc = spp::benchmarks::make_panda_chain<double>();
    auto sc = make_panda_static<double>();

    Eigen::Vector<double, 7> q_known{0.2, -0.3, 0.1, -0.5, 0.4, -0.2, 0.3};
    verify_ik_parity<decltype(kc), decltype(sc), 7>(kc, sc, q_known);
}

// ============================================================================
// ABB IRB120
// ============================================================================

TEST_CASE("static_chain FK parity - ABB IRB120", "[static_chain][parity]")
{
    auto kc = spp::benchmarks::make_abb_irb120_chain<double>();
    auto sc = make_abb_irb120_static<double>();

    std::array<Eigen::Vector<double, 6>, 5> configs = {{
        {0.3, -0.2, 0.4, 0.1, -0.3, 0.2},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.5, -0.4, 0.3, -0.2, 0.1, 0.4},
        {-0.3, 0.2, -0.4, 0.5, -0.1, 0.3},
        {0.1, -0.1, 0.1, -0.1, 0.1, -0.1}
    }};

    constexpr double tol = 100 * std::numeric_limits<double>::epsilon();
    verify_fk_parity<decltype(kc), decltype(sc), 6>(kc, sc, configs, tol);
    verify_jacobian_parity<decltype(kc), decltype(sc), 6>(kc, sc, configs, tol);
}

TEST_CASE("static_chain IK parity - ABB IRB120", "[static_chain][parity]")
{
    auto kc = spp::benchmarks::make_abb_irb120_chain<double>();
    auto sc = make_abb_irb120_static<double>();

    Eigen::Vector<double, 6> q_known{0.3, -0.2, 0.4, 0.1, -0.3, 0.2};
    verify_ik_parity<decltype(kc), decltype(sc), 6>(kc, sc, q_known);
}

// ============================================================================
// Kinova Jaco2
// ============================================================================

TEST_CASE("static_chain FK parity - Jaco2", "[static_chain][parity]")
{
    auto kc = spp::benchmarks::make_jaco2_chain<double>();
    auto sc = make_jaco2_static<double>();

    std::array<Eigen::Vector<double, 6>, 5> configs = {{
        {0.3, -0.2, 0.4, 0.1, -0.3, 0.2},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.5, -0.4, 0.3, -0.2, 0.1, 0.4},
        {-0.3, 0.2, -0.4, 0.5, -0.1, 0.3},
        {0.1, -0.1, 0.1, -0.1, 0.1, -0.1}
    }};

    constexpr double tol = 100 * std::numeric_limits<double>::epsilon();
    verify_fk_parity<decltype(kc), decltype(sc), 6>(kc, sc, configs, tol);
    verify_jacobian_parity<decltype(kc), decltype(sc), 6>(kc, sc, configs, tol);
}

TEST_CASE("static_chain IK parity - Jaco2", "[static_chain][parity]")
{
    auto kc = spp::benchmarks::make_jaco2_chain<double>();
    auto sc = make_jaco2_static<double>();

    Eigen::Vector<double, 6> q_known{0.3, -0.2, 0.4, 0.1, -0.3, 0.2};
    verify_ik_parity<decltype(kc), decltype(sc), 6>(kc, sc, q_known);
}

// ============================================================================
// Fetch
// ============================================================================

TEST_CASE("static_chain FK parity - Fetch", "[static_chain][parity]")
{
    auto kc = spp::benchmarks::make_fetch_chain<double>();
    auto sc = make_fetch_static<double>();

    std::array<Eigen::Vector<double, 7>, 5> configs = {{
        {0.2, -0.3, 0.1, -0.5, 0.4, -0.2, 0.3},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.5, -0.4, 0.3, -0.2, 0.1, 0.4, -0.3},
        {-0.3, 0.2, -0.4, 0.5, -0.1, 0.3, -0.2},
        {0.1, -0.1, 0.1, -0.1, 0.1, -0.1, 0.1}
    }};

    constexpr double tol = 100 * std::numeric_limits<double>::epsilon();
    verify_fk_parity<decltype(kc), decltype(sc), 7>(kc, sc, configs, tol);
    verify_jacobian_parity<decltype(kc), decltype(sc), 7>(kc, sc, configs, tol);
}

TEST_CASE("static_chain IK parity - Fetch", "[static_chain][parity]")
{
    auto kc = spp::benchmarks::make_fetch_chain<double>();
    auto sc = make_fetch_static<double>();

    Eigen::Vector<double, 7> q_known{0.2, -0.3, 0.1, -0.5, 0.4, -0.2, 0.3};
    verify_ik_parity<decltype(kc), decltype(sc), 7>(kc, sc, q_known);
}

// ============================================================================
// Rethink Baxter
// ============================================================================

TEST_CASE("static_chain FK parity - Baxter", "[static_chain][parity]")
{
    auto kc = spp::benchmarks::make_baxter_chain<double>();
    auto sc = make_baxter_static<double>();

    std::array<Eigen::Vector<double, 7>, 5> configs = {{
        {0.2, -0.3, 0.1, -0.5, 0.4, -0.2, 0.3},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {0.5, -0.4, 0.3, -0.2, 0.1, 0.4, -0.3},
        {-0.3, 0.2, -0.4, 0.5, -0.1, 0.3, -0.2},
        {0.1, -0.1, 0.1, -0.1, 0.1, -0.1, 0.1}
    }};

    constexpr double tol = 100 * std::numeric_limits<double>::epsilon();
    verify_fk_parity<decltype(kc), decltype(sc), 7>(kc, sc, configs, tol);
    verify_jacobian_parity<decltype(kc), decltype(sc), 7>(kc, sc, configs, tol);
}

TEST_CASE("static_chain IK parity - Baxter", "[static_chain][parity]")
{
    auto kc = spp::benchmarks::make_baxter_chain<double>();
    auto sc = make_baxter_static<double>();

    Eigen::Vector<double, 7> q_known{0.2, -0.3, 0.1, -0.5, 0.4, -0.2, 0.3};
    verify_ik_parity<decltype(kc), decltype(sc), 7>(kc, sc, q_known);
}
