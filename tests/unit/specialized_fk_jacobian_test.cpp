/// @file specialized_fk_jacobian_test.cpp
/// @brief Verifies numerical equivalence between specialized (axis-specific)
///        and generic FK/Jacobian paths for all 9 benchmark robot geometries,
///        plus near-zero and zero-angle edge cases.

#include "../fixtures/chain_factories.h"
#include "../fixtures/prismatic_chains.h"

#include <cartan/serial/fk/jacobian.h>
#include <cartan/serial/fk/jacobian_matrix.h>
#include <cartan/serial/fk/forward_kinematics.h>
#include <cartan/serial/fk/forward_kinematics_matrix.h>
#include <cartan/serial/fk/detail/axis_specializations.h>
#include <cartan/serial/chain/static_chain.h>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <random>

namespace spp = cartan;

// ============================================================================
// Static chain factory helpers (mirrors chain_factories.h runtime geometry)
// ============================================================================

template <typename Scalar>
auto make_3r_planar_static()
{
    auto kc = spp::fixtures::make_3r_planar_chain<Scalar>();
    return spp::static_chain<Scalar, spp::revolute_z, spp::revolute_z, spp::revolute_z>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_ur3e_static()
{
    auto kc = spp::fixtures::make_ur3e_chain<Scalar>();
    return spp::static_chain<Scalar,
        spp::revolute_z, spp::revolute_y, spp::revolute_y,
        spp::revolute_y, spp::revolute_z, spp::revolute_y>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_lbr_med14_static()
{
    auto kc = spp::fixtures::make_lbr_med14_chain<Scalar>();
    return spp::static_chain<Scalar,
        spp::revolute_z, spp::revolute_y, spp::revolute_z, spp::revolute_y,
        spp::revolute_z, spp::revolute_y, spp::revolute_z>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_kr6_sixx_static()
{
    auto kc = spp::fixtures::make_kr6_sixx_chain<Scalar>();
    return spp::static_chain<Scalar,
        spp::revolute_z, spp::revolute_y, spp::revolute_y,
        spp::revolute_x, spp::revolute_y, spp::revolute_x>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_panda_static()
{
    auto kc = spp::fixtures::make_panda_chain<Scalar>();
    return spp::static_chain<Scalar,
        spp::revolute_z, spp::revolute_y, spp::revolute_z, spp::revolute_y,
        spp::revolute_z, spp::revolute_y, spp::revolute_z>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_abb_irb120_static()
{
    auto kc = spp::fixtures::make_abb_irb120_chain<Scalar>();
    return spp::static_chain<Scalar,
        spp::revolute_z, spp::revolute_y, spp::revolute_y,
        spp::revolute_x, spp::revolute_y, spp::revolute_x>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_jaco2_static()
{
    auto kc = spp::fixtures::make_jaco2_chain<Scalar>();
    return spp::static_chain<Scalar,
        spp::revolute_z, spp::revolute_y, spp::revolute_y,
        spp::revolute_x, spp::revolute_y, spp::revolute_x>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_fetch_static()
{
    auto kc = spp::fixtures::make_fetch_chain<Scalar>();
    return spp::static_chain<Scalar,
        spp::revolute_z, spp::revolute_y, spp::revolute_x, spp::revolute_y,
        spp::revolute_x, spp::revolute_y, spp::revolute_x>(
        kc.home(), kc.axes(), kc.limits());
}

template <typename Scalar>
auto make_baxter_static()
{
    auto kc = spp::fixtures::make_baxter_chain<Scalar>();
    return spp::static_chain<Scalar,
        spp::revolute_z, spp::revolute_y, spp::revolute_x, spp::revolute_y,
        spp::revolute_x, spp::revolute_y, spp::revolute_x>(
        kc.home(), kc.axes(), kc.limits());
}

// ============================================================================
// Helpers
// ============================================================================

template <typename StaticChain, int N>
void verify_fk_parity_specialized(
    const StaticChain& sc,
    int num_configs,
    unsigned seed)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist(-1.5, 1.5);

    spp::detail::generic_chain_wrapper<StaticChain> wrapped{sc};

    for (int c = 0; c < num_configs; ++c)
    {
        Eigen::Vector<double, N> q;
        for (int j = 0; j < N; ++j)
            q(j) = dist(rng);

        auto fk_spec = spp::forward_kinematics(sc, q);
        auto fk_gen = spp::forward_kinematics(wrapped, q);

        // End-effector comparison
        double trans_diff = (fk_spec.end_effector.translation()
                             - fk_gen.end_effector.translation()).norm();
        REQUIRE(trans_diff < 1e-12);

        auto rot_diff = (fk_spec.end_effector.rotation().inverse()
                         * fk_gen.end_effector.rotation()).log();
        REQUIRE(rot_diff.norm() < 1e-12);

        // Intermediate comparison
        for (int i = 0; i < N; ++i)
        {
            double interm_trans = (fk_spec.intermediates[static_cast<std::size_t>(i)].translation()
                                   - fk_gen.intermediates[static_cast<std::size_t>(i)].translation()).norm();
            REQUIRE(interm_trans < 1e-12);

            auto interm_rot = (fk_spec.intermediates[static_cast<std::size_t>(i)].rotation().inverse()
                               * fk_gen.intermediates[static_cast<std::size_t>(i)].rotation()).log();
            REQUIRE(interm_rot.norm() < 1e-12);
        }
    }
}

template <typename StaticChain, int N>
void verify_jacobian_parity_specialized(
    const StaticChain& sc,
    int num_configs,
    unsigned seed)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist(-1.5, 1.5);

    spp::detail::generic_chain_wrapper<StaticChain> wrapped{sc};

    for (int c = 0; c < num_configs; ++c)
    {
        Eigen::Vector<double, N> q;
        for (int j = 0; j < N; ++j)
            q(j) = dist(rng);

        auto fk_spec = spp::forward_kinematics(sc, q);
        auto fk_gen = spp::forward_kinematics(wrapped, q);

        auto Js_spec = spp::space_jacobian(sc, fk_spec);
        auto Js_gen = spp::space_jacobian(wrapped, fk_gen);
        REQUIRE((Js_spec - Js_gen).norm() < 1e-10);

        auto Jb_spec = spp::body_jacobian(sc, fk_spec);
        auto Jb_gen = spp::body_jacobian(wrapped, fk_gen);
        REQUIRE((Jb_spec - Jb_gen).norm() < 1e-10);
    }
}

// ============================================================================
// Test 1: FK parity -- specialized vs generic for all 9 robots
// ============================================================================

TEST_CASE("Specialized FK parity", "[specialized][fk]")
{
    constexpr unsigned seed = 42;
    constexpr int num_configs = 10;

    SECTION("3R planar")
    {
        auto sc = make_3r_planar_static<double>();
        verify_fk_parity_specialized<decltype(sc), 3>(sc, num_configs, seed);
    }

    SECTION("UR3e")
    {
        auto sc = make_ur3e_static<double>();
        verify_fk_parity_specialized<decltype(sc), 6>(sc, num_configs, seed);
    }

    SECTION("LBR Med14")
    {
        auto sc = make_lbr_med14_static<double>();
        verify_fk_parity_specialized<decltype(sc), 7>(sc, num_configs, seed);
    }

    SECTION("KR6 SIXX")
    {
        auto sc = make_kr6_sixx_static<double>();
        verify_fk_parity_specialized<decltype(sc), 6>(sc, num_configs, seed);
    }

    SECTION("Franka Panda")
    {
        auto sc = make_panda_static<double>();
        verify_fk_parity_specialized<decltype(sc), 7>(sc, num_configs, seed);
    }

    SECTION("ABB IRB120")
    {
        auto sc = make_abb_irb120_static<double>();
        verify_fk_parity_specialized<decltype(sc), 6>(sc, num_configs, seed);
    }

    SECTION("Kinova Jaco2")
    {
        auto sc = make_jaco2_static<double>();
        verify_fk_parity_specialized<decltype(sc), 6>(sc, num_configs, seed);
    }

    SECTION("Fetch")
    {
        auto sc = make_fetch_static<double>();
        verify_fk_parity_specialized<decltype(sc), 7>(sc, num_configs, seed);
    }

    SECTION("Rethink Baxter")
    {
        auto sc = make_baxter_static<double>();
        verify_fk_parity_specialized<decltype(sc), 7>(sc, num_configs, seed);
    }
}

// ============================================================================
// Test 2: Jacobian parity -- specialized vs generic for all 9 robots
// ============================================================================

TEST_CASE("Specialized Jacobian parity", "[specialized][jacobian]")
{
    constexpr unsigned seed = 42;
    constexpr int num_configs = 10;

    SECTION("3R planar")
    {
        auto sc = make_3r_planar_static<double>();
        verify_jacobian_parity_specialized<decltype(sc), 3>(sc, num_configs, seed);
    }

    SECTION("UR3e")
    {
        auto sc = make_ur3e_static<double>();
        verify_jacobian_parity_specialized<decltype(sc), 6>(sc, num_configs, seed);
    }

    SECTION("LBR Med14")
    {
        auto sc = make_lbr_med14_static<double>();
        verify_jacobian_parity_specialized<decltype(sc), 7>(sc, num_configs, seed);
    }

    SECTION("KR6 SIXX")
    {
        auto sc = make_kr6_sixx_static<double>();
        verify_jacobian_parity_specialized<decltype(sc), 6>(sc, num_configs, seed);
    }

    SECTION("Franka Panda")
    {
        auto sc = make_panda_static<double>();
        verify_jacobian_parity_specialized<decltype(sc), 7>(sc, num_configs, seed);
    }

    SECTION("ABB IRB120")
    {
        auto sc = make_abb_irb120_static<double>();
        verify_jacobian_parity_specialized<decltype(sc), 6>(sc, num_configs, seed);
    }

    SECTION("Kinova Jaco2")
    {
        auto sc = make_jaco2_static<double>();
        verify_jacobian_parity_specialized<decltype(sc), 6>(sc, num_configs, seed);
    }

    SECTION("Fetch")
    {
        auto sc = make_fetch_static<double>();
        verify_jacobian_parity_specialized<decltype(sc), 7>(sc, num_configs, seed);
    }

    SECTION("Rethink Baxter")
    {
        auto sc = make_baxter_static<double>();
        verify_jacobian_parity_specialized<decltype(sc), 7>(sc, num_configs, seed);
    }
}

// ============================================================================
// Test 3: Near-zero angle edge case
// ============================================================================

TEST_CASE("Specialized FK near-zero stability", "[specialized][edge]")
{
    auto sc = make_ur3e_static<double>();

    Eigen::Vector<double, 6> q;
    q.setConstant(1e-15);

    auto fk = spp::forward_kinematics(sc, q);
    REQUIRE_FALSE(fk.end_effector.translation().hasNaN());
    REQUIRE_FALSE(fk.end_effector.rotation().matrix().hasNaN());

    auto Js = spp::space_jacobian(sc, fk);
    REQUIRE_FALSE(Js.hasNaN());

    auto Jb = spp::body_jacobian(sc, fk);
    REQUIRE_FALSE(Jb.hasNaN());

    // Near-zero angles should produce a pose close to home
    double trans_diff = (fk.end_effector.translation()
                         - sc.home().translation()).norm();
    REQUIRE(trans_diff < 1e-10);
}

// ============================================================================
// Test 4: FK parity at exact zero angles
// ============================================================================

TEST_CASE("Specialized FK at zero config", "[specialized][edge]")
{
    SECTION("UR3e")
    {
        auto sc = make_ur3e_static<double>();
        Eigen::Vector<double, 6> q = Eigen::Vector<double, 6>::Zero();

        auto fk = spp::forward_kinematics(sc, q);

        double trans_diff = (fk.end_effector.translation()
                             - sc.home().translation()).norm();
        REQUIRE(trans_diff < 1e-14);

        auto rot_diff = (fk.end_effector.rotation().inverse()
                         * sc.home().rotation()).log();
        REQUIRE(rot_diff.norm() < 1e-14);
    }

    SECTION("3R planar")
    {
        auto sc = make_3r_planar_static<double>();
        Eigen::Vector<double, 3> q = Eigen::Vector<double, 3>::Zero();

        auto fk = spp::forward_kinematics(sc, q);

        double trans_diff = (fk.end_effector.translation()
                             - sc.home().translation()).norm();
        REQUIRE(trans_diff < 1e-14);
    }

    SECTION("Panda")
    {
        auto sc = make_panda_static<double>();
        Eigen::Vector<double, 7> q = Eigen::Vector<double, 7>::Zero();

        auto fk = spp::forward_kinematics(sc, q);

        double trans_diff = (fk.end_effector.translation()
                             - sc.home().translation()).norm();
        REQUIRE(trans_diff < 1e-14);

        auto rot_diff = (fk.end_effector.rotation().inverse()
                         * sc.home().rotation()).log();
        REQUIRE(rot_diff.norm() < 1e-14);
    }
}

// ============================================================================
// Test 5: Signed-direction prismatic FK/Jacobian against the se3::exp oracle
//
// The generic chain wrapper drives the se3::exp(axis.to_vector() * q) Product-
// of-Exponentials path, which carries the prismatic direction sign correctly
// and serves as the oracle. The specialized (static_chain, compile-time tag)
// and runtime (kinematic_chain, cached joint_kind) fast paths must match it on
// a chain whose prismatic joints point along -z and -x.
// ============================================================================

TEST_CASE("Prismatic sign FK matches se3::exp oracle", "[specialized][prismatic]")
{
    auto sc = spp::fixtures::make_rppr_signed_static<double>();
    verify_fk_parity_specialized<decltype(sc), 4>(sc, 16, 1234);

    // Runtime (kinematic_chain) path against the same generic oracle.
    auto kc = spp::fixtures::make_rppr_signed_chain<double>();
    spp::detail::generic_chain_wrapper<decltype(kc)> wrapped{kc};

    std::mt19937 rng(1234);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    for (int c = 0; c < 16; ++c)
    {
        Eigen::Vector<double, 4> q;
        for (int j = 0; j < 4; ++j)
            q(j) = dist(rng);

        auto fk_rt = spp::forward_kinematics(kc, q);
        auto fk_or = spp::forward_kinematics(wrapped, q);
        REQUIRE((fk_rt.end_effector.translation()
                 - fk_or.end_effector.translation()).norm() < 1e-12);
    }
}

TEST_CASE("Prismatic sign Jacobian matches adjoint-screw oracle",
          "[specialized][prismatic][jacobian]")
{
    auto sc = spp::fixtures::make_rppr_signed_static<double>();

    // Quaternion-path specialized Jacobian against the generic wrapper oracle.
    verify_jacobian_parity_specialized<decltype(sc), 4>(sc, 16, 4321);

    // Build the explicit oracle J from the generic PoE intermediates and check
    // BOTH the quaternion path and the matrix path, BOTH the identity column
    // (col 0) and the adjoint columns (col i > 0).
    spp::detail::generic_chain_wrapper<decltype(sc)> wrapped{sc};

    std::mt19937 rng(4321);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    for (int c = 0; c < 16; ++c)
    {
        Eigen::Vector<double, 4> q;
        for (int j = 0; j < 4; ++j)
            q(j) = dist(rng);

        auto fk_gen = spp::forward_kinematics(wrapped, q);

        Eigen::Matrix<double, 6, 4> J_oracle;
        J_oracle.col(0) = sc.axis(0).to_vector();
        for (int i = 1; i < 4; ++i)
        {
            J_oracle.col(i) =
                fk_gen.intermediates[static_cast<std::size_t>(i - 1)].adjoint()
                * sc.axis(i).to_vector();
        }

        auto fk_quat = spp::forward_kinematics(sc, q);
        auto Js_quat = spp::space_jacobian(sc, fk_quat);
        REQUIRE((Js_quat - J_oracle).norm() < 1e-10);

        auto fk_mat = spp::forward_kinematics_matrix(sc, q);
        auto Js_mat = spp::space_jacobian(sc, fk_mat);
        REQUIRE((Js_mat - J_oracle).norm() < 1e-10);

        // Runtime (kinematic_chain) matrix and quaternion paths.
        auto kc = spp::fixtures::make_rppr_signed_chain<double>();
        auto fk_kc = spp::forward_kinematics(kc, q);
        auto Js_kc = spp::space_jacobian(kc, fk_kc);
        REQUIRE((Js_kc - J_oracle).norm() < 1e-10);

        auto fk_kc_mat = spp::forward_kinematics_matrix(kc, q);
        auto Js_kc_mat = spp::space_jacobian(kc, fk_kc_mat);
        REQUIRE((Js_kc_mat - J_oracle).norm() < 1e-10);
    }
}
