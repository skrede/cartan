/// @file kdl_fk_comparison_test.cpp
/// @brief Validates that all KDL chain definitions in benchmark_utils.h
///        produce identical FK to their cartan PoE counterparts.

#include "benchmark_utils.h"

#include <cartan/serial/fk/forward_kinematics.h>

#include <kdl/chainfksolverpos_recursive.hpp>

#include <catch2/catch_test_macros.hpp>

#include <random>
#include <cmath>

namespace
{

template <int N, typename MakeChain, typename MakeKdlChain>
void compare_fk(MakeChain make_chain, MakeKdlChain make_kdl_chain,
                const char* robot_name, int trials = 50)
{
    INFO("Robot: " << robot_name);

    auto chain = make_chain();
    auto kdl_chain = make_kdl_chain();
    KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);

    int n = static_cast<int>(chain.num_joints());
    REQUIRE(n == static_cast<int>(kdl_chain.getNrOfJoints()));

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> unit(0.0, 1.0);

    for (int trial = 0; trial < trials; ++trial)
    {
        INFO("Trial: " << trial);

        Eigen::Matrix<double, N, 1> q;
        if constexpr (N == cartan::dynamic) { q.resize(n); }

        for (int j = 0; j < n; ++j)
        {
            double lo = chain.limits()[static_cast<std::size_t>(j)].position_min;
            double hi = chain.limits()[static_cast<std::size_t>(j)].position_max;
            q(j) = lo + (hi - lo) * unit(rng);
        }

        auto fk_result = cartan::forward_kinematics(chain, q);
        auto pos_cartan = fk_result.end_effector.translation();
        auto rot_cartan = fk_result.end_effector.rotation().matrix();

        KDL::JntArray kdl_q(static_cast<unsigned>(n));
        for (int j = 0; j < n; ++j) { kdl_q(static_cast<unsigned>(j)) = q(j); }
        KDL::Frame kdl_frame;
        fk_solver.JntToCart(kdl_q, kdl_frame);

        constexpr double tol = 1e-10;

        REQUIRE(std::abs(pos_cartan(0) - kdl_frame.p.x()) < tol);
        REQUIRE(std::abs(pos_cartan(1) - kdl_frame.p.y()) < tol);
        REQUIRE(std::abs(pos_cartan(2) - kdl_frame.p.z()) < tol);

        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                REQUIRE(std::abs(rot_cartan(r, c) - kdl_frame.M(r, c)) < tol);
    }
}

} // anonymous namespace

TEST_CASE("FK comparison: UR3e", "[kdl][fk]")
{
    compare_fk<6>(
        cartan::fixtures::make_ur3e_chain<double>,
        cartan::fixtures::make_ur3e_kdl_chain,
        "UR3e");
}

TEST_CASE("FK comparison: LBR Med 14", "[kdl][fk]")
{
    compare_fk<7>(
        cartan::fixtures::make_lbr_med14_chain<double>,
        cartan::fixtures::make_lbr_med14_kdl_chain,
        "LBR Med 14");
}

TEST_CASE("FK comparison: KR6 SIXX", "[kdl][fk]")
{
    compare_fk<6>(
        cartan::fixtures::make_kr6_sixx_chain<double>,
        cartan::fixtures::make_kr6_sixx_kdl_chain,
        "KR6 SIXX");
}

TEST_CASE("FK comparison: Panda", "[kdl][fk]")
{
    compare_fk<7>(
        cartan::fixtures::make_panda_chain<double>,
        cartan::fixtures::make_panda_kdl_chain,
        "Panda");
}

TEST_CASE("FK comparison: ABB IRB 120", "[kdl][fk]")
{
    compare_fk<6>(
        cartan::fixtures::make_abb_irb120_chain<double>,
        cartan::fixtures::make_abb_irb120_kdl_chain,
        "ABB IRB 120");
}

TEST_CASE("FK comparison: Jaco2", "[kdl][fk]")
{
    compare_fk<6>(
        cartan::fixtures::make_jaco2_chain<double>,
        cartan::fixtures::make_jaco2_kdl_chain,
        "Jaco2");
}

TEST_CASE("FK comparison: Fetch", "[kdl][fk]")
{
    compare_fk<7>(
        cartan::fixtures::make_fetch_chain<double>,
        cartan::fixtures::make_fetch_kdl_chain,
        "Fetch");
}

TEST_CASE("FK comparison: Baxter", "[kdl][fk]")
{
    compare_fk<7>(
        cartan::fixtures::make_baxter_chain<double>,
        cartan::fixtures::make_baxter_kdl_chain,
        "Baxter");
}

TEST_CASE("FK comparison: Kuka LWR4", "[kdl][fk]")
{
    compare_fk<7>(
        cartan::fixtures::make_kuka_lwr4_chain<double>,
        cartan::fixtures::make_kuka_lwr4_kdl_chain,
        "Kuka LWR4");
}
