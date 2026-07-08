#include "cartan/analytical.h"
#include "cartan/serial_chain.h"

#include "../fixtures/chain_factories.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <random>
#include <numbers>

using namespace cartan;

// Solve and re-verify at the same 1e-9 bar the OPW round-trip uses: the closed
// form never gets to grade its own homework, every returned branch is checked
// by an independent forward map.
static constexpr double tolerance = 1e-9;

// Worst of position and orientation reconstruction error of q against target.
template <typename Chain>
static double fk_error(const Chain& chain,
                       const Eigen::Vector<double, 6>& q,
                       const se3<double>& target)
{
    auto fk = forward_kinematics(chain, q);
    const double pe =
        (fk.end_effector.translation() - target.translation()).norm();
    const double oe = (fk.end_effector.rotation().inverse()
        * target.rotation()).log().norm();
    return std::max(pe, oe);
}

TEST_CASE("IRB120: the reconciled chain factory is a spherical wrist Pieper "
          "admits")
{
    auto chain = fixtures::make_abb_irb120_chain<double>();
    auto solver = pieper_6r_solver<decltype(chain)>::make(chain, tolerance);

    // Before the wrist-center reconciliation axes 4,5,6 missed a common point
    // and this gate rejected the chain; now it passes.
    REQUIRE(solver.has_value());
}

TEST_CASE("IRB120: reconciled-geometry FK accuracy over a workspace-spanning "
          "sample")
{
    auto chain = fixtures::make_abb_irb120_chain<double>();
    auto solver = pieper_6r_solver<decltype(chain)>::make(chain, tolerance);
    REQUIRE(solver.has_value());

    std::mt19937_64 rng(0xABB120ull);
    std::uniform_real_distribution<double> angle(
        -std::numbers::pi, std::numbers::pi);

    constexpr int samples = 1200;
    double worst = 0.0;
    for (int t = 0; t < samples; ++t)
    {
        Eigen::Vector<double, 6> q_known;
        for (int k = 0; k < 6; ++k)
            q_known(k) = angle(rng);

        auto target = forward_kinematics(chain, q_known).end_effector;
        auto result = solver->solve(target);

        INFO("sample " << t << " q_known = " << q_known.transpose());
        REQUIRE(result.has_value());
        REQUIRE(result->count >= 1);

        for (int i = 0; i < result->count; ++i)
        {
            const auto& sol = result->solutions[static_cast<std::size_t>(i)];
            for (int k = 0; k < 6; ++k)
                CHECK_FALSE(std::isnan(sol(k)));
            const double e = fk_error(chain, sol, target);
            worst = std::max(worst, e);
            CHECK(e < tolerance);
        }
    }
    INFO("worst per-call FK error = " << worst);
    CHECK(worst < tolerance);
}

TEST_CASE("IRB120: reconciled chain and static factory agree on FK away from "
          "home")
{
    auto kc = fixtures::make_abb_irb120_chain<double>();
    auto sc = fixtures::make_abb_irb120_static<double>();

    std::array<Eigen::Vector<double, 6>, 5> configs = {{
        {0.3, -0.2, 0.4, 0.1, -0.3, 0.2},
        {0.5, -0.4, 0.3, -0.2, 0.1, 0.4},
        {-0.3, 0.2, -0.4, 0.5, -0.1, 0.3},
        {0.1, -0.1, 0.1, -0.1, 0.1, -0.1},
        {1.0, 0.6, -0.7, 0.9, -1.1, 0.5}
    }};

    constexpr double tol = 1e-12;
    for (const auto& q : configs)
    {
        auto a = forward_kinematics(kc, q).end_effector;
        auto b = forward_kinematics(sc, q).end_effector;
        const double pe = (a.translation() - b.translation()).norm();
        const double oe =
            (a.rotation().inverse() * b.rotation()).log().norm();
        CHECK(pe < tol);
        CHECK(oe < tol);
    }
}
