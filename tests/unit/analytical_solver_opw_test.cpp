#include "cartan/analytical.h"
#include "cartan/serial_chain.h"

#include "../fixtures/opw_chains.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <random>
#include <vector>
#include <numbers>

using namespace cartan;

// The always-on correctness bar for the OPW solver: a green home-pose-only test
// is explicitly insufficient, so the round-trip below spans over a thousand
// full-joint-range configurations and re-verifies every returned solution by an
// independent forward map at 1e-9 (never trusting the solver's own report).
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

// A 6R chain that is NOT ortho-parallel: axis 2 (Z) is not parallel to axis 3
// (Z here would be parallel, so axis 2 is Z and axis 3 is X) -- the parallel
// gate of make() must reject it. Built inline so the rejection premise is
// self-evident from the joint tags rather than borrowed from a fixture.
static auto make_non_ortho_parallel_chain()
{
    auto s0 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s1 = screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0.4});
    // Axis 3 about X is NOT parallel to axis 2 about Y: parallel gate fails.
    auto s2 = screw_axis<double>::revolute({1, 0, 0}, {0.4, 0, 0.4});
    auto s3 = screw_axis<double>::revolute({1, 0, 0}, {0.8, 0, 0.4});
    auto s4 = screw_axis<double>::revolute({0, 1, 0}, {0.8, 0, 0.4});
    auto s5 = screw_axis<double>::revolute({1, 0, 0}, {0.8, 0, 0.4});

    joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};
    std::array<joint_limits<double>, 6> limits{lim, lim, lim, lim, lim, lim};

    return static_chain<double, revolute_z, revolute_y, revolute_x,
                        revolute_x, revolute_y, revolute_x>(
        se3<double>(so3<double>::identity(),
                    Eigen::Vector3d(0.88, 0, 0.4)),
        {s0, s1, s2, s3, s4, s5}, limits);
}

TEST_CASE("OPW: FK round-trip reconstructs KR6 R900 targets at 1e-9 over a "
          "workspace-spanning sample")
{
    auto chain = fixtures::make_kr6_r900_opw_chain<double>();
    auto params = fixtures::kr6_r900_opw_parameters<double>();

    // Solve AT the correctness bar: the acceptance tolerance binds both the
    // position and orientation FK back-check, so every emitted solution already
    // reconstructs the target to 1e-9. The independent re-check below then holds
    // for every returned solution, not merely the best one.
    auto solver = opw_6r_solver<decltype(chain)>::make(chain, params, tolerance);
    REQUIRE(solver.has_value());

    std::mt19937_64 rng(0xC0FFEE1234ull);
    std::uniform_real_distribution<double> angle(
        -std::numbers::pi, std::numbers::pi);

    constexpr int samples = 1200;
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
            CHECK(fk_error(chain, sol, target) < tolerance);
        }
    }
}

TEST_CASE("OPW: make() accepts the offset-shoulder KR6 R900 chain")
{
    // The dual of Pieper's rejection: the lateral shoulder offset (a1 != 0) that
    // Pieper's shoulder-intersection gate rejects is exactly the geometry OPW
    // admits. The same chain that a Pieper factory would refuse must construct.
    auto chain = fixtures::make_kr6_r900_opw_chain<double>();
    auto params = fixtures::kr6_r900_opw_parameters<double>();

    auto solver = opw_6r_solver<decltype(chain)>::make(chain, params);
    REQUIRE(solver.has_value());

    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.4, 0.5, 0.2, -0.3, 0.1;
    auto target = forward_kinematics(chain, q_known).end_effector;
    auto result = solver->solve(target);

    REQUIRE(result.has_value());
    REQUIRE(result->count >= 1);
}

TEST_CASE("OPW: make() rejects a non-ortho-parallel chain")
{
    auto chain = make_non_ortho_parallel_chain();
    auto params = fixtures::kr6_r900_opw_parameters<double>();

    auto solver = opw_6r_solver<decltype(chain)>::make(chain, params);

    REQUIRE_FALSE(solver.has_value());
    CHECK(solver.error().reason == analytical_failure::degenerate_geometry);
}

TEST_CASE("OPW: wrist singularity returns FK-verified folded solutions")
{
    auto chain = fixtures::make_kr6_r900_opw_chain<double>();
    auto params = fixtures::kr6_r900_opw_parameters<double>();

    // Solve at the correctness bar so the assertion "every returned solution
    // FK-verifies at 1e-9" holds: at the exact locus the fold is exact, and any
    // alternate branch that only reaches 1e-8 near a secondary singularity is
    // filtered rather than reported.
    auto solver = opw_6r_solver<decltype(chain)>::make(chain, params, tolerance);
    REQUIRE(solver.has_value());

    // Targets ON the wrist-singular locus: internal theta5 = q(4) = 0 exactly,
    // swept over several theta4/theta6 pairs (the two wrist-flip branches
    // coincide there, so the solution count must collapse below the generic 8).
    for (double q4 : {0.0, 0.7, -1.3})
    {
        for (double q6 : {0.0, 0.5, -0.9})
        {
            Eigen::Vector<double, 6> q;
            q << 0.3, -0.4, 0.5, q4, 0.0, q6;
            auto target = forward_kinematics(chain, q).end_effector;

            auto result = solver->solve(target);

            INFO("singular target q4 = " << q4 << " q6 = " << q6);
            // A value, NOT the singular_configuration error channel.
            REQUIRE(result.has_value());
            REQUIRE(result->count >= 1);
            // The wrist-flip branches coincide: count shrinks below 8.
            CHECK(result->count < 8);

            for (int i = 0; i < result->count; ++i)
            {
                const auto& sol = result->solutions[static_cast<std::size_t>(i)];
                for (int k = 0; k < 6; ++k)
                    CHECK_FALSE(std::isnan(sol(k)));
                CHECK(fk_error(chain, sol, target) < tolerance);
            }
        }
    }
}

TEST_CASE("OPW: the error channel is reserved for genuine failures")
{
    auto chain = fixtures::make_kr6_r900_opw_chain<double>();
    auto params = fixtures::kr6_r900_opw_parameters<double>();
    auto solver = *opw_6r_solver<decltype(chain)>::make(chain, params);

    // A target far outside the workspace: no branch can verify, so solve() must
    // report unreachable out of band rather than fabricating a solution.
    auto far = se3<double>(so3<double>::identity(),
                           Eigen::Vector3d(100.0, 100.0, 100.0));
    auto result = solver.solve(far);

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().reason == analytical_failure::unreachable);
    // The diagnostic carries a positive workspace overshoot, never a joint value.
    CHECK(result.error().workspace_distance > 0.0);
}

TEST_CASE("OPW: the sin(theta5) fold threshold sits in the empirical "
          "fold-beats-naive crossover band")
{
    // The wrist-fold threshold is not inherited from the reference's hardcoded
    // 1e-6; it is pinned by measuring where the fold path begins to beat the
    // naive theta4/theta6 atan2 decomposition. Near the locus the naive path
    // amplifies rounding as ~eps/|sin(theta5)|, while pinning theta4 = 0 injects
    // an O(|sin(theta5)|) residual. Their worst-case FK errors cross in a narrow
    // band; the default threshold must lie inside it.
    auto chain = fixtures::make_kr6_r900_opw_chain<double>();
    auto params = fixtures::kr6_r900_opw_parameters<double>();

    // Permissive acceptance so BOTH the fold and the naive reconstruction are
    // emitted (not filtered), letting the independent re-check expose each
    // path's true worst-case FK error.
    const double permissive = 1e-2;

    // A spread of shoulder/elbow configurations; joint 5 is swept onto the
    // locus so the worst case is taken over the whole family, not one pose.
    std::mt19937_64 rng(0x5EED9005ull);
    std::uniform_real_distribution<double> angle(
        -std::numbers::pi, std::numbers::pi);
    std::vector<Eigen::Vector<double, 6>> bases;
    for (int i = 0; i < 24; ++i)
    {
        Eigen::Vector<double, 6> q;
        for (int k = 0; k < 6; ++k)
            q(k) = angle(rng);
        q(4) = 0.0;
        bases.push_back(q);
    }

    // Worst-case FK error of the fold path (sing_tol above delta) versus the
    // naive path (sing_tol = 0) at a given distance delta from the locus.
    auto worst_pair = [&](double delta) -> std::pair<double, double>
    {
        auto fold = *opw_6r_solver<decltype(chain)>::make(
            chain, params, permissive, delta * 3.0);
        auto naive = *opw_6r_solver<decltype(chain)>::make(
            chain, params, permissive, 0.0);

        double fold_worst = 0.0;
        double naive_worst = 0.0;
        for (auto q : bases)
        {
            q(4) = delta;
            auto target = forward_kinematics(chain, q).end_effector;

            auto worst = [&](const auto& r) -> double
            {
                double mx = 0.0;
                if (r)
                    for (int i = 0; i < r->count; ++i)
                        mx = std::max(mx, fk_error(
                            chain, r->solutions[static_cast<std::size_t>(i)],
                            target));
                return mx;
            };
            fold_worst = std::max(fold_worst, worst(fold.solve(target)));
            naive_worst = std::max(naive_worst, worst(naive.solve(target)));
        }
        return {fold_worst, naive_worst};
    };

    // Report the transition for the record (observed crossover ~2.2e-8..3.1e-8
    // for this arm; the naive path wins above it, the fold wins below).
    for (double d : {1e-6, 1e-7, 3e-8, 2e-8, 1e-8, 1e-9})
    {
        auto [f, n] = worst_pair(d);
        INFO("delta = " << d << " fold_worst = " << f
             << " naive_worst = " << n);
        CHECK(true);
        std::printf("  |sin th5|~%.1e : fold=%.3e naive=%.3e -> %s\n",
                    d, f, n, f < n ? "fold" : "naive");
    }

    // Bracket the crossover with an order-of-magnitude margin either side.
    const double below = 5e-9;   // comfortably inside the fold-wins region
    const double above = 5e-7;   // comfortably inside the naive-wins region

    auto [fb, nb] = worst_pair(below);
    INFO("below-band delta = " << below
         << " fold_worst = " << fb << " naive_worst = " << nb);
    CHECK(fb < nb);              // fold must win below the band

    auto [fa, na] = worst_pair(above);
    INFO("above-band delta = " << above
         << " fold_worst = " << fa << " naive_worst = " << na);
    CHECK(na < fa);             // naive must win above the band

    // The crossover therefore lies strictly inside (below, above); the pinned
    // default must too.
    CHECK(opw_6r_solver<decltype(chain)>::default_singularity_tolerance > below);
    CHECK(opw_6r_solver<decltype(chain)>::default_singularity_tolerance < above);
}
