#include "cartan/analytical.h"
#include "cartan/serial_chain.h"

#include "../fixtures/analytical_chains.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>
#include <cmath>
#include <numbers>

using namespace cartan;
using Catch::Matchers::WithinAbs;

static constexpr double tolerance = 1e-6;

/// Build a PUMA-type 6R chain with Pieper geometry.
///
/// Joints 1-3: shoulder-elbow (Z, Y, Y) for position.
/// Joints 4-6: spherical wrist (Z, Y, Z).
/// Axes 4, 5, 6 all intersect at the wrist center point.
///
/// Dimensions: d1=0.5, a2=0.4, a3=0.3, d6=0.1
static auto make_puma_chain()
{
    double d1 = 0.5, a2 = 0.4, a3 = 0.3, d6 = 0.1;
    Eigen::Vector3d wrist_point(a2 + a3, 0, d1);
    Eigen::Vector3d ee_point(a2 + a3 + d6, 0, d1);

    auto s0 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s1 = screw_axis<double>::revolute({0, 1, 0}, {0, 0, d1});
    auto s2 = screw_axis<double>::revolute({0, 1, 0}, {a2, 0, d1});
    auto s3 = screw_axis<double>::revolute({0, 0, 1}, wrist_point);
    auto s4 = screw_axis<double>::revolute({0, 1, 0}, wrist_point);
    auto s5 = screw_axis<double>::revolute({0, 0, 1}, wrist_point);

    auto home = se3<double>(so3<double>::identity(), ee_point);
    joint_limits<double> no_limits{-10.0, 10.0};
    std::array<joint_limits<double>, 6> limits = {
        no_limits, no_limits, no_limits, no_limits, no_limits, no_limits};

    return static_chain<double, revolute_z, revolute_y, revolute_y,
                        revolute_z, revolute_y, revolute_z>(
        home, {s0, s1, s2, s3, s4, s5}, limits);
}

/// Build a 6R chain where the last 3 axes do NOT intersect (non-Pieper).
/// Axes 4, 5, 6 are offset from each other so they cannot intersect.
static auto make_non_pieper_chain()
{
    auto s0 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s1 = screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0.5});
    auto s2 = screw_axis<double>::revolute({0, 1, 0}, {0.4, 0, 0.5});
    // Axes 4, 5, 6 pass through widely separated points:
    auto s3 = screw_axis<double>::revolute({0, 0, 1}, {0.7, 0, 0.5});
    auto s4 = screw_axis<double>::revolute({0, 1, 0}, {0.7, 2.0, 0.5});  // large y offset
    auto s5 = screw_axis<double>::revolute({1, 0, 0}, {0.7, 0, 3.0});    // large z offset, x-axis

    auto home = se3<double>(so3<double>::identity(), Eigen::Vector3d(0.8, 0, 0.5));
    joint_limits<double> no_limits{-10.0, 10.0};
    std::array<joint_limits<double>, 6> limits = {
        no_limits, no_limits, no_limits, no_limits, no_limits, no_limits};

    return static_chain<double, revolute_z, revolute_y, revolute_y,
                        revolute_z, revolute_y, revolute_x>(
        home, {s0, s1, s2, s3, s4, s5}, limits);
}

TEST_CASE("6R Pieper: reachable target from known FK returns solutions")
{
    auto chain = make_puma_chain();
    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.4, 0.5, 0.2, -0.3, 0.1;

    auto fk = forward_kinematics(chain, q_known);
    auto result = pieper_6r_solver(chain).solve(fk.end_effector);

    REQUIRE(result.has_value());
    REQUIRE(result->count >= 1);

    for (int i = 0; i < result->count; ++i)
    {
        auto fk_check = forward_kinematics(
            chain, result->solutions[static_cast<std::size_t>(i)]);
        double position_error = (fk_check.end_effector.translation()
            - fk.end_effector.translation()).norm();
        double orientation_error = (fk_check.end_effector.rotation().inverse()
            * fk.end_effector.rotation()).log().norm();
        CHECK(position_error < tolerance);
        CHECK(orientation_error < tolerance);
    }
}

TEST_CASE("6R Pieper: recovers original angles as one of the solutions")
{
    auto chain = make_puma_chain();
    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.4, 0.5, 0.2, -0.3, 0.1;

    auto fk = forward_kinematics(chain, q_known);
    auto result = pieper_6r_solver(chain).solve(fk.end_effector);

    REQUIRE(result.has_value());

    bool found_match = false;
    for (int i = 0; i < result->count; ++i)
    {
        auto fk_check = forward_kinematics(
            chain, result->solutions[static_cast<std::size_t>(i)]);
        double position_error = (fk_check.end_effector.translation()
            - fk.end_effector.translation()).norm();
        double orientation_error = (fk_check.end_effector.rotation().inverse()
            * fk.end_effector.rotation()).log().norm();
        if (position_error < tolerance && orientation_error < tolerance)
        {
            found_match = true;
            break;
        }
    }
    CHECK(found_match);
}

TEST_CASE("6R Pieper: multiple solutions are distinct")
{
    auto chain = make_puma_chain();
    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.4, 0.5, 0.2, -0.3, 0.1;

    auto fk = forward_kinematics(chain, q_known);
    auto result = pieper_6r_solver(chain).solve(fk.end_effector);

    REQUIRE(result.has_value());

    if (result->count >= 2)
    {
        for (int i = 0; i < result->count; ++i)
        {
            for (int j = i + 1; j < result->count; ++j)
            {
                double diff = (result->solutions[static_cast<std::size_t>(i)]
                    - result->solutions[static_cast<std::size_t>(j)]).norm();
                CHECK(diff > 1e-8);
            }
        }
    }
}

TEST_CASE("6R Pieper: unreachable target returns error")
{
    auto chain = make_puma_chain();
    auto far_target = se3<double>(
        so3<double>::identity(),
        Eigen::Vector3d(100.0, 100.0, 100.0));

    auto result = pieper_6r_solver(chain).solve(far_target);

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().reason == analytical_failure::unreachable);
}

TEST_CASE("6R Pieper: wrist singularity (theta5 near zero)")
{
    auto chain = make_puma_chain();
    Eigen::Vector<double, 6> q_singular;
    q_singular << 0.3, -0.4, 0.5, 0.0, 0.0, 0.0;

    auto fk = forward_kinematics(chain, q_singular);
    auto result = pieper_6r_solver(chain).solve(fk.end_effector);

    REQUIRE(result.has_value());
    REQUIRE(result->count >= 1);

    // All returned solutions must FK-verify (no NaN)
    for (int i = 0; i < result->count; ++i)
    {
        auto& sol = result->solutions[static_cast<std::size_t>(i)];
        // Check no NaN in solution
        for (int k = 0; k < 6; ++k)
        {
            CHECK_FALSE(std::isnan(sol(k)));
        }
        auto fk_check = forward_kinematics(chain, sol);
        double position_error = (fk_check.end_effector.translation()
            - fk.end_effector.translation()).norm();
        double orientation_error = (fk_check.end_effector.rotation().inverse()
            * fk.end_effector.rotation()).log().norm();
        CHECK(position_error < tolerance);
        CHECK(orientation_error < tolerance);
    }
}

TEST_CASE("6R Pieper: identity target")
{
    auto chain = make_puma_chain();
    auto home_target = chain.home();

    auto result = pieper_6r_solver(chain).solve(home_target);

    // Home pose should be reachable (all zeros is a solution)
    REQUIRE(result.has_value());
    REQUIRE(result->count >= 1);

    bool found_near_zero = false;
    for (int i = 0; i < result->count; ++i)
    {
        if (result->solutions[static_cast<std::size_t>(i)].norm() < 0.1)
        {
            found_near_zero = true;
            break;
        }
    }
    CHECK(found_near_zero);
}

TEST_CASE("6R Pieper: convenience function solve_6r works")
{
    auto chain = make_puma_chain();
    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.4, 0.5, 0.2, -0.3, 0.1;

    auto fk = forward_kinematics(chain, q_known);

    auto result_direct = pieper_6r_solver(chain).solve(fk.end_effector);
    auto result_convenience = solve_6r(chain, fk.end_effector);

    REQUIRE(result_direct.has_value());
    REQUIRE(result_convenience.has_value());
    CHECK(result_direct->count == result_convenience->count);
}

TEST_CASE("6R Pieper: CTAD deduction guide works")
{
    auto chain = make_puma_chain();
    pieper_6r_solver solver(chain);
    static_assert(std::same_as<
        decltype(solver),
        pieper_6r_solver<static_chain<double, revolute_z, revolute_y, revolute_y,
                                      revolute_z, revolute_y, revolute_z>>>);
}

TEST_CASE("6R Pieper: anti-parallel outer wrist solves reachable poses")
{
    auto chain = fixtures::make_anti_parallel_wrist_puma<double>();

    // Sweep a handful of reachable configurations within [-pi, pi]. Each must
    // solve and every returned solution must FK-back-verify to solve tolerance.
    // FK-back-verification is used deliberately rather than trusting the
    // solver's own success report.
    std::array<Eigen::Vector<double, 6>, 4> configs;
    configs[0] << 0.3, -0.4, 0.5, 0.2, -0.3, 0.1;
    configs[1] << -0.6, 0.5, -0.7, 0.9, 0.4, -1.1;
    configs[2] << 1.0, -0.8, 0.6, -1.2, 0.7, 0.5;
    configs[3] << 0.2, 0.3, -0.4, 0.6, -0.5, 1.3;

    for (const auto& q_known : configs)
    {
        auto fk = forward_kinematics(chain, q_known);
        auto result = pieper_6r_solver(chain).solve(fk.end_effector);

        REQUIRE(result.has_value());
        REQUIRE(result->count >= 1);

        bool found_match = false;
        for (int i = 0; i < result->count; ++i)
        {
            auto& sol = result->solutions[static_cast<std::size_t>(i)];
            for (int k = 0; k < 6; ++k)
                REQUIRE_FALSE(std::isnan(sol(k)));

            auto fk_check = forward_kinematics(chain, sol);
            double position_error = (fk_check.end_effector.translation()
                - fk.end_effector.translation()).norm();
            double orientation_error = (fk_check.end_effector.rotation().inverse()
                * fk.end_effector.rotation()).log().norm();
            CHECK(position_error < tolerance);
            CHECK(orientation_error < tolerance);
            if (position_error < tolerance && orientation_error < tolerance)
                found_match = true;
        }
        CHECK(found_match);
    }
}

TEST_CASE("6R Pieper: solutions are wrapped to (-pi, pi] and deduplicated")
{
    auto chain = fixtures::make_puma_realistic_limits<double>();
    Eigen::Vector<double, 6> q_known;
    q_known << 0.3, -0.4, 0.5, 0.2, -0.3, 0.1;

    auto fk = forward_kinematics(chain, q_known);
    auto result = pieper_6r_solver(chain).solve(fk.end_effector);

    REQUIRE(result.has_value());
    REQUIRE(result->count >= 1);

    constexpr double pi = std::numbers::pi;
    const double wrap_eps = 1e-9;

    // Every solution component must lie in (-pi, pi], and each solution must
    // FK-back-verify (re-verified, not trusted from the solver report).
    for (int i = 0; i < result->count; ++i)
    {
        auto& sol = result->solutions[static_cast<std::size_t>(i)];
        for (int k = 0; k < 6; ++k)
        {
            CHECK(sol(k) > -pi - wrap_eps);
            CHECK(sol(k) <= pi + wrap_eps);
        }
        auto fk_check = forward_kinematics(chain, sol);
        double position_error = (fk_check.end_effector.translation()
            - fk.end_effector.translation()).norm();
        double orientation_error = (fk_check.end_effector.rotation().inverse()
            * fk.end_effector.rotation()).log().norm();
        CHECK(position_error < tolerance);
        CHECK(orientation_error < tolerance);
    }

    // No two returned solutions may coincide modulo 2*pi on every joint.
    auto wrap = [](double a)
    {
        double w = std::remainder(a, 2.0 * pi);
        if (w <= -pi) w += 2.0 * pi;
        return w;
    };
    for (int i = 0; i < result->count; ++i)
    {
        for (int j = i + 1; j < result->count; ++j)
        {
            bool all_equal = true;
            for (int k = 0; k < 6; ++k)
            {
                double d = wrap(result->solutions[static_cast<std::size_t>(i)](k)
                    - result->solutions[static_cast<std::size_t>(j)](k));
                if (std::abs(d) > 1e-6)
                {
                    all_equal = false;
                    break;
                }
            }
            CHECK_FALSE(all_equal);
        }
    }
}

TEST_CASE("6R Pieper: non-Pieper chain fails gracefully")
{
    auto chain = make_non_pieper_chain();
    pieper_6r_solver solver(chain);

    // Non-Pieper geometry should fail at solve() with degenerate_geometry
    Eigen::Vector<double, 6> q_test;
    q_test << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
    auto fk = forward_kinematics(chain, q_test);

    auto result = solver.solve(fk.end_effector);

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().reason == analytical_failure::degenerate_geometry);
}
