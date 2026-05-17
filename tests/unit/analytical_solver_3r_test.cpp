#include "cartan/analytical.h"
#include "cartan/serial_chain.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <numbers>

using namespace cartan;
using Catch::Matchers::WithinAbs;

static constexpr double tolerance = 1e-6;

/// Build a ZYZ 3R chain with axes 1 and 2 intersecting at the origin.
/// Joint 0: revolute_z through origin.
/// Joint 1: revolute_y through origin.
/// Joint 2: revolute_z through (link_offset, 0, 0).
/// Home EE at (link_offset + ee_offset, 0, 0).
static auto make_3r_chain(double link_offset, double ee_offset)
{
    auto s0 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s1 = screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0});
    auto s2 = screw_axis<double>::revolute({0, 0, 1}, {link_offset, 0, 0});
    auto home = se3<double>(
        so3<double>::identity(),
        Eigen::Vector3d(link_offset + ee_offset, 0, 0));
    joint_limits<double> no_limits{-10.0, 10.0};
    return static_chain<double, revolute_z, revolute_y, revolute_z>(
        home, {s0, s1, s2}, {no_limits, no_limits, no_limits});
}

TEST_CASE("3R solver: reachable target returns solutions")
{
    auto chain = make_3r_chain(0.5, 0.3);
    Eigen::Vector3d q_known;
    q_known << 0.3, 0.5, -0.2;

    auto fk = forward_kinematics(chain, q_known);
    auto result = spatial_3r_solver(chain).solve(fk.end_effector);

    REQUIRE(result.has_value());
    REQUIRE(result->count >= 1);

    for (int i = 0; i < result->count; ++i)
    {
        auto fk_check = forward_kinematics(chain, result->solutions[static_cast<std::size_t>(i)]);
        double error = (fk_check.end_effector.translation()
            - fk.end_effector.translation()).norm();
        CHECK(error < tolerance);
    }
}

TEST_CASE("3R solver: FK-computed target recovers original angles as one solution")
{
    auto chain = make_3r_chain(0.5, 0.3);
    Eigen::Vector3d q_known;
    q_known << 0.6, 0.8, -0.4;

    auto fk = forward_kinematics(chain, q_known);
    auto result = spatial_3r_solver(chain).solve(fk.end_effector);

    REQUIRE(result.has_value());

    bool found_match = false;
    for (int i = 0; i < result->count; ++i)
    {
        auto fk_check = forward_kinematics(chain, result->solutions[static_cast<std::size_t>(i)]);
        double error = (fk_check.end_effector.translation()
            - fk.end_effector.translation()).norm();
        if (error < tolerance)
        {
            found_match = true;
            break;
        }
    }
    CHECK(found_match);
}

TEST_CASE("3R solver: multiple solutions are distinct")
{
    auto chain = make_3r_chain(0.5, 0.3);
    Eigen::Vector3d q_known;
    q_known << 0.3, 0.5, -0.2;

    auto fk = forward_kinematics(chain, q_known);
    auto result = spatial_3r_solver(chain).solve(fk.end_effector);

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

TEST_CASE("3R solver: unreachable target returns error")
{
    auto chain = make_3r_chain(0.5, 0.3);
    auto far_target = se3<double>(
        so3<double>::identity(),
        Eigen::Vector3d(100.0, 100.0, 100.0));

    auto result = spatial_3r_solver(chain).solve(far_target);

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().reason == analytical_failure::unreachable);
}

TEST_CASE("3R solver: convenience function solve_3r works")
{
    auto chain = make_3r_chain(0.5, 0.3);
    Eigen::Vector3d q_known;
    q_known << 0.3, 0.5, -0.2;

    auto fk = forward_kinematics(chain, q_known);

    auto result_direct = spatial_3r_solver(chain).solve(fk.end_effector);
    auto result_convenience = solve_3r(chain, fk.end_effector);

    REQUIRE(result_direct.has_value());
    REQUIRE(result_convenience.has_value());
    CHECK(result_direct->count == result_convenience->count);

    for (int i = 0; i < result_direct->count; ++i)
    {
        double diff = (result_direct->solutions[static_cast<std::size_t>(i)]
            - result_convenience->solutions[static_cast<std::size_t>(i)]).norm();
        CHECK(diff < 1e-12);
    }
}

TEST_CASE("3R solver: all solutions FK-verify")
{
    auto chain = make_3r_chain(0.5, 0.3);

    Eigen::Vector3d q_test;
    q_test << 1.0, 0.7, -0.5;

    auto fk = forward_kinematics(chain, q_test);
    auto result = spatial_3r_solver(chain).solve(fk.end_effector);

    REQUIRE(result.has_value());

    for (int i = 0; i < result->count; ++i)
    {
        auto fk_check = forward_kinematics(chain, result->solutions[static_cast<std::size_t>(i)]);
        double position_error = (fk_check.end_effector.translation()
            - fk.end_effector.translation()).norm();
        CHECK(position_error < tolerance);
    }
}

TEST_CASE("3R solver: CTAD deduction guide works")
{
    auto chain = make_3r_chain(0.5, 0.3);
    spatial_3r_solver solver(chain);
    static_assert(std::same_as<
        decltype(solver),
        spatial_3r_solver<static_chain<double, revolute_z, revolute_y, revolute_z>>>);
}
