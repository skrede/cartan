#include "cartan/analytical.h"
#include "cartan/serial_chain.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstddef>
#include <numbers>

using namespace cartan;
using Catch::Matchers::WithinAbs;

static constexpr double tolerance = 1e-6;

// 2R chain: two revolute_y joints in the XZ plane.
// Joint 0 at origin, Joint 1 at (L1, 0, 0). Home EE at (L1+L2, 0, 0).
auto make_2r_chain(double L1, double L2)
{
    auto s0 = screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0});
    auto s1 = screw_axis<double>::revolute({0, 1, 0}, {L1, 0, 0});
    auto home = se3<double>(
        so3<double>::identity(),
        Eigen::Vector3d(L1 + L2, 0, 0));
    joint_limits<double> no_limits{-10.0, 10.0};
    return static_chain<double, revolute_y, revolute_y>(
        home, {s0, s1}, {no_limits, no_limits});
}

static se3<double> target_at(double x, double y, double z)
{
    return se3<double>(so3<double>::identity(), Eigen::Vector3d(x, y, z));
}

TEST_CASE("2R solver: reachable interior target returns 2 solutions")
{
    auto chain = make_2r_chain(1.0, 1.0);
    auto result = planar_2r_solver(chain).solve(target_at(1.0, 0, 0));

    REQUIRE(result.has_value());
    REQUIRE(result->count == 2);

    for (std::size_t i = 0; i < static_cast<std::size_t>(result->count); ++i)
    {
        auto fk = forward_kinematics(chain, result->solutions[i]);
        double error = (fk.end_effector.translation()
            - Eigen::Vector3d(1.0, 0, 0)).norm();
        CHECK(error < tolerance);
    }
}

TEST_CASE("2R solver: fully extended boundary returns 1 solution")
{
    auto chain = make_2r_chain(1.0, 1.0);
    auto result = planar_2r_solver(chain).solve(target_at(2.0, 0, 0));

    REQUIRE(result.has_value());
    CHECK(result->count == 1);

    auto fk = forward_kinematics(chain, result->solutions[0]);
    double error = (fk.end_effector.translation()
        - Eigen::Vector3d(2.0, 0, 0)).norm();
    CHECK(error < tolerance);
}

TEST_CASE("2R solver: fully folded boundary returns 1 solution")
{
    auto chain = make_2r_chain(2.0, 1.0);
    auto result = planar_2r_solver(chain).solve(target_at(1.0, 0, 0));

    REQUIRE(result.has_value());
    CHECK(result->count == 1);
}

TEST_CASE("2R solver: unreachable target returns error")
{
    auto chain = make_2r_chain(1.0, 1.0);
    auto result = planar_2r_solver(chain).solve(target_at(3.0, 0, 0));

    REQUIRE(!result.has_value());
    CHECK(result.error().reason == analytical_failure::unreachable);
    CHECK(result.error().workspace_distance > 0);
}

TEST_CASE("2R solver: unreachable target inside hole returns error")
{
    auto chain = make_2r_chain(3.0, 1.0);
    auto result = planar_2r_solver(chain).solve(target_at(0, 0, 0));

    REQUIRE(!result.has_value());
    CHECK(result.error().reason == analytical_failure::unreachable);
}

TEST_CASE("2R solver: solutions are distinct configurations")
{
    auto chain = make_2r_chain(1.0, 1.0);
    auto result = planar_2r_solver(chain).solve(target_at(1.0, 0, 0.5));

    REQUIRE(result.has_value());
    REQUIRE(result->count == 2);
    double diff = (result->solutions[0] - result->solutions[1]).norm();
    CHECK(diff > 0.01);
}

TEST_CASE("2R solver: convenience function solve_2r matches solver")
{
    auto chain = make_2r_chain(1.0, 1.0);
    auto target = target_at(1.0, 0, 0.5);

    auto result_solver = planar_2r_solver(chain).solve(target);
    auto result_free = solve_2r(chain, target);

    REQUIRE(result_solver.has_value());
    REQUIRE(result_free.has_value());
    CHECK(result_solver->count == result_free->count);

    for (std::size_t i = 0; i < static_cast<std::size_t>(result_solver->count); ++i)
    {
        double diff = (result_solver->solutions[i]
            - result_free->solutions[i]).norm();
        CHECK(diff < tolerance);
    }
}

TEST_CASE("2R solver: CTAD deduction guide works")
{
    auto chain = make_2r_chain(1.0, 1.0);
    planar_2r_solver solver(chain);
    auto result = solver.solve(target_at(1.0, 0, 0));
    REQUIRE(result.has_value());
}

// 2R chain with a BENT home: the second link leaves the first-link direction
// at a known angle at the home configuration. The closed form carries this
// constant home-bend angle into the joint-2 solutions.
static auto make_bent_home_2r_chain(double L1, double L2)
{
    auto s0 = screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0});
    auto s1 = screw_axis<double>::revolute({0, 1, 0}, {L1, 0, 0});
    // Home end-effector bent out of the first-link direction (along +z).
    auto home = se3<double>(
        so3<double>::identity(),
        Eigen::Vector3d(L1, 0, L2));
    joint_limits<double> no_limits{-10.0, 10.0};
    return static_chain<double, revolute_y, revolute_y>(
        home, {s0, s1}, {no_limits, no_limits});
}

TEST_CASE("2R solver: factory validates a straight-home chain")
{
    auto chain = make_2r_chain(1.0, 1.0);
    auto solver = planar_2r_solver<decltype(chain)>::make(chain);

    REQUIRE(solver.has_value());

    auto result = solver->solve(target_at(1.0, 0, 0));
    REQUIRE(result.has_value());
    CHECK(result->count == 2);
}

TEST_CASE("2R solver: bent home is solved and FK-reconstructs the target")
{
    auto chain = make_bent_home_2r_chain(1.0, 1.0);
    auto solver = planar_2r_solver<decltype(chain)>::make(chain);

    // The bent geometry is now admitted rather than rejected.
    REQUIRE(solver.has_value());

    // Home EE sits at (L1, 0, L2); its neighborhood is reachable.
    auto target = target_at(0.8, 0, 0.9);
    auto result = solver->solve(target);

    REQUIRE(result.has_value());
    REQUIRE(result->count > 0);
    for (std::size_t i = 0; i < static_cast<std::size_t>(result->count); ++i)
    {
        auto fk = forward_kinematics(chain, result->solutions[i]);
        double error = (fk.end_effector.translation()
            - Eigen::Vector3d(0.8, 0, 0.9)).norm();
        CHECK(error < tolerance);
    }
}

TEST_CASE("2R solver: bent home recovers the target at the home configuration")
{
    auto chain = make_bent_home_2r_chain(1.0, 1.0);
    auto solver = planar_2r_solver<decltype(chain)>::make(chain);
    REQUIRE(solver.has_value());

    // Zero joints must map back to the bent home end-effector (1, 0, 1).
    auto result = solver->solve(target_at(1.0, 0, 1.0));
    REQUIRE(result.has_value());
    REQUIRE(result->count > 0);
    for (std::size_t i = 0; i < static_cast<std::size_t>(result->count); ++i)
    {
        auto fk = forward_kinematics(chain, result->solutions[i]);
        double error = (fk.end_effector.translation()
            - Eigen::Vector3d(1.0, 0, 1.0)).norm();
        CHECK(error < tolerance);
    }
}

TEST_CASE("2R solver: different link lengths")
{
    auto chain = make_2r_chain(1.5, 0.7);
    auto result = planar_2r_solver(chain).solve(target_at(1.0, 0, 0.5));

    REQUIRE(result.has_value());
    for (std::size_t i = 0; i < static_cast<std::size_t>(result->count); ++i)
    {
        auto fk = forward_kinematics(chain, result->solutions[i]);
        double error = (fk.end_effector.translation()
            - Eigen::Vector3d(1.0, 0, 0.5)).norm();
        CHECK(error < tolerance);
    }
}
