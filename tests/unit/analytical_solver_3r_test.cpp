#include "../support/kinematics_helpers.h"
#include "../support/joint_limits_helpers.h"

#include "cartan/analytical.h"
#include "cartan/serial_chain.h"

#include "../fixtures/analytical_chains.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <numbers>

using namespace cartan;
using Catch::Matchers::WithinAbs;

static constexpr double tolerance = 1e-6;

using zyz_3r_chain = static_chain<double, revolute_z, revolute_y, revolute_z>;

/// Build a ZYZ 3R chain with axes 1 and 2 intersecting at the origin.
/// Joint 0: revolute_z through origin.
/// Joint 1: revolute_y through origin.
/// Joint 2: revolute_z through (link_offset, 0, 0).
/// Home EE at (link_offset + ee_offset, 0, 0).
static zyz_3r_chain make_3r_chain(double link_offset, double ee_offset)
{
    auto s0 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s1 = screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0});
    auto s2 = screw_axis<double>::revolute({0, 0, 1}, {link_offset, 0, 0});
    auto home = se3<double>(
        so3<double>::identity(),
        Eigen::Vector3d(link_offset + ee_offset, 0, 0));
    auto no_limits = testing::limits(-10.0, 10.0);
    return testing::unwrap(
        zyz_3r_chain::make(home, {s0, s1, s2}, {no_limits, no_limits, no_limits}),
        "make_3r_chain");
}

TEST_CASE("3R solver: reachable target returns solutions")
{
    auto chain = make_3r_chain(0.5, 0.3);
    Eigen::Vector3d q_known;
    q_known << 0.3, 0.5, -0.2;

    auto fk = testing::fk_at(chain, q_known);
    auto solver = spatial_3r_solver<zyz_3r_chain>::make(chain);
    REQUIRE(solver.has_value());
    auto result = solver->solve(fk.end_effector);

    REQUIRE(result.has_value());
    REQUIRE(result->count >= 1);

    for (int i = 0; i < result->count; ++i)
    {
        auto fk_check = testing::fk_at(chain, result->solutions[static_cast<std::size_t>(i)]);
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

    auto fk = testing::fk_at(chain, q_known);
    auto solver = spatial_3r_solver<zyz_3r_chain>::make(chain);
    REQUIRE(solver.has_value());
    auto result = solver->solve(fk.end_effector);

    REQUIRE(result.has_value());

    bool found_match = false;
    for (int i = 0; i < result->count; ++i)
    {
        auto fk_check = testing::fk_at(chain, result->solutions[static_cast<std::size_t>(i)]);
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

    auto fk = testing::fk_at(chain, q_known);
    auto solver = spatial_3r_solver<zyz_3r_chain>::make(chain);
    REQUIRE(solver.has_value());
    auto result = solver->solve(fk.end_effector);

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

    auto solver = spatial_3r_solver<zyz_3r_chain>::make(chain);
    REQUIRE(solver.has_value());
    auto result = solver->solve(far_target);

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().reason == analytical_failure::unreachable);
}

TEST_CASE("3R solver: an unreachable target carries no substituted length")
{
    // The inequality that failed is inside the distance-constraint subproblem,
    // whose error channel carries no payload, so the reason travels alone.
    // Pre-fix the report carried 172.7444355, the distance from the target to
    // the home end-effector position -- a length no inequality in this solve
    // compared against anything, and one that vanishes for a target sitting at
    // the home end-effector even when the failure is real.
    auto chain = make_3r_chain(0.5, 0.3);
    const Eigen::Vector3d far_point(100.0, 100.0, 100.0);
    auto solver = spatial_3r_solver<zyz_3r_chain>::make(chain);
    REQUIRE(solver.has_value());
    auto result = solver->solve(
        se3<double>(so3<double>::identity(), far_point));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().reason == analytical_failure::unreachable);
    CHECK_FALSE(result.error().workspace_distance.has_value());
    // The premise: the substituted length really was this one.
    CHECK_THAT((far_point - chain.home().translation()).norm(),
        WithinAbs(172.7444355, 1e-6));
}

TEST_CASE("3R solver: convenience function solve_3r works")
{
    auto chain = make_3r_chain(0.5, 0.3);
    Eigen::Vector3d q_known;
    q_known << 0.3, 0.5, -0.2;

    auto fk = testing::fk_at(chain, q_known);

    auto solver = spatial_3r_solver<zyz_3r_chain>::make(chain);
    REQUIRE(solver.has_value());
    auto result_direct = solver->solve(fk.end_effector);
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

    auto fk = testing::fk_at(chain, q_test);
    auto solver = spatial_3r_solver<zyz_3r_chain>::make(chain);
    REQUIRE(solver.has_value());
    auto result = solver->solve(fk.end_effector);

    REQUIRE(result.has_value());

    for (int i = 0; i < result->count; ++i)
    {
        auto fk_check = testing::fk_at(chain, result->solutions[static_cast<std::size_t>(i)]);
        double position_error = (fk_check.end_effector.translation()
            - fk.end_effector.translation()).norm();
        CHECK(position_error < tolerance);
    }
}

TEST_CASE("3R solver: the acceptance length gates the shoulder gap before the "
          "back-check ever runs")
{
    // Axes 1 and 2 miss each other by 1e-4, a hundred times the module default
    // acceptance length, so at the default the factory refuses the chain. Raise
    // the acceptance length past the gap and the chain is admitted: the
    // subproblems never see the gap -- they solve their own equations exactly at
    // the assumed meeting point -- so candidates are generated whose FK position
    // residual lands past the module default and short of 1e-2.
    auto chain = testing::unwrap(
        fixtures::make_skew_shoulder_3r<double>(1e-4), "skew shoulder 3R chain");
    Eigen::Vector3d q_known;
    q_known << 0.3, 0.5, -0.2;
    auto target = testing::fk_at(chain, q_known).end_effector;

    auto at_default = spatial_3r_solver<zyz_3r_chain>::make(chain);
    REQUIRE_FALSE(at_default.has_value());
    CHECK(at_default.error().reason == analytical_failure::degenerate_geometry);

    auto at_loose = spatial_3r_solver<zyz_3r_chain>::make(
        chain, verification_tolerance<double>(1e-2, 1e-2));
    REQUIRE(at_loose.has_value());
    auto result = at_loose->solve(target);
    REQUIRE(result.has_value());
    REQUIRE(result->count > 0);

    for (int i = 0; i < result->count; ++i)
    {
        auto fk = testing::fk_at(
            chain, result->solutions[static_cast<std::size_t>(i)]);
        double residual =
            (fk.end_effector.translation() - target.translation()).norm();
        CHECK(residual > 1e-6);
        CHECK(residual < 1e-2);
    }
}

TEST_CASE("3R solver: the configured acceptance tolerance reaches the FK "
          "back-check, position field first")
{
    // Same shape as the 2R case: this solver checks position only, so a zero
    // position field must refuse every candidate while a zero orientation field
    // leaves them all standing. One field at a time -- a pair of zeros would be
    // satisfied by either and pin neither.
    auto chain = make_3r_chain(0.5, 0.3);
    Eigen::Vector3d q_known;
    q_known << 0.3, 0.5, -0.2;
    auto target = testing::fk_at(chain, q_known).end_effector;

    auto solve_at = [&](verification_tolerance<double> acceptance)
    {
        auto solver = spatial_3r_solver<zyz_3r_chain>::make(chain, acceptance);
        REQUIRE(solver.has_value());
        return solver->solve(target);
    };

    auto defaulted = spatial_3r_solver<zyz_3r_chain>::make(chain);
    REQUIRE(defaulted.has_value());
    auto at_default = defaulted->solve(target);
    REQUIRE(at_default.has_value());
    REQUIRE(at_default->count > 0);

    auto lax_position = solve_at(verification_tolerance<double>(1e-2, 0.0));
    REQUIRE(lax_position.has_value());
    CHECK(lax_position->count == at_default->count);

    auto zero_position = solve_at(verification_tolerance<double>(0.0, 1e-2));
    REQUIRE_FALSE(zero_position.has_value());
    CHECK(zero_position.error().reason == analytical_failure::verification_failed);
}

// The decomposition rotates about the point where the first two axes meet, and
// a skew pair has none. Before this factory existed there was no validated
// construction path to fail against at all: the public constructor accepted the
// chain silently, the decomposition ran against a reference point wrong by half
// the separation, and the failure surfaced per pose as a failed back-check.
TEST_CASE("3R solver: a skew shoulder is refused at construction")
{
    auto chain = testing::unwrap(
        fixtures::make_skew_shoulder_3r<double>(0.15), "skew shoulder 3R chain");

    auto solver = spatial_3r_solver<zyz_3r_chain>::make(chain);

    REQUIRE_FALSE(solver.has_value());
    CHECK(solver.error().reason == analytical_failure::degenerate_geometry);
    CHECK_FALSE(solver.error().workspace_distance.has_value());
}

// A parallel shoulder is refused ahead of the meeting-point test, because the
// shared closest-approach helpers answer a parallel pair with a fallback point
// rather than reporting, so a meeting-point test alone would pass it. The two
// shoulder axes here are the same line, so their closest approach is zero and
// the meeting-point test does pass it: only the parallel test refuses this
// chain. Before this factory existed there was no validated construction path
// to fail against at all -- the constructor accepted the chain silently and
// every solve failed per pose as a failed back-check.
TEST_CASE("3R solver: a parallel shoulder is refused at construction")
{
    using yyy_3r_chain = static_chain<double, revolute_y, revolute_y, revolute_y>;
    auto no_limits = testing::limits(-10.0, 10.0);
    auto chain = testing::unwrap(
        yyy_3r_chain::make(
            se3<double>(so3<double>::identity(), Eigen::Vector3d(0.7, 0, 0)),
            {screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0}),
             screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0}),
             screw_axis<double>::revolute({0, 1, 0}, {0.4, 0, 0})},
            {no_limits, no_limits, no_limits}),
        "parallel shoulder 3R chain");

    auto solver = spatial_3r_solver<yyy_3r_chain>::make(chain);

    REQUIRE_FALSE(solver.has_value());
    CHECK(solver.error().reason == analytical_failure::degenerate_geometry);
    CHECK_FALSE(solver.error().workspace_distance.has_value());
}

// The reference point is where the two shoulder axes meet, which is not in
// general either axis's stored point: a screw axis stores the foot of the
// perpendicular from the origin, and here the axes meet at (0, 0, 1) while
// those feet are (0, 0, 0) and (0, 0, 1). The solver's own line-intersection
// helper negated both line parameters, so it answered (0, 0, 0) -- the meeting
// point reflected through the first foot -- and every solve failed.
TEST_CASE("3R solver: shoulder axes meeting away from their stored points are solved")
{
    auto no_limits = testing::limits(-10.0, 10.0);
    auto chain = testing::unwrap(
        zyz_3r_chain::make(
            se3<double>(so3<double>::identity(), Eigen::Vector3d(0.8, 0, 1.0)),
            {screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0}),
             screw_axis<double>::revolute({0, 1, 0}, {0, 0, 1.0}),
             screw_axis<double>::revolute({0, 0, 1}, {0.5, 0, 1.0})},
            {no_limits, no_limits, no_limits}),
        "raised shoulder 3R chain");

    auto solver = spatial_3r_solver<zyz_3r_chain>::make(chain);
    REQUIRE(solver.has_value());

    Eigen::Vector3d q_known;
    q_known << 0.3, 0.5, -0.2;
    auto target = testing::fk_at(chain, q_known).end_effector;
    auto result = solver->solve(target);

    REQUIRE(result.has_value());
    REQUIRE(result->count >= 1);
    for (int i = 0; i < result->count; ++i)
    {
        auto fk = testing::fk_at(
            chain, result->solutions[static_cast<std::size_t>(i)]);
        CHECK((fk.end_effector.translation() - target.translation()).norm()
            < tolerance);
    }
}
