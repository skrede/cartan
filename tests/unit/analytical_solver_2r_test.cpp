#include "../support/kinematics_helpers.h"
#include "../support/joint_limits_helpers.h"

#include "cartan/analytical.h"
#include "cartan/serial_chain.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <vector>
#include <cstddef>
#include <numbers>

using namespace cartan;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

static constexpr double check_tolerance = 1e-6;

using planar_2r_chain = static_chain<double, revolute_y, revolute_y>;

// 2R chain: two revolute_y joints in the XZ plane.
// Joint 0 at origin, Joint 1 at (L1, 0, 0). Home EE at (L1+L2, 0, 0).
planar_2r_chain make_2r_chain(double L1, double L2)
{
    auto s0 = screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0});
    auto s1 = screw_axis<double>::revolute({0, 1, 0}, {L1, 0, 0});
    auto home = se3<double>(
        so3<double>::identity(),
        Eigen::Vector3d(L1 + L2, 0, 0));
    auto no_limits = testing::limits(-10.0, 10.0);
    return testing::unwrap(
        planar_2r_chain::make(home, {s0, s1}, {no_limits, no_limits}), "make_2r_chain");
}

static se3<double> target_at(double x, double y, double z)
{
    return se3<double>(so3<double>::identity(), Eigen::Vector3d(x, y, z));
}

TEST_CASE("2R solver: reachable interior target returns 2 solutions")
{
    auto chain = make_2r_chain(1.0, 1.0);
    auto solver = planar_2r_solver<planar_2r_chain>::make(chain);
    REQUIRE(solver.has_value());
    auto result = solver->solve(target_at(1.0, 0, 0));

    REQUIRE(result.has_value());
    REQUIRE(result->count == 2);

    for (std::size_t i = 0; i < static_cast<std::size_t>(result->count); ++i)
    {
        auto fk = testing::fk_at(chain, result->solutions[i]);
        double error = (fk.end_effector.translation()
            - Eigen::Vector3d(1.0, 0, 0)).norm();
        CHECK(error < check_tolerance);
    }
}

TEST_CASE("2R solver: fully extended boundary returns 1 solution")
{
    auto chain = make_2r_chain(1.0, 1.0);
    auto solver = planar_2r_solver<planar_2r_chain>::make(chain);
    REQUIRE(solver.has_value());
    auto result = solver->solve(target_at(2.0, 0, 0));

    REQUIRE(result.has_value());
    CHECK(result->count == 1);

    auto fk = testing::fk_at(chain, result->solutions[0]);
    double error = (fk.end_effector.translation()
        - Eigen::Vector3d(2.0, 0, 0)).norm();
    CHECK(error < check_tolerance);
}

TEST_CASE("2R solver: fully folded boundary returns 1 solution")
{
    auto chain = make_2r_chain(2.0, 1.0);
    auto solver = planar_2r_solver<planar_2r_chain>::make(chain);
    REQUIRE(solver.has_value());
    auto result = solver->solve(target_at(1.0, 0, 0));

    REQUIRE(result.has_value());
    CHECK(result->count == 1);
}

TEST_CASE("2R solver: unreachable target returns error")
{
    auto chain = make_2r_chain(1.0, 1.0);
    auto solver = planar_2r_solver<planar_2r_chain>::make(chain);
    REQUIRE(solver.has_value());
    auto result = solver->solve(target_at(3.0, 0, 0));

    REQUIRE(!result.has_value());
    CHECK(result.error().reason == analytical_failure::unreachable);
    REQUIRE(result.error().workspace_distance.has_value());
    CHECK(*result.error().workspace_distance > 0);
}

TEST_CASE("2R solver: unreachable target inside hole returns error")
{
    auto chain = make_2r_chain(3.0, 1.0);
    auto solver = planar_2r_solver<planar_2r_chain>::make(chain);
    REQUIRE(solver.has_value());
    auto result = solver->solve(target_at(0, 0, 0));

    REQUIRE(!result.has_value());
    CHECK(result.error().reason == analytical_failure::unreachable);
}

TEST_CASE("2R solver: solutions are distinct configurations")
{
    auto chain = make_2r_chain(1.0, 1.0);
    auto solver = planar_2r_solver<planar_2r_chain>::make(chain);
    REQUIRE(solver.has_value());
    auto result = solver->solve(target_at(1.0, 0, 0.5));

    REQUIRE(result.has_value());
    REQUIRE(result->count == 2);
    double diff = (result->solutions[0] - result->solutions[1]).norm();
    CHECK(diff > 0.01);
}

TEST_CASE("2R solver: convenience function solve_2r matches solver")
{
    auto chain = make_2r_chain(1.0, 1.0);
    auto target = target_at(1.0, 0, 0.5);

    auto solver = planar_2r_solver<planar_2r_chain>::make(chain);
    REQUIRE(solver.has_value());
    auto result_solver = solver->solve(target);
    auto result_free = solve_2r(chain, target);

    REQUIRE(result_solver.has_value());
    REQUIRE(result_free.has_value());
    CHECK(result_solver->count == result_free->count);

    for (std::size_t i = 0; i < static_cast<std::size_t>(result_solver->count); ++i)
    {
        double diff = (result_solver->solutions[i]
            - result_free->solutions[i]).norm();
        CHECK(diff < check_tolerance);
    }
}

// 2R chain with a BENT home: the second link leaves the first-link direction
// at a known angle at the home configuration. The closed form carries this
// constant home-bend angle into the joint-2 solutions.
static planar_2r_chain make_bent_home_2r_chain(double L1, double L2)
{
    auto s0 = screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0});
    auto s1 = screw_axis<double>::revolute({0, 1, 0}, {L1, 0, 0});
    // Home end-effector bent out of the first-link direction (along +z).
    auto home = se3<double>(
        so3<double>::identity(),
        Eigen::Vector3d(L1, 0, L2));
    auto no_limits = testing::limits(-10.0, 10.0);
    return testing::unwrap(
        planar_2r_chain::make(home, {s0, s1}, {no_limits, no_limits}), "make_bent_home_2r_chain");
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
        auto fk = testing::fk_at(chain, result->solutions[i]);
        double error = (fk.end_effector.translation()
            - Eigen::Vector3d(0.8, 0, 0.9)).norm();
        CHECK(error < check_tolerance);
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
        auto fk = testing::fk_at(chain, result->solutions[i]);
        double error = (fk.end_effector.translation()
            - Eigen::Vector3d(1.0, 0, 1.0)).norm();
        CHECK(error < check_tolerance);
    }
}

TEST_CASE("2R solver: different link lengths")
{
    auto chain = make_2r_chain(1.5, 0.7);
    auto solver = planar_2r_solver<planar_2r_chain>::make(chain);
    REQUIRE(solver.has_value());
    auto result = solver->solve(target_at(1.0, 0, 0.5));

    REQUIRE(result.has_value());
    for (std::size_t i = 0; i < static_cast<std::size_t>(result->count); ++i)
    {
        auto fk = testing::fk_at(chain, result->solutions[i]);
        double error = (fk.end_effector.translation()
            - Eigen::Vector3d(1.0, 0, 0.5)).norm();
        CHECK(error < check_tolerance);
    }
}

TEST_CASE("2R solver: the configured acceptance tolerance reaches the FK "
          "back-check, position field first")
{
    // The planar closed form is exact on a planar chain -- its residual is
    // round-off -- so no geometry puts a residual between a tight configured
    // bound and the module default. A threshold of zero needs no band: no norm
    // is below zero. The two fields are driven to zero one at a time, because a
    // pair of zeros would be satisfied by either field and pin neither.
    //
    // This solver checks position only, so the position field must decide both
    // probes: a loose position field admits, a zero one refuses, and the
    // orientation field never enters.
    auto chain = make_2r_chain(1.0, 1.0);
    auto target = target_at(1.0, 0, 0.5);

    auto solve_at = [&](verification_tolerance<double> acceptance)
    {
        auto solver = planar_2r_solver<planar_2r_chain>::make(chain, acceptance);
        REQUIRE(solver.has_value());
        return solver->solve(target);
    };

    auto defaulted = planar_2r_solver<planar_2r_chain>::make(chain);
    REQUIRE(defaulted.has_value());
    auto at_default = defaulted->solve(target);
    REQUIRE(at_default.has_value());
    CHECK(at_default->count == 2);

    auto lax_position = solve_at(verification_tolerance<double>(1e-2, 0.0));
    REQUIRE(lax_position.has_value());
    CHECK(lax_position->count == 2);

    auto zero_position = solve_at(verification_tolerance<double>(0.0, 1e-2));
    REQUIRE_FALSE(zero_position.has_value());
    CHECK(zero_position.error().reason == analytical_failure::verification_failed);
}

// The assertion is on the reason and not on the success flag. Equal links
// reaching their own base point divide zero by zero for the shoulder angle, and
// what a caller sees of that depends on which downstream guard catches the
// resulting candidate; the reason is wrong in every configuration, so it is the
// one observable that pins the classification.
TEST_CASE("2R solver: equal links reaching the base point are singular")
{
    auto chain = make_2r_chain(1.0, 1.0);
    auto solver = planar_2r_solver<planar_2r_chain>::make(chain);
    REQUIRE(solver.has_value());
    auto result = solver->solve(target_at(0, 0, 0));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().reason == analytical_failure::singular_configuration);
    CHECK_FALSE(result.error().workspace_distance.has_value());
}

// Reaching the base point folds one link back along the other, so the deficit
// is exactly the length the shorter link falls short by. The expectation is
// derived from the two lengths the fixture is built from, never from the
// solver. While the reach gates compared squared lengths against a
// dimensionless constant, a difference this small left the target inside the
// gate: a genuine reach violation was reported as a failed back-check carrying
// no deficit at all.
TEST_CASE("2R solver: a near-equal-link base-point target carries its deficit")
{
    constexpr double link_1 = 1.0;
    constexpr double link_2 = 1.0 - 1e-4;
    auto chain = make_2r_chain(link_1, link_2);
    auto solver = planar_2r_solver<planar_2r_chain>::make(chain);
    REQUIRE(solver.has_value());
    auto result = solver->solve(target_at(0, 0, 0));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().reason == analytical_failure::unreachable);
    REQUIRE(result.error().workspace_distance.has_value());
    CHECK_THAT(*result.error().workspace_distance,
        WithinRel(std::abs(link_1 - link_2), 1e-9));
}

// A reach gate and the back-check must agree on what counts as the same point:
// a gate tighter than the acceptance length refuses a target whose solution the
// same solver would go on to certify. This target is a hundredth of the
// acceptance length beyond the fully extended boundary, and against squared
// lengths offset by a dimensionless constant the gate was tighter than that by
// orders of magnitude and refused it as unreachable.
TEST_CASE("2R solver: a target inside the acceptance length of the boundary is solved")
{
    auto chain = make_2r_chain(1.0, 1.0);
    auto reached = Eigen::Vector3d(2.0 + 1e-8, 0, 0);
    auto solver = planar_2r_solver<planar_2r_chain>::make(chain);
    REQUIRE(solver.has_value());
    auto result = solver->solve(target_at(reached.x(), 0, 0));

    REQUIRE(result.has_value());
    REQUIRE(result->count == 1);
    auto fk = testing::fk_at(chain, result->solutions[0]);
    CHECK((fk.end_effector.translation() - reached).norm() < check_tolerance);
}

// The fixture's joints turn about y, so its mechanism plane is the xz plane and
// a y displacement leaves it. The in-plane part of this target sits mid-annulus
// and is comfortably reachable, so the case pins the plane precondition rather
// than a reach violation. Until the precondition was enforced at solve time the
// derivation's in-plane assumption was simply taken: the solver answered for the
// in-plane shadow, and the FK back-check threw that answer away and reported a
// failed verification.
TEST_CASE("2R solver: a target off the mechanism plane is unreachable")
{
    constexpr double out_of_plane = 0.1;
    auto chain = make_2r_chain(1.0, 1.0);
    auto solver = planar_2r_solver<planar_2r_chain>::make(chain);
    REQUIRE(solver.has_value());
    auto result = solver->solve(target_at(1.0, out_of_plane, 0));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().reason == analytical_failure::unreachable);
    REQUIRE(result.error().workspace_distance.has_value());
    CHECK_THAT(*result.error().workspace_distance, WithinRel(out_of_plane, 1e-12));
}

// The derivation needs a plane both rotations move in, and two perpendicular
// axes offer none. Until the factory tested for it the chain was admitted: the
// constructor manufactured a plane normal from the cross product of the two
// axes, a plane containing neither rotation's motion circle, and every solve
// then reported `unreachable` carrying the target's component along the first
// link -- a length no reach inequality had compared against anything.
TEST_CASE("2R solver: perpendicular axes are refused at construction")
{
    using perpendicular_2r_chain = static_chain<double, revolute_y, revolute_z>;
    auto no_limits = testing::limits(-10.0, 10.0);
    auto chain = testing::unwrap(
        perpendicular_2r_chain::make(
            se3<double>(so3<double>::identity(), Eigen::Vector3d(2, 0, 0)),
            {screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0}),
             screw_axis<double>::revolute({0, 0, 1}, {1, 0, 0})},
            {no_limits, no_limits}),
        "perpendicular 2R chain");

    auto solver = planar_2r_solver<perpendicular_2r_chain>::make(chain);

    REQUIRE_FALSE(solver.has_value());
    CHECK(solver.error().reason == analytical_failure::degenerate_geometry);
    CHECK_FALSE(solver.error().workspace_distance.has_value());
}

// The joint kinds are carried in the type of a static_chain, so this gate is
// only reachable through a runtime-sized chain. The first axis is placed off the
// origin deliberately: a prismatic axis recovers its joint point as omega x v,
// which is zero, so a chain carrying one alongside an axis through the origin is
// refused by the zero-length-link gate whatever the joint-kind gate does. Offset
// this way the chain clears every other gate, and only the joint-kind gate
// accounts for the refusal.
TEST_CASE("2R solver: a prismatic joint is refused at construction")
{
    std::vector<screw_axis<double>> axes
        = {screw_axis<double>::revolute({0, 1, 0}, {0, 0, 1}),
           screw_axis<double>::prismatic({0, 0, 1})};
    std::vector<joint_limits<double>> limits(2, testing::limits(-10.0, 10.0));
    kinematic_chain<double, dynamic> chain(
        se3<double>(so3<double>::identity(), Eigen::Vector3d(1, 0, 1)),
        std::move(axes), std::move(limits));

    auto solver = planar_2r_solver<kinematic_chain<double, dynamic>>::make(chain);

    REQUIRE_FALSE(solver.has_value());
    CHECK(solver.error().reason == analytical_failure::degenerate_geometry);
    CHECK_FALSE(solver.error().workspace_distance.has_value());
}

// The second link length the derivation names is an in-plane length, so a home
// end-effector off the plane is not a chain it can answer for. Until the factory
// tested for it the chain was admitted, the second link was measured as a
// three-dimensional distance, and the failure surfaced per pose as a failed
// back-check.
//
// Only the end-effector needs the test. A screw axis carries a line rather than
// a point on it, and the point recovered as omega x v is the foot of the
// perpendicular from the origin, so both recovered joint points are
// perpendicular to a shared axis direction: once the axes are parallel the first
// link is in the plane identically, and no chain can be built that is not.
TEST_CASE("2R solver: a home end-effector off the mechanism plane is refused at "
          "construction")
{
    constexpr double out_of_plane = 0.25;
    auto no_limits = testing::limits(-10.0, 10.0);
    auto chain = testing::unwrap(
        planar_2r_chain::make(
            se3<double>(
                so3<double>::identity(), Eigen::Vector3d(2.0, out_of_plane, 0)),
            {screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0}),
             screw_axis<double>::revolute({0, 1, 0}, {1.0, 0, 0})},
            {no_limits, no_limits}),
        "off-plane 2R chain");

    auto solver = planar_2r_solver<planar_2r_chain>::make(chain);

    REQUIRE_FALSE(solver.has_value());
    CHECK(solver.error().reason == analytical_failure::degenerate_geometry);
    CHECK_FALSE(solver.error().workspace_distance.has_value());

    // The premise: displacing the second joint along the shared axis direction
    // instead would leave the recovered joint point, and so the chain, unchanged.
    auto displaced = screw_axis<double>::revolute({0, 1, 0}, {1.0, out_of_plane, 0});
    CHECK((displaced.omega().cross(displaced.v()) - Eigen::Vector3d(1, 0, 0)).norm()
        == 0.0);
}
