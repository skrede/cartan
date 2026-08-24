#include "../support/kinematics_helpers.h"

#include "cartan/analytical.h"
#include "cartan/serial_chain.h"

#include "../fixtures/opw_chains.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

using namespace cartan;
using Catch::Matchers::WithinRel;

// What the OPW solver reports when it refuses a target: which reason, and what
// magnitude stands behind it. A deficit read off an inequality other than the
// one that failed is worse than no deficit at all, because it is in the right
// unit and a caller will move a target by it.

// The independent-reconstruction bar every emitted solution must clear.
static constexpr double reconstruction_bar = 1e-9;

TEST_CASE("OPW: the lateral-cylinder deficit is the radial excess")
{
    // Pre-fix the guard reported sqrt(b^2 - r^2), a chord half-length: 0.06245
    // against a radial excess of 0.01, and the ratio grows without bound as the
    // radius approaches the offset.
    auto chain = fixtures::make_offset_plane_opw_chain<double>();
    auto params = fixtures::offset_plane_opw_parameters<double>();
    auto solver = opw_6r_solver<decltype(chain)>::make(chain, params);
    REQUIRE(solver.has_value());

    // Wrist center inside the cylinder, placed high enough above the shoulder
    // that the clamped shoulder-wrist distance still lies within reach: the
    // cylinder is then the only inequality that failed.
    const double radius = 0.95 * params.b;
    const Eigen::Vector3d center(radius, 0.0, params.c1 + 0.3);
    auto result = solver->solve(se3<double>(so3<double>::identity(),
        center + Eigen::Vector3d(0.0, 0.0, params.c4)));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().reason == analytical_failure::unreachable);
    REQUIRE(result.error().workspace_distance.has_value());
    CHECK_THAT(*result.error().workspace_distance,
        WithinRel(params.b - radius, 1e-12));
}

TEST_CASE("OPW: an out-of-reach target reports the nearer shoulder family")
{
    // Pre-fix only the front family was reach-checked, so the report carried
    // 0.2 -- that family's shortfall -- where the back family is only 0.1 short
    // and is the one a caller moving the target would aim at.
    auto chain = fixtures::make_offset_plane_opw_chain<double>();
    auto params = fixtures::offset_plane_opw_parameters<double>();
    auto solver = opw_6r_solver<decltype(chain)>::make(chain, params);
    REQUIRE(solver.has_value());

    // Both families sit inside the inner reach bound. At the shoulder height the
    // vertical term drops out, leaving the front shoulder |planar - a1| from the
    // wrist center and the back one planar + a1; the back family is therefore
    // the nearer of the two whenever the bound is missed from inside.
    const double planar = 0.05;
    const double reach_min =
        std::abs(params.c2 - std::hypot(params.a2, params.c3));
    const Eigen::Vector3d center(std::hypot(planar, params.b), 0.0, params.c1);
    auto result = solver->solve(se3<double>(so3<double>::identity(),
        center + Eigen::Vector3d(0.0, 0.0, params.c4)));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().reason == analytical_failure::unreachable);
    REQUIRE(result.error().workspace_distance.has_value());
    CHECK_THAT(*result.error().workspace_distance,
        WithinRel(reach_min - (planar + params.a1), 1e-12));
}

TEST_CASE("OPW: the cylinder and the reach deficits merge as a conjunction")
{
    // Both constraints must hold, so the report carries the larger deficit.
    // Pre-fix the cylinder deficit suppressed the reach one outright: this
    // target is 2.10 outside the workspace and the report was 0.01, and the
    // ratio is unbounded in the depth below the shoulder. An under-report in
    // the right unit is worse than the over-report it replaced: a caller that
    // overshoots converges, one that undershoots retries forever.
    auto chain = fixtures::make_offset_plane_opw_chain<double>();
    auto params = fixtures::offset_plane_opw_parameters<double>();
    auto solver = opw_6r_solver<decltype(chain)>::make(chain, params);
    REQUIRE(solver.has_value());

    // Inside the cylinder and far below the shoulder. Moving the wrist center
    // radially out onto the cylinder leaves it a1 horizontally and `drop` below
    // the shoulder, so the reach shortfall from there is what remains after the
    // radial motion, not a length standing in place of it.
    const double drop = 3.0;
    const double radius = 0.95 * params.b;
    const Eigen::Vector3d center(radius, 0.0, params.c1 - drop);
    auto result = solver->solve(se3<double>(so3<double>::identity(),
        center + Eigen::Vector3d(0.0, 0.0, params.c4)));

    const double remaining = std::hypot(params.a1, drop)
        - (params.c2 + std::hypot(params.a2, params.c3));
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().reason == analytical_failure::unreachable);
    REQUIRE(result.error().workspace_distance.has_value());
    CHECK(*result.error().workspace_distance > params.b - radius);
    CHECK_THAT(*result.error().workspace_distance, WithinRel(remaining, 1e-12));
}

TEST_CASE("OPW: the cylinder guard fires exactly where the radius is clamped")
{
    // The guard used to leave a band of one acceptance length below the offset
    // in which the radius was clamped but no deficit recorded, so a target
    // genuinely inside the cylinder was called a singular configuration and
    // carried nothing. That band is the caller's own position field, so it is
    // as wide as the caller makes it -- here 0.05 against an offset of 0.2.
    auto chain = fixtures::make_offset_plane_opw_chain<double>();
    auto params = fixtures::offset_plane_opw_parameters<double>();
    // A zero orientation field so no branch back-checks and the error channel
    // is reached whatever the widened position field would otherwise admit.
    auto solver = opw_6r_solver<decltype(chain)>::make(
        chain, params, verification_tolerance<double>(0.05, 0.0));
    REQUIRE(solver.has_value());

    const double radius = 0.17;
    // The premise: inside the cylinder, but by less than the position field.
    REQUIRE(radius < params.b);
    REQUIRE(params.b - radius < 0.05);

    const Eigen::Vector3d center(radius, 0.0, params.c1 + 0.3);
    auto result = solver->solve(se3<double>(so3<double>::identity(),
        center + Eigen::Vector3d(0.0, 0.0, params.c4)));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().reason == analytical_failure::unreachable);
    REQUIRE(result.error().workspace_distance.has_value());
    CHECK_THAT(*result.error().workspace_distance,
        WithinRel(params.b - radius, 1e-12));
}

TEST_CASE("OPW: a shoulder family with a vanishing denominator emits nothing")
{
    // Wrist center on the front shoulder point, so that family's law-of-cosines
    // denominator 2*s1*c2 vanishes. Pre-fix the arc-cosine of zero stood in for
    // the undefined angle and four branches carrying a quarter-turn shoulder
    // angle reached the unverified channel; only the back family is defined.
    auto chain = fixtures::make_kr6_r900_opw_chain<double>();
    auto params = fixtures::kr6_r900_opw_parameters<double>();
    const Eigen::Vector3d center(params.a1, 0.0, params.c1);
    auto target = se3<double>(so3<double>::identity(),
        center + Eigen::Vector3d(0.0, 0.0, params.c4));

    auto raw = opw_6r_solver<decltype(chain), opw_raw>::make(chain, params);
    REQUIRE(raw.has_value());
    auto emitted = raw->solve(target);
    REQUIRE(emitted.has_value());
    CHECK(emitted->count == 4);

    // Nothing the unverified channel emits is a fabrication: every branch it
    // returns reconstructs the target, and the verifying solver keeps them all.
    for (int i = 0; i < emitted->count; ++i)
    {
        auto fk = testing::fk_at(
            chain, emitted->solutions[static_cast<std::size_t>(i)]);
        CHECK((fk.end_effector.translation() - target.translation()).norm()
            < reconstruction_bar);
        CHECK((fk.end_effector.rotation().inverse()
            * target.rotation()).log().norm() < reconstruction_bar);
    }

    auto verified = opw_6r_solver<decltype(chain)>::make(chain, params);
    REQUIRE(verified.has_value());
    auto kept = verified->solve(target);
    REQUIRE(kept.has_value());
    CHECK(kept->count == emitted->count);
}
