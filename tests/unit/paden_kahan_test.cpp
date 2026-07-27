#include "cartan/analytical/paden_kahan.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <limits>
#include <numbers>
#include <type_traits>

using namespace cartan;
using Catch::Matchers::WithinAbs;

static constexpr double tolerance = 1e-10;

// The separation of the two threshold species is only worth its cost if the
// compiler enforces it, so pin it here rather than trusting a naming convention.
static_assert(!std::is_convertible_v<length_tolerance<double>, direction_tolerance<double>>);
static_assert(!std::is_convertible_v<direction_tolerance<double>, length_tolerance<double>>);
static_assert(!std::is_constructible_v<direction_tolerance<double>, length_tolerance<double>>);
static_assert(!std::is_constructible_v<length_tolerance<double>, direction_tolerance<double>>);

// A braced scalar walks past a type distinction that rests only on the member
// type, which is why both thresholds are explicit-only rather than aggregates.
static_assert(!std::is_convertible_v<double, length_tolerance<double>>);
static_assert(!std::is_convertible_v<double, direction_tolerance<double>>);

TEST_CASE("paden_kahan_1: 90-degree rotation about z through origin")
{
    vector3<double> omega{0, 0, 1};
    vector3<double> q{0, 0, 0};
    vector3<double> p{1, 0, 0};
    vector3<double> p_prime{0, 1, 0};

    auto result = paden_kahan_1(omega, q, p, p_prime);
    REQUIRE(result.has_value());
    CHECK_THAT(*result, WithinAbs(std::numbers::pi / 2, tolerance));
}

TEST_CASE("paden_kahan_1: rotation about z through offset point")
{
    vector3<double> omega{0, 0, 1};
    vector3<double> q{1, 0, 0};
    vector3<double> p{2, 0, 0};
    // Rotating (2,0,0) about z-axis through (1,0,0) by pi/2:
    // relative = (1,0,0), rotated = (0,1,0), absolute = (1,1,0)
    vector3<double> p_prime{1, 1, 0};

    auto result = paden_kahan_1(omega, q, p, p_prime);
    REQUIRE(result.has_value());
    CHECK_THAT(*result, WithinAbs(std::numbers::pi / 2, tolerance));
}

TEST_CASE("paden_kahan_1: unreachable (different distances from axis)")
{
    vector3<double> omega{0, 0, 1};
    vector3<double> q{0, 0, 0};
    vector3<double> p{1, 0, 0};
    vector3<double> p_prime{0, 2, 0};

    auto result = paden_kahan_1(omega, q, p, p_prime);
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error() == analytical_failure::unreachable);
}

TEST_CASE("paden_kahan_1: negative angle")
{
    vector3<double> omega{0, 0, 1};
    vector3<double> q{0, 0, 0};
    vector3<double> p{1, 0, 0};
    vector3<double> p_prime{0, -1, 0};

    auto result = paden_kahan_1(omega, q, p, p_prime);
    REQUIRE(result.has_value());
    CHECK_THAT(*result, WithinAbs(-std::numbers::pi / 2, tolerance));
}

TEST_CASE("paden_kahan_1: coincident points on the axis are a singular instance")
{
    // Two coincident points on the axis satisfy the equation for every angle.
    // The reported reason used to be degenerate_geometry, which names the
    // chain's geometry when the geometry is fine and the instance is what has a
    // continuum of answers.
    vector3<double> omega{0, 0, 1};
    vector3<double> q{0, 0, 0};
    vector3<double> p{0, 0, 2};
    vector3<double> p_prime{0, 0, 2};

    auto result = paden_kahan_1(omega, q, p, p_prime);
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error() == analytical_failure::singular_configuration);
}

TEST_CASE("paden_kahan_1: an axial displacement admits no rotation")
{
    // No rotation about z can change a z component, yet the axial components
    // were computed and discarded, so this instance used to return pi/2 with a
    // reconstruction residual of a full unit length.
    vector3<double> omega{0, 0, 1};
    vector3<double> q{0, 0, 0};
    vector3<double> p{1, 0, 0};
    vector3<double> p_prime{0, 1, 1};

    auto result = paden_kahan_1(omega, q, p, p_prime);
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error() == analytical_failure::unreachable);
}

TEST_CASE("paden_kahan_1: two axis points at different heights are unreachable")
{
    // Both points lie on the axis, so no rotation moves one onto the other.
    // The on-axis branch used to fire first and report degenerate_geometry,
    // naming the geometry rather than the absent solution.
    vector3<double> omega{0, 0, 1};
    vector3<double> q{0, 0, 0};
    vector3<double> p{0, 0, 2};
    vector3<double> p_prime{0, 0, 3};

    auto result = paden_kahan_1(omega, q, p, p_prime);
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error() == analytical_failure::unreachable);
}

TEST_CASE("paden_kahan_1: nonfinite input and a non-unit axis reach the error channel")
{
    // A NaN point used to travel through every comparison and leave as a NaN
    // angle on the success channel; a doubled axis used to be silently treated
    // as a unit one.
    vector3<double> omega{0, 0, 1};
    vector3<double> q{0, 0, 0};
    vector3<double> p{1, 0, 0};
    vector3<double> p_prime{0, 1, 0};

    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    auto nan_point = paden_kahan_1(omega, q, vector3<double>{nan, 0, 0}, p_prime);
    REQUIRE_FALSE(nan_point.has_value());
    CHECK(nan_point.error() == analytical_failure::non_finite_input);

    auto inf_target = paden_kahan_1(omega, q, p, vector3<double>{0, inf, 0});
    REQUIRE_FALSE(inf_target.has_value());
    CHECK(inf_target.error() == analytical_failure::non_finite_input);

    auto nan_axis = paden_kahan_1(vector3<double>{0, 0, nan}, q, p, p_prime);
    REQUIRE_FALSE(nan_axis.has_value());
    CHECK(nan_axis.error() == analytical_failure::non_finite_input);

    auto scaled_axis = paden_kahan_1(vector3<double>{0, 0, 2}, q, p, p_prime);
    REQUIRE_FALSE(scaled_axis.has_value());
    CHECK(scaled_axis.error() == analytical_failure::degenerate_geometry);
}

TEST_CASE("paden_kahan_1: returned angles agree with a planar closed form")
{
    // The angle a z-rotation needs to carry (r, 0, h) to (r cos a, r sin a, h)
    // is a, written out here rather than taken from the rotation helper the
    // implementation itself uses.
    const double r = 1.7;
    const double h = -0.4;
    vector3<double> omega{0, 0, 1};
    vector3<double> q{0, 0, h};
    vector3<double> p{r, 0, 0};

    // Counts angles that were actually recovered, not loop trips: a CHECK does
    // not abort, so a solve that fails or returns a wrong angle leaves the
    // count short and the assertion below fires.
    int recovered = 0;
    for (int i = -17; i <= 17; ++i)
    {
        const double angle = i * (std::numbers::pi / 18.0);
        vector3<double> p_prime{r * std::cos(angle), r * std::sin(angle), 0};

        auto result = paden_kahan_1(omega, q, p, p_prime);
        CHECK(result.has_value());
        if (!result)
            continue;
        CHECK_THAT(*result, WithinAbs(angle, 1e-12));
        if (std::abs(*result - angle) < 1e-12)
            ++recovered;
    }
    REQUIRE(recovered == 35);
}

TEST_CASE("paden_kahan_1: two tolerable errors that compound are refused")
{
    // Pins the reconstruction residual, and nothing else does: the axial and
    // radius conditions each pass here, and only the residual on the original
    // equation sees that together they exceed the threshold. Disabling the
    // reconstruction check makes this case return pi/2.
    const double offset = 0.9e-6;
    vector3<double> omega{0, 0, 1};
    vector3<double> q{0, 0, 0};
    vector3<double> p{1, 0, 0};
    vector3<double> p_prime{0, 1 + offset, offset};

    // Both scalar conditions pass, stated independently of the implementation.
    CHECK(std::abs(omega.dot(p_prime - p)) < 1e-6);
    CHECK(std::abs(p.norm() - std::hypot(1 + offset, 0.0)) < 1e-6);

    // A z-rotation by pi/2 carries (1,0,0) to (0,1,0), which is the closest any
    // rotation about z gets to p_prime; the closed form is written out here
    // rather than taken from the rotation helper the implementation uses.
    vector3<double> best{0, 1, 0};
    CHECK((best - p_prime).norm() > 1e-6);

    auto result = paden_kahan_1(omega, q, p, p_prime);
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error() == analytical_failure::unreachable);
}

TEST_CASE("paden_kahan_1: the length threshold is relative to the working radius")
{
    // An absolute threshold tightens as 1/radius, so exactly consistent
    // instances on a large mechanism get rejected as unreachable. Pins the
    // scaling: with an absolute threshold the float rows below fail.
    const double angle = 0.7;
    int accepted = 0;
    for (double radius : {1.0, 10.0, 100.0, 1000.0})
    {
        vector3<double> omega{0, 0, 1};
        vector3<double> q{0, 0, 0};
        vector3<double> p{radius, 0, 0};
        vector3<double> p_prime{radius * std::cos(angle), radius * std::sin(angle), 0};

        auto wide = paden_kahan_1(omega, q, p, p_prime);
        CHECK(wide.has_value());
        if (wide)
            ++accepted;

        vector3<float> omega_f{0, 0, 1};
        vector3<float> q_f{0, 0, 0};
        vector3<float> p_f{static_cast<float>(radius), 0, 0};
        vector3<float> p_prime_f{
            static_cast<float>(radius * std::cos(angle)),
            static_cast<float>(radius * std::sin(angle)), 0};

        auto narrow = paden_kahan_1(omega_f, q_f, p_f, p_prime_f);
        CHECK(narrow.has_value());
        if (narrow)
            ++accepted;
    }
    REQUIRE(accepted == 8);
}

TEST_CASE("paden_kahan_1_direction: non-unit and nonfinite arguments are refused")
{
    // The dimensionless threshold means nothing against a residual of some
    // other scale, so a position pair must not be judged by it. Before this,
    // p=(1000,0,0) to (0,1000,0) returned pi/2 with 1e-6 judging a residual of
    // scale 1000.
    vector3<double> omega{0, 0, 1};

    auto positions = paden_kahan_1_direction(
        omega, vector3<double>{1000, 0, 0}, vector3<double>{0, 1000, 0});
    REQUIRE_FALSE(positions.has_value());
    CHECK(positions.error() == analytical_failure::degenerate_geometry);

    auto shrunk = paden_kahan_1_direction(
        omega, vector3<double>{0.5, 0, 0}, vector3<double>{0, 0.5, 0});
    REQUIRE_FALSE(shrunk.has_value());
    CHECK(shrunk.error() == analytical_failure::degenerate_geometry);

    const double nan = std::numeric_limits<double>::quiet_NaN();
    auto nonfinite = paden_kahan_1_direction(
        omega, vector3<double>{nan, 0, 0}, vector3<double>{0, 1, 0});
    REQUIRE_FALSE(nonfinite.has_value());
    CHECK(nonfinite.error() == analytical_failure::non_finite_input);

    // The unit pair the wrist decomposition actually passes still solves.
    auto unit = paden_kahan_1_direction(
        omega, vector3<double>{1, 0, 0}, vector3<double>{0, 1, 0});
    REQUIRE(unit.has_value());
    CHECK_THAT(*unit, WithinAbs(std::numbers::pi / 2, tolerance));
}

TEST_CASE("paden_kahan_2: two rotations mapping a known point")
{
    vector3<double> omega1{0, 0, 1};
    vector3<double> omega2{0, 1, 0};
    vector3<double> q{0, 0, 0};

    // Apply theta2 = pi/4 about y, then theta1 = pi/3 about z to point (1,0,0)
    double theta2_expected = std::numbers::pi / 4;
    double theta1_expected = std::numbers::pi / 3;

    // After rotating (1,0,0) about y by pi/4: (cos(pi/4), 0, -sin(pi/4))
    double c4 = std::cos(theta2_expected);
    double s4 = std::sin(theta2_expected);
    vector3<double> intermediate{c4, 0, -s4};

    // After rotating intermediate about z by pi/3: (c4*cos(pi/3), c4*sin(pi/3), -s4)
    double c3 = std::cos(theta1_expected);
    double s3 = std::sin(theta1_expected);
    vector3<double> p_prime{c4 * c3, c4 * s3, -s4};

    vector3<double> p{1, 0, 0};

    auto result = paden_kahan_2(omega1, omega2, q, p, p_prime);
    REQUIRE(result.has_value());
    REQUIRE(result->count >= 1);

    // Check that at least one solution matches the expected angles
    bool found = false;
    for (std::size_t i = 0; i < static_cast<std::size_t>(result->count); ++i)
    {
        auto [t1, t2] = result->solutions[i];
        if (std::abs(t1 - theta1_expected) < 1e-8 && std::abs(t2 - theta2_expected) < 1e-8)
        {
            found = true;
            break;
        }
    }
    CHECK(found);
}

TEST_CASE("paden_kahan_2: degenerate case (parallel axes)")
{
    vector3<double> omega1{0, 0, 1};
    vector3<double> omega2{0, 0, 1};
    vector3<double> q{0, 0, 0};
    vector3<double> p{1, 0, 0};
    vector3<double> p_prime{0, 1, 0};

    auto result = paden_kahan_2(omega1, omega2, q, p, p_prime);
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error() == analytical_failure::degenerate_geometry);
}

TEST_CASE("paden_kahan_2: a displaced target is refused rather than answered")
{
    // Guards the composed pair: whichever condition catches it, no pair that
    // fails to reconstruct the displaced target may leave on the success
    // channel. Reconstruction here is an explicit pair of axis-angle rotations,
    // not the helper the implementation calls.
    vector3<double> omega1{0, 0, 1};
    vector3<double> omega2{0, 1, 0};
    vector3<double> q{0, 0, 0};

    const double theta1 = std::numbers::pi / 3;
    const double theta2 = std::numbers::pi / 4;
    const double c2 = std::cos(theta2);
    const double s2 = std::sin(theta2);
    vector3<double> p{1, 0, 0};
    vector3<double> consistent{
        c2 * std::cos(theta1), c2 * std::sin(theta1), -s2};
    vector3<double> displaced = consistent + vector3<double>{1e-3, 0, 0};

    auto result = paden_kahan_2(omega1, omega2, q, p, displaced);
    if (result.has_value())
    {
        for (int i = 0; i < result->count; ++i)
        {
            auto [t1, t2] = result->solutions[static_cast<std::size_t>(i)];
            const double a = std::cos(t2);
            const double b = std::sin(t2);
            vector3<double> mid{a, 0, -b};
            vector3<double> got{
                mid.x() * std::cos(t1) - mid.y() * std::sin(t1),
                mid.x() * std::sin(t1) + mid.y() * std::cos(t1),
                mid.z()};
            CHECK((got - displaced).norm() < 1e-6);
        }
    }
    else
    {
        CHECK(result.error() == analytical_failure::unreachable);
    }
}

TEST_CASE("paden_kahan_3: nonfinite input and a non-unit axis reach the error channel")
{
    // Every comparison in the distance constraint is false against a NaN, so
    // each of these used to skip all four guards and leave through the
    // two-solution branch reporting count = 2 NaN angles as a success.
    vector3<double> omega{0, 0, 1};
    vector3<double> q{0, 0, 0};
    vector3<double> p{1, 0, 0};
    vector3<double> p_prime{2, 0, 0};
    const double delta = std::sqrt(2.0);

    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    auto nan_axis = paden_kahan_3(vector3<double>{0, 0, nan}, q, p, p_prime, delta);
    REQUIRE_FALSE(nan_axis.has_value());
    CHECK(nan_axis.error() == analytical_failure::non_finite_input);

    auto nan_point = paden_kahan_3(omega, q, vector3<double>{nan, 0, 0}, p_prime, delta);
    REQUIRE_FALSE(nan_point.has_value());
    CHECK(nan_point.error() == analytical_failure::non_finite_input);

    auto inf_target = paden_kahan_3(omega, q, p, vector3<double>{inf, 0, 0}, delta);
    REQUIRE_FALSE(inf_target.has_value());
    CHECK(inf_target.error() == analytical_failure::non_finite_input);

    auto nan_delta = paden_kahan_3(omega, q, p, p_prime, nan);
    REQUIRE_FALSE(nan_delta.has_value());
    CHECK(nan_delta.error() == analytical_failure::non_finite_input);

    auto scaled_axis = paden_kahan_3(vector3<double>{0, 0, 2}, q, p, p_prime, delta);
    REQUIRE_FALSE(scaled_axis.has_value());
    CHECK(scaled_axis.error() == analytical_failure::degenerate_geometry);
}

TEST_CASE("paden_kahan_2: nonfinite input reaches the error channel")
{
    // The two-rotation subproblem shares subproblem 1's input gate; pinned here
    // so the three subproblems are covered by the same standard.
    vector3<double> omega1{0, 0, 1};
    vector3<double> omega2{0, 1, 0};
    vector3<double> q{0, 0, 0};
    vector3<double> p_prime{0, 1, 0};

    const double nan = std::numeric_limits<double>::quiet_NaN();

    auto nan_point = paden_kahan_2(
        omega1, omega2, q, vector3<double>{nan, 0, 0}, p_prime);
    REQUIRE_FALSE(nan_point.has_value());
    CHECK(nan_point.error() == analytical_failure::non_finite_input);

    auto scaled_axis = paden_kahan_2(
        vector3<double>{0, 0, 2}, omega2, q, vector3<double>{1, 0, 0}, p_prime);
    REQUIRE_FALSE(scaled_axis.has_value());
    CHECK(scaled_axis.error() == analytical_failure::degenerate_geometry);
}

TEST_CASE("paden_kahan_3: an on-axis instance splits by achievable distance")
{
    // The rotation cannot move a point lying on its own axis, so the achieved
    // distance is constant in the angle. Both halves used to be reported as
    // degenerate_geometry, which names neither the continuum nor its absence.
    vector3<double> omega{0, 0, 1};
    vector3<double> q{0, 0, 0};
    vector3<double> p{0, 0, 1};
    vector3<double> p_prime{0, 0, 0};

    auto achievable = paden_kahan_3(omega, q, p, p_prime, 1.0);
    REQUIRE_FALSE(achievable.has_value());
    CHECK(achievable.error() == analytical_failure::singular_configuration);

    auto impossible = paden_kahan_3(omega, q, p, p_prime, 5.0);
    REQUIRE_FALSE(impossible.has_value());
    CHECK(impossible.error() == analytical_failure::unreachable);
}

TEST_CASE("paden_kahan_3: two solutions for distance constraint")
{
    vector3<double> omega{0, 0, 1};
    vector3<double> q{0, 0, 0};
    vector3<double> p{1, 0, 0};
    vector3<double> p_prime{0, 0, 0};

    // Distance from rotated p to p_prime = distance from a unit-circle point to origin
    // For delta = 1.0, ||exp(theta)*p - 0|| = 1 always (p stays on unit circle), so
    // all angles work. Use a different setup.

    // p at (2,0,0), p_prime at (0,0,0), rotate about z through origin
    // ||rotated_p - p_prime|| = ||rotated_p|| = 2 always. Bad example.

    // Better: p=(1,0,0), p_prime=(2,0,0), omega=(0,0,1), q=(0,0,0), delta=sqrt(2)
    // Rotated p = (cos(t), sin(t), 0), distance to (2,0,0) = sqrt((cos(t)-2)^2 + sin^2(t))
    // = sqrt(5 - 4*cos(t)). For delta=sqrt(2): 5-4cos(t) = 2, cos(t) = 3/4
    vector3<double> p2{1, 0, 0};
    vector3<double> p_prime2{2, 0, 0};
    double delta = std::sqrt(2.0);

    auto result = paden_kahan_3(omega, q, p2, p_prime2, delta);
    REQUIRE(result.has_value());
    CHECK(result->count == 2);

    // Verify both solutions geometrically
    for (std::size_t i = 0; i < static_cast<std::size_t>(result->count); ++i)
    {
        double theta = result->solutions[i];
        vector3<double> rotated{std::cos(theta), std::sin(theta), 0};
        double dist = (rotated - p_prime2).norm();
        CHECK_THAT(dist, WithinAbs(delta, 1e-10));
    }
}

TEST_CASE("paden_kahan_3: unreachable (distance too large)")
{
    vector3<double> omega{0, 0, 1};
    vector3<double> q{0, 0, 0};
    vector3<double> p{1, 0, 0};
    vector3<double> p_prime{2, 0, 0};

    // Max distance from (cos(t),sin(t),0) to (2,0,0) is 3 (at t=pi)
    // Min distance is 1 (at t=0). delta=5 is impossible.
    auto result = paden_kahan_3(omega, q, p, p_prime, 5.0);
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error() == analytical_failure::unreachable);
}

TEST_CASE("paden_kahan_3: tangent case (single solution)")
{
    vector3<double> omega{0, 0, 1};
    vector3<double> q{0, 0, 0};
    vector3<double> p{1, 0, 0};
    vector3<double> p_prime{2, 0, 0};

    // Max distance from (cos(t),sin(t),0) to (2,0,0) = 3 (at t=pi)
    // Min distance = 1 (at t=0)
    // For delta=3: 5-4cos(t)=9, cos(t)=-1, t=pi. Single solution.
    double delta = 3.0;

    auto result = paden_kahan_3(omega, q, p, p_prime, delta);
    REQUIRE(result.has_value());
    CHECK(result->count == 1);
    CHECK_THAT(result->solutions[0], WithinAbs(std::numbers::pi, 1e-10));
}
