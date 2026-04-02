#include "liepp/analytical/paden_kahan.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <numbers>

using namespace liepp;
using Catch::Matchers::WithinAbs;

static constexpr double tolerance = 1e-10;

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
