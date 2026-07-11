#include "cartan/analytical/detail/angle_unwrap.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <numbers>

using cartan::detail::default_feasibility_tol;
using cartan::detail::unwrap_to_range_nearest;

namespace
{
constexpr double pi     = std::numbers::pi_v<double>;
constexpr double two_pi = 2.0 * pi;
constexpr double close_tol = 1e-12;

const double tol = default_feasibility_tol<double>();
}

TEST_CASE("symmetric range yields a reference-independent representative")
{
    const double at_zero = unwrap_to_range_nearest(0.5, -pi, pi, 0.0, tol);
    const double at_ten = unwrap_to_range_nearest(0.5, -pi, pi, 10.0, tol);
    REQUIRE(std::abs(at_zero - 0.5) < close_tol);
    REQUIRE(std::abs(at_ten - 0.5) < close_tol);
}

TEST_CASE("asymmetric range hits the arc or falls outside it")
{
    const double hit = unwrap_to_range_nearest(1.0, -0.5, 2.5, 0.0, tol);
    REQUIRE(std::abs(hit - 1.0) < close_tol);
    REQUIRE(hit >= -0.5);
    REQUIRE(hit <= 2.5);

    const double miss = unwrap_to_range_nearest(4.0, -0.5, 2.5, 0.0, tol);
    REQUIRE((miss < -0.5 || miss > 2.5));
}

TEST_CASE("multi-turn range tracks the reference across a full turn")
{
    const double lo = -3.0 * pi;
    const double hi = 3.0 * pi;
    const double at_zero = unwrap_to_range_nearest(0.5, lo, hi, 0.0, tol);
    const double at_turn = unwrap_to_range_nearest(0.5, lo, hi, two_pi, tol);
    REQUIRE(std::abs(at_zero - 0.5) < close_tol);
    REQUIRE(std::abs((at_turn - at_zero) - two_pi) < close_tol);
}

TEST_CASE("tight range with no reachable representative falls outside")
{
    const double result = unwrap_to_range_nearest(0.5, 0.1, 0.2, 0.0, tol);
    REQUIRE((result < 0.1 || result > 0.2));
}

TEST_CASE("fully continuous joint is the identity")
{
    const double inf = std::numeric_limits<double>::infinity();
    const double result = unwrap_to_range_nearest(42.0, -inf, inf, 0.0, tol);
    REQUIRE(std::abs(result - 42.0) < close_tol);
}
