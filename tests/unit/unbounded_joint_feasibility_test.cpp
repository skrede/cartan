#include "../support/joint_limits_helpers.h"

#include <cartan/serial/ik/detail/limit_enforcement.h>
#include <cartan/serial/ik/solver/detail/halton_seed_generator.h>

#include <cartan/analytical/detail/angle_unwrap.h>

#include <cartan/lie/se3.h>
#include <cartan/lie/so3.h>
#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/kinematic_chain.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <limits>
#include <numbers>

using Catch::Matchers::WithinAbs;

namespace
{

constexpr double pi = std::numbers::pi;
constexpr double inf = std::numeric_limits<double>::infinity();

using chain6 = cartan::kinematic_chain<double, 6>;
using position6 = Eigen::Vector<double, 6>;

/// A 6R chain whose first joint is feasible from ten radians upward and has no
/// upper bound. Every other joint carries ordinary finite bounds, so a failure
/// here is attributable to the one-sided joint alone.
chain6 make_one_sided_chain()
{
    auto s1 = cartan::screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s2 = cartan::screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0.089});
    auto s3 = cartan::screw_axis<double>::revolute({0, 1, 0}, {0.425, 0, 0.089});
    auto s4 = cartan::screw_axis<double>::revolute({0, 1, 0}, {0.817, 0, 0.089});
    auto s5 = cartan::screw_axis<double>::revolute({0, 0, -1}, {0.817, 0.109, 0});
    auto s6 = cartan::screw_axis<double>::revolute({0, 1, 0}, {0.817, 0, -0.006});

    cartan::vector3<double> home_trans;
    home_trans << 0.817, 0.191, -0.006;
    auto home = cartan::se3<double>(cartan::so3<double>::identity(), home_trans);

    auto open = cartan::testing::limits(10.0, inf);
    auto lim = cartan::testing::limits(-2 * pi, 2 * pi);
    return chain6(home, {s1, s2, s3, s4, s5, s6}, {open, lim, lim, lim, lim, lim});
}

}

TEST_CASE("every multistart seed satisfies a one-sided joint's real limits", "[unbounded]")
{
    auto chain = make_one_sided_chain();
    position6 reference = position6::Zero();
    reference[0] = 10.5;
    cartan::halton_seed_generator<chain6> gen(chain, reference);

    // Centered on +/-half the fallback width about the origin instead, the first
    // three seeds of joint 0 are 1.9635, -1.1781 and 5.10509 -- every one of them
    // below the joint's lower bound of ten.
    const double tol = cartan::detail::default_feasibility_tol<double>();
    for (int i = 0; i < 64; ++i)
    {
        auto seed = gen(i);
        INFO("seed index " << i << ", joint 0 at " << seed[0]);
        CHECK(cartan::detail::within_limits<chain6>(seed, chain, tol));
    }
}

TEST_CASE("a joint unbounded on both sides unwraps to the nearest equivalent", "[unbounded]")
{
    const double tol = cartan::detail::default_feasibility_tol<double>();
    const double result =
        cartan::detail::unwrap_to_range_nearest(-3.10, -inf, inf, 3.10, tol);
    CHECK_THAT(result, WithinAbs(-3.10 + 2.0 * pi, 1e-12));
    CHECK(std::abs(result - 3.10) < std::abs(-3.10 - 3.10));
}

TEST_CASE("one finite bound leaves the unwrap unchanged", "[unbounded]")
{
    const double tol = cartan::detail::default_feasibility_tol<double>();
    CHECK_THAT(cartan::detail::unwrap_to_range_nearest(-3.10, 0.0, inf, 3.10, tol),
        WithinAbs(-3.10 + 2.0 * pi, 1e-12));
    CHECK_THAT(cartan::detail::unwrap_to_range_nearest(-3.10, -inf, 0.0, 3.10, tol),
        WithinAbs(-3.10, 1e-12));
}

TEST_CASE("a non-finite unwrap reference returns the angle unchanged", "[unbounded]")
{
    const double tol = cartan::detail::default_feasibility_tol<double>();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THAT(cartan::detail::unwrap_to_range_nearest(-3.10, -inf, inf, nan, tol),
        WithinAbs(-3.10, 1e-12));
    CHECK_THAT(cartan::detail::unwrap_to_range_nearest(-3.10, -inf, inf, inf, tol),
        WithinAbs(-3.10, 1e-12));
}
