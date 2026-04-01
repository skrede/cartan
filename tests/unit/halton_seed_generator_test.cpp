#include <liepp/serial/ik/halton_seed_generator.h>

#include <liepp/lie/se3.h>
#include <liepp/lie/so3.h>
#include <liepp/serial/chain/screw_axis.h>
#include <liepp/serial/chain/joint_limits.h>
#include <liepp/serial/chain/kinematic_chain.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <numbers>

namespace spp = liepp;

// ============================================================================
// Helpers
// ============================================================================

static spp::kinematic_chain<double, 6> make_ur5_like_chain()
{
    auto s1 = spp::screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s2 = spp::screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0.089});
    auto s3 = spp::screw_axis<double>::revolute({0, 1, 0}, {0.425, 0, 0.089});
    auto s4 = spp::screw_axis<double>::revolute({0, 1, 0}, {0.817, 0, 0.089});
    auto s5 = spp::screw_axis<double>::revolute({0, 0, -1}, {0.817, 0.109, 0});
    auto s6 = spp::screw_axis<double>::revolute({0, 1, 0}, {0.817, 0, -0.006});

    spp::vector3<double> home_trans;
    home_trans << 0.817, 0.191, -0.006;
    auto home = spp::se3<double>(spp::so3<double>::identity(), home_trans);

    spp::joint_limits<double> lim{-2 * std::numbers::pi, 2 * std::numbers::pi};
    return spp::kinematic_chain<double, 6>(home, {s1, s2, s3, s4, s5, s6},
                                  {lim, lim, lim, lim, lim, lim});
}

// ============================================================================
// halton_element tests
// ============================================================================

TEST_CASE("halton_element base 2 values", "[halton][element]")
{
    using Catch::Matchers::WithinAbs;

    CHECK_THAT(spp::halton_element<double>(1, 2), WithinAbs(0.5, 1e-12));
    CHECK_THAT(spp::halton_element<double>(2, 2), WithinAbs(0.25, 1e-12));
    CHECK_THAT(spp::halton_element<double>(3, 2), WithinAbs(0.75, 1e-12));
    CHECK_THAT(spp::halton_element<double>(4, 2), WithinAbs(0.125, 1e-12));
}

TEST_CASE("halton_element base 3 values", "[halton][element]")
{
    using Catch::Matchers::WithinAbs;

    CHECK_THAT(spp::halton_element<double>(1, 3), WithinAbs(1.0 / 3.0, 1e-12));
    CHECK_THAT(spp::halton_element<double>(2, 3), WithinAbs(2.0 / 3.0, 1e-12));
    CHECK_THAT(spp::halton_element<double>(3, 3), WithinAbs(1.0 / 9.0, 1e-12));
}

// ============================================================================
// halton_seed_generator tests
// ============================================================================

TEST_CASE("halton_seed_generator produces seeds within limits", "[halton][generator]")
{
    auto chain = make_ur5_like_chain();
    spp::halton_seed_generator<spp::kinematic_chain<double, 6>> gen(chain);

    for (int i = 0; i < 100; ++i)
    {
        auto seed = gen(i);
        REQUIRE(seed.size() == 6);
        for (int j = 0; j < 6; ++j)
        {
            auto lim = chain.limits()[static_cast<std::size_t>(j)];
            CHECK(seed[j] >= lim.position_min);
            CHECK(seed[j] <= lim.position_max);
        }
    }
}

TEST_CASE("halton_seed_generator is deterministic", "[halton][generator]")
{
    auto chain = make_ur5_like_chain();
    spp::halton_seed_generator<spp::kinematic_chain<double, 6>> gen(chain);

    auto seed_a = gen(42);
    auto seed_b = gen(42);

    for (int j = 0; j < 6; ++j)
    {
        CHECK(seed_a[j] == seed_b[j]);
    }
}

TEST_CASE("halton_seed_generator seeds have low discrepancy", "[halton][generator]")
{
    auto chain = make_ur5_like_chain();
    spp::halton_seed_generator<spp::kinematic_chain<double, 6>> gen(chain);

    // Generate seeds at indices 21-30 and check no two are within epsilon
    std::vector<Eigen::Vector<double, 6>> seeds;
    for (int i = 21; i <= 30; ++i)
    {
        seeds.push_back(gen(i));
    }

    for (std::size_t i = 0; i < seeds.size(); ++i)
    {
        for (std::size_t j = i + 1; j < seeds.size(); ++j)
        {
            double dist = (seeds[i] - seeds[j]).norm();
            CHECK(dist > 0.01);
        }
    }
}

// ============================================================================
// wrap_joint_angle tests
// ============================================================================

TEST_CASE("wrap_joint_angle in range is identity", "[halton][wrap]")
{
    double result = spp::wrap_joint_angle(0.5, 0.0, 1.0);
    CHECK(result == 0.5);
}

TEST_CASE("wrap_joint_angle below min wraps modulo 2pi", "[halton][wrap]")
{
    using Catch::Matchers::WithinAbs;

    constexpr double pi = std::numbers::pi;
    // q = -7.0, limits = [-pi, pi]
    // -7.0 is below -pi, wrapping modulo 2pi should bring it into range
    double result = spp::wrap_joint_angle(-7.0, -pi, pi);
    CHECK(result >= -pi);
    CHECK(result <= pi);

    // Expected: -7 + 2pi ~ -0.717
    CHECK_THAT(result, WithinAbs(-7.0 + 2.0 * pi, 1e-10));
}

TEST_CASE("wrap_joint_angle above max wraps modulo 2pi", "[halton][wrap]")
{
    using Catch::Matchers::WithinAbs;

    constexpr double pi = std::numbers::pi;
    // q = 7.0, limits = [-pi, pi]
    double result = spp::wrap_joint_angle(7.0, -pi, pi);
    CHECK(result >= -pi);
    CHECK(result <= pi);

    // Expected: 7 - 2pi ~ 0.717
    CHECK_THAT(result, WithinAbs(7.0 - 2.0 * pi, 1e-10));
}

TEST_CASE("wrap_joint_angle falls back to clamp for narrow range", "[halton][wrap]")
{
    // Range < 2pi: [0, 1] -- wrapping a value like 5.0 modulo 2pi won't land in [0,1]
    // so it should fall back to clamping to q_max
    double result = spp::wrap_joint_angle(5.0, 0.0, 1.0);
    CHECK(result == 1.0);

    // Wrapping below: -5.0 with narrow range should clamp to q_min
    result = spp::wrap_joint_angle(-5.0, 0.0, 1.0);
    CHECK(result == 0.0);
}
