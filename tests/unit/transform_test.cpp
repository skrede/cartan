#include <liepp/frames/transform.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <cmath>
#include <numbers>
#include <type_traits>

using Catch::Approx;

// Frame tags
struct world {};
struct base {};
struct tool {};

// ============================================================================
// Frame chain composition
// ============================================================================

TEMPLATE_TEST_CASE("transform: frame chain composition", "[transform]", double, float)
{
    using S = TestType;
    S margin = std::is_same_v<S, float> ? S(1e-4) : S(1e-10);

    liepp::vector6<S> v_ab;
    v_ab << S(0.1), S(-0.2), S(0.3), S(1.0), S(2.0), S(3.0);
    liepp::vector6<S> v_bc;
    v_bc << S(-0.1), S(0.3), S(0.1), S(-0.5), S(1.5), S(0.5);

    auto t_ab = liepp::transform<world, base, S>{liepp::se3<S>::exp(v_ab)};
    auto t_bc = liepp::transform<base, tool, S>{liepp::se3<S>::exp(v_bc)};

    auto t_ac = t_ab * t_bc;

    // Result type
    static_assert(std::is_same_v<
        decltype(t_ac),
        liepp::transform<world, tool, S, liepp::strict_policy>>);

    // Numerical check
    auto expected = (t_ab.matrix() * t_bc.matrix()).eval();
    REQUIRE((t_ac.matrix() - expected).norm() < margin);
}

// ============================================================================
// Inverse flips tags
// ============================================================================

TEST_CASE("transform: inverse flips frame tags", "[transform]")
{
    liepp::vector6<double> v;
    v << 0.3, -0.1, 0.5, 1.0, -2.0, 0.5;
    auto t_wb = liepp::transform<world, base>{liepp::se3<double>::exp(v)};

    auto t_bw = t_wb.inverse();

    static_assert(std::is_same_v<
        decltype(t_bw),
        liepp::transform<base, world, double, liepp::strict_policy>>);

    // inverse cancels
    auto I = liepp::matrix4<double>::Identity();
    REQUIRE(((t_wb * t_bw).matrix() - I).norm() < 1e-10);
}

// ============================================================================
// Mixed-policy compose
// ============================================================================

TEST_CASE("transform: mixed-policy compose uses stricter", "[transform][policy]")
{
    liepp::vector6<double> v_a;
    v_a << 0.1, -0.2, 0.3, 1.0, 0.0, 0.0;
    liepp::vector6<double> v_b;
    v_b << -0.1, 0.2, 0.1, 0.0, 1.0, 0.0;

    auto strict_t = liepp::transform<world, base, double, liepp::strict_policy>{
        liepp::se3<double, liepp::strict_policy>::exp(v_a)};
    auto fast_t = liepp::transform<base, tool, double, liepp::fast_policy>{
        liepp::se3<double, liepp::fast_policy>::exp(v_b)};

    auto result = strict_t * fast_t;

    static_assert(std::is_same_v<
        decltype(result),
        liepp::transform<world, tool, double, liepp::strict_policy>>);

    auto expected = (strict_t.matrix() * fast_t.matrix()).eval();
    REQUIRE((result.matrix() - expected).norm() < 1e-10);
}

// ============================================================================
// Identity
// ============================================================================

TEST_CASE("transform: identity", "[transform]")
{
    auto t = liepp::transform<world, base>::identity();
    auto I = liepp::matrix4<double>::Identity();
    REQUIRE((t.matrix() - I).norm() < 1e-14);
}

// ============================================================================
// Forwarded methods
// ============================================================================

TEST_CASE("transform: forwarded methods", "[transform]")
{
    liepp::vector6<double> v;
    v << 0.3, -0.1, 0.5, 1.0, -2.0, 0.5;
    auto inner = liepp::se3<double>::exp(v);
    auto t = liepp::transform<world, base>{inner};

    SECTION("matrix()")
    {
        REQUIRE((t.matrix() - inner.matrix()).norm() < 1e-14);
    }

    SECTION("rotation()")
    {
        REQUIRE((t.rotation().matrix() - inner.rotation().matrix()).norm() < 1e-14);
    }

    SECTION("translation()")
    {
        REQUIRE((t.translation() - inner.translation()).norm() < 1e-14);
    }

    SECTION("log()")
    {
        REQUIRE((t.log() - inner.log()).norm() < 1e-14);
    }

    SECTION("act()")
    {
        liepp::vector3<double> p;
        p << 1.0, 2.0, 3.0;
        REQUIRE((t.act(p) - inner.act(p)).norm() < 1e-14);
    }
}

// ============================================================================
// from_matrix roundtrip
// ============================================================================

TEST_CASE("transform: from_matrix roundtrip", "[transform]")
{
    liepp::vector6<double> v;
    v << 0.3, -0.1, 0.5, 1.0, -2.0, 0.5;
    auto t = liepp::transform<world, base>{liepp::se3<double>::exp(v)};
    auto result = liepp::transform<world, base>::from_matrix(t.matrix());
    REQUIRE(result.has_value());
    REQUIRE((result.value().matrix() - t.matrix()).norm() < 1e-10);
}

// ============================================================================
// act() on points
// ============================================================================

TEST_CASE("transform: act on point", "[transform]")
{
    // Pure translation
    auto rot = liepp::so3<double>::identity();
    liepp::vector3<double> trans;
    trans << 1.0, 2.0, 3.0;
    auto t = liepp::transform<world, base>{liepp::se3<double>(rot, trans)};

    liepp::vector3<double> p;
    p << 0.0, 0.0, 0.0;
    auto result = t.act(p);
    REQUIRE(result(0) == Approx(1.0));
    REQUIRE(result(1) == Approx(2.0));
    REQUIRE(result(2) == Approx(3.0));
}

// ============================================================================
// m_value accessibility
// ============================================================================

TEST_CASE("transform: m_value is accessible as se3", "[transform]")
{
    auto t = liepp::transform<world, base>{liepp::se3<double>::identity()};
    auto M = t.m_value.matrix();
    auto I = liepp::matrix4<double>::Identity();
    REQUIRE((M - I).norm() < 1e-14);
}
