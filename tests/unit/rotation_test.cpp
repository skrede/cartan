#include <liepp/frames/rotation.h>

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
enum class frame_enum { world, base, tool };

// ============================================================================
// Frame chain composition
// ============================================================================

TEMPLATE_TEST_CASE("rotation: frame chain composition", "[rotation]", double, float)
{
    using S = TestType;
    S margin = std::is_same_v<S, float> ? S(1e-5) : S(1e-12);

    liepp::vector3<S> phi_ab;
    phi_ab << S(0.3), S(-0.2), S(0.5);
    liepp::vector3<S> phi_bc;
    phi_bc << S(-0.1), S(0.4), S(0.2);

    auto r_ab = liepp::rotation<world, base, S>{liepp::so3<S>::exp(phi_ab)};
    auto r_bc = liepp::rotation<base, tool, S>{liepp::so3<S>::exp(phi_bc)};

    auto r_ac = r_ab * r_bc;

    // Result type should be rotation<world, tool>
    static_assert(std::is_same_v<
        decltype(r_ac),
        liepp::rotation<world, tool, S, liepp::strict_policy>>);

    // Numerical check: matrix product
    auto expected = (r_ab.matrix() * r_bc.matrix()).eval();
    REQUIRE((r_ac.matrix() - expected).norm() < margin);
}

// ============================================================================
// Inverse flips tags
// ============================================================================

TEST_CASE("rotation: inverse flips frame tags", "[rotation]")
{
    liepp::vector3<double> phi;
    phi << 0.5, -0.3, 0.8;
    auto r_wb = liepp::rotation<world, base>{liepp::so3<double>::exp(phi)};

    auto r_bw = r_wb.inverse();

    static_assert(std::is_same_v<
        decltype(r_bw),
        liepp::rotation<base, world, double, liepp::strict_policy>>);

    // inverse cancels
    auto I = liepp::matrix3<double>::Identity();
    REQUIRE(((r_wb * r_bw).matrix() - I).norm() < 1e-12);
}

// ============================================================================
// Mixed-policy compose
// ============================================================================

TEST_CASE("rotation: mixed-policy compose uses stricter", "[rotation][policy]")
{
    liepp::vector3<double> phi_a;
    phi_a << 0.3, 0.1, -0.2;
    liepp::vector3<double> phi_b;
    phi_b << -0.1, 0.4, 0.2;

    auto strict_r = liepp::rotation<world, base, double, liepp::strict_policy>{
        liepp::so3<double, liepp::strict_policy>::exp(phi_a)};
    auto fast_r = liepp::rotation<base, tool, double, liepp::fast_policy>{
        liepp::so3<double, liepp::fast_policy>::exp(phi_b)};

    auto result = strict_r * fast_r;

    static_assert(std::is_same_v<
        decltype(result),
        liepp::rotation<world, tool, double, liepp::strict_policy>>);

    auto expected = (strict_r.matrix() * fast_r.matrix()).eval();
    REQUIRE((result.matrix() - expected).norm() < 1e-12);
}

// ============================================================================
// Identity
// ============================================================================

TEST_CASE("rotation: identity", "[rotation]")
{
    auto r = liepp::rotation<world, base>::identity();
    auto I = liepp::matrix3<double>::Identity();
    REQUIRE((r.matrix() - I).norm() < 1e-14);
}

// ============================================================================
// Forwarded methods
// ============================================================================

TEST_CASE("rotation: forwarded methods", "[rotation]")
{
    liepp::vector3<double> phi;
    phi << 0.5, -0.3, 0.8;
    auto inner = liepp::so3<double>::exp(phi);
    auto r = liepp::rotation<world, base>{inner};

    SECTION("matrix()")
    {
        REQUIRE((r.matrix() - inner.matrix()).norm() < 1e-14);
    }

    SECTION("quaternion_ref()")
    {
        REQUIRE(r.quaternion_ref().w() == Approx(inner.quaternion_ref().w()));
    }

    SECTION("log()")
    {
        auto log_val = r.log();
        auto expected = inner.log();
        REQUIRE((log_val - expected).norm() < 1e-14);
    }

    SECTION("act()")
    {
        liepp::vector3<double> v;
        v << 1.0, 2.0, 3.0;
        REQUIRE((r.act(v) - inner.act(v)).norm() < 1e-14);
    }
}

// ============================================================================
// from_matrix and from_quaternion
// ============================================================================

TEST_CASE("rotation: from_matrix roundtrip", "[rotation]")
{
    liepp::vector3<double> phi;
    phi << 0.3, -0.5, 0.7;
    auto r = liepp::rotation<world, base>{liepp::so3<double>::exp(phi)};
    auto result = liepp::rotation<world, base>::from_matrix(r.matrix());
    REQUIRE(result.has_value());
    REQUIRE((result.value().matrix() - r.matrix()).norm() < 1e-12);
}

TEST_CASE("rotation: from_quaternion", "[rotation]")
{
    liepp::quaternion<double> q(1.0, 0.0, 0.0, 0.0);
    auto result = liepp::rotation<world, base>::from_quaternion(q);
    REQUIRE(result.has_value());
    auto I = liepp::matrix3<double>::Identity();
    REQUIRE((result.value().matrix() - I).norm() < 1e-14);
}

// ============================================================================
// Different tag types
// ============================================================================

TEST_CASE("rotation: enum class tags", "[rotation]")
{
    using world_e = std::integral_constant<frame_enum, frame_enum::world>;
    using base_e = std::integral_constant<frame_enum, frame_enum::base>;

    auto r = liepp::rotation<world_e, base_e>::identity();
    auto I = liepp::matrix3<double>::Identity();
    REQUIRE((r.matrix() - I).norm() < 1e-14);
}

// ============================================================================
// m_value accessibility
// ============================================================================

TEST_CASE("rotation: m_value is accessible as so3", "[rotation]")
{
    auto r = liepp::rotation<world, base>{liepp::so3<double>::identity()};
    auto R = r.m_value.matrix();
    auto I = liepp::matrix3<double>::Identity();
    REQUIRE((R - I).norm() < 1e-14);
}
