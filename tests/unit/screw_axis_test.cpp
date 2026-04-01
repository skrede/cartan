#include <liepp/serial/chain/screw_axis.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <cmath>

using Catch::Approx;

// ============================================================================
// screw_axis revolute factory
// ============================================================================

TEST_CASE("screw_axis revolute factory", "[screw_axis]")
{
    SECTION("axis at origin: v = 0")
    {
        auto sa = liepp::screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
        REQUIRE(sa.omega()(0) == Approx(0.0).margin(1e-12));
        REQUIRE(sa.omega()(1) == Approx(0.0).margin(1e-12));
        REQUIRE(sa.omega()(2) == Approx(1.0).margin(1e-12));
        REQUIRE(sa.v().norm() < 1e-12);
        REQUIRE(sa.is_revolute());
        REQUIRE_FALSE(sa.is_prismatic());
    }

    SECTION("axis offset from origin: v = -omega x point")
    {
        double L = 0.5;
        auto sa = liepp::screw_axis<double>::revolute({0, 0, 1}, {L, 0, 0});
        REQUIRE(sa.omega()(2) == Approx(1.0).margin(1e-12));
        // v = -(0,0,1) x (0.5,0,0) = -(0*0-1*0, 1*0.5-0*0, 0*0-0*0.5) = -(0, 0.5, 0) = (0, -0.5, 0)
        // Wait: -(0,0,1) x (0.5,0,0) = -( (0*0-1*0), (1*0.5-0*0), (0*0-0*0.5) ) = -(0, 0.5, 0) = (0, -0.5, 0)
        // Actually let me recalculate: w x p = (0,0,1) x (0.5,0,0) = (0*0-1*0, 1*0.5-0*0, 0*0-0*0.5) = (0, 0.5, 0)
        // v = -w x p = (0, -0.5, 0)
        // But plan says v=[0,L,0]. Let me check: plan says revolute({0,0,1}, {L,0,0}) -> v=[0,L,0]
        // That would be wrong. v = -omega x point = -(0,0,1)x(L,0,0) = -(0, L, 0) = (0, -L, 0)
        // The plan's behavior section says v=[0,L,0] which contradicts v = -omega x point.
        // The formula v = -omega x point gives (0, -L, 0). Let me verify with Lynch & Park:
        // S = (omega, -omega x q) where q is a point on the axis. So v = -omega x q.
        // (0,0,1) x (L,0,0) = (0*0-1*0, 1*L-0*0, 0*0-0*L) = (0, L, 0)
        // v = -(0, L, 0) = (0, -L, 0)
        REQUIRE(sa.v()(0) == Approx(0.0).margin(1e-12));
        REQUIRE(sa.v()(1) == Approx(-L).margin(1e-12));
        REQUIRE(sa.v()(2) == Approx(0.0).margin(1e-12));
    }

    SECTION("normalizes non-unit axis")
    {
        auto sa = liepp::screw_axis<double>::revolute({0, 0, 2}, {0, 0, 0});
        REQUIRE(sa.omega().norm() == Approx(1.0).margin(1e-12));
        REQUIRE(sa.omega()(2) == Approx(1.0).margin(1e-12));
    }
}

// ============================================================================
// screw_axis prismatic factory
// ============================================================================

TEST_CASE("screw_axis prismatic factory", "[screw_axis]")
{
    SECTION("unit direction")
    {
        auto sa = liepp::screw_axis<double>::prismatic({1, 0, 0});
        REQUIRE(sa.omega().norm() < 1e-12);
        REQUIRE(sa.v()(0) == Approx(1.0).margin(1e-12));
        REQUIRE(sa.v()(1) == Approx(0.0).margin(1e-12));
        REQUIRE(sa.v()(2) == Approx(0.0).margin(1e-12));
        REQUIRE(sa.is_prismatic());
        REQUIRE_FALSE(sa.is_revolute());
    }

    SECTION("normalizes non-unit direction")
    {
        auto sa = liepp::screw_axis<double>::prismatic({2, 0, 0});
        REQUIRE(sa.v()(0) == Approx(1.0).margin(1e-12));
        REQUIRE(sa.v().norm() == Approx(1.0).margin(1e-12));
    }
}

// ============================================================================
// screw_axis from_vector validation
// ============================================================================

TEST_CASE("screw_axis from_vector validation", "[screw_axis]")
{
    SECTION("unit omega passes")
    {
        liepp::vector6<double> vec;
        vec << 0, 0, 1, 0, -0.5, 0;
        auto result = liepp::screw_axis<double>::from_vector(vec);
        REQUIRE(result.has_value());
        REQUIRE(result->omega()(2) == Approx(1.0).margin(1e-12));
    }

    SECTION("non-unit omega fails")
    {
        liepp::vector6<double> vec;
        vec << 0, 0, 2, 0, 0, 0;
        auto result = liepp::screw_axis<double>::from_vector(vec);
        REQUIRE_FALSE(result.has_value());
    }

    SECTION("zero omega with unit v passes")
    {
        liepp::vector6<double> vec;
        vec << 0, 0, 0, 1, 0, 0;
        auto result = liepp::screw_axis<double>::from_vector(vec);
        REQUIRE(result.has_value());
        REQUIRE(result->is_prismatic());
    }

    SECTION("zero omega with non-unit v fails")
    {
        liepp::vector6<double> vec;
        vec << 0, 0, 0, 2, 0, 0;
        auto result = liepp::screw_axis<double>::from_vector(vec);
        REQUIRE_FALSE(result.has_value());
    }
}

// ============================================================================
// screw_axis to_vector roundtrip
// ============================================================================

TEST_CASE("screw_axis to_vector roundtrip", "[screw_axis]")
{
    auto sa = liepp::screw_axis<double>::revolute({0, 0, 1}, {0.5, 0, 0});
    auto vec = sa.to_vector();
    auto result = liepp::screw_axis<double>::from_vector(vec);
    REQUIRE(result.has_value());
    REQUIRE((result->omega() - sa.omega()).norm() < 1e-12);
    REQUIRE((result->v() - sa.v()).norm() < 1e-12);
}

// ============================================================================
// screw_axis float scalar
// ============================================================================

TEST_CASE("screw_axis float scalar", "[screw_axis][float]")
{
    auto sa = liepp::screw_axis<float>::revolute({0, 0, 1}, {0, 0, 0});
    REQUIRE(sa.omega()(2) == Approx(1.0f).margin(1e-6f));
    REQUIRE(sa.is_revolute());

    auto sp = liepp::screw_axis<float>::prismatic({0, 1, 0});
    REQUIRE(sp.v()(1) == Approx(1.0f).margin(1e-6f));
    REQUIRE(sp.is_prismatic());
}
