#include <cartan/frames/framed_twist.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <type_traits>

using Catch::Approx;

// Frame tags
struct world {};
struct base {};
struct tool {};

// ============================================================================
// adjoint_map with known transform and twist
// ============================================================================

TEST_CASE("framed_twist: adjoint_map frame propagation", "[framed_twist]")
{
    // Create a known transform
    cartan::vector6<double> v;
    v << 0.3, -0.1, 0.5, 1.0, -2.0, 0.5;
    auto T = cartan::transform<world, base>{cartan::se3<double>::exp(v)};

    // Create a twist in the base frame
    cartan::twist<double> tw;
    tw.omega << 0.0, 0.0, 1.0;
    tw.v << 0.0, 0.0, 0.0;
    auto ft = cartan::framed_twist<base>{tw};

    // Apply adjoint map
    auto result = cartan::adjoint_map(T, ft);

    // Result type should be framed_twist<world>
    static_assert(std::is_same_v<
        decltype(result),
        cartan::framed_twist<world, double>>);

    // Numerical check: matches raw Ad_T * V
    cartan::matrix6<double> Ad = T.m_value.adjoint();
    cartan::vector6<double> expected = Ad * tw.to_vector();
    REQUIRE((result.to_vector() - expected).norm() < 1e-12);
}

// ============================================================================
// Accessors
// ============================================================================

TEST_CASE("framed_twist: omega and v accessors", "[framed_twist]")
{
    cartan::twist<double> tw;
    tw.omega << 1.0, 2.0, 3.0;
    tw.v << 4.0, 5.0, 6.0;
    auto ft = cartan::framed_twist<world>{tw};

    REQUIRE(ft.omega()(0) == Approx(1.0));
    REQUIRE(ft.omega()(1) == Approx(2.0));
    REQUIRE(ft.omega()(2) == Approx(3.0));
    REQUIRE(ft.v()(0) == Approx(4.0));
    REQUIRE(ft.v()(1) == Approx(5.0));
    REQUIRE(ft.v()(2) == Approx(6.0));
}

// ============================================================================
// from_vector factory
// ============================================================================

TEST_CASE("framed_twist: from_vector factory", "[framed_twist]")
{
    cartan::vector6<double> vec;
    vec << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
    auto ft = cartan::framed_twist<world>::from_vector(vec);
    REQUIRE((ft.to_vector() - vec).norm() < 1e-14);
}

// ============================================================================
// to_vector roundtrip
// ============================================================================

TEST_CASE("framed_twist: to_vector roundtrip", "[framed_twist]")
{
    cartan::twist<double> tw;
    tw.omega << 0.1, -0.3, 0.5;
    tw.v << 1.0, 2.0, -1.0;
    auto ft = cartan::framed_twist<base>{tw};

    auto vec = ft.to_vector();
    auto ft2 = cartan::framed_twist<base>::from_vector(vec);
    REQUIRE((ft2.to_vector() - tw.to_vector()).norm() < 1e-14);
}

// ============================================================================
// adjoint_map numerical verification against raw se3
// ============================================================================

TEST_CASE("framed_twist: adjoint_map numerical match", "[framed_twist]")
{
    // Non-trivial transform
    cartan::vector6<double> v_T;
    v_T << 0.5, -0.3, 0.8, 2.0, -1.0, 0.5;
    auto T = cartan::transform<world, base>{cartan::se3<double>::exp(v_T)};

    // Non-trivial twist
    cartan::vector6<double> twist_vec;
    twist_vec << 0.2, -0.4, 0.6, 1.0, -0.5, 0.3;
    auto ft = cartan::framed_twist<base>::from_vector(twist_vec);

    auto result = cartan::adjoint_map(T, ft);

    // Compare with raw computation
    cartan::vector6<double> expected = T.m_value.adjoint() * twist_vec;
    REQUIRE((result.to_vector() - expected).norm() < 1e-12);
}

// ============================================================================
// m_value accessibility
// ============================================================================

TEST_CASE("framed_twist: m_value is accessible as twist", "[framed_twist]")
{
    cartan::twist<double> tw;
    tw.omega << 1.0, 0.0, 0.0;
    tw.v << 0.0, 0.0, 0.0;
    auto ft = cartan::framed_twist<world>{tw};
    REQUIRE(ft.m_value.omega(0) == Approx(1.0));
}
