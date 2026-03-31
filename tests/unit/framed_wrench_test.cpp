#include <liepp/frames/framed_wrench.h>

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
// coadjoint_map with known transform and wrench
// ============================================================================

TEST_CASE("framed_wrench: coadjoint_map frame propagation", "[framed_wrench]")
{
    // Create a known transform
    liepp::vector6<double> v;
    v << 0.3, -0.1, 0.5, 1.0, -2.0, 0.5;
    auto T = liepp::transform<world, base>{liepp::se3<double>::exp(v)};

    // Create a wrench in the base frame
    liepp::vector3<double> moment;
    moment << 0.0, 0.0, 1.0;
    liepp::vector3<double> force;
    force << 1.0, 0.0, 0.0;
    auto fw = liepp::framed_wrench<base>::from_moment_force(moment, force);

    // Apply coadjoint map
    auto result = liepp::coadjoint_map(T, fw);

    // Result type should be framed_wrench<world>
    static_assert(std::is_same_v<
        decltype(result),
        liepp::framed_wrench<world, double>>);

    // Numerical check: matches raw CoAd_T * W
    liepp::matrix6<double> CoAd = T.m_value.coadjoint();
    liepp::vector6<double> expected = CoAd * fw.m_value;
    REQUIRE((result.m_value - expected).norm() < 1e-12);
}

// ============================================================================
// Moment and force accessors
// ============================================================================

TEST_CASE("framed_wrench: moment and force accessors", "[framed_wrench]")
{
    liepp::vector3<double> m;
    m << 1.0, 2.0, 3.0;
    liepp::vector3<double> f;
    f << 4.0, 5.0, 6.0;
    auto fw = liepp::framed_wrench<world>::from_moment_force(m, f);

    // Moment-first ordering: [moment; force]
    REQUIRE(fw.moment()(0) == Approx(1.0));
    REQUIRE(fw.moment()(1) == Approx(2.0));
    REQUIRE(fw.moment()(2) == Approx(3.0));
    REQUIRE(fw.force()(0) == Approx(4.0));
    REQUIRE(fw.force()(1) == Approx(5.0));
    REQUIRE(fw.force()(2) == Approx(6.0));
}

// ============================================================================
// from_moment_force factory
// ============================================================================

TEST_CASE("framed_wrench: from_moment_force factory", "[framed_wrench]")
{
    liepp::vector3<double> m;
    m << 1.0, 2.0, 3.0;
    liepp::vector3<double> f;
    f << 4.0, 5.0, 6.0;
    auto fw = liepp::framed_wrench<world>::from_moment_force(m, f);

    liepp::vector6<double> expected;
    expected << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
    REQUIRE((fw.m_value - expected).norm() < 1e-14);
}

// ============================================================================
// coadjoint_map numerical verification
// ============================================================================

TEST_CASE("framed_wrench: coadjoint_map numerical match", "[framed_wrench]")
{
    // Non-trivial transform
    liepp::vector6<double> v_T;
    v_T << 0.5, -0.3, 0.8, 2.0, -1.0, 0.5;
    auto T = liepp::transform<world, base>{liepp::se3<double>::exp(v_T)};

    // Non-trivial wrench
    liepp::vector6<double> wrench_vec;
    wrench_vec << 0.2, -0.4, 0.6, 1.0, -0.5, 0.3;
    auto fw = liepp::framed_wrench<base>{wrench_vec};

    auto result = liepp::coadjoint_map(T, fw);

    // Compare with raw computation
    liepp::vector6<double> expected = T.m_value.coadjoint() * wrench_vec;
    REQUIRE((result.m_value - expected).norm() < 1e-12);
}

// ============================================================================
// m_value stores moment-first ordering
// ============================================================================

TEST_CASE("framed_wrench: m_value moment-first ordering", "[framed_wrench]")
{
    liepp::vector3<double> m;
    m << 10.0, 20.0, 30.0;
    liepp::vector3<double> f;
    f << 40.0, 50.0, 60.0;
    auto fw = liepp::framed_wrench<world>::from_moment_force(m, f);

    // First 3 elements are moment, last 3 are force
    REQUIRE(fw.m_value(0) == Approx(10.0));
    REQUIRE(fw.m_value(1) == Approx(20.0));
    REQUIRE(fw.m_value(2) == Approx(30.0));
    REQUIRE(fw.m_value(3) == Approx(40.0));
    REQUIRE(fw.m_value(4) == Approx(50.0));
    REQUIRE(fw.m_value(5) == Approx(60.0));
}

// ============================================================================
// m_value accessibility
// ============================================================================

TEST_CASE("framed_wrench: m_value is accessible as vector6", "[framed_wrench]")
{
    liepp::vector6<double> v;
    v << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
    auto fw = liepp::framed_wrench<world>{v};
    REQUIRE((fw.m_value - v).norm() < 1e-14);
}
