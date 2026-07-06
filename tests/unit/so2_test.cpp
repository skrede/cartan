#include <cartan/lie/so2.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <cmath>
#include <numbers>

using Catch::Approx;

// ============================================================================
// exp/log roundtrip
// ============================================================================

TEMPLATE_TEST_CASE("so2: exp/log roundtrip", "[so2]", double, float)
{
    using S = TestType;
    constexpr S pi = std::numbers::pi_v<S>;

    SECTION("theta = 0")
    {
        auto r = cartan::so2<S>::exp(S(0));
        REQUIRE(r.log() == Approx(S(0)).margin(S(1e-12)));
    }

    SECTION("theta = pi/4")
    {
        auto r = cartan::so2<S>::exp(pi / S(4));
        REQUIRE(r.log() == Approx(pi / S(4)).margin(S(1e-6)));
    }

    SECTION("theta = pi/2")
    {
        auto r = cartan::so2<S>::exp(pi / S(2));
        REQUIRE(r.log() == Approx(pi / S(2)).margin(S(1e-6)));
    }

    SECTION("theta = pi")
    {
        auto r = cartan::so2<S>::exp(pi);
        REQUIRE(std::abs(r.log()) == Approx(pi).margin(S(1e-6)));
    }

    SECTION("theta = -pi/2")
    {
        auto r = cartan::so2<S>::exp(-pi / S(2));
        REQUIRE(r.log() == Approx(-pi / S(2)).margin(S(1e-6)));
    }

    SECTION("theta = -pi")
    {
        auto r = cartan::so2<S>::exp(-pi);
        REQUIRE(std::abs(r.log()) == Approx(pi).margin(S(1e-6)));
    }

    SECTION("theta = 2*pi wraps to 0")
    {
        auto r = cartan::so2<S>::exp(S(2) * pi);
        REQUIRE(r.log() == Approx(S(0)).margin(S(1e-5)));
    }
}

// ============================================================================
// Identity properties
// ============================================================================

TEST_CASE("so2: identity properties", "[so2]")
{
    auto id = cartan::so2<double>::identity();
    auto x = cartan::so2<double>::exp(0.7);

    SECTION("identity * x == x")
    {
        auto result = id * x;
        REQUIRE(result.log() == Approx(x.log()).margin(1e-14));
    }

    SECTION("x * identity == x")
    {
        auto result = x * id;
        REQUIRE(result.log() == Approx(x.log()).margin(1e-14));
    }

    SECTION("identity matrix is 2x2 Identity")
    {
        auto m = id.matrix();
        REQUIRE(m(0, 0) == Approx(1.0));
        REQUIRE(m(0, 1) == Approx(0.0));
        REQUIRE(m(1, 0) == Approx(0.0));
        REQUIRE(m(1, 1) == Approx(1.0));
    }
}

// ============================================================================
// Inverse
// ============================================================================

TEST_CASE("so2: inverse cancels rotation", "[so2]")
{
    auto x = cartan::so2<double>::exp(1.23);
    auto result = x * x.inverse();
    REQUIRE(result.log() == Approx(0.0).margin(1e-14));
}

// ============================================================================
// Compose = angle addition
// ============================================================================

TEST_CASE("so2: compose is angle addition", "[so2]")
{
    double a = 0.3;
    double b = 0.5;
    auto ra = cartan::so2<double>::exp(a);
    auto rb = cartan::so2<double>::exp(b);

    auto composed = ra * rb;
    REQUIRE(composed.log() == Approx(a + b).margin(1e-14));
}

// ============================================================================
// matrix() output
// ============================================================================

TEST_CASE("so2: matrix returns correct 2x2 rotation", "[so2]")
{
    double theta = 0.7;
    auto r = cartan::so2<double>::exp(theta);
    auto m = r.matrix();

    REQUIRE(m(0, 0) == Approx(std::cos(theta)).margin(1e-14));
    REQUIRE(m(0, 1) == Approx(-std::sin(theta)).margin(1e-14));
    REQUIRE(m(1, 0) == Approx(std::sin(theta)).margin(1e-14));
    REQUIRE(m(1, 1) == Approx(std::cos(theta)).margin(1e-14));
}

// ============================================================================
// from_matrix
// ============================================================================

TEST_CASE("so2: from_matrix with valid rotation", "[so2]")
{
    double theta = 0.5;
    cartan::matrix2<double> R;
    R << std::cos(theta), -std::sin(theta),
         std::sin(theta),  std::cos(theta);

    auto result = cartan::so2<double>::from_matrix(R);
    REQUIRE(result.has_value());
    REQUIRE(result.value().log() == Approx(theta).margin(1e-14));
}

TEST_CASE("so2: from_matrix rejects non-orthogonal matrix", "[so2]")
{
    cartan::matrix2<double> bad;
    bad << 1.0, 1.0,
           0.0, 1.0;

    auto result = cartan::so2<double>::from_matrix(bad);
    REQUIRE_FALSE(result.has_value());
    REQUIRE(result.error() == cartan::lie_failure::non_orthogonal);
}

TEST_CASE("so2: from_matrix rejects negative determinant", "[so2]")
{
    cartan::matrix2<double> reflection;
    reflection << -1.0,  0.0,
                   0.0,  1.0;

    auto result = cartan::so2<double>::from_matrix(reflection);
    REQUIRE_FALSE(result.has_value());
    REQUIRE(result.error() == cartan::lie_failure::improper_rotation);
}

// ============================================================================
// Policy behavior
// ============================================================================

TEST_CASE("so2: strict policy normalizes on construction", "[so2][policy]")
{
    // Non-unit cos/sin pair: (3, 4) has magnitude 5
    cartan::so2<double, cartan::strict_policy> r(3.0, 4.0);
    REQUIRE(r.cos_angle() == Approx(3.0 / 5.0).margin(1e-14));
    REQUIRE(r.sin_angle() == Approx(4.0 / 5.0).margin(1e-14));
}

TEST_CASE("so2: fast policy does NOT normalize on construction", "[so2][policy]")
{
    cartan::so2<double, cartan::fast_policy> r(3.0, 4.0);
    REQUIRE(r.cos_angle() == Approx(3.0).margin(1e-14));
    REQUIRE(r.sin_angle() == Approx(4.0).margin(1e-14));
}

// ============================================================================
// Mixed-policy compose
// ============================================================================

TEST_CASE("so2: mixed-policy compose returns strict result", "[so2][policy]")
{
    auto strict_r = cartan::so2<double, cartan::strict_policy>::exp(0.3);
    auto fast_r = cartan::so2<double, cartan::fast_policy>::exp(0.5);

    auto result = strict_r * fast_r;

    // Result type should be strict_policy (stricter of the two)
    static_assert(std::is_same_v<
        decltype(result),
        cartan::so2<double, cartan::strict_policy>>);

    REQUIRE(result.log() == Approx(0.8).margin(1e-14));
}

// ============================================================================
// act (rotate a 2D vector)
// ============================================================================

TEST_CASE("so2: act rotates 2D vector", "[so2]")
{
    constexpr double pi = std::numbers::pi;
    auto r = cartan::so2<double>::exp(pi / 2.0);
    cartan::vector2<double> v;
    v << 1.0, 0.0;

    auto rotated = r.act(v);
    REQUIRE(rotated(0) == Approx(0.0).margin(1e-14));
    REQUIRE(rotated(1) == Approx(1.0).margin(1e-14));
}

// ============================================================================
// Float variant (LIE-12)
// ============================================================================

TEST_CASE("so2: float scalar operations", "[so2][float]")
{
    constexpr float pi = std::numbers::pi_v<float>;
    auto r = cartan::so2<float>::exp(pi / 4.0f);

    REQUIRE(r.log() == Approx(pi / 4.0f).margin(1e-6f));
    REQUIRE(r.cos_angle() == Approx(std::cos(pi / 4.0f)).margin(1e-6f));

    auto inv = r.inverse();
    auto product = r * inv;
    REQUIRE(product.log() == Approx(0.0f).margin(1e-5f));
}

// ============================================================================
// Default constructor is identity
// ============================================================================

TEMPLATE_TEST_CASE("so2: default constructor is identity", "[so2]", double, float)
{
    using S = TestType;
    S tol = std::is_same_v<S, float> ? S(1e-6) : S(1e-12);
    cartan::so2<S> d{};
    REQUIRE(d.isApprox(cartan::so2<S>::identity(), tol));
    REQUIRE(d.cos_angle() == Approx(S(1)).margin(tol));
    REQUIRE(d.sin_angle() == Approx(S(0)).margin(tol));
}

// ============================================================================
// Manifold-aware isApprox
// ============================================================================

TEMPLATE_TEST_CASE("so2: isApprox reflexive and tolerance-sensitive", "[so2]", double, float)
{
    using S = TestType;
    S tol = std::is_same_v<S, float> ? S(1e-5) : S(1e-10);
    auto r = cartan::so2<S>::exp(S(0.6));
    REQUIRE(r.isApprox(r, tol));
    REQUIRE(r.isApprox(cartan::so2<S>::exp(S(0.6) + tol / S(10)), tol));
    REQUIRE_FALSE(r.isApprox(cartan::so2<S>::exp(S(0.6) + S(0.1)), tol));
}

TEST_CASE("so2: isApprox is angle-wrap safe", "[so2]")
{
    constexpr double pi = std::numbers::pi;
    auto a = cartan::so2<double>::exp(pi - 1e-10);
    auto b = cartan::so2<double>::exp(pi + 1e-10);  // wraps to near -pi
    // Naive angle() difference is ~2*pi; manifold-aware isApprox sees ~2e-10
    REQUIRE(std::abs(a.angle() - b.angle()) > 1.0);
    REQUIRE(a.isApprox(b, 1e-8));
}
