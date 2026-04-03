#include <cartan/lie/se2.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <cmath>
#include <numbers>

using Catch::Approx;

// ============================================================================
// exp/log roundtrip
// ============================================================================

TEMPLATE_TEST_CASE("se2: exp/log roundtrip", "[se2]", double, float)
{
    using S = TestType;

    SECTION("zero twist gives identity")
    {
        cartan::vector3<S> v = cartan::vector3<S>::Zero();
        auto T = cartan::se2<S>::exp(v);
        auto log_v = T.log();
        REQUIRE(log_v.norm() == Approx(S(0)).margin(S(1e-10)));
    }

    SECTION("pure rotation")
    {
        cartan::vector3<S> v;
        v << S(0.5), S(0), S(0);  // omega-first
        auto T = cartan::se2<S>::exp(v);
        auto log_v = T.log();
        REQUIRE((log_v - v).norm() == Approx(S(0)).margin(S(1e-6)));
    }

    SECTION("pure translation")
    {
        cartan::vector3<S> v;
        v << S(0), S(1), S(2);  // omega=0, vx=1, vy=2
        auto T = cartan::se2<S>::exp(v);
        auto log_v = T.log();
        REQUIRE((log_v - v).norm() == Approx(S(0)).margin(S(1e-6)));
    }

    SECTION("combined rotation and translation")
    {
        cartan::vector3<S> v;
        v << S(0.3), S(1.5), S(-0.7);
        auto T = cartan::se2<S>::exp(v);
        auto log_v = T.log();
        REQUIRE((log_v - v).norm() == Approx(S(0)).margin(S(1e-5)));
    }
}

// ============================================================================
// Pure translation case
// ============================================================================

TEST_CASE("se2: pure translation produces identity rotation + translation", "[se2]")
{
    cartan::vector3<double> v;
    v << 0.0, 1.0, 2.0;

    auto T = cartan::se2<double>::exp(v);

    // Rotation should be identity
    REQUIRE(T.rotation().log() == Approx(0.0).margin(1e-14));

    // Translation should be (1, 2)
    REQUIRE(T.translation()(0) == Approx(1.0).margin(1e-14));
    REQUIRE(T.translation()(1) == Approx(2.0).margin(1e-14));
}

// ============================================================================
// Compose matches matrix multiplication
// ============================================================================

TEST_CASE("se2: compose matches matrix multiplication", "[se2]")
{
    cartan::vector3<double> v1;
    v1 << 0.3, 1.0, -0.5;
    cartan::vector3<double> v2;
    v2 << -0.2, 0.7, 1.2;

    auto T1 = cartan::se2<double>::exp(v1);
    auto T2 = cartan::se2<double>::exp(v2);

    auto composed = T1 * T2;
    auto matrix_product = (T1.matrix() * T2.matrix()).eval();

    REQUIRE((composed.matrix() - matrix_product).norm() == Approx(0.0).margin(1e-13));
}

// ============================================================================
// Inverse
// ============================================================================

TEST_CASE("se2: inverse cancels transform", "[se2]")
{
    cartan::vector3<double> v;
    v << 0.7, -1.3, 2.1;

    auto T = cartan::se2<double>::exp(v);
    auto result = T * T.inverse();

    // Should be identity
    REQUIRE(result.rotation().log() == Approx(0.0).margin(1e-13));
    REQUIRE(result.translation().norm() == Approx(0.0).margin(1e-13));
}

// ============================================================================
// Adjoint
// ============================================================================

TEST_CASE("se2: adjoint transforms twists correctly", "[se2]")
{
    // For SE(2), Ad_T * V should match the matrix-form adjoint action
    cartan::vector3<double> twist;
    twist << 0.3, 1.0, -0.5;

    auto T = cartan::se2<double>::exp(twist);
    auto Ad = T.adjoint();

    // The adjoint should be a 3x3 matrix
    REQUIRE(Ad.rows() == 3);
    REQUIRE(Ad.cols() == 3);

    // Verify adjoint property: Ad_T * V = T * exp(V) * T^{-1} via log
    cartan::vector3<double> V;
    V << 0.1, 0.5, -0.3;

    auto ad_V = Ad * V;  // adjoint action on V

    // Compare with direct computation: log(T * exp(V) * T^{-1})
    auto TexpV = T * cartan::se2<double>::exp(V);
    auto TexpVTinv = TexpV * T.inverse();
    auto direct = TexpVTinv.log();

    REQUIRE((ad_V - direct).norm() == Approx(0.0).margin(1e-12));
}

// ============================================================================
// from_matrix
// ============================================================================

TEST_CASE("se2: from_matrix with valid SE(2) matrix", "[se2]")
{
    cartan::vector3<double> v;
    v << 0.5, 1.0, -2.0;
    auto T = cartan::se2<double>::exp(v);

    auto result = cartan::se2<double>::from_matrix(T.matrix());
    REQUIRE(result.has_value());
    REQUIRE((result.value().matrix() - T.matrix()).norm() == Approx(0.0).margin(1e-13));
}

TEST_CASE("se2: from_matrix rejects invalid matrix", "[se2]")
{
    Eigen::Matrix<double, 3, 3> bad;
    bad << 1.0, 1.0, 0.0,
           0.0, 1.0, 0.0,
           0.0, 0.0, 1.0;

    auto result = cartan::se2<double>::from_matrix(bad);
    REQUIRE_FALSE(result.has_value());
}

TEST_CASE("se2: from_matrix rejects wrong bottom row", "[se2]")
{
    Eigen::Matrix<double, 3, 3> bad;
    bad << 1.0, 0.0, 1.0,
           0.0, 1.0, 2.0,
           1.0, 0.0, 1.0;  // bottom row should be [0, 0, 1]

    auto result = cartan::se2<double>::from_matrix(bad);
    REQUIRE_FALSE(result.has_value());
}

// ============================================================================
// matrix()
// ============================================================================

TEST_CASE("se2: matrix produces correct 3x3 homogeneous", "[se2]")
{
    auto T = cartan::se2<double>::identity();
    auto m = T.matrix();

    REQUIRE((m - Eigen::Matrix3d::Identity()).norm() == Approx(0.0).margin(1e-15));
}

// ============================================================================
// rotation() and translation() accessors
// ============================================================================

TEST_CASE("se2: rotation and translation accessors", "[se2]")
{
    cartan::vector3<double> v;
    v << 0.5, 1.0, 2.0;
    auto T = cartan::se2<double>::exp(v);

    // rotation should have angle ~0.5
    REQUIRE(T.rotation().log() == Approx(0.5).margin(1e-14));

    // Translation is NOT (1.0, 2.0) because SE(2) exp couples rotation and translation
    // Just verify it is nonzero
    REQUIRE(T.translation().norm() > 0.0);
}

// ============================================================================
// act (transform a point)
// ============================================================================

TEST_CASE("se2: act transforms a 2D point", "[se2]")
{
    // Pure translation
    cartan::vector3<double> v;
    v << 0.0, 3.0, 4.0;
    auto T = cartan::se2<double>::exp(v);

    cartan::vector2<double> p;
    p << 1.0, 1.0;
    auto result = T.act(p);
    REQUIRE(result(0) == Approx(4.0).margin(1e-14));
    REQUIRE(result(1) == Approx(5.0).margin(1e-14));
}

// ============================================================================
// Float variant (LIE-12)
// ============================================================================

TEST_CASE("se2: float scalar operations", "[se2][float]")
{
    cartan::vector3<float> v;
    v << 0.3f, 1.0f, -0.5f;

    auto T = cartan::se2<float>::exp(v);
    auto log_v = T.log();
    REQUIRE((log_v - v).norm() == Approx(0.0f).margin(1e-4f));

    auto inv_result = T * T.inverse();
    REQUIRE(inv_result.rotation().log() == Approx(0.0f).margin(1e-5f));
    REQUIRE(inv_result.translation().norm() == Approx(0.0f).margin(1e-5f));
}

// ============================================================================
// Mixed-policy compose (D-08)
// ============================================================================

TEST_CASE("se2: mixed-policy compose returns strict result", "[se2][policy]")
{
    cartan::vector3<double> v1;
    v1 << 0.1, 1.0, 0.0;
    cartan::vector3<double> v2;
    v2 << 0.2, 0.0, 1.0;

    auto strict_T = cartan::se2<double, cartan::strict_policy>::exp(v1);
    auto fast_T = cartan::se2<double, cartan::fast_policy>::exp(v2);

    auto result = strict_T * fast_T;

    static_assert(std::is_same_v<
        decltype(result),
        cartan::se2<double, cartan::strict_policy>>);

    // Just verify it's a valid transform
    auto matrix_product = (strict_T.matrix() * fast_T.matrix()).eval();
    REQUIRE((result.matrix() - matrix_product).norm() == Approx(0.0).margin(1e-13));
}
