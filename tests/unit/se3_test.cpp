#include <cartan/lie/se3.h>

#include <cartan/lie/hat_vee.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <cmath>
#include <numbers>

using Catch::Approx;

// ============================================================================
// exp/log roundtrip
// ============================================================================

TEMPLATE_TEST_CASE("se3: exp/log roundtrip", "[se3]", double, float)
{
    using S = TestType;
    S margin = std::is_same_v<S, float> ? S(1e-3) : S(1e-10);

    SECTION("pure rotation (rho=0)")
    {
        cartan::vector6<S> v = cartan::vector6<S>::Zero();
        v(0) = S(0.5);
        v(1) = S(-0.3);
        v(2) = S(0.8);
        auto T = cartan::se3<S>::exp(v);
        auto v_back = T.log();
        REQUIRE((v_back - v).norm() < margin);
    }

    SECTION("pure translation (omega=0)")
    {
        cartan::vector6<S> v = cartan::vector6<S>::Zero();
        v(3) = S(1.0);
        v(4) = S(2.0);
        v(5) = S(3.0);
        auto T = cartan::se3<S>::exp(v);
        auto v_back = T.log();
        REQUIRE((v_back - v).norm() < margin);
    }

    SECTION("combined rotation + translation")
    {
        cartan::vector6<S> v;
        v << S(0.3), S(-0.2), S(0.5), S(1.0), S(-0.5), S(0.8);
        auto T = cartan::se3<S>::exp(v);
        auto v_back = T.log();
        REQUIRE((v_back - v).norm() < margin);
    }

    SECTION("near-zero twist")
    {
        cartan::vector6<S> v;
        v << S(1e-10), S(2e-10), S(3e-10), S(1e-10), S(2e-10), S(3e-10);
        auto T = cartan::se3<S>::exp(v);
        auto v_back = T.log();
        REQUIRE((v_back - v).norm() < margin);
    }

    SECTION("large rotation (theta near pi)")
    {
        cartan::vector6<S> v;
        S theta = S(3.0);
        cartan::vector3<S> axis;
        axis << S(0), S(0), S(1);
        v.template head<3>() = theta * axis;
        v.template tail<3>() << S(0.5), S(-0.3), S(1.2);
        auto T = cartan::se3<S>::exp(v);
        auto v_back = T.log();
        // Compare via matrix since axis may flip near pi
        auto T_back = cartan::se3<S>::exp(v_back);
        REQUIRE((T_back.matrix() - T.matrix()).norm() < margin);
    }
}

// ============================================================================
// exp(zero) == identity
// ============================================================================

TEST_CASE("se3: exp(zero) is identity", "[se3]")
{
    auto T = cartan::se3<double>::exp(cartan::vector6<double>::Zero());
    auto I = cartan::matrix4<double>::Identity();
    REQUIRE((T.matrix() - I).norm() < 1e-14);
}

// ============================================================================
// Pure translation preserves identity rotation
// ============================================================================

TEST_CASE("se3: pure translation produces identity rotation", "[se3]")
{
    cartan::vector6<double> v = cartan::vector6<double>::Zero();
    v(3) = 1.0;
    v(4) = 2.0;
    v(5) = 3.0;
    auto T = cartan::se3<double>::exp(v);
    auto I3 = cartan::matrix3<double>::Identity();
    REQUIRE((T.rotation().matrix() - I3).norm() < 1e-12);
    REQUIRE(T.translation()(0) == Approx(1.0).margin(1e-12));
    REQUIRE(T.translation()(1) == Approx(2.0).margin(1e-12));
    REQUIRE(T.translation()(2) == Approx(3.0).margin(1e-12));
}

// ============================================================================
// Compose matches matrix product
// ============================================================================

TEST_CASE("se3: compose matches matrix product", "[se3]")
{
    cartan::vector6<double> v_a;
    v_a << 0.1, 0.5, -0.3, 1.0, -0.5, 0.2;
    cartan::vector6<double> v_b;
    v_b << -0.4, 0.2, 0.6, 0.3, 0.8, -0.4;

    auto a = cartan::se3<double>::exp(v_a);
    auto b = cartan::se3<double>::exp(v_b);
    auto composed = a * b;

    auto expected = (a.matrix() * b.matrix()).eval();
    REQUIRE((composed.matrix() - expected).norm() < 1e-10);
}

// ============================================================================
// Inverse
// ============================================================================

TEST_CASE("se3: inverse cancels transform", "[se3]")
{
    cartan::vector6<double> v;
    v << 0.7, -0.3, 1.2, 0.5, -0.8, 0.3;
    auto T = cartan::se3<double>::exp(v);
    auto result = T * T.inverse();
    auto I = cartan::matrix4<double>::Identity();
    REQUIRE((result.matrix() - I).norm() < 1e-10);
}

// ============================================================================
// Adjoint 6x6 block layout: [R, 0; [p]R, R]
// ============================================================================

TEST_CASE("se3: adjoint 6x6 block layout", "[se3]")
{
    cartan::vector6<double> v;
    v << 0.5, -0.3, 0.8, 1.0, -0.5, 0.3;
    auto T = cartan::se3<double>::exp(v);
    auto Ad = T.adjoint();
    auto R = T.rotation().matrix();
    auto p = T.translation();

    // Top-left 3x3 = R
    REQUIRE((Ad.block<3, 3>(0, 0) - R).norm() < 1e-12);

    // Top-right 3x3 = 0
    REQUIRE(Ad.block<3, 3>(0, 3).norm() < 1e-12);

    // Bottom-left 3x3 = hat(p) * R
    auto pR = (cartan::hat(p) * R).eval();
    REQUIRE((Ad.block<3, 3>(3, 0) - pR).norm() < 1e-12);

    // Bottom-right 3x3 = R
    REQUIRE((Ad.block<3, 3>(3, 3) - R).norm() < 1e-12);
}

// ============================================================================
// Adjoint identity: Ad_T * V == vee(T * hat(V) * T^{-1})
// ============================================================================

TEST_CASE("se3: adjoint identity numerical check", "[se3]")
{
    cartan::vector6<double> v_T;
    v_T << 0.5, -0.3, 0.8, 1.0, -0.5, 0.3;
    auto T = cartan::se3<double>::exp(v_T);

    cartan::vector6<double> V;
    V << 0.2, -0.4, 0.6, 0.8, -0.1, 0.3;

    // Method 1: Ad_T * V
    cartan::vector6<double> lhs = T.adjoint() * V;

    // Method 2: vee(T * hat(V) * T^{-1})
    cartan::matrix4<double> V_hat = cartan::hat(V);
    cartan::matrix4<double> result = T.matrix() * V_hat * T.inverse().matrix();
    cartan::vector6<double> rhs = cartan::vee(result);

    REQUIRE((lhs - rhs).norm() < 1e-10);
}

// ============================================================================
// Coadjoint identity: coadjoint == inverse().adjoint().transpose()
// ============================================================================

TEST_CASE("se3: coadjoint identity", "[se3]")
{
    cartan::vector6<double> v;
    v << 0.5, -0.3, 0.8, 1.0, -0.5, 0.3;
    auto T = cartan::se3<double>::exp(v);

    auto coad = T.coadjoint();
    auto expected = T.inverse().adjoint().transpose().eval();
    REQUIRE((coad - expected).norm() < 1e-10);
}

// ============================================================================
// from_matrix with valid / invalid SE(3)
// ============================================================================

TEST_CASE("se3: from_matrix with valid SE(3)", "[se3]")
{
    cartan::vector6<double> v;
    v << 0.3, -0.5, 0.7, 1.0, 2.0, 3.0;
    auto T = cartan::se3<double>::exp(v);
    auto result = cartan::se3<double>::from_matrix(T.matrix());
    REQUIRE(result.has_value());
    REQUIRE((result.value().matrix() - T.matrix()).norm() < 1e-10);
}

TEST_CASE("se3: from_matrix rejects invalid bottom row", "[se3]")
{
    cartan::matrix4<double> bad = cartan::matrix4<double>::Identity();
    bad(3, 0) = 0.5;
    auto result = cartan::se3<double>::from_matrix(bad);
    REQUIRE_FALSE(result.has_value());
    REQUIRE(result.error() == cartan::lie_failure::invalid_affine_row);
}

// ============================================================================
// matrix() produces correct 4x4 homogeneous
// ============================================================================

TEST_CASE("se3: matrix produces correct 4x4 homogeneous", "[se3]")
{
    cartan::vector6<double> v;
    v << 0.3, -0.5, 0.7, 1.0, 2.0, 3.0;
    auto T = cartan::se3<double>::exp(v);
    auto M = T.matrix();

    // Check bottom row
    REQUIRE(M(3, 0) == Approx(0.0).margin(1e-14));
    REQUIRE(M(3, 1) == Approx(0.0).margin(1e-14));
    REQUIRE(M(3, 2) == Approx(0.0).margin(1e-14));
    REQUIRE(M(3, 3) == Approx(1.0).margin(1e-14));

    // Check rotation block matches
    REQUIRE((M.block<3, 3>(0, 0) - T.rotation().matrix()).norm() < 1e-14);

    // Check translation block matches
    REQUIRE((M.block<3, 1>(0, 3) - T.translation()).norm() < 1e-14);
}

// ============================================================================
// rotation() and translation() accessors
// ============================================================================

TEST_CASE("se3: rotation and translation accessors", "[se3]")
{
    cartan::vector6<double> v;
    v << 0.3, -0.2, 0.5, 1.0, -0.5, 0.8;
    auto T = cartan::se3<double>::exp(v);

    // rotation returns an so3 with a valid matrix
    auto R = T.rotation().matrix();
    auto RtR = R.transpose() * R;
    REQUIRE((RtR - cartan::matrix3<double>::Identity()).norm() < 1e-12);

    // translation returns a vector3
    auto t = T.translation();
    REQUIRE(t.size() == 3);
}

// ============================================================================
// Policy behavior
// ============================================================================

TEST_CASE("se3: strict vs fast policy", "[se3][policy]")
{
    cartan::vector6<double> v;
    v << 0.3, -0.2, 0.5, 1.0, -0.5, 0.8;

    auto T_strict = cartan::se3<double, cartan::strict_policy>::exp(v);
    auto T_fast = cartan::se3<double, cartan::fast_policy>::exp(v);

    REQUIRE((T_strict.matrix() - T_fast.matrix()).norm() < 1e-10);
}

// ============================================================================
// Mixed-policy compose
// ============================================================================

TEST_CASE("se3: mixed-policy compose returns strict result", "[se3][policy]")
{
    cartan::vector6<double> v_a;
    v_a << 0.1, 0.2, -0.1, 0.5, 0.3, 0.1;
    cartan::vector6<double> v_b;
    v_b << -0.1, 0.3, 0.2, 0.2, -0.1, 0.4;

    auto strict_T = cartan::se3<double, cartan::strict_policy>::exp(v_a);
    auto fast_T = cartan::se3<double, cartan::fast_policy>::exp(v_b);

    auto result = strict_T * fast_T;

    static_assert(std::is_same_v<
        decltype(result),
        cartan::se3<double, cartan::strict_policy>>);

    auto expected = (strict_T.matrix() * fast_T.matrix()).eval();
    REQUIRE((result.matrix() - expected).norm() < 1e-10);
}

// ============================================================================
// Float scalar variant (LIE-12)
// ============================================================================

TEST_CASE("se3: float scalar operations", "[se3][float]")
{
    cartan::vector6<float> v;
    v << 0.3f, -0.2f, 0.5f, 1.0f, -0.5f, 0.8f;

    auto T = cartan::se3<float>::exp(v);
    auto v_back = T.log();
    REQUIRE((v_back - v).norm() < 1e-3f);

    auto inv = T.inverse();
    auto product = T * inv;
    auto I = cartan::matrix4<float>::Identity();
    REQUIRE((product.matrix() - I).norm() < 1e-4f);
}

// ============================================================================
// SE(3) log with pure rotation (Pitfall 4: p=0 but rotation != I)
// ============================================================================

TEST_CASE("se3: log with pure rotation (zero translation)", "[se3]")
{
    // Create SE(3) with rotation but no translation
    auto rot = cartan::so3<double>::exp(cartan::vector3<double>(0.5, -0.3, 0.8));
    auto T = cartan::se3<double>(rot, cartan::vector3<double>::Zero());

    auto v = T.log();

    // Omega part should match the so3 log
    cartan::vector3<double> omega = v.head<3>();
    cartan::vector3<double> expected_omega = rot.log();
    REQUIRE((omega - expected_omega).norm() < 1e-12);

    // Rho part should be zero (since translation is zero)
    REQUIRE(v.tail<3>().norm() < 1e-12);
}
