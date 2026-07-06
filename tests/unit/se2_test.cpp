#include <cartan/lie/se2.h>
#include <cartan/detail/epsilon.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <cmath>
#include <limits>
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
    REQUIRE(result.error() == cartan::lie_failure::non_orthogonal);
}

TEST_CASE("se2: from_matrix rejects wrong bottom row", "[se2]")
{
    Eigen::Matrix<double, 3, 3> bad;
    bad << 1.0, 0.0, 1.0,
           0.0, 1.0, 2.0,
           1.0, 0.0, 1.0;  // bottom row should be [0, 0, 1]

    auto result = cartan::se2<double>::from_matrix(bad);
    REQUIRE_FALSE(result.has_value());
    REQUIRE(result.error() == cartan::lie_failure::invalid_affine_row);
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
// Mixed-policy compose
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

// ============================================================================
// Small-omega rotation preservation
//
// exp/log must route rotation through SO(2) at every omega. so2::exp (cos/sin)
// and so2::log (atan2) are well-conditioned everywhere, so no special-case is
// justified. The historical short-circuit at |omega| < sqrt(epsilon) returned
// identity rotation, silently discarding up to 0.0198 deg in float, and its log
// mirror dropped the V-inverse (omega/2) off-diagonal term. These cases pin the
// rotation-preserving contract.
// ============================================================================

TEMPLATE_TEST_CASE("se2: small-omega exp/log round-trip preserves rotation",
                   "[se2][sinc]", double, float)
{
    using S = TestType;

    // Fixed translation part of the twist.
    const S vx = S(1.5);
    const S vy = S(-0.7);

    // Omega grid straddling the historical sqrt(epsilon) short-circuit
    // (float ~3.45e-4, double ~1.49e-8): the small end used to collapse to
    // identity rotation and lose omega entirely.
    for (S omega : {S(1e-6), S(1e-5), S(1e-4), S(3e-4)})
    {
        cartan::vector3<S> v;
        v << omega, vx, vy;

        auto T = cartan::se2<S>::exp(v);
        auto recovered = T.log();

        // Rotation must be recovered, not dropped to zero. On the pre-fix code
        // recovered(0) is 0 for omega below the short-circuit -> 100% rel error.
        REQUIRE(std::abs(recovered(0) - omega) <= S(1e-5) * omega);

        // Translation part round-trips too.
        REQUIRE(std::abs(recovered(1) - vx) <= S(1e-4) * (S(1) + std::abs(vx)));
        REQUIRE(std::abs(recovered(2) - vy) <= S(1e-4) * (S(1) + std::abs(vy)));
    }
}

TEST_CASE("se2: float rotation just below sqrt(epsilon) is not collapsed",
          "[se2][sinc][float]")
{
    using S = float;

    // omega ~ 0.0172 deg, just below sqrt(epsilon(float)) ~ 3.45e-4 where the
    // historical branch fired and returned identity rotation.
    const S omega = S(3e-4);
    REQUIRE(omega < cartan::detail::sqrt_epsilon_v<S>);

    cartan::vector3<S> v;
    v << omega, S(1.0), S(2.0);

    auto T = cartan::se2<S>::exp(v);

    // The embedded rotation must carry omega, not identity. sin_angle ~ omega.
    REQUIRE(T.rotation().sin_angle() == Approx(std::sin(omega)).margin(S(1e-7)));
    REQUIRE(T.rotation().log() == Approx(omega).epsilon(S(1e-4)));

    // Explicitly reject the pre-fix collapse-to-identity behavior.
    REQUIRE(std::abs(T.rotation().sin_angle()) > S(1e-6));
}

// ============================================================================
// Coefficient stability sweep + recorded guard
//
// Oracle is a long-double evaluation of the closed forms using the RAW
// long-double epsilon (std::numeric_limits<long double>::epsilon()), NOT the
// sqrt_epsilon trait -- the trait's long-double branch is unreliable. The
// cancellation-free b = 2 sin^2(omega/2)/omega matches the oracle to machine
// eps across the whole small-omega grid, whereas the naive (1-cos(omega))/omega
// blows up (~13% relative error near omega=1.6e-8 in double). The guard trait
// se2_sinc_guard_v marks the switch to the Taylor limbs; it exists only to
// dodge the literal 0/0 at omega==0 and sits far below sqrt(epsilon), because
// the closed forms stay exact for every omega != 0.
// ============================================================================

TEMPLATE_TEST_CASE("se2: small-omega coefficient stability and recorded guard",
                   "[se2][sinc]", double, float)
{
    using S = TestType;

    const S s_eps = std::numeric_limits<S>::epsilon();

    // Stable closed b matches the long-double oracle to a few eps across the
    // grid; the recorded guard sits below sqrt(epsilon) yet both the closed and
    // Taylor limbs agree to eps there (validated pointwise below).
    REQUIRE(cartan::detail::se2_sinc_guard_v<S> < cartan::detail::sqrt_epsilon_v<S>);

    for (int e = -8; e <= -2; ++e)
    {
        S w = S(std::pow(10.0, double(e)));

        // Oracle: long double, raw long-double epsilon (not the trait).
        long double lw = static_cast<long double>(w);
        long double b_oracle = 2.0L * std::sin(lw / 2.0L) * std::sin(lw / 2.0L) / lw;

        // Stable closed form in Scalar precision.
        S sh = std::sin(w / S(2));
        S b_closed = S(2) * sh * sh / w;

        long double b_re =
            std::abs((static_cast<long double>(b_closed) - b_oracle) / b_oracle);
        REQUIRE(b_re <= 8.0L * static_cast<long double>(s_eps));

        // Naive (1-cos)/omega loses accuracy near zero -- documents the defect
        // the stable form fixes (blow-up cited above). Not used in production.
        S b_naive = (S(1) - std::cos(w)) / w;
        (void)b_naive;
    }

    // At the recorded guard the Taylor limbs and the closed limbs agree to eps,
    // so switching to Taylor below the guard introduces no discontinuity.
    const S g = cartan::detail::se2_sinc_guard_v<S>;
    {
        long double lg = static_cast<long double>(g);
        long double a_oracle = std::sin(lg) / lg;
        long double b_oracle = 2.0L * std::sin(lg / 2.0L) * std::sin(lg / 2.0L) / lg;
        long double hc_oracle = (lg / 2.0L) / std::tan(lg / 2.0L);

        S a_taylor = S(1) - g * g / S(6);
        S b_taylor = g / S(2);
        S hc_taylor = S(1) - g * g / S(12);

        REQUIRE(std::abs((static_cast<long double>(a_taylor) - a_oracle) / a_oracle)
                <= 4.0L * static_cast<long double>(s_eps));
        REQUIRE(std::abs((static_cast<long double>(b_taylor) - b_oracle) / b_oracle)
                <= 4.0L * static_cast<long double>(s_eps));
        REQUIRE(std::abs((static_cast<long double>(hc_taylor) - hc_oracle) / hc_oracle)
                <= 4.0L * static_cast<long double>(s_eps));
    }
}

// ============================================================================
// Default constructor is identity
// ============================================================================

TEMPLATE_TEST_CASE("se2: default constructor is identity", "[se2]", double, float)
{
    using S = TestType;
    S tol = std::is_same_v<S, float> ? S(1e-6) : S(1e-12);
    cartan::se2<S> d{};
    REQUIRE(d.isApprox(cartan::se2<S>::identity(), tol));
    REQUIRE((d.matrix() - Eigen::Matrix<S, 3, 3>::Identity()).norm() < tol);
}

// ============================================================================
// Manifold-aware isApprox
// ============================================================================

TEMPLATE_TEST_CASE("se2: isApprox reflexive and tolerance-sensitive", "[se2]", double, float)
{
    using S = TestType;
    S tol = std::is_same_v<S, float> ? S(1e-5) : S(1e-10);
    cartan::vector3<S> v;
    v << S(0.4), S(0.3), S(-0.2);
    auto t = cartan::se2<S>::exp(v);
    REQUIRE(t.isApprox(t, tol));

    cartan::vector3<S> v_far = v + cartan::vector3<S>::Constant(S(0.1));
    REQUIRE_FALSE(t.isApprox(cartan::se2<S>::exp(v_far), tol));
}

TEST_CASE("se2: isApprox is rotation-wrap safe", "[se2]")
{
    constexpr double pi = std::numbers::pi;
    cartan::vector2<double> trans(0.5, -0.3);
    // Rotation parts straddle +/-pi but denote the same element (differ by ~2e-10)
    cartan::se2<double> a(cartan::so2<double>::exp(pi - 1e-10), trans);
    cartan::se2<double> b(cartan::so2<double>::exp(pi + 1e-10), trans);
    REQUIRE(a.isApprox(b, 1e-6));
}
