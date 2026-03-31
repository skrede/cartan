#include <liepp/lie/so3.h>

#include <liepp/lie/hat_vee.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <cmath>
#include <numbers>

using Catch::Approx;

// ============================================================================
// exp/log roundtrip
// ============================================================================

TEMPLATE_TEST_CASE("so3: exp/log roundtrip", "[so3]", double, float)
{
    using S = TestType;
    constexpr S pi = std::numbers::pi_v<S>;
    S margin = std::is_same_v<S, float> ? S(1e-4) : S(1e-10);

    SECTION("theta near zero (1e-10)")
    {
        liepp::vector3<S> phi;
        phi << S(1e-10), S(2e-10), S(3e-10);
        auto r = liepp::so3<S>::exp(phi);
        auto phi_back = r.log();
        REQUIRE(phi_back(0) == Approx(phi(0)).margin(margin));
        REQUIRE(phi_back(1) == Approx(phi(1)).margin(margin));
        REQUIRE(phi_back(2) == Approx(phi(2)).margin(margin));
    }

    SECTION("theta = pi/4")
    {
        S theta = pi / S(4);
        liepp::vector3<S> phi;
        phi << theta, S(0), S(0);
        auto r = liepp::so3<S>::exp(phi);
        auto phi_back = r.log();
        REQUIRE(phi_back(0) == Approx(phi(0)).margin(margin));
        REQUIRE(phi_back(1) == Approx(phi(1)).margin(margin));
        REQUIRE(phi_back(2) == Approx(phi(2)).margin(margin));
    }

    SECTION("theta = pi/2")
    {
        S theta = pi / S(2);
        liepp::vector3<S> axis;
        axis << S(0), S(1), S(0);
        liepp::vector3<S> phi = theta * axis;
        auto r = liepp::so3<S>::exp(phi);
        auto phi_back = r.log();
        REQUIRE(phi_back(0) == Approx(phi(0)).margin(margin));
        REQUIRE(phi_back(1) == Approx(phi(1)).margin(margin));
        REQUIRE(phi_back(2) == Approx(phi(2)).margin(margin));
    }

    SECTION("theta = 2.9")
    {
        liepp::vector3<S> axis;
        axis << S(1) / std::sqrt(S(3)), S(1) / std::sqrt(S(3)), S(1) / std::sqrt(S(3));
        liepp::vector3<S> phi = S(2.9) * axis;
        auto r = liepp::so3<S>::exp(phi);
        auto phi_back = r.log();
        // Compare rotation matrices to handle quaternion double-cover
        auto R1 = liepp::so3<S>::exp(phi_back).matrix();
        auto R2 = r.matrix();
        REQUIRE((R1 - R2).norm() < margin);
    }

    SECTION("theta = 3.1")
    {
        liepp::vector3<S> axis;
        axis << S(0), S(0), S(1);
        liepp::vector3<S> phi = S(3.1) * axis;
        auto r = liepp::so3<S>::exp(phi);
        auto phi_back = r.log();
        auto R1 = liepp::so3<S>::exp(phi_back).matrix();
        auto R2 = r.matrix();
        REQUIRE((R1 - R2).norm() < margin);
    }

    SECTION("theta near pi (pi - 1e-6)")
    {
        S theta = pi - S(1e-6);
        liepp::vector3<S> axis;
        axis << S(1) / std::sqrt(S(3)), S(1) / std::sqrt(S(3)), S(1) / std::sqrt(S(3));
        liepp::vector3<S> phi = theta * axis;
        auto r = liepp::so3<S>::exp(phi);
        auto phi_back = r.log();
        auto R1 = liepp::so3<S>::exp(phi_back).matrix();
        auto R2 = r.matrix();
        S roundtrip_margin = std::is_same_v<S, float> ? S(1e-3) : S(1e-8);
        REQUIRE((R1 - R2).norm() < roundtrip_margin);
    }
}

// ============================================================================
// exp(zero) == identity
// ============================================================================

TEST_CASE("so3: exp(zero) is identity", "[so3]")
{
    auto r = liepp::so3<double>::exp(liepp::vector3<double>::Zero());
    auto m = r.matrix();
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            double expected = (i == j) ? 1.0 : 0.0;
            REQUIRE(m(i, j) == Approx(expected).margin(1e-14));
        }
    }
}

// ============================================================================
// Log produces no NaN at singularities
// ============================================================================

TEST_CASE("so3: log produces no NaN at theta~0", "[so3]")
{
    liepp::vector3<double> phi;
    phi << 1e-15, 0.0, 0.0;
    auto r = liepp::so3<double>::exp(phi);
    auto result = r.log();
    REQUIRE(std::isfinite(result(0)));
    REQUIRE(std::isfinite(result(1)));
    REQUIRE(std::isfinite(result(2)));
}

TEST_CASE("so3: log produces no NaN at theta~pi", "[so3]")
{
    constexpr double pi = std::numbers::pi;
    liepp::vector3<double> axis;
    axis << 1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0);
    liepp::vector3<double> phi = (pi - 1e-12) * axis;
    auto r = liepp::so3<double>::exp(phi);
    auto result = r.log();
    REQUIRE(std::isfinite(result(0)));
    REQUIRE(std::isfinite(result(1)));
    REQUIRE(std::isfinite(result(2)));
}

// ============================================================================
// Compose: matrix product matches
// ============================================================================

TEST_CASE("so3: compose matches matrix product", "[so3]")
{
    liepp::vector3<double> phi_a;
    phi_a << 0.1, 0.5, -0.3;
    liepp::vector3<double> phi_b;
    phi_b << -0.4, 0.2, 0.6;

    auto a = liepp::so3<double>::exp(phi_a);
    auto b = liepp::so3<double>::exp(phi_b);
    auto composed = a * b;

    auto expected = (a.matrix() * b.matrix()).eval();
    auto actual = composed.matrix();
    REQUIRE((actual - expected).norm() < 1e-12);
}

// ============================================================================
// Inverse
// ============================================================================

TEST_CASE("so3: inverse cancels rotation", "[so3]")
{
    liepp::vector3<double> phi;
    phi << 0.7, -0.3, 1.2;
    auto x = liepp::so3<double>::exp(phi);
    auto result = x * x.inverse();
    auto I = liepp::matrix3<double>::Identity();
    REQUIRE((result.matrix() - I).norm() < 1e-12);
}

// ============================================================================
// Adjoint == rotation matrix for SO(3)
// ============================================================================

TEST_CASE("so3: adjoint equals rotation matrix", "[so3]")
{
    liepp::vector3<double> phi;
    phi << 0.5, -0.3, 0.8;
    auto r = liepp::so3<double>::exp(phi);
    REQUIRE((r.adjoint() - r.matrix()).norm() < 1e-12);
}

// ============================================================================
// Coadjoint == rotation matrix for SO(3)
// ============================================================================

TEST_CASE("so3: coadjoint equals rotation matrix", "[so3]")
{
    liepp::vector3<double> phi;
    phi << 0.5, -0.3, 0.8;
    auto r = liepp::so3<double>::exp(phi);
    REQUIRE((r.coadjoint() - r.matrix()).norm() < 1e-12);
}

// ============================================================================
// Left Jacobian: J_l * J_l_inv == I
// ============================================================================

TEST_CASE("so3: left Jacobian inverse identity", "[so3]")
{
    liepp::vector3<double> phi;
    phi << 0.5, -0.8, 1.1;
    auto Jl = liepp::so3<double>::left_jacobian(phi);
    auto Jl_inv = liepp::so3<double>::left_jacobian_inv(phi);
    auto I = liepp::matrix3<double>::Identity();
    REQUIRE((Jl * Jl_inv - I).norm() < 1e-10);
}

// ============================================================================
// Right Jacobian == left(-phi)
// ============================================================================

TEST_CASE("so3: right Jacobian equals left(-phi)", "[so3]")
{
    liepp::vector3<double> phi;
    phi << 0.3, -0.6, 0.9;
    auto Jr = liepp::so3<double>::right_jacobian(phi);
    auto Jl_neg = liepp::so3<double>::left_jacobian(-phi);
    REQUIRE((Jr - Jl_neg).norm() < 1e-12);
}

// ============================================================================
// Left Jacobian at phi~0 approaches identity
// ============================================================================

TEST_CASE("so3: left Jacobian at phi~0 is near identity", "[so3]")
{
    liepp::vector3<double> phi;
    phi << 1e-10, 2e-10, 3e-10;
    auto Jl = liepp::so3<double>::left_jacobian(phi);
    auto I = liepp::matrix3<double>::Identity();
    REQUIRE((Jl - I).norm() < 1e-6);
}

// ============================================================================
// Left Jacobian numerical verification (finite difference)
// ============================================================================

TEST_CASE("so3: left Jacobian numerical verification", "[so3]")
{
    liepp::vector3<double> phi;
    phi << 0.5, -0.3, 0.8;
    auto Jl = liepp::so3<double>::left_jacobian(phi);

    double delta = 1e-7;
    liepp::matrix3<double> Jl_fd;
    for (int i = 0; i < 3; ++i)
    {
        liepp::vector3<double> phi_plus = phi;
        phi_plus(i) += delta;
        auto R_plus = liepp::so3<double>::exp(phi_plus).matrix();
        auto R_base = liepp::so3<double>::exp(phi).matrix();
        // Finite difference in tangent space: log(R_plus * R_base^T)
        liepp::matrix3<double> dR = R_plus * R_base.transpose();
        // For small perturbation, the log of dR gives the tangent vector
        liepp::vector3<double> d_omega;
        d_omega << (dR(2, 1) - dR(1, 2)) / 2.0,
                   (dR(0, 2) - dR(2, 0)) / 2.0,
                   (dR(1, 0) - dR(0, 1)) / 2.0;
        Jl_fd.col(i) = d_omega / delta;
    }

    REQUIRE((Jl - Jl_fd).norm() < 1e-5);
}

// ============================================================================
// from_matrix with valid / invalid
// ============================================================================

TEST_CASE("so3: from_matrix with valid SO(3)", "[so3]")
{
    liepp::vector3<double> phi;
    phi << 0.3, -0.5, 0.7;
    auto r = liepp::so3<double>::exp(phi);
    auto result = liepp::so3<double>::from_matrix(r.matrix());
    REQUIRE(result.has_value());
    REQUIRE((result.value().matrix() - r.matrix()).norm() < 1e-12);
}

TEST_CASE("so3: from_matrix rejects non-orthogonal matrix", "[so3]")
{
    liepp::matrix3<double> bad = liepp::matrix3<double>::Random();
    auto result = liepp::so3<double>::from_matrix(bad);
    REQUIRE_FALSE(result.has_value());
}

// ============================================================================
// from_quaternion with valid / invalid
// ============================================================================

TEST_CASE("so3: from_quaternion with valid quaternion", "[so3]")
{
    liepp::quaternion<double> q(1.0, 0.0, 0.0, 0.0);
    auto result = liepp::so3<double>::from_quaternion(q);
    REQUIRE(result.has_value());
    auto I = liepp::matrix3<double>::Identity();
    REQUIRE((result.value().matrix() - I).norm() < 1e-14);
}

TEST_CASE("so3: from_quaternion rejects non-unit quaternion", "[so3]")
{
    liepp::quaternion<double> q(2.0, 3.0, 4.0, 5.0);
    auto result = liepp::so3<double>::from_quaternion(q);
    REQUIRE_FALSE(result.has_value());
}

// ============================================================================
// Policy behavior
// ============================================================================

TEST_CASE("so3: strict policy normalizes quaternion", "[so3][policy]")
{
    // Non-unit quaternion (2, 0, 0, 0) -> should normalize to (1, 0, 0, 0)
    liepp::quaternion<double> q(2.0, 0.0, 0.0, 0.0);
    liepp::so3<double, liepp::strict_policy> r(q);
    REQUIRE(r.quaternion_ref().squaredNorm() == Approx(1.0).margin(1e-14));
}

TEST_CASE("so3: fast policy does NOT normalize", "[so3][policy]")
{
    liepp::quaternion<double> q(2.0, 0.0, 0.0, 0.0);
    liepp::so3<double, liepp::fast_policy> r(q);
    REQUIRE(r.quaternion_ref().w() == Approx(2.0).margin(1e-14));
}

// ============================================================================
// Mixed-policy compose
// ============================================================================

TEST_CASE("so3: mixed-policy compose returns strict result", "[so3][policy]")
{
    liepp::vector3<double> phi_a;
    phi_a << 0.3, 0.1, -0.2;
    liepp::vector3<double> phi_b;
    phi_b << -0.1, 0.4, 0.2;

    auto strict_r = liepp::so3<double, liepp::strict_policy>::exp(phi_a);
    auto fast_r = liepp::so3<double, liepp::fast_policy>::exp(phi_b);

    auto result = strict_r * fast_r;

    static_assert(std::is_same_v<
        decltype(result),
        liepp::so3<double, liepp::strict_policy>>);

    // Result should be valid
    auto expected = (strict_r.matrix() * fast_r.matrix()).eval();
    REQUIRE((result.matrix() - expected).norm() < 1e-12);
}

// ============================================================================
// Float scalar variant (LIE-12)
// ============================================================================

TEST_CASE("so3: float scalar operations", "[so3][float]")
{
    constexpr float pi = std::numbers::pi_v<float>;
    liepp::vector3<float> phi;
    phi << pi / 4.0f, 0.0f, 0.0f;

    auto r = liepp::so3<float>::exp(phi);
    auto phi_back = r.log();
    REQUIRE(phi_back(0) == Approx(phi(0)).margin(1e-5f));

    auto inv = r.inverse();
    auto product = r * inv;
    auto I = liepp::matrix3<float>::Identity();
    REQUIRE((product.matrix() - I).norm() < 1e-5f);
}

// ============================================================================
// matrix() roundtrips with from_matrix
// ============================================================================

TEST_CASE("so3: matrix roundtrip with from_matrix", "[so3]")
{
    liepp::vector3<double> phi;
    phi << 1.0, -0.5, 0.3;
    auto r = liepp::so3<double>::exp(phi);
    auto R = r.matrix();
    auto result = liepp::so3<double>::from_matrix(R);
    REQUIRE(result.has_value());
    REQUIRE((result.value().matrix() - R).norm() < 1e-12);
}
