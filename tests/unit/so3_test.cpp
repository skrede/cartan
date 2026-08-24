#include <cartan/lie/so3.h>
#include <cartan/lie/hat_vee.h>

#include <cartan/detail/epsilon.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <cmath>
#include <limits>
#include <numbers>
#include <string_view>

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
        cartan::vector3<S> phi;
        phi << S(1e-10), S(2e-10), S(3e-10);
        auto r = cartan::so3<S>::exp(phi);
        auto phi_back = r.log();
        REQUIRE(phi_back(0) == Approx(phi(0)).margin(margin));
        REQUIRE(phi_back(1) == Approx(phi(1)).margin(margin));
        REQUIRE(phi_back(2) == Approx(phi(2)).margin(margin));
    }

    SECTION("theta = pi/4")
    {
        S theta = pi / S(4);
        cartan::vector3<S> phi;
        phi << theta, S(0), S(0);
        auto r = cartan::so3<S>::exp(phi);
        auto phi_back = r.log();
        REQUIRE(phi_back(0) == Approx(phi(0)).margin(margin));
        REQUIRE(phi_back(1) == Approx(phi(1)).margin(margin));
        REQUIRE(phi_back(2) == Approx(phi(2)).margin(margin));
    }

    SECTION("theta = pi/2")
    {
        S theta = pi / S(2);
        cartan::vector3<S> axis;
        axis << S(0), S(1), S(0);
        cartan::vector3<S> phi = theta * axis;
        auto r = cartan::so3<S>::exp(phi);
        auto phi_back = r.log();
        REQUIRE(phi_back(0) == Approx(phi(0)).margin(margin));
        REQUIRE(phi_back(1) == Approx(phi(1)).margin(margin));
        REQUIRE(phi_back(2) == Approx(phi(2)).margin(margin));
    }

    SECTION("theta = 2.9")
    {
        cartan::vector3<S> axis;
        axis << S(1) / std::sqrt(S(3)), S(1) / std::sqrt(S(3)), S(1) / std::sqrt(S(3));
        cartan::vector3<S> phi = S(2.9) * axis;
        auto r = cartan::so3<S>::exp(phi);
        auto phi_back = r.log();
        // Compare rotation matrices to handle quaternion double-cover
        auto R1 = cartan::so3<S>::exp(phi_back).matrix();
        auto R2 = r.matrix();
        REQUIRE((R1 - R2).norm() < margin);
    }

    SECTION("theta = 3.1")
    {
        cartan::vector3<S> axis;
        axis << S(0), S(0), S(1);
        cartan::vector3<S> phi = S(3.1) * axis;
        auto r = cartan::so3<S>::exp(phi);
        auto phi_back = r.log();
        auto R1 = cartan::so3<S>::exp(phi_back).matrix();
        auto R2 = r.matrix();
        REQUIRE((R1 - R2).norm() < margin);
    }

    SECTION("theta near pi (pi - 1e-6)")
    {
        S theta = pi - S(1e-6);
        cartan::vector3<S> axis;
        axis << S(1) / std::sqrt(S(3)), S(1) / std::sqrt(S(3)), S(1) / std::sqrt(S(3));
        cartan::vector3<S> phi = theta * axis;
        auto r = cartan::so3<S>::exp(phi);
        auto phi_back = r.log();
        auto R1 = cartan::so3<S>::exp(phi_back).matrix();
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
    auto r = cartan::so3<double>::exp(cartan::vector3<double>::Zero());
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
    cartan::vector3<double> phi;
    phi << 1e-15, 0.0, 0.0;
    auto r = cartan::so3<double>::exp(phi);
    auto result = r.log();
    REQUIRE(std::isfinite(result(0)));
    REQUIRE(std::isfinite(result(1)));
    REQUIRE(std::isfinite(result(2)));
}

TEST_CASE("so3: log produces no NaN at theta~pi", "[so3]")
{
    constexpr double pi = std::numbers::pi;
    cartan::vector3<double> axis;
    axis << 1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0);
    cartan::vector3<double> phi = (pi - 1e-12) * axis;
    auto r = cartan::so3<double>::exp(phi);
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
    cartan::vector3<double> phi_a;
    phi_a << 0.1, 0.5, -0.3;
    cartan::vector3<double> phi_b;
    phi_b << -0.4, 0.2, 0.6;

    auto a = cartan::so3<double>::exp(phi_a);
    auto b = cartan::so3<double>::exp(phi_b);
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
    cartan::vector3<double> phi;
    phi << 0.7, -0.3, 1.2;
    auto x = cartan::so3<double>::exp(phi);
    auto result = x * x.inverse();
    auto I = cartan::matrix3<double>::Identity();
    REQUIRE((result.matrix() - I).norm() < 1e-12);
}

// ============================================================================
// Adjoint == rotation matrix for SO(3)
// ============================================================================

TEST_CASE("so3: adjoint equals rotation matrix", "[so3]")
{
    cartan::vector3<double> phi;
    phi << 0.5, -0.3, 0.8;
    auto r = cartan::so3<double>::exp(phi);
    REQUIRE((r.adjoint() - r.matrix()).norm() < 1e-12);
}

// ============================================================================
// Coadjoint == rotation matrix for SO(3)
// ============================================================================

TEST_CASE("so3: coadjoint equals rotation matrix", "[so3]")
{
    cartan::vector3<double> phi;
    phi << 0.5, -0.3, 0.8;
    auto r = cartan::so3<double>::exp(phi);
    REQUIRE((r.coadjoint() - r.matrix()).norm() < 1e-12);
}

// ============================================================================
// Left Jacobian: J_l * J_l_inv == I
// ============================================================================

TEST_CASE("so3: left Jacobian inverse identity", "[so3]")
{
    cartan::vector3<double> phi;
    phi << 0.5, -0.8, 1.1;
    auto Jl = cartan::so3<double>::left_jacobian(phi);
    auto Jl_inv = cartan::so3<double>::left_jacobian_inv(phi);
    auto I = cartan::matrix3<double>::Identity();
    REQUIRE((Jl * Jl_inv - I).norm() < 1e-10);
}

// ============================================================================
// Right Jacobian == left(-phi)
// ============================================================================

TEST_CASE("so3: right Jacobian equals left(-phi)", "[so3]")
{
    cartan::vector3<double> phi;
    phi << 0.3, -0.6, 0.9;
    auto Jr = cartan::so3<double>::right_jacobian(phi);
    auto Jl_neg = cartan::so3<double>::left_jacobian(-phi);
    REQUIRE((Jr - Jl_neg).norm() < 1e-12);
}

// ============================================================================
// Left Jacobian at phi~0 approaches identity
// ============================================================================

TEST_CASE("so3: left Jacobian at phi~0 is near identity", "[so3]")
{
    cartan::vector3<double> phi;
    phi << 1e-10, 2e-10, 3e-10;
    auto Jl = cartan::so3<double>::left_jacobian(phi);
    auto I = cartan::matrix3<double>::Identity();
    REQUIRE((Jl - I).norm() < 1e-6);
}

// ============================================================================
// Left Jacobian numerical verification (finite difference)
// ============================================================================

TEST_CASE("so3: left Jacobian numerical verification", "[so3]")
{
    cartan::vector3<double> phi;
    phi << 0.5, -0.3, 0.8;
    auto Jl = cartan::so3<double>::left_jacobian(phi);

    double delta = 1e-7;
    cartan::matrix3<double> Jl_fd;
    for (int i = 0; i < 3; ++i)
    {
        cartan::vector3<double> phi_plus = phi;
        phi_plus(i) += delta;
        auto R_plus = cartan::so3<double>::exp(phi_plus).matrix();
        auto R_base = cartan::so3<double>::exp(phi).matrix();
        // Finite difference in tangent space: log(R_plus * R_base^T)
        cartan::matrix3<double> dR = R_plus * R_base.transpose();
        // For small perturbation, the log of dR gives the tangent vector
        cartan::vector3<double> d_omega;
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
    cartan::vector3<double> phi;
    phi << 0.3, -0.5, 0.7;
    auto r = cartan::so3<double>::exp(phi);
    auto result = cartan::so3<double>::from_matrix(r.matrix());
    REQUIRE(result.has_value());
    REQUIRE(((*result).matrix() - r.matrix()).norm() < 1e-12);
}

TEST_CASE("so3: from_matrix rejects non-orthogonal matrix", "[so3]")
{
    cartan::matrix3<double> bad = cartan::matrix3<double>::Random();
    auto result = cartan::so3<double>::from_matrix(bad);
    REQUIRE_FALSE(result.has_value());
    REQUIRE(result.error() == cartan::lie_failure::non_orthogonal);
}

// ============================================================================
// from_quaternion with valid / invalid
// ============================================================================

TEST_CASE("so3: from_quaternion with valid quaternion", "[so3]")
{
    cartan::quaternion<double> q(1.0, 0.0, 0.0, 0.0);
    auto result = cartan::so3<double>::from_quaternion(q);
    REQUIRE(result.has_value());
    auto I = cartan::matrix3<double>::Identity();
    REQUIRE(((*result).matrix() - I).norm() < 1e-14);
}

TEST_CASE("so3: from_quaternion rejects non-unit quaternion", "[so3]")
{
    cartan::quaternion<double> q(2.0, 3.0, 4.0, 5.0);
    auto result = cartan::so3<double>::from_quaternion(q);
    REQUIRE_FALSE(result.has_value());
    REQUIRE(result.error() == cartan::lie_failure::non_unit_quaternion);
}

// ============================================================================
// Policy behavior
// ============================================================================

TEST_CASE("so3: strict policy normalizes quaternion", "[so3][policy]")
{
    // Non-unit quaternion (2, 0, 0, 0) -> should normalize to (1, 0, 0, 0)
    cartan::quaternion<double> q(2.0, 0.0, 0.0, 0.0);
    cartan::so3<double, cartan::strict_policy> r(q);
    REQUIRE(r.quaternion_ref().squaredNorm() == Approx(1.0).margin(1e-14));
}

TEST_CASE("so3: fast policy does NOT normalize", "[so3][policy]")
{
    cartan::quaternion<double> q(2.0, 0.0, 0.0, 0.0);
    cartan::so3<double, cartan::fast_policy> r(q);
    REQUIRE(r.quaternion_ref().w() == Approx(2.0).margin(1e-14));
}

// ============================================================================
// Mixed-policy compose
// ============================================================================

TEST_CASE("so3: mixed-policy compose returns strict result", "[so3][policy]")
{
    cartan::vector3<double> phi_a;
    phi_a << 0.3, 0.1, -0.2;
    cartan::vector3<double> phi_b;
    phi_b << -0.1, 0.4, 0.2;

    auto strict_r = cartan::so3<double, cartan::strict_policy>::exp(phi_a);
    auto fast_r = cartan::so3<double, cartan::fast_policy>::exp(phi_b);

    auto result = strict_r * fast_r;

    static_assert(std::is_same_v<
        decltype(result),
        cartan::so3<double, cartan::strict_policy>>);

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
    cartan::vector3<float> phi;
    phi << pi / 4.0f, 0.0f, 0.0f;

    auto r = cartan::so3<float>::exp(phi);
    auto phi_back = r.log();
    REQUIRE(phi_back(0) == Approx(phi(0)).margin(1e-5f));

    auto inv = r.inverse();
    auto product = r * inv;
    auto I = cartan::matrix3<float>::Identity();
    REQUIRE((product.matrix() - I).norm() < 1e-5f);
}

// ============================================================================
// matrix() roundtrips with from_matrix
// ============================================================================

TEST_CASE("so3: matrix roundtrip with from_matrix", "[so3]")
{
    cartan::vector3<double> phi;
    phi << 1.0, -0.5, 0.3;
    auto r = cartan::so3<double>::exp(phi);
    auto R = r.matrix();
    auto result = cartan::so3<double>::from_matrix(R);
    REQUIRE(result.has_value());
    REQUIRE(((*result).matrix() - R).norm() < 1e-12);
}

// ============================================================================
// Default constructor is identity
// ============================================================================

TEMPLATE_TEST_CASE("so3: default constructor is identity", "[so3]", double, float)
{
    using S = TestType;
    S tol = std::is_same_v<S, float> ? S(1e-6) : S(1e-12);
    cartan::so3<S> d{};
    REQUIRE(d.isApprox(cartan::so3<S>::identity(), tol));
    REQUIRE((d.matrix() - cartan::matrix3<S>::Identity()).norm() < tol);
}

// ============================================================================
// Manifold-aware isApprox
// ============================================================================

TEMPLATE_TEST_CASE("so3: isApprox reflexive and tolerance-sensitive", "[so3]", double, float)
{
    using S = TestType;
    S tol = std::is_same_v<S, float> ? S(1e-5) : S(1e-10);
    cartan::vector3<S> phi;
    phi << S(0.3), S(-0.5), S(0.7);
    auto r = cartan::so3<S>::exp(phi);
    REQUIRE(r.isApprox(r, tol));

    cartan::vector3<S> phi_near = phi + cartan::vector3<S>::Constant(tol / S(10));
    REQUIRE(r.isApprox(cartan::so3<S>::exp(phi_near), tol));

    cartan::vector3<S> phi_far = phi + cartan::vector3<S>::Constant(S(0.1));
    REQUIRE_FALSE(r.isApprox(cartan::so3<S>::exp(phi_far), tol));
}

TEST_CASE("so3: isApprox is quaternion double-cover safe", "[so3]")
{
    cartan::vector3<double> phi;
    phi << 0.4, -0.2, 0.9;
    auto r = cartan::so3<double>::exp(phi);
    cartan::quaternion<double> q = r.quaternion_ref();
    cartan::quaternion<double> neg(-q.w(), -q.x(), -q.y(), -q.z());
    cartan::so3<double> r_neg(neg);
    // q and -q are the same rotation: manifold-aware isApprox must treat them as equal
    REQUIRE(r.isApprox(r_neg, 1e-12));
}

// ============================================================================
// Nonfinite rejection
// ============================================================================

/// The orthogonality and determinant tests as they read without a finiteness
/// guard in front of them. The comparisons are negated exactly as the factory
/// spells them: `!(deviation > tol)` is true for a NaN where `deviation <= tol`
/// is false, so the two are not interchangeable here.
template <typename S>
static bool unguarded_matrix_gate_admits(const cartan::matrix3<S>& R)
{
    const S tol = cartan::detail::sqrt_epsilon_v<S>;
    const cartan::matrix3<S> RtR = R.transpose() * R;
    return !((RtR - cartan::matrix3<S>::Identity()).norm() > tol)
        && !(std::abs(R.determinant() - S(1)) > tol);
}

template <typename S>
static bool unguarded_quaternion_gate_admits(const cartan::quaternion<S>& q)
{
    return !(std::abs(q.squaredNorm() - S(1)) > cartan::detail::sqrt_epsilon_v<S>);
}

/// The chain above admits every entry and every value class but one -- an
/// infinity at (0,0) -- which is why a finiteness test applied to the result
/// cannot stand in for one applied to the input. Everywhere else a 0 * inf
/// product, inside R^T * R or inside the cofactor expansion of det(R), turns the
/// infinity into a NaN before it is compared, and `deviation > tol` is false for
/// a NaN. At (0,0) the infinity multiplies a finite minor and reaches the
/// determinant intact, so that entry alone is refused -- as an improper rotation.
template <typename S>
static bool unguarded_admits_entry(int i, int j, S poison)
{
    return std::isnan(poison) || i != 0 || j != 0;
}

template <typename S>
static void expect_entry_rejected(int i, int j, S poison)
{
    cartan::matrix3<S> R = cartan::matrix3<S>::Identity();
    R(i, j) = poison;

    auto result = cartan::so3<S>::from_matrix(R);
    REQUIRE_FALSE(result.has_value());
    REQUIRE(result.error() == cartan::lie_failure::non_finite_input);
    REQUIRE(unguarded_matrix_gate_admits<S>(R) == unguarded_admits_entry<S>(i, j, poison));
}

template <typename S>
static void expect_diagonal_rejected(S poison)
{
    for (int i = 0; i < 3; ++i)
    {
        expect_entry_rejected<S>(i, i, poison);
    }
}

template <typename S>
static void expect_offdiagonal_rejected(S poison)
{
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            if (i != j)
            {
                expect_entry_rejected<S>(i, j, poison);
            }
        }
    }
}

template <typename S>
static void expect_coefficients_rejected(S poison, bool unguarded_admits)
{
    for (int c = 0; c < 4; ++c)
    {
        cartan::quaternion<S> q(S(1), S(0), S(0), S(0));
        q.coeffs()(c) = poison;

        auto result = cartan::so3<S>::from_quaternion(q);
        REQUIRE_FALSE(result.has_value());
        REQUIRE(result.error() == cartan::lie_failure::non_finite_input);
        REQUIRE(unguarded_quaternion_gate_admits<S>(q) == unguarded_admits);
    }
}

TEMPLATE_TEST_CASE("so3: from_matrix rejects nonfinite entries on the diagonal",
    "[so3][nonfinite]", double, float)
{
    using S = TestType;
    using lim = std::numeric_limits<S>;

    expect_diagonal_rejected<S>(lim::quiet_NaN());
    expect_diagonal_rejected<S>(-lim::quiet_NaN());
    expect_diagonal_rejected<S>(lim::infinity());
    expect_diagonal_rejected<S>(-lim::infinity());
}

TEMPLATE_TEST_CASE("so3: from_matrix rejects nonfinite entries off the diagonal",
    "[so3][nonfinite]", double, float)
{
    using S = TestType;
    using lim = std::numeric_limits<S>;

    expect_offdiagonal_rejected<S>(lim::quiet_NaN());
    expect_offdiagonal_rejected<S>(-lim::quiet_NaN());
    expect_offdiagonal_rejected<S>(lim::infinity());
    expect_offdiagonal_rejected<S>(-lim::infinity());
}

TEMPLATE_TEST_CASE("so3: from_quaternion rejects nonfinite coefficients",
    "[so3][nonfinite]", double, float)
{
    using S = TestType;
    using lim = std::numeric_limits<S>;

    expect_coefficients_rejected<S>(lim::quiet_NaN(), true);
    expect_coefficients_rejected<S>(-lim::quiet_NaN(), true);
    expect_coefficients_rejected<S>(lim::infinity(), false);
    expect_coefficients_rejected<S>(-lim::infinity(), false);
}

/// Ties the two predicates above to the factories they stand in for. On finite
/// input each pair must agree exactly, so widening a tolerance or replacing a
/// comparison in so3::from_matrix or so3::from_quaternion breaks these cases --
/// which the nonfinite corpus above cannot do, since both of its sides are
/// written here. The reflection is the only shape that reaches the determinant
/// test: scaling a rotation trips orthogonality first, so that branch cannot be
/// straddled marginally while orthogonality still passes.
TEMPLATE_TEST_CASE("so3: the unguarded chains agree with the factories on finite input",
    "[so3][nonfinite]", double, float)
{
    using S = TestType;
    const S tol = cartan::detail::sqrt_epsilon_v<S>;

    cartan::vector3<S> phi;
    phi << S(0.3), S(-0.5), S(0.7);
    const cartan::matrix3<S> exact = cartan::so3<S>::exp(phi).matrix();

    cartan::matrix3<S> marginal = exact;
    marginal(0, 0) += S(2) * tol;

    cartan::matrix3<S> reflection = cartan::matrix3<S>::Identity();
    reflection(2, 2) = S(-1);

    const cartan::matrix3<S> gross = cartan::matrix3<S>::Identity() * S(2);

    for (const cartan::matrix3<S>& R : {exact, marginal, reflection, gross})
    {
        REQUIRE(unguarded_matrix_gate_admits<S>(R)
            == cartan::so3<S>::from_matrix(R).has_value());
    }

    const cartan::quaternion<S> unit = cartan::so3<S>::exp(phi).quaternion_ref();
    const cartan::quaternion<S> marginal_q(
        unit.w() * (S(1) + tol), unit.x(), unit.y(), unit.z());
    const cartan::quaternion<S> gross_q(S(2), S(3), S(4), S(5));

    for (const cartan::quaternion<S>& q : {unit, marginal_q, gross_q})
    {
        REQUIRE(unguarded_quaternion_gate_admits<S>(q)
            == cartan::so3<S>::from_quaternion(q).has_value());
    }
}

TEST_CASE("so3: lie_failure carries a diagnostic for nonfinite input", "[so3][nonfinite]")
{
    REQUIRE_FALSE(std::string_view(
        cartan::message(cartan::lie_failure::non_finite_input)).empty());
}
