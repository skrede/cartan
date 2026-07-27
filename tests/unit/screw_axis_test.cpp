#include <cartan/serial/chain/screw_axis.h>

#include <cartan/detail/epsilon.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <cmath>
#include <limits>

using Catch::Approx;

// ============================================================================
// screw_axis revolute factory
// ============================================================================

TEST_CASE("screw_axis revolute factory", "[screw_axis]")
{
    SECTION("axis at origin: v = 0")
    {
        auto sa = cartan::screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
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
        auto sa = cartan::screw_axis<double>::revolute({0, 0, 1}, {L, 0, 0});
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
        auto sa = cartan::screw_axis<double>::revolute({0, 0, 2}, {0, 0, 0});
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
        auto sa = cartan::screw_axis<double>::prismatic({1, 0, 0});
        REQUIRE(sa.omega().norm() < 1e-12);
        REQUIRE(sa.v()(0) == Approx(1.0).margin(1e-12));
        REQUIRE(sa.v()(1) == Approx(0.0).margin(1e-12));
        REQUIRE(sa.v()(2) == Approx(0.0).margin(1e-12));
        REQUIRE(sa.is_prismatic());
        REQUIRE_FALSE(sa.is_revolute());
    }

    SECTION("normalizes non-unit direction")
    {
        auto sa = cartan::screw_axis<double>::prismatic({2, 0, 0});
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
        cartan::vector6<double> vec;
        vec << 0, 0, 1, 0, -0.5, 0;
        auto result = cartan::screw_axis<double>::from_vector(vec);
        REQUIRE(result.has_value());
        REQUIRE(result->omega()(2) == Approx(1.0).margin(1e-12));
    }

    SECTION("non-unit omega fails")
    {
        cartan::vector6<double> vec;
        vec << 0, 0, 2, 0, 0, 0;
        auto result = cartan::screw_axis<double>::from_vector(vec);
        REQUIRE_FALSE(result.has_value());
    }

    SECTION("zero omega with unit v passes")
    {
        cartan::vector6<double> vec;
        vec << 0, 0, 0, 1, 0, 0;
        auto result = cartan::screw_axis<double>::from_vector(vec);
        REQUIRE(result.has_value());
        REQUIRE(result->is_prismatic());
    }

    SECTION("zero omega with non-unit v fails")
    {
        cartan::vector6<double> vec;
        vec << 0, 0, 0, 2, 0, 0;
        auto result = cartan::screw_axis<double>::from_vector(vec);
        REQUIRE_FALSE(result.has_value());
    }
}

// ============================================================================
// screw_axis to_vector roundtrip
// ============================================================================

TEST_CASE("screw_axis to_vector roundtrip", "[screw_axis]")
{
    auto sa = cartan::screw_axis<double>::revolute({0, 0, 1}, {0.5, 0, 0});
    auto vec = sa.to_vector();
    auto result = cartan::screw_axis<double>::from_vector(vec);
    REQUIRE(result.has_value());
    REQUIRE((result->omega() - sa.omega()).norm() < 1e-12);
    REQUIRE((result->v() - sa.v()).norm() < 1e-12);
}

// ============================================================================
// screw_axis float scalar
// ============================================================================

TEST_CASE("screw_axis float scalar", "[screw_axis][float]")
{
    auto sa = cartan::screw_axis<float>::revolute({0, 0, 1}, {0, 0, 0});
    REQUIRE(sa.omega()(2) == Approx(1.0f).margin(1e-6f));
    REQUIRE(sa.is_revolute());

    auto sp = cartan::screw_axis<float>::prismatic({0, 1, 0});
    REQUIRE(sp.v()(1) == Approx(1.0f).margin(1e-6f));
    REQUIRE(sp.is_prismatic());
}

// ============================================================================
// from_vector nonfinite rejection
// ============================================================================

/// The unit-constraint chain as it reads without a finiteness guard in front of
/// it, with each comparison negated exactly as the factory spells it. The branch
/// selection is the interesting part: `omega_norm > tol` is false for a NaN, so a
/// nonfinite rotation axis fell through to the prismatic branch and was accepted
/// there whenever the linear part happened to be unit.
template <typename S>
static bool unguarded_gate_admits(const cartan::vector6<S>& s)
{
    const S tol = cartan::detail::sqrt_epsilon_v<S>;
    const S omega_norm = s.template head<3>().norm();
    if (omega_norm > tol)
    {
        return !(std::abs(omega_norm - S(1)) > tol);
    }
    return !(std::abs(s.template tail<3>().norm() - S(1)) > tol);
}

template <typename S>
static void expect_rejected(const cartan::vector6<S>& s, bool unguarded_admits)
{
    auto result = cartan::screw_axis<S>::from_vector(s);
    REQUIRE_FALSE(result.has_value());
    REQUIRE(result.error() == cartan::lie_failure::non_finite_input);
    REQUIRE(unguarded_gate_admits<S>(s) == unguarded_admits);
}

/// Poisons the rotation axis while the linear part stays unit, which is what
/// made the prismatic branch accept the value rather than merely reach it.
template <typename S>
static void expect_angular_rejected(S poison, bool unguarded_admits)
{
    for (int i = 0; i < 3; ++i)
    {
        cartan::vector6<S> s;
        s << S(0), S(0), S(1), S(1), S(0), S(0);
        s(i) = poison;
        expect_rejected<S>(s, unguarded_admits);
    }
}

template <typename S>
static void expect_revolute_linear_rejected(S poison, bool unguarded_admits)
{
    for (int i = 3; i < 6; ++i)
    {
        cartan::vector6<S> s;
        s << S(0), S(0), S(1), S(0), S(0), S(0);
        s(i) = poison;
        expect_rejected<S>(s, unguarded_admits);
    }
}

template <typename S>
static void expect_prismatic_linear_rejected(S poison, bool unguarded_admits)
{
    for (int i = 3; i < 6; ++i)
    {
        cartan::vector6<S> s;
        s << S(0), S(0), S(0), S(0), S(0), S(1);
        s(i) = poison;
        expect_rejected<S>(s, unguarded_admits);
    }
}

TEMPLATE_TEST_CASE("screw_axis: from_vector rejects a nonfinite angular component",
    "[screw_axis][nonfinite]", double, float)
{
    using S = TestType;
    using lim = std::numeric_limits<S>;

    expect_angular_rejected<S>(lim::quiet_NaN(), true);
    expect_angular_rejected<S>(-lim::quiet_NaN(), true);
    expect_angular_rejected<S>(lim::infinity(), false);
    expect_angular_rejected<S>(-lim::infinity(), false);
}

TEMPLATE_TEST_CASE("screw_axis: from_vector rejects a nonfinite linear component",
    "[screw_axis][nonfinite]", double, float)
{
    using S = TestType;
    using lim = std::numeric_limits<S>;

    expect_revolute_linear_rejected<S>(lim::quiet_NaN(), true);
    expect_revolute_linear_rejected<S>(-lim::quiet_NaN(), true);
    expect_revolute_linear_rejected<S>(lim::infinity(), true);
    expect_revolute_linear_rejected<S>(-lim::infinity(), true);

    expect_prismatic_linear_rejected<S>(lim::quiet_NaN(), true);
    expect_prismatic_linear_rejected<S>(-lim::quiet_NaN(), true);
    expect_prismatic_linear_rejected<S>(lim::infinity(), false);
    expect_prismatic_linear_rejected<S>(-lim::infinity(), false);
}

TEMPLATE_TEST_CASE("screw_axis: a finite unit axis still constructs unchanged",
    "[screw_axis][nonfinite]", double, float)
{
    using S = TestType;
    cartan::vector6<S> s;
    s << S(0), S(0), S(1), S(0), S(-0.5), S(0);

    auto result = cartan::screw_axis<S>::from_vector(s);
    REQUIRE(result.has_value());
    REQUIRE(result->is_revolute());
    // from_vector copies the components, so the round trip is bit-exact.
    REQUIRE((result->to_vector() - s).norm() == S(0));
}

/// Ties the predicate above to the factory it stands in for. On finite input the
/// two must agree exactly, so widening a tolerance, or moving the branch
/// selection, breaks this case -- which the nonfinite corpus above cannot do,
/// since both of its sides are written here. The corpus covers both branches and
/// straddles the unit tolerance on each.
TEMPLATE_TEST_CASE("screw_axis: the unguarded chain agrees with from_vector on finite input",
    "[screw_axis][nonfinite]", double, float)
{
    using S = TestType;
    const S tol = cartan::detail::sqrt_epsilon_v<S>;

    cartan::vector6<S> revolute_unit;
    revolute_unit << S(0), S(0), S(1), S(0), S(-0.5), S(0);

    cartan::vector6<S> revolute_marginal = revolute_unit;
    revolute_marginal(2) = S(1) + S(2) * tol;

    cartan::vector6<S> revolute_gross = revolute_unit;
    revolute_gross(2) = S(2);

    cartan::vector6<S> prismatic_unit;
    prismatic_unit << S(0), S(0), S(0), S(1), S(0), S(0);

    cartan::vector6<S> prismatic_marginal = prismatic_unit;
    prismatic_marginal(3) = S(1) + S(2) * tol;

    cartan::vector6<S> prismatic_gross = prismatic_unit;
    prismatic_gross(3) = S(2);

    for (const cartan::vector6<S>& s : {revolute_unit, revolute_marginal, revolute_gross,
             prismatic_unit, prismatic_marginal, prismatic_gross})
    {
        REQUIRE(unguarded_gate_admits<S>(s)
            == cartan::screw_axis<S>::from_vector(s).has_value());
    }
}

/// revolute() and prismatic() normalize without validating, and keep that
/// spelling deliberately: it is the construction form the chain fixtures and the
/// description loaders use. A nonfinite axis therefore reaches the stored omega,
/// and is_revolute()'s comparison is false for a NaN, so the joint is reported as
/// prismatic. Validating a whole chain of axes at once is the only place that can
/// be caught without changing this spelling.
TEMPLATE_TEST_CASE("screw_axis: the unvalidated factories still admit a nonfinite axis",
    "[screw_axis][nonfinite]", double, float)
{
    using S = TestType;
    using lim = std::numeric_limits<S>;

    auto sa = cartan::screw_axis<S>::revolute(
        {lim::quiet_NaN(), S(0), S(1)}, {S(0), S(0), S(0)});
    REQUIRE_FALSE(sa.omega().allFinite());
    REQUIRE_FALSE(sa.is_revolute());

    auto sp = cartan::screw_axis<S>::prismatic({lim::infinity(), S(0), S(0)});
    REQUIRE_FALSE(sp.v().allFinite());
}
