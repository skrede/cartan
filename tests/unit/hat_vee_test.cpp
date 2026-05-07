#include <cartan/lie/hat_vee.h>

#include <cartan/lie/fwd.h>
#include <cartan/lie/policy.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using Catch::Approx;

// ============================================================================
// hat/vee for R^3 <-> so(3) (3x3 skew-symmetric)
// ============================================================================

TEST_CASE("hat: vector3 produces correct skew-symmetric matrix", "[hat_vee]")
{
    cartan::vector3<double> v;
    v << 1.0, 2.0, 3.0;

    auto S = cartan::hat(v);

    // Expected: [[0, -3, 2], [3, 0, -1], [-2, 1, 0]]
    REQUIRE(S(0, 0) == Approx(0.0));
    REQUIRE(S(0, 1) == Approx(-3.0));
    REQUIRE(S(0, 2) == Approx(2.0));
    REQUIRE(S(1, 0) == Approx(3.0));
    REQUIRE(S(1, 1) == Approx(0.0));
    REQUIRE(S(1, 2) == Approx(-1.0));
    REQUIRE(S(2, 0) == Approx(-2.0));
    REQUIRE(S(2, 1) == Approx(1.0));
    REQUIRE(S(2, 2) == Approx(0.0));
}

TEST_CASE("hat: skew-symmetry property S == -S^T", "[hat_vee]")
{
    cartan::vector3<double> v;
    v << 4.5, -1.2, 0.7;

    auto S = cartan::hat(v);
    cartan::matrix3<double> neg_St = -S.transpose();

    REQUIRE((S - neg_St).norm() == Approx(0.0).margin(1e-15));
}

TEST_CASE("hat: hat(v) * w == v.cross(w)", "[hat_vee]")
{
    cartan::vector3<double> v;
    v << 1.0, 2.0, 3.0;
    cartan::vector3<double> w;
    w << 4.0, 5.0, 6.0;

    auto result = (cartan::hat(v) * w).eval();
    auto expected = v.cross(w);

    REQUIRE((result - expected).norm() == Approx(0.0).margin(1e-15));
}

TEST_CASE("vee: vee(hat(v)) == v roundtrip for vector3", "[hat_vee]")
{
    cartan::vector3<double> v;
    v << -2.5, 0.3, 7.1;

    auto roundtrip = cartan::vee(cartan::hat(v));
    REQUIRE((roundtrip - v).norm() == Approx(0.0).margin(1e-15));
}

// ============================================================================
// hat/vee for R^6 <-> se(3) (4x4 twist matrix)
// ============================================================================

TEST_CASE("hat: vector6 produces correct 4x4 se(3) matrix (omega-first)", "[hat_vee]")
{
    // V = (omega, v) = (omega_x, omega_y, omega_z, v_x, v_y, v_z)
    cartan::vector6<double> V;
    V << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

    auto M = cartan::hat(V);

    // Top-left 3x3 = hat(omega=[1,2,3])
    REQUIRE(M(0, 0) == Approx(0.0));
    REQUIRE(M(0, 1) == Approx(-3.0));
    REQUIRE(M(0, 2) == Approx(2.0));
    REQUIRE(M(1, 0) == Approx(3.0));
    REQUIRE(M(1, 1) == Approx(0.0));
    REQUIRE(M(1, 2) == Approx(-1.0));
    REQUIRE(M(2, 0) == Approx(-2.0));
    REQUIRE(M(2, 1) == Approx(1.0));
    REQUIRE(M(2, 2) == Approx(0.0));

    // Top-right 3x1 = v = [4, 5, 6]
    REQUIRE(M(0, 3) == Approx(4.0));
    REQUIRE(M(1, 3) == Approx(5.0));
    REQUIRE(M(2, 3) == Approx(6.0));

    // Bottom row = [0, 0, 0, 0]
    REQUIRE(M(3, 0) == Approx(0.0));
    REQUIRE(M(3, 1) == Approx(0.0));
    REQUIRE(M(3, 2) == Approx(0.0));
    REQUIRE(M(3, 3) == Approx(0.0));
}

TEST_CASE("vee: vee(hat(V)) == V roundtrip for vector6", "[hat_vee]")
{
    cartan::vector6<double> V;
    V << -1.0, 0.5, 2.3, 4.1, -0.7, 3.3;

    auto roundtrip = cartan::vee(cartan::hat(V));
    REQUIRE((roundtrip - V).norm() == Approx(0.0).margin(1e-15));
}

// ============================================================================
// Float scalar support (LIE-12)
// ============================================================================

TEST_CASE("hat/vee: float scalar roundtrip for vector3", "[hat_vee][float]")
{
    cartan::vector3<float> v;
    v << 1.0f, -2.0f, 3.0f;

    auto roundtrip = cartan::vee(cartan::hat(v));
    REQUIRE((roundtrip - v).norm() == Approx(0.0f).margin(1e-6f));
}

TEST_CASE("hat/vee: float scalar roundtrip for vector6", "[hat_vee][float]")
{
    cartan::vector6<float> V;
    V << 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f;

    auto roundtrip = cartan::vee(cartan::hat(V));
    REQUIRE((roundtrip - V).norm() == Approx(0.0f).margin(1e-6f));
}

// ============================================================================
// Policy infrastructure
// ============================================================================

TEST_CASE("policy: strict_policy has normalize_on_construct=true", "[policy]")
{
    STATIC_REQUIRE(cartan::strict_policy::normalize_on_construct == true);
    STATIC_REQUIRE(cartan::strict_policy::assert_valid == true);
}

TEST_CASE("policy: fast_policy has normalize_on_construct=false", "[policy]")
{
    STATIC_REQUIRE(cartan::fast_policy::normalize_on_construct == false);
    STATIC_REQUIRE(cartan::fast_policy::assert_valid == false);
}

TEST_CASE("policy: stricter_policy selects strict if either is strict", "[policy]")
{
    using ss = cartan::stricter_policy<cartan::strict_policy, cartan::strict_policy>;
    using sf = cartan::stricter_policy<cartan::strict_policy, cartan::fast_policy>;
    using fs = cartan::stricter_policy<cartan::fast_policy, cartan::strict_policy>;
    using ff = cartan::stricter_policy<cartan::fast_policy, cartan::fast_policy>;

    STATIC_REQUIRE(std::is_same_v<ss, cartan::strict_policy>);
    STATIC_REQUIRE(std::is_same_v<sf, cartan::strict_policy>);
    STATIC_REQUIRE(std::is_same_v<fs, cartan::strict_policy>);
    STATIC_REQUIRE(std::is_same_v<ff, cartan::fast_policy>);
}

TEST_CASE("policy: lie_group_policy concept accepts valid policies", "[policy]")
{
    STATIC_REQUIRE(cartan::lie_group_policy<cartan::strict_policy>);
    STATIC_REQUIRE(cartan::lie_group_policy<cartan::fast_policy>);
}
