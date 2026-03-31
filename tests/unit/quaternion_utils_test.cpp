#include <liepp/lie/quaternion_utils.h>

#include <liepp/lie/axis_angle.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <cmath>
#include <numbers>

using Catch::Approx;

// ============================================================================
// quat_slerp
// ============================================================================

TEMPLATE_TEST_CASE("quat_slerp: t=0 returns q1", "[quaternion_utils]", double, float)
{
    using S = TestType;
    S margin = std::is_same_v<S, float> ? S(1e-5) : S(1e-12);

    liepp::quaternion<S> q1(std::cos(S(0.3)), std::sin(S(0.3)), S(0), S(0));
    liepp::quaternion<S> q2(std::cos(S(0.8)), S(0), std::sin(S(0.8)), S(0));

    auto result = liepp::quat_slerp(q1, q2, S(0));
    REQUIRE(std::abs(result.w() - q1.w()) < margin);
    REQUIRE(std::abs(result.x() - q1.x()) < margin);
    REQUIRE(std::abs(result.y() - q1.y()) < margin);
    REQUIRE(std::abs(result.z() - q1.z()) < margin);
}

TEMPLATE_TEST_CASE("quat_slerp: t=1 returns q2", "[quaternion_utils]", double, float)
{
    using S = TestType;
    S margin = std::is_same_v<S, float> ? S(1e-5) : S(1e-12);

    liepp::quaternion<S> q1(std::cos(S(0.3)), std::sin(S(0.3)), S(0), S(0));
    liepp::quaternion<S> q2(std::cos(S(0.8)), S(0), std::sin(S(0.8)), S(0));

    auto result = liepp::quat_slerp(q1, q2, S(1));
    REQUIRE(std::abs(result.w() - q2.w()) < margin);
    REQUIRE(std::abs(result.x() - q2.x()) < margin);
    REQUIRE(std::abs(result.y() - q2.y()) < margin);
    REQUIRE(std::abs(result.z() - q2.z()) < margin);
}

TEST_CASE("quat_slerp: same quaternion returns itself", "[quaternion_utils]")
{
    liepp::quaternion<double> q(std::cos(0.5), std::sin(0.5), 0.0, 0.0);
    auto result = liepp::quat_slerp(q, q, 0.5);
    REQUIRE(std::abs(result.w() - q.w()) < 1e-12);
    REQUIRE(std::abs(result.x() - q.x()) < 1e-12);
    REQUIRE(std::abs(result.y() - q.y()) < 1e-12);
    REQUIRE(std::abs(result.z() - q.z()) < 1e-12);
}

TEST_CASE("quat_slerp: antipodal quaternions", "[quaternion_utils]")
{
    // q and -q represent the same rotation; slerp should handle this
    liepp::quaternion<double> q1(1.0, 0.0, 0.0, 0.0);
    liepp::quaternion<double> q2(-1.0, 0.0, 0.0, 0.0); // same rotation as q1

    auto result = liepp::quat_slerp(q1, q2, 0.5);
    // Result should still represent identity rotation
    auto R = result.toRotationMatrix();
    auto I = liepp::matrix3<double>::Identity();
    REQUIRE((R - I).norm() < 1e-10);
}

TEST_CASE("quat_slerp: midpoint is geometrically between", "[quaternion_utils]")
{
    liepp::quaternion<double> q1(1.0, 0.0, 0.0, 0.0); // identity
    // 90 degree rotation about z
    double angle = std::numbers::pi / 2.0;
    liepp::quaternion<double> q2(std::cos(angle / 2), 0.0, 0.0, std::sin(angle / 2));

    auto mid = liepp::quat_slerp(q1, q2, 0.5);
    // Midpoint should be ~45 degree rotation about z
    double expected_angle = angle / 2.0;
    liepp::quaternion<double> expected(
        std::cos(expected_angle / 2), 0.0, 0.0, std::sin(expected_angle / 2));

    REQUIRE(std::abs(mid.w() - expected.w()) < 1e-10);
    REQUIRE(std::abs(mid.z() - expected.z()) < 1e-10);
}

// ============================================================================
// quat_normalize
// ============================================================================

TEST_CASE("quat_normalize: non-unit quaternion becomes unit", "[quaternion_utils]")
{
    liepp::quaternion<double> q(2.0, 3.0, 4.0, 5.0);
    auto result = liepp::quat_normalize(q);
    REQUIRE(result.norm() == Approx(1.0).margin(1e-14));
}

// ============================================================================
// quat_to_matrix / matrix_to_quat roundtrip
// ============================================================================

TEMPLATE_TEST_CASE("quat_to_matrix/matrix_to_quat roundtrip", "[quaternion_utils]", double, float)
{
    using S = TestType;
    S margin = std::is_same_v<S, float> ? S(1e-5) : S(1e-12);

    liepp::quaternion<S> q(std::cos(S(0.7)), std::sin(S(0.7)) / std::sqrt(S(3)),
                               std::sin(S(0.7)) / std::sqrt(S(3)),
                               std::sin(S(0.7)) / std::sqrt(S(3)));
    q.normalize();

    auto R = liepp::quat_to_matrix(q);
    auto q_back = liepp::matrix_to_quat(R);

    // Quaternion double-cover: q and -q are the same rotation
    S dot = std::abs(q.w() * q_back.w() + q.x() * q_back.x() +
                     q.y() * q_back.y() + q.z() * q_back.z());
    REQUIRE(dot == Approx(S(1)).margin(margin));
}

// ============================================================================
// from_wxyz / from_xyzw
// ============================================================================

TEST_CASE("from_wxyz: identity quaternion (1,0,0,0)", "[quaternion_utils]")
{
    auto q = liepp::from_wxyz(1.0, 0.0, 0.0, 0.0);
    REQUIRE(q.w() == Approx(1.0));
    REQUIRE(q.x() == Approx(0.0).margin(1e-14));
    REQUIRE(q.y() == Approx(0.0).margin(1e-14));
    REQUIRE(q.z() == Approx(0.0).margin(1e-14));
}

TEST_CASE("from_xyzw: identity quaternion (0,0,0,1)", "[quaternion_utils]")
{
    auto q = liepp::from_xyzw(0.0, 0.0, 0.0, 1.0);
    REQUIRE(q.w() == Approx(1.0));
    REQUIRE(q.x() == Approx(0.0).margin(1e-14));
    REQUIRE(q.y() == Approx(0.0).margin(1e-14));
    REQUIRE(q.z() == Approx(0.0).margin(1e-14));
}

TEST_CASE("from_wxyz and from_xyzw produce same result", "[quaternion_utils]")
{
    auto q1 = liepp::from_wxyz(0.5, 0.5, 0.5, 0.5);
    auto q2 = liepp::from_xyzw(0.5, 0.5, 0.5, 0.5);
    REQUIRE(q1.w() == Approx(q2.w()));
    REQUIRE(q1.x() == Approx(q2.x()));
    REQUIRE(q1.y() == Approx(q2.y()));
    REQUIRE(q1.z() == Approx(q2.z()));
}

// ============================================================================
// to_wxyz serialization
// ============================================================================

TEST_CASE("to_wxyz serialization", "[quaternion_utils]")
{
    liepp::quaternion<double> q(0.5, 0.1, 0.2, 0.3);
    auto v = liepp::to_wxyz(q);
    REQUIRE(v(0) == Approx(0.5));
    REQUIRE(v(1) == Approx(0.1));
    REQUIRE(v(2) == Approx(0.2));
    REQUIRE(v(3) == Approx(0.3));
}

// ============================================================================
// quat_hamilton_product
// ============================================================================

TEST_CASE("quat_hamilton_product: matches Eigen operator*", "[quaternion_utils]")
{
    liepp::quaternion<double> q1(0.5, 0.5, 0.5, 0.5);
    liepp::quaternion<double> q2(1.0, 0.0, 0.0, 0.0);

    auto product = liepp::quat_hamilton_product(q1, q2);
    auto expected = q1 * q2;
    REQUIRE(std::abs(product.w() - expected.w()) < 1e-14);
    REQUIRE(std::abs(product.x() - expected.x()) < 1e-14);
    REQUIRE(std::abs(product.y() - expected.y()) < 1e-14);
    REQUIRE(std::abs(product.z() - expected.z()) < 1e-14);
}

// ============================================================================
// axis_angle to/from SO(3) roundtrip
// ============================================================================

TEMPLATE_TEST_CASE("axis_angle: to/from SO(3) roundtrip", "[axis_angle]", double, float)
{
    using S = TestType;
    S margin = std::is_same_v<S, float> ? S(1e-4) : S(1e-10);

    SECTION("90 degrees about z")
    {
        liepp::vector3<S> axis;
        axis << S(0), S(0), S(1);
        S angle = std::numbers::pi_v<S> / S(2);

        auto r = liepp::from_axis_angle(liepp::axis_angle<S>{axis, angle});
        auto aa = liepp::to_axis_angle(r);

        REQUIRE(aa.angle == Approx(angle).margin(margin));
        REQUIRE(std::abs(aa.axis(2)) == Approx(S(1)).margin(margin));
    }

    SECTION("arbitrary axis and angle")
    {
        liepp::vector3<S> axis;
        axis << S(1) / std::sqrt(S(3)), S(1) / std::sqrt(S(3)), S(1) / std::sqrt(S(3));
        S angle = S(1.5);

        auto r = liepp::from_axis_angle(liepp::axis_angle<S>{axis, angle});
        auto aa = liepp::to_axis_angle(r);

        REQUIRE(aa.angle == Approx(angle).margin(margin));
        // Axes should match (or be negated if angle was remapped)
        REQUIRE(std::abs(aa.axis.dot(axis)) == Approx(S(1)).margin(margin));
    }
}

TEST_CASE("axis_angle: theta near zero", "[axis_angle]")
{
    liepp::vector3<double> phi;
    phi << 1e-14, 0.0, 0.0;
    auto r = liepp::so3<double>::exp(phi);
    auto aa = liepp::to_axis_angle(r);

    REQUIRE(aa.angle == Approx(0.0).margin(1e-10));
    // Axis should be finite (not NaN)
    REQUIRE(std::isfinite(aa.axis(0)));
    REQUIRE(std::isfinite(aa.axis(1)));
    REQUIRE(std::isfinite(aa.axis(2)));
}

TEST_CASE("axis_angle: theta near pi", "[axis_angle]")
{
    constexpr double pi = std::numbers::pi;
    liepp::vector3<double> axis;
    axis << 1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0), 1.0 / std::sqrt(3.0);
    double angle = pi - 1e-6;

    auto r = liepp::from_axis_angle(liepp::axis_angle<double>{axis, angle});
    auto aa = liepp::to_axis_angle(r);

    // Rotation matrices should match even if angle/axis are remapped
    auto R1 = liepp::from_axis_angle(aa).matrix();
    auto R2 = r.matrix();
    REQUIRE((R1 - R2).norm() < 1e-8);
}

TEST_CASE("axis_angle: from_angle_axis_vector", "[axis_angle]")
{
    liepp::vector3<double> phi;
    phi << 0.3, -0.5, 0.7;
    auto aa = liepp::from_angle_axis_vector(phi);

    REQUIRE(aa.angle == Approx(phi.norm()).margin(1e-12));
    REQUIRE(std::abs(aa.axis.dot(phi.normalized())) == Approx(1.0).margin(1e-12));
}

// ============================================================================
// screw_axis
// ============================================================================

TEST_CASE("screw_axis: pure rotation twist", "[axis_angle]")
{
    // omega = unit z, v = omega x point on axis
    liepp::vector3<double> omega;
    omega << 0.0, 0.0, 1.0;
    liepp::vector3<double> v;
    v << -1.0, 0.0, 0.0; // omega x (1,0,0) = (0,0,1) x (1,0,0) = (0,1,0)... let's use v = (-1,0,0) = (0,0,1) x (0,1,0)

    auto sa = liepp::to_screw_params(omega, v);

    REQUIRE(sa.s_hat(2) == Approx(1.0).margin(1e-12));
    REQUIRE(sa.h == Approx(0.0).margin(1e-12)); // Pure rotation: no pitch
}

TEST_CASE("screw_axis: pure translation twist", "[axis_angle]")
{
    liepp::vector3<double> omega = liepp::vector3<double>::Zero();
    liepp::vector3<double> v;
    v << 0.0, 0.0, 3.0; // Translation along z

    auto sa = liepp::to_screw_params(omega, v);

    REQUIRE(sa.s_hat(2) == Approx(1.0).margin(1e-12));
    REQUIRE(std::isinf(sa.h)); // Pure translation: infinite pitch
}

// ============================================================================
// Float scalar tests for quaternion utils
// ============================================================================

TEST_CASE("quaternion_utils: float scalar operations", "[quaternion_utils][float]")
{
    auto q = liepp::from_wxyz(1.0f, 0.0f, 0.0f, 0.0f);
    REQUIRE(q.w() == Approx(1.0f));

    auto R = liepp::quat_to_matrix(q);
    auto I = liepp::matrix3<float>::Identity();
    REQUIRE((R - I).norm() < 1e-6f);

    auto v = liepp::to_wxyz(q);
    REQUIRE(v(0) == Approx(1.0f));

    auto q2 = liepp::from_xyzw(0.0f, 0.0f, 0.0f, 1.0f);
    auto slerped = liepp::quat_slerp(q, q2, 0.5f);
    REQUIRE(slerped.norm() == Approx(1.0f).margin(1e-6f));
}
