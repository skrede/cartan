#include <liepp/lie/twist.h>

#include <liepp/lie/hat_vee.h>
#include <liepp/lie/quaternion_utils.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <cmath>
#include <numbers>

using Catch::Approx;

// ============================================================================
// twist from_vector / to_vector roundtrip
// ============================================================================

TEMPLATE_TEST_CASE("twist: from_vector / to_vector roundtrip", "[twist]", double, float)
{
    using S = TestType;
    S margin = std::is_same_v<S, float> ? S(1e-6) : S(1e-14);

    liepp::vector6<S> v;
    v << S(0.1), S(-0.2), S(0.3), S(0.4), S(-0.5), S(0.6);

    auto tw = liepp::twist<S>::from_vector(v);
    auto v_back = tw.to_vector();
    REQUIRE((v_back - v).norm() < margin);
}

// ============================================================================
// twist_to_se3: pure rotation
// ============================================================================

TEST_CASE("twist_to_se3: pure rotation produces pure rotation SE(3)", "[twist]")
{
    liepp::twist<double> tw;
    tw.omega << 0.0, 0.0, 1.0;   // Rotate about z
    tw.v << 0.0, 0.0, 0.0;       // No translation

    double theta = std::numbers::pi / 4.0;
    auto T = liepp::twist_to_se3(tw, theta);

    // Should have rotation but zero translation
    REQUIRE(T.translation().norm() < 1e-12);

    // Rotation matrix should match so3::exp(theta * z_hat)
    liepp::vector3<double> phi;
    phi << 0.0, 0.0, theta;
    auto R_expected = liepp::so3<double>::exp(phi).matrix();
    REQUIRE((T.rotation().matrix() - R_expected).norm() < 1e-12);
}

// ============================================================================
// twist_to_se3: pure translation
// ============================================================================

TEST_CASE("twist_to_se3: pure translation produces pure translation SE(3)", "[twist]")
{
    liepp::twist<double> tw;
    tw.omega << 0.0, 0.0, 0.0;
    tw.v << 0.0, 0.0, 1.0;       // Translate along z

    double d = 3.0;
    auto T = liepp::twist_to_se3(tw, d);

    // Rotation should be identity
    auto I = liepp::matrix3<double>::Identity();
    REQUIRE((T.rotation().matrix() - I).norm() < 1e-12);

    // Translation should be d * v
    REQUIRE(T.translation()(2) == Approx(3.0).margin(1e-12));
}

// ============================================================================
// twist_to_se3: general screw motion
// ============================================================================

TEST_CASE("twist_to_se3: general screw motion", "[twist]")
{
    liepp::twist<double> tw;
    tw.omega << 0.0, 0.0, 1.0;
    tw.v << 0.0, 0.0, 0.5;       // h = 0.5 pitch

    double theta = std::numbers::pi / 2.0;
    auto T = liepp::twist_to_se3(tw, theta);

    // Should have both rotation and translation
    auto R = T.rotation().matrix();
    auto I = liepp::matrix3<double>::Identity();
    REQUIRE((R - I).norm() > 0.1); // Not identity rotation
    REQUIRE(T.translation().norm() > 0.1); // Has translation
}

// ============================================================================
// se3_to_twist roundtrip
// ============================================================================

TEMPLATE_TEST_CASE("se3_to_twist roundtrip", "[twist]", double, float)
{
    using S = TestType;
    S margin = std::is_same_v<S, float> ? S(1e-3) : S(1e-10);

    SECTION("general rigid body motion")
    {
        liepp::vector6<S> v;
        v << S(0.3), S(-0.2), S(0.5), S(0.1), S(-0.4), S(0.2);
        auto T = liepp::se3<S>::exp(v);

        auto tw = liepp::se3_to_twist(T);
        auto T_back = liepp::se3<S>::exp(tw.to_vector());

        REQUIRE((T_back.matrix() - T.matrix()).norm() < margin);
    }

    SECTION("pure rotation")
    {
        liepp::vector6<S> v = liepp::vector6<S>::Zero();
        v(0) = S(0.5);
        v(1) = S(-0.3);
        v(2) = S(0.8);
        auto T = liepp::se3<S>::exp(v);

        auto tw = liepp::se3_to_twist(T);
        auto T_back = liepp::se3<S>::exp(tw.to_vector());

        REQUIRE((T_back.matrix() - T.matrix()).norm() < margin);
    }
}

// ============================================================================
// to_screw_motion: pure rotation has d=0
// ============================================================================

TEST_CASE("to_screw_motion: pure rotation has d=0", "[twist]")
{
    liepp::twist<double> tw;
    tw.omega << 0.0, 0.0, 1.0;
    tw.v << 0.0, 0.0, 0.0;

    auto sm = liepp::to_screw_motion(tw);

    REQUIRE(sm.d == Approx(0.0).margin(1e-12));
    REQUIRE(sm.theta == Approx(1.0).margin(1e-12)); // |omega| = 1
}

// ============================================================================
// to_screw_motion: pure translation has theta=0
// ============================================================================

TEST_CASE("to_screw_motion: pure translation has theta=0", "[twist]")
{
    liepp::twist<double> tw;
    tw.omega << 0.0, 0.0, 0.0;
    tw.v << 0.0, 0.0, 3.0;

    auto sm = liepp::to_screw_motion(tw);

    REQUIRE(sm.theta == Approx(0.0).margin(1e-12));
    REQUIRE(sm.d == Approx(3.0).margin(1e-12));
    REQUIRE(sm.axis.s_hat(2) == Approx(1.0).margin(1e-12));
}

// ============================================================================
// to_screw_motion: general screw with known pitch
// ============================================================================

TEST_CASE("to_screw_motion: general screw with known pitch", "[twist]")
{
    // Unit twist with omega along z, v with pitch component
    liepp::twist<double> tw;
    tw.omega << 0.0, 0.0, 1.0;
    tw.v << -1.0, 0.0, 0.5; // h = omega_hat . v = 0.5

    auto sm = liepp::to_screw_motion(tw);

    REQUIRE(sm.theta == Approx(1.0).margin(1e-12));
    REQUIRE(sm.d == Approx(0.5).margin(1e-12)); // d = h * theta
    REQUIRE(sm.axis.h == Approx(0.5).margin(1e-12));
}

// ============================================================================
// from_screw_motion roundtrip
// ============================================================================

TEST_CASE("from_screw_motion roundtrip", "[twist]")
{
    liepp::twist<double> tw;
    tw.omega << 0.0, 0.0, 1.0;
    tw.v << -1.0, 0.0, 0.3;

    auto sm = liepp::to_screw_motion(tw);
    auto tw_back = liepp::from_screw_motion(sm);

    REQUIRE((tw_back.to_vector() - tw.to_vector()).norm() < 1e-10);
}

TEST_CASE("from_screw_motion roundtrip: pure translation", "[twist]")
{
    liepp::twist<double> tw;
    tw.omega = liepp::vector3<double>::Zero();
    tw.v << 0.0, 2.0, 0.0;

    auto sm = liepp::to_screw_motion(tw);
    auto tw_back = liepp::from_screw_motion(sm);

    REQUIRE((tw_back.to_vector() - tw.to_vector()).norm() < 1e-10);
}

// ============================================================================
// Float scalar tests
// ============================================================================

TEST_CASE("twist: float scalar operations", "[twist][float]")
{
    liepp::vector6<float> v;
    v << 0.1f, -0.2f, 0.3f, 0.4f, -0.5f, 0.6f;

    auto tw = liepp::twist<float>::from_vector(v);
    auto v_back = tw.to_vector();
    REQUIRE((v_back - v).norm() < 1e-6f);

    auto T = liepp::twist_to_se3(tw, 1.0f);
    auto tw_back = liepp::se3_to_twist(T);
    auto T_back = liepp::se3<float>::exp(tw_back.to_vector());
    REQUIRE((T_back.matrix() - T.matrix()).norm() < 1e-3f);
}

// ============================================================================
// Umbrella header test
// ============================================================================

TEST_CASE("umbrella header includes all lie types", "[umbrella]")
{
    // Verify we can use all types after including liepp.h
    // (this test includes twist.h directly, but we verify liepp.h
    //  compiles correctly in the overall verification step)
    auto r = liepp::so3<double>::identity();
    auto T = liepp::se3<double>::identity();
    liepp::twist<double> tw;
    tw.omega = liepp::vector3<double>::Zero();
    tw.v = liepp::vector3<double>::UnitX();
    auto q = liepp::from_wxyz(1.0, 0.0, 0.0, 0.0);
    liepp::axis_angle<double> aa{liepp::vector3<double>::UnitZ(), 0.5};

    REQUIRE(r.matrix().isApprox(liepp::matrix3<double>::Identity()));
    REQUIRE(T.translation().norm() < 1e-14);
    REQUIRE(q.w() == Approx(1.0));
    REQUIRE(aa.angle == Approx(0.5));
}
