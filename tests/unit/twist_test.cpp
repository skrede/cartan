#include <cartan/lie/twist.h>

#include <cartan/lie/hat_vee.h>
#include <cartan/lie/quaternion_utils.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <cmath>
#include <random>
#include <numbers>

using Catch::Approx;

// ============================================================================
// twist from_vector / to_vector roundtrip
// ============================================================================

TEMPLATE_TEST_CASE("twist: from_vector / to_vector roundtrip", "[twist]", double, float)
{
    using S = TestType;
    S margin = std::is_same_v<S, float> ? S(1e-6) : S(1e-14);

    cartan::vector6<S> v;
    v << S(0.1), S(-0.2), S(0.3), S(0.4), S(-0.5), S(0.6);

    auto tw = cartan::twist<S>::from_vector(v);
    auto v_back = tw.to_vector();
    REQUIRE((v_back - v).norm() < margin);
}

// ============================================================================
// twist_to_se3: pure rotation
// ============================================================================

TEST_CASE("twist_to_se3: pure rotation produces pure rotation SE(3)", "[twist]")
{
    cartan::twist<double> tw;
    tw.omega << 0.0, 0.0, 1.0;   // Rotate about z
    tw.v << 0.0, 0.0, 0.0;       // No translation

    double theta = std::numbers::pi / 4.0;
    auto T = cartan::twist_to_se3(tw, theta);

    // Should have rotation but zero translation
    REQUIRE(T.translation().norm() < 1e-12);

    // Rotation matrix should match so3::exp(theta * z_hat)
    cartan::vector3<double> phi;
    phi << 0.0, 0.0, theta;
    auto R_expected = cartan::so3<double>::exp(phi).matrix();
    REQUIRE((T.rotation().matrix() - R_expected).norm() < 1e-12);
}

// ============================================================================
// twist_to_se3: pure translation
// ============================================================================

TEST_CASE("twist_to_se3: pure translation produces pure translation SE(3)", "[twist]")
{
    cartan::twist<double> tw;
    tw.omega << 0.0, 0.0, 0.0;
    tw.v << 0.0, 0.0, 1.0;       // Translate along z

    double d = 3.0;
    auto T = cartan::twist_to_se3(tw, d);

    // Rotation should be identity
    auto I = cartan::matrix3<double>::Identity();
    REQUIRE((T.rotation().matrix() - I).norm() < 1e-12);

    // Translation should be d * v
    REQUIRE(T.translation()(2) == Approx(3.0).margin(1e-12));
}

// ============================================================================
// twist_to_se3: general screw motion
// ============================================================================

TEST_CASE("twist_to_se3: general screw motion", "[twist]")
{
    cartan::twist<double> tw;
    tw.omega << 0.0, 0.0, 1.0;
    tw.v << 0.0, 0.0, 0.5;       // h = 0.5 pitch

    double theta = std::numbers::pi / 2.0;
    auto T = cartan::twist_to_se3(tw, theta);

    // Should have both rotation and translation
    auto R = T.rotation().matrix();
    auto I = cartan::matrix3<double>::Identity();
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
        cartan::vector6<S> v;
        v << S(0.3), S(-0.2), S(0.5), S(0.1), S(-0.4), S(0.2);
        auto T = cartan::se3<S>::exp(v);

        auto tw = cartan::se3_to_twist(T);
        auto T_back = cartan::se3<S>::exp(tw.to_vector());

        REQUIRE((T_back.matrix() - T.matrix()).norm() < margin);
    }

    SECTION("pure rotation")
    {
        cartan::vector6<S> v = cartan::vector6<S>::Zero();
        v(0) = S(0.5);
        v(1) = S(-0.3);
        v(2) = S(0.8);
        auto T = cartan::se3<S>::exp(v);

        auto tw = cartan::se3_to_twist(T);
        auto T_back = cartan::se3<S>::exp(tw.to_vector());

        REQUIRE((T_back.matrix() - T.matrix()).norm() < margin);
    }
}

// ============================================================================
// to_screw_motion: pure rotation has d=0
// ============================================================================

TEST_CASE("to_screw_motion: pure rotation has d=0", "[twist]")
{
    cartan::twist<double> tw;
    tw.omega << 0.0, 0.0, 1.0;
    tw.v << 0.0, 0.0, 0.0;

    auto sm = cartan::to_screw_motion(tw);

    REQUIRE(sm.d == Approx(0.0).margin(1e-12));
    REQUIRE(sm.theta == Approx(1.0).margin(1e-12)); // |omega| = 1
}

// ============================================================================
// to_screw_motion: pure translation has theta=0
// ============================================================================

TEST_CASE("to_screw_motion: pure translation has theta=0", "[twist]")
{
    cartan::twist<double> tw;
    tw.omega << 0.0, 0.0, 0.0;
    tw.v << 0.0, 0.0, 3.0;

    auto sm = cartan::to_screw_motion(tw);

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
    cartan::twist<double> tw;
    tw.omega << 0.0, 0.0, 1.0;
    tw.v << -1.0, 0.0, 0.5; // h = omega_hat . v = 0.5

    auto sm = cartan::to_screw_motion(tw);

    REQUIRE(sm.theta == Approx(1.0).margin(1e-12));
    REQUIRE(sm.d == Approx(0.5).margin(1e-12)); // d = h * theta
    REQUIRE(sm.axis.h == Approx(0.5).margin(1e-12));
}

// ============================================================================
// from_screw_motion roundtrip
// ============================================================================

TEST_CASE("from_screw_motion roundtrip", "[twist]")
{
    cartan::twist<double> tw;
    tw.omega << 0.0, 0.0, 1.0;
    tw.v << -1.0, 0.0, 0.3;

    auto sm = cartan::to_screw_motion(tw);
    auto tw_back = cartan::from_screw_motion(sm);

    REQUIRE((tw_back.to_vector() - tw.to_vector()).norm() < 1e-10);
}

TEST_CASE("from_screw_motion roundtrip: pure translation", "[twist]")
{
    cartan::twist<double> tw;
    tw.omega = cartan::vector3<double>::Zero();
    tw.v << 0.0, 2.0, 0.0;

    auto sm = cartan::to_screw_motion(tw);
    auto tw_back = cartan::from_screw_motion(sm);

    REQUIRE((tw_back.to_vector() - tw.to_vector()).norm() < 1e-10);
}

// ============================================================================
// to_screw_motion / to_screw_params: non-unit-norm sweep against the
// Modern Robotics screw-parameter oracle (Lynch & Park, Def. 3.24).
//
// For a twist V = (omega, v) with omega_norm = ||omega|| and
// omega_hat = omega / omega_norm, the screw parameters are:
//     pitch    h = (omega_hat . v) / omega_norm
//     point    q = (omega_hat x v) / omega_norm
//     angle    theta = omega_norm
//     distance d = h * theta
// At omega_norm == 1 every term collapses onto the unit-twist formulas, so the
// sweep deliberately drives ||omega|| through {0.3, 0.7, 2.0, 5.0}, where the
// 1/omega_norm factor on the pitch (and on axis_angle's point q) is observable.
// ============================================================================

TEST_CASE("to_screw_motion / to_screw_params: non-unit-norm MR oracle sweep", "[twist][screw]")
{
    std::mt19937 rng(0xC0FFEEu);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    constexpr double scalar_margin = 1e-13;
    constexpr double vector_margin = 1e-12;

    for (double omega_norm : {0.3, 0.7, 2.0, 5.0})
    {
        for (int sample = 0; sample < 32; ++sample)
        {
            cartan::vector3<double> omega_dir;
            omega_dir << dist(rng), dist(rng), dist(rng);
            // Guard against a near-zero direction (ill-defined omega_hat).
            if (omega_dir.norm() < 1e-3)
                omega_dir = cartan::vector3<double>::UnitZ();
            omega_dir.normalize();

            cartan::twist<double> tw;
            tw.omega = omega_norm * omega_dir;
            tw.v << dist(rng), dist(rng), dist(rng);

            const cartan::vector3<double> omega_hat = tw.omega / omega_norm;

            // Modern Robotics oracle (Def. 3.24).
            const double oracle_h = omega_hat.dot(tw.v) / omega_norm;
            const cartan::vector3<double> oracle_q =
                omega_hat.cross(tw.v) / omega_norm;
            const double oracle_theta = omega_norm;
            const double oracle_d = oracle_h * oracle_theta;

            // twist.h: to_screw_motion must match the oracle at non-unit norm.
            auto sm = cartan::to_screw_motion(tw);
            REQUIRE(sm.theta == Approx(oracle_theta).margin(scalar_margin));
            REQUIRE(sm.axis.h == Approx(oracle_h).margin(scalar_margin));
            REQUIRE(sm.d == Approx(oracle_d).margin(scalar_margin));
            REQUIRE((sm.axis.q - oracle_q).norm() < vector_margin);
            REQUIRE((sm.axis.s_hat - omega_hat).norm() < vector_margin);

            // axis_angle.h: to_screw_params must agree with the same oracle.
            auto sp = cartan::to_screw_params(tw.omega, tw.v);
            REQUIRE(sp.h == Approx(oracle_h).margin(scalar_margin));
            REQUIRE((sp.q - oracle_q).norm() < vector_margin);
            REQUIRE((sp.s_hat - omega_hat).norm() < vector_margin);

            // Round-trip must reproduce the original twist exactly.
            auto tw_back = cartan::from_screw_motion(sm);
            REQUIRE((tw_back.omega - tw.omega).norm() < vector_margin);
            REQUIRE((tw_back.v - tw.v).norm() < vector_margin);
        }
    }
}

// ============================================================================
// Float scalar tests
// ============================================================================

TEST_CASE("twist: float scalar operations", "[twist][float]")
{
    cartan::vector6<float> v;
    v << 0.1f, -0.2f, 0.3f, 0.4f, -0.5f, 0.6f;

    auto tw = cartan::twist<float>::from_vector(v);
    auto v_back = tw.to_vector();
    REQUIRE((v_back - v).norm() < 1e-6f);

    auto T = cartan::twist_to_se3(tw, 1.0f);
    auto tw_back = cartan::se3_to_twist(T);
    auto T_back = cartan::se3<float>::exp(tw_back.to_vector());
    REQUIRE((T_back.matrix() - T.matrix()).norm() < 1e-3f);
}

// ============================================================================
// Umbrella header test
// ============================================================================

TEST_CASE("umbrella header includes all lie types", "[umbrella]")
{
    // Verify we can use all types after including cartan.h
    // (this test includes twist.h directly, but we verify cartan.h
    //  compiles correctly in the overall verification step)
    auto r = cartan::so3<double>::identity();
    auto T = cartan::se3<double>::identity();
    cartan::twist<double> tw;
    tw.omega = cartan::vector3<double>::Zero();
    tw.v = cartan::vector3<double>::UnitX();
    auto q = cartan::from_wxyz(1.0, 0.0, 0.0, 0.0);
    cartan::axis_angle<double> aa{cartan::vector3<double>::UnitZ(), 0.5};

    REQUIRE(r.matrix().isApprox(cartan::matrix3<double>::Identity()));
    REQUIRE(T.translation().norm() < 1e-14);
    REQUIRE(q.w() == Approx(1.0));
    REQUIRE(aa.angle == Approx(0.5));
}
