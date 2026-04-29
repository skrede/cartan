#include <cartan/lie/se3_left_jacobian.h>
#include <cartan/serial/ik/policy/error_weight.h>
#include <cartan/lie/se3.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

using Catch::Approx;

// ============================================================================
// SE(3) left Jacobian at identity
// ============================================================================

TEST_CASE("SE(3) left Jacobian at identity", "[se3_left_jacobian]")
{
    cartan::vector6<double> zero = cartan::vector6<double>::Zero();
    auto J = cartan::se3_left_jacobian(zero);
    auto I = cartan::matrix6<double>::Identity();
    REQUIRE((J - I).norm() < 1e-12);
}

// ============================================================================
// SE(3) left Jacobian inverse is inverse
// ============================================================================

TEST_CASE("SE(3) left Jacobian inverse is inverse", "[se3_left_jacobian]")
{
    auto I6 = cartan::matrix6<double>::Identity();

    SECTION("small angle (0.01)")
    {
        cartan::vector6<double> v;
        v << 0.005, -0.003, 0.008, 0.1, -0.2, 0.3;
        auto J = cartan::se3_left_jacobian(v);
        auto J_inv = cartan::se3_left_jacobian_inv(v);
        REQUIRE((J_inv * J - I6).norm() < 1e-10);
    }

    SECTION("moderate angle (1.5)")
    {
        cartan::vector6<double> v;
        v << 0.9, -0.6, 0.9, 1.0, -0.5, 0.2;
        auto J = cartan::se3_left_jacobian(v);
        auto J_inv = cartan::se3_left_jacobian_inv(v);
        REQUIRE((J_inv * J - I6).norm() < 1e-10);
    }

    SECTION("near-pi angle (3.1)")
    {
        cartan::vector6<double> v;
        cartan::vector3<double> axis;
        axis << 0.0, 0.0, 1.0;
        v.head<3>() = 3.1 * axis;
        v.tail<3>() << 0.5, -0.3, 1.2;
        auto J = cartan::se3_left_jacobian(v);
        auto J_inv = cartan::se3_left_jacobian_inv(v);
        REQUIRE((J_inv * J - I6).norm() < 1e-10);
    }
}

// ============================================================================
// SE(3) left Jacobian matches finite difference
// ============================================================================

TEST_CASE("SE(3) left Jacobian matches finite difference", "[se3_left_jacobian]")
{
    cartan::vector6<double> v;
    v << 0.3, -0.2, 0.5, 1.0, -0.5, 0.2;

    auto J = cartan::se3_left_jacobian(v);
    auto T = cartan::se3<double>::exp(v);

    double eps = 1e-7;
    cartan::matrix6<double> J_fd;

    // Left Jacobian: exp(xi + delta) ~ exp(J_l(xi) * delta) * exp(xi)
    // So J_l(xi) * e_j ~ log(exp(xi + eps*e_j) * exp(-xi)) / eps
    for (int i = 0; i < 6; ++i)
    {
        cartan::vector6<double> v_plus = v;
        cartan::vector6<double> v_minus = v;
        v_plus(i) += eps;
        v_minus(i) -= eps;

        auto T_plus = cartan::se3<double>::exp(v_plus);
        auto T_minus = cartan::se3<double>::exp(v_minus);

        // Materialize the log() prvalues before subtracting; otherwise the
        // Eigen expression template binds to dangling temporaries.
        cartan::vector6<double> log_plus  = (T_plus  * T.inverse()).log();
        cartan::vector6<double> log_minus = (T_minus * T.inverse()).log();
        cartan::vector6<double> diff = log_plus - log_minus;
        J_fd.col(i) = diff / (2.0 * eps);
    }

    REQUIRE((J - J_fd).norm() < 1e-6);
}

// ============================================================================
// SE(3) Q matrix near theta=0 is finite
// ============================================================================

TEST_CASE("SE(3) Q matrix near theta=0 is finite", "[se3_left_jacobian]")
{
    cartan::vector3<double> omega;
    omega << 1e-12, 2e-12, 3e-12;
    cartan::vector3<double> rho;
    rho << 1.0, -0.5, 0.3;

    auto Q = cartan::se3_Q_matrix(omega, rho);

    REQUIRE(Q.allFinite());
    // Q should be close to 0.5 * hat(rho) for omega ~ 0
    auto rho_hat = cartan::hat(rho);
    REQUIRE((Q - 0.5 * rho_hat).norm() < 1e-6);
}

// ============================================================================
// SE(3) left Jacobian handles pure translation
// ============================================================================

TEST_CASE("SE(3) left Jacobian handles pure translation", "[se3_left_jacobian]")
{
    cartan::vector6<double> v = cartan::vector6<double>::Zero();
    v.tail<3>() << 1.0, 2.0, 3.0;

    auto J = cartan::se3_left_jacobian(v);
    REQUIRE(J.allFinite());

    // For omega=0: top-left = I, top-right = 0, bottom-right = I
    // bottom-left = Q = 0.5*hat(rho) (not zero!)
    auto I3 = cartan::matrix3<double>::Identity();
    REQUIRE((J.block<3, 3>(0, 0) - I3).norm() < 1e-10);
    REQUIRE(J.block<3, 3>(0, 3).norm() < 1e-10);
    REQUIRE((J.block<3, 3>(3, 3) - I3).norm() < 1e-10);

    cartan::vector3<double> rho;
    rho << 1.0, 2.0, 3.0;
    cartan::matrix3<double> expected_Q = 0.5 * cartan::hat(rho);
    REQUIRE((J.block<3, 3>(3, 0) - expected_Q).norm() < 1e-10);
}

// ============================================================================
// error_weight default is ones
// ============================================================================

TEST_CASE("error_weight default is ones", "[error_weight]")
{
    cartan::error_weight<double> w;
    REQUIRE((w.weights - cartan::vector6<double>::Ones()).norm() < 1e-14);

    cartan::vector6<double> v;
    v << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
    auto result = w.apply(v);
    REQUIRE((result - v).norm() < 1e-14);

    cartan::error_weight<double> w2;
    w2.weights << 2.0, 3.0, 4.0, 5.0, 6.0, 7.0;
    auto scaled = w2.apply(v);
    cartan::vector6<double> expected;
    expected << 2.0, 6.0, 12.0, 20.0, 30.0, 42.0;
    REQUIRE((scaled - expected).norm() < 1e-14);
}
