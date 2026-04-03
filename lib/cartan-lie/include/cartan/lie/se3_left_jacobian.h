#ifndef HPP_GUARD_CARTAN_LIE_SE3_LEFT_JACOBIAN_H
#define HPP_GUARD_CARTAN_LIE_SE3_LEFT_JACOBIAN_H

#include "cartan/types.h"
#include "cartan/detail/epsilon.h"

#include "cartan/lie/so3.h"
#include "cartan/lie/hat_vee.h"

#include <cmath>

namespace cartan
{

/// Q matrix for SE(3) left Jacobian computation.
/// Adapted from Barfoot, State Estimation for Robotics, Eq. 8.91a
/// to cartan's omega-first convention.
template <typename Scalar>
[[nodiscard]] matrix3<Scalar> se3_Q_matrix(const vector3<Scalar>& omega, const vector3<Scalar>& rho)
{
    Scalar phi_sq = omega.squaredNorm();

    Scalar c1, c2, c3;

    if (phi_sq < detail::sqrt_epsilon_v<Scalar>)
    {
        c1 = Scalar(1) / Scalar(6) - phi_sq / Scalar(120);
        c2 = Scalar(1) / Scalar(24) - phi_sq / Scalar(720);
        c3 = Scalar(1) / Scalar(120) - phi_sq / Scalar(5040);
    }
    else
    {
        Scalar phi = std::sqrt(phi_sq);
        Scalar phi_cu = phi_sq * phi;
        Scalar phi_4 = phi_sq * phi_sq;
        Scalar phi_5 = phi_4 * phi;
        Scalar s = std::sin(phi);
        Scalar c = std::cos(phi);

        c1 = (phi - s) / phi_cu;
        c2 = (phi_sq + Scalar(2) * c - Scalar(2)) / (Scalar(2) * phi_4);
        c3 = (Scalar(2) * phi - Scalar(3) * s + phi * c) / (Scalar(2) * phi_5);
    }

    matrix3<Scalar> omega_hat = hat(omega);
    matrix3<Scalar> rho_hat = hat(rho);
    matrix3<Scalar> omega_sq = omega_hat * omega_hat;

    return Scalar(0.5) * rho_hat
         + c1 * (omega_hat * rho_hat + rho_hat * omega_hat + omega_hat * rho_hat * omega_hat)
         + c2 * (omega_sq * rho_hat + rho_hat * omega_sq - Scalar(3) * omega_hat * rho_hat * omega_hat)
         + c3 * (omega_sq * rho_hat * omega_hat + omega_hat * rho_hat * omega_sq);
}

/// SE(3) left Jacobian.
/// Uses cartan's omega-first convention: top-left = J_so3, bottom-left = Q.
/// Ref: Barfoot, State Estimation for Robotics, Eq. 8.91, adapted to omega-first.
template <typename Scalar>
[[nodiscard]] matrix6<Scalar> se3_left_jacobian(const vector6<Scalar>& xi)
{
    vector3<Scalar> omega = xi.template head<3>();
    vector3<Scalar> rho = xi.template tail<3>();

    matrix3<Scalar> J_so3 = so3<Scalar>::left_jacobian(omega);
    matrix3<Scalar> Q = se3_Q_matrix(omega, rho);

    matrix6<Scalar> J = matrix6<Scalar>::Zero();
    J.template block<3, 3>(0, 0) = J_so3;
    J.template block<3, 3>(3, 0) = Q;
    J.template block<3, 3>(3, 3) = J_so3;

    return J;
}

/// SE(3) inverse left Jacobian.
/// Uses cartan's omega-first convention.
/// Ref: Barfoot, State Estimation for Robotics, Eq. 8.100b, adapted to omega-first.
template <typename Scalar>
[[nodiscard]] matrix6<Scalar> se3_left_jacobian_inv(const vector6<Scalar>& xi)
{
    vector3<Scalar> omega = xi.template head<3>();
    vector3<Scalar> rho = xi.template tail<3>();

    matrix3<Scalar> J_inv = so3<Scalar>::left_jacobian_inv(omega);
    matrix3<Scalar> Q = se3_Q_matrix(omega, rho);

    matrix6<Scalar> J = matrix6<Scalar>::Zero();
    J.template block<3, 3>(0, 0) = J_inv;
    J.template block<3, 3>(3, 0) = -J_inv * Q * J_inv;
    J.template block<3, 3>(3, 3) = J_inv;

    return J;
}

}

#endif
