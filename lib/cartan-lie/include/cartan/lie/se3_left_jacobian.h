#ifndef HPP_GUARD_CARTAN_LIE_SE3_LEFT_JACOBIAN_H
#define HPP_GUARD_CARTAN_LIE_SE3_LEFT_JACOBIAN_H

#include "cartan/types.h"
#include "cartan/detail/epsilon.h"

#include "cartan/lie/so3.h"
#include "cartan/lie/hat_vee.h"

#include <cmath>
#include <type_traits>

namespace cartan
{

namespace detail
{

/// The three scalar coefficients (c1, c2, c3) of the SE(3) Q-matrix expansion.
template <typename Scalar>
struct se3_q_coeffs
{
    Scalar c1;
    Scalar c2;
    Scalar c3;
};

/// Cancellation-free small-angle series for the Q-matrix coefficients, carried
/// to third order in phi_sq. Accurate to machine precision for small phi where
/// the closed forms lose significance to catastrophic cancellation.
///
/// c1 = 1/6   - phi^2/120  + phi^4/5040
/// c2 = 1/24  - phi^2/720  + phi^4/40320
/// c3 = 1/120 - phi^2/2520 + phi^4/120960
template <typename Scalar>
constexpr se3_q_coeffs<Scalar> se3_q_taylor_coeffs(Scalar phi_sq)
{
    const Scalar phi_4 = phi_sq * phi_sq;
    return {
        Scalar(1) / Scalar(6)   - phi_sq / Scalar(120)  + phi_4 / Scalar(5040),
        Scalar(1) / Scalar(24)  - phi_sq / Scalar(720)  + phi_4 / Scalar(40320),
        Scalar(1) / Scalar(120) - phi_sq / Scalar(2520) + phi_4 / Scalar(120960),
    };
}

/// Exact closed forms for the Q-matrix coefficients (Barfoot, Eq. 8.91a). These
/// are accurate for larger phi but suffer cancellation as phi -> 0.
template <typename Scalar>
se3_q_coeffs<Scalar> se3_q_closed_coeffs(Scalar phi)
{
    const Scalar phi_sq = phi * phi;
    const Scalar phi_cu = phi_sq * phi;
    const Scalar phi_4 = phi_sq * phi_sq;
    const Scalar phi_5 = phi_4 * phi;
    const Scalar s = std::sin(phi);
    const Scalar c = std::cos(phi);

    return {
        (phi - s) / phi_cu,
        (phi_sq + Scalar(2) * c - Scalar(2)) / (Scalar(2) * phi_4),
        (Scalar(2) * phi - Scalar(3) * s + phi * c) / (Scalar(2) * phi_5),
    };
}

/// Per-precision threshold (compared against phi_sq) below which the Q-matrix
/// uses the cancellation-free series instead of the closed form. The values are
/// fixed by a recorded long-double-oracle relative-error sweep: the crossover
/// where the series error first exceeds the closed-form error differs by roughly
/// 2.5x between float and double, so the switch is precision-dependent.
template <typename Scalar>
struct q_taylor_switch_traits
{
    static_assert(std::is_floating_point_v<Scalar>);
    // double: the series and closed form cross over near phi_sq ~ 0.015; 0.01
    // keeps the series active where it is still the more accurate of the two.
    static constexpr Scalar value = Scalar(0.01);
};

template <>
struct q_taylor_switch_traits<float>
{
    // float: the cancellation-free series beats the closed form across the whole
    // small-angle band (their crossover is out near phi ~ 0.55), so the switch is
    // bounded below by the closed form's danger zone rather than by a crossover.
    // It must exceed 0.3^2 = 0.09 so the series covers phi up to 0.3, where the
    // closed form is otherwise catastrophic; 0.1 clears that with margin and stays
    // well below where the third-order truncation grows.
    static constexpr float value = 0.1f;
};

template <typename Scalar>
inline constexpr Scalar q_taylor_switch_v = q_taylor_switch_traits<Scalar>::value;

}

/// Q matrix for SE(3) left Jacobian computation.
/// Adapted from Barfoot, State Estimation for Robotics, Eq. 8.91a
/// to cartan's omega-first convention.
template <typename Scalar>
matrix3<Scalar> se3_Q_matrix(const vector3<Scalar>& omega, const vector3<Scalar>& rho)
{
    Scalar phi_sq = omega.squaredNorm();

    detail::se3_q_coeffs<Scalar> coeffs =
        (phi_sq < detail::q_taylor_switch_v<Scalar>)
            ? detail::se3_q_taylor_coeffs<Scalar>(phi_sq)
            : detail::se3_q_closed_coeffs<Scalar>(std::sqrt(phi_sq));

    const Scalar c1 = coeffs.c1;
    const Scalar c2 = coeffs.c2;
    const Scalar c3 = coeffs.c3;

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
matrix6<Scalar> se3_left_jacobian(const vector6<Scalar>& xi)
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
matrix6<Scalar> se3_left_jacobian_inv(const vector6<Scalar>& xi)
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
