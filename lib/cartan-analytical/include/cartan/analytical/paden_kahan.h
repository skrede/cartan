#ifndef HPP_GUARD_CARTAN_ANALYTICAL_PADEN_KAHAN_H
#define HPP_GUARD_CARTAN_ANALYTICAL_PADEN_KAHAN_H

#include "cartan/analytical/analytical_types.h"
#include "cartan/analytical/detail/clamped_trig.h"

#include "cartan/detail/epsilon.h"
#include "cartan/types.h"

#include <cmath>
#include "cartan/expected.h"
#include <numbers>

namespace cartan
{

/// Paden-Kahan subproblem 1: rotation about a single axis.
///
/// Find theta such that exp([omega]*theta) applied at point q maps p to p'.
/// Both p and p' must be equidistant from the axis of rotation.
///
/// Reference: Murray, Li and Sastry (1994), Section 3.3.1.
template <typename Scalar>
[[nodiscard]] cartan::expected<Scalar, analytical_failure>
paden_kahan_1(
    const vector3<Scalar>& omega,
    const vector3<Scalar>& q,
    const vector3<Scalar>& p,
    const vector3<Scalar>& p_prime)
{
    vector3<Scalar> u = p - q;
    vector3<Scalar> u_prime = p_prime - q;

    vector3<Scalar> u_perp = u - omega.dot(u) * omega;
    vector3<Scalar> u_prime_perp = u_prime - omega.dot(u_prime) * omega;

    Scalar u_norm = u_perp.norm();
    Scalar u_prime_norm = u_prime_perp.norm();

    if (std::abs(u_norm - u_prime_norm) >
        detail::sqrt_epsilon_v<Scalar> * std::max({u_norm, u_prime_norm, Scalar(1)}))
    {
        return cartan::unexpected(analytical_failure::unreachable);
    }

    Scalar cos_theta = u_perp.dot(u_prime_perp) / (u_norm * u_prime_norm);
    Scalar sin_theta = omega.dot(u_perp.cross(u_prime_perp)) / (u_norm * u_prime_norm);

    return std::atan2(sin_theta, cos_theta);
}

/// Result type for Paden-Kahan subproblem 2 (two rotations, up to 2 solutions).
template <typename Scalar>
struct paden_kahan_2_result
{
    std::array<std::pair<Scalar, Scalar>, 2> solutions;
    int count{0};
};

/// Paden-Kahan subproblem 2: two successive rotations about intersecting axes.
///
/// Find (theta1, theta2) such that exp([omega1]*theta1) * exp([omega2]*theta2)
/// applied at point q maps p to p'. Axes omega1 and omega2 must intersect at q.
/// Returns up to 2 solution pairs.
///
/// Reference: Murray, Li and Sastry (1994), Section 3.3.2.
template <typename Scalar>
[[nodiscard]] cartan::expected<paden_kahan_2_result<Scalar>, analytical_failure>
paden_kahan_2(
    const vector3<Scalar>& omega1,
    const vector3<Scalar>& omega2,
    const vector3<Scalar>& q,
    const vector3<Scalar>& p,
    const vector3<Scalar>& p_prime)
{
    vector3<Scalar> u = p - q;
    vector3<Scalar> u_prime = p_prime - q;

    Scalar c = omega1.dot(omega2);
    Scalar one_minus_c_sq = Scalar(1) - c * c;

    if (std::abs(one_minus_c_sq) < detail::epsilon_v<Scalar>)
        return cartan::unexpected(analytical_failure::degenerate_geometry);

    Scalar alpha = (omega1.dot(u_prime) - c * omega2.dot(u)) / one_minus_c_sq;
    Scalar beta = (omega2.dot(u) - c * omega1.dot(u_prime)) / one_minus_c_sq;

    vector3<Scalar> cross = omega1.cross(omega2);
    Scalar cross_sq_norm = cross.squaredNorm();

    Scalar gamma_sq = (u.squaredNorm() - alpha * alpha - beta * beta
        - Scalar(2) * alpha * beta * c) / cross_sq_norm;

    if (gamma_sq < -detail::sqrt_epsilon_v<Scalar>)
        return cartan::unexpected(analytical_failure::unreachable);

    gamma_sq = std::max(gamma_sq, Scalar(0));
    Scalar gamma = std::sqrt(gamma_sq);

    paden_kahan_2_result<Scalar> result;
    int signs = (gamma < detail::sqrt_epsilon_v<Scalar>) ? 1 : 2;

    for (int sign = 0; sign < signs; ++sign)
    {
        Scalar g = (sign == 0) ? gamma : -gamma;
        vector3<Scalar> z = q + alpha * omega1 + beta * omega2 + g * cross;

        auto theta2 = paden_kahan_1(omega2, q, p, z);
        if (!theta2)
            continue;

        auto theta1 = paden_kahan_1(omega1, q, z, p_prime);
        if (!theta1)
            continue;

        result.solutions[static_cast<std::size_t>(result.count)] = {*theta1, *theta2};
        ++result.count;
    }

    if (result.count == 0)
        return cartan::unexpected(analytical_failure::unreachable);

    return result;
}

/// Result type for Paden-Kahan subproblem 3 (distance constraint, up to 2 solutions).
template <typename Scalar>
struct paden_kahan_3_result
{
    std::array<Scalar, 2> solutions;
    int count{0};
};

/// Paden-Kahan subproblem 3: rotation with distance constraint.
///
/// Find theta such that || exp([omega]*theta) * p - p' || = delta,
/// where rotation is about axis omega through point q. Returns up to 2 solutions.
///
/// Reference: Murray, Li and Sastry (1994), Section 3.3.3.
template <typename Scalar>
[[nodiscard]] cartan::expected<paden_kahan_3_result<Scalar>, analytical_failure>
paden_kahan_3(
    const vector3<Scalar>& omega,
    const vector3<Scalar>& q,
    const vector3<Scalar>& p,
    const vector3<Scalar>& p_prime,
    Scalar delta)
{
    vector3<Scalar> u = p - q;
    vector3<Scalar> u_prime = p_prime - q;

    vector3<Scalar> u_perp = u - omega.dot(u) * omega;
    vector3<Scalar> u_prime_perp = u_prime - omega.dot(u_prime) * omega;

    Scalar u_par = omega.dot(u);
    Scalar u_prime_par = omega.dot(u_prime);

    Scalar delta_perp_sq = delta * delta - (u_par - u_prime_par) * (u_par - u_prime_par);

    if (delta_perp_sq < -detail::sqrt_epsilon_v<Scalar>)
        return cartan::unexpected(analytical_failure::unreachable);

    delta_perp_sq = std::max(delta_perp_sq, Scalar(0));

    Scalar u_perp_norm = u_perp.norm();
    Scalar u_prime_perp_norm = u_prime_perp.norm();

    if (u_perp_norm < detail::sqrt_epsilon_v<Scalar>
        || u_prime_perp_norm < detail::sqrt_epsilon_v<Scalar>)
    {
        return cartan::unexpected(analytical_failure::degenerate_geometry);
    }

    Scalar cos_theta_0 = (u_perp.squaredNorm() + u_prime_perp.squaredNorm() - delta_perp_sq)
        / (Scalar(2) * u_perp_norm * u_prime_perp_norm);

    if (std::abs(cos_theta_0) > Scalar(1) + detail::sqrt_epsilon_v<Scalar>)
        return cartan::unexpected(analytical_failure::unreachable);

    Scalar theta_0 = detail::safe_acos(cos_theta_0);
    Scalar theta_base = std::atan2(
        omega.dot(u_perp.cross(u_prime_perp)),
        u_perp.dot(u_prime_perp));

    paden_kahan_3_result<Scalar> result;

    if (theta_0 < detail::sqrt_epsilon_v<Scalar>
        || std::abs(theta_0 - std::numbers::pi_v<Scalar>) < detail::sqrt_epsilon_v<Scalar>)
    {
        result.solutions[0] = theta_base + theta_0;
        result.count = 1;
    }
    else
    {
        result.solutions[0] = theta_base + theta_0;
        result.solutions[1] = theta_base - theta_0;
        result.count = 2;
    }

    return result;
}

}

#endif
