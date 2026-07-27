#ifndef HPP_GUARD_CARTAN_ANALYTICAL_DETAIL_SUBPROBLEM_1_H
#define HPP_GUARD_CARTAN_ANALYTICAL_DETAIL_SUBPROBLEM_1_H

#include "cartan/analytical/analytical_types.h"
#include "cartan/analytical/detail/axis_rotation.h"

#include "cartan/detail/epsilon.h"
#include "cartan/types.h"

#include <cmath>
#include "cartan/expected.h"
#include <optional>

namespace cartan::detail
{

/// Neither solvability condition below has an answer for a nonfinite input, and
/// both read omega as a unit vector. The screw-axis type normalizes on
/// construction, so a non-unit axis only reaches here from a direct caller of
/// the free subproblem functions.
template <typename Scalar>
std::optional<analytical_failure> subproblem_1_input_failure(
    const vector3<Scalar>& omega,
    const vector3<Scalar>& u,
    const vector3<Scalar>& u_prime)
{
    if (!omega.allFinite() || !u.allFinite() || !u_prime.allFinite())
        return analytical_failure::non_finite_input;
    if (std::abs(omega.norm() - Scalar(1)) > sqrt_epsilon_v<Scalar>)
        return analytical_failure::degenerate_geometry;
    return std::nullopt;
}

/// The two scalar conditions a rotation about omega imposes on a displacement
/// pair. A rotation is the identity on the axial part of a displacement and a
/// planar rotation on the perpendicular part, so the axial components must
/// agree and the perpendicular radii must agree; failing either means no angle
/// solves the equation.
template <typename Scalar>
bool rotation_conditions_hold(
    const vector3<Scalar>& omega,
    const vector3<Scalar>& u,
    const vector3<Scalar>& u_prime,
    Scalar tolerance)
{
    Scalar radius = (u - omega.dot(u) * omega).norm();
    Scalar radius_prime = (u_prime - omega.dot(u_prime) * omega).norm();
    return std::abs(omega.dot(u_prime - u)) <= tolerance
        && std::abs(radius - radius_prime) <= tolerance;
}

/// Residual of the original equation, which the two scalar conditions do not
/// imply: it is what catches a sign or branch error and accumulated round-off.
template <typename Scalar>
bool subproblem_1_reconstructs(
    const vector3<Scalar>& omega,
    const vector3<Scalar>& u,
    const vector3<Scalar>& u_prime,
    Scalar theta,
    Scalar tolerance)
{
    vector3<Scalar> origin = vector3<Scalar>::Zero();
    vector3<Scalar> rotated = rotate_point_about_axis(omega, origin, u, theta);
    return (rotated - u_prime).norm() <= tolerance;
}

/// Paden-Kahan subproblem 1 over displacements from the axis point: find theta
/// with exp([omega]*theta) mapping u to u_prime, the axis passing through the
/// origin. Shared by both public entry points so the solvability conditions
/// exist once; the threshold is bare because each entry point carries the unit.
///
/// Reference: Murray, Li and Sastry, A Mathematical Introduction to Robotic
/// Manipulation, CRC Press 1994, Chapter 3 Section 3.3, Subproblem 1.
template <typename Scalar>
cartan::expected<Scalar, analytical_failure> subproblem_1(
    const vector3<Scalar>& omega,
    const vector3<Scalar>& u,
    const vector3<Scalar>& u_prime,
    Scalar tolerance)
{
    if (auto failure = subproblem_1_input_failure(omega, u, u_prime))
        return cartan::unexpected(*failure);
    if (!rotation_conditions_hold(omega, u, u_prime, tolerance))
        return cartan::unexpected(analytical_failure::unreachable);

    vector3<Scalar> u_perp = u - omega.dot(u) * omega;
    vector3<Scalar> u_prime_perp = u_prime - omega.dot(u_prime) * omega;
    if (u_perp.norm() <= tolerance && u_prime_perp.norm() <= tolerance)
        return cartan::unexpected(analytical_failure::singular_configuration);

    Scalar theta = std::atan2(
        omega.dot(u_perp.cross(u_prime_perp)), u_perp.dot(u_prime_perp));
    if (!subproblem_1_reconstructs(omega, u, u_prime, theta, tolerance))
        return cartan::unexpected(analytical_failure::unreachable);
    return theta;
}

}

#endif
