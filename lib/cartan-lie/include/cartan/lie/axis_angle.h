#ifndef HPP_GUARD_CARTAN_LIE_AXIS_ANGLE_H
#define HPP_GUARD_CARTAN_LIE_AXIS_ANGLE_H

/// Axis-angle representation and screw parameters for SO(3). Provides
/// conversion between axis-angle and SO(3) via exp/log maps, and screw
/// parameter extraction from twist (omega, v) pairs.
///
/// Reference: Lynch & Park, Modern Robotics, Section 3.2.3, p. 77-86 (axis-angle).
///            Lynch & Park, Modern Robotics, Def. 3.24, p. 102 (screw axis).
///            Barfoot, State Estimation for Robotics, Ch. 8, p. 280-300.

#include "cartan/types.h"
#include "cartan/detail/epsilon.h"

#include "cartan/lie/so3.h"

#include <cmath>
#include <limits>

namespace cartan
{

/// Axis-angle representation of a 3D rotation.
/// axis is the unit rotation axis, angle is the rotation magnitude in radians.
/// Reference: Lynch & Park, Modern Robotics, Section 3.2.3, p. 77-86.
template <typename Scalar>
struct axis_angle
{
    vector3<Scalar> axis;   ///< Unit rotation axis
    Scalar angle;           ///< Rotation angle in radians [0, pi]
};

/// Screw parameters for a rigid body motion.
/// Represents the geometric screw: a line in space (point q, direction s_hat)
/// with pitch h (ratio of linear to angular motion).
/// Reference: Lynch & Park, Modern Robotics, Def. 3.24, p. 102.
template <typename Scalar>
struct screw_params
{
    vector3<Scalar> q;       ///< Point on the screw axis
    vector3<Scalar> s_hat;   ///< Unit direction of screw axis
    Scalar h;                ///< Pitch (0 = pure rotation, infinity = pure translation)
};

/// Extract axis-angle representation from SO(3) via the log map.
/// For theta near zero, returns zero angle with UnitX as arbitrary axis.
/// Reference: Lynch & Park, Modern Robotics, Section 3.2.3, p. 77-86.
///            Barfoot, State Estimation for Robotics, Eq. 8.22, p. 284.
template <typename Scalar, typename Policy>
[[nodiscard]] axis_angle<Scalar> to_axis_angle(const so3<Scalar, Policy>& r)
{
    vector3<Scalar> phi = r.log();
    Scalar theta = phi.norm();

    if (theta < detail::epsilon_v<Scalar>)
    {
        return axis_angle<Scalar>{vector3<Scalar>::UnitX(), Scalar(0)};
    }

    return axis_angle<Scalar>{phi / theta, theta};
}

/// Convert axis-angle representation to SO(3) via the exp map.
/// Reference: Lynch & Park, Modern Robotics, Prop. 3.11, p. 82.
template <typename Scalar>
[[nodiscard]] so3<Scalar> from_axis_angle(const axis_angle<Scalar>& aa)
{
    return so3<Scalar>::exp(aa.angle * aa.axis);
}

/// Parse an angle-axis vector (phi = theta * axis) into axis-angle components.
/// Equivalent to extracting components from the log map output.
/// Reference: Lynch & Park, Modern Robotics, Section 3.2.3, p. 77-86.
template <typename Scalar>
[[nodiscard]] axis_angle<Scalar> from_angle_axis_vector(const vector3<Scalar>& phi)
{
    Scalar theta = phi.norm();

    if (theta < detail::epsilon_v<Scalar>)
    {
        return axis_angle<Scalar>{vector3<Scalar>::UnitX(), Scalar(0)};
    }

    return axis_angle<Scalar>{phi / theta, theta};
}

/// Extract screw parameters from twist components (omega, v).
/// For rotation (|omega| > 0): s_hat = omega/|omega|,
///   pitch h = (s_hat . v)/|omega|, point q = (s_hat x v)/|omega|.
/// For pure translation (|omega| ~= 0): s_hat = v/|v|, q = zero, h = infinity.
/// Reference: Lynch & Park, Modern Robotics, Def. 3.24, p. 102.
template <typename Scalar>
[[nodiscard]] screw_params<Scalar> to_screw_params(
    const vector3<Scalar>& omega,
    const vector3<Scalar>& v)
{
    Scalar omega_norm = omega.norm();

    if (omega_norm < detail::epsilon_v<Scalar>)
    {
        // Pure translation: infinite pitch
        Scalar v_norm = v.norm();
        vector3<Scalar> dir = (v_norm > detail::epsilon_v<Scalar>)
            ? vector3<Scalar>(v / v_norm)
            : vector3<Scalar>::UnitX();
        return screw_params<Scalar>{
            vector3<Scalar>::Zero(),
            dir,
            std::numeric_limits<Scalar>::infinity()
        };
    }

    // Rotation (possibly with translation along axis).
    // Modern Robotics Def. 3.24: h = (s_hat . v)/|omega|, q = (s_hat x v)/|omega|.
    vector3<Scalar> omega_hat = omega / omega_norm;
    Scalar h = omega_hat.dot(v) / omega_norm;
    vector3<Scalar> q = omega_hat.cross(v) / omega_norm;

    return screw_params<Scalar>{q, omega_hat, h};
}

}

#endif
