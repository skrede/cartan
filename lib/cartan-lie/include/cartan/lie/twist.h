#ifndef HPP_GUARD_CARTAN_LIE_TWIST_H
#define HPP_GUARD_CARTAN_LIE_TWIST_H

/// Twist (spatial velocity) representation and screw motion parameters.
/// A twist V = (omega, v) is a 6-vector in se(3) using omega-first convention
/// (per Lynch & Park). The omega part is the angular velocity, v is the
/// linear velocity. This module provides the twist struct, SE(3) conversions,
/// and screw motion parameter decomposition.
///
/// Reference: Lynch & Park, Modern Robotics, Section 3.3, p. 86-106.
///            Barfoot, State Estimation for Robotics, Ch. 8, p. 280-300.

#include "cartan/types.h"
#include "cartan/detail/epsilon.h"

#include "cartan/lie/se3.h"
#include "cartan/lie/axis_angle.h"

#include <cmath>

namespace cartan
{

/// Twist (spatial velocity) representation: V = (omega, v).
/// omega is the angular velocity (3-vector), v is the linear velocity (3-vector).
/// Uses omega-first convention per Lynch & Park, Modern Robotics, p. 86-106.
template <typename Scalar>
struct twist
{
    vector3<Scalar> omega;  ///< Angular velocity (omega-first per Lynch & Park)
    vector3<Scalar> v;      ///< Linear velocity

    /// Construct from 6-vector (omega-first).
    /// Reference: Lynch & Park, Modern Robotics, Eq. 3.82, p. 103.
    [[nodiscard]] static twist from_vector(const vector6<Scalar>& vec)
    {
        twist tw;
        tw.omega = vec.template head<3>();
        tw.v = vec.template tail<3>();
        return tw;
    }

    /// Convert to 6-vector (omega-first).
    /// Reference: Lynch & Park, Modern Robotics, Eq. 3.82, p. 103.
    [[nodiscard]] vector6<Scalar> to_vector() const
    {
        vector6<Scalar> vec;
        vec.template head<3>() = omega;
        vec.template tail<3>() = v;
        return vec;
    }
};

/// Compute the rigid body motion from a unit twist applied for angle/distance theta.
/// Returns se3::exp(theta * twist_vector).
/// Reference: Lynch & Park, Modern Robotics, Prop. 3.22, p. 99.
template <typename Scalar>
[[nodiscard]] se3<Scalar> twist_to_se3(const twist<Scalar>& tw, Scalar theta)
{
    return se3<Scalar>::exp(theta * tw.to_vector());
}

/// Extract twist from SE(3) via the log map.
/// Returns twist::from_vector(T.log()). The twist encodes the axis-angle and
/// linear components of the rigid body displacement.
/// Reference: Lynch & Park, Modern Robotics, Eq. 3.91-3.92, p. 104.
///            Barfoot, State Estimation for Robotics, Eq. 8.35, p. 290.
template <typename Scalar, typename Policy>
[[nodiscard]] twist<Scalar> se3_to_twist(const se3<Scalar, Policy>& T)
{
    return twist<Scalar>::from_vector(T.log());
}

/// Screw motion parameters decomposed from a twist.
/// Represents the motion as rotation of angle theta about a screw axis,
/// with translation distance d along the axis.
/// Reference: Lynch & Park, Modern Robotics, Section 3.3.2, p. 95-102.
template <typename Scalar>
struct screw_motion
{
    screw_params<Scalar> axis;  ///< Screw parameters (from axis_angle.h)
    Scalar theta;             ///< Rotation angle (radians)
    Scalar d;                 ///< Translation distance along axis
};

/// Decompose a twist into screw motion parameters.
/// For rotation (|omega| > epsilon): theta = |omega|, d = h * theta.
/// For pure translation (|omega| ~ 0): theta = 0, d = |v|.
/// Reference: Lynch & Park, Modern Robotics, Def. 3.24, p. 102.
template <typename Scalar>
[[nodiscard]] screw_motion<Scalar> to_screw_motion(const twist<Scalar>& tw)
{
    Scalar omega_norm = tw.omega.norm();

    if (omega_norm < detail::epsilon_v<Scalar>)
    {
        // Pure translation
        Scalar d = tw.v.norm();
        vector3<Scalar> dir = (d > detail::epsilon_v<Scalar>)
            ? vector3<Scalar>(tw.v / d)
            : vector3<Scalar>::UnitX();

        return screw_motion<Scalar>{
            screw_params<Scalar>{
                vector3<Scalar>::Zero(),
                dir,
                std::numeric_limits<Scalar>::infinity()
            },
            Scalar(0),
            d
        };
    }

    // Rotation (possibly with translation along axis).
    // Modern Robotics Def. 3.24: pitch h = (omega_hat . v)/|omega|.
    vector3<Scalar> omega_hat = tw.omega / omega_norm;
    Scalar h = omega_hat.dot(tw.v) / omega_norm;
    Scalar theta = omega_norm;
    Scalar d = h * theta;

    // Point on the screw axis: q = omega_hat x v / |omega|
    // When omega is unit: q = omega_hat x v
    // But for non-unit omega in general twist: q = omega_hat x v / omega_norm
    vector3<Scalar> q = omega_hat.cross(tw.v) / omega_norm;

    return screw_motion<Scalar>{
        screw_params<Scalar>{q, omega_hat, h},
        theta,
        d
    };
}

/// Reconstruct a twist from screw motion parameters (inverse of to_screw_motion).
/// For rotation (theta = |omega| > 0):
///   omega = theta * s_hat
///   v = h * theta * s_hat - theta * (s_hat x q)
/// The parallel term h * theta * s_hat carries the pitch (v_parallel =
/// (s_hat . v) s_hat = h * theta * s_hat), and the perpendicular term
/// -theta * (s_hat x q) inverts the point relation q = (s_hat x v)/|omega|.
/// For pure translation (theta = 0): omega = 0, v = d * s_hat.
/// Reference: Lynch & Park, Modern Robotics, Def. 3.24, p. 102.
template <typename Scalar>
[[nodiscard]] twist<Scalar> from_screw_motion(const screw_motion<Scalar>& sm)
{
    twist<Scalar> tw;

    if (sm.theta < detail::epsilon_v<Scalar>)
    {
        // Pure translation
        tw.omega = vector3<Scalar>::Zero();
        tw.v = sm.d * sm.axis.s_hat;
        return tw;
    }

    // Rotation case: omega = theta * s_hat, and v splits into the pitch-aligned
    // parallel component and the axis-offset perpendicular component.
    tw.omega = sm.theta * sm.axis.s_hat;
    tw.v = sm.axis.h * sm.theta * sm.axis.s_hat
         - sm.theta * sm.axis.s_hat.cross(sm.axis.q);

    return tw;
}

}

#endif
