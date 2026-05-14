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

    // Rotation (possibly with translation along axis)
    vector3<Scalar> omega_hat = tw.omega / omega_norm;
    Scalar h = omega_hat.dot(tw.v);
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

/// Reconstruct a twist from screw motion parameters.
/// For rotation: omega = theta * s_hat, v = (s_hat x q) * theta + h * theta * s_hat.
/// Wait -- reconstruct the original twist, not a unit twist times theta.
/// Original twist: omega = theta * s_hat (or if we stored the twist as unit * theta).
/// Actually, the twist stores omega and v directly. From screw_motion:
///   omega = theta * s_hat
///   v = -s_hat.cross(q) * theta + h * theta * s_hat (for finite pitch)
/// For pure translation (theta=0): omega = 0, v = d * s_hat.
/// Reference: Lynch & Park, Modern Robotics, Section 3.3.2, p. 95-102.
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

    // Rotation case: reconstruct omega and v
    tw.omega = sm.theta * sm.axis.s_hat;

    // v = omega_hat x (omega_hat x q) * omega_norm + h * omega
    // Simpler: v = -omega_hat x q * omega_norm + h * omega
    // Actually from the forward direction:
    //   omega_hat = omega / |omega|
    //   h = omega_hat . v
    //   q = omega_hat x v / |omega|
    // So: omega_hat x v = q * |omega|
    // And: v = omega_hat x (q * |omega|) + h * omega_hat * |omega|
    //        = |omega| * (omega_hat x q) + h * omega
    // Wait, cross product: omega_hat x v = |omega| * q
    // So v = ? Let's derive directly:
    //   v has component along omega_hat: h (pitch)
    //   v has component perpendicular: omega_hat x (|omega| * q) = ...
    // Actually: v = h * omega_hat * |omega| + |omega| * omega_hat x q
    //           = |omega| * (h * omega_hat + omega_hat x q)
    // But |omega| = theta here, and omega_hat = s_hat
    Scalar omega_norm = sm.theta;
    tw.v = sm.axis.h * sm.axis.s_hat * omega_norm
         + omega_norm * sm.axis.s_hat.cross(sm.axis.q);

    // Simplify: v = theta * (h * s_hat + s_hat x q)
    // But we need to verify: omega_hat x v / |omega| = q
    // omega_hat x v = omega_hat x [theta * (h * s_hat + s_hat x q)]
    //               = theta * (h * s_hat x s_hat + s_hat x (s_hat x q))
    //               = theta * (0 + (s_hat . q) s_hat - q)
    //               = theta * ((s_hat . q) s_hat - q)
    // Hmm, this doesn't simplify nicely. Let me use the direct formula:
    // From the forward path:
    //   q = omega_hat.cross(v) / omega_norm
    //   h = omega_hat.dot(v)
    // The vector v decomposes as:
    //   v = h * omega_hat + omega_hat.cross(q * omega_norm)
    //     = h * omega_hat + omega_norm * omega_hat.cross(q)
    // But this would give us:
    //   omega_hat.cross(v) = omega_hat.cross(omega_norm * omega_hat.cross(q))
    //                      = omega_norm * ((omega_hat . q) omega_hat - q)  [BAC-CAB]
    // If q is perpendicular to omega_hat (which it should be since q = omega_hat x v / |omega|),
    // then omega_hat . q = 0, so:
    //   omega_hat.cross(v) = -omega_norm * q
    // Then q = omega_hat.cross(v) / omega_norm = -q ??? That's wrong.
    //
    // Let me re-derive. From to_screw_motion:
    //   q = omega_hat.cross(v) / omega_norm
    // And omega_norm = |omega| = theta (since omega = theta * s_hat for unit s_hat).
    // Wait no: the twist stores omega directly, so |omega| could be anything.
    //
    // Actually in to_screw_motion, omega_norm = tw.omega.norm(), and
    //   q = omega_hat.cross(tw.v) / omega_norm
    // So tw.v can be reconstructed: we know h and q.
    //   tw.v_parallel = h * omega_hat     (component along omega)
    //   tw.v_perp = tw.v - tw.v_parallel  (perpendicular component)
    // q = omega_hat.cross(tw.v_perp) / omega_norm  (since omega_hat.cross(tw.v_parallel) = 0)
    //   = omega_hat.cross(tw.v_perp) / omega_norm
    // So tw.v_perp = -omega_norm * omega_hat.cross(q)  (from BAC-CAB: a x (a x b) = (a.b)a - b for unit a)
    // Wait: omega_hat x (omega_hat x v_perp) = (omega_hat . v_perp) omega_hat - v_perp = -v_perp
    // And: omega_hat x (q * omega_norm) = omega_norm * omega_hat x q
    // We need: omega_hat x v_perp = omega_norm * q
    // So: v_perp = -omega_hat x (omega_norm * q) = -omega_norm * (omega_hat x q)
    // (Using the identity: if a x b = c, then b = -(a x c) when a is unit and b perp to a)
    // Therefore: tw.v = h * omega_hat - omega_norm * (omega_hat x q)
    tw.v = sm.axis.h * sm.axis.s_hat
         - sm.theta * sm.axis.s_hat.cross(sm.axis.q);

    return tw;
}

}

#endif
