#ifndef HPP_GUARD_CARTAN_LIE_QUATERNION_UTILS_H
#define HPP_GUARD_CARTAN_LIE_QUATERNION_UTILS_H

/// @file quaternion_utils.h
/// @brief Quaternion utility functions supplementing Eigen's quaternion type.
///
/// These functions delegate to Eigen where possible, following the
/// "Don't Hand-Roll" principle from the cartan design research (D-12).
/// Named constructors from_wxyz / from_xyzw / to_wxyz provide explicit
/// serialization order per ctrlpp convention (w-first external API).
///
/// Reference: Eigen quaternion documentation; Barfoot, State Estimation
///            for Robotics, Section 8.1, p. 280-282.

#include "cartan/types.h"

namespace cartan
{

/// Spherical linear interpolation between two unit quaternions.
/// Delegates to Eigen's slerp implementation which handles antipodal quaternions.
/// @param q1 Start quaternion (unit)
/// @param q2 End quaternion (unit)
/// @param t  Interpolation parameter in [0, 1]
/// @return Interpolated unit quaternion
/// Reference: Shoemake, "Animating rotation with quaternion curves", SIGGRAPH 1985.
template <typename Scalar>
[[nodiscard]] quaternion<Scalar> quat_slerp(
    const quaternion<Scalar>& q1,
    const quaternion<Scalar>& q2,
    Scalar t)
{
    return q1.slerp(t, q2);
}

/// Normalize a quaternion to unit length.
/// Delegates to Eigen's normalized().
/// Reference: Unit quaternion constraint ||q|| = 1 for rotation representation.
template <typename Scalar>
[[nodiscard]] quaternion<Scalar> quat_normalize(const quaternion<Scalar>& q)
{
    return q.normalized();
}

/// Convert unit quaternion to 3x3 rotation matrix.
/// Delegates to Eigen's toRotationMatrix().
/// Reference: Barfoot, State Estimation for Robotics, Eq. 8.3, p. 281.
template <typename Scalar>
[[nodiscard]] matrix3<Scalar> quat_to_matrix(const quaternion<Scalar>& q)
{
    return q.toRotationMatrix();
}

/// Convert 3x3 rotation matrix to quaternion.
/// Delegates to Eigen's quaternion constructor from rotation matrix.
/// Reference: Barfoot, State Estimation for Robotics, Eq. 8.3 (inverse).
template <typename Scalar>
[[nodiscard]] quaternion<Scalar> matrix_to_quat(const matrix3<Scalar>& R)
{
    return quaternion<Scalar>(R);
}

/// Named quaternion constructor: w-first order (w, x, y, z).
/// This matches the ctrlpp external API convention (D-12).
/// Eigen's quaternion constructor is also w-first: Quaternion(w, x, y, z).
/// Reference: Hamilton convention; Barfoot, Section 8.1.
template <typename Scalar>
[[nodiscard]] quaternion<Scalar> from_wxyz(Scalar w, Scalar x, Scalar y, Scalar z)
{
    return quaternion<Scalar>(w, x, y, z);
}

/// Named quaternion constructor: xyzw order (x, y, z, w).
/// Matches Eigen's internal coefficient storage order [x, y, z, w].
/// Reorders to Eigen's constructor which expects (w, x, y, z).
/// Reference: Eigen documentation on quaternion storage layout.
template <typename Scalar>
[[nodiscard]] quaternion<Scalar> from_xyzw(Scalar x, Scalar y, Scalar z, Scalar w)
{
    return quaternion<Scalar>(w, x, y, z);
}

/// Serialize quaternion to [w, x, y, z] 4-vector per ctrlpp convention (D-12).
/// Reference: Hamilton convention output format.
template <typename Scalar>
[[nodiscard]] vector<Scalar, 4> to_wxyz(const quaternion<Scalar>& q)
{
    vector<Scalar, 4> v;
    v << q.w(), q.x(), q.y(), q.z();
    return v;
}

/// Hamilton quaternion product: q1 * q2.
/// Delegates to Eigen's operator* which uses Hamilton convention.
/// Reference: Hamilton, "On quaternions" (1843); Barfoot, Eq. 8.6, p. 282.
template <typename Scalar>
[[nodiscard]] quaternion<Scalar> quat_hamilton_product(
    const quaternion<Scalar>& q1,
    const quaternion<Scalar>& q2)
{
    return q1 * q2;
}

}

#endif
