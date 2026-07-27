#ifndef HPP_GUARD_CARTAN_SERIAL_CHAIN_JOINT_KIND_H
#define HPP_GUARD_CARTAN_SERIAL_CHAIN_JOINT_KIND_H

/// Runtime axis classification for kinematic_chain fast-path dispatch.
///
/// joint_kind labels a screw_axis according to whether it is axis-aligned
/// with one of {±e_x, ±e_y, ±e_z}. kinematic_chain caches the detected kind
/// per joint at construction so the FK/Jacobian inner loops can branch into
/// the same compile-time specializations used by static_chain.

#include "cartan/types.h"

#include "cartan/serial/chain/screw_axis.h"

#include <cmath>
#include <cstdint>

namespace cartan
{

/// Axis classification for kinematic_chain runtime dispatch.
///
/// general is the catch-all for arbitrary screw axes that do not match any
/// principal-axis pattern; it routes back to the generic se3::exp path.
enum class joint_kind : std::uint8_t
{
    general = 0,
    revolute_x,
    revolute_y,
    revolute_z,
    prismatic_x,
    prismatic_y,
    prismatic_z,
};

namespace detail
{

/// Per-component deviation an axis may carry and still be snapped onto a
/// principal axis.
///
/// Absolute, dimensionless and independent of Scalar by intent. A threshold
/// derived from machine precision makes the same code safe in double and
/// unsafe in float, where sqrt(epsilon) is 3.45e-4 -- a misalignment of about
/// a hundredth of a degree, silently discarded. The compared quantity is an
/// unsquared per-component deviation in the axis components' own units, which
/// for a near-principal unit axis is the misalignment angle in radians to
/// first order, so the value below is read directly as an angle.
///
/// It sits 2.44x above the largest deviation the fixture set requires to be
/// snapped (4.102071e-10, bit-identical in both scalars) and 3.0e5 times below
/// the smallest deviation that must not be. The error a snap induces was
/// measured over 400 random configurations on three description-derived chains
/// (6R/0.50 m, 7R/0.85 m, 7R/1.30 m):
///
///     |dp| <= 1.4 * n * L * delta      |dtheta| <= 1.7 * n * delta
///
/// with n the joint count, L the maximum moment arm and delta this tolerance;
/// the analytic worst case is sqrt(2)*pi, about 4.44. On the largest of those
/// chains the value below licenses about 13 nm of end-effector error, an angle
/// no robot axis can be specified to.
///
/// One surprising property, because it will bite whoever relaxes the unit
/// assumption: below about 6e-8 the unit test is exact in single precision, as
/// no representable float lies between 1 and 1 + 1.19e-7. That is harmless
/// only while every axis reaching here is normalized, which makes the on-axis
/// component exactly one; the off-axis test keeps real tolerance either way.
///
/// Both margins are properties of the fixture set and of the description
/// parser's arithmetic, so re-measure them if either changes and lock the
/// result here; the two-sided target in
/// tests/boundary/axis_classification_test.cpp pins the margins but does not
/// re-derive the induced-error law.
template <typename Scalar>
inline constexpr Scalar k_axis_snap_tolerance_v = Scalar(1e-9);

}

/// Detect the joint_kind of a screw axis.
///
/// Recognizes axes whose omega (revolute) or v (prismatic) is +/-e_x, +/-e_y
/// or +/-e_z to within the axis-snap tolerance. A +/-e_k axis and its negation
/// map to the same joint_kind; the downstream specializations recover the sign
/// from the axis itself -- the signed component for revolute joints, and the
/// signed screw_axis::v() direction for prismatic joints.
/// All other axes, and every nonfinite one, return joint_kind::general.
template <typename Scalar>
inline joint_kind detect_joint_kind(const screw_axis<Scalar>& axis)
{
    auto is_unit = [](Scalar x)
    {
        return std::abs(std::abs(x) - Scalar(1))
               < detail::k_axis_snap_tolerance_v<Scalar>;
    };
    auto is_zero = [](Scalar x)
    {
        return std::abs(x) < detail::k_axis_snap_tolerance_v<Scalar>;
    };

    const auto& w = axis.omega();

    if (axis.is_revolute())
    {
        if (is_unit(w(0)) && is_zero(w(1)) && is_zero(w(2))) return joint_kind::revolute_x;
        if (is_zero(w(0)) && is_unit(w(1)) && is_zero(w(2))) return joint_kind::revolute_y;
        if (is_zero(w(0)) && is_zero(w(1)) && is_unit(w(2))) return joint_kind::revolute_z;
        return joint_kind::general;
    }

    const auto& v = axis.v();
    if (is_unit(v(0)) && is_zero(v(1)) && is_zero(v(2))) return joint_kind::prismatic_x;
    if (is_zero(v(0)) && is_unit(v(1)) && is_zero(v(2))) return joint_kind::prismatic_y;
    if (is_zero(v(0)) && is_zero(v(1)) && is_unit(v(2))) return joint_kind::prismatic_z;
    return joint_kind::general;
}

}

#endif
