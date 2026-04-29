#ifndef HPP_GUARD_CARTAN_SERIAL_CHAIN_JOINT_KIND_H
#define HPP_GUARD_CARTAN_SERIAL_CHAIN_JOINT_KIND_H

/// @file joint_kind.h
/// @brief Runtime axis classification for kinematic_chain fast-path dispatch.
///
/// joint_kind labels a screw_axis according to whether it is axis-aligned
/// with one of {±e_x, ±e_y, ±e_z}. kinematic_chain caches the detected kind
/// per joint at construction so the FK/Jacobian inner loops can branch into
/// the same compile-time specializations used by static_chain.

#include "cartan/types.h"
#include "cartan/detail/epsilon.h"

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

/// Detect the joint_kind of a screw axis.
///
/// Recognizes axes whose omega (revolute) or v (prismatic) is exactly ±e_x,
/// ±e_y, or ±e_z within sqrt-epsilon. The sign is irrelevant: the
/// downstream specializations read the magnitude from the axis itself.
/// All other axes return joint_kind::general.
template <typename Scalar>
[[nodiscard]] inline joint_kind detect_joint_kind(const screw_axis<Scalar>& axis)
{
    const Scalar tol = detail::sqrt_epsilon_v<Scalar>;
    auto is_unit = [tol](Scalar x) { return std::abs(std::abs(x) - Scalar(1)) < tol; };
    auto is_zero = [tol](Scalar x) { return std::abs(x) < tol; };

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
