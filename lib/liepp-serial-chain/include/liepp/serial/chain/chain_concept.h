#ifndef HPP_GUARD_LIEPP_SERIAL_CHAIN_CHAIN_CONCEPT_H
#define HPP_GUARD_LIEPP_SERIAL_CHAIN_CHAIN_CONCEPT_H

/// @file chain_concept.h
/// @brief Concept constraining serial chain types for FK/Jacobian/IK consumers.
///
/// The chain concept captures the minimal surface needed by forward kinematics,
/// Jacobian computation, and IK solvers. Designed top-down from consumer needs:
/// home configuration, per-joint axis access, bulk axes/limits access, joint count,
/// scalar type, and compile-time joint count constant.

#include "liepp/serial/chain/screw_axis.h"

#include "liepp/lie/se3.h"

#include <concepts>

namespace liepp
{

/// Concept for a serial kinematic chain.
///
/// Requires the minimal interface consumed by FK, Jacobian, and IK:
///   - scalar_type: the floating-point type used throughout
///   - joints: static constexpr int (liepp::dynamic or compile-time count)
///   - home(): the end-effector home configuration (M matrix)
///   - num_joints(): runtime joint count
///   - axis(i): per-element screw axis access
///   - axes(): bulk screw axis container (unconstrained return type)
///   - limits(): bulk joint limits container (unconstrained return type)
template <typename C>
concept chain = requires(const C& c, int i)
{
    typename C::scalar_type;
    { C::joints } -> std::convertible_to<int>;
    { c.home() } -> std::convertible_to<const se3<typename C::scalar_type>&>;
    { c.num_joints() } -> std::convertible_to<int>;
    { c.axis(i) } -> std::convertible_to<screw_axis<typename C::scalar_type>>;
    { c.axes() };
    { c.limits() };
};

}

#endif
