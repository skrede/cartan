#ifndef HPP_GUARD_LIEPP_KINEMATICS_VELOCITY_H
#define HPP_GUARD_LIEPP_KINEMATICS_VELOCITY_H

/// @file velocity.h
/// @brief End-effector velocity kinematics.
///
/// Computes end-effector spatial twist from joint positions and velocities
/// using the space Jacobian.
///
/// Reference: Lynch & Park, Modern Robotics, Eq. 5.10, p. 178.

#include "liepp/kinematics/jacobian.h"
#include "liepp/kinematics/forward_kinematics.h"

#include "liepp/chain/joint_state.h"

namespace liepp
{

/// End-effector spatial twist: V_s = J_s(q) * dq.
///
/// Computes forward kinematics internally to obtain the space Jacobian,
/// then multiplies by joint velocities.
///
/// Reference: Lynch & Park, Modern Robotics, Eq. 5.10, p. 178.
///
/// @tparam N      Number of joints (compile-time), or liepp::dynamic.
/// @tparam Scalar Floating-point type.
/// @param chain   Kinematic chain with screw axes and home configuration.
/// @param q       Joint position vector.
/// @param dq      Joint velocity vector.
/// @return        6-vector spatial twist V_s.
template <typename Scalar, int N>
vector6<Scalar> end_effector_velocity(
    const kinematic_chain<Scalar, N>& chain,
    const typename joint_state<Scalar, N>::position_type& q,
    const typename joint_state<Scalar, N>::velocity_type& dq)
{
    auto fk = forward_kinematics(chain, q);
    auto J_s = space_jacobian(chain, fk);
    return J_s * dq;
}

} // namespace liepp

#endif
