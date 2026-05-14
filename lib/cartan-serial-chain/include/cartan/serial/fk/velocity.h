#ifndef HPP_GUARD_CARTAN_SERIAL_FK_VELOCITY_H
#define HPP_GUARD_CARTAN_SERIAL_FK_VELOCITY_H

/// End-effector velocity kinematics.
///
/// Computes end-effector spatial twist from joint positions and velocities
/// using the space Jacobian.
///
/// Reference: Lynch & Park, Modern Robotics, Eq. 5.10, p. 178.

#include "cartan/serial/fk/jacobian.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include "cartan/serial/chain/joint_state.h"

namespace cartan
{

/// End-effector spatial twist: V_s = J_s(q) * dq.
///
/// Computes forward kinematics internally to obtain the space Jacobian,
/// then multiplies by joint velocities.
///
/// Reference: Lynch & Park, Modern Robotics, Eq. 5.10, p. 178.
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

}

#endif
