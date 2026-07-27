#ifndef HPP_GUARD_CARTAN_SERIAL_FK_VELOCITY_H
#define HPP_GUARD_CARTAN_SERIAL_FK_VELOCITY_H

/// End-effector velocity kinematics.
///
/// Computes end-effector spatial twist from joint positions and velocities
/// using the space Jacobian.
///
/// Reference: Lynch & Park, Modern Robotics, Eq. 5.10, p. 178.

#include "cartan/serial/fk/jacobian.h"
#include "cartan/serial/fk/detail/shape_validation.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include "cartan/serial/chain/joint_state.h"

namespace cartan
{

/// End-effector spatial twist for a caller that has already established that
/// q and dq both hold exactly chain.num_joints() finite components. Neither
/// precondition is checked here, and violating either is undefined behavior:
/// a vector shorter than the joint count reads past its end; an over-long q is
/// truncated to the joint count; an over-long dq is truncated in double but
/// reads past the last Jacobian column in float, because the product's
/// vectorized evaluator traverses the operand rather than the matrix.
template <typename Scalar, int N>
vector6<Scalar> end_effector_velocity_unchecked(
    const kinematic_chain<Scalar, N>& chain,
    const typename joint_state<Scalar, N>::position_type& q,
    const typename joint_state<Scalar, N>::velocity_type& dq)
{
    auto fk = forward_kinematics_unchecked(chain, q);
    auto J_s = space_jacobian_unchecked(chain, fk);
    return J_s * dq;
}

/// End-effector spatial twist: V_s = J_s(q) * dq.
///
/// Computes forward kinematics internally to obtain the space Jacobian,
/// then multiplies by joint velocities.
///
/// Both vectors are taken at the caller's own type and validated before
/// anything converts them, for the reason spelled out over
/// `forward_kinematics`. Joint positions are established before joint
/// velocities, so a call violating both contracts reports the position
/// failure.
template <typename Scalar, int N, typename QDerived, typename DqDerived>
cartan::expected<vector6<Scalar>, chain_failure> end_effector_velocity(
    const kinematic_chain<Scalar, N>& chain,
    const Eigen::MatrixBase<QDerived>& q,
    const Eigen::MatrixBase<DqDerived>& dq)
{
    if (auto positions = detail::check_joint_positions(chain, q); !positions)
    {
        return cartan::unexpected(positions.error());
    }
    return detail::guarded(detail::check_joint_velocities(chain, dq),
        [&] { return end_effector_velocity_unchecked(chain, q.derived(), dq.derived()); });
}

}

#endif
