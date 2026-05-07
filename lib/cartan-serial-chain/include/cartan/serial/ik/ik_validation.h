#ifndef HPP_GUARD_CARTAN_SERIAL_IK_IK_VALIDATION_H
#define HPP_GUARD_CARTAN_SERIAL_IK_IK_VALIDATION_H

/// @file ik_validation.h
/// @brief FK-based validation free functions for IK results.

#include "cartan/serial/ik/ik_result.h"
#include "cartan/serial/ik/ik_status.h"

#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/chain_concept.h"

#include "cartan/serial/fk/forward_kinematics.h"

#include <algorithm>
#include <vector>

namespace cartan
{

template <chain Chain>
[[nodiscard]] bool verify_solution(
    const Chain& chain,
    const se3<typename Chain::scalar_type>& target,
    const typename joint_state<typename Chain::scalar_type, Chain::joints>::position_type& q,
    const convergence_criteria<typename Chain::scalar_type>& criteria)
{
    auto fk = forward_kinematics(chain, q);
    auto V_b = (fk.end_effector.inverse() * target).log();
    return V_b.template head<3>().norm() < criteria.orientation_tol
        && V_b.template tail<3>().norm() < criteria.position_tol;
}

template <chain Chain, typename Scalar, int N>
[[nodiscard]] std::vector<ik_result<Scalar, N>> filter_valid_solutions(
    const Chain& chain,
    const se3<Scalar>& target,
    std::vector<ik_result<Scalar, N>> solutions,
    const convergence_criteria<Scalar>& criteria)
{
    std::erase_if(solutions, [&](const ik_result<Scalar, N>& r)
    {
        return !verify_solution(chain, target, r.solution.position, criteria);
    });
    return solutions;
}

}

#endif
