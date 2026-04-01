#ifndef HPP_GUARD_LIEPP_SERIAL_IK_IK_SOLVE_POLICY_H
#define HPP_GUARD_LIEPP_SERIAL_IK_IK_SOLVE_POLICY_H

/// @file ik_solve_policy.h
/// @brief Single-parameter concept for IK solve policies.
///
/// The ik_solve_policy concept replaces the three-parameter ik_stepper concept.
/// It extracts Scalar, N, and LimitsPolicy from nested type traits on the policy
/// itself (scalar_type, joints, limits_type), enabling CTAD and simpler composition.
///
/// Reference: Decision D-09.

#include "liepp/serial/ik/ik_types.h"

#include "liepp/lie/se3.h"
#include "liepp/serial/chain/joint_state.h"
#include "liepp/serial/chain/kinematic_chain.h"

#include <concepts>

namespace liepp
{

/// Concept for a single-parameter IK solve policy.
///
/// A conforming type S must expose:
///   - S::scalar_type    (floating-point type)
///   - S::joints         (static constexpr int, joint count or dynamic)
///   - S::limits_type    (limit enforcement policy)
///
/// And satisfy the step/query lifecycle interface parameterized by those traits.
template <typename S>
concept ik_solve_policy = requires
{
    typename S::scalar_type;
    typename S::limits_type;
    { S::joints } -> std::convertible_to<int>;
} && requires(
    S& s,
    const kinematic_chain<typename S::scalar_type, S::joints>& chain,
    const se3<typename S::scalar_type>& target,
    const typename joint_state<typename S::scalar_type, S::joints>::position_type& q0,
    const convergence_criteria<typename S::scalar_type>& criteria)
{
    { s.setup(chain, target, q0, criteria) };
    { s.step(chain) } -> std::same_as<ik_status>;
    { s.converged() } -> std::convertible_to<bool>;
    { s.solution() } -> std::convertible_to<
        typename joint_state<typename S::scalar_type, S::joints>::position_type>;
    { s.error_norm() } -> std::convertible_to<typename S::scalar_type>;
    { s.iterations() } -> std::convertible_to<int>;
    { s.abort() };
};

}

#endif
