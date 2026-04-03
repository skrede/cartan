#ifndef HPP_GUARD_CARTAN_SERIAL_IK_IK_SOLVE_POLICY_H
#define HPP_GUARD_CARTAN_SERIAL_IK_IK_SOLVE_POLICY_H

/// @file ik_solve_policy.h
/// @brief Single-parameter concept for IK solve policies.
///
/// The ik_solve_policy concept extracts S::chain_type to determine the chain
/// type the policy operates on, along with S::scalar_type, S::joints, and
/// S::limits_type for trait access. This enables policies to work with any
/// chain type satisfying the chain concept.

#include "cartan/serial/ik/ik_types.h"

#include "cartan/lie/se3.h"
#include "cartan/serial/chain/joint_state.h"

#include <concepts>

namespace cartan
{

/// Concept for a single-parameter IK solve policy.
///
/// A conforming type S must expose:
///   - S::chain_type    (the chain type, satisfying the chain concept)
///   - S::scalar_type   (floating-point type)
///   - S::joints        (static constexpr int, joint count or dynamic)
///   - S::limits_type   (limit enforcement policy)
///
/// And satisfy the step/query lifecycle interface parameterized by those traits.
template <typename S>
concept ik_solve_policy = requires
{
    typename S::chain_type;
    typename S::scalar_type;
    typename S::limits_type;
    { S::joints } -> std::convertible_to<int>;
} && requires(
    S& s,
    const typename S::chain_type& chain,
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
