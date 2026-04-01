#ifndef HPP_GUARD_LIEPP_SERIAL_IK_DETAIL_LIMIT_ENFORCEMENT_H
#define HPP_GUARD_LIEPP_SERIAL_IK_DETAIL_LIMIT_ENFORCEMENT_H

/// @file detail/limit_enforcement.h
/// @brief Shared limit enforcement utility for IK solve policies.
///
/// Extracts the limit enforcement logic from ik_solver into a reusable
/// free function template. Policies call this in their step() method
/// instead of relying on post-hoc enforcement by the solver.

#include "liepp/serial/ik/limits_policy.h"

#include "liepp/serial/chain/joint_state.h"
#include "liepp/serial/fk/jacobian.h"
#include "liepp/serial/chain/chain_concept.h"
#include "liepp/serial/fk/forward_kinematics.h"

#include <Eigen/SVD>

#include <concepts>

namespace liepp
{
namespace detail
{

/// Apply limit enforcement to joint configuration q using the given LimitsPolicy.
///
/// If LimitsPolicy is no_limits, this is a no-op (optimized away via if constexpr).
/// If LimitsPolicy satisfies has_extended_enforce, computes FK, body Jacobian, and
/// SVD for null-space projection before calling enforce_extended.
/// Otherwise, calls the simple enforce method.
///
/// @tparam LimitsPolicy  Limit enforcement policy type.
/// @tparam Chain         A type satisfying the chain concept.
/// @param q              Joint configuration to enforce limits on (modified in place).
/// @param chain          Chain providing joint limits.
template <typename LimitsPolicy, chain Chain>
void enforce_limits(
    typename joint_state<typename Chain::scalar_type, Chain::joints>::position_type& q,
    const Chain& chain)
{
    using Scalar = typename Chain::scalar_type;
    static constexpr int N = Chain::joints;

    if constexpr (std::same_as<LimitsPolicy, no_limits>)
    {
        return;
    }
    else if constexpr (has_extended_enforce<LimitsPolicy, Chain>)
    {
        auto fk = forward_kinematics(chain, q);
        auto J_b = body_jacobian(chain, fk);

        constexpr unsigned int svd_options = (N == dynamic)
            ? (Eigen::ComputeThinU | Eigen::ComputeThinV)
            : (Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::JacobiSVD<jacobian_matrix<Scalar, N>> svd(J_b, svd_options);

        LimitsPolicy::template enforce_extended<Chain>(
            q, chain.limits(), J_b, svd);
    }
    else
    {
        LimitsPolicy::template enforce<Chain>(q, chain.limits());
    }
}

}
}

#endif
