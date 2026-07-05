#ifndef HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_LIMIT_ENFORCEMENT_H
#define HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_LIMIT_ENFORCEMENT_H

/// Shared limit enforcement utility for IK solve policies.
///
/// Extracts the limit enforcement logic from ik_solver into a reusable
/// free function template. Policies call this in their step() method
/// instead of relying on post-hoc enforcement by the solver.

#include "cartan/serial/ik/policy/limits_policy.h"

#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/fk/jacobian.h"
#include "cartan/serial/chain/chain_concept.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include <Eigen/SVD>

#include <concepts>

namespace cartan
{
namespace detail
{

/// Apply limit enforcement to joint configuration q using the given LimitsPolicy.
///
/// If LimitsPolicy is no_limits, this is a no-op (optimized away via if constexpr).
/// If LimitsPolicy satisfies has_extended_enforce, computes FK, body Jacobian, and
/// SVD for null-space projection before calling enforce_extended.
/// Otherwise, calls the simple enforce method.
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

        // V must be full: matrixV() is then n x n and V.rightCols(n - rank)
        // spans the true Jacobian kernel. A thin V is only n x min(m, n), so
        // for a wide (redundant) Jacobian it omits the kernel columns and hands
        // back row-space directions instead, corrupting the null-space step.
        // U may stay thin for a dynamic matrix (thin U is illegal for a
        // fixed-size one, hence the branch), and its 6 rows make thin and full
        // U identical in size regardless.
        constexpr unsigned int svd_options = (N == dynamic)
            ? (Eigen::ComputeThinU | Eigen::ComputeFullV)
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
