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

#include <cmath>
#include <limits>
#include <concepts>
#include <cstddef>

namespace cartan
{
namespace detail
{

/// Default tolerance for the feasibility check. Set to sqrt(machine epsilon):
/// the joint value only needs to sit within a half-precision-scale band of the
/// finite bound, so a configuration that is exactly at (or a rounding step past)
/// a limit still reads as feasible while one that has genuinely drifted out of
/// range does not. This is the boundary tolerance for within_limits and is a
/// candidate for an empirical sweep should a bound-hugging solver need slack.
template <typename Scalar>
[[nodiscard]] inline Scalar default_feasibility_tol() noexcept
{
    return std::sqrt(std::numeric_limits<Scalar>::epsilon());
}

/// Feasibility predicate: true iff every finitely-bounded joint of q lies within
/// [position_min - tol, position_max + tol]. Joints whose bound is non-finite
/// (unbounded / continuous joints using +/-infinity) are skipped on that side,
/// so a joint with one finite and one infinite bound is still checked against
/// the finite side. This is a CHECK only -- it never mutates q. It is the gate a
/// no_limits trust-region solver consults before declaring convergence so that a
/// pose-converged but out-of-range configuration is reported as a joint-limit
/// failure rather than a trustworthy solution.
template <chain Chain>
[[nodiscard]] bool within_limits(
    const typename joint_state<typename Chain::scalar_type, Chain::joints>::position_type& q,
    const Chain& chain,
    typename Chain::scalar_type tol)
{
    const auto& limits = chain.limits();
    int n = chain.num_joints();
    for (int i = 0; i < n; ++i)
    {
        auto idx = static_cast<std::size_t>(i);
        auto lo = limits[idx].position_min;
        auto hi = limits[idx].position_max;
        if (std::isfinite(lo) && q(i) < lo - tol)
        {
            return false;
        }
        if (std::isfinite(hi) && q(i) > hi + tol)
        {
            return false;
        }
    }
    return true;
}

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
