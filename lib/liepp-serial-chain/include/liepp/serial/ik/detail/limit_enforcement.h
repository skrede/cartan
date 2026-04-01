#ifndef HPP_GUARD_LIEPP_IK_DETAIL_LIMIT_ENFORCEMENT_H
#define HPP_GUARD_LIEPP_IK_DETAIL_LIMIT_ENFORCEMENT_H

/// @file detail/limit_enforcement.h
/// @brief Shared limit enforcement utility for IK solve policies.
///
/// Extracts the limit enforcement logic from ik_solver into a reusable
/// free function template. Policies call this in their step() method
/// instead of relying on post-hoc enforcement by the solver.
///
/// Reference: Decision D-13.

#include "liepp/ik/limits_policy.h"

#include "liepp/chain/joint_state.h"
#include "liepp/kinematics/jacobian.h"
#include "liepp/chain/kinematic_chain.h"
#include "liepp/kinematics/forward_kinematics.h"

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
/// @tparam Scalar        Floating-point type.
/// @tparam N             Number of joints (compile-time), or liepp::dynamic.
/// @param q              Joint configuration to enforce limits on (modified in place).
/// @param chain          Kinematic chain providing joint limits.
template <typename LimitsPolicy, typename Scalar, int N>
void enforce_limits(
    typename joint_state<Scalar, N>::position_type& q,
    const kinematic_chain<Scalar, N>& chain)
{
    if constexpr (std::same_as<LimitsPolicy, no_limits>)
    {
        return;
    }
    else if constexpr (has_extended_enforce<LimitsPolicy, Scalar, N>)
    {
        auto fk = forward_kinematics(chain, q);
        auto J_b = body_jacobian(chain, fk);

        constexpr unsigned int svd_options = (N == dynamic)
            ? (Eigen::ComputeThinU | Eigen::ComputeThinV)
            : (Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::JacobiSVD<jacobian_matrix<Scalar, N>> svd(J_b, svd_options);

        LimitsPolicy::template enforce_extended<Scalar, N>(
            q, chain.limits(), J_b, svd);
    }
    else
    {
        LimitsPolicy::template enforce<Scalar, N>(q, chain.limits());
    }
}

}
}

#endif
