#ifndef HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_FEASIBLE_SET_H
#define HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_FEASIBLE_SET_H

/// Which set of joint bounds a policy solves over, determined once so the
/// runner reports it rather than the caller guessing from the policy's name.

#include "cartan/serial/ik/ik_status.h"

#include <cmath>
#include <concepts>

namespace cartan::detail
{

/// Opt-in marker for a policy that hands its backend a finite interval
/// substituted for a non-finite joint bound. A policy that box-projects instead
/// solves over the declared bounds, because clamping against an infinity is a
/// correct no-op, and says nothing.
template <typename Policy>
concept declares_bound_substitution = requires
{
    { Policy::substitutes_unbounded_bounds } -> std::convertible_to<bool>;
};

template <typename Chain>
bool has_unbounded_joint(const Chain& chain)
{
    for (int i = 0; i < chain.num_joints(); ++i)
    {
        const auto& limit = chain.limits()[static_cast<std::size_t>(i)];
        if (!std::isfinite(limit.position_min()) || !std::isfinite(limit.position_max()))
        {
            return true;
        }
    }
    return false;
}

/// A substituting policy only actually substitutes where a bound is non-finite,
/// so on a fully bounded chain every policy solves the declared set and the
/// split does not arise.
template <typename Policy, typename Chain>
feasible_set feasible_set_solved(const Chain& chain)
{
    if constexpr (declares_bound_substitution<Policy>)
    {
        return has_unbounded_joint(chain) ? feasible_set::substituted : feasible_set::declared;
    }
    else
    {
        return feasible_set::declared;
    }
}

}

#endif
