#ifndef HPP_GUARD_CARTAN_ANALYTICAL_SELECTION_H
#define HPP_GUARD_CARTAN_ANALYTICAL_SELECTION_H

#include "cartan/analytical/range_status.h"
#include "cartan/analytical/analytical_types.h"
#include "cartan/analytical/unwrapped_result.h"

#include "cartan/expected.h"

#include <Eigen/Dense>

#include <limits>
#include <cstddef>
#include <algorithm>

namespace cartan
{

/// Collapse a solution set to the single branch minimizing ||q - q_seed||_2
/// (compared via squared norm). Returns analytical_failure::unreachable when the
/// result carries no branches; upstream solvers normally surface that directly.
template <typename Scalar, int N, int MaxSolutions>
cartan::expected<Eigen::Vector<Scalar, N>, analytical_error<Scalar>>
closest_to_seed(const analytical_result<Scalar, N, MaxSolutions>& r,
                const Eigen::Vector<Scalar, N>& q_seed)
{
    auto it = std::min_element(
        r.begin(), r.end(),
        [&](const Eigen::Vector<Scalar, N>& a, const Eigen::Vector<Scalar, N>& b)
        {
            return (a - q_seed).squaredNorm() < (b - q_seed).squaredNorm();
        });

    if (it == r.end())
    {
        return cartan::unexpected(analytical_error<Scalar>{
            analytical_failure::unreachable, Scalar(0)});
    }

    return *it;
}

/// Range-aware branch collapse: pick the branch nearest q_seed among those
/// tagged in_range, skipping a nearer branch that no 2*pi-equivalent brings
/// within limits. Only when no branch is in range does it fall back to the
/// nearest overall, which the source result already tags joint_limits_violated
/// (never silently out of range, never silently dropped).
template <typename Scalar, int N, int MaxSolutions>
cartan::expected<Eigen::Vector<Scalar, N>, analytical_error<Scalar>>
closest_to_seed(const unwrapped_result<Scalar, N, MaxSolutions>& r,
                const Eigen::Vector<Scalar, N>& q_seed)
{
    int best_in_range = -1;
    int best_overall = -1;
    Scalar near_in_range = std::numeric_limits<Scalar>::infinity();
    Scalar near_overall = std::numeric_limits<Scalar>::infinity();

    for (int i = 0; i < r.count; ++i)
    {
        const Scalar d = (r.solutions[static_cast<std::size_t>(i)] - q_seed).squaredNorm();
        if (d < near_overall) { near_overall = d; best_overall = i; }
        if (r.tags[static_cast<std::size_t>(i)] == range_status::in_range && d < near_in_range)
        {
            near_in_range = d;
            best_in_range = i;
        }
    }

    const int pick = best_in_range >= 0 ? best_in_range : best_overall;
    if (pick < 0)
    {
        return cartan::unexpected(analytical_error<Scalar>{
            analytical_failure::unreachable, Scalar(0)});
    }

    return r.solutions[static_cast<std::size_t>(pick)];
}

}

#endif
