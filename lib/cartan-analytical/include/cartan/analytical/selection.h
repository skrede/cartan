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

/// Pairs a selected joint configuration with the range verdict of the branch it
/// came from, so a successful pick can still report that no branch was in range.
template <typename Scalar, int N>
struct selected_solution
{
    Eigen::Vector<Scalar, N> q;
    range_status status;
};

/// Range-aware branch collapse: pick the branch nearest q_seed among those
/// tagged in_range, skipping a nearer branch that no 2*pi-equivalent brings
/// within limits. When no branch is in range it falls back to the nearest
/// overall and reports status == joint_limits_violated, so a valued result no
/// longer implies in-range -- the caller inspects .status. Only an empty result
/// fails, as analytical_failure::unreachable.
template <typename Scalar, int N, int MaxSolutions>
cartan::expected<selected_solution<Scalar, N>, analytical_error<Scalar>>
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

    const bool in_range_pick = best_in_range >= 0;
    const int pick = in_range_pick ? best_in_range : best_overall;
    if (pick < 0)
    {
        return cartan::unexpected(analytical_error<Scalar>{
            analytical_failure::unreachable, Scalar(0)});
    }

    return selected_solution<Scalar, N>{
        r.solutions[static_cast<std::size_t>(pick)],
        in_range_pick ? range_status::in_range : range_status::joint_limits_violated};
}

}

#endif
