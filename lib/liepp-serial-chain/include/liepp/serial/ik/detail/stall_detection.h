#ifndef HPP_GUARD_LIEPP_SERIAL_IK_DETAIL_STALL_DETECTION_H
#define HPP_GUARD_LIEPP_SERIAL_IK_DETAIL_STALL_DETECTION_H

#include "liepp/serial/ik/ik_types.h"

#include <cmath>
#include <vector>
#include <algorithm>

namespace liepp::detail
{

/// Check for stall or divergence in iterative IK solvers.
///
/// Maintains a sliding window of error history and detects:
/// - Divergence: current error exceeds divergence_factor * initial_error
/// - Stall: maximum consecutive error change within the window is below threshold
///
/// Returns ik_status::diverged, ik_status::stalled, or ik_status::running.
template <typename Scalar>
ik_status check_stall_divergence(
    std::vector<Scalar>& error_history,
    Scalar current_error,
    Scalar initial_error,
    int stall_window,
    Scalar stall_threshold,
    Scalar divergence_factor)
{
    error_history.push_back(current_error);
    if (static_cast<int>(error_history.size()) > stall_window)
    {
        error_history.erase(error_history.begin());
    }

    if (current_error > divergence_factor * initial_error)
    {
        return ik_status::diverged;
    }

    if (static_cast<int>(error_history.size()) >= stall_window)
    {
        Scalar max_change{0};
        for (std::size_t i = 1; i < error_history.size(); ++i)
        {
            max_change = std::max(max_change,
                std::abs(error_history[i] - error_history[i - 1]));
        }
        if (max_change < stall_threshold)
        {
            return ik_status::stalled;
        }
    }

    return ik_status::running;
}

}

#endif
