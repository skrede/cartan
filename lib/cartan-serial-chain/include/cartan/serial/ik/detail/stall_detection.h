#ifndef HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_STALL_DETECTION_H
#define HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_STALL_DETECTION_H

#include "cartan/serial/ik/ik_status.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <algorithm>

namespace cartan::detail
{

/// Default capacity for the stall-detection error-history ring.
///
/// The stall check keeps a sliding window of the most recent error norms. The
/// largest window requested by any solver is 15 (the band-limited damped
/// least-squares variant); the remaining solvers use 10 or 5. A capacity of 32
/// leaves ample headroom above that maximum so every window in use is retained
/// in full, yielding stall and divergence decisions bit-identical to an
/// unbounded history. A window request exceeding the capacity is clamped to it
/// (see check_stall_divergence), so the fixed array can never overflow.
inline constexpr std::size_t default_error_history_capacity = 32;

/// Fixed-capacity ring buffer of error norms for stall detection.
///
/// Backed by a std::array with head and size indices, it overwrites the oldest
/// value once full and never allocates. Logical indexing runs oldest to newest,
/// which is the order the consecutive-delta scan requires.
template <typename Scalar, std::size_t Cap = default_error_history_capacity>
class error_ring
{
public:
    /// Append a value, overwriting the oldest once the ring is full.
    void push(Scalar value)
    {
        if (m_size < Cap)
        {
            m_data[(m_head + m_size) % Cap] = value;
            ++m_size;
        }
        else
        {
            m_data[m_head] = value;
            m_head = (m_head + 1) % Cap;
        }
    }

    /// Drop all retained values; subsequent pushes start a fresh window.
    void clear()
    {
        m_head = 0;
        m_size = 0;
    }

    /// Number of values currently retained (at most Cap).
    std::size_t size() const
    {
        return m_size;
    }

    /// Logical access in oldest-to-newest order; i must be < size().
    Scalar operator[](std::size_t i) const
    {
        return m_data[(m_head + i) % Cap];
    }

private:
    std::array<Scalar, Cap> m_data{};
    std::size_t m_head{0};
    std::size_t m_size{0};
};

/// Check for stall or divergence in iterative IK solvers.
///
/// Maintains a sliding window of error history and detects:
/// - Divergence: current error exceeds divergence_factor * initial_error
/// - Stall: maximum consecutive error change within the window is below threshold
///
/// The effective window is clamped to the ring capacity; for every stall_window
/// within capacity the retained values and the max-consecutive-delta scan are
/// identical to an unbounded history, so decisions are bit-identical.
///
/// Returns ik_status::diverged, ik_status::stalled, or ik_status::running.
template <typename Scalar, std::size_t Cap>
ik_status check_stall_divergence(
    error_ring<Scalar, Cap>& error_history,
    Scalar current_error,
    Scalar initial_error,
    int stall_window,
    Scalar stall_threshold,
    Scalar divergence_factor)
{
    error_history.push(current_error);

    const std::size_t window = std::min<std::size_t>(
        static_cast<std::size_t>(stall_window), Cap);

    if (current_error > divergence_factor * initial_error)
    {
        return ik_status::diverged;
    }

    if (error_history.size() >= window)
    {
        const std::size_t start = error_history.size() - window;
        Scalar max_change{0};
        for (std::size_t i = start + 1; i < error_history.size(); ++i)
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
