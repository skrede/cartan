#ifndef HPP_GUARD_CARTAN_SERIAL_CHAIN_JOINT_LIMITS_H
#define HPP_GUARD_CARTAN_SERIAL_CHAIN_JOINT_LIMITS_H

/// Joint limits for kinematic chain joints.
///
/// Stores required position bounds and optional velocity, effort,
/// and acceleration limits. Aggregate-initializable.

#include <optional>

namespace cartan
{

/// Joint limits with required position bounds and optional dynamic limits.
/// Aggregate-initializable: joint_limits<double>{-3.14, 3.14} or
/// joint_limits<double>{-3.14, 3.14, 2.0, 50.0, 10.0}.
template <typename Scalar = double>
struct joint_limits
{
    Scalar position_min;                              ///< Minimum joint position (required)
    Scalar position_max;                              ///< Maximum joint position (required)
    std::optional<Scalar> velocity_max{};             ///< Maximum joint velocity (optional)
    std::optional<Scalar> effort_max{};               ///< Maximum joint effort/torque (optional)
    std::optional<Scalar> acceleration_max{};         ///< Maximum joint acceleration (optional)

    /// Check whether a position value lies within [position_min, position_max].
    [[nodiscard]] bool contains(Scalar position) const
    {
        return position >= position_min && position <= position_max;
    }
};

}

#endif
