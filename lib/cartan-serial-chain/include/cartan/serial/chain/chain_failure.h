#ifndef HPP_GUARD_CARTAN_SERIAL_CHAIN_CHAIN_FAILURE_H
#define HPP_GUARD_CARTAN_SERIAL_CHAIN_CHAIN_FAILURE_H

namespace cartan
{

/// Failure modes for chain construction and for the checked kinematics
/// boundaries. Allocation-free and matchable; separate from lie_failure
/// because it names chain, joint and limit concepts the Lie-group layer has
/// no knowledge of.
enum class chain_failure
{
    dimension_mismatch,           ///< Joint-vector or fk_result length differs from the chain's joint count.
    tag_axis_contradiction,       ///< A stored screw axis is not the +/-e_k named by its compile-time joint tag.
    reversed_position_bounds,     ///< Position bounds are not an interval: reversed, or both the same infinity.
    negative_velocity_limit,      ///< A joint velocity bound is negative.
    negative_effort_limit,        ///< A joint effort bound is negative.
    negative_acceleration_limit,  ///< A joint acceleration bound is negative.
    non_finite_input              ///< An input component is NaN or infinite.
};

/// Human-readable diagnostic for a chain_failure, for logging and binding
/// exception messages. Returns a static string literal; no allocation.
constexpr const char* message(chain_failure failure)
{
    switch (failure)
    {
    case chain_failure::dimension_mismatch:
        return "Vector length does not match the chain's joint count";
    case chain_failure::tag_axis_contradiction:
        return "Screw axis contradicts its compile-time joint tag";
    case chain_failure::reversed_position_bounds:
        return "Position bounds are not an interval: reversed, or both the same infinity";
    case chain_failure::negative_velocity_limit:
        return "Joint velocity bound is negative";
    case chain_failure::negative_effort_limit:
        return "Joint effort bound is negative";
    case chain_failure::negative_acceleration_limit:
        return "Joint acceleration bound is negative";
    case chain_failure::non_finite_input:
        return "Input contains a NaN or infinite component";
    }
    return "Unknown chain_failure";
}

}

#endif
