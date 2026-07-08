#ifndef HPP_GUARD_CARTAN_ANALYTICAL_RANGE_STATUS_H
#define HPP_GUARD_CARTAN_ANALYTICAL_RANGE_STATUS_H

namespace cartan
{

/// Per-solution range verdict for an unwrapped analytical solution. Distinct
/// from analytical_failure, which names a whole-solve failure: a solve can
/// succeed yet still return a branch that no 2*pi-equivalent brings inside the
/// joint limits, tagged joint_limits_violated here rather than dropped.
enum class range_status
{
    in_range,
    joint_limits_violated
};

}

#endif
