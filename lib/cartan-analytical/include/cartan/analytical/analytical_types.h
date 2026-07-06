#ifndef HPP_GUARD_CARTAN_ANALYTICAL_ANALYTICAL_TYPES_H
#define HPP_GUARD_CARTAN_ANALYTICAL_ANALYTICAL_TYPES_H

#include "cartan/serial/chain/joint_state.h"

#include <array>
#include "cartan/expected.h"

namespace cartan
{

/// Failure modes for the analytical IK solvers.
enum class analytical_failure
{
    unreachable,             ///< Target lies outside the mechanism's workspace.
    degenerate_geometry,     ///< Joint geometry violates a subproblem precondition (e.g. parallel axes where intersection is required).
    singular_configuration,  ///< Mechanism is at a kinematic singularity for the requested target.
    verification_failed      ///< Candidate solutions exist but none survived the FK back-check.
};

/// Failure diagnostic for analytical solvers. `reason` names the failure mode;
/// `workspace_distance` is the magnitude (in the chain's linear unit) by which
/// the target exceeds the reachable workspace when `reason` is
/// `analytical_failure::unreachable`, and zero otherwise.
template <typename Scalar>
struct analytical_error
{
    analytical_failure reason;
    Scalar workspace_distance{};
};

/// Multi-solution result for an analytical IK solver. N is the joint count
/// (compile-time); MaxSolutions is the per-solver upper bound on solution
/// count (e.g. 2 for planar_2r_solver, 4 for spatial_3r_solver, 8 for
/// pieper_6r_solver). The `solutions` array is sized to MaxSolutions; only
/// the first `count` entries are populated and FK-verified. Use the
/// begin()/end() iterators to traverse the populated subset.
template <typename Scalar, int N, int MaxSolutions>
struct analytical_result
{
    using position_type = Eigen::Vector<Scalar, N>;

    std::array<position_type, static_cast<std::size_t>(MaxSolutions)> solutions;
    int count{0};

    auto begin() const { return solutions.begin(); }
    auto end() const { return solutions.begin() + count; }
};

}

#endif
