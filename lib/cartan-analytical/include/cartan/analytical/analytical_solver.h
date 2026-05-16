#ifndef HPP_GUARD_CARTAN_ANALYTICAL_ANALYTICAL_SOLVER_H
#define HPP_GUARD_CARTAN_ANALYTICAL_ANALYTICAL_SOLVER_H

#include "cartan/analytical/analytical_types.h"

#include "cartan/lie/se3.h"

#include <concepts>

namespace cartan
{

/// Concept for closed-form IK solvers. A conforming type S exposes
/// `S::chain_type` (the chain type it solves over), `S::scalar_type`
/// (floating-point), `S::joints` (compile-time joint count), and
/// `S::max_solutions` (the per-solver upper bound on the number of
/// solutions). The single `solve(target)` method returns
/// `analytical_result<scalar, joints, max_solutions>` on success or
/// `analytical_error<scalar>` on failure. All returned solutions are
/// FK-verified by the solver internally; only verified solutions are
/// reported to the caller.
template <typename S>
concept analytical_solver = requires
{
    typename S::chain_type;
    typename S::scalar_type;
    { S::joints } -> std::convertible_to<int>;
    { S::max_solutions } -> std::convertible_to<int>;
} && requires(
    const S& s,
    const se3<typename S::scalar_type>& target)
{
    { s.solve(target) } -> std::same_as<
        cartan::expected<
            analytical_result<typename S::scalar_type, S::joints, S::max_solutions>,
            analytical_error<typename S::scalar_type>>>;
};

}

#endif
