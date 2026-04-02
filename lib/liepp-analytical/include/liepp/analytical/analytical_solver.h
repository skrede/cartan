#ifndef HPP_GUARD_LIEPP_ANALYTICAL_ANALYTICAL_SOLVER_H
#define HPP_GUARD_LIEPP_ANALYTICAL_ANALYTICAL_SOLVER_H

#include "liepp/analytical/analytical_types.h"

#include "liepp/lie/se3.h"

#include <concepts>

namespace liepp
{

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
        std::expected<
            analytical_result<typename S::scalar_type, S::joints, S::max_solutions>,
            analytical_error<typename S::scalar_type>>>;
};

}

#endif
