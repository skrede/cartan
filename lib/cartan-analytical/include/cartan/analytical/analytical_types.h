#ifndef HPP_GUARD_CARTAN_ANALYTICAL_ANALYTICAL_TYPES_H
#define HPP_GUARD_CARTAN_ANALYTICAL_ANALYTICAL_TYPES_H

#include "cartan/serial/chain/joint_state.h"

#include <array>
#include <expected>

namespace cartan
{

enum class analytical_failure
{
    unreachable,
    degenerate_geometry,
    singular_configuration,
    verification_failed
};

template <typename Scalar>
struct analytical_error
{
    analytical_failure reason;
    Scalar workspace_distance{};
};

template <typename Scalar, int N, int MaxSolutions>
struct analytical_result
{
    using position_type = Eigen::Vector<Scalar, N>;

    std::array<position_type, MaxSolutions> solutions;
    int count{0};

    [[nodiscard]] auto begin() const { return solutions.begin(); }
    [[nodiscard]] auto end() const { return solutions.begin() + count; }
};

}

#endif
