#ifndef HPP_GUARD_CARTAN_ANALYTICAL_UNWRAPPED_RESULT_H
#define HPP_GUARD_CARTAN_ANALYTICAL_UNWRAPPED_RESULT_H

#include "cartan/analytical/range_status.h"

#include "cartan/serial/chain/joint_state.h"

#include <array>

namespace cartan
{

/// Multi-solution result parallel to analytical_result, carrying a per-solution
/// range verdict. N is the joint count and MaxSolutions the per-solver upper
/// bound; `solutions` and the parallel `tags` array are both sized to
/// MaxSolutions and only the first `count` entries are populated. tags[i] is the
/// range_status of solutions[i]. Traverse the populated subset via begin()/end().
template <typename Scalar, int N, int MaxSolutions>
struct unwrapped_result
{
    using position_type = Eigen::Vector<Scalar, N>;

    std::array<position_type, static_cast<std::size_t>(MaxSolutions)> solutions;
    std::array<range_status, static_cast<std::size_t>(MaxSolutions)> tags;
    int count{0};

    auto begin() const { return solutions.begin(); }
    auto end() const { return solutions.begin() + count; }
};

}

#endif
