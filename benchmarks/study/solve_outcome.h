#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_SOLVE_OUTCOME_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_SOLVE_OUTCOME_H

/// @file solve_outcome.h
/// @brief What a participant is given and what it hands back.
///
/// Neither type carries a bound: the budget says how much work a solve may
/// spend and at what tolerance it stops, and the outcome says what came back
/// and what it cost. The problem itself travels separately, as the feasible set.

#include "counting_chain.h"

#include <cartan/serial/chain/joint_state.h>

#include <cstdint>
#include <string_view>

namespace cartan::bench
{

/// `requested` is what the rung asked for in the axis's own unit -- kernel
/// evaluations, or nanoseconds for a participant budgeted by the clock. What a
/// run reports on that axis is the value it achieved, which is a different
/// number: the study has already published a requested figure once.
///
/// A wall-clock-budgeted solve and a work-budgeted one are not comparable under
/// contention, which is why the cap travels with every figure it produced
/// whether or not that participant resolved.
struct solve_budget
{
    int index;
    int units;
    double tolerance;
    std::string_view axis;
    double time_cap_ms;
    std::int64_t requested;
};

template <int N>
struct solve_outcome
{
    int iterations;
    bool self_reported;
    kernel_counts counts;
    typename cartan::joint_state<double, N>::position_type q;
};

}

#endif
