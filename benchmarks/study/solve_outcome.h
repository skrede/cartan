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

#include <string_view>

namespace cartan::bench
{

struct solve_budget
{
    int index;
    int units;
    double tolerance;
    std::string_view axis;
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
