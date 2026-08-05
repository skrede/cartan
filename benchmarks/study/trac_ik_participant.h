#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_TRAC_IK_PARTICIPANT_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_TRAC_IK_PARTICIPANT_H

/// @file trac_ik_participant.h
/// @brief The wall-clock-budgeted comparator under the one participant signature.
///
/// It takes its bounds from the feasible set's own derived arrays, through the
/// same accessor every other participant reads: the previously published
/// comparison handed this solver explicit bounds while the cartan row ran
/// unconstrained, and there is now no parameter through which that could be
/// written again.
///
/// Its iteration lives inside its own solve entry point and its kinematics
/// solvers are private members with no injection point, so the harness cannot
/// count its kernel evaluations. That is recorded as a property of the
/// participant rather than repaired with an estimate.

#include "feasible_set.h"
#include "solve_outcome.h"

#include "../benchmark_utils.h"

#include <cartan/lie/se3.h>
#include <cartan/serial/chain/joint_state.h>

#include <trac_ik/trac_ik.hpp>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <cmath>
#include <cstdlib>

namespace cartan::bench
{

template <int N>
class trac_ik_solver
{
public:
    using position_type = typename cartan::joint_state<double, N>::position_type;

    trac_ik_solver(const feasible_set<N>& feasible, double tolerance)
        : m_solver(
              feasible.comparator(), feasible.bounds().lower, feasible.bounds().upper,
              static_cast<double>(comparator_time_cap_ms) / 1000.0,
              tolerance / std::sqrt(3.0), TRAC_IK::Speed)
    {
        // Speed mode seeds its internal random restarts from rand().
        std::srand(42);
    }

    solve_outcome<N> operator()(
        const feasible_set<N>&,
        const cartan::se3<double>& target,
        const position_type& seed,
        const solve_budget&)
    {
        KDL::JntArray start(static_cast<unsigned int>(N));
        KDL::JntArray reached(static_cast<unsigned int>(N));
        for (unsigned int i = 0; i < static_cast<unsigned int>(N); ++i)
        {
            start(i) = seed(static_cast<int>(i));
        }
        const int status =
            m_solver.CartToJnt(start, cartan::fixtures::se3_to_kdl_frame(target), reached);
        position_type q;
        for (int i = 0; i < N; ++i)
        {
            q(i) = reached(static_cast<unsigned int>(i));
        }
        return solve_outcome<N>{0, status >= 0, kernel_counts{0, 0}, q};
    }

private:
    TRAC_IK::TRAC_IK m_solver;
};

}

#endif
