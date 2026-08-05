#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_PINOCCHIO_PARTICIPANT_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_PINOCCHIO_PARTICIPANT_H

/// @file pinocchio_participant.h
/// @brief The peer solver under the one call signature every participant has.

#include "pinocchio_lm.h"
#include "feasible_set.h"
#include "solve_outcome.h"
#include "counting_chain.h"
#include "pinocchio_chain.h"

#include <cartan/lie/se3.h>
#include <cartan/serial/chain/joint_state.h>

#include <Eigen/Dense>

namespace cartan::bench
{

/// The model is built once from the feasible set's own chain, which is why the
/// call operator does not read the feasible set again: solving a second
/// description of the same robot is the defect this study exists to remove.
template <int N>
class pinocchio_lm_solver
{
public:
    using position_type = typename cartan::joint_state<double, N>::position_type;

    explicit pinocchio_lm_solver(const feasible_set<N>& feasible)
        : m_work(N)
        , m_peer(build_pinocchio_model<N>(feasible.chain(), "study"))
    {
    }

    solve_outcome<N> operator()(
        const feasible_set<N>&,
        const cartan::se3<double>& target,
        const position_type& seed,
        const solve_budget& budget)
    {
        kernel_counts counts{0, 0};
        const auto answer = detail::run_pinocchio_lm(
            m_peer, m_work, to_pinocchio(target), Eigen::VectorXd(seed), budget, counts);
        return solve_outcome<N>{
            answer.iterations, answer.converged, counts, position_type(answer.q)};
    }

private:
    detail::pin_lm_scratch m_work;
    pinocchio_model m_peer;
};

}

#endif
