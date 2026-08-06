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
#include <cartan/serial/ik/detail/limit_enforcement.h>

#include <Eigen/Dense>

#include <functional>

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
        : m_chain(feasible.chain())
        , m_work(N)
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
        // The gate every unconstrained trust-region solve in this repository
        // consults at convergence, and what makes this loop comparable with the
        // one beside it: without it a pose reached at an unwrapped 2*pi-equivalent
        // angle is claimed as a success and then adjudicated out of limits, which
        // answers a different question than the peer was asked.
        position_type q(answer.q);
        const bool feasible = cartan::detail::feasible_after_canonicalization(
            q, m_chain.get(), cartan::detail::default_feasibility_tol<double>());
        return solve_outcome<N>{answer.iterations, answer.converged && feasible, counts, q};
    }

private:
    std::reference_wrapper<const typename feasible_set<N>::chain_type> m_chain;
    detail::pin_lm_scratch m_work;
    pinocchio_model m_peer;
};

}

#endif
