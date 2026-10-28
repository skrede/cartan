#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_PINOCCHIO_RESTART_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_PINOCCHIO_RESTART_H

/// @file pinocchio_restart.h
/// @brief The peer library's kernels under the restarting strategy.
///
/// The single-start pair differs only in whose forward kinematics and Jacobian
/// run, and the restarting pair has to differ in the same one thing. So the
/// seeds come from the same Halton generator the wrapper beside it uses, the
/// restart is triggered by the same stall rule at the same thresholds, and the
/// units a failing attempt spent are charged against the same shared total.
/// Without that, a difference between the two restarting participants would be
/// a difference between two restart strategies rather than between two kernels.

#include "pinocchio_lm_run.h"
#include "feasible_set.h"
#include "solve_outcome.h"
#include "pinocchio_chain.h"

#include <cartan/lie/se3.h>
#include <cartan/serial/chain/joint_state.h>
#include <cartan/serial/ik/detail/limit_enforcement.h>
#include <cartan/serial/ik/solver/detail/halton_seed_generator.h>

#include <Eigen/Dense>

#include <functional>

namespace cartan::bench
{

constexpr int k_pin_max_restarts = 20;

template <int N>
class pinocchio_restart_lm_solver
{
public:
    using position_type = typename cartan::joint_state<double, N>::position_type;

    explicit pinocchio_restart_lm_solver(const feasible_set<N>& feasible)
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
        const auto goal = to_pinocchio(target);
        const cartan::halton_seed_generator<typename feasible_set<N>::chain_type> seeds(
            m_chain.get(), seed);

        Eigen::VectorXd start(seed);
        position_type best(start);
        int spent = 0;
        for (int restart = 0; restart <= k_pin_max_restarts && spent < budget.units; ++restart)
        {
            const auto answer = detail::run_pinocchio_lm<true>(
                m_peer, m_work, goal, start, remaining(budget, spent), counts);
            spent += answer.iterations;
            best = position_type(answer.q);
            // The limits live outside this loop's solver rather than inside its
            // policy, so a numerically converged but out-of-limits answer has to
            // be refused here. Breaking on convergence alone would retire a
            // target the strategy beside it would have re-seeded.
            if (answer.converged && admissible(best))
            {
                return solve_outcome<N>{spent, true, counts, best};
            }
            start = Eigen::VectorXd(seeds(restart));
        }
        return solve_outcome<N>{spent, false, counts, best};
    }

private:
    std::reference_wrapper<const typename feasible_set<N>::chain_type> m_chain;
    detail::pin_lm_scratch m_work;
    pinocchio_model m_peer;

    static solve_budget remaining(const solve_budget& budget, int spent)
    {
        solve_budget left = budget;
        left.units = budget.units - spent;
        return left;
    }

    /// The same canonicalization gate the single-start adapter applies, for the
    /// same reason: a pose reached at an unwrapped equivalent angle is not a
    /// success the comparator was asked for.
    bool admissible(position_type& q) const
    {
        return cartan::detail::feasible_after_canonicalization(
            q, m_chain.get(), cartan::detail::default_feasibility_tol<double>());
    }
};

}

#endif
