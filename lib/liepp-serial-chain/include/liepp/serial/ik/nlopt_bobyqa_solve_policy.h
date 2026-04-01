#ifndef HPP_GUARD_LIEPP_IK_NLOPT_BOBYQA_SOLVE_POLICY_H
#define HPP_GUARD_LIEPP_IK_NLOPT_BOBYQA_SOLVE_POLICY_H

/// @file nlopt_bobyqa_solve_policy.h
/// @brief NLopt BOBYQA derivative-free IK solve policy with box constraints.
///
/// Wraps NLopt's BOBYQA derivative-free algorithm for constrained IK,
/// using joint limits as box constraints. The objective minimizes
/// 0.5 * ||V_b||^2 without gradient information.
///
/// Guarded by LIEPP_HAS_NLOPT: only available when NLopt is linked.
///
/// Reference: Decisions D-04, D-07, D-10.

#ifdef LIEPP_HAS_NLOPT

#include "liepp/ik/ik_types.h"
#include "liepp/ik/limits_policy.h"
#include "liepp/ik/ik_solve_policy.h"
#include "liepp/ik/detail/nlopt_common.h"

#include "liepp/lie/se3.h"
#include "liepp/chain/joint_state.h"
#include "liepp/chain/kinematic_chain.h"
#include "liepp/kinematics/forward_kinematics.h"

#include <nlopt.hpp>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace liepp
{

/// NLopt BOBYQA solve policy for constrained IK with box constraints.
///
/// Uses the BOBYQA derivative-free algorithm with joint limits as box
/// bounds. Each step() call runs budget_per_step NLopt evaluations,
/// allowing cooperative scheduling with other solvers.
///
/// Reference: D-07 (BOBYQA provides diversity), D-10 (renamed from sqp_stepper).
template <typename Scalar = double, int N = dynamic, typename LimitsPolicy = clamp_limits>
class nlopt_bobyqa_solve_policy
{
public:
    static_assert(std::is_floating_point_v<Scalar>, "nlopt_bobyqa_solve_policy requires a floating-point Scalar type");

    using scalar_type = Scalar;
    static constexpr int joints = N;
    using limits_type = LimitsPolicy;

    using position_type = typename joint_state<Scalar, N>::position_type;

    /// Tunable parameters for the BOBYQA solve policy.
    ///
    /// NLopt BOBYQA does not preserve internal state across optimize()
    /// calls. Each step() starts a fresh optimization from the current
    /// joint configuration. budget_per_step must be large enough for
    /// BOBYQA to converge in a single call. 500 is sufficient for
    /// typical 6-7 DOF IK problems.
    struct options
    {
        Scalar xtol_rel{Scalar(1e-8)};
        int budget_per_step{500};
        int max_restarts{10};
        Scalar restart_scale{Scalar(0.5)};
    };

    nlopt_bobyqa_solve_policy() = default;

    explicit nlopt_bobyqa_solve_policy(const options& opts)
        : m_options(opts)
    {
    }

    /// Initialize the solve policy with chain, target, seed, and criteria.
    void setup(
        const kinematic_chain<Scalar, N>& chain,
        const se3<Scalar>& target,
        const position_type& q0,
        const convergence_criteria<Scalar>& criteria)
    {
        m_chain = &chain;
        m_target = target;
        m_criteria = criteria;
        m_iterations = 0;
        m_restart_count = 0;
        m_error_norm = std::numeric_limits<Scalar>::max();
        m_status = ik_status::running;

        m_q_vec = detail::eigen_to_stdvec<Scalar, N>(q0);

        m_opt = nlopt::opt(nlopt::LN_BOBYQA, static_cast<unsigned>(chain.num_joints()));
        detail::set_nlopt_bounds<Scalar, N>(m_opt, chain);
        m_opt.set_min_objective(objective_func, this);
        m_opt.set_xtol_rel(static_cast<double>(m_options.xtol_rel));
        m_eval_count = m_options.budget_per_step;
        m_opt.set_maxeval(m_eval_count);
    }

    /// Execute one BOBYQA optimization round with budget_per_step evaluations.
    ik_status step(const kinematic_chain<Scalar, N>& chain)
    {
        if (m_status != ik_status::running)
        {
            return m_status;
        }

        m_chain = &chain;

        m_eval_count += m_options.budget_per_step;
        m_opt.set_maxeval(m_eval_count);

        double min_val = 0.0;
        nlopt::result result = detail::run_nlopt_optimize(m_opt, m_q_vec, min_val);

        // Handle exception-sourced results immediately
        if (result == nlopt::FAILURE)
        {
            m_status = ik_status::diverged;
            return m_status;
        }
        if (result == nlopt::ROUNDOFF_LIMITED)
        {
            m_error_norm = detail::compute_body_error_norm<Scalar, N>(*m_chain, m_target, m_q_vec);
            bool conv = detail::check_nlopt_convergence<Scalar, N>(*m_chain, m_target, m_criteria, m_q_vec);
            m_status = conv ? ik_status::converged : ik_status::stalled;
            return m_status;
        }

        ++m_iterations;

        m_prev_error = m_error_norm;
        m_error_norm = detail::compute_body_error_norm<Scalar, N>(*m_chain, m_target, m_q_vec);

        bool conv = detail::check_nlopt_convergence<Scalar, N>(*m_chain, m_target, m_criteria, m_q_vec);
        bool error_stalled = std::abs(m_error_norm - m_prev_error) <
            Scalar(1e-10) * (Scalar(1) + m_error_norm);
        bool can_restart = m_restart_count < m_options.max_restarts;

        if (detail::needs_restart(result, conv, error_stalled))
        {
            ++m_restart_count;
            can_restart = m_restart_count < m_options.max_restarts;
        }

        m_status = detail::map_nlopt_result(result, conv, error_stalled, can_restart);

        if (m_status == ik_status::running && detail::needs_restart(result, conv, error_stalled))
        {
            detail::perturb_nlopt_solution<Scalar, N>(m_q_vec, *m_chain, m_options.restart_scale, m_rng);
            detail::reset_nlopt_optimizer<Scalar, N>(
                m_opt, nlopt::LN_BOBYQA, *m_chain, objective_func, this,
                static_cast<double>(m_options.xtol_rel), m_options.budget_per_step, m_eval_count);
        }

        detail::enforce_and_sync_limits<LimitsPolicy, Scalar, N>(m_q_vec, chain);

        return m_status;
    }

    /// Whether the solve policy has converged.
    [[nodiscard]] bool converged() const { return m_status == ik_status::converged; }

    /// Current joint configuration.
    [[nodiscard]] position_type solution() const
    {
        return detail::stdvec_to_eigen<Scalar, N>(m_q_vec);
    }

    /// Current error norm.
    [[nodiscard]] Scalar error_norm() const { return m_error_norm; }

    /// Number of step() calls executed.
    [[nodiscard]] int iterations() const { return m_iterations; }

    /// Abort the solver.
    void abort()
    {
        m_opt.force_stop();
        m_status = ik_status::stalled;
    }

    /// Current solve policy status.
    [[nodiscard]] ik_status status() const { return m_status; }

private:
    /// NLopt objective: 0.5 * ||V_b||^2 (no gradient -- BOBYQA is derivative-free).
    static double objective_func(
        const std::vector<double>& x,
        std::vector<double>& grad,
        void* data)
    {
        auto* self = static_cast<nlopt_bobyqa_solve_policy*>(data);
        auto q = detail::stdvec_to_eigen<Scalar, N>(x);

        auto fk = forward_kinematics(*self->m_chain, q);
        auto V_b = (fk.end_effector.inverse() * self->m_target).log();

        (void)grad;
        return 0.5 * static_cast<double>(V_b.squaredNorm());
    }

    nlopt::opt m_opt{nlopt::LN_BOBYQA, 1};
    const kinematic_chain<Scalar, N>* m_chain{nullptr};
    se3<Scalar> m_target{se3<Scalar>::identity()};
    convergence_criteria<Scalar> m_criteria{};
    std::vector<double> m_q_vec;
    options m_options{};
    Scalar m_error_norm{std::numeric_limits<Scalar>::max()};
    Scalar m_prev_error{std::numeric_limits<Scalar>::max()};
    int m_iterations{};
    int m_eval_count{};
    int m_restart_count{};
    ik_status m_status{ik_status::running};
    std::mt19937 m_rng{std::random_device{}()};
};

}

#endif

#endif
