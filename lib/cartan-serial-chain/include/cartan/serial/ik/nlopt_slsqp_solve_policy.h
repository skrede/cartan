#ifndef HPP_GUARD_LIEPP_SERIAL_IK_NLOPT_SLSQP_SOLVE_POLICY_H
#define HPP_GUARD_LIEPP_SERIAL_IK_NLOPT_SLSQP_SOLVE_POLICY_H

/// @file nlopt_slsqp_solve_policy.h
/// @brief NLopt SLSQP gradient-based IK solve policy with box constraints.
///
/// Wraps NLopt's LD_SLSQP algorithm for constrained IK, using joint
/// limits as box constraints and the analytical gradient from
/// ik_se3_objective::evaluate_with_gradient (SE(3) log Jacobian).
///
/// This is the same algorithm TRAC-IK uses for its nonlinear
/// optimization path.
///
/// Guarded by LIEPP_HAS_NLOPT: only available when NLopt is linked.
///
/// Reference: Decisions D-04, D-08, D-11.

#ifdef LIEPP_HAS_NLOPT

#include "liepp/serial/ik/ik_types.h"
#include "liepp/serial/ik/limits_policy.h"
#include "liepp/serial/ik/ik_solve_policy.h"
#include "liepp/serial/ik/analytical_gradient.h"
#include "liepp/serial/ik/detail/nlopt_common.h"

#include "liepp/lie/se3.h"
#include "liepp/serial/chain/joint_state.h"
#include "liepp/serial/chain/chain_concept.h"
#include "liepp/serial/chain/kinematic_chain.h"

#include <nlopt.hpp>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace liepp
{

/// NLopt SLSQP solve policy for constrained IK with box constraints.
///
/// Uses the LD_SLSQP gradient-based SQP algorithm with joint limits
/// as box bounds and analytical gradient via SE(3) log Jacobian.
/// Each step() call runs budget_per_step NLopt evaluations, allowing
/// cooperative scheduling with other solvers.
///
/// Reference: D-04 (NLopt LD_SLSQP), D-11 (gradient via log Jacobian).
template <chain Chain, typename LimitsPolicy = clamp_limits>
class nlopt_slsqp_solve_policy
{
public:
    using chain_type = Chain;
    using scalar_type = typename Chain::scalar_type;
    static constexpr int joints = Chain::joints;
    using limits_type = LimitsPolicy;

    using position_type = typename joint_state<scalar_type, joints>::position_type;

    static_assert(std::is_floating_point_v<scalar_type>, "nlopt_slsqp_solve_policy requires a floating-point Scalar type");

    /// Tunable parameters for the SLSQP solve policy.
    struct options
    {
        scalar_type xtol_rel{scalar_type(1e-8)};
        scalar_type ftol_rel{scalar_type(1e-12)};
        int budget_per_step{500};
        int max_restarts{10};
        scalar_type restart_scale{scalar_type(0.5)};
    };

    nlopt_slsqp_solve_policy() = default;

    explicit nlopt_slsqp_solve_policy(const options& opts)
        : m_options(opts)
    {
    }

    /// Initialize the solve policy with chain, target, seed, and criteria.
    void setup(
        const chain_type& chain,
        const se3<scalar_type>& target,
        const position_type& q0,
        const convergence_criteria<scalar_type>& criteria)
    {
        m_chain = &chain;
        m_target = target;
        m_criteria = criteria;
        m_iterations = 0;
        m_restart_count = 0;
        m_error_norm = std::numeric_limits<scalar_type>::max();
        m_status = ik_status::running;

        m_q_vec = detail::eigen_to_stdvec<scalar_type, joints>(q0);

        m_opt = nlopt::opt(nlopt::LD_SLSQP, static_cast<unsigned>(chain.num_joints()));
        detail::set_nlopt_bounds<scalar_type, joints>(m_opt, chain);
        m_opt.set_min_objective(objective_func, this);
        m_opt.set_xtol_rel(static_cast<double>(m_options.xtol_rel));
        m_opt.set_ftol_rel(static_cast<double>(m_options.ftol_rel));
        m_eval_count = m_options.budget_per_step;
        m_opt.set_maxeval(m_eval_count);
    }

    /// Execute one SLSQP optimization round with budget_per_step evaluations.
    ik_status step(const chain_type& chain)
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
            m_error_norm = detail::compute_body_error_norm<scalar_type, joints>(*m_chain, m_target, m_q_vec);
            bool conv = detail::check_nlopt_convergence<scalar_type, joints>(*m_chain, m_target, m_criteria, m_q_vec);
            m_status = conv ? ik_status::converged : ik_status::stalled;
            return m_status;
        }

        ++m_iterations;

        m_prev_error = m_error_norm;
        m_error_norm = detail::compute_body_error_norm<scalar_type, joints>(*m_chain, m_target, m_q_vec);

        bool conv = detail::check_nlopt_convergence<scalar_type, joints>(*m_chain, m_target, m_criteria, m_q_vec);
        bool error_stalled = std::abs(m_error_norm - m_prev_error) <
            scalar_type(1e-10) * (scalar_type(1) + m_error_norm);
        bool can_restart = m_restart_count < m_options.max_restarts;

        if (detail::needs_restart(result, conv, error_stalled))
        {
            ++m_restart_count;
            can_restart = m_restart_count < m_options.max_restarts;
        }

        m_status = detail::map_nlopt_result(result, conv, error_stalled, can_restart);

        if (m_status == ik_status::running && detail::needs_restart(result, conv, error_stalled))
        {
            detail::perturb_nlopt_solution<scalar_type, joints>(m_q_vec, *m_chain, m_options.restart_scale, m_rng);
            detail::reset_nlopt_optimizer<scalar_type, joints>(
                m_opt, nlopt::LD_SLSQP, *m_chain, objective_func, this,
                static_cast<double>(m_options.xtol_rel), m_options.budget_per_step, m_eval_count);
            m_opt.set_ftol_rel(static_cast<double>(m_options.ftol_rel));
        }

        detail::enforce_and_sync_limits<LimitsPolicy, scalar_type, joints>(m_q_vec, chain);

        return m_status;
    }

    [[nodiscard]] bool converged() const { return m_status == ik_status::converged; }

    [[nodiscard]] position_type solution() const
    {
        return detail::stdvec_to_eigen<scalar_type, joints>(m_q_vec);
    }

    [[nodiscard]] scalar_type error_norm() const { return m_error_norm; }

    [[nodiscard]] int iterations() const { return m_iterations; }

    void abort()
    {
        m_opt.force_stop();
        m_status = ik_status::stalled;
    }

    [[nodiscard]] ik_status status() const { return m_status; }

private:
    /// NLopt objective with analytical gradient via SE(3) log Jacobian.
    static double objective_func(
        const std::vector<double>& x,
        std::vector<double>& grad,
        void* data)
    {
        auto* self = static_cast<nlopt_slsqp_solve_policy*>(data);
        int n = static_cast<int>(x.size());
        auto q = detail::stdvec_to_eigen<scalar_type, joints>(x);

        if (!grad.empty())
        {
            auto result = ik_se3_objective<chain_type>::evaluate_with_gradient(
                *self->m_chain, self->m_target, q);

            for (int i = 0; i < n; ++i)
            {
                grad[static_cast<std::size_t>(i)] = static_cast<double>(result.gradient(i));
            }

            return static_cast<double>(result.info.objective);
        }

        auto result = ik_se3_objective<chain_type>::evaluate(
            *self->m_chain, self->m_target, q);
        return static_cast<double>(result.objective);
    }

    nlopt::opt m_opt{nlopt::LD_SLSQP, 1};
    const chain_type* m_chain{nullptr};
    se3<scalar_type> m_target{se3<scalar_type>::identity()};
    convergence_criteria<scalar_type> m_criteria{};
    std::vector<double> m_q_vec;
    options m_options{};
    scalar_type m_error_norm{std::numeric_limits<scalar_type>::max()};
    scalar_type m_prev_error{std::numeric_limits<scalar_type>::max()};
    int m_iterations{};
    int m_eval_count{};
    int m_restart_count{};
    ik_status m_status{ik_status::running};
    std::mt19937 m_rng{std::random_device{}()};
};

}

#endif

#endif
