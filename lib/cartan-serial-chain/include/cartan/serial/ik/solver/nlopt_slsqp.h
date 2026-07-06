#ifndef HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_NLOPT_SLSQP_H
#define HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_NLOPT_SLSQP_H

/// NLopt SLSQP gradient-based IK solve policy with box constraints.
///
/// Wraps NLopt's LD_SLSQP algorithm for constrained IK, using joint
/// limits as box constraints and the analytical gradient from
/// ik_se3_objective::evaluate_with_gradient (SE(3) log Jacobian).
///
/// This is the same algorithm TRAC-IK uses for its nonlinear
/// optimization path.
///
/// Guarded by CARTAN_HAS_NLOPT: only available when NLopt is linked.
///
/// Reference: Decisions D-04, D-08, D-11.

#ifdef CARTAN_HAS_NLOPT

#include "cartan/serial/ik/ik_status.h"
#include "cartan/serial/ik/policy/limits_policy.h"
#include "cartan/serial/ik/concepts/solve_concept.h"
#include "cartan/serial/ik/solver/detail/analytical_gradient.h"
#include "cartan/serial/ik/detail/nlopt_common.h"

#include "cartan/lie/se3.h"
#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/chain_concept.h"
#include "cartan/serial/chain/kinematic_chain.h"

#include <nlopt.hpp>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace cartan
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
class nlopt_slsqp
{
public:
    using chain_type = Chain;
    using scalar_type = typename Chain::scalar_type;
    static constexpr int joints = Chain::joints;
    using limits_type = LimitsPolicy;

    using position_type = typename joint_state<scalar_type, joints>::position_type;

    static_assert(std::is_floating_point_v<scalar_type>, "nlopt_slsqp requires a floating-point Scalar type");

    /// Tunable parameters for the SLSQP solve policy.
    struct options
    {
        scalar_type xtol_rel{scalar_type(1e-8)};
        scalar_type ftol_rel{scalar_type(1e-12)};
        int budget_per_step{500};
        int max_restarts{10};
        scalar_type restart_scale{scalar_type(0.5)};
    };

    nlopt_slsqp() = default;

    explicit nlopt_slsqp(const options& opts)
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
        m_objective_calls = 0;
        m_error_norm = std::numeric_limits<scalar_type>::max();
        m_status = ik_status::running;

        m_q_vec = cartan::detail::eigen_to_stdvec<scalar_type, joints>(q0);

        m_opt = nlopt::opt(nlopt::LD_SLSQP, static_cast<unsigned>(chain.num_joints()));
        cartan::detail::set_nlopt_bounds<scalar_type, joints>(m_opt, chain);
        m_opt.set_min_objective(objective_func, this);
        m_opt.set_xtol_rel(static_cast<double>(m_options.xtol_rel));
        m_opt.set_ftol_rel(static_cast<double>(m_options.ftol_rel));
        m_eval_count = m_options.budget_per_step;
        m_opt.set_maxeval(m_eval_count);
    }

    /// Execute up to N SLSQP optimization rounds.
    ///
    /// One algorithmic-work unit = one NLopt SLSQP optimize() call (each
    /// internally bounded by budget_per_step NLopt evaluations).
    step_result<scalar_type> step(const chain_type& chain, int N)
    {
        int units = 0;
        m_chain = &chain;
        while (units < N && m_status == ik_status::running)
        {
            m_eval_count += m_options.budget_per_step;
            m_opt.set_maxeval(m_eval_count);

            double min_val = 0.0;
            nlopt::result result = cartan::detail::run_nlopt_optimize(m_opt, m_q_vec, min_val);
            ++units;

            // Handle exception-sourced results immediately
            if (result == nlopt::FAILURE)
            {
                m_status = ik_status::diverged;
                break;
            }
            if (result == nlopt::ROUNDOFF_LIMITED)
            {
                m_error_norm = cartan::detail::compute_body_error_norm<scalar_type, joints>(*m_chain, m_target, m_q_vec);
                bool conv = cartan::detail::check_nlopt_convergence<scalar_type, joints>(*m_chain, m_target, m_criteria, m_q_vec);
                m_status = conv ? ik_status::converged : ik_status::stalled;
                break;
            }

            ++m_iterations;

            m_prev_error = m_error_norm;
            m_error_norm = cartan::detail::compute_body_error_norm<scalar_type, joints>(*m_chain, m_target, m_q_vec);

            bool conv = cartan::detail::check_nlopt_convergence<scalar_type, joints>(*m_chain, m_target, m_criteria, m_q_vec);
            bool error_stalled = std::abs(m_error_norm - m_prev_error) <
                scalar_type(1e-10) * (scalar_type(1) + m_error_norm);
            bool can_restart = m_restart_count < m_options.max_restarts;

            if (cartan::detail::needs_restart(result, conv, error_stalled))
            {
                ++m_restart_count;
                can_restart = m_restart_count < m_options.max_restarts;
            }

            m_status = cartan::detail::map_nlopt_result(result, conv, error_stalled, can_restart);

            if (m_status == ik_status::running && cartan::detail::needs_restart(result, conv, error_stalled))
            {
                cartan::detail::perturb_nlopt_solution<scalar_type, joints>(m_q_vec, *m_chain, m_options.restart_scale, m_rng);
                cartan::detail::reset_nlopt_optimizer<scalar_type, joints>(
                    m_opt, nlopt::LD_SLSQP, *m_chain, objective_func, this,
                    static_cast<double>(m_options.xtol_rel), m_options.budget_per_step, m_eval_count);
                m_opt.set_ftol_rel(static_cast<double>(m_options.ftol_rel));
            }

            if (m_iterations >= m_criteria.max_iterations_per_attempt)
            {
                m_status = ik_status::iteration_limit;
                break;
            }

            cartan::detail::enforce_and_sync_limits<LimitsPolicy, scalar_type, joints>(m_q_vec, chain);
        }
        return {m_status, {units, m_error_norm}};
    }

    [[nodiscard]] bool converged() const { return m_status == ik_status::converged; }

    [[nodiscard]] position_type solution() const
    {
        return cartan::detail::stdvec_to_eigen<scalar_type, joints>(m_q_vec);
    }

    [[nodiscard]] scalar_type error_norm() const { return m_error_norm; }

    [[nodiscard]] int iterations() const { return m_iterations; }

    /// Cumulative number of times `objective_func` was invoked by nlopt
    /// since the last `setup()` call. Counts both value-only and
    /// value+gradient evaluations (no distinction — nlopt dispatches
    /// through the same callback and branches on `grad.empty()`). This
    /// is the correct denominator for a per-nlopt-inner-iteration wall
    /// measurement: one invocation per nlopt inner iteration regardless
    /// of how many cartan-outer `step()` rounds or internal restarts
    /// happened during the pose's solve. Zero if `setup()` has not been
    /// called.
    [[nodiscard]] std::uint64_t nlopt_objective_calls() const
    {
        return m_objective_calls;
    }

    /// Number of times the cartan-side `needs_restart` perturbation loop
    /// fired during this pose's solve (see `nlopt_slsqp::step`). Each
    /// fire corresponds to one nlopt optimize() that returned
    /// SUCCESS/FTOL/XTOL without pose-tolerance convergence (or
    /// MAXEVAL with stalled error), followed by
    /// `perturb_nlopt_solution` + `reset_nlopt_optimizer` and a
    /// subsequent step() that re-runs optimize on the perturbed seed.
    /// Capped by `options::max_restarts` (default 10). Reset in
    /// `setup()`.
    [[nodiscard]] int nlopt_restart_count() const
    {
        return m_restart_count;
    }

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
        auto* self = static_cast<nlopt_slsqp*>(data);
        ++self->m_objective_calls;
        int n = static_cast<int>(x.size());
        auto q = cartan::detail::stdvec_to_eigen<scalar_type, joints>(x);

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
    std::uint64_t m_objective_calls{};
    ik_status m_status{ik_status::running};
    std::mt19937 m_rng{0};
};

}

#endif

#endif
