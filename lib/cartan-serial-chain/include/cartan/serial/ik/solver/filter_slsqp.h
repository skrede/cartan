#ifndef HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_FILTER_SLSQP_H
#define HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_FILTER_SLSQP_H

/// argmin-backed filter SLSQP IK solve policy with box constraints.
///
/// Wraps argmin's filter_slsqp_policy for constrained IK. Uses
/// Fletcher-Leyffer 2002 filter acceptance instead of an L1 merit function,
/// eliminating penalty parameter tuning. Compact L-BFGS Hessian with Kraft
/// LSQ/LSEI QP subproblem solver.

#include "cartan/serial/ik/ik_status.h"
#include "cartan/serial/ik/policy/error_weight.h"
#include "cartan/serial/ik/policy/limits_policy.h"
#include "cartan/serial/ik/concepts/solve_concept.h"
#include "cartan/serial/ik/detail/convergence.h"
#include "cartan/serial/ik/detail/argmin_problem.h"
#include "cartan/serial/ik/detail/stall_detection.h"
#include "cartan/serial/ik/detail/limit_enforcement.h"

#include "cartan/lie/se3.h"
#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/chain_concept.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include <argmin/solver/options.h>
#include <argmin/solver/convergence.h>
#include <argmin/solver/basic_solver.h>
#include <argmin/solver/filter_slsqp_policy.h>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <array>
#include <optional>
#include <random>
#include <type_traits>
#include <cstdint>

namespace cartan::ik
{

template <chain Chain,
          typename LimitsPolicy = clamp_limits,
          typename Convergence = argmin::default_convergence>
class filter_slsqp
{
public:
    using chain_type = Chain;
    using scalar_type = typename Chain::scalar_type;
    static constexpr int joints = Chain::joints;
    using limits_type = LimitsPolicy;

    using position_type = typename joint_state<scalar_type, joints>::position_type;

    static_assert(std::is_floating_point_v<scalar_type>, "filter_slsqp requires a floating-point Scalar type");

    struct options
    {
        scalar_type stall_threshold{scalar_type(1e-10)};
        scalar_type divergence_factor{scalar_type(10)};
        int stall_window{5};
        int max_restarts{10};
        scalar_type restart_scale{scalar_type(0.5)};
        std::optional<unsigned> rng_seed{};

        double line_search_c1{1e-4};
        double gradient_threshold{1e-12};
        double objective_threshold{1e-14};
        double step_threshold{1e-14};
        double objective_threshold_rel{1e-10};
        double step_threshold_rel{1e-10};
    };

    filter_slsqp() = default;

    explicit filter_slsqp(const options& opts)
        : m_options{opts}
    {}

    void setup(
        const Chain& chain,
        const se3<scalar_type>& target,
        const position_type& q0,
        const convergence_criteria<scalar_type>& criteria)
    {
        m_chain = &chain;
        m_target = target;
        m_criteria = criteria;
        m_iterations = 0;
        m_error_norm = std::numeric_limits<scalar_type>::max();
        m_status = ik_status::running;
        m_termination_reason = ik_termination_reason::unknown;
        m_error_history.clear();
        m_restart_count = 0;
        if (m_options.rng_seed)
            m_rng.seed(*m_options.rng_seed);

        auto fk = forward_kinematics(chain, q0);
        auto V_b = (target.inverse() * fk.end_effector).log();
        m_initial_error = V_b.norm();

        m_problem.emplace(chain, target, m_weight);

        int n = chain.num_joints();
        Eigen::VectorXd x0(n);
        for (int i = 0; i < n; ++i)
            x0[i] = static_cast<double>(q0[i]);

        build_argmin_opts(m_nab_opts);

        typename argmin::filter_slsqp_policy<joints>::options_type policy_opts{};
        policy_opts.line_search.c1 = m_options.line_search_c1;

        m_solver.emplace(*m_problem, x0, m_nab_opts, policy_opts);
    }

    step_result<scalar_type> step(const Chain& chain, int N)
    {
        int units = 0;
        m_chain = &chain;
        while (units < N && m_status == ik_status::running)
        {
            // One algorithmic-work unit = one argmin-internal SLSQP iteration.
            auto prev_iter = m_solver->state().iteration;
            auto result = m_solver->step_n(
                static_cast<std::uint32_t>(1), m_nab_opts);
            auto inner_units = static_cast<int>(m_solver->state().iteration - prev_iter);
            if (inner_units <= 0)
            {
                inner_units = 1;
            }
            m_iterations += inner_units;
            units += inner_units;

            sync_solution_from_solver();

            auto fk = forward_kinematics(chain, m_q);
            auto V_b = (m_target.inverse() * fk.end_effector).log();
            m_error_norm = V_b.norm();

            if (cartan::detail::is_converged_unweighted(V_b, m_criteria))
            {
                m_status = ik_status::converged;
                m_termination_reason = ik_termination_reason::converged;
                break;
            }

            if (m_iterations >= m_criteria.max_iterations_per_attempt)
            {
                m_status = ik_status::iteration_limit;
                m_termination_reason = ik_termination_reason::iteration_limit;
                break;
            }

            auto stall_result = cartan::detail::check_stall_divergence(
                m_error_history, m_error_norm, m_initial_error,
                m_options.stall_window, m_options.stall_threshold,
                m_options.divergence_factor);
            if (stall_result != ik_status::running)
            {
                m_status = stall_result;
                m_termination_reason = (stall_result == ik_status::diverged)
                    ? ik_termination_reason::divergence_detected
                    : ik_termination_reason::stall_detected;
                break;
            }

            if (is_inner_terminal(result.status))
            {
                if (m_restart_count < m_options.max_restarts)
                {
                    ++m_restart_count;
                    auto q_perturbed = perturb_solution(m_q, *m_chain);

                    m_error_history.clear();
                    auto fk_new = forward_kinematics(*m_chain, q_perturbed);
                    auto V_b_new = (m_target.inverse() * fk_new.end_effector).log();
                    m_initial_error = V_b_new.norm();

                    int n = m_chain->num_joints();
                    Eigen::VectorXd x0(n);
                    for (int i = 0; i < n; ++i)
                        x0[i] = static_cast<double>(q_perturbed[i]);
                    m_solver->reset_clear(x0);

                    build_argmin_opts(m_nab_opts);

                    m_q = q_perturbed;
                    continue;
                }

                m_status = ik_status::stalled;
                m_termination_reason = map_argmin_status(result.status);
                break;
            }

            cartan::detail::enforce_limits<LimitsPolicy>(m_q, chain);
        }
        return {m_status, {units, m_error_norm}};
    }

    [[nodiscard]] bool converged() const { return m_status == ik_status::converged; }
    [[nodiscard]] const position_type& solution() const { return m_q; }
    [[nodiscard]] scalar_type error_norm() const { return m_error_norm; }
    [[nodiscard]] int iterations() const { return m_iterations; }
    [[nodiscard]] ik_status status() const { return m_status; }
    [[nodiscard]] ik_termination_reason termination_reason() const { return m_termination_reason; }
    [[nodiscard]] int restart_count() const { return m_restart_count; }

    [[nodiscard]] std::uint64_t line_search_calls() const
    {
        if (!m_solver) return 0;
        using state_t = std::remove_cvref_t<decltype(m_solver->state())>;
        if constexpr (requires(const state_t& s) { s.line_search_calls; })
            return m_solver->state().line_search_calls;
        else
            return 0;
    }

    [[nodiscard]] std::uint32_t argmin_iterations() const
    {
        if (!m_solver) return 0;
        using state_t = std::remove_cvref_t<decltype(m_solver->state())>;
        if constexpr (requires(const state_t& s) { s.iteration; })
            return m_solver->state().iteration;
        else
            return 0;
    }

    [[nodiscard]] auto last_check_results() const
    {
        if constexpr (requires { m_nab_opts.convergence.last_check_results(); })
            return m_nab_opts.convergence.last_check_results();
        else
            return std::array<std::optional<argmin::solver_status>, 4>{};
    }

    void abort()
    {
        m_status = ik_status::stalled;
        m_termination_reason = ik_termination_reason::solver_aborted;
    }

private:
    using argmin_solver = argmin::basic_solver<
        argmin::filter_slsqp_policy<joints>, joints, cartan::detail::argmin_ik_problem<Chain>>;
    using argmin_opts_type = argmin::solver_options<Convergence>;

    position_type perturb_solution(const position_type& q, const Chain& chain)
    {
        int n = chain.num_joints();
        const auto& limits = chain.limits();
        std::uniform_real_distribution<double> dist(-1.0, 1.0);

        position_type q_new;
        if constexpr (joints == dynamic)
            q_new.resize(n);

        for (int i = 0; i < n; ++i)
        {
            auto idx = static_cast<std::size_t>(i);
            const auto raw_range = limits[idx].position_max - limits[idx].position_min;
            const auto range = cartan::detail::finite_range_or(raw_range,
                cartan::detail::k_unbounded_angular_range_v<scalar_type>);
            auto perturbation = static_cast<scalar_type>(dist(m_rng)) * m_options.restart_scale * range;
            q_new[i] = std::clamp(
                q[i] + perturbation,
                limits[idx].position_min,
                limits[idx].position_max);
        }
        return q_new;
    }

    void build_argmin_opts(argmin_opts_type& opts) const
    {
        // Per-attempt cap; argmin's setup-time cap is generous because the
        // per-call budget is now threaded through step_n(N) from the runner.
        opts.max_iterations = static_cast<std::uint32_t>(m_criteria.max_iterations_per_attempt);
        if constexpr (std::is_same_v<Convergence, argmin::default_convergence>)
        {
            opts.set_gradient_threshold(m_options.gradient_threshold);
            opts.set_objective_threshold(m_options.objective_threshold);
            opts.set_step_threshold(m_options.step_threshold);
        }
        else
        {
            opts.set_objective_threshold_rel(m_options.objective_threshold_rel);
            opts.set_step_threshold_rel(m_options.step_threshold_rel);
        }
    }

    static constexpr bool is_inner_terminal(argmin::solver_status s) noexcept
    {
        switch (s)
        {
            case argmin::solver_status::converged:
            case argmin::solver_status::ftol_reached:
            case argmin::solver_status::xtol_reached:
            case argmin::solver_status::stalled:
            case argmin::solver_status::objective_stalled:
            case argmin::solver_status::roundoff_limited:
            case argmin::solver_status::aborted:
            case argmin::solver_status::diverged:
            case argmin::solver_status::max_iterations:
                return true;
            default:
                return false;
        }
    }

    static constexpr ik_termination_reason map_argmin_status(argmin::solver_status s) noexcept
    {
        switch (s)
        {
            case argmin::solver_status::converged:         return ik_termination_reason::solver_converged_pose_missed;
            case argmin::solver_status::ftol_reached:      return ik_termination_reason::solver_ftol_reached;
            case argmin::solver_status::xtol_reached:      return ik_termination_reason::solver_xtol_reached;
            case argmin::solver_status::stalled:           return ik_termination_reason::solver_stalled;
            case argmin::solver_status::objective_stalled: return ik_termination_reason::solver_objective_stalled;
            case argmin::solver_status::roundoff_limited:  return ik_termination_reason::solver_roundoff_limited;
            case argmin::solver_status::aborted:           return ik_termination_reason::solver_aborted;
            case argmin::solver_status::budget_exhausted:  return ik_termination_reason::solver_budget_exhausted;
            case argmin::solver_status::max_iterations:    return ik_termination_reason::solver_max_iterations;
            case argmin::solver_status::diverged:          return ik_termination_reason::solver_diverged;
            default:                                        return ik_termination_reason::unknown;
        }
    }

    void sync_solution_from_solver()
    {
        const auto& x = m_solver->state().x;
        int n = m_chain->num_joints();
        if constexpr (joints == dynamic)
            m_q.resize(n);
        for (int i = 0; i < n; ++i)
            m_q[i] = static_cast<scalar_type>(x[i]);
    }

    const Chain* m_chain{nullptr};
    se3<scalar_type> m_target{se3<scalar_type>::identity()};
    convergence_criteria<scalar_type> m_criteria{};
    error_weight<scalar_type> m_weight{};
    options m_options{};
    position_type m_q{};
    std::vector<scalar_type> m_error_history;
    scalar_type m_initial_error{};
    scalar_type m_error_norm{std::numeric_limits<scalar_type>::max()};
    int m_iterations{};
    ik_status m_status{ik_status::running};
    ik_termination_reason m_termination_reason{ik_termination_reason::unknown};
    std::optional<cartan::detail::argmin_ik_problem<Chain>> m_problem;
    std::optional<argmin_solver> m_solver;
    argmin_opts_type m_nab_opts{};
    int m_restart_count{};
    std::mt19937 m_rng{0};
};

}

#endif
