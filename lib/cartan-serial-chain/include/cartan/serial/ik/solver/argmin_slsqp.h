#ifndef HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_ARGMIN_SLSQP_H
#define HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_ARGMIN_SLSQP_H

/// @file argmin_slsqp.h
/// @brief nablapp-backed SLSQP gradient-based IK solve policy with box constraints.
///
/// Wraps nablapp's kraft_slsqp_policy for constrained IK, using joint
/// limits as box constraints and the analytical gradient from
/// ik_se3_objective (SE(3) log Jacobian). Always available -- nablapp
/// is a required dependency of cartan::kinematics.
///
/// Reference: Kraft 1988, N&W Ch. 18 (SQP methods).

#include "cartan/serial/ik/ik_status.h"
#include "cartan/serial/ik/policy/error_weight.h"
#include "cartan/serial/ik/policy/limits_policy.h"
#include "cartan/serial/ik/concepts/solve_concept.h"
#include "cartan/serial/ik/detail/convergence.h"
#include "cartan/serial/ik/detail/nablapp_problem.h"
#include "cartan/serial/ik/detail/stall_detection.h"
#include "cartan/serial/ik/detail/limit_enforcement.h"

#include "cartan/lie/se3.h"
#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/chain_concept.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include <nablapp/solver/options.h>
#include <nablapp/solver/basic_solver.h>
#include <nablapp/solver/kraft_slsqp_policy.h>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <cstdint>

namespace cartan::ik
{

/// nablapp-backed SLSQP solve policy for constrained IK with box constraints.
///
/// Uses Kraft's Sequential Least Squares Programming algorithm via nablapp,
/// with analytical gradient through the SE(3) log Jacobian. Each step()
/// call runs a budget of nablapp iterations, allowing cooperative scheduling
/// with other policies in basic_ik_solver.
///
/// This is the default (unprefixed) SLSQP policy. The NLopt-backed variant
/// is available as nlopt_slsqp_solve_policy behind CARTAN_HAS_NLOPT.
template <chain Chain, typename LimitsPolicy = clamp_limits>
class argmin_slsqp
{
public:
    using chain_type = Chain;
    using scalar_type = typename Chain::scalar_type;
    static constexpr int joints = Chain::joints;
    using limits_type = LimitsPolicy;

    using position_type = typename joint_state<scalar_type, joints>::position_type;

    static_assert(std::is_floating_point_v<scalar_type>, "argmin_slsqp requires a floating-point Scalar type");

    struct options
    {
        int budget_per_step{50};
        scalar_type stall_threshold{scalar_type(1e-10)};
        scalar_type divergence_factor{scalar_type(10)};
        int stall_window{5};

        /// Armijo sufficient-decrease parameter c1 forwarded to
        /// kraft_slsqp_policy's embedded merit line search. Nablapp's default
        /// is 1e-4; looser values (1e-3) accept more trial steps and cut
        /// backtracks on expensive-objective problems like IK where each
        /// phi(alpha) call costs a full FK pass. Tightening below 1e-5 is
        /// generally a no-op because the Armijo condition on a well-scaled
        /// merit function dominates before c1 matters.
        double line_search_c1{1e-4};
    };

    argmin_slsqp() = default;

    explicit argmin_slsqp(const options& opts)
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

        auto fk = forward_kinematics(chain, q0);
        auto V_b = (target.inverse() * fk.end_effector).log();
        m_initial_error = V_b.norm();

        m_problem.emplace(chain, target, m_weight);

        int n = chain.num_joints();
        Eigen::VectorXd x0(n);
        for (int i = 0; i < n; ++i)
        {
            x0[i] = static_cast<double>(q0[i]);
        }

        nablapp::solver_options<> nab_opts;
        nab_opts.max_iterations = m_options.budget_per_step;
        nab_opts.set_gradient_threshold(1e-12);
        nab_opts.set_objective_threshold(1e-14);
        nab_opts.set_step_threshold(1e-14);

        typename nablapp::kraft_slsqp_policy<joints>::options_type policy_opts{};
        policy_opts.line_search.c1 = m_options.line_search_c1;

        m_solver.emplace(*m_problem, x0, nab_opts, policy_opts);
    }

    ik_status step(const Chain& chain)
    {
        if (m_status != ik_status::running)
        {
            return m_status;
        }

        m_chain = &chain;
        ++m_iterations;

        if (m_iterations >= m_criteria.max_iterations)
        {
            m_status = ik_status::iteration_limit;
            m_termination_reason = ik_termination_reason::iteration_limit;
            return m_status;
        }

        auto result = m_solver->step_n(m_options.budget_per_step);

        sync_solution_from_solver();

        auto fk = forward_kinematics(chain, m_q);
        auto V_b = (m_target.inverse() * fk.end_effector).log();
        m_error_norm = V_b.norm();

        if (cartan::detail::is_converged_unweighted(V_b, m_criteria))
        {
            m_status = ik_status::converged;
            m_termination_reason = ik_termination_reason::converged;
            return m_status;
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
            return m_status;
        }

        if (is_inner_terminal(result.status))
        {
            m_status = ik_status::stalled;
            m_termination_reason = map_nablapp_status(result.status);
            return m_status;
        }

        cartan::detail::enforce_limits<LimitsPolicy>(m_q, chain);

        return m_status;
    }

    [[nodiscard]] bool converged() const { return m_status == ik_status::converged; }
    [[nodiscard]] const position_type& solution() const { return m_q; }
    [[nodiscard]] scalar_type error_norm() const { return m_error_norm; }
    [[nodiscard]] int iterations() const { return m_iterations; }
    [[nodiscard]] ik_status status() const { return m_status; }
    [[nodiscard]] ik_termination_reason termination_reason() const { return m_termination_reason; }

    /// Cumulative number of phi(alpha) line-search calls the underlying
    /// kraft_slsqp_policy has made since setup(). Zero if the solver has
    /// not been set up. Exposed so benchmarks can measure average backtracks
    /// per solver step without reaching into nablapp internals.
    [[nodiscard]] std::uint64_t line_search_calls() const
    {
        return m_solver ? m_solver->state().line_search_calls : 0;
    }

    /// Cumulative number of kraft_slsqp_policy::step() invocations since
    /// setup(), read from the inner nablapp solver's state. This is the
    /// inner step count that pairs with line_search_calls to compute
    /// average backtracks per nablapp step. Zero if the solver has not
    /// been set up.
    [[nodiscard]] std::uint32_t nablapp_iterations() const
    {
        return m_solver ? m_solver->state().iteration : 0;
    }
    void abort()
    {
        m_status = ik_status::stalled;
        m_termination_reason = ik_termination_reason::solver_aborted;
    }

private:
    using nablapp_solver = nablapp::basic_solver<
        nablapp::kraft_slsqp_policy<joints>, joints, cartan::detail::nablapp_ik_problem<Chain>>;

    static constexpr bool is_inner_terminal(nablapp::solver_status s) noexcept
    {
        switch (s)
        {
            case nablapp::solver_status::converged:
            case nablapp::solver_status::ftol_reached:
            case nablapp::solver_status::xtol_reached:
            case nablapp::solver_status::stalled:
            case nablapp::solver_status::objective_stalled:
            case nablapp::solver_status::roundoff_limited:
            case nablapp::solver_status::aborted:
            case nablapp::solver_status::diverged:
            case nablapp::solver_status::max_iterations:
                return true;
            default:
                return false;
        }
    }

    static constexpr ik_termination_reason map_nablapp_status(nablapp::solver_status s) noexcept
    {
        switch (s)
        {
            case nablapp::solver_status::converged:         return ik_termination_reason::solver_converged_pose_missed;
            case nablapp::solver_status::ftol_reached:      return ik_termination_reason::solver_ftol_reached;
            case nablapp::solver_status::xtol_reached:      return ik_termination_reason::solver_xtol_reached;
            case nablapp::solver_status::stalled:           return ik_termination_reason::solver_stalled;
            case nablapp::solver_status::objective_stalled: return ik_termination_reason::solver_objective_stalled;
            case nablapp::solver_status::roundoff_limited:  return ik_termination_reason::solver_roundoff_limited;
            case nablapp::solver_status::aborted:           return ik_termination_reason::solver_aborted;
            case nablapp::solver_status::budget_exhausted:  return ik_termination_reason::solver_budget_exhausted;
            case nablapp::solver_status::max_iterations:    return ik_termination_reason::solver_max_iterations;
            case nablapp::solver_status::diverged:          return ik_termination_reason::solver_diverged;
            default:                                        return ik_termination_reason::unknown;
        }
    }

    void sync_solution_from_solver()
    {
        const auto& x = m_solver->state().x;
        int n = m_chain->num_joints();
        if constexpr (joints == dynamic)
        {
            m_q.resize(n);
        }
        for (int i = 0; i < n; ++i)
        {
            m_q[i] = static_cast<scalar_type>(x[i]);
        }
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
    std::optional<cartan::detail::nablapp_ik_problem<Chain>> m_problem;
    std::optional<nablapp_solver> m_solver;
};

}

#endif
