#ifndef HPP_GUARD_LIEPP_SERIAL_IK_AUGMENTED_LAGRANGIAN_SOLVE_POLICY_H
#define HPP_GUARD_LIEPP_SERIAL_IK_AUGMENTED_LAGRANGIAN_SOLVE_POLICY_H

/// @file augmented_lagrangian_solve_policy.h
/// @brief nablapp-backed augmented Lagrangian IK solve policy.
///
/// Wraps nablapp's augmented_lagrangian_policy with lbfgsb_policy as
/// the inner solver. Uses formulation B (inequality constraints encoding
/// joint limits as g_i(q) >= 0) for the outer augmented Lagrangian and
/// box constraints for the inner L-BFGS-B subproblem.
///
/// Reference: K&W Section 10.9, N&W Section 17.4.

#include "liepp/serial/ik/ik_types.h"
#include "liepp/serial/ik/error_weight.h"
#include "liepp/serial/ik/limits_policy.h"
#include "liepp/serial/ik/ik_solve_policy.h"
#include "liepp/serial/ik/detail/convergence.h"
#include "liepp/serial/ik/detail/stall_detection.h"
#include "liepp/serial/ik/detail/limit_enforcement.h"
#include "liepp/serial/ik/detail/nablapp_constrained_problem.h"

#include "liepp/lie/se3.h"
#include "liepp/serial/chain/joint_state.h"
#include "liepp/serial/chain/chain_concept.h"
#include "liepp/serial/fk/forward_kinematics.h"

#include <nablapp/solver/options.h>
#include <nablapp/solver/basic_solver.h>
#include <nablapp/solver/lbfgsb_policy.h>
#include <nablapp/solver/augmented_lagrangian_policy.h>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace liepp
{

/// nablapp-backed augmented Lagrangian solve policy for constrained IK.
///
/// Converts the constrained IK problem into a sequence of bound-constrained
/// subproblems solved by L-BFGS-B. Joint limits are expressed as inequality
/// constraints (formulation B) in the outer augmented Lagrangian loop.
/// Each step() performs one outer iteration (full inner solve + multiplier
/// update), so the budget is higher than gradient-based policies.
template <chain Chain, typename LimitsPolicy = clamp_limits>
class augmented_lagrangian_solve_policy
{
public:
    using chain_type = Chain;
    using scalar_type = typename Chain::scalar_type;
    static constexpr int joints = Chain::joints;
    using limits_type = LimitsPolicy;

    using position_type = typename joint_state<scalar_type, joints>::position_type;

    static_assert(std::is_floating_point_v<scalar_type>, "augmented_lagrangian_solve_policy requires a floating-point Scalar type");

    struct options
    {
        int budget_per_step{100};
        scalar_type stall_threshold{scalar_type(1e-10)};
        scalar_type divergence_factor{scalar_type(10)};
        int stall_window{5};
    };

    augmented_lagrangian_solve_policy() = default;

    explicit augmented_lagrangian_solve_policy(const options& opts)
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
        nab_opts.set_gradient_threshold(1e-14);
        nab_opts.set_objective_threshold(1e-16);
        nab_opts.set_step_threshold(1e-16);

        m_solver.emplace(*m_problem, x0, nab_opts);
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
            return m_status;
        }

        auto result = m_solver->step_n(m_options.budget_per_step);

        sync_solution_from_solver();

        auto fk = forward_kinematics(chain, m_q);
        auto V_b = (m_target.inverse() * fk.end_effector).log();
        m_error_norm = V_b.norm();

        if (detail::is_converged_unweighted(V_b, m_criteria))
        {
            m_status = ik_status::converged;
            return m_status;
        }

        auto stall_result = detail::check_stall_divergence(
            m_error_history, m_error_norm, m_initial_error,
            m_options.stall_window, m_options.stall_threshold,
            m_options.divergence_factor);
        if (stall_result != ik_status::running)
        {
            m_status = stall_result;
            return m_status;
        }

        if (result.status == nablapp::solver_status::converged
            || result.status == nablapp::solver_status::ftol_reached
            || result.status == nablapp::solver_status::stalled
            || result.status == nablapp::solver_status::xtol_reached
            || result.status == nablapp::solver_status::roundoff_limited
            || result.status == nablapp::solver_status::objective_stalled
            || result.status == nablapp::solver_status::aborted)
        {
            m_status = ik_status::stalled;
            return m_status;
        }

        detail::enforce_limits<LimitsPolicy>(m_q, chain);

        return m_status;
    }

    [[nodiscard]] bool converged() const { return m_status == ik_status::converged; }
    [[nodiscard]] const position_type& solution() const { return m_q; }
    [[nodiscard]] scalar_type error_norm() const { return m_error_norm; }
    [[nodiscard]] int iterations() const { return m_iterations; }
    [[nodiscard]] ik_status status() const { return m_status; }
    void abort() { m_status = ik_status::stalled; }

private:
    using nablapp_solver = nablapp::basic_solver<
        nablapp::augmented_lagrangian_policy<nablapp::lbfgsb_policy<>>>;

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
    std::optional<detail::nablapp_constrained_ik_problem<Chain>> m_problem;
    std::optional<nablapp_solver> m_solver;
};

}

#endif
