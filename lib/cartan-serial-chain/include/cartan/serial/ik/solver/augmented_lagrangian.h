#ifndef HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_AUGMENTED_LAGRANGIAN_H
#define HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_AUGMENTED_LAGRANGIAN_H

/// @file augmented_lagrangian.h
/// @brief argmin-backed augmented Lagrangian IK solve policy.
///
/// Wraps argmin's augmented_lagrangian_policy with lbfgsb_policy as
/// the inner solver. Uses formulation B (inequality constraints encoding
/// joint limits as g_i(q) >= 0) for the outer augmented Lagrangian and
/// box constraints for the inner L-BFGS-B subproblem.
///
/// Reference: K&W Section 10.9, N&W Section 17.4.

#include "cartan/serial/ik/ik_status.h"
#include "cartan/serial/ik/policy/error_weight.h"
#include "cartan/serial/ik/policy/limits_policy.h"
#include "cartan/serial/ik/concepts/solve_concept.h"
#include "cartan/serial/ik/detail/convergence.h"
#include "cartan/serial/ik/detail/stall_detection.h"
#include "cartan/serial/ik/detail/limit_enforcement.h"
#include "cartan/serial/ik/detail/argmin_constrained_problem.h"

#include "cartan/lie/se3.h"
#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/chain_concept.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include <argmin/solver/options.h>
#include <argmin/solver/basic_solver.h>
#include <argmin/solver/lbfgsb_policy.h>
#include <argmin/solver/augmented_lagrangian_policy.h>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace cartan::ik
{

/// argmin-backed augmented Lagrangian solve policy for constrained IK.
///
/// Converts the constrained IK problem into a sequence of bound-constrained
/// subproblems solved by L-BFGS-B. Joint limits are expressed as inequality
/// constraints (formulation B) in the outer augmented Lagrangian loop.
/// Each step() performs one outer iteration (full inner solve + multiplier
/// update), so the budget is higher than gradient-based policies.
template <chain Chain, typename LimitsPolicy = clamp_limits>
class augmented_lagrangian
{
public:
    using chain_type = Chain;
    using scalar_type = typename Chain::scalar_type;
    static constexpr int joints = Chain::joints;
    using limits_type = LimitsPolicy;

    using position_type = typename joint_state<scalar_type, joints>::position_type;

    static_assert(std::is_floating_point_v<scalar_type>, "augmented_lagrangian requires a floating-point Scalar type");

    struct options
    {
        scalar_type stall_threshold{scalar_type(1e-10)};
        scalar_type divergence_factor{scalar_type(10)};
        int stall_window{5};
    };

    augmented_lagrangian() = default;

    explicit augmented_lagrangian(const options& opts)
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

        argmin::solver_options<> nab_opts;
        // Per-attempt cap; argmin's setup-time cap is generous because the
        // per-call budget is now threaded through step_n(N) from the runner.
        nab_opts.max_iterations = m_criteria.max_iterations_per_attempt;
        nab_opts.set_gradient_threshold(1e-14);
        nab_opts.set_objective_threshold(1e-16);
        nab_opts.set_step_threshold(1e-16);

        m_solver.emplace(*m_problem, x0, nab_opts);
    }

    step_result<scalar_type> step(const Chain& chain, int N)
    {
        int units = 0;
        m_chain = &chain;
        while (units < N && m_status == ik_status::running)
        {
            // One algorithmic-work unit = one argmin-internal augmented-Lagrangian outer iteration.
            auto prev_iter = m_solver->state().outer_iter;
            auto result = m_solver->step_n(1);
            auto inner_units = static_cast<int>(m_solver->state().outer_iter - prev_iter);
            if (inner_units <= 0)
            {
                m_status = ik_status::stalled;
                break;
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
                break;
            }

            if (m_iterations >= m_criteria.max_iterations_per_attempt)
            {
                m_status = ik_status::iteration_limit;
                break;
            }

            auto stall_result = cartan::detail::check_stall_divergence(
                m_error_history, m_error_norm, m_initial_error,
                m_options.stall_window, m_options.stall_threshold,
                m_options.divergence_factor);
            if (stall_result != ik_status::running)
            {
                m_status = stall_result;
                break;
            }

            if (result.status == argmin::solver_status::converged
                || result.status == argmin::solver_status::ftol_reached
                || result.status == argmin::solver_status::stalled
                || result.status == argmin::solver_status::xtol_reached
                || result.status == argmin::solver_status::roundoff_limited
                || result.status == argmin::solver_status::objective_stalled
                || result.status == argmin::solver_status::aborted)
            {
                m_status = ik_status::stalled;
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
    void abort() { m_status = ik_status::stalled; }

private:
    using argmin_solver = argmin::basic_solver<
        argmin::augmented_lagrangian_policy<argmin::lbfgsb_policy<joints>>, joints,
        cartan::detail::argmin_constrained_ik_problem<Chain>>;

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
    std::optional<cartan::detail::argmin_constrained_ik_problem<Chain>> m_problem;
    std::optional<argmin_solver> m_solver;
};

}

#endif
