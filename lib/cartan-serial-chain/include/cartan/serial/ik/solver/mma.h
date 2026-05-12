#ifndef HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_MMA_H
#define HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_MMA_H

/// @file mma.h
/// @brief argmin-backed Method of Moving Asymptotes IK solve policy.
///
/// Wraps argmin's mma_policy over a bound-constrained IK problem.
/// Joint limits are passed through as box bounds and MMA builds
/// Svanberg's convex separable reciprocal approximation around them.
/// No outer augmented-Lagrangian layer is required since MMA handles
/// the box-constrained formulation natively.
///
/// Reference: Svanberg 1987, "The method of moving asymptotes",
///            IJNME 24:359-373. Svanberg 2002, "A class of globally
///            convergent optimization methods based on conservative
///            convex separable approximations", SIAM J. Optim. 12(2).

#include "cartan/serial/ik/ik_status.h"
#include "cartan/serial/ik/policy/error_weight.h"
#include "cartan/serial/ik/policy/limits_policy.h"
#include "cartan/serial/ik/concepts/solve_concept.h"
#include "cartan/serial/ik/detail/convergence.h"
#include "cartan/serial/ik/detail/stall_detection.h"
#include "cartan/serial/ik/detail/limit_enforcement.h"
#include "cartan/serial/ik/detail/argmin_bounded_ik_problem.h"

#include "cartan/lie/se3.h"
#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/chain_concept.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include <argmin/solver/options.h>
#include <argmin/solver/basic_solver.h>
#include <argmin/solver/mma_policy.h>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <cstdint>
#include <optional>
#include <vector>
#include <algorithm>

namespace cartan::ik
{

/// argmin-backed MMA solve policy for bound-constrained IK.
///
/// Each step() performs one outer Svanberg iteration (asymptote update +
/// conservative convex subproblem solve). Joint limits enter as box
/// bounds; no inequality constraints are emitted. The policy-internal
/// stall cascade (asymptote-floor, rho-saturated, KKT-jump) surfaces
/// through step_result.policy_status and is left at argmin defaults
/// unless overridden via mma_options.
template <chain Chain, typename LimitsPolicy = clamp_limits>
class mma
{
public:
    using chain_type = Chain;
    using scalar_type = typename Chain::scalar_type;
    static constexpr int joints = Chain::joints;
    using limits_type = LimitsPolicy;

    using position_type = typename joint_state<scalar_type, joints>::position_type;

    static_assert(std::is_floating_point_v<scalar_type>, "mma requires a floating-point Scalar type");

    struct options
    {
        scalar_type stall_threshold{scalar_type(1e-10)};
        scalar_type divergence_factor{scalar_type(10)};
        int stall_window{5};

        // Pass-through of argmin mma_policy options. Leaving fields
        // unset keeps argmin's defaults (K_asymptote = K_saturation = 5,
        // kkt_jump_threshold_factor = 1000, stall_tolerance_threshold
        // resolved to 1e-6 by basic_solver::forward_policy_hints).
        typename argmin::mma_policy<joints>::options_type policy_options{};

        double gradient_threshold{1e-14};
        double objective_threshold{1e-16};
        double step_threshold{1e-16};
    };

    mma() = default;

    explicit mma(const options& opts)
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

        // argmin's iterations_ is a cumulative counter across all
        // step_n() calls; when iterations_ >= opts.max_iterations step_n
        // early-returns with max_iterations status and does no work. The
        // cartan wrapper loops step_n(1) many times, so max_iterations must
        // be a large upper bound — the cartan-level max_iterations_per_attempt
        // (and the runner-level max_total_work_units) drive outer termination.
        argmin::solver_options<> nab_opts;
        nab_opts.max_iterations = static_cast<std::uint32_t>(
            std::max<int>(m_criteria.max_iterations_per_attempt, 100000));
        nab_opts.set_gradient_threshold(m_options.gradient_threshold);
        nab_opts.set_objective_threshold(m_options.objective_threshold);
        nab_opts.set_step_threshold(m_options.step_threshold);

        m_solver.emplace(*m_problem, x0, nab_opts, m_options.policy_options);
    }

    step_result<scalar_type> step(const Chain& chain, int N)
    {
        int units = 0;
        m_chain = &chain;
        while (units < N && m_status == ik_status::running)
        {
            // One algorithmic-work unit = one argmin-internal MMA outer iteration.
            auto prev_iter = m_solver->state().iteration;
            auto result = m_solver->step_n(1);
            auto inner_units = static_cast<int>(m_solver->state().iteration - prev_iter);
            if (inner_units <= 0)
            {
                m_status = ik_status::stalled;
                break;
            }
            m_iterations += inner_units;
            units += inner_units;

            // argmin step_n returns the best-seen iterate in result.x (not
            // state().x, which is the last trial). For oscillation-prone
            // policies like MMA/GCMMA this distinction matters — read from
            // result.x so downstream FK / convergence checks see the iterate
            // argmin actually endorses.
            sync_solution_from_result(result.x);

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
        argmin::mma_policy<joints>, joints,
        cartan::detail::argmin_bounded_ik_problem<Chain>>;

    template <typename Derived>
    void sync_solution_from_result(const Eigen::MatrixBase<Derived>& x)
    {
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
    std::optional<cartan::detail::argmin_bounded_ik_problem<Chain>> m_problem;
    std::optional<argmin_solver> m_solver;
};

}

#endif
