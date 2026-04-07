#ifndef HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_ARGMIN_BOBYQA_H
#define HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_ARGMIN_BOBYQA_H

/// @file argmin_bobyqa.h
/// @brief nablapp-backed BOBYQA derivative-free IK solve policy with box constraints.
///
/// Wraps nablapp's bobyqa_policy for constrained IK, using joint limits
/// as box constraints. Derivative-free -- uses only objective evaluations.
/// Always available -- nablapp is a required dependency of cartan::kinematics.
///
/// Reference: Powell 2009, BOBYQA algorithm for bound constrained optimization.

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
#include <nablapp/solver/bobyqa_policy.h>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace cartan::ik
{

/// nablapp-backed BOBYQA solve policy for constrained IK with box constraints.
///
/// Uses Powell's Bound Optimization BY Quadratic Approximation via nablapp.
/// Derivative-free: builds a quadratic interpolation model of the objective
/// and uses trust-region steps. Each step() call runs a budget of nablapp
/// iterations for cooperative scheduling in basic_ik_solver.
///
/// This is the default (unprefixed) BOBYQA policy. The NLopt-backed variant
/// is available as nlopt_bobyqa_solve_policy behind CARTAN_HAS_NLOPT.
template <chain Chain, typename LimitsPolicy = clamp_limits>
class argmin_bobyqa
{
public:
    using chain_type = Chain;
    using scalar_type = typename Chain::scalar_type;
    static constexpr int joints = Chain::joints;
    using limits_type = LimitsPolicy;

    using position_type = typename joint_state<scalar_type, joints>::position_type;

    static_assert(std::is_floating_point_v<scalar_type>, "argmin_bobyqa requires a floating-point Scalar type");

    struct options
    {
        int budget_per_step{50};
        scalar_type stall_threshold{scalar_type(1e-10)};
        scalar_type divergence_factor{scalar_type(10)};
        int stall_window{5};
    };

    bobyqa_solve_policy() = default;

    explicit bobyqa_solve_policy(const options& opts)
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
        nab_opts.set_gradient_threshold(0.0);
        nab_opts.set_objective_threshold(1e-14);
        nab_opts.set_stationarity_threshold(std::numeric_limits<double>::infinity());
        nab_opts.set_step_threshold(1e-14);

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
        nablapp::bobyqa_policy<joints>, joints, detail::nablapp_ik_problem<Chain>>;

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
    std::optional<detail::nablapp_ik_problem<Chain>> m_problem;
    std::optional<nablapp_solver> m_solver;
};

}

#endif
