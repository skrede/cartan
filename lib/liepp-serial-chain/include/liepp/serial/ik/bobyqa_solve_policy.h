#ifndef HPP_GUARD_LIEPP_IK_BOBYQA_SOLVE_POLICY_H
#define HPP_GUARD_LIEPP_IK_BOBYQA_SOLVE_POLICY_H

/// @file bobyqa_solve_policy.h
/// @brief nablapp-backed BOBYQA derivative-free IK solve policy with box constraints.
///
/// Wraps nablapp's bobyqa_policy for constrained IK, using joint limits
/// as box constraints. Derivative-free -- uses only objective evaluations.
/// Always available -- nablapp is a required dependency of liepp::kinematics.
///
/// Reference: Powell 2009, BOBYQA algorithm for bound constrained optimization.

#include "liepp/ik/ik_types.h"
#include "liepp/ik/error_weight.h"
#include "liepp/ik/limits_policy.h"
#include "liepp/ik/ik_solve_policy.h"
#include "liepp/ik/detail/convergence.h"
#include "liepp/ik/detail/nablapp_problem.h"
#include "liepp/ik/detail/stall_detection.h"
#include "liepp/ik/detail/limit_enforcement.h"

#include "liepp/lie/se3.h"
#include "liepp/chain/joint_state.h"
#include "liepp/chain/kinematic_chain.h"
#include "liepp/kinematics/forward_kinematics.h"

#include <nablapp/solver/basic_solver.h>
#include <nablapp/solver/bobyqa_policy.h>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace liepp
{

/// nablapp-backed BOBYQA solve policy for constrained IK with box constraints.
///
/// Uses Powell's Bound Optimization BY Quadratic Approximation via nablapp.
/// Derivative-free: builds a quadratic interpolation model of the objective
/// and uses trust-region steps. Each step() call runs a budget of nablapp
/// iterations for cooperative scheduling in basic_ik_solver.
///
/// This is the default (unprefixed) BOBYQA policy. The NLopt-backed variant
/// is available as nlopt_bobyqa_solve_policy behind LIEPP_HAS_NLOPT.
template <typename Scalar = double, int N = dynamic, typename LimitsPolicy = clamp_limits>
class bobyqa_solve_policy
{
public:
    static_assert(std::is_floating_point_v<Scalar>, "bobyqa_solve_policy requires a floating-point Scalar type");

    using scalar_type = Scalar;
    static constexpr int joints = N;
    using limits_type = LimitsPolicy;

    using position_type = typename joint_state<Scalar, N>::position_type;

    struct options
    {
        int budget_per_step{50};
        Scalar stall_threshold{Scalar(1e-10)};
        Scalar divergence_factor{Scalar(10)};
        int stall_window{5};
    };

    bobyqa_solve_policy() = default;

    explicit bobyqa_solve_policy(const options& opts)
        : m_options{opts}
    {}

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
        m_error_norm = std::numeric_limits<Scalar>::max();
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

        nablapp::solver_options<double> nab_opts;
        nab_opts.max_iterations = m_options.budget_per_step;
        nab_opts.gradient_tolerance = 0.0;
        nab_opts.objective_tolerance = 1e-14;
        nab_opts.step_tolerance = 1e-14;

        m_solver.emplace(*m_problem, x0, nab_opts);
    }

    ik_status step(const kinematic_chain<Scalar, N>& chain)
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
            || result.status == nablapp::solver_status::stalled)
        {
            m_status = ik_status::stalled;
            return m_status;
        }

        detail::enforce_limits<LimitsPolicy>(m_q, chain);

        return m_status;
    }

    [[nodiscard]] bool converged() const { return m_status == ik_status::converged; }
    [[nodiscard]] const position_type& solution() const { return m_q; }
    [[nodiscard]] Scalar error_norm() const { return m_error_norm; }
    [[nodiscard]] int iterations() const { return m_iterations; }
    [[nodiscard]] ik_status status() const { return m_status; }
    void abort() { m_status = ik_status::stalled; }

private:
    using nablapp_solver = nablapp::basic_solver<nablapp::bobyqa_policy>;

    void sync_solution_from_solver()
    {
        const auto& x = m_solver->state().x;
        int n = m_chain->num_joints();
        if constexpr (N == dynamic)
        {
            m_q.resize(n);
        }
        for (int i = 0; i < n; ++i)
        {
            m_q[i] = static_cast<Scalar>(x[i]);
        }
    }

    const kinematic_chain<Scalar, N>* m_chain{nullptr};
    se3<Scalar> m_target{se3<Scalar>::identity()};
    convergence_criteria<Scalar> m_criteria{};
    error_weight<Scalar> m_weight{};
    options m_options{};
    position_type m_q{};
    std::vector<Scalar> m_error_history;
    Scalar m_initial_error{};
    Scalar m_error_norm{std::numeric_limits<Scalar>::max()};
    int m_iterations{};
    ik_status m_status{ik_status::running};
    std::optional<detail::nablapp_ik_problem<Scalar, N>> m_problem;
    std::optional<nablapp_solver> m_solver;
};

template <typename Scalar, int N, typename LimitsPolicy>
bobyqa_solve_policy(const kinematic_chain<Scalar, N>&, const se3<Scalar>&,
                    const typename joint_state<Scalar, N>::position_type&,
                    LimitsPolicy) -> bobyqa_solve_policy<Scalar, N, LimitsPolicy>;

}

#endif
