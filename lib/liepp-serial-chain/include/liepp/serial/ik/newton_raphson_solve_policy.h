#ifndef HPP_GUARD_LIEPP_SERIAL_IK_NEWTON_RAPHSON_SOLVE_POLICY_H
#define HPP_GUARD_LIEPP_SERIAL_IK_NEWTON_RAPHSON_SOLVE_POLICY_H

/// @file newton_raphson_solve_policy.h
/// @brief Newton-Raphson IK solve policy with undamped Gauss-Newton Hessian
///        and backtracking Armijo line search for globalization.
///
/// Uses J^T J (no damping) as the Gauss-Newton Hessian approximation,
/// with small diagonal regularization for singularity handling. The
/// backtracking line search provides globalization where LM uses its
/// damping parameter, yielding a different convergence basin that is
/// valuable for racing diversity.
///
/// Note: For a Gauss-Newton solver without line search, use
/// projected_lm_solve_policy with options{.initial_lambda_factor = 0.0}.
/// This solve policy adds backtracking line search for globalization,
/// which changes convergence behavior vs the damping-based LM approach.
///
/// Reference: Nocedal & Wright, Numerical Optimization, Ch. 3 (line search),
///            Ch. 10 (nonlinear least squares, Gauss-Newton).
///            Decisions D-05, D-06, D-08.

#include "liepp/types.h"

#include "liepp/serial/ik/ik_types.h"
#include "liepp/serial/ik/limits_policy.h"
#include "liepp/serial/ik/error_weight.h"
#include "liepp/serial/ik/ik_solve_policy.h"
#include "liepp/serial/ik/analytical_gradient.h"
#include "liepp/serial/ik/detail/convergence.h"
#include "liepp/serial/ik/detail/stall_detection.h"
#include "liepp/serial/ik/detail/limit_enforcement.h"

#include "liepp/lie/se3.h"
#include "liepp/serial/chain/joint_state.h"
#include "liepp/serial/chain/kinematic_chain.h"
#include "liepp/serial/fk/jacobian.h"
#include "liepp/serial/fk/forward_kinematics.h"

#include <Eigen/Dense>

#include <cmath>
#include <vector>
#include <algorithm>

namespace liepp
{

/// Newton-Raphson IK solve policy with Gauss-Newton Hessian and Armijo line search.
///
/// Each step() call:
/// 1. Forward kinematics and body-frame error
/// 2. Convergence check (weighted angular/linear norms)
/// 3. Analytical gradient via evaluate_with_gradient (SE(3) log Jacobian)
/// 4. Build regularized Gauss-Newton Hessian: H = J_b^T J_b + epsilon * I
/// 5. Newton descent: dq = -H^{-1} * grad
/// 6. Backtracking Armijo line search with bound clamping
/// 7. Stall/divergence detection
///
/// Reference: Nocedal & Wright, Numerical Optimization, Ch. 3, 10.
template <typename Scalar = double, int N = dynamic, typename LimitsPolicy = clamp_limits,
          typename ObjectivePolicy = ik_se3_objective<Scalar, N>>
class newton_raphson_solve_policy
{
public:
    static_assert(std::is_floating_point_v<Scalar>, "newton_raphson_solve_policy requires a floating-point Scalar type");

    using scalar_type = Scalar;
    static constexpr int joints = N;
    using limits_type = LimitsPolicy;

    using position_type = typename joint_state<Scalar, N>::position_type;

    /// Tunable parameters for the Newton-Raphson solve policy.
    struct options
    {
        Scalar epsilon{Scalar(1e-8)};             ///< Diagonal regularization for singular J^T J
        Scalar line_search_c{Scalar(1e-4)};       ///< Armijo sufficient decrease constant
        Scalar line_search_shrink{Scalar(0.5)};   ///< Step size reduction factor
        int max_line_search_steps{20};             ///< Max backtracking iterations
        Scalar stall_threshold{Scalar(1e-10)};    ///< Min error improvement
        Scalar divergence_factor{Scalar(10)};     ///< Max error growth before diverged
        int stall_window{10};                     ///< Consecutive stall iterations before stalled
    };

    newton_raphson_solve_policy() = default;

    explicit newton_raphson_solve_policy(const options& opts)
        : m_options(opts)
    {
    }

    /// Initialize the solve policy (satisfies ik_solve_policy concept).
    void setup(
        const kinematic_chain<Scalar, N>& chain,
        const se3<Scalar>& target,
        const position_type& q0,
        const convergence_criteria<Scalar>& criteria)
    {
        setup(chain, target, q0, criteria, error_weight<Scalar>{});
    }

    /// Initialize with error weighting for position/orientation emphasis.
    void setup(
        const kinematic_chain<Scalar, N>& chain,
        const se3<Scalar>& target,
        const position_type& q0,
        const convergence_criteria<Scalar>& criteria,
        const error_weight<Scalar>& weight)
    {
        m_target = target;
        m_q = q0;
        m_criteria = criteria;
        m_weight = weight;
        m_iterations = 0;
        m_stall_count = 0;
        m_status = ik_status::running;
        m_error_history.clear();

        int n = chain.num_joints();
        if constexpr (N == dynamic)
        {
            m_lower.resize(n);
            m_upper.resize(n);
        }
        for (int i = 0; i < n; ++i)
        {
            m_lower(i) = chain.limits()[static_cast<std::size_t>(i)].position_min;
            m_upper(i) = chain.limits()[static_cast<std::size_t>(i)].position_max;
        }

        // Clamp initial seed to limits
        m_q = m_q.cwiseMax(m_lower).cwiseMin(m_upper);

        // Evaluate initial objective
        auto result = ObjectivePolicy::evaluate(chain, m_target, m_q, m_weight);
        m_error_norm = result.body_error.norm();
        m_initial_error = m_error_norm;
    }

    /// Execute one Newton-Raphson iteration with Armijo line search.
    ik_status step(const kinematic_chain<Scalar, N>& chain)
    {
        if (m_status != ik_status::running)
        {
            return m_status;
        }

        // (1) Evaluate objective, gradient, and body error
        auto result = ObjectivePolicy::evaluate_with_gradient(
            chain, m_target, m_q, m_weight);
        Scalar f_current = result.info.objective;
        auto& grad = result.gradient;
        auto& body_error = result.info.body_error;

        // (2) Weighted convergence check
        if (detail::is_converged(body_error, m_weight, m_criteria))
        {
            m_error_norm = body_error.norm();
            m_status = ik_status::converged;
            return m_status;
        }

        // (3) Iteration limit
        ++m_iterations;
        if (m_iterations >= m_criteria.max_iterations)
        {
            m_error_norm = body_error.norm();
            m_status = ik_status::iteration_limit;
            return m_status;
        }

        // (4) Body Jacobian for Gauss-Newton Hessian
        auto fk = forward_kinematics(chain, m_q);
        auto J_b = body_jacobian(chain, fk);
        int n = static_cast<int>(J_b.cols());

        // (5) Regularized Gauss-Newton Hessian: H = J_b^T J_b + epsilon * I
        auto H = (J_b.transpose() * J_b).eval();
        for (int i = 0; i < n; ++i)
        {
            H(i, i) += m_options.epsilon;
        }

        // (6) Newton descent direction: dq = -H^{-1} * grad
        position_type dq = -H.ldlt().solve(grad);

        // (7) Backtracking Armijo line search with bound clamping
        Scalar directional_derivative = grad.dot(dq); // Should be negative
        Scalar alpha = Scalar(1);
        bool step_accepted = false;
        position_type q_trial = m_q;

        for (int ls = 0; ls < m_options.max_line_search_steps; ++ls)
        {
            q_trial = (m_q + alpha * dq).cwiseMax(m_lower).cwiseMin(m_upper);

            auto trial = ObjectivePolicy::evaluate(chain, m_target, q_trial, m_weight);
            Scalar f_trial = trial.objective;

            if (f_trial <= f_current + m_options.line_search_c * alpha * directional_derivative)
            {
                m_q = q_trial;
                m_error_norm = trial.body_error.norm();
                step_accepted = true;
                break;
            }

            alpha *= m_options.line_search_shrink;
        }

        if (!step_accepted)
        {
            // Accept smallest step anyway to avoid getting stuck
            m_q = q_trial;
            m_error_norm = ObjectivePolicy::evaluate(chain, m_target, m_q, m_weight)
                .body_error.norm();
        }

        // (8) Stall and divergence detection
        auto stall_result = detail::check_stall_divergence(
            m_error_history, m_error_norm, m_initial_error,
            m_options.stall_window, m_options.stall_threshold,
            m_options.divergence_factor);
        if (stall_result != ik_status::running)
        {
            m_status = stall_result;
            return m_status;
        }

        // Enforce joint limits via LimitsPolicy
        detail::enforce_limits<LimitsPolicy>(m_q, chain);

        return m_status;
    }

    [[nodiscard]] bool converged() const { return m_status == ik_status::converged; }
    [[nodiscard]] const position_type& solution() const { return m_q; }
    [[nodiscard]] Scalar error_norm() const { return m_error_norm; }
    [[nodiscard]] int iterations() const { return m_iterations; }
    void abort() {}
    [[nodiscard]] ik_status status() const { return m_status; }

private:
    se3<Scalar> m_target{se3<Scalar>::identity()};
    position_type m_q{};
    position_type m_lower{};
    position_type m_upper{};
    convergence_criteria<Scalar> m_criteria{};
    error_weight<Scalar> m_weight{};
    options m_options{};
    std::vector<Scalar> m_error_history;
    Scalar m_initial_error{std::numeric_limits<Scalar>::max()};
    Scalar m_error_norm{std::numeric_limits<Scalar>::max()};
    int m_iterations{};
    int m_stall_count{};
    ik_status m_status{ik_status::running};
};

}

#endif
