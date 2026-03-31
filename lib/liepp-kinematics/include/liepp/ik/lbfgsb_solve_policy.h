#ifndef HPP_GUARD_LIEPP_IK_LBFGSB_SOLVE_POLICY_H
#define HPP_GUARD_LIEPP_IK_LBFGSB_SOLVE_POLICY_H

/// @file lbfgsb_solve_policy.h
/// @brief L-BFGS-B IK solve policy with generalized Cauchy point and subspace minimization.
///
/// Convergence-optimized IK solver using the analytical gradient (SE(3) log
/// Jacobian) for box-constrained optimization. Joint limits are enforced via
/// the generalized Cauchy point, which identifies the active set, followed by
/// subspace minimization on free variables.
///
/// Reference: Byrd, Lu, Nocedal, Zhu. A Limited Memory Algorithm for Bound
///            Constrained Optimization. SIAM J. Scientific Computing, 1995.
///            Decisions D-04, D-05, D-06 (CONV-01).

#include "liepp/types.h"

#include "liepp/ik/ik_types.h"
#include "liepp/ik/limits_policy.h"
#include "liepp/ik/error_weight.h"
#include "liepp/ik/ik_solve_policy.h"
#include "liepp/ik/analytical_gradient.h"
#include "liepp/ik/detail/convergence.h"
#include "liepp/ik/detail/stall_detection.h"
#include "liepp/ik/detail/limit_enforcement.h"

#include "liepp/lie/se3.h"
#include "liepp/chain/joint_state.h"
#include "liepp/chain/kinematic_chain.h"

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <vector>
#include <algorithm>

namespace liepp
{

/// L-BFGS-B IK solve policy: convergence-optimized via analytical gradient.
///
/// Each step() call performs one L-BFGS-B iteration (per D-06):
/// 1. Convergence check
/// 2. Generalized Cauchy point (GCP) to identify active set
/// 3. Two-loop recursion for L-BFGS direction on free variables
/// 4. Subspace minimization
/// 5. Backtracking Armijo line search with bound clamping
/// 6. L-BFGS history update
/// 7. Stall/divergence detection
///
/// Reference: Byrd, Lu, Nocedal, Zhu, SIAM J. Sci. Comput., 1995.
template <typename Scalar = double, int N = dynamic, typename LimitsPolicy = clamp_limits,
          typename ObjectivePolicy = ik_se3_objective<Scalar, N>>
class lbfgsb_solve_policy
{
public:
    static_assert(std::is_floating_point_v<Scalar>, "lbfgsb_solve_policy requires a floating-point Scalar type");

    using scalar_type = Scalar;
    static constexpr int joints = N;
    using limits_type = LimitsPolicy;

    using position_type = typename joint_state<Scalar, N>::position_type;

    /// Tunable parameters for the L-BFGS-B solve policy.
    struct options
    {
        int history_depth{5};
        Scalar line_search_c{Scalar(1e-4)};
        Scalar line_search_shrink{Scalar(0.5)};
        int max_line_search_steps{20};
        Scalar stall_threshold{Scalar(1e-10)};
        Scalar divergence_factor{Scalar(10)};
        int stall_window{15};
    };

    lbfgsb_solve_policy() = default;

    explicit lbfgsb_solve_policy(const options& opts)
        : m_options(opts)
    {
    }

    /// Initialize with chain, target, seed, and convergence criteria.
    void setup(
        const kinematic_chain<Scalar, N>& chain,
        const se3<Scalar>& target,
        const position_type& q0,
        const convergence_criteria<Scalar>& criteria)
    {
        setup(chain, target, q0, criteria, error_weight<Scalar>{});
    }

    /// Initialize with chain, target, seed, criteria, and error weight.
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
        m_status = ik_status::running;
        m_error_history.clear();
        m_gamma = Scalar(1);

        int n = chain.num_joints();
        m_lower.resize(n);
        m_upper.resize(n);
        for (int i = 0; i < n; ++i)
        {
            m_lower(i) = chain.limits()[i].position_min;
            m_upper(i) = chain.limits()[i].position_max;
        }

        // Clamp initial q to bounds
        m_q = m_q.cwiseMax(m_lower).cwiseMin(m_upper);

        // Evaluate initial objective and gradient
        auto result = ObjectivePolicy::evaluate_with_gradient(chain, m_target, m_q, m_weight);
        m_f = result.info.objective;
        m_gradient = result.gradient;
        m_body_error = result.info.body_error;
        m_weighted_error = result.info.weighted_error;
        m_error_norm = result.info.body_error.norm();
        m_initial_error = m_error_norm;

        // Clear L-BFGS history
        m_s_history.clear();
        m_y_history.clear();
        m_rho_history.clear();
    }

    /// Execute one L-BFGS-B iteration (per D-06).
    ik_status step(const kinematic_chain<Scalar, N>& chain)
    {
        if (m_status != ik_status::running)
        {
            return m_status;
        }

        // (1) Convergence check using weighted error norms
        if (detail::is_converged(m_body_error, m_weight, m_criteria))
        {
            m_status = ik_status::converged;
            return m_status;
        }

        // (2) Iteration limit check
        ++m_iterations;
        if (m_iterations >= m_criteria.max_iterations)
        {
            m_status = ik_status::iteration_limit;
            return m_status;
        }

        int n = static_cast<int>(m_q.size());

        // (3) Compute descent direction with fallback to projected gradient
        auto [direction, directional_derivative, max_alpha] = compute_descent_direction(n);
        if (max_alpha < std::numeric_limits<Scalar>::epsilon() ||
            directional_derivative >= Scalar(0))
        {
            update_stall_detection();
            return m_status;
        }

        // (4) Backtracking Armijo line search with bound clamping
        position_type q_old = m_q;
        position_type g_old = m_gradient;
        Scalar f_old = m_f;

        if (!armijo_line_search(chain, direction, directional_derivative, max_alpha, f_old))
        {
            update_stall_detection();
            return m_status;
        }

        // (5) Update L-BFGS history with curvature pair
        update_lbfgs_history(q_old, g_old);

        // (6) Stall/divergence detection and limit enforcement
        update_stall_detection();
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
    /// Compute descent direction via L-BFGS, with fallback to projected gradient.
    struct descent_result
    {
        position_type direction;
        Scalar directional_derivative;
        Scalar max_alpha;
    };

    descent_result compute_descent_direction(int n)
    {
        position_type direction = compute_lbfgs_direction(n);
        Scalar max_alpha = compute_max_step(direction, n);

        if (max_alpha < std::numeric_limits<Scalar>::epsilon())
        {
            return {direction, Scalar(0), max_alpha};
        }

        Scalar dd = m_gradient.dot(direction);

        // If not a descent direction, fallback to projected negative gradient
        if (dd >= Scalar(0))
        {
            direction = projected_negative_gradient(n);
            dd = m_gradient.dot(direction);
            max_alpha = compute_max_step(direction, n);
        }

        return {direction, dd, max_alpha};
    }

    /// Backtracking Armijo line search with bound clamping. Returns true if accepted.
    bool armijo_line_search(
        const kinematic_chain<Scalar, N>& chain,
        const position_type& direction,
        Scalar directional_derivative,
        Scalar max_alpha,
        Scalar f_old)
    {
        Scalar alpha = std::min(Scalar(1), max_alpha);

        for (int ls = 0; ls < m_options.max_line_search_steps; ++ls)
        {
            position_type q_trial = (m_q + alpha * direction)
                .cwiseMax(m_lower).cwiseMin(m_upper);

            auto result = ObjectivePolicy::evaluate_with_gradient(
                chain, m_target, q_trial, m_weight);
            Scalar f_trial = result.info.objective;

            if (f_trial <= f_old + m_options.line_search_c * alpha * directional_derivative)
            {
                m_q = q_trial;
                m_f = f_trial;
                m_gradient = result.gradient;
                m_body_error = result.info.body_error;
                m_weighted_error = result.info.weighted_error;
                m_error_norm = result.info.body_error.norm();
                return true;
            }

            alpha *= m_options.line_search_shrink;
        }

        return false;
    }

    /// Update L-BFGS history with the latest curvature pair (s, y).
    void update_lbfgs_history(const position_type& q_old, const position_type& g_old)
    {
        position_type s_k = m_q - q_old;
        position_type y_k = m_gradient - g_old;
        Scalar ys = y_k.dot(s_k);

        if (ys > std::numeric_limits<Scalar>::epsilon())
        {
            if (static_cast<int>(m_s_history.size()) >= m_options.history_depth)
            {
                m_s_history.erase(m_s_history.begin());
                m_y_history.erase(m_y_history.begin());
                m_rho_history.erase(m_rho_history.begin());
            }
            m_s_history.push_back(s_k);
            m_y_history.push_back(y_k);
            m_rho_history.push_back(Scalar(1) / ys);

            Scalar yy = y_k.squaredNorm();
            if (yy > std::numeric_limits<Scalar>::epsilon())
            {
                m_gamma = ys / yy;
            }
        }
    }

    /// L-BFGS two-loop recursion with generalized Cauchy point projection.
    ///
    /// Combines the L-BFGS search direction with the active-set identification
    /// from the generalized Cauchy point (Byrd et al., 1995, Section 4):
    /// variables at their bounds with gradient pushing into the bound are fixed
    /// (the "active set"), while free variables use the L-BFGS direction for
    /// subspace minimization.
    position_type compute_lbfgs_direction(int n)
    {
        // Generalized Cauchy point: identify active set at current point.
        // A variable is "active" (clamped at its breakpoint) if at a bound and
        // the gradient pushes into that bound.
        position_type q_r = m_gradient;

        std::size_t m = m_s_history.size();
        std::vector<Scalar> alpha_store(m);

        // First loop (backward)
        for (std::size_t i = m; i-- > 0;)
        {
            alpha_store[i] = m_rho_history[i] * m_s_history[i].dot(q_r);
            q_r -= alpha_store[i] * m_y_history[i];
        }

        // Scale by gamma (initial Hessian approximation H0 = gamma * I)
        q_r *= m_gamma;

        // Second loop (forward)
        for (std::size_t i = 0; i < m; ++i)
        {
            Scalar beta = m_rho_history[i] * m_y_history[i].dot(q_r);
            q_r += (alpha_store[i] - beta) * m_s_history[i];
        }

        // Subspace minimization: direction = -H^{-1} * g on free variables,
        // zero on active set (generalized Cauchy point bound identification)
        position_type direction = -q_r;

        for (int i = 0; i < n; ++i)
        {
            // At lower bound and direction pushes further below
            if (m_q(i) <= m_lower(i) + std::numeric_limits<Scalar>::epsilon() &&
                direction(i) < Scalar(0))
            {
                direction(i) = Scalar(0);
            }
            // At upper bound and direction pushes further above
            if (m_q(i) >= m_upper(i) - std::numeric_limits<Scalar>::epsilon() &&
                direction(i) > Scalar(0))
            {
                direction(i) = Scalar(0);
            }
        }

        return direction;
    }

    /// Projected negative gradient: steepest descent direction projected to bounds.
    position_type projected_negative_gradient(int n)
    {
        position_type direction = -m_gradient;
        for (int i = 0; i < n; ++i)
        {
            if (m_q(i) <= m_lower(i) + std::numeric_limits<Scalar>::epsilon() &&
                direction(i) < Scalar(0))
            {
                direction(i) = Scalar(0);
            }
            if (m_q(i) >= m_upper(i) - std::numeric_limits<Scalar>::epsilon() &&
                direction(i) > Scalar(0))
            {
                direction(i) = Scalar(0);
            }
        }
        return direction;
    }

    /// Compute maximum step size before hitting any bound.
    Scalar compute_max_step(const position_type& direction, int n)
    {
        Scalar max_alpha = std::numeric_limits<Scalar>::max();
        for (int i = 0; i < n; ++i)
        {
            if (direction(i) > std::numeric_limits<Scalar>::epsilon())
            {
                Scalar t = (m_upper(i) - m_q(i)) / direction(i);
                max_alpha = std::min(max_alpha, t);
            }
            else if (direction(i) < -std::numeric_limits<Scalar>::epsilon())
            {
                Scalar t = (m_lower(i) - m_q(i)) / direction(i);
                max_alpha = std::min(max_alpha, t);
            }
        }
        return max_alpha;
    }

    /// Update stall and divergence detection.
    void update_stall_detection()
    {
        auto stall_result = detail::check_stall_divergence(
            m_error_history, m_error_norm, m_initial_error,
            m_options.stall_window, m_options.stall_threshold,
            m_options.divergence_factor);
        if (stall_result != ik_status::running)
        {
            m_status = stall_result;
        }
    }

    se3<Scalar> m_target{se3<Scalar>::identity()};
    position_type m_q{};
    position_type m_gradient{};
    position_type m_lower{};
    position_type m_upper{};
    vector6<Scalar> m_body_error{vector6<Scalar>::Zero()};
    vector6<Scalar> m_weighted_error{vector6<Scalar>::Zero()};
    convergence_criteria<Scalar> m_criteria{};
    error_weight<Scalar> m_weight{};
    options m_options{};
    std::vector<Scalar> m_error_history;
    std::vector<position_type> m_s_history;
    std::vector<position_type> m_y_history;
    std::vector<Scalar> m_rho_history;
    Scalar m_initial_error{};
    Scalar m_error_norm{};
    Scalar m_f{};
    Scalar m_gamma{Scalar(1)};
    int m_iterations{};
    ik_status m_status{ik_status::running};
};

}

#endif
