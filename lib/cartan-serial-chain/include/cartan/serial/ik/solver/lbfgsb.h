#ifndef HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_LBFGSB_H
#define HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_LBFGSB_H

/// @file lbfgsb.h
/// @brief L-BFGS-B IK solve policy with generalized Cauchy point and subspace minimization.
///
/// Convergence-optimized IK solver using the analytical gradient (SE(3) log
/// Jacobian) for box-constrained optimization. Joint limits are enforced via
/// the generalized Cauchy point, which identifies the active set, followed by
/// subspace minimization on free variables.
///
/// Reference: Byrd, Lu, Nocedal, Zhu. A Limited Memory Algorithm for Bound
///            Constrained Optimization. SIAM J. Scientific Computing, 1995.

#include "cartan/types.h"

#include "cartan/serial/ik/ik_status.h"
#include "cartan/serial/ik/policy/limits_policy.h"
#include "cartan/serial/ik/policy/error_weight.h"
#include "cartan/serial/ik/concepts/solve_concept.h"
#include "cartan/serial/ik/solver/detail/analytical_gradient.h"
#include "cartan/serial/ik/detail/convergence.h"
#include "cartan/serial/ik/detail/stall_detection.h"
#include "cartan/serial/ik/detail/limit_enforcement.h"

#include "cartan/lie/se3.h"
#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/chain_concept.h"

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <vector>
#include <algorithm>

namespace cartan::ik
{

/// L-BFGS-B IK solve policy: convergence-optimized via analytical gradient.
///
/// Each step() call performs one L-BFGS-B iteration:
/// 1. Convergence check
/// 2. Generalized Cauchy point (GCP) to identify active set
/// 3. Two-loop recursion for L-BFGS direction on free variables
/// 4. Subspace minimization
/// 5. Backtracking Armijo line search with bound clamping
/// 6. L-BFGS history update
/// 7. Stall/divergence detection
///
/// Reference: Byrd, Lu, Nocedal, Zhu, SIAM J. Sci. Comput., 1995.
template <chain Chain, typename LimitsPolicy = clamp_limits,
          typename ObjectivePolicy = ik_se3_objective<Chain>>
class builtin_lbfgsb
{
public:
    using chain_type = Chain;
    using scalar_type = typename Chain::scalar_type;
    static constexpr int joints = Chain::joints;
    using limits_type = LimitsPolicy;

    using position_type = typename joint_state<scalar_type, joints>::position_type;

    static_assert(std::is_floating_point_v<scalar_type>, "builtin_lbfgsb requires a floating-point Scalar type");

    struct options
    {
        int history_depth{5};
        scalar_type line_search_c{scalar_type(1e-4)};
        scalar_type line_search_shrink{scalar_type(0.5)};
        int max_line_search_steps{20};
        scalar_type stall_threshold{scalar_type(1e-10)};
        scalar_type divergence_factor{scalar_type(10)};
        int stall_window{15};
    };

    builtin_lbfgsb() = default;

    explicit builtin_lbfgsb(const options& opts)
        : m_options(opts)
    {
    }

    void setup(
        const Chain& chain,
        const se3<scalar_type>& target,
        const position_type& q0,
        const convergence_criteria<scalar_type>& criteria)
    {
        setup(chain, target, q0, criteria, error_weight<scalar_type>{});
    }

    void setup(
        const Chain& chain,
        const se3<scalar_type>& target,
        const position_type& q0,
        const convergence_criteria<scalar_type>& criteria,
        const error_weight<scalar_type>& weight)
    {
        m_target = target;
        m_q = q0;
        m_criteria = criteria;
        m_weight = weight;
        m_iterations = 0;
        m_status = ik_status::running;
        m_error_history.clear();
        m_gamma = scalar_type(1);

        int n = chain.num_joints();
        m_lower.resize(n);
        m_upper.resize(n);
        for (int i = 0; i < n; ++i)
        {
            m_lower(i) = chain.limits()[i].position_min;
            m_upper(i) = chain.limits()[i].position_max;
        }

        m_q = m_q.cwiseMax(m_lower).cwiseMin(m_upper);

        auto result = ObjectivePolicy::evaluate_with_gradient(chain, m_target, m_q, m_weight);
        m_f = result.info.objective;
        m_gradient = result.gradient;
        m_body_error = result.info.body_error;
        m_weighted_error = result.info.weighted_error;
        m_error_norm = result.info.body_error.norm();
        m_initial_error = m_error_norm;

        m_s_history.clear();
        m_y_history.clear();
        m_rho_history.clear();
    }

    ik_status step(const Chain& chain)
    {
        if (m_status != ik_status::running)
        {
            return m_status;
        }

        if (detail::is_converged(m_body_error, m_weight, m_criteria))
        {
            m_status = ik_status::converged;
            return m_status;
        }

        ++m_iterations;
        if (m_iterations >= m_criteria.max_iterations)
        {
            m_status = ik_status::iteration_limit;
            return m_status;
        }

        int n = static_cast<int>(m_q.size());

        auto [direction, directional_derivative, max_alpha] = compute_descent_direction(n);
        if (max_alpha < std::numeric_limits<scalar_type>::epsilon() ||
            directional_derivative >= scalar_type(0))
        {
            update_stall_detection();
            return m_status;
        }

        position_type q_old = m_q;
        position_type g_old = m_gradient;
        scalar_type f_old = m_f;

        if (!armijo_line_search(chain, direction, directional_derivative, max_alpha, f_old))
        {
            update_stall_detection();
            return m_status;
        }

        update_lbfgs_history(q_old, g_old);

        update_stall_detection();
        detail::enforce_limits<LimitsPolicy>(m_q, chain);

        return m_status;
    }

    [[nodiscard]] bool converged() const { return m_status == ik_status::converged; }
    [[nodiscard]] const position_type& solution() const { return m_q; }
    [[nodiscard]] scalar_type error_norm() const { return m_error_norm; }
    [[nodiscard]] int iterations() const { return m_iterations; }
    void abort() {}
    [[nodiscard]] ik_status status() const { return m_status; }

private:
    struct descent_result
    {
        position_type direction;
        scalar_type directional_derivative;
        scalar_type max_alpha;
    };

    descent_result compute_descent_direction(int n)
    {
        position_type direction = compute_lbfgs_direction(n);
        scalar_type max_alpha = compute_max_step(direction, n);

        if (max_alpha < std::numeric_limits<scalar_type>::epsilon())
        {
            return {direction, scalar_type(0), max_alpha};
        }

        scalar_type dd = m_gradient.dot(direction);

        if (dd >= scalar_type(0))
        {
            direction = projected_negative_gradient(n);
            dd = m_gradient.dot(direction);
            max_alpha = compute_max_step(direction, n);
        }

        return {direction, dd, max_alpha};
    }

    bool armijo_line_search(
        const Chain& chain,
        const position_type& direction,
        scalar_type directional_derivative,
        scalar_type max_alpha,
        scalar_type f_old)
    {
        scalar_type alpha = std::min(scalar_type(1), max_alpha);

        for (int ls = 0; ls < m_options.max_line_search_steps; ++ls)
        {
            position_type q_trial = (m_q + alpha * direction)
                .cwiseMax(m_lower).cwiseMin(m_upper);

            auto result = ObjectivePolicy::evaluate_with_gradient(
                chain, m_target, q_trial, m_weight);
            scalar_type f_trial = result.info.objective;

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

    void update_lbfgs_history(const position_type& q_old, const position_type& g_old)
    {
        position_type s_k = m_q - q_old;
        position_type y_k = m_gradient - g_old;
        scalar_type ys = y_k.dot(s_k);

        if (ys > std::numeric_limits<scalar_type>::epsilon())
        {
            if (static_cast<int>(m_s_history.size()) >= m_options.history_depth)
            {
                m_s_history.erase(m_s_history.begin());
                m_y_history.erase(m_y_history.begin());
                m_rho_history.erase(m_rho_history.begin());
            }
            m_s_history.push_back(s_k);
            m_y_history.push_back(y_k);
            m_rho_history.push_back(scalar_type(1) / ys);

            scalar_type yy = y_k.squaredNorm();
            if (yy > std::numeric_limits<scalar_type>::epsilon())
            {
                m_gamma = ys / yy;
            }
        }
    }

    position_type compute_lbfgs_direction(int n)
    {
        position_type q_r = m_gradient;

        std::size_t m = m_s_history.size();
        std::vector<scalar_type> alpha_store(m);

        for (std::size_t i = m; i-- > 0;)
        {
            alpha_store[i] = m_rho_history[i] * m_s_history[i].dot(q_r);
            q_r -= alpha_store[i] * m_y_history[i];
        }

        q_r *= m_gamma;

        for (std::size_t i = 0; i < m; ++i)
        {
            scalar_type beta = m_rho_history[i] * m_y_history[i].dot(q_r);
            q_r += (alpha_store[i] - beta) * m_s_history[i];
        }

        position_type direction = -q_r;

        for (int i = 0; i < n; ++i)
        {
            if (m_q(i) <= m_lower(i) + std::numeric_limits<scalar_type>::epsilon() &&
                direction(i) < scalar_type(0))
            {
                direction(i) = scalar_type(0);
            }
            if (m_q(i) >= m_upper(i) - std::numeric_limits<scalar_type>::epsilon() &&
                direction(i) > scalar_type(0))
            {
                direction(i) = scalar_type(0);
            }
        }

        return direction;
    }

    position_type projected_negative_gradient(int n)
    {
        position_type direction = -m_gradient;
        for (int i = 0; i < n; ++i)
        {
            if (m_q(i) <= m_lower(i) + std::numeric_limits<scalar_type>::epsilon() &&
                direction(i) < scalar_type(0))
            {
                direction(i) = scalar_type(0);
            }
            if (m_q(i) >= m_upper(i) - std::numeric_limits<scalar_type>::epsilon() &&
                direction(i) > scalar_type(0))
            {
                direction(i) = scalar_type(0);
            }
        }
        return direction;
    }

    scalar_type compute_max_step(const position_type& direction, int n)
    {
        scalar_type max_alpha = std::numeric_limits<scalar_type>::max();
        for (int i = 0; i < n; ++i)
        {
            if (direction(i) > std::numeric_limits<scalar_type>::epsilon())
            {
                scalar_type t = (m_upper(i) - m_q(i)) / direction(i);
                max_alpha = std::min(max_alpha, t);
            }
            else if (direction(i) < -std::numeric_limits<scalar_type>::epsilon())
            {
                scalar_type t = (m_lower(i) - m_q(i)) / direction(i);
                max_alpha = std::min(max_alpha, t);
            }
        }
        return max_alpha;
    }

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

    se3<scalar_type> m_target{se3<scalar_type>::identity()};
    position_type m_q{};
    position_type m_gradient{};
    position_type m_lower{};
    position_type m_upper{};
    vector6<scalar_type> m_body_error{vector6<scalar_type>::Zero()};
    vector6<scalar_type> m_weighted_error{vector6<scalar_type>::Zero()};
    convergence_criteria<scalar_type> m_criteria{};
    error_weight<scalar_type> m_weight{};
    options m_options{};
    std::vector<scalar_type> m_error_history;
    std::vector<position_type> m_s_history;
    std::vector<position_type> m_y_history;
    std::vector<scalar_type> m_rho_history;
    scalar_type m_initial_error{};
    scalar_type m_error_norm{};
    scalar_type m_f{};
    scalar_type m_gamma{scalar_type(1)};
    int m_iterations{};
    ik_status m_status{ik_status::running};
};

#ifdef CARTAN_BUILD_ARGMIN
#include "cartan/serial/ik/solver/argmin_lbfgsb.h"
template <chain Chain, typename LimitsPolicy = clamp_limits>
using lbfgsb = argmin_lbfgsb<Chain, LimitsPolicy>;
#else
template <chain Chain, typename LimitsPolicy = clamp_limits>
using lbfgsb = builtin_lbfgsb<Chain, LimitsPolicy>;
#endif

}

#endif
