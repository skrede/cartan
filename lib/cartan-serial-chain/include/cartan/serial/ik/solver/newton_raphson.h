#ifndef HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_NEWTON_RAPHSON_H
#define HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_NEWTON_RAPHSON_H

/// Newton-Raphson IK solve policy with undamped Gauss-Newton Hessian
///        and backtracking Armijo line search for globalization.
///
/// Reference: Nocedal & Wright, Numerical Optimization, Ch. 3 (line search),
///            Ch. 10 (nonlinear least squares, Gauss-Newton).

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
#include "cartan/serial/fk/jacobian.h"
#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/chain_concept.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include <Eigen/Dense>

#include <cmath>
#include <vector>
#include <algorithm>

namespace cartan::ik
{

/// Newton-Raphson IK solve policy with Gauss-Newton Hessian and Armijo line search.
template <chain Chain, typename LimitsPolicy = clamp_limits,
          typename ObjectivePolicy = ik_se3_objective<Chain>>
class newton_raphson
{
public:
    using chain_type = Chain;
    using scalar_type = typename Chain::scalar_type;
    static constexpr int joints = Chain::joints;
    using limits_type = LimitsPolicy;

    using position_type = typename joint_state<scalar_type, joints>::position_type;

    static_assert(std::is_floating_point_v<scalar_type>, "newton_raphson requires a floating-point Scalar type");

    struct options
    {
        scalar_type epsilon{scalar_type(1e-8)};
        scalar_type line_search_c{scalar_type(1e-4)};
        scalar_type line_search_shrink{scalar_type(0.5)};
        int max_line_search_steps{20};
        scalar_type stall_threshold{scalar_type(1e-10)};
        scalar_type divergence_factor{scalar_type(10)};
        int stall_window{10};
    };

    newton_raphson() = default;

    explicit newton_raphson(const options& opts)
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
        m_stall_count = 0;
        m_status = ik_status::running;
        m_error_history.clear();

        int n = chain.num_joints();
        if constexpr (joints == dynamic)
        {
            m_lower.resize(n);
            m_upper.resize(n);
        }
        for (int i = 0; i < n; ++i)
        {
            m_lower(i) = chain.limits()[static_cast<std::size_t>(i)].position_min;
            m_upper(i) = chain.limits()[static_cast<std::size_t>(i)].position_max;
        }

        m_q = m_q.cwiseMax(m_lower).cwiseMin(m_upper);

        auto result = ObjectivePolicy::evaluate(chain, m_target, m_q, m_weight);
        m_error_norm = result.body_error.norm();
        m_initial_error = m_error_norm;
    }

    step_result<scalar_type> step(const Chain& chain, int N)
    {
        int units = 0;
        while (units < N && m_status == ik_status::running)
        {
            auto result = ObjectivePolicy::evaluate_with_gradient(
                chain, m_target, m_q, m_weight);
            scalar_type f_current = result.info.objective;
            auto& grad = result.gradient;
            auto& body_error = result.info.body_error;

            if (cartan::detail::is_converged(body_error, m_weight, m_criteria))
            {
                m_error_norm = body_error.norm();
                m_status = ik_status::converged;
                break;
            }

            ++m_iterations;
            ++units;
            if (m_iterations >= m_criteria.max_iterations_per_attempt)
            {
                m_error_norm = body_error.norm();
                m_status = ik_status::iteration_limit;
                break;
            }

            auto fk = forward_kinematics(chain, m_q);
            auto J_b = body_jacobian(chain, fk);
            int n = static_cast<int>(J_b.cols());

            auto H = (J_b.transpose() * J_b).eval();
            for (int i = 0; i < n; ++i)
            {
                H(i, i) += m_options.epsilon;
            }

            position_type dq = -H.ldlt().solve(grad);

            scalar_type directional_derivative = grad.dot(dq);
            scalar_type alpha = scalar_type(1);
            bool step_accepted = false;
            position_type q_trial = m_q;

            for (int ls = 0; ls < m_options.max_line_search_steps; ++ls)
            {
                q_trial = (m_q + alpha * dq).cwiseMax(m_lower).cwiseMin(m_upper);

                auto trial = ObjectivePolicy::evaluate(chain, m_target, q_trial, m_weight);
                scalar_type f_trial = trial.objective;

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
                m_q = q_trial;
                m_error_norm = ObjectivePolicy::evaluate(chain, m_target, m_q, m_weight)
                    .body_error.norm();
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

            cartan::detail::enforce_limits<LimitsPolicy>(m_q, chain);
        }
        return {m_status, {units, m_error_norm}};
    }

    [[nodiscard]] bool converged() const { return m_status == ik_status::converged; }
    [[nodiscard]] const position_type& solution() const { return m_q; }
    [[nodiscard]] scalar_type error_norm() const { return m_error_norm; }
    [[nodiscard]] int iterations() const { return m_iterations; }
    void abort() {}
    [[nodiscard]] ik_status status() const { return m_status; }

private:
    se3<scalar_type> m_target{se3<scalar_type>::identity()};
    position_type m_q{};
    position_type m_lower{};
    position_type m_upper{};
    convergence_criteria<scalar_type> m_criteria{};
    error_weight<scalar_type> m_weight{};
    options m_options{};
    std::vector<scalar_type> m_error_history;
    scalar_type m_initial_error{std::numeric_limits<scalar_type>::max()};
    scalar_type m_error_norm{std::numeric_limits<scalar_type>::max()};
    int m_iterations{};
    int m_stall_count{};
    ik_status m_status{ik_status::running};
};

}

#endif
