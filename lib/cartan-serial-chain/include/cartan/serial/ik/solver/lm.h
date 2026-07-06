#ifndef HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_LM_H
#define HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_LM_H

/// Levenberg-Marquardt IK solve policy with Nielsen-style lambda update.
///
/// Body-frame Newton-Raphson with trust-region damping. The gain ratio
/// controls whether a step is accepted and how the damping parameter
/// lambda adapts: good steps reduce lambda (more Gauss-Newton),
/// poor steps increase lambda (more gradient descent).
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 6.2, p. 227-233.
///            Nielsen, H.B., Damping Parameter in Marquardt's Method, 1999.

#include "cartan/types.h"

#include "cartan/serial/ik/ik_status.h"
#include "cartan/serial/ik/policy/limits_policy.h"
#include "cartan/serial/ik/concepts/solve_concept.h"
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

namespace cartan
{

/// Levenberg-Marquardt IK solve policy with Nielsen lambda update strategy.
///
/// Each step() call: compute FK, body-frame error, Jacobian, Hessian
/// approximation H = J^T J, gradient g = J^T V_b, solve (H + lambda*I) dq = g,
/// evaluate gain ratio, accept/reject step, update lambda.
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 6.2.
///            Nielsen, Damping Parameter in Marquardt's Method, 1999.
template <chain Chain, typename LimitsPolicy = no_limits>
class builtin_lm
{
public:
    using chain_type = Chain;
    using scalar_type = typename Chain::scalar_type;
    static constexpr int joints = Chain::joints;
    using limits_type = LimitsPolicy;

    using position_type = typename joint_state<scalar_type, joints>::position_type;

    static_assert(std::is_floating_point_v<scalar_type>, "builtin_lm requires a floating-point Scalar type");

    struct options
    {
        scalar_type initial_lambda_factor{scalar_type(1e-3)};
        scalar_type stall_threshold{scalar_type(1e-6)};
        scalar_type divergence_factor{scalar_type(10)};
        int stall_window{5};
    };

    builtin_lm() = default;

    explicit builtin_lm(const options& opts)
        : m_options(opts)
    {
    }

    void setup(
        const Chain& chain,
        const se3<scalar_type>& target,
        const position_type& q0,
        const convergence_criteria<scalar_type>& criteria)
    {
        m_target = target;
        m_q = q0;
        m_criteria = criteria;
        m_iterations = 0;
        m_status = ik_status::running;
        m_nu = scalar_type(2);
        m_error_history.clear();

        auto fk = forward_kinematics(chain, m_q);
        m_V_b = (fk.end_effector.inverse() * m_target).log();
        m_error_norm = m_V_b.norm();
        m_initial_error = m_error_norm;

        auto J_b = body_jacobian(chain, fk);
        int n = static_cast<int>(J_b.cols());
        auto JtJ = (J_b.transpose() * J_b).eval();
        scalar_type max_diag{0};
        for (int i = 0; i < n; ++i)
        {
            max_diag = std::max(max_diag, JtJ(i, i));
        }
        m_lambda = m_options.initial_lambda_factor * max_diag;
        if (m_lambda < std::numeric_limits<scalar_type>::epsilon())
        {
            m_lambda = scalar_type(1e-4);
        }
    }

    step_result<scalar_type> step(const Chain& chain, int N)
    {
        int units = 0;
        while (units < N && m_status == ik_status::running)
        {
            auto fk = forward_kinematics(chain, m_q);
            m_V_b = (fk.end_effector.inverse() * m_target).log();

            if (cartan::detail::is_converged_unweighted(m_V_b, m_criteria))
            {
                m_error_norm = m_V_b.norm();
                // Converged implies feasible as a property of the pose-equivalence
                // class: a pose reached at an unwrapped 2*pi-equivalent angle is
                // recovered to its in-limits representative and accepted, while a
                // pose only reachable outside the box is downgraded to a joint-
                // limit failure. Canonicalizes the FINAL iterate once and stores
                // the in-limits representative; never a clamp of the working step.
                m_status = cartan::detail::feasible_after_canonicalization(
                        m_q, chain, cartan::detail::default_feasibility_tol<scalar_type>())
                    ? ik_status::converged
                    : ik_status::joint_limit_hit;
                break;
            }

            ++m_iterations;
            ++units;
            if (m_iterations >= m_criteria.max_iterations_per_attempt)
            {
                m_error_norm = m_V_b.norm();
                m_status = ik_status::iteration_limit;
                break;
            }

            auto J_b = body_jacobian(chain, fk);
            int n = static_cast<int>(J_b.cols());

            auto H = (J_b.transpose() * J_b).eval();
            auto g = (J_b.transpose() * m_V_b).eval();

            using dq_matrix = std::conditional_t<
                joints == dynamic,
                Eigen::MatrixX<scalar_type>,
                Eigen::Matrix<scalar_type, joints, joints>>;

            dq_matrix A = H;
            for (int i = 0; i < n; ++i)
            {
                A(i, i) += m_lambda;
            }

            position_type dq = A.ldlt().solve(g);

            // Guard against a diverged trust region. When the achievable pose
            // residual floors above the requested tolerance (common in float),
            // no trial step reduces the error, so every step is rejected and
            // the Nielsen update grows lambda without bound until it overflows
            // to a non-finite value. A non-finite damping parameter yields a
            // non-finite increment, which would push a non-finite rotation
            // vector into the exponential map. Terminate the attempt as a clean
            // divergence (the error channel) rather than stepping into
            // undefined behavior on a solve that can never converge.
            if (!std::isfinite(m_lambda) || !dq.allFinite())
            {
                m_error_norm = m_V_b.norm();
                m_status = ik_status::diverged;
                break;
            }

            position_type q_trial = m_q + dq;
            auto fk_trial = forward_kinematics(chain, q_trial);
            auto V_b_trial = (fk_trial.end_effector.inverse() * m_target).log();
            const bool accepted = evaluate_gain_and_update_damping(dq, g, q_trial, V_b_trial);

            m_error_norm = m_V_b.norm();

            // Stall/divergence detection runs only on accepted steps. Rejected
            // LM steps are part of the normal damping calibration: a string of
            // rejects leaves m_V_b (and therefore m_error_norm) untouched, which
            // would otherwise trip the "no progress" rule deterministically and
            // kill the solve before lambda has time to grow into a usable regime.
            // m_initial_error gates divergence, so skipping the check on reject
            // is safe -- m_error_norm is unchanged from the last accepted step,
            // which already passed.
            if (accepted)
            {
                auto stall_result = cartan::detail::check_stall_divergence(
                    m_error_history, m_error_norm, m_initial_error,
                    m_options.stall_window, m_options.stall_threshold,
                    m_options.divergence_factor);
                if (stall_result != ik_status::running)
                {
                    m_status = stall_result;
                    break;
                }
            }

            cartan::detail::enforce_limits<LimitsPolicy>(m_q, chain);
        }
        return {m_status, {units, m_error_norm}};
    }

    bool converged() const { return m_status == ik_status::converged; }
    const position_type& solution() const { return m_q; }
    scalar_type error_norm() const { return m_error_norm; }
    int iterations() const { return m_iterations; }
    void abort() {}
    scalar_type lambda() const { return m_lambda; }
    ik_status status() const { return m_status; }

private:
    bool evaluate_gain_and_update_damping(
        const position_type& dq,
        const auto& g,
        const position_type& q_trial,
        const vector6<scalar_type>& V_b_trial)
    {
        scalar_type error_old_sq = m_V_b.squaredNorm();
        scalar_type error_new_sq = V_b_trial.squaredNorm();
        scalar_type predicted_reduction = (dq.transpose() * (m_lambda * dq + g)).value();
        scalar_type rho{0};
        if (std::abs(predicted_reduction) > std::numeric_limits<scalar_type>::epsilon())
        {
            rho = (error_old_sq - error_new_sq) / predicted_reduction;
        }

        if (rho > scalar_type(0))
        {
            m_q = q_trial;
            m_V_b = V_b_trial;
            scalar_type factor = scalar_type(1) - std::pow(scalar_type(2) * rho - scalar_type(1), scalar_type(3));
            m_lambda *= std::max(scalar_type(1) / scalar_type(3), factor);
            m_nu = scalar_type(2);
            return true;
        }
        m_lambda *= m_nu;
        m_nu *= scalar_type(2);
        return false;
    }

    se3<scalar_type> m_target{se3<scalar_type>::identity()};
    position_type m_q{};
    vector6<scalar_type> m_V_b{vector6<scalar_type>::Zero()};
    convergence_criteria<scalar_type> m_criteria{};
    options m_options{};
    cartan::detail::error_ring<scalar_type> m_error_history;
    scalar_type m_initial_error{};
    scalar_type m_error_norm{};
    scalar_type m_lambda{};
    scalar_type m_nu{scalar_type(2)};
    int m_iterations{};
    ik_status m_status{ik_status::running};
};

#ifndef CARTAN_BUILD_ARGMIN
template <chain Chain, typename LimitsPolicy = no_limits>
using lm =builtin_lm<Chain, LimitsPolicy>;
#endif

}

#ifdef CARTAN_BUILD_ARGMIN
#include "cartan/serial/ik/solver/argmin_lm.h"

namespace cartan
{
template <chain Chain, typename LimitsPolicy = no_limits>
using lm =argmin_lm<Chain, LimitsPolicy>;
}
#endif

#endif
