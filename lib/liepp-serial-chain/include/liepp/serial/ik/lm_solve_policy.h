#ifndef HPP_GUARD_LIEPP_IK_LM_SOLVE_POLICY_H
#define HPP_GUARD_LIEPP_IK_LM_SOLVE_POLICY_H

/// @file lm_solve_policy.h
/// @brief Levenberg-Marquardt IK solve policy with Nielsen-style lambda update.
///
/// Body-frame Newton-Raphson with trust-region damping. The gain ratio
/// controls whether a step is accepted and how the damping parameter
/// lambda adapts: good steps reduce lambda (more Gauss-Newton),
/// poor steps increase lambda (more gradient descent).
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 6.2, p. 227-233.
///            Nielsen, H.B., Damping Parameter in Marquardt's Method, 1999.
///            Decisions IK-03, D-04.

#include "liepp/types.h"

#include "liepp/ik/ik_types.h"
#include "liepp/ik/limits_policy.h"
#include "liepp/ik/ik_solve_policy.h"
#include "liepp/ik/detail/convergence.h"
#include "liepp/ik/detail/stall_detection.h"
#include "liepp/ik/detail/limit_enforcement.h"

#include "liepp/lie/se3.h"
#include "liepp/chain/joint_state.h"
#include "liepp/kinematics/jacobian.h"
#include "liepp/chain/kinematic_chain.h"
#include "liepp/kinematics/forward_kinematics.h"

#include <Eigen/Dense>

#include <cmath>
#include <vector>
#include <algorithm>

namespace liepp
{

/// Levenberg-Marquardt IK solve policy with Nielsen lambda update strategy.
///
/// Each step() call: compute FK, body-frame error, Jacobian, Hessian
/// approximation H = J^T J, gradient g = J^T V_b, solve (H + lambda*I) dq = g,
/// evaluate gain ratio, accept/reject step, update lambda.
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 6.2.
///            Nielsen, Damping Parameter in Marquardt's Method, 1999.
template <typename Scalar = double, int N = dynamic, typename LimitsPolicy = clamp_limits>
class lm_solve_policy
{
public:
    static_assert(std::is_floating_point_v<Scalar>, "lm_solve_policy requires a floating-point Scalar type");

    using scalar_type = Scalar;
    static constexpr int joints = N;
    using limits_type = LimitsPolicy;

    using position_type = typename joint_state<Scalar, N>::position_type;

    /// Tunable parameters for the LM solve policy.
    struct options
    {
        Scalar initial_lambda_factor{Scalar(1e-3)};
        Scalar stall_threshold{Scalar(1e-6)};
        Scalar divergence_factor{Scalar(10)};
        int stall_window{5};
    };

    lm_solve_policy() = default;

    explicit lm_solve_policy(const options& opts)
        : m_options(opts)
    {
    }

    /// Initialize the solve policy with chain, target pose, seed configuration,
    /// and convergence criteria.
    void setup(
        const kinematic_chain<Scalar, N>& chain,
        const se3<Scalar>& target,
        const position_type& q0,
        const convergence_criteria<Scalar>& criteria)
    {
        m_target = target;
        m_q = q0;
        m_criteria = criteria;
        m_iterations = 0;
        m_status = ik_status::running;
        m_nu = Scalar(2);
        m_error_history.clear();

        auto fk = forward_kinematics(chain, m_q);
        m_V_b = (fk.end_effector.inverse() * m_target).log();
        m_error_norm = m_V_b.norm();
        m_initial_error = m_error_norm;

        // Initial lambda from max diagonal of J^T J (Nielsen initialization)
        auto J_b = body_jacobian(chain, fk);
        int n = static_cast<int>(J_b.cols());
        auto JtJ = (J_b.transpose() * J_b).eval();
        Scalar max_diag{0};
        for (int i = 0; i < n; ++i)
        {
            max_diag = std::max(max_diag, JtJ(i, i));
        }
        m_lambda = m_options.initial_lambda_factor * max_diag;
        if (m_lambda < std::numeric_limits<Scalar>::epsilon())
        {
            m_lambda = Scalar(1e-4);
        }
    }

    /// Execute one Levenberg-Marquardt iteration.
    ///
    /// Lynch & Park, Modern Robotics, Ch. 6.2, with Nielsen lambda update.
    ik_status step(const kinematic_chain<Scalar, N>& chain)
    {
        if (m_status != ik_status::running)
        {
            return m_status;
        }

        // (a) Forward kinematics and body-frame error
        auto fk = forward_kinematics(chain, m_q);
        m_V_b = (fk.end_effector.inverse() * m_target).log();

        // (b) Convergence check: separate angular and linear
        if (detail::is_converged_unweighted(m_V_b, m_criteria))
        {
            m_error_norm = m_V_b.norm();
            m_status = ik_status::converged;
            return m_status;
        }

        // (c) Iteration limit check
        ++m_iterations;
        if (m_iterations >= m_criteria.max_iterations)
        {
            m_error_norm = m_V_b.norm();
            m_status = ik_status::iteration_limit;
            return m_status;
        }

        // (d) Body Jacobian
        auto J_b = body_jacobian(chain, fk);
        int n = static_cast<int>(J_b.cols());

        // (e) Gauss-Newton approximation: H = J^T J, g = J^T V_b
        auto H = (J_b.transpose() * J_b).eval();
        auto g = (J_b.transpose() * m_V_b).eval();

        // (f) Solve (H + lambda * I) dq = g
        using dq_matrix = std::conditional_t<
            N == dynamic,
            Eigen::MatrixX<Scalar>,
            Eigen::Matrix<Scalar, N, N>>;

        dq_matrix A = H;
        for (int i = 0; i < n; ++i)
        {
            A(i, i) += m_lambda;
        }

        position_type dq = A.ldlt().solve(g);

        // (g) Evaluate trial step and update damping
        position_type q_trial = m_q + dq;
        auto fk_trial = forward_kinematics(chain, q_trial);
        auto V_b_trial = (fk_trial.end_effector.inverse() * m_target).log();
        evaluate_gain_and_update_damping(dq, g, q_trial, V_b_trial);

        // Update error norm
        m_error_norm = m_V_b.norm();

        // (k) Stall and divergence detection
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

    /// Whether the solve policy has converged.
    [[nodiscard]] bool converged() const { return m_status == ik_status::converged; }

    /// Current joint configuration (the candidate solution).
    [[nodiscard]] const position_type& solution() const { return m_q; }

    /// Current error norm (body-frame twist magnitude).
    [[nodiscard]] Scalar error_norm() const { return m_error_norm; }

    /// Number of iterations executed so far.
    [[nodiscard]] int iterations() const { return m_iterations; }

    /// Abort the solver (no-op for LM per D-05).
    void abort() {}

    /// Current damping parameter lambda.
    [[nodiscard]] Scalar lambda() const { return m_lambda; }

    /// Current solve policy status.
    [[nodiscard]] ik_status status() const { return m_status; }

private:
    /// Compute gain ratio and accept/reject step with Nielsen lambda update.
    void evaluate_gain_and_update_damping(
        const position_type& dq,
        const auto& g,
        const position_type& q_trial,
        const vector6<Scalar>& V_b_trial)
    {
        Scalar error_old_sq = m_V_b.squaredNorm();
        Scalar error_new_sq = V_b_trial.squaredNorm();
        Scalar predicted_reduction = (dq.transpose() * (m_lambda * dq + g)).value();
        Scalar rho{0};
        if (std::abs(predicted_reduction) > std::numeric_limits<Scalar>::epsilon())
        {
            rho = (error_old_sq - error_new_sq) / predicted_reduction;
        }

        if (rho > Scalar(0))
        {
            m_q = q_trial;
            m_V_b = V_b_trial;
            Scalar factor = Scalar(1) - std::pow(Scalar(2) * rho - Scalar(1), Scalar(3));
            m_lambda *= std::max(Scalar(1) / Scalar(3), factor);
            m_nu = Scalar(2);
        }
        else
        {
            m_lambda *= m_nu;
            m_nu *= Scalar(2);
        }
    }

    se3<Scalar> m_target{se3<Scalar>::identity()};
    position_type m_q{};
    vector6<Scalar> m_V_b{vector6<Scalar>::Zero()};
    convergence_criteria<Scalar> m_criteria{};
    options m_options{};
    std::vector<Scalar> m_error_history;
    Scalar m_initial_error{};
    Scalar m_error_norm{};
    Scalar m_lambda{};
    Scalar m_nu{Scalar(2)};
    int m_iterations{};
    ik_status m_status{ik_status::running};
};

}

#endif
