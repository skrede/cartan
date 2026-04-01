#ifndef HPP_GUARD_LIEPP_SERIAL_IK_DLS_SOLVE_POLICY_H
#define HPP_GUARD_LIEPP_SERIAL_IK_DLS_SOLVE_POLICY_H

/// @file dls_solve_policy.h
/// @brief Damped least squares IK solve policy with SVD-based adaptive damping.
///
/// Body-frame Newton-Raphson iteration with Nakamura's adaptive damping
/// to handle near-singular configurations gracefully. The damping factor
/// increases as the smallest singular value of the body Jacobian drops
/// below a threshold, preventing wild joint velocities near singularities.
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 6.2, p. 227-233.
///            Nakamura, Y., Advanced Robotics: Redundancy and Optimization, 1991.
///            Decisions IK-02, D-04.

#include "liepp/types.h"

#include "liepp/serial/ik/ik_types.h"
#include "liepp/serial/ik/limits_policy.h"
#include "liepp/serial/ik/ik_solve_policy.h"
#include "liepp/serial/ik/detail/convergence.h"
#include "liepp/serial/ik/detail/stall_detection.h"
#include "liepp/serial/ik/detail/limit_enforcement.h"

#include "liepp/lie/se3.h"
#include "liepp/serial/chain/joint_state.h"
#include "liepp/serial/fk/jacobian.h"
#include "liepp/serial/chain/kinematic_chain.h"
#include "liepp/serial/fk/forward_kinematics.h"

#include <Eigen/SVD>
#include <Eigen/Dense>

#include <cmath>
#include <vector>
#include <algorithm>

namespace liepp
{

/// Damped least squares IK solve policy with SVD-based adaptive damping (Nakamura).
///
/// Each step() call: compute FK, body-frame error, check convergence,
/// compute body Jacobian, SVD, adaptive damping, pseudoinverse step.
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 6.2, Eq. 6.8-6.10.
///            Nakamura, Advanced Robotics, Ch. 11 (adaptive DLS).
template <typename Scalar = double, int N = dynamic, typename LimitsPolicy = clamp_limits>
class dls_solve_policy
{
public:
    static_assert(std::is_floating_point_v<Scalar>, "dls_solve_policy requires a floating-point Scalar type");

    using scalar_type = Scalar;
    static constexpr int joints = N;
    using limits_type = LimitsPolicy;

    using position_type = typename joint_state<Scalar, N>::position_type;

    /// Tunable parameters for the DLS solve policy.
    struct options
    {
        Scalar singularity_threshold{Scalar(0.01)};
        Scalar lambda_max{Scalar(0.04)};
        Scalar stall_threshold{Scalar(1e-6)};
        Scalar divergence_factor{Scalar(10)};
        int stall_window{5};
    };

    dls_solve_policy() = default;

    explicit dls_solve_policy(const options& opts)
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
        m_error_history.clear();
        m_condition_number = Scalar(0);
        m_manipulability_value = Scalar(0);

        auto fk = forward_kinematics(chain, m_q);
        auto V_b = (fk.end_effector.inverse() * m_target).log();
        m_error_norm = V_b.norm();
        m_initial_error = m_error_norm;
    }

    /// Execute one Newton-Raphson iteration with adaptive DLS damping.
    ///
    /// Lynch & Park, Modern Robotics, Ch. 6.2, Algorithm 6.2, p. 233.
    ik_status step(const kinematic_chain<Scalar, N>& chain)
    {
        if (m_status != ik_status::running)
        {
            return m_status;
        }

        // (a) Forward kinematics
        auto fk = forward_kinematics(chain, m_q);

        // (b) Body-frame error twist: V_b = log(T_current^{-1} * T_target)
        auto V_b = (fk.end_effector.inverse() * m_target).log();

        // (c) Convergence check: separate angular and linear per Lynch & Park
        if (detail::is_converged_unweighted(V_b, m_criteria))
        {
            m_error_norm = V_b.norm();
            m_status = ik_status::converged;
            return m_status;
        }

        // (d) Iteration limit check
        ++m_iterations;
        if (m_iterations >= m_criteria.max_iterations)
        {
            m_error_norm = V_b.norm();
            m_status = ik_status::iteration_limit;
            return m_status;
        }

        // (e) Body Jacobian
        auto J_b = body_jacobian(chain, fk);

        // (f) SVD of the body Jacobian
        // Use ComputeFullU/V for fixed-size matrices (Eigen requirement),
        // ComputeThinU/V for dynamic matrices (more efficient).
        constexpr unsigned int svd_options = (N == dynamic)
            ? (Eigen::ComputeThinU | Eigen::ComputeThinV)
            : (Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::JacobiSVD<jacobian_matrix<Scalar, N>> svd(J_b, svd_options);

        auto sigma = svd.singularValues();
        int rank = static_cast<int>(sigma.size());

        // (g) Adaptive damping (Nakamura): lambda depends on sigma_min
        Scalar sigma_min = sigma(rank - 1);
        Scalar sigma_max = sigma(0);
        Scalar lambda_sq{0};

        if (sigma_min < m_options.singularity_threshold)
        {
            Scalar ratio = Scalar(1) - (sigma_min / m_options.singularity_threshold)
                           * (sigma_min / m_options.singularity_threshold);
            lambda_sq = m_options.lambda_max * m_options.lambda_max * ratio;
        }

        // (h) Damped pseudoinverse step: dq = V_r * diag(sigma_i/(sigma_i^2 + lambda^2)) * U_r^T * V_b
        //     where U_r and V_r are the first `rank` columns (thin SVD equivalent).
        Eigen::VectorX<Scalar> damped(rank);
        for (int i = 0; i < rank; ++i)
        {
            damped(i) = sigma(i) / (sigma(i) * sigma(i) + lambda_sq);
        }

        auto U_r = svd.matrixU().leftCols(rank);
        auto V_r = svd.matrixV().leftCols(rank);
        position_type dq = V_r * damped.asDiagonal() * U_r.transpose() * V_b;

        // (i) Update joint configuration
        m_q += dq;

        // Update error
        m_error_norm = V_b.norm();

        // (j) Stall and divergence detection
        auto stall_result = detail::check_stall_divergence(
            m_error_history, m_error_norm, m_initial_error,
            m_options.stall_window, m_options.stall_threshold,
            m_options.divergence_factor);
        if (stall_result != ik_status::running)
        {
            m_status = stall_result;
            return m_status;
        }

        // (k) Store condition number
        m_condition_number = (sigma_min > Scalar(0))
            ? sigma_max / sigma_min
            : std::numeric_limits<Scalar>::infinity();

        // Manipulability: product of singular values (Yoshikawa, 1985)
        m_manipulability_value = Scalar(1);
        for (int i = 0; i < rank; ++i)
        {
            m_manipulability_value *= sigma(i);
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

    /// Abort the solver (no-op for DLS per D-05).
    void abort() {}

    /// Condition number of the body Jacobian (sigma_max / sigma_min).
    [[nodiscard]] Scalar condition_number() const { return m_condition_number; }

    /// Manipulability measure: product of singular values (Yoshikawa, 1985).
    [[nodiscard]] Scalar manipulability() const { return m_manipulability_value; }

    /// Current solve policy status.
    [[nodiscard]] ik_status status() const { return m_status; }

private:
    se3<Scalar> m_target{se3<Scalar>::identity()};
    position_type m_q{};
    convergence_criteria<Scalar> m_criteria{};
    options m_options{};
    std::vector<Scalar> m_error_history;
    Scalar m_manipulability_value{};
    Scalar m_condition_number{};
    Scalar m_initial_error{};
    Scalar m_error_norm{};
    int m_iterations{};
    ik_status m_status{ik_status::running};
};

}

#endif
