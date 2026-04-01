#ifndef HPP_GUARD_LIEPP_SERIAL_IK_PROJECTED_LM_SOLVE_POLICY_H
#define HPP_GUARD_LIEPP_SERIAL_IK_PROJECTED_LM_SOLVE_POLICY_H

/// @file projected_lm_solve_policy.h
/// @brief Projected Levenberg-Marquardt IK solve policy with active-set box
///        projection and optional dogleg trust-region step.
///
/// Speed-optimized solver: active-set projection handles joint limits
/// within the optimization step (not post-hoc clamping). The dogleg
/// option interpolates between Cauchy point and Gauss-Newton step.
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 6.2, p. 227-233.
///            Nielsen, H.B., Damping Parameter in Marquardt's Method, 1999.
///            Nocedal & Wright, Numerical Optimization, Ch. 4 (dogleg).
///            Decisions D-01, D-02, D-03, SPEED-01, SPEED-02.

#include "liepp/types.h"

#include "liepp/serial/ik/ik_types.h"
#include "liepp/serial/ik/limits_policy.h"
#include "liepp/serial/ik/error_weight.h"
#include "liepp/serial/ik/ik_solve_policy.h"
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

/// Projected Levenberg-Marquardt IK solve policy with active-set box projection.
///
/// Each step() call: compute FK, body-frame error, Jacobian, identify
/// active-set (joints at limits with gradient pushing outward), solve
/// reduced (H + lambda*I) dq = g for free variables only, clamp trial step.
///
/// When use_dogleg is enabled, the step interpolates between the Cauchy
/// point and Gauss-Newton step within a trust region.
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 6.2.
///            Nielsen, Damping Parameter in Marquardt's Method, 1999.
template <typename Scalar = double, int N = dynamic, typename LimitsPolicy = no_limits>
class projected_lm_solve_policy
{
public:
    static_assert(std::is_floating_point_v<Scalar>, "projected_lm_solve_policy requires a floating-point Scalar type");

    using scalar_type = Scalar;
    static constexpr int joints = N;
    using limits_type = LimitsPolicy;

    using position_type = typename joint_state<Scalar, N>::position_type;

    /// Tunable parameters for the projected LM solve policy.
    struct options
    {
        Scalar initial_lambda_factor{Scalar(1e-3)};
        Scalar stall_threshold{Scalar(1e-6)};
        Scalar divergence_factor{Scalar(10)};
        int stall_window{5};
        bool use_dogleg{false};
        Scalar trust_region_radius{Scalar(1.0)};
    };

    projected_lm_solve_policy() = default;

    explicit projected_lm_solve_policy(const options& opts)
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
        m_status = ik_status::running;
        m_nu = Scalar(2);
        m_delta = m_options.trust_region_radius;
        m_error_history.clear();

        // Read joint limits from chain
        int n = chain.num_joints();
        if constexpr (N == dynamic)
        {
            m_q_min.resize(n);
            m_q_max.resize(n);
        }
        for (int i = 0; i < n; ++i)
        {
            m_q_min(i) = chain.limits()[static_cast<std::size_t>(i)].position_min;
            m_q_max(i) = chain.limits()[static_cast<std::size_t>(i)].position_max;
        }

        // Clamp initial seed to limits
        m_q = m_q.cwiseMax(m_q_min).cwiseMin(m_q_max);

        auto fk = forward_kinematics(chain, m_q);
        m_V_b = (fk.end_effector.inverse() * m_target).log();
        m_error_norm = m_V_b.norm();
        m_initial_error = m_error_norm;

        // Initial lambda from max diagonal of J^T J (Nielsen initialization)
        auto J_b = body_jacobian(chain, fk);
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

    /// Execute one projected Levenberg-Marquardt iteration.
    ik_status step(const kinematic_chain<Scalar, N>& chain)
    {
        if (m_status != ik_status::running)
        {
            return m_status;
        }

        // (a) Forward kinematics, body-frame error, and convergence check
        auto fk = forward_kinematics(chain, m_q);
        m_V_b = (fk.end_effector.inverse() * m_target).log();

        if (auto s = check_convergence_and_limits(); s != ik_status::running)
        {
            return s;
        }

        // (b) Body Jacobian and Gauss-Newton approximation
        auto J_b = body_jacobian(chain, fk);
        int n = static_cast<int>(J_b.cols());
        auto H = (J_b.transpose() * J_b).eval();
        auto g = (J_b.transpose() * m_V_b).eval();

        // (c) Active-set projection and reduced system solve
        auto free_indices = identify_active_set(g, n);
        position_type dq = solve_projected_system(J_b, H, g, free_indices, n);

        // (d) Evaluate trial step and compute gain ratio
        auto [q_trial, V_b_trial, rho] = evaluate_trial_step(chain, dq, H, g);

        // (e) Accept/reject step and update damping parameters
        update_damping(rho, dq, q_trial, V_b_trial);

        // (f) Error norm update and stall/divergence detection
        m_error_norm = m_V_b.norm();
        auto stall_result = detail::check_stall_divergence(
            m_error_history, m_error_norm, m_initial_error,
            m_options.stall_window, m_options.stall_threshold,
            m_options.divergence_factor);
        if (stall_result != ik_status::running)
        {
            m_status = stall_result;
            return m_status;
        }

        detail::enforce_limits<LimitsPolicy>(m_q, chain);
        return m_status;
    }

    [[nodiscard]] bool converged() const { return m_status == ik_status::converged; }
    [[nodiscard]] const position_type& solution() const { return m_q; }
    [[nodiscard]] Scalar error_norm() const { return m_error_norm; }
    [[nodiscard]] int iterations() const { return m_iterations; }
    void abort() {}
    [[nodiscard]] Scalar lambda() const { return m_lambda; }
    void set_lambda(Scalar l) { m_lambda = l; }
    [[nodiscard]] ik_status status() const { return m_status; }

private:
    /// Check convergence and iteration limit; return non-running status if done.
    ik_status check_convergence_and_limits()
    {
        if (detail::is_converged(m_V_b, m_weight, m_criteria))
        {
            m_error_norm = m_V_b.norm();
            m_status = ik_status::converged;
            return m_status;
        }

        ++m_iterations;
        if (m_iterations >= m_criteria.max_iterations)
        {
            m_error_norm = m_V_b.norm();
            m_status = ik_status::iteration_limit;
            return m_status;
        }

        return ik_status::running;
    }

    /// Identify the active set: joints at bounds with gradient pushing outward.
    std::vector<int> identify_active_set(const auto& g, int n)
    {
        std::vector<int> free_indices;
        free_indices.reserve(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i)
        {
            bool at_lower = (m_q(i) <= m_q_min(i) + std::numeric_limits<Scalar>::epsilon());
            bool at_upper = (m_q(i) >= m_q_max(i) - std::numeric_limits<Scalar>::epsilon());
            bool active = (at_lower && g(i) < Scalar(0)) ||
                          (at_upper && g(i) > Scalar(0));
            if (!active)
            {
                free_indices.push_back(i);
            }
        }
        return free_indices;
    }

    /// Solve the reduced system on free variables (standard LM or dogleg).
    template <typename JacobianType, typename HessianType, typename GradientType>
    position_type solve_projected_system(
        const JacobianType& J_b,
        const HessianType& H,
        const GradientType& g,
        const std::vector<int>& free_indices,
        int n)
    {
        int n_free = static_cast<int>(free_indices.size());
        position_type dq;
        if constexpr (N == dynamic)
        {
            dq = position_type::Zero(n);
        }
        else
        {
            dq = position_type::Zero();
        }

        if (n_free == 0)
        {
            return dq;
        }

        // Extract reduced system for free variables
        Eigen::MatrixX<Scalar> H_free(n_free, n_free);
        Eigen::VectorX<Scalar> g_free(n_free);
        for (int i = 0; i < n_free; ++i)
        {
            g_free(i) = g(free_indices[static_cast<std::size_t>(i)]);
            for (int j = 0; j < n_free; ++j)
            {
                H_free(i, j) = H(free_indices[static_cast<std::size_t>(i)],
                                 free_indices[static_cast<std::size_t>(j)]);
            }
        }

        Eigen::VectorX<Scalar> dq_free;
        if (m_options.use_dogleg)
        {
            dq_free = dogleg_step(J_b, H_free, g_free, free_indices, n_free);
        }
        else
        {
            Eigen::MatrixX<Scalar> A = H_free;
            for (int i = 0; i < n_free; ++i)
            {
                A(i, i) += m_lambda;
            }
            dq_free = A.ldlt().solve(g_free);
        }

        for (int i = 0; i < n_free; ++i)
        {
            dq(free_indices[static_cast<std::size_t>(i)]) = dq_free(i);
        }
        return dq;
    }

    /// Evaluate trial step: compute trial FK, error, and gain ratio.
    template <typename HessianType, typename GradientType>
    struct trial_result
    {
        position_type q_trial;
        vector6<Scalar> V_b_trial;
        Scalar rho;
    };

    template <typename HessianType, typename GradientType>
    auto evaluate_trial_step(
        const kinematic_chain<Scalar, N>& chain,
        const position_type& dq,
        const HessianType& H,
        const GradientType& g) -> trial_result<HessianType, GradientType>
    {
        position_type q_trial = (m_q + dq).cwiseMax(m_q_min).cwiseMin(m_q_max);
        auto fk_trial = forward_kinematics(chain, q_trial);
        auto V_b_trial = (fk_trial.end_effector.inverse() * m_target).log();

        Scalar error_old_sq = m_V_b.squaredNorm();
        Scalar error_new_sq = V_b_trial.squaredNorm();
        Scalar actual_reduction = Scalar(0.5) * (error_old_sq - error_new_sq);

        Scalar predicted_reduction;
        if (m_options.use_dogleg)
        {
            predicted_reduction = dq.dot(g) - Scalar(0.5) * dq.dot(H * dq);
        }
        else
        {
            predicted_reduction = Scalar(0.5) * (dq.transpose() * (m_lambda * dq + g)).value();
        }

        Scalar rho{0};
        if (std::abs(predicted_reduction) > std::numeric_limits<Scalar>::epsilon())
        {
            rho = actual_reduction / predicted_reduction;
        }

        return {q_trial, V_b_trial, rho};
    }

    /// Accept/reject step and update damping (lambda or trust-region radius).
    void update_damping(
        Scalar rho,
        const position_type& dq,
        const position_type& q_trial,
        const vector6<Scalar>& V_b_trial)
    {
        if (rho > Scalar(0))
        {
            m_q = q_trial;
            m_V_b = V_b_trial;

            if (m_options.use_dogleg)
            {
                if (rho > Scalar(0.75))
                {
                    m_delta = std::max(m_delta, Scalar(3) * dq.norm());
                }
                else if (rho < Scalar(0.25))
                {
                    m_delta *= Scalar(0.25);
                }
            }
            else
            {
                Scalar factor = Scalar(1) - std::pow(Scalar(2) * rho - Scalar(1), Scalar(3));
                m_lambda *= std::max(Scalar(1) / Scalar(3), factor);
                m_nu = Scalar(2);
            }
        }
        else
        {
            if (m_options.use_dogleg)
            {
                m_delta *= Scalar(0.25);
            }
            else
            {
                m_lambda *= m_nu;
                m_nu *= Scalar(2);
            }
        }
    }

    /// Dogleg step computation on the free (unconstrained) subspace.
    template <int JRows>
    Eigen::VectorX<Scalar> dogleg_step(
        const Eigen::Matrix<Scalar, 6, JRows>& J_b,
        const Eigen::MatrixX<Scalar>& H_free,
        const Eigen::VectorX<Scalar>& g_free,
        const std::vector<int>& free_indices,
        int n_free)
    {
        // Steepest descent direction (g = J^T V_b points toward solution)
        Eigen::VectorX<Scalar> delta_sd = g_free;

        // Extract J_b columns for free indices
        Eigen::Matrix<Scalar, 6, Eigen::Dynamic> J_b_free(6, n_free);
        for (int i = 0; i < n_free; ++i)
        {
            J_b_free.col(i) = J_b.col(free_indices[static_cast<std::size_t>(i)]);
        }

        // Optimal steepest descent step size
        Scalar g_sq = g_free.squaredNorm();
        auto Jg = (J_b_free * g_free).eval();
        Scalar Jg_sq = Jg.squaredNorm();
        Scalar t = (Jg_sq > std::numeric_limits<Scalar>::epsilon())
            ? g_sq / Jg_sq
            : Scalar(1);

        // Gauss-Newton step (with small regularization)
        Eigen::MatrixX<Scalar> H_reg = H_free;
        for (int i = 0; i < n_free; ++i)
        {
            H_reg(i, i) += std::numeric_limits<Scalar>::epsilon() * Scalar(100);
        }
        Eigen::VectorX<Scalar> delta_gn = H_reg.ldlt().solve(g_free);

        Scalar delta_gn_norm = delta_gn.norm();
        Scalar sd_scaled_norm = t * delta_sd.norm();

        // Case 1: GN step within trust region
        if (delta_gn_norm <= m_delta)
        {
            return delta_gn;
        }

        // Case 2: Scaled steepest descent exceeds trust region
        if (sd_scaled_norm >= m_delta)
        {
            return (m_delta / delta_sd.norm()) * delta_sd;
        }

        // Case 3: Dogleg interpolation
        Eigen::VectorX<Scalar> a = t * delta_sd;
        Eigen::VectorX<Scalar> b = delta_gn;
        Eigen::VectorX<Scalar> d = b - a;

        Scalar a_sq = a.squaredNorm();
        Scalar d_sq = d.squaredNorm();
        Scalar a_dot_d = a.dot(d);
        Scalar delta_sq = m_delta * m_delta;

        // Solve ||a + beta * d||^2 = Delta^2 for beta in [0, 1]
        Scalar discriminant = a_dot_d * a_dot_d - d_sq * (a_sq - delta_sq);
        Scalar beta = (-a_dot_d + std::sqrt(std::max(Scalar(0), discriminant))) / d_sq;
        beta = std::clamp(beta, Scalar(0), Scalar(1));

        return a + beta * d;
    }

    se3<Scalar> m_target{se3<Scalar>::identity()};
    position_type m_q{};
    position_type m_q_min{};
    position_type m_q_max{};
    vector6<Scalar> m_V_b{vector6<Scalar>::Zero()};
    convergence_criteria<Scalar> m_criteria{};
    error_weight<Scalar> m_weight{};
    options m_options{};
    std::vector<Scalar> m_error_history;
    Scalar m_initial_error{};
    Scalar m_error_norm{};
    Scalar m_lambda{};
    Scalar m_nu{Scalar(2)};
    Scalar m_delta{Scalar(1)};
    int m_iterations{};
    ik_status m_status{ik_status::running};
};

}

#endif
