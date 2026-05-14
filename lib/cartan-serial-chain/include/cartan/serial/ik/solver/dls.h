#ifndef HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_DLS_H
#define HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_DLS_H

/// Damped least squares IK solve policy with SVD-based adaptive damping.
///
/// Body-frame Newton-Raphson iteration with Nakamura's adaptive damping
/// to handle near-singular configurations gracefully. The damping factor
/// increases as the smallest singular value of the body Jacobian drops
/// below a threshold, preventing wild joint velocities near singularities.
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 6.2, p. 227-233.
///            Nakamura, Y., Advanced Robotics: Redundancy and Optimization, 1991.

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

#include <Eigen/SVD>
#include <Eigen/Dense>

#include <cmath>
#include <vector>
#include <algorithm>

namespace cartan::ik
{

/// Damped least squares IK solve policy with SVD-based adaptive damping (Nakamura).
///
/// Each step() call: compute FK, body-frame error, check convergence,
/// compute body Jacobian, SVD, adaptive damping, pseudoinverse step.
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 6.2, Eq. 6.8-6.10.
///            Nakamura, Advanced Robotics, Ch. 11 (adaptive DLS).
template <chain Chain, typename LimitsPolicy = no_limits>
class dls
{
public:
    using chain_type = Chain;
    using scalar_type = typename Chain::scalar_type;
    static constexpr int joints = Chain::joints;
    using limits_type = LimitsPolicy;

    using position_type = typename joint_state<scalar_type, joints>::position_type;

    static_assert(std::is_floating_point_v<scalar_type>, "dls requires a floating-point Scalar type");

    /// Tunable parameters for the DLS solve policy.
    struct options
    {
        scalar_type singularity_threshold{scalar_type(0.01)};
        scalar_type lambda_max{scalar_type(0.04)};
        scalar_type stall_threshold{scalar_type(1e-6)};
        scalar_type divergence_factor{scalar_type(10)};
        int stall_window{5};
    };

    dls() = default;

    explicit dls(const options& opts)
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
        m_error_history.clear();
        m_condition_number = scalar_type(0);
        m_manipulability_value = scalar_type(0);

        auto fk = forward_kinematics(chain, m_q);
        auto V_b = (fk.end_effector.inverse() * m_target).log();
        m_error_norm = V_b.norm();
        m_initial_error = m_error_norm;
    }

    step_result<scalar_type> step(const Chain& chain, int N)
    {
        int units = 0;
        while (units < N && m_status == ik_status::running)
        {
            auto fk = forward_kinematics(chain, m_q);
            auto V_b = (fk.end_effector.inverse() * m_target).log();

            if (cartan::detail::is_converged_unweighted(V_b, m_criteria))
            {
                m_error_norm = V_b.norm();
                m_status = ik_status::converged;
                // Pre-step convergence check is itself one algorithmic-work
                // unit: the FK + body-twist + tolerance test that just fired
                // is the work performed this iteration. Billing zero here
                // breaks the runner's min_units_per_step contract on entry-
                // is-converged paths (basic_ik_runner.solve() would loop
                // forever under min_distance objective with no forward
                // progress on units).
                ++m_iterations;
                ++units;
                break;
            }

            ++m_iterations;
            ++units;
            if (m_iterations >= m_criteria.max_iterations_per_attempt)
            {
                m_error_norm = V_b.norm();
                m_status = ik_status::iteration_limit;
                break;
            }

            auto J_b = body_jacobian(chain, fk);

            constexpr unsigned int svd_options = (joints == dynamic)
                ? (Eigen::ComputeThinU | Eigen::ComputeThinV)
                : (Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::JacobiSVD<jacobian_matrix<scalar_type, joints>> svd(J_b, svd_options);

            auto sigma = svd.singularValues();
            int rank = static_cast<int>(sigma.size());

            scalar_type sigma_min = sigma(rank - 1);
            scalar_type sigma_max = sigma(0);
            scalar_type lambda_sq{0};

            if (sigma_min < m_options.singularity_threshold)
            {
                scalar_type ratio = scalar_type(1) - (sigma_min / m_options.singularity_threshold)
                               * (sigma_min / m_options.singularity_threshold);
                lambda_sq = m_options.lambda_max * m_options.lambda_max * ratio;
            }

            Eigen::VectorX<scalar_type> damped(rank);
            for (int i = 0; i < rank; ++i)
            {
                damped(i) = sigma(i) / (sigma(i) * sigma(i) + lambda_sq);
            }

            auto U_r = svd.matrixU().leftCols(rank);
            auto V_r = svd.matrixV().leftCols(rank);
            position_type dq = V_r * damped.asDiagonal() * U_r.transpose() * V_b;

            m_q += dq;
            m_error_norm = V_b.norm();

            auto stall_result = cartan::detail::check_stall_divergence(
                m_error_history, m_error_norm, m_initial_error,
                m_options.stall_window, m_options.stall_threshold,
                m_options.divergence_factor);
            if (stall_result != ik_status::running)
            {
                m_status = stall_result;
                break;
            }

            m_condition_number = (sigma_min > scalar_type(0))
                ? sigma_max / sigma_min
                : std::numeric_limits<scalar_type>::infinity();

            m_manipulability_value = scalar_type(1);
            for (int i = 0; i < rank; ++i)
            {
                m_manipulability_value *= sigma(i);
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
    [[nodiscard]] scalar_type condition_number() const { return m_condition_number; }
    [[nodiscard]] scalar_type manipulability() const { return m_manipulability_value; }
    [[nodiscard]] ik_status status() const { return m_status; }

private:
    se3<scalar_type> m_target{se3<scalar_type>::identity()};
    position_type m_q{};
    convergence_criteria<scalar_type> m_criteria{};
    options m_options{};
    std::vector<scalar_type> m_error_history;
    scalar_type m_manipulability_value{};
    scalar_type m_condition_number{};
    scalar_type m_initial_error{};
    scalar_type m_error_norm{};
    int m_iterations{};
    ik_status m_status{ik_status::running};
};

}

#endif
