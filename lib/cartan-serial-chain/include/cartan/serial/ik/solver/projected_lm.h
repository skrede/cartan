#ifndef HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_PROJECTED_LM_H
#define HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_PROJECTED_LM_H

/// Projected Levenberg-Marquardt IK solve policy with active-set box
///        projection, optional dogleg trust-region step, and built-in
///        Halton-seed re-seed on stall.
///
/// Speed-optimized solver: active-set projection handles joint limits
/// within the optimization step (not post-hoc clamping). The dogleg
/// option interpolates between Cauchy point and Gauss-Newton step.
///
/// On stall / divergence / per-attempt iteration_limit, the solver
/// internally re-seeds the joint configuration from a deterministic Halton
/// sequence and continues. Phase-30 H2 falsification confirmed that for
/// projected LM the load-bearing recovery mechanism is the fresh
/// low-discrepancy seed, not warm-start lambda preservation; the restart
/// loop therefore lives inside the solver rather than in a separate
/// wrapper layer.
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 6.2, p. 227-233.
///            Nielsen, H.B., Damping Parameter in Marquardt's Method, 1999.
///            Nocedal & Wright, Numerical Optimization, Ch. 4 (dogleg).
///            Beeson & Ames, "TRAC-IK", 2015 (multi-start strategy).

#include "cartan/types.h"

#include "cartan/serial/ik/ik_status.h"
#include "cartan/serial/ik/policy/limits_policy.h"
#include "cartan/serial/ik/policy/error_weight.h"
#include "cartan/serial/ik/concepts/solve_concept.h"
#include "cartan/serial/ik/detail/convergence.h"
#include "cartan/serial/ik/detail/stall_detection.h"
#include "cartan/serial/ik/detail/limit_enforcement.h"
#include "cartan/serial/ik/solver/detail/halton_seed_generator.h"

#include "cartan/lie/se3.h"
#include "cartan/serial/fk/jacobian.h"
#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/chain_concept.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include <Eigen/Dense>

#include <cmath>
#include <array>
#include <vector>
#include <limits>
#include <optional>
#include <algorithm>
#include <type_traits>

namespace cartan
{

/// Square storage for the active (free) joint sub-Hessian. For a compile-time
/// joint count N the storage is max-size-fixed: an N x N capacity buffer held
/// inline (no heap) that is resized to the runtime free-joint count without
/// allocating. Because the operand is still dispatched at runtime size, the
/// factorization operates on the identical n_free x n_free block and yields
/// bit-identical results to the heap-backed dynamic matrix it replaces. For a
/// runtime-DOF chain (N == dynamic) it falls back to the dynamic matrix.
template <typename Scalar, int N>
using free_matrix = std::conditional_t<
    N == dynamic,
    Eigen::MatrixX<Scalar>,
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, 0, N, N>>;

/// Column-vector counterpart of free_matrix (max-size-fixed N x 1 for fixed N).
template <typename Scalar, int N>
using free_vector = std::conditional_t<
    N == dynamic,
    Eigen::VectorX<Scalar>,
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, N, 1>>;

/// Projected Levenberg-Marquardt IK solve policy with active-set box projection
/// and built-in Halton-seed re-seed on stall.
template <chain Chain, typename LimitsPolicy = no_limits>
class projected_lm
{
public:
    using chain_type = Chain;
    using scalar_type = typename Chain::scalar_type;
    static constexpr int joints = Chain::joints;
    using limits_type = LimitsPolicy;

    using position_type = typename joint_state<scalar_type, joints>::position_type;

    static_assert(std::is_floating_point_v<scalar_type>, "projected_lm requires a floating-point Scalar type");

private:
    // Max-size-fixed temporaries for the per-step active-set solve. On a
    // fixed-N chain these hold their storage inline and resize within the N
    // bound without heap allocation; the LDLT path therefore performs no
    // per-step allocation. (dls uses JacobiSVD, which may allocate internally,
    // so the allocation-free guarantee is scoped to this solver.)
    static constexpr int active_capacity = (joints == dynamic) ? 1 : joints;

    using free_mat = free_matrix<scalar_type, joints>;
    using free_vec = free_vector<scalar_type, joints>;
    using free_jac = std::conditional_t<
        joints == dynamic,
        Eigen::Matrix<scalar_type, 6, Eigen::Dynamic>,
        Eigen::Matrix<scalar_type, 6, Eigen::Dynamic, 0, 6, active_capacity>>;

    using free_index_storage = std::conditional_t<
        joints == dynamic,
        std::vector<int>,
        std::array<int, static_cast<std::size_t>(active_capacity)>>;

    // Active (non-clamped) joint index list. For fixed N this is a std::array
    // plus a runtime count -- no std::vector, so no per-step heap allocation.
    struct active_set
    {
        free_index_storage indices{};
        int count{0};

        [[nodiscard]] int operator[](int i) const
        {
            return indices[static_cast<std::size_t>(i)];
        }
    };

public:

    struct options
    {
        scalar_type initial_lambda_factor{scalar_type(1e-3)};
        scalar_type stall_threshold{scalar_type(1e-6)};
        scalar_type divergence_factor{scalar_type(10)};
        int stall_window{5};
        int max_restarts{20};
        bool use_dogleg{false};
        scalar_type trust_region_radius{scalar_type(1.0)};
    };

    projected_lm() = default;

    explicit projected_lm(const options& opts)
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
        m_seed_gen.emplace(chain);
        m_restart_count = 0;
        m_total_iterations = 0;
        m_best_error = std::numeric_limits<scalar_type>::max();
        m_aborted = false;

        initialize_attempt(chain, target, q0, criteria, weight);
    }

    step_result<scalar_type> step(const Chain& chain, int N)
    {
        int units = 0;
        if (m_aborted)
        {
            return {ik_status::stalled, {0, m_error_norm}};
        }
        while (units < N && m_status != ik_status::converged)
        {
            auto inner_status = perform_lm_iteration(chain);
            ++m_total_iterations;
            ++units;

            if (inner_status == ik_status::converged)
            {
                m_status = ik_status::converged;
                break;
            }
            if (inner_status == ik_status::running)
            {
                continue;
            }

            if (m_error_norm < m_best_error)
            {
                m_best_error = m_error_norm;
            }

            if (m_restart_count >= m_options.max_restarts)
            {
                m_status = inner_status;
                break;
            }

            auto q_new = (*m_seed_gen)(m_restart_count);
            ++m_restart_count;

            // Free restart event: re-initialize the attempt without billing
            // additional units beyond the failing inner attempt's per-iter charge.
            initialize_attempt(chain, m_target, q_new, m_criteria, m_weight);
        }
        return {m_status, {units, m_error_norm}};
    }

    [[nodiscard]] bool converged() const { return m_status == ik_status::converged; }
    [[nodiscard]] const position_type& solution() const { return m_q; }
    [[nodiscard]] scalar_type error_norm() const { return m_error_norm; }
    [[nodiscard]] int iterations() const { return m_total_iterations; }
    void abort() { m_aborted = true; }
    [[nodiscard]] scalar_type lambda() const { return m_lambda; }
    void set_lambda(scalar_type l) { m_lambda = l; }
    [[nodiscard]] ik_status status() const { return m_status; }

private:
    void initialize_attempt(
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
        m_nu = scalar_type(2);
        m_delta = m_options.trust_region_radius;
        m_error_history.clear();

        int n = chain.num_joints();
        if constexpr (joints == dynamic)
        {
            m_q_min.resize(n);
            m_q_max.resize(n);
        }
        for (int i = 0; i < n; ++i)
        {
            m_q_min(i) = chain.limits()[static_cast<std::size_t>(i)].position_min;
            m_q_max(i) = chain.limits()[static_cast<std::size_t>(i)].position_max;
        }

        m_q = m_q.cwiseMax(m_q_min).cwiseMin(m_q_max);

        auto fk = forward_kinematics(chain, m_q);
        m_V_b = (fk.end_effector.inverse() * m_target).log();
        m_error_norm = m_V_b.norm();
        m_initial_error = m_error_norm;

        auto J_b = body_jacobian(chain, fk);
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

    ik_status perform_lm_iteration(const Chain& chain)
    {
        if (m_status != ik_status::running)
        {
            return m_status;
        }

        auto fk = forward_kinematics(chain, m_q);
        m_V_b = (fk.end_effector.inverse() * m_target).log();

        if (auto s = check_convergence_and_limits(); s != ik_status::running)
        {
            return s;
        }

        auto J_b = body_jacobian(chain, fk);
        int n = static_cast<int>(J_b.cols());
        auto H = (J_b.transpose() * J_b).eval();
        auto g = (J_b.transpose() * m_V_b).eval();

        auto free_indices = identify_active_set(g, n);
        position_type dq = solve_projected_system(J_b, H, g, free_indices, n);

        auto [q_trial, V_b_trial, rho] = evaluate_trial_step(chain, dq, H, g);

        update_damping(rho, dq, q_trial, V_b_trial);

        m_error_norm = m_V_b.norm();
        auto stall_result = cartan::detail::check_stall_divergence(
            m_error_history, m_error_norm, m_initial_error,
            m_options.stall_window, m_options.stall_threshold,
            m_options.divergence_factor);
        if (stall_result != ik_status::running)
        {
            m_status = stall_result;
            return m_status;
        }

        cartan::detail::enforce_limits<LimitsPolicy>(m_q, chain);
        return m_status;
    }

    ik_status check_convergence_and_limits()
    {
        if (cartan::detail::is_converged(m_V_b, m_weight, m_criteria))
        {
            m_error_norm = m_V_b.norm();
            m_status = ik_status::converged;
            return m_status;
        }

        ++m_iterations;
        if (m_iterations >= m_criteria.max_iterations_per_attempt)
        {
            m_error_norm = m_V_b.norm();
            m_status = ik_status::iteration_limit;
            return m_status;
        }

        return ik_status::running;
    }

    active_set identify_active_set(const auto& g, int n)
    {
        active_set free_indices;
        if constexpr (joints == dynamic)
        {
            free_indices.indices.reserve(static_cast<std::size_t>(n));
        }
        for (int i = 0; i < n; ++i)
        {
            bool at_lower = (m_q(i) <= m_q_min(i) + std::numeric_limits<scalar_type>::epsilon());
            bool at_upper = (m_q(i) >= m_q_max(i) - std::numeric_limits<scalar_type>::epsilon());
            bool active = (at_lower && g(i) < scalar_type(0)) ||
                          (at_upper && g(i) > scalar_type(0));
            if (!active)
            {
                if constexpr (joints == dynamic)
                {
                    free_indices.indices.push_back(i);
                }
                else
                {
                    free_indices.indices[static_cast<std::size_t>(free_indices.count)] = i;
                }
                ++free_indices.count;
            }
        }
        return free_indices;
    }

    template <typename JacobianType, typename HessianType, typename GradientType>
    position_type solve_projected_system(
        const JacobianType& J_b,
        const HessianType& H,
        const GradientType& g,
        const active_set& free_indices,
        int n)
    {
        int n_free = free_indices.count;
        position_type dq;
        if constexpr (joints == dynamic)
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

        free_mat H_free(n_free, n_free);
        free_vec g_free(n_free);
        for (int i = 0; i < n_free; ++i)
        {
            g_free(i) = g(free_indices[i]);
            for (int j = 0; j < n_free; ++j)
            {
                H_free(i, j) = H(free_indices[i], free_indices[j]);
            }
        }

        free_vec dq_free;
        if (m_options.use_dogleg)
        {
            dq_free = dogleg_step(J_b, H_free, g_free, free_indices, n_free);
        }
        else
        {
            free_mat A = H_free;
            for (int i = 0; i < n_free; ++i)
            {
                A(i, i) += m_lambda;
            }
            dq_free = A.ldlt().solve(g_free);
        }

        for (int i = 0; i < n_free; ++i)
        {
            dq(free_indices[i]) = dq_free(i);
        }
        return dq;
    }

    template <typename HessianType, typename GradientType>
    struct trial_result
    {
        position_type q_trial;
        vector6<scalar_type> V_b_trial;
        scalar_type rho;
    };

    template <typename HessianType, typename GradientType>
    auto evaluate_trial_step(
        const Chain& chain,
        const position_type& dq,
        const HessianType& H,
        const GradientType& g) -> trial_result<HessianType, GradientType>
    {
        position_type q_trial = (m_q + dq).cwiseMax(m_q_min).cwiseMin(m_q_max);
        auto fk_trial = forward_kinematics(chain, q_trial);
        auto V_b_trial = (fk_trial.end_effector.inverse() * m_target).log();

        scalar_type error_old_sq = m_V_b.squaredNorm();
        scalar_type error_new_sq = V_b_trial.squaredNorm();
        scalar_type actual_reduction = scalar_type(0.5) * (error_old_sq - error_new_sq);

        scalar_type predicted_reduction;
        if (m_options.use_dogleg)
        {
            predicted_reduction = dq.dot(g) - scalar_type(0.5) * dq.dot(H * dq);
        }
        else
        {
            predicted_reduction = scalar_type(0.5) * (dq.transpose() * (m_lambda * dq + g)).value();
        }

        scalar_type rho{0};
        if (std::abs(predicted_reduction) > std::numeric_limits<scalar_type>::epsilon())
        {
            rho = actual_reduction / predicted_reduction;
        }

        return {q_trial, V_b_trial, rho};
    }

    void update_damping(
        scalar_type rho,
        const position_type& dq,
        const position_type& q_trial,
        const vector6<scalar_type>& V_b_trial)
    {
        if (rho > scalar_type(0))
        {
            m_q = q_trial;
            m_V_b = V_b_trial;

            if (m_options.use_dogleg)
            {
                if (rho > scalar_type(0.75))
                {
                    m_delta = std::max(m_delta, scalar_type(3) * dq.norm());
                }
                else if (rho < scalar_type(0.25))
                {
                    m_delta *= scalar_type(0.25);
                }
            }
            else
            {
                scalar_type factor = scalar_type(1) - std::pow(scalar_type(2) * rho - scalar_type(1), scalar_type(3));
                m_lambda *= std::max(scalar_type(1) / scalar_type(3), factor);
                m_nu = scalar_type(2);
            }
        }
        else
        {
            if (m_options.use_dogleg)
            {
                m_delta *= scalar_type(0.25);
            }
            else
            {
                m_lambda *= m_nu;
                m_nu *= scalar_type(2);
            }
        }
    }

    template <typename JacobianType>
    free_vec dogleg_step(
        const JacobianType& J_b,
        const free_mat& H_free,
        const free_vec& g_free,
        const active_set& free_indices,
        int n_free)
    {
        free_vec delta_sd = g_free;

        free_jac J_b_free(6, n_free);
        for (int i = 0; i < n_free; ++i)
        {
            J_b_free.col(i) = J_b.col(free_indices[i]);
        }

        scalar_type g_sq = g_free.squaredNorm();
        auto Jg = (J_b_free * g_free).eval();
        scalar_type Jg_sq = Jg.squaredNorm();
        scalar_type t = (Jg_sq > std::numeric_limits<scalar_type>::epsilon())
            ? g_sq / Jg_sq
            : scalar_type(1);

        free_mat H_reg = H_free;
        for (int i = 0; i < n_free; ++i)
        {
            H_reg(i, i) += std::numeric_limits<scalar_type>::epsilon() * scalar_type(100);
        }
        free_vec delta_gn = H_reg.ldlt().solve(g_free);

        scalar_type delta_gn_norm = delta_gn.norm();
        scalar_type sd_scaled_norm = t * delta_sd.norm();

        if (delta_gn_norm <= m_delta)
        {
            return delta_gn;
        }

        if (sd_scaled_norm >= m_delta)
        {
            return (m_delta / delta_sd.norm()) * delta_sd;
        }

        free_vec a = t * delta_sd;
        free_vec b = delta_gn;
        free_vec d = b - a;

        scalar_type a_sq = a.squaredNorm();
        scalar_type d_sq = d.squaredNorm();
        scalar_type a_dot_d = a.dot(d);
        scalar_type delta_sq = m_delta * m_delta;

        scalar_type discriminant = a_dot_d * a_dot_d - d_sq * (a_sq - delta_sq);
        scalar_type beta = (-a_dot_d + std::sqrt(std::max(scalar_type(0), discriminant))) / d_sq;
        beta = std::clamp(beta, scalar_type(0), scalar_type(1));

        return a + beta * d;
    }

    se3<scalar_type> m_target{se3<scalar_type>::identity()};
    position_type m_q{};
    position_type m_q_min{};
    position_type m_q_max{};
    vector6<scalar_type> m_V_b{vector6<scalar_type>::Zero()};
    convergence_criteria<scalar_type> m_criteria{};
    error_weight<scalar_type> m_weight{};
    options m_options{};
    std::optional<halton_seed_generator<Chain>> m_seed_gen{};
    cartan::detail::error_ring<scalar_type> m_error_history;
    scalar_type m_initial_error{};
    scalar_type m_error_norm{};
    scalar_type m_lambda{};
    scalar_type m_nu{scalar_type(2)};
    scalar_type m_delta{scalar_type(1)};
    scalar_type m_best_error{std::numeric_limits<scalar_type>::max()};
    int m_iterations{};
    int m_total_iterations{};
    int m_restart_count{};
    ik_status m_status{ik_status::running};
    bool m_aborted{false};
};

}

#endif
