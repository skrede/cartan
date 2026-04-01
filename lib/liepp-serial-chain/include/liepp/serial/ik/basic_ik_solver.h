#ifndef HPP_GUARD_LIEPP_SERIAL_IK_BASIC_IK_SOLVER_H
#define HPP_GUARD_LIEPP_SERIAL_IK_BASIC_IK_SOLVER_H

/// @file basic_ik_solver.h
/// @brief Variadic policy-based IK solver with cooperative interleaved racing.
///
/// basic_ik_solver<Policies...> provides direct solve for a single policy and
/// cooperative round-robin racing for two or more policies. The single-policy
/// path produces identical code to a non-variadic solver (zero overhead).
/// The multi-policy path absorbs the round-robin tick logic formerly in
/// racing_scheduler: each step() performs one round-robin across all active
/// policies, parking converged ones and selecting the best result based on
/// the configured objective.
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 6.2, p. 227-233.
///            Phase 13 decisions D-01 through D-09.

#include "liepp/serial/ik/ik_types.h"
#include "liepp/serial/ik/ik_solve_policy.h"
#include "liepp/serial/ik/halton_seed_generator.h"

#include "liepp/lie/se3.h"
#include "liepp/serial/chain/joint_state.h"
#include "liepp/serial/fk/jacobian.h"
#include "liepp/serial/chain/kinematic_chain.h"
#include "liepp/serial/fk/forward_kinematics.h"

#include <Eigen/SVD>

#include <array>
#include <cmath>
#include <tuple>
#include <limits>
#include <optional>
#include <expected>
#include <concepts>
#include <type_traits>

namespace liepp
{

/// Check that all policies in a variadic pack agree on scalar_type and joints.
template <typename First, typename... Rest>
consteval bool all_policies_agree()
{
    if constexpr (sizeof...(Rest) == 0)
        return true;
    else
        return ((std::same_as<typename First::scalar_type, typename Rest::scalar_type>
                 && First::joints == Rest::joints) && ...);
}

/// Variadic policy-based IK solver with cooperative interleaved racing.
///
/// When instantiated with a single policy, behaves identically to a
/// non-variadic solver with zero overhead. With two or more policies,
/// provides cooperative round-robin racing with parking and objective-based
/// result selection.
///
/// Thread safety: Different solver instances may safely operate concurrently
/// on the same const kinematic_chain. A single solver instance must not be
/// used from multiple threads without synchronization.
///
/// @tparam Policies  One or more IK solve policies satisfying ik_solve_policy.
template <typename... Policies>
    requires (sizeof...(Policies) >= 1) && (ik_solve_policy<Policies> && ...)
class basic_ik_solver
{
    using first_policy = std::tuple_element_t<0, std::tuple<Policies...>>;

    static_assert(std::is_floating_point_v<typename first_policy::scalar_type>,
        "basic_ik_solver requires a floating-point Scalar type");
    static_assert(all_policies_agree<Policies...>(),
        "All policies must agree on scalar_type and joints");

public:
    using scalar_type = typename first_policy::scalar_type;
    static constexpr int joints = first_policy::joints;

    using position_type = typename joint_state<scalar_type, joints>::position_type;

    basic_ik_solver() = default;

    explicit basic_ik_solver(Policies... policies)
        : m_policies(std::move(policies)...)
    {
    }

    /// Initialize the solver with chain, target, seed, criteria, and options.
    ///
    /// The first policy receives the user's q0. Remaining policies (if any)
    /// receive deterministic Halton seeds within joint limits.
    void setup(
        const kinematic_chain<scalar_type, joints>& chain,
        const se3<scalar_type>& target,
        const position_type& q0,
        const convergence_criteria<scalar_type>& criteria,
        const solver_options<scalar_type>& options = {})
    {
        m_chain = &chain;
        m_target = target;
        m_criteria = criteria;
        m_objective = options.objective;
        m_status = ik_status::running;
        m_best_error = std::numeric_limits<scalar_type>::max();
        m_best_manipulability = scalar_type(0);
        m_best_isotropy = scalar_type(0);
        m_total_iterations = 0;
        m_found_convergence = false;
        m_best_q = q0;

        if constexpr (sizeof...(Policies) == 1)
        {
            std::get<0>(m_policies).setup(chain, target, q0, criteria);
        }
        else
        {
            m_max_total_iterations = options.max_total_iterations;
            m_early_stop = false;
            m_parked = {};
            m_results = {};
            m_best_solver_index = -1;

            m_seed_gen.emplace(chain);

            // First policy gets user's q0
            std::get<0>(m_policies).setup(chain, target, q0, criteria);

            // Remaining policies get Halton seeds
            setup_remaining_policies(chain, target, criteria, options.halton_seed,
                std::make_index_sequence<sizeof...(Policies) - 1>{});
        }
    }

    /// Execute one solver step.
    ///
    /// For single-policy: delegates directly to the policy's step().
    /// For multi-policy: one round-robin across all active (non-parked) policies.
    ik_status step()
    {
        if (m_status != ik_status::running)
        {
            return m_status;
        }

        if constexpr (sizeof...(Policies) == 1)
        {
            return step_single();
        }
        else
        {
            return step_multi();
        }
    }

    /// Execute n round-robin rounds, stopping early on terminal status.
    ik_status step_n(int n)
    {
        for (int i = 0; i < n; ++i)
        {
            auto s = step();
            if (s != ik_status::running)
            {
                return s;
            }
        }
        return m_status;
    }

    /// Run to completion. Returns the best result or an error.
    std::expected<ik_result<scalar_type, joints>, ik_error<scalar_type, joints>> solve()
    {
        if constexpr (sizeof...(Policies) == 1)
        {
            for (int i = 0; i < m_criteria.max_iterations * 2; ++i)
            {
                auto s = step();
                if (s != ik_status::running)
                    break;
            }
        }
        else
        {
            while (step() == ik_status::running) {}
        }

        return build_result();
    }

    /// Whether the solver has converged.
    [[nodiscard]] bool converged() const { return m_status == ik_status::converged; }

    /// Current best error norm.
    [[nodiscard]] scalar_type error_norm() const
    {
        if constexpr (sizeof...(Policies) == 1)
        {
            return std::get<0>(m_policies).error_norm();
        }
        else
        {
            if (m_best_solver_index >= 0)
            {
                scalar_type best = std::numeric_limits<scalar_type>::max();
                find_best_error(best, std::index_sequence_for<Policies...>{});
                return best;
            }
            return lowest_error_norm(std::index_sequence_for<Policies...>{});
        }
    }

    /// Number of iterations executed (total across all policies).
    [[nodiscard]] int iterations() const { return m_total_iterations; }

    /// Current best joint configuration.
    [[nodiscard]] const position_type& current_q() const { return m_best_q; }

    /// Current solver status.
    [[nodiscard]] ik_status status() const { return m_status; }

    /// Abort all policies.
    void abort()
    {
        abort_all(std::index_sequence_for<Policies...>{});
        m_status = ik_status::running; // allow status query after abort
    }

private:
    /// Result saved when a policy is parked (converged or terminal failure).
    struct parked_result
    {
        position_type q;
        scalar_type error_norm;
        int iterations;
        bool converged;
    };

    // ── Single-policy fast path ──────────────────────────────────────────

    ik_status step_single()
    {
        ++m_total_iterations;
        auto policy_status = std::get<0>(m_policies).step(*m_chain);
        position_type q = std::get<0>(m_policies).solution();

        if (policy_status == ik_status::converged)
        {
            m_found_convergence = true;

            if (m_objective == ik_objective::speed)
            {
                m_status = ik_status::converged;
                m_best_q = q;
                return m_status;
            }

            update_best(q);

            if (m_total_iterations >= m_criteria.max_iterations)
            {
                m_status = ik_status::converged;
                return m_status;
            }

            std::get<0>(m_policies).setup(*m_chain, m_target, q, m_criteria);
            return ik_status::running;
        }

        if (policy_status == ik_status::diverged ||
            policy_status == ik_status::stalled ||
            policy_status == ik_status::iteration_limit)
        {
            m_status = m_found_convergence ? ik_status::converged : policy_status;
            return m_status;
        }

        return ik_status::running;
    }

    // ── Multi-policy racing path ─────────────────────────────────────────

    ik_status step_multi()
        requires (sizeof...(Policies) > 1)
    {
        bool any_running = false;

        [&]<std::size_t... Is>(std::index_sequence<Is...>)
        {
            (step_one<Is>(any_running), ...);
        }(std::index_sequence_for<Policies...>{});

        ++m_total_iterations;

        if (m_total_iterations >= m_max_total_iterations)
        {
            m_status = m_found_convergence ? ik_status::converged : ik_status::iteration_limit;
            return m_status;
        }

        if (!any_running)
        {
            m_status = m_found_convergence ? ik_status::converged : ik_status::iteration_limit;
            return m_status;
        }

        return ik_status::running;
    }

    template <std::size_t I>
    void step_one(bool& any_running)
        requires (sizeof...(Policies) > 1)
    {
        if (m_early_stop || m_parked[I])
            return;

        auto& policy = std::get<I>(m_policies);
        auto status = policy.step(*m_chain);

        if (status == ik_status::converged)
        {
            m_parked[I] = true;
            m_results[I].emplace(parked_result{
                .q = policy.solution(),
                .error_norm = policy.error_norm(),
                .iterations = policy.iterations(),
                .converged = true
            });
            m_found_convergence = true;

            if (m_objective == ik_objective::speed)
            {
                m_early_stop = true;
                m_best_solver_index = static_cast<int>(I);
                m_best_q = policy.solution();
                park_all();
                return;
            }
        }
        else if (is_terminal(status))
        {
            m_parked[I] = true;
            m_results[I].emplace(parked_result{
                .q = policy.solution(),
                .error_norm = policy.error_norm(),
                .iterations = policy.iterations(),
                .converged = false
            });
        }
        else
        {
            any_running = true;
        }
    }

    static bool is_terminal(ik_status s)
    {
        return s == ik_status::diverged
            || s == ik_status::stalled
            || s == ik_status::iteration_limit
            || s == ik_status::joint_limit_hit;
    }

    void park_all()
        requires (sizeof...(Policies) > 1)
    {
        m_parked.fill(true);
    }

    // ── Setup helpers ────────────────────────────────────────────────────

    template <std::size_t... Is>
    void setup_remaining_policies(
        const kinematic_chain<scalar_type, joints>& chain,
        const se3<scalar_type>& target,
        const convergence_criteria<scalar_type>& criteria,
        unsigned int halton_seed_offset,
        std::index_sequence<Is...>)
    {
        (setup_policy<Is + 1>(chain, target, criteria, halton_seed_offset, Is), ...);
    }

    template <std::size_t I>
    void setup_policy(
        const kinematic_chain<scalar_type, joints>& chain,
        const se3<scalar_type>& target,
        const convergence_criteria<scalar_type>& criteria,
        unsigned int halton_seed_offset,
        std::size_t policy_index)
    {
        auto seed = (*m_seed_gen)(static_cast<int>(policy_index + halton_seed_offset));
        std::get<I>(m_policies).setup(chain, target, seed, criteria);
    }

    // ── Objective tracking (single-policy) ───────────────────────────────

    void update_best(const position_type& q)
    {
        scalar_type err = std::get<0>(m_policies).error_norm();

        switch (m_objective)
        {
            case ik_objective::min_distance:
            {
                if (err < m_best_error)
                {
                    m_best_error = err;
                    m_best_q = q;
                }
                break;
            }
            case ik_objective::max_manipulability:
            {
                auto fk = forward_kinematics(*m_chain, q);
                auto J_b = body_jacobian(*m_chain, fk);
                constexpr unsigned int svd_opts = (joints == dynamic)
                    ? (Eigen::ComputeThinU | Eigen::ComputeThinV)
                    : (Eigen::ComputeFullU | Eigen::ComputeFullV);
                Eigen::JacobiSVD<jacobian_matrix<scalar_type, joints>> svd(J_b, svd_opts);
                auto sigma = svd.singularValues();
                scalar_type manip = scalar_type(1);
                for (int i = 0; i < static_cast<int>(sigma.size()); ++i)
                {
                    manip *= sigma(i);
                }
                if (manip > m_best_manipulability)
                {
                    m_best_manipulability = manip;
                    m_best_q = q;
                }
                break;
            }
            case ik_objective::max_isotropy:
            {
                auto fk = forward_kinematics(*m_chain, q);
                auto J_b = body_jacobian(*m_chain, fk);
                constexpr unsigned int svd_opts = (joints == dynamic)
                    ? (Eigen::ComputeThinU | Eigen::ComputeThinV)
                    : (Eigen::ComputeFullU | Eigen::ComputeFullV);
                Eigen::JacobiSVD<jacobian_matrix<scalar_type, joints>> svd(J_b, svd_opts);
                auto sigma = svd.singularValues();
                int rank = static_cast<int>(sigma.size());
                scalar_type isotropy = (sigma(0) > scalar_type(0))
                    ? sigma(rank - 1) / sigma(0)
                    : scalar_type(0);
                if (isotropy > m_best_isotropy)
                {
                    m_best_isotropy = isotropy;
                    m_best_q = q;
                }
                break;
            }
            default:
            {
                m_best_q = q;
                break;
            }
        }
    }

    // ── Result construction ──────────────────────────────────────────────

    std::expected<ik_result<scalar_type, joints>, ik_error<scalar_type, joints>> build_result()
    {
        if (m_status == ik_status::converged)
        {
            if constexpr (sizeof...(Policies) == 1)
            {
                ik_result<scalar_type, joints> result;
                result.solution = joint_state<scalar_type, joints>::from_position(m_best_q);
                result.iterations = m_total_iterations;
                result.final_error_norm = std::get<0>(m_policies).error_norm();
                result.solver_index = 0;
                return result;
            }
            else
            {
                return select_best_result(std::index_sequence_for<Policies...>{});
            }
        }

        return build_error();
    }

    template <std::size_t... Is>
    std::expected<ik_result<scalar_type, joints>, ik_error<scalar_type, joints>>
    select_best_result(std::index_sequence<Is...>)
        requires (sizeof...(Policies) > 1)
    {
        // For speed objective, m_best_solver_index is already set
        if (m_objective == ik_objective::speed && m_best_solver_index >= 0)
        {
            return make_result_from_parked(m_best_solver_index);
        }

        // For other objectives, find best among converged results
        int best_index = -1;
        scalar_type best_metric = std::numeric_limits<scalar_type>::max();

        auto check = [&]<std::size_t I>()
        {
            if (m_results[I] && m_results[I]->converged)
            {
                scalar_type metric = m_results[I]->error_norm;
                if (m_objective == ik_objective::min_distance)
                {
                    if (metric < best_metric)
                    {
                        best_metric = metric;
                        best_index = static_cast<int>(I);
                    }
                }
                else
                {
                    // For max_manipulability / max_isotropy, we'd need to
                    // compute SVD-based metrics. Use error_norm as tiebreaker.
                    if (best_index < 0 || metric < best_metric)
                    {
                        best_metric = metric;
                        best_index = static_cast<int>(I);
                    }
                }
            }
        };
        (check.template operator()<Is>(), ...);

        if (best_index >= 0)
        {
            return make_result_from_parked(best_index);
        }

        return build_error();
    }

    std::expected<ik_result<scalar_type, joints>, ik_error<scalar_type, joints>>
    make_result_from_parked(int index)
        requires (sizeof...(Policies) > 1)
    {
        ik_result<scalar_type, joints> result;
        auto& pr = *m_results[static_cast<std::size_t>(index)];
        result.solution = joint_state<scalar_type, joints>::from_position(pr.q);
        result.iterations = m_total_iterations;
        result.final_error_norm = pr.error_norm;
        result.solver_index = index;
        return result;
    }

    std::expected<ik_result<scalar_type, joints>, ik_error<scalar_type, joints>> build_error()
    {
        ik_error<scalar_type, joints> err;
        err.near_singular = false;
        err.condition_number = scalar_type(0);

        if constexpr (sizeof...(Policies) == 1)
        {
            err.last_q = std::get<0>(m_policies).solution();
            err.last_error_norm = std::get<0>(m_policies).error_norm();
        }
        else
        {
            // Find the policy with lowest error norm
            scalar_type min_err = std::numeric_limits<scalar_type>::max();
            int best_fail = 0;
            for (std::size_t i = 0; i < sizeof...(Policies); ++i)
            {
                if (m_results[i] && m_results[i]->error_norm < min_err)
                {
                    min_err = m_results[i]->error_norm;
                    best_fail = static_cast<int>(i);
                }
            }
            if (m_results[static_cast<std::size_t>(best_fail)])
            {
                err.last_q = m_results[static_cast<std::size_t>(best_fail)]->q;
                err.last_error_norm = m_results[static_cast<std::size_t>(best_fail)]->error_norm;
            }
            else
            {
                err.last_q = m_best_q;
                err.last_error_norm = std::numeric_limits<scalar_type>::max();
            }
        }

        switch (m_status)
        {
            case ik_status::diverged:
                err.reason = ik_failure::diverged;
                break;
            case ik_status::stalled:
                err.reason = ik_failure::stalled;
                break;
            case ik_status::iteration_limit:
                err.reason = ik_failure::iteration_limit;
                break;
            case ik_status::joint_limit_hit:
                err.reason = ik_failure::joint_limit_violation;
                break;
            default:
                err.reason = ik_failure::iteration_limit;
                break;
        }

        return std::unexpected(err);
    }

    // ── Accessor helpers ─────────────────────────────────────────────────

    template <std::size_t... Is>
    void find_best_error(scalar_type& best, std::index_sequence<Is...>) const
        requires (sizeof...(Policies) > 1)
    {
        auto check = [&]<std::size_t I>()
        {
            if (m_results[I] && m_results[I]->converged)
            {
                if (m_results[I]->error_norm < best)
                    best = m_results[I]->error_norm;
            }
        };
        (check.template operator()<Is>(), ...);
    }

    template <std::size_t... Is>
    scalar_type lowest_error_norm(std::index_sequence<Is...>) const
        requires (sizeof...(Policies) > 1)
    {
        scalar_type best = std::numeric_limits<scalar_type>::max();
        auto check = [&]<std::size_t I>()
        {
            auto norm = std::get<I>(m_policies).error_norm();
            if (norm < best)
                best = norm;
        };
        (check.template operator()<Is>(), ...);
        return best;
    }

    template <std::size_t... Is>
    void abort_all(std::index_sequence<Is...>)
    {
        (std::get<Is>(m_policies).abort(), ...);
    }

    // ── Data members ─────────────────────────────────────────────────────

    std::tuple<Policies...> m_policies{};
    const kinematic_chain<scalar_type, joints>* m_chain{nullptr};
    se3<scalar_type> m_target{se3<scalar_type>::identity()};
    convergence_criteria<scalar_type> m_criteria{};
    position_type m_best_q{};
    scalar_type m_best_manipulability{};
    scalar_type m_best_isotropy{};
    scalar_type m_best_error{};
    ik_objective m_objective{ik_objective::speed};
    ik_status m_status{ik_status::running};
    int m_total_iterations{};
    bool m_found_convergence{false};

    // Multi-policy members (only used when sizeof...(Policies) > 1, but
    // declared unconditionally for simplicity; if constexpr gates all usage).
    std::array<bool, sizeof...(Policies)> m_parked{};
    std::array<std::optional<parked_result>, sizeof...(Policies)> m_results{};
    std::optional<halton_seed_generator<scalar_type, joints>> m_seed_gen{};
    int m_max_total_iterations{500};
    int m_best_solver_index{-1};
    bool m_early_stop{false};
};

// CTAD deduction guide: deduce Policies... from constructor arguments
template <typename... Policies>
basic_ik_solver(Policies...) -> basic_ik_solver<Policies...>;

}

#endif
