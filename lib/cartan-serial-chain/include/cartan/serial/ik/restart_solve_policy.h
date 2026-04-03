#ifndef HPP_GUARD_LIEPP_SERIAL_IK_RESTART_SOLVE_POLICY_H
#define HPP_GUARD_LIEPP_SERIAL_IK_RESTART_SOLVE_POLICY_H

/// @file restart_solve_policy.h
/// @brief Restart wrapper solve policy that re-seeds from Halton sequences on
///        stall or diverge, with warm-start lambda preservation.
///
/// Wraps any inner policy satisfying ik_solve_policy. When the inner policy
/// reports stalled, diverged, or iteration_limit, the wrapper generates a
/// new seed configuration from a Halton sequence and re-initializes the
/// inner policy. The best damping parameter (lambda) from near-miss
/// attempts is preserved across restarts for warm-starting.
///
/// Reference: Beeson & Ames, "TRAC-IK", 2015 (multi-start strategy).
///            Decisions D-07 (restart as stepper), D-08 (step intercept),
///            D-11 (warm-start lambda), MULTI-01, MULTI-04.

#include "liepp/serial/ik/ik_types.h"
#include "liepp/serial/ik/error_weight.h"
#include "liepp/serial/ik/ik_solve_policy.h"
#include "liepp/serial/ik/halton_seed_generator.h"
#include "liepp/serial/ik/projected_lm_solve_policy.h"

#include "liepp/lie/se3.h"
#include "liepp/serial/chain/joint_state.h"
#include "liepp/serial/chain/chain_concept.h"

#include <limits>
#include <concepts>
#include <optional>
#include <type_traits>

namespace liepp
{

namespace detail
{

/// Detect if a policy has a set_lambda(Scalar) method for warm-start support.
template <typename S>
concept has_set_lambda = requires(S& s, typename S::scalar_type l)
{
    { s.set_lambda(l) };
};

/// Detect if a policy has a lambda() const method for reading current damping.
template <typename S>
concept has_lambda = requires(const S& s)
{
    { s.lambda() } -> std::convertible_to<typename S::scalar_type>;
};

}

/// Restart wrapper solve policy with warm-start lambda preservation.
///
/// Satisfies ik_solve_policy so it composes transparently with basic_ik_solver
/// and racing_scheduler. When the inner policy reports a terminal non-converged
/// status (stalled, diverged, iteration_limit), the wrapper re-seeds from
/// Halton sequences. The best lambda from near-miss attempts is carried
/// forward to accelerate convergence on subsequent restarts.
///
/// @tparam Chain        Chain type satisfying the chain concept.
/// @tparam InnerPolicy  Policy type satisfying ik_solve_policy concept.
/// @tparam LimitsPolicy Limit enforcement policy (defaults to inner policy's limits_type).
template <chain Chain,
          typename InnerPolicy = projected_lm_solve_policy<Chain>,
          typename LimitsPolicy = typename InnerPolicy::limits_type>
class restart_solve_policy
{
    using scalar_type_impl = typename Chain::scalar_type;
    static_assert(std::is_floating_point_v<scalar_type_impl>,
        "restart_solve_policy requires a floating-point Scalar type");

public:
    using chain_type = Chain;
    using scalar_type = scalar_type_impl;
    static constexpr int joints = Chain::joints;
    using limits_type = LimitsPolicy;

    using position_type = typename joint_state<scalar_type, joints>::position_type;

    struct options
    {
        int max_restarts{20};
    };

    restart_solve_policy() = default;

    explicit restart_solve_policy(InnerPolicy inner)
        : m_inner(std::move(inner))
    {
    }

    explicit restart_solve_policy(const options& opts)
        : m_options(opts)
    {
    }

    restart_solve_policy(const options& opts, InnerPolicy inner)
        : m_inner(std::move(inner))
        , m_options(opts)
    {
    }

    /// Initialize the policy (satisfies ik_solve_policy concept).
    void setup(
        const Chain& chain,
        const se3<scalar_type>& target,
        const position_type& q0,
        const convergence_criteria<scalar_type>& criteria)
    {
        m_chain = &chain;
        m_target = target;
        m_criteria = criteria;
        m_weight.reset();
        m_seed_gen.emplace(chain);
        m_restart_count = 0;
        m_total_iterations = 0;
        m_best_lambda = scalar_type(0);
        m_best_error = std::numeric_limits<scalar_type>::max();
        m_aborted = false;

        m_inner.setup(chain, target, q0, criteria);
    }

    /// Initialize with error weighting for position/orientation emphasis.
    void setup(
        const Chain& chain,
        const se3<scalar_type>& target,
        const position_type& q0,
        const convergence_criteria<scalar_type>& criteria,
        const error_weight<scalar_type>& weight)
    {
        m_chain = &chain;
        m_target = target;
        m_criteria = criteria;
        m_weight = weight;
        m_seed_gen.emplace(chain);
        m_restart_count = 0;
        m_total_iterations = 0;
        m_best_lambda = scalar_type(0);
        m_best_error = std::numeric_limits<scalar_type>::max();
        m_aborted = false;

        if constexpr (requires { m_inner.setup(chain, target, q0, criteria, weight); })
        {
            m_inner.setup(chain, target, q0, criteria, weight);
        }
        else
        {
            m_inner.setup(chain, target, q0, criteria);
        }
    }

    /// Execute one step, intercepting terminal states for re-seed.
    ///
    /// Per D-08: delegates to inner policy, intercepts stalled/diverged/
    /// iteration_limit to trigger re-seed from Halton sequence.
    /// Per D-11: preserves best lambda from near-miss attempts.
    ik_status step(const Chain& chain)
    {
        if (m_aborted)
        {
            return ik_status::stalled;
        }

        auto status = m_inner.step(chain);
        ++m_total_iterations;

        if (status == ik_status::converged)
        {
            return ik_status::converged;
        }

        if (status == ik_status::running)
        {
            return ik_status::running;
        }

        // Terminal non-converged: stalled, diverged, or iteration_limit
        // Track warm-start lambda from near-miss (D-11, MULTI-04)
        track_best_lambda();

        if (m_restart_count >= m_options.max_restarts)
        {
            return status;
        }

        // Generate next seed from Halton sequence
        auto q_new = (*m_seed_gen)(m_restart_count);

        // Re-setup inner policy with new seed
        if (m_weight.has_value())
        {
            if constexpr (requires { m_inner.setup(chain, m_target, q_new, m_criteria, *m_weight); })
            {
                m_inner.setup(chain, m_target, q_new, m_criteria, *m_weight);
            }
            else
            {
                m_inner.setup(chain, m_target, q_new, m_criteria);
            }
        }
        else
        {
            m_inner.setup(chain, m_target, q_new, m_criteria);
        }

        // Warm-start lambda (D-11): set best lambda if available
        apply_warm_start_lambda();

        ++m_restart_count;
        return ik_status::running;
    }

    [[nodiscard]] bool converged() const { return m_inner.converged(); }
    [[nodiscard]] position_type solution() const { return m_inner.solution(); }
    [[nodiscard]] scalar_type error_norm() const { return m_inner.error_norm(); }
    [[nodiscard]] int iterations() const { return m_total_iterations; }

    void abort()
    {
        m_aborted = true;
        m_inner.abort();
    }

private:
    /// Track best lambda from near-miss attempts for warm-start.
    void track_best_lambda()
    {
        scalar_type current_error = m_inner.error_norm();
        if (current_error < m_best_error)
        {
            m_best_error = current_error;
            if constexpr (detail::has_lambda<InnerPolicy>)
            {
                m_best_lambda = m_inner.lambda();
            }
        }
    }

    /// Apply warm-start lambda to inner policy after re-setup.
    void apply_warm_start_lambda()
    {
        if constexpr (detail::has_set_lambda<InnerPolicy>)
        {
            if (m_best_lambda > scalar_type(0))
            {
                m_inner.set_lambda(m_best_lambda);
            }
        }
    }

    InnerPolicy m_inner{};
    options m_options{};
    const Chain* m_chain{nullptr};
    se3<scalar_type> m_target{se3<scalar_type>::identity()};
    convergence_criteria<scalar_type> m_criteria{};
    std::optional<error_weight<scalar_type>> m_weight{};
    std::optional<halton_seed_generator<Chain>> m_seed_gen{};
    int m_restart_count{};
    int m_total_iterations{};
    scalar_type m_best_lambda{};
    scalar_type m_best_error{std::numeric_limits<scalar_type>::max()};
    bool m_aborted{false};
};

// Deduction guide: deduce everything from inner policy
template <typename InnerPolicy>
restart_solve_policy(InnerPolicy) -> restart_solve_policy<
    typename InnerPolicy::chain_type,
    InnerPolicy,
    typename InnerPolicy::limits_type>;

// Deduction guide: deduce from options + inner policy
template <typename InnerPolicy>
restart_solve_policy(typename restart_solve_policy<
    typename InnerPolicy::chain_type,
    InnerPolicy,
    typename InnerPolicy::limits_type>::options,
    InnerPolicy) -> restart_solve_policy<
    typename InnerPolicy::chain_type,
    InnerPolicy,
    typename InnerPolicy::limits_type>;

}

#endif
