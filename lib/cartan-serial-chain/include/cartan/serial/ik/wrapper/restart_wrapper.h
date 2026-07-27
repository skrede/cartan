#ifndef HPP_GUARD_CARTAN_SERIAL_IK_WRAPPER_RESTART_WRAPPER_H
#define HPP_GUARD_CARTAN_SERIAL_IK_WRAPPER_RESTART_WRAPPER_H

/// Restart wrapper solve policy that re-seeds from Halton sequences on
///        stall or diverge, with warm-start lambda preservation.
///
/// Wraps any inner policy satisfying solve_policy. When the inner policy
/// reports stalled, diverged, or iteration_limit, the wrapper generates a
/// new seed configuration from a Halton sequence and re-initializes the
/// inner policy. The best damping parameter (lambda) from near-miss
/// attempts is preserved across restarts for warm-starting.
///
/// Reference: Beeson & Ames, "TRAC-IK", 2015 (multi-start strategy).

#include "cartan/serial/ik/ik_status.h"
#include "cartan/serial/ik/policy/error_weight.h"
#include "cartan/serial/ik/concepts/solve_concept.h"
#include "cartan/serial/ik/detail/setup_validation.h"
#include "cartan/serial/ik/detail/limit_enforcement.h"
#include "cartan/serial/ik/solver/detail/halton_seed_generator.h"
#include "cartan/serial/ik/solver/projected_lm.h"

#include "cartan/lie/se3.h"
#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/chain_concept.h"

#include <limits>
#include <concepts>
#include <optional>
#include <functional>
#include <type_traits>

namespace cartan
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
/// Satisfies solve_policy so it composes transparently with basic_ik_runner
/// and racing. When the inner policy reports a terminal non-converged
/// status (stalled, diverged, iteration_limit), the wrapper re-seeds from
/// Halton sequences. The best lambda from near-miss attempts is carried
/// forward to accelerate convergence on subsequent restarts.
template <chain Chain,
          typename InnerPolicy = projected_lm<Chain>,
          typename LimitsPolicy = typename InnerPolicy::limits_type>
class restart_wrapper
{
    using scalar_type_impl = typename Chain::scalar_type;
    static_assert(std::is_floating_point_v<scalar_type_impl>,
        "restart_wrapper requires a floating-point Scalar type");

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

    restart_wrapper()
        : m_best_q(detail::poison_joint_position<scalar_type, joints>())
    {
    }

    explicit restart_wrapper(InnerPolicy inner)
        : m_inner(std::move(inner))
        , m_best_q(detail::poison_joint_position<scalar_type, joints>())
    {
    }

    explicit restart_wrapper(const options& opts)
        : m_options(opts)
        , m_best_q(detail::poison_joint_position<scalar_type, joints>())
    {
    }

    restart_wrapper(const options& opts, InnerPolicy inner)
        : m_inner(std::move(inner))
        , m_options(opts)
        , m_best_q(detail::poison_joint_position<scalar_type, joints>())
    {
    }

    void setup(
        const Chain& chain,
        const se3<scalar_type>& target,
        const position_type& q0,
        const convergence_criteria<scalar_type>& criteria)
    {
        m_chain = std::cref(chain);
        m_target = target;
        m_criteria = criteria;
        m_weight.reset();
        m_seed_gen.emplace(chain);
        m_restart_count = 0;
        m_total_iterations = 0;
        m_best_lambda = scalar_type(0);
        m_best_error = std::numeric_limits<scalar_type>::max();
        m_best_q_error = std::numeric_limits<scalar_type>::max();
        m_best_feasible = false;
        m_best_valid = false;
        m_aborted = false;
        m_best_q = position_type::Zero(chain.num_joints());
        m_setup_joints = chain.num_joints();

        // Assigned on both outcomes, not only on failure: a wrapper is reusable,
        // and a latch written only when the check fails would report the stale
        // rejection for every later solve.
        auto held = cartan::detail::validate_solve_inputs(chain, target, q0);
        m_precondition = held ? ik_status::running : held.error();
        if (!held)
        {
            return;
        }

        m_inner.setup(chain, target, q0, criteria);
    }

    void setup(
        const Chain& chain,
        const se3<scalar_type>& target,
        const position_type& q0,
        const convergence_criteria<scalar_type>& criteria,
        const error_weight<scalar_type>& weight)
    {
        m_chain = std::cref(chain);
        m_target = target;
        m_criteria = criteria;
        m_weight = weight;
        m_seed_gen.emplace(chain);
        m_restart_count = 0;
        m_total_iterations = 0;
        m_best_lambda = scalar_type(0);
        m_best_error = std::numeric_limits<scalar_type>::max();
        m_best_q_error = std::numeric_limits<scalar_type>::max();
        m_best_feasible = false;
        m_best_valid = false;
        m_aborted = false;
        m_best_q = position_type::Zero(chain.num_joints());
        m_setup_joints = chain.num_joints();

        auto held = cartan::detail::validate_solve_inputs(chain, target, q0);
        m_precondition = held ? ik_status::running : held.error();
        if (!held)
        {
            return;
        }

        if constexpr (requires { m_inner.setup(chain, target, q0, criteria, weight); })
        {
            m_inner.setup(chain, target, q0, criteria, weight);
        }
        else
        {
            m_inner.setup(chain, target, q0, criteria);
        }
    }

    /// Deleted rvalue overloads: the wrapper borrows the chain for its whole
    /// lifetime without owning it, so a temporary bound here would dangle the
    /// moment setup() returns. A plain `const Chain&` parameter would silently
    /// bind an rvalue, so the temporary is rejected at the call boundary
    /// instead -- effectively `setup(Chain&&) = delete` (borrowed-not-owned:
    /// the chain must outlive the wrapper).
    void setup(
        Chain&&,
        const se3<scalar_type>&,
        const position_type&,
        const convergence_criteria<scalar_type>&) = delete;

    void setup(
        Chain&&,
        const se3<scalar_type>&,
        const position_type&,
        const convergence_criteria<scalar_type>&,
        const error_weight<scalar_type>&) = delete;

    step_result<scalar_type> step(const Chain& chain, int N)
    {
        // The restart path re-seeds from a generator built on the setup-time
        // chain and hands the result to the step-time one, so the two must be
        // the same chain before either is touched.
        m_precondition = cartan::detail::chain_bound_status(
            m_precondition, m_setup_joints, chain);

        // A wrapper-level rejection never reached the inner solver, so there is
        // nothing to step and nothing to restart from.
        if (cartan::detail::is_setup_failure(m_precondition))
        {
            return {m_precondition, {0, error_norm()}};
        }

        if (m_aborted)
        {
            return {ik_status::stalled, {0, m_inner.error_norm()}};
        }

        auto inner_result = m_inner.step(chain, N);
        m_total_iterations += inner_result.metrics.units_consumed;

        if (inner_result.status == ik_status::converged
            || inner_result.status == ik_status::running)
        {
            return inner_result;
        }

        // A bad argument is not a failed attempt: it is returned before the
        // best-candidate tracking so it neither counts against the restart
        // budget nor pollutes the best-so-far, and no fresh seed can repair it.
        if (cartan::detail::is_setup_failure(inner_result.status))
        {
            return inner_result;
        }

        // Inner terminal-but-not-converged: try a fresh Halton seed restart.
        // Restart events charge zero additional units beyond what the failing
        // inner attempt already billed.
        track_best_lambda();

        if (m_restart_count >= m_options.max_restarts)
        {
            return inner_result;
        }

        auto q_new = (*m_seed_gen)(m_restart_count);

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

        apply_warm_start_lambda();

        ++m_restart_count;
        return {ik_status::running, {inner_result.metrics.units_consumed, m_inner.error_norm()}};
    }

    // All three read through the same gate: a refused setup ran no attempt, so
    // the inner solver still holds the previous solve's answer and reporting it
    // would present that solve as this one's. They are part of the solve_policy
    // concept, so a caller reading them instead of step()'s status must not see
    // a converged solve with an error norm of zero.
    bool converged() const
    {
        return !cartan::detail::is_setup_failure(m_precondition) && m_inner.converged();
    }

    // On a converged solve the live inner iterate is the answer; on a terminal
    // solve report the feasibility-first best-so-far captured across restarts
    // rather than the last (discarded) attempt.
    position_type solution() const
    {
        if (cartan::detail::is_setup_failure(m_precondition))
        {
            return m_best_q;
        }
        if (m_inner.converged() || !m_best_valid)
        {
            return m_inner.solution();
        }
        return m_best_q;
    }

    scalar_type error_norm() const
    {
        if (cartan::detail::is_setup_failure(m_precondition))
        {
            return std::numeric_limits<scalar_type>::max();
        }
        if (m_inner.converged() || !m_best_valid)
        {
            return m_inner.error_norm();
        }
        return m_best_q_error;
    }
    int iterations() const { return m_total_iterations; }
    int restarts() const { return m_restart_count; }

    void abort()
    {
        m_aborted = true;
        m_inner.abort();
    }

private:
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
        update_best_q(current_error);
    }

    // Feasibility-first best-so-far retention, independent of the warm-start
    // lambda bookkeeping above: a feasible attempt always beats an infeasible
    // one, and among equally-feasible attempts the lower pose error wins.
    void update_best_q(scalar_type current_error)
    {
        // Rank the pose-equivalence class: canonicalize a COPY of the inner
        // solution into limits and retain the in-limits representative. The inner
        // iterate is not mutated (this wrapper only reads its solution), and the
        // pose is invariant under the wrap so current_error is unchanged. When no
        // chain is bound the class cannot be evaluated, so the raw solution is
        // retained as before.
        position_type q_canon = m_inner.solution();
        bool feasible = m_chain.has_value()
            && cartan::detail::feasible_after_canonicalization(
                q_canon, m_chain->get(),
                cartan::detail::default_feasibility_tol<scalar_type>());
        bool better = !m_best_valid
            || (feasible && !m_best_feasible)
            || (feasible == m_best_feasible && current_error < m_best_q_error);
        if (better)
        {
            m_best_q = q_canon;
            m_best_q_error = current_error;
            m_best_feasible = feasible;
            m_best_valid = true;
        }
    }

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
    std::optional<std::reference_wrapper<const Chain>> m_chain{};
    se3<scalar_type> m_target{se3<scalar_type>::identity()};
    convergence_criteria<scalar_type> m_criteria{};
    std::optional<error_weight<scalar_type>> m_weight{};
    std::optional<halton_seed_generator<Chain>> m_seed_gen{};
    ik_status m_precondition{ik_status::not_initialized};
    int m_restart_count{};
    int m_setup_joints{-1};
    int m_total_iterations{};
    scalar_type m_best_lambda{};
    scalar_type m_best_error{std::numeric_limits<scalar_type>::max()};
    position_type m_best_q;
    scalar_type m_best_q_error{std::numeric_limits<scalar_type>::max()};
    bool m_best_feasible{false};
    bool m_best_valid{false};
    bool m_aborted{false};
};

// Deduction guide: deduce everything from inner policy
template <typename InnerPolicy>
restart_wrapper(InnerPolicy) -> restart_wrapper<
    typename InnerPolicy::chain_type,
    InnerPolicy,
    typename InnerPolicy::limits_type>;

// Deduction guide: deduce from options + inner policy
template <typename InnerPolicy>
restart_wrapper(typename restart_wrapper<
    typename InnerPolicy::chain_type,
    InnerPolicy,
    typename InnerPolicy::limits_type>::options,
    InnerPolicy) -> restart_wrapper<
    typename InnerPolicy::chain_type,
    InnerPolicy,
    typename InnerPolicy::limits_type>;

}

#endif
