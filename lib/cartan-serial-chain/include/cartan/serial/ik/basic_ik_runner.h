#ifndef HPP_GUARD_CARTAN_SERIAL_IK_BASIC_IK_RUNNER_H
#define HPP_GUARD_CARTAN_SERIAL_IK_BASIC_IK_RUNNER_H

/// Variadic policy-based IK solver with cooperative interleaved racing.
///
/// basic_ik_runner<Policies...> provides direct solve for a single policy and
/// cooperative round-robin racing for two or more policies. The single-policy
/// path produces identical code to a non-variadic solver (zero overhead).
/// The multi-policy path absorbs the round-robin tick logic formerly in
/// racing_scheduler: each step() performs one round-robin across all active
/// policies, parking converged ones and selecting the best result based on
/// the configured objective.
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 6.2, p. 227-233.

#include "cartan/serial/ik/ik_result.h"
#include "cartan/serial/ik/ik_status.h"
#include "cartan/serial/ik/detail/convergence.h"
#include "cartan/serial/ik/detail/feasible_set.h"
#include "cartan/serial/ik/concepts/solve_concept.h"
#include "cartan/serial/ik/detail/setup_validation.h"
#include "cartan/serial/ik/detail/selection_metrics.h"
#include "cartan/serial/ik/solver/detail/halton_seed_generator.h"

#include "cartan/lie/se3.h"
#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/chain_concept.h"

#include <array>
#include <cmath>
#include <tuple>
#include <limits>
#include <optional>
#include "cartan/expected.h"
#include <concepts>
#include <algorithm>
#include <functional>
#include <type_traits>

namespace cartan
{

/// Check that all policies in a variadic pack agree on chain_type, scalar_type, and joints.
template <typename First, typename... Rest>
consteval bool all_policies_agree()
{
    if constexpr (sizeof...(Rest) == 0)
        return true;
    else
        return ((std::same_as<typename First::scalar_type, typename Rest::scalar_type>
                 && std::same_as<typename First::chain_type, typename Rest::chain_type>
                 && First::joints == Rest::joints) && ...);
}

/// Detects whether a policy exposes `termination_reason()` for fine-grained
/// failure diagnostics. Policies that opt out are reported as
/// `ik_termination_reason::unknown`.
template <typename Policy>
concept reports_termination_reason = requires(const Policy& p)
{
    { p.termination_reason() } -> std::convertible_to<ik_termination_reason>;
};

template <typename Policy>
constexpr ik_termination_reason policy_termination_reason(const Policy& p) noexcept
{
    if constexpr (reports_termination_reason<Policy>)
        return p.termination_reason();
    else
        return ik_termination_reason::unknown;
}

/// Variadic policy-based IK solver with cooperative interleaved racing.
///
/// When instantiated with a single policy, behaves identically to a
/// non-variadic solver with zero overhead. With two or more policies,
/// provides cooperative round-robin racing with parking and objective-based
/// result selection.
///
/// Thread safety: Different solver instances may safely operate concurrently
/// on the same const chain. A single solver instance must not be
/// used from multiple threads without synchronization.
template <typename... Policies>
    requires (sizeof...(Policies) >= 1) && (solve_policy<Policies> && ...)
class basic_ik_runner
{
    using first_policy = std::tuple_element_t<0, std::tuple<Policies...>>;

    static_assert(std::is_floating_point_v<typename first_policy::scalar_type>,
        "basic_ik_runner requires a floating-point Scalar type");
    static_assert(all_policies_agree<Policies...>(),
        "All policies must agree on chain_type, scalar_type, and joints");

public:
    using chain_type = typename first_policy::chain_type;
    using scalar_type = typename first_policy::scalar_type;
    static constexpr int joints = first_policy::joints;

    using position_type = typename joint_state<scalar_type, joints>::position_type;

    basic_ik_runner()
        : m_best_q(detail::poison_joint_position<scalar_type, joints>())
        , m_reference_q(detail::poison_joint_position<scalar_type, joints>())
    {
    }

    explicit basic_ik_runner(Policies... policies)
        : m_policies(std::move(policies)...)
        , m_best_q(detail::poison_joint_position<scalar_type, joints>())
        , m_reference_q(detail::poison_joint_position<scalar_type, joints>())
    {
    }

    void setup(
        const chain_type& chain,
        const se3<scalar_type>& target,
        const position_type& q0,
        const convergence_criteria<scalar_type>& criteria,
        const solver_options<scalar_type>& options = {})
    {
        m_chain = std::cref(chain);
        m_target = target;
        m_criteria = criteria;
        m_objective = options.objective;
        m_length = options.characteristic_length;
        m_restart_index = static_cast<int>(options.halton_seed);
        reset_state(chain);

        if (auto refused = refuse_setup(chain, target, q0); refused)
        {
            m_status = *refused;
            return;
        }

        m_best_q = q0;
        m_reference_q = q0;
        m_seed_gen.emplace(chain, q0);
        std::get<0>(m_policies).setup(chain, target, q0, criteria);

        if constexpr (sizeof...(Policies) > 1)
        {
            setup_remaining_policies(chain, target, criteria, options.halton_seed,
                std::make_index_sequence<sizeof...(Policies) - 1>{});
        }
    }

    /// Deleted rvalue overload: the runner borrows the chain without owning it,
    /// so binding a temporary here would leave a dangling reference the moment
    /// setup() returns. Rejecting the temporary at the call boundary is the
    /// only safe guard -- a `const chain_type&` parameter alone would silently
    /// bind (and then dangle) an rvalue.
    void setup(
        chain_type&&,
        const se3<scalar_type>&,
        const position_type&,
        const convergence_criteria<scalar_type>&,
        const solver_options<scalar_type>& = {}) = delete;

    /// Precondition: setup() must be called before step(). Invoked beforehand,
    /// step() has no chain to drive and returns ik_status::not_initialized --
    /// the same answer status() and solve() give -- rather than dereferencing
    /// an empty borrow.
    ik_status step()
    {
        return charge_and_step(1).status;
    }

    ik_status step_n(int n)
    {
        for (int i = 0; i < n; ++i)
        {
            if (charge_and_step(1).status != ik_status::running)
            {
                break;
            }
        }
        return m_status;
    }

    cartan::expected<ik_result<scalar_type, joints>, ik_error<scalar_type, joints>> solve()
    {
        if (!m_chain)
        {
            return cartan::unexpected(
                ik_error<scalar_type, joints>{.reason = ik_failure::not_initialized});
        }

        while (charge_and_step(m_criteria.max_total_work_units).status == ik_status::running)
        {
        }

        return build_result();
    }

    bool converged() const { return m_status == ik_status::converged; }

    /// A runner whose setup was refused, or never called, has no policy behind
    /// it that measured anything; a policy's own accumulator reads zero there,
    /// which is the value a converged solve reports.
    scalar_type error_norm() const
    {
        if (cartan::detail::is_setup_failure(m_status))
        {
            return std::numeric_limits<scalar_type>::quiet_NaN();
        }
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

    int iterations() const { return m_total_iterations; }
    const position_type& current_q() const { return m_best_q; }
    ik_status status() const { return m_status; }

    /// A refused setup is not a state a caller can abort out of: the arguments
    /// are still the ones setup() rejected, so clearing the latch here would
    /// let the next solve() run against a policy that was never configured.
    void abort()
    {
        if (cartan::detail::is_setup_failure(m_status))
        {
            return;
        }
        abort_all(std::index_sequence_for<Policies...>{});
        m_status = ik_status::aborted;
    }

private:
    struct parked_result
    {
        position_type q;
        scalar_type error_norm{};
        std::optional<scalar_type> metric{};
        int iterations{};
        bool converged{false};
        ik_termination_reason termination_reason{ik_termination_reason::unknown};
        feasible_set solved_feasible_set{feasible_set::declared};
    };

    /// Recorded per policy rather than per runner: the pack may mix a policy
    /// that substitutes a finite interval for a non-finite bound with one that
    /// box-projects against the declared bounds, and those two race over
    /// different feasible sets.
    template <typename Policy>
    feasible_set policy_feasible_set() const
    {
        return cartan::detail::feasible_set_solved<Policy>(m_chain->get());
    }

    /// Everything setup() establishes before a policy is touched, so a rejected
    /// call leaves neither a half-configured policy nor the previous solve's
    /// convergence flag, iterate and counters readable.
    std::optional<ik_status> refuse_setup(
        const chain_type& chain,
        const se3<scalar_type>& target,
        const position_type& q0) const
    {
        if (auto held = cartan::detail::validate_solve_inputs(chain, target, q0); !held)
        {
            return held.error();
        }
        if (chain.num_joints() == 0 && !reaches_target_without_joints(chain, target, q0))
        {
            return ik_status::unreachable;
        }
        return cartan::detail::selection_admissibility(m_objective, chain, m_length);
    }

    /// A chain with no joints has a one-point workspace, so a target away from
    /// that point is certifiably infeasible rather than merely not found. It is
    /// the only place on the iterative path where infeasibility is provable: an
    /// iterative solver that has joints to move cannot conclude it from a failed
    /// search. The test is the solver's own convergence test, so exactly the
    /// targets a solve would have accepted are the ones not refused here.
    bool reaches_target_without_joints(
        const chain_type& chain,
        const se3<scalar_type>& target,
        const position_type& q0) const
    {
        auto fk = forward_kinematics_unchecked(chain, q0);
        auto twist = (fk.end_effector.inverse() * target).log();
        return cartan::detail::is_converged_unweighted(twist, m_criteria);
    }

    /// The single owner of the work-unit accumulator and of the total-budget
    /// comparison. step(), step_n() and solve() reach the policies only through
    /// here, so the three agree on the budget, on the terminal status they
    /// latch and on the metrics they report by construction rather than by
    /// three implementations happening to coincide.
    ///
    /// min_units_per_step contract: a step that returns ik_status::running must
    /// bill at least one work unit. A `{running, units=0}` return signals a
    /// solver that cannot make forward progress (e.g. the inner policy
    /// converged at the entry q with no work and was restarted there under a
    /// non-speed objective), and is terminated on the same rule as an exhausted
    /// budget.
    step_result<scalar_type> charge_and_step(int requested)
    {
        if (m_status != ik_status::running)
        {
            return {m_status, {0, error_norm()}};
        }

        const int remaining = m_criteria.max_total_work_units - m_total_iterations;
        auto stepped = dispatch_step(std::min(requested, remaining));
        m_total_iterations += stepped.metrics.units_consumed;

        const bool spent = stepped.metrics.units_consumed == 0
            || m_total_iterations >= m_criteria.max_total_work_units;

        if (spent && m_status == ik_status::running)
        {
            m_status = m_found_convergence ? ik_status::converged : ik_status::iteration_limit;
        }

        return {m_status, stepped.metrics};
    }

    step_result<scalar_type> dispatch_step(int units)
    {
        if constexpr (sizeof...(Policies) == 1)
        {
            return step_single_metrics(units);
        }
        else
        {
            return step_multi();
        }
    }

    void reset_state(const chain_type& chain)
    {
        m_status = ik_status::running;
        m_best_metric = std::nullopt;
        m_total_iterations = 0;
        m_found_convergence = false;
        m_best_q = detail::poison_joint_position<scalar_type, joints>(chain.num_joints());
        m_early_stop = false;
        m_parked = {};
        m_results = {};
        m_best_solver_index = -1;
    }

    step_result<scalar_type> step_single_metrics(int N)
    {
        auto inner = std::get<0>(m_policies).step(m_chain->get(), N);
        auto policy_status = inner.status;
        position_type q = std::get<0>(m_policies).solution();

        if (policy_status == ik_status::converged)
        {
            m_found_convergence = true;

            if (m_objective == ik_objective::speed)
            {
                m_status = ik_status::converged;
                m_best_q = q;
                return inner;
            }

            update_best(q);
            std::get<0>(m_policies).setup(m_chain->get(), m_target, restart_seed(), m_criteria);
            return {ik_status::running, inner.metrics};
        }

        if (policy_status == ik_status::diverged ||
            policy_status == ik_status::stalled ||
            policy_status == ik_status::iteration_limit)
        {
            m_status = m_found_convergence ? ik_status::converged : policy_status;
            return {m_status, inner.metrics};
        }

        return inner;
    }

    /// One round-robin across the active policies. A tick is atomic, so it bills
    /// the sum of the units its policies consumed and the caller's budget is
    /// honored at tick granularity: the last tick can carry the accumulator
    /// past the budget by at most one unit per still-active policy.
    step_result<scalar_type> step_multi()
        requires (sizeof...(Policies) > 1)
    {
        bool any_running = false;
        int units = 0;

        [&]<std::size_t... Is>(std::index_sequence<Is...>)
        {
            (tick_policy<Is>(any_running, units), ...);
        }(std::index_sequence_for<Policies...>{});

        if (!any_running)
        {
            m_status = m_found_convergence ? ik_status::converged : ik_status::iteration_limit;
        }

        return {m_status, {units, error_norm()}};
    }

    template <std::size_t I>
    void tick_policy(bool& any_running, int& units)
        requires (sizeof...(Policies) > 1)
    {
        if (m_early_stop || m_parked[I])
            return;

        auto& policy = std::get<I>(m_policies);
        auto ticked = policy.step(m_chain->get(), 1);
        auto status = ticked.status;
        units += ticked.metrics.units_consumed;

        if (status == ik_status::converged)
        {
            m_parked[I] = true;
            m_results[I].emplace(parked_result{
                .q = policy.solution(),
                .error_norm = policy.error_norm(),
                .metric = candidate_metric(policy.solution(), policy.error_norm()),
                .iterations = policy.iterations(),
                .converged = true,
                .termination_reason = policy_termination_reason(policy),
                .solved_feasible_set = policy_feasible_set<std::tuple_element_t<I, std::tuple<Policies...>>>()
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
                .converged = false,
                .termination_reason = policy_termination_reason(policy),
                .solved_feasible_set = policy_feasible_set<std::tuple_element_t<I, std::tuple<Policies...>>>()
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
            || s == ik_status::joint_limit_hit
            || s == ik_status::aborted
            || s == ik_status::not_initialized
            || s == ik_status::dimension_mismatch
            || s == ik_status::non_finite_input;
    }

    void park_all()
        requires (sizeof...(Policies) > 1)
    {
        m_parked.fill(true);
    }

    template <std::size_t... Is>
    void setup_remaining_policies(
        const chain_type& chain,
        const se3<scalar_type>& target,
        const convergence_criteria<scalar_type>& criteria,
        unsigned int halton_seed_offset,
        std::index_sequence<Is...>)
    {
        (setup_policy<Is + 1>(chain, target, criteria, halton_seed_offset, Is), ...);
    }

    template <std::size_t I>
    void setup_policy(
        const chain_type& chain,
        const se3<scalar_type>& target,
        const convergence_criteria<scalar_type>& criteria,
        unsigned int halton_seed_offset,
        std::size_t policy_index)
    {
        auto seed = (*m_seed_gen)(static_cast<int>(policy_index + halton_seed_offset));
        std::get<I>(m_policies).setup(chain, target, seed, criteria);
    }

    std::optional<scalar_type> candidate_metric(const position_type& q, scalar_type error_norm) const
    {
        return cartan::detail::selection_metric(
            m_objective, m_chain->get(), q, m_reference_q, error_norm, m_length);
    }

    void update_best(const position_type& q)
    {
        auto metric = candidate_metric(q, std::get<0>(m_policies).error_norm());

        if (cartan::detail::improves_on(m_objective, metric, m_best_metric))
        {
            m_best_metric = metric;
            m_best_q = q;
        }
    }

    /// A continuation under a non-speed objective needs a start the policy has
    /// not already converged at: re-seeding it at the candidate it just returned
    /// converges again for zero work, which the budget guard reads as a spent
    /// solve, so a single start would be reported as a multistart.
    position_type restart_seed()
    {
        return (*m_seed_gen)(m_restart_index++);
    }

    cartan::expected<ik_result<scalar_type, joints>, ik_error<scalar_type, joints>> build_result()
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
                result.selection_metric = m_best_metric;
                result.selection_objective = m_objective;
                result.solved_feasible_set = policy_feasible_set<first_policy>();
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
    cartan::expected<ik_result<scalar_type, joints>, ik_error<scalar_type, joints>>
    select_best_result(std::index_sequence<Is...>)
        requires (sizeof...(Policies) > 1)
    {
        if (m_objective == ik_objective::speed && m_best_solver_index >= 0)
        {
            return make_result_from_parked(m_best_solver_index);
        }

        int best_index = -1;
        std::optional<scalar_type> best_metric{};

        auto check = [&]<std::size_t I>()
        {
            if (m_results[I] && m_results[I]->converged
                && cartan::detail::improves_on(m_objective, m_results[I]->metric, best_metric))
            {
                best_metric = m_results[I]->metric;
                best_index = static_cast<int>(I);
            }
        };
        (check.template operator()<Is>(), ...);

        if (best_index >= 0)
        {
            return make_result_from_parked(best_index);
        }

        return build_error();
    }

    cartan::expected<ik_result<scalar_type, joints>, ik_error<scalar_type, joints>>
    make_result_from_parked(int index)
        requires (sizeof...(Policies) > 1)
    {
        ik_result<scalar_type, joints> result;
        auto& pr = *m_results[static_cast<std::size_t>(index)];
        result.solution = joint_state<scalar_type, joints>::from_position(pr.q);
        result.iterations = m_total_iterations;
        result.final_error_norm = pr.error_norm;
        result.solver_index = index;
        result.selection_metric = pr.metric;
        result.selection_objective = m_objective;
        result.solved_feasible_set = pr.solved_feasible_set;
        return result;
    }

    cartan::expected<ik_result<scalar_type, joints>, ik_error<scalar_type, joints>> build_error()
    {
        ik_error<scalar_type, joints> err;
        err.termination_reason = ik_termination_reason::unknown;

        // A refused setup ran no iteration, so neither field was measured and
        // both keep the poison: reading an iterate off a policy would hand back
        // the previous solve's, a zero vector reads as the home configuration,
        // and the largest representable residual reads as a measured distance.
        // The iterate carries the chain's joint count so its size stays
        // readable, which the type's own default cannot supply.
        if (cartan::detail::is_setup_failure(m_status))
        {
            err.last_q = m_best_q;
            err.reason = cartan::detail::failure_reason_for(m_status);
            return cartan::unexpected(err);
        }

        if constexpr (sizeof...(Policies) == 1)
        {
            err.last_q = std::get<0>(m_policies).solution();
            err.last_error_norm = std::get<0>(m_policies).error_norm();
            err.termination_reason = policy_termination_reason(std::get<0>(m_policies));
        }
        else
        {
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
                const auto& best = *m_results[static_cast<std::size_t>(best_fail)];
                err.last_q = best.q;
                err.last_error_norm = best.error_norm;
                err.termination_reason = best.termination_reason;
            }
            else
            {
                report_live_iterate(err, std::index_sequence_for<Policies...>{});
            }
        }

        err.reason = cartan::detail::failure_reason_for(m_status);
        return cartan::unexpected(err);
    }

    /// The lowest-residual live iterate, for a budget exhausted with every
    /// policy still running: no policy parked, so those iterates are the only
    /// configurations the solve measured. Reporting the seed instead would name
    /// the configuration the solve started at as the one it failed at.
    template <std::size_t... Is>
    void report_live_iterate(ik_error<scalar_type, joints>& err, std::index_sequence<Is...>) const
        requires (sizeof...(Policies) > 1)
    {
        scalar_type best = std::numeric_limits<scalar_type>::max();
        auto check = [&]<std::size_t I>()
        {
            const auto& policy = std::get<I>(m_policies);
            if (policy.error_norm() < best)
            {
                best = policy.error_norm();
                err.last_q = policy.solution();
                err.last_error_norm = best;
                err.termination_reason = policy_termination_reason(policy);
            }
        };
        (check.template operator()<Is>(), ...);
    }

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

    std::tuple<Policies...> m_policies{};
    std::optional<std::reference_wrapper<const chain_type>> m_chain{};
    se3<scalar_type> m_target{se3<scalar_type>::identity()};
    convergence_criteria<scalar_type> m_criteria{};
    position_type m_best_q;
    position_type m_reference_q;
    std::optional<scalar_type> m_best_metric{};
    ik_objective m_objective{ik_objective::speed};
    ik_status m_status{ik_status::not_initialized};
    scalar_type m_length{1};
    int m_total_iterations{};
    int m_restart_index{};
    bool m_found_convergence{false};

    std::array<bool, sizeof...(Policies)> m_parked{};
    std::array<std::optional<parked_result>, sizeof...(Policies)> m_results{};
    std::optional<halton_seed_generator<chain_type>> m_seed_gen{};
    int m_best_solver_index{-1};
    bool m_early_stop{false};
};

// CTAD deduction guide: deduce Policies... from constructor arguments
template <typename... Policies>
basic_ik_runner(Policies...) -> basic_ik_runner<Policies...>;

}

#endif
