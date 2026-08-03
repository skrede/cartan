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
#include "cartan/serial/ik/concepts/solve_concept.h"
#include "cartan/serial/ik/detail/setup_validation.h"
#include "cartan/serial/ik/solver/detail/halton_seed_generator.h"

#include "cartan/lie/se3.h"
#include "cartan/serial/fk/jacobian.h"
#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/chain_concept.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include <Eigen/SVD>

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
    {
    }

    explicit basic_ik_runner(Policies... policies)
        : m_policies(std::move(policies)...)
        , m_best_q(detail::poison_joint_position<scalar_type, joints>())
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
        reset_state(chain);

        // Reset first, validate second, and latch before any policy is touched:
        // a rejected seed must leave neither a half-configured policy nor the
        // previous solve's convergence flag, iterate and counters readable.
        if (auto held = cartan::detail::validate_solve_inputs(chain, target, q0); !held)
        {
            m_status = held.error();
            return;
        }

        m_best_q = q0;
        std::get<0>(m_policies).setup(chain, target, q0, criteria);

        if constexpr (sizeof...(Policies) > 1)
        {
            m_seed_gen.emplace(chain, q0);
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

    scalar_type error_norm() const
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
        m_status = ik_status::running;
    }

private:
    struct parked_result
    {
        position_type q;
        scalar_type error_norm{};
        int iterations{};
        bool converged{false};
        ik_termination_reason termination_reason{ik_termination_reason::unknown};
    };

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
        m_best_error = std::numeric_limits<scalar_type>::max();
        m_best_manipulability = scalar_type(0);
        m_best_isotropy = scalar_type(0);
        m_total_iterations = 0;
        m_found_convergence = false;
        m_best_q = position_type::Zero(chain.num_joints());
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
            std::get<0>(m_policies).setup(m_chain->get(), m_target, q, m_criteria);
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
                .iterations = policy.iterations(),
                .converged = true,
                .termination_reason = policy_termination_reason(policy)
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
                .termination_reason = policy_termination_reason(policy)
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
                auto fk = forward_kinematics_unchecked(m_chain->get(), q);
                auto J_b = body_jacobian_unchecked(m_chain->get(), fk);
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
                auto fk = forward_kinematics_unchecked(m_chain->get(), q);
                auto J_b = body_jacobian_unchecked(m_chain->get(), fk);
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
        return result;
    }

    cartan::expected<ik_result<scalar_type, joints>, ik_error<scalar_type, joints>> build_error()
    {
        ik_error<scalar_type, joints> err;
        err.near_singular = false;
        err.condition_number = scalar_type(0);
        err.termination_reason = ik_termination_reason::unknown;

        // A refused setup ran no iteration, so there is no last iterate to
        // report; reading one off a policy would hand back the previous solve's.
        if (cartan::detail::is_setup_failure(m_status))
        {
            err.last_q = m_best_q;
            err.last_error_norm = std::numeric_limits<scalar_type>::max();
            err.reason = cartan::detail::setup_failure_reason(m_status);
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

        return cartan::unexpected(err);
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
    scalar_type m_best_manipulability{};
    scalar_type m_best_isotropy{};
    scalar_type m_best_error{};
    ik_objective m_objective{ik_objective::speed};
    ik_status m_status{ik_status::not_initialized};
    int m_total_iterations{};
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
