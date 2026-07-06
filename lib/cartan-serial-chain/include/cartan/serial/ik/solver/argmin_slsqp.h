#ifndef HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_ARGMIN_SLSQP_H
#define HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_ARGMIN_SLSQP_H

/// argmin-backed SLSQP gradient-based IK solve policy with box constraints.
///
/// Wraps argmin's kraft_slsqp_policy for constrained IK, using joint
/// limits as box constraints and the analytical gradient from
/// ik_se3_objective (SE(3) log Jacobian). Always available -- argmin
/// is a required dependency of cartan::kinematics.
///
/// Reference: Kraft 1988, N&W Ch. 18 (SQP methods).

#include "cartan/serial/ik/ik_status.h"
#include "cartan/serial/ik/policy/error_weight.h"
#include "cartan/serial/ik/policy/limits_policy.h"
#include "cartan/serial/ik/concepts/solve_concept.h"
#include "cartan/serial/ik/detail/convergence.h"
#include "cartan/serial/ik/detail/argmin_problem.h"
#include "cartan/serial/ik/detail/stall_detection.h"
#include "cartan/serial/ik/detail/limit_enforcement.h"

#include "cartan/lie/se3.h"
#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/chain_concept.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include <argmin/solver/options.h>
#include <argmin/solver/convergence.h>
#include <argmin/solver/basic_solver.h>
#include <argmin/solver/kraft_slsqp_policy.h>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <array>
#include <optional>
#include <random>
#include <type_traits>
#include <cstdint>

namespace cartan
{

/// argmin-backed SLSQP solve policy for constrained IK with box constraints.
///
/// Uses Kraft's Sequential Least Squares Programming algorithm via argmin,
/// with analytical gradient through the SE(3) log Jacobian. Each step()
/// call runs a budget of argmin iterations, allowing cooperative scheduling
/// with other policies in basic_ik_runner.
///
/// This is the default (unprefixed) SLSQP policy. The NLopt-backed variant
/// is available as cartan::nlopt_slsqp behind CARTAN_HAS_NLOPT.
///
/// The Convergence template parameter lets consumers opt out of argmin's
/// default four-criterion convergence policy in favor of alternatives like
/// `argmin::slsqp_compatible_convergence` which mirrors NLopt's SLSQP
/// stop rules (ftol_rel + xtol_rel + stall, no gradient-norm check). See
/// `argmin/solver/convergence.h`. The relative thresholds are configured
/// via `options::objective_threshold_rel` and `options::step_threshold_rel`
/// when the alternative policy is used.
///
/// Note: argmin's `kraft_slsqp_policy` once carried a per-Mode NTTP that
/// distinguished `sqp_mode::accurate` and `sqp_mode::fast`. Upstream
/// (commit 18ab9463 in the FetchContent-pinned argmin checkout) removed
/// the strictly-worse `fast` mode, and the surviving per-knob defaults
/// match the former `accurate` values. The Mode parameter is therefore
/// gone from this wrapper as well; the `argmin_slsqp_fast` alias survives
/// as a documentation-only synonym (no behavioral difference vs the
/// unprefixed `argmin_slsqp`).
template <chain Chain,
          typename LimitsPolicy = clamp_limits,
          typename Convergence = argmin::default_convergence>
class argmin_slsqp
{
public:
    using chain_type = Chain;
    using scalar_type = typename Chain::scalar_type;
    static constexpr int joints = Chain::joints;
    using limits_type = LimitsPolicy;

    using position_type = typename joint_state<scalar_type, joints>::position_type;

    static_assert(std::is_floating_point_v<scalar_type>, "argmin_slsqp requires a floating-point Scalar type");

    struct options
    {
        scalar_type stall_threshold{scalar_type(1e-10)};
        scalar_type divergence_factor{scalar_type(10)};
        int stall_window{5};
        int max_restarts{10};
        scalar_type restart_scale{scalar_type(0.5)};
        std::optional<unsigned> rng_seed{};

        /// Armijo sufficient-decrease parameter c1 forwarded to
        /// kraft_slsqp_policy's embedded merit line search. Argmin's default
        /// is 1e-4; looser values (1e-3) accept more trial steps and cut
        /// backtracks on expensive-objective problems like IK where each
        /// phi(alpha) call costs a full FK pass. Tightening below 1e-5 is
        /// generally a no-op because the Armijo condition on a well-scaled
        /// merit function dominates before c1 matters.
        double line_search_c1{1e-4};

        /// Absolute thresholds consumed by the default argmin convergence
        /// policy (gradient / objective / step + stall). Ignored when
        /// Convergence is `argmin::slsqp_compatible_convergence`.
        double gradient_threshold{1e-12};
        double objective_threshold{1e-14};
        double step_threshold{1e-14};

        /// Relative thresholds consumed by `slsqp_compatible_convergence`
        /// (objective_tolerance_rel + step_tolerance_rel + stall). Default
        /// values mirror NLopt SLSQP's `ftol_rel` / `xtol_rel` = 1e-10.
        /// Ignored when Convergence is `argmin::default_convergence`.
        double objective_threshold_rel{1e-10};
        double step_threshold_rel{1e-10};

        /// Stride that gates the post-step active-set Lagrange multiplier
        /// re-estimation call on argmin's line-search SQP policies. `k = 1`
        /// re-estimates the multipliers on every SQP iteration (the
        /// pre-Phase-41 behavior on argmin's N&W lineage); `k > 1` reuses
        /// stale multipliers on (k-1)/k of steps and only refreshes the
        /// active-set estimate every k-th step. Reference: Bertsekas 1996
        /// §4.2 (stale-multiplier reuse).
        ///
        /// For `argmin::kraft_slsqp_policy` (the underlying policy of this
        /// wrapper across both Mode values), this field is wired for API
        /// uniformity but the KKT-leg uses the QP-native multipliers
        /// (`qp_res.lambda` / `qp_res.mu`) on every step regardless of k —
        /// argmin documents this as a behavioral no-op. The wiring exists
        /// so cartan benchmarks can directly empirical-verify the no-op
        /// prediction on the cartan IK pose-batch corpus, and so callers
        /// can experiment with k > 1 should kraft_slsqp's KKT-leg behavior
        /// change in a future argmin revision.
        ///
        /// Default tracks argmin's per-policy constexpr:
        /// `kraft_slsqp_policy<joints>::default_multiplier_reest_every_k`
        /// (currently `1`; the per-Mode `5` default disappeared with the
        /// removal of `sqp_mode::fast` from kraft's NTTP slot upstream).
        std::size_t multiplier_reest_every_k{
            argmin::kraft_slsqp_policy<joints>::default_multiplier_reest_every_k};
    };

    argmin_slsqp() = default;

    explicit argmin_slsqp(const options& opts)
        : m_options{opts}
    {}

    void setup(
        const Chain& chain,
        const se3<scalar_type>& target,
        const position_type& q0,
        const convergence_criteria<scalar_type>& criteria)
    {
        m_chain = &chain;
        m_target = target;
        m_criteria = criteria;
        m_iterations = 0;
        m_attempt_iterations = 0;
        m_error_norm = std::numeric_limits<scalar_type>::max();
        m_status = ik_status::running;
        m_termination_reason = ik_termination_reason::unknown;
        m_error_history.clear();
        m_restart_count = 0;
        m_best_q_error = std::numeric_limits<scalar_type>::max();
        m_best_feasible = false;
        m_best_valid = false;
        if (m_options.rng_seed)
            m_rng.seed(*m_options.rng_seed);

        auto fk = forward_kinematics(chain, q0);
        auto V_b = (target.inverse() * fk.end_effector).log();
        m_initial_error = V_b.norm();

        m_problem.emplace(chain, target, m_weight);

        int n = chain.num_joints();
        Eigen::VectorXd x0(n);
        for (int i = 0; i < n; ++i)
        {
            x0[i] = static_cast<double>(q0[i]);
        }

        build_argmin_opts(m_nab_opts);

        typename argmin::kraft_slsqp_policy<joints>::options_type policy_opts{};
        policy_opts.line_search.c1 = m_options.line_search_c1;
        policy_opts.multiplier_reest_every_k = m_options.multiplier_reest_every_k;

        m_solver.emplace(*m_problem, x0, m_nab_opts, policy_opts);
    }

    step_result<scalar_type> step(const Chain& chain, int N)
    {
        int units = 0;
        m_chain = &chain;
        while (units < N && m_status == ik_status::running)
        {
            // One algorithmic-work unit = one argmin-internal SLSQP outer iteration.
            auto prev_iter = m_solver->state().iteration;
            auto result = m_solver->step_n(
                static_cast<std::uint32_t>(1), m_nab_opts);
            auto inner_units = static_cast<int>(m_solver->state().iteration - prev_iter);
            if (inner_units <= 0)
            {
                // Inner solver did no work. Force a restart (free) so we can
                // still make progress on cold-restart-bound problems; if no
                // restart budget is left, fall through to the terminal branch.
                inner_units = 1;
            }
            m_iterations += inner_units;
            m_attempt_iterations += inner_units;
            units += inner_units;

            sync_solution_from_solver();

            auto fk = forward_kinematics(chain, m_q);
            auto V_b = (m_target.inverse() * fk.end_effector).log();
            m_error_norm = V_b.norm();
            update_best(chain);

            if (cartan::detail::is_converged_unweighted(V_b, m_criteria))
            {
                m_status = ik_status::converged;
                m_termination_reason = ik_termination_reason::converged;
                break;
            }

            // Three reasons to end an attempt and consider a cold-restart:
            //   1. inner argmin reported a terminal status (ftol/xtol/etc),
            //   2. per-attempt cap exhausted on the cartan-side counter,
            //   3. cartan-side stall/divergence detector fired on m_error_norm.
            // All three signal "this attempt cannot make further progress."
            // With restart budget remaining, perturb and re-seed the inner.
            // Pre-Plan-01 path (1) fired first because step_n(50) gave the
            // inner enough context to detect its own stall before the cartan
            // detector accumulated enough error history. Post-Plan-01 step_n(1)
            // starves the inner of that context; stall now consistently
            // wins the race on unreachable targets, so it must participate
            // in the same restart-or-give-up predicate.
            auto stall_result = cartan::detail::check_stall_divergence(
                m_error_history, m_error_norm, m_initial_error,
                m_options.stall_window, m_options.stall_threshold,
                m_options.divergence_factor);
            const bool attempt_capped =
                m_attempt_iterations >= m_criteria.max_iterations_per_attempt;
            const bool inner_terminal = is_inner_terminal(result.status);
            const bool cartan_stall = (stall_result != ik_status::running);
            if (inner_terminal || attempt_capped || cartan_stall)
            {
                if (m_restart_count < m_options.max_restarts)
                {
                    ++m_restart_count;
                    auto q_perturbed = perturb_solution(m_q, *m_chain);

                    m_error_history.clear();
                    auto fk_new = forward_kinematics(*m_chain, q_perturbed);
                    auto V_b_new = (m_target.inverse() * fk_new.end_effector).log();
                    m_initial_error = V_b_new.norm();

                    int n = m_chain->num_joints();
                    Eigen::VectorXd x0(n);
                    for (int i = 0; i < n; ++i)
                        x0[i] = static_cast<double>(q_perturbed[i]);
                    m_solver->reset_clear(x0);

                    build_argmin_opts(m_nab_opts);

                    m_attempt_iterations = 0;
                    m_q = q_perturbed;
                    // Cold-restart event itself charges 0 additional units beyond
                    // the inner attempt's already-billed inner_units.
                    continue;
                }

                if (cartan_stall)
                {
                    m_status = stall_result;
                    m_termination_reason = (stall_result == ik_status::diverged)
                        ? ik_termination_reason::divergence_detected
                        : ik_termination_reason::stall_detected;
                }
                else if (inner_terminal)
                {
                    m_status = ik_status::stalled;
                    m_termination_reason = map_argmin_status(result.status);
                }
                else
                {
                    m_status = ik_status::iteration_limit;
                    m_termination_reason = ik_termination_reason::iteration_limit;
                }
                break;
            }

            cartan::detail::enforce_limits<LimitsPolicy>(m_q, chain);
        }
        return {m_status, {units, m_error_norm}};
    }

    [[nodiscard]] bool converged() const { return m_status == ik_status::converged; }
    // Report the feasibility-first best-so-far iterate across restarts, not the
    // last perturbed attempt, so a terminal solve still surfaces its best result.
    [[nodiscard]] const position_type& solution() const { return m_best_valid ? m_best_q : m_q; }
    [[nodiscard]] scalar_type error_norm() const { return m_best_valid ? m_best_q_error : m_error_norm; }
    [[nodiscard]] int iterations() const { return m_iterations; }
    [[nodiscard]] ik_status status() const { return m_status; }
    [[nodiscard]] ik_termination_reason termination_reason() const { return m_termination_reason; }
    [[nodiscard]] int restart_count() const { return m_restart_count; }

    /// Cumulative number of phi(alpha) line-search calls the underlying
    /// kraft_slsqp_policy has made since setup(). Zero if the solver has
    /// not been set up. Exposed so benchmarks can measure average backtracks
    /// per solver step without reaching into argmin internals. Gracefully
    /// returns 0 on argmin versions predating the counter member (shipped
    /// in argmin commit 95ffe8d).
    [[nodiscard]] std::uint64_t line_search_calls() const
    {
        if (!m_solver) return 0;
        using state_t = std::remove_cvref_t<decltype(m_solver->state())>;
        if constexpr (requires(const state_t& s) { s.line_search_calls; })
            return m_solver->state().line_search_calls;
        else
            return 0;
    }

    /// Cumulative number of kraft_slsqp_policy::step() invocations since
    /// setup(), read from the inner argmin solver's state. This is the
    /// inner step count that pairs with line_search_calls to compute
    /// average backtracks per argmin step. Zero if the solver has not
    /// been set up.
    [[nodiscard]] std::uint32_t argmin_iterations() const
    {
        if (!m_solver) return 0;
        using state_t = std::remove_cvref_t<decltype(m_solver->state())>;
        if constexpr (requires(const state_t& s) { s.iteration; })
            return m_solver->state().iteration;
        else
            return 0;
    }

    /// Per-criterion convergence telemetry from the last solve, read from
    /// the argmin opts owned by this argmin_slsqp instance. Each entry is
    /// `std::optional<argmin::solver_status>`: nullopt means the criterion
    /// did not fire on the terminating iteration, non-nullopt means it did
    /// (the value is the argmin solver_status the criterion would have
    /// reported). All criteria are evaluated on every iteration — the
    /// first-in-order non-nullopt entry is the terminator that actually
    /// drove the solver's exit, but later criteria also populate the
    /// array so consumers can see which additional criteria would have
    /// fired. Requires argmin HEAD >= 21a0acf.
    ///
    /// Note: reads from the member `m_nab_opts.convergence.last_check_results_`
    /// rather than from `m_solver->convergence()` because basic_solver's
    /// explicit-opts `step_n(budget, opts)` overload (which argmin_slsqp
    /// uses to forward a typed convergence policy per call) writes the
    /// per-iteration telemetry into the caller's opts, not into the solver's
    /// stored_convergence_. The stored_convergence_ back-copy is only done
    /// by the no-opts `step_n(budget)` overload.
    [[nodiscard]] auto last_check_results() const
    {
        if constexpr (requires { m_nab_opts.convergence.last_check_results(); })
            return m_nab_opts.convergence.last_check_results();
        else
            return std::array<std::optional<argmin::solver_status>, 4>{};
    }
    void abort()
    {
        m_status = ik_status::stalled;
        m_termination_reason = ik_termination_reason::solver_aborted;
    }

private:
    using argmin_solver = argmin::basic_solver<
        argmin::kraft_slsqp_policy<joints>, joints, cartan::detail::argmin_ik_problem<Chain>>;
    using argmin_opts_type = argmin::solver_options<Convergence>;

    position_type perturb_solution(const position_type& q, const Chain& chain)
    {
        int n = chain.num_joints();
        const auto& limits = chain.limits();
        std::uniform_real_distribution<double> dist(-1.0, 1.0);

        position_type q_new;
        if constexpr (joints == dynamic)
            q_new.resize(n);

        for (int i = 0; i < n; ++i)
        {
            auto idx = static_cast<std::size_t>(i);
            const auto raw_range = limits[idx].position_max - limits[idx].position_min;
            const auto range = cartan::detail::finite_range_or(raw_range,
                cartan::detail::k_unbounded_angular_range_v<scalar_type>);
            auto perturbation = static_cast<scalar_type>(dist(m_rng)) * m_options.restart_scale * range;
            q_new[i] = std::clamp(
                q[i] + perturbation,
                limits[idx].position_min,
                limits[idx].position_max);
        }
        return q_new;
    }

    void build_argmin_opts(argmin_opts_type& opts) const
    {
        // Per-attempt cap; argmin's setup-time cap is generous because the
        // per-call budget is now threaded through step_n(N) from the runner.
        opts.max_iterations = static_cast<std::uint32_t>(m_criteria.max_iterations_per_attempt);
        if constexpr (std::is_same_v<Convergence, argmin::default_convergence>)
        {
            opts.set_gradient_threshold(m_options.gradient_threshold);
            opts.set_objective_threshold(m_options.objective_threshold);
            opts.set_step_threshold(m_options.step_threshold);
        }
        else
        {
            opts.set_objective_threshold_rel(m_options.objective_threshold_rel);
            opts.set_step_threshold_rel(m_options.step_threshold_rel);
        }
    }

    static constexpr bool is_inner_terminal(argmin::solver_status s) noexcept
    {
        switch (s)
        {
            case argmin::solver_status::converged:
            case argmin::solver_status::ftol_reached:
            case argmin::solver_status::xtol_reached:
            case argmin::solver_status::stalled:
            case argmin::solver_status::objective_stalled:
            case argmin::solver_status::roundoff_limited:
            case argmin::solver_status::aborted:
            case argmin::solver_status::diverged:
            case argmin::solver_status::max_iterations:
                return true;
            default:
                return false;
        }
    }

    static constexpr ik_termination_reason map_argmin_status(argmin::solver_status s) noexcept
    {
        switch (s)
        {
            case argmin::solver_status::converged:         return ik_termination_reason::solver_converged_pose_missed;
            case argmin::solver_status::ftol_reached:      return ik_termination_reason::solver_ftol_reached;
            case argmin::solver_status::xtol_reached:      return ik_termination_reason::solver_xtol_reached;
            case argmin::solver_status::stalled:           return ik_termination_reason::solver_stalled;
            case argmin::solver_status::objective_stalled: return ik_termination_reason::solver_objective_stalled;
            case argmin::solver_status::roundoff_limited:  return ik_termination_reason::solver_roundoff_limited;
            case argmin::solver_status::aborted:           return ik_termination_reason::solver_aborted;
            case argmin::solver_status::budget_exhausted:  return ik_termination_reason::solver_budget_exhausted;
            case argmin::solver_status::max_iterations:    return ik_termination_reason::solver_max_iterations;
            case argmin::solver_status::diverged:          return ik_termination_reason::solver_diverged;
            default:                                        return ik_termination_reason::unknown;
        }
    }

    void sync_solution_from_solver()
    {
        const auto& x = m_solver->state().x;
        int n = m_chain->num_joints();
        if constexpr (joints == dynamic)
        {
            m_q.resize(n);
        }
        for (int i = 0; i < n; ++i)
        {
            m_q[i] = static_cast<scalar_type>(x[i]);
        }
    }

    // Retain the feasibility-first best-so-far iterate: a feasible iterate always
    // beats an infeasible one, and among equally-feasible iterates the lower pose
    // error wins. Consulted every synced iteration so the best survives restarts.
    void update_best(const Chain& chain)
    {
        // Rank the pose-equivalence class, not the raw iterate: canonicalize a
        // COPY into limits (the synced working iterate m_q is left untouched --
        // wrapping it would be a clamp) and store the in-limits representative as
        // best. The pose is invariant under the wrap, so m_error_norm is unchanged.
        position_type q_canon = m_q;
        bool feasible = cartan::detail::feasible_after_canonicalization(
            q_canon, chain, cartan::detail::default_feasibility_tol<scalar_type>());
        bool better = !m_best_valid
            || (feasible && !m_best_feasible)
            || (feasible == m_best_feasible && m_error_norm < m_best_q_error);
        if (better)
        {
            m_best_q = q_canon;
            m_best_q_error = m_error_norm;
            m_best_feasible = feasible;
            m_best_valid = true;
        }
    }

    const Chain* m_chain{nullptr};
    se3<scalar_type> m_target{se3<scalar_type>::identity()};
    convergence_criteria<scalar_type> m_criteria{};
    error_weight<scalar_type> m_weight{};
    options m_options{};
    position_type m_q{};
    position_type m_best_q{};
    scalar_type m_best_q_error{std::numeric_limits<scalar_type>::max()};
    bool m_best_feasible{false};
    bool m_best_valid{false};
    cartan::detail::error_ring<scalar_type> m_error_history;
    scalar_type m_initial_error{};
    scalar_type m_error_norm{std::numeric_limits<scalar_type>::max()};
    int m_iterations{};
    int m_attempt_iterations{};
    ik_status m_status{ik_status::running};
    ik_termination_reason m_termination_reason{ik_termination_reason::unknown};
    std::optional<cartan::detail::argmin_ik_problem<Chain>> m_problem;
    std::optional<argmin_solver> m_solver;
    argmin_opts_type m_nab_opts{};
    int m_restart_count{};
    // Default to a deterministic seed so per-pose benchmark captures are
    // reproducible across binary runs. Callers wanting nondeterministic
    // restarts should set options::rng_seed to a varying value at construction.
    std::mt19937 m_rng{0};
};

/// NLopt-compatible convergence variant of argmin_slsqp.
///
/// Drops argmin's gradient_tolerance_criterion in favor of NLopt's
/// ftol_rel + xtol_rel + stall stop rules (`argmin::slsqp_compatible_convergence`).
/// Intended for IK workloads where the outer pose tolerance is the
/// ground truth and the inner solver just needs to make efficient
/// progress toward it — not for unconstrained minimization where a
/// gradient-norm stationarity certificate is load-bearing.
template <chain Chain, typename LimitsPolicy = clamp_limits>
using argmin_slsqp_nlopt_compat =
    argmin_slsqp<Chain, LimitsPolicy, argmin::slsqp_compatible_convergence>;

/// Former fast-mode variant of argmin_slsqp.
///
/// Upstream (argmin commit 18ab9463 in the FetchContent-pinned checkout)
/// removed the strictly-worse `sqp_mode::fast` branch from
/// `kraft_slsqp_policy`'s NTTP slot; the surviving per-knob defaults
/// match the former `accurate` values. The alias survives as a
/// documentation-only synonym so existing benchmark / example call sites
/// can resolve the name; there is no behavioral difference between
/// `argmin_slsqp_fast<Chain>` and `argmin_slsqp<Chain>` after this change.
template <chain Chain, typename LimitsPolicy = clamp_limits>
using argmin_slsqp_fast = argmin_slsqp<Chain, LimitsPolicy>;

}

#endif
