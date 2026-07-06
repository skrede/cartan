#ifndef HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_ARGMIN_PROJECTED_GN_H
#define HPP_GUARD_CARTAN_SERIAL_IK_SOLVER_ARGMIN_PROJECTED_GN_H

/// argmin-backed projected Gauss-Newton IK solve policy with active-set bounds.
///
/// Wraps argmin's projected_gn_policy for bound-constrained nonlinear
/// least-squares IK. Identifies free/active variables at each iteration
/// and solves the reduced damped Gauss-Newton system on the free subspace.
/// Two globalization modes: Nielsen (1999) adaptive lambda damping (default)
/// and dogleg trust-region interpolation.
///
/// Reference: Nocedal & Wright Sections 10.2-10.3 (Gauss-Newton), 16.6
///            (active-set identification), 16.7 (projected search).
///            Nielsen, H. B. (1999) "Damping Parameter in Marquardt's Method".

#include "cartan/serial/ik/ik_status.h"
#include "cartan/serial/ik/policy/error_weight.h"
#include "cartan/serial/ik/policy/limits_policy.h"
#include "cartan/serial/ik/concepts/solve_concept.h"
#include "cartan/serial/ik/detail/convergence.h"
#include "cartan/serial/ik/detail/stall_detection.h"
#include "cartan/serial/ik/detail/limit_enforcement.h"
#include "cartan/serial/ik/detail/argmin_least_squares_problem.h"

#include "cartan/lie/se3.h"
#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/chain_concept.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include <argmin/solver/options.h>
#include <argmin/solver/convergence.h>
#include <argmin/solver/basic_solver.h>
#include <argmin/solver/projected_gn_policy.h>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <random>
#include <vector>
#include <optional>
#include <algorithm>
#include <type_traits>
#include <cstdint>

namespace cartan::ik
{

/// argmin-backed projected Gauss-Newton solve policy for bound-constrained IK.
///
/// Uses active-set identification (N&W 16.6) with projection onto joint
/// bounds (N&W 16.7). The Gauss-Newton Hessian approximation H = J^T J
/// is formed from the body Jacobian, and the reduced damped system on
/// free variables is solved each step. Globalization via Nielsen (1999)
/// lambda damping or dogleg trust-region.
///
/// Exposes the cartan `solve_policy` interface plus a cold-restart
/// perturb-retry loop matching argmin_slsqp: on terminal inner status
/// with pose tolerance unmet, the solver is reset with a perturbed
/// initial point (up to `max_restarts` times).
template <chain Chain,
          typename LimitsPolicy = clamp_limits,
          typename Convergence = argmin::default_convergence>
class argmin_projected_gn
{
public:
    using chain_type = Chain;
    using scalar_type = typename Chain::scalar_type;
    static constexpr int joints = Chain::joints;
    using limits_type = LimitsPolicy;

    using position_type = typename joint_state<scalar_type, joints>::position_type;

    static_assert(std::is_floating_point_v<scalar_type>,
                  "argmin_projected_gn requires a floating-point Scalar type");

    struct options
    {
        double initial_lambda{0.0};
        double tau{1e-3};
        double diagonal_min_clamp{1e-8};
        double lambda_min{1e-20};
        double lambda_max{1e20};

        bool use_dogleg{false};
        std::optional<double> trust_region_radius{};
        std::optional<double> trust_region_expand_threshold{};
        std::optional<double> trust_region_shrink_threshold{};

        scalar_type stall_threshold{scalar_type(1e-10)};
        scalar_type divergence_factor{scalar_type(10)};
        int stall_window{5};

        int max_restarts{10};
        scalar_type restart_scale{scalar_type(0.5)};
        std::optional<unsigned> rng_seed{};

        double gradient_threshold{1e-12};
        double objective_threshold{1e-14};
        double step_threshold{1e-14};
    };

    argmin_projected_gn() = default;

    explicit argmin_projected_gn(const options& opts)
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
        if (m_options.rng_seed)
            m_rng.seed(*m_options.rng_seed);

        auto fk = forward_kinematics(chain, q0);
        auto V_b = (target.inverse() * fk.end_effector).log();
        m_initial_error = V_b.norm();
        m_q = q0;

        m_problem.emplace(chain, target);

        int n = chain.num_joints();
        Eigen::VectorXd x0(n);
        for (int i = 0; i < n; ++i)
        {
            x0[i] = static_cast<double>(q0[i]);
        }

        build_argmin_opts(m_nab_opts);

        typename argmin::projected_gn_policy::options_type policy_opts{};
        policy_opts.initial_lambda = m_options.initial_lambda;
        policy_opts.tau = m_options.tau;
        policy_opts.diagonal_min_clamp = m_options.diagonal_min_clamp;
        policy_opts.lambda_min = m_options.lambda_min;
        policy_opts.lambda_max = m_options.lambda_max;
        policy_opts.use_dogleg = m_options.use_dogleg;
        policy_opts.trust_region_radius = m_options.trust_region_radius;
        policy_opts.trust_region_expand_threshold = m_options.trust_region_expand_threshold;
        policy_opts.trust_region_shrink_threshold = m_options.trust_region_shrink_threshold;

        m_solver.emplace(*m_problem, x0, m_nab_opts, policy_opts);
    }

    step_result<scalar_type> step(const Chain& chain, int N)
    {
        int units = 0;
        m_chain = &chain;
        while (units < N && m_status == ik_status::running)
        {
            // One algorithmic-work unit = one argmin-internal outer GN iteration.
            auto prev_iter = m_solver->state().iteration;
            auto result = m_solver->step_n(
                static_cast<std::uint32_t>(1), m_nab_opts);
            auto inner_units = static_cast<int>(m_solver->state().iteration - prev_iter);
            if (inner_units <= 0)
            {
                inner_units = 1;
            }
            m_iterations += inner_units;
            m_attempt_iterations += inner_units;
            units += inner_units;

            sync_solution_from_solver();

            auto fk = forward_kinematics(chain, m_q);
            auto V_b = (m_target.inverse() * fk.end_effector).log();
            m_error_norm = V_b.norm();

            if (cartan::detail::is_converged_unweighted(V_b, m_criteria))
            {
                m_status = ik_status::converged;
                m_termination_reason = ik_termination_reason::converged;
                break;
            }

            // Three reasons to end an attempt and consider a cold-restart:
            // inner-solver terminal status, per-attempt cap exhausted, or
            // cartan-side stall/divergence. All three signal "this attempt
            // cannot make further progress." With restart budget remaining,
            // perturb and re-seed the inner.
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
                    // Cold-restart event itself charges 0 additional units.
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
    [[nodiscard]] const position_type& solution() const { return m_q; }
    [[nodiscard]] scalar_type error_norm() const { return m_error_norm; }
    [[nodiscard]] int iterations() const { return m_iterations; }
    [[nodiscard]] ik_status status() const { return m_status; }
    [[nodiscard]] ik_termination_reason termination_reason() const { return m_termination_reason; }
    [[nodiscard]] int restart_count() const { return m_restart_count; }

    void abort()
    {
        m_status = ik_status::stalled;
        m_termination_reason = ik_termination_reason::solver_aborted;
    }

private:
    using argmin_solver = argmin::basic_solver<
        argmin::projected_gn_policy, joints, cartan::detail::argmin_ik_least_squares_problem<Chain>>;
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

    const Chain* m_chain{nullptr};
    se3<scalar_type> m_target{se3<scalar_type>::identity()};
    convergence_criteria<scalar_type> m_criteria{};
    options m_options{};
    position_type m_q{};
    cartan::detail::error_ring<scalar_type> m_error_history;
    scalar_type m_initial_error{};
    scalar_type m_error_norm{std::numeric_limits<scalar_type>::max()};
    int m_iterations{};
    int m_attempt_iterations{};
    ik_status m_status{ik_status::running};
    ik_termination_reason m_termination_reason{ik_termination_reason::unknown};
    std::optional<cartan::detail::argmin_ik_least_squares_problem<Chain>> m_problem;
    std::optional<argmin_solver> m_solver;
    argmin_opts_type m_nab_opts{};
    int m_restart_count{};
    std::mt19937 m_rng{0};
};

}

#endif
