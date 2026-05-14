#ifndef HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_NLOPT_COMMON_H
#define HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_NLOPT_COMMON_H

/// Shared NLopt wrapper helpers for NLopt-backed IK solve policies.
///
/// Extracts conversion, bounds setup, optimization dispatch, result mapping,
/// convergence checking, perturbation, and limit enforcement utilities shared
/// between cartan::ik::nlopt_slsqp and cartan::ik::nlopt_bobyqa.
///
/// All functions live in cartan::detail and are guarded by CARTAN_HAS_NLOPT.
///
/// Reference: Decision D-17 (extract shared NLopt boilerplate).

#ifdef CARTAN_HAS_NLOPT

#include "cartan/types.h"

#include "cartan/serial/ik/ik_status.h"
#include "cartan/serial/ik/policy/limits_policy.h"
#include "cartan/serial/ik/detail/limit_enforcement.h"

#include "cartan/lie/se3.h"
#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/kinematic_chain.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include <nlopt.hpp>

#include <cmath>
#include <limits>
#include <random>
#include <vector>
#include <algorithm>

namespace cartan
{
namespace detail
{

/// Convert Eigen vector to std::vector<double> for NLopt.
template <typename Scalar, int N>
std::vector<double> eigen_to_stdvec(
    const typename joint_state<Scalar, N>::position_type& v)
{
    int n = static_cast<int>(v.size());
    std::vector<double> out(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i)
    {
        out[static_cast<std::size_t>(i)] = static_cast<double>(v(i));
    }
    return out;
}

/// Convert std::vector<double> back to Eigen position_type.
template <typename Scalar, int N>
typename joint_state<Scalar, N>::position_type stdvec_to_eigen(
    const std::vector<double>& x)
{
    int n = static_cast<int>(x.size());
    typename joint_state<Scalar, N>::position_type q;
    if constexpr (N == dynamic)
    {
        q.resize(n);
    }
    for (int i = 0; i < n; ++i)
    {
        q(i) = static_cast<Scalar>(x[static_cast<std::size_t>(i)]);
    }
    return q;
}

/// Set NLopt box bounds from chain joint limits.
template <typename Scalar, int N>
void set_nlopt_bounds(
    nlopt::opt& opt,
    const kinematic_chain<Scalar, N>& chain)
{
    int n = chain.num_joints();
    std::vector<double> lb(static_cast<std::size_t>(n));
    std::vector<double> ub(static_cast<std::size_t>(n));
    const auto& limits = chain.limits();
    for (int i = 0; i < n; ++i)
    {
        auto idx = static_cast<std::size_t>(i);
        lb[idx] = static_cast<double>(limits[idx].position_min);
        ub[idx] = static_cast<double>(limits[idx].position_max);
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
}

/// Run NLopt optimization with exception handling.
///
/// Returns the nlopt::result on success. On roundoff_limited, returns
/// nlopt::ROUNDOFF_LIMITED. On other exceptions, returns nlopt::FAILURE.
inline nlopt::result run_nlopt_optimize(
    nlopt::opt& opt,
    std::vector<double>& x,
    double& opt_f)
{
    try
    {
        return opt.optimize(x, opt_f);
    }
    catch (const nlopt::roundoff_limited&)
    {
        return nlopt::ROUNDOFF_LIMITED;
    }
    catch (const std::exception&)
    {
        return nlopt::FAILURE;
    }
}

/// Map NLopt result code and convergence state to ik_status.
///
/// Handles the common result-mapping switch shared by both NLopt policies.
inline ik_status map_nlopt_result(
    nlopt::result result,
    bool converged,
    bool error_stalled,
    bool can_restart)
{
    switch (result)
    {
        case nlopt::SUCCESS:
        case nlopt::FTOL_REACHED:
        case nlopt::XTOL_REACHED:
            if (converged) return ik_status::converged;
            return can_restart ? ik_status::running : ik_status::stalled;

        case nlopt::MAXEVAL_REACHED:
        case nlopt::MAXTIME_REACHED:
            if (converged) return ik_status::converged;
            if (error_stalled)
            {
                return can_restart ? ik_status::running : ik_status::stalled;
            }
            return ik_status::running;

        case nlopt::ROUNDOFF_LIMITED:
            if (converged) return ik_status::converged;
            return ik_status::stalled;

        case nlopt::FAILURE:
            return ik_status::diverged;

        default:
            if (converged) return ik_status::converged;
            return ik_status::diverged;
    }
}

/// Check whether the result code indicates a restart is needed.
///
/// Returns true for local-minimum results (SUCCESS/FTOL/XTOL without
/// convergence, or MAXEVAL with stalled error).
inline bool needs_restart(
    nlopt::result result,
    bool converged,
    bool error_stalled)
{
    switch (result)
    {
        case nlopt::SUCCESS:
        case nlopt::FTOL_REACHED:
        case nlopt::XTOL_REACHED:
            return !converged;

        case nlopt::MAXEVAL_REACHED:
        case nlopt::MAXTIME_REACHED:
            return !converged && error_stalled;

        default:
            return false;
    }
}

/// Perturb the solution within joint limits to escape local minima.
template <typename Scalar, int N>
void perturb_nlopt_solution(
    std::vector<double>& x,
    const kinematic_chain<Scalar, N>& chain,
    Scalar restart_scale,
    std::mt19937& rng)
{
    int n = static_cast<int>(x.size());
    const auto& limits = chain.limits();
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    for (int i = 0; i < n; ++i)
    {
        auto idx = static_cast<std::size_t>(i);
        double range = static_cast<double>(
            limits[idx].position_max - limits[idx].position_min);
        double perturbation = dist(rng) *
            static_cast<double>(restart_scale) * range;
        x[idx] = std::clamp(
            x[idx] + perturbation,
            static_cast<double>(limits[idx].position_min),
            static_cast<double>(limits[idx].position_max));
    }
}

/// Reset an NLopt optimizer after perturbation with fresh bounds and budget.
template <typename Scalar, int N, typename ObjectiveFunc>
void reset_nlopt_optimizer(
    nlopt::opt& opt,
    nlopt::algorithm algorithm,
    const kinematic_chain<Scalar, N>& chain,
    ObjectiveFunc objective_func,
    void* data,
    double xtol_rel,
    int budget_per_step,
    int& eval_count)
{
    int n = chain.num_joints();
    opt = nlopt::opt(algorithm, static_cast<unsigned>(n));
    set_nlopt_bounds<Scalar, N>(opt, chain);
    opt.set_min_objective(objective_func, data);
    opt.set_xtol_rel(xtol_rel);
    eval_count = budget_per_step;
    opt.set_maxeval(eval_count);
}

/// Compute body-frame error norm from current q.
///
/// Uses T_target^{-1} * T_fk convention (SLSQP style) when invert_order
/// is false, or T_fk^{-1} * T_target (BOBYQA style) when invert_order
/// is true. Both conventions yield the same norm for convergence checking.
template <typename Scalar, int N>
Scalar compute_body_error_norm(
    const kinematic_chain<Scalar, N>& chain,
    const se3<Scalar>& target,
    const std::vector<double>& x)
{
    auto q = stdvec_to_eigen<Scalar, N>(x);
    auto fk = forward_kinematics(chain, q);
    auto V_b = (target.inverse() * fk.end_effector).log();
    return V_b.norm();
}

/// Check convergence against IK criteria (angular + linear tolerances).
template <typename Scalar, int N>
bool check_nlopt_convergence(
    const kinematic_chain<Scalar, N>& chain,
    const se3<Scalar>& target,
    const convergence_criteria<Scalar>& criteria,
    const std::vector<double>& x)
{
    auto q = stdvec_to_eigen<Scalar, N>(x);
    auto fk = forward_kinematics(chain, q);
    auto V_b = (target.inverse() * fk.end_effector).log();

    Scalar angular_err = V_b.template head<3>().norm();
    Scalar linear_err = V_b.template tail<3>().norm();

    return angular_err < criteria.orientation_tol
        && linear_err < criteria.position_tol;
}

/// Enforce joint limits via LimitsPolicy and sync back to std::vector.
template <typename LimitsPolicy, typename Scalar, int N>
void enforce_and_sync_limits(
    std::vector<double>& x,
    const kinematic_chain<Scalar, N>& chain)
{
    auto q = stdvec_to_eigen<Scalar, N>(x);
    enforce_limits<LimitsPolicy>(q, chain);
    int n = static_cast<int>(x.size());
    for (int i = 0; i < n; ++i)
    {
        x[static_cast<std::size_t>(i)] = static_cast<double>(q(i));
    }
}

}
}

#endif

#endif
