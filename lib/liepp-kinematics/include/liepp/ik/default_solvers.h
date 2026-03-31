#ifndef HPP_GUARD_LIEPP_IK_DEFAULT_SOLVERS_H
#define HPP_GUARD_LIEPP_IK_DEFAULT_SOLVERS_H

/// @file default_solvers.h
/// @brief Factory functions, builders, and type aliases for common IK solver configurations.
///
/// Provides three layers of solver construction:
///   1. Type aliases (speed_solver, convergence_solver, default_solver) for direct use.
///   2. Preset factories (make_speed_solver, make_convergence_solver, make_default_solver)
///      returning builders with .build() as the materialization point.
///   3. Composable builder (make_solver) for custom policy compositions via .policy().build().
///
/// Reference: Decisions D-07 (solver presets), D-08 (racing default), D-10 (builder pattern),
///            D-11 (type aliases), D-12 (factory namespace).

#include "liepp/ik/limits_policy.h"
#include "liepp/ik/basic_ik_solver.h"
#include "liepp/ik/ik_solve_policy.h"
#include "liepp/ik/lbfgsb_solve_policy.h"
#include "liepp/ik/restart_solve_policy.h"
#include "liepp/ik/projected_lm_solve_policy.h"

#include <tuple>

namespace liepp
{

// ---------------------------------------------------------------------------
// Type aliases (D-11)
// ---------------------------------------------------------------------------

/// Speed-optimized: restart-wrapped projected LM (fast per-iteration, multi-start).
template <typename Scalar = double, int N = dynamic>
using speed_solver = restart_solve_policy<Scalar, N,
    projected_lm_solve_policy<Scalar, N, no_limits>, no_limits>;

/// Convergence-optimized: restart-wrapped L-BFGS-B (robust convergence, multi-start).
template <typename Scalar = double, int N = dynamic>
using convergence_solver = restart_solve_policy<Scalar, N,
    lbfgsb_solve_policy<Scalar, N, no_limits>, no_limits>;

/// Default: races speed_solver against convergence_solver via variadic basic_ik_solver.
template <typename Scalar = double, int N = dynamic>
using default_solver = basic_ik_solver<speed_solver<Scalar, N>, convergence_solver<Scalar, N>>;

// ---------------------------------------------------------------------------
// Composable builder (D-10)
// ---------------------------------------------------------------------------

/// Type-accumulating solver builder for custom policy compositions.
///
/// Usage: `auto solver = make_solver<double, 7>().policy(p1).policy(p2).build();`
/// `.build()` is the materialization point — everything before it is configuration.
template <typename Scalar, int N, typename... Policies>
class solver_builder
{
public:
    explicit solver_builder(std::tuple<Policies...> policies)
        : m_policies(std::move(policies))
    {
    }

    /// Add a policy to the solver composition.
    template <typename Policy>
        requires ik_solve_policy<Policy>
    auto policy(Policy p) &&
    {
        return solver_builder<Scalar, N, Policies..., Policy>{
            std::tuple_cat(std::move(m_policies), std::make_tuple(std::move(p)))
        };
    }

    /// Build the configured solver.
    auto build() &&
    {
        return std::apply([](auto&&... ps) {
            return basic_ik_solver{std::forward<decltype(ps)>(ps)...};
        }, std::move(m_policies));
    }

private:
    std::tuple<Policies...> m_policies;
};

// ---------------------------------------------------------------------------
// Preset builders (D-10)
// ---------------------------------------------------------------------------

/// Preset solver builder — wraps a fixed single-policy configuration.
///
/// Exposes parameter tuning (restart count, etc.) but NOT policy composition.
/// Call `.build()` to materialize the solver. This enables future extensions
/// like `.from_config(cfg).build()`.
template <typename Scalar, int N, typename Policy>
class preset_solver_builder
{
public:
    explicit preset_solver_builder(Policy policy)
        : m_policy(std::move(policy))
    {
    }

    /// Build the configured solver.
    auto build() &&
    {
        return basic_ik_solver{std::move(m_policy)};
    }

private:
    Policy m_policy;
};

/// Preset builder for two-policy racing configurations.
template <typename Scalar, int N, typename Policy1, typename Policy2>
class preset_racing_builder
{
public:
    explicit preset_racing_builder(Policy1 p1, Policy2 p2)
        : m_policy1(std::move(p1))
        , m_policy2(std::move(p2))
    {
    }

    /// Build the configured solver.
    auto build() &&
    {
        return basic_ik_solver{std::move(m_policy1), std::move(m_policy2)};
    }

private:
    Policy1 m_policy1;
    Policy2 m_policy2;
};

// ---------------------------------------------------------------------------
// Preset factory functions (D-10, D-12)
// ---------------------------------------------------------------------------

/// Create a speed-optimized solver builder (restart-wrapped projected LM).
/// Call `.build()` to materialize the solver.
template <typename Scalar = double, int N = dynamic>
auto make_speed_solver()
{
    return preset_solver_builder<Scalar, N, speed_solver<Scalar, N>>{
        speed_solver<Scalar, N>{}
    };
}

/// Create a convergence-optimized solver builder (restart-wrapped L-BFGS-B).
/// Call `.build()` to materialize the solver.
template <typename Scalar = double, int N = dynamic>
auto make_convergence_solver()
{
    return preset_solver_builder<Scalar, N, convergence_solver<Scalar, N>>{
        convergence_solver<Scalar, N>{}
    };
}

/// Create the default solver builder (races speed + convergence policies).
/// Call `.build()` to materialize the solver.
template <typename Scalar = double, int N = dynamic>
auto make_default_solver()
{
    return preset_racing_builder<Scalar, N,
        speed_solver<Scalar, N>, convergence_solver<Scalar, N>>{
        speed_solver<Scalar, N>{}, convergence_solver<Scalar, N>{}
    };
}

/// Create a composable solver builder. Chain .policy(p) calls, finish with .build().
template <typename Scalar = double, int N = dynamic>
auto make_solver()
{
    return solver_builder<Scalar, N>{std::tuple<>{}};
}

}

#endif
