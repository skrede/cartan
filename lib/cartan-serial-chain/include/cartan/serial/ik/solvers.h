#ifndef HPP_GUARD_CARTAN_SERIAL_IK_SOLVERS_H
#define HPP_GUARD_CARTAN_SERIAL_IK_SOLVERS_H

/// @file solvers.h
/// @brief Factory functions, builders, and type aliases for common IK solver configurations.
///
/// Provides three layers of solver construction:
///   1. Type aliases (speed_ik_runner, robust_ik_runner, dual_ik_runner) for direct use.
///   2. Preset factories (make_speed_ik_runner, make_robust_ik_runner, make_dual_ik_runner)
///      returning builders with .build() as the materialization point.
///   3. Composable builder (make_solver) for custom policy compositions via .policy().build().

#include "cartan/serial/ik/policy/limits_policy.h"
#include "cartan/serial/ik/basic_ik_runner.h"
#include "cartan/serial/ik/concepts/solve_concept.h"
#include "cartan/serial/ik/solver/lbfgsb.h"
#include "cartan/serial/ik/wrapper/restart_wrapper.h"
#include "cartan/serial/ik/solver/projected_lm.h"

#include "cartan/serial/chain/chain_concept.h"

#include <tuple>

namespace cartan
{

// ---------------------------------------------------------------------------
// Type aliases
// ---------------------------------------------------------------------------

/// Speed-optimized: restart-wrapped projected LM (fast per-iteration, multi-start).
template <chain Chain>
using speed_ik_runner = ik::restart_wrapper<Chain,
    ik::projected_lm<Chain, no_limits>, no_limits>;

/// Convergence-optimized: restart-wrapped L-BFGS-B (robust convergence, multi-start).
template <chain Chain>
using robust_ik_runner = ik::restart_wrapper<Chain,
    ik::builtin_lbfgsb<Chain, no_limits>, no_limits>;

/// Default: races speed_ik_runner against robust_ik_runner via variadic basic_ik_runner.
template <chain Chain>
using dual_ik_runner = basic_ik_runner<speed_ik_runner<Chain>, robust_ik_runner<Chain>>;

// ---------------------------------------------------------------------------
// Composable builder
// ---------------------------------------------------------------------------

/// Type-accumulating solver builder for custom policy compositions.
///
/// Usage: `auto solver = make_solver<MyChain>().policy(p1).policy(p2).build();`
/// `.build()` is the materialization point -- everything before it is configuration.
template <chain Chain, typename... Policies>
class solver_builder
{
public:
    explicit solver_builder(std::tuple<Policies...> policies)
        : m_policies(std::move(policies))
    {
    }

    template <typename Policy>
        requires ik::solve_policy<Policy>
    auto policy(Policy p) &&
    {
        return solver_builder<Chain, Policies..., Policy>{
            std::tuple_cat(std::move(m_policies), std::make_tuple(std::move(p)))
        };
    }

    auto build() &&
    {
        return std::apply([](auto&&... ps) {
            return basic_ik_runner{std::forward<decltype(ps)>(ps)...};
        }, std::move(m_policies));
    }

private:
    std::tuple<Policies...> m_policies;
};

// ---------------------------------------------------------------------------
// Preset builders
// ---------------------------------------------------------------------------

/// Preset solver builder -- wraps a fixed single-policy configuration.
template <chain Chain, typename Policy>
class preset_solver_builder
{
public:
    explicit preset_solver_builder(Policy policy)
        : m_policy(std::move(policy))
    {
    }

    auto build() &&
    {
        return basic_ik_runner{std::move(m_policy)};
    }

private:
    Policy m_policy;
};

/// Preset builder for two-policy racing configurations.
template <chain Chain, typename Policy1, typename Policy2>
class preset_racing_builder
{
public:
    explicit preset_racing_builder(Policy1 p1, Policy2 p2)
        : m_policy1(std::move(p1))
        , m_policy2(std::move(p2))
    {
    }

    auto build() &&
    {
        return basic_ik_runner{std::move(m_policy1), std::move(m_policy2)};
    }

private:
    Policy1 m_policy1;
    Policy2 m_policy2;
};

// ---------------------------------------------------------------------------
// Preset factory functions
// ---------------------------------------------------------------------------

/// Create a speed-optimized solver builder (restart-wrapped projected LM).
template <chain Chain>
auto make_speed_ik_runner()
{
    return preset_solver_builder<Chain, speed_ik_runner<Chain>>{
        speed_ik_runner<Chain>{}
    };
}

/// Create a convergence-optimized solver builder (restart-wrapped L-BFGS-B).
template <chain Chain>
auto make_robust_ik_runner()
{
    return preset_solver_builder<Chain, robust_ik_runner<Chain>>{
        robust_ik_runner<Chain>{}
    };
}

/// Create the default solver builder (races speed + convergence policies).
template <chain Chain>
auto make_dual_ik_runner()
{
    return preset_racing_builder<Chain,
        speed_ik_runner<Chain>, robust_ik_runner<Chain>>{
        speed_ik_runner<Chain>{}, robust_ik_runner<Chain>{}
    };
}

/// Create a composable solver builder. Chain .policy(p) calls, finish with .build().
template <chain Chain>
auto make_solver()
{
    return solver_builder<Chain>{std::tuple<>{}};
}

}

#endif
