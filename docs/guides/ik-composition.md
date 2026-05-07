# IK Solver Composition Guide

This guide shows how to assemble inverse kinematics solvers from Cartan's composable solve policies. The `basic_ik_solver<Policies...>` template accepts one or more policies: a single policy gives direct solve, multiple policies give cooperative interleaved racing.

**Prerequisites:** Familiarity with forward kinematics ([PoE Walkthrough](poe-walkthrough.md)) and basic IK concepts.

## Single Policy

The simplest case: one solve policy, direct solve.

```cpp
#include <cartan/ik/basic_ik_solver.h>
#include <cartan/ik/lm_solve_policy.h>

cartan::basic_ik_solver solver{cartan::lm_solve_policy<double, 6>{}};

auto target = cartan::se3<double>( /* desired pose */ );
cartan::convergence_criteria<double> criteria{1e-6, 1e-6, 200};
Eigen::Vector<double, 6> q0 = Eigen::Vector<double, 6>::Zero();

solver.setup(chain, target, q0, criteria);
auto result = solver.solve();

if (result.has_value())
{
    auto& r = result.value();
    std::cout << "Converged in " << r.iterations << " iterations\n";
    std::cout << "Solution: " << r.solution.position.transpose() << "\n";
}
else
{
    auto& e = result.error();
    std::cout << "Failed: error norm = " << e.last_error_norm << "\n";
}
```

The `solve()` method returns `std::expected<ik_result, ik_error>`, giving structured success/failure without exceptions.

## Restart Wrapping

The `restart_solve_policy` wraps any inner policy with multi-start capability. When the inner policy stalls, diverges, or hits its iteration limit, the wrapper re-seeds from a Halton sequence and retries with warm-start lambda preservation.

```cpp
#include <cartan/ik/basic_ik_solver.h>
#include <cartan/ik/restart_solve_policy.h>
#include <cartan/ik/lm_solve_policy.h>

// restart_solve_policy wraps lm_solve_policy
cartan::basic_ik_solver solver{
    cartan::restart_solve_policy{cartan::lm_solve_policy<double, 6>{}}
};

solver.setup(chain, target, q0, criteria);
auto result = solver.solve();
```

This is the recommended pattern for production use. Restart+LM achieves 99.9% success rate on standard benchmarks.

## Racing (Multiple Policies)

Pass two or more policies to `basic_ik_solver` for cooperative interleaved racing. Each `step()` performs one round-robin across all active policies, parking converged ones and selecting the best result based on the configured objective.

```cpp
#include <cartan/ik/basic_ik_solver.h>
#include <cartan/ik/restart_solve_policy.h>
#include <cartan/ik/projected_lm_solve_policy.h>
#include <cartan/ik/lbfgsb_solve_policy.h>

// Two restart-wrapped policies racing cooperatively
cartan::basic_ik_solver solver{
    cartan::restart_solve_policy{cartan::projected_lm_solve_policy<double, 7>{}},
    cartan::restart_solve_policy{cartan::lbfgsb_solve_policy<double, 7>{}}
};

solver.setup(chain, target, q0, criteria);
auto result = solver.solve();

if (result.has_value())
{
    std::cout << "Solver " << result.value().solver_index << " won\n";
}
```

The first policy receives the user's `q0`; remaining policies receive deterministic Halton seeds within joint limits. For the `speed` objective (default), the first policy to converge wins. For other objectives (`min_distance`, `max_manipulability`, `max_isotropy`), all policies run to completion and the best is selected.

This cooperative model is the key differentiator vs TRAC-IK: all policies run in the calling thread without thread spawning or proliferation.

## CTAD

Class template argument deduction allows constructing `basic_ik_solver` without explicit template arguments. Scalar type and joint count are deduced from the policy traits:

```cpp
// Scalar=double, N=6 deduced from lm_solve_policy<double, 6>
cartan::basic_ik_solver solver{cartan::lm_solve_policy<double, 6>{}};

// Scalar=double, N=7 deduced from the policies (all must agree)
cartan::basic_ik_solver solver{
    cartan::restart_solve_policy{cartan::projected_lm_solve_policy<double, 7>{}},
    cartan::restart_solve_policy{cartan::lbfgsb_solve_policy<double, 7>{}}
};
```

Specify `Scalar` and `N` only at the leaf policy level. Everything above deduces.

## Preset Solvers

Three type aliases provide ready-to-use solver configurations:

| Alias | Composition | Use Case |
|-------|-------------|----------|
| `speed_solver<double, 7>` | `restart_solve_policy<projected_lm_solve_policy>` | Fast per-iteration, multi-start |
| `convergence_solver<double, 7>` | `restart_solve_policy<lbfgsb_solve_policy>` | Robust convergence, multi-start |
| `default_solver<double, 7>` | `basic_ik_solver<speed_solver, convergence_solver>` | Races both, first convergence wins |

```cpp
#include <cartan/ik/default_solvers.h>

// Use the default racing solver directly
cartan::default_solver<double, 7> solver;
solver.setup(chain, target, q0, criteria);
auto result = solver.solve();
```

## Builder Pattern

Factory functions return builders requiring `.build()` as the materialization point. This enables future extensions like `.from_config(cfg).build()`.

### Preset Builders

```cpp
#include <cartan/ik/default_solvers.h>

auto solver = cartan::make_speed_solver<double, 7>().build();
auto solver = cartan::make_convergence_solver<double, 7>().build();
auto solver = cartan::make_default_solver<double, 7>().build();
```

### Composable Builder

Chain `.policy()` calls to accumulate policies, finish with `.build()`:

```cpp
auto solver = cartan::make_solver<double, 7>()
    .policy(cartan::restart_solve_policy{cartan::lm_solve_policy<double, 7>{}})
    .policy(cartan::dls_solve_policy<double, 7>{})
    .build();
```

## argmin Solvers

The argmin-backed policies (`slsqp_solve_policy`, `bobyqa_solve_policy`) are always available as argmin is a required dependency of `cartan::kinematics`. They provide constrained optimization with joint limits as box bounds.

```cpp
#include <cartan/ik/basic_ik_solver.h>
#include <cartan/ik/slsqp_solve_policy.h>

cartan::basic_ik_solver solver{cartan::slsqp_solve_policy<double, 7>{}};
solver.setup(chain, target, q0, criteria);
auto result = solver.solve();
```

BOBYQA is derivative-free, useful when the gradient is expensive or unreliable:

```cpp
#include <cartan/ik/basic_ik_solver.h>
#include <cartan/ik/bobyqa_solve_policy.h>

cartan::basic_ik_solver solver{cartan::bobyqa_solve_policy<double, 7>{}};
```

## Mixing Families

Any combination of native, argmin, and NLopt policies can race together in a single `basic_ik_solver`:

```cpp
#include <cartan/ik/basic_ik_solver.h>
#include <cartan/ik/restart_solve_policy.h>
#include <cartan/ik/projected_lm_solve_policy.h>
#include <cartan/ik/slsqp_solve_policy.h>

// Race a native restart+projected-LM against argmin SLSQP
cartan::basic_ik_solver solver{
    cartan::restart_solve_policy{cartan::projected_lm_solve_policy<double, 7>{}},
    cartan::slsqp_solve_policy<double, 7>{}
};

solver.setup(chain, target, q0, criteria);
auto result = solver.solve();
```

All policies in a single `basic_ik_solver` must agree on `scalar_type` and `joints`. The solver enforces this with a `static_assert`.

## Further Reading

- [IK Methods Theory](../background/ik-methods.md) -- mathematical derivation of DLS, LM, convergence criteria, and null-space projection
- [API Reference: IK](../api/ik.md) -- full function signatures for all IK types and solve policies
- [PoE Walkthrough](poe-walkthrough.md) -- building kinematic chains from scratch
