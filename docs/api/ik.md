# IK Solver API

Policy-based inverse kinematics with composable solve policies and variadic cooperative racing. The solver is assembled from independent `_solve_policy` components: a single policy yields direct solve, multiple policies yield cooperative interleaved racing. This composition replaces TRAC-IK's thread-per-solver model with zero-overhead cooperative scheduling.

See [IK Methods](../background/ik-methods.md) | [IK Composition Guide](../guides/ik-composition.md)

## Headers

| Type | Header |
|------|--------|
| `liepp::basic_ik_solver<Policies...>` | `<liepp/ik/basic_ik_solver.h>` |
| `liepp::ik_solve_policy` concept | `<liepp/ik/ik_solve_policy.h>` |
| Types: `ik_status`, `ik_result`, `ik_error`, `convergence_criteria`, `solver_options` | `<liepp/ik/ik_types.h>` |
| Type aliases, factory functions, builders | `<liepp/ik/default_solvers.h>` |
| `liepp::lm_solve_policy` | `<liepp/ik/lm_solve_policy.h>` |
| `liepp::projected_lm_solve_policy` | `<liepp/ik/projected_lm_solve_policy.h>` |
| `liepp::lbfgsb_solve_policy` | `<liepp/ik/lbfgsb_solve_policy.h>` |
| `liepp::dls_solve_policy` | `<liepp/ik/dls_solve_policy.h>` |
| `liepp::newton_raphson_solve_policy` | `<liepp/ik/newton_raphson_solve_policy.h>` |
| `liepp::restart_solve_policy` | `<liepp/ik/restart_solve_policy.h>` |
| `liepp::no_limits`, `liepp::clamp_limits` | `<liepp/ik/limits_policy.h>` |
| `liepp::slsqp_solve_policy` (nablapp) | `<liepp/ik/slsqp_solve_policy.h>` |
| `liepp::bobyqa_solve_policy` (nablapp) | `<liepp/ik/bobyqa_solve_policy.h>` |
| `liepp::nlopt_slsqp_solve_policy` | `<liepp/ik/nlopt_slsqp_solve_policy.h>` (requires `LIEPP_HAS_NLOPT`) |
| `liepp::nlopt_bobyqa_solve_policy` | `<liepp/ik/nlopt_bobyqa_solve_policy.h>` (requires `LIEPP_HAS_NLOPT`) |

## Quick Start

Minimal working example using CTAD:

```cpp
#include <liepp/ik/basic_ik_solver.h>
#include <liepp/ik/restart_solve_policy.h>
#include <liepp/ik/lm_solve_policy.h>

// CTAD deduces Scalar=double, N=6 from the policy
liepp::basic_ik_solver solver{
    liepp::restart_solve_policy{liepp::lm_solve_policy<double, 6>{}}
};

solver.setup(chain, target, q0, criteria);
auto result = solver.solve();

if (result.has_value())
{
    auto& r = result.value();
    std::cout << "Converged in " << r.iterations << " iterations\n";
    std::cout << "Solution: " << r.solution.position.transpose() << "\n";
}
```

## basic_ik_solver<Policies...>

Variadic policy-based IK solver with cooperative interleaved racing. When instantiated with a single policy, behaves identically to a non-variadic solver with zero overhead. With two or more policies, provides cooperative round-robin racing with parking and objective-based result selection.

```cpp
template <typename... Policies>
    requires (sizeof...(Policies) >= 1) && (ik_solve_policy<Policies> && ...)
class basic_ik_solver;
```

All policies must agree on `scalar_type` and `joints` (enforced by `static_assert`). Scalar and joint count are deduced from the first policy:

```cpp
using scalar_type = typename first_policy::scalar_type;
static constexpr int joints = first_policy::joints;
```

### CTAD

A deduction guide allows constructing without explicit template arguments:

```cpp
// Single policy -- direct solve
liepp::basic_ik_solver solver{liepp::lm_solve_policy<double, 6>{}};

// Two policies -- cooperative racing
liepp::basic_ik_solver solver{
    liepp::restart_solve_policy{liepp::projected_lm_solve_policy<double, 7>{}},
    liepp::restart_solve_policy{liepp::lbfgsb_solve_policy<double, 7>{}}
};
```

### Constructor

```cpp
basic_ik_solver() = default;
explicit basic_ik_solver(Policies... policies);
```

### setup

```cpp
void setup(
    const kinematic_chain<scalar_type, joints>& chain,
    const se3<scalar_type>& target,
    const position_type& q0,
    const convergence_criteria<scalar_type>& criteria,
    const solver_options<scalar_type>& options = {});
```

Initialize the solver with a kinematic chain, target SE(3) pose, seed joint configuration, convergence criteria, and optional solver options. For multi-policy solvers, the first policy receives the user's `q0`; remaining policies receive deterministic Halton seeds within joint limits.

### step

```cpp
ik_status step();
```

Execute one solver step. For single-policy: delegates directly to the policy's `step()`. For multi-policy: one round-robin across all active (non-parked) policies.

### step_n

```cpp
ik_status step_n(int n);
```

Execute `n` round-robin rounds, stopping early on terminal status.

### solve

```cpp
std::expected<ik_result<scalar_type, joints>, ik_error<scalar_type, joints>> solve();
```

Convenience method: loops `step()` until convergence or failure. Returns `ik_result` on success or `ik_error` on failure via `std::expected`.

### Query Methods

```cpp
bool converged() const;
scalar_type error_norm() const;
int iterations() const;
const position_type& current_q() const;
ik_status status() const;
void abort();
```

### Thread Safety

Different solver instances may safely operate concurrently on the same `const kinematic_chain`. A single solver instance must not be used from multiple threads without synchronization.

## ik_solve_policy Concept

Single-parameter concept defining the interface that all IK solve policies must satisfy. Policies are pull-based passive objects: the solver calls `setup()` once, then `step()` repeatedly.

```cpp
template <typename S>
concept ik_solve_policy = requires
{
    typename S::scalar_type;
    typename S::limits_type;
    { S::joints } -> std::convertible_to<int>;
} && requires(
    S& s,
    const kinematic_chain<typename S::scalar_type, S::joints>& chain,
    const se3<typename S::scalar_type>& target,
    const typename joint_state<typename S::scalar_type, S::joints>::position_type& q0,
    const convergence_criteria<typename S::scalar_type>& criteria)
{
    { s.setup(chain, target, q0, criteria) };
    { s.step(chain) } -> std::same_as<ik_status>;
    { s.converged() } -> std::convertible_to<bool>;
    { s.solution() } -> std::convertible_to<position_type>;
    { s.error_norm() } -> std::convertible_to<typename S::scalar_type>;
    { s.iterations() } -> std::convertible_to<int>;
    { s.abort() };
};
```

A conforming type must expose:

| Trait | Purpose |
|-------|---------|
| `S::scalar_type` | Floating-point type |
| `S::joints` | `static constexpr int`, joint count or `dynamic` |
| `S::limits_type` | Limit enforcement policy type |

## Solve Policies

### lm_solve_policy

Levenberg-Marquardt with Nielsen-style lambda update. Trust-region approach where the gain ratio controls step acceptance and damping adaptation.

```cpp
template <typename Scalar = double, int N = dynamic, typename LimitsPolicy = clamp_limits>
class lm_solve_policy;
```

**Options:**

| Field | Default | Description |
|-------|---------|-------------|
| `initial_lambda_factor` | `1e-3` | Lambda = factor * max(diag(J^T J)) |
| `stall_threshold` | `1e-6` | Error change threshold for stall detection |
| `divergence_factor` | `10` | Error ratio triggering divergence |
| `stall_window` | `5` | Window size for stall detection |

### projected_lm_solve_policy

Projected Levenberg-Marquardt with box constraints and dogleg trust-region. Enforces joint limits during the step itself, not as post-hoc clamping.

```cpp
template <typename Scalar = double, int N = dynamic, typename LimitsPolicy = no_limits>
class projected_lm_solve_policy;
```

Note: defaults to `no_limits` because constraint enforcement is built into the algorithm.

### lbfgsb_solve_policy

L-BFGS-B with generalized Cauchy point and subspace minimization. Robust convergence for high-dimensional IK problems.

```cpp
template <typename Scalar = double, int N = dynamic, typename LimitsPolicy = clamp_limits>
class lbfgsb_solve_policy;
```

### dls_solve_policy

Damped Least Squares with SVD-based adaptive damping (Nakamura). Body-frame Newton-Raphson iteration where the damping factor increases as the smallest singular value drops below a threshold.

```cpp
template <typename Scalar = double, int N = dynamic, typename LimitsPolicy = clamp_limits>
class dls_solve_policy;
```

**Options:**

| Field | Default | Description |
|-------|---------|-------------|
| `singularity_threshold` | `0.01` | sigma_min threshold for damping |
| `lambda_max` | `0.04` | Maximum damping factor |
| `stall_threshold` | `1e-6` | Error change threshold for stall detection |
| `divergence_factor` | `10` | Error ratio triggering divergence |
| `stall_window` | `5` | Window size for stall detection |

### newton_raphson_solve_policy

Newton-Raphson IK with weighted convergence checking (separate angular and linear tolerances).

```cpp
template <typename Scalar = double, int N = dynamic, typename LimitsPolicy = clamp_limits>
class newton_raphson_solve_policy;
```

### slsqp_solve_policy (nablapp)

nablapp-backed SLSQP gradient-based solve policy with box constraints. Uses Kraft's Sequential Least Squares Programming algorithm via nablapp, with analytical gradient through the SE(3) log Jacobian. Always available (nablapp is a required dependency of `liepp::kinematics`).

```cpp
template <typename Scalar = double, int N = dynamic, typename LimitsPolicy = clamp_limits>
class slsqp_solve_policy;
```

### bobyqa_solve_policy (nablapp)

nablapp-backed BOBYQA derivative-free solve policy with box constraints. Uses the BOBYQA algorithm via nablapp, with joint limits as box bounds.

```cpp
template <typename Scalar = double, int N = dynamic, typename LimitsPolicy = clamp_limits>
class bobyqa_solve_policy;
```

### nlopt_slsqp_solve_policy

NLopt SLSQP solve policy. Same algorithm as `slsqp_solve_policy` but backed by NLopt instead of nablapp. Guarded by `LIEPP_HAS_NLOPT`.

```cpp
template <typename Scalar = double, int N = dynamic, typename LimitsPolicy = clamp_limits>
class nlopt_slsqp_solve_policy;
```

### nlopt_bobyqa_solve_policy

NLopt BOBYQA solve policy. Same algorithm as `bobyqa_solve_policy` but backed by NLopt. Guarded by `LIEPP_HAS_NLOPT`.

```cpp
template <typename Scalar = double, int N = dynamic, typename LimitsPolicy = clamp_limits>
class nlopt_bobyqa_solve_policy;
```

## restart_solve_policy

Restart wrapper that re-seeds from Halton sequences on stall or diverge, with warm-start lambda preservation. Wraps any inner policy satisfying `ik_solve_policy`.

```cpp
template <typename Scalar = double, int N = dynamic,
          typename InnerPolicy = projected_lm_solve_policy<Scalar, N>,
          typename LimitsPolicy = typename InnerPolicy::limits_type>
class restart_solve_policy;
```

When the inner policy reports `stalled`, `diverged`, or `iteration_limit`, the wrapper generates a new seed configuration from a Halton sequence and re-initializes the inner policy. The best damping parameter (lambda) from near-miss attempts is preserved across restarts for warm-starting (when the inner policy supports `set_lambda()`/`lambda()`).

## Limits Policies

Stateless policy structs controlling joint limit enforcement.

### clamp_limits

```cpp
struct clamp_limits;
```

Hard clamping: clamps each `q(i)` to `[position_min, position_max]`. Default for most solve policies.

### no_limits

```cpp
struct no_limits;
```

No-op: applies no enforcement. Used when the policy handles constraints internally (e.g., `projected_lm_solve_policy`, or NLopt/nablapp policies with box constraints).

## Type Aliases and Factories

Defined in `<liepp/ik/default_solvers.h>`.

### Type Aliases

```cpp
// Speed-optimized: restart-wrapped projected LM
template <typename Scalar = double, int N = dynamic>
using speed_solver = restart_solve_policy<Scalar, N,
    projected_lm_solve_policy<Scalar, N, no_limits>, no_limits>;

// Convergence-optimized: restart-wrapped L-BFGS-B
template <typename Scalar = double, int N = dynamic>
using convergence_solver = restart_solve_policy<Scalar, N,
    lbfgsb_solve_policy<Scalar, N, no_limits>, no_limits>;

// Default: races speed_solver against convergence_solver
template <typename Scalar = double, int N = dynamic>
using default_solver = basic_ik_solver<speed_solver<Scalar, N>, convergence_solver<Scalar, N>>;
```

### Factory Functions

All factories return builders requiring `.build()` as the materialization point:

```cpp
// Preset factories
auto solver = liepp::make_speed_solver<double, 7>().build();
auto solver = liepp::make_convergence_solver<double, 7>().build();
auto solver = liepp::make_default_solver<double, 7>().build();

// Composable builder
auto solver = liepp::make_solver<double, 7>()
    .policy(liepp::lm_solve_policy<double, 7>{})
    .policy(liepp::dls_solve_policy<double, 7>{})
    .build();
```

## Types

### convergence_criteria

```cpp
template <typename Scalar = double>
struct convergence_criteria {
    Scalar position_tol{Scalar(1e-6)};
    Scalar orientation_tol{Scalar(1e-6)};
    int max_iterations{100};
};
```

Separate position (linear, meters) and orientation (angular, radians) tolerances.

### solver_options

```cpp
template <typename Scalar = double>
struct solver_options {
    ik_objective objective{ik_objective::speed};
    int max_total_iterations{500};
    unsigned int halton_seed{42};
};
```

Controls multi-policy solver racing behavior. Separate from `convergence_criteria`, which controls per-policy behavior.

### ik_result

```cpp
template <typename Scalar = double, int N = dynamic>
struct ik_result {
    joint_state<Scalar, N> solution;
    Scalar final_error_norm{};
    int iterations{};
    int solver_index{};
};
```

### ik_error

```cpp
template <typename Scalar = double, int N = dynamic>
struct ik_error {
    ik_failure reason;
    typename joint_state<Scalar, N>::position_type last_q;
    Scalar last_error_norm{};
    Scalar condition_number{};
    bool near_singular{};
};
```

### ik_status

```cpp
enum class ik_status {
    running,
    converged,
    diverged,
    stalled,
    joint_limit_hit,
    iteration_limit
};
```

### ik_objective

```cpp
enum class ik_objective {
    speed,
    min_distance,
    max_manipulability,
    max_isotropy
};
```

### ik_failure

```cpp
enum class ik_failure {
    unreachable,
    diverged,
    stalled,
    iteration_limit,
    joint_limit_violation,
    aborted
};
```

## Three Solver Families

liepp provides IK solve policies from three independent solver backends:

### Native liepp

Always available. Pure C++/Eigen implementations with zero external dependencies:

- `lm_solve_policy` -- Levenberg-Marquardt with Nielsen lambda update
- `projected_lm_solve_policy` -- Projected LM with box constraints and dogleg trust-region
- `lbfgsb_solve_policy` -- L-BFGS-B with generalized Cauchy point
- `dls_solve_policy` -- Damped Least Squares with adaptive SVD damping
- `newton_raphson_solve_policy` -- Newton-Raphson with weighted convergence

### nablapp

Always available (nablapp is a required dependency of `liepp::kinematics`). Provides gradient-based and derivative-free constrained optimization:

- `slsqp_solve_policy` -- Kraft's SLSQP with analytical gradient via SE(3) log Jacobian
- `bobyqa_solve_policy` -- BOBYQA derivative-free with box constraints

### NLopt (optional)

Available when `LIEPP_HAS_NLOPT` is defined. Legacy backend, targeted for removal when nablapp fully replaces it:

- `nlopt_slsqp_solve_policy` -- NLopt LD_SLSQP
- `nlopt_bobyqa_solve_policy` -- NLopt BOBYQA

## Edge Cases

- **Single vs. multi-policy:** `basic_ik_solver` with one policy compiles to identical code as a non-variadic solver (zero overhead). With multiple policies, cooperative round-robin racing runs all policies in the calling thread.
- **Convergence checking** uses separate angular and linear tolerances, not combined norm. A solution that is positionally accurate but rotationally off (or vice versa) will not be accepted.
- **nablapp and NLopt policies** handle joint limits natively via box constraints. Use `no_limits` or `clamp_limits` as appropriate; the policy's internal constraint handling takes precedence.
- **Near-singularity:** DLS adaptively increases damping as `sigma_min` drops. LM increases lambda on rejected steps. Both avoid wild joint motions near singularities.
