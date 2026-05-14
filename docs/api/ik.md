# ik

Headers under `cartan/serial/ik/` (sublib `cartan-serial-chain`). Iterative IK
solvers live in the `cartan::ik::` namespace; runners, status / result types,
and convergence criteria live in `cartan::`. Link with `cartan::serial-chain`
(or the convenience target `cartan::cartan`). The umbrella header
`<cartan/serial/ik.h>` includes the runner, every solver, the wrappers, the
limits / weight policies, and the type aliases / builders.

Policy-based iterative IK with the work-unit accounting contract:
`step(chain, int N) -> step_result<Scalar>`. The runner asks each solver for
up to `N` algorithmic work units and accumulates the returned
`units_consumed` against `convergence_criteria::max_total_work_units`. Single
policy yields direct solve; two or more policies yield cooperative
round-robin racing. `restart_wrapper` adds an outer Halton-seed restart loop
on stall. `exhaustive_ik_runner` enumerates all valid solutions via
multi-start.

See [IK Methods](../background/ik-methods.md) | [IK Composition Guide](../guides/ik-composition.md) | [Analytical IK](analytical.md)

## Headers

| Form | Header |
|------|--------|
| All IK | `#include <cartan/serial/ik.h>` |
| `cartan::basic_ik_runner` | `#include <cartan/serial/ik/basic_ik_runner.h>` |
| `cartan::convergence_criteria`, `cartan::ik_status`, `cartan::ik_termination_reason`, `cartan::ik_failure`, `cartan::ik_objective`, `cartan::step_metrics`, `cartan::step_result`, `cartan::solver_options` | `#include <cartan/serial/ik/ik_status.h>` |
| `cartan::ik_result`, `cartan::ik_error` | `#include <cartan/serial/ik/ik_result.h>` |
| `cartan::ik::solve_policy` concept, `cartan::ik::step_one` | `#include <cartan/serial/ik/concepts/solve_concept.h>` |
| `cartan::no_limits`, `cartan::clamp_limits`, `cartan::null_space_limits` | `#include <cartan/serial/ik/policy/limits_policy.h>` |
| `cartan::error_weight` | `#include <cartan/serial/ik/policy/error_weight.h>` |
| `cartan::ik::lm` (alias for `builtin_lm`) | `#include <cartan/serial/ik/solver/lm.h>` |
| `cartan::ik::lbfgsb` (alias for `builtin_lbfgsb`) | `#include <cartan/serial/ik/solver/lbfgsb.h>` |
| `cartan::ik::projected_lm`, `cartan::ik::dls`, `cartan::ik::newton_raphson` | `#include <cartan/serial/ik/solver/{projected_lm,dls,newton_raphson}.h>` |
| argmin-backed: `cartan::ik::argmin_lm`, `argmin_lbfgsb`, `argmin_slsqp`, `argmin_bobyqa`, `argmin_projected_gn`, `argmin_projected_gradient_gn` | `#include <cartan/serial/ik/solver/argmin_*.h>` |
| NLopt-backed: `cartan::ik::nlopt_slsqp`, `cartan::ik::nlopt_bobyqa` | `#include <cartan/serial/ik/solver/nlopt_*.h>` (requires `CARTAN_HAS_NLOPT`) |
| SQP family: `cartan::ik::nw_sqp`, `filter_nw_sqp`, `filter_slsqp` | `#include <cartan/serial/ik/solver/{nw_sqp,filter_nw_sqp,filter_slsqp}.h>` |
| MMA / GCMMA / CMA-ES / aug. Lagrangian: `cartan::ik::mma`, `gcmma`, `cmaes`, `augmented_lagrangian` | `#include <cartan/serial/ik/solver/{mma,gcmma,cmaes,augmented_lagrangian}.h>` |
| `cartan::ik::restart_wrapper` | `#include <cartan/serial/ik/wrapper/restart_wrapper.h>` |
| `cartan::exhaustive_ik_runner`, `cartan::exhaustive_options`, `cartan::exhaustive_result`, `cartan::ranking_strategy` | `#include <cartan/serial/ik/solver/exhaustive_ik_runner.h>` |
| Type aliases + builders: `speed_ik_runner`, `robust_ik_runner`, `dual_ik_runner`, `make_solver`, `make_speed_ik_runner`, `make_robust_ik_runner`, `make_dual_ik_runner` | `#include <cartan/serial/ik/solvers.h>` |

## Quick Start

Minimal working example on a 3-DOF planar arm using the LM policy:

```cpp
#include <cartan/serial_chain.h>
#include <numbers>

cartan::vector3<double> z{0, 0, 1};
auto s1 = cartan::screw_axis<double>::revolute(z, {0, 0, 0});
auto s2 = cartan::screw_axis<double>::revolute(z, {1, 0, 0});
auto s3 = cartan::screw_axis<double>::revolute(z, {2, 0, 0});
auto home = cartan::se3<double>(cartan::so3<double>::identity(), {3, 0, 0});
cartan::joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};
cartan::kinematic_chain<double, 3> chain(home, {s1, s2, s3}, {lim, lim, lim});

Eigen::Vector3d q_known{0.3, -0.5, 0.2};
auto target = cartan::forward_kinematics(chain, q_known).end_effector;

Eigen::Vector3d q0{0.0, 0.0, 0.0};
cartan::convergence_criteria<double> criteria{1e-6, 1e-6, 100, 200};

cartan::basic_ik_runner<cartan::ik::lm<cartan::kinematic_chain<double, 3>>> solver;
solver.setup(chain, target, q0, criteria);
auto result = solver.solve();

if (result.has_value())
{
    std::cout << "Solution: " << result.value().solution.position.transpose() << "\n";
}
```

`setup()` initializes the solver state with chain, target, seed, and
convergence criteria. `solve()` drives the runner loop, asking the inner
policy for `max_total_work_units` total units of work and stopping at
convergence, divergence, or the work-unit cap. `result.has_value()`
distinguishes success (`ik_result` payload) from failure (`ik_error`
payload).

## basic_ik_runner

```cpp
template <typename... Policies>
    requires (sizeof...(Policies) >= 1)
          && (cartan::ik::solve_policy<Policies> && ...)
class basic_ik_runner;
```

Variadic policy-based IK runner. With a single policy, behaves identically
to a non-variadic solver with zero overhead. With two or more policies,
provides cooperative round-robin racing: each `step()` ticks all active
policies once; converged policies are parked and the runner selects the
best result based on the configured objective.

All policies must agree on `chain_type`, `scalar_type`, and `joints`
(enforced by `static_assert`). Scalar and joint count are deduced from
the first policy.

### setup

```cpp
void setup(
    const chain_type& chain,
    const se3<scalar_type>& target,
    const position_type& q0,
    const convergence_criteria<scalar_type>& criteria,
    const solver_options<scalar_type>& options = {});
```

Initialize with chain, target pose, seed configuration, convergence
criteria, and optional racing options. For multi-policy runners, the
first policy receives the user's `q0`; remaining policies receive
deterministic Halton seeds within joint limits.

### step

```cpp
ik_status step();
```

Execute one solver step. For single-policy: drives the inner policy for
one algorithmic work unit. For multi-policy: one round-robin across all
active (non-parked) policies.

### step_n

```cpp
ik_status step_n(int n);
```

Execute `n` round-robin rounds, stopping early on terminal status.

### solve

```cpp
std::expected<ik_result<scalar_type, joints>, ik_error<scalar_type, joints>>
solve();
```

Convenience method: drives the runner until convergence or the work-unit
cap is hit. Single-policy mode uses the total-budget accumulator loop
(asks the inner policy for as many units as remain in the budget;
accumulates returned `units_consumed`). Multi-policy mode loops `step()`
until all policies are parked or `max_total_iterations` is hit.

### Query methods

```cpp
bool converged() const;
scalar_type error_norm() const;
int iterations() const;
const position_type& current_q() const;
ik_status status() const;
void abort();
```

### Thread safety

Different runner instances may operate concurrently on the same
`const chain`. A single runner instance must not be used from multiple
threads without synchronization.

### Aliases

Defined in `<cartan/serial/ik/solvers.h>`:

```cpp
template <chain Chain>
using speed_ik_runner = cartan::ik::restart_wrapper<Chain,
    cartan::ik::projected_lm<Chain, no_limits>, no_limits>;

template <chain Chain>
using robust_ik_runner = cartan::ik::restart_wrapper<Chain,
    cartan::ik::builtin_lbfgsb<Chain, no_limits>, no_limits>;

template <chain Chain>
using dual_ik_runner = basic_ik_runner<
    speed_ik_runner<Chain>, robust_ik_runner<Chain>>;
```

`speed_ik_runner` wraps `projected_lm` in a restart loop for fast
per-iteration convergence with multi-start fallback. `robust_ik_runner`
wraps L-BFGS-B for robust convergence on stiff problems. `dual_ik_runner`
races the two via `basic_ik_runner`.

### Builders

```cpp
template <chain Chain> auto make_solver();
template <chain Chain> auto make_speed_ik_runner();
template <chain Chain> auto make_robust_ik_runner();
template <chain Chain> auto make_dual_ik_runner();
```

Preset builders return a builder type with `.build()` as the
materialization point. The composable `make_solver` accepts chained
`.policy(p)` calls and produces a `basic_ik_runner` of the accumulated
policies:

```cpp
auto solver = cartan::make_solver<MyChain>()
    .policy(cartan::ik::lm<MyChain>{})
    .policy(cartan::ik::dls<MyChain>{})
    .build();
```

## solve_policy concept

```cpp
namespace cartan::ik {

template <typename S>
concept solve_policy = requires
{
    typename S::chain_type;
    typename S::scalar_type;
    typename S::limits_type;
    { S::joints } -> std::convertible_to<int>;
} && requires(
    S& s,
    const typename S::chain_type& chain,
    const se3<typename S::scalar_type>& target,
    const typename joint_state<typename S::scalar_type, S::joints>::position_type& q0,
    const convergence_criteria<typename S::scalar_type>& criteria)
{
    { s.setup(chain, target, q0, criteria) };
    { s.step(chain, int{}) } -> std::same_as<step_result<typename S::scalar_type>>;
    { s.converged() } -> std::convertible_to<bool>;
    { s.solution() } -> std::convertible_to<
        typename joint_state<typename S::scalar_type, S::joints>::position_type>;
    { s.error_norm() } -> std::convertible_to<typename S::scalar_type>;
    { s.iterations() } -> std::convertible_to<int>;
    { s.abort() };
};

}
```

A conforming policy exposes `chain_type`, `scalar_type`, `joints`, and
`limits_type`. The `step(chain, int N) -> step_result<Scalar>` method drives
the solver for up to `N` algorithmic work units forward and returns the
status plus the metrics (`units_consumed`, `error_norm`). The runner
accumulates `units_consumed` against `convergence_criteria::max_total_work_units`.

### step_one helper

```cpp
template <typename S>
    requires cartan::ik::solve_policy<S>
auto step_one(S& s, const typename S::chain_type& chain);
```

Ergonomic single-unit helper. Drives the solver for one algorithmic-work
unit. Primary callers are enumeration drivers (e.g.
`exhaustive_ik_runner`), tests, and debug observers that want
step-by-step visibility into solver progress.

## convergence_criteria

```cpp
template <typename Scalar = double>
struct convergence_criteria
{
    Scalar position_tol{Scalar(1e-6)};
    Scalar orientation_tol{Scalar(1e-6)};
    int max_iterations_per_attempt{100};
    int max_total_work_units{200};
};
```

Separate position (linear) and orientation (angular) tolerances per
Lynch & Park, Modern Robotics, Ch. 6.2.

- `position_tol` — pose-error position tolerance.
- `orientation_tol` — pose-error orientation tolerance.
- `max_iterations_per_attempt` — bounds a single solver attempt. The
  per-attempt cap consulted by every solver's internal iteration counter
  and by self-restarting solvers as the per-attempt budget before
  triggering a restart.
- `max_total_work_units` — bounds the runner-level total budget,
  measured in algorithmic work units (1 unit = one major iteration of
  the solver's design). The runner accumulates
  `step_result::metrics.units_consumed` against this cap. Internal
  restarts charge zero units beyond the failing inner attempt that
  triggered them. The cap is literal (no slack).

## Status and Result Types

### ik_status

```cpp
enum class ik_status
{
    running,
    converged,
    diverged,
    stalled,
    joint_limit_hit,
    iteration_limit
};
```

Stepper-level control flow signal returned by `step()` calls.

### ik_termination_reason

```cpp
enum class ik_termination_reason
{
    unknown,
    converged,
    iteration_limit,
    stall_detected,
    divergence_detected,
    joint_limit_hit,
    solver_converged_pose_missed,
    solver_ftol_reached,
    solver_xtol_reached,
    solver_objective_stalled,
    solver_roundoff_limited,
    solver_stalled,
    solver_aborted,
    solver_budget_exhausted,
    solver_max_iterations,
    solver_diverged
};
```

Fine-grained termination reason for diagnostics. `ik_status` is too coarse
to distinguish the six underlying argmin terminators that collapse into
`ik_status::stalled`; policies that wrap a lower-level solver report the
specific inner terminator via `termination_reason()`. Policies that do not
opt in report `ik_termination_reason::unknown`, and `basic_ik_runner`
propagates the reported value into `ik_error::termination_reason`.

### ik_objective

```cpp
enum class ik_objective
{
    speed,
    min_distance,
    max_manipulability,
    max_isotropy
};
```

Controls multi-policy racing selection. `speed` stops at the first
converging policy; the other objectives keep all policies running and
select the best converged result by min error norm, max manipulability
(`product of singular values`), or max isotropy (`sigma_min / sigma_max`).

### ik_failure

```cpp
enum class ik_failure
{
    unreachable,
    diverged,
    stalled,
    iteration_limit,
    joint_limit_violation,
    aborted
};
```

Failure reason reported in `ik_error`.

### step_metrics

```cpp
template <typename Scalar = double>
struct step_metrics
{
    int units_consumed{};
    Scalar error_norm{};
};
```

Accounting / observability metrics returned by `step()`. `units_consumed`
is the number of algorithmic work units charged by the call;
`error_norm` is the most recent task-error magnitude.

### step_result

```cpp
template <typename Scalar = double>
struct step_result
{
    ik_status status{ik_status::running};
    step_metrics<Scalar> metrics{};
};
```

Splits the control-flow signal (`status`) from accounting metrics. Future
metric fields extend `step_metrics` without changing the outer return
shape.

### solver_options

```cpp
template <typename Scalar = double>
struct solver_options
{
    ik_objective objective{ik_objective::speed};
    int max_total_iterations{500};
    unsigned int halton_seed{42};
};
```

Controls multi-policy racing behavior: the racing objective, the
aggregate round-robin tick cap, and the Halton seed offset for
reproducible secondary-policy seeding.

### ik_result

```cpp
template <typename Scalar = double, int N = dynamic>
struct ik_result
{
    joint_state<Scalar, N> solution;
    Scalar final_error_norm{};
    int iterations{};
    int solver_index{};
};
```

Successful IK outcome. `solver_index` identifies which policy produced
the solution in multi-policy racing.

### ik_error

```cpp
template <typename Scalar = double, int N = dynamic>
struct ik_error
{
    ik_failure reason;
    ik_termination_reason termination_reason{ik_termination_reason::unknown};
    typename joint_state<Scalar, N>::position_type last_q;
    Scalar last_error_norm{};
    Scalar condition_number{};
    bool near_singular{};
};
```

Failure diagnostic. `last_q` is the joint configuration at the time of
failure; `last_error_norm` is the residual at that configuration.

## Limits Policies

Stateless policy structs controlling joint-limit enforcement on the hot path.

### no_limits

```cpp
struct no_limits;
```

No-op: applies no enforcement. Use when the policy handles constraints
internally (e.g., `projected_lm`, NLopt/argmin policies with box
constraints). Default for `cartan::ik::lm`, `cartan::ik::projected_lm`,
`cartan::ik::lbfgsb`-family policies that are LM-trust-region-based.
**Critical**: post-step `clamp_limits` invalidates LM trust-region
semantics; `no_limits` is the correct default for the LM family.

### clamp_limits

```cpp
struct clamp_limits;
```

Hard clamping: clamps each `q(i)` to `[position_min, position_max]`.
Simple and robust, but may cause discontinuities at boundaries. Default
for argmin/NLopt policies that already enforce box constraints
internally (`argmin_slsqp`, `argmin_bobyqa`, `nlopt_*`, `lbfgsb`).

### null_space_limits

```cpp
struct null_space_limits;
```

Null-space projection: for redundant chains (DOF > 6), pushes joints
toward their range midpoints via gradient projection into the Jacobian
null space, then safety-clamps. Improves distance from joint limits
without degrading the primary IK task.

Reference: Lynch & Park, Modern Robotics, Ch. 6.3, p. 235-237.
           Liegeois, A., "Automatic Supervisory Control of Configuration
           and Behavior of Multibody Mechanisms," 1977.

## error_weight

```cpp
template <typename Scalar = double>
struct error_weight
{
    vector6<Scalar> weights{vector6<Scalar>::Ones()};

    vector6<Scalar> apply(const vector6<Scalar>& v) const;
    Scalar weighted_angular_norm(const vector6<Scalar>& v) const;
    Scalar weighted_linear_norm(const vector6<Scalar>& v) const;
};
```

Per-component weight on the 6-vector pose error. Position and
orientation components can be weighted independently for tasks where
one dominates the other. The default (all-ones) gives equal weight to
all components.

## Solvers

### cartan::ik::lm

```cpp
template <chain Chain, typename LimitsPolicy = no_limits>
using lm = builtin_lm<Chain, LimitsPolicy>;
```

Public alias for the built-in Levenberg-Marquardt solver. Body-frame
Newton-Raphson with trust-region damping (Nielsen-style lambda update).
Each step: compute FK, body-frame error, Jacobian, Hessian approximation
`H = J^T J`, gradient `g = J^T V_b`, solve `(H + lambda*I) dq = g`,
evaluate gain ratio, accept/reject step, update lambda.

Reference: Lynch & Park, Modern Robotics, Ch. 6.2, p. 227-233.
           Nielsen, "Damping Parameter in Marquardt's Method", 1999.

### cartan::ik::lbfgsb

```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
using lbfgsb = builtin_lbfgsb<Chain, LimitsPolicy>;
```

Public alias for the built-in L-BFGS-B solver. Convergence-optimized via
analytical gradient through the SE(3) log Jacobian, with the generalized
Cauchy point to identify the active set followed by subspace
minimization on free variables and a backtracking Armijo line search.

Reference: Byrd, Lu, Nocedal, Zhu, "A Limited Memory Algorithm for Bound
           Constrained Optimization", SIAM J. Sci. Comput., 1995.

### cartan::ik::projected_lm

```cpp
template <chain Chain, typename LimitsPolicy = no_limits>
class projected_lm;
```

Projected Levenberg-Marquardt with active-set box projection and an
optional dogleg trust-region step. Enforces joint limits within the
optimization step (not post-hoc clamping). Carries a built-in
self-restart on stall via Halton re-seed, so wrapping `projected_lm` in
an outer `restart_wrapper` is no longer the recommended composition —
the bare `projected_lm` already delivers the multi-start behavior.

### cartan::ik::dls

```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class dls;
```

Damped Least Squares with SVD-based adaptive damping (Nakamura).
Body-frame Newton-Raphson iteration where the damping factor increases
as the smallest singular value drops below a threshold.

### cartan::ik::newton_raphson

```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class newton_raphson;
```

Newton-Raphson IK with undamped Gauss-Newton Hessian and backtracking
Armijo line search for globalization. Separate angular and linear
convergence tolerances.

Reference: Nocedal & Wright, *Numerical Optimization*, Ch. 3 (line
           search), Ch. 10 (nonlinear least squares, Gauss-Newton).

### cartan::ik::argmin_lm

```cpp
template <chain Chain, typename LimitsPolicy = no_limits>
class argmin_lm;
```

argmin-backed Levenberg-Marquardt with the least-squares adapter
exposing the 6-element body-frame error as residuals and the body
Jacobian. Joint limits are enforced via clamping after each step since
LM is unconstrained.

### cartan::ik::argmin_lbfgsb

```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class argmin_lbfgsb;
```

argmin-backed L-BFGS-B for bound-constrained IK using the analytical
gradient via the SE(3) log Jacobian.

Reference: Byrd, Lu, Nocedal, Zhu (1995).

### cartan::ik::argmin_slsqp

```cpp
template <chain Chain,
          typename LimitsPolicy = clamp_limits,
          typename Convergence = argmin::default_convergence,
          argmin::sqp_mode Mode = argmin::sqp_mode::accurate>
class argmin_slsqp;
```

argmin-backed SLSQP for constrained IK with box constraints. Uses
Kraft's Sequential Least Squares Programming algorithm with the
analytical gradient via the SE(3) log Jacobian. The `Convergence`
template parameter lets consumers opt out of argmin's default
four-criterion convergence policy in favor of alternatives like
`argmin::slsqp_compatible_convergence` (NLopt-style ftol+xtol+stall).

### cartan::ik::argmin_bobyqa

```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class argmin_bobyqa;
```

argmin-backed BOBYQA derivative-free solver with box constraints.
Builds a quadratic interpolation model of the objective and uses
trust-region steps.

Reference: Powell, M.J.D., "The BOBYQA Algorithm for Bound Constrained
           Optimization Without Derivatives", 2009.

### cartan::ik::argmin_projected_gn

```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class argmin_projected_gn;
```

argmin-backed projected Gauss-Newton with active-set bounds.

### cartan::ik::argmin_projected_gradient_gn

```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class argmin_projected_gradient_gn;
```

argmin-backed projected-gradient Gauss-Newton with Armijo backtracking.

### cartan::ik::nlopt_slsqp

```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class nlopt_slsqp;
```

NLopt SLSQP solver. Same algorithm as `argmin_slsqp` but backed by
NLopt. Guarded by `CARTAN_HAS_NLOPT`.

### cartan::ik::nlopt_bobyqa

```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class nlopt_bobyqa;
```

NLopt BOBYQA solver. Same algorithm as `argmin_bobyqa` but backed by
NLopt. Guarded by `CARTAN_HAS_NLOPT`.

### cartan::ik::nw_sqp

```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class nw_sqp;
```

argmin-backed Nocedal-Wright SQP with inequality constraints.

Reference: Nocedal & Wright, *Numerical Optimization*, Ch. 18 (SQP).

### cartan::ik::filter_nw_sqp

```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class filter_nw_sqp;
```

argmin-backed filter Nocedal-Wright SQP.

### cartan::ik::filter_slsqp

```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class filter_slsqp;
```

argmin-backed filter SLSQP with box constraints.

### cartan::ik::mma

```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class mma;
```

argmin-backed Method of Moving Asymptotes.

### cartan::ik::gcmma

```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class gcmma;
```

argmin-backed Globally Convergent MMA. Extends MMA with per-component
conservativity coefficients that grow on non-conservative inner-loop
trials and decay between outer iterations, yielding the global
convergence guarantee.

### cartan::ik::cmaes

```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class cmaes;
```

argmin-backed Covariance Matrix Adaptation Evolution Strategy.
Derivative-free, population-based; useful when the analytical gradient
is unavailable or unreliable.

### cartan::ik::augmented_lagrangian

```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class augmented_lagrangian;
```

argmin-backed augmented Lagrangian solver for constrained IK.

## restart_wrapper

```cpp
namespace cartan::ik {

template <chain Chain,
          typename InnerPolicy = projected_lm<Chain>,
          typename LimitsPolicy = typename InnerPolicy::limits_type>
class restart_wrapper;

}
```

Restart wrapper around any inner policy satisfying `solve_policy`. When
the inner policy reports `stalled`, `diverged`, or `iteration_limit`, the
wrapper generates a new seed configuration from a Halton sequence and
re-initializes the inner policy. The best damping parameter (lambda)
from near-miss attempts is preserved across restarts for warm-starting
(when the inner policy supports `set_lambda()`/`lambda()`). Budgets via
the work-unit contract: the restart event itself charges zero additional
units beyond the inner attempt that triggered it; only the underlying
iterations bill.

Note: `projected_lm` carries a built-in self-restart fold-in, so the
recommended composition for the speed family is the bare `projected_lm`
rather than `restart_wrapper<projected_lm>`. Wrapping is still useful
for non-self-restarting inner policies (e.g. `lbfgsb`).

Reference: Beeson & Ames, "TRAC-IK", 2015 (multi-start strategy).

## Exhaustive Enumeration

### exhaustive_ik_runner

```cpp
template <chain Chain, typename Policy>
    requires cartan::ik::solve_policy<Policy>
class exhaustive_ik_runner;
```

Collects all valid IK solutions for a given target via multi-start
enumeration, deduplicating by joint-space proximity.

### exhaustive_options

```cpp
template <typename Scalar = double>
struct exhaustive_options
{
    int max_restarts{100};
    Scalar dedup_tolerance{Scalar(1e-3)};
    ranking_strategy ranking{ranking_strategy::distance_to_seed};
};
```

- `max_restarts` — Halton-seed restart budget.
- `dedup_tolerance` — joint-space distance threshold for deduplication.
- `ranking` — how to order the deduplicated solutions.

### exhaustive_result

```cpp
template <typename Scalar = double, int N = dynamic>
struct exhaustive_result
{
    std::vector<ik_result<Scalar, N>> solutions;
    int restarts_attempted{};
    int solutions_before_dedup{};
    int fk_validations_failed{};
};
```

### ranking_strategy

```cpp
enum class ranking_strategy
{
    distance_to_seed,
    min_error,
    mid_range
};
```

Sorts the deduplicated `solutions`:

- `distance_to_seed` — nearest-first by joint-space distance to the seed.
- `min_error` — lowest residual error first.
- `mid_range` — distance from joint-limit midpoints first.

## See also

- [Analytical IK](analytical.md) — closed-form solvers for analytic-friendly
  mechanisms (planar 2R, spatial 3R, Pieper-geometry 6R).
- [Kinematics](kinematics.md) — forward kinematics and Jacobians that the
  IK runners consume.
- [Background: IK Methods](../background/ik-methods.md) — theory survey.
- [Guide: IK Composition](../guides/ik-composition.md) — task-oriented
  walkthrough of stepper, scheduler, and solver composition.
