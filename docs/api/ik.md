# ik

Headers under `cartan/serial/ik/` (sublib `cartan-serial-chain`). Iterative IK
solvers, runners, status / result types, and convergence criteria all live in
the `cartan::` namespace. Link with `cartan::serial-chain`
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
| `cartan::solve_policy` concept, `cartan::step_one` | `#include <cartan/serial/ik/concepts/solve_concept.h>` |
| `cartan::no_limits`, `cartan::clamp_limits`, `cartan::null_space_limits` | `#include <cartan/serial/ik/policy/limits_policy.h>` |
| `cartan::error_weight` | `#include <cartan/serial/ik/policy/error_weight.h>` |
| `cartan::lm` (alias for `builtin_lm`) | `#include <cartan/serial/ik/solver/lm.h>` |
| `cartan::lbfgsb` (alias for `builtin_lbfgsb`) | `#include <cartan/serial/ik/solver/lbfgsb.h>` |
| `cartan::projected_lm`, `cartan::dls`, `cartan::newton_raphson` | `#include <cartan/serial/ik/solver/{projected_lm,dls,newton_raphson}.h>` |
| argmin-backed: `cartan::argmin_lm`, `argmin_lbfgsb`, `argmin_slsqp`, `argmin_bobyqa`, `argmin_projected_gn`, `argmin_projected_gradient_gn` | `#include <cartan/serial/ik/solver/argmin_*.h>` (requires `CARTAN_HAS_ARGMIN`) |
| NLopt-backed: `cartan_examples::nlopt_slsqp`, `cartan_examples::nlopt_bobyqa` | carried as a solve-policy example, not library surface: see `examples/nlopt_policy/` |
| SQP family: `cartan::nw_sqp`, `filter_nw_sqp`, `filter_slsqp` | `#include <cartan/serial/ik/solver/{nw_sqp,filter_nw_sqp,filter_slsqp}.h>` |
| MMA / GCMMA / CMA-ES / aug. Lagrangian: `cartan::mma`, `gcmma`, `cmaes`, `augmented_lagrangian` | `#include <cartan/serial/ik/solver/{mma,gcmma,cmaes,augmented_lagrangian}.h>` |
| `cartan::restart_wrapper` | `#include <cartan/serial/ik/wrapper/restart_wrapper.h>` |
| `cartan::exhaustive_ik_runner`, `cartan::exhaustive_options`, `cartan::exhaustive_result`, `cartan::ranking_strategy` | `#include <cartan/serial/ik/solver/exhaustive_ik_runner.h>` |
| `cartan::verify_solution`, `cartan::filter_valid_solutions` | `#include <cartan/serial/ik/ik_validation.h>` |
| Type aliases + builders: `speed_ik_runner`, `robust_ik_runner`, `dual_ik_runner`, `make_solver`, `make_speed_ik_runner`, `make_robust_ik_runner`, `make_dual_ik_runner` | `#include <cartan/serial/ik/solvers.h>` |

## Quick Start

Minimal working example on a 3-DOF planar arm using the LM policy. This is a
fragment, not a program, so it has nowhere to return a failure to and unwraps
`joint_limits::make` and `forward_kinematics` with `.value()`. That accessor
throws `bad_expected_access` carrying the failure, or fail-stops on the
exceptions-off targets cartan supports — either way it is not the form to copy.
Branch on the result and report through `cartan::message` instead, as the
complete example in the
[IK composition guide](../guides/ik-composition.md#complete-example) does.

<!-- cartan:unbuilt kind=illustration reason="carries its own include directives above the statements, which a fragment wrapper cannot host inside a function" -->
```cpp
#include <cartan/serial_chain.h>
#include <numbers>

cartan::vector3<double> z{0, 0, 1};
auto s1 = cartan::screw_axis<double>::revolute(z, {0, 0, 0});
auto s2 = cartan::screw_axis<double>::revolute(z, {1, 0, 0});
auto s3 = cartan::screw_axis<double>::revolute(z, {2, 0, 0});
auto home = cartan::se3<double>(cartan::so3<double>::identity(), {3, 0, 0});
auto lim = cartan::joint_limits<double>::make(-std::numbers::pi, std::numbers::pi).value();
cartan::kinematic_chain<double, 3> chain(home, {s1, s2, s3}, {lim, lim, lim});

Eigen::Vector3d q_known{0.3, -0.5, 0.2};
auto target = cartan::forward_kinematics(chain, q_known).value().end_effector;

Eigen::Vector3d q0{0.0, 0.0, 0.0};
cartan::convergence_criteria<double> criteria{1e-6, 1e-6, 100, 200};

cartan::basic_ik_runner<cartan::lm<cartan::kinematic_chain<double, 3>>> solver;
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

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <typename... Policies>
    requires (sizeof...(Policies) >= 1)
          && (cartan::solve_policy<Policies> && ...)
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

<!-- cartan:unbuilt kind=declaration -->
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

<!-- cartan:unbuilt kind=declaration -->
```cpp
ik_status step();
```

Execute one solver step. For single-policy: drives the inner policy for
one algorithmic work unit. For multi-policy: one round-robin across all
active (non-parked) policies.

### step_n

<!-- cartan:unbuilt kind=declaration -->
```cpp
ik_status step_n(int n);
```

Execute `n` round-robin rounds, stopping early on terminal status.

### solve

<!-- cartan:unbuilt kind=declaration -->
```cpp
cartan::expected<ik_result<scalar_type, joints>, ik_error<scalar_type, joints>>
solve();
```

Convenience method: drives the runner until convergence or the work-unit
cap is hit. `step()`, `step_n()` and `solve()` share one charging path, so
all three accumulate `units_consumed` against
`convergence_criteria::max_total_work_units`, stop once it is spent, and
latch a terminal status before returning. A multi-policy round-robin tick
is atomic and bills the sum of its policies' units, so the last tick can
carry the accumulator past the cap by at most one unit per still-active
policy.

### Query methods

<!-- cartan:unbuilt kind=declaration -->
```cpp
bool converged() const;
scalar_type error_norm() const;
int iterations() const;
const position_type& current_q() const;
ik_status status() const;
void abort();
```

`abort()` is terminal for the solve it interrupts. It latches `aborted`, every
policy observes that at its next step boundary and consumes no further work, and
a `solve()` afterwards fails with `ik_failure::aborted`. To run again, call
`setup()` afresh — that is what clears the abort; there is no resume.

`abort()` does not clear a `setup()` that was refused: the arguments are still
the ones that were rejected, and there is no configured policy behind them, so
the setup-failure status stands.

### Thread safety

Different runner instances may operate concurrently on the same
`const chain`. A single runner instance must not be used from multiple
threads without synchronization.

### Aliases

Defined in `<cartan/serial/ik/solvers.h>`:

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <chain Chain>
using speed_ik_runner = cartan::restart_wrapper<Chain,
    cartan::projected_lm<Chain, no_limits>, no_limits>;

template <chain Chain>
using robust_ik_runner = cartan::restart_wrapper<Chain,
    cartan::builtin_lbfgsb<Chain, no_limits>, no_limits>;

template <chain Chain>
using dual_ik_runner = basic_ik_runner<
    speed_ik_runner<Chain>, robust_ik_runner<Chain>>;
```

`speed_ik_runner` wraps `projected_lm` in a restart loop for fast
per-iteration convergence with multi-start fallback. `robust_ik_runner`
wraps L-BFGS-B for robust convergence on stiff problems. `dual_ik_runner`
races the two via `basic_ik_runner`.

### Builders

<!-- cartan:unbuilt kind=declaration -->
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

<!-- cartan:unbuilt kind=sketch reason="names a chain type the page never defines, so it shows the builder's shape" -->
```cpp
auto solver = cartan::make_solver<MyChain>()
    .policy(cartan::lm<MyChain>{})
    .policy(cartan::dls<MyChain>{})
    .build();
```

## solve_policy concept

<!-- cartan:unbuilt kind=declaration -->
```cpp
namespace cartan {

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

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <typename S>
    requires cartan::solve_policy<S>
auto step_one(S& s, const typename S::chain_type& chain);
```

Ergonomic single-unit helper. Drives the solver for one algorithmic-work
unit. Primary callers are enumeration drivers (e.g.
`exhaustive_ik_runner`), tests, and debug observers that want
step-by-step visibility into solver progress.

## convergence_criteria

<!-- cartan:unbuilt kind=declaration -->
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

<!-- cartan:unbuilt kind=declaration -->
```cpp
enum class ik_status
{
    running,
    converged,
    diverged,
    stalled,
    joint_limit_hit,
    iteration_limit,
    aborted,
    not_initialized,
    dimension_mismatch,
    non_finite_input,
    unsupported_configuration
};

constexpr const char* message(ik_status status);
```

Stepper-level control flow signal returned by `step()` calls.

The last four are terminal before any iteration runs. Every solve policy starts
in `not_initialized`, so stepping one that was never set up performs no iteration
and consumes no work units instead of reading a default-constructed joint
vector, and every policy's work loop refuses to run from a latched terminal
status.

`setup()` returns `void`, so it reports a rejected seed or target by latching
`dimension_mismatch` or `non_finite_input`, and `basic_ik_runner` reports a
selection it cannot rank by latching `unsupported_configuration`. Every solve policy validates its
arguments this way, as do `basic_ik_runner`, `restart_wrapper` and
`exhaustive_ik_runner`; the policies that require the optional numeric backend
are no exception, so a solve driven straight through one of them, rather than
through a runner or the wrapper, is checked exactly as the others are. Because
the chain is a parameter of `step()` and not only of `setup()`, each policy also
records the setup-time joint count and refuses a `step()` whose chain does not
match it. `basic_ik_runner` maps a latched status onto the same-named
`ik_failure` reason.

`message()` returns a static diagnostic string; it allocates nothing.

### ik_termination_reason

<!-- cartan:unbuilt kind=declaration -->
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

<!-- cartan:unbuilt kind=declaration -->
```cpp
enum class ik_objective
{
    speed,
    min_error_norm,
    min_joint_distance,
    max_manipulability,
    max_isotropy
};
```

Controls multi-policy racing selection. `speed` stops at the first
converging policy; the other objectives keep all policies running and
select the best converged result by min error norm, min joint-space
displacement from the seed configuration, max manipulability
(`product of singular values`), or max isotropy (`sigma_min / sigma_max`).

Each objective has one definition, read by both the single-policy and the
racing selection paths. The two Jacobian measures divide the body Jacobian's
linear rows by `solver_options::characteristic_length` before the
decomposition, so the singular values are commensurable; both are undefined on
a chain with no joints, and `setup()` refuses that combination rather than
ranking on a fabricated value. `min_joint_distance` measures a Euclidean
displacement, so it is refused on a chain mixing revolute and prismatic joints,
where the components carry different units.

The winning candidate's metric is reported on `ik_result::selection_metric`
together with the objective it was computed under. Under `speed` nothing is
ranked and the metric is absent.

### ik_failure

<!-- cartan:unbuilt kind=declaration -->
```cpp
enum class ik_failure
{
    unreachable,
    diverged,
    stalled,
    iteration_limit,
    joint_limit_violation,
    aborted,
    not_initialized,
    dimension_mismatch,
    non_finite_input,
    unsupported_configuration
};

constexpr const char* message(ik_failure failure);
```

Failure reason reported in `ik_error`. The last four name a solve that was
refused before it ran: `solve()` without a preceding `setup()`, a seed whose
length differs from the chain's joint count, a seed or target holding a NaN or
an infinity, and a selection objective the chain or the characteristic length
leaves undefined.

`message()` returns a static diagnostic string; it allocates nothing.

### step_metrics

<!-- cartan:unbuilt kind=declaration -->
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

<!-- cartan:unbuilt kind=declaration -->
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

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <typename Scalar = double>
struct solver_options
{
    ik_objective objective{ik_objective::speed};
    unsigned int halton_seed{42};
    Scalar characteristic_length{1};
};
```

Controls multi-policy racing behavior: the racing objective, the Halton seed
offset for reproducible secondary-policy seeding, and the characteristic length
the two Jacobian objectives normalize by. The total work budget lives on
`convergence_criteria::max_total_work_units` and bounds the racing loop as well.

`characteristic_length` is in the chain's linear unit and applies to the
selection objectives alone; it is not a library-wide scale. Its default of one
reproduces the unnormalized arithmetic exactly, so it states the unit scale
those measures always assumed rather than changing any ranking. A zero,
negative or non-finite value is refused at `setup()`.

### ik_result

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <typename Scalar = double, int N = dynamic>
struct ik_result
{
    joint_state<Scalar, N> solution;
    Scalar final_error_norm{};
    int iterations{};
    int solver_index{};
    std::optional<Scalar> selection_metric{};
    ik_objective selection_objective{ik_objective::speed};
};
```

Successful IK outcome. `solver_index` identifies which policy produced
the solution in multi-policy racing. `selection_metric` is the value that
candidate was ranked on under `selection_objective`, and is absent where the
objective ranks nothing.

### ik_error

<!-- cartan:unbuilt kind=declaration -->
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

<!-- cartan:unbuilt kind=declaration -->
```cpp
struct no_limits;
```

No-op: applies no enforcement. Use when the policy handles constraints
internally (e.g., `projected_lm`, argmin policies with box
constraints). Default for `cartan::lm` and `cartan::projected_lm`
— the LM trust-region family for which post-step clamping would
invalidate the trust-region step.
**Critical**: post-step `clamp_limits` invalidates LM trust-region
semantics; `no_limits` is the correct default for the LM family.

### clamp_limits

<!-- cartan:unbuilt kind=declaration -->
```cpp
struct clamp_limits;
```

Hard clamping: clamps each `q(i)` to `[position_min, position_max]`.
Simple and robust, but may cause discontinuities at boundaries. Default
for the native `lbfgsb`, and for the argmin policies that already
enforce box constraints internally (`argmin_slsqp`, `argmin_bobyqa`,
the argmin policies).

### null_space_limits

<!-- cartan:unbuilt kind=declaration -->
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

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <typename Scalar = double>
struct error_weight
{
    vector6<Scalar> weights{vector6<Scalar>::Ones()};

    vector6<Scalar> apply(const vector6<Scalar>& v) const;
};
```

Per-component weight on the 6-vector pose error. Position and
orientation components can be weighted independently for tasks where
one dominates the other. The default (all-ones) gives equal weight to
all components.

A weight is supplied through the five-argument `setup()` overload, which
`newton_raphson`, `lbfgsb` and `projected_lm` provide; those three apply it
throughout their step mathematics. Policies without that overload do not
accept a weight, and `restart_wrapper` offers the overload only when its
inner policy does, so passing one where it cannot be honored fails to
compile rather than being ignored. The weight steers the step; the
convergence gate always reads the raw component norms.

## Solvers

### cartan::lm

<!-- cartan:unbuilt kind=declaration -->
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

### cartan::lbfgsb

<!-- cartan:unbuilt kind=declaration -->
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

### cartan::projected_lm

<!-- cartan:unbuilt kind=declaration -->
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

### cartan::dls

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class dls;
```

Damped Least Squares with SVD-based adaptive damping (Nakamura).
Body-frame Newton-Raphson iteration where the damping factor increases
as the smallest singular value drops below a threshold.

### cartan::newton_raphson

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class newton_raphson;
```

Newton-Raphson IK with undamped Gauss-Newton Hessian and backtracking
Armijo line search for globalization. Separate angular and linear
convergence tolerances.

Reference: Nocedal & Wright, *Numerical Optimization*, Ch. 3 (line
           search), Ch. 10 (nonlinear least squares, Gauss-Newton).

### cartan::argmin_lm

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <chain Chain, typename LimitsPolicy = no_limits>
class argmin_lm;
```

argmin-backed Levenberg-Marquardt with the least-squares adapter
exposing the 6-element body-frame error as residuals and the body
Jacobian. Joint limits are enforced via clamping after each step since
LM is unconstrained.

### cartan::argmin_lbfgsb

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class argmin_lbfgsb;
```

argmin-backed L-BFGS-B for bound-constrained IK using the analytical
gradient via the SE(3) log Jacobian.

Reference: Byrd, Lu, Nocedal, Zhu (1995).

### cartan::argmin_slsqp

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <chain Chain,
          typename LimitsPolicy = clamp_limits,
          typename Convergence = argmin::default_convergence>
class argmin_slsqp;
```

argmin-backed SLSQP for constrained IK with box constraints. Uses
Kraft's Sequential Least Squares Programming algorithm with the
analytical gradient via the SE(3) log Jacobian. The `Convergence`
template parameter lets consumers opt out of argmin's default
four-criterion convergence policy in favor of alternatives like
`argmin::slsqp_compatible_convergence` (NLopt-style ftol+xtol+stall).

### cartan::argmin_bobyqa

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class argmin_bobyqa;
```

argmin-backed BOBYQA derivative-free solver with box constraints.
Builds a quadratic interpolation model of the objective and uses
trust-region steps.

Reference: Powell, M.J.D., "The BOBYQA Algorithm for Bound Constrained
           Optimization Without Derivatives", 2009.

### cartan::argmin_projected_gn

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class argmin_projected_gn;
```

argmin-backed projected Gauss-Newton with active-set bounds.

### cartan::argmin_projected_gradient_gn

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class argmin_projected_gradient_gn;
```

argmin-backed projected-gradient Gauss-Newton with Armijo backtracking.

### cartan::nw_sqp

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class nw_sqp;
```

argmin-backed Nocedal-Wright SQP with inequality constraints.

Reference: Nocedal & Wright, *Numerical Optimization*, Ch. 18 (SQP).

### cartan::filter_nw_sqp

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class filter_nw_sqp;
```

argmin-backed filter Nocedal-Wright SQP.

### cartan::filter_slsqp

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class filter_slsqp;
```

argmin-backed filter SLSQP with box constraints.

### cartan::mma

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class mma;
```

argmin-backed Method of Moving Asymptotes.

### cartan::gcmma

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class gcmma;
```

argmin-backed Globally Convergent MMA. Extends MMA with per-component
conservativity coefficients that grow on non-conservative inner-loop
trials and decay between outer iterations, yielding the global
convergence guarantee.

### cartan::cmaes

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class cmaes;
```

argmin-backed Covariance Matrix Adaptation Evolution Strategy.
Derivative-free, population-based; useful when the analytical gradient
is unavailable or unreliable.

### cartan::augmented_lagrangian

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <chain Chain, typename LimitsPolicy = clamp_limits>
class augmented_lagrangian;
```

argmin-backed augmented Lagrangian solver for constrained IK.

## restart_wrapper

<!-- cartan:unbuilt kind=declaration -->
```cpp
namespace cartan {

template <chain Chain,
          typename InnerPolicy = projected_lm<Chain>,
          typename LimitsPolicy = typename InnerPolicy::limits_type>
class restart_wrapper;

}
```

Restart wrapper around any inner policy satisfying `solve_policy`. When
the inner policy reports `stalled`, `diverged`, or `iteration_limit`, the
wrapper generates a new seed configuration from a Halton sequence and
re-initializes the inner policy. A rejected seed or target is not one of those:
every `setup()` overload validates its arguments and latches the terminal status
in the wrapper, so `step()` returns `dimension_mismatch` or `non_finite_input`
unchanged without consuming a restart, and `restarts()` stays at zero. A later
well-formed `setup()` clears the latch, so a wrapper that refused one call is
still usable. `converged()`, `solution()` and `error_norm()` read through the
same latch: a refused setup ran no attempt, so they report `false`, a zero
configuration and `numeric_limits<scalar_type>::max()` rather than the previous
solve's answer. The best damping parameter (lambda)
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

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <chain Chain, typename Policy>
    requires cartan::solve_policy<Policy>
class exhaustive_ik_runner;
```

Collects all valid IK solutions for a given target via multi-start
enumeration, deduplicating by joint-space proximity.

### exhaustive_options

<!-- cartan:unbuilt kind=declaration -->
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

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <typename Scalar = double, int N = dynamic>
struct exhaustive_result
{
    std::vector<ik_result<Scalar, N>> solutions;
    std::optional<ik_failure> failure{};
    int restarts_attempted{};
    int solutions_before_dedup{};
    int fk_validations_failed{};
};
```

`failure` is engaged only when the enumeration was refused before it could run:
a seed of the wrong length, or a nonfinite seed or target. The runner returns at
once in that case rather than working through the remaining seeds, so
`restarts_attempted` is 1. An enumeration that ran and found nothing reports an
empty `solutions` with no `failure`, which is a different answer.

### ranking_strategy

<!-- cartan:unbuilt kind=declaration -->
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

## Validation

FK-based validation free functions for IK results. Both live in
`namespace cartan` and are reachable transitively via
`<cartan/serial/ik.h>` and `<cartan/serial_chain.h>`.

### verify_solution

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <chain Chain>
bool verify_solution(
    const Chain& chain,
    const se3<typename Chain::scalar_type>& target,
    const typename joint_state<typename Chain::scalar_type, Chain::joints>::position_type& q,
    const convergence_criteria<typename Chain::scalar_type>& criteria);
```

Recomputes forward kinematics at `q` through the checked entry point, takes the
body-frame log of `fk.end_effector.inverse() * target`, and returns `true` iff
the orientation and position components are both below the corresponding
tolerances in `criteria`. A `q` the boundary refuses -- wrong length, or holding
a NaN or an infinity -- is not verified: the function returns `false`. Used by `exhaustive_ik_runner` and by callers
building custom multi-start drivers that need an explicit FK back-check.

### filter_valid_solutions

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <chain Chain, typename Scalar, int N>
std::vector<ik_result<Scalar, N>> filter_valid_solutions(
    const Chain& chain,
    const se3<Scalar>& target,
    std::vector<ik_result<Scalar, N>> solutions,
    const convergence_criteria<Scalar>& criteria);
```

Drops every `ik_result` whose `solution.position` fails `verify_solution`
against the same `criteria`. Useful as a final pass over the output of
`exhaustive_ik_runner` or any other multi-solution driver that collected
candidates without an inline FK back-check.

## See also

- [Analytical IK](analytical.md) — closed-form solvers for analytic-friendly
  mechanisms (planar 2R, spatial 3R, Pieper-geometry 6R).
- [Kinematics](kinematics.md) — forward kinematics and Jacobians that the
  IK runners consume.
- [Background: IK Methods](../background/ik-methods.md) — theory survey.
- [Guide: IK Composition](../guides/ik-composition.md) — task-oriented
  walkthrough of policy, runner, and restart-wrapper composition, including
  variadic-policy racing via `basic_ik_runner` and multi-start via
  `restart_wrapper`.
