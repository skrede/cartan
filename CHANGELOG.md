# Changelog

All notable user-facing changes to this project are documented in this file.

The format is based on [Keep a Changelog 1.1.0](https://keepachangelog.com/en/1.1.0/),
and the project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- `cartan::lie_failure::non_finite_input`, reported by `so2`/`so3`/`se2`/`se3`
  `from_matrix`, `so3::from_quaternion`, the frame-tagged `rotation`/`transform`
  wrappers and `screw_axis::from_vector` when an input component is NaN or
  infinite. Each factory now tests finiteness on its raw input before any other
  check, so a nonfinite value is refused rather than admitted by a tolerance
  comparison that is false for a NaN.

### Changed
- **Breaking.** `ik_objective::min_distance` is removed and replaced by
  `ik_objective::min_error_norm`. The old name promised a distance and selected
  on the pose residual; the name was the untruth, so the name changed and the
  behavior did not. There is no alias and no deprecation path: C++ code naming
  the old value fails to compile, and Python code naming
  `IkObjective.min_distance` fails at attribute lookup rather than at a type
  check. `ik_objective::min_joint_distance` is added and supplies the capability
  the removed name promised — the Euclidean displacement from the seed
  configuration, which the runner now stores at `setup()`. Because it is a
  Euclidean norm over joint coordinates, it is refused on a chain mixing
  revolute and prismatic joints, whose components carry different units; a
  caller-supplied scale for that case may be added later, and refusing now
  forecloses nothing.
- **Breaking.** `ik_objective::max_manipulability` and
  `ik_objective::max_isotropy` were documented as the racing selection's
  criteria but were never implemented on the racing path: it ranked every
  objective on the stored error norm and silently returned the lowest-residual
  candidate under all three. Both measures now have one definition, read by the
  single-policy and the racing paths alike, so the two agree by construction
  rather than by two implementations happening to coincide. On a nine-joint
  measurement fixture the three objectives previously returned the same
  candidate; they now return the candidate each objective actually ranks
  highest.
- **Breaking.** Both Jacobian measures are undefined, rather than one or zero,
  where the decomposition has no singular values. A chain with no joints
  previously drove a product over an empty set, which evaluates to one and would
  have reported such a chain as maximally manipulable; in practice it faulted
  inside the decomposition's own construction before reaching that value.
  `setup()` now refuses the combination. `ik_status` and `ik_failure` each gain
  `unsupported_configuration`, so a switch over either that was previously
  exhaustive needs one more arm.
- `solver_options` gains `characteristic_length`, in the chain's linear unit,
  which divides the body Jacobian's linear rows before the decomposition so the
  singular values are commensurable with the angular rows. It applies to the
  selection objectives alone and is not a library-wide scale. Its default of one
  reproduces the previous arithmetic exactly, so nothing is reranked by adopting
  it; a zero, negative or non-finite value is refused at `setup()`. Note that on
  a square body Jacobian the length cannot reorder the manipulability measure at
  all — it rescales every candidate by the same factor — though it does reorder
  the isotropy measure, and reorders manipulability on a chain with fewer than
  six joints.
- `ik_result` gains `selection_metric` and `selection_objective`, the value the
  winning candidate was ranked on and the objective it was computed under. The
  metric is absent under `speed`, which ranks nothing, so an unranked win reads
  as absent rather than as a zero.
- **Breaking.** A single-policy solve under a non-`speed` objective now performs
  a real multistart. It previously re-seeded its policy at the configuration it
  had just converged to, which converged again immediately for zero work and
  tripped the budget guard: one start dressed as a multistart. Measured on a
  six-joint fixture with a three-thousand-unit budget, the old path spent five
  units; the new one spends the budget it was given and returns a better-ranked
  configuration. Callers who relied on the old early return will see such a
  solve consume its stated budget.
- **Breaking.** `abort()` is now terminal for the solve it interrupts. The runner
  previously assigned itself `running` immediately after aborting its policies,
  so `status()` reported `running` and the caller's instruction left no trace on
  the runner at all; with a policy whose `abort()` body was empty — `dls`, `lm`,
  `newton_raphson`, `lbfgsb` — the abort was ignored outright and the whole
  `solve()` that followed still returned a solution. Every other policy did stop,
  but reported the abort as `ik_status::stalled`, indistinguishable from a solve
  that ran out of progress on its own. `abort()` now latches a new
  `ik_status::aborted`; every policy, the restart wrapper and the runner report
  that one state; a step taken after an abort consumes no work; and a `solve()`
  after an abort fails with `ik_failure::aborted`, a reason that was declared,
  given a message and bound to Python but never produced until now. There is no
  resume: code that aborted and then re-solved must call `setup()` again, which
  is what clears the abort. `ik_status` gains a value, so a switch over it that
  was previously exhaustive now needs an `aborted` arm. There is no deprecation
  path and no compatibility shim.
- **Breaking.** `basic_ik_runner::step()` and `step_n()` now charge the work they
  consume against `convergence_criteria::max_total_work_units` and stop once it
  is spent. Previously neither touched the runner's accumulator and neither read
  the budget, so a hand-driven caller ran without bound while `iterations()`
  reported zero and `status()` stayed `running`: five thousand `step()` calls
  against a five-unit budget reported no work at all. `step()`, `step_n()` and
  `solve()` now share one charging path and report the same iteration count, the
  same terminal status and the same error norm from the same setup. Code that
  drove `step()` past the budget and relied on it continuing must raise
  `max_total_work_units`; there is no deprecation path and no compatibility
  shim.
- **Breaking.** `solve()` latches a terminal status before it returns. It
  previously left the runner `running` after exhausting the budget, which made
  the returned failure reason a fall-through default rather than a report of
  what happened, and left `status()` contradicting the returned error. A caller
  that read `status()` after `solve()` and treated `running` as "resumable" now
  sees `converged` or `iteration_limit`.
- **Breaking.** `solver_options::max_total_iterations` is removed, along with the
  `max_total_iterations` keyword argument and field on the Python `IkConfig`.
  Multi-policy racing counted round-robin ticks against this separate cap while
  single-policy solves counted algorithmic work units against
  `max_total_work_units`, and `ik_result::iterations` reported whichever the
  solve happened to produce — one field with two incompatible meanings. Racing
  now bills the work units its policies consume against `max_total_work_units`
  like every other path, so `iterations` means work units everywhere. A racing
  round-robin tick is atomic and can carry the accumulator past the cap by at
  most one unit per still-active policy. Express a racing bound in work units:
  the previous default of 500 ticks over two policies is roughly 1000 units, not
  the 200-unit default of `max_total_work_units`, so a racing solve left on the
  defaults now does less work and may fail where it previously converged. There
  is no deprecation path and no compatibility shim.
- **Behavior change.** `projected_lm` applies the task weight it is given. The
  five-argument `setup` stored an `error_weight` and no part of the step
  mathematics read it, so two weights spanning six orders of magnitude in
  opposite directions produced first steps identical to the last mantissa bit.
  The weight now scales the residual and the Jacobian's rows, and through them
  the normal matrix, the gradient, both trust-region reductions, the dogleg
  Cauchy scale, the damping initialization and the stall detector. The reported
  `error_norm()` keeps its unweighted meaning, so accuracy figures stay in the
  units they were in. Every solve reached through the four-argument `setup` runs
  at the identity weight and is bit-identical to before.
- **Breaking.** `restart_wrapper`'s weighted `setup` overload exists only when
  the wrapped policy has one. It previously accepted an `error_weight` for any
  inner policy and dropped it silently when the inner policy could not take one;
  passing a weight to such a wrapper is now a compile error naming the
  unsatisfied constraint. `cartan::detail::is_converged` — a weighted
  convergence check no solver called — is removed along with
  `error_weight::weighted_angular_norm` and `error_weight::weighted_linear_norm`,
  which were reachable only through it. Ten backend-adapted policies
  (`argmin_bobyqa`, `argmin_lbfgsb`, `argmin_slsqp`, `augmented_lagrangian`,
  `cmaes`, `filter_nw_sqp`, `filter_slsqp`, `gcmma`, `mma`, `nw_sqp`) no longer
  carry a weight member; none was settable, so each was permanently the identity.
- **Behavior change.** `newton_raphson` keeps its current configuration when the
  line search accepts no step length, and reports `stalled` on the spot. It
  previously committed the last rejected trial regardless, which raised the error
  on 58 of 60 steps on a bounded chain, and its sufficient-decrease test measured
  the raw direction scaled by the step length rather than the displacement the
  joint-box projection actually produced. A bare policy that used to keep
  stepping now stops; a restart-wrapped one reseeds instead of spending its whole
  budget drifting, which is where the behavior improves most.
- **Behavior change.** A joint with one infinite bound gets a substituted search
  interval anchored to its finite side. The two consumers that need an absolute
  coordinate — the constrained-problem adapter and the multistart seed generator
  — previously took half the fallback width as a coordinate, so a `[10, +inf)`
  joint was given `[10, 6.283]`, an empty interval, and every multistart seed for
  it landed below its lower bound. A fully unbounded joint is now centered on a
  caller-supplied reference configuration instead of on zero, and that reference
  is a required constructor argument at both consumers. `unwrap_to_range_nearest`
  with both bounds infinite returns the equivalent angle nearest its reference
  rather than the input unchanged. Chains with finite bounds are unaffected.
- **Behavior change.** Inputs that were previously accepted are now rejected, and
  some that were rejected now report a different code. A NaN was accepted by all
  of the factories above; an infinity was accepted by `so3::from_matrix` outside
  the (0,0) entry and by the `se2`/`se3` translation blocks. Where a nonfinite
  input already produced a failure, the code changes to `non_finite_input`:
  `so2::from_matrix` reported `non_orthogonal` or `improper_rotation` depending on
  the position, `se2`/`se3::from_matrix` reported `invalid_affine_row`,
  `so3::from_quaternion` reported `non_unit_quaternion`, and
  `screw_axis::from_vector` reported `non_unit_screw_axis`. Code matching on the
  old value for a nonfinite input needs updating; matching for finite input is
  unaffected.
- `screw_axis::from_vector` no longer classifies an axis whose angular part holds
  a NaN as a prismatic joint. `screw_axis::revolute` and `screw_axis::prismatic`
  are unchanged and still normalize without validating.
- **Breaking.** `static_chain` is now constructed through
  `static_chain<Scalar, Joints...>::make(home, axes, limits)`, which returns
  `cartan::expected<static_chain, chain_failure>`; the constructor is private.
  `make` reports `non_finite_input` for a nonfinite home pose or screw axis, and
  `tag_axis_contradiction` for an axis that is not the principal axis its
  compile-time tag names. Both were previously a debug-only assert, which meant
  a release build constructed such a chain silently and evaluated a robot with
  one immovable joint. The static predicate `axes_match_tags` is removed.
- **Behavior change.** The tag-versus-axis comparison is **exact**, with no
  tolerance, so an axis taken from a parsed description may be refused where it
  was previously accepted: composing a URDF's `<origin rpy>` rotations leaves
  residue in the axis components — the vendored UR descriptions give
  `(0, 1, -2.05103e-10)` for a nominally `+y` joint. `detect_joint_kind` still
  snaps such an axis and `kinematic_chain` is unaffected; only `static_chain`
  construction is stricter. Round the axis onto its principal direction before
  building a `static_chain` from a description, or use `kinematic_chain`. Either
  sign of a principal axis is accepted, so a joint given as `axis="0 0 -1"`
  remains expressible under `revolute_z`.
- `detect_joint_kind` compares against an absolute per-component tolerance of
  `1e-9` rather than one derived from machine precision. The previous threshold
  was `sqrt(epsilon)`: `1.5e-8` in `double` but `3.4e-4` in `float`, so the same
  code silently discarded a misalignment of about a hundredth of a degree in
  single precision. Chains from `load_urdf<float>` now classify as `general` and
  take the generic evaluation path, which evaluates the true axis and is the
  more faithful of the two; a `double` parse is unaffected.
- **Breaking.** The closed-form solvers are constructed through a validating
  factory: `planar_2r_solver::make`, `spatial_3r_solver::make` and
  `pieper_6r_solver::make`, each returning
  `cartan::expected<solver, analytical_error<Scalar>>`. The public constructors
  and the deduction guides are removed, and `solve_2r`, `solve_3r` and
  `solve_pieper_6r` — in C++ and in Python — route through the factory. Each
  factory rejects the preconditions its derivation consumes rather than letting
  a chain that violates them construct and then fail every solve: a planar 2R
  whose axes are not parallel, or whose home end-effector is off the plane; a
  spatial 3R whose shoulder axes do not intersect; a six-axis chain that is not
  Pieper. A rejected chain now reports a typed reason at construction, where
  previously it constructed silently and every solve returned `unreachable`
  carrying a length no reach inequality had compared against.
- **Breaking.** `analytical_error<Scalar>`'s magnitude is
  `std::optional<Scalar>`. Every failure whose contract carries no magnitude now
  says so explicitly instead of reporting a zero or a stale value, and that
  absence crosses into Python as `None`. Where a magnitude is reported it is now
  the deficit read off the inequality that actually failed: an out-of-workspace
  Pieper 6R target whose branches were all rejected is reported as a failed
  verification rather than certified unreachable, and a reason forwarded from a
  Paden-Kahan subproblem carries no magnitude at all rather than the caller's
  substituted length.
- **Behavior change.** The Paden-Kahan subproblems reject nonfinite input and a
  non-unit axis on the error channel. `paden_kahan_3` had no input guard: every
  comparison in its body is false against a NaN, so a NaN axis, point, target or
  distance fell through to the two-solution branch and was reported as a success
  carrying two NaN angles. The single-axis rotation subproblem additionally
  enforces the axial condition and the reconstruction residual it previously
  computed and discarded, both against a relative threshold that scales with the
  mechanism rather than an absolute one that tightens as the arm grows.
- **Behavior change.** Each analytical solver verifies its candidate solutions at
  the acceptance tolerance it was constructed with. The shared forward-kinematics
  back-check previously took a default, so a solver configured at `1e-9` still
  verified at that default and a solver configured at `1e-3` rejected solutions
  it was asked to accept. Default-configured results are unchanged.
- **Behavior change.** A folded equal-link planar 2R — target at the base point,
  both links the same length — is reported as `singular_configuration` with no
  solutions, decided before the law of cosines divides by the in-plane distance.
  It previously divided by zero, produced two candidates with a not-a-number
  shoulder angle, and reported `verification_failed`. Both reach gates now
  compare lengths against the same acceptance length the back-check applies,
  rather than comparing squared distances against squared reaches.

## [0.4.1] - 2026-07-06

### Added
- Apache License 2.0 stamped at repo root; `CONTRIBUTING.md` covering issue
  filing, PR workflow, build/test instructions, coding conventions, commit
  message format, and the project's branching model.
- `cartan::version()`, `cartan::version_major()`, `cartan::version_minor()`,
  `cartan::version_patch()` runtime accessors in `cartan/version.h`, plus
  `CARTAN_VERSION_{MAJOR,MINOR,PATCH,STRING}` compile-time macros generated
  from the CMake project version.
- URDF loading module (`cartan-urdf`) — a pugixml-based URDF parser that builds
  product-of-exponentials kinematic chains, exposed through the `<cartan/urdf.h>`
  umbrella header. Gated behind `CARTAN_BUILD_URDF` (OFF by default).
- Python bindings (`cartan._core`) built with nanobind and packaged via
  scikit-build-core, exposing the `cartan` and `cartan.analytical` submodules.
  Gated behind `CARTAN_BUILD_PYTHON`.
- Embedded packaging — an `idf_component.yml` ESP-IDF component manifest at the
  repository root that declares its Eigen dependency and registers cartan's three
  public include trees, so `idf.py add-dependency "skrede/cartan"` yields a
  component that builds out of the box. A compile-only ESP32 smoke scaffold checks
  the public headers build under the Espressif xtensa and riscv32 toolchains, with
  the exceptions-disabled embedded build and the cross-compile toolchain enforced
  in continuous integration.
- Install/export layer — `find_package(cartan CONFIG REQUIRED)` now works: a
  full export set (`cartan::cartan` alongside the per-module targets), installed
  public headers, and generated `cartanConfig.cmake` / `cartanConfigVersion.cmake`
  files that re-discover Eigen3 (and pugixml for URDF-inclusive installs) through
  `find_dependency`. The interface targets declare a `cxx_std_20` compile feature,
  so a consumer building against an older standard receives a clear requirement
  error instead of a wall of header diagnostics.
- This `CHANGELOG.md` covering the 0.1.0 through 0.4.1 history.

### Changed
- Language baseline relaxed from C++23 to C++20, making C++20 the stated
  requirement across the README badge, `docs/getting-started.md`,
  `CONTRIBUTING.md`, and the package metadata. The one C++23 dependency,
  `std::expected`, is covered by an in-tree `<cartan/expected.h>` polyfill
  (`cartan::expected`), so the public headers compile on the exceptions-off
  embedded GCC backends that do not yet ship the C++23 standard library.
- README license badge and footer corrected from MIT to Apache 2.0 (the
  `LICENSE` file at repo root has always carried Apache 2.0 text; the badge
  and footer were the inconsistency).
- CMake `project(cartan VERSION ...)` is now the single source of the library
  version at `0.4.1`, with explicit `LANGUAGES CXX`. `cartan-lie` installs the
  generated `cartan/version.h` header so `find_package(cartan)` consumers can
  include it alongside the rest of the public API.
- The shared iterative-solver stall detection now keeps its recent-error window
  in a fixed-capacity ring buffer instead of a heap-backed growable container,
  removing per-iteration heap churn from the stall and divergence check. The
  retained window, and the stall and divergence decisions derived from it, are
  numerically identical to the previous unbounded history.
- `projected_lm` is now allocation-free per step on fixed-size chains: its
  active-set LDLT solve runs over max-size-fixed temporaries, so the documented
  steady-state IK path performs no heap allocation. This is a storage refactor
  only; solver outputs are bit-for-bit identical to the previous heap-backed
  implementation, and a host test proves the steady-state step loop allocates
  no heap using both an Eigen no-malloc trap and a global new/delete counter.

### Fixed
- Prismatic joint axis sign was dropped in the fast-path forward-kinematics and
  Jacobian specializations, so forward kinematics was off by `2q` for
  negative-axis prismatic joints; several latent dynamic-chain FK/Jacobian
  defects on the same path were closed alongside it.
- The thin-SVD null-space projection was wrong for dynamic redundant chains, and
  the low-discrepancy restart-seed generator read out of bounds above ten joints.
- The URDF loader is hardened against kinematic cycles and malformed or untrusted
  input instead of walking them unbounded.
- Screw-pitch parameterization was corrected for non-unit twists, matching the
  standard product-of-exponentials pitch convention.
- Analytical 6R solution output hygiene (angle wrapping, duplicate removal, and
  anti-parallel outer-wrist sign selection), plus construction-time geometry
  validation and an explicit shoulder-singularity error channel for the
  closed-form solvers.
- The first Paden-Kahan subproblem returned a not-a-number inside a success
  result when the query point lay on the rotation axis; it now reports the
  degenerate case through the error channel instead.
- Kinematic-chain axis access is now bounds-checked and chain sizes are validated
  at runtime, turning previously out-of-range access into a reported error.
- The compile-only ESP32 smoke invoked the first Paden-Kahan subproblem with the
  wrong arity and never actually built; it now calls the four-argument overload
  with a solvable rotation and compiles under the embedded toolchain.

### Removed
- The Arduino `library.properties` manifest, which advertised a `src/` source
  layout the header-only repository does not provide.

## [0.3.0] — 2026-05-13

### Added
- Stream A: per-policy algorithmic-work-unit accounting on `solve_concept`.
  Every iterative solver implements
  `step(chain, int N) -> step_result<Scalar>{ ik_status status; step_metrics{ int units_consumed; Scalar error_norm; } metrics; }`.
  The runner asks each solver for up to N units of work per cycle; the solver
  returns the actual units consumed. Wrapper-style restarts
  (`restart_wrapper`, `projected_lm`'s baked-in restart, `argmin_slsqp`
  cold-restart) charge zero for the restart event; only the underlying
  iterations bill.
- `cartan::ik::step_one(s, chain)` free helper as the ergonomic one-unit shim.
- Stream B: head-to-head closed-form vs iterative IK bench matrix on
  `(planar 2R, spatial 3R, ABB IRB120, KR6 R900)` with wall + accuracy +
  workspace-coverage cells. New `closest_to_seed` multi-solution selection
  wrapper over `analytical_result::solutions` gives apples-to-apples
  comparison vs seed-deterministic iterative solvers.
- Synthetic planar 2R (`l1=0.5`, `l2=0.4`) and spatial 3R (ZYZ, link offsets
  `0.5 / 0.3`) fixtures in `profiling/chain_factories.h` as paired
  `static_chain` / `kinematic_chain` factories.
- Per-family `*_total_units` bench-cell budget constants in
  `ik_comparison_pinocchio_benchmarks.cpp` with empirical-basis rationale
  comments.
- `tests/unit/solver_unit_count_test.cpp` regression test fan-out across the
  11 in-tree iterative solver types (43 cases / 107 assertions / 5 property
  blocks).

### Changed
- `convergence_criteria<Scalar>` split its single `max_iterations` field into
  `max_iterations_per_attempt{100}` (bounds a single solver attempt) and
  `max_total_work_units{200}` (bounds the runner-level total budget across
  restart attempts).
- `basic_ik_runner::solve()` cap is now literal — the previous `*2` slack
  factor is gone. Callers wanting restart slack set `max_total_work_units`
  higher explicitly.
- LM-family bench cells locked at `lm_family_total_units = 800` after a
  saturation-ladder walk (`lm=400/800/1600/3200`; saturation reached at 800,
  with `lm=1600` and `lm=3200` bit-identical cell-by-cell). +6.90 pp net
  Success_pct across the 36-cell LM gate vs the v0.2.0 baseline.
- Pose-batch IK bench cells gain a `multiplier_reest_every_k` knob forwarded
  to `argmin_slsqp` options.

### Fixed
- Three argmin shim restart-count regressions surfaced by the finer-grained
  `step_n(N)` granularity (`is_inner_terminal` now includes the per-attempt
  cap; restart fires before stall-detector preempts).
- `basic_ik_runner::solve()` infinite-loop guard when an inner solver returns
  `{units=0, status=converged}` on `min_distance`-objective restart.
- `cmaes` precision tolerance flake observed under the finer step granularity.

### Notes
- The v0.3.0 runner-budget refactor exposes a structural asymmetry that no
  single per-family budget value can close cell-by-cell: 5
  `builtin_lm_restart` cells saturate at 100% (positive-delta, worst
  `ur3e_builtin_lm_restart` at +4.30 pp above the pre-refactor truncated
  baseline), 4 `argmin_lm` default cells sit 0.25-0.50 pp below baseline
  (negative-delta from argmin's higher per-iter cost). Gate criterion revised
  from cell-by-cell `±0.20 pp` to net-SR-non-regression + per-cell
  negative-only `-1.00 pp` floor — a durable lesson for any future bench
  budget reform.

## [0.2.2] — 2026-04-14

### Added
- `cartan::ik::argmin_projected_gn` and `cartan::ik::argmin_projected_gradient_gn`
  wrappers over nablapp's projected Gauss-Newton policies.
- PGN entries wired into `basic_ik_full_benchmarks` (9-robot) and
  `ik_comparison_benchmarks` (UR3e head-to-head).
- `cartan::ik::mma` and `cartan::ik::gcmma` wrappers over nablapp's method of
  moving asymptotes policies.

## [0.2.1] — 2026-04-13

### Added
- `cartan::ik::` namespace with structured directory hierarchy (`solver/`,
  `wrapper/`, `concepts/`, `policy/`). Suffix-free type names
  (`cartan::ik::lm`, `cartan::ik::projected_lm`, `cartan::ik::lbfgsb`, ...).
- Dual-implementation `builtin_*` and `argmin_*` prefixes with conditional
  short-alias defaults.
- Convenience headers `<cartan/serial/ik.h>` and `<cartan/serial/fk.h>`.
- `exhaustive_ik_runner` with FK validation, joint-space dedup, and three
  ranking strategies.
- Cold-restart perturb-retry loop for `argmin_slsqp` (59.8% UR3e direct-drive
  success, up from 19.9% one-shot baseline).

### Changed
- `CARTAN_BUILD_ARGMIN` guards on all benchmark and profiling files for clean
  argmin-free builds.

## [0.2.0] — 2026-04-03

### Added
- Module rename: `cartan-kinematics` → `cartan-serial-chain` with a unified
  chain concept.
- `static_chain<Scalar, Joints...>` with compile-time joint types alongside
  the existing `kinematic_chain<Scalar, N>` runtime form.
- FK / Jacobian / IK solver generalization over the chain concept; both chain
  types satisfy a unified concept and share the entire IK stack.
- Specialized FK and Jacobian for `static_chain` that exploit compile-time
  axis knowledge for measurable speedup.
- Analytical IK module `cartan-analytical` with closed-form solvers: planar
  2R, spatial 3R, 6R Pieper (spherical-wrist), plus Paden-Kahan subproblems
  SP1/SP2/SP3 as building blocks. Multi-solution output via
  `analytical_result<Scalar, N, MaxSolutions>`.
- nablapp IK integration benchmarks across all robot geometries.

### Changed
- `liepp` → `Cartan` rename across all code, CMake, docs, and tests.
- Performance: fixed-size Eigen on hot paths; Lie algebra inner-loop tuning.

## [0.1.3] — 2026-03-31

### Added
- Variadic `basic_ik_solver<Policies...>` with cooperative racing replacing
  all scheduler types (`RacingScheduler`, `FallbackScheduler` deleted; one
  composable type via CTAD).
- Factory functions with builder-pattern materialization.
- nablapp integrated as default SLSQP/BOBYQA backend via FetchContent;
  benchmark parity with NLopt confirmed within 0.5%.
- Physical library split: `cartan-lie` (19 headers) and `cartan-kinematics`
  (35 headers, later renamed `cartan-serial-chain`) with three CMake
  INTERFACE targets and module umbrella headers.

### Changed
- Template arg reorder to `<Scalar, N>` across all 23 IK headers (enables
  CTAD, matches ctrlpp convention).
- `_stepper` → `_solve_policy` rename with CTAD deduction guides.
- Function decomposition: `projected_lm`, `lbfgsb`, and `lm` step() methods
  decomposed from 105-192 lines down to 47-75 lines via named algorithmic
  sub-functions. ~350 lines of shared detail extracted into
  `cartan/serial/ik/detail/`.

## [0.1.2] — 2026-03-29

### Added
- SE(3) left Jacobian with Taylor-stable Q matrix.
- Analytical IK gradient via right-trivialized log differential.
- Projected Levenberg-Marquardt with active-set box projection and dogleg
  trust-region.
- L-BFGS-B with generalized Cauchy point active-set identification and Armijo
  line search.
- Restart wrapper with Halton low-discrepancy seed generator and Beeson-Ames
  joint wrapping; warm-start lambda preservation across restarts.
- SLSQP and Newton-Raphson solvers.
- Full matrix IK benchmark: 9 robots × 14 solvers proves Restart+LM matches
  TRAC-IK at 4-8× speed.

### Changed
- `sqp_stepper` renamed to `bobyqa_stepper` (derivative-free BOBYQA) with
  deprecated alias; new `slsqp_stepper` wrapping NLopt LD_SLSQP with
  analytical gradient via SE(3) log Jacobian.

## [0.1.1] — 2026-03-28

### Added
- Full project rename: spatialpp → liepp (predecessor of the later
  liepp → Cartan rename in v0.2.0).
- Convention cleanup across 76 files: `HPP_GUARD_` header guards (no
  `#pragma once`), include ordering, brace style.
- DOF sweep tests covering 1-7 × {double, float} × {fixed, dynamic} chain
  types with 8 robot chain geometries.
- Complete LaTeX background pages for SO(2), SE(2), SO(3), SE(3), PoE
  kinematics, space/body Jacobians, IK methods (DLS/LM/SQP), null-space
  projection, and frame tags.
- Benchmark infrastructure: 21 Lie group, 5 FK, 10 Jacobian, 13 IK
  benchmarks; head-to-head vs TRAC-IK across 9 robots.

### Fixed
- `lm_stepper<dynamic, float>` Eigen `MatrixXd`-hardcoding bug caught by the
  DOF sweep tests.

## [0.1.0] — 2026-03-27

### Added
- Lie group primitives (SO(2), SE(2), SO(3), SE(3)) with singularity-safe
  exp/log and property-based tests.
- Quaternion-based SO(3) with `atan2` log map (avoids θ ≈ π singularity
  branch).
- SE(3) with left Jacobian-based exp/log and 6×6 adjoint, validated by
  RapidCheck property tests.
- Compile-time frame-tagged wrappers (`transform<From, To>`,
  `rotation<From, To>`, twist, wrench) with structural template matching and
  zero runtime overhead.
- PoE-based kinematic chain with screw axis factories, joint limits, and
  fixed/dynamic storage via the `storage_selector` trait.
- Forward kinematics with intermediate caching; fixed-N unrolled fold for
  1-7 DOF; dynamic runtime loop.
- Space and body Jacobians with compile-time unrolling (N=1-7) and
  finite-difference validation within `1e-6`.
- DLS and LM IK steppers with the `ik_stepper` concept, SVD-based adaptive
  damping, Nielsen lambda update, and separate angular/linear convergence.
- Policy-based IK solver template with clamp/null-space limit enforcement,
  NLopt SQP stepper, and `ik_objective`-driven secondary optimization.
- Racing and fallback schedulers with tick policies for cooperative
  multi-solver IK.
- CMake project skeleton (originally C++23, later relaxed to a C++20 baseline
  in 0.4.1), Eigen INTERFACE target, NLopt optional backend, seven presets.
- CI pipeline with GCC-14 / Clang-18 matrix, ASan + UBSan + MSan sanitizer
  jobs, clang-tidy adapted for spatialpp headers.
