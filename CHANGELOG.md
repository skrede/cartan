# Changelog

All notable user-facing changes to this project are documented in this file.

The format is based on [Keep a Changelog 1.1.0](https://keepachangelog.com/en/1.1.0/),
and the project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased] — 0.4.1

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
- README license badge and footer corrected from MIT to Apache 2.0 (the
  `LICENSE` file at repo root has always carried Apache 2.0 text; the badge
  and footer were the inconsistency).
- CMake `project(cartan VERSION ...)` is now the single source of the library
  version at `0.4.1`, with explicit `LANGUAGES CXX`. `cartan-lie` installs the
  generated `cartan/version.h` header so `find_package(cartan)` consumers can
  include it alongside the rest of the public API.

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
- CMake project skeleton with C++23, Eigen INTERFACE target, NLopt optional
  backend, seven presets.
- CI pipeline with GCC-14 / Clang-18 matrix, ASan + UBSan + MSan sanitizer
  jobs, clang-tidy adapted for spatialpp headers.
