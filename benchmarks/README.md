# Cartan benchmarks

Standalone benchmark suite for Cartan Lie group operations, forward kinematics, Jacobians, and inverse kinematics solvers. Includes direct comparison against TRAC-IK.

## What this repository provides, and what you supply

This repository provides the harness, the robot fixtures, the raw records and the
analysis script. **The comparator libraries are yours to obtain.** cartan does not
download, copy or patch a library it measures itself against: which revision of a
comparator produced a published number is not a question this build gets to answer
on your machine's behalf, and a copy of somebody else's library in this tree is a
copy nobody updates.

Every benchmark dependency sits in exactly one of three acquisition classes, and
the class it sits in is recorded beside its version in the environment manifest
written next to every capture:

| Class | Members | How it is obtained |
|---|---|---|
| Owned / collaborator | the robot-description supplier, the optimization backend | fetched at an immutable commit, and the commit the build resolves is verified against the declared pin |
| Mainstream third-party | TRAC-IK, Pinocchio, orocos-kdl, NLopt, LAPACK, opw_kinematics, ik-geo | discovered through package configuration, a project find module, or a path you supply; never copied into this tree |
| Generated | the IKFast solver source under `third_party/ikfast_kr6r900/` | stays here, with its codegen provenance in its own README |

The two classes exist because the test is the same in both directions: can a
reader obtain this themselves. Nothing packages the first class, so a reader has
no other way to get it. Every member of the second class is a package-manager
line away for anyone who wants it.

**Generated solver source is not vendoring.** The IKFast file is codegen output
for one specific robot — closer to a fixture than to a dependency — and
regenerating it needs an OpenRAVE toolchain that is genuinely hard to stand up.
It is named here so the no-copying rule reads as a rule with a stated boundary
rather than a rule with an unexplained exception.

A comparator that is enabled but not found announces itself three times: at
configure time, at the start of a capture, and in the manifest's absence list.
Configure with `-DCARTAN_STRICT_COMPARATORS=ON` to refuse a partial participant
set outright. A comparator that is found but cannot say which version it is fails
the configure: a number attributed to an unidentifiable library is a number
attributed to nothing.

## Prerequisites

- CMake >= 3.28
- C++20 compiler: GCC 10+, Clang 13+, MSVC 17.x+
- Eigen >= 3.4
- Google Benchmark — the measurement harness. This is the one third-party
  dependency the build may still download for you (`CARTAN_FETCH_BENCHMARK_DEPS`),
  because it produces the measurement rather than participating in it.

### For the comparison benchmarks

- orocos-kdl (Arch: `yay -S orocos-kdl`) — needed by every benchmark target
- NLopt (Arch: `sudo pacman -S nlopt`)
- Pinocchio, LAPACK
- TRAC-IK: no package configuration ships with it, so either install it where
  `cmake/FindTracIK.cmake` will find it, or pass
  `-DCARTAN_TRAC_IK_SOURCE_DIR=<checkout>`
- opw_kinematics: pass `-DCARTAN_OPW_KINEMATICS_SOURCE_DIR=<checkout>`
- `cargo`, for the ik-geo shim, which builds the published crate from the registry

## Kernel-evaluation counts exist for some participants and not others

The study's machine-independent axis is the number of forward-kinematics and
Jacobian evaluations a solve spends. The harness counts those by substituting a
counting chain into the solvers whose iteration it drives — cartan and Pinocchio.
It cannot count TRAC-IK: that solver owns its iteration loop inside its own solve
entry point and its kinematics solvers are private members with no injection
point. So TRAC-IK appears on the wall-clock companion curve only.

Where a count does not exist, `fk_evals` and `jac_evals` are written **empty, not
zero**, the per-cell tier carries a `kernel_countable` column, and the rebuilt
table prints *not counted*. A zero would place the participant at the origin of
the axis, which is a stronger and wronger claim than an acknowledged gap. No
kernel count is ever derived, modelled or back-derived from a wall-clock
measurement.

## What the comparison harnesses are, and what they are not

`ik_comparison_pinocchio_benchmarks` and `basic_ik_full_benchmarks` build their chains from
the hand-coded fixtures in `tests/fixtures/chain_factories.h`, whose joint limits are a
synthetic symmetric box of ±π on every joint. They are a **cartan-internal solver-family
comparison over synthetic bounds**, useful for ranking cartan's own solvers against each other
and for keeping an eye on a regression, and they are **not a source of any cross-library claim**.
The study harness (`ik_study_capture`, `benchmarks/study/`) is: it takes its bounds from
revision-pinned upstream robot descriptions, hands one feasible set to every participant, and
adjudicates every solve itself. Quote the study, not these.

The distinction is measured rather than stylistic. On the symmetric box every joint's arc is
exactly one full turn, so canonicalization always succeeds and a limits check cannot fail: 0 of
1500 adjudicated rows fail it on the box, against 410 of 1500 on the manufacturer's declared
bounds. A comparison whose feasibility question has only one possible answer is not measuring
feasibility.

### Retired cells and why

A cell that vanished from a table with no recorded reason is indistinguishable from a cell that
was inconvenient, so each retirement is written down here.

- **The cartan-versus-TRAC-IK head-to-head harness (54 cells).** Retired outright. Its cartan
  rows solved an unconstrained problem while the comparator solved a box-constrained one, and
  its cartan success column reported the solver's own status rather than an independently
  recomputed verdict. Both defects are structural, and the study replaces the harness with one
  feasible set no cell can opt out of and a verdict the harness computes for every participant.
- **`cartan_restart_lm_clamped`, `argmin_mma` and `argmin_gcmma`.** Retired with that file: they
  were benchmarked nowhere else, and the study does not declare them as participants. `mma` keeps
  its compile coverage in `tests/unit/argmin_solver_instantiation_test.cpp` and now has no
  measured cell at all; `gcmma` has since been removed from the library altogether.
- **Every `_no_limits` method variant (136 cells).** Retired: 136 of 136 limited-versus-unbounded
  pairs produced identical success rates, because the default cells clamp while the variant cells
  run unbounded and every target is drawn from inside the box, so no bound ever binds. The
  deliberate unbounded measurement now lives in the study as its own table over
  description-sourced bounds, where a bound can actually bind.
- **The multiplier-re-estimation sweep (108 cells).** Retired: the optimization backend's own
  test suite asserts, at the revision this build pins, that the parameter is not consumed at any
  call site by the sequential-programming policy cartan resolves through, and that two different
  values must produce bit-identical iteration counts and objectives. cartan forwards the field
  correctly; the knob is inert by upstream design, so the cells measured nothing.
- **`tools/ik_accuracy_sweep.py`.** Retired: it drove the deleted harness through an environment
  variable. Driving every participant at one accuracy is now `ik_study_calibrate` plus the
  capture's `--iso-accuracy` mode, which calibrates against achieved rather than requested error.

### The BOBYQA cell reads near zero, and that is a budget statement

`argmin_bobyqa` reports 0.00–0.05% success in this harness while genuinely spending 350–450 µs.
That is not a wiring defect. Measured on UR3e, IRB 120 and LBR Med 14 at 2000 targets each: the
harness caps a solver attempt at 100 iterations, and BOBYQA's accepted solves need 220–305 at
the median on these chains. Lifting the cap alone takes the three cells to 9.30 / 14.80 / 21.15%.
A derivative-free method needs an order of magnitude more objective evaluations than a
Jacobian-driven one, and this harness gives every family the same per-attempt allowance; the
near-zero row is what that allowance buys BOBYQA, not evidence that it is broken.

A separate, real defect was found alongside it and fixed: cartan's generic stall detector aborted
BOBYQA during normal trust-region contraction. With the cap lifted, that fix moves the same three
cells from 9.30 / 14.80 / 21.15% to 22.30 / 35.70 / 58.75%. It changes nothing at the published
cap, where the iteration limit binds first.

## Building

From the repository root:

```bash
cmake --preset=dev \
    -DCARTAN_BUILD_BENCHMARKS=ON \
    -DCARTAN_CMAKE_FETCH_DEPS=ON

cmake --build --preset=dev
```

Google Benchmark is fetched automatically via CMake FetchContent.

To also build SQP benchmarks (requires NLopt):

```bash
cmake --preset=dev \
    -DCARTAN_BUILD_BENCHMARKS=ON \
    -DCARTAN_EXAMPLE_NLOPT_POLICY=ON \
    -DCARTAN_CMAKE_FETCH_DEPS=ON
```

## Running

### All benchmarks

```bash
# From build directory
./benchmarks/lie_group_benchmarks
./benchmarks/fk_benchmarks
./benchmarks/jacobian_benchmarks
./benchmarks/ik_dls_benchmarks
./benchmarks/ik_lm_benchmarks
./benchmarks/ik_sqp_benchmarks        # requires NLopt
./benchmarks/ik_racing_benchmarks
./benchmarks/ik_fallback_benchmarks
./benchmarks/ik_comparison_pinocchio_benchmarks  # requires orocos-kdl + Pinocchio
```

### Filtered execution

```bash
# Run only SO(3) benchmarks
./benchmarks/lie_group_benchmarks --benchmark_filter="bm_so3"

# Run only UR3e comparison
./benchmarks/ik_comparison_pinocchio_benchmarks --benchmark_filter="ur3e"

# Run only Panda comparison
./benchmarks/ik_comparison_pinocchio_benchmarks --benchmark_filter="panda"
```

### JSON output (for post-processing)

```bash
./benchmarks/ik_comparison_pinocchio_benchmarks \
    --benchmark_format=json \
    --benchmark_out=comparison_results.json
```

### Statistical repetitions (p95, stddev)

To obtain p95 and other statistical aggregates beyond the single-run mean:

```bash
./benchmarks/ik_comparison_pinocchio_benchmarks \
    --benchmark_repetitions=10 \
    --benchmark_report_aggregates_only=true
```

This runs the full 10,000-iteration benchmark 10 times and reports mean, median, stddev, and cv across repetitions. Combine with `--benchmark_format=json` for machine-readable output.

## Benchmark Structure

| File | What It Measures | Robots |
|------|------------------|--------|
| `lie_group_benchmarks.cpp` | SO(2)/SE(2)/SO(3)/SE(3) exp, log, compose, inverse, adjoint, coadjoint | N/A |
| `fk_benchmarks.cpp` | Forward kinematics for 3/6/7-DOF (fixed-N and dynamic) | 3R, UR3e, LBR Med 14 |
| `jacobian_benchmarks.cpp` | Space and body Jacobians for 3/6/7-DOF | 3R, UR3e, LBR Med 14 |
| `ik_dls_benchmarks.cpp` | DLS stepper IK for 3/6/7-DOF | 3R, UR3e, LBR Med 14 |
| `ik_lm_benchmarks.cpp` | LM stepper IK for 3/6/7-DOF | 3R, UR3e, LBR Med 14 |
| `ik_sqp_benchmarks.cpp` | SQP stepper IK for 3/6/7-DOF (NLopt) | 3R, UR3e, LBR Med 14 |
| `ik_racing_benchmarks.cpp` | Racing scheduler IK for 6/7-DOF | UR3e, LBR Med 14 |
| `ik_fallback_benchmarks.cpp` | Fallback scheduler IK for 6/7-DOF | UR3e, LBR Med 14 |
| `ik_comparison_pinocchio_benchmarks.cpp` | cartan's own solver families side by side, with TRAC-IK and a Pinocchio-primitive LM alongside them, over synthetic bounds | UR3e, KR6, IRB120, Jaco2, LBR Med 14, Panda, Fetch, Baxter, LWR 4+ |
| `opw_comparison_benchmarks.cpp` | Cartan `opw_6r_solver` vs `opw_kinematics` reference (same OPW formulation -> bit-identical): branch-for-branch parity + solve timing | KR6 R900 |
| `ikfast_comparison_benchmarks.cpp` | Cartan `opw_6r_solver` vs an OpenRAVE-generated IKFast solver (independent derivation -> ~1e-10 agreement): branch-for-branch parity + solve timing | KR6 R900 |
| `ikgeo_comparison_benchmarks.cpp` | Cartan `opw_6r_solver` vs the ik-geo crate (Elias & Wen subproblem method, through this project's own foreign-function shim; independent formulation -> ~1e-13 agreement): branch-for-branch parity + solve timing | KR6 R900 |
| `benchmark_utils.h` | Shared chain factories (~10 robots), target generation | All |

## Methodology

- **Random seed:** 42 (fixed for reproducibility)
- **IK iterations:** 10,000 solves per configuration
- **Targets:** FK-generated from random joint configurations (guaranteed reachable)
- **Convergence:** position and orientation tolerance default 1e-5, max 100 iterations. A run
  that needs every participant driven at one *achieved* accuracy rather than one requested
  tolerance uses `ik_study_calibrate` and the study capture's `--iso-accuracy` mode, which
  calibrates each solver's own tolerance against the accuracy the harness recomputes.
- **TRAC-IK cap:** 50 ms per solve (bounds a rare non-converging target; a timed-out solve
  counts as a failure)
- **Reported metrics:** solve time, success rate, avg iterations, avg position error, avg orientation error
- **Compiler optimization:** Benchmarks build with release flags; `DoNotOptimize` prevents dead code elimination

## Dependencies

| Library | Class | How it is obtained | Required by |
|---------|-------|--------------------|-------------|
| Google Benchmark | mainstream | installed package, or fetched at a pinned commit under `CARTAN_FETCH_BENCHMARK_DEPS` | all benchmarks |
| orocos-kdl | mainstream | installed package | all benchmarks |
| NLopt | mainstream | installed package | SQP and full-matrix benchmarks |
| Pinocchio | mainstream | installed package | peer comparison benchmarks and the study |
| TRAC-IK | mainstream | `cmake/FindTracIK.cmake`, or `CARTAN_TRAC_IK_SOURCE_DIR` | comparison benchmarks and the study |
| LAPACK | mainstream | installed package | IKFast comparison benchmark (polynomial roots) |
| opw_kinematics | mainstream | `CARTAN_OPW_KINEMATICS_SOURCE_DIR` | OPW comparison benchmark |
| ik-geo | mainstream | the crate registry, built by `cargo` through this project's own shim | ik-geo comparison benchmark |
| robot-description supplier | owned | fetched at a verified immutable commit | the study's joint limits |
| optimization backend | owned | fetched at a verified immutable commit | argmin cells |
| IKFast solver (generated) | generated | `third_party/ikfast_kr6r900/`, provenance in its own README | IKFast comparison benchmark |

## Results

See [docs/benchmarks.md](../docs/benchmarks.md) for full results and analysis.
