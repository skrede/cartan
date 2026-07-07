# Cartan benchmarks

Standalone benchmark suite for Cartan Lie group operations, forward kinematics, Jacobians, and inverse kinematics solvers. Includes direct comparison against TRAC-IK.

## Prerequisites

- CMake >= 3.28
- C++20 compiler: GCC 10+, Clang 13+, MSVC 17.x+
- Eigen >= 3.4

### For TRAC-IK comparison benchmarks

- orocos-kdl (Arch: `yay -S orocos-kdl`)
- NLopt (Arch: `sudo pacman -S nlopt`)

### For SQP stepper benchmarks

- NLopt (same as above)

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
    -DCARTAN_BUILD_NLOPT=ON \
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
./benchmarks/ik_comparison_benchmarks  # requires orocos-kdl + NLopt
```

### Filtered execution

```bash
# Run only SO(3) benchmarks
./benchmarks/lie_group_benchmarks --benchmark_filter="bm_so3"

# Run only UR3e comparison
./benchmarks/ik_comparison_benchmarks --benchmark_filter="ur3e"

# Run only Panda comparison
./benchmarks/ik_comparison_benchmarks --benchmark_filter="panda"
```

### JSON output (for post-processing)

```bash
./benchmarks/ik_comparison_benchmarks \
    --benchmark_format=json \
    --benchmark_out=comparison_results.json
```

### Statistical repetitions (p95, stddev)

To obtain p95 and other statistical aggregates beyond the single-run mean:

```bash
./benchmarks/ik_comparison_benchmarks \
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
| `ik_comparison_benchmarks.cpp` | Cartan vs TRAC-IK head-to-head | UR3e, KR6, IRB120, Jaco2, LBR Med 14, Panda, Fetch, Baxter, LWR 4+ |
| `opw_comparison_benchmarks.cpp` | Cartan `opw_6r_solver` vs `opw_kinematics` reference (same OPW formulation -> bit-identical): branch-for-branch parity + solve timing | KR6 R900 |
| `ikfast_comparison_benchmarks.cpp` | Cartan `opw_6r_solver` vs an OpenRAVE-generated IKFast solver (independent derivation -> ~1e-10 agreement): branch-for-branch parity + solve timing | KR6 R900 |
| `ikgeo_comparison_benchmarks.cpp` | Cartan `opw_6r_solver` vs ik-geo (Elias & Wen subproblem method, via a Rust FFI shim; independent formulation -> ~1e-13 agreement): branch-for-branch parity + solve timing | KR6 R900 |
| `benchmark_utils.h` | Shared chain factories (~10 robots), target generation | All |

## Methodology

- **Random seed:** 42 (fixed for reproducibility)
- **IK iterations:** 10,000 solves per configuration
- **Targets:** FK-generated from random joint configurations (guaranteed reachable)
- **Convergence:** position tolerance 1e-5, orientation tolerance 1e-5, max 100 iterations
- **TRAC-IK timeout:** 10.0s (convergence-based, not time-limited)
- **Reported metrics:** solve time, success rate, avg iterations, avg position error, avg orientation error
- **Compiler optimization:** Benchmarks build with release flags; `DoNotOptimize` prevents dead code elimination

## Dependencies

| Library | Source | Required By |
|---------|--------|-------------|
| Google Benchmark v1.9.5 | FetchContent (automatic) | All benchmarks |
| orocos-kdl | System package | Comparison benchmarks |
| NLopt | System package | Comparison + SQP benchmarks |
| spdlog | FetchContent (automatic) | TRAC-IK internals |
| opw_kinematics v0.5.5 | FetchContent (SHA-pinned, source-only) or `CARTAN_OPW_KINEMATICS_SOURCE_DIR` | OPW comparison benchmark |
| IKFast (generated) | Vendored under `third_party/ikfast_kr6r900/` (see its README for provenance) | IKFast comparison benchmark |
| LAPACK | System package | IKFast comparison benchmark (polynomial roots) |
| ik-geo (Rust) | Vendored under `third_party/ikgeo_ffi/`; built with `cargo` | ik-geo comparison benchmark |

## Results

See [docs/benchmarks.md](../docs/benchmarks.md) for full results and analysis.
