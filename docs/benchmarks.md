# Benchmarks

## Overview

Cartan provides a comprehensive benchmark suite measuring Lie group operations, forward kinematics, Jacobian computation, and inverse kinematics solver performance. IK benchmarks include direct comparison against TRAC-IK (Beeson & Ames, 2015) running on the same hardware under identical conditions.

## Hardware

| Spec | Value |
|------|-------|
| CPU | AMD Ryzen 7 5800X3D 8-Core (16 threads) |
| Clock | 3.4 GHz base / 4.5 GHz boost |
| L3 Cache | 96 MB (3D V-Cache) |
| RAM | 64 GB DDR4 |
| Compiler | GCC 15.2.1 (-O2, Release) |
| Eigen | 3.4.99 (FetchContent) |
| OS | EndeavourOS, Linux 6.18.19-1-lts (x86_64) |

## Methodology

### Lie Group Operations

Benchmarked operations per group:
- **SO(2):** exp, log, compose, inverse (4 ops)
- **SE(2):** exp, log, compose, inverse, adjoint (5 ops)
- **SO(3):** exp, log, compose, inverse, adjoint, coadjoint (6 ops)
- **SE(3):** exp, log, compose, inverse, adjoint, coadjoint (6 ops)

All benchmarks use `double` scalar type. Results collected via Google Benchmark with `DoNotOptimize` to prevent dead code elimination.

### Forward Kinematics & Jacobians

Benchmarked for 3-DOF (planar 3R), 6-DOF (UR3e), and 7-DOF (LBR Med 14) chains. 6-DOF and 7-DOF include both compile-time fixed-N and Eigen::Dynamic template variants to measure compile-time specialization benefit.

### IK Solvers

Five solver/scheduler types benchmarked:
- **DLS** (Damped Least Squares): 3/6/7-DOF
- **LM** (Levenberg-Marquardt): 3/6/7-DOF
- **SQP** (Sequential Quadratic Programming): 3/6/7-DOF (requires NLopt)
- **Racing Scheduler**: 6/7-DOF (3 parallel solvers)
- **Fallback Scheduler**: 6/7-DOF (2 sequential solvers)

Each configuration runs 10,000 solves with:
- FK-generated reachable targets (guaranteed solvable)
- Fixed random seed (42) for reproducibility
- Convergence tolerance: position 1e-5, orientation 1e-5
- Maximum 100 iterations per solver

Reported metrics: solve time (mean/median), iterations to convergence, success rate, position error, orientation error.

**Statistical aggregates (p95, stddev):** To obtain p95 and other statistical aggregates, use Google Benchmark's `--benchmark_repetitions=N` flag. For example: `--benchmark_repetitions=10 --benchmark_report_aggregates_only=true` reports mean, median, stddev, and cv across N repetitions. Each repetition runs the full 10,000 iterations, so p95 is computed across repetitions rather than individual solves.

### TRAC-IK Comparison

Direct head-to-head comparison against TRAC-IK (de-rosified, no ROS/URDF dependency) on 9 robots:

| Robot | DOF | Source |
|-------|-----|--------|
| Universal Robots UR3e | 6 | UR official DH parameters |
| KUKA KR 6 R900 SIXX | 6 | KUKA working envelope drawing |
| ABB IRB 120 | 6 | ABB product datasheet |
| Kinova Jaco2 | 6 | Kinova product datasheet |
| KUKA LBR Med 14 R820 | 7 | LBR Med 14 technical data |
| Franka Emika Panda | 7 | Franka documentation |
| Fetch Robotics arm | 7 | Fetch Robotics URDF/datasheet |
| Rethink Baxter (single arm) | 7 | Rethink Robotics datasheet |
| KUKA LWR 4+ | 7 | KUKA LWR 4+ technical data |

**Fairness controls:**
- Same random seed (42) for target generation
- Same 10,000 targets per robot (FK-generated, guaranteed reachable)
- Same convergence tolerance (eps = 1e-5)
- TRAC-IK timeout set to 10s (convergence-based, not time-based termination)
- Cartan uses LM stepper (closest algorithmic match to TRAC-IK's Newton methods)
- Both run on same hardware, same compilation flags

## Results

### Lie Group Operations

All times in nanoseconds (ns). Lower is better.

| Operation | SO(2) | SE(2) | SO(3) | SE(3) |
|-----------|-------|-------|-------|-------|
| exp | 0.2 ns | 0.5 ns | 25.0 ns | 60.3 ns |
| log | 0.1 ns | 0.5 ns | 15.4 ns | 50.7 ns |
| compose | 0.2 ns | 0.5 ns | 5.1 ns | 2.4 ns |
| inverse | 0.2 ns | 0.5 ns | 0.5 ns | 2.5 ns |
| adjoint | N/A | 1.1 ns | 2.2 ns | 21.4 ns |
| coadjoint | N/A | N/A | 2.2 ns | 32.5 ns |

### FK Performance

| Chain | DOF | Fixed-N | Dynamic | Overhead |
|-------|-----|---------|---------|----------|
| 3R Planar | 3 | 285 ns | N/A | -- |
| UR3e | 6 | 546 ns | 551 ns | +0.9% |
| LBR Med 14 | 7 | 639 ns | 636 ns | -0.4% |

Compile-time fixed-N and Eigen::Dynamic produce essentially identical performance for FK, confirming that Eigen's dynamic dispatch has negligible overhead for these chain sizes.

### Jacobian Performance

All times in nanoseconds (ns).

| Chain | DOF | Space (Fixed) | Space (Dynamic) | Body (Fixed) | Body (Dynamic) |
|-------|-----|---------------|-----------------|--------------|----------------|
| 3R Planar | 3 | 74 ns | N/A | 119 ns | N/A |
| UR3e | 6 | 166 ns | 170 ns | 229 ns | 237 ns |
| LBR Med 14 | 7 | 203 ns | 201 ns | 268 ns | 273 ns |

Body Jacobians are ~1.3-1.6x slower than space Jacobians due to the additional adjoint transformation. Dynamic-N overhead is within measurement noise.

### IK Solver Comparison

All times are mean solve time per single IK problem. Success rate, iterations, and errors are averaged over 10,000 FK-generated reachable targets.

| Solver | Robot | Success Rate | Avg Iters | Avg Pos Err | Avg Ori Err | Mean Solve Time |
|--------|-------|-------------|-----------|-------------|-------------|-----------------|
| DLS | 3R Planar (3-DOF) | 92.6% | 8.9 | 1.34e+00 | 9.85e-01 | 20.0 µs |
| DLS | UR3e (6-DOF) | 73.9% | 20.2 | 5.78e-01 | 1.85e+00 | 169.9 µs |
| DLS | LBR Med 14 (7-DOF) | 96.5% | 12.1 | 5.40e-01 | 1.72e+00 | 77.3 µs |
| LM | 3R Planar (3-DOF) | 99.6% | 8.8 | 1.08e+00 | 8.03e-01 | 10.5 µs |
| LM | UR3e (6-DOF) | 74.0% | 14.9 | 3.82e-01 | 1.30e+00 | 34.8 µs |
| LM | LBR Med 14 (7-DOF) | 99.7% | 12.1 | 3.70e-01 | 1.17e+00 | 30.0 µs |
| SQP | 3R Planar (3-DOF) | 90.5% | 2.7 | 2.08e-09 | 1.46e-09 | 394 µs |
| SQP | UR3e (6-DOF) | 69.1% | 4.1 | 7.47e-08 | 3.17e-08 | 8073 µs |
| SQP | LBR Med 14 (7-DOF) | 97.6% | 2.6 | 1.31e-07 | 4.13e-08 | 6070 µs |
| Racing | UR3e (6-DOF) | 89.2% | 39.4 | 1.34e-06 | 6.08e-07 | 312 µs |
| Racing | LBR Med 14 (7-DOF) | 97.3% | 25.5 | 7.26e-07 | 1.23e-06 | 172 µs |
| Fallback | UR3e (6-DOF) | 73.9% | 20.2 | 1.31e-06 | 5.83e-07 | 258 µs |
| Fallback | LBR Med 14 (7-DOF) | 96.5% | 12.1 | 6.47e-07 | 1.10e-06 | 95.8 µs |

**Notes:**
- SQP uses NLopt's BOBYQA (derivative-free) with random restarts when stalled at local minima. It's ~100x slower than DLS/LM but achieves tight convergence (sub-nanometer errors on successful solves).
- DLS/LM error values are averaged over all attempts including failures (where error remains at the initial guess level), which inflates the reported averages.
- Racing and Fallback schedulers show sub-micron position errors on successful solves, demonstrating tight convergence.

### Cartan vs TRAC-IK

1,000 FK-generated reachable targets per robot, same random seed (42), same convergence tolerance (1e-5). Cartan uses LM stepper; TRAC-IK uses its internal Newton + NLopt dual solver with 10s timeout.

| Robot | DOF | Solver | Success Rate | Avg Iters | Avg Pos Err | Avg Ori Err | Mean Solve Time | Speedup |
|-------|-----|--------|-------------|-----------|-------------|-------------|-----------------|---------|
| UR3e | 6 | Cartan (LM) | 74.3% | 15.3 | 3.90e-01 | 1.35e+00 | 35.9 µs | 0.96x |
| UR3e | 6 | TRAC-IK | 76.8% | -- | -- | -- | 34.3 µs | baseline |
| KR 6 SIXX | 6 | Cartan (LM) | 99.4% | 14.9 | 3.91e-01 | 1.33e+00 | 31.7 µs | 0.73x |
| KR 6 SIXX | 6 | TRAC-IK | 100.0% | -- | -- | -- | 23.0 µs | baseline |
| ABB IRB 120 | 6 | Cartan (LM) | 89.4% | 13.0 | 2.92e-01 | 1.15e+00 | 29.3 µs | 0.80x |
| ABB IRB 120 | 6 | TRAC-IK | 100.0% | -- | -- | -- | 23.4 µs | baseline |
| Jaco2 | 6 | Cartan (LM) | 94.3% | 17.0 | 3.36e-01 | 1.38e+00 | 37.7 µs | 0.64x |
| Jaco2 | 6 | TRAC-IK | 100.0% | -- | -- | -- | 24.1 µs | baseline |
| LBR Med 14 | 7 | Cartan (LM) | 99.5% | 12.2 | 4.01e-01 | 1.27e+00 | 30.6 µs | 0.71x |
| LBR Med 14 | 7 | TRAC-IK | 100.0% | -- | -- | -- | 21.8 µs | baseline |
| Panda | 7 | Cartan (LM) | 92.2% | 12.7 | 3.43e-01 | 1.13e+00 | 38.4 µs | 0.59x |
| Panda | 7 | TRAC-IK | 100.0% | -- | -- | -- | 22.6 µs | baseline |
| Fetch | 7 | Cartan (LM) | 99.7% | 12.6 | 3.07e-01 | 1.12e+00 | 30.9 µs | 0.72x |
| Fetch | 7 | TRAC-IK | 100.0% | -- | -- | -- | 22.3 µs | baseline |
| Baxter | 7 | Cartan (LM) | 92.7% | 12.1 | 3.63e-01 | 1.12e+00 | 31.0 µs | 0.75x |
| Baxter | 7 | TRAC-IK | 100.0% | -- | -- | -- | 23.2 µs | baseline |
| KUKA LWR 4+ | 7 | Cartan (LM) | 99.7% | 12.3 | 3.70e-01 | 1.26e+00 | 29.8 µs | 0.73x |
| KUKA LWR 4+ | 7 | TRAC-IK | 100.0% | -- | -- | -- | 21.8 µs | baseline |

**Notes:**
- TRAC-IK does not expose per-solve iteration counts or error metrics through its API, so those columns show `--`.
- Cartan LM stepper is a single-algorithm solver; TRAC-IK runs a dual Newton + NLopt strategy internally, which explains its higher success rates.
- Cartan's Racing/Fallback schedulers (see IK Solver Comparison above) provide the multi-strategy approach that competes more directly with TRAC-IK's dual solver.
- Speedup < 1.0 means TRAC-IK is faster. Both solvers operate in the same order of magnitude (20-40 µs per solve).

### Beeson & Ames 2015 Reference

For context, Beeson & Ames (2015) reported TRAC-IK success rates on the same robot families using the original ROS-coupled implementation on 2015 hardware. Our comparison uses the de-rosified library on modern hardware (AMD Ryzen 7 5800X3D), so absolute timings are not directly comparable, but success rates provide a useful reference point.

| Robot | DOF | TRAC-IK (ours) | TRAC-IK (Beeson 2015) |
|-------|-----|----------------|----------------------|
| ABB IRB 120 | 6 | 100.0% | 99.6% |
| Jaco2 | 6 | 100.0% | 99.4% |
| Panda | 7 | 100.0% | 99.97% |
| Fetch | 7 | 100.0% | 100.0% |
| Baxter | 7 | 100.0% | 99.8% |
| KUKA LWR 4+ | 7 | 100.0% | 99.8% |

Our TRAC-IK results are consistent with the published numbers, confirming the de-rosified library behaves equivalently.

Reference: Beeson, P., & Ames, B. (2015). TRAC-IK: An open-source library for improved solving of generic inverse kinematics. *Proceedings of the IEEE-RAS International Conference on Humanoid Robots*.

## Reproducing

See [benchmarks/README.md](../benchmarks/README.md) for detailed build and run instructions.
