<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/cartan-banner-dark.svg">
    <img alt="Cartan" src="docs/cartan-banner.svg" width="640">
  </picture>
</p>

![Linux](https://github.com/skrede/cartan/actions/workflows/linux.yml/badge.svg)
![macOS](https://github.com/skrede/cartan/actions/workflows/macos.yml/badge.svg)
![Windows](https://github.com/skrede/cartan/actions/workflows/windows.yml/badge.svg)
[![codecov](https://codecov.io/gh/skrede/cartan/branch/master/graph/badge.svg)](https://codecov.io/gh/skrede/cartan)
![License](https://img.shields.io/badge/license-Apache_2.0-blue.svg)
![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)
![Status](https://img.shields.io/badge/status-public%20preview-orange.svg)

A C++20 rigid-motion and serial kinematics library.

## Status

**Public preview.** The C++ API and Python bindings are under development; 
expect breaking changes onwards to a stable `v1.0.0` release. 
The library builds and tests on Linux, macOS, and Windows and can be installed 
using `FetchContent` (see below). Python bindings will be available as `pip install cartan-bindings` (WIP).

## Features

- **Lie groups:** SO(2), SE(2), SO(3), SE(3) with exp/log, adjoint, coadjoint, left/right Jacobians
- **Product of Exponentials kinematics:** screw-theory-based FK with compile-time unrolled PoE for 1-7 DOF chains
- **Space and body Jacobians:** reusing cached FK intermediate products for zero redundant computation
- **Compile-time frame safety:** `transform<From, To>` and `rotation<From, To>` wrappers with structural type checking
- **Policy-based IK solvers:** DLS, Levenberg-Marquardt, and SQP steppers composable with racing and fallback schedulers
- **Header-only:** no compiled library artifacts, Eigen as the sole required dependency

## Scope

Cartan is a Lie-group, forward-kinematics, and inverse-kinematics library &mdash; and
deliberately only that. It stays a small, composable library rather than a
robotics framework, so the following are **non-goals**, each better served by a
purpose-built tool cartan composes with:

- **Dynamics** (RNEA, mass matrix, gravity/Coriolis); use [Pinocchio](https://github.com/stack-of-tasks/pinocchio).
- **Collision detection** &mdash; use [hpp-fcl / Coal](https://github.com/coal-library/coal).
- **Motion / trajectory planning** &mdash; use a planner (OMPL) or a time-parameterizer
  ([TOPP-RA](https://github.com/hungpham2511/toppra)).
- **State estimation / filtering** &mdash; see [ctrlpp](https://github.com/skrede/ctrlpp).
- **Visualization** &mdash; use [threepp](https://github.com/markaren/threepp) or your own renderer.
- **Middleware bindings** (ROS / KDL / orocos) &mdash; zero coupling by design.
- **Custom linear algebra** &mdash; Eigen only; cartan reinvents no matrix math.

Cartan composes with these rather than absorbing them: its `SE3` / `SO3` types
and FK / Jacobian outputs feed directly into a dynamics solver, a collision
checker, or a planner. The guiding principle is *library, not framework* &mdash;
cartan owns kinematics and stays out of everything else.

## Requirements

- C++20 compiler: GCC 10+, Clang 13+, MSVC 17.x+
- CMake 3.28+
- Eigen 3.4+ (auto-fetched via FetchContent)
- For embedded targets: an exceptions-off C++20 GCC backend &mdash; ESP-IDF 5.1+ / 6.x
  (esp32, esp32c3) or arm-none-eabi (cortex-m7, cortex-m4f).

## Quick Install

### CMake FetchContent (recommended)

```cmake
include(FetchContent)
FetchContent_Declare(
    cartan
    GIT_REPOSITORY https://github.com/skrede/cartan.git
    GIT_TAG        master
)
FetchContent_MakeAvailable(cartan)

target_link_libraries(my_app PRIVATE cartan::cartan)
```

This pulls Cartan and its Eigen dependency automatically. No manual installation required.

### find_package

```cmake
find_package(cartan CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE cartan::cartan)
```

### ESP-IDF Component Manager

Cartan ships an `idf_component.yml` at the repo root. Add it to your firmware
project's `main/idf_component.yml` once it is published to the ESP Component
Registry, or pin a Git revision directly:

```yaml
dependencies:
  skrede/cartan:
    git: https://github.com/skrede/cartan.git
    version: "master"
```

Continuous integration cross-compiles a representative translation unit &mdash;
forward kinematics, the body Jacobian, an allocation-free projected
Levenberg-Marquardt IK step, and a Paden-Kahan subproblem &mdash; with C++
exceptions disabled (`-fno-exceptions`) for four target families: esp32
(xtensa) and esp32c3 (riscv32) via ESP-IDF, and Cortex-M7 and Cortex-M4F via
arm-none-eabi. This proves the public headers compile for those targets
exceptions-off; it does not run them on hardware. The compile-only sources
live under `tests/embedded/esp32-smoke/` (see that directory's README for the
`idf.py build` recipe) and `tests/embedded/arm-crosscompile/`.

## Documentation

- [Getting Started](docs/getting-started.md) &mdash; zero to compiling in 5 minutes.
- [Documentation Index](docs/README.md) &mdash; API reference, background theory, guides.
- [Examples](examples/) &mdash; runnable programs for every feature area.
- [Tutorials](examples/tutorials/) &mdash; step-by-step walkthroughs for FK, Jacobians, IK, and URDF loading.
- [Python](python/) &mdash; tutorials and setup guide for Cartan's Python API.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for the contribution workflow, coding conventions, and commit message format.

## License

Apache License 2.0 &mdash; see [LICENSE](LICENSE) for the full text.

Copyright 2026 Aleksander Skrede.

## Declaration of AI use
This library has been &mdash; and will be &mdash; developed with extensive use of Claude code (Sonnet, Opus and Fable).
