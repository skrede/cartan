# Cartan

![Linux](https://github.com/skrede/cartan/actions/workflows/linux.yml/badge.svg)
![macOS](https://github.com/skrede/cartan/actions/workflows/macos.yml/badge.svg)
![Windows](https://github.com/skrede/cartan/actions/workflows/windows.yml/badge.svg)
[![codecov](https://codecov.io/gh/skrede/cartan/branch/master/graph/badge.svg)](https://codecov.io/gh/skrede/cartan)
![License](https://img.shields.io/badge/license-Apache_2.0-blue.svg)
![C++23](https://img.shields.io/badge/C%2B%2B-23-blue.svg)

C++ Lie group and kinematics library for robotics.

### Rotations on SO(3)

Map an axis-angle vector into SO(3) via the exponential map and recover it via the logarithmic map; the round-trip error is near machine epsilon.

```cpp
#include <cartan/lie/so3.h>
#include <iostream>

int main()
{
    using SO3 = cartan::so3<double>;
    cartan::vector3<double> phi{0.1, 0.2, 0.3};

    auto R = SO3::exp(phi);
    auto phi_back = R.log();

    std::cout << "Round-trip error: " << (phi - phi_back).norm() << "\n";
}
```

### Transformations on SE(3)

Compose a rigid-body transform from a 6-vector twist via the SE(3) exponential map, then recover the twist via the logarithm.

```cpp
#include <cartan/lie/se3.h>
#include <iostream>

int main()
{
    using SE3 = cartan::se3<double>;
    cartan::vector6<double> twist;
    twist << 0.0, 0.0, 0.3, 0.1, 0.2, 0.0;  // (omega, rho) -- angular part then linear part

    auto T = SE3::exp(twist);
    auto twist_back = T.log();

    std::cout << "Round-trip error: " << (twist - twist_back).norm() << "\n";
}
```

### Forward kinematics on a 2-link planar arm

Build a 2-link planar arm programmatically from screw axes and compute the end-effector pose at a joint configuration.

<details>
<summary>FK on a 2-link planar arm</summary>

```cpp
#include <cartan/serial_chain.h>
#include <iostream>
#include <numbers>

int main()
{
    using vec3 = cartan::vector3<double>;
    using SE3 = cartan::se3<double>;
    using SO3 = cartan::so3<double>;

    auto s1 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(0, 0, 0));
    auto s2 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(1, 0, 0));
    auto home = SE3(SO3::identity(), vec3(2, 0, 0));

    cartan::joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};
    cartan::kinematic_chain<double, 2> chain(home, {s1, s2}, {lim, lim});

    Eigen::Vector2d q(0.5, -0.3);
    auto fk = cartan::forward_kinematics(chain, q);

    std::cout << "End-effector:\n" << fk.end_effector.matrix() << "\n";
}
```

</details>

### Inverse kinematics on the same arm

Pick a known joint configuration on the same 2-link arm, FK-walk it to a target pose, then back-solve for the joints from a different seed using Levenberg-Marquardt.

<details>
<summary>IK on the same 2-link planar arm</summary>

```cpp
#include <cartan/serial_chain.h>
#include <iostream>
#include <numbers>

int main()
{
    using vec3 = cartan::vector3<double>;

    auto s1 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(0, 0, 0));
    auto s2 = cartan::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(1, 0, 0));
    auto home = cartan::se3<double>(cartan::so3<double>::identity(), vec3(2, 0, 0));
    cartan::joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};
    cartan::kinematic_chain<double, 2> chain(home, {s1, s2}, {lim, lim});

    Eigen::Vector2d q_known{0.3, -0.5};
    auto target = cartan::forward_kinematics(chain, q_known).end_effector;

    Eigen::Vector2d q0{0.0, 0.0};
    cartan::convergence_criteria<double> criteria{1e-6, 1e-6, 100, 200};

    cartan::basic_ik_runner<cartan::ik::lm<cartan::kinematic_chain<double, 2>>> solver;
    solver.setup(chain, target, q0, criteria);
    auto result = solver.solve();

    if (result.has_value())
        std::cout << "Solution: " << result.value().solution.position.transpose() << "\n";
}
```

</details>

Save any of these snippets as `main.cpp` alongside a `CMakeLists.txt` that pulls cartan via FetchContent (see the Quick Install section below). The same four snippets appear in `docs/getting-started.md` with full build instructions.

## Features

- **Lie groups:** SO(2), SE(2), SO(3), SE(3) with exp/log, adjoint, coadjoint, left/right Jacobians
- **Product of Exponentials kinematics:** screw-theory-based FK with compile-time unrolled PoE for 1-7 DOF chains
- **Space and body Jacobians:** reusing cached FK intermediate products for zero redundant computation
- **Compile-time frame safety:** `transform<From, To>` and `rotation<From, To>` wrappers with structural type checking
- **Policy-based IK solvers:** DLS, Levenberg-Marquardt, and SQP steppers composable with racing and fallback schedulers
- **Header-only:** no compiled library artifacts, Eigen as the sole required dependency

## Quick Install

### CMake FetchContent (recommended)

```cmake
include(FetchContent)
FetchContent_Declare(
    cartan
    GIT_REPOSITORY https://github.com/skrede/cartan.git
    GIT_TAG        main
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
    version: "main"
```

A compile-only smoke test under `tests/embedded/esp32-smoke/` verifies the
public headers build under xtensa-esp32-elf and riscv32-esp-elf; see that
directory's README for the `idf.py build` recipe.

### Arduino IDE (manual install)

Cartan ships a `library.properties` at the repo root. Clone or copy the
repository into your Arduino `libraries/` directory (typically
`~/Documents/Arduino/libraries/cartan/`), then `#include <cartan/lie.h>` from
a sketch targeted at ESP32, Teensy 4.x, or RP2040. Classic Arduino AVR boards
(Uno, Mega, Nano) are unsupported — cartan requires a modern C++20 compiler
and Eigen, neither of which fit on an AVR.

## Requirements

- C++20 compiler: GCC 10+, Clang 13+, MSVC 17.x+
- CMake 3.28+
- Eigen 3.4+ (auto-fetched via FetchContent)
- For embedded targets: ESP-IDF 5.1+, or any Arduino board package whose
  toolchain offers a C++20 GCC backend (esp32, teensy, rp2040).

## Documentation

- [Getting Started](docs/getting-started.md) -- zero to compiling in 5 minutes
- [Documentation Index](docs/README.md) -- API reference, background theory, guides
- [Examples](examples/) -- runnable programs for every feature area
- [Tutorials](examples/tutorials/) -- step-by-step walkthroughs for FK, Jacobians, IK, and URDF loading

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for the contribution workflow, coding conventions, and commit message format.

## License

Apache License 2.0 -- see [LICENSE](LICENSE) for the full text.

Copyright 2026 Aleksander Skrede.
