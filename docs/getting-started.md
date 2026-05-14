# Getting Started

## Prerequisites

- C++23 compiler: GCC 14+, Clang 18+, MSVC 17.10+
- CMake 3.28+
- Eigen 3.4+ (fetched automatically via FetchContent)

## Installation

### FetchContent (recommended)

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

This pulls Cartan and its Eigen dependency automatically. No manual
installation required.

### find_package

```cmake
find_package(cartan CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE cartan::cartan)
```

## Your first Lie group operation

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

Save this as `main.cpp` and create the following `CMakeLists.txt` in the
same directory:

```cmake
cmake_minimum_required(VERSION 3.28)
project(hello_cartan)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
    cartan
    GIT_REPOSITORY https://github.com/skrede/cartan.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(cartan)

add_executable(hello_cartan main.cpp)
target_link_libraries(hello_cartan PRIVATE cartan::cartan)
```

Build and run:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/hello_cartan
```

Expected output (the trailing digit varies by toolchain at the last
ULP or two):

```
Round-trip error: 1.38778e-16
```

## Transformations on SE(3)

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

The 6-vector groups the angular part `omega` and the linear part `rho`
in that order; `SE3::exp` builds the rigid-body transform whose tangent
at the identity is that twist, and `log()` recovers the original twist.

## Forward kinematics

Build a 2-link planar arm programmatically from screw axes and compute the end-effector pose at a joint configuration.

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

Two revolute joints about the z-axis, each with a unit link length, give
an end-effector at (2, 0, 0) at the zero configuration. The output is
the 4x4 homogeneous transformation matrix of the end-effector at joint
angles `q = (0.5, -0.3)` radians.

## Inverse kinematics

Now run inverse kinematics on the same arm. We pick a known joint
configuration, FK-walk it to a target pose, and back-solve from a
different seed using Levenberg-Marquardt.

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

For a deeper walkthrough of solver composition, racing, and the
work-unit budget, see the [IK Composition Guide](guides/ik-composition.md).
The IK theory (Newton-Raphson, Levenberg-Marquardt, body-frame error) is
covered in [IK Methods](background/ik-methods.md).

## Adding more joints

Chains are built from a sequence of `screw_axis` objects plus a home
pose and per-joint `joint_limits`. The `kinematic_chain` template's
second argument is the compile-time joint count (or `cartan::dynamic`
for runtime). `screw_axis` offers both `::revolute(axis, point)` and
`::prismatic(direction)` factory methods, so mixed-joint chains follow
the same construction pattern.

For a deeper walkthrough of the Product of Exponentials formulation
and chain construction patterns, see the
[PoE Walkthrough](guides/poe-walkthrough.md).

## Next steps

- [Documentation Index](README.md) -- API reference, guides, background theory
- [Background](background/) -- mathematical derivations for SO(3), SE(3), PoE, Jacobians, IK methods
- [Guides](guides/) -- PoE walkthrough, IK composition, frame tags
- [Examples](../examples/) -- runnable programs for every feature area
