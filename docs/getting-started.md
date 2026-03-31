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
    liepp
    GIT_REPOSITORY https://github.com/skrede/liepp.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(liepp)

target_link_libraries(my_app PRIVATE liepp::liepp)
```

This pulls liepp and its Eigen dependency automatically. No manual
installation required.

### find_package

```cmake
find_package(liepp CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE liepp::liepp)
```

## Your first Lie group operation

The following program maps an axis-angle vector into SO(3) via the
exponential map, then recovers it with the logarithmic map. The round-trip
error should be near machine epsilon.

```cpp
#include <liepp/lie/so3.h>
#include <iostream>
#include <numbers>

int main()
{
    using SO3 = liepp::so3<double>;
    liepp::vector3<double> phi{0.1, 0.2, 0.3};

    auto R = SO3::exp(phi);
    auto phi_back = R.log();

    std::cout << "Original:  " << phi.transpose() << "\n";
    std::cout << "Recovered: " << phi_back.transpose() << "\n";
    std::cout << "Error:     " << (phi - phi_back).norm() << "\n";
}
```

Save this as `main.cpp` and create the following `CMakeLists.txt` in the
same directory:

```cmake
cmake_minimum_required(VERSION 3.28)
project(hello_liepp)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
    liepp
    GIT_REPOSITORY https://github.com/skrede/liepp.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(liepp)

add_executable(hello_liepp main.cpp)
target_link_libraries(hello_liepp PRIVATE liepp::liepp)
```

Build and run:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/hello_liepp
```

Expected output:

```
Original:  0.1 0.2 0.3
Recovered: 0.1 0.2 0.3
Error:     1.38778e-16
```

## Forward kinematics

Now let us compute the forward kinematics of a 2-link planar arm using
the Product of Exponentials (PoE) formula. Two revolute joints about the
z-axis, each with a unit link length, give an end-effector at (2, 0, 0)
at the zero configuration.

```cpp
#include <liepp/liepp.h>
#include <iostream>

int main()
{
    using vec3 = liepp::vector3<double>;
    using SE3 = liepp::se3<double>;
    using SO3 = liepp::so3<double>;

    // 2-link planar arm: two revolute joints about z, unit link lengths
    auto s1 = liepp::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(0, 0, 0));
    auto s2 = liepp::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(1, 0, 0));
    auto home = SE3(SO3::identity(), vec3(2, 0, 0));

    liepp::joint_limits<double> lim{-M_PI, M_PI};
    liepp::kinematic_chain<2, double> chain(home, {s1, s2}, {lim, lim});

    Eigen::Vector2d q(0.5, -0.3);
    auto fk = liepp::forward_kinematics(chain, q);

    std::cout << "End-effector:\n" << fk.end_effector.matrix() << "\n";
}
```

Replace `main.cpp` with this program, rebuild, and run. The output is the
4x4 homogeneous transformation matrix of the end-effector at joint angles
q = (0.5, -0.3) radians.

## Next steps

- [API Reference](README.md) -- function signatures, parameters, edge cases
- [Background](README.md) -- full mathematical derivations (SO(3), SE(3), PoE, Jacobians, IK)
- [Guides](README.md) -- PoE walkthrough, IK composition, frame tags
- [Examples](../examples/) -- runnable programs for every feature area
