# PoE Kinematics Walkthrough

This guide walks through building a kinematic chain and computing forward kinematics and Jacobians using liepp's Product of Exponentials (PoE) formulation. By the end, you will have a complete working example from screw axis definition to Jacobian output.

**Prerequisites:** liepp installed and linked via CMake. Familiarity with 3D rotations and rigid body transforms.

## 1. Defining Screw Axes

Every joint in a serial robot is described by a **screw axis** in the space frame at the home (zero) configuration. liepp provides factory methods for the two common joint types:

**Revolute joints** rotate about an axis passing through a point:

```cpp
#include <liepp/chain/screw_axis.h>

using vec3 = liepp::vector3<double>;

// Joint rotates about the z-axis, passing through the origin
auto s1 = liepp::screw_axis<double>::revolute(
    vec3(0, 0, 1),    // omega: rotation axis direction
    vec3(0, 0, 0));   // point: any point on the axis
```

The factory normalizes the axis direction and computes `v = -omega x point` internally, giving the 6D screw vector `(omega, v)`.

**Prismatic joints** translate along a direction:

```cpp
// Joint slides along the x-axis
auto s_prismatic = liepp::screw_axis<double>::prismatic(vec3(1, 0, 0));
```

For prismatic joints, `omega = 0` and `v` is the unit translation direction.

> **Tip:** To identify screw axes from a robot's geometry, place the robot in its home configuration (all joints at zero). For each revolute joint, find the axis direction and any point the axis passes through. For prismatic joints, identify the sliding direction.

## 2. Home Configuration

The **home configuration** (often called the M matrix) is the SE(3) pose of the end-effector when all joint angles are zero:

```cpp
#include <liepp/lie/se3.h>
#include <liepp/lie/so3.h>

// End-effector at (3, 0, 0) with identity rotation at zero config
vec3 home_translation(3.0, 0.0, 0.0);
auto home = liepp::se3<double>(liepp::so3<double>::identity(), home_translation);
```

The home pose encodes both the position and orientation of the end-effector frame. For many robots at their zero configuration, the rotation is identity, but this depends on the frame conventions chosen.

## 3. Building the Chain

Combine the screw axes, home configuration, and joint limits into a `kinematic_chain`:

```cpp
#include <liepp/chain/kinematic_chain.h>
#include <liepp/chain/joint_limits.h>
#include <numbers>

// Joint limits: [-pi, pi] for each joint
liepp::joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};

// Fixed-size 3-DOF chain (N known at compile time)
liepp::kinematic_chain<3, double> chain(
    home,
    {s1, s2, s3},
    {lim, lim, lim});
```

For chains where the DOF is determined at runtime, use `liepp::dynamic`:

```cpp
// Dynamic-size chain (N determined at runtime)
liepp::kinematic_chain<liepp::dynamic, double> dyn_chain = chain.to_dynamic();
```

Fixed-size chains enable compile-time loop unrolling for N=1-7, giving zero-overhead performance on the hot path.

## 4. Computing Forward Kinematics

Forward kinematics computes the end-effector pose for a given joint configuration using the PoE formula:

    T(q) = exp([S1]q1) * exp([S2]q2) * ... * exp([Sn]qn) * M

```cpp
#include <liepp/kinematics/forward_kinematics.h>

Eigen::Vector3d q{0.5, -0.3, 0.8};   // joint angles in radians
auto fk = liepp::forward_kinematics(chain, q);

// End-effector SE(3) pose
liepp::se3<double> T_ee = fk.end_effector;
std::cout << "Position: " << T_ee.translation().transpose() << "\n";
std::cout << "Rotation:\n" << T_ee.rotation().matrix() << "\n";
```

The `fk_result` also caches intermediate products `exp([S1]q1)`, `exp([S1]q1)*exp([S2]q2)`, etc. These are reused by the Jacobian computation to avoid redundant matrix exponentials.

## 5. Computing Jacobians

The **space Jacobian** maps joint velocities to the spatial twist of the end-effector. The **body Jacobian** maps to the body-frame twist. Both accept the cached `fk_result`:

```cpp
#include <liepp/kinematics/jacobian.h>

// Space Jacobian: V_s = J_s(q) * dq
auto Js = liepp::space_jacobian(chain, fk);
std::cout << "Space Jacobian (6x3):\n" << Js << "\n";

// Body Jacobian: V_b = J_b(q) * dq
auto Jb = liepp::body_jacobian(chain, fk);
std::cout << "Body Jacobian (6x3):\n" << Jb << "\n";
```

Both functions take the pre-computed `fk` result to avoid recomputing the exponential products. This is important in IK loops where FK and Jacobians are computed every iteration.

## 6. Complete Example: 3-DOF Planar Arm

A full working example -- three revolute joints about the z-axis with unit link lengths:

```cpp
#include <liepp/liepp.h>
#include <iostream>
#include <numbers>

int main()
{
    using vec3 = liepp::vector3<double>;

    // Three revolute joints about z, with links along x
    auto s1 = liepp::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(0, 0, 0));
    auto s2 = liepp::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(1, 0, 0));
    auto s3 = liepp::screw_axis<double>::revolute(vec3(0, 0, 1), vec3(2, 0, 0));

    // Home: end-effector at (3, 0, 0) when all joints are zero
    auto home = liepp::se3<double>(
        liepp::so3<double>::identity(), vec3(3, 0, 0));

    liepp::joint_limits<double> lim{-std::numbers::pi, std::numbers::pi};
    liepp::kinematic_chain<3, double> chain(
        home, {s1, s2, s3}, {lim, lim, lim});

    // Compute FK at q = (0.5, -0.3, 0.8)
    Eigen::Vector3d q{0.5, -0.3, 0.8};
    auto fk = liepp::forward_kinematics(chain, q);

    std::cout << "End-effector position: "
              << fk.end_effector.translation().transpose() << "\n";

    // Compute Jacobians
    auto Js = liepp::space_jacobian(chain, fk);
    auto Jb = liepp::body_jacobian(chain, fk);
    std::cout << "Space Jacobian:\n" << Js << "\n";
    std::cout << "Body Jacobian:\n" << Jb << "\n";
}
```

## Further Reading

- [PoE Kinematics Theory](../background/poe-kinematics.md) -- mathematical derivation of the PoE formula, screw theory, and intermediate product caching
- [API Reference: Kinematics](../api/kinematics.md) -- full function signatures, edge cases, and return types
- [API Reference: Chain](../api/chain.md) -- kinematic_chain, screw_axis, and joint_limits details
