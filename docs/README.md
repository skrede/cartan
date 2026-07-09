# Cartan Documentation

## Getting Started

- [Getting Started](getting-started.md) -- zero to compiling in 5 minutes

## API Reference

- [lie](api/lie.md) -- SO(2), SE(2), SO(3), SE(3) Lie group classes
- [frames](api/frames.md) -- compile-time frame-tagged wrappers (transform, rotation, twist, wrench)
- [chain](api/chain.md) -- kinematic chain, screw axis, joint limits, joint state
- [kinematics](api/kinematics.md) -- forward kinematics, space/body Jacobians, velocity kinematics
- [ik](api/ik.md) -- IK solver, steppers (DLS, LM, SQP), schedulers (racing, fallback), policies
- [python](python.md) -- Python install, API parity, tutorial mirrors, and example counterpart status

## Background

Mathematical foundations with full derivations and textbook references.

- [SO(2)](background/so2.md) -- 2D rotation group
- [SE(2)](background/se2.md) -- 2D rigid body transformation group
- [SO(3)](background/so3.md) -- 3D rotation group
- [SE(3)](background/se3.md) -- 3D rigid body transformation group
- [PoE Kinematics](background/poe-kinematics.md) -- Product of Exponentials and screw theory
- [Jacobians](background/jacobians.md) -- space and body Jacobians
- [IK Methods](background/ik-methods.md) -- DLS, Levenberg-Marquardt, SQP algorithms
- [Null-Space Projection](background/null-space.md) -- null-space projection for joint limit avoidance
- [Frame Tags](background/frame-tags.md) -- compile-time frame safety design rationale
- [Notation](background/notation.md) -- notation convention table (Lynch & Park / Barfoot / Siciliano)

## Guides

Task-oriented walkthroughs with worked examples.

- [PoE Walkthrough](guides/poe-walkthrough.md) -- forward kinematics from scratch
- [IK Composition](guides/ik-composition.md) -- stepper + scheduler + solver composition
- [Frame Tags Guide](guides/frame-tags-guide.md) -- using frame tags for compile-time safety

## Examples

Runnable programs demonstrating every feature area. Build with `CARTAN_BUILD_EXAMPLES=ON`.

- [examples/](../examples/) -- lie basics, frame safety, FK/Jacobians, basic IK, IK composition
