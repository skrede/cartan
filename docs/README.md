# Cartan Documentation

## Getting Started

- [Getting Started](getting-started.md) &mdash; zero to compiling in 5 minutes
- [Benchmarks](benchmarks.md) &mdash; head-to-head FK, Jacobian, and IK comparisons against TRAC-IK/KDL and Pinocchio

## API Reference

- [lie](api/lie.md) &mdash; SO(2), SE(2), SO(3), SE(3) Lie group classes.
- [frames](api/frames.md) &mdash; Compile-time frame-tagged wrappers (transform, rotation, twist, wrench).
- [chain](api/chain.md) &mdash; Kinematic chain, screw axis, joint limits, joint state.
- [kinematics](api/kinematics.md) &mdash; Forward kinematics, space/body Jacobians, velocity kinematics.
- [ik](api/ik.md) &mdash; IK solvers (DLS, LM, SQP) and variadic-policy racing runners (`basic_ik_runner`, `restart_wrapper`).
- [analytical](api/analytical.md) &mdash; closed-form solvers (Pieper 6R, planar 2R, 3R, OPW) and Paden-Kahan subproblems.
- [python](python.md) &mdash; Python install, API parity, tutorial mirrors, and example counterpart status.

## Background

Mathematical foundations with full derivations and textbook references.

- [SO(2)](background/so2.md) &mdash; 2D rotation group
- [SE(2)](background/se2.md) &mdash; 2D rigid body transformation group
- [SO(3)](background/so3.md) &mdash; 3D rotation group
- [SE(3)](background/se3.md) &mdash; 3D rigid body transformation group
- [PoE Kinematics](background/poe-kinematics.md) &mdash; Product of Exponentials and screw theory
- [Jacobians](background/jacobians.md) &mdash; space and body Jacobians
- [IK Methods](background/ik-methods.md) &mdash; DLS, Levenberg-Marquardt, SQP algorithms
- [Null-Space Projection](background/null-space.md) &mdash; null-space projection for joint limit avoidance
- [Frame Tags](background/frame-tags.md) &mdash; compile-time frame safety design rationale
- [Notation](background/notation.md) &mdash; notation convention table (Lynch & Park / Barfoot / Siciliano)

## Guides

Task-oriented walkthroughs with worked examples.

- [PoE Walkthrough](guides/poe-walkthrough.md) &mdash; forward kinematics from scratch
- [IK Composition](guides/ik-composition.md) &mdash; variadic-policy racing via `basic_ik_runner` + `restart_wrapper`
- [Frame Tags Guide](guides/frame-tags-guide.md) &mdash; using frame tags for compile-time safety

## Examples

Runnable programs demonstrating the core feature areas. Build with `CARTAN_BUILD_EXAMPLES=ON`.

- [examples/](../examples/) &mdash; lie basics, frame safety, FK/Jacobians, basic IK, IK composition
