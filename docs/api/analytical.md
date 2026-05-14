# analytical

Headers under `cartan/analytical/` (sublib `cartan-analytical`). Public symbols
live in the `cartan::` namespace. Link with `cartan::analytical` (or the
convenience target `cartan::cartan`). The umbrella header
`<cartan/analytical.h>` includes all closed-form solvers and subproblem
utilities.

Closed-form inverse-kinematics solvers built on the Paden-Kahan subproblem
decomposition. Covers planar 2R mechanisms, spatial 3R mechanisms, and 6R
mechanisms with Pieper geometry (last three revolute axes intersecting at a
common wrist center). All solvers verify their candidate joint configurations
against the chain's forward kinematics internally; only FK-verified solutions
are returned. Failures distinguish workspace unreachability, degenerate
geometry, kinematic singularity, and verification miss.

See [IK Methods](../background/ik-methods.md) | [PoE Kinematics](../background/poe-kinematics.md)

## Headers

| Form | Header |
|------|--------|
| All analytical | `#include <cartan/analytical.h>` |
| `cartan::pieper_6r_solver`, `cartan::solve_6r` | `#include <cartan/analytical/solver_6r.h>` |
| `cartan::spatial_3r_solver`, `cartan::solve_3r` | `#include <cartan/analytical/solver_3r.h>` |
| `cartan::planar_2r_solver`, `cartan::solve_2r` | `#include <cartan/analytical/solver_2r.h>` |
| `cartan::paden_kahan_1`, `paden_kahan_2`, `paden_kahan_3` | `#include <cartan/analytical/paden_kahan.h>` |
| `cartan::paden_kahan_2_result`, `paden_kahan_3_result` | `#include <cartan/analytical/paden_kahan.h>` |
| `cartan::analytical_result`, `analytical_error`, `analytical_failure` | `#include <cartan/analytical/analytical_types.h>` |
| `cartan::analytical_solver` concept | `#include <cartan/analytical/analytical_solver.h>` |

## Result Types

### analytical_result

Multi-solution result for an analytical solver.

```cpp
template <typename Scalar, int N, int MaxSolutions>
struct analytical_result
{
    using position_type = Eigen::Vector<Scalar, N>;

    std::array<position_type, MaxSolutions> solutions;
    int count{0};

    auto begin() const;
    auto end() const;
};
```

`N` is the joint count (compile-time). `MaxSolutions` is the per-solver upper
bound on the number of solutions: 2 for `planar_2r_solver`, 4 for
`spatial_3r_solver`, 8 for `pieper_6r_solver`. The `solutions` array is sized
to `MaxSolutions`; only the first `count` entries are populated and
FK-verified. Use the `begin()`/`end()` iterators to traverse the populated
subset (idiomatic ranged-`for`).

### analytical_error

Failure diagnostic returned via `std::expected<..., analytical_error<Scalar>>`.

```cpp
template <typename Scalar>
struct analytical_error
{
    analytical_failure reason;
    Scalar workspace_distance{};
};
```

`reason` names the failure mode. `workspace_distance` is the magnitude (in the
chain's linear unit) by which the target exceeds the reachable workspace when
`reason` is `analytical_failure::unreachable`, and zero for other failure
modes.

### analytical_failure

```cpp
enum class analytical_failure
{
    unreachable,
    degenerate_geometry,
    singular_configuration,
    verification_failed
};
```

- `unreachable` — target lies outside the mechanism's workspace.
- `degenerate_geometry` — joint geometry violates a subproblem precondition
  (e.g. parallel axes where intersection is required).
- `singular_configuration` — the mechanism is at a kinematic singularity for
  the requested target.
- `verification_failed` — candidate solutions exist but none survived the FK
  back-check.

### paden_kahan_2_result

```cpp
template <typename Scalar>
struct paden_kahan_2_result
{
    std::array<std::pair<Scalar, Scalar>, 2> solutions;
    int count{0};
};
```

Result of Paden-Kahan subproblem 2 (two intersecting axes). Carries up to 2
`(theta1, theta2)` pairs; only the first `count` entries are populated.

### paden_kahan_3_result

```cpp
template <typename Scalar>
struct paden_kahan_3_result
{
    std::array<Scalar, 2> solutions;
    int count{0};
};
```

Result of Paden-Kahan subproblem 3 (rotation with distance constraint).
Carries up to 2 angle solutions; only the first `count` entries are populated.

## analytical_solver concept

```cpp
template <typename S>
concept analytical_solver = requires
{
    typename S::chain_type;
    typename S::scalar_type;
    { S::joints } -> std::convertible_to<int>;
    { S::max_solutions } -> std::convertible_to<int>;
} && requires(const S& s, const se3<typename S::scalar_type>& target)
{
    { s.solve(target) } -> std::same_as<
        std::expected<
            analytical_result<typename S::scalar_type, S::joints, S::max_solutions>,
            analytical_error<typename S::scalar_type>>>;
};
```

A conforming type exposes `chain_type` (the chain type it solves over),
`scalar_type` (floating-point), `joints` (compile-time joint count), and
`max_solutions` (per-solver upper bound). The single `solve(target)` method
returns `analytical_result` on success or `analytical_error` on failure.
Solvers must FK-verify their candidates internally; only verified solutions
are reported to the caller. `planar_2r_solver`, `spatial_3r_solver`, and
`pieper_6r_solver` are the in-tree conforming implementations.

## Paden-Kahan Subproblems

### paden_kahan_1

```cpp
template <typename Scalar>
std::expected<Scalar, analytical_failure>
paden_kahan_1(
    const vector3<Scalar>& omega,
    const vector3<Scalar>& q,
    const vector3<Scalar>& p,
    const vector3<Scalar>& p_prime);
```

Subproblem 1: rotation about a single axis. Finds `theta` such that
`exp([omega] * theta)` applied at point `q` maps point `p` to point `p_prime`.
Both `p` and `p_prime` must be equidistant from the axis of rotation; if they
are not, returns `analytical_failure::unreachable`. Returns a single angle on
success.

Reference: Murray, Li and Sastry (1994), Section 3.3.1.

### paden_kahan_2

```cpp
template <typename Scalar>
std::expected<paden_kahan_2_result<Scalar>, analytical_failure>
paden_kahan_2(
    const vector3<Scalar>& omega1,
    const vector3<Scalar>& omega2,
    const vector3<Scalar>& q,
    const vector3<Scalar>& p,
    const vector3<Scalar>& p_prime);
```

Subproblem 2: two successive rotations about intersecting axes. Finds
`(theta1, theta2)` such that `exp([omega1]*theta1) * exp([omega2]*theta2)`
applied at point `q` maps `p` to `p_prime`. Axes `omega1` and `omega2` must
intersect at `q`. Returns up to 2 solution pairs.

Reference: Murray, Li and Sastry (1994), Section 3.3.2.

### paden_kahan_3

```cpp
template <typename Scalar>
std::expected<paden_kahan_3_result<Scalar>, analytical_failure>
paden_kahan_3(
    const vector3<Scalar>& omega,
    const vector3<Scalar>& q,
    const vector3<Scalar>& p,
    const vector3<Scalar>& p_prime,
    Scalar delta);
```

Subproblem 3: rotation with distance constraint. Finds `theta` such that
`|| exp([omega]*theta) * p - p_prime || = delta`, where rotation is about
axis `omega` through point `q`. Returns up to 2 solutions.

Reference: Murray, Li and Sastry (1994), Section 3.3.3.

## planar_2r_solver

Closed-form IK for a planar 2R mechanism (two revolute joints whose axes are
parallel and define a common mechanism plane). Returns up to 2 solutions
("elbow up" / "elbow down").

```cpp
template <typename Scalar, joint_tag... Joints>
class planar_2r_solver;
```

The `Joints` parameter pack must contain exactly two joint tags, all
revolute (`is_revolute == true`). Verified at compile time via
`static_assert`.

```cpp
using chain_type = static_chain<Scalar, Joints...>;
static constexpr int joints = 2;
static constexpr int max_solutions = 2;
```

### Constructor

```cpp
explicit planar_2r_solver(const chain_type& chain);
```

Pre-computes the link lengths, the base point, and an orthonormal basis for
the mechanism plane from the chain's screw axes and home pose. Construction
is `O(1)` in the chain's joint count.

### Method

```cpp
std::expected<analytical_result<Scalar, 2, 2>, analytical_error<Scalar>>
solve(const se3<Scalar>& target) const;
```

Solves position-only IK for the given target end-effector pose. The
target's translation is projected onto the mechanism plane; the law of
cosines yields the elbow angle, and the shoulder angle follows.
Solutions outside `[L1 - L2, L1 + L2]` reach are rejected as
`analytical_failure::unreachable`. Each candidate is FK-verified; only
verified solutions are returned.

### Free function

```cpp
template <typename Scalar, joint_tag... Joints>
auto solve_2r(
    const static_chain<Scalar, Joints...>& chain,
    const se3<Scalar>& target);
```

Convenience wrapper: constructs a `planar_2r_solver` from the given chain
and immediately invokes `solve(target)`.

Reference: Lynch & Park, Modern Robotics, Section 6.1.2 (planar two-link
inverse kinematics).

## spatial_3r_solver

Closed-form IK for spatial 3R mechanisms using Paden-Kahan subproblems.
Returns up to 4 solutions.

```cpp
template <typename Scalar, joint_tag... Joints>
class spatial_3r_solver;
```

The `Joints` parameter pack must contain exactly three joint tags. The
solver requires that the first two joint axes intersect at a common point
(the standard configuration for 3R mechanisms, e.g. spherical wrists with
an offset third joint).

```cpp
using chain_type = static_chain<Scalar, Joints...>;
static constexpr int joints = 3;
static constexpr int max_solutions = 4;
```

Decomposition:

1. Subproblem 3 finds up to 2 candidates for `theta3` via a distance
   constraint.
2. Subproblem 2 finds up to 2 `(theta1, theta2)` pairs for each `theta3`
   candidate.
3. All candidates are FK-verified; only verified solutions are returned.

### Constructor

```cpp
explicit spatial_3r_solver(const chain_type& chain);
```

Captures the chain by value for use during `solve`. Extracts the common
intersection point of the first two joint axes for use by the subproblem
decomposition.

### Method

```cpp
std::expected<analytical_result<Scalar, 3, 4>, analytical_error<Scalar>>
solve(const se3<Scalar>& target) const;
```

Returns up to 4 verified joint configurations achieving the target pose
(position component only; orientation is set by the chain's mechanism).

### Free function

```cpp
template <typename Scalar, joint_tag... Joints>
auto solve_3r(
    const static_chain<Scalar, Joints...>& chain,
    const se3<Scalar>& target);
```

Convenience wrapper: constructs a `spatial_3r_solver` from the given
chain and immediately invokes `solve(target)`.

Reference: Murray, Li and Sastry, *A Mathematical Introduction to Robotic
Manipulation* (1994), Section 3.3.

## pieper_6r_solver

Closed-form IK for 6R mechanisms with Pieper geometry (last three revolute
axes intersecting at a common wrist center). Returns up to 8 solutions.

```cpp
template <typename Scalar, joint_tag... Joints>
class pieper_6r_solver;
```

The `Joints` parameter pack must contain exactly six joint tags. The wrist
center decomposition assumes joints 4, 5, 6 share a common intersection
point (Pieper geometry). Industrial 6R arms commonly satisfy this
constraint (KR6 R900, UR5, ABB IRB120 with appropriate geometry, etc.).

```cpp
using chain_type = static_chain<Scalar, Joints...>;
static constexpr int joints = 6;
static constexpr int max_solutions = 8;
```

Decomposition:

1. Inverse position: find joints 1-3 from the wrist-center position using
   subproblem 3 + subproblem 2/1 (up to 4 candidates).
2. Inverse orientation: for each position solution, find joints 4-6 from
   the wrist rotation matrix via Euler-angle extraction (up to 2 per
   position solution = 8 total).
3. All solutions are FK-verified with both position and orientation
   checks.

### Constructor

```cpp
explicit pieper_6r_solver(const chain_type& chain);
```

Captures the chain by value. Pre-computes the wrist-center geometry from
the joint-4/5/6 screw axes.

### Method

```cpp
std::expected<analytical_result<Scalar, 6, 8>, analytical_error<Scalar>>
solve(const se3<Scalar>& target) const;
```

Returns up to 8 verified joint configurations achieving the target SE(3)
pose (both position and orientation).

### Free function

```cpp
template <typename Scalar, joint_tag... Joints>
auto solve_6r(
    const static_chain<Scalar, Joints...>& chain,
    const se3<Scalar>& target);
```

Convenience wrapper: constructs a `pieper_6r_solver` from the given
chain and immediately invokes `solve(target)`.

Reference: Lynch & Park, Modern Robotics, Section 6.1.1.
           Murray, Li and Sastry (1994), Section 3.3.

## Edge Cases

- **Pieper-incompatible 6R geometry:** If the last three axes do not
  intersect within the wrist-finder's tolerance, the wrist decomposition
  fails and `solve` returns `analytical_failure::degenerate_geometry`. For
  such mechanisms, fall back to an iterative IK solver from
  `docs/api/ik.md`.
- **Multiple equivalent solutions:** Analytical solvers enumerate all
  algebraically distinct joint configurations. A downstream selection
  policy (closest-to-seed, distance-to-midpoint, etc.) chooses among
  them.
- **FK verification gate:** Candidates that algebraically satisfy the
  subproblem decomposition but fail the FK back-check (within the
  solver's pose-tolerance) are dropped. If all candidates fail
  verification, the solver returns
  `analytical_failure::verification_failed`.

## See also

- [IK Reference](ik.md) — iterative IK runners and policies.
- [Background: IK Methods](../background/ik-methods.md) — theory survey.
- [Background: PoE Kinematics](../background/poe-kinematics.md) — the
  Product of Exponentials formulation that the solvers consume from
  `static_chain`.
