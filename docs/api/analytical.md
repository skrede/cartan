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
| `cartan::paden_kahan_1`, `paden_kahan_1_direction`, `paden_kahan_2`, `paden_kahan_3` | `#include <cartan/analytical/paden_kahan.h>` |
| `cartan::paden_kahan_2_result`, `paden_kahan_3_result` | `#include <cartan/analytical/paden_kahan.h>` |
| `cartan::analytical_result`, `analytical_error`, `analytical_failure` | `#include <cartan/analytical/analytical_types.h>` |
| `cartan::length_tolerance`, `direction_tolerance`, `verification_tolerance` | `#include <cartan/analytical/analytical_types.h>` |
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

Failure diagnostic returned via `cartan::expected<..., analytical_error<Scalar>>`.

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
    verification_failed,
    non_finite_input
};

constexpr const char* message(analytical_failure failure);
```

- `unreachable` — target lies outside the mechanism's workspace.
- `degenerate_geometry` — joint geometry violates a subproblem precondition
  (e.g. parallel axes where intersection is required).
- `singular_configuration` — the mechanism is at a kinematic singularity for
  the requested target.
- `verification_failed` — candidate solutions exist but none survived the FK
  back-check.
- `non_finite_input` — an input or candidate joint value is NaN or infinite.
  Named to match `chain_failure::non_finite_input` and
  `ik_failure::non_finite_input`, which name the same defect.

`message()` returns a static diagnostic string; it allocates nothing.

### Tolerance types

```cpp
template <typename Scalar>
class length_tolerance
{
public:
    constexpr explicit length_tolerance(Scalar value);
    constexpr Scalar value() const;
};

template <typename Scalar>
class direction_tolerance
{
public:
    constexpr explicit direction_tolerance(Scalar value);
    constexpr Scalar value() const;
};

template <typename Scalar>
class verification_tolerance
{
public:
    constexpr verification_tolerance(Scalar position, Scalar orientation);
    constexpr Scalar position() const;
    constexpr Scalar orientation() const;
};

template <typename Scalar>
inline constexpr length_tolerance<Scalar> default_length_tolerance_v{Scalar(1e-6)};
template <typename Scalar>
inline constexpr direction_tolerance<Scalar> default_direction_tolerance_v{Scalar(1e-6)};
template <typename Scalar>
inline constexpr verification_tolerance<Scalar> default_verification_tolerance_v{
    Scalar(1e-6), Scalar(1e-6)};
```

A tolerance is named for the quantity it measures. `verification_tolerance`
carries the FK back-check's position (linear unit) and orientation (radians, as
the norm of the residual rotation vector) thresholds separately. The shared FK
back-check every analytical solver filters its candidates through takes one of
these as a **required** parameter with no default, so a solver that holds an
acceptance tolerance and forgets to forward it does not compile; each of the
four solvers stores one and forwards it.

**`length_tolerance` is relative, `direction_tolerance` is absolute.** A
`length_tolerance` residual is compared against `value()` scaled by the larger
of the two displacement norms and one, because round-off in a position residual
grows linearly with distance from the axis: a fixed threshold tightens as `1/r`
and rejects correct answers on a large mechanism. `default_length_tolerance_v`
is therefore one part per million of the working radius — one micrometer at
unit radius, if that radius happens to be read in meters, but the constant
carries no assumption about the unit. A `direction_tolerance` judges unit
direction vectors, for which that same scale factor is exactly one, so the
threshold is absolute and genuinely dimensionless.

`length_tolerance` and `direction_tolerance` do **not** convert to one another
in either direction. Passing one where the other is expected is a compile
error, which is why subproblem 1 has two entry points rather than one that
takes a bare scalar. Both constructors are `explicit` and both values are
private, so a braced scalar — `f(w, u, v, {1e-6})` — does not slip past the
distinction either.

`default_direction_tolerance_v` is calibrated on the round-off measured in the
Pieper wrist decomposition at **double** precision. Single-precision round-off
there reaches the constant's own magnitude, so instantiating the *asymmetric*
wrist path (the branch that reaches `paden_kahan_1_direction`) at `float` is
not supported until the constant is re-measured for it. This is documented, not
enforced: a `float` solver still instantiates and runs.

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
        cartan::expected<
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
cartan::expected<Scalar, analytical_failure>
paden_kahan_1(
    const vector3<Scalar>& omega,
    const vector3<Scalar>& q,
    const vector3<Scalar>& p,
    const vector3<Scalar>& p_prime,
    length_tolerance<Scalar> tolerance = default_length_tolerance_v<Scalar>);
```

Subproblem 1, position form: rotation about a single axis. Finds `theta` such
that `exp([omega] * theta)` applied at point `q` maps point `p` to point
`p_prime`.

A rotation about `omega` is the identity on the axial part of a displacement
from the axis and a planar rotation on the perpendicular part, so both must
agree: the two points must share their component along `omega` and their
distance from the axis. The computed angle is then required to reconstruct
`p_prime` from `p`. Failing any of the three returns
`analytical_failure::unreachable`.

Two points coinciding on the axis satisfy the equation for every angle and
return `analytical_failure::singular_configuration`; the function does not
enumerate that continuum. A nonfinite argument returns
`analytical_failure::non_finite_input` and a non-unit `omega` returns
`analytical_failure::degenerate_geometry`.

Reference: Murray, Li and Sastry (1994), Section 3.3, Subproblem 1.

### paden_kahan_1_direction

```cpp
template <typename Scalar>
cartan::expected<Scalar, analytical_failure>
paden_kahan_1_direction(
    const vector3<Scalar>& omega,
    const vector3<Scalar>& u,
    const vector3<Scalar>& u_prime,
    direction_tolerance<Scalar> tolerance = default_direction_tolerance_v<Scalar>);
```

Subproblem 1 for unit direction vectors about an axis through the origin. Same
conditions and same failure vocabulary as the position form, but every residual
it judges is a difference of unit directions and so is dimensionless — hence the
`direction_tolerance` and the absence of an axis-point argument. This is the
form the Pieper wrist decomposition uses.

Reference: Murray, Li and Sastry (1994), Section 3.3, Subproblem 1.

### paden_kahan_2

```cpp
template <typename Scalar>
cartan::expected<paden_kahan_2_result<Scalar>, analytical_failure>
paden_kahan_2(
    const vector3<Scalar>& omega1,
    const vector3<Scalar>& omega2,
    const vector3<Scalar>& q,
    const vector3<Scalar>& p,
    const vector3<Scalar>& p_prime,
    length_tolerance<Scalar> tolerance = default_length_tolerance_v<Scalar>);
```

Subproblem 2: two successive rotations about intersecting axes. Finds
`(theta1, theta2)` such that `exp([omega1]*theta1) * exp([omega2]*theta2)`
applied at point `q` maps `p` to `p_prime`. Axes `omega1` and `omega2` must
intersect at `q`. Returns up to 2 solution pairs. The tolerance is forwarded to
both internal subproblem-1 calls, so every condition subproblem 1 enforces is
enforced on each candidate pair here too. A nonfinite argument returns
`analytical_failure::non_finite_input` and a non-unit `omega1` or `omega2`
returns `analytical_failure::degenerate_geometry`; a candidate pair that fails
any subproblem-1 condition is skipped, and `analytical_failure::unreachable` is
reported when no candidate survives.

Reference: Murray, Li and Sastry (1994), Section 3.3.2.

### paden_kahan_3

```cpp
template <typename Scalar>
cartan::expected<paden_kahan_3_result<Scalar>, analytical_failure>
paden_kahan_3(
    const vector3<Scalar>& omega,
    const vector3<Scalar>& q,
    const vector3<Scalar>& p,
    const vector3<Scalar>& p_prime,
    Scalar delta,
    length_tolerance<Scalar> tolerance = default_length_tolerance_v<Scalar>);
```

Subproblem 3: rotation with distance constraint. Finds `theta` such that
`|| exp([omega]*theta) * p - p_prime || = delta`, where rotation is about
axis `omega` through point `q`. Returns up to 2 solutions.

When either point lies on the axis the achieved distance is constant in
`theta`, so the constraint is met by every angle or by none: within `tolerance`
of `delta` the result is `analytical_failure::singular_configuration`, outside
it `analytical_failure::unreachable`.

A nonfinite argument — including `delta` — returns
`analytical_failure::non_finite_input`, and a non-unit `omega` returns
`analytical_failure::degenerate_geometry`. Without those guards every
comparison in the body is false against a NaN and the two-solution branch
reports `count == 2` NaN angles as a success.

Reference: Murray, Li and Sastry (1994), Section 3.3.3.

## planar_2r_solver

Closed-form IK for a planar 2R mechanism (two revolute joints whose axes are
parallel and define a common mechanism plane). Returns up to 2 solutions
("elbow up" / "elbow down").

```cpp
template <chain Chain>
class planar_2r_solver;
```

The `Chain` type must satisfy the `chain` concept (e.g. `static_chain` or a
`kinematic_chain`, fixed or dynamic). It must model a two-joint mechanism with
both joints revolute. The joint count and revolute-only requirement are checked
at construction; a chain that violates them yields a solver that fails every
`solve` with `analytical_failure::degenerate_geometry`.

```cpp
using chain_type = Chain;
using scalar_type = typename Chain::scalar_type;
static constexpr int joints = 2;
static constexpr int max_solutions = 2;
```

### Constructor

```cpp
explicit planar_2r_solver(
    const chain_type& chain,
    verification_tolerance<scalar_type> tolerance
        = default_verification_tolerance_v<scalar_type>);

static cartan::expected<planar_2r_solver, analytical_error<scalar_type>>
make(const chain_type& chain,
     verification_tolerance<scalar_type> tolerance
         = default_verification_tolerance_v<scalar_type>);
```

Pre-computes the link lengths, the base point, and an orthonormal basis for
the mechanism plane from the chain's screw axes and home pose. Construction
is `O(1)` in the chain's joint count. `make` additionally rejects a chain that
is not two revolute joints or that has a zero-length link. The tolerance is the
bound the FK back-check applies to each candidate: `position()` is a distance
in the chain's linear unit and `orientation()` an angle in radians. This solver
solves position only, so only `position()` gates its results.

### Method

```cpp
cartan::expected<analytical_result<scalar_type, 2, 2>, analytical_error<scalar_type>>
solve(const se3<scalar_type>& target) const;
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
template <chain Chain>
class spatial_3r_solver;
```

The `Chain` type must satisfy the `chain` concept and model a three-joint
mechanism. The solver requires that the first two joint axes intersect at a common point
(the standard configuration for 3R mechanisms, e.g. spherical wrists with
an offset third joint).

```cpp
using chain_type = Chain;
using scalar_type = typename Chain::scalar_type;
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
explicit spatial_3r_solver(
    const chain_type& chain,
    verification_tolerance<scalar_type> tolerance
        = default_verification_tolerance_v<scalar_type>);
```

Captures the chain by value for use during `solve`. Extracts the common
intersection point of the first two joint axes for use by the subproblem
decomposition. The tolerance's `position()` field bounds the FK back-check's
position residual and is forwarded as the `length_tolerance` of both
subproblems; `orientation()` is unused, because this solver solves position
only.

### Method

```cpp
cartan::expected<analytical_result<scalar_type, 3, 4>, analytical_error<scalar_type>>
solve(const se3<scalar_type>& target) const;
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
template <chain Chain>
class pieper_6r_solver;
```

The `Chain` type must satisfy the `chain` concept and model a six-joint
mechanism. The wrist
center decomposition assumes joints 4, 5, 6 share a common intersection
point (Pieper geometry). Industrial 6R arms commonly satisfy this
constraint (KR6 R900, PUMA 560, ABB IRB120 with appropriate geometry, etc.).

```cpp
using chain_type = Chain;
using scalar_type = typename Chain::scalar_type;
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
explicit pieper_6r_solver(
    const chain_type& chain,
    verification_tolerance<scalar_type> tolerance
        = default_verification_tolerance_v<scalar_type>);

static cartan::expected<pieper_6r_solver, analytical_error<scalar_type>>
make(const chain_type& chain,
     verification_tolerance<scalar_type> tolerance
         = default_verification_tolerance_v<scalar_type>);

static constexpr scalar_type default_position_tolerance;
static constexpr scalar_type default_orientation_tolerance;
```

Captures the chain by value. Pre-computes the wrist-center geometry from
the joint-4/5/6 screw axes. The tolerance's `position()` field is a distance in
the chain's linear unit and `orientation()` an angle in radians; the FK
back-check compares each residual against its own field, so a length never
gates an angle. `make` validates the Pieper preconditions before returning a
solver and judges both of them — the shoulder-axis gap and the wrist
sphericity — against `position()`, the same threshold the back-check applies,
so an admitted chain is solvable to the bound the caller asked for.

### Method

```cpp
cartan::expected<analytical_result<scalar_type, 6, 8>, analytical_error<scalar_type>>
solve(const se3<scalar_type>& target) const;
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
  subproblem decomposition but fail the FK back-check are dropped. The bound
  is the solver's own `verification_tolerance`, not a module-wide default:
  the position residual is judged against its `position()` field and the
  orientation residual against `orientation()`. If all candidates fail
  verification, the solver returns
  `analytical_failure::verification_failed`.

## See also

- [IK Reference](ik.md) — iterative IK runners and policies.
- [Background: IK Methods](../background/ik-methods.md) — theory survey.
- [Background: PoE Kinematics](../background/poe-kinematics.md) — the
  Product of Exponentials formulation that the solvers consume from
  `static_chain`.
