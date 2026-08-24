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
| `cartan::opw_6r_solver`, `cartan::opw_parameters` | `#include <cartan/analytical/solver_opw.h>` |
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

<!-- cartan:unbuilt kind=declaration -->
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

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <typename Scalar>
struct analytical_error
{
    analytical_failure reason;
    std::optional<Scalar> workspace_distance;
};
```

`reason` names the failure mode. `workspace_distance` is present only where a
geometric inequality was evaluated and failed, and is then a deficit measured at
such an inequality in the chain's linear unit; it is absent for every other
failure. Absence is not zero: a target sitting exactly on the workspace boundary
has a deficit of zero, so zero cannot also stand for "no magnitude was computed".
A degenerate geometry, a singular configuration, a nonfinite input, and a failed
back-check all carry no magnitude.

Where several inequalities were evaluated the value is a **lower bound on the
motion the target needs**, not a per-reason figure. Inequalities that must all
hold contribute the largest of their deficits, and inequalities that are
alternatives contribute the smallest of theirs. One consequence is worth stating
plainly: a solver that fails a check computing no deficit, having already
measured one at an earlier check, reports what it measured — so an absent
magnitude means "nothing was measured anywhere", while a present one names a
distance the target is short by and not the inequality that ended the solve.

A reason decided by a Paden-Kahan subproblem and forwarded carries no magnitude
either, whichever reason it is: the inequality that failed is inside the
subproblem, whose error channel has no payload for a deficit, so a forwarding
solver has nothing to report and substitutes nothing in its place. Every
magnitude that is present was measured at a geometric inequality: the OPW
lateral-offset guard reports the radial difference between the offset and the
wrist center's radius in that plane, and the shoulder-wrist reach guard reports
the smaller of the two shoulder families' shortfalls, the two families being
alternatives rather than joint requirements. Those two guards are joint
requirements of each other, so a target failing both reports the larger.

### analytical_failure

<!-- cartan:unbuilt kind=declaration -->
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
  the requested target, or the decomposition broke down and placed no candidate
  to check.
- `verification_failed` — candidates were constructed and every one of them was
  rejected by the FK back-check. This is a rejection, not a proof that no
  solution exists.
- `non_finite_input` — an input or candidate joint value is NaN or infinite.
  Named to match `chain_failure::non_finite_input` and
  `ik_failure::non_finite_input`, which name the same defect.

`message()` returns a static diagnostic string; it allocates nothing.

### Tolerance types

<!-- cartan:unbuilt kind=declaration -->
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

template <>
inline constexpr verification_tolerance<float> default_verification_tolerance_v<float>{
    1e-4f, 1e-4f};
```

`default_verification_tolerance_v` is per scalar. `1e-6` is about eight `float`
epsilons, below what a `float` forward map reconstructs, so at that value a
`float` solver reports whole-solve failures on poses it in fact reaches; the
`float` default is `1e-4` and is backed by a recorded sweep in
`tests/unit/analytical_float_tolerance_sweep_test.cpp`. Every other scalar keeps
`1e-6`. The Python bindings are `double`-only, so no Python default changes.

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

<!-- cartan:unbuilt kind=declaration -->
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

<!-- cartan:unbuilt kind=declaration -->
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

<!-- cartan:unbuilt kind=declaration -->
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

<!-- cartan:unbuilt kind=declaration -->
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

<!-- cartan:unbuilt kind=declaration -->
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

<!-- cartan:unbuilt kind=declaration -->
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

<!-- cartan:unbuilt kind=declaration -->
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

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <chain Chain>
class planar_2r_solver;
```

The `Chain` type must satisfy the `chain` concept (e.g. `static_chain` or a
`kinematic_chain`, fixed or dynamic). It must model a two-joint mechanism with
both joints revolute. The constructor is private and `make` is the only way in,
so a chain that violates a requirement is refused rather than yielding a solver.

<!-- cartan:unbuilt kind=declaration -->
```cpp
using chain_type = Chain;
using scalar_type = typename Chain::scalar_type;
static constexpr int joints = 2;
static constexpr int max_solutions = 2;
```

### Factory

<!-- cartan:unbuilt kind=declaration -->
```cpp
static cartan::expected<planar_2r_solver, analytical_error<scalar_type>>
make(const chain_type& chain,
     verification_tolerance<scalar_type> tolerance
         = default_verification_tolerance_v<scalar_type>);
```

The only public construction path. Rejects, with
`analytical_failure::degenerate_geometry` and no `workspace_distance`:

| Rejected chain | Why the derivation needs it |
| --- | --- |
| not exactly two joints, or a non-revolute joint | the closed form is a two-revolute one |
| axes not parallel (their cross product exceeds `sqrt(epsilon)`) | non-parallel axes share no plane, so there is no mechanism plane to solve in |
| home end-effector further than `tolerance.position()` from the plane through the first axis point | the second link length the derivation names is an in-plane length |
| either link shorter than `sqrt(epsilon)` | the law of cosines divides by both |

The second joint needs no such test: a screw axis carries a line rather than a
point on it, and the point recovered as `omega x v` is the foot of the
perpendicular from the origin, so with parallel axes both recovered joint points
are perpendicular to the shared direction and the first link lies in the plane
identically.

An admitted chain then pre-computes the link lengths, the base point, and an
orthonormal basis for the mechanism plane; construction is `O(1)` in the chain's
joint count. The tolerance is also the bound the FK back-check applies to each
candidate: `position()` is a distance in the chain's linear unit and
`orientation()` an angle in radians. This solver solves position only, so only
`position()` gates its results.

### Method

<!-- cartan:unbuilt kind=declaration -->
```cpp
cartan::expected<analytical_result<scalar_type, 2, 2>, analytical_error<scalar_type>>
solve(const se3<scalar_type>& target) const;
```

Solves position-only IK for the given target end-effector pose.

The derivation assumes a target lying in the mechanism plane, and that plane is
the reachable set: a target whose distance out of it exceeds the tolerance's
`position()` is `analytical_failure::unreachable` and carries that distance as
its `workspace_distance`. The in-plane distance is then compared, as a length,
against the reach interval `[|L1 - L2|, L1 + L2]` widened by the same
acceptance length; a target outside the interval is `unreachable` and carries
the deficit at whichever inequality failed. A target within the acceptance
length of a reach boundary is solved rather than refused, so the gate and the
back-check agree on what counts as the same point.

Equal links reaching the base point are `analytical_failure::singular_configuration`,
with no `workspace_distance` and no solutions. The shoulder angle is free there
and the elbow folds back along the first link, so a continuum of configurations
attains the target; one arbitrary member of that continuum would read as a
complete answer, so none is returned. A caller wanting a member of the family
fixes one joint angle and solves for the other.

Otherwise the law of cosines yields the elbow angle and the shoulder angle
follows. Each candidate is FK-verified; only verified solutions are returned.

### Free function

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <typename Scalar, joint_tag... Joints>
cartan::expected<analytical_result<Scalar, 2, 2>, analytical_error<Scalar>>
solve_2r(
    const static_chain<Scalar, Joints...>& chain,
    const se3<Scalar>& target);
```

Convenience wrapper: validates the given chain through `make` and immediately
invokes `solve(target)`. A rejected chain travels out on the same diagnostic
channel a failed solve uses, so the two need no separate handling at the call
site.

Reference: Lynch & Park, Modern Robotics, Section 6.1.2 (planar two-link
inverse kinematics).

## spatial_3r_solver

Closed-form IK for spatial 3R mechanisms using Paden-Kahan subproblems.
Returns up to 4 solutions.

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <chain Chain>
class spatial_3r_solver;
```

The `Chain` type must satisfy the `chain` concept and model a three-joint
mechanism whose first two joint axes meet at a common point (the standard
configuration for 3R mechanisms, e.g. spherical wrists with an offset third
joint). The constructor is private and `make` is the only way in, so a chain
that violates a requirement is refused rather than yielding a solver.

<!-- cartan:unbuilt kind=declaration -->
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

### Factory

<!-- cartan:unbuilt kind=declaration -->
```cpp
static cartan::expected<spatial_3r_solver, analytical_error<scalar_type>>
make(const chain_type& chain,
     verification_tolerance<scalar_type> tolerance
         = default_verification_tolerance_v<scalar_type>);
```

The only public construction path. Rejects, with
`analytical_failure::degenerate_geometry` and no `workspace_distance`:

| Rejected chain | Why the derivation needs it |
| --- | --- |
| not exactly three joints, or a non-revolute joint | the decomposition is a three-revolute one |
| first two axes parallel (their cross product at or below `sqrt(epsilon)`) | a parallel pair has no meeting point, and the closest-approach helpers answer one with a fallback point rather than reporting, so this is tested first |
| first two axes further apart than `tolerance.position()` at closest approach | subproblem 2 references both axes to their meeting point |
| home end-effector within `tolerance.position()` of the third axis | the distance constraint of subproblem 3 is then met by every angle or by none |

An admitted chain captures the chain by value for use during `solve` and derives
the meeting point of the first two axes once, rather than on every solve. The
tolerance's `position()` field bounds the FK back-check's position residual;
`orientation()` is unused, because this solver solves position only. It is not
forwarded to the subproblems, which apply the module's default
`length_tolerance`.

### Method

<!-- cartan:unbuilt kind=declaration -->
```cpp
cartan::expected<analytical_result<scalar_type, 3, 4>, analytical_error<scalar_type>>
solve(const se3<scalar_type>& target) const;
```

Returns up to 4 verified joint configurations achieving the target pose
(position component only; orientation is set by the chain's mechanism).

### Free function

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <typename Scalar, joint_tag... Joints>
cartan::expected<analytical_result<Scalar, 3, 4>, analytical_error<Scalar>>
solve_3r(
    const static_chain<Scalar, Joints...>& chain,
    const se3<Scalar>& target);
```

Convenience wrapper: validates the given chain through `make` and immediately
invokes `solve(target)`. A rejected chain travels out on the same diagnostic
channel a failed solve uses, so the two need no separate handling at the call
site.

Reference: Murray, Li and Sastry, *A Mathematical Introduction to Robotic
Manipulation* (1994), Section 3.3.

## pieper_6r_solver

Closed-form IK for 6R mechanisms with Pieper geometry (last three revolute
axes intersecting at a common wrist center). Returns up to 8 solutions.

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <chain Chain>
class pieper_6r_solver;
```

The `Chain` type must satisfy the `chain` concept and model a six-joint
mechanism. The wrist
center decomposition assumes joints 4, 5, 6 share a common intersection
point (Pieper geometry). Industrial 6R arms commonly satisfy this
constraint (KR6 R900, PUMA 560, ABB IRB120 with appropriate geometry, etc.).

<!-- cartan:unbuilt kind=declaration -->
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

### Factory

<!-- cartan:unbuilt kind=declaration -->
```cpp
static cartan::expected<pieper_6r_solver, analytical_error<scalar_type>>
make(const chain_type& chain,
     verification_tolerance<scalar_type> tolerance
         = default_verification_tolerance_v<scalar_type>);
```

The constructor is private and `make` is the only way in, so a chain the
Pieper decomposition does not support cannot reach a solver. `make` requires
six revolute joints and judges both geometric preconditions — the
shoulder-axis gap between joints 1 and 2, and the sphericity of the joint-4/5/6
wrist — against `position()`, the same threshold the FK back-check applies, so
an admitted chain yields at least one branch that verifies to the bound the
caller asked for. A chain failing any of these is rejected with
`degenerate_geometry` and no magnitude.

The solver captures the chain by value and pre-computes the wrist-center
geometry from the wrist center `make` already derived. The tolerance's
`position()` field is a distance in the chain's linear unit and `orientation()`
an angle in radians; the FK back-check compares each residual against its own
field, so a length never gates an angle.

Admission promises **one** verifying branch, not all eight. Measured on a
near-spherical family whose wrist axes miss the common center by `d`, the
branches' position error spreads over `0.21*d` to `2.04*d`; only the lower
factor is below one. At the module default the family returns eight branches at
`d = 1e-7` and four at `d = 9e-7`. The gate admits `d` strictly below the
position tolerance — it takes `9e-7` and refuses `1e-6` at a tolerance of
`1e-6`.

### Method

<!-- cartan:unbuilt kind=declaration -->
```cpp
cartan::expected<analytical_result<scalar_type, 6, 8>, analytical_error<scalar_type>>
solve(const se3<scalar_type>& target) const;
```

Returns up to 8 verified joint configurations achieving the target SE(3)
pose (both position and orientation).

### Free function

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <typename Scalar, joint_tag... Joints>
cartan::expected<analytical_result<Scalar, 6, 8>, analytical_error<Scalar>>
solve_6r(
    const static_chain<Scalar, Joints...>& chain,
    const se3<Scalar>& target);
```

Convenience wrapper: constructs a `pieper_6r_solver` through `make` and
immediately invokes `solve(target)`. A chain the factory rejects is forwarded
as that same construction failure, so this path applies both geometry gates.

Reference: Lynch & Park, Modern Robotics, Section 6.1.1.
           Murray, Li and Sastry (1994), Section 3.3.

## opw_6r_solver

Closed-form IK for ortho-parallel 6R arms with a spherical wrist and a lateral
shoulder offset — the geometry `pieper_6r_solver`'s shoulder-intersection gate
rejects. Returns up to 8 solutions.

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <chain Chain, typename Verification = opw_verified>
class opw_6r_solver;
```

`Verification` selects the FK back-check: `opw_verified` filters every branch
through it and collapses duplicates, `opw_raw` emits every finite branch
unchecked.

### Factory

<!-- cartan:unbuilt kind=declaration -->
```cpp
static cartan::expected<opw_6r_solver, analytical_error<scalar_type>>
make(const chain_type& chain,
     const opw_parameters<scalar_type>& params,
     verification_tolerance<scalar_type> tolerance
         = default_verification_tolerance_v<scalar_type>,
     scalar_type singularity_tolerance = default_singularity_tolerance);

static constexpr scalar_type default_position_tolerance;
static constexpr scalar_type default_orientation_tolerance;
static constexpr scalar_type default_singularity_tolerance;
```

There is no public constructor: `make` validates the OPW preconditions (six
revolute joints, axis 1 perpendicular to axis 2, axis 2 parallel to axis 3, and
a spherical wrist within `tolerance.position()`) and is the only way in.

`tolerance` is the FK back-check's acceptance bound, `position()` a distance in
the chain's linear unit and `orientation()` an angle in radians. `position()`
also bounds the lateral-offset cylinder gate inside `solve`, which judges the
wrist center's radius in that plane against the offset as two lengths.
`singularity_tolerance` is a separate scalar and stays one: it thresholds
`|sin(theta5)|`, a dimensionless quantity, below which the wrist fold path is
taken. Its default is pinned empirically rather than copied from the reference
implementation's `1e-6`; see the constant's comment in the header.

The two `default_*_tolerance` constants republish the module default's two
fields one scalar at a time, because the Python bindings need each as a default
argument and cannot spell the two-field type. Prefer
`default_verification_tolerance_v` in C++.

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
