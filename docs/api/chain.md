# chain

Headers under `cartan/serial/chain/` (sublib `cartan-serial-chain`). Public
symbols live in the `cartan::` namespace. Link with `cartan::serial-chain` (or
the convenience target `cartan::cartan`).

Kinematic chain representation using Product of Exponentials (PoE) formulation.
Defines screw axes, joint limits, joint state, joint tags, and two chain
containers: `kinematic_chain` (compile-time or runtime joint count) and
`static_chain` (joint-type tags fully encoded in the template parameter pack).
Both satisfy the `chain` concept consumed by the FK, Jacobian, and IK modules.

See [PoE Kinematics](../background/poe-kinematics.md)

## Headers

| Form | Header |
|------|--------|
| All chain types | `#include <cartan/serial_chain.h>` |
| `cartan::kinematic_chain` | `#include <cartan/serial/chain/kinematic_chain.h>` |
| `cartan::static_chain` | `#include <cartan/serial/chain/static_chain.h>` |
| `cartan::screw_axis` | `#include <cartan/serial/chain/screw_axis.h>` |
| `cartan::joint_limits` | `#include <cartan/serial/chain/joint_limits.h>` |
| `cartan::joint_state` | `#include <cartan/serial/chain/joint_state.h>` |
| `cartan::joint_kind`, `cartan::detect_joint_kind` | `#include <cartan/serial/chain/joint_kind.h>` |
| `cartan::revolute_x/y/z`, `cartan::prismatic_x/y/z` joint tags | `#include <cartan/serial/chain/joint_tags.h>` |
| `cartan::chain` concept, `cartan::joint_tag` concept | `#include <cartan/serial/chain/chain_concept.h>` |
| `cartan::dynamic` | `#include <cartan/types.h>` |
| `cartan::detail::storage_t` | `#include <cartan/serial/chain/storage_trait.h>` |

## screw_axis

Screw axis for a kinematic joint in PoE form. Revolute joints have a unit
rotation axis (`||omega|| = 1`); prismatic joints have `omega = 0` and unit
translation direction (`||v|| = 1`).

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <typename Scalar = double>
class screw_axis;
```

### Static Factory Methods

<!-- cartan:unbuilt kind=declaration -->
```cpp
static screw_axis revolute(const vector3<Scalar>& axis, const vector3<Scalar>& point);
```

Construct a revolute joint screw axis. `axis` is the rotation axis direction
(will be normalized). `point` is a point on the rotation axis. The linear
component is computed as `v = -omega x point`.

<!-- cartan:unbuilt kind=declaration -->
```cpp
static screw_axis prismatic(const vector3<Scalar>& direction);
```

Construct a prismatic joint screw axis. `direction` is the translation
direction (will be normalized). Sets `omega = 0`.

<!-- cartan:unbuilt kind=declaration -->
```cpp
static cartan::expected<screw_axis, lie_failure> from_vector(const vector6<Scalar>& vec);
```

Construct from a 6-vector `(omega, v)` with unit constraint validation. For
revolute axes, requires `||omega|| = 1`. For prismatic axes (`omega ~ 0`),
requires `||v|| = 1`. Returns `cartan::unexpected(lie_failure::non_finite_input)`
if any component is NaN or infinite — tested first, before the branch that
distinguishes revolute from prismatic — and
`cartan::unexpected(lie_failure::non_unit_screw_axis)` if a finite axis violates
its unit constraint (see [Error Handling](lie.md#error-handling)).

### Member Methods

<!-- cartan:unbuilt kind=declaration -->
```cpp
const vector3<Scalar>& omega() const;
const vector3<Scalar>& v() const;
```

Angular and linear velocity components.

<!-- cartan:unbuilt kind=declaration -->
```cpp
vector6<Scalar> to_vector() const;
```

Export as 6-vector `(omega, v)` in omega-first convention.

<!-- cartan:unbuilt kind=declaration -->
```cpp
bool is_revolute() const;
bool is_prismatic() const;
```

Joint type queries. Revolute when `||omega||^2 > epsilon`; prismatic
otherwise.

## joint_limits

Joint limits with required position bounds and optional dynamic limits. The five
values are private and read-only, and `make` is the only supported way to obtain
one, so no ordinary expression constructs an invalid set of limits or assigns a
valid one back into an invalid state. The type is still trivially copyable, so
`std::bit_cast` and `std::memcpy` remain well-defined routes around that.

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <typename Scalar = double>
class joint_limits
{
public:
    static cartan::expected<joint_limits, chain_failure> make(
        Scalar position_min,
        Scalar position_max,
        std::optional<Scalar> velocity_max = std::nullopt,
        std::optional<Scalar> effort_max = std::nullopt,
        std::optional<Scalar> acceleration_max = std::nullopt);

    Scalar position_min() const;
    Scalar position_max() const;
    std::optional<Scalar> velocity_max() const;
    std::optional<Scalar> effort_max() const;
    std::optional<Scalar> acceleration_max() const;

    std::optional<bool> contains(Scalar position) const;
    bool contains_or(Scalar position, bool when_nonfinite) const;
};
```

Construction, with both outcomes handled. This block is compiled by the build's
documentation-snippet gate, so a signature change here breaks the build rather
than rotting:

<!-- cartan:snippet name=limits-construction tu -->
```cpp
#include <cartan/serial_chain.h>

#include <iostream>

int main()
{
    auto lim = cartan::joint_limits<double>::make(-3.14, 3.14);
    if (!lim.has_value())
    {
        std::cerr << "position bounds rejected: "
                  << cartan::message(lim.error()) << "\n";
        return 1;
    }

    auto all = cartan::joint_limits<double>::make(-3.14, 3.14, 2.0, 50.0, 10.0);
    if (!all.has_value())
    {
        std::cerr << "full limits rejected: "
                  << cartan::message(all.error()) << "\n";
        return 1;
    }

    std::cout << "range " << lim->position_min() << " .. " << lim->position_max()
              << ", velocity cap " << *all->velocity_max() << "\n";
    return 0;
}
```

`make` rejects a NaN in any bound, position bounds that do not describe a
non-empty interval, a negative velocity, effort or acceleration bound, and an
infinite dynamic bound. It is not `constexpr`: `std::isnan` and `std::isfinite`
are not constant expressions before C++23 and the compiler floor is C++20.

| Rejection | `chain_failure` |
| --- | --- |
| NaN in any bound; infinite velocity, effort or acceleration | `non_finite_input` |
| bounds that are not an interval -- `position_max < position_min`, and also `(+inf, +inf)` or `(-inf, -inf)`, which an ordering test alone admits because an infinity is not less than itself | `reversed_position_bounds` |
| negative velocity bound | `negative_velocity_limit` |
| negative effort bound | `negative_effort_limit` |
| negative acceleration bound | `negative_acceleration_limit` |

Positive and negative infinity are **legal position bounds**, signed outward:
`(-inf, +inf)` is the unbounded continuous joint the URDF loader writes and the
unbounded-joint helpers below consume, and one bound may be infinite while the
other is finite. `(+inf, +inf)` and `(-inf, -inf)` are refused -- they describe
no interval. The asymmetry with the dynamic bounds is deliberate: no part of the
library treats an infinite velocity, effort or acceleration limit as meaningful.

`contains` returns an empty optional for a non-finite position rather than
`false`. Both bound comparisons are false for a NaN, which would read as
"outside the limits" when the truth is that the question has no answer;
non-finite joint values are rejected upstream at the checked entry points. Note
that `if (lim.contains(q))` tests whether the question was *answerable*, not
whether `q` is in range; `contains_or(q, false)` is the spelling to reach for.

## joint_state

Joint state holding a position vector and an optional velocity vector.
Parameterized by scalar type and joint count `N` (fixed or
`cartan::dynamic`).

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <typename Scalar = double, int N = dynamic>
struct joint_state
{
    position_type position;                   // Joint positions
    std::optional<velocity_type> velocity{};  // Joint velocities (optional)
};
```

For fixed `N`, `position_type` is `Eigen::Vector<Scalar, N>`. For dynamic,
it is `Eigen::VectorX<Scalar>`.

### Methods

<!-- cartan:unbuilt kind=declaration -->
```cpp
static joint_state from_position(const position_type& q);
```

Create a joint state from position only (no velocity).

<!-- cartan:unbuilt kind=declaration -->
```cpp
int num_joints() const;
```

Number of joints in this state.

## Joint Tags

Six compile-time tag types describing the principal revolute and prismatic
joints (axis aligned with `+e_x`, `+e_y`, or `+e_z`). Used as template
parameter packs in `static_chain` to encode joint types at compile time and
enable `if constexpr` dispatch on joint type in FK / Jacobian.

<!-- cartan:unbuilt kind=declaration -->
```cpp
struct revolute_x { static constexpr bool is_revolute = true; /* ... */ };
struct revolute_y { static constexpr bool is_revolute = true; /* ... */ };
struct revolute_z { static constexpr bool is_revolute = true; /* ... */ };
struct prismatic_x { static constexpr bool is_revolute = false; /* ... */ };
struct prismatic_y { static constexpr bool is_revolute = false; /* ... */ };
struct prismatic_z { static constexpr bool is_revolute = false; /* ... */ };
```

Each tag exposes:

- `static constexpr bool is_revolute` — `true` for `revolute_*`, `false` for
  `prismatic_*`.
- A `static constexpr` axis accessor template (`omega<Scalar>()` for
  revolute tags, `direction<Scalar>()` for prismatic tags) returning the
  principal axis as a `vector3<Scalar>`.

## joint_tag concept

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <typename T>
concept joint_tag = requires {
    { T::is_revolute } -> std::convertible_to<bool>;
};
```

Constrains types usable as joint tags in a `static_chain` parameter pack.
A conforming type exposes `static constexpr bool is_revolute`. The six
in-tree tags (`revolute_x/y/z`, `prismatic_x/y/z`) all satisfy the
concept.

## joint_kind

Runtime axis classification used by `kinematic_chain` to dispatch into
the same compile-time specializations as `static_chain`.

<!-- cartan:unbuilt kind=declaration -->
```cpp
enum class joint_kind : std::uint8_t
{
    general = 0,
    revolute_x,
    revolute_y,
    revolute_z,
    prismatic_x,
    prismatic_y,
    prismatic_z,
};
```

- `general` is the catch-all for arbitrary screw axes that do not match any
  principal-axis pattern; it routes back to the generic `se3::exp` path.
- The other six values correspond to the six joint-tag types and select the
  matching specialization in FK / Jacobian.

### detect_joint_kind

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <typename Scalar>
joint_kind detect_joint_kind(const screw_axis<Scalar>& axis);
```

Inspect a `screw_axis` and return its `joint_kind`. Recognizes axes whose
`omega` (revolute) or `v` (prismatic) is `±e_x`, `±e_y`, or `±e_z` to
within a per-component deviation of `1e-9`. The sign is irrelevant:
downstream specializations read the magnitude from the axis itself. All
other axes return `joint_kind::general`.

**This is not a finiteness gate and must not be used as one.** Only the
component the branch tests is examined. A nonfinite `omega` classifies as
`general`, because no comparison against a NaN holds; but a revolute axis
with an exactly principal `omega` and a nonfinite `v` is reported as that
principal kind. `static_chain::make` and the `kinematic_chain` constructor
test the whole six-vector — this function answers a different question.

The tolerance is absolute and the same in every scalar type, rather than
derived from machine precision. A precision-derived threshold is `1.5e-8`
in `double` but `3.4e-4` in `float`, which would silently discard a
misalignment of about a hundredth of a degree in single precision — the
same code safe in one scalar and unsafe in another.

`1e-9` is an engineering judgment about physical meaninglessness, not a
statement about the arithmetic. One nanoradian is about `5.7e-8` degrees;
where a robot axis actually sits is fixed by machining and assembly, which
are coarser than that by orders of magnitude. A deviation below `1e-9`
therefore cannot describe a real misalignment — it is residue from
composing the rotations that produced the axis.

Snapping such an axis is an approximation, and its cost is small. Measured
on the fixture set, the induced end-effector error follows

    |Δp| ≈ 1.4 · n · L · δ      |Δθ| ≈ 1.7 · n · δ

for a joint count `n`, the largest moment arm `L` in the chain, and the
tolerance `δ`. **Both constants are measured, not derived** — they are
empirical fits over the fixture chains, not bounds proved from the PoE
product, which is why these are written as approximations and not as
inequalities. Worked once at the largest chain in that set, a 7-joint arm
of about 1.3 m reach: `1.4 · 7 · 1.3 m · 1e-9 ≈ 1.3e-8 m`, about **13 nm**.

One consequence is worth stating plainly, because it is visible in
practice. Composing a description's `<origin rpy>` rotations leaves residue
in the axis components, and its size is set by **the scalar the parse runs
in**. In `float`, a quarter turn contributes about `4.4e-8` per rotation
(`cos` of the single-precision `pi/2`) and a half turn about `8.7e-8`
(`sin` of the single-precision `pi`); composing rotations accumulates them,
and the worst deviation measured across the fixture set is about `1.75e-7`.
All of those are far above this tolerance, so a joint whose origin composes
such rotations is classified `general` and takes the generic evaluation
path rather than a specialization. Not every joint of a single-precision
parse is affected — whether one is depends on the rotations its own origin
composes. A `double` parse of the same file is unaffected; its worst
measured deviation is `4.1e-10`.

Because the residue is a property of the arithmetic's precision rather than
of the description, it is **removable, and cartan does not remove it
today**. `parse_origin` builds the rotation through `rotation_from_rpy`,
which evaluates the trigonometry in the parse scalar; evaluating it in
`double` and narrowing the result takes the same axis from `-1.19e-7` to
`-2.2e-16`, which snaps. That change is not made here because it moves
chains from the generic path onto the specialized one, which is a decision
about the robot model and not a cleanup. Treat the current behavior as the
behavior, not as a floor.

Losing the specialization is a performance cost and not a correctness one:
the generic path evaluates the true axis and is strictly the more faithful
of the two.

**If you need the specialized path for a description-derived model, load
the chain at double precision** — `load_urdf<double>` — **and keep it
there**, narrowing only the quantities you hand downstream, such as a
computed pose or Jacobian. There is no scalar conversion on a chain:
`kinematic_chain::to_dynamic()` preserves `Scalar`, and none of
`static_chain`, `screw_axis`, `se3`, `so3` or `joint_limits` offers a
`cast<>` or a converting constructor, so a `double` chain cannot be turned
into a `float` one short of rebuilding every axis, the home pose and every
limit by hand. Note also that no tolerance setting buys the specialization
back at `float` without also admitting misalignments a single-precision
parse cannot distinguish from real ones.

## kinematic_chain

Kinematic chain in Product of Exponentials form. The PoE formula computes
forward kinematics as:

```
T(q) = exp([S1]q1) * exp([S2]q2) * ... * exp([Sn]qn) * M
```

where `S_i` are space-frame screw axes and `M` is the home
(zero-configuration) end-effector pose.

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <typename Scalar = double, int N = dynamic>
class kinematic_chain;
```

### Template Parameters

| Parameter | Meaning |
|-----------|---------|
| `Scalar` | Floating-point type (`double` or `float`). |
| `N` | Number of joints. Positive integer for compile-time fixed size, or `cartan::dynamic` (default) for runtime size. |

### Constructor

<!-- cartan:unbuilt kind=declaration -->
```cpp
kinematic_chain(
    const se3<Scalar>& home,
    screw_storage axes,
    limits_storage limits);
```

- `home` — End-effector pose at zero configuration (the M matrix).
- `axes` — Space-frame screw axes `S1..Sn`. For fixed `N`:
  `std::array<screw_axis<Scalar>, N>`. For dynamic:
  `std::vector<screw_axis<Scalar>>`.
- `limits` — Joint position/velocity limits. Same storage pattern as
  `axes`.

Throws `std::invalid_argument` if `axes.size() != limits.size()`. Caches
`joint_kind` per joint so
FK / Jacobian can dispatch into compile-time specializations.

### Accessors

<!-- cartan:unbuilt kind=declaration -->
```cpp
const se3<Scalar>& home() const;
const screw_storage& axes() const;
const limits_storage& limits() const;
int num_joints() const;
const screw_axis<Scalar>& axis(int i) const;
joint_kind kind(int i) const;
const kind_storage& kinds() const;
```

### Conversion

<!-- cartan:unbuilt kind=declaration -->
```cpp
kinematic_chain<Scalar, dynamic> to_dynamic() const
    requires (N != dynamic);
```

Convert a fixed-size chain to a dynamic chain. Only available when `N` is
a fixed (non-dynamic) value.

## static_chain

Compile-time parameterized serial chain. Joint types and axes are encoded
as template parameters via joint tags; runtime link data (home pose,
screw axes, joint limits) is stored in fixed-size `std::array` containers
sized by the parameter pack.

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <typename Scalar, joint_tag... Joints>
class static_chain;
```

The joint count and joint types are visible to the compiler, enabling
specialized FK and Jacobian implementations that exploit per-joint axis
knowledge for measurable speed wins over the generic
`kinematic_chain` path.

### Preconditions

- `Scalar` must be a floating-point type. Violation triggers
  `static_assert("static_chain requires a floating-point Scalar type")`.
- `sizeof...(Joints) > 0` — an empty joint parameter pack is rejected at
  compile time. Instantiating `static_chain<double>` (no joint tags)
  triggers `static_assert("static_chain requires at least one joint")`.
  Use `kinematic_chain<Scalar, dynamic>` for chains whose joint count
  is not known until runtime, or `kinematic_chain<Scalar, N>` for
  fixed-`N` chains constructed via runtime data.

### Type Aliases

<!-- cartan:unbuilt kind=declaration -->
```cpp
using scalar_type = Scalar;
static constexpr int joints = sizeof...(Joints);
using limits_storage = std::array<joint_limits<Scalar>, sizeof...(Joints)>;
using axes_storage = std::array<screw_axis<Scalar>, sizeof...(Joints)>;
```

### Construction

<!-- cartan:unbuilt kind=declaration -->
```cpp
static cartan::expected<static_chain, chain_failure> make(
    const se3<Scalar>& home,
    axes_storage axes,
    limits_storage limits);
```

- `home` — End-effector pose at zero configuration.
- `axes` — Space-frame screw axes `S1..Sn`, fixed-size by the parameter
  pack.
- `limits` — Joint position/velocity limits, fixed-size by the parameter
  pack.

`make` is the only way to obtain a `static_chain`; the constructor is
private. Two failures are possible, and both are returned as values, so a
release build behaves exactly as a debug build does:

- `chain_failure::non_finite_input` — the home pose or any screw-axis
  component is NaN or infinite. The `screw_axis::revolute` and
  `screw_axis::prismatic` factories normalize without validating, so a
  nonfinite input reaches the axis silently; this is where it is caught.
- `chain_failure::tag_axis_contradiction` — a stored axis is not the
  principal axis its compile-time tag names. The comparison is exact, with
  no tolerance: the tag and the axis both come from the caller, so a near
  match is a contradiction rather than a rounding artifact. Either sign is
  accepted — only the axis *line* is constrained, so a joint whose
  description gives `axis="0 0 -1"` stays expressible under `revolute_z`,
  and the specialization recovers the sign from the axis itself.

The check matters because the tag-dispatched fast path reads the component
its tag names as the signed magnitude. A screw about `y` stored under
`revolute_z` would read a zero there, and the joint would contribute
nothing at any joint value — a plausible pose for a robot with one
immovable joint.

#### Building a static chain from a parsed description

Exactness has a consequence worth planning for: **an axis that came out of a
URDF may not be exactly principal, and `make` will refuse it.** Composing a
description's `<origin rpy>` rotations leaves residue in the axis
components — the vendored UR descriptions yield `(0, 1, -2.05103e-10)` for a
joint that is nominally about `+y`, and single-precision parses are three
orders worse. `detect_joint_kind` snaps such an axis and `kinematic_chain`
takes the specialized path for it; `static_chain::make` returns
`tag_axis_contradiction` for the same value.

The two are meant to disagree. A tag is a claim the caller makes and can
therefore be held to exactly; a parsed axis is measured data, where a
tolerance is the only workable rule. If you want a `static_chain` from a
description, round the axis onto its principal direction yourself — which
makes the approximation explicit and yours — or use
`kinematic_chain`, which classifies at construction and needs no tag.

### Accessors

<!-- cartan:unbuilt kind=declaration -->
```cpp
const se3<Scalar>& home() const;
int num_joints() const;
const screw_axis<Scalar>& axis(int i) const;
const axes_storage& axes() const;
const limits_storage& limits() const;
```

## chain concept

<!-- cartan:unbuilt kind=declaration -->
```cpp
template <typename C>
concept chain = requires(const C& c, int i)
{
    typename C::scalar_type;
    { C::joints } -> std::convertible_to<int>;
    { c.home() } -> std::convertible_to<const se3<typename C::scalar_type>&>;
    { c.num_joints() } -> std::convertible_to<int>;
    { c.axis(i) } -> std::convertible_to<screw_axis<typename C::scalar_type>>;
    { c.axes() };
    { c.limits() };
};
```

Concept consumed by FK, Jacobian, and IK code. Captures the minimal surface
needed: scalar type, compile-time joint count, home configuration, runtime
joint count, per-element axis access, and bulk axes / limits access. Both
`kinematic_chain` and `static_chain` satisfy the concept; downstream
templates accept any conforming chain type without binding to one of the
two in-tree implementations.

## storage_trait

Compile-time selector between `std::array` (fixed `N`) and `std::vector`
(dynamic) storage.

The `dynamic` sentinel (`= -1`) it keys on is declared in `<cartan/types.h>`.

<!-- cartan:unbuilt kind=declaration -->
```cpp
namespace detail {

template <int N, typename T>
using storage_t = /* std::array<T, N> when N >= 0, std::vector<T> when N == dynamic */;

}
```

`kinematic_chain` and `joint_state` both use `storage_t` internally so the
same algorithms work with both fixed and dynamic sizing.
