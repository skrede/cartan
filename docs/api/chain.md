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
| `cartan::dynamic`, `cartan::detail::storage_t` | `#include <cartan/serial/chain/storage_trait.h>` |

## screw_axis

Screw axis for a kinematic joint in PoE form. Revolute joints have a unit
rotation axis (`||omega|| = 1`); prismatic joints have `omega = 0` and unit
translation direction (`||v|| = 1`).

```cpp
template <typename Scalar = double>
class screw_axis;
```

### Static Factory Methods

```cpp
static screw_axis revolute(const vector3<Scalar>& axis, const vector3<Scalar>& point);
```

Construct a revolute joint screw axis. `axis` is the rotation axis direction
(will be normalized). `point` is a point on the rotation axis. The linear
component is computed as `v = -omega x point`.

```cpp
static screw_axis prismatic(const vector3<Scalar>& direction);
```

Construct a prismatic joint screw axis. `direction` is the translation
direction (will be normalized). Sets `omega = 0`.

```cpp
static cartan::expected<screw_axis, lie_failure> from_vector(const vector6<Scalar>& vec);
```

Construct from a 6-vector `(omega, v)` with unit constraint validation. For
revolute axes, requires `||omega|| = 1`. For prismatic axes (`omega ~ 0`),
requires `||v|| = 1`. Returns `cartan::unexpected(lie_failure::non_unit_screw_axis)`
on validation failure (see [Error Handling](lie.md#error-handling)).

### Member Methods

```cpp
const vector3<Scalar>& omega() const;
const vector3<Scalar>& v() const;
```

Angular and linear velocity components.

```cpp
vector6<Scalar> to_vector() const;
```

Export as 6-vector `(omega, v)` in omega-first convention.

```cpp
bool is_revolute() const;
bool is_prismatic() const;
```

Joint type queries. Revolute when `||omega||^2 > epsilon`; prismatic
otherwise.

## joint_limits

Joint limits with required position bounds and optional dynamic limits.
Aggregate-initializable.

```cpp
template <typename Scalar = double>
struct joint_limits
{
    Scalar position_min;                       // Required
    Scalar position_max;                       // Required
    std::optional<Scalar> velocity_max{};      // Optional
    std::optional<Scalar> effort_max{};        // Optional
    std::optional<Scalar> acceleration_max{};  // Optional
};
```

Construction examples:

```cpp
joint_limits<double>{-3.14, 3.14}                    // Position only
joint_limits<double>{-3.14, 3.14, 2.0, 50.0, 10.0}  // All limits
```

### Methods

```cpp
bool contains(Scalar position) const;
```

Check whether a position value lies within `[position_min, position_max]`.

## joint_state

Joint state holding a position vector and an optional velocity vector.
Parameterized by scalar type and joint count `N` (fixed or
`cartan::dynamic`).

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

```cpp
static joint_state from_position(const position_type& q);
```

Create a joint state from position only (no velocity).

```cpp
int num_joints() const;
```

Number of joints in this state.

## Joint Tags

Six compile-time tag types describing the principal revolute and prismatic
joints (axis aligned with `+e_x`, `+e_y`, or `+e_z`). Used as template
parameter packs in `static_chain` to encode joint types at compile time and
enable `if constexpr` dispatch on joint type in FK / Jacobian.

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

```cpp
template <typename Scalar>
joint_kind detect_joint_kind(const screw_axis<Scalar>& axis);
```

Inspect a `screw_axis` and return its `joint_kind`. Recognizes axes whose
`omega` (revolute) or `v` (prismatic) is exactly `±e_x`, `±e_y`, or `±e_z`
within `sqrt(epsilon)`. The sign is irrelevant: downstream specializations
read the magnitude from the axis itself. All other axes return
`joint_kind::general`.

## kinematic_chain

Kinematic chain in Product of Exponentials form. The PoE formula computes
forward kinematics as:

```
T(q) = exp([S1]q1) * exp([S2]q2) * ... * exp([Sn]qn) * M
```

where `S_i` are space-frame screw axes and `M` is the home
(zero-configuration) end-effector pose.

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

Asserts `axes.size() == limits.size()`. Caches `joint_kind` per joint so
FK / Jacobian can dispatch into compile-time specializations.

### Accessors

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

```cpp
using scalar_type = Scalar;
static constexpr int joints = sizeof...(Joints);
using limits_storage = std::array<joint_limits<Scalar>, sizeof...(Joints)>;
using axes_storage = std::array<screw_axis<Scalar>, sizeof...(Joints)>;
```

### Constructor

```cpp
static_chain(
    const se3<Scalar>& home,
    axes_storage axes,
    limits_storage limits);
```

- `home` — End-effector pose at zero configuration.
- `axes` — Space-frame screw axes `S1..Sn`, fixed-size by the parameter
  pack.
- `limits` — Joint position/velocity limits, fixed-size by the parameter
  pack.

### Accessors

```cpp
const se3<Scalar>& home() const;
int num_joints() const;
const screw_axis<Scalar>& axis(int i) const;
const axes_storage& axes() const;
const limits_storage& limits() const;
```

## chain concept

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

```cpp
inline constexpr int dynamic = -1;

namespace detail {

template <int N, typename T>
using storage_t = /* std::array<T, N> when N >= 0, std::vector<T> when N == dynamic */;

}
```

`kinematic_chain` and `joint_state` both use `storage_t` internally so the
same algorithms work with both fixed and dynamic sizing.
