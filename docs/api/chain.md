# chain

Part of the `liepp::kinematics` module (`lib/liepp-kinematics/`). Link with `liepp::kinematics` (or the convenience target `liepp::liepp`).

Kinematic chain representation using Product of Exponentials (PoE) formulation. Defines screw axes, joint limits, joint state, and the `kinematic_chain` container. Supports both compile-time fixed and runtime dynamic joint counts.

See [PoE Kinematics](../background/poe-kinematics.md)

## Headers

| Form | Header |
|------|--------|
| All chain types | `#include <liepp/chain/chain.h>` |
| `liepp::kinematic_chain` | `#include <liepp/chain/kinematic_chain.h>` |
| `liepp::screw_axis` | `#include <liepp/chain/screw_axis.h>` |
| `liepp::joint_limits` | `#include <liepp/chain/joint_limits.h>` |
| `liepp::joint_state` | `#include <liepp/chain/joint_state.h>` |
| `liepp::dynamic`, `liepp::detail::storage_t` | `#include <liepp/chain/storage_trait.h>` |

## screw_axis

Screw axis for a kinematic joint in PoE form. Revolute joints have a unit rotation axis (`||omega|| = 1`); prismatic joints have `omega = 0` and unit translation direction (`||v|| = 1`).

```cpp
template <typename Scalar = double>
class screw_axis;
```

### Static Factory Methods

```cpp
static screw_axis revolute(const vector3<Scalar>& axis, const vector3<Scalar>& point);
```

Construct a revolute joint screw axis. `axis` is the rotation axis direction (will be normalized). `point` is a point on the rotation axis. The linear component is computed as `v = -omega x point`.

```cpp
static screw_axis prismatic(const vector3<Scalar>& direction);
```

Construct a prismatic joint screw axis. `direction` is the translation direction (will be normalized). Sets `omega = 0`.

```cpp
static std::expected<screw_axis, std::string> from_vector(const vector6<Scalar>& vec);
```

Construct from a 6-vector `(omega, v)` with unit constraint validation. For revolute axes, requires `||omega|| = 1`. For prismatic axes (`omega ~ 0`), requires `||v|| = 1`. Returns error string on validation failure.

### Member Methods

```cpp
const vector3<Scalar>& omega() const;
```

Angular velocity component (rotation axis for revolute, zero for prismatic).

```cpp
const vector3<Scalar>& v() const;
```

Linear velocity component.

```cpp
vector6<Scalar> to_vector() const;
```

Export as 6-vector `(omega, v)` in omega-first convention.

```cpp
bool is_revolute() const;
bool is_prismatic() const;
```

Joint type queries. Revolute when `||omega||^2 > epsilon`; prismatic otherwise.

## joint_limits

Joint limits with required position bounds and optional dynamic limits. Aggregate-initializable.

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

Joint state holding position vector and optional velocity vector. Parameterized by scalar type and joint count `N` (fixed or `liepp::dynamic`).

```cpp
template <typename Scalar = double, int N = dynamic>
struct joint_state
{
    position_type position;                   // Joint positions
    std::optional<velocity_type> velocity{};  // Joint velocities (optional)
};
```

For fixed `N`, `position_type` is `Eigen::Vector<Scalar, N>`. For dynamic, it is `Eigen::VectorX<Scalar>`.

### Methods

```cpp
static joint_state from_position(const position_type& q);
```

Create a joint state from position only (no velocity).

```cpp
int num_joints() const;
```

Number of joints in this state.

## kinematic_chain

Kinematic chain in Product of Exponentials form. The PoE formula computes forward kinematics as:

```
T(q) = exp([S1]q1) * exp([S2]q2) * ... * exp([Sn]qn) * M
```

where `S_i` are space-frame screw axes and `M` is the home (zero-configuration) end-effector pose.

```cpp
template <typename Scalar = double, int N = dynamic>
class kinematic_chain;
```

### Template Parameters

| Parameter | Meaning |
|-----------|---------|
| `Scalar` | Floating-point type (`double` or `float`). |
| `N` | Number of joints. Use a positive integer for compile-time fixed size, or `liepp::dynamic` (default) for runtime size. |

### Constructor

```cpp
kinematic_chain(
    const se3<Scalar>& home,
    screw_storage axes,
    limits_storage limits);
```

- `home` -- End-effector pose at zero configuration (the M matrix).
- `axes` -- Space-frame screw axes `S1..Sn`. For fixed `N`: `std::array<screw_axis<Scalar>, N>`. For dynamic: `std::vector<screw_axis<Scalar>>`.
- `limits` -- Joint position/velocity limits. Same storage pattern as axes.

Asserts `axes.size() == limits.size()`.

### Accessors

```cpp
const se3<Scalar>& home() const;
```

Home configuration (M matrix).

```cpp
const screw_storage& axes() const;
```

Space-frame screw axes.

```cpp
const limits_storage& limits() const;
```

Joint limits.

```cpp
int num_joints() const;
```

Number of joints in the chain.

### Conversion

```cpp
kinematic_chain<Scalar, dynamic> to_dynamic() const
    requires (N != dynamic);
```

Convert a fixed-size chain to a dynamic chain. Only available when `N` is a fixed (non-dynamic) value.

## storage_trait

Compile-time selector between `std::array` (fixed `N`) and `std::vector` (dynamic) storage.

```cpp
inline constexpr int dynamic = -1;

namespace detail {

template <int N, typename T>
using storage_t = /* std::array<T, N> when N >= 0, std::vector<T> when N == dynamic */;

}
```

All chain containers (`kinematic_chain`, `joint_state`) use `storage_t` internally so that the same algorithms work with both fixed and dynamic sizing.
