# lie

Part of the `cartan::lie` module (`lib/cartan-lie/`). Link with `cartan::lie` (or the convenience target `cartan::cartan`). The umbrella header `<cartan/lie.h>` includes all Lie group types.

Lie group classes for SO(2), SE(2), SO(3), SE(3) with exponential/logarithmic maps, composition, adjoints, and supporting utilities. All classes are parameterized by `Scalar` type and `Policy` (normalization behavior).

See [SO(2) Theory](../background/so2.md) | [SE(2) Theory](../background/se2.md) | [SO(3) Theory](../background/so3.md) | [SE(3) Theory](../background/se3.md)

## Headers

| Form | Header |
|------|--------|
| `cartan::so2` | `#include <cartan/lie/so2.h>` |
| `cartan::se2` | `#include <cartan/lie/se2.h>` |
| `cartan::so3` | `#include <cartan/lie/so3.h>` |
| `cartan::se3` | `#include <cartan/lie/se3.h>` |
| `cartan::hat`, `cartan::vee` | `#include <cartan/lie/hat_vee.h>` |
| `cartan::axis_angle`, `cartan::screw_params` | `#include <cartan/lie/axis_angle.h>` |
| `cartan::quat_slerp`, `cartan::from_wxyz`, ... | `#include <cartan/lie/quaternion_utils.h>` |
| `cartan::twist`, `cartan::screw_motion` | `#include <cartan/lie/twist.h>` |
| `cartan::se3_left_jacobian`, `cartan::se3_left_jacobian_inv`, `cartan::se3_Q_matrix` | `#include <cartan/lie/se3_left_jacobian.h>` |
| `cartan::strict_policy`, `cartan::fast_policy`, `cartan::trusted_unit_t`, `cartan::trusted_unit`, `cartan::lie_group_policy` concept | `#include <cartan/lie/policy.h>` |
| Forward declarations | `#include <cartan/lie/fwd.h>` |
| Foundation types | `#include <cartan/types.h>` |
| Epsilon constants | `#include <cartan/detail/epsilon.h>` |
| `cartan::expected`, `cartan::unexpected` | `#include <cartan/expected.h>` |
| `cartan::lie_failure`, `cartan::message` | `#include <cartan/lie/lie_failure.h>` |

## Foundation Types

Defined in `types.h`. Thin aliases over Eigen types used throughout Cartan.

```cpp
template <typename Scalar, std::size_t N>
using vector = Eigen::Matrix<Scalar, static_cast<int>(N), 1>;

template <typename Scalar>
using vector2 = Eigen::Matrix<Scalar, 2, 1>;

template <typename Scalar>
using vector3 = Eigen::Matrix<Scalar, 3, 1>;

template <typename Scalar>
using vector6 = Eigen::Matrix<Scalar, 6, 1>;

template <typename Scalar>
using matrix2 = Eigen::Matrix<Scalar, 2, 2>;

template <typename Scalar>
using matrix3 = Eigen::Matrix<Scalar, 3, 3>;

template <typename Scalar>
using matrix4 = Eigen::Matrix<Scalar, 4, 4>;

template <typename Scalar>
using matrix6 = Eigen::Matrix<Scalar, 6, 6>;

template <typename Scalar>
using quaternion = Eigen::Quaternion<Scalar>;
```

### Epsilon Constants

Defined in `detail/epsilon.h`. Compile-time numerical thresholds for Taylor branch switching.

```cpp
namespace cartan::detail {

template <typename Scalar>
inline constexpr Scalar epsilon_v = std::numeric_limits<Scalar>::epsilon();

template <typename Scalar>
inline constexpr Scalar sqrt_epsilon_v;  // sqrt(epsilon), computed at compile time

}
```

`epsilon_v<double>` is approximately `2.22e-16`. `sqrt_epsilon_v<double>` is approximately `1.49e-8`. Used throughout Lie group operations to select Taylor expansion branches near singularities.

## Error Handling

Defined in `expected.h`. The validating factories (`from_matrix`, `from_quaternion`, `screw_axis::from_vector`) return their result in a `cartan::expected<T, E>`, a sum type holding either a value of `T` or an error of `E`.

```cpp
template <typename T, typename E>
class expected;   // holds either a T (success) or an E (failure)

template <typename E>
class unexpected;  // wraps an error value for expected's error state
```

`cartan::expected<T, E>` is a 1:1 API mirror of the C++23 standard `expected` type — the same observers (`has_value()`, `operator bool`, `operator*`, `value()`, `error()`, `value_or()`), the same monadic operations (`and_then`, `or_else`, `transform`, `transform_error`), and the same construction rules. Cartan ships its own polyfill so the library compiles under a **C++20** baseline: the standard `expected` requires a C++23 standard library, which is not yet universal across Cartan's deployment targets (embedded toolchains, older `libstdc++`). When the toolchain does provide a C++23 standard library, `cartan::expected` gains explicit conversions to and from the standard `expected`.

The Lie factories report failure through the `cartan::lie_failure` enum (see `lie_failure.h`), an allocation-free, matchable error code shared by `so2`/`so3`/`se2`/`se3` `from_matrix`, `so3::from_quaternion`, the frame-tagged `rotation`/`transform` wrappers, and `screw_axis::from_vector`. `cartan::message(lie_failure)` maps a code to a static human-readable diagnostic.

```cpp
enum class lie_failure
{
    non_orthogonal,       // R^T * R deviates from identity
    improper_rotation,    // det(R) != 1: a reflection, not a rotation
    non_unit_quaternion,  // ||q||^2 deviates from 1
    invalid_affine_row,   // homogeneous bottom row is not [0..0 1]
    non_unit_screw_axis   // revolute ||omega|| != 1 or prismatic ||v|| != 1
};

auto r = cartan::so3<double>::from_matrix(M);
if (!r)
    std::cerr << cartan::message(r.error()) << '\n';
else
    use(*r);
```

## Policy

Defined in `policy.h`. Controls normalization and assertion behavior on construction.

```cpp
struct strict_policy
{
    static constexpr bool normalize_on_construct = true;
    static constexpr bool assert_valid = true;
};

struct fast_policy
{
    static constexpr bool normalize_on_construct = false;
    static constexpr bool assert_valid = false;
};

template <typename P1, typename P2>
using stricter_policy = std::conditional_t<
    P1::normalize_on_construct || P2::normalize_on_construct,
    strict_policy, fast_policy>;

template <typename P>
concept lie_group_policy = requires {
    { P::normalize_on_construct } -> std::convertible_to<bool>;
    { P::assert_valid } -> std::convertible_to<bool>;
};
```

`strict_policy` (default) normalizes group elements on construction and asserts validity. Use for safety-critical paths. `fast_policy` skips all checks -- use on hot paths where inputs are known-valid. When composing elements with different policies, `stricter_policy` selects the stricter of the two.

### lie_group_policy concept

```cpp
template <typename P>
concept lie_group_policy = requires {
    { P::normalize_on_construct } -> std::convertible_to<bool>;
    { P::assert_valid } -> std::convertible_to<bool>;
};
```

Concept constraining valid Lie group policy types. A conforming type
exposes `normalize_on_construct` and `assert_valid` as `static constexpr
bool` members. The frames module (`rotation<From, To, Scalar, Policy>`,
`transform<From, To, Scalar, Policy>`) constrains its `Policy` template
parameter against this concept; `strict_policy` and `fast_policy`
both satisfy it.

### trusted_unit_t and trusted_unit

```cpp
struct trusted_unit_t {};
inline constexpr trusted_unit_t trusted_unit{};
```

Tag type for the unchecked "trusted unit" construction path on `so3` and
`se3`. Use when the caller can prove the input quaternion is already
unit-length (e.g. exact axis-angle output from `exp`, conjugate of a
unit, identity construction, or an FK accumulator's per-step output).
Construction with the tag skips the normalize step unconditionally;
debug builds still validate via `assert()`. The canonical hot-path use
sites are inside FK accumulators where `so3` / `se3` products of unit
quaternions accumulate `O(N * eps)` deviation that is cheaper to absorb
into a single final renormalize than to spend at every per-step
construction.

Pass as the second argument to `so3(quaternion, trusted_unit_t)`,
`se3(se3<P2>, trusted_unit_t)`, or any other constructor / method that
documents a `trusted_unit_t` overload.

## se3_left_jacobian

Defined in `se3_left_jacobian.h`. SE(3) left Jacobian and its inverse,
plus the `Q` matrix used in the SE(3) left-Jacobian decomposition. Uses
cartan's omega-first twist convention throughout.

```cpp
template <typename Scalar>
matrix3<Scalar> se3_Q_matrix(const vector3<Scalar>& omega, const vector3<Scalar>& rho);
```

`Q` matrix appearing in the bottom-left 3x3 block of the SE(3) left
Jacobian decomposition. Adapted from Barfoot to omega-first ordering.

```cpp
template <typename Scalar>
matrix6<Scalar> se3_left_jacobian(const vector6<Scalar>& xi);
```

SE(3) left Jacobian for the twist `xi = (omega, rho)` (omega-first).
Block structure: top-left `J_so3(omega)`, top-right zero, bottom-left
`Q(omega, rho)`, bottom-right `J_so3(omega)`.

```cpp
template <typename Scalar>
matrix6<Scalar> se3_left_jacobian_inv(const vector6<Scalar>& xi);
```

Inverse SE(3) left Jacobian. Block structure: top-left
`J_so3^{-1}(omega)`, top-right zero, bottom-left `-J^{-1} Q J^{-1}`,
bottom-right `J_so3^{-1}(omega)`.

Reference: Barfoot, *State Estimation for Robotics*, Eq. 8.91 (left
           Jacobian), Eq. 8.100b (inverse), adapted to omega-first
           convention.

## so2

2D rotation group. Internal representation: `(cos, sin)` pair.

```cpp
template <typename Scalar, typename Policy = strict_policy>
class so2;
```

### Static Methods

```cpp
static so2 exp(Scalar theta);
```

Exponential map: angle (radians) to SO(2) rotation.

```cpp
static so2 identity();
```

Identity element (zero rotation).

```cpp
static cartan::expected<so2, lie_failure> from_matrix(const matrix2<Scalar>& R);
```

Construct from 2x2 rotation matrix with validation. Returns error if `R^T * R` deviates from identity or `det(R) != 1`.

### Member Methods

```cpp
Scalar log() const;
```

Logarithmic map: rotation to angle in `(-pi, pi]`.

```cpp
so2 inverse() const;
```

Group inverse (`R^{-1} = R^T`).

```cpp
template <typename P2>
auto operator*(const so2<Scalar, P2>& rhs) const -> so2<Scalar, stricter_policy<Policy, P2>>;
```

Group composition via angle-addition formulas. Result policy is the stricter of the two operands.

```cpp
matrix2<Scalar> matrix() const;
```

Convert to 2x2 rotation matrix `[[cos, -sin], [sin, cos]]`.

```cpp
Scalar angle() const;
```

Angle accessor (same as `log()`).

```cpp
Scalar cos_angle() const;
Scalar sin_angle() const;
```

Direct access to cosine and sine components.

```cpp
vector2<Scalar> act(const vector2<Scalar>& v) const;
```

Rotate a 2D vector: `R * v`.

## se2

2D rigid body transformation group. Internal representation: `so2` rotation + `vector2` translation.

```cpp
template <typename Scalar, typename Policy = strict_policy>
class se2;
```

### Static Methods

```cpp
static se2 exp(const vector3<Scalar>& v);
```

Exponential map: `se(2)` twist `(omega, vx, vy)` to SE(2) transform. Omega-first convention. Handles `omega ~ 0` via Taylor expansion to avoid division by zero.

```cpp
static se2 identity();
```

Identity element (no rotation, no translation).

```cpp
static cartan::expected<se2, lie_failure> from_matrix(const Eigen::Matrix<Scalar, 3, 3>& T);
```

Construct from 3x3 homogeneous matrix with validation. Checks rotation block and bottom row `[0, 0, 1]`.

### Member Methods

```cpp
vector3<Scalar> log() const;
```

Logarithmic map: SE(2) to twist `(omega, vx, vy)`. Handles `omega ~ 0` via Taylor expansion.

```cpp
se2 inverse() const;
```

Group inverse: `T^{-1} = (R^{-1}, -R^{-1} * t)`.

```cpp
template <typename P2>
auto operator*(const se2<Scalar, P2>& rhs) const -> se2<Scalar, stricter_policy<Policy, P2>>;
```

Group composition.

```cpp
Eigen::Matrix<Scalar, 3, 3> matrix() const;
```

Convert to 3x3 homogeneous transformation matrix.

```cpp
Eigen::Matrix<Scalar, 3, 3> adjoint() const;
```

3x3 adjoint representation acting on `se(2)` twists.

```cpp
const so2<Scalar, Policy>& rotation() const;
const vector2<Scalar>& translation() const;
```

Access rotation and translation components.

```cpp
vector2<Scalar> act(const vector2<Scalar>& p) const;
```

Transform a 2D point: `R * p + t`.

## so3

3D rotation group. Internal representation: unit quaternion (`Eigen::Quaternion<Scalar>`).

```cpp
template <typename Scalar, typename Policy = strict_policy>
class so3;
```

### Constructors

```cpp
explicit so3(const quaternion<Scalar>& q);
```

Construct from a quaternion. Under `strict_policy` the input is normalized
to unit length on construction; under `fast_policy` the input is taken
as-is.

```cpp
so3(const quaternion<Scalar>& q, trusted_unit_t);
```

Construct from a known-unit quaternion, bypassing normalization. Caller
must guarantee `||q|| ~= 1`; debug builds validate via `assert`. See
[`trusted_unit_t`](#trusted_unit_t-and-trusted_unit) for the canonical
hot-path use cases (FK accumulator boundary, conjugate of a unit,
axis-aligned `exp`).

### Static Methods

```cpp
static so3 exp(const vector3<Scalar>& phi);
```

Exponential map: axis-angle vector `phi` (axis * angle) to SO(3) via quaternion form. Uses Taylor expansion near `||phi|| ~ 0` for numerical stability.

```cpp
static so3 identity();
```

Identity element (unit quaternion `w=1`).

```cpp
static cartan::expected<so3, lie_failure> from_matrix(const matrix3<Scalar>& R);
```

Construct from 3x3 rotation matrix with validation.

```cpp
static cartan::expected<so3, lie_failure> from_quaternion(const quaternion<Scalar>& q);
```

Construct from quaternion with unit-norm validation.

```cpp
static matrix3<Scalar> left_jacobian(const vector3<Scalar>& phi);
```

SO(3) left Jacobian `J_l(phi)`. Used in SE(3) exponential map.

```cpp
static matrix3<Scalar> right_jacobian(const vector3<Scalar>& phi);
```

SO(3) right Jacobian: `J_r(phi) = J_l(-phi)`.

```cpp
static matrix3<Scalar> left_jacobian_inv(const vector3<Scalar>& phi);
```

Inverse left Jacobian `J_l^{-1}(phi)`. Used in SE(3) logarithmic map.

```cpp
static matrix3<Scalar> right_jacobian_inv(const vector3<Scalar>& phi);
```

Inverse right Jacobian: `J_r^{-1}(phi) = J_l^{-1}(-phi)`.

### Member Methods

```cpp
vector3<Scalar> log() const;
```

Logarithmic map: SO(3) to axis-angle vector via quaternion `atan2` approach. Avoids the `theta ~ pi` eigenvector branch entirely; only `theta ~ 0` needs Taylor.

```cpp
so3 inverse() const;
```

Group inverse via quaternion conjugate.

```cpp
template <typename P2>
auto operator*(const so3<Scalar, P2>& rhs) const -> so3<Scalar, stricter_policy<Policy, P2>>;
```

Group composition via Hamilton quaternion product.

```cpp
matrix3<Scalar> adjoint() const;
```

Adjoint representation: `Ad_R = R` (the rotation matrix itself for SO(3)).

```cpp
matrix3<Scalar> coadjoint() const;
```

Coadjoint representation: `Ad_R^{-T} = R` for SO(3) (since R is orthogonal).

```cpp
matrix3<Scalar> matrix() const;
```

Convert to 3x3 rotation matrix.

```cpp
const quaternion<Scalar>& quaternion_ref() const;
```

Access the internal quaternion (read-only).

```cpp
vector3<Scalar> act(const vector3<Scalar>& v) const;
```

Rotate a 3D vector: `R * v`.

### Edge Cases

- **Near-zero angle (`||phi|| < epsilon`):** `exp` and `log` switch to Taylor expansions automatically.
- **Near-pi angle:** The quaternion `atan2`-based `log` avoids the eigenvector branch at `theta ~ pi` entirely, producing correct results without special-casing.
- **Double cover:** `log` canonicalizes to the `w >= 0` quaternion hemisphere for unique output.

## se3

3D rigid body transformation group. Internal representation: `so3` rotation + `vector3` translation. 7 scalars total (4 quaternion + 3 translation).

```cpp
template <typename Scalar, typename Policy = strict_policy>
class se3;
```

### Constructors

```cpp
se3(const so3<Scalar, Policy>& rot, const vector3<Scalar>& trans);
```

Construct from rotation and translation components.

```cpp
template <typename P2>
se3(const se3<Scalar, P2>& other, trusted_unit_t);
```

Construct from an `se3` with a different `Policy` parameter, transferring
the rotation as a known-unit quaternion (skipping renormalize). Caller
must guarantee the source rotation is unit; debug builds validate. See
[`trusted_unit_t`](#trusted_unit_t-and-trusted_unit) — the typical use
is rebinding the policy on the boundary of an FK accumulator.

### Static Methods

```cpp
static se3 exp(const vector6<Scalar>& v);
```

Exponential map: `se(3)` twist `(omega, rho)` to SE(3) transform. Omega-first convention. Uses `so3::left_jacobian` for the translation component: `t = J_l(omega) * rho`.

```cpp
static se3 identity();
```

Identity element.

```cpp
static cartan::expected<se3, lie_failure> from_matrix(const matrix4<Scalar>& T);
```

Construct from 4x4 homogeneous matrix with validation. Checks rotation block and bottom row `[0, 0, 0, 1]`.

### Member Methods

```cpp
vector6<Scalar> log() const;
```

Logarithmic map: SE(3) to twist `(omega, rho)`. Uses `so3::left_jacobian_inv` for the linear component: `rho = J_l^{-1}(omega) * t`.

```cpp
se3 inverse() const;
```

Group inverse: `T^{-1} = (R^{-1}, -R^{-1} * t)`.

```cpp
template <typename P2>
auto operator*(const se3<Scalar, P2>& rhs) const -> se3<Scalar, stricter_policy<Policy, P2>>;
```

Group composition.

```cpp
template <typename P2>
se3<Scalar, fast_policy> compose_trusted(const se3<Scalar, P2>& rhs) const;
```

Group composition without renormalizing the result rotation. Returns
`fast_policy` `se3`; caller takes responsibility for accumulated drift.
Designed for FK chain accumulators where `N` successive unit-quaternion
products keep `||q||^2 - 1` bounded by `O(N * eps)`, making per-step
renormalization wasted work.

```cpp
matrix6<Scalar> adjoint() const;
```

6x6 adjoint representation. Omega-first twist ordering:
```
[Ad_T] = [ R       0  ]
         [ [p]R    R  ]
```
where `[p]` is the skew-symmetric matrix of the translation.

```cpp
matrix6<Scalar> coadjoint() const;
```

Coadjoint representation: `Ad_T^{-T}`.

```cpp
matrix4<Scalar> matrix() const;
```

Convert to 4x4 homogeneous transformation matrix.

```cpp
const so3<Scalar, Policy>& rotation() const;
const vector3<Scalar>& translation() const;
```

Access rotation and translation components.

```cpp
vector3<Scalar> act(const vector3<Scalar>& p) const;
```

Transform a 3D point: `R * p + t`.

## hat and vee

Defined in `hat_vee.h`. Isomorphisms between vectors and Lie algebra matrices.

### hat (3-vector)

```cpp
template <typename Scalar>
matrix3<Scalar> hat(const vector3<Scalar>& v);
```

Constructs a 3x3 skew-symmetric matrix from a 3-vector. Property: `hat(v) * w = v x w` (cross product).

### vee (3x3)

```cpp
template <typename Scalar>
vector3<Scalar> vee(const matrix3<Scalar>& S);
```

Extracts a 3-vector from a 3x3 skew-symmetric matrix. Inverse of `hat`: `vee(hat(v)) == v`.

### hat (6-vector)

```cpp
template <typename Scalar>
matrix4<Scalar> hat(const vector6<Scalar>& V);
```

Constructs a 4x4 `se(3)` twist matrix from a 6-vector `(omega, v)`. Layout: top-left 3x3 = `hat(omega)`, top-right 3x1 = `v`, bottom row = zeros.

### vee (4x4)

```cpp
template <typename Scalar>
vector6<Scalar> vee(const matrix4<Scalar>& M);
```

Extracts a 6-vector from a 4x4 `se(3)` twist matrix. Inverse of `hat`: `vee(hat(V)) == V`.

## axis_angle

Defined in `axis_angle.h`. Axis-angle representation and screw parameter extraction.

### Structs

```cpp
template <typename Scalar>
struct axis_angle {
    vector3<Scalar> axis;   // Unit rotation axis
    Scalar angle;           // Rotation angle in radians [0, pi]
};

template <typename Scalar>
struct screw_params {
    vector3<Scalar> q;       // Point on the screw axis
    vector3<Scalar> s_hat;   // Unit direction of screw axis
    Scalar h;                // Pitch (0 = pure rotation, infinity = pure translation)
};
```

### Functions

```cpp
template <typename Scalar, typename Policy>
axis_angle<Scalar> to_axis_angle(const so3<Scalar, Policy>& r);
```

Extract axis-angle from SO(3) via the log map. For `theta ~ 0`, returns zero angle with `UnitX` as arbitrary axis.

```cpp
template <typename Scalar>
so3<Scalar> from_axis_angle(const axis_angle<Scalar>& aa);
```

Convert axis-angle to SO(3) via the exp map.

```cpp
template <typename Scalar>
axis_angle<Scalar> from_angle_axis_vector(const vector3<Scalar>& phi);
```

Parse an angle-axis vector `phi = theta * axis` into axis and angle components.

```cpp
template <typename Scalar>
screw_params<Scalar> to_screw_params(const vector3<Scalar>& omega, const vector3<Scalar>& v);
```

Extract screw parameters from twist components `(omega, v)`. For any
non-zero `omega`, let `omega_hat = omega / ||omega||` (the unit rotation
axis). Then `s_hat = omega_hat`, `q = (omega_hat x v) / ||omega||`,
`h = (omega_hat . v) / ||omega||`.
For pure translation (`||omega|| ~ 0`): `s_hat = v / ||v||`, `q = 0`,
`h = infinity`.

## quaternion_utils

Defined in `quaternion_utils.h`. Utility functions supplementing Eigen's quaternion type.

```cpp
template <typename Scalar>
quaternion<Scalar> quat_slerp(const quaternion<Scalar>& q1, const quaternion<Scalar>& q2, Scalar t);
```

Spherical linear interpolation between two unit quaternions. `t` in `[0, 1]`.

```cpp
template <typename Scalar>
quaternion<Scalar> quat_normalize(const quaternion<Scalar>& q);
```

Normalize a quaternion to unit length.

```cpp
template <typename Scalar>
matrix3<Scalar> quat_to_matrix(const quaternion<Scalar>& q);
```

Convert unit quaternion to 3x3 rotation matrix.

```cpp
template <typename Scalar>
quaternion<Scalar> matrix_to_quat(const matrix3<Scalar>& R);
```

Convert 3x3 rotation matrix to quaternion.

```cpp
template <typename Scalar>
quaternion<Scalar> from_wxyz(Scalar w, Scalar x, Scalar y, Scalar z);
```

Named constructor: w-first order `(w, x, y, z)`.

```cpp
template <typename Scalar>
quaternion<Scalar> from_xyzw(Scalar x, Scalar y, Scalar z, Scalar w);
```

Named constructor: xyzw order (Eigen internal storage order).

```cpp
template <typename Scalar>
vector<Scalar, 4> to_wxyz(const quaternion<Scalar>& q);
```

Serialize quaternion to `[w, x, y, z]` 4-vector.

```cpp
template <typename Scalar>
quaternion<Scalar> quat_hamilton_product(const quaternion<Scalar>& q1, const quaternion<Scalar>& q2);
```

Hamilton quaternion product: `q1 * q2`.

## twist

Defined in `twist.h`. Twist (spatial velocity) representation and screw motion decomposition.

### Structs

```cpp
template <typename Scalar>
struct twist {
    vector3<Scalar> omega;  // Angular velocity (omega-first)
    vector3<Scalar> v;      // Linear velocity

    static twist from_vector(const vector6<Scalar>& vec);
    vector6<Scalar> to_vector() const;
};

template <typename Scalar>
struct screw_motion {
    screw_params<Scalar> axis;  // Screw parameters
    Scalar theta;               // Rotation angle (radians)
    Scalar d;                   // Translation distance along axis
};
```

### Functions

```cpp
template <typename Scalar>
se3<Scalar> twist_to_se3(const twist<Scalar>& tw, Scalar theta);
```

Compute the rigid body motion from a unit twist applied for angle/distance `theta`. Returns `se3::exp(theta * twist_vector)`.

```cpp
template <typename Scalar, typename Policy>
twist<Scalar> se3_to_twist(const se3<Scalar, Policy>& T);
```

Extract twist from SE(3) via the log map.

```cpp
template <typename Scalar>
screw_motion<Scalar> to_screw_motion(const twist<Scalar>& tw);
```

Decompose a twist into screw motion parameters. For rotation (`||omega|| > epsilon`): `theta = ||omega||`, `d = h * theta`. For pure translation: `theta = 0`, `d = ||v||`.

```cpp
template <typename Scalar>
twist<Scalar> from_screw_motion(const screw_motion<Scalar>& sm);
```

Reconstruct a twist from screw motion parameters.
