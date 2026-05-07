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
| `cartan::strict_policy`, `cartan::fast_policy` | `#include <cartan/lie/policy.h>` |
| Forward declarations | `#include <cartan/lie/fwd.h>` |
| Foundation types | `#include <cartan/types.h>` |
| Epsilon constants | `#include <cartan/detail/epsilon.h>` |

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
static std::expected<so2, std::string> from_matrix(const matrix2<Scalar>& R);
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
static std::expected<se2, std::string> from_matrix(const Eigen::Matrix<Scalar, 3, 3>& T);
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
static std::expected<so3, std::string> from_matrix(const matrix3<Scalar>& R);
```

Construct from 3x3 rotation matrix with validation.

```cpp
static std::expected<so3, std::string> from_quaternion(const quaternion<Scalar>& q);
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
static std::expected<se3, std::string> from_matrix(const matrix4<Scalar>& T);
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

Extract screw parameters from twist components. For rotation (`||omega|| ~ 1`): `s_hat = omega`, `q = omega x v`, `h = omega . v`. For pure translation (`||omega|| ~ 0`): `s_hat = v/||v||`, `h = infinity`.

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
