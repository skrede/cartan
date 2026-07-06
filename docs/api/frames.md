# frames

Part of the `cartan::lie` module (`lib/cartan-lie/`). Link with `cartan::lie` (or the convenience target `cartan::cartan`).

Compile-time frame-tagged wrappers for rotations, transforms, twists, and wrenches. The `From` and `To` template parameters enforce correct frame composition at compile time. Zero runtime overhead: aggregate structs forwarding to their inner Lie group value.

See [Frame Tags Design](../background/frame-tags.md)

## Headers

| Form | Header |
|------|--------|
| All frame types | `#include <cartan/frames/frames.h>` |
| `cartan::rotation<From, To>` | `#include <cartan/frames/rotation.h>` |
| `cartan::transform<From, To>` | `#include <cartan/frames/transform.h>` |
| `cartan::framed_twist<Frame>` | `#include <cartan/frames/framed_twist.h>` |
| `cartan::framed_wrench<Frame>` | `#include <cartan/frames/framed_wrench.h>` |

## rotation

Compile-time frame-tagged wrapper over `so3`. Aggregate struct with public `m_value` member.

```cpp
template <typename From, typename To, typename Scalar = double, lie_group_policy Policy = strict_policy>
struct rotation
{
    so3<Scalar, Policy> m_value;
};
```

`From` and `To` are unconstrained frame tag types -- empty structs, enum classes, or int NTTP wrappers all work.

### Methods

```cpp
template <typename C, lie_group_policy P2>
auto operator*(const rotation<To, C, Scalar, P2>& rhs) const
    -> rotation<From, C, Scalar, stricter_policy<Policy, P2>>;
```

Compose two rotations. **`operator*` only compiles when the `To` frame of the left operand matches the `From` frame of the right operand.** `rotation<A,B> * rotation<B,C>` produces `rotation<A,C>`.

```cpp
rotation<To, From, Scalar, Policy> inverse() const;
```

Inverse flips frame tags: `rotation<A,B>.inverse()` produces `rotation<B,A>`.

```cpp
matrix3<Scalar> matrix() const;
```

Convert to 3x3 rotation matrix.

```cpp
const quaternion<Scalar>& quaternion_ref() const;
```

Access the internal quaternion (read-only).

```cpp
vector3<Scalar> log() const;
```

Logarithmic map: SO(3) to `so(3)`.

```cpp
vector3<Scalar> act(const vector3<Scalar>& v) const;
```

Rotate a 3D vector.

```cpp
static rotation identity();
```

Identity rotation.

```cpp
static cartan::expected<rotation, lie_failure> from_matrix(const matrix3<Scalar>& R);
static cartan::expected<rotation, lie_failure> from_quaternion(const quaternion<Scalar>& q);
```

Validated constructors from matrix or quaternion. On failure they return a `cartan::unexpected` carrying a `lie_failure` code (see [Error Handling](lie.md#error-handling)).

## transform

Compile-time frame-tagged wrapper over `se3`. Aggregate struct with public `m_value` member.

```cpp
template <typename From, typename To, typename Scalar = double, lie_group_policy Policy = strict_policy>
struct transform
{
    se3<Scalar, Policy> m_value;
};
```

### Methods

```cpp
template <typename C, lie_group_policy P2>
auto operator*(const transform<To, C, Scalar, P2>& rhs) const
    -> transform<From, C, Scalar, stricter_policy<Policy, P2>>;
```

Compose two transforms. **`operator*` only compiles when the `To` frame of the left operand matches the `From` frame of the right operand.** `transform<A,B> * transform<B,C>` produces `transform<A,C>`.

```cpp
transform<To, From, Scalar, Policy> inverse() const;
```

Inverse flips frame tags: `transform<A,B>.inverse()` produces `transform<B,A>`.

```cpp
matrix4<Scalar> matrix() const;
```

Convert to 4x4 homogeneous transformation matrix.

```cpp
const so3<Scalar, Policy>& rotation() const;
```

Access the rotation component.

```cpp
const vector3<Scalar>& translation() const;
```

Access the translation component.

```cpp
vector6<Scalar> log() const;
```

Logarithmic map: SE(3) to `se(3)`.

```cpp
vector3<Scalar> act(const vector3<Scalar>& p) const;
```

Transform a 3D point: `R * p + t`.

```cpp
static transform identity();
```

Identity transform.

```cpp
static cartan::expected<transform, lie_failure> from_matrix(const matrix4<Scalar>& T);
```

Validated constructor from 4x4 homogeneous matrix. On failure it returns a `cartan::unexpected` carrying a `lie_failure` code (see [Error Handling](lie.md#error-handling)).

## framed_twist

Compile-time frame-tagged wrapper over `twist`. The `Frame` tag indicates the frame in which the twist is expressed.

```cpp
template <typename Frame, typename Scalar = double>
struct framed_twist
{
    twist<Scalar> m_value;
};
```

### Methods

```cpp
const vector3<Scalar>& omega() const;
const vector3<Scalar>& v() const;
```

Access angular and linear velocity components.

```cpp
vector6<Scalar> to_vector() const;
```

Convert to 6-vector (omega-first).

```cpp
static framed_twist from_vector(const vector6<Scalar>& vec);
```

Construct from 6-vector.

### Free Function: adjoint_map

```cpp
template <typename From, typename To, typename Scalar, lie_group_policy Policy>
framed_twist<From, Scalar>
adjoint_map(const transform<From, To, Scalar, Policy>& T,
            const framed_twist<To, Scalar>& tw);
```

Adjoint map: transforms a twist from frame `To` to frame `From`. Computes `Ad_T * V`. Frame enforcement is structural: the twist `Frame` must match the transform's `To`.

## framed_wrench

Compile-time frame-tagged wrapper over a 6-vector wrench. Moment-first `[moment; force]` ordering consistent with omega-first twist convention.

```cpp
template <typename Frame, typename Scalar = double>
struct framed_wrench
{
    vector6<Scalar> m_value;
};
```

### Methods

```cpp
auto moment() const;  // First 3 elements
auto force() const;   // Last 3 elements
```

Access moment and force components.

```cpp
static framed_wrench from_moment_force(const vector3<Scalar>& m, const vector3<Scalar>& f);
```

Construct from separate moment and force vectors.

### Free Function: coadjoint_map

```cpp
template <typename From, typename To, typename Scalar, lie_group_policy Policy>
framed_wrench<From, Scalar>
coadjoint_map(const transform<From, To, Scalar, Policy>& T,
              const framed_wrench<To, Scalar>& w);
```

Coadjoint map: transforms a wrench from frame `To` to frame `From`. Computes `(Ad_{T^{-1}})^T * W`. Frame enforcement is structural.

## Edge Cases

- **Frame mismatch:** Attempting to compose `rotation<A,B> * rotation<C,D>` where `B != C` produces a compile error. This is the primary safety guarantee of the frames module.
- **Zero overhead:** All frame types are aggregate structs. The `m_value` member is public for direct access when frame safety is not needed. The compiler generates identical code to using raw `so3`/`se3`.
