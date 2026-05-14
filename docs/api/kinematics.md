# kinematics

Headers under `cartan/serial/fk/` (sublib `cartan-serial-chain`). Public
symbols live in the `cartan::` namespace. Link with `cartan::serial-chain`
(or the convenience target `cartan::cartan`).

Forward kinematics and Jacobian computation using the Product of Exponentials
formulation. All functions accept a chain (`kinematic_chain`, `static_chain`,
or any type satisfying the `chain` concept) and reuse cached intermediate
products from `fk_result` to avoid redundant `exp()` calls. Matrix-form FK is
also provided for downstream consumers that prefer 3x3 rotation matrices over
quaternions in the per-joint intermediates.

See [PoE Kinematics](../background/poe-kinematics.md) | [Jacobians](../background/jacobians.md)

## Headers

| Form | Header |
|------|--------|
| All kinematics | `#include <cartan/serial/fk.h>` |
| `cartan::forward_kinematics` | `#include <cartan/serial/fk/forward_kinematics.h>` |
| `cartan::forward_kinematics_matrix`, `cartan::fk_matrix_result`, `cartan::pose_matrix` | `#include <cartan/serial/fk/forward_kinematics_matrix.h>` |
| `cartan::space_jacobian`, `cartan::body_jacobian` | `#include <cartan/serial/fk/jacobian.h>` |
| `cartan::jacobian_matrix` | `#include <cartan/serial/fk/jacobian_matrix.h>` |
| `cartan::end_effector_velocity` | `#include <cartan/serial/fk/velocity.h>` |
| `cartan::fk_result` | `#include <cartan/serial/fk/fk_result.h>` |

## fk_result

Result of forward kinematics via Product of Exponentials. Caches all
intermediate products for Jacobian reuse.

```cpp
template <typename Scalar = double, int N = dynamic>
struct fk_result
{
    se3<Scalar> end_effector;                // End-effector pose T(q) = T_1...T_n * M
    intermediate_storage intermediates;      // Partial products T_i for Jacobian reuse
};
```

`intermediates[i]` holds the partial product
`exp([S1]q1) * ... * exp([S_{i+1}]q_{i+1})`. For fixed `N`,
`intermediate_storage` is `std::array<se3<Scalar>, N>`. For dynamic, it is
`std::vector<se3<Scalar>>`.

### Methods

```cpp
int num_joints() const;
```

Number of joints reflected in this result.

## forward_kinematics

Three overloads cover the supported chain types.

### kinematic_chain overload

```cpp
template <typename Scalar, int N>
fk_result<Scalar, N> forward_kinematics(
    const kinematic_chain<Scalar, N>& chain,
    const typename joint_state<Scalar, N>::position_type& q);
```

Computes the PoE formula:

```
T(q) = exp([S1]q1) * exp([S2]q2) * ... * exp([Sn]qn) * M
```

Dispatches to compile-time unrolled fold expressions for fixed-size chains
with `N=1-7` joints (zero-overhead expansion). For dynamic or larger chains,
uses a runtime loop.

### static_chain overload

```cpp
template <typename Scalar, joint_tag... Joints>
fk_result<Scalar, sizeof...(Joints)> forward_kinematics(
    const static_chain<Scalar, Joints...>& chain,
    const typename joint_state<Scalar, sizeof...(Joints)>::position_type& q);
```

Specialized forward kinematics for `static_chain` exploiting compile-time
joint-tag knowledge. Each joint's SE(3) exponential uses axis-specific
quaternion construction and sparse left-Jacobian entries instead of the
generic Rodrigues exponential map. Wins over the generic `chain`-concept
overload via partial ordering on `static_chain<Scalar, Joints...>`.

### Generic chain overload

```cpp
template <chain Chain>
fk_result<typename Chain::scalar_type, Chain::joints> forward_kinematics(
    const Chain& chain,
    const typename joint_state<typename Chain::scalar_type, Chain::joints>::position_type& q);
```

Forward kinematics for any chain type satisfying the `chain` concept. Uses
per-element `axis(i)` access and a runtime loop. The `kinematic_chain` and
`static_chain` overloads are more constrained and win for those specific
types via partial ordering; this overload handles any future chain types.

Reference: Lynch & Park, Modern Robotics, Eq. 4.10, p. 138.

## forward_kinematics_matrix

Matrix-form Product of Exponentials FK. Stores rotation as a 3x3 matrix in
the per-joint cumulative intermediates, avoiding the quaternion product in
compose and the quaternion-to-matrix conversion that downstream Jacobian
computation would otherwise pay on every column.

```cpp
template <typename Scalar, int N>
fk_matrix_result<Scalar, N> forward_kinematics_matrix(
    const kinematic_chain<Scalar, N>& chain,
    const typename joint_state<Scalar, N>::position_type& q);
```

Empirically faster than the quaternion-form `forward_kinematics` on chains
where the Jacobian is the downstream consumer. Use this overload when
building space / body Jacobians in tight inner loops; use `forward_kinematics`
when SE(3) composition or `act(p)` is the downstream consumer.

### fk_matrix_result

```cpp
template <typename Scalar = double, int N = dynamic>
struct fk_matrix_result
{
    pose_matrix<Scalar> end_effector;
    intermediate_matrix_storage intermediates;
};
```

Same shape as `fk_result`, but `end_effector` and each intermediate are
stored as `pose_matrix<Scalar>` (a 4x4 homogeneous matrix) rather than
`se3<Scalar>`.

### pose_matrix

```cpp
template <typename Scalar>
using pose_matrix = Eigen::Matrix<Scalar, 4, 4>;
```

Alias for 4x4 homogeneous transformation matrix. Used by
`fk_matrix_result` and by Jacobian computations that consume the matrix
form.

## space_jacobian

Space Jacobian mapping joint velocities to the end-effector spatial twist.

```cpp
template <typename Scalar, int N>
jacobian_matrix<Scalar, N> space_jacobian(
    const kinematic_chain<Scalar, N>& chain,
    const fk_result<Scalar, N>& fk);
```

Computes `J_s(q)` where column `i` is:

```
J_si = Ad_{T_{i-1}} * S_i
```

with `T_0 = I` (identity). Maps joint velocities to the spatial twist:
`V_s = J_s(q) * dq`. Reuses the cached intermediate products in `fk` to
avoid redundant `exp()` calls.

Returns a `jacobian_matrix<Scalar, N>` — a `6 x N` matrix (fixed) or
`6 x Dynamic` matrix.

Dispatch: same compile-time unrolling as `forward_kinematics` for `N=1-7`.

## body_jacobian

Body Jacobian mapping joint velocities to the end-effector body-frame twist.

```cpp
template <typename Scalar, int N>
jacobian_matrix<Scalar, N> body_jacobian(
    const kinematic_chain<Scalar, N>& chain,
    const fk_result<Scalar, N>& fk);
```

Computed as `J_b(q) = Ad_{T^{-1}} * J_s(q)`, where `T` is the end-effector
pose from `fk`. Maps joint velocities to the body-frame twist:
`V_b = J_b(q) * dq`.

## jacobian_matrix

```cpp
template <typename Scalar, int N>
using jacobian_matrix = std::conditional_t<
    N == dynamic,
    Eigen::Matrix<Scalar, 6, Eigen::Dynamic>,
    Eigen::Matrix<Scalar, 6, N>>;
```

Type alias returned by `space_jacobian` and `body_jacobian`. Selects
fixed-column or dynamic-column storage based on `N`.

## end_effector_velocity

Compute the end-effector spatial twist from joint positions and velocities.

```cpp
template <typename Scalar, int N>
vector6<Scalar> end_effector_velocity(
    const kinematic_chain<Scalar, N>& chain,
    const typename joint_state<Scalar, N>::position_type& q,
    const typename joint_state<Scalar, N>::velocity_type& dq);
```

Convenience function that computes FK internally, builds the space
Jacobian, then returns `V_s = J_s(q) * dq`. Less efficient than
calling `forward_kinematics` + `space_jacobian` separately when the
Jacobian is also needed downstream.

Reference: Lynch & Park, Modern Robotics, Eq. 5.10, p. 178.

## Edge Cases

- **`space_jacobian` and `body_jacobian` accept `fk_result` to avoid
  redundant `exp()` calls.** Compute FK first, then pass the result to
  the Jacobian function. `end_effector_velocity` computes FK internally
  (use it only if you do not need the Jacobian separately).
- **Fixed vs. dynamic dispatch:** for `N=1-7`, compile-time fold
  expressions produce zero-overhead code. For `N > 7` or `N == dynamic`,
  a runtime loop is used. The API is identical in both cases.
- **Matrix vs. quaternion form:** `forward_kinematics_matrix` is the
  faster path when the Jacobian is the downstream consumer.
  `forward_kinematics` is the natural choice when SE(3) composition
  or pose transforms are the downstream consumer.
