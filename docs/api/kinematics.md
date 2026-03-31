# kinematics

Part of the `liepp::kinematics` module (`lib/liepp-kinematics/`). Link with `liepp::kinematics` (or the convenience target `liepp::liepp`).

Forward kinematics and Jacobian computation using Product of Exponentials formulation. All functions accept a `kinematic_chain` and reuse cached intermediate products from `fk_result` to avoid redundant `exp()` calls.

See [PoE Kinematics](../background/poe-kinematics.md) | [Jacobians](../background/jacobians.md)

## Headers

| Form | Header |
|------|--------|
| All kinematics | `#include <liepp/kinematics/kinematics.h>` |
| `liepp::forward_kinematics` | `#include <liepp/kinematics/forward_kinematics.h>` |
| `liepp::space_jacobian`, `liepp::body_jacobian` | `#include <liepp/kinematics/jacobian.h>` |
| `liepp::end_effector_velocity` | `#include <liepp/kinematics/velocity.h>` |
| `liepp::fk_result` | `#include <liepp/kinematics/fk_result.h>` |

## fk_result

Result of forward kinematics via Product of Exponentials. Caches all intermediate products for Jacobian reuse.

```cpp
template <typename Scalar = double, int N = dynamic>
struct fk_result
{
    se3<Scalar> end_effector;                // End-effector pose T(q) = T_1...T_n * M
    intermediate_storage intermediates;      // Partial products T_i for Jacobian reuse
};
```

`intermediates[i]` holds the partial product `exp([S1]q1) * ... * exp([S_{i+1}]q_{i+1})`. For fixed `N`, `intermediate_storage` is `std::array<se3<Scalar>, N>`. For dynamic, it is `std::vector<se3<Scalar>>`.

### Methods

```cpp
int num_joints() const;
```

Number of joints reflected in this result.

## forward_kinematics

Compute end-effector pose and cache intermediate products for Jacobian computation.

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

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `chain` | `const kinematic_chain<Scalar, N>&` | Kinematic chain with screw axes and home configuration. |
| `q` | `position_type` | Joint position vector. For fixed `N`: `Eigen::Vector<Scalar, N>`. For dynamic: `Eigen::VectorX<Scalar>`. |

**Returns:** `fk_result<Scalar, N>` containing the end-effector SE(3) pose and all intermediate products.

**Dispatch:** For fixed-size chains with `N=1-7` joints, uses compile-time unrolled fold expressions for zero-overhead expansion. For dynamic or larger chains, uses a runtime loop.

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

with `T_0 = I` (identity). Maps joint velocities to spatial twist: `V_s = J_s(q) * dq`.

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `chain` | `const kinematic_chain<Scalar, N>&` | Kinematic chain with screw axes. |
| `fk` | `const fk_result<Scalar, N>&` | Forward kinematics result with cached intermediates. |

**Returns:** `jacobian_matrix<Scalar, N>` -- a `6 x N` matrix (fixed) or `6 x Dynamic` matrix.

```cpp
template <typename Scalar, int N>
using jacobian_matrix = std::conditional_t<
    N == dynamic,
    Eigen::Matrix<Scalar, 6, Eigen::Dynamic>,
    Eigen::Matrix<Scalar, 6, N>>;
```

**Dispatch:** Same compile-time unrolling as `forward_kinematics` for `N=1-7`.

## body_jacobian

Body Jacobian mapping joint velocities to the end-effector body-frame twist.

```cpp
template <typename Scalar, int N>
jacobian_matrix<Scalar, N> body_jacobian(
    const kinematic_chain<Scalar, N>& chain,
    const fk_result<Scalar, N>& fk);
```

Computed as `J_b(q) = Ad_{T^{-1}} * J_s(q)`, where `T` is the end-effector pose from `fk`. Maps joint velocities to body-frame twist: `V_b = J_b(q) * dq`.

**Parameters:** Same as `space_jacobian`.

**Returns:** `jacobian_matrix<Scalar, N>` -- 6xN body-frame Jacobian.

## end_effector_velocity

Compute end-effector spatial twist from joint positions and velocities.

```cpp
template <typename Scalar, int N>
vector6<Scalar> end_effector_velocity(
    const kinematic_chain<Scalar, N>& chain,
    const typename joint_state<Scalar, N>::position_type& q,
    const typename joint_state<Scalar, N>::velocity_type& dq);
```

Convenience function that computes FK internally, builds the space Jacobian, then returns `V_s = J_s(q) * dq`.

**Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `chain` | `const kinematic_chain<Scalar, N>&` | Kinematic chain. |
| `q` | `position_type` | Joint position vector. |
| `dq` | `velocity_type` | Joint velocity vector. |

**Returns:** `vector6<Scalar>` -- 6-vector spatial twist `V_s`.

## Edge Cases

- **`space_jacobian` and `body_jacobian` accept `fk_result` to avoid redundant `exp()` calls.** Always compute FK first, then pass the result to Jacobian functions. If you call `end_effector_velocity`, it computes FK internally (less efficient if you also need the Jacobian separately).
- **Fixed vs. dynamic dispatch:** For `N=1-7`, compile-time fold expressions produce zero-overhead code. For `N > 7` or `N == dynamic`, a runtime loop is used. The API is identical in both cases.
- **SVD for body Jacobian:** The body Jacobian is computed as `Ad_{T^{-1}} * J_s`, which involves a 6x6 adjoint multiplication. No SVD is performed here; SVD is only used in the IK module.
