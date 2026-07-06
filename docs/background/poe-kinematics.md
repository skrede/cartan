# Product of Exponentials and Forward Kinematics

The **Product of Exponentials (PoE)** formula provides an elegant, coordinate-free
approach to forward kinematics based on screw theory. Unlike the
Denavit-Hartenberg (D-H) convention, the PoE formula requires no link reference
frames -- only a fixed space frame $\{s\}$, an end-effector body frame $\{b\}$,
and the screw axes of each joint expressed in the space frame
[1, Ch. 4, pp. 135--168].

Cartan uses the PoE formulation exclusively. The advantages over D-H parameters
include: no frame-attachment ambiguity (different D-H conventions lead to
different parameters for the same robot), uniform treatment of revolute and
prismatic joints, and global coordinates that compose naturally with Lie group
operations [1, Sec. 4.5, pp. 158].

## Screw Motions

A **screw axis** $\mathcal{S} = (\omega, v) \in \mathbb{R}^6$ describes the instantaneous
motion of a rigid body. It is a twist normalized so that either
$\lVert \omega \rVert = 1$ (revolute) or $\omega = 0, \lVert v \rVert = 1$
(prismatic) [1, Def. 3.24, p. 102].

### Revolute Joints

For a revolute joint with rotation axis direction $\hat{\omega} \in \mathbb{R}^3$
($\lVert \hat{\omega} \rVert = 1$) passing through a point $q \in \mathbb{R}^3$:

$$
\mathcal{S} = \begin{bmatrix} \omega \\ v \end{bmatrix} = \begin{bmatrix} \hat{\omega} \\ -\hat{\omega} \times q \end{bmatrix}
$$

The angular component $\omega$ is the unit rotation axis. The linear component
$v = -\omega \times q$ captures the translational velocity that a point at the
origin of $\{s\}$ would experience if the rigid body were rotating about the
joint axis at unit angular speed. Any point $q$ on the axis gives the same
screw axis, since the cross product $\omega \times q$ is invariant to translation
along $\omega$.

**Physical interpretation:** Imagine freezing all other joints and rotating
this joint at 1 rad/s. The resulting spatial twist of the distal links is
exactly $\mathcal{S}$.

### Prismatic Joints

For a prismatic joint translating along direction $\hat{v} \in \mathbb{R}^3$
($\lVert \hat{v} \rVert = 1$):

$$
\mathcal{S} = \begin{bmatrix} 0 \\ \hat{v} \end{bmatrix}
$$

There is no rotation ($\omega = 0$), and the linear component is the unit
translation direction.

### The Screw Axis as an se(3) Element

The $4 \times 4$ matrix representation of a screw axis is
[1, Eq. 3.63, p. 101]:

$$
[\mathcal{S}] = \begin{bmatrix} [\omega]_\times & v \\ 0 & 0 \end{bmatrix} \in se(3)
$$

where $[\omega]_\times$ is the $3 \times 3$ skew-symmetric matrix of $\omega$.
Multiplying by a joint displacement $\theta$ gives $[\mathcal{S}]\theta$, and
exponentiating produces the corresponding $SE(3)$ transformation:
$e^{[\mathcal{S}]\theta} \in SE(3)$.

## Product of Exponentials Formula

### Space Form

Consider an $n$-joint serial chain with fixed space frame $\{s\}$ and
end-effector frame $\{b\}$. Let $M \in SE(3)$ denote the end-effector
configuration when all joint angles are zero (the **home configuration**).
Let $\mathcal{S}_1, \ldots, \mathcal{S}_n$ be the screw axes of the joints
expressed in the fixed space frame when the robot is at its zero position
[1, Sec. 4.1.1, pp. 139--140].

The forward kinematics is:

$$
T(\theta) = e^{[\mathcal{S}_1]\theta_1} \, e^{[\mathcal{S}_2]\theta_2} \cdots e^{[\mathcal{S}_n]\theta_n} \, M
$$

This is the **space form** of the Product of Exponentials formula
[1, Eq. 4.14, p. 140].

**Step-by-step derivation.** Starting from the rightmost joint $n$:

1. When only joint $n$ moves by $\theta_n$, the end-effector undergoes a screw
   displacement: $T = e^{[\mathcal{S}_n]\theta_n} M$.

2. When joint $n-1$ also moves, it applies a screw motion to the entire
   sub-assembly (link $n$ and the end-effector):
   $T = e^{[\mathcal{S}_{n-1}]\theta_{n-1}} \left( e^{[\mathcal{S}_n]\theta_n} M \right)$.

3. Continuing inductively to joint 1 yields the full PoE formula.

The key insight is that each screw axis $\mathcal{S}_i$ is defined in the
**fixed** space frame at the home position. The space-frame representation of
$\mathcal{S}_i$ is unaffected by displacements of more distal joints
(joints $i+1, \ldots, n$), which is why the formula works without needing to
recompute screw axes as joints move [1, Sec. 4.1.3, p. 148].

**Why the exponentials compose left-to-right.** In the space form, $M$ is first
transformed by the most distal joint, progressively moving inward to more
proximal joints. Each $e^{[\mathcal{S}_i]\theta_i}$ pre-multiplies the running
product because it describes a fixed-frame screw displacement.

### The Home Configuration

The matrix $M \in SE(3)$ encodes the end-effector pose when all joints are at
their zero values. It is the "reference" configuration from which all screw
displacements are measured.

For a planar 2R arm with link lengths $L_1, L_2$, with both joints at zero
the end-effector tip is at $(L_1 + L_2, 0)$ with identity orientation:

$$
M = \begin{bmatrix} 1 & 0 & 0 & L_1 + L_2 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

## Space Frame vs Body Frame PoE

### Body Form

The PoE can also be expressed with screw axes in the end-effector (body)
frame. Each body-frame screw axis $\mathcal{B}_i$ is related to the
space-frame axis by the adjoint of $M^{-1}$
[1, Eq. 4.16, p. 147]:

$$
\mathcal{B}_i = \text{Ad}_{M^{-1}} \mathcal{S}_i
$$

The body form of the PoE is:

$$
T(\theta) = M \, e^{[\mathcal{B}_1]\theta_1} \, e^{[\mathcal{B}_2]\theta_2} \cdots e^{[\mathcal{B}_n]\theta_n}
$$

In the body form, $M$ is first transformed by joint 1, progressively moving
outward to more distal joints. The body-frame screw axis for a more distal
joint is unaffected by the displacement of a more proximal joint.

### Relationship Between Forms

The two forms are equivalent and related by the matrix identity
$Me^{M^{-1}PM} = e^P M$. Cartan uses the **space form** as its primary
convention, consistent with Lynch & Park's recommendation and the natural
composition of fixed-frame screw displacements.

## Worked Example: 2-Link Planar Arm

Consider a 2R planar arm with link lengths $L_1 = 1$ m and $L_2 = 1$ m.
Both revolute joints rotate about the $\hat{z}$-axis. The space frame
$\{s\}$ is at the base, and the end-effector frame $\{b\}$ is at the tip.

**Step 1: Home configuration.** With $\theta_1 = \theta_2 = 0$, the arm is
fully extended along $\hat{x}$:

$$
M = \begin{bmatrix} 1 & 0 & 0 & 2 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

**Step 2: Screw axes.** Both joints are revolute about $\hat{z} = (0,0,1)$.

- Joint 1 passes through the origin: $q_1 = (0,0,0)$, so
  $v_1 = -\omega_1 \times q_1 = (0,0,0)$.

$$
\mathcal{S}_1 = (0, 0, 1, 0, 0, 0)
$$

- Joint 2 passes through $(L_1, 0, 0) = (1,0,0)$:
  $v_2 = -(0,0,1) \times (1,0,0) = (0, -1, 0)$.

$$
\mathcal{S}_2 = (0, 0, 1, 0, -1, 0)
$$

**Step 3: Forward kinematics at $\theta_1 = \pi/4, \theta_2 = \pi/4$.**

$$
T(\theta) = e^{[\mathcal{S}_1]\pi/4} \, e^{[\mathcal{S}_2]\pi/4} \, M
$$

Computing the first exponential (pure rotation about origin):

$$
e^{[\mathcal{S}_1]\pi/4} = \begin{bmatrix} \cos(\pi/4) & -\sin(\pi/4) & 0 & 0 \\ \sin(\pi/4) & \cos(\pi/4) & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix} = \begin{bmatrix} 0.707 & -0.707 & 0 & 0 \\ 0.707 & 0.707 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

Computing the second exponential (rotation about $(1,0,0)$):

$$
e^{[\mathcal{S}_2]\pi/4} = \begin{bmatrix} 0.707 & -0.707 & 0 & 0.293 \\ 0.707 & 0.707 & 0 & -0.707 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

Multiplying out $T = e^{[\mathcal{S}_1]\pi/4} \, e^{[\mathcal{S}_2]\pi/4} \, M$
yields the end-effector at position $(x, y) \approx (0.707, 1.707)$ rotated
by $\pi/2$ from the base frame, which matches the geometric intuition of
both joints bent 45 degrees.

## Cartan Mapping

The PoE concepts map directly to Cartan's API:

| Concept | Cartan API |
|---------|-----------|
| Screw axis $\mathcal{S}$ (revolute) | `screw_axis::revolute(omega, point)` |
| Screw axis $\mathcal{S}$ (prismatic) | `screw_axis::prismatic(direction)` |
| Screw axis from 6-vector | `screw_axis::from_vector(vec)` |
| Kinematic chain $(M, \mathcal{S}_1 \ldots \mathcal{S}_n)$ | `kinematic_chain(home, axes, limits)` |
| Forward kinematics $T(\theta)$ | `forward_kinematics(chain, q)` |
| Intermediate products $T_i$ | `fk_result::intermediates` |
| End-effector pose | `fk_result::end_effector` |

Cartan computes FK via the space-form PoE. For fixed-size chains ($N = 1$--$7$),
it uses a compile-time unrolled fold expression for zero-overhead expansion.
For dynamic or larger chains, it uses a runtime loop. In both cases, all
intermediate products $T_i = e^{[\mathcal{S}_1]\theta_1} \cdots e^{[\mathcal{S}_i]\theta_i}$
are cached in `fk_result::intermediates` for reuse by Jacobian computations.

See [API Reference](../api/kinematics.md) for function signatures and usage.
See [SE(3) Theory](se3.md) for the underlying Lie group operations.

## Bibliography

[1] K. M. Lynch and F. C. Park, "Modern Robotics: Mechanics, Planning, and
Control," Cambridge University Press, 2017.

[2] R. M. Murray, Z. Li, and S. S. Sastry, "A Mathematical Introduction to
Robotic Manipulation," CRC Press, 1994.

[3] R. W. Brockett, "Robotic Manipulators and the Product of Exponentials
Formula," in *Mathematical Theory of Networks and Systems*, Lecture Notes in
Control and Information Sciences, vol. 58, Springer, 1984, pp. 120--129.
