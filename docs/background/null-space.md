# Null-Space Projection and Joint Limit Avoidance

For **redundant manipulators** -- robots with more joints than task-space
degrees of freedom ($n > 6$) -- the Jacobian has a non-trivial null space.
Motions in this null space produce zero end-effector displacement, allowing
secondary objectives (such as joint limit avoidance) to be optimized without
affecting the primary IK task [1, Ch. 6, pp. 235--237].

This page derives the null-space projector, the combined update step for
joint limit avoidance, and maps to Cartan's limits policies.

## Null Space of the Jacobian

### Definition

The **null space** of the body Jacobian $J_b \in \mathbb{R}^{6 \times n}$ is
the set of joint velocities that produce zero end-effector twist
[1, Sec. 6.3, p. 235]:

$$
\mathcal{N}(J_b) = \{\Delta\theta \in \mathbb{R}^n : J_b \, \Delta\theta = 0\}
$$

By the rank-nullity theorem:

$$
\dim(\mathcal{N}(J_b)) = n - \text{rank}(J_b)
$$

For a non-singular $n$-DOF manipulator with $n > 6$, the Jacobian has rank 6
and the null space has dimension $n - 6$. This means there is an
$(n-6)$-dimensional family of joint motions that leave the end-effector
stationary.

### Physical Interpretation

Null-space motions are **self-motions** of the robot: the links reconfigure
while the end-effector remains fixed. A 7-DOF arm like the KUKA LBR Med has
a 1-dimensional null space at non-singular configurations -- the "elbow
swivel" is the classic example of a null-space self-motion.

## Null-Space Projector

### Construction from the Pseudoinverse

The **null-space projector** $P \in \mathbb{R}^{n \times n}$ maps any joint
velocity vector into the null space of $J_b$ [1, Eq. 6.13, p. 236]:

$$
P = I_n - J_b^\dagger J_b
$$

where $J_b^\dagger$ is the Moore-Penrose pseudoinverse of $J_b$. For any
$\Delta\theta_0 \in \mathbb{R}^n$:

$$
J_b (P \, \Delta\theta_0) = J_b \Delta\theta_0 - J_b J_b^\dagger J_b \Delta\theta_0 = J_b \Delta\theta_0 - J_b \Delta\theta_0 = 0
$$

confirming that $P \, \Delta\theta_0$ lies in $\mathcal{N}(J_b)$.

### SVD-Based Construction

In practice, the null-space projector is computed from the SVD of $J_b$.
Given $J_b = U \Sigma V^\top$ with rank $r$, the last $n - r$ columns of $V$
(denoted $V_{\text{null}}$) span the null space:

$$
P = V_{\text{null}} \, V_{\text{null}}^\top
$$

This is numerically more stable than computing $J_b^\dagger J_b$ directly,
especially when the Jacobian is near-singular.

## Joint Limit Avoidance

### The Combined Update Step

The general form of a null-space-augmented IK step combines a primary task
(minimizing the error twist) with a secondary objective (pushing joints away
from their limits) [1, Eq. 6.14, p. 236]:

$$
\Delta\theta = J_b^\dagger \, \xi_b + \alpha \, P \, \nabla h(\theta)
$$

where:

- $J_b^\dagger \xi_b$ is the primary IK step (minimizes body-frame error)
- $\nabla h(\theta)$ is the gradient of a secondary cost function
- $P$ is the null-space projector
- $\alpha > 0$ is a step size for the secondary objective

The projection $P \nabla h(\theta)$ ensures the secondary motion does not
interfere with the primary task.

### Joint-Centering Gradient

A common choice for $h(\theta)$ penalizes deviation from the joint range
midpoints (Liegeois, 1977):

$$
h(\theta) = -\frac{1}{2n} \sum_{i=1}^{n} \left(\frac{\theta_i - \bar{\theta}_i}{\theta_{i,\max} - \theta_{i,\min}}\right)^2
$$

where $\bar{\theta}_i = (\theta_{i,\min} + \theta_{i,\max}) / 2$ is the
midpoint of joint $i$'s range. The gradient pushes each joint toward its
midpoint:

$$
\frac{\partial h}{\partial \theta_i} = -\frac{\theta_i - \bar{\theta}_i}{(\theta_{i,\max} - \theta_{i,\min})^2}
$$

After projection into the null space and addition to the primary step, joints
drift toward their midpoints without affecting the end-effector trajectory.

### Why Null-Space Projection Is Needed

Without projection, adding a joint-centering term directly to the IK step
would perturb the end-effector from its target. The projector $P$ ensures
that only the component of $\nabla h$ that has zero effect on the end-effector
is used. This is the fundamental insight: the null space decouples task-space
control from configuration-space optimization.

## Cartan's Limits Policies

Cartan implements joint limit enforcement through **policy-based design**. Three
policies are provided, selectable at compile time:

### `no_limits`

No enforcement. Use when the stepper itself handles constraints (e.g.,
`sqp_stepper` with box bounds from NLopt).

### `clamp_limits`

Hard clamping: each $\theta_i$ is clamped to
$[\theta_{i,\min}, \theta_{i,\max}]$ after each step. Simple and robust, but
may cause discontinuities at joint boundaries and does not exploit the null
space.

### `null_space_limits`

Null-space projection toward joint midpoints, followed by safety clamping.
This is the most sophisticated policy and implements the theory above:

1. Compute the null-space gradient: $\Delta\theta_{\text{null},i} = -\alpha (\theta_i - \bar{\theta}_i) / (\theta_{i,\max} - \theta_{i,\min})^2$
2. Extract the null-space basis $V_{\text{null}}$ from the SVD of $J_b$
3. Project: $\Delta\theta_{\text{proj}} = V_{\text{null}} V_{\text{null}}^\top \Delta\theta_{\text{null}}$
4. Apply: $\theta \leftarrow \theta + \Delta\theta_{\text{proj}}$
5. Safety clamp to hard limits

This policy requires the body Jacobian and its SVD, which are available via
the `enforce_extended` interface (detected at compile time via the
`has_extended_enforce` concept).

### Policy Concept

All limits policies satisfy a common interface:

```cpp
template <int N, typename Scalar>
static void enforce(position_type& q, const limits_storage& limits);
```

The `null_space_limits` policy additionally provides:

```cpp
template <int N, typename Scalar>
static void enforce_extended(
    position_type& q, const limits_storage& limits,
    const jacobian_matrix<N, Scalar>& J_b,
    const JacobiSVD<...>& svd, Scalar gain = 0.5);
```

See [API Reference](../api/ik.md) for the complete limits policy interface.
See [IK Methods](ik-methods.md) for the IK algorithms that use these policies.

## Bibliography

[1] K. M. Lynch and F. C. Park, "Modern Robotics: Mechanics, Planning, and
Control," Cambridge University Press, 2017.

[2] A. Liegeois, "Automatic Supervisory Control of the Configuration and
Behavior of Multibody Mechanisms," *IEEE Transactions on Systems, Man, and
Cybernetics*, vol. 7, no. 12, pp. 868--871, 1977.

[3] Y. Nakamura, "Advanced Robotics: Redundancy and Optimization,"
Addison-Wesley, 1991.

[4] B. Siciliano, L. Sciavicco, L. Villani, and G. Oriolo, "Robotics:
Modelling, Planning and Control," 2nd ed., Springer, 2009.
