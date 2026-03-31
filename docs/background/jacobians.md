# Space and Body Jacobians

The **Jacobian** of a serial manipulator maps joint velocities
$\dot{\theta} \in \mathbb{R}^n$ to the end-effector twist
$\mathcal{V} \in \mathbb{R}^6$. Unlike the classical geometric Jacobian, the
Jacobian defined through the Product of Exponentials relates joint velocities
to a spatial twist -- a 6-vector combining angular and linear velocity in a
single reference frame [1, Ch. 5, pp. 169--194].

This page derives the space and body Jacobians column-by-column, discusses
singularities and manipulability, and maps the theory to liepp's implementation.

## Space Jacobian

### Definition

The **space Jacobian** $J_s(\theta) \in \mathbb{R}^{6 \times n}$ maps joint
velocities to the end-effector spatial twist expressed in the fixed space
frame $\{s\}$ [1, Eq. 5.10, p. 178]:

$$
\mathcal{V}_s = J_s(\theta) \, \dot{\theta}
$$

where $\mathcal{V}_s = (\omega_s, v_s) \in \mathbb{R}^6$ is the spatial twist
of the end-effector.

### Column-by-Column Derivation

Each column $J_{s,i}(\theta)$ of the space Jacobian corresponds to the
contribution of joint $i$ to the end-effector twist when $\dot{\theta}_i = 1$
and all other joint velocities are zero.

Recall the space-form PoE:
$T(\theta) = e^{[\mathcal{S}_1]\theta_1} \cdots e^{[\mathcal{S}_n]\theta_n} M$.
Taking the time derivative and extracting the spatial twist yields
[1, Eq. 5.11, p. 178]:

$$
J_{s,i}(\theta) = \text{Ad}_{e^{[\mathcal{S}_1]\theta_1} \cdots e^{[\mathcal{S}_{i-1}]\theta_{i-1}}} \, \mathcal{S}_i
$$

Writing out each column explicitly:

$$
J_s(\theta) = \begin{bmatrix} J_{s,1} & J_{s,2} & \cdots & J_{s,n} \end{bmatrix}
$$

where:

$$
\begin{aligned}
J_{s,1} &= \mathcal{S}_1 \\
J_{s,2} &= \text{Ad}_{e^{[\mathcal{S}_1]\theta_1}} \, \mathcal{S}_2 \\
J_{s,3} &= \text{Ad}_{e^{[\mathcal{S}_1]\theta_1} e^{[\mathcal{S}_2]\theta_2}} \, \mathcal{S}_3 \\
&\;\;\vdots \\
J_{s,n} &= \text{Ad}_{e^{[\mathcal{S}_1]\theta_1} \cdots e^{[\mathcal{S}_{n-1}]\theta_{n-1}}} \, \mathcal{S}_n
\end{aligned}
$$

**Key observation:** The first column is always $\mathcal{S}_1$ -- the base
joint's screw axis is fixed in the space frame and requires no adjoint
transformation. Each subsequent column applies the adjoint of the accumulated
product of exponentials up to (but not including) that joint.

**Physical interpretation:** The adjoint $\text{Ad}_T$ transforms a twist from
one frame to another. The accumulated product
$T_{i-1} = e^{[\mathcal{S}_1]\theta_1} \cdots e^{[\mathcal{S}_{i-1}]\theta_{i-1}}$
represents the transformation due to all joints proximal to joint $i$. The
adjoint of $T_{i-1}$ maps the screw axis $\mathcal{S}_i$ (defined at the home
position) to its current configuration, accounting for the motion of all
preceding joints.

### The Adjoint Representation

For $T = \begin{bmatrix} R & p \\ 0 & 1 \end{bmatrix} \in SE(3)$, the
$6 \times 6$ adjoint matrix is [1, Eq. 3.83, p. 107]:

$$
\text{Ad}_T = \begin{bmatrix} R & 0 \\ [p]_\times R & R \end{bmatrix}
$$

The adjoint acts on a twist $\mathcal{V} = (\omega, v)$ as:
$\text{Ad}_T \mathcal{V} = (R\omega, \; [p]_\times R\omega + Rv)$.

## Body Jacobian

### Definition

The **body Jacobian** $J_b(\theta) \in \mathbb{R}^{6 \times n}$ maps joint
velocities to the end-effector twist expressed in the body frame $\{b\}$
[1, Eq. 5.22, p. 185]:

$$
\mathcal{V}_b = J_b(\theta) \, \dot{\theta}
$$

### Relationship to the Space Jacobian

The body Jacobian is obtained from the space Jacobian by a single adjoint
transformation [1, Eq. 5.22, p. 185]:

$$
J_b(\theta) = \text{Ad}_{T^{-1}(\theta)} \, J_s(\theta)
$$

where $T(\theta)$ is the end-effector pose from the PoE. This requires only
one $6 \times 6$ matrix multiply applied to the entire $6 \times n$ space
Jacobian -- a constant-cost transformation regardless of the number of joints.

**Derivation.** The spatial and body twists are related by
$\mathcal{V}_b = \text{Ad}_{T^{-1}} \mathcal{V}_s$. Substituting
$\mathcal{V}_s = J_s \dot{\theta}$ gives
$\mathcal{V}_b = \text{Ad}_{T^{-1}} J_s \dot{\theta} = J_b \dot{\theta}$.

### When to Use Body vs Space Jacobian

- **Space Jacobian** ($J_s$): Natural for spatial motion planning and control
  in the world frame. Used when the task is defined in the fixed frame.

- **Body Jacobian** ($J_b$): Natural for tool-frame control, where the task is
  defined relative to the end-effector (e.g., "move the tool tip forward by
  1 cm"). Inverse kinematics algorithms typically use $J_b$ because the error
  twist is most naturally expressed in the body frame.

## Singularities

A configuration $\theta$ is **singular** when the Jacobian $J(\theta)$ loses
rank. At a singularity, certain end-effector velocities become unachievable --
the manipulator loses one or more degrees of freedom in task space
[1, Sec. 5.3, pp. 186--190].

### Formal Definition

The rank of $J_s(\theta)$ (or equivalently $J_b(\theta)$, since the adjoint
is always full rank) drops below its maximum value. For a 6-DOF manipulator,
the Jacobian is $6 \times 6$ and the robot is singular when $\det(J) = 0$.
For redundant manipulators ($n > 6$), the Jacobian is $6 \times n$ and the
robot is singular when $\text{rank}(J) < 6$.

### Physical Interpretation

At a singularity:

1. **Lost motion directions:** There exist end-effector twists
   $\mathcal{V} \neq 0$ for which no joint velocity $\dot{\theta}$ satisfies
   $J\dot{\theta} = \mathcal{V}$.

2. **Infinite joint velocities:** To achieve a finite end-effector velocity in
   the degenerate direction, infinite joint velocities would be required.

3. **Self-motion:** There may exist non-zero joint velocities
   $\dot{\theta} \neq 0$ for which $J\dot{\theta} = 0$ -- the joints move but
   the end-effector does not. These self-motions span the null space of $J$.

### Manipulability

The **manipulability measure** $\mu(\theta)$ quantifies how far a configuration
is from singularity [1, Sec. 5.3, p. 186]:

$$
\mu(\theta) = \sqrt{\det(J(\theta) \, J(\theta)^\top)}
$$

When $\mu = 0$, the configuration is singular. The manipulability ellipsoid,
defined by the singular values of $J$, visualizes the set of achievable
end-effector velocities for a unit-norm joint velocity vector.

## Numerical Computation in liepp

liepp computes the space Jacobian column-by-column using the cached
intermediate products from `fk_result`. This avoids redundant matrix
exponential computations:

1. The forward kinematics `forward_kinematics(chain, q)` stores all
   intermediate products $T_i = e^{[\mathcal{S}_1]\theta_1} \cdots e^{[\mathcal{S}_i]\theta_i}$
   in `fk_result::intermediates`.

2. `space_jacobian(chain, fk)` computes each column as:
   - Column 0: $\mathcal{S}_1$ (no adjoint needed)
   - Column $i > 0$: $\text{Ad}_{T_{i-1}} \mathcal{S}_i$ using the cached
     `fk.intermediates[i-1]`

3. `body_jacobian(chain, fk)` applies a single $6 \times 6$ adjoint of
   $T^{-1}$ to the entire space Jacobian:
   $J_b = \text{Ad}_{T^{-1}} J_s$.

For fixed-size chains ($N = 1$--$7$), the column loop is unrolled at compile
time via fold expressions. For dynamic or larger chains, a runtime loop is
used. Both paths produce identical results.

## liepp Mapping

| Concept | liepp API |
|---------|-----------|
| Space Jacobian $J_s(\theta)$ | `space_jacobian(chain, fk)` |
| Body Jacobian $J_b(\theta)$ | `body_jacobian(chain, fk)` |
| Spatial twist $\mathcal{V}_s = J_s \dot{\theta}$ | `end_effector_velocity(chain, q, dq)` |
| Jacobian matrix type | `jacobian_matrix<N, Scalar>` |
| Cached intermediates for Jacobian | `fk_result::intermediates` |

The `end_effector_velocity` function is a convenience that computes FK and the
space Jacobian internally, returning the 6-vector spatial twist directly.

See [API Reference](../api/kinematics.md) for full function signatures.
See [PoE Kinematics](poe-kinematics.md) for the underlying forward kinematics.

## Bibliography

[1] K. M. Lynch and F. C. Park, "Modern Robotics: Mechanics, Planning, and
Control," Cambridge University Press, 2017.

[2] T. Yoshikawa, "Manipulability of Robotic Mechanisms," *International
Journal of Robotics Research*, vol. 4, no. 2, pp. 3--9, 1985.

[3] B. Siciliano, L. Sciavicco, L. Villani, and G. Oriolo, "Robotics:
Modelling, Planning and Control," 2nd ed., Springer, 2009.
