# SE(3) and Rigid Body Transformations

The special Euclidean group SE(3) is the group of all rigid body
transformations in three-dimensional space. Each element combines a rotation
(from SO(3)) with a translation, representing the full 6-DOF configuration of
a rigid body. SE(3) is the core mathematical object in robotics: every
end-effector pose, every joint transformation, and every sensor frame is an
element of SE(3) [1, Ch. 3, pp. 86--106].

This page covers homogeneous transformation matrices, twists and screw
motions, the exponential and logarithmic maps with full derivations, and the
adjoint representation.

## Homogeneous Transformation Matrices

A rigid body transformation is a pair $(R, p)$ where $R \in \text{SO}(3)$ is a
rotation and $p \in \mathbb{R}^3$ is a translation. The set of all such pairs
forms SE(3) [1, Ch. 3, p. 86]:

$$
\text{SE}(3) = \{ (R, p) : R \in \text{SO}(3), \; p \in \mathbb{R}^3 \}
$$

This is a **6-dimensional** Lie group (3 DOF rotation + 3 DOF translation).

### Matrix Representation

SE(3) elements are represented as $4 \times 4$ homogeneous transformation
matrices:

$$
T = \begin{bmatrix} R & p \\ 0_{1 \times 3} & 1 \end{bmatrix} \in \mathbb{R}^{4 \times 4}
$$

where $R \in \text{SO}(3)$ and $p \in \mathbb{R}^3$.

### Composition

Transformations compose by matrix multiplication [1, Ch. 3, p. 92]:

$$
T_1 T_2 = \begin{bmatrix} R_1 & p_1 \\ 0 & 1 \end{bmatrix} \begin{bmatrix} R_2 & p_2 \\ 0 & 1 \end{bmatrix} = \begin{bmatrix} R_1 R_2 & R_1 p_2 + p_1 \\ 0 & 1 \end{bmatrix}
$$

The rotation components compose multiplicatively, and the translation of $T_2$
is first rotated by $R_1$ before adding $p_1$.

### Inverse

The inverse has a convenient closed form [1, Eq. 3.64, p. 94]:

$$
T^{-1} = \begin{bmatrix} R^\top & -R^\top p \\ 0 & 1 \end{bmatrix}
$$

This avoids a general $4 \times 4$ matrix inversion by exploiting the
structure of SE(3).

### Action on Points

A transformation $T$ acts on a point $q \in \mathbb{R}^3$:

$$
T \cdot q = R q + p
$$

or in homogeneous coordinates:
$T \begin{bmatrix} q \\ 1 \end{bmatrix} = \begin{bmatrix} Rq + p \\ 1 \end{bmatrix}$.

## Twists and the Lie Algebra se(3)

The Lie algebra $\mathfrak{se}(3)$ is the tangent space to SE(3) at the
identity. Its elements are $4 \times 4$ matrices of the form
[1, Ch. 3, p. 103]:

$$
[\mathcal{V}]^\wedge = \begin{bmatrix} [\omega]_\times & v \\ 0_{1 \times 3} & 0 \end{bmatrix} \in \mathbb{R}^{4 \times 4}
$$

where $[\omega]_\times \in \mathfrak{so}(3)$ is a $3 \times 3$ skew-symmetric
matrix and $v \in \mathbb{R}^3$.

### Twist Representation

The Lie algebra element is compactly represented as a 6-vector called a
**twist**. Cartan uses **omega-first** ordering following Lynch and Park [1]:

$$
\mathcal{V} = \begin{pmatrix} \omega \\ v \end{pmatrix} \in \mathbb{R}^6
$$

where $\omega \in \mathbb{R}^3$ is the angular velocity and
$v \in \mathbb{R}^3$ is the linear velocity.

> **Convention note:** Cartan uses omega-first twist ordering
> $\mathcal{V} = (\omega, v)$. Barfoot [2] uses the opposite ordering
> $\boldsymbol{\xi} = (\boldsymbol{\rho}, \boldsymbol{\phi})$ with
> translation first. See [Notation Conventions](notation.md) for conversion
> details.

### Hat and Vee Maps

The **hat map** $[\cdot]^\wedge : \mathbb{R}^6 \to \mathfrak{se}(3)$ sends a
6-vector to its $4 \times 4$ matrix:

$$
\left[\begin{pmatrix} \omega \\ v \end{pmatrix}\right]^\wedge = \begin{bmatrix} [\omega]_\times & v \\ 0 & 0 \end{bmatrix}
$$

The **vee operator** $(\cdot)^\vee$ is its inverse, recovering the 6-vector.

### Spatial and Body Twists

A **spatial twist** $\mathcal{V}_s = (\omega_s, v_s)$ is expressed in the
space (fixed) frame. A **body twist** $\mathcal{V}_b = (\omega_b, v_b)$ is
expressed in the body (moving) frame. They are related by the adjoint:

$$
\mathcal{V}_s = [\text{Ad}_T] \, \mathcal{V}_b
$$

## Screw Motions

Every twist can be interpreted geometrically as a **screw motion**: a
simultaneous rotation about and translation along an axis in space
[1, Def. 3.24, p. 102].

### Screw Axis

A **screw axis** $\mathcal{S} = (\omega, v)$ with $\|\omega\| = 1$ (for
revolute joints) or $\omega = 0$ (for prismatic joints) defines the geometry
of a joint:

- **Revolute:** $\mathcal{S} = (\hat{\omega}, -\hat{\omega} \times q + h\hat{\omega})$
  where $q$ is a point on the rotation axis and $h$ is the pitch (linear motion
  per radian of rotation).
- **Prismatic:** $\mathcal{S} = (0, \hat{v})$ where $\hat{v}$ is the unit
  translation direction.
- **General screw:** A rotation of angle $\theta$ about the screw axis
  combined with a translation of $h\theta$ along it.

### Chasles' Theorem

Every rigid body displacement can be represented as a rotation about some axis
combined with a translation along that axis. This is the screw motion
interpretation, and it means every element of SE(3) can be written as a single
matrix exponential $e^{[\mathcal{S}]\theta}$ for some screw axis
$\mathcal{S}$ and angle/distance $\theta$ [1, Theorem 3.1, p. 100].

## Exponential Map

The exponential map takes a twist $\mathcal{V} = (\omega, v)$ to a
transformation $T \in \text{SE}(3)$ [1, Prop. 3.25, Eq. 3.88, p. 103].

### Derivation

Starting from $T = \exp([\mathcal{V}]^\wedge)$, we separate the rotation and
translation components. For a screw axis $\mathcal{S} = (\omega, v)$ with
$\|\omega\| = 1$ and screw angle $\theta$:

$$
e^{[\mathcal{S}]\theta} = \begin{bmatrix} e^{[\omega]_\times\theta} & G(\theta) v \\ 0 & 1 \end{bmatrix}
$$

**The rotation block** is given by Rodrigues' formula (see [SO(3)](so3.md)):

$$
e^{[\omega]_\times\theta} = I + \sin\theta \, [\hat{\omega}]_\times + (1 - \cos\theta) \, [\hat{\omega}]_\times^2
$$

**The translation block** requires computing the integral:

$$
G(\theta) = \int_0^\theta e^{[\omega]_\times s} \, ds
$$

Substituting Rodrigues' formula and integrating term by term:

$$
G(\theta) = \int_0^\theta \left(I + \sin s \, [\hat{\omega}]_\times + (1 - \cos s) \, [\hat{\omega}]_\times^2\right) ds
$$

$$
= I\theta + (1 - \cos\theta) \, [\hat{\omega}]_\times + (\theta - \sin\theta) \, [\hat{\omega}]_\times^2
$$

The full exponential map is:

$$
\boxed{e^{[\mathcal{S}]\theta} = \begin{bmatrix} e^{[\omega]_\times\theta} & \left(I\theta + (1 - \cos\theta)[\hat{\omega}]_\times + (\theta - \sin\theta)[\hat{\omega}]_\times^2\right) v \\ 0 & 1 \end{bmatrix}}
$$

### General Form (Non-Unit $\omega$)

For an arbitrary twist $\mathcal{V} = (\omega, v)$ where $\phi = \omega$ is a
general rotation vector with $\theta = \|\phi\|$, the formula generalizes.
Cartan's implementation uses the **left Jacobian** $J_l(\phi)$ of SO(3) as the
coupling matrix [2, Eq. 8.33, p. 289]:

$$
T = \exp([\mathcal{V}]^\wedge) = \begin{bmatrix} \exp([\omega]_\times) & J_l(\omega) \, v \\ 0 & 1 \end{bmatrix}
$$

where $J_l(\omega)$ is the SO(3) left Jacobian evaluated at $\omega$. Note
that $G(\theta) = \theta \, J_l(\theta\hat{\omega})$ when $\omega = \theta\hat{\omega}$
is the unit-axis form: the coupling matrix $G(\theta)$ carries an extra factor
of $\theta$ because it multiplies the unit-axis linear velocity $v$, whereas
$J_l(\omega)$ multiplies the already-scaled twist component $\theta v$.

### Pure Translation ($\omega = 0$)

When $\omega = 0$, the exponential reduces to:

$$
\exp\left(\begin{bmatrix} 0 & v \\ 0 & 0 \end{bmatrix}\right) = \begin{bmatrix} I & v \\ 0 & 1 \end{bmatrix}
$$

Cartan handles this via the Taylor branch of $J_l$: $J_l(0) = I$.

## Logarithmic Map

The logarithmic map is the inverse of the exponential, taking a transformation
$T = (R, p)$ back to a twist [1, Eq. 3.91--3.92, p. 104; 2, Eq. 8.35,
p. 290].

### Derivation

**Step 1:** Extract the rotational component via the SO(3) logarithmic map:

$$
\omega = \log(R)
$$

This returns the 3-vector rotation vector $\phi = \theta\hat{\omega}$.

**Step 2:** Recover the linear velocity by inverting the coupling:

$$
p = J_l(\omega) \, v \quad \Longrightarrow \quad v = J_l(\omega)^{-1} \, p
$$

where $J_l^{-1}(\omega)$ is the inverse left Jacobian of SO(3) (see
[SO(3)](so3.md#left-and-right-jacobians)).

### Summary

$$
\boxed{\log(T) = \begin{pmatrix} \omega \\ J_l(\omega)^{-1} \, p \end{pmatrix}}
$$

This is precisely Cartan's `se3::log()` implementation: compute $\omega$ from
the rotation quaternion, then multiply $p$ by the inverse left Jacobian.

## Adjoint Representation

The adjoint representation of SE(3) is a $6 \times 6$ matrix that describes
how twists transform under a change of reference frame
[1, Def. 3.20, p. 98].

### Derivation

For $T = (R, p) \in \text{SE}(3)$ and a twist
$\mathcal{V} = (\omega, v)$, the adjoint action is defined as conjugation at
the algebra level:

$$
[\text{Ad}_T \mathcal{V}]^\wedge = T [\mathcal{V}]^\wedge T^{-1}
$$

Computing the right-hand side:

$$
T [\mathcal{V}]^\wedge T^{-1} = \begin{bmatrix} R & p \\ 0 & 1 \end{bmatrix} \begin{bmatrix} [\omega]_\times & v \\ 0 & 0 \end{bmatrix} \begin{bmatrix} R^\top & -R^\top p \\ 0 & 1 \end{bmatrix}
$$

Multiplying the first two matrices:

$$
= \begin{bmatrix} R[\omega]_\times & Rv \\ 0 & 0 \end{bmatrix} \begin{bmatrix} R^\top & -R^\top p \\ 0 & 1 \end{bmatrix}
$$

$$
= \begin{bmatrix} R[\omega]_\times R^\top & -R[\omega]_\times R^\top p + Rv \\ 0 & 0 \end{bmatrix}
$$

Using the identity $R[\omega]_\times R^\top = [R\omega]_\times$:

$$
= \begin{bmatrix} [R\omega]_\times & -[R\omega]_\times p + Rv \\ 0 & 0 \end{bmatrix} = \begin{bmatrix} [R\omega]_\times & [p]_\times R\omega + Rv \\ 0 & 0 \end{bmatrix}
$$

where in the last step we used $-[R\omega]_\times p = [p]_\times R\omega$
(the antisymmetry of the cross product: $a \times b = -b \times a$).

Extracting the twist vector via the vee operator:

$$
\text{Ad}_T \mathcal{V} = \begin{pmatrix} R\omega \\ [p]_\times R\omega + Rv \end{pmatrix} = \begin{bmatrix} R & 0 \\ [p]_\times R & R \end{bmatrix} \begin{pmatrix} \omega \\ v \end{pmatrix}
$$

Therefore the $6 \times 6$ adjoint matrix in omega-first ordering is:

$$
\boxed{[\text{Ad}_T] = \begin{bmatrix} R & 0 \\ [p]_\times R & R \end{bmatrix}}
$$

### Interpretation

The adjoint matrix transforms a twist from one frame to another:

$$
\mathcal{V}_s = [\text{Ad}_T] \, \mathcal{V}_b
$$

where $\mathcal{V}_b$ is the body-frame twist and $\mathcal{V}_s$ is the
space-frame twist.

- The **top-left block** $R$ rotates the angular velocity.
- The **bottom-left block** $[p]_\times R$ accounts for the "lever arm"
  coupling: angular velocity about a distant point induces linear velocity at
  the origin, proportional to the displacement $p$.
- The **bottom-right block** $R$ rotates the linear velocity.
- The **top-right block** is zero: linear velocity does not induce angular
  velocity (there is no "inverse lever arm" effect).

### Adjoint of the Inverse

The adjoint of $T^{-1}$ is:

$$
[\text{Ad}_{T^{-1}}] = \begin{bmatrix} R^\top & 0 \\ -R^\top [p]_\times & R^\top \end{bmatrix}
$$

This is the inverse of $[\text{Ad}_T]$: $[\text{Ad}_T]^{-1} = [\text{Ad}_{T^{-1}}]$.

### Barfoot Convention Comparison

Barfoot [2] uses translation-first (v-first) twist ordering, so his adjoint
has the blocks permuted:

$$
[\text{Ad}_T]_{\text{Barfoot}} = \begin{bmatrix} R & [p]_\times R \\ 0 & R \end{bmatrix}
$$

See [Notation Conventions](notation.md#twist-ordering-convention) for
conversion details.

## Product of Exponentials

The SE(3) exponential map underlies the **Product of Exponentials (PoE)**
formula for forward kinematics [1, Ch. 4]. For an $n$-joint serial chain with
screw axes $\mathcal{S}_1, \ldots, \mathcal{S}_n$ in the space frame and
joint angles $\theta_1, \ldots, \theta_n$:

$$
T(\theta) = e^{[\mathcal{S}_1]\theta_1} \, e^{[\mathcal{S}_2]\theta_2} \cdots e^{[\mathcal{S}_n]\theta_n} \, M
$$

where $M$ is the end-effector pose at the home (zero-angle) configuration.
Each factor $e^{[\mathcal{S}_i]\theta_i}$ is computed via the SE(3) exponential
map derived above.

This formula is the basis for Cartan's `forward_kinematics()`, `space_jacobian()`,
and `body_jacobian()` implementations. See the
[Kinematics background page](../background/) and
[API Reference](../api/kinematics.md) for details.

## Cartan Mapping

| Math | Cartan C++ | Notes |
|------|-----------|-------|
| $T \in \text{SE}(3)$ | `cartan::se3<Scalar>` | Internal: `so3` + `vector3` |
| $4 \times 4$ matrix | `se3<Scalar>::matrix()` | Returns `matrix4<Scalar>` |
| $\exp(\mathcal{V})$ | `se3<Scalar>::exp(V)` | `vector6<Scalar>` omega-first |
| $\log(T)$ | `se3<Scalar>::log()` | Returns `vector6<Scalar>` |
| $T_1 T_2$ | `t1 * t2` | `operator*` composes transforms |
| $T^{-1}$ | `t.inverse()` | Uses $R^\top$ and $-R^\top p$ |
| $I$ | `se3<Scalar>::identity()` | |
| $[\text{Ad}_T]$ | `t.adjoint()` | Returns `matrix6<Scalar>` (6x6) |
| $[\text{Ad}_T]^{-\top}$ | `t.coadjoint()` | Via `inverse().adjoint().transpose()` |
| $T \cdot q$ | `t.act(q)` | Transforms `vector3<Scalar>` |
| Hat map ($\mathbb{R}^6 \to 4\times4$) | `cartan::hat(V)` | `vector6` $\to$ `matrix4` |
| Vee map ($4\times4 \to \mathbb{R}^6$) | `cartan::vee(M)` | `matrix4` $\to$ `vector6` |
| Screw params | `to_screw_params(omega, v)` | Returns `screw_params<Scalar>` |

See [API Reference](../api/lie.md#se3) for complete method signatures and edge
case behavior.

## Bibliography

[1] K. M. Lynch and F. C. Park, *Modern Robotics: Mechanics, Planning, and
Control*, Cambridge University Press, 2017.

[2] T. D. Barfoot, *State Estimation for Robotics*, 2nd ed., Cambridge
University Press, 2024.

[3] B. Siciliano, L. Sciavicco, L. Villani, and G. Oriolo, *Foundations of
Robotics*, Springer, 2025.
