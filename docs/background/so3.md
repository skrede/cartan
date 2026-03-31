# SO(3) and Rotations

The special orthogonal group SO(3) is the group of all rotations in
three-dimensional space. It forms a Lie group: a smooth manifold that is also
a group, meaning rotations can be composed and inverted smoothly. Understanding
SO(3) is essential for robotic manipulation, attitude estimation, and any
application involving 3D orientation [1, Ch. 3, pp. 68--86].

This page covers rotation representations, the Lie group structure, the
exponential and logarithmic maps (including a full derivation of Rodrigues'
formula), and the adjoint representation. All formulas match liepp's
implementation.

## Rotation Matrices

A rotation matrix $R \in \mathbb{R}^{3 \times 3}$ satisfies the orthogonality
constraints [1, Ch. 3, p. 68]:

$$
R^\top R = I, \qquad \det(R) = +1
$$

The set of all such matrices forms SO(3):

$$
\text{SO}(3) = \{ R \in \mathbb{R}^{3 \times 3} : R^\top R = I, \; \det(R) = 1 \}
$$

SO(3) is a **3-dimensional** manifold embedded in $\mathbb{R}^{3 \times 3}$.
It has 9 entries but only 3 degrees of freedom, constrained by 6 independent
orthogonality conditions (the upper triangle of $R^\top R = I$).

### Composition and Inverse

Rotations compose by matrix multiplication. If $R_1$ rotates frame $\{a\}$ to
$\{b\}$ and $R_2$ rotates frame $\{b\}$ to $\{c\}$, then $R_2 R_1$ rotates
frame $\{a\}$ to $\{c\}$ [1, Ch. 3, p. 70].

The inverse of a rotation is its transpose:

$$
R^{-1} = R^\top
$$

This follows immediately from the orthogonality constraint $R^\top R = I$.

## Axis-Angle Representation

Any rotation $R \in \text{SO}(3)$ (other than the identity) can be described by
a unit axis $\hat{\omega} \in \mathbb{R}^3$ with $\|\hat{\omega}\| = 1$ and an
angle $\theta \in [0, \pi]$. The rotation vector is the product
[1, Sec. 3.2.3, pp. 77--86]:

$$
\phi = \theta \hat{\omega} \in \mathbb{R}^3
$$

This is a **minimal (3-parameter) representation**. Its norm encodes the angle
($\theta = \|\phi\|$) and its direction encodes the axis
($\hat{\omega} = \phi / \|\phi\|$).

**Singularity at identity:** When $\theta = 0$, the axis direction
$\hat{\omega}$ is undefined (any axis gives the same identity rotation). This
singularity is unavoidable for any 3-parameter rotation representation -- a
topological consequence of SO(3) not being contractible. liepp handles the
$\theta \approx 0$ case via Taylor series in both the exponential and
logarithmic maps.

## The Lie Algebra so(3)

The Lie algebra of SO(3), denoted $\mathfrak{so}(3)$, is the tangent space at
the identity. It consists of all $3 \times 3$ skew-symmetric matrices
[1, Ch. 3, p. 75; 2, Sec. 7.2.1, pp. 223--225]:

$$
\mathfrak{so}(3) = \{ [\phi]_\times : \phi \in \mathbb{R}^3 \}
$$

### The Hat Map

The **hat** (or wedge) operator maps a 3-vector to its skew-symmetric matrix:

$$
[\phi]_\times = \begin{bmatrix} 0 & -\phi_3 & \phi_2 \\ \phi_3 & 0 & -\phi_1 \\ -\phi_2 & \phi_1 & 0 \end{bmatrix}
$$

A key property is that $[\phi]_\times \, v = \phi \times v$ (the cross
product) for any $v \in \mathbb{R}^3$.

### The Vee Operator

The inverse **vee** operator recovers the 3-vector from a skew-symmetric
matrix:

$$
[\phi]_\times^\vee = \phi
$$

### Relationship to Angular Velocity

If $R(t)$ is a time-varying rotation, then $\dot{R} R^{-1} = [\omega^s]_\times$
where $\omega^s$ is the angular velocity in the space (fixed) frame, and
$R^{-1} \dot{R} = [\omega^b]_\times$ where $\omega^b$ is the angular velocity
in the body frame [1, Ch. 3, p. 75].

## Exponential Map (Rodrigues' Formula)

The exponential map takes an element of the Lie algebra (a tangent vector)
to the Lie group (a rotation). For $\phi \in \mathbb{R}^3$ with
$\theta = \|\phi\|$ and $\hat{\omega} = \phi / \theta$
[1, Prop. 3.11, Eq. 3.51, p. 82]:

### Step-by-Step Derivation

We derive the closed-form expression from the matrix exponential Taylor
series. Define $\Phi = [\phi]_\times = \theta [\hat{\omega}]_\times$, so:

$$
\exp(\Phi) = \sum_{k=0}^{\infty} \frac{1}{k!} \Phi^k
$$

**Key identity for skew-symmetric matrices.** For a unit-axis skew-symmetric
matrix $\Omega = [\hat{\omega}]_\times$, we have [1, p. 82]:

$$
\Omega^3 = -\Omega
$$

This can be verified using the identity
$[\hat{\omega}]_\times^2 = \hat{\omega}\hat{\omega}^\top - I$ (valid when
$\|\hat{\omega}\| = 1$), which gives
$[\hat{\omega}]_\times^3 = [\hat{\omega}]_\times (\hat{\omega}\hat{\omega}^\top - I) = -[\hat{\omega}]_\times$.

From $\Omega^3 = -\Omega$, all higher powers reduce:

$$
\Omega^4 = -\Omega^2, \quad \Omega^5 = \Omega, \quad \Omega^6 = \Omega^2, \quad \ldots
$$

**Grouping the Taylor series.** Since $\Phi = \theta\Omega$:

$$
\exp(\theta\Omega) = I + \sum_{k=1}^{\infty} \frac{\theta^k}{k!} \Omega^k
$$

Separating odd and even powers:

$$
= I + \left(\theta - \frac{\theta^3}{3!} + \frac{\theta^5}{5!} - \cdots\right) \Omega + \left(\frac{\theta^2}{2!} - \frac{\theta^4}{4!} + \frac{\theta^6}{6!} - \cdots\right) \Omega^2
$$

Recognizing the Taylor series for $\sin\theta$ and $1 - \cos\theta$:

$$
\boxed{\exp([\hat{\omega}]_\times \theta) = I + \sin\theta \, [\hat{\omega}]_\times + (1 - \cos\theta) \, [\hat{\omega}]_\times^2}
$$

This is **Rodrigues' rotation formula**. It converts an axis-angle
representation directly to a rotation matrix without trigonometric
decomposition.

### General Form (Non-Unit Axis)

For an arbitrary rotation vector $\phi$ (not necessarily unit length) with
$\theta = \|\phi\|$, the formula becomes [2, Eq. 7.18, p. 229]:

$$
\exp([\phi]_\times) = I + \frac{\sin\theta}{\theta} [\phi]_\times + \frac{1 - \cos\theta}{\theta^2} [\phi]_\times^2
$$

This is the form implemented in liepp. The sinc-type coefficients
$\frac{\sin\theta}{\theta}$ and $\frac{1 - \cos\theta}{\theta^2}$ avoid the
need to extract the unit axis explicitly.

### Numerical Stability Near $\theta = 0$

When $\theta \approx 0$, the coefficients suffer from $0/0$ indeterminacy.
Taylor expansions provide numerically stable alternatives
[2, p. 229]:

$$
\frac{\sin\theta}{\theta} \approx 1 - \frac{\theta^2}{6} + \frac{\theta^4}{120}
$$

$$
\frac{1 - \cos\theta}{\theta^2} \approx \frac{1}{2} - \frac{\theta^2}{24} + \frac{\theta^4}{720}
$$

liepp switches to these Taylor branches when $\theta^2$ is below a
scalar-dependent threshold (`detail::epsilon_v<Scalar>`).

### Quaternion Form

liepp's SO(3) implementation uses unit quaternions internally. The exponential
map in quaternion form is [2, Eq. 7.20, p. 230]:

$$
\exp(\phi) = \begin{bmatrix} \cos(\theta/2) \\ \frac{\sin(\theta/2)}{\theta} \phi \end{bmatrix}
$$

with Taylor expansion $\frac{\sin(\theta/2)}{\theta} \approx \frac{1}{2} - \frac{\theta^2}{48}$ near $\theta = 0$.

## Logarithmic Map

The logarithmic map is the inverse of the exponential map, taking a rotation
back to the Lie algebra [1, p. 82; 2, Eq. 7.22, p. 231].

### Rotation Matrix Form

Given $R \in \text{SO}(3)$, extract the rotation angle:

$$
\theta = \arccos\left(\frac{\text{tr}(R) - 1}{2}\right)
$$

Then the rotation vector is:

$$
\phi = [\log(R)]^\vee = \frac{\theta}{2\sin\theta} (R - R^\top)^\vee
$$

**Singularities:**

1. **$\theta = 0$ (identity):** The axis is undefined; $R - R^\top = 0$.
   liepp returns $\phi = 0$ (the zero vector in the Lie algebra).

2. **$\theta = \pi$ (half-turn):** $\sin\theta = 0$, making the formula
   $\theta / (2\sin\theta)$ undefined. The axis must be recovered from the
   eigenvector of $R$ corresponding to eigenvalue $+1$. This is the most
   numerically delicate case.

### Quaternion Form (liepp Implementation)

liepp avoids the $\theta = \pi$ branch entirely by using the quaternion
logarithm [2, Eq. 7.22, p. 231]:

$$
\log(q) = \frac{2 \, \text{atan2}(\|q_{xyz}\|, q_w)}{\|q_{xyz}\|} \, q_{xyz}
$$

with Taylor expansion near $\|q_{xyz}\| \approx 0$ (i.e., $\theta \approx 0$):

$$
\frac{2 \, \text{atan2}(n, w)}{n} \approx \frac{2}{w} - \frac{2 n^2}{3 w^3}
$$

The quaternion is first canonicalized to the $q_w \geq 0$ hemisphere (since
$q$ and $-q$ represent the same rotation), and then the atan2-based formula
handles all angles $\theta \in [0, \pi]$ without branching. This is the
approach described in [2, Sec. 7.1.3, p. 284] and implemented in liepp's
`detail::so3_log_impl`.

## Adjoint Representation

The adjoint representation of SO(3) describes how Lie algebra elements
(angular velocities) transform under conjugation by group elements
[1, Ch. 3, p. 75]:

$$
\text{Ad}_R : \mathfrak{so}(3) \to \mathfrak{so}(3), \qquad \text{Ad}_R(\phi) = R \phi
$$

### Derivation

For $R \in \text{SO}(3)$ and $[\phi]_\times \in \mathfrak{so}(3)$, the
adjoint action on the algebra is:

$$
R \, [\phi]_\times \, R^{-1} = R \, [\phi]_\times \, R^\top = [R\phi]_\times
$$

This identity follows from the property
$R(a \times b) = (Ra) \times (Rb)$ for rotation matrices, which gives
$R [\phi]_\times R^\top v = R(\phi \times R^\top v) = (R\phi) \times v = [R\phi]_\times v$.

Therefore, in the vector representation:

$$
\text{Ad}_R(\phi) = R \phi
$$

**For SO(3), the adjoint representation is simply the rotation matrix itself
applied to tangent vectors.** The $3 \times 3$ adjoint matrix is just $R$.

This is used in:
- Transforming angular velocities between frames:
  $\omega^s = R \, \omega^b$
- Computing how Lie algebra elements change under rotation
- Building the SE(3) adjoint (which contains $R$ as a block)

## Left and Right Jacobians

The left Jacobian of SO(3) relates perturbations in the Lie algebra to
perturbations on the group. It appears in the first-order approximation
[2, Sec. 7.4, pp. 240--245]:

$$
\exp(\phi + \delta\phi) \approx \exp(J_l(\phi) \, \delta\phi) \, \exp(\phi)
$$

### Left Jacobian

$$
J_l(\phi) = \frac{\sin\theta}{\theta} I + \left(1 - \frac{\sin\theta}{\theta}\right) \hat{\omega}\hat{\omega}^\top + \frac{1 - \cos\theta}{\theta} [\hat{\omega}]_\times
$$

where $\theta = \|\phi\|$ and $\hat{\omega} = \phi/\theta$
[2, Eq. 7.82b, p. 298].

### Right Jacobian

The right Jacobian satisfies $J_r(\phi) = J_l(-\phi)$. It appears in the
alternative perturbation convention:

$$
\exp(\phi + \delta\phi) \approx \exp(\phi) \, \exp(J_r(\phi) \, \delta\phi)
$$

### Inverse Jacobians

The inverse left Jacobian is [2, Eq. 7.84, p. 299]:

$$
J_l^{-1}(\phi) = \frac{\theta}{2} \cot\frac{\theta}{2} \, I + \left(1 - \frac{\theta}{2}\cot\frac{\theta}{2}\right) \hat{\omega}\hat{\omega}^\top - \frac{\theta}{2} [\hat{\omega}]_\times
$$

These Jacobians appear in:
- SE(3) exponential and logarithmic maps (the translation component uses $J_l$)
- Pose graph optimization and SLAM
- Error-state Kalman filters (MEKF)
- Interpolation between rotations

## Unit Quaternions

liepp uses unit quaternions as the internal representation of SO(3) elements.
A unit quaternion $q = (q_w, q_x, q_y, q_z)$ with $\|q\| = 1$ provides a
compact, singularity-free representation. Quaternions form a **double cover**
of SO(3): both $q$ and $-q$ represent the same rotation
[2, Sec. 7.1.3, pp. 218--222].

### Quaternion to Rotation Matrix

The equivalent rotation matrix is [2, Eq. 7.14, p. 220]:

$$
R(q) = (q_w^2 - \|q_{xyz}\|^2) I + 2 q_{xyz} q_{xyz}^\top + 2 q_w [q_{xyz}]_\times
$$

### Hamilton Product

Rotation composition via quaternions uses the Hamilton product:

$$
p \otimes q = \begin{bmatrix} p_w q_w - p_{xyz} \cdot q_{xyz} \\ p_w q_{xyz} + q_w p_{xyz} + p_{xyz} \times q_{xyz} \end{bmatrix}
$$

The inverse quaternion is the conjugate: $q^{-1} = (q_w, -q_{xyz})$.

## liepp Mapping

| Math | liepp C++ | Notes |
|------|-----------|-------|
| $R \in \text{SO}(3)$ | `liepp::so3<Scalar>` | Internal: unit quaternion |
| $3 \times 3$ matrix | `so3<Scalar>::matrix()` | Returns `matrix3<Scalar>` |
| Quaternion | `so3<Scalar>::quaternion_ref()` | Returns `const quaternion<Scalar>&` |
| $\exp(\phi)$ | `so3<Scalar>::exp(phi)` | `vector3<Scalar>` $\to$ `so3` |
| $\log(R)$ | `so3<Scalar>::log()` | Returns `vector3<Scalar>` (rotation vector) |
| $R_1 R_2$ | `r1 * r2` | Hamilton product internally |
| $R^{-1}$ | `r.inverse()` | Quaternion conjugate |
| $I$ | `so3<Scalar>::identity()` | $q = (1, 0, 0, 0)$ |
| $\text{Ad}_R$ | `r.adjoint()` | Returns $R$ as `matrix3<Scalar>` |
| $J_l(\phi)$ | `so3<Scalar>::left_jacobian(phi)` | Static method |
| $J_r(\phi)$ | `so3<Scalar>::right_jacobian(phi)` | $= J_l(-\phi)$ |
| $J_l^{-1}(\phi)$ | `so3<Scalar>::left_jacobian_inv(phi)` | Static method |
| $R v$ | `r.act(v)` | Rotates `vector3<Scalar>` |
| Axis-angle | `to_axis_angle(r)` | Returns `axis_angle<Scalar>` |
| From axis-angle | `from_axis_angle(aa)` | Returns `so3<Scalar>` |

See [API Reference](../api/lie.md#so3) for complete method signatures and edge
case behavior.

## Bibliography

[1] K. M. Lynch and F. C. Park, *Modern Robotics: Mechanics, Planning, and
Control*, Cambridge University Press, 2017.

[2] T. D. Barfoot, *State Estimation for Robotics*, 2nd ed., Cambridge
University Press, 2024.

[3] B. Siciliano, L. Sciavicco, L. Villani, and G. Oriolo, *Foundations of
Robotics*, Springer, 2025.
