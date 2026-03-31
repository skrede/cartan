# SE(2) and Planar Rigid Motions

The special Euclidean group SE(2) is the group of all rigid body
transformations in the two-dimensional plane. Each element combines a rotation
(from SO(2)) with a translation, representing the full configuration space of a
planar rigid body. SE(2) arises naturally in mobile robotics, planar
manipulators, and 2D SLAM [1, Ch. 3, pp. 86--90].

This page covers the group structure, the Lie algebra with its twist
representation, the exponential and logarithmic maps, and the adjoint
representation.

## Homogeneous Transformation Matrices

A planar rigid body transformation is a pair $(R, p)$ where $R \in \text{SO}(2)$
is a rotation and $p \in \mathbb{R}^2$ is a translation. The set of all such
pairs forms SE(2) [1, Ch. 3, p. 86]:

$$
\text{SE}(2) = \{ (R, p) : R \in \text{SO}(2), \; p \in \mathbb{R}^2 \}
$$

This is a 3-dimensional Lie group (1 DOF rotation + 2 DOF translation).

### Matrix Representation

SE(2) elements are represented as $3 \times 3$ homogeneous transformation
matrices:

$$
T = \begin{bmatrix} R & p \\ 0 & 1 \end{bmatrix} = \begin{bmatrix} \cos\theta & -\sin\theta & p_x \\ \sin\theta & \cos\theta & p_y \\ 0 & 0 & 1 \end{bmatrix}
$$

### Group Operations

- **Composition:** $T_1 T_2 = \begin{bmatrix} R_1 R_2 & R_1 p_2 + p_1 \\ 0 & 1 \end{bmatrix}$. The rotation components compose multiplicatively, and the translation of $T_2$ is first rotated by $R_1$ before adding $p_1$.

- **Identity:** $T = \begin{bmatrix} I & 0 \\ 0 & 1 \end{bmatrix}$.

- **Inverse:** $T^{-1} = \begin{bmatrix} R^\top & -R^\top p \\ 0 & 1 \end{bmatrix}$.

### Action on Points

A transformation $T$ acts on a point $q \in \mathbb{R}^2$ by:

$$
T \cdot q = R q + p
$$

or in homogeneous coordinates: $T \begin{bmatrix} q \\ 1 \end{bmatrix} = \begin{bmatrix} Rq + p \\ 1 \end{bmatrix}$.

## The Lie Algebra se(2)

The Lie algebra $\mathfrak{se}(2)$ is the tangent space to SE(2) at the
identity. Its elements are $3 \times 3$ matrices of the form
[1, Ch. 3, p. 87]:

$$
[\xi]^\wedge = \begin{bmatrix} [\omega]_\times & v \\ 0 & 0 \end{bmatrix} = \begin{bmatrix} 0 & -\omega & v_x \\ \omega & 0 & v_y \\ 0 & 0 & 0 \end{bmatrix}
$$

where $\omega \in \mathbb{R}$ is the angular component and
$v = (v_x, v_y) \in \mathbb{R}^2$ is the linear velocity component.

### Twist Representation

The Lie algebra element is compactly represented as a 3-vector called a
**twist**. liepp uses **omega-first** ordering following Lynch and Park
[1, Ch. 3]:

$$
\xi = \begin{pmatrix} \omega \\ v_x \\ v_y \end{pmatrix}
$$

The **hat map** (wedge operator) $[\cdot]^\wedge$ sends the 3-vector to the
$3 \times 3$ matrix, and the **vee operator** $(\cdot)^\vee$ recovers the
vector.

> **Convention note:** liepp uses omega-first twist ordering
> $\xi = (\omega, v_x, v_y)$, consistent with Lynch and Park's convention
> where the angular component precedes the linear component.

## Exponential Map

The exponential map takes a twist $\xi \in \mathfrak{se}(2)$ to a
transformation $T \in \text{SE}(2)$.

### Derivation

Starting from the matrix exponential $T = \exp([\xi]^\wedge)$, we separate the
rotation and translation components [1, Eq. 3.77--3.78, p. 88].

**Case 1: $\omega \neq 0$ (rotation present)**

The rotation component is simply $R = \exp([\omega]_\times)$, which gives
$R(\omega)$ as derived in the [SO(2) page](so2.md).

For the translation, we need to evaluate:

$$
p = \left(\int_0^1 e^{[\omega]_\times s} \, ds\right) v = G(\omega) \, v
$$

Computing the integral by expanding the matrix exponential:

$$
G(\omega) = \int_0^1 \begin{bmatrix} \cos(\omega s) & -\sin(\omega s) \\ \sin(\omega s) & \cos(\omega s) \end{bmatrix} ds = \frac{1}{\omega} \begin{bmatrix} \sin\omega & -(1 - \cos\omega) \\ 1 - \cos\omega & \sin\omega \end{bmatrix}
$$

Equivalently, in coefficient form:

$$
G(\omega) = \begin{bmatrix} \frac{\sin\omega}{\omega} & -\frac{1 - \cos\omega}{\omega} \\ \frac{1 - \cos\omega}{\omega} & \frac{\sin\omega}{\omega} \end{bmatrix}
$$

The translation is then:

$$
p = G(\omega) v = \begin{pmatrix} \frac{\sin\omega}{\omega} v_x - \frac{1 - \cos\omega}{\omega} v_y \\ \frac{1 - \cos\omega}{\omega} v_x + \frac{\sin\omega}{\omega} v_y \end{pmatrix}
$$

**Case 2: $\omega = 0$ (pure translation)**

When $\omega \to 0$, using Taylor expansions $\frac{\sin\omega}{\omega} \to 1$
and $\frac{1 - \cos\omega}{\omega} \to 0$:

$$
G(0) = I, \qquad p = v
$$

This is the pure translation case. liepp handles this branch explicitly to
avoid division by zero.

### Summary

$$
\boxed{\exp\left(\begin{pmatrix} \omega \\ v_x \\ v_y \end{pmatrix}^\wedge\right) = \begin{bmatrix} R(\omega) & G(\omega) v \\ 0 & 1 \end{bmatrix}}
$$

## Logarithmic Map

The logarithmic map is the inverse of the exponential, taking a transformation
$T = (R, p)$ back to a twist $\xi$.

### Derivation

**Step 1:** Extract the angular component via the SO(2) logarithmic map:

$$
\omega = \text{atan2}(\sin\theta, \cos\theta)
$$

**Step 2:** Recover the linear velocity by inverting $p = G(\omega) v$, i.e.,
$v = G(\omega)^{-1} p$.

**Case $\omega \neq 0$:** The matrix $G(\omega)$ is invertible. Its inverse
can be computed in closed form using the half-angle identity
[1, pp. 89--90]:

$$
G(\omega)^{-1} = \begin{bmatrix} \frac{\omega}{2} \cot\frac{\omega}{2} & \frac{\omega}{2} \\ -\frac{\omega}{2} & \frac{\omega}{2} \cot\frac{\omega}{2} \end{bmatrix}
$$

Therefore:

$$
v = G(\omega)^{-1} p = \begin{pmatrix} \frac{\omega}{2}\cot\frac{\omega}{2} \, p_x + \frac{\omega}{2} \, p_y \\ -\frac{\omega}{2} \, p_x + \frac{\omega}{2}\cot\frac{\omega}{2} \, p_y \end{pmatrix}
$$

**Case $\omega = 0$:** $G(0)^{-1} = I$, so $v = p$.

### Summary

$$
\boxed{\log(T) = \begin{pmatrix} \omega \\ G(\omega)^{-1} p \end{pmatrix}}
$$

## Adjoint Representation

The adjoint representation of SE(2) describes how twists transform under
a change of reference frame. For $T = (R, p) \in \text{SE}(2)$ and a twist
$\xi = (\omega, v_x, v_y)$, the adjoint action is
[1, Def. 3.20, adapted for 2D]:

$$
\text{Ad}_T \xi = T \xi T^{-1}
$$

evaluated at the Lie algebra level. Working this out for SE(2) yields the
$3 \times 3$ adjoint matrix:

### Derivation

Let $T = \begin{bmatrix} R & p \\ 0 & 1 \end{bmatrix}$ and
$[\xi]^\wedge = \begin{bmatrix} [\omega]_\times & v \\ 0 & 0 \end{bmatrix}$.

Conjugation gives:

$$
T [\xi]^\wedge T^{-1} = \begin{bmatrix} R [\omega]_\times R^\top & R v + \omega \begin{bmatrix} -p_y \\ p_x \end{bmatrix} \\ 0 & 0 \end{bmatrix}
$$

Since SO(2) is abelian, $R[\omega]_\times R^\top = [\omega]_\times$, and the
angular component is unchanged. Extracting the twist vector using the vee
operator, the adjoint matrix in omega-first ordering is:

$$
\boxed{[\text{Ad}_T] = \begin{bmatrix} 1 & 0 & 0 \\ p_y & \cos\theta & -\sin\theta \\ -p_x & \sin\theta & \cos\theta \end{bmatrix}}
$$

This maps a twist expressed in one frame to the equivalent twist in another
frame. The top row reflects that the angular velocity is frame-invariant for
planar motions. The bottom $2 \times 2$ block is the rotation $R$, and the
$p_y, -p_x$ terms couple the angular velocity into a linear velocity shift
(the "lever arm" effect).

## liepp Mapping

| Math | liepp C++ | Notes |
|------|-----------|-------|
| $T \in \text{SE}(2)$ | `liepp::se2<Scalar>` | Internal: `so2` + `vector2` |
| $3 \times 3$ matrix | `se2<Scalar>::matrix()` | Returns `Eigen::Matrix<Scalar,3,3>` |
| $\exp(\xi)$ | `se2<Scalar>::exp(v)` | `vector3<Scalar>` $(\omega, v_x, v_y)$ |
| $\log(T)$ | `se2<Scalar>::log()` | Returns `vector3<Scalar>` |
| $T_1 T_2$ | `t1 * t2` | `operator*` composes transforms |
| $T^{-1}$ | `t.inverse()` | |
| $I$ | `se2<Scalar>::identity()` | |
| $\text{Ad}_T$ | `t.adjoint()` | Returns `Eigen::Matrix<Scalar,3,3>` |
| $T \cdot q$ | `t.act(q)` | Transforms `vector2<Scalar>` |

See [API Reference](../api/lie.md#se2) for complete method signatures and edge
case behavior.

## Bibliography

[1] K. M. Lynch and F. C. Park, *Modern Robotics: Mechanics, Planning, and
Control*, Cambridge University Press, 2017.

[2] T. D. Barfoot, *State Estimation for Robotics*, 2nd ed., Cambridge
University Press, 2024.

[3] B. Siciliano, L. Sciavicco, L. Villani, and G. Oriolo, *Foundations of
Robotics*, Springer, 2025.
