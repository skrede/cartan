# SO(2) and Planar Rotations

The special orthogonal group SO(2) is the group of all rotations in the
two-dimensional plane. It is the simplest Lie group encountered in robotics,
forming the rotational component of planar rigid body motions. Understanding
SO(2) provides a gentle introduction to the Lie group machinery (exponential
and logarithmic maps, Lie algebras, adjoint representations) that generalizes
directly to SO(3) and SE(3) [1, Ch. 3, pp. 68--69].

## Rotation Matrices in 2D

A rotation in the plane is described by a $2 \times 2$ orthogonal matrix with
determinant $+1$. The set of all such matrices forms the special orthogonal
group [1, Ch. 3, p. 68]:

$$
\text{SO}(2) = \{ R \in \mathbb{R}^{2 \times 2} : R^\top R = I, \; \det(R) = 1 \}
$$

SO(2) is a **1-dimensional** manifold (a circle) embedded in
$\mathbb{R}^{2 \times 2}$. Despite having 4 matrix entries, the orthogonality
constraints $R^\top R = I$ (3 independent equations for a symmetric matrix)
combined with $\det(R) = 1$ leave exactly 1 degree of freedom.

### Parameterization by Angle

Every element of SO(2) can be written in terms of a single angle
$\theta \in (-\pi, \pi]$ [1, Eq. 3.10, p. 68]:

$$
R(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}
$$

One can verify directly that $R(\theta)^\top R(\theta) = I$ and
$\det(R(\theta)) = \cos^2\theta + \sin^2\theta = 1$.

### Group Operations

SO(2) is a group under matrix multiplication:

- **Composition:** $R(\alpha) R(\beta) = R(\alpha + \beta)$. Rotations compose
  by adding angles (equivalently, multiplying the matrices).
- **Identity:** $R(0) = I$.
- **Inverse:** $R(\theta)^{-1} = R(\theta)^\top = R(-\theta)$. For any
  orthogonal matrix, the inverse equals the transpose.

## The Lie Algebra so(2)

The Lie algebra $\mathfrak{so}(2)$ is the tangent space to SO(2) at the
identity element. It consists of all $2 \times 2$ skew-symmetric matrices
[1, Ch. 3, p. 69]:

$$
\mathfrak{so}(2) = \{ [\theta]_\times : \theta \in \mathbb{R} \}
$$

where the **hat map** (wedge operator) sends a scalar to its skew-symmetric
matrix:

$$
[\theta]_\times = \theta \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix} = \begin{bmatrix} 0 & -\theta \\ \theta & 0 \end{bmatrix}
$$

The inverse **vee operator** recovers $\theta$ from $[\theta]_\times$ by
extracting the $(2,1)$ entry.

Geometrically, $\mathfrak{so}(2)$ is simply $\mathbb{R}$ (the real line), and
the hat map embeds it into the space of $2 \times 2$ matrices. The Lie bracket
is trivial: $[[\alpha]_\times, [\beta]_\times] = 0$ for all $\alpha, \beta$,
reflecting that SO(2) is an abelian (commutative) group.

## Exponential Map

The exponential map takes an element of the Lie algebra to the Lie group.
For SO(2), it maps an angular velocity (or angle) to a rotation matrix.

### Derivation via Matrix Exponential

The matrix exponential is defined by the Taylor series [2, Ch. 7, p. 213]:

$$
\exp([\theta]_\times) = \sum_{k=0}^{\infty} \frac{1}{k!} [\theta]_\times^k
$$

Define $A = [\theta]_\times = \theta \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}$. Computing powers of $A$:

$$
A^0 = I, \qquad A^1 = \theta \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}
$$

$$
A^2 = \theta^2 \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}^2 = \theta^2 \begin{bmatrix} -1 & 0 \\ 0 & -1 \end{bmatrix} = -\theta^2 I
$$

$$
A^3 = A^2 \cdot A = -\theta^2 A = -\theta^3 \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}
$$

$$
A^4 = A^2 \cdot A^2 = \theta^4 I
$$

The pattern repeats with period 4, identical to the powers of $i$ in complex
arithmetic. Grouping even and odd powers:

$$
\exp([\theta]_\times) = \left(\sum_{k=0}^{\infty} \frac{(-1)^k \theta^{2k}}{(2k)!}\right) I + \left(\sum_{k=0}^{\infty} \frac{(-1)^k \theta^{2k+1}}{(2k+1)!}\right) \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}
$$

Recognizing the Taylor series for cosine and sine:

$$
\exp([\theta]_\times) = \cos\theta \cdot I + \sin\theta \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix} = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix} = R(\theta)
$$

Thus the exponential map for SO(2) is:

$$
\boxed{\exp([\theta]_\times) = R(\theta)}
$$

This confirms that the angle $\theta$ parameterizing the Lie algebra directly
gives the rotation angle in the group.

## Logarithmic Map

The logarithmic map is the inverse of the exponential map, taking a rotation
back to the Lie algebra. Given $R \in \text{SO}(2)$ with entries
$R = \begin{bmatrix} c & -s \\ s & c \end{bmatrix}$ where $c = \cos\theta$,
$s = \sin\theta$:

$$
\log(R) = \text{atan2}(s, c) = \text{atan2}(r_{21}, r_{11})
$$

This returns $\theta \in (-\pi, \pi]$. The function $\text{atan2}$ is used
rather than $\arctan$ to correctly handle all four quadrants.

**Singularities:** The logarithmic map for SO(2) has no singularities in the
interior of $(-\pi, \pi)$. At $\theta = \pm\pi$ the two angles identify (the
map wraps around), but atan2 handles this correctly by convention.

## Adjoint Representation

The adjoint representation describes how the Lie algebra transforms under
conjugation by group elements [1, Ch. 3]:

$$
\text{Ad}_R : \mathfrak{so}(2) \to \mathfrak{so}(2), \qquad \text{Ad}_R(\theta) = R [\theta]_\times R^{-1}
$$

For SO(2), since $[\theta]_\times = \theta J$ where
$J = \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}$ commutes with every
$R \in \text{SO}(2)$ (both are functions of $J$), we have:

$$
R [\theta]_\times R^{-1} = R \theta J R^{-1} = \theta R J R^{-1} = \theta J = [\theta]_\times
$$

Therefore **the adjoint representation of SO(2) is trivial:**
$\text{Ad}_R = 1$ (the identity on the 1-dimensional Lie algebra). This
reflects the commutativity of SO(2) -- conjugation by any rotation leaves
angular velocities unchanged.

## liepp Mapping

The following table maps the mathematical objects to their liepp C++
implementations:

| Math | liepp C++ | Notes |
|------|-----------|-------|
| $R \in \text{SO}(2)$ | `liepp::so2<Scalar>` | Internal: $(c, s)$ pair |
| $R(\theta)$ | `so2<Scalar>::matrix()` | Returns `matrix2<Scalar>` |
| $\exp(\theta)$ | `so2<Scalar>::exp(theta)` | Scalar $\to$ `so2` |
| $\log(R)$ | `so2<Scalar>::log()` | Returns `Scalar` |
| $R_1 R_2$ | `r1 * r2` | `operator*` composes rotations |
| $R^{-1}$ | `r.inverse()` | Negates sine component |
| $I$ | `so2<Scalar>::identity()` | $(1, 0)$ |
| $R v$ | `r.act(v)` | Rotates `vector2<Scalar>` |

See [API Reference](../api/lie.md#so2) for complete method signatures and edge
case behavior.

## Bibliography

[1] K. M. Lynch and F. C. Park, *Modern Robotics: Mechanics, Planning, and
Control*, Cambridge University Press, 2017.

[2] T. D. Barfoot, *State Estimation for Robotics*, 2nd ed., Cambridge
University Press, 2024.

[3] B. Siciliano, L. Sciavicco, L. Villani, and G. Oriolo, *Foundations of
Robotics*, Springer, 2025.
