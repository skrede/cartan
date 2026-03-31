# Notation Conventions

liepp follows the notation and conventions of Lynch and Park's *Modern
Robotics* [1] as its primary reference. Siciliano et al.'s *Foundations of
Robotics* [3] serves as a secondary reference for classical robotics concepts,
and Barfoot's *State Estimation for Robotics* [2] as a tertiary reference for
Lie group operations and uncertainty.

This page maps the notation used in all three textbooks to liepp's C++ API,
and documents the key convention choices that affect formula translation between
references.

## Notation Table

The following table maps mathematical concepts across the three reference
textbooks and to their liepp C++ implementations.

### Lie Groups and Basic Operations

| Concept | Lynch & Park [1] | Siciliano et al. [3] | Barfoot [2] | liepp C++ |
|---------|-----------------|---------------------|-------------|-----------|
| 2D rotation group | $\text{SO}(2)$ | $\text{SO}(2)$ | $\text{SO}(2)$ | `liepp::so2<Scalar>` |
| 3D rotation group | $\text{SO}(3)$ | $\text{SO}(3)$ | $\text{SO}(3)$ | `liepp::so3<Scalar>` |
| 2D rigid motion group | $\text{SE}(2)$ | $\text{SE}(2)$ | $\text{SE}(2)$ | `liepp::se2<Scalar>` |
| 3D rigid motion group | $\text{SE}(3)$ | $\text{SE}(3)$ | $\text{SE}(3)$ | `liepp::se3<Scalar>` |
| Rotation matrix | $R$ | $R$ | $\mathbf{C}$ | `so3<Scalar>::matrix()` |
| Homogeneous transform | $T$ | $T$ | $\mathbf{T}$ | `se3<Scalar>::matrix()` |
| Identity element | $I$ | $I$ | $\mathbf{1}$ | `::identity()` |
| Group composition | $R_1 R_2$ / $T_1 T_2$ | $R_1 R_2$ / $T_1 T_2$ | $\mathbf{C}_1 \mathbf{C}_2$ | `operator*` |
| Group inverse | $R^{-1} = R^\top$ | $R^{-1}$ | $\mathbf{C}^{-1}$ | `.inverse()` |

### Rotation Representations

| Concept | Lynch & Park [1] | Siciliano et al. [3] | Barfoot [2] | liepp C++ |
|---------|-----------------|---------------------|-------------|-----------|
| Rotation vector | $\hat{\omega}\theta$ | $\phi$ (angle-axis) | $\boldsymbol{\phi}$ | `vector3<Scalar>` (to `exp`) |
| Rotation angle | $\theta$ | $\theta$ | $\phi = \|\boldsymbol{\phi}\|$ | `axis_angle<Scalar>::angle` |
| Rotation axis | $\hat{\omega}$ | $\hat{r}$ (unit vector) | $\mathbf{a} = \boldsymbol{\phi}/\phi$ | `axis_angle<Scalar>::axis` |
| Unit quaternion | -- (implicit) | -- | $\mathbf{q} = (q_w, \mathbf{q}_{xyz})$ | `so3<Scalar>::quaternion_ref()` |

### Lie Algebra and Maps

| Concept | Lynch & Park [1] | Siciliano et al. [3] | Barfoot [2] | liepp C++ |
|---------|-----------------|---------------------|-------------|-----------|
| so(3) element | $[\omega]_\times \in \mathfrak{so}(3)$ | $S(\omega)$ (skew) | $\boldsymbol{\phi}^\wedge$ | `hat(omega)` |
| Hat (wedge) operator | $[\cdot]$ or $[\cdot]_\times$ | $S(\cdot)$ | $(\cdot)^\wedge$ | `liepp::hat()` |
| Vee operator | $[\cdot]^\vee$ | -- | $(\cdot)^\vee$ | `liepp::vee()` |
| Exponential map (SO3) | $e^{[\hat{\omega}]\theta}$ | $e^{S(\omega)\theta}$ | $\exp(\boldsymbol{\phi}^\wedge)$ | `so3<Scalar>::exp(phi)` |
| Logarithmic map (SO3) | $\log R$ | -- | $\ln(\mathbf{C})^\vee$ | `so3<Scalar>::log()` |
| Exponential map (SE3) | $e^{[\mathcal{S}]\theta}$ | -- | $\exp(\boldsymbol{\xi}^\wedge)$ | `se3<Scalar>::exp(V)` |
| Logarithmic map (SE3) | $\log T$ | -- | $\ln(\mathbf{T})^\vee$ | `se3<Scalar>::log()` |

### Twists, Screws, and Velocities

| Concept | Lynch & Park [1] | Siciliano et al. [3] | Barfoot [2] | liepp C++ |
|---------|-----------------|---------------------|-------------|-----------|
| Spatial twist | $\mathcal{V} = (\omega, v)$ | -- | $\boldsymbol{\varpi}^s$ | `vector6<Scalar>` omega-first |
| Body twist | $\mathcal{V}_b = (\omega_b, v_b)$ | -- | $\boldsymbol{\varpi}^b$ | `vector6<Scalar>` omega-first |
| Screw axis | $\mathcal{S} = (\omega, v)$, $\|\omega\|=1$ | -- | -- | `screw_axis<Scalar>` |
| Screw angle/distance | $\theta$ (revolute) / $d$ (prismatic) | -- | -- | Joint variable `q` |
| Angular velocity | $\omega$ | $\omega$ | $\boldsymbol{\omega}$ | Top 3 of twist vector |
| Linear velocity | $v$ | $\dot{p}$ | $\boldsymbol{\nu}$ | Bottom 3 of twist vector |

### Kinematics

| Concept | Lynch & Park [1] | Siciliano et al. [3] | Barfoot [2] | liepp C++ |
|---------|-----------------|---------------------|-------------|-----------|
| Forward kinematics | $T(\theta) = e^{[\mathcal{S}_1]\theta_1} \cdots e^{[\mathcal{S}_n]\theta_n} M$ | $T(q)$ | $\mathbf{T}(q)$ | `forward_kinematics(chain, q)` |
| Home configuration | $M$ | -- | -- | `chain<...>::home_transform()` |
| Space Jacobian | $J_s(\theta)$ | $J$ (geometric) | $\mathbf{J}$ | `space_jacobian(chain, q)` |
| Body Jacobian | $J_b(\theta)$ | -- | -- | `body_jacobian(chain, q)` |
| Adjoint (SE3) | $[\text{Ad}_T]$ (6x6) | -- | $\mathcal{T}$ or $\text{Ad}(\mathbf{T})$ | `se3<Scalar>::adjoint()` |
| Adjoint (SO3) | $R$ (3x3, acts on $\omega$) | -- | $\mathbf{C}$ | `so3<Scalar>::adjoint()` |
| Left Jacobian (SO3) | -- | -- | $\mathbf{J}$ | `so3<Scalar>::left_jacobian(phi)` |
| Right Jacobian (SO3) | -- | -- | $\mathbf{J}(-\boldsymbol{\phi})$ | `so3<Scalar>::right_jacobian(phi)` |

### Inverse Kinematics

| Concept | Lynch & Park [1] | Siciliano et al. [3] | Barfoot [2] | liepp C++ |
|---------|-----------------|---------------------|-------------|-----------|
| IK error twist | $\mathcal{V}_b = \log(T_{sd}^{-1} T_{sb})$ | $e = x_d - x$ | -- | Body-frame log error |
| Damped least squares | -- | DLS / Levenberg | -- | `dls_stepper<DOF, Scalar>` |
| Levenberg-Marquardt | -- | LM | -- | `lm_stepper<DOF, Scalar>` |
| Null-space projection | $(I - J^\dagger J) z$ | $(I - J^\dagger J) q_0$ | -- | Null-space API |

## Twist Ordering Convention

liepp uses **omega-first** twist ordering following Lynch and Park [1]:

$$
\xi = \begin{pmatrix} \omega \\ v \end{pmatrix} \in \mathbb{R}^6
$$

where $\omega \in \mathbb{R}^3$ is the angular velocity and
$v \in \mathbb{R}^3$ is the linear velocity.

**Barfoot** [2] uses the opposite ordering:

$$
\boldsymbol{\xi} = \begin{pmatrix} \boldsymbol{\rho} \\ \boldsymbol{\phi} \end{pmatrix}
$$

where $\boldsymbol{\rho}$ (translational) comes first and
$\boldsymbol{\phi}$ (rotational) comes second.

**When adapting formulas from Barfoot to liepp:** swap the top-3 and bottom-3
components of any 6-vector, and correspondingly rearrange the rows and columns
of any $6 \times 6$ matrix (adjoint, Jacobians, covariance).

Concretely, if Barfoot's adjoint is:

$$
\text{Ad}_T^{\text{Barfoot}} = \begin{bmatrix} \mathbf{C} & [\mathbf{r}]_\times \mathbf{C} \\ \mathbf{0} & \mathbf{C} \end{bmatrix}
$$

then liepp's adjoint (omega-first) is:

$$
[\text{Ad}_T]^{\text{liepp}} = \begin{bmatrix} R & 0 \\ [p]_\times R & R \end{bmatrix}
$$

where $R = \mathbf{C}$ and $p = \mathbf{r}$, with rows and columns permuted
to place the angular block in the top-left.

## Frame Conventions

liepp uses the **space-frame Product of Exponentials (PoE)** formulation from
Lynch and Park [1, Ch. 4]:

$$
T(\theta) = e^{[\mathcal{S}_1]\theta_1} \, e^{[\mathcal{S}_2]\theta_2} \cdots e^{[\mathcal{S}_n]\theta_n} \, M
$$

where:
- $\mathcal{S}_i$ are screw axes expressed in the **space (fixed) frame**
- $M$ is the home (zero-angle) configuration of the end effector
- Exponentials multiply from left to right in joint order

**Siciliano et al.** [3] use the geometric Jacobian $J$ which relates joint
velocities to the end-effector twist. Their Jacobian corresponds to liepp's
**space Jacobian** $J_s$ when both are expressed in the same frame, but the
frame attachment conventions may differ. Siciliano defines the Jacobian columns
from DH parameters rather than PoE screw axes, so numerical values may need
conversion.

**Barfoot** [2] uses a similar PoE-like formulation but with the opposite twist
ordering (see above). When converting kinematic expressions, both the twist
vectors and the Jacobian columns must be reordered.

## Rodrigues' Formula Variants

The exponential map for SO(3) appears in slightly different forms across
textbooks. All are mathematically equivalent:

**Lynch and Park** [1, Eq. 3.51]:

$$
e^{[\hat{\omega}]\theta} = I + \sin\theta \, [\hat{\omega}]_\times + (1 - \cos\theta) \, [\hat{\omega}]_\times^2
$$

where $\hat{\omega}$ is the **unit** axis.

**Barfoot** [2, Eq. 7.18]:

$$
\exp(\boldsymbol{\phi}^\wedge) = I + \frac{\sin\theta}{\theta} \boldsymbol{\phi}^\wedge + \frac{1 - \cos\theta}{\theta^2} (\boldsymbol{\phi}^\wedge)^2
$$

where $\theta = \|\boldsymbol{\phi}\|$ and $\boldsymbol{\phi}$ is **not**
necessarily unit length. This form uses sinc-type coefficients and is the form
implemented in liepp (it handles arbitrary-magnitude rotation vectors directly).

## Bibliography

[1] K. M. Lynch and F. C. Park, *Modern Robotics: Mechanics, Planning, and
Control*, Cambridge University Press, 2017.

[2] T. D. Barfoot, *State Estimation for Robotics*, 2nd ed., Cambridge
University Press, 2024.

[3] B. Siciliano, L. Sciavicco, L. Villani, and G. Oriolo, *Foundations of
Robotics*, Springer, 2025.
