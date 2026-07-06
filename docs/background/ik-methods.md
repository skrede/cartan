# Inverse Kinematics Methods

**Inverse kinematics (IK)** is the problem of finding joint angles
$\theta \in \mathbb{R}^n$ such that the forward kinematics maps to a desired
end-effector pose $T_d \in SE(3)$. Formally, given the PoE forward kinematics
$T(\theta) = e^{[\mathcal{S}_1]\theta_1} \cdots e^{[\mathcal{S}_n]\theta_n} M$,
find $\theta$ satisfying $T(\theta) = T_d$
[1, Ch. 6, pp. 227--240].

Unlike forward kinematics (which has a unique closed-form solution), inverse
kinematics may have zero, one, or infinitely many solutions, and generally
requires iterative numerical methods.

Cartan provides three IK steppers: Damped Least Squares (DLS),
Levenberg-Marquardt (LM), and Sequential Quadratic Programming (SQP). All
operate on the body-frame error twist and expose a cooperative `step()`
interface.

## The IK Problem

### Error Twist Formulation

Rather than solving $T(\theta) = T_d$ directly, IK is formulated as
minimizing an error twist. The **body-frame error twist** is
[1, Eq. 6.6, p. 228]:

$$
\xi_b = \log\!\left(T(\theta)^{-1} \, T_d\right) \in \mathbb{R}^6
$$

This is the se(3) logarithm of the relative transformation between the
current and target poses, expressed in the body frame. When
$\lVert \xi_b \rVert = 0$, the end-effector is at the target.

The error twist naturally separates into angular and linear components:

$$
\xi_b = \begin{bmatrix} \xi_\omega \\ \xi_v \end{bmatrix}, \qquad \xi_\omega \in \mathbb{R}^3, \; \xi_v \in \mathbb{R}^3
$$

where $\lVert \xi_\omega \rVert$ measures orientation error and
$\lVert \xi_v \rVert$ measures position error.

### Linearization

For small joint updates $\Delta\theta$, the first-order Taylor expansion of
the error twist gives [1, Eq. 6.8, p. 230]:

$$
\xi_b \approx J_b(\theta) \, \Delta\theta
$$

where $J_b(\theta)$ is the body Jacobian. This linearization is the foundation
for all Newton-type IK methods.

## Newton-Raphson IK

The simplest iterative approach solves the linearized system directly
[1, Sec. 6.2, p. 228]:

$$
\Delta\theta = J_b^\dagger \, \xi_b
$$

where $J_b^\dagger$ is the Moore-Penrose pseudoinverse (or the true inverse
when $J_b$ is square and full-rank). The joint update is applied as
$\theta \leftarrow \theta + \Delta\theta$.

**Properties:**
- **Quadratic convergence** near the solution (when the Jacobian is well-conditioned)
- **No damping:** Diverges or produces huge joint velocities near singularities
- **Not used directly in Cartan:** Both DLS and LM add damping to handle singularities

## Damped Least Squares (DLS)

### Derivation

The DLS method replaces the pseudoinverse with a regularized solution.
Instead of minimizing $\lVert J_b \Delta\theta - \xi_b \rVert^2$, DLS
minimizes [1, Eq. 6.10, p. 231]:

$$
\min_{\Delta\theta} \; \lVert J_b \Delta\theta - \xi_b \rVert^2 + \lambda^2 \lVert \Delta\theta \rVert^2
$$

The closed-form solution is:

$$
\Delta\theta = J_b^\top (J_b J_b^\top + \lambda^2 I)^{-1} \xi_b
$$

or equivalently:

$$
\Delta\theta = (J_b^\top J_b + \lambda^2 I)^{-1} J_b^\top \xi_b
$$

### Role of the Damping Parameter

The damping factor $\lambda$ controls the trade-off between convergence speed
and robustness near singularities:

- **$\lambda \to 0$:** Recovers the undamped pseudoinverse (fast but unstable near singularities)
- **$\lambda$ large:** Step approaches the gradient descent direction $J_b^\top \xi_b$ (slow but stable)

### SVD-Based Adaptive Damping (Nakamura)

Cartan's `dls_stepper` uses Nakamura's adaptive damping scheme. The body
Jacobian is decomposed via SVD: $J_b = U \Sigma V^\top$. The damping factor
adapts based on the smallest singular value $\sigma_{\min}$:

$$
\lambda^2 = \begin{cases}
\lambda_{\max}^2 \left(1 - \left(\frac{\sigma_{\min}}{\sigma_0}\right)^2 \right) & \text{if } \sigma_{\min} < \sigma_0 \\
0 & \text{otherwise}
\end{cases}
$$

where $\sigma_0$ is the singularity threshold. The damped pseudoinverse step
becomes:

$$
\Delta\theta = V \, \text{diag}\!\left(\frac{\sigma_i}{\sigma_i^2 + \lambda^2}\right) U^\top \xi_b
$$

This smoothly increases damping as the robot approaches a singularity, avoiding
discontinuities in joint velocities.

### Cartan Implementation

```
dls_stepper<N, Scalar>
```

- **Setup:** `setup(chain, target, q0, criteria)` -- initializes with chain, target pose, seed, and convergence criteria
- **Step:** `step(chain)` -- one Newton-Raphson iteration with adaptive DLS damping
- **Diagnostics:** `condition_number()`, `manipulability()` -- monitor proximity to singularities

## Levenberg-Marquardt (LM)

### Derivation

The Levenberg-Marquardt algorithm formulates IK as a nonlinear least-squares
problem: minimize $f(\theta) = \frac{1}{2} \lVert \xi_b(\theta) \rVert^2$.
The LM update solves:

$$
\Delta\theta = (J_b^\top J_b + \lambda I)^{-1} J_b^\top \xi_b
$$

Writing $H = J_b^\top J_b$ for the Gauss-Newton Hessian approximation and
$g = J_b^\top \xi_b$ for the gradient, each step solves the damped linear
system $(H + \lambda I)\,\Delta\theta = g$.

This interpolates between two regimes:

- **$\lambda \to 0$ (Gauss-Newton):** $\Delta\theta \approx (J_b^\top J_b)^{-1} J_b^\top \xi_b$ -- fast quadratic convergence near the solution
- **$\lambda \to \infty$ (gradient descent):** $\Delta\theta \approx \frac{1}{\lambda} \, J_b^\top \xi_b$ -- small, safe steps in the steepest descent direction

The damping term is the identity matrix $\lambda I$ (Levenberg's original
form), so the same $\lambda$ regularizes every direction uniformly, exactly
as in DLS. This keeps the damped normal-equation matrix symmetric positive
definite and solvable by a Cholesky ($LDL^\top$) factorization.

### Gain Ratio and Lambda Update

The quality of each step is evaluated via the **gain ratio** (Nielsen, 1999):

$$
\rho = \frac{f(\theta) - f(\theta + \Delta\theta)}{L(\mathbf{0}) - L(\Delta\theta)}
$$

where the numerator is the actual reduction in error and the denominator is
the predicted reduction from the linearized model
$L(\Delta\theta) = \frac{1}{2}\lVert J_b \Delta\theta - \xi_b \rVert^2 + \frac{\lambda}{2}\lVert \Delta\theta \rVert^2$.

The lambda update rule:

- **$\rho > 0$ (step accepted):** $\lambda \leftarrow \lambda \cdot \max\!\left(\tfrac{1}{3}, \, 1 - (2\rho - 1)^3\right)$, $\nu \leftarrow 2$
- **$\rho \leq 0$ (step rejected):** $\lambda \leftarrow \lambda \cdot \nu$, $\nu \leftarrow 2\nu$

This trust-region strategy automatically adapts the damping: good steps reduce
$\lambda$ (more aggressive), poor steps increase $\lambda$ (more conservative).

### Cartan Implementation

```
lm_stepper<N, Scalar>
```

- **Setup:** `setup(chain, target, q0, criteria)` -- initializes lambda from $\lambda_0 \cdot \max(\text{diag}(J_b^\top J_b))$ (Nielsen initialization)
- **Step:** `step(chain)` -- one LM iteration with gain ratio evaluation and lambda update
- **Diagnostic:** `lambda()` -- current damping parameter

## Sequential Quadratic Programming (SQP)

### Formulation

SQP formulates IK as a constrained optimization problem:

$$
\min_\theta \; \frac{1}{2} \lVert \xi_b(\theta) \rVert^2 \qquad \text{subject to} \qquad \theta_{\min} \leq \theta \leq \theta_{\max}
$$

where $\theta_{\min}, \theta_{\max}$ are the joint position limits. At each
iteration, SQP approximates the original problem as a quadratic program (QP):

$$
\min_{\Delta\theta} \; \frac{1}{2} \Delta\theta^\top H \, \Delta\theta + g^\top \Delta\theta \qquad \text{s.t.} \qquad \theta_{\min} - \theta \leq \Delta\theta \leq \theta_{\max} - \theta
$$

where $H \approx J_b^\top J_b$ (Gauss-Newton Hessian approximation) and
$g = -J_b^\top \xi_b$ (negative gradient).

### Cartan Implementation

```
sqp_stepper<N, Scalar>   // requires CARTAN_HAS_NLOPT
```

Cartan wraps NLopt's SLSQP algorithm. Joint limits from the `kinematic_chain`
are passed as box constraints. The stepper runs a configurable budget of NLopt
evaluations per `step()` call, enabling cooperative scheduling with other
solvers.

- **When to use:** When joint limits are critical (surgical robots, constrained workspaces)
- **Trade-off:** More expensive per iteration than DLS/LM, but respects hard constraints throughout

## Convergence Criteria

IK convergence is checked via separate angular and linear tolerances on the
body-frame error twist [1, Sec. 6.2, p. 228]:

$$
\lVert \xi_\omega \rVert < \epsilon_\omega \qquad \text{and} \qquad \lVert \xi_v \rVert < \epsilon_v
$$

Cartan uses the `convergence_criteria<Scalar>` struct:

| Parameter | Meaning | Default |
|-----------|---------|---------|
| `position_tol` | Linear error tolerance $\epsilon_v$ | $10^{-6}$ |
| `orientation_tol` | Angular error tolerance $\epsilon_\omega$ | $10^{-6}$ |
| `max_iterations` | Maximum iterations before `iteration_limit` | 100 |

Separating angular and linear tolerances is essential because they have
different physical units (radians vs meters) and different magnitudes in
practice. A combined norm $\lVert \xi_b \rVert < \epsilon$ conflates the two,
making it difficult to set appropriate tolerances.

Cartan's `epsilon_v<Scalar>` trait provides scalar-appropriate default
tolerances: `double` uses $10^{-6}$, `float` uses $10^{-4}$, reflecting
the difference in machine precision.

### Termination Conditions

All three steppers report status via `ik_status`:

| Status | Meaning |
|--------|---------|
| `converged` | Both angular and linear errors below tolerance |
| `iteration_limit` | Maximum iterations reached |
| `stalled` | Error not improving (within stall window) |
| `diverged` | Error exceeds divergence factor times initial error |
| `joint_limit_hit` | Joint limit violated (SQP box constraint) |

## Cartan's Tick-Based Architecture

Cartan's IK architecture is designed around **cooperative scheduling**. Each
stepper is a passive object with a `step()` method that performs exactly one
iteration. The scheduler (not the stepper) controls iteration allocation.

### The `ik_stepper` Concept

All steppers satisfy the `ik_stepper` concept:

```cpp
concept ik_stepper = requires(S& s, ...) {
    { s.setup(chain, target, q0, criteria) };
    { s.step(chain) } -> std::same_as<ik_status>;
    { s.converged() } -> std::convertible_to<bool>;
    { s.solution() };
    { s.error_norm() };
    { s.iterations() };
    { s.abort() };
};
```

### Cooperative Scheduling

The `racing_scheduler` runs multiple steppers cooperatively:

1. Each tick, the scheduler calls `step()` on each active stepper
2. The first stepper to converge wins -- its solution is returned
3. Steppers that stall or diverge are deactivated
4. The scheduler controls iteration budget via **tick policies**:
   - `round_robin_tick`: 1 step per solver per tick
   - `budget_tick`: batch steps based on total budget
   - `time_boxed_tick`: step until time budget exhausted

This architecture replaces TRAC-IK's thread-per-solver model. Instead of
spawning $O(N)$ threads, Cartan schedules $N$ steppers cooperatively on a
single thread. Multiple IK queries can run concurrently on separate threads
without internal thread proliferation.

See [API Reference](../api/ik.md) for solver and scheduler interfaces.
See [Jacobians](jacobians.md) for the body Jacobian used in all IK methods.

## Bibliography

[1] K. M. Lynch and F. C. Park, "Modern Robotics: Mechanics, Planning, and
Control," Cambridge University Press, 2017.

[2] Y. Nakamura, "Advanced Robotics: Redundancy and Optimization,"
Addison-Wesley, 1991.

[3] H. B. Nielsen, "Damping Parameter in Marquardt's Method," Technical
Report IMM-REP-1999-05, Technical University of Denmark, 1999.

[4] K. Levenberg, "A Method for the Solution of Certain Non-Linear Problems
in Least Squares," *Quarterly of Applied Mathematics*, vol. 2, no. 2,
pp. 164--168, 1944.

[5] D. W. Marquardt, "An Algorithm for Least-Squares Estimation of Nonlinear
Parameters," *Journal of the Society for Industrial and Applied Mathematics*,
vol. 11, no. 2, pp. 431--441, 1963.
