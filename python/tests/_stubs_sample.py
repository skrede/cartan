"""Exercise every public binding so type-checkers see the full surface.

This module is consumed by test_stubs.py, which runs pyright and mypy in
strict mode against it as a subprocess. It is also executable directly via
`python python/tests/_stubs_sample.py` as a lightweight sanity check.

The file-level `# pyright: strict` marker enables pyright's strict type
checking; recent pyright versions (>= 1.1.400) dropped the `--strict` CLI
flag in favor of file-level or config-level enablement. mypy is run with
its `--strict` CLI flag, which has been stable for many releases.
"""

# pyright: strict

from __future__ import annotations

import numpy as np

import cartan


# ---------------------------------------------------------------------------
# Lie group surface (SO3, SE3)
# ---------------------------------------------------------------------------

R: cartan.SO3 = cartan.SO3.identity()
omega = np.zeros(3, dtype=np.float64)
R2: cartan.SO3 = cartan.SO3.exp(omega)
R_inv: cartan.SO3 = R2.inverse()
R_back = R2.log()
M_rot = R2.matrix()
adj_R = R2.adjoint()
R_from: cartan.SO3 = cartan.SO3.from_matrix(np.eye(3, dtype=np.float64))
p3 = np.zeros(3, dtype=np.float64)
p3_rot = R2.act(p3)

T: cartan.SE3 = cartan.SE3.identity()
twist = np.zeros(6, dtype=np.float64)
T2: cartan.SE3 = cartan.SE3.exp(twist)
T_inv: cartan.SE3 = T2.inverse()
twist_back = T2.log()
M_T = T2.matrix()
adj_T = T2.adjoint()
trans = T2.translation
rot: cartan.SO3 = T2.rotation
T_from: cartan.SE3 = cartan.SE3.from_matrix(np.eye(4, dtype=np.float64))
p3_xf = T2.act(p3)

# ---------------------------------------------------------------------------
# Chain surface (ScrewAxis, JointLimits, KinematicChain)
# ---------------------------------------------------------------------------

axis_z = np.array([0.0, 0.0, 1.0], dtype=np.float64)
origin = np.zeros(3, dtype=np.float64)
s: cartan.ScrewAxis = cartan.ScrewAxis.revolute(axis_z, origin)
s_prismatic: cartan.ScrewAxis = cartan.ScrewAxis.prismatic(axis_z)
s_from: cartan.ScrewAxis = cartan.ScrewAxis.from_vector(s.to_vector())
omega_s = s.omega()
v_s = s.v()
six_vec = s.to_vector()
is_rev: bool = s.is_revolute()

lim: cartan.JointLimits = cartan.JointLimits(
    -3.14, 3.14, velocity_max=2.0, effort_max=5.0, acceleration_max=10.0
)
lim_min: float = lim.position_min
lim_max: float = lim.position_max
in_range: bool = lim.contains(0.0)

chain: cartan.KinematicChain = cartan.KinematicChain(T2, [s], [lim])
home_pose: cartan.SE3 = chain.home()
n_joints: int = chain.num_joints()
axis0: cartan.ScrewAxis = chain.axis(0)

# ---------------------------------------------------------------------------
# FK / Jacobian surface
# ---------------------------------------------------------------------------

q = np.zeros(n_joints, dtype=np.float64)
T_ee: cartan.SE3 = cartan.forward_kinematics(chain, q)
Js = cartan.space_jacobian(chain, q)
Jb = cartan.body_jacobian(chain, q)

# ---------------------------------------------------------------------------
# URDF surface
#
# UrdfError is registered via the raw CPython type-object API (PyType_FromSpec
# + PyModule_AddObject) so that it can subclass RuntimeError; that path is
# invisible to nanobind's automatic stub generator and the class does not
# appear in _core.pyi. The narrow mitigation here is to import UrdfError via
# cartan._core and explicitly cast its type for the strict checkers (the
# class is unambiguously present at runtime and the test suite covers it
# via test_urdf.py).
#
# Similarly, load_urdf's signature in the stub is `str | os.PathLike` with an
# unparameterized PathLike, which pyright's strict mode reports as a partially
# unknown type. We pass a plain `str`, which type-checks cleanly.
# ---------------------------------------------------------------------------

from typing import Callable, cast

from cartan._core import (
    UrdfFailure,
    UrdfLoadResult,
    UrdfMetadata,
    load_urdf as _raw_load_urdf,  # pyright: ignore[reportUnknownVariableType]
)

# Narrow the load_urdf signature for strict mode: the generated stub uses
# `str | os.PathLike` with an unparameterized PathLike[T] which strict pyright
# reports as a partially unknown type. We only need the str overload here.
load_urdf: Callable[[str], UrdfLoadResult] = cast(
    "Callable[[str], UrdfLoadResult]", _raw_load_urdf
)

UrdfError: type[RuntimeError] = cast(
    "type[RuntimeError]", getattr(cartan, "UrdfError")
)

try:
    result: UrdfLoadResult = load_urdf("nonexistent.urdf")
    meta: UrdfMetadata = result.metadata
    base_name: str = meta.base_link_name
    tool_name: str = meta.tool_link_name
    names: list[str] = meta.joint_names
    if names:
        idx: int = meta.joint_index(names[0])
    loaded_chain: cartan.KinematicChain = result.chain
except UrdfError as e:
    # UrdfError exposes .kind: UrdfFailure and .detail: str at runtime; the
    # static type here is the upcast RuntimeError, so the attribute access is
    # cast through getattr to keep strict mode happy.
    kind: UrdfFailure = cast("UrdfFailure", getattr(e, "kind"))
    detail: str = cast("str", getattr(e, "detail"))


# ---------------------------------------------------------------------------
# Iterative IK surface (IkConfig, IkResult, enums, solve_ik trio)
#
# IkConfig is kwargs-only at the binding boundary; every field is exposed
# both as a getter and as a setter on the bound class. solve_ik / _speed /
# _robust share the (chain, target, q_seed, config=None) signature and
# return cartan.IkResult.
# ---------------------------------------------------------------------------

ik_chain: cartan.KinematicChain = cartan.KinematicChain(
    cartan.SE3.identity(),
    [cartan.ScrewAxis.revolute(np.array([0.0, 0.0, 1.0], dtype=np.float64),
                                np.zeros(3, dtype=np.float64))],
    [cartan.JointLimits(-3.14, 3.14)],
)

ik_config: cartan.IkConfig = cartan.IkConfig(
    position_tol=1e-7,
    orientation_tol=1e-7,
    objective=cartan.IkObjective.speed,
)
ik_pos_tol: float = ik_config.position_tol
ik_obj: cartan.IkObjective = ik_config.objective

ik_q_seed = np.zeros(ik_chain.num_joints(), dtype=np.float64)
ik_target: cartan.SE3 = cartan.SE3.identity()

ik_result: cartan.IkResult = cartan.solve_ik(ik_chain, ik_target, ik_q_seed, ik_config)
ik_converged: bool = ik_result.converged
ik_iterations: int = ik_result.iterations
ik_error: float = ik_result.error_norm
ik_failure_reason: str = ik_result.failure_reason
ik_termination: cartan.IkTerminationReason = ik_result.termination_reason
ik_near_singular: bool = ik_result.near_singular
ik_condition_number: float = ik_result.condition_number

ik_result_speed: cartan.IkResult = cartan.solve_ik_speed(ik_chain, ik_target, ik_q_seed)
ik_result_robust: cartan.IkResult = cartan.solve_ik_robust(ik_chain, ik_target, ik_q_seed)

# Spot-check the IkFailure enum so the strict-mode gate sees the binding.
ik_failure_enum: cartan.IkFailure = cartan.IkFailure.unreachable


# ---------------------------------------------------------------------------
# Analytical surface (cartan.analytical.*)
#
# The submodule attribute is re-exported from cartan._core, so all twelve
# names land at module path `cartan.analytical.<name>`. AnalyticalStatus and
# AnalyticalResult are also aliased on the top-level cartan namespace by
# __init__.py for ergonomic call-site access.
# ---------------------------------------------------------------------------

analytical_result: cartan.AnalyticalResult = cartan.analytical.solve_pieper_6r(
    ik_chain, ik_target,
)
analytical_status: cartan.AnalyticalStatus = analytical_result.status
analytical_error_metric: float = analytical_result.error_metric
analytical_solutions = analytical_result.solutions

closest_solution = cartan.analytical.closest_to_seed(analytical_result, ik_q_seed)

analytical_valid: bool = cartan.analytical.verify_solution(
    ik_chain, ik_q_seed, ik_target, 1e-6,
)
analytical_filtered: cartan.AnalyticalResult = cartan.analytical.filter_valid_solutions(
    ik_chain, analytical_result, ik_target, 1e-6,
)
analytical_ranked: cartan.AnalyticalResult = cartan.analytical.solve_all(
    ik_chain, ik_target, q_seed=ik_q_seed,
)

# planar_2R and spatial_3R follow the same (chain, target) -> AnalyticalResult
# signature; included so the strict gate covers all three closed-form solvers.
analytical_planar: cartan.AnalyticalResult = cartan.analytical.solve_planar_2r(
    ik_chain, ik_target,
)
analytical_spatial: cartan.AnalyticalResult = cartan.analytical.solve_3r(
    ik_chain, ik_target,
)

# Paden-Kahan subproblem 1 / 2 / 3 free functions.
pk_omega = np.array([0.0, 0.0, 1.0], dtype=np.float64)
pk_q = np.array([1.0, 0.0, 0.0], dtype=np.float64)
pk_p = np.array([0.0, 1.0, 0.0], dtype=np.float64)
pk_p_prime = np.zeros(3, dtype=np.float64)

pk1_theta: float | None = cartan.analytical.paden_kahan_1(
    pk_omega, pk_q, pk_p, pk_p_prime,
)
pk2_pairs: list[tuple[float, float]] = cartan.analytical.paden_kahan_2(
    pk_omega, pk_omega, pk_q, pk_p, pk_p_prime,
)
pk3_thetas: list[float] = cartan.analytical.paden_kahan_3(
    pk_omega, pk_q, pk_p, pk_p_prime, 1.0,
)


# ---------------------------------------------------------------------------
# Exhaustive multi-start runner.
#
# Constructor is keyword-only after the chain argument. .solve takes a target
# SE3 and the same keyword-only knobs as the C++ exhaustive_ik_runner.
# ---------------------------------------------------------------------------

exhaustive_runner: cartan.ExhaustiveIKRunner = cartan.ExhaustiveIKRunner(
    ik_chain, policy=cartan.IkPolicy.speed,
)
exhaustive_policy: cartan.IkPolicy = cartan.IkPolicy.speed
exhaustive_ranking: cartan.RankingStrategy = cartan.RankingStrategy.distance_to_seed
exhaustive_solutions: list[cartan.IkResult] = exhaustive_runner.solve(
    ik_target,
    max_restarts=10,
    ranking=exhaustive_ranking,
)


# ---------------------------------------------------------------------------
# argmin-backed iterative IK trio.
#
# The three solve_ik_argmin_* symbols are only present when the wheel was
# built with CARTAN_BUILD_ARGMIN=ON; cartan.has_argmin reports the build
# flag. The has_argmin attribute is always present (its value differs across
# builds). The getattr + cast indirection here mirrors the URDF block above:
# strict mode sees the symbol come back as the correct callable signature
# whether the build is ON or OFF, so the sample stays stub-portable.
# ---------------------------------------------------------------------------

has_argmin: bool = cartan.has_argmin

SolverFn = Callable[
    [cartan.KinematicChain, cartan.SE3, np.ndarray, cartan.IkConfig | None],
    cartan.IkResult,
]

if has_argmin:
    argmin_slsqp: SolverFn = cast(SolverFn, getattr(cartan, "solve_ik_argmin_slsqp"))
    argmin_lm: SolverFn = cast(SolverFn, getattr(cartan, "solve_ik_argmin_lm"))
    argmin_lbfgsb: SolverFn = cast(SolverFn, getattr(cartan, "solve_ik_argmin_lbfgsb"))
    argmin_result_slsqp: cartan.IkResult = argmin_slsqp(ik_chain, ik_target, ik_q_seed, None)
    argmin_result_lm: cartan.IkResult = argmin_lm(ik_chain, ik_target, ik_q_seed, None)
    argmin_result_lbfgsb: cartan.IkResult = argmin_lbfgsb(ik_chain, ik_target, ik_q_seed, None)


if __name__ == "__main__":
    # Allow direct invocation as a smoke check; the real gate is the
    # subprocess pyright/mypy run in test_stubs.py.
    print("ok")
