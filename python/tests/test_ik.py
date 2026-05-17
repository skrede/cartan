"""Iterative IK trio: solve_ik / solve_ik_speed / solve_ik_robust + IkConfig + IkResult.

Exercises the always-returned IkResult shape, IkConfig keyword-only construction,
the convergence path on FK-walked targets, the hard-fail ValueError path, and
the bound enum surface (IkObjective + IkFailure).
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np
import pytest

import cartan


# Looser than IkConfig.position_tol to absorb numerical noise on the back-solve.
TOL_ERROR_NORM = 1e-5


def _random_q_within_limits(
    chain: cartan.KinematicChain,
    rng: np.random.Generator,
    *,
    fallback_half_range: float = np.pi,
) -> np.ndarray:
    """Draw a random joint vector clamped to a sane range.

    Joints that report quasi-infinite URDF defaults (Float-typed default
    "no-limit" values) are sampled within +/- fallback_half_range around 0
    so the rng.uniform call doesn't overflow.
    """
    n = chain.num_joints()
    lims = chain.limits()
    out = np.empty(n, dtype=np.float64)
    for j in range(n):
        lo = lims[j].position_min
        hi = lims[j].position_max
        if not (np.isfinite(lo) and np.isfinite(hi)) or (hi - lo) > 1e6:
            lo = -fallback_half_range
            hi = fallback_half_range
        mid = 0.5 * (lo + hi)
        half = 0.4 * (hi - lo)
        out[j] = rng.uniform(mid - half, mid + half)
    return out


# ---------------------------------------------------------------------------
# IkConfig: defaults, keyword-only construction, mutation.
# ---------------------------------------------------------------------------


def test_ik_config_defaults() -> None:
    cfg = cartan.IkConfig()
    assert cfg.max_iterations_per_attempt == 100
    assert cfg.max_total_work_units == 200
    assert cfg.position_tol == pytest.approx(1e-6)
    assert cfg.orientation_tol == pytest.approx(1e-6)
    assert cfg.max_total_iterations == 500
    assert cfg.objective == cartan.IkObjective.speed
    assert cfg.halton_seed == 42


def test_ik_config_kwargs_override() -> None:
    cfg = cartan.IkConfig(
        position_tol=1e-7,
        halton_seed=99,
        objective=cartan.IkObjective.min_distance,
    )
    assert cfg.position_tol == pytest.approx(1e-7)
    assert cfg.halton_seed == 99
    assert cfg.objective == cartan.IkObjective.min_distance
    # Other fields keep their defaults.
    assert cfg.max_iterations_per_attempt == 100
    assert cfg.orientation_tol == pytest.approx(1e-6)


def test_ik_config_positional_rejected() -> None:
    # nb::kw_only() enforces keyword-only construction.
    with pytest.raises(TypeError):
        cartan.IkConfig(100)  # type: ignore[misc]


def test_ik_config_mutable_fields() -> None:
    cfg = cartan.IkConfig()
    cfg.position_tol = 1e-8
    cfg.max_total_iterations = 1000
    assert cfg.position_tol == pytest.approx(1e-8)
    assert cfg.max_total_iterations == 1000


def test_ik_config_repr_populated() -> None:
    rep = repr(cartan.IkConfig(position_tol=1e-7))
    assert "IkConfig" in rep
    assert "position_tol" in rep


# ---------------------------------------------------------------------------
# Enum surface: spot-check the four bound enums are accessible and distinct.
# ---------------------------------------------------------------------------


def test_ik_objective_enum_values() -> None:
    for name in ("speed", "min_distance", "max_manipulability", "max_isotropy"):
        assert hasattr(cartan.IkObjective, name), f"IkObjective missing variant {name}"
    assert cartan.IkObjective.speed != cartan.IkObjective.min_distance


def test_ik_failure_enum_values() -> None:
    for name in (
        "unreachable",
        "diverged",
        "stalled",
        "iteration_limit",
        "joint_limit_violation",
        "aborted",
    ):
        assert hasattr(cartan.IkFailure, name), f"IkFailure missing variant {name}"


def test_ik_termination_reason_enum_values() -> None:
    for name in (
        "unknown",
        "converged",
        "iteration_limit",
        "stall_detected",
        "divergence_detected",
        "joint_limit_hit",
        "solver_converged_pose_missed",
        "solver_ftol_reached",
        "solver_xtol_reached",
        "solver_objective_stalled",
        "solver_roundoff_limited",
        "solver_stalled",
        "solver_aborted",
        "solver_budget_exhausted",
        "solver_max_iterations",
        "solver_diverged",
    ):
        assert hasattr(cartan.IkTerminationReason, name), (
            f"IkTerminationReason missing variant {name}"
        )


# ---------------------------------------------------------------------------
# Convergence: FK-walked targets on cartanbot, parametrized over the trio.
# ---------------------------------------------------------------------------


SolverFn = Callable[..., cartan.IkResult]


@pytest.mark.parametrize(
    "solver_fn,solver_name",
    [
        (cartan.solve_ik, "solve_ik"),
        (cartan.solve_ik_speed, "solve_ik_speed"),
        (cartan.solve_ik_robust, "solve_ik_robust"),
    ],
)
def test_solve_ik_trio_converges_on_fk_walked_target(
    cartanbot_chain: cartan.KinematicChain,
    solver_fn: SolverFn,
    solver_name: str,
) -> None:
    chain = cartanbot_chain
    rng = np.random.default_rng(seed=42)
    q_truth = _random_q_within_limits(chain, rng)
    target = cartan.forward_kinematics(chain, q_truth)
    q_seed = q_truth + rng.uniform(-0.05, 0.05, size=chain.num_joints())

    result = solver_fn(chain, target, q_seed)

    assert result.converged, (
        f"{solver_name} did not converge on cartanbot FK-walked target: "
        f"{result!r}"
    )
    assert result.error_norm < TOL_ERROR_NORM
    assert result.q.shape == (chain.num_joints(),)
    assert result.q.dtype == np.float64
    assert result.iterations > 0
    # On success the populated termination_reason is the converged variant.
    assert result.termination_reason == cartan.IkTerminationReason.converged
    assert result.failure_reason == ""
    # C-06: condition_number is 0.0 on the success path.
    assert result.condition_number == 0.0
    assert result.near_singular is False


@pytest.mark.parametrize(
    "solver_fn,solver_name",
    [
        (cartan.solve_ik, "solve_ik"),
        (cartan.solve_ik_speed, "solve_ik_speed"),
        (cartan.solve_ik_robust, "solve_ik_robust"),
    ],
)
def test_solve_ik_trio_converges_on_ur3e(
    ur3e_chain: cartan.KinematicChain,
    solver_fn: SolverFn,
    solver_name: str,
) -> None:
    chain = ur3e_chain
    rng = np.random.default_rng(seed=7)
    q_truth = _random_q_within_limits(chain, rng)
    target = cartan.forward_kinematics(chain, q_truth)
    q_seed = q_truth + rng.uniform(-0.05, 0.05, size=chain.num_joints())

    result = solver_fn(chain, target, q_seed)
    assert result.converged, f"{solver_name} did not converge on ur3e: {result!r}"
    assert result.error_norm < TOL_ERROR_NORM
    assert result.q.shape == (chain.num_joints(),)


# ---------------------------------------------------------------------------
# IkResult: shape and repr.
# ---------------------------------------------------------------------------


def test_ik_result_repr_populated(cartanbot_chain: cartan.KinematicChain) -> None:
    chain = cartanbot_chain
    rng = np.random.default_rng(seed=1)
    q_truth = _random_q_within_limits(chain, rng)
    target = cartan.forward_kinematics(chain, q_truth)

    result = cartan.solve_ik(chain, target, q_truth.copy())

    rep = repr(result)
    assert "IkResult" in rep
    assert "converged=" in rep
    assert "iterations=" in rep
    assert "error_norm=" in rep


def test_ik_result_fields_are_read_only(
    cartanbot_chain: cartan.KinematicChain,
) -> None:
    # IkResult is an immutable value object — every field is def_ro on the
    # binding side, so assignment must raise.
    chain = cartanbot_chain
    rng = np.random.default_rng(seed=2)
    q_truth = _random_q_within_limits(chain, rng)
    target = cartan.forward_kinematics(chain, q_truth)
    result = cartan.solve_ik(chain, target, q_truth.copy())
    with pytest.raises((AttributeError, TypeError)):
        result.converged = False  # type: ignore[misc]


# ---------------------------------------------------------------------------
# Hard-fail paths: joint-count mismatch and non-finite target raise ValueError.
# ---------------------------------------------------------------------------


def test_solve_ik_value_error_on_joint_count_mismatch(
    cartanbot_chain: cartan.KinematicChain,
) -> None:
    chain = cartanbot_chain
    n = chain.num_joints()
    target = cartan.forward_kinematics(chain, np.zeros(n))

    with pytest.raises(ValueError):
        cartan.solve_ik(chain, target, np.zeros(n + 1))
    if n > 1:
        with pytest.raises(ValueError):
            cartan.solve_ik(chain, target, np.zeros(n - 1))


def test_solve_ik_speed_value_error_on_joint_count_mismatch(
    cartanbot_chain: cartan.KinematicChain,
) -> None:
    chain = cartanbot_chain
    n = chain.num_joints()
    target = cartan.forward_kinematics(chain, np.zeros(n))
    with pytest.raises(ValueError):
        cartan.solve_ik_speed(chain, target, np.zeros(n + 1))


def test_solve_ik_robust_value_error_on_joint_count_mismatch(
    cartanbot_chain: cartan.KinematicChain,
) -> None:
    chain = cartanbot_chain
    n = chain.num_joints()
    target = cartan.forward_kinematics(chain, np.zeros(n))
    with pytest.raises(ValueError):
        cartan.solve_ik_robust(chain, target, np.zeros(n + 1))


def test_solve_ik_value_error_on_nan_target(
    cartanbot_chain: cartan.KinematicChain,
) -> None:
    chain = cartanbot_chain
    n = chain.num_joints()
    # SE3.exp does not validate the input twist, so we can construct an SE3
    # with a NaN translation directly through it.
    nan_twist = np.array([0.0, 0.0, 0.0, np.nan, 0.0, 0.0])
    nan_target = cartan.SE3.exp(nan_twist)
    with pytest.raises(ValueError):
        cartan.solve_ik(chain, nan_target, np.zeros(n))


# ---------------------------------------------------------------------------
# Dtype hygiene: noconvert() should reject float32 q_seed (binding contract).
# ---------------------------------------------------------------------------


def test_solve_ik_rejects_float32_q_seed(
    cartanbot_chain: cartan.KinematicChain,
) -> None:
    chain = cartanbot_chain
    n = chain.num_joints()
    target = cartan.forward_kinematics(chain, np.zeros(n))
    q32 = np.zeros(n, dtype=np.float32)
    with pytest.raises(TypeError):
        cartan.solve_ik(chain, target, q32)


# ---------------------------------------------------------------------------
# config kwarg: accepts an IkConfig, accepts None, accepts an absent kwarg.
# ---------------------------------------------------------------------------


def test_solve_ik_accepts_explicit_config(
    cartanbot_chain: cartan.KinematicChain,
) -> None:
    chain = cartanbot_chain
    rng = np.random.default_rng(seed=11)
    q_truth = _random_q_within_limits(chain, rng)
    target = cartan.forward_kinematics(chain, q_truth)
    q_seed = q_truth + rng.uniform(-0.02, 0.02, size=chain.num_joints())

    cfg = cartan.IkConfig(position_tol=1e-7, orientation_tol=1e-7)
    result = cartan.solve_ik(chain, target, q_seed, cfg)
    assert result.converged
    assert result.error_norm < TOL_ERROR_NORM
