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
    assert cfg.objective == cartan.IkObjective.speed
    assert cfg.halton_seed == 42


def test_ik_config_kwargs_override() -> None:
    cfg = cartan.IkConfig(
        position_tol=1e-7,
        halton_seed=99,
        objective=cartan.IkObjective.min_error_norm,
    )
    assert cfg.position_tol == pytest.approx(1e-7)
    assert cfg.halton_seed == 99
    assert cfg.objective == cartan.IkObjective.min_error_norm
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
    cfg.max_total_work_units = 1000
    assert cfg.position_tol == pytest.approx(1e-8)
    assert cfg.max_total_work_units == 1000


def test_ik_config_repr_populated() -> None:
    rep = repr(cartan.IkConfig(position_tol=1e-7))
    assert "IkConfig" in rep
    assert "position_tol" in rep


def test_ik_config_characteristic_length() -> None:
    # Defaults to one in the chain's linear unit, which reproduces the
    # unnormalized Jacobian measures exactly.
    assert cartan.IkConfig().characteristic_length == pytest.approx(1.0)
    cfg = cartan.IkConfig(characteristic_length=0.25)
    assert cfg.characteristic_length == pytest.approx(0.25)
    cfg.characteristic_length = 0.5
    assert cfg.characteristic_length == pytest.approx(0.5)


# ---------------------------------------------------------------------------
# Enum surface: spot-check the four bound enums are accessible and distinct.
# ---------------------------------------------------------------------------


def test_ik_objective_enum_values() -> None:
    for name in (
        "speed",
        "min_error_norm",
        "min_joint_distance",
        "max_manipulability",
        "max_isotropy",
    ):
        assert hasattr(cartan.IkObjective, name), f"IkObjective missing variant {name}"
    assert not hasattr(cartan.IkObjective, "min_distance")
    assert cartan.IkObjective.speed != cartan.IkObjective.min_error_norm


def test_ik_failure_enum_values() -> None:
    for name in (
        "unreachable",
        "diverged",
        "stalled",
        "iteration_limit",
        "joint_limit_violation",
        "aborted",
        "unsupported_configuration",
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
    # The two fabricated diagnostics are gone from the result rather than
    # reporting a zero condition number and a definite negative on a solve that
    # measured neither. Conditioning is asked of the configuration instead.
    assert not hasattr(result, "condition_number")
    assert not hasattr(result, "near_singular")
    assert result.solved_feasible_set == cartan.FeasibleSet.declared


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


def test_solve_ik_reports_the_selection_it_made(
    cartanbot_chain: cartan.KinematicChain,
) -> None:
    chain = cartanbot_chain
    rng = np.random.default_rng(seed=17)
    q_truth = _random_q_within_limits(chain, rng)
    target = cartan.forward_kinematics(chain, q_truth)
    q_seed = q_truth + rng.uniform(-0.02, 0.02, size=chain.num_joints())

    # The speed objective ranks nothing, so its metric is absent rather than a
    # zero a caller could read as a measurement.
    speed = cartan.solve_ik(chain, target, q_seed)
    assert speed.converged
    assert speed.selection_objective == cartan.IkObjective.speed
    assert speed.selection_metric is None

    cfg = cartan.IkConfig(objective=cartan.IkObjective.max_manipulability)
    ranked = cartan.solve_ik(chain, target, q_seed, cfg)
    assert ranked.converged
    assert ranked.selection_objective == cartan.IkObjective.max_manipulability
    assert ranked.selection_metric is not None
    assert ranked.selection_metric > 0.0


def test_singularity_analysis_reads_one_spectrum(
    cartanbot_chain: cartan.KinematicChain,
) -> None:
    chain = cartanbot_chain
    rng = np.random.default_rng(seed=23)
    q = _random_q_within_limits(chain, rng)

    sigma = cartan.singular_values(chain, q)
    assert sigma is not None
    assert sigma.shape == (min(6, chain.num_joints()),)
    assert np.all(np.diff(sigma) <= 0.0), "singular values are largest first"

    kappa = cartan.condition_number(sigma)
    assert kappa is not None
    assert kappa == pytest.approx(sigma[0] / sigma[-1])
    assert cartan.manipulability(sigma) == pytest.approx(float(np.prod(sigma)))
    assert cartan.isotropy(sigma) == pytest.approx(1.0 / kappa)

    # The threshold is the caller's, and it is in the signature rather than
    # baked into a stored flag, so the same configuration answers both ways.
    assert cartan.is_near_singular(sigma, kappa * 0.5) is True
    assert cartan.is_near_singular(sigma, kappa * 2.0) is False
    assert cartan.is_near_singular(chain, q, kappa * 0.5) is True


def test_singularity_analysis_is_undefined_rather_than_wrong_without_a_spectrum() -> None:
    empty = np.zeros(0, dtype=np.float64)

    # A chain with no joints is a valid chain and an entirely zero Jacobian is a
    # valid Jacobian; the caller did nothing wrong, so the measure is absent
    # rather than an error to be caught.
    for measure in (
        cartan.condition_number,
        cartan.manipulability,
        cartan.isotropy,
        cartan.is_near_singular,
    ):
        assert measure(empty) is None

    assert cartan.isotropy(np.zeros(3, dtype=np.float64)) is None

    jointless = cartan.KinematicChain(cartan.SE3.identity(), [], [])
    assert cartan.singular_values(jointless, empty) is None
    assert cartan.is_near_singular(jointless, empty) is None


def test_singularity_analysis_refuses_a_configuration_the_chain_cannot_accept(
    cartanbot_chain: cartan.KinematicChain,
) -> None:
    chain = cartanbot_chain

    # A mis-sized or non-finite q is a bad argument, so it raises the same
    # ValueError body_jacobian raises for it rather than answering None.
    with pytest.raises(ValueError, match="does not produce a Jacobian"):
        cartan.singular_values(chain, np.zeros(chain.num_joints() - 1, dtype=np.float64))

    poisoned = np.zeros(chain.num_joints(), dtype=np.float64)
    poisoned[0] = np.nan
    with pytest.raises(ValueError, match="does not produce a Jacobian"):
        cartan.is_near_singular(chain, poisoned)


def test_feasible_set_enum_values() -> None:
    for name in ("declared", "substituted"):
        assert hasattr(cartan.FeasibleSet, name), f"FeasibleSet missing variant {name}"
    assert cartan.FeasibleSet.declared != cartan.FeasibleSet.substituted
