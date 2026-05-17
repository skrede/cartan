"""argmin-backed iterative IK solvers (gated on cartan.has_argmin).

When the cartan extension is built with CARTAN_BUILD_ARGMIN=ON, three
additional free functions are registered: solve_ik_argmin_slsqp,
solve_ik_argmin_lm, solve_ik_argmin_lbfgsb. Each shares the IkConfig /
IkResult contract with the iterative trio. When the build flag is off,
the symbols are absent from the cartan module and cartan.has_argmin is
False; the whole module skips cleanly via the module-level pytestmark.
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np
import pytest

import cartan


pytestmark = pytest.mark.skipif(
    not cartan.has_argmin,
    reason="cartan built without CARTAN_BUILD_ARGMIN=ON",
)


# Looser than IkConfig.position_tol to absorb numerical noise on the back-solve.
TOL_ERROR_NORM = 1e-5


def _random_q_within_limits(
    chain: cartan.KinematicChain,
    rng: np.random.Generator,
    *,
    fallback_half_range: float = np.pi,
) -> np.ndarray:
    """Draw a random joint vector well inside the chain's limits.

    Mirrors the helper in test_ik.py so both modules sample seeds the same
    way; the test_ik.py copy stays private to keep test files independent.
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
# has_argmin sanity + symbol presence.
# ---------------------------------------------------------------------------


def test_has_argmin_is_true() -> None:
    # The module-level pytestmark guarantees we only reach this when the
    # build flag is on; assert the value matches the gate.
    assert cartan.has_argmin is True


def test_argmin_symbols_present() -> None:
    assert hasattr(cartan, "solve_ik_argmin_slsqp")
    assert hasattr(cartan, "solve_ik_argmin_lm")
    assert hasattr(cartan, "solve_ik_argmin_lbfgsb")


# ---------------------------------------------------------------------------
# Per-solver convergence smoke on the UR5e fixture (FK-walked target).
# Lambda-wrapped solver_fn defers attribute lookup to the test body so a
# CARTAN_BUILD_ARGMIN=OFF host doesn't fail collection on parametrize
# evaluation.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "solver_fn, solver_name",
    [
        (lambda: cartan.solve_ik_argmin_slsqp, "solve_ik_argmin_slsqp"),
        (lambda: cartan.solve_ik_argmin_lm, "solve_ik_argmin_lm"),
        (lambda: cartan.solve_ik_argmin_lbfgsb, "solve_ik_argmin_lbfgsb"),
    ],
)
def test_solve_ik_argmin_trio_converges_on_fk_walked_target(
    ur5e_chain: cartan.KinematicChain,
    solver_fn: Callable[[], Callable[..., cartan.IkResult]],
    solver_name: str,
) -> None:
    """FK-walked target + nearby seed must converge for each argmin solver."""
    chain = ur5e_chain
    rng = np.random.default_rng(seed=42)
    q_truth = _random_q_within_limits(chain, rng)
    target = cartan.forward_kinematics(chain, q_truth)
    q_seed = q_truth + rng.uniform(-0.05, 0.05, size=q_truth.size)

    solver = solver_fn()
    # A slightly higher per-attempt budget keeps the argmin solvers from
    # bottoming out on the default 100-iteration cap; the iterative trio
    # uses dual-policy racing which absorbs slow inner convergence.
    cfg = cartan.IkConfig(max_iterations_per_attempt=200, max_total_work_units=400)
    result = solver(chain, target, q_seed, cfg)

    assert result.converged, f"{solver_name} failed: {result!r}"
    assert result.error_norm < TOL_ERROR_NORM, (
        f"{solver_name} did not reach tolerance: {result!r}"
    )


# ---------------------------------------------------------------------------
# Hard-fail guards: ValueError on joint-count mismatch + non-finite target.
# ---------------------------------------------------------------------------


def test_solve_ik_argmin_slsqp_value_error_on_joint_count_mismatch(
    ur5e_chain: cartan.KinematicChain,
) -> None:
    chain = ur5e_chain
    target = cartan.forward_kinematics(chain, np.zeros(chain.num_joints()))
    with pytest.raises(ValueError):
        cartan.solve_ik_argmin_slsqp(
            chain, target, np.zeros(chain.num_joints() + 1)
        )


def test_solve_ik_argmin_lm_value_error_on_nan_target(
    ur5e_chain: cartan.KinematicChain,
) -> None:
    chain = ur5e_chain
    bad_target = cartan.SE3.exp(
        np.array([0.0, 0.0, 0.0, np.nan, 0.0, 0.0])
    )
    with pytest.raises(ValueError):
        cartan.solve_ik_argmin_lm(
            chain, bad_target, np.zeros(chain.num_joints())
        )
