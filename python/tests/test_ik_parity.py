"""Cross-solver and cross-method parity for the iterative + analytical IK surface.

The parity matrix exercised here:

(a) **Iterative-trio parity.** For a single FK-walked target on each of the
    four arm fixtures, all three iterative entry points
    (`solve_ik` / `solve_ik_speed` / `solve_ik_robust`) converge to a joint
    vector whose FK round-trip matches the target within `POS_TOL`. The
    dual-policy racing of `solve_ik` is expected to agree with the
    single-policy `_speed` and `_robust` siblings at default `IkConfig`.

(b) **Analytical-as-FK-verified-subset-of-iterative.** On the canonical IRB120
    Pieper fixture, every closed-form branch returned by
    `solve_pieper_6r` FK-verifies the target at `FK_TOL`. The iterative
    `solve_ik` result also FK-verifies the same target. The two need not
    land on the same joint vector (the iterative solver converges to
    whichever 2*pi multiple its seed is closest to; the analytical solver
    returns canonical representatives), so the parity assertion is on
    workspace pose agreement, not on joint-vector equality. KR6 and UR5e
    are excluded from the Pieper branch because their vendored URDFs do
    not satisfy the strict wrist-intersection precondition (Plan 06
    summary captures the 20-seed sweep that surfaced this).

(c) **`solve_all` ranking correctness.** `solve_all(chain, target, q_seed=seed)`
    returns the surviving Pieper branches sorted by ascending L2 distance
    to the seed. `solve_all(...).solutions[0]` must equal
    `closest_to_seed(solve_pieper_6r(...), seed)` to within float
    round-off.

(d) **`ExhaustiveIKRunner` gate per Pieper fixture.** 50 random reachable
    FK-walked targets per (chain, policy=speed) cell; each `runner.solve`
    must return at least one solution. This is the non-slow smoke
    counterpart of the 100-target slow gate in `test_exhaustive.py`.

(e) **argmin family under `importorskip` guard.** Spot-check that the three
    argmin-backed entry points converge on a UR5e FK-walked target when
    the wheel is built with `CARTAN_BUILD_ARGMIN=ON`; skip cleanly
    otherwise.
"""

from __future__ import annotations

import numpy as np
import pytest

import cartan


# Tolerance on the FK round-trip position-error norm for the parity checks.
# Looser than IkConfig.position_tol (1e-6) to absorb the second-step FK
# multiplication's numerical noise on top of the converged-IK first step.
POS_TOL = 1e-5

# Tolerance for the strict analytical FK-verify check (every Pieper branch
# must reproduce the target up to this position-error norm).
FK_TOL = 1e-6


def _draw_q(chain: cartan.KinematicChain, rng: np.random.Generator) -> np.ndarray:
    """Draw a random joint vector clamped to a sane (mid +/- 0.4 * span) range.

    Joints with quasi-infinite URDF defaults (e.g. continuous-revolute
    fallback bounds) are sampled in +/- pi to keep the rng.uniform call
    well-conditioned.
    """
    n = chain.num_joints()
    lims = chain.limits()
    out = np.empty(n, dtype=np.float64)
    for j in range(n):
        lo = lims[j].position_min
        hi = lims[j].position_max
        if not (np.isfinite(lo) and np.isfinite(hi)) or (hi - lo) > 1e6:
            lo, hi = -np.pi, np.pi
        mid = 0.5 * (lo + hi)
        half = 0.4 * (hi - lo)
        out[j] = rng.uniform(mid - half, mid + half)
    return out


# ---------------------------------------------------------------------------
# (a) Iterative-trio parity on a single FK-walked target per fixture.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "chain_fixture",
    ["ur5e_chain", "kr6_chain", "irb120_chain", "cartanbot_chain"],
)
def test_iterative_trio_parity_on_fk_walked_target(
    chain_fixture: str,
    request: pytest.FixtureRequest,
) -> None:
    """`solve_ik` / `_speed` / `_robust` converge on the same target.

    Joint vectors may differ across the trio because of seed-dependent
    branch selection; the parity assertion is on FK round-trip position
    error, not on joint-vector equality.
    """
    chain = request.getfixturevalue(chain_fixture)
    rng = np.random.default_rng(seed=42)
    q_truth = _draw_q(chain, rng)
    target = cartan.forward_kinematics(chain, q_truth)
    q_seed = q_truth + rng.uniform(-0.05, 0.05, size=q_truth.size)

    solvers = [
        ("solve_ik", cartan.solve_ik),
        ("solve_ik_speed", cartan.solve_ik_speed),
        ("solve_ik_robust", cartan.solve_ik_robust),
    ]
    for name, solver in solvers:
        result = solver(chain, target, q_seed)
        assert result.converged, f"{chain_fixture}/{name} failed: {result!r}"
        T_back = cartan.forward_kinematics(chain, result.q)
        pos_err = float(np.linalg.norm(T_back.translation - target.translation))
        assert pos_err < POS_TOL, (
            f"{chain_fixture}/{name} FK round-trip pos_err={pos_err} > {POS_TOL}"
        )


# ---------------------------------------------------------------------------
# (b) Analytical-as-FK-verified-subset-of-iterative on Pieper geometry.
#
# Per Plan 06 SUMMARY: only the IRB120 vendored URDF satisfies the strict
# wrist-axis-intersection precondition. UR5e returns degenerate_geometry
# and KR6 returns unreachable on FK-walked random targets, so they are
# excluded from this parity branch (their iterative IK works fine -- see
# test (a) above -- but their closed-form branch is empty by design).
# ---------------------------------------------------------------------------


def test_analytical_solutions_are_fk_verified_subset_of_iterative(
    irb120_chain: cartan.KinematicChain,
) -> None:
    """Every Pieper branch FK-verifies; iterative agrees in workspace pose."""
    chain = irb120_chain
    rng = np.random.default_rng(seed=7)
    q_truth = _draw_q(chain, rng)
    target = cartan.forward_kinematics(chain, q_truth)
    q_seed = q_truth + rng.uniform(-0.05, 0.05, size=q_truth.size)

    a_result = cartan.analytical.solve_pieper_6r(chain, target)
    assert a_result.status == cartan.AnalyticalStatus.ok, (
        f"IRB120 unexpectedly returned status={a_result.status}; "
        f"this fixture is the canonical Pieper happy path."
    )
    assert len(a_result.solutions) >= 1

    # Every analytical solution FK-verifies the target.
    for i, q_a in enumerate(a_result.solutions):
        T_a = cartan.forward_kinematics(chain, q_a)
        pos_err = float(np.linalg.norm(T_a.translation - target.translation))
        assert pos_err < FK_TOL, (
            f"IRB120 analytical branch {i} FK pos_err={pos_err} > {FK_TOL}"
        )

    # Iterative converges and its FK round-trip matches the target. We do
    # NOT assert joint-vector equality vs any analytical branch -- the
    # iterative solver returns whichever 2*pi multiple its seed is closest
    # to, while the analytical solver returns canonical representatives in
    # the [-pi, pi] range. Both produce FK-equivalent poses; the parity is
    # workspace-level, not configuration-space-level.
    ik = cartan.solve_ik(chain, target, q_seed)
    assert ik.converged
    T_iter = cartan.forward_kinematics(chain, ik.q)
    iter_pos_err = float(np.linalg.norm(T_iter.translation - target.translation))
    assert iter_pos_err < POS_TOL


# ---------------------------------------------------------------------------
# (c) solve_all ranking correctness: first element matches closest_to_seed.
# ---------------------------------------------------------------------------


def test_solve_all_first_element_matches_closest_to_seed(
    irb120_chain: cartan.KinematicChain,
) -> None:
    """`solve_all(..., q_seed=seed).solutions[0] == closest_to_seed(...)`."""
    chain = irb120_chain
    rng = np.random.default_rng(seed=11)
    q_truth = _draw_q(chain, rng)
    target = cartan.forward_kinematics(chain, q_truth)
    seed = q_truth + rng.uniform(-0.05, 0.05, size=q_truth.size)

    ranked = cartan.analytical.solve_all(chain, target, q_seed=seed)
    plain = cartan.analytical.solve_pieper_6r(chain, target)
    expected_first = cartan.analytical.closest_to_seed(plain, seed)

    assert ranked.status == cartan.AnalyticalStatus.ok
    assert len(ranked.solutions) >= 1
    assert expected_first is not None
    np.testing.assert_allclose(
        ranked.solutions[0], expected_first, atol=1e-12
    )


# ---------------------------------------------------------------------------
# (d) ExhaustiveIKRunner per-Pieper-fixture gate: 50-target smoke version
#     of the SC#4a >= 1-solution gate. Full 100-target gate lives in
#     test_exhaustive.py's slow suite.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "chain_fixture",
    ["ur5e_chain", "kr6_chain", "irb120_chain"],
)
def test_exhaustive_runner_gate_per_pieper_fixture(
    chain_fixture: str,
    request: pytest.FixtureRequest,
) -> None:
    """50 FK-walked targets, each must return >= 1 IkResult on the speed policy."""
    chain = request.getfixturevalue(chain_fixture)
    runner = cartan.ExhaustiveIKRunner(chain, policy=cartan.IkPolicy.speed)
    rng = np.random.default_rng(seed=22)
    for i in range(50):
        q_truth = _draw_q(chain, rng)
        target = cartan.forward_kinematics(chain, q_truth)
        solutions = runner.solve(target, max_restarts=30)
        assert len(solutions) >= 1, (
            f"{chain_fixture} returned 0 solutions on target {i}"
        )


# ---------------------------------------------------------------------------
# (e) argmin family parity under cartan.has_argmin guard.
# ---------------------------------------------------------------------------


def test_argmin_family_parity_under_importorskip(
    ur5e_chain: cartan.KinematicChain,
) -> None:
    """When `cartan.has_argmin`, all three argmin solvers converge on UR5e.

    Skips cleanly on -OFF builds where the symbols are absent.
    """
    if not cartan.has_argmin:
        pytest.skip("cartan built without CARTAN_BUILD_ARGMIN=ON")

    chain = ur5e_chain
    rng = np.random.default_rng(seed=33)
    q_truth = _draw_q(chain, rng)
    target = cartan.forward_kinematics(chain, q_truth)
    q_seed = q_truth + rng.uniform(-0.05, 0.05, size=q_truth.size)

    # The argmin lambdas are single-policy and need a slightly higher inner
    # budget than the dual-policy iterative trio's default; mirror the
    # convergence-test budget from test_argmin.py.
    cfg = cartan.IkConfig(
        max_iterations_per_attempt=200, max_total_work_units=400
    )

    for solver, name in [
        (cartan.solve_ik_argmin_slsqp, "argmin_slsqp"),
        (cartan.solve_ik_argmin_lm, "argmin_lm"),
        (cartan.solve_ik_argmin_lbfgsb, "argmin_lbfgsb"),
    ]:
        result = solver(chain, target, q_seed, cfg)
        assert result.converged, f"{name} failed: {result!r}"
        assert result.error_norm < POS_TOL, (
            f"{name} did not reach tolerance: {result!r}"
        )
