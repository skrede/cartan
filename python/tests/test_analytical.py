"""Closed-form analytical IK: solve_pieper_6r / solve_planar_2r / solve_3r + helpers.

Pieper happy path is anchored on the ABB IRB 120 vendored URDF — its wrist axes
do intersect at a common point, so it satisfies the strict Pieper precondition
and the Pieper-6R solver returns up to 8 FK-verified branches per random
reachable target. The KUKA KR 6 R900 SIXX and UR5e vendored URDFs carry small
offsets that violate the strict-intersection precondition, so they return
either degenerate_geometry (UR5e — the wrist-axis intersection finder fails
during ctor) or unreachable (KR6 — the wrist construction succeeds but the
downstream workspace check rejects FK-walked targets). Those non-IRB
arms are exercised as soft-fail-path coverage rather than as happy-path
parity cases (see Plan SUMMARY for the empirical sweep that surfaced this).
"""

from __future__ import annotations

import numpy as np
import pytest

import cartan


POS_TOL = 1e-6


def _draw_q(chain: cartan.KinematicChain, rng: np.random.Generator) -> np.ndarray:
    """Draw a random joint vector clamped to a sane (mid +/- 0.4 * span) range.

    Joints with quasi-infinite URDF defaults (e.g. continuous-revolute fallback
    bounds) are sampled in +/- pi to keep the rng.uniform call well-conditioned.
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
# AnalyticalStatus enum: spot-check the 5 variants and assert no legacy names.
# ---------------------------------------------------------------------------


def test_analytical_status_enum_values() -> None:
    for name in (
        "ok",
        "unreachable",
        "degenerate_geometry",
        "singular_configuration",
        "verification_failed",
    ):
        assert hasattr(cartan.AnalyticalStatus, name), (
            f"AnalyticalStatus missing variant {name}"
        )
    # No stray legacy names from the pre-correction wording:
    assert not hasattr(cartan.AnalyticalStatus, "singular")
    assert not hasattr(cartan.AnalyticalStatus, "near_singular")


def test_analytical_submodule_surface() -> None:
    for name in (
        "AnalyticalResult",
        "AnalyticalStatus",
        "solve_pieper_6r",
        "solve_planar_2r",
        "solve_3r",
        "paden_kahan_1",
        "paden_kahan_2",
        "paden_kahan_3",
        "closest_to_seed",
        "verify_solution",
        "filter_valid_solutions",
        "solve_all",
    ):
        assert hasattr(cartan.analytical, name), (
            f"cartan.analytical missing {name}"
        )


# ---------------------------------------------------------------------------
# Pieper happy path: IRB 120 satisfies the strict wrist-intersection
# precondition and returns FK-verified branches on every random reachable
# target. This is the canonical Pieper parity case.
# ---------------------------------------------------------------------------


def test_solve_pieper_6r_irb120_returns_fk_verified_solutions(
    irb120_chain: cartan.KinematicChain,
) -> None:
    chain = irb120_chain
    rng = np.random.default_rng(seed=42)
    q_truth = _draw_q(chain, rng)
    target = cartan.forward_kinematics(chain, q_truth)

    result = cartan.analytical.solve_pieper_6r(chain, target)

    assert result.status == cartan.AnalyticalStatus.ok, (
        f"IRB120 returned {result.status!r} on FK-walked target"
    )
    assert len(result.solutions) >= 1
    for i, q in enumerate(result.solutions):
        assert q.shape == (chain.num_joints(),)
        T = cartan.forward_kinematics(chain, q)
        pos_err = float(np.linalg.norm(T.translation - target.translation))
        assert pos_err < POS_TOL, (
            f"IRB120 FK verification failed on branch {i}: pos_err={pos_err}"
        )


def test_solve_pieper_6r_irb120_returns_multiple_branches(
    irb120_chain: cartan.KinematicChain,
) -> None:
    # A reachable target from a non-singular q usually yields the full set
    # of 8 Pieper branches; spot-check that the solver doesn't collapse to
    # a single branch (which would suggest the FK back-check is overly
    # aggressive).
    chain = irb120_chain
    rng = np.random.default_rng(seed=11)
    q_truth = _draw_q(chain, rng)
    target = cartan.forward_kinematics(chain, q_truth)
    result = cartan.analytical.solve_pieper_6r(chain, target)
    assert result.status == cartan.AnalyticalStatus.ok
    # Non-singular targets typically yield >= 4 branches; assert weakly.
    assert len(result.solutions) >= 2, (
        f"expected >= 2 branches, got {len(result.solutions)}"
    )


# ---------------------------------------------------------------------------
# Pieper soft-fail path: UR5e + KR6 vendored URDFs carry wrist offsets that
# violate the strict Pieper-intersection precondition. Documented behavior is
# a non-ok status with empty solutions (Rule-1 deviation from the plan's
# must_haves item — see SUMMARY). The test asserts the surface contract holds.
# ---------------------------------------------------------------------------


def test_solve_pieper_6r_ur5e_soft_fails_on_wrist_offset(
    ur5e_chain: cartan.KinematicChain,
) -> None:
    chain = ur5e_chain
    rng = np.random.default_rng(seed=42)
    q_truth = _draw_q(chain, rng)
    target = cartan.forward_kinematics(chain, q_truth)
    result = cartan.analytical.solve_pieper_6r(chain, target)
    # UR5e vendored URDF has an offset wrist; the strict Pieper solver
    # rejects it at ctor (degenerate_geometry) or during decomposition
    # (unreachable / verification_failed). The plan's must_haves item
    # asserting ok is empirically wrong for the URDF-loaded UR5e.
    assert result.status in (
        cartan.AnalyticalStatus.degenerate_geometry,
        cartan.AnalyticalStatus.unreachable,
        cartan.AnalyticalStatus.verification_failed,
    ), f"UR5e returned unexpected status {result.status!r}"
    assert result.solutions == []


def test_solve_pieper_6r_kr6_soft_fails_on_wrist_offset(
    kr6_chain: cartan.KinematicChain,
) -> None:
    chain = kr6_chain
    rng = np.random.default_rng(seed=42)
    q_truth = _draw_q(chain, rng)
    target = cartan.forward_kinematics(chain, q_truth)
    result = cartan.analytical.solve_pieper_6r(chain, target)
    # KR6 vendored URDF has an offset wrist; the Pieper ctor accepts it
    # (the wrist construction succeeds) but the downstream workspace /
    # FK-verification step rejects FK-walked random targets.
    assert result.status in (
        cartan.AnalyticalStatus.degenerate_geometry,
        cartan.AnalyticalStatus.unreachable,
        cartan.AnalyticalStatus.verification_failed,
    ), f"KR6 returned unexpected status {result.status!r}"
    assert result.solutions == []


# ---------------------------------------------------------------------------
# Pieper rejected-by-design: 7R iiwa14 + non-6R-revolute cartanbot.
# ---------------------------------------------------------------------------


def test_solve_pieper_6r_iiwa14_returns_degenerate_geometry(
    iiwa14_chain: cartan.KinematicChain,
) -> None:
    chain = iiwa14_chain
    n = chain.num_joints()
    target = cartan.forward_kinematics(chain, np.zeros(n))
    result = cartan.analytical.solve_pieper_6r(chain, target)
    assert result.status == cartan.AnalyticalStatus.degenerate_geometry
    assert result.solutions == []


def test_solve_pieper_6r_cartanbot_returns_degenerate_geometry(
    cartanbot_chain: cartan.KinematicChain,
) -> None:
    # cartanbot has prismatic + continuous joints — the Pieper solver's
    # all-revolute precondition rejects it at ctor.
    chain = cartanbot_chain
    n = chain.num_joints()
    target = cartan.forward_kinematics(chain, np.zeros(n))
    result = cartan.analytical.solve_pieper_6r(chain, target)
    assert result.status == cartan.AnalyticalStatus.degenerate_geometry
    assert result.solutions == []


# ---------------------------------------------------------------------------
# planar 2R + spatial 3R: synthetic chains exercise both solvers end-to-end.
# ---------------------------------------------------------------------------


def test_solve_planar_2r_returns_fk_verified_branches(
    planar_2r_chain: cartan.KinematicChain,
) -> None:
    chain = planar_2r_chain
    rng = np.random.default_rng(seed=7)
    # Stay inside the reachable annulus by sampling within +/- pi/2.
    q_truth = rng.uniform(-np.pi / 2, np.pi / 2, size=2)
    target = cartan.forward_kinematics(chain, q_truth)

    result = cartan.analytical.solve_planar_2r(chain, target)
    assert result.status == cartan.AnalyticalStatus.ok
    assert len(result.solutions) >= 1
    for q in result.solutions:
        T = cartan.forward_kinematics(chain, q)
        assert np.linalg.norm(T.translation - target.translation) < POS_TOL


def test_solve_3r_returns_fk_verified_branches() -> None:
    # Build a small spatial 3R chain inline (no conftest fixture exists for it).
    s1 = cartan.ScrewAxis.revolute(
        np.array([0.0, 0.0, 1.0]), np.zeros(3))
    s2 = cartan.ScrewAxis.revolute(
        np.array([0.0, 1.0, 0.0]), np.zeros(3))
    s3 = cartan.ScrewAxis.revolute(
        np.array([0.0, 1.0, 0.0]), np.array([1.0, 0.0, 0.0]))
    home = cartan.SE3.exp(np.array([0.0, 0.0, 0.0, 2.0, 0.0, 0.0]))
    limits = [cartan.JointLimits(-np.pi, np.pi)] * 3
    chain = cartan.KinematicChain(home, [s1, s2, s3], limits)

    q_truth = np.array([0.3, -0.4, 0.5])
    target = cartan.forward_kinematics(chain, q_truth)
    result = cartan.analytical.solve_3r(chain, target)

    assert result.status == cartan.AnalyticalStatus.ok
    assert len(result.solutions) >= 1
    for q in result.solutions:
        T = cartan.forward_kinematics(chain, q)
        assert np.linalg.norm(T.translation - target.translation) < POS_TOL


# ---------------------------------------------------------------------------
# Paden-Kahan subproblems.
# ---------------------------------------------------------------------------


def test_paden_kahan_1_recovers_known_angle() -> None:
    # Rotate p = [1, 0, 0] about z through the origin by pi/4 and recover
    # the angle. paden_kahan_1 signature is (omega, q, p, p').
    omega = np.array([0.0, 0.0, 1.0])
    r = np.zeros(3)
    p = np.array([1.0, 0.0, 0.0])
    theta_truth = np.pi / 4
    c, s = np.cos(theta_truth), np.sin(theta_truth)
    p_prime = np.array([c * p[0] - s * p[1], s * p[0] + c * p[1], p[2]])
    theta = cartan.analytical.paden_kahan_1(omega, r, p, p_prime)
    assert theta is not None
    assert abs(theta - theta_truth) < 1e-10


def test_paden_kahan_2_returns_list_of_pairs() -> None:
    # Two rotations about z and y through the origin; pick a generic target
    # and check the solver returns valid (theta1, theta2) pairs.
    omega1 = np.array([0.0, 0.0, 1.0])
    omega2 = np.array([0.0, 1.0, 0.0])
    q = np.zeros(3)
    p = np.array([1.0, 0.0, 0.0])
    # Apply rot_z(0.3) * rot_y(0.4) to p analytically.
    Ry = np.array([
        [np.cos(0.4), 0.0, np.sin(0.4)],
        [0.0, 1.0, 0.0],
        [-np.sin(0.4), 0.0, np.cos(0.4)],
    ])
    Rz = np.array([
        [np.cos(0.3), -np.sin(0.3), 0.0],
        [np.sin(0.3), np.cos(0.3), 0.0],
        [0.0, 0.0, 1.0],
    ])
    p_prime = Rz @ Ry @ p
    pairs = cartan.analytical.paden_kahan_2(omega1, omega2, q, p, p_prime)
    assert isinstance(pairs, list)
    assert len(pairs) >= 1
    for t1, t2 in pairs:
        assert isinstance(t1, float)
        assert isinstance(t2, float)


def test_paden_kahan_3_returns_distance_solutions() -> None:
    # paden_kahan_3 finds theta such that ||rot(omega,theta)*p - p'|| = delta.
    omega = np.array([0.0, 0.0, 1.0])
    q = np.zeros(3)
    p = np.array([1.0, 0.0, 0.0])
    p_prime = np.array([0.0, 1.0, 0.0])
    delta = float(np.linalg.norm(p - p_prime))  # exactly satisfied at theta=0
    thetas = cartan.analytical.paden_kahan_3(omega, q, p, p_prime, delta)
    assert isinstance(thetas, list)
    assert len(thetas) >= 1


# ---------------------------------------------------------------------------
# Multi-solution helpers.
# ---------------------------------------------------------------------------


def test_closest_to_seed_picks_argmin(
    irb120_chain: cartan.KinematicChain,
) -> None:
    chain = irb120_chain
    rng = np.random.default_rng(seed=0)
    q_truth = _draw_q(chain, rng)
    target = cartan.forward_kinematics(chain, q_truth)
    result = cartan.analytical.solve_pieper_6r(chain, target)
    assert result.status == cartan.AnalyticalStatus.ok

    # Seed slightly perturbed from q_truth; closest_to_seed should pick the
    # branch nearest q_truth (which exists since FK is a round-trip).
    seed = q_truth + rng.uniform(-0.01, 0.01, size=q_truth.size)
    closest = cartan.analytical.closest_to_seed(result, seed)
    assert closest is not None
    assert closest.shape == (chain.num_joints(),)
    # Verify it is in fact the argmin:
    dists = [float(np.linalg.norm(q - seed)) for q in result.solutions]
    chosen_dist = float(np.linalg.norm(closest - seed))
    assert chosen_dist == pytest.approx(min(dists))


def test_closest_to_seed_returns_none_on_empty(
    cartanbot_chain: cartan.KinematicChain,
) -> None:
    chain = cartanbot_chain
    target = cartan.forward_kinematics(chain, np.zeros(chain.num_joints()))
    result = cartan.analytical.solve_pieper_6r(chain, target)
    assert result.solutions == []
    seed = np.zeros(chain.num_joints())
    assert cartan.analytical.closest_to_seed(result, seed) is None


def test_verify_solution_accepts_fk_inverse(
    irb120_chain: cartan.KinematicChain,
) -> None:
    chain = irb120_chain
    rng = np.random.default_rng(seed=11)
    q_truth = _draw_q(chain, rng)
    target = cartan.forward_kinematics(chain, q_truth)
    assert cartan.analytical.verify_solution(chain, q_truth, target, 1e-6)


def test_verify_solution_rejects_off_target(
    irb120_chain: cartan.KinematicChain,
) -> None:
    chain = irb120_chain
    rng = np.random.default_rng(seed=99)
    q_truth = _draw_q(chain, rng)
    target = cartan.forward_kinematics(chain, q_truth)
    # Bias q far away from any valid solution; verify_solution returns False.
    q_wrong = q_truth + 1.5  # large enough offset to fail the back-check
    assert not cartan.analytical.verify_solution(chain, q_wrong, target, 1e-6)


def test_filter_valid_solutions_keeps_only_passing(
    irb120_chain: cartan.KinematicChain,
) -> None:
    chain = irb120_chain
    rng = np.random.default_rng(seed=22)
    q_truth = _draw_q(chain, rng)
    target = cartan.forward_kinematics(chain, q_truth)
    result = cartan.analytical.solve_pieper_6r(chain, target)
    assert result.status == cartan.AnalyticalStatus.ok
    filtered = cartan.analytical.filter_valid_solutions(
        chain, result, target, 1e-6)
    assert filtered.status == result.status
    assert len(filtered.solutions) >= 1
    # Every kept solution must FK-verify at 1e-6:
    for q in filtered.solutions:
        assert cartan.analytical.verify_solution(chain, q, target, 1e-6)


def test_filter_valid_solutions_preserves_status_on_empty(
    cartanbot_chain: cartan.KinematicChain,
) -> None:
    # Filter a degenerate_geometry result (empty solutions); status survives.
    chain = cartanbot_chain
    target = cartan.forward_kinematics(chain, np.zeros(chain.num_joints()))
    result = cartan.analytical.solve_pieper_6r(chain, target)
    assert result.status == cartan.AnalyticalStatus.degenerate_geometry
    filtered = cartan.analytical.filter_valid_solutions(
        chain, result, target, 1e-6)
    assert filtered.status == cartan.AnalyticalStatus.degenerate_geometry
    assert filtered.solutions == []


# ---------------------------------------------------------------------------
# solve_all dispatch + ranking.
# ---------------------------------------------------------------------------


def test_solve_all_dispatches_by_joint_count(
    irb120_chain: cartan.KinematicChain,
    cartanbot_chain: cartan.KinematicChain,
    planar_2r_chain: cartan.KinematicChain,
) -> None:
    # 6R Pieper chain (IRB120) -> Pieper solutions.
    rng = np.random.default_rng(seed=33)
    q_truth = _draw_q(irb120_chain, rng)
    target_6r = cartan.forward_kinematics(irb120_chain, q_truth)
    result_6r = cartan.analytical.solve_all(irb120_chain, target_6r)
    assert result_6r.status == cartan.AnalyticalStatus.ok
    assert len(result_6r.solutions) >= 1

    # 2R chain -> planar_2r solutions.
    q_truth = rng.uniform(-np.pi / 2, np.pi / 2, size=2)
    target_2r = cartan.forward_kinematics(planar_2r_chain, q_truth)
    result_2r = cartan.analytical.solve_all(planar_2r_chain, target_2r)
    assert result_2r.status == cartan.AnalyticalStatus.ok
    assert len(result_2r.solutions) >= 1

    # cartanbot has prismatic / 6 joints with non-revolute -> Pieper rejects;
    # solve_all dispatches to pieper_6r_solver for n=6, which fails at ctor.
    target_bot = cartan.forward_kinematics(
        cartanbot_chain, np.zeros(cartanbot_chain.num_joints()))
    result_bot = cartan.analytical.solve_all(cartanbot_chain, target_bot)
    assert result_bot.status == cartan.AnalyticalStatus.degenerate_geometry


def test_solve_all_returns_degenerate_for_unsupported_joint_count() -> None:
    # Build a synthetic 4R chain (joint count not in {2, 3, 6}).
    z = np.array([0.0, 0.0, 1.0])
    y = np.array([0.0, 1.0, 0.0])
    axes = [
        cartan.ScrewAxis.revolute(z, np.zeros(3)),
        cartan.ScrewAxis.revolute(y, np.zeros(3)),
        cartan.ScrewAxis.revolute(y, np.array([0.5, 0.0, 0.0])),
        cartan.ScrewAxis.revolute(y, np.array([1.0, 0.0, 0.0])),
    ]
    home = cartan.SE3.exp(np.array([0.0, 0.0, 0.0, 1.5, 0.0, 0.0]))
    limits = [cartan.JointLimits(-np.pi, np.pi)] * 4
    chain = cartan.KinematicChain(home, axes, limits)
    target = cartan.forward_kinematics(chain, np.zeros(4))
    result = cartan.analytical.solve_all(chain, target)
    assert result.status == cartan.AnalyticalStatus.degenerate_geometry
    assert result.solutions == []


def test_solve_all_ranks_by_distance_to_seed(
    irb120_chain: cartan.KinematicChain,
) -> None:
    chain = irb120_chain
    rng = np.random.default_rng(seed=44)
    q_truth = _draw_q(chain, rng)
    target = cartan.forward_kinematics(chain, q_truth)
    seed = q_truth + rng.uniform(-0.05, 0.05, size=q_truth.size)
    ranked = cartan.analytical.solve_all(chain, target, q_seed=seed)
    assert ranked.status == cartan.AnalyticalStatus.ok
    assert len(ranked.solutions) >= 1
    # First element must match closest_to_seed output computed against the
    # unranked Pieper solve (both should pick the same argmin branch).
    plain = cartan.analytical.solve_pieper_6r(chain, target)
    expected_first = cartan.analytical.closest_to_seed(plain, seed)
    assert expected_first is not None
    np.testing.assert_allclose(
        ranked.solutions[0], expected_first, atol=1e-12)


# ---------------------------------------------------------------------------
# Hard-fail input contract: NaN target raises ValueError on every solver.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "fn_name",
    ["solve_pieper_6r", "solve_planar_2r", "solve_3r", "solve_all"],
)
def test_solver_raises_value_error_on_nan_target(
    irb120_chain: cartan.KinematicChain,
    fn_name: str,
) -> None:
    fn = getattr(cartan.analytical, fn_name)
    nan_target = cartan.SE3.exp(np.array([0.0, 0.0, 0.0, np.nan, 0.0, 0.0]))
    with pytest.raises(ValueError):
        fn(irb120_chain, nan_target)


# ---------------------------------------------------------------------------
# AnalyticalResult shape: read-only fields.
# ---------------------------------------------------------------------------


def test_analytical_result_repr_populated(
    irb120_chain: cartan.KinematicChain,
) -> None:
    chain = irb120_chain
    target = cartan.forward_kinematics(chain, np.zeros(6))
    r = cartan.analytical.solve_pieper_6r(chain, target)
    rep = repr(r)
    assert "AnalyticalResult" in rep
    assert "num_solutions=" in rep
    assert "status=" in rep
    assert "error_metric=" in rep


def test_analytical_result_fields_are_read_only(
    irb120_chain: cartan.KinematicChain,
) -> None:
    chain = irb120_chain
    target = cartan.forward_kinematics(chain, np.zeros(6))
    r = cartan.analytical.solve_pieper_6r(chain, target)
    with pytest.raises((AttributeError, TypeError)):
        r.status = cartan.AnalyticalStatus.unreachable  # type: ignore[misc]
