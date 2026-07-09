"""Installed-wheel smoke tests for the public cartan Python surface."""

from __future__ import annotations

import numpy as np

import cartan


def _se3_from_matrix(matrix: np.ndarray) -> cartan.SE3:
    return cartan.SE3.from_matrix(matrix)


def _ur5e_like_chain() -> cartan.KinematicChain:
    axes = [
        cartan.ScrewAxis.revolute(
            np.array([0.0, 0.0, 1.0], dtype=np.float64),
            np.array([0.0, 0.0, 0.0], dtype=np.float64),
        ),
        cartan.ScrewAxis.revolute(
            np.array([1.22464680e-16, 1.0, -2.05103490e-10], dtype=np.float64),
            np.array([0.0, 0.0, 0.1625], dtype=np.float64),
        ),
        cartan.ScrewAxis.revolute(
            np.array([1.22464680e-16, 1.0, -2.05103490e-10], dtype=np.float64),
            np.array([0.425, 0.0, 0.1625], dtype=np.float64),
        ),
        cartan.ScrewAxis.revolute(
            np.array([1.22464680e-16, 1.0, -2.05103490e-10], dtype=np.float64),
            np.array([0.8172, 0.0, 0.1625], dtype=np.float64),
        ),
        cartan.ScrewAxis.revolute(
            np.array([-5.02358629e-26, -4.10207091e-10, -1.0], dtype=np.float64),
            np.array([0.8172, 0.1333, 0.0], dtype=np.float64),
        ),
        cartan.ScrewAxis.revolute(
            np.array([2.51179352e-26, 1.0, -2.05103490e-10], dtype=np.float64),
            np.array([0.8172, 0.0, 0.0628], dtype=np.float64),
        ),
    ]
    home_matrix = np.array(
        [
            [-1.0, 7.85046230e-17, 2.35513869e-16, 0.8172],
            [2.35513869e-16, 2.05103601e-10, 1.0, 0.2329],
            [7.85046229e-17, 1.0, -2.05103934e-10, 0.0628],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    limits = [
        cartan.JointLimits(-2.0 * np.pi, 2.0 * np.pi),
        cartan.JointLimits(-2.0 * np.pi, 2.0 * np.pi),
        cartan.JointLimits(-np.pi, np.pi),
        cartan.JointLimits(-2.0 * np.pi, 2.0 * np.pi),
        cartan.JointLimits(-2.0 * np.pi, 2.0 * np.pi),
        cartan.JointLimits(-2.0 * np.pi, 2.0 * np.pi),
    ]
    return cartan.KinematicChain(_se3_from_matrix(home_matrix), axes, limits)


def test_installed_wheel_exports_core_surface() -> None:
    assert cartan.__version__.startswith("0.4.2")
    for name in (
        "SE3",
        "KinematicChain",
        "forward_kinematics",
        "solve_ik",
    ):
        assert hasattr(cartan, name), f"cartan missing {name}"


def test_ur5e_fk_round_trip() -> None:
    chain = _ur5e_like_chain()
    q = np.array([0.0, -1.2, 1.4, -0.7, 0.8, 0.2], dtype=np.float64)

    pose = cartan.forward_kinematics(chain, q)
    matrix = pose.matrix()
    assert matrix.shape == (4, 4)
    assert np.isfinite(matrix).all()

    if hasattr(cartan.SE3, "from_matrix"):
        reconstructed = cartan.SE3.from_matrix(matrix)
        np.testing.assert_allclose(reconstructed.matrix(), matrix, atol=1e-12)


def test_ur5e_iterative_ik_smoke() -> None:
    chain = _ur5e_like_chain()
    q_truth = np.array([0.0, -1.2, 1.4, -0.7, 0.8, 0.2], dtype=np.float64)
    target = cartan.forward_kinematics(chain, q_truth)
    q_seed = q_truth + np.array([0.005, -0.005, 0.003, -0.003, 0.004, -0.004])
    config = cartan.IkConfig(
        max_iterations_per_attempt=200,
        max_total_work_units=400,
        position_tol=1e-10,
        orientation_tol=1e-10,
    )

    result = cartan.solve_ik(chain, target, q_seed, config)

    assert result.converged, f"solve_ik failed with status: {result!r}"
    back = cartan.forward_kinematics(chain, result.q)
    pose_error = float(np.linalg.norm((back.inverse() * target).log()))
    assert pose_error < 1e-8
