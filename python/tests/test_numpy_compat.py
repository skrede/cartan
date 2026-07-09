"""NumPy compatibility smoke tests for installed cartan wheels."""

from __future__ import annotations

import numpy as np

import cartan


def _se3_from_rotation_translation(
    rotation: cartan.SO3, translation: np.ndarray,
) -> cartan.SE3:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation.matrix()
    matrix[:3, 3] = translation
    return cartan.SE3.from_matrix(matrix)


def _planar_chain() -> cartan.KinematicChain:
    z = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    axes = [
        cartan.ScrewAxis.revolute(z, np.array([0.0, 0.0, 0.0], dtype=np.float64)),
        cartan.ScrewAxis.revolute(z, np.array([1.0, 0.0, 0.0], dtype=np.float64)),
    ]
    home = _se3_from_rotation_translation(
        cartan.SO3.identity(), np.array([2.0, 0.0, 0.0], dtype=np.float64))
    limits = [cartan.JointLimits(-np.pi, np.pi)] * 2
    return cartan.KinematicChain(home, axes, limits)


def test_numpy_fk_and_jacobians() -> None:
    chain = _planar_chain()
    q = np.array([0.3, -0.4], dtype=np.float64)

    pose = cartan.forward_kinematics(chain, q)
    matrix = pose.matrix()
    space = cartan.space_jacobian(chain, q)
    body = cartan.body_jacobian(chain, q)

    assert matrix.dtype == np.float64
    assert space.dtype == np.float64
    assert body.dtype == np.float64
    assert matrix.shape == (4, 4)
    assert space.shape == (6, 2)
    assert body.shape == (6, 2)
    assert np.isfinite(matrix).all()
    assert np.isfinite(space).all()
    assert np.isfinite(body).all()


def test_numpy_se3_log_returns_analytical_array() -> None:
    twist = np.array([0.1, -0.2, 0.05, 0.3, -0.1, 0.2], dtype=np.float64)
    pose = cartan.SE3.exp(twist)
    recovered = pose.log()

    assert recovered.dtype == np.float64
    assert recovered.shape == (6,)
    np.testing.assert_allclose(recovered, twist, atol=1e-12)


def test_numpy_analytical_planar_solution_arrays() -> None:
    chain = _planar_chain()
    q_truth = np.array([0.35, -0.45], dtype=np.float64)
    target = cartan.forward_kinematics(chain, q_truth)

    result = cartan.analytical.solve_planar_2r(chain, target)

    assert result.status == cartan.AnalyticalStatus.ok
    assert result.solutions
    for q in result.solutions:
        assert q.dtype == np.float64
        assert q.shape == (2,)
        back = cartan.forward_kinematics(chain, q)
        position_error = float(np.linalg.norm(back.translation - target.translation))
        assert position_error < 1e-8
