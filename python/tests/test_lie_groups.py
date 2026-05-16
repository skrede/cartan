"""Lie group SO(3) and SE(3) numerical invariants."""

from __future__ import annotations

import numpy as np
import pytest
from hypothesis import assume, given, settings
from hypothesis.extra.numpy import arrays
from hypothesis.strategies import floats

import cartan


TOL = 1e-12

# Hypothesis strategies for randomized SO(3)/SE(3) coverage.
# Per-element bounds in [-pi, pi] do NOT prevent norm growth up to pi*sqrt(N);
# tests that need to avoid the antipodal singularity guard with hypothesis.assume.
rotation_vectors = arrays(
    dtype=np.float64,
    shape=(3,),
    elements=floats(
        min_value=-np.pi,
        max_value=np.pi,
        allow_nan=False,
        allow_infinity=False,
    ),
)

twists = arrays(
    dtype=np.float64,
    shape=(6,),
    elements=floats(
        min_value=-np.pi,
        max_value=np.pi,
        allow_nan=False,
        allow_infinity=False,
    ),
)


def test_so3_identity_log_is_zero() -> None:
    R = cartan.SO3.identity()
    assert np.linalg.norm(R.log()) < TOL


def test_so3_exp_log_roundtrip_small() -> None:
    omega = np.array([0.1, 0.2, 0.3])
    R = cartan.SO3.exp(omega)
    assert np.linalg.norm(R.log() - omega) < TOL


def test_so3_exp_log_roundtrip_random() -> None:
    rng = np.random.default_rng(0)
    for _ in range(20):
        omega = rng.uniform(-1.0, 1.0, 3)
        recovered = cartan.SO3.exp(omega).log()
        assert np.linalg.norm(recovered - omega) < 1e-10


def test_so3_matrix_is_orthogonal() -> None:
    omega = np.array([0.7, -0.2, 0.5])
    R = cartan.SO3.exp(omega).matrix()
    assert np.allclose(R @ R.T, np.eye(3), atol=TOL)
    assert pytest.approx(np.linalg.det(R), abs=TOL) == 1.0


def test_se3_identity_log_is_zero() -> None:
    T = cartan.SE3.identity()
    assert np.linalg.norm(T.log()) < TOL


def test_se3_exp_log_roundtrip() -> None:
    xi = np.array([0.1, 0.2, 0.3, 0.5, -0.5, 1.0])
    T = cartan.SE3.exp(xi)
    assert np.linalg.norm(T.log() - xi) < 1e-13


def test_se3_inverse_composes_to_identity() -> None:
    xi = np.array([0.4, -0.1, 0.2, 0.3, 0.5, -0.7])
    T = cartan.SE3.exp(xi)
    identity_twist = (T * T.inverse()).log()
    assert np.linalg.norm(identity_twist) < TOL


def test_se3_matrix_homogeneous_structure() -> None:
    T = cartan.SE3.exp(np.array([0.1, 0.2, 0.3, 1.0, 2.0, 3.0]))
    M = T.matrix()
    assert M.shape == (4, 4)
    assert np.allclose(M[3, :], np.array([0.0, 0.0, 0.0, 1.0]), atol=TOL)


def test_se3_adjoint_shape() -> None:
    T = cartan.SE3.exp(np.array([0.1, 0.2, 0.3, 1.0, 2.0, 3.0]))
    Ad = T.adjoint()
    assert Ad.shape == (6, 6)


def test_se3_act_matches_matrix_multiply() -> None:
    T = cartan.SE3.exp(np.array([0.1, 0.2, 0.3, 1.0, 2.0, 3.0]))
    p = np.array([0.7, -0.5, 0.2])
    via_act = T.act(p)
    via_matrix = (T.matrix() @ np.append(p, 1.0))[:3]
    assert np.linalg.norm(via_act - via_matrix) < TOL


def test_so3_composition_associative() -> None:
    rng = np.random.default_rng(42)
    R1 = cartan.SO3.exp(rng.uniform(-1, 1, 3))
    R2 = cartan.SO3.exp(rng.uniform(-1, 1, 3))
    R3 = cartan.SO3.exp(rng.uniform(-1, 1, 3))
    a = ((R1 * R2) * R3).matrix()
    b = (R1 * (R2 * R3)).matrix()
    assert np.allclose(a, b, atol=1e-13)


# ---------------------------------------------------------------------------
# Hypothesis-driven property tests (bulk coverage of identity invariants).
# ---------------------------------------------------------------------------


@given(omega=rotation_vectors)
@settings(max_examples=100, deadline=None)
def test_so3_exp_log_roundtrip_property(omega: np.ndarray) -> None:
    # The SO(3) log map is antipodal-ambiguous near ||omega|| = pi: both +omega
    # and -omega exponentiate to the same rotation, so log can legitimately
    # return the opposite branch. Skip those examples; the near-pi behavior is
    # covered separately by test_so3_exp_log_near_pi_antipodal.
    assume(np.linalg.norm(omega) < np.pi - 1e-3)
    R = cartan.SO3.exp(omega)
    omega_back = R.log()
    np.testing.assert_allclose(omega_back, omega, atol=1e-6)


@given(omega=rotation_vectors)
@settings(max_examples=100, deadline=None)
def test_so3_inverse_identity_property(omega: np.ndarray) -> None:
    R = cartan.SO3.exp(omega)
    identity = R * R.inverse()
    np.testing.assert_allclose(identity.matrix(), np.eye(3), atol=1e-12)


@given(twist=twists)
@settings(max_examples=100, deadline=None)
def test_se3_exp_log_roundtrip_property(twist: np.ndarray) -> None:
    # SE(3) carries the same antipodal-singularity behavior in its rotational
    # sub-component; guard analogously to the SO(3) property test.
    assume(np.linalg.norm(twist[:3]) < np.pi - 1e-3)
    T = cartan.SE3.exp(twist)
    twist_back = T.log()
    np.testing.assert_allclose(twist_back, twist, atol=1e-6)


def test_so3_exp_log_near_pi_antipodal() -> None:
    # Regression sentinel for the antipodal singularity. At ||omega|| -> pi
    # the log map can return either +omega or -omega; both map back to the
    # same SO(3) under exp. We assert the SO(3) round-trip exp(log(R)) == R
    # at machine precision, which is unambiguous even when the log branch
    # disagrees with the input rotation vector.
    omega = np.array([np.pi - 1e-7, 0.0, 0.0])
    R = cartan.SO3.exp(omega)
    omega_back = R.log()
    R_back = cartan.SO3.exp(omega_back)
    assert np.allclose(R_back.matrix(), R.matrix(), atol=1e-12)
