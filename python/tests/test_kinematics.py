"""Forward kinematics and Jacobian numerical correctness on bound chains."""

from __future__ import annotations

import numpy as np
import pytest

import cartan


TOL = 1e-12


def test_planar_2r_home_at_zero(planar_2r_chain: cartan.KinematicChain) -> None:
    T = cartan.forward_kinematics(planar_2r_chain, np.zeros(2))
    assert np.allclose(T.matrix(), planar_2r_chain.home().matrix(), atol=TOL)


def test_planar_2r_rotated_quarter_turn(planar_2r_chain: cartan.KinematicChain) -> None:
    T = cartan.forward_kinematics(planar_2r_chain, np.array([np.pi / 2, 0.0]))
    assert np.allclose(T.matrix()[:3, 3], np.array([0.0, 2.0, 0.0]), atol=1e-12)


def test_planar_2r_folded_configuration(planar_2r_chain: cartan.KinematicChain) -> None:
    T = cartan.forward_kinematics(planar_2r_chain, np.array([np.pi / 2, -np.pi / 2]))
    assert np.allclose(T.matrix()[:3, 3], np.array([1.0, 1.0, 0.0]), atol=TOL)


def test_planar_2r_fk_matches_manual_poe(planar_2r_chain: cartan.KinematicChain) -> None:
    q1, q2 = 0.7, -0.3
    T_fk = cartan.forward_kinematics(planar_2r_chain, np.array([q1, q2]))
    s1 = planar_2r_chain.axis(0)
    s2 = planar_2r_chain.axis(1)
    T_manual = (
        cartan.SE3.exp(s1.to_vector() * q1)
        * cartan.SE3.exp(s2.to_vector() * q2)
        * planar_2r_chain.home()
    )
    assert np.linalg.norm(T_fk.matrix() - T_manual.matrix()) < TOL


def test_space_jacobian_matches_fk_finite_difference(planar_2r_chain: cartan.KinematicChain) -> None:
    q = np.array([0.3, -0.4])
    Js = cartan.space_jacobian(planar_2r_chain, q)
    assert Js.shape == (6, 2)

    dq = np.array([1e-6, -7e-7])
    T0 = cartan.forward_kinematics(planar_2r_chain, q)
    T1 = cartan.forward_kinematics(planar_2r_chain, q + dq)
    delta = (T1 * T0.inverse()).log()
    fd_error = np.linalg.norm(delta - Js @ dq)
    assert fd_error < 1e-9


def test_body_jacobian_shape_and_finite_difference(planar_2r_chain: cartan.KinematicChain) -> None:
    q = np.array([0.5, 0.8])
    Jb = cartan.body_jacobian(planar_2r_chain, q)
    assert Jb.shape == (6, 2)

    dq = np.array([2e-7, 5e-7])
    T0 = cartan.forward_kinematics(planar_2r_chain, q)
    T1 = cartan.forward_kinematics(planar_2r_chain, q + dq)
    delta = (T0.inverse() * T1).log()
    fd_error = np.linalg.norm(delta - Jb @ dq)
    assert fd_error < 1e-9


def test_ur3e_fk_zero_is_home(ur3e_chain: cartan.KinematicChain) -> None:
    q = np.zeros(ur3e_chain.num_joints())
    T = cartan.forward_kinematics(ur3e_chain, q)
    assert np.allclose(T.matrix(), ur3e_chain.home().matrix(), atol=TOL)


def test_ur3e_jacobian_shape(ur3e_chain: cartan.KinematicChain) -> None:
    rng = np.random.default_rng(0)
    q = rng.uniform(-1.0, 1.0, ur3e_chain.num_joints())
    Js = cartan.space_jacobian(ur3e_chain, q)
    assert Js.shape == (6, ur3e_chain.num_joints())
    assert Js.dtype == np.float64


# ---------------------------------------------------------------------------
# Layout sweep: confirms the FK/Jacobian bindings accept non-contiguous
# joint-vector views (the binding contract is nb::DRef<const Eigen::VectorXd>).
# ---------------------------------------------------------------------------


@pytest.fixture
def q_base(cartanbot_chain: cartan.KinematicChain) -> np.ndarray:
    n = cartanbot_chain.num_joints()
    return np.linspace(-0.1, 0.6, n, dtype=np.float64)


@pytest.fixture(params=["contiguous_c", "transposed_view", "strided_view"])
def q_layout(request: pytest.FixtureRequest, q_base: np.ndarray) -> np.ndarray:
    n = q_base.shape[0]
    if request.param == "contiguous_c":
        return q_base.copy()
    if request.param == "transposed_view":
        # (1, n) transposed -> (n, 1) view -> column 0 -> non-contiguous (n,).
        return q_base.reshape(1, -1).T[:, 0]
    if request.param == "strided_view":
        big = np.empty(2 * n, dtype=np.float64)
        big[0::2] = q_base
        return big[0::2]
    raise ValueError(f"unhandled layout param: {request.param}")


def test_fk_layout_invariant(
    cartanbot_chain: cartan.KinematicChain,
    q_layout: np.ndarray,
    q_base: np.ndarray,
) -> None:
    baseline = cartan.forward_kinematics(cartanbot_chain, q_base.copy())
    result = cartan.forward_kinematics(cartanbot_chain, q_layout)
    np.testing.assert_allclose(result.matrix(), baseline.matrix(), atol=1e-15)


def test_space_jacobian_layout_invariant(
    cartanbot_chain: cartan.KinematicChain,
    q_layout: np.ndarray,
    q_base: np.ndarray,
) -> None:
    baseline = cartan.space_jacobian(cartanbot_chain, q_base.copy())
    result = cartan.space_jacobian(cartanbot_chain, q_layout)
    np.testing.assert_allclose(result, baseline, atol=1e-15)


def test_body_jacobian_layout_invariant(
    cartanbot_chain: cartan.KinematicChain,
    q_layout: np.ndarray,
    q_base: np.ndarray,
) -> None:
    baseline = cartan.body_jacobian(cartanbot_chain, q_base.copy())
    result = cartan.body_jacobian(cartanbot_chain, q_layout)
    np.testing.assert_allclose(result, baseline, atol=1e-15)


# ---------------------------------------------------------------------------
# Dtype-mismatch negative tests: confirms noconvert() raises TypeError on
# float32 input (the binding contract requires float64).
# ---------------------------------------------------------------------------


def test_fk_dtype_mismatch_raises(cartanbot_chain: cartan.KinematicChain) -> None:
    q32 = np.zeros(cartanbot_chain.num_joints(), dtype=np.float32)
    with pytest.raises(TypeError):
        cartan.forward_kinematics(cartanbot_chain, q32)


def test_space_jacobian_dtype_mismatch_raises(
    cartanbot_chain: cartan.KinematicChain,
) -> None:
    q32 = np.zeros(cartanbot_chain.num_joints(), dtype=np.float32)
    with pytest.raises(TypeError):
        cartan.space_jacobian(cartanbot_chain, q32)


def test_body_jacobian_dtype_mismatch_raises(
    cartanbot_chain: cartan.KinematicChain,
) -> None:
    q32 = np.zeros(cartanbot_chain.num_joints(), dtype=np.float32)
    with pytest.raises(TypeError):
        cartan.body_jacobian(cartanbot_chain, q32)


# ---------------------------------------------------------------------------
# FFI memory-safety negative tests: raw integer indices and constructor
# argument lengths cross the binding boundary into native container access.
# An out-of-range axis index must raise a Python exception rather than read
# out of bounds, and a size-mismatched chain must be rejected even in a
# Release (-DNDEBUG) build where debug assertions are compiled out.
# ---------------------------------------------------------------------------


def _make_planar_2r() -> cartan.KinematicChain:
    z = np.array([0.0, 0.0, 1.0])
    s1 = cartan.ScrewAxis.revolute(z, np.array([0.0, 0.0, 0.0]))
    s2 = cartan.ScrewAxis.revolute(z, np.array([1.0, 0.0, 0.0]))
    home = cartan.SE3.exp(np.array([0.0, 0.0, 0.0, 2.0, 0.0, 0.0]))
    limits = [cartan.JointLimits(-np.pi, np.pi)] * 2
    return cartan.KinematicChain(home, [s1, s2], limits)


def test_axis_negative_index_raises(planar_2r_chain: cartan.KinematicChain) -> None:
    with pytest.raises(IndexError):
        planar_2r_chain.axis(-1)


def test_axis_oversized_index_raises(planar_2r_chain: cartan.KinematicChain) -> None:
    n = planar_2r_chain.num_joints()
    with pytest.raises(IndexError):
        planar_2r_chain.axis(n)


def test_axis_far_out_of_bounds_index_raises(
    planar_2r_chain: cartan.KinematicChain,
) -> None:
    with pytest.raises(IndexError):
        planar_2r_chain.axis(1_000_000)


def test_axis_valid_indices_still_work(
    planar_2r_chain: cartan.KinematicChain,
) -> None:
    # The bounds guard must not break the valid path (0 <= i < num_joints).
    for i in range(planar_2r_chain.num_joints()):
        axis = planar_2r_chain.axis(i)
        assert axis.to_vector().shape == (6,)


def test_kinematic_chain_mismatched_sizes_raises() -> None:
    # Two axes but a single joint-limit entry: the size invariant must be
    # enforced with a real runtime check that survives -DNDEBUG, not a debug
    # assert that is elided from a shipped Release wheel.
    z = np.array([0.0, 0.0, 1.0])
    s1 = cartan.ScrewAxis.revolute(z, np.array([0.0, 0.0, 0.0]))
    s2 = cartan.ScrewAxis.revolute(z, np.array([1.0, 0.0, 0.0]))
    home = cartan.SE3.exp(np.array([0.0, 0.0, 0.0, 2.0, 0.0, 0.0]))
    with pytest.raises((ValueError, RuntimeError)):
        cartan.KinematicChain(home, [s1, s2], [cartan.JointLimits(-1.0, 1.0)])


def test_kinematic_chain_matched_sizes_still_construct() -> None:
    # The ctor validation must not reject a well-formed chain.
    chain = _make_planar_2r()
    assert chain.num_joints() == 2
