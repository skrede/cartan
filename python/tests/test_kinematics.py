"""Forward kinematics and Jacobian numerical correctness on bound chains."""

from __future__ import annotations

import numpy as np

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
