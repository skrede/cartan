"""Shared pytest fixtures for the cartan Python test suite."""

from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import pytest

import cartan


REPO_ROOT = Path(__file__).resolve().parent.parent.parent
URDF_DIR = REPO_ROOT / "tests" / "fixtures" / "urdf"


@pytest.fixture(scope="session")
def planar_2r_chain() -> cartan.KinematicChain:
    """Two-link planar chain with revolute joints around z, links of length 1.0."""
    z = np.array([0.0, 0.0, 1.0])
    s1 = cartan.ScrewAxis.revolute(z, np.array([0.0, 0.0, 0.0]))
    s2 = cartan.ScrewAxis.revolute(z, np.array([1.0, 0.0, 0.0]))
    home = cartan.SE3.exp(np.array([0.0, 0.0, 0.0, 2.0, 0.0, 0.0]))
    limits = [cartan.JointLimits(-np.pi, np.pi)] * 2
    return cartan.KinematicChain(home, [s1, s2], limits)


@pytest.fixture(scope="session")
def cartanbot_chain() -> cartan.KinematicChain:
    """Always-on test fixture; six-joint serial arm."""
    if not hasattr(cartan, "load_urdf"):
        pytest.skip("cartan was built without URDF support")
    urdf = URDF_DIR / "cartanbot.urdf"
    if not urdf.exists():
        pytest.skip(f"URDF fixture missing: {urdf}")
    return cartan.load_urdf(str(urdf)).chain


def _maybe_load_extended(filename: str) -> cartan.KinematicChain | None:
    if not hasattr(cartan, "load_urdf"):
        return None
    urdf = URDF_DIR / "extended" / filename
    if not urdf.exists():
        return None
    return cartan.load_urdf(str(urdf)).chain


@pytest.fixture(scope="session")
def ur3e_chain() -> cartan.KinematicChain:
    chain = _maybe_load_extended("ur3e.urdf")
    if chain is None:
        pytest.skip("UR3e URDF fixture not available")
    return chain
