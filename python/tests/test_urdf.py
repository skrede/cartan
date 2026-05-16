"""URDF loader binding behavior."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

import cartan


URDF_NOT_AVAILABLE = not hasattr(cartan, "load_urdf")
pytestmark = pytest.mark.skipif(URDF_NOT_AVAILABLE, reason="cartan built without URDF")


def test_cartanbot_loads_with_expected_topology(cartanbot_chain: cartan.KinematicChain) -> None:
    assert cartanbot_chain.num_joints() == 6


def test_cartanbot_metadata_carries_joint_names() -> None:
    repo_root = Path(__file__).resolve().parent.parent.parent
    result = cartan.load_urdf(str(repo_root / "tests" / "fixtures" / "urdf" / "cartanbot.urdf"))
    assert result.metadata.base_link_name == "base_link"
    assert result.metadata.tool_link_name == "tool0"
    assert result.metadata.joint_names == [
        f"joint{i}" for i in range(1, 7)
    ]


def test_load_urdf_raises_runtime_error_on_missing_file() -> None:
    with pytest.raises(RuntimeError, match="cartan.load_urdf failed"):
        cartan.load_urdf("/nonexistent.urdf")


def test_ur3e_loads_and_fk_is_reproducible(ur3e_chain: cartan.KinematicChain) -> None:
    rng = np.random.default_rng(7)
    q = rng.uniform(-1.0, 1.0, ur3e_chain.num_joints())
    T1 = cartan.forward_kinematics(ur3e_chain, q).matrix()
    T2 = cartan.forward_kinematics(ur3e_chain, q).matrix()
    assert np.array_equal(T1, T2)
