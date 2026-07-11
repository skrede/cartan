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


def test_load_urdf_raises_typed_error_on_missing_file() -> None:
    with pytest.raises(cartan.UrdfError) as excinfo:
        cartan.load_urdf("/nonexistent.urdf")
    assert isinstance(excinfo.value.kind, cartan.UrdfFailure)
    assert isinstance(excinfo.value.detail, str)
    assert excinfo.value.detail != ""


def test_urdf_error_is_subclass_of_runtime_error() -> None:
    assert issubclass(cartan.UrdfError, RuntimeError)


def test_urdf_metadata_joint_index_happy_path() -> None:
    repo_root = Path(__file__).resolve().parent.parent.parent
    result = cartan.load_urdf(str(repo_root / "tests" / "fixtures" / "urdf" / "cartanbot.urdf"))
    meta = result.metadata
    for i, name in enumerate(meta.joint_names):
        assert meta.joint_index(name) == i


def test_urdf_metadata_joint_index_raises_keyerror_on_unknown() -> None:
    repo_root = Path(__file__).resolve().parent.parent.parent
    result = cartan.load_urdf(str(repo_root / "tests" / "fixtures" / "urdf" / "cartanbot.urdf"))
    meta = result.metadata
    with pytest.raises(KeyError):
        meta.joint_index("no_such_joint_42")


def test_urdf_numeric_values_parse_correctly_with_numpy_present() -> None:
    repo_root = Path(__file__).resolve().parent.parent.parent
    chain = cartan.load_urdf(
        str(repo_root / "tests" / "fixtures" / "urdf" / "cartanbot.urdf")
    ).chain
    assert chain.num_joints() == 6
    home = cartan.forward_kinematics(chain, np.zeros(chain.num_joints()))
    np.testing.assert_allclose(home.translation, [0.0, 0.05, 1.40], atol=1e-12)


def test_irb120_axes_parse_correctly_with_numpy_present(
    irb120_chain: cartan.KinematicChain,
) -> None:
    assert irb120_chain.num_joints() == 6
    expected = [[0, 0, 1], [0, 1, 0], [0, 1, 0], [1, 0, 0], [0, 1, 0], [1, 0, 0]]
    for i, omega in enumerate(expected):
        np.testing.assert_allclose(irb120_chain.axis(i).omega(), omega, atol=1e-12)


def test_ur3e_loads_and_fk_is_reproducible(ur3e_chain: cartan.KinematicChain) -> None:
    rng = np.random.default_rng(7)
    q = rng.uniform(-1.0, 1.0, ur3e_chain.num_joints())
    T1 = cartan.forward_kinematics(ur3e_chain, q).matrix()
    T2 = cartan.forward_kinematics(ur3e_chain, q).matrix()
    assert np.array_equal(T1, T2)
