"""URDF loader binding behavior."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

import cartan


URDF_NOT_AVAILABLE = not hasattr(cartan, "load_urdf")
pytestmark = pytest.mark.skipif(URDF_NOT_AVAILABLE, reason="cartan built without URDF")

FIXTURES = Path(__file__).resolve().parent.parent.parent / "tests" / "fixtures" / "urdf"


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


def test_urdf_failure_names_every_kind_the_loader_can_produce() -> None:
    # 18 is the number of enumerators in cartan/urdf/error.h. A kind added there
    # and left unregistered reaches a caller as a value this enum cannot name,
    # which the translator surfaces as a bare ValueError rather than UrdfError.
    assert len(cartan.UrdfFailure.__members__) == 18


@pytest.mark.parametrize(
    ("fixture", "expected"),
    [
        ("adversarial_zero_axis.urdf", "zero_axis"),
        ("adversarial_revolute_no_limit.urdf", "missing_joint_limit"),
        ("adversarial_duplicate_name.urdf", "duplicate_name"),
        ("adversarial_non_finite.urdf", "non_finite_value"),
        ("adversarial_multi_parent.urdf", "multi_parent_link"),
        ("reversed_limit.urdf", "invalid_joint_limit"),
    ],
)
def test_load_urdf_reports_the_expected_failure_kind(fixture: str, expected: str) -> None:
    with pytest.raises(cartan.UrdfError) as excinfo:
        cartan.load_urdf(str(FIXTURES / fixture))
    assert excinfo.value.kind == getattr(cartan.UrdfFailure, expected)


def test_urdf_error_carries_the_readers_code_for_a_parse_failure() -> None:
    with pytest.raises(cartan.UrdfError) as excinfo:
        cartan.load_urdf(str(FIXTURES / "parser_unclosed.urdf"))
    assert excinfo.value.kind == cartan.UrdfFailure.malformed_xml
    assert excinfo.value.meios_code == "xml_parse_error"


def test_urdf_error_has_no_readers_code_for_a_post_read_failure() -> None:
    with pytest.raises(cartan.UrdfError) as excinfo:
        cartan.load_urdf(str(FIXTURES / "extractor_branched.urdf"))
    assert excinfo.value.kind == cartan.UrdfFailure.branched_kinematic_tree
    assert excinfo.value.meios_code is None


def test_xacro_document_loads_and_yields_its_expanded_joints() -> None:
    result = cartan.load_urdf(str(FIXTURES / "xacro_minimal.urdf.xacro"))
    assert result.metadata.joint_names == ["link_1_joint", "tool_joint"]
    assert result.chain.num_joints() == 2


def test_a_clean_description_reports_no_diagnostics() -> None:
    result = cartan.load_urdf(str(FIXTURES / "cartanbot.urdf"))
    assert result.diagnostics == []


def test_an_unresolvable_mesh_is_reported_at_the_warn_tier() -> None:
    urdf = FIXTURES / "extended" / "irb120.urdf"
    if not urdf.exists():
        pytest.skip("IRB120 URDF fixture not available")
    result = cartan.load_urdf(str(urdf))
    unresolved = [d for d in result.diagnostics if d.meios_code == "unresolved_asset"]
    assert unresolved != []
    assert all(d.severity == "warn" for d in unresolved)
    assert all(d.message != "" for d in unresolved)
    assert unresolved[0].location is not None
    assert unresolved[0].location.line > 0


def test_diagnostic_types_are_named_by_the_package_namespace() -> None:
    urdf = FIXTURES / "extended" / "irb120.urdf"
    if not urdf.exists():
        pytest.skip("IRB120 URDF fixture not available")
    diagnostic = cartan.load_urdf(str(urdf)).diagnostics[0]
    assert isinstance(diagnostic, cartan.UrdfDiagnostic)
    assert isinstance(diagnostic.location, cartan.UrdfSourceLocation)
