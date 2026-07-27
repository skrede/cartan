"""Bound entry points reject exactly what the C++ boundary rejects.

The suite refuses to run against a stale compiled extension. A non-rebuilding
editable install lets `import cartan` resolve to a `_core` built weeks ago, so
every assertion below would pass against code nobody changed. The module-level
guard prints the loaded path, its content hash and its mtime, and fails the
collection when the extension is older than the binding sources it was built
from, or when it lies outside CARTAN_CORE_EXPECTED_PREFIX if that is set.
"""

from __future__ import annotations

import hashlib
import os
from collections.abc import Callable
from pathlib import Path

import numpy as np
import pytest

import cartan
import cartan._core as _core


BINDING_SOURCES = Path(__file__).resolve().parents[1] / "src"

CORE_PATH = Path(_core.__file__).resolve()
CORE_SHA256 = hashlib.sha256(CORE_PATH.read_bytes()).hexdigest()
CORE_MTIME = CORE_PATH.stat().st_mtime


def _newest_binding_source() -> tuple[Path, float] | None:
    sources = [p for p in BINDING_SOURCES.rglob("*") if p.suffix in (".cpp", ".h")]
    if not sources:
        return None
    newest = max(sources, key=lambda p: p.stat().st_mtime)
    return newest, newest.stat().st_mtime


def _guard_against_a_stale_extension() -> None:
    print(f"cartan._core: {CORE_PATH}")
    print(f"sha256: {CORE_SHA256}")
    print(f"mtime: {CORE_MTIME}")
    prefix = os.environ.get("CARTAN_CORE_EXPECTED_PREFIX")
    if prefix and not str(CORE_PATH).startswith(prefix):
        raise RuntimeError(f"{CORE_PATH} lies outside the expected prefix {prefix}")
    newest = _newest_binding_source()
    if newest is not None and newest[1] > CORE_MTIME:
        raise RuntimeError(
            f"{CORE_PATH} predates the binding source {newest[0]}; rebuild before testing"
        )


_guard_against_a_stale_extension()

ENTRY_POINTS = [cartan.forward_kinematics, cartan.space_jacobian, cartan.body_jacobian]
LIMIT_ATTRIBUTES = [
    "position_min",
    "position_max",
    "velocity_max",
    "effort_max",
    "acceleration_max",
]


EntryPoint = Callable[..., object]


@pytest.fixture(params=ENTRY_POINTS, ids=lambda fn: fn.__name__)
def entry_point(request: pytest.FixtureRequest) -> EntryPoint:
    return request.param


def test_extension_provenance_is_reported() -> None:
    print(f"asserted against {CORE_PATH} sha256 {CORE_SHA256}")
    assert CORE_PATH.is_file()
    assert len(CORE_SHA256) == 64


@pytest.mark.parametrize("length", [0, 1, 3], ids=["empty", "one_short", "one_long"])
def test_entry_point_rejects_a_mis_sized_joint_vector(
    entry_point: EntryPoint, planar_2r_chain: cartan.KinematicChain, length: int
) -> None:
    with pytest.raises(ValueError) as excinfo:
        entry_point(planar_2r_chain, np.zeros(length))
    assert str(excinfo.value)


@pytest.mark.parametrize("value", [np.nan, np.inf, -np.inf], ids=["nan", "posinf", "neginf"])
@pytest.mark.parametrize("index", [0, 1])
def test_entry_point_rejects_a_nonfinite_joint_value(
    entry_point: EntryPoint, planar_2r_chain: cartan.KinematicChain, value: float, index: int
) -> None:
    q = np.zeros(2)
    q[index] = value
    with pytest.raises(ValueError) as excinfo:
        entry_point(planar_2r_chain, q)
    assert str(excinfo.value)


def test_rejection_is_a_value_error_and_not_a_runtime_error(
    entry_point: EntryPoint, planar_2r_chain: cartan.KinematicChain
) -> None:
    # The generic expected caster raises RuntimeError; the per-site unwrap
    # exists so a boundary violation does not arrive through it.
    with pytest.raises(ValueError) as excinfo:
        entry_point(planar_2r_chain, np.zeros(5))
    assert not isinstance(excinfo.value, RuntimeError)


def test_entry_point_accepts_a_well_formed_joint_vector(
    entry_point: EntryPoint, planar_2r_chain: cartan.KinematicChain
) -> None:
    result = entry_point(planar_2r_chain, np.array([0.3, -0.4]))
    if entry_point is cartan.forward_kinematics:
        assert result.matrix().shape == (4, 4)
    else:
        assert result.shape == (6, 2)


@pytest.fixture(params=["contiguous", "transposed", "strided"])
def q_view(request: pytest.FixtureRequest) -> np.ndarray:
    q = np.array([0.3, -0.4])
    if request.param == "contiguous":
        return q
    if request.param == "transposed":
        return q.reshape(1, -1).T[:, 0]
    strided = np.empty(4)
    strided[0::2] = q
    return strided[0::2]


def test_validation_preserves_the_layout_contract(
    entry_point: EntryPoint, planar_2r_chain: cartan.KinematicChain, q_view: np.ndarray
) -> None:
    baseline = entry_point(planar_2r_chain, np.array([0.3, -0.4]))
    result = entry_point(planar_2r_chain, q_view)
    if entry_point is cartan.forward_kinematics:
        np.testing.assert_allclose(result.matrix(), baseline.matrix(), atol=1e-15)
    else:
        np.testing.assert_allclose(result, baseline, atol=1e-15)


@pytest.mark.parametrize(
    ("args", "kwargs"),
    [
        ((1.0, -1.0), {}),
        ((np.inf, np.inf), {}),
        ((-np.inf, -np.inf), {}),
        ((-1.0, np.nan), {}),
        ((-1.0, 1.0), {"velocity_max": -1.0}),
        ((-1.0, 1.0), {"effort_max": -1.0}),
        ((-1.0, 1.0), {"acceleration_max": -1.0}),
    ],
)
def test_joint_limits_rejects_what_the_factory_rejects(
    args: tuple[float, float], kwargs: dict[str, float]
) -> None:
    with pytest.raises(ValueError) as excinfo:
        cartan.JointLimits(*args, **kwargs)
    assert str(excinfo.value)


def test_joint_limits_accepts_the_unbounded_pair() -> None:
    limits = cartan.JointLimits(-np.inf, np.inf)
    assert limits.position_min == -np.inf
    assert limits.position_max == np.inf


@pytest.mark.parametrize("attribute", LIMIT_ATTRIBUTES)
def test_joint_limit_attributes_reject_assignment(attribute: str) -> None:
    limits = cartan.JointLimits(-1.0, 1.0, velocity_max=2.0)
    with pytest.raises(AttributeError):
        setattr(limits, attribute, 0.0)


def test_joint_limit_attributes_still_read_back() -> None:
    limits = cartan.JointLimits(
        -1.5, 2.5, velocity_max=2.0, effort_max=5.0, acceleration_max=10.0
    )
    assert [getattr(limits, name) for name in LIMIT_ATTRIBUTES] == [
        -1.5,
        2.5,
        2.0,
        5.0,
        10.0,
    ]
