"""Bound entry points reject exactly what the C++ boundary rejects.

The stale-extension guard lives in conftest.py, so every entry path into the
suite passes through it rather than only this module. What is asserted here is
the boundary itself: the exception class, the message that identifies which
check fired, and the layout contract the checks must not break.
"""

from __future__ import annotations

import hashlib
import re
from collections.abc import Callable
from pathlib import Path

import numpy as np
import pytest

import cartan
import cartan._core as _core


CORE_PATH = Path(_core.__file__).resolve()
CORE_SHA256 = hashlib.sha256(CORE_PATH.read_bytes()).hexdigest()

# The failure's own message, as chain_failure::message() spells it. Asserting
# the text is what distinguishes the per-site pre-unwrap from the generic
# caster's fallback, and one check firing from the other.
DIMENSION_MESSAGE = "Vector length does not match the chain's joint count"
NONFINITE_MESSAGE = "Input contains a NaN or infinite component"

ENTRY_POINTS = [cartan.forward_kinematics, cartan.space_jacobian, cartan.body_jacobian]
SEED_SOLVERS = [cartan.solve_ik, cartan.solve_ik_speed, cartan.solve_ik_robust]
LIMIT_ATTRIBUTES = [
    "position_min",
    "position_max",
    "velocity_max",
    "effort_max",
    "acceleration_max",
]
NONFINITE = [np.nan, np.inf, -np.inf]

EntryPoint = Callable[..., object]


@pytest.fixture(params=ENTRY_POINTS, ids=lambda fn: fn.__name__)
def entry_point(request: pytest.FixtureRequest) -> EntryPoint:
    return request.param


@pytest.fixture(scope="session")
def planar_2r_target(planar_2r_chain: cartan.KinematicChain) -> cartan.SE3:
    return cartan.forward_kinematics(planar_2r_chain, np.array([0.4, -0.7]))


@pytest.fixture(scope="session")
def planar_2r_solutions(
    planar_2r_chain: cartan.KinematicChain, planar_2r_target: cartan.SE3
) -> cartan.AnalyticalResult:
    result = cartan.analytical.solve_planar_2r(planar_2r_chain, planar_2r_target)
    assert result.solutions, "fixture needs a solved result to rank"
    return result


def test_extension_provenance_is_reported() -> None:
    print(f"asserted against {CORE_PATH} sha256 {CORE_SHA256}")
    assert CORE_PATH.is_file()
    assert re.fullmatch(r"[0-9a-f]{64}", _core.__source_digest__)


@pytest.mark.parametrize("length", [0, 1, 3], ids=["empty", "one_short", "one_long"])
def test_entry_point_rejects_a_mis_sized_joint_vector(
    entry_point: EntryPoint, planar_2r_chain: cartan.KinematicChain, length: int
) -> None:
    with pytest.raises(ValueError, match=re.escape(DIMENSION_MESSAGE)):
        entry_point(planar_2r_chain, np.zeros(length))


@pytest.mark.parametrize("value", NONFINITE, ids=["nan", "posinf", "neginf"])
@pytest.mark.parametrize("index", [0, 1])
def test_entry_point_rejects_a_nonfinite_joint_value(
    entry_point: EntryPoint,
    planar_2r_chain: cartan.KinematicChain,
    value: float,
    index: int,
) -> None:
    q = np.zeros(2)
    q[index] = value
    with pytest.raises(ValueError, match=re.escape(NONFINITE_MESSAGE)):
        entry_point(planar_2r_chain, q)


def test_rejection_carries_the_boundary_message_not_the_caster_fallback(
    entry_point: EntryPoint, planar_2r_chain: cartan.KinematicChain
) -> None:
    # The generic expected caster raises RuntimeError carrying its own text.
    # Equality against the failure's own message is what proves the per-site
    # pre-unwrap was the path taken.
    with pytest.raises(ValueError) as excinfo:
        entry_point(planar_2r_chain, np.zeros(5))
    assert str(excinfo.value) == DIMENSION_MESSAGE


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
    if request.param == "contiguous":
        return np.array([0.3, -0.4])
    if request.param == "transposed":
        # Column 0 of a genuine 2-D transpose: stride 16, not contiguous.
        return np.array([[0.3, 9.0], [-0.4, 9.0]]).T[0]
    strided = np.empty(4)
    strided[0::2] = np.array([0.3, -0.4])
    return strided[0::2]


def test_validation_preserves_the_layout_contract(
    entry_point: EntryPoint, planar_2r_chain: cartan.KinematicChain, q_view: np.ndarray
) -> None:
    assert q_view.tolist() == [0.3, -0.4]
    baseline = entry_point(planar_2r_chain, np.array([0.3, -0.4]))
    result = entry_point(planar_2r_chain, q_view)
    if entry_point is cartan.forward_kinematics:
        np.testing.assert_allclose(result.matrix(), baseline.matrix(), atol=1e-15)
    else:
        np.testing.assert_allclose(result, baseline, atol=1e-15)


# ---------------------------------------------------------------------------
# The seed-taking surface: every bound entry point that accepts a joint vector,
# not only the three that compute kinematics from one.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("solver", SEED_SOLVERS, ids=lambda fn: fn.__name__)
@pytest.mark.parametrize("value", NONFINITE, ids=["nan", "posinf", "neginf"])
def test_iterative_solver_rejects_a_nonfinite_seed(
    solver: EntryPoint,
    planar_2r_chain: cartan.KinematicChain,
    planar_2r_target: cartan.SE3,
    value: float,
) -> None:
    with pytest.raises(ValueError, match="non-finite"):
        solver(planar_2r_chain, planar_2r_target, np.array([value, 0.0]))


@pytest.mark.parametrize("solver", SEED_SOLVERS, ids=lambda fn: fn.__name__)
def test_iterative_solver_rejects_a_mis_sized_seed(
    solver: EntryPoint,
    planar_2r_chain: cartan.KinematicChain,
    planar_2r_target: cartan.SE3,
) -> None:
    with pytest.raises(ValueError, match="q_seed.size"):
        solver(planar_2r_chain, planar_2r_target, np.zeros(5))


def test_exhaustive_runner_rejects_a_nonfinite_seed(
    planar_2r_chain: cartan.KinematicChain, planar_2r_target: cartan.SE3
) -> None:
    runner = cartan.ExhaustiveIKRunner(planar_2r_chain, policy=cartan.IkPolicy.speed)
    with pytest.raises(ValueError, match="non-finite"):
        runner.solve(planar_2r_target, q_seed=np.array([np.nan, 0.0]))


def test_verify_solution_rejects_a_nonfinite_joint_vector(
    planar_2r_chain: cartan.KinematicChain, planar_2r_target: cartan.SE3
) -> None:
    with pytest.raises(ValueError, match="non-finite"):
        cartan.analytical.verify_solution(
            planar_2r_chain, np.array([np.nan, 0.0]), planar_2r_target
        )


@pytest.mark.parametrize("value", NONFINITE, ids=["nan", "posinf", "neginf"])
def test_closest_to_seed_rejects_a_nonfinite_seed(
    planar_2r_solutions: cartan.AnalyticalResult, value: float
) -> None:
    with pytest.raises(ValueError, match="non-finite"):
        cartan.analytical.closest_to_seed(planar_2r_solutions, np.array([value, 0.0]))


@pytest.mark.parametrize("length", [0, 1, 3], ids=["empty", "one_short", "one_long"])
def test_closest_to_seed_rejects_a_mis_sized_seed(
    planar_2r_solutions: cartan.AnalyticalResult, length: int
) -> None:
    # A mis-sized seed made every distance +inf, so min_element found no strict
    # ordering and returned the first branch -- a wrong answer, not an error.
    with pytest.raises(ValueError, match="q_seed.size"):
        cartan.analytical.closest_to_seed(planar_2r_solutions, np.zeros(length))


def test_closest_to_seed_still_ranks_a_well_formed_seed(
    planar_2r_solutions: cartan.AnalyticalResult,
) -> None:
    seed = np.array([0.4, -0.7])
    closest = cartan.analytical.closest_to_seed(planar_2r_solutions, seed)
    assert closest is not None
    distances = [float(np.linalg.norm(q - seed)) for q in planar_2r_solutions.solutions]
    assert float(np.linalg.norm(closest - seed)) == pytest.approx(min(distances))


def test_solve_all_rejects_a_mis_sized_seed(
    planar_2r_chain: cartan.KinematicChain, planar_2r_target: cartan.SE3
) -> None:
    with pytest.raises(ValueError, match="q_seed.size"):
        cartan.analytical.solve_all(planar_2r_chain, planar_2r_target, q_seed=np.zeros(7))


@pytest.mark.parametrize("value", NONFINITE, ids=["nan", "posinf", "neginf"])
def test_solve_all_rejects_a_nonfinite_seed(
    planar_2r_chain: cartan.KinematicChain, planar_2r_target: cartan.SE3, value: float
) -> None:
    with pytest.raises(ValueError, match="non-finite"):
        cartan.analytical.solve_all(
            planar_2r_chain, planar_2r_target, q_seed=np.array([value, 0.0])
        )


def test_solve_all_still_ranks_a_well_formed_seed(
    planar_2r_chain: cartan.KinematicChain, planar_2r_target: cartan.SE3
) -> None:
    ranked = cartan.analytical.solve_all(
        planar_2r_chain, planar_2r_target, q_seed=np.array([0.4, -0.7])
    )
    assert ranked.status == cartan.AnalyticalStatus.ok
    assert ranked.solutions


@pytest.mark.parametrize("value", NONFINITE, ids=["nan", "posinf", "neginf"])
def test_unwrapped_solver_rejects_a_nonfinite_seed(
    planar_2r_chain: cartan.KinematicChain, planar_2r_target: cartan.SE3, value: float
) -> None:
    with pytest.raises(ValueError, match="non-finite"):
        cartan.analytical.solve_unwrapped_planar_2r(
            planar_2r_chain, planar_2r_target, q_seed=np.array([value, 0.0])
        )


# ---------------------------------------------------------------------------
# JointLimits: validated construction, and five attributes that cannot be
# written back into a state the factory refused.
# ---------------------------------------------------------------------------


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
        ((-1.0, 1.0), {"velocity_max": np.inf}),
        ((-1.0, 1.0), {"effort_max": np.nan}),
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
