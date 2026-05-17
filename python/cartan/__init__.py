"""Python bindings for cartan — C++23 Lie group, kinematics, and IK library."""

from ._core import (
    SE3,
    SO3,
    IkConfig,
    IkPolicy,
    IkResult,
    IkFailure,
    IkObjective,
    JointLimits,
    ScrewAxis,
    KinematicChain,
    RankingStrategy,
    IkTerminationReason,
    ExhaustiveIKRunner,
    __version__,
    analytical,
    has_argmin,
    solve_ik,
    body_jacobian,
    solve_ik_speed,
    solve_ik_robust,
    space_jacobian,
    forward_kinematics,
)

AnalyticalStatus = analytical.AnalyticalStatus
AnalyticalResult = analytical.AnalyticalResult

__all__ = [
    "SE3",
    "SO3",
    "IkConfig",
    "IkPolicy",
    "IkResult",
    "IkFailure",
    "IkObjective",
    "JointLimits",
    "ScrewAxis",
    "KinematicChain",
    "RankingStrategy",
    "IkTerminationReason",
    "ExhaustiveIKRunner",
    "AnalyticalStatus",
    "AnalyticalResult",
    "__version__",
    "analytical",
    "has_argmin",
    "solve_ik",
    "body_jacobian",
    "solve_ik_speed",
    "solve_ik_robust",
    "space_jacobian",
    "forward_kinematics",
]

try:
    from ._core import (
        UrdfError,
        UrdfFailure,
        UrdfLoadResult,
        UrdfMetadata,
        load_urdf,
    )

    __all__ += [
        "UrdfError",
        "UrdfFailure",
        "UrdfLoadResult",
        "UrdfMetadata",
        "load_urdf",
    ]
except ImportError:
    pass

# argmin-backed iterative IK solvers, gated on CARTAN_BUILD_ARGMIN=ON at
# build time. When the extension was built without argmin, these symbols
# are absent from _core; cartan.has_argmin == False and accessing
# solve_ik_argmin_* raises AttributeError (the Pythonic convention for
# build-time-optional features). The gate on has_argmin is intentionally
# checked before the import so the AttributeError surface stays on the
# package boundary rather than the underlying extension.
if has_argmin:
    from ._core import (
        solve_ik_argmin_slsqp,
        solve_ik_argmin_lm,
        solve_ik_argmin_lbfgsb,
    )

    __all__ += [
        "solve_ik_argmin_slsqp",
        "solve_ik_argmin_lm",
        "solve_ik_argmin_lbfgsb",
    ]
