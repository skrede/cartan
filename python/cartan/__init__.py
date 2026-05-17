"""Python bindings for cartan — C++23 Lie group, kinematics, and IK library."""

from ._core import (
    SE3,
    SO3,
    IkConfig,
    IkResult,
    IkFailure,
    IkObjective,
    JointLimits,
    ScrewAxis,
    KinematicChain,
    IkTerminationReason,
    __version__,
    analytical,
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
    "IkResult",
    "IkFailure",
    "IkObjective",
    "JointLimits",
    "ScrewAxis",
    "KinematicChain",
    "IkTerminationReason",
    "AnalyticalStatus",
    "AnalyticalResult",
    "__version__",
    "analytical",
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
