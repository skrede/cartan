"""Python bindings for cartan — C++23 Lie group, kinematics, and IK library."""

from ._core import (
    SE3,
    SO3,
    JointLimits,
    KinematicChain,
    ScrewAxis,
    __version__,
    body_jacobian,
    forward_kinematics,
    space_jacobian,
)

__all__ = [
    "SE3",
    "SO3",
    "JointLimits",
    "KinematicChain",
    "ScrewAxis",
    "__version__",
    "body_jacobian",
    "forward_kinematics",
    "space_jacobian",
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
