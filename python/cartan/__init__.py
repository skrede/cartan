"""Python bindings for cartan — C++20 Lie group, kinematics, and IK library."""

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
    ExhaustiveIKRunner,
    __version__,
    analytical,
    isotropy,
    solve_ik,
    body_jacobian,
    solve_ik_speed,
    solve_ik_robust,
    space_jacobian,
    manipulability,
    singular_values,
    condition_number,
    is_near_singular,
    forward_kinematics,
)

AnalyticalStatus = analytical.AnalyticalStatus
AnalyticalResult = analytical.AnalyticalResult
OPWParameters = analytical.OPWParameters
OPWBranch = analytical.OPWBranch
RangeStatus = analytical.RangeStatus
UnwrappedResult = analytical.UnwrappedResult

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
    "ExhaustiveIKRunner",
    "AnalyticalStatus",
    "AnalyticalResult",
    "OPWParameters",
    "OPWBranch",
    "RangeStatus",
    "UnwrappedResult",
    "__version__",
    "analytical",
    "isotropy",
    "solve_ik",
    "body_jacobian",
    "solve_ik_speed",
    "solve_ik_robust",
    "space_jacobian",
    "manipulability",
    "singular_values",
    "condition_number",
    "is_near_singular",
    "forward_kinematics",
]

try:
    from ._core import (
        UrdfDiagnostic,
        UrdfError,
        UrdfFailure,
        UrdfLoadResult,
        UrdfMetadata,
        UrdfSourceLocation,
        load_urdf,
        load_urdf_transform,
    )

    __all__ += [
        "UrdfDiagnostic",
        "UrdfError",
        "UrdfFailure",
        "UrdfLoadResult",
        "UrdfMetadata",
        "UrdfSourceLocation",
        "load_urdf",
        "load_urdf_transform",
    ]
except ImportError:
    pass
