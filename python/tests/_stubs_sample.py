"""Exercise every public binding so type-checkers see the full surface.

This module is consumed by test_stubs.py, which runs pyright and mypy in
strict mode against it as a subprocess. It is also executable directly via
`python python/tests/_stubs_sample.py` as a lightweight sanity check.

The file-level `# pyright: strict` marker enables pyright's strict type
checking; recent pyright versions (>= 1.1.400) dropped the `--strict` CLI
flag in favor of file-level or config-level enablement. mypy is run with
its `--strict` CLI flag, which has been stable for many releases.
"""

# pyright: strict

from __future__ import annotations

import numpy as np

import cartan


# ---------------------------------------------------------------------------
# Lie group surface (SO3, SE3)
# ---------------------------------------------------------------------------

R: cartan.SO3 = cartan.SO3.identity()
omega = np.zeros(3, dtype=np.float64)
R2: cartan.SO3 = cartan.SO3.exp(omega)
R_inv: cartan.SO3 = R2.inverse()
R_back = R2.log()
M_rot = R2.matrix()
adj_R = R2.adjoint()
R_from: cartan.SO3 = cartan.SO3.from_matrix(np.eye(3, dtype=np.float64))
p3 = np.zeros(3, dtype=np.float64)
p3_rot = R2.act(p3)

T: cartan.SE3 = cartan.SE3.identity()
twist = np.zeros(6, dtype=np.float64)
T2: cartan.SE3 = cartan.SE3.exp(twist)
T_inv: cartan.SE3 = T2.inverse()
twist_back = T2.log()
M_T = T2.matrix()
adj_T = T2.adjoint()
trans = T2.translation
rot: cartan.SO3 = T2.rotation
T_from: cartan.SE3 = cartan.SE3.from_matrix(np.eye(4, dtype=np.float64))
p3_xf = T2.act(p3)

# ---------------------------------------------------------------------------
# Chain surface (ScrewAxis, JointLimits, KinematicChain)
# ---------------------------------------------------------------------------

axis_z = np.array([0.0, 0.0, 1.0], dtype=np.float64)
origin = np.zeros(3, dtype=np.float64)
s: cartan.ScrewAxis = cartan.ScrewAxis.revolute(axis_z, origin)
s_prismatic: cartan.ScrewAxis = cartan.ScrewAxis.prismatic(axis_z)
s_from: cartan.ScrewAxis = cartan.ScrewAxis.from_vector(s.to_vector())
omega_s = s.omega()
v_s = s.v()
six_vec = s.to_vector()
is_rev: bool = s.is_revolute()

lim: cartan.JointLimits = cartan.JointLimits(
    -3.14, 3.14, velocity_max=2.0, effort_max=5.0, acceleration_max=10.0
)
lim_min: float = lim.position_min
lim_max: float = lim.position_max
in_range: bool = lim.contains(0.0)

chain: cartan.KinematicChain = cartan.KinematicChain(T2, [s], [lim])
home_pose: cartan.SE3 = chain.home()
n_joints: int = chain.num_joints()
axis0: cartan.ScrewAxis = chain.axis(0)

# ---------------------------------------------------------------------------
# FK / Jacobian surface
# ---------------------------------------------------------------------------

q = np.zeros(n_joints, dtype=np.float64)
T_ee: cartan.SE3 = cartan.forward_kinematics(chain, q)
Js = cartan.space_jacobian(chain, q)
Jb = cartan.body_jacobian(chain, q)

# ---------------------------------------------------------------------------
# URDF surface
#
# UrdfError is registered via the raw CPython type-object API (PyType_FromSpec
# + PyModule_AddObject) so that it can subclass RuntimeError; that path is
# invisible to nanobind's automatic stub generator and the class does not
# appear in _core.pyi. The narrow mitigation here is to import UrdfError via
# cartan._core and explicitly cast its type for the strict checkers (the
# class is unambiguously present at runtime and the test suite covers it
# via test_urdf.py).
#
# Similarly, load_urdf's signature in the stub is `str | os.PathLike` with an
# unparameterized PathLike, which pyright's strict mode reports as a partially
# unknown type. We pass a plain `str`, which type-checks cleanly.
# ---------------------------------------------------------------------------

from typing import Callable, cast

from cartan._core import (
    UrdfFailure,
    UrdfLoadResult,
    UrdfMetadata,
    load_urdf as _raw_load_urdf,  # pyright: ignore[reportUnknownVariableType]
)

# Narrow the load_urdf signature for strict mode: the generated stub uses
# `str | os.PathLike` with an unparameterized PathLike[T] which strict pyright
# reports as a partially unknown type. We only need the str overload here.
load_urdf: Callable[[str], UrdfLoadResult] = cast(
    "Callable[[str], UrdfLoadResult]", _raw_load_urdf
)

UrdfError: type[RuntimeError] = cast(
    "type[RuntimeError]", getattr(cartan, "UrdfError")
)

try:
    result: UrdfLoadResult = load_urdf("nonexistent.urdf")
    meta: UrdfMetadata = result.metadata
    base_name: str = meta.base_link_name
    tool_name: str = meta.tool_link_name
    names: list[str] = meta.joint_names
    if names:
        idx: int = meta.joint_index(names[0])
    loaded_chain: cartan.KinematicChain = result.chain
except UrdfError as e:
    # UrdfError exposes .kind: UrdfFailure and .detail: str at runtime; the
    # static type here is the upcast RuntimeError, so the attribute access is
    # cast through getattr to keep strict mode happy.
    kind: UrdfFailure = cast("UrdfFailure", getattr(e, "kind"))
    detail: str = cast("str", getattr(e, "detail"))


if __name__ == "__main__":
    # Allow direct invocation as a smoke check; the real gate is the
    # subprocess pyright/mypy run in test_stubs.py.
    print("ok")
