# Python Reference

Cartan ships Python bindings for the same Lie group, kinematics, and IK core
used by the C++ library. The Python package is intended for NumPy-first
robotics workflows: build chains, evaluate FK/Jacobians, load URDFs, and solve
IK without leaving Python.

## Installation

Stable package path:

```bash
pip install cartan-bindings
```

Editable source checkout with scikit-build-core rebuild enabled:

```bash
pip install --no-build-isolation --config-settings=editable.rebuild=true -Cbuild-dir=build -ve .
```

Development package indexes should be treated as explicit test sources, not as
ordinary installation defaults:

```bash
pip install --index-url https://test.pypi.org/simple/ --extra-index-url https://pypi.org/simple/ cartan-bindings
```

## Quick Smoke

```python
import numpy as np
import cartan

axis = np.array([0.0, 0.0, 1.0])
s1 = cartan.ScrewAxis.revolute(axis, np.array([0.0, 0.0, 0.0]))
s2 = cartan.ScrewAxis.revolute(axis, np.array([1.0, 0.0, 0.0]))
home = cartan.SE3.exp(np.array([0.0, 0.0, 0.0, 2.0, 0.0, 0.0]))
limits = [cartan.JointLimits(-np.pi, np.pi)] * 2
chain = cartan.KinematicChain(home, [s1, s2], limits)

pose = cartan.forward_kinematics(chain, np.array([0.5, -0.3]))
print(pose.matrix())
```

## Feature Parity

Feature family | C++ API | Python API | Docs/examples coverage | Status | Notes
--- | --- | --- | --- | --- | ---
Lie groups | `cartan::so3`, `cartan::se3`, plus SO(2)/SE(2) and frame wrappers | `cartan.SO3`, `cartan.SE3` | Quick smoke above; C++ `examples/lie_basics.cpp`; Python tests cover SO(3)/SE(3) | Partial | Python exposes the 3D groups used by kinematics and IK. SO(2), SE(2), and compile-time frame wrappers remain C++ only.
Chains | `cartan::screw_axis`, `joint_limits`, `kinematic_chain`, `static_chain` | `cartan.ScrewAxis`, `cartan.JointLimits`, `cartan.KinematicChain` | `python/tutorials/02_fk_and_jacobians.py` | Partial | Python uses runtime dynamic chains. Static-chain templates and joint-tag packs are C++ compile-time facilities.
FK | `cartan::forward_kinematics` | `cartan.forward_kinematics` | `python/tutorials/01_urdf_walkthrough.py`, `python/tutorials/02_fk_and_jacobians.py` | Bound and tested | Returns `cartan.SE3` directly in Python.
Jacobians | `cartan::space_jacobian`, `cartan::body_jacobian` | `cartan.space_jacobian`, `cartan.body_jacobian` | `python/tutorials/02_fk_and_jacobians.py` | Bound and tested | Returns NumPy `float64` arrays.
URDF | `cartan::load_urdf` and `urdf_load_result` | `cartan.load_urdf`, `UrdfLoadResult`, `UrdfMetadata`, `UrdfError` | `python/tutorials/01_urdf_walkthrough.py` | Bound and tested | Available when the extension is built with URDF support.
Iterative IK | `basic_ik_runner`, `solve_ik` family, native policies | `cartan.solve_ik`, `solve_ik_speed`, `solve_ik_robust`, `IkConfig`, `IkResult` | `python/tutorials/01_urdf_walkthrough.py`, `python/tutorials/03_ik_composition.py` | Bound and tested | Optional argmin-backed functions appear only when the extension is built with argmin support.
Analytical IK | Pieper 6R, planar 2R, spatial 3R, Paden-Kahan helpers | `cartan.analytical.solve_pieper_6r`, `solve_planar_2r`, `solve_3r`, `paden_kahan_1/2/3`, `solve_all` | `python/tutorials/03_ik_composition.py`, `docs/api/analytical.md` | Bound and tested | Analytical results expose verified solution lists and status values.
OPW | `cartan::opw_parameters`, `opw_6r_solver`, `opw_branch` | `cartan.OPWParameters`, `cartan.OPWBranch`, `cartan.analytical.solve_opw_6r` | Python analytical tests; parity table here | Bound and tested | Covers offset-shoulder spherical-wrist industrial arms through explicit OPW parameters.
Unwrap | `cartan::unwrapped_solver`, `unwrapped_result`, `range_status` | `cartan.UnwrappedResult`, `cartan.RangeStatus`, `solve_unwrapped_opw_6r`, `solve_unwrapped_pieper_6r`, `solve_unwrapped_3r`, `solve_unwrapped_planar_2r` | Python analytical tests; parity table here | Bound and tested | Python functions return every branch with per-solution range tags.
Exhaustive runner | `cartan::exhaustive_ik_runner` | `cartan.ExhaustiveIKRunner`, `IkPolicy`, `RankingStrategy` | Python exhaustive-runner tests | Bound and tested | Returns FK-verified ranked `IkResult` branches.
Install/export | CMake package config, install/export targets, scikit-build-core wheel build | `pip install cartan-bindings`, editable install command above | This page and package metadata | In progress | Wheel matrix and publishing workflow are handled separately from tutorial parity.

## Tutorial Mirrors

Run the Python tutorials from a source checkout with a built extension:

```bash
PYTHONPATH=python python python/tutorials/01_urdf_walkthrough.py
PYTHONPATH=python python python/tutorials/02_fk_and_jacobians.py
PYTHONPATH=python python python/tutorials/03_ik_composition.py
```

The third tutorial also writes a machine-readable CSV for parity checks:

```bash
PYTHONPATH=python python python/tutorials/03_ik_composition.py --csv build/python-tutorial-03.csv
```

## Example Counterparts

C++ example | Python counterpart | Status | Reason
--- | --- | --- | ---
examples/argmin_solvers.cpp | None | C++ only for now | Demonstrates optional C++ backend wiring. Python exposes optional argmin functions when built with argmin support, but no standalone Python example is shipped yet.
examples/basic_ik.cpp | python/tutorials/01_urdf_walkthrough.py | Mirrored by tutorial | The Python tutorial covers FK-walked target generation, iterative IK, and FK back-verification on a loaded chain.
examples/fk_jacobian.cpp | python/tutorials/02_fk_and_jacobians.py | Mirrored by tutorial | The Python tutorial prints FK, space Jacobian, and body Jacobian for planar and spatial chains.
examples/frame_safety.cpp | None | C++ only | Compile-time frame tags are a C++ type-system feature and are not exposed as Python runtime classes.
examples/ik_composition.cpp | python/tutorials/03_ik_composition.py | Mirrored by tutorial | The Python tutorial compares closed-form and iterative IK on FK-walked targets.
examples/ik_service_multi.cpp | None | C++ only for now | Demonstrates C++ service-style orchestration around native solver types; Python users compose the bound solver functions directly.
examples/ik_service_single.cpp | None | C++ only for now | Demonstrates a C++ single-service wrapper over native solver types; Python users call `cartan.solve_ik*` directly.
examples/lie_basics.cpp | None | No standalone counterpart | Python SO(3)/SE(3) usage is covered in the quick smoke and tests; SO(2)/SE(2) remain C++ only.
examples/tutorials/01_urdf_walkthrough.cpp | python/tutorials/01_urdf_walkthrough.py | Direct mirror | Both load a URDF, FK-walk a reachable target, solve IK, and FK-back-verify the result.
examples/tutorials/02_fk_and_jacobians.cpp | python/tutorials/02_fk_and_jacobians.py | Direct mirror | Both build planar 3R and spatial 6R chains from screw axes, then print FK and Jacobians.
examples/tutorials/03_ik_composition.cpp | python/tutorials/03_ik_composition.py | Direct mirror | Both run a deterministic 50-target closed-form versus iterative IK comparison and can emit CSV rows.

## Composition Boundaries

Cartan deliberately stays focused on Lie groups, serial-chain kinematics, and
IK. The following adjacent robotics tasks compose with Cartan instead of
becoming package dependencies.

### Visualization

Use `meshcat` as an optional visualization layer around Cartan poses:

```python
import meshcat
import numpy as np
import cartan

viewer = meshcat.Visualizer()
pose = cartan.SE3.exp(np.array([0.0, 0.0, 0.2, 0.4, 0.0, 0.2]))
viewer["target"].set_transform(pose.matrix())
```

### Collision checking

Use `hpp-fcl` for collision queries and feed it transforms computed by
Cartan:

```python
import cartan

def link_pose(chain: cartan.KinematicChain, q):
    return cartan.forward_kinematics(chain, q).matrix()
```

### Trajectory optimization

Use `toppra` for path parameterization after Cartan has produced reachable
waypoints:

```python
import numpy as np
import cartan

def waypoint(chain: cartan.KinematicChain, q):
    return np.asarray(cartan.forward_kinematics(chain, q).translation)
```

### Dynamics

Dynamics are a separate concern. Use a companion dynamics package such as
`cartan-serial-dynamics` when it is available, with Cartan supplying poses,
Jacobians, and IK seeds.
