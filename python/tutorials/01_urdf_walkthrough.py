"""Load a URDF, FK-walk a reachable pose, solve IK, and back-verify."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

import cartan


def _default_urdf() -> Path:
    return (
        Path(__file__).resolve().parents[2]
        / "tests"
        / "fixtures"
        / "urdf"
        / "cartanbot.urdf"
    )


def _draw_within_limits(
    chain: cartan.KinematicChain, rng: np.random.Generator,
) -> np.ndarray:
    q = np.empty(chain.num_joints(), dtype=np.float64)
    for i, lim in enumerate(chain.limits()):
        lo = lim.position_min
        hi = lim.position_max
        if not np.isfinite(lo):
            lo = -np.pi
        if not np.isfinite(hi):
            hi = np.pi
        q[i] = rng.uniform(lo, hi)
    return q


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if len(args) > 1:
        print("usage: 01_urdf_walkthrough.py [robot.urdf]", file=sys.stderr)
        return 2

    if not hasattr(cartan, "load_urdf"):
        print("cartan was built without URDF support")
        return 0

    urdf_file = Path(args[0]) if args else _default_urdf()
    print(f"Loading URDF: {urdf_file}")

    loaded = cartan.load_urdf(str(urdf_file))
    chain = loaded.chain
    metadata = loaded.metadata
    print(
        f"Loaded {chain.num_joints()}-DOF chain "
        f"({metadata.base_link_name} -> {metadata.tool_link_name})"
    )

    rng = np.random.default_rng(seed=42)
    q_truth = _draw_within_limits(chain, rng)
    target = cartan.forward_kinematics(chain, q_truth)

    print("Ground-truth q:", np.array2string(q_truth, precision=6))
    print("FK-walked target pose:")
    print(np.array2string(target.matrix(), precision=6, suppress_small=True))

    q_seed = np.zeros(chain.num_joints(), dtype=np.float64)
    config = cartan.IkConfig(
        position_tol=1e-6,
        orientation_tol=1e-6,
        max_iterations_per_attempt=200,
        max_total_work_units=200,
    )
    result = cartan.solve_ik(chain, target, q_seed, config)
    if not result.converged:
        print(
            "IK did not converge; this target may be outside the solver basin "
            "from the zero seed."
        )
        return 0

    fk_verify = cartan.forward_kinematics(chain, result.q)
    pose_err = float(np.linalg.norm((fk_verify.inverse() * target).log()))
    print(f"IK converged in {result.iterations} iterations")
    print("Recovered q:   ", np.array2string(result.q, precision=6))
    print("Ground-truth q:", np.array2string(q_truth, precision=6))
    print(f"Pose error (FK back-verify): {pose_err:.6e}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
