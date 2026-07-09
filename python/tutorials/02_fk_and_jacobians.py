"""Forward kinematics and Jacobians on planar and spatial chains."""

from __future__ import annotations

import sys

import numpy as np

import cartan


def _se3_from_rotation_translation(
    rotation: cartan.SO3, translation: np.ndarray,
) -> cartan.SE3:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation.matrix()
    matrix[:3, 3] = translation
    return cartan.SE3.from_matrix(matrix)


def _print_matrix(label: str, value: np.ndarray) -> None:
    print(label)
    print(np.array2string(value, precision=6, suppress_small=True))
    print()


def _planar_3r() -> tuple[cartan.KinematicChain, np.ndarray]:
    z = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    axes = [
        cartan.ScrewAxis.revolute(z, np.array([0.0, 0.0, 0.0])),
        cartan.ScrewAxis.revolute(z, np.array([0.30, 0.0, 0.0])),
        cartan.ScrewAxis.revolute(z, np.array([0.55, 0.0, 0.0])),
    ]
    home = _se3_from_rotation_translation(
        cartan.SO3.identity(), np.array([0.75, 0.0, 0.0], dtype=np.float64))
    limits = [cartan.JointLimits(-np.pi, np.pi)] * 3
    q = np.array([0.4, -0.3, 0.5], dtype=np.float64)
    return cartan.KinematicChain(home, axes, limits), q


def _kr6_6r() -> tuple[cartan.KinematicChain, np.ndarray]:
    axes = [
        cartan.ScrewAxis.revolute(
            np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, 0.0])),
        cartan.ScrewAxis.revolute(
            np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 0.400])),
        cartan.ScrewAxis.revolute(
            np.array([0.0, 1.0, 0.0]), np.array([0.455, 0.0, 0.400])),
        cartan.ScrewAxis.revolute(
            np.array([1.0, 0.0, 0.0]), np.array([0.875, 0.0, 0.400])),
        cartan.ScrewAxis.revolute(
            np.array([0.0, 1.0, 0.0]), np.array([0.875, 0.0, 0.400])),
        cartan.ScrewAxis.revolute(
            np.array([1.0, 0.0, 0.0]), np.array([0.935, 0.0, 0.400])),
    ]
    home = _se3_from_rotation_translation(
        cartan.SO3.identity(), np.array([0.935, 0.0, 0.400], dtype=np.float64))
    limits = [cartan.JointLimits(-np.pi, np.pi)] * 6
    q = np.array([0.2, -0.4, 0.3, -0.5, 0.6, -0.2], dtype=np.float64)
    return cartan.KinematicChain(home, axes, limits), q


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if args:
        print("usage: 02_fk_and_jacobians.py", file=sys.stderr)
        return 2

    for title, factory in [
        ("Section 1: planar 3R", _planar_3r),
        ("Section 2: spatial 6R (KUKA KR 6 R900 SIXX)", _kr6_6r),
    ]:
        chain, q = factory()
        fk = cartan.forward_kinematics(chain, q)
        print(f"=== {title} ===")
        print("Joint configuration q (rad):", np.array2string(q, precision=6))
        _print_matrix("End-effector pose T(q):", fk.matrix())
        _print_matrix("Space Jacobian J_s:", cartan.space_jacobian(chain, q))
        _print_matrix("Body  Jacobian J_b:", cartan.body_jacobian(chain, q))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
