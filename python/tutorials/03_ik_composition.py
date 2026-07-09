"""Compare closed-form Pieper IK and iterative projected LM on FK-walked targets."""

from __future__ import annotations

import csv
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import cartan


@dataclass(frozen=True)
class CallRecord:
    success: bool
    wall_us: float
    pos_err: float
    multi_solutions: int


@dataclass(frozen=True)
class Aggregate:
    mean_wall_us: float
    max_wall_us: float
    mean_pos_err: float
    success_rate: float
    mean_multi_solutions: float


class DeterministicRng:
    def __init__(self, seed: int) -> None:
        self._state = seed & ((1 << 64) - 1)

    def uniform(self, lo: float, hi: float) -> float:
        self._state = (
            self._state * 6364136223846793005 + 1442695040888963407
        ) & ((1 << 64) - 1)
        unit = (self._state >> 11) / float(1 << 53)
        return lo + (hi - lo) * unit


def _se3_from_rotation_translation(
    rotation: cartan.SO3, translation: np.ndarray,
) -> cartan.SE3:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation.matrix()
    matrix[:3, 3] = translation
    return cartan.SE3.from_matrix(matrix)


def _make_chain() -> cartan.KinematicChain:
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
    return cartan.KinematicChain(home, axes, limits)


def _pose_error(chain: cartan.KinematicChain, q: np.ndarray, target: cartan.SE3) -> float:
    fk = cartan.forward_kinematics(chain, q)
    return float(np.linalg.norm((fk.inverse() * target).log()))


def _summarize(records: list[CallRecord]) -> Aggregate:
    if not records:
        return Aggregate(0.0, 0.0, 0.0, 0.0, 0.0)

    successes = [r for r in records if r.success]
    wall = np.array([r.wall_us for r in records], dtype=np.float64)
    if successes:
        err = np.array([r.pos_err for r in successes], dtype=np.float64)
        multi = np.array([r.multi_solutions for r in successes], dtype=np.float64)
        mean_pos_err = float(err.mean())
        mean_multi = float(multi.mean())
    else:
        mean_pos_err = 0.0
        mean_multi = 0.0

    return Aggregate(
        mean_wall_us=float(wall.mean()),
        max_wall_us=float(wall.max()),
        mean_pos_err=mean_pos_err,
        success_rate=float(len(successes) / len(records)),
        mean_multi_solutions=mean_multi,
    )


def _write_csv(
    path: Path,
    closed_form: list[CallRecord],
    iterative: list[CallRecord],
) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            ["target_index", "solver", "success", "wall_us", "pos_err", "multi_solutions"]
        )
        for i, record in enumerate(closed_form):
            writer.writerow([
                i,
                "pieper_6r_solver",
                int(record.success),
                f"{record.wall_us:.17g}",
                f"{record.pos_err:.17g}",
                record.multi_solutions,
            ])
        for i, record in enumerate(iterative):
            writer.writerow([
                i,
                "projected_lm",
                int(record.success),
                f"{record.wall_us:.17g}",
                f"{record.pos_err:.17g}",
                record.multi_solutions,
            ])


def _run_race() -> tuple[list[CallRecord], list[CallRecord]]:
    chain = _make_chain()
    rng = DeterministicRng(seed=42)
    targets = [
        cartan.forward_kinematics(
            chain,
            np.array([rng.uniform(-np.pi, np.pi) for _ in range(6)]),
        )
        for _ in range(50)
    ]
    q_seed = np.zeros(6, dtype=np.float64)
    config = cartan.IkConfig(
        position_tol=1e-6,
        orientation_tol=1e-6,
        max_iterations_per_attempt=200,
        max_total_work_units=200,
    )

    closed_form: list[CallRecord] = []
    iterative: list[CallRecord] = []
    for target in targets:
        t0 = time.perf_counter_ns()
        raw = cartan.analytical.solve_pieper_6r(chain, target)
        t1 = time.perf_counter_ns()
        best = cartan.analytical.closest_to_seed(raw, q_seed)
        if raw.status == cartan.AnalyticalStatus.ok and best is not None:
            closed_form.append(
                CallRecord(
                    success=True,
                    wall_us=(t1 - t0) / 1000.0,
                    pos_err=_pose_error(chain, best, target),
                    multi_solutions=len(raw.solutions),
                )
            )
        else:
            closed_form.append(
                CallRecord(False, (t1 - t0) / 1000.0, 0.0, len(raw.solutions))
            )

        t0 = time.perf_counter_ns()
        result = cartan.solve_ik_speed(chain, target, q_seed, config)
        t1 = time.perf_counter_ns()
        pos_err = _pose_error(chain, result.q, target) if result.converged else 0.0
        iterative.append(
            CallRecord(
                success=result.converged and pos_err < 1e-4,
                wall_us=(t1 - t0) / 1000.0,
                pos_err=pos_err,
                multi_solutions=1,
            )
        )

    return closed_form, iterative


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    csv_path: Path | None = None
    i = 0
    while i < len(args):
        if args[i] == "--csv":
            if i + 1 >= len(args):
                print("usage: 03_ik_composition.py [--csv path]", file=sys.stderr)
                return 2
            csv_path = Path(args[i + 1])
            i += 2
        else:
            print("usage: 03_ik_composition.py [--csv path]", file=sys.stderr)
            return 2

    closed_form, iterative = _run_race()
    if csv_path is not None:
        _write_csv(csv_path, closed_form, iterative)
        return 0

    cf = _summarize(closed_form)
    it = _summarize(iterative)
    print("Race over N = 50 FK-walked random targets on KR6 R900")
    print(
        f"{'solver':>20} | {'mean_us':>10} | {'max_us':>10} | "
        f"{'mean_pos_err':>14} | {'success_rate':>13} | {'multi_solutions':>16}"
    )
    print("-" * 85)
    print(
        f"{'pieper_6r_solver':>20} | {cf.mean_wall_us:10.2f} | "
        f"{cf.max_wall_us:10.2f} | {cf.mean_pos_err:14.3e} | "
        f"{cf.success_rate * 100.0:12.1f}% | {cf.mean_multi_solutions:16.2f}"
    )
    print(
        f"{'projected_lm':>20} | {it.mean_wall_us:10.2f} | "
        f"{it.max_wall_us:10.2f} | {it.mean_pos_err:14.3e} | "
        f"{it.success_rate * 100.0:12.1f}% | {1:16d}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
