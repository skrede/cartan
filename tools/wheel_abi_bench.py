#!/usr/bin/env python3
"""Benchmark installed cartan wheel FK call overhead."""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time

import numpy as np

import cartan


def _se3_from_matrix(matrix: np.ndarray) -> cartan.SE3:
    return cartan.SE3.from_matrix(matrix)


def _ur5e_like_chain() -> cartan.KinematicChain:
    axes = [
        cartan.ScrewAxis.revolute(
            np.array([0.0, 0.0, 1.0], dtype=np.float64),
            np.array([0.0, 0.0, 0.0], dtype=np.float64),
        ),
        cartan.ScrewAxis.revolute(
            np.array([1.22464680e-16, 1.0, -2.05103490e-10], dtype=np.float64),
            np.array([0.0, 0.0, 0.1625], dtype=np.float64),
        ),
        cartan.ScrewAxis.revolute(
            np.array([1.22464680e-16, 1.0, -2.05103490e-10], dtype=np.float64),
            np.array([0.425, 0.0, 0.1625], dtype=np.float64),
        ),
        cartan.ScrewAxis.revolute(
            np.array([1.22464680e-16, 1.0, -2.05103490e-10], dtype=np.float64),
            np.array([0.8172, 0.0, 0.1625], dtype=np.float64),
        ),
        cartan.ScrewAxis.revolute(
            np.array([-5.02358629e-26, -4.10207091e-10, -1.0], dtype=np.float64),
            np.array([0.8172, 0.1333, 0.0], dtype=np.float64),
        ),
        cartan.ScrewAxis.revolute(
            np.array([2.51179352e-26, 1.0, -2.05103490e-10], dtype=np.float64),
            np.array([0.8172, 0.0, 0.0628], dtype=np.float64),
        ),
    ]
    home_matrix = np.array(
        [
            [-1.0, 7.85046230e-17, 2.35513869e-16, 0.8172],
            [2.35513869e-16, 2.05103601e-10, 1.0, 0.2329],
            [7.85046229e-17, 1.0, -2.05103934e-10, 0.0628],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    limits = [
        cartan.JointLimits(-2.0 * np.pi, 2.0 * np.pi),
        cartan.JointLimits(-2.0 * np.pi, 2.0 * np.pi),
        cartan.JointLimits(-np.pi, np.pi),
        cartan.JointLimits(-2.0 * np.pi, 2.0 * np.pi),
        cartan.JointLimits(-2.0 * np.pi, 2.0 * np.pi),
        cartan.JointLimits(-2.0 * np.pi, 2.0 * np.pi),
    ]
    return cartan.KinematicChain(_se3_from_matrix(home_matrix), axes, limits)


def _run_repeat(chain: cartan.KinematicChain, q: np.ndarray, iterations: int) -> float:
    start = time.perf_counter_ns()
    for _ in range(iterations):
        cartan.forward_kinematics(chain, q)
    elapsed_ns = time.perf_counter_ns() - start
    return elapsed_ns / 1000.0 / iterations


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--iterations", type=int, default=10_000)
    args = parser.parse_args(argv)

    if args.repeats < 5:
        parser.error("--repeats must be at least 5")
    if args.iterations <= 0:
        parser.error("--iterations must be positive")

    chain = _ur5e_like_chain()
    q = np.array([0.0, -1.2, 1.4, -0.7, 0.8, 0.2], dtype=np.float64)

    cartan.forward_kinematics(chain, q)
    samples = [_run_repeat(chain, q, args.iterations) for _ in range(args.repeats)]
    result = {
        "repeats": args.repeats,
        "iterations": args.iterations,
        "median_us_per_call": statistics.median(samples),
        "min_us_per_call": min(samples),
        "python_version": sys.version.split()[0],
    }
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
