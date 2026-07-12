#!/usr/bin/env python3
"""Sweep the IK comparison benchmark across accuracy gates.

Drives ``ik_comparison_benchmarks`` at a series of convergence tolerances by
setting ``CARTAN_BENCH_TOL`` per run. Every solver (cartan's LM family and
TRAC-IK) converges and is FK-verified at the same gate, so the output is a
matched speed-vs-accuracy curve rather than a single operating point.

Each run's gate is also stamped into the benchmark JSON ``context`` via
``--benchmark_context`` so the raw files are self-describing.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from collections import defaultdict
from pathlib import Path


def _run_one(binary: Path, tol: float, out: Path, args: argparse.Namespace) -> Path:
    env = dict(os.environ, CARTAN_BENCH_TOL=repr(tol))
    cmd: list[str] = []
    if args.taskset_core is not None:
        cmd += ["taskset", "-c", str(args.taskset_core)]
    cmd += [
        str(binary),
        f"--benchmark_filter={args.filter}",
        f"--benchmark_min_time={args.min_time}",
        f"--benchmark_repetitions={args.repetitions}",
        "--benchmark_report_aggregates_only=true",
        f"--benchmark_context=gate_tol={tol!r}",
        f"--benchmark_out={out}",
        "--benchmark_out_format=json",
    ]
    subprocess.run(cmd, env=env, check=True, stdout=subprocess.DEVNULL)
    return out


def _median_rows(path: Path) -> dict[str, dict[str, float]]:
    data = json.loads(path.read_text())
    rows: dict[str, dict[str, float]] = {}
    for b in data["benchmarks"]:
        if b.get("aggregate_name") not in (None, "median"):
            continue
        if b.get("run_type") == "aggregate" and b.get("aggregate_name") != "median":
            continue
        name = b["name"].split("/")[0].removeprefix("bm_comparison_")
        rows[name] = {
            "time": b["real_time"],
            "unit": b["time_unit"],
            "success": b.get("Success_rate", float("nan")),
            "iterations": b.get("avg_iterations", float("nan")),
            "pos_error": b.get("avg_position_error", float("nan")),
            "ori_error": b.get("avg_orientation_error", float("nan")),
        }
    return rows


def _print_curve(curve: dict[str, dict[float, dict[str, float]]], tols: list[float]) -> None:
    for name in sorted(curve):
        print(f"\n{name}")
        hdr = "  gate".ljust(12) + "time".rjust(12) + "succ%".rjust(9) + "iters".rjust(9) + "pos_err".rjust(12)
        print(hdr)
        for tol in tols:
            m = curve[name].get(tol)
            if m is None:
                continue
            print(
                f"  {tol:<10.1e}"
                f"{m['time']:>8.1f}{m['unit']:<3}"
                f"{m['success']:>9.1f}"
                f"{m['iterations']:>9.1f}"
                f"{m['pos_error']:>12.2e}"
            )


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("binary", type=Path, help="path to ik_comparison_benchmarks")
    p.add_argument("--tolerances", default="1e-4,1e-5,1e-6,1e-7",
                   help="comma-separated accuracy gates to sweep")
    p.add_argument("--filter", default=".",
                   help="google-benchmark filter regex (default: all cells)")
    p.add_argument("--repetitions", type=int, default=3)
    p.add_argument("--min-time", default="0.5s")
    p.add_argument("--taskset-core", type=int, default=None,
                   help="pin the run to a single core for stable timing")
    p.add_argument("--out-dir", type=Path, required=True,
                   help="directory for the per-gate benchmark JSON files")
    p.add_argument("--curve-json", type=Path, default=None,
                   help="optional consolidated curve output")
    args = p.parse_args()

    tols = [float(t) for t in args.tolerances.split(",")]
    args.out_dir.mkdir(parents=True, exist_ok=True)

    curve: dict[str, dict[float, dict[str, float]]] = defaultdict(dict)
    for tol in tols:
        out = args.out_dir / f"gate_{tol:.0e}.json"
        print(f"[sweep] gate={tol:.1e} -> {out.name}", file=sys.stderr)
        _run_one(args.binary, tol, out, args)
        for name, metrics in _median_rows(out).items():
            curve[name][tol] = metrics

    _print_curve(curve, tols)
    if args.curve_json is not None:
        args.curve_json.write_text(json.dumps(
            {name: {f"{t:.0e}": m for t, m in by_tol.items()} for name, by_tol in curve.items()},
            indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
