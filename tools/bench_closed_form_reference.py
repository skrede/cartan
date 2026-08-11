#!/usr/bin/env python3
"""Emit the closed-form reference record from the benchmark binaries a build produced.

A closed-form solve has no budget, no iteration count and no convergence
tolerance, so it cannot be scored on the axes the iterative study uses. What can
be asked of it is whether it is exact, how many branches it returns, and how much
of the pose space it admits -- and those are the columns emitted here.

No timing is read or written. That is what lets this record be captured without
the quiet-machine conditions the iterative study required: none of these
quantities is sensitive to machine contention.

A comparator whose comparison binary the build did not produce is reported as a
named absence with the reason, never dropped from the output.
"""

import argparse
import csv
import json
import re
import subprocess
import sys
from pathlib import Path

COLUMNS = [
    "robot",
    "solver",
    "exact",
    "branch_count",
    "pose_space_coverage_pct",
    "max_position_error_m",
    "max_orientation_error_rad",
    "n_poses",
]

# Each witness cross-checks cartan's analytic solver on one arm. The formulation
# column is what tells a reader whether agreement to the last bit is expected: a
# solver deriving the inverse independently agrees at machine precision instead.
WITNESSES = [
    ("opw_comparison_benchmarks", "opw_kinematics", "kr6_sixx", "shared"),
    ("ikfast_comparison_benchmarks", "ikfast_kr6r900", "kr6_sixx", "independent"),
    ("ikgeo_comparison_benchmarks", "ik_geo", "kr6_sixx", "independent"),
]

ABSENCE = ("its comparison benchmark was not built, so the comparator was not "
           "found when this build was configured")

RECEIPT = {
    "branches": re.compile(r"cartan branches \(total\)\s*:\s*(\d+)"),
    "matched": re.compile(r"branches matched(?: exact)?\s*:\s*(\d+)"),
    "disagreement": re.compile(r"max joint disagreement \(rad\)\s*:\s*(\S+)"),
    "result": re.compile(r"RESULT:\s*(\w+)"),
}


def run(binary):
    out = subprocess.run([str(binary), "--benchmark_min_time=1x"],
                         capture_output=True, text=True, check=False)
    return out.stdout + out.stderr


def parse_receipt(text):
    found = {}
    for key, pattern in RECEIPT.items():
        match = pattern.search(text)
        if match is None:
            return None
        found[key] = match.group(1)
    return found


def load_cells(json_path):
    cells = {}
    for entry in json.load(open(json_path))["benchmarks"]:
        name = entry["name"].split("/")[0]
        cells[name] = entry
    return cells


def emit(rows, out_path):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bin-dir", required=True, type=Path,
                        help="the directory the benchmark binaries were built into")
    parser.add_argument("--cells", required=True, type=Path,
                        help="the closed_form_ik_benchmarks json this run produced")
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--robots", default="abb_irb120,kr6_sixx",
                        help="the arms cartan's analytic solver is measured on")
    args = parser.parse_args()

    cells = load_cells(args.cells)
    rows = []
    absences = []

    for robot in args.robots.split(","):
        solve = cells.get(f"bm_{robot}_pieper_6r")
        cover = cells.get(f"bm_{robot}_coverage")
        if solve is None:
            absences.append((robot, "cartan_opw_6r", "no analytic cell in this build"))
            continue
        rows.append({
            "robot": robot,
            "solver": "cartan_opw_6r",
            "exact": "yes" if solve["Success_pct"] >= 100.0 else "no",
            "branch_count": "",
            "pose_space_coverage_pct": f"{cover['coverage_pct']:.1f}" if cover else "",
            "max_position_error_m": f"{solve['pos_err_max']:.6e}",
            "max_orientation_error_rad": f"{solve['ori_err_max']:.6e}",
            "n_poses": int(solve["n_poses"]),
        })

    for binary, name, robot, formulation in WITNESSES:
        path = args.bin_dir / binary
        if not path.exists():
            absences.append((robot, name, ABSENCE))
            continue
        receipt = parse_receipt(run(path))
        if receipt is None:
            absences.append((robot, name, "its comparison ran but printed no parity receipt"))
            continue
        agreed = receipt["branches"] == receipt["matched"] and receipt["result"] == "PASS"
        rows.append({
            "robot": robot,
            "solver": name,
            "exact": "yes" if agreed else "no",
            "branch_count": receipt["matched"],
            "pose_space_coverage_pct": "",
            "max_position_error_m": "",
            "max_orientation_error_rad": "",
            "n_poses": "",
        })
        print(f"{name}: {receipt['matched']}/{receipt['branches']} branches matched, "
              f"max joint disagreement {receipt['disagreement']} rad ({formulation} formulation)")

    emit(rows, args.out)
    print(f"wrote {len(rows)} row(s) to {args.out}")
    for robot, name, reason in absences:
        print(f"ABSENT {name} on {robot}: {reason}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
