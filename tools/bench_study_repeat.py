#!/usr/bin/env python3
"""Whether the machine was quiet, measured rather than asserted.

The per-cell `wall_ns_cv` in the records is dispersion across the distinct
targets inside one cell -- how much harder some problems are than others. It is
not evidence about the machine, and a reader who took it for that would conclude
the box was noisy from a number that says nothing about the box.

Machine noise needs the same work measured more than once. So the sweep is run
several times and the same cell's median wall time is compared across passes: a
low coefficient of variation there is the claim that contention did not move the
numbers, and it is the quantity earlier captures reported when they said sub-one
percent.
"""

import argparse
import csv
import statistics
from pathlib import Path

from bench_study_records import Refusal

IDENTITY = ("table", "robot", "limits_provenance", "stratum", "solver", "budget_index")

MEASURE = "wall_ns_median"


def read_pass(root, table):
    paths = sorted(root.rglob(f"table_{table}_cells.csv"))
    if not paths:
        raise Refusal(f"no table_{table}_cells.csv resolves below {root}, so this pass has "
                      f"nothing to compare")
    measured = {}
    for path in paths:
        with path.open(encoding="utf-8", newline="") as handle:
            for line, row in enumerate(csv.DictReader(handle), start=2):
                missing = [field for field in (*IDENTITY, MEASURE) if row.get(field) is None]
                if missing:
                    raise Refusal(f"{path} line {line}: carries no {missing}")
                key = tuple(row[field] for field in IDENTITY)
                if key in measured:
                    raise Refusal(f"{path} line {line}: {key} appears twice in one pass, so its "
                                  f"repeatability would be computed against itself")
                measured[key] = float(row[MEASURE])
    return measured


def variation(values):
    mean = statistics.fmean(values)
    if mean == 0.0:
        return 0.0
    return statistics.stdev(values) / mean


def compare(passes, table):
    measured = [read_pass(root, table) for root in passes]
    shared = set(measured[0])
    for other in measured[1:]:
        shared &= set(other)
    if not shared:
        raise Refusal(f"table {table}: the passes share no cell, so nothing can be compared "
                      f"across them")
    rows = []
    for key in sorted(shared):
        values = [pass_measured[key] for pass_measured in measured]
        rows.append((key, values, variation(values)))
    dropped = sorted(set(measured[0]) - shared)
    if dropped:
        print(f"table {table}: {len(dropped)} cells are not present in every pass and are "
              f"excluded from the repeatability figure")
    return rows


def write(rows, destination, passes):
    header = [*IDENTITY, *(f"wall_ns_median_pass_{index + 1}" for index in range(passes)),
              "wall_ns_cv_across_passes"]
    with destination.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for key, values, cv in rows:
            writer.writerow([*key, *(f"{value:.0f}" for value in values), f"{cv:.6f}"])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pass-root", required=True, action="append",
                        help="a sweep pass to compare; give the flag once per pass")
    parser.add_argument("--table", action="append", default=[], help="defaults to a, b and c")
    parser.add_argument("--out", required=True, help="where the repeatability record is written")
    arguments = parser.parse_args()

    passes = [Path(root).resolve() for root in arguments.pass_root]
    if len(passes) < 2:
        raise Refusal("repeatability needs at least two passes; one pass measures each cell once "
                      "and says nothing about whether the machine moved")
    out = Path(arguments.out).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)

    every = []
    for table in arguments.table or ["a", "b", "c"]:
        rows = compare(passes, table)
        every.extend(rows)
        spread = sorted(cv for _, _, cv in rows)
        print(f"table {table}: {len(rows)} cells, cv median {statistics.median(spread):.4%}, "
              f"max {spread[-1]:.4%}")
    write(every, out, len(passes))
    spread = sorted(cv for _, _, cv in every)
    print(f"{len(every)} cells across {len(passes)} passes: cv median "
          f"{statistics.median(spread):.4%}, max {spread[-1]:.4%}, written to {out}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Refusal as refused:
        print(f"bench_study_repeat: {refused}")
        raise SystemExit(1) from refused
