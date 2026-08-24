#!/usr/bin/env python3
"""Render the study's figures from a capture's cell records.

The records this reads are not carried in the repository -- the figures are what
the repository publishes, and this is what produced them. Point --root at a
capture, whether this project's or one taken on the reader's own machine, and
the same four figures come out for it.

Bounds nobody declared cannot reject a configuration, so the synthetic-limits
arm is dropped before anything is drawn: a figure over invented bounds is a
figure about the invention. The three periodic rules are separate experiments,
so a run covers one table.

    python3 tools/bench_study_figures.py --root <capture>/ladder \\
        --table c --out docs/benchmarks/figures
"""

import sys
import argparse
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import figure_style
import bench_study_panels
import bench_study_records as records

def cells_under(root: pathlib.Path, table: str) -> tuple:
    path = records.one_file(root, table, "cells", [])
    rows = records.read_csv(path, records.CELL_COLUMNS)
    records.one_table(path, rows, table)
    kept, dropped = records.without_synthetic(rows)
    if not kept:
        raise records.Refusal(f"{path}: every row carries synthetic limits; nothing to draw")
    return [row for _, row in kept], dropped


def arguments():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", required=True, type=lambda v: pathlib.Path(v).resolve())
    parser.add_argument("--table", default="c")
    parser.add_argument("--out", required=True, type=pathlib.Path)
    parser.add_argument("--only", action="append", default=[],
                        choices=sorted(bench_study_panels.FIGURES))
    return parser.parse_args()


def main() -> int:
    args = arguments()
    if not args.root.is_dir():
        print(f"error: no capture at {args.root}", file=sys.stderr)
        return 2
    try:
        cells, dropped = cells_under(args.root, args.table)
    except records.Refusal as refusal:
        print(f"refused: {refusal}", file=sys.stderr)
        return 3
    args.out.mkdir(parents=True, exist_ok=True)
    wanted = args.only or sorted(bench_study_panels.FIGURES)
    written = []
    for name in wanted:
        build = bench_study_panels.FIGURES[name]
        written += figure_style.render(lambda palette: build(cells, palette), args.out, name)
    print(f"{len(cells)} cells from table {args.table} under {args.root}, "
          f"{dropped} synthetic-limit rows dropped")
    for path in written:
        print(f"  wrote {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
