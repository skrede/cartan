#!/usr/bin/env python3
"""Rebuild one published study table from the records the harness emitted.

Standard library only, so a reader who clones the repository needs python3 and
nothing else to reproduce every figure in the report.

Two input tiers rebuild the same table. The per-cell aggregates and the
per-stratum paired differences ship in the repository; the per-target rows are a
sidecar an order of magnitude larger and ship as a release asset. `--cross-check`
rebuilds from both and refuses on any published figure that differs, which is
what makes the smaller shipped tier evidence rather than a summary of evidence.

Order statistics are nearest rank over the whole target set, and a non-finite
error sorts to the worst end rather than being dropped: a distribution reported
only over the targets a solver survived is conditioned on that solver's own
success. The figures that are so conditioned say so in their names -- the
accuracy the iso-accuracy mode matches, and every paired difference, which exists
only where both solvers were accepted on the same target.

Nothing here combines two tables, and nothing here emits a ratio summarizing a
table, a robot or the study. Each periodic rule is its own experiment, and a
reader who wants to compare them has to do it deliberately.
"""

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

from bench_study_clauses import HEADINGS, PAIRED_HEADINGS
from bench_study_manifest import check_kernel_values, check_participants, one_manifest
from bench_study_records import (
    CELL_COLUMNS, CELL_KERNEL_FIELDS, KERNEL_FIELDS, STRATA_COLUMNS, TARGET_COLUMNS, Refusal,
    one_file, one_table, read_csv, without_synthetic,
)
from bench_study_render import render, table_row
from bench_study_summary import (
    paired_from_strata, paired_from_targets, strata_text, summarize_cells, summarize_rows,
)

EXIT_REFUSED = 1

EXIT_HELP = """exit codes:
  0  the table was rebuilt
  1  a record, a path, a participant list or a cross-check was refused; the message says which
  2  the arguments are wrong (argparse)
"""

HEADING_LINES = frozenset({table_row(HEADINGS), table_row(PAIRED_HEADINGS)})

# The excluded-row count is a cell count in one tier and a target count in the
# other, so comparing it verbatim would refuse every cross-check on a
# difference of unit rather than of figure.
SOURCE_LINES = ("Records:", "Paired differences:", "Participants declared by:",
                "Excluded from every figure here:")


@dataclass
class Study:
    table: str
    tier: str
    records: Path
    paired_source: str
    manifest: object
    summaries: dict
    paired: dict
    absences: list
    uncountable: list
    excluded: int


def load_tier(root, table, tier, explicit):
    path = one_file(root, table, tier, explicit)
    rows = read_csv(path, {"targets": TARGET_COLUMNS, "cells": CELL_COLUMNS,
                           "strata": STRATA_COLUMNS}[tier])
    one_table(path, rows, table)
    kept, excluded = without_synthetic(rows)
    if not kept and excluded:
        raise Refusal(f"{path}: every row was excluded for standing on bounds no description "
                      f"declared, so there is nothing left to summarize")
    if not kept:
        raise Refusal(f"{path}: carries no rows, so there is nothing to summarize")
    return path, kept, excluded


def load_paired(root, table, manifest):
    path, rows, _ = load_tier(root, table, "strata", [])
    present = set()
    for column in ("solver_a", "solver_b"):
        check_kernel_values(path, rows, manifest, ("delta_kernel_evals_median",), column)
        present |= {row[column] for _, row in rows}
    check_participants(manifest, present)
    return f"`{path}`", paired_from_strata(rows)


def analyse(root, table, source, explicit, manifest):
    tier = "targets" if source == "targets" else "cells"
    path, rows, excluded = load_tier(root, table, tier, explicit)
    present = {row["solver"] for _, row in rows}
    absences = check_participants(manifest, present)
    check_kernel_values(path, rows, manifest,
                        KERNEL_FIELDS if tier == "targets" else CELL_KERNEL_FIELDS)
    if tier == "targets":
        paired_source, paired = "computed from the per-target rows", paired_from_targets(rows)
        summaries = summarize_rows(rows)
    else:
        paired_source, paired = load_paired(root, table, manifest)
        summaries = summarize_cells(rows)
    uncountable = [name for name in manifest.uncountable() if name in present]
    return Study(table, tier, path, paired_source, manifest, summaries, paired, absences,
                 uncountable, excluded)


def comparable(text):
    return [line for line in text.splitlines() if not line.startswith(SOURCE_LINES)]


def counted_figures(lines):
    return sum(len(line.split("|")) - 2 for line in lines
               if line.startswith("| ") and line not in HEADING_LINES)


def cross_check(document, other):
    ours, theirs = comparable(document), comparable(render(other))
    for line, (mine, yours) in enumerate(zip(ours, theirs), start=1):
        if mine != yours:
            raise Refusal(f"the two record tiers disagree at rendered line {line}:\n"
                          f"  {other.tier} tier: {yours}\n  this tier:    {mine}")
    if len(ours) != len(theirs):
        raise Refusal(f"the {other.tier} tier renders {len(theirs)} lines against {len(ours)}, so "
                      f"one tier carries a cell the other does not")
    return counted_figures(ours)


def emit_strata(arguments, study):
    if study.tier != "targets":
        raise Refusal("--strata-out writes the paired differences the per-target rows define; "
                      "run it with --from targets")
    Path(arguments.strata_out).write_text(strata_text(study.paired), encoding="utf-8")
    print(f"bench_study_report: {len(study.paired)} paired rows written to "
          f"{arguments.strata_out}")


def build(arguments):
    root = Path(arguments.root).resolve()
    if not root.is_dir():
        raise Refusal(f"{root} is not a directory")
    manifest = one_manifest(root, arguments.manifest)
    study = analyse(root, arguments.table, arguments.source, arguments.input, manifest)
    document = render(study)
    Path(arguments.out).write_text(document, encoding="utf-8")
    if arguments.strata_out:
        emit_strata(arguments, study)
    print(f"bench_study_report: table {arguments.table} rebuilt from the {study.tier} tier of "
          f"{study.records} into {arguments.out}")
    if arguments.cross_check is None:
        return
    other_root = Path(arguments.cross_check or root).resolve()
    other = analyse(other_root, arguments.table,
                    "aggregates" if study.tier == "targets" else "targets", [], manifest)
    print(f"bench_study_report: {cross_check(document, other)} published figures agree between "
          f"the {study.tier} and {other.tier} tiers")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, epilog=EXIT_HELP,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", required=True,
                        help="the directory below which every input must resolve")
    parser.add_argument("--table", required=True, help="the periodic rule table to rebuild")
    parser.add_argument("--from", dest="source", required=True,
                        choices=("aggregates", "targets"), help="which record tier to read")
    parser.add_argument("--out", required=True, help="where the markdown table is written")
    parser.add_argument("--manifest", default="",
                        help="the capture's environment record, instead of discovery below "
                             "the root")
    parser.add_argument("--cross-check", nargs="?", const="", default=None,
                        help="rebuild from the other tier as well and refuse on any published "
                             "figure that differs; takes the root that tier resolves below")
    parser.add_argument("--strata-out", default="",
                        help="write the per-stratum paired differences the per-target rows define")
    parser.add_argument("--input", action="append", default=[],
                        help="an explicit record file, instead of discovery below the root")
    arguments = parser.parse_args(argv)
    try:
        build(arguments)
    except Refusal as refused:
        print(f"bench_study_report: {refused}", file=sys.stderr)
        return EXIT_REFUSED
    return 0


if __name__ == "__main__":
    sys.exit(main())
