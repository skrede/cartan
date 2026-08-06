#!/usr/bin/env python3
"""Turns a sweep's per-process output into the record set a report can stand on.

The capture writes one directory per table, robot and stratum. The report script
reads one root carrying one environment record and one file per tier. Assembling
the first into the second is what this does: tier files are concatenated under a
single header, and the environment records are merged only if they agree.

Concatenation is refused rather than attempted when two tier files disagree about
their columns, because a table assembled from two column orders is silently wrong
in a way no downstream check would catch.
"""

import argparse
import json
from pathlib import Path

from bench_study_agree import merge
from bench_study_records import Refusal

TIERS = ("cells", "reachability")


def tier_files(root, table, tier):
    return sorted(root.rglob(f"table_{table}_{tier}.csv"))


def concatenate(paths, destination):
    """Streamed rather than joined in memory: the per-target tier of a full sweep is
    hundreds of megabytes, and reading it whole to write it back out would make the
    assembler the largest allocation in the study."""
    if not paths:
        raise Refusal(f"{destination}: nothing to assemble from, so no header is known")
    header = ""
    rows = 0
    with destination.open("w", encoding="utf-8") as out:
        for path in paths:
            with path.open(encoding="utf-8") as source:
                first = source.readline().rstrip("\n")
                if not first:
                    raise Refusal(f"{path}: carries no header, so its columns are unknown")
                if not header:
                    header = first
                    out.write(header + "\n")
                elif first != header:
                    raise Refusal(f"{path}: columns differ from {paths[0]}; a tier assembled "
                                  f"from two column orders would be wrong in a way no later "
                                  f"check would catch")
                for line in source:
                    if line.strip():
                        out.write(line if line.endswith("\n") else line + "\n")
                        rows += 1
    return rows


def assemble_table(capture, table, tiers, out):
    written = {}
    for tier in tiers:
        paths = tier_files(capture, table, tier)
        if not paths:
            if tier == "reachability":
                continue
            raise Refusal(f"no table_{table}_{tier}.csv resolves below {capture}, so table "
                          f"{table} has no {tier} record to report")
        written[tier] = concatenate(paths, out / f"table_{table}_{tier}.csv")
    return written


def report_sizes(out):
    """The shipped set lives in a source repository, so its size is a number the
    decision to ship it has to be made against rather than discovered later."""
    files = sorted(path for path in out.rglob("*") if path.is_file())
    total = sum(path.stat().st_size for path in files)
    largest = max(files, key=lambda path: path.stat().st_size)
    print(f"assembled {len(files)} files, {total / 1e6:.2f} MB total; largest "
          f"{largest.relative_to(out)} at {largest.stat().st_size / 1e6:.2f} MB")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture", required=True, action="append",
                        help="a sweep pass root; the first is the pass whose records ship")
    parser.add_argument("--table", action="append", default=[],
                        help="a table to assemble; defaults to a, b and c")
    parser.add_argument("--tier", action="append", default=[],
                        help=f"a tier to assemble; defaults to {' and '.join(TIERS)}")
    parser.add_argument("--manifest-from", default="",
                        help="the root the environment records are merged from; defaults to the "
                             "first capture, and is given explicitly when assembling the "
                             "per-target tier, whose sidecar carries no record of its own")
    parser.add_argument("--out", required=True, help="where the assembled record set is written")
    arguments = parser.parse_args()

    captures = [Path(root).resolve() for root in arguments.capture]
    tables = arguments.table or ["a", "b", "c"]
    tiers = arguments.tier or list(TIERS)
    out = Path(arguments.out).resolve()
    out.mkdir(parents=True, exist_ok=True)

    for table in tables:
        written = assemble_table(captures[0], table, tiers, out)
        for tier, rows in written.items():
            print(f"table {table} {tier}: {rows} rows")

    records = Path(arguments.manifest_from).resolve() if arguments.manifest_from else captures[0]
    merged = merge(sorted(records.rglob("environment.json")), len(captures))
    (out / "environment.json").write_text(json.dumps(merged, indent=4) + "\n", encoding="utf-8")
    print(f"merged {merged['assembled_from']['invocations']} environment records into one")
    report_sizes(out)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Refusal as refused:
        print(f"bench_study_assemble: {refused}")
        raise SystemExit(1) from refused
