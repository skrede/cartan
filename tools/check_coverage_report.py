#!/usr/bin/env python3
"""Refuse a coverage report that measured nothing, and name every header it did not reach.

A rate is a ratio, and a ratio over an empty denominator is 1.0. gcovr publishes
that ratio and exits 0, so a filter that matches no source reads downstream as
total coverage. That failure is checked first and reported on its own exit code,
because "the filter is broken" and "coverage moved" send a reader to different
places.

Two Cobertura documents are read, produced by the same build. The unmerged one
sums gcov's per-instantiation records; the merged one collapses them. Both are
printed, each labelled with the question it answers. No percentage is compared
against anything here.

The denominator can also shrink silently: gcov emits data only for translation
units the build compiled, so a configuration change that stops compiling a module
removes its headers from the report instead of scoring them zero, and the
published rate improves. Every tracked header absent from the report must
therefore carry a recorded reason, and a row that the report contradicts is
refused as stale, so the record cannot grow unchecked.

Which rows a report can contradict is not uniform, and the difference is measured
rather than assumed. A row claiming the header is not compiled is contradicted
the moment the report names it. A row claiming the header declares only types is
not: an instrument may synthesize records for implicitly-defined special members
and attribute them to a declaring line, and against these sources Clang's does
while GCC's does not. That claim is about the source, so a reader settles it by
opening the file and a report cannot.

Exit codes are distinct so a caller can tell which failure occurred:
0 clean, 1 an absent header has no recorded reason, 2 the check could not be
trusted, 3 a report measured nothing, 4 a recorded row is stale.
"""

import argparse
import pathlib
import subprocess
import sys
import xml.etree.ElementTree as ET

REASONS = ("declaration-only", "backend-not-built", "no-instantiating-test")

# Reasons a coverage report can contradict by naming the header at all.
REFUTABLE = ("backend-not-built", "no-instantiating-test")

UNMERGED_NOTE = ("one entry per source line per template instantiation: a line instantiated "
                 "five times contributes five to this denominator")
MERGED_NOTE = ("one entry per source line, covered when any instantiation executed it; "
               "a multi-line statement collapses onto its first line")


class CheckError(Exception):
    """The check could not be trusted, as opposed to a finding about the sources."""


def document(path):
    """The Cobertura root element, with an unreadable or empty file refused rather than skipped."""
    try:
        return ET.parse(path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise CheckError(f"{path} is not a readable coverage report: {exc}") from exc


def totals(root):
    """(lines covered, lines valid, branches covered, branches valid) for a whole document."""
    try:
        return tuple(int(root.get(key)) for key in
                     ("lines-covered", "lines-valid", "branches-covered", "branches-valid"))
    except (TypeError, ValueError) as exc:
        raise CheckError(f"coverage report carries no line and branch counts: {exc}") from exc


def emptiness(name, counts):
    """The message for a report whose denominator is empty, or None when it measured something."""
    _, lines, _, branches = counts
    if lines > 0 and branches > 0:
        return None
    return (f"{name}: lines-valid={lines}, branches-valid={branches}. The filters matched no "
            "source. The rate in this document is a ratio over an empty denominator and is not "
            "a measurement; gcovr publishes it as 1.0 and exits 0.")


def report(name, counts, note):
    covered_lines, lines, covered_branches, branches = counts
    print(f"{name:8} line   {covered_lines:6d}/{lines:<6d} {100 * covered_lines / lines:6.2f}%")
    print(f"{name:8} branch {covered_branches:6d}/{branches:<6d} "
          f"{100 * covered_branches / branches:6.2f}%")
    print(f"{name:8} {note}")


def tracked_headers(root):
    """Every tracked header under the module include trees, taken from git, not the filesystem."""
    trees = sorted(str(tree.relative_to(root)) for tree in root.glob("lib/*/include"))
    if not trees:
        raise CheckError(f"no module include tree under {root}; nothing would be compared")
    result = subprocess.run(["git", "-C", str(root), "ls-files", "--", *trees],
                            capture_output=True, text=True)
    if result.returncode != 0:
        raise CheckError(f"git ls-files under {root} failed: {result.stderr.strip()}")
    headers = {line for line in result.stdout.split() if line.endswith(".h")}
    if not headers:
        raise CheckError(f"git tracks no header under {trees}; nothing would be compared")
    return headers


def row(where, line):
    """(header, reason) for one record line; a malformed row is refused, not skipped."""
    header, _, reason = line.partition(" ")
    words = reason.split()
    if not words:
        raise CheckError(f"{where}: row carries no reason")
    if words[0] not in REASONS:
        raise CheckError(f"{where}: reason must start with one of {REASONS}")
    if words[0] == "backend-not-built" and len(words) < 2:
        raise CheckError(f"{where}: backend-not-built must name the build option")
    return header, " ".join(words)


def recorded(path):
    """header -> reason, from the two-column record of what the report does not reach."""
    try:
        text = path.read_text()
    except OSError as exc:
        raise CheckError(f"{path} is not a readable record: {exc}") from exc
    lines = enumerate(text.splitlines(), start=1)
    return dict(row(f"{path}:{number}", line) for number, line in lines
                if line.strip() and not line.lstrip().startswith("#"))


def absence_findings(absent, rows, tracked):
    """(unrecorded headers, stale rows) — the two directions the record can be wrong in."""
    stale = [f"{header}: {reason} — but the header is measured again"
             for header, reason in sorted(rows.items())
             if header in tracked and header not in absent and reason.startswith(REFUTABLE)]
    stale += [f"{header}: {reason} — but git tracks no such header"
              for header, reason in sorted(rows.items()) if header not in tracked]
    return sorted(absent - set(rows)), stale


def publish(unmerged_counts, merged_counts):
    report("unmerged", unmerged_counts, UNMERGED_NOTE)
    report("merged", merged_counts, MERGED_NOTE)
    print("headline: the unmerged line figure, because the merged one flatters by collapsing "
          "multi-line statements; the merged figure answers whether a line is reachable by "
          "some instantiation, which is a different question.")


def check_absences(source_root, record, merged):
    """Refuse an absent header with no recorded reason, and a recorded row that is no longer true."""
    tracked = tracked_headers(source_root)
    rows = recorded(record)
    absent = tracked - {node.get("filename") for node in merged.iter("class")}
    unrecorded, stale = absence_findings(absent, rows, tracked)
    if unrecorded:
        print("absent from the report with no recorded reason:", *unrecorded, sep="\n  ",
              file=sys.stderr)
        return 1
    if stale:
        print("recorded as unmeasured but no longer:", *stale, sep="\n  ", file=sys.stderr)
        return 4
    print(f"measured {len(tracked) - len(absent)} of {len(tracked)} tracked headers; "
          f"{len(rows)} recorded as unmeasured with a reason")
    return 0


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--unmerged", required=True, type=pathlib.Path)
    parser.add_argument("--merged", required=True, type=pathlib.Path)
    parser.add_argument("--unmeasured", required=True, type=pathlib.Path)
    parser.add_argument("--source-root", required=True, type=pathlib.Path)
    return parser.parse_args()


def main():
    args = parse_args()
    unmerged, merged = document(args.unmerged), document(args.merged)
    counts = (totals(unmerged), totals(merged))
    empty = [m for m in (emptiness("unmerged", counts[0]), emptiness("merged", counts[1])) if m]
    if empty:
        print("\n".join(empty), file=sys.stderr)
        return 3
    publish(*counts)
    return check_absences(args.source_root, args.unmeasured, merged)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except CheckError as error:
        print(f"refusing to report clean: {error}", file=sys.stderr)
        sys.exit(2)
