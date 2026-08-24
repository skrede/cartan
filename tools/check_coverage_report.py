#!/usr/bin/env python3
"""Refuse a coverage report that measured nothing, and name every header it did not reach.

A rate over an empty denominator is 1.0, and gcovr publishes it and exits 0, so a
filter matching no source reads downstream as total coverage. That is checked
first and on its own exit code: a broken filter is not a coverage drop.

Two Cobertura documents are read, produced by the same build: the unmerged one
sums gcov's per-instantiation records, the merged one collapses them. Both are
printed, labelled with the question each answers. No percentage is compared
against anything here.

The denominator can also shrink silently, because gcov emits data only for
translation units the build compiled: a change that stops compiling a module
removes its headers rather than scoring them zero, and the rate improves. Every
absent tracked header must carry a recorded reason, and a report naming one
refutes its row.

The record is per instrument. gcov flavors disagree about which headers produce
records at all -- Clang's emits entries for implicitly-defined special members,
attributed to the declaring line, where GCC's emits none -- and one record per
flavor is what keeps every reason strictly refutable.

Exit codes are distinct so a caller can tell which failure occurred: 0 clean, 1 an
absent header has no reason, 2 untrusted check, 3 nothing measured, 4 stale row.
"""

import argparse
import pathlib
import subprocess
import sys
import xml.etree.ElementTree as ET

REASONS = ("declaration-only", "backend-not-built", "no-instantiating-test")
HEADER_SUFFIXES = (".h", ".hpp", ".hxx", ".inl", ".ipp")

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
    counts = []
    for key in ("lines-covered", "lines-valid", "branches-covered", "branches-valid"):
        value = root.get(key)
        if value is None or not value.isdigit():
            raise CheckError(f"coverage report carries no usable {key} attribute: {value!r}")
        counts.append(int(value))
    return tuple(counts)


def emptiness(name, counts):
    """The message for a denominator that is empty, or None when neither denominator is."""
    _, lines, _, branches = counts
    if lines == 0:
        return (f"{name}: lines-valid=0. The filters matched no source, so neither rate here is a "
                "measurement; gcovr publishes both as 1.0 and exits 0.")
    if branches == 0:
        return (f"{name}: lines-valid={lines} but branches-valid=0. The filters did match source, "
                "so this is not a filter fault; the build carried no branch data.")
    return None


def report(name, counts, note):
    covered_lines, lines, covered_branches, branches = counts
    print(f"{name:8} line   {covered_lines:6d}/{lines:<6d} {100 * covered_lines / lines:6.2f}%")
    print(f"{name:8} branch {covered_branches:6d}/{branches:<6d} "
          f"{100 * covered_branches / branches:6.2f}%")
    print(f"{name:8} {note}")


def publish(unmerged_counts, merged_counts):
    report("unmerged", unmerged_counts, UNMERGED_NOTE)
    report("merged", merged_counts, MERGED_NOTE)
    print("headline: the unmerged line figure, because the merged one flatters by collapsing "
          "multi-line statements; the merged one answers reachability by some instantiation.")


def tracked_headers(root):
    """Every tracked header under the module include trees, taken from git, not the filesystem."""
    trees = sorted(str(tree.relative_to(root)) for tree in root.glob("lib/*/include"))
    if not trees:
        raise CheckError(f"no module include tree under {root}; nothing would be compared")
    try:
        listing = subprocess.run(["git", "-C", str(root), "ls-files", "-z", "--", *trees],
                                 capture_output=True, text=True)
    except OSError as exc:
        raise CheckError(f"could not run git under {root}: {exc}") from exc
    if listing.returncode != 0:
        raise CheckError(f"git ls-files under {root} failed: {listing.stderr.strip()}")
    headers = {name for name in listing.stdout.split("\0") if name.endswith(HEADER_SUFFIXES)}
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
    """header -> reason, from the two-column record of what this instrument does not reach."""
    try:
        text = path.read_text()
    except OSError as exc:
        raise CheckError(f"{path} is not a readable record: {exc}") from exc
    rows = {}
    for number, line in enumerate(text.splitlines(), start=1):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        header, reason = row(f"{path}:{number}", line)
        if header in rows:
            raise CheckError(f"{path}:{number}: {header} is recorded twice")
        rows[header] = reason
    return rows


def absence_findings(absent, rows, tracked):
    """(unrecorded headers, stale rows) — the two directions the record can be wrong in."""
    stale = [f"{header}: {reason} — but the header is measured again"
             for header, reason in sorted(rows.items())
             if header in tracked and header not in absent]
    stale += [f"{header}: {reason} — but git tracks no such header"
              for header, reason in sorted(rows.items()) if header not in tracked]
    return sorted(absent - set(rows)), stale


def check_absences(source_root, record, merged):
    """Refuse an absent header with no recorded reason, and a row the report contradicts."""
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
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--unmerged", required=True, type=pathlib.Path,
                        help="Cobertura report from the run that sums instantiations")
    parser.add_argument("--merged", required=True, type=pathlib.Path,
                        help="Cobertura report from the run that collapses instantiations")
    parser.add_argument("--unmeasured", required=True, type=pathlib.Path,
                        help="record of the headers this instrument's report does not reach")
    parser.add_argument("--source-root", required=True, type=pathlib.Path,
                        help="repository root the reports' paths are relative to")
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
