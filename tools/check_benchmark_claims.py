#!/usr/bin/env python3
"""Refuse a published benchmark figure that no shipped record establishes.

A claim outliving its experiment is the failure this gate exists to catch: the
document keeps asserting a number after the run behind it was retired, and a
reader has no way to tell. So every measured figure has to sit under a source
marker naming a record file that is actually there, and the marker's path has to
resolve inside the evidence directory rather than anywhere the writer likes.

Two further rules are about what the figures may say rather than where they came
from. The iterative study carries no summary multiplier, because the same
experiment produced very different ratios on one robot under different machine
contention; the microbenchmark sections legitimately do carry ratios, so the ban
is scoped to the study's own sections rather than to the file. And no
planning-artifact identifier appears anywhere, because an external reader sees
only what ships.
"""

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

MARKER = re.compile(r"<!--\s*source:\s*(\S+)\s*-->")
SECTION = re.compile(r"^##\s+(.*)$")
STUDY_SECTION = re.compile(r"^##\s+The iterative study\b")

# A line states a measurement when it carries a figure with a unit, a percentage,
# a power of ten or a ratio -- or when it is a table row with a number in it. A
# bare count in prose ("three tables", "six strata") is not a measurement and is
# deliberately not matched.
# A trailing `x` counts as a multiplier only where a letter or digit does not
# follow it, so a hex literal such as 0x1000004c is not read as one.
RATIO = r"\d+(?:\.\d+)?\s*(?:×|x(?![0-9A-Za-z])|-fold|times\s+(?:faster|slower))"

CLAIM = re.compile(
    r"\d+(?:\.\d+)?\s*%"
    r"|\b\d+(?:\.\d+)?\s*(?:ns|µs|us|ms|rad|pp)\b"
    r"|\d(?:\.\d+)?[eE][-+]?\d+"
    r"|" + RATIO
)
TABLE_ROW = re.compile(r"^\s*\|.*\d")
SEPARATOR = re.compile(r"^\s*\|[\s|:-]+\|\s*$")

MULTIPLIER = re.compile(RATIO)

PLANNING = re.compile(
    r"\bphase\s+\d"
    r"|\bplan\s+\d"
    r"|\b(?:BENCH|GATE|SEM|SEED|REQ)-\d"
    r"|\bmilestone\b"
    r"|\bv\d+\.\d+\.\d+\b",
    re.IGNORECASE,
)

EXIT_FINDING = 1
EXIT_NO_INPUT = 2


@dataclass
class Finding:
    line: int
    rule: str
    detail: str


def markers_in(text: str) -> list[tuple[int, str]]:
    found = []
    for number, line in enumerate(text.splitlines(), start=1):
        match = MARKER.search(line)
        if match:
            found.append((number, match.group(1)))
    return found


def covered_lines(text: str) -> set[int]:
    """Lines a source marker vouches for.

    A marker's scope runs to the next marker or to the next top-level heading,
    whichever comes first. A heading ends the scope so that a marker cannot be
    written once at the top of the document and treated as covering every figure
    below it.
    """
    covered: set[int] = set()
    active = False
    for number, line in enumerate(text.splitlines(), start=1):
        if MARKER.search(line):
            active = True
            continue
        if SECTION.match(line):
            active = False
        if active:
            covered.add(number)
    return covered


def study_lines(text: str) -> set[int]:
    inside = False
    region: set[int] = set()
    for number, line in enumerate(text.splitlines(), start=1):
        if STUDY_SECTION.match(line):
            inside = True
            continue
        if inside and SECTION.match(line):
            inside = False
        if inside:
            region.add(number)
    return region


def resolve(marker: str, evidence_root: Path, repo_root: Path) -> tuple[Path, bool]:
    candidate = (repo_root / marker).resolve()
    root = evidence_root.resolve()
    return candidate, candidate == root or root in candidate.parents


def check(doc: Path, evidence_root: Path, repo_root: Path) -> list[Finding]:
    text = doc.read_text()
    findings: list[Finding] = []
    vouched = covered_lines(text)
    study = study_lines(text)

    for number, marker in markers_in(text):
        path, inside = resolve(marker, evidence_root, repo_root)
        if not inside:
            findings.append(Finding(number, "escaping marker",
                                    f"{marker} resolves outside {evidence_root}"))
        elif not path.is_file():
            findings.append(Finding(number, "missing record",
                                    f"{marker} names no file under {evidence_root}"))

    for number, line in enumerate(text.splitlines(), start=1):
        if MARKER.search(line) or SEPARATOR.match(line):
            continue
        states_measurement = CLAIM.search(line) or TABLE_ROW.match(line)
        if states_measurement and number not in vouched:
            findings.append(Finding(number, "unsourced claim", line.strip()[:90]))
        if number in study and MULTIPLIER.search(line):
            findings.append(Finding(number, "multiplier in the study", line.strip()[:90]))
        planning = PLANNING.search(line)
        if planning:
            findings.append(Finding(number, "planning reference", planning.group(0)))

    return findings


def run(args: argparse.Namespace) -> int:
    if not args.doc.is_file():
        print(f"error: no document at {args.doc}", file=sys.stderr)
        return EXIT_NO_INPUT
    if not args.evidence_root.is_dir():
        print(f"error: no evidence directory at {args.evidence_root}", file=sys.stderr)
        return EXIT_NO_INPUT

    findings = check(args.doc, args.evidence_root, args.repo_root)
    for finding in sorted(findings, key=lambda f: (f.line, f.rule)):
        print(f"{args.doc}:{finding.line}: {finding.rule}: {finding.detail}", file=sys.stderr)
    if findings:
        print(f"{len(findings)} finding(s) in {args.doc}", file=sys.stderr)
        return EXIT_FINDING

    count = len(markers_in(args.doc.read_text()))
    print(f"{args.doc}: every figure is sourced; {count} marker(s) resolve under "
          f"{args.evidence_root}")
    return 0


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Refuse a published benchmark figure no shipped record establishes.")
    parser.add_argument("--doc", type=Path, required=True)
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--repo-root", type=Path, default=Path("."),
                        help="what a marker's path is relative to (default: current directory)")
    return parser.parse_args(argv)


if __name__ == "__main__":
    raise SystemExit(run(parse_args(sys.argv[1:])))
