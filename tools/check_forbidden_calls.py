#!/usr/bin/env python3
"""Refuse a description-reader call that destroys information, and a build that enables evaluation.

Only load(path, options, log) reports a diagnostic below the error tier: the
sink-less overload discards the whole warn tier, and load_into's failure arm
forwards one of the five fields its error carries. Neither ban is a verdict on
the overloads -- the second is a defect the supplier is repairing -- it is that a
consumer must not depend on which of a supplier's overloads currently happens to
forward its error.

The build-configuration rules sit here rather than in a runtime test because a
runtime test can only observe the default the build already chose. The declared
default needs a check of its own, on the same schedule as the calls.
"""

import argparse
import subprocess
import sys
from pathlib import Path

from forbidden_rules import (EVAL_OPTION, EVAL_UNPINNED, build_findings, cxx_findings,
                             doc_findings, detail_findings, in_build_scope, in_cxx_scope,
                             in_docs_scope, in_urdf_detail_scope, pins_option)

EXIT_FORBIDDEN = 1
EXIT_UNPINNED = 3
EXIT_NO_INPUT = 6

EXIT_HELP = """exit codes:
  0  no forbidden call, and the evaluation backend is pinned off
  1  a forbidden call or a forbidden build setting appears in a tracked file
  2  the arguments are wrong (argparse)
  3  no tracked build file pins the evaluation backend off, so the build inherits a default
  6  the scan could not read what it was asked to read: no tracked C++ or build file at all
"""


def tracked_files(root: Path) -> list[str]:
    listing = subprocess.run(
        ["git", "-C", str(root), "ls-files", "-z"],
        check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    ).stdout
    names = listing.decode("utf-8", "surrogateescape").split("\0")
    return sorted(name for name in names if name)


def read_text(path: Path) -> str:
    return path.read_bytes().decode("utf-8", "surrogateescape")


def scan(root: Path, names: list[str], in_scope, rules) -> tuple[list[str], list[tuple]]:
    scanned, findings = [], []
    for name in names:
        path = root / name
        if not in_scope(name) or not path.is_file():
            continue
        scanned.append(name)
        findings += [(name, *finding) for finding in rules(read_text(path))]
    return scanned, sorted(findings)


def report(findings: list[tuple]) -> None:
    for name, line, matched, reason in findings:
        print(f"error: {name}:{line}: {matched}: {reason}", file=sys.stderr)


def pinning_files(root: Path, names: list[str]) -> list[str]:
    return [name for name in names if pins_option(read_text(root / name))]


def run(args: argparse.Namespace) -> int:
    root = args.source_root
    try:
        names = tracked_files(root)
    except (OSError, subprocess.CalledProcessError) as exc:
        print(f"error: cannot list the tracked files under {root}: {exc}", file=sys.stderr)
        return EXIT_NO_INPUT
    sources, calls = scan(root, names, in_cxx_scope, cxx_findings)
    builds, settings = scan(root, names, in_build_scope, build_findings)
    _, detail = scan(root, names, in_urdf_detail_scope, detail_findings)
    _, docs = scan(root, names, in_docs_scope, doc_findings)
    if not sources and not builds:
        print(f"error: no tracked C++ or build file was found under {root}", file=sys.stderr)
        return EXIT_NO_INPUT
    report(calls + settings + detail + docs)
    if calls or settings or detail or docs:
        return EXIT_FORBIDDEN
    pins = pinning_files(root, builds)
    if not pins:
        print(f"error: {EVAL_UNPINNED}", file=sys.stderr)
        return EXIT_UNPINNED
    print(f"checked {len(sources)} tracked C++ file(s) and {len(builds)} tracked build file(s); "
          f"no forbidden call, and {EVAL_OPTION} is pinned off in {', '.join(pins)}")
    return 0


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Refuse the description-reader calls and the build settings cartan bans.",
        epilog=EXIT_HELP,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--source-root", type=Path, default=Path("."),
                        help="repository root to check (default: the current directory)")
    return parser.parse_args(argv)


if __name__ == "__main__":
    raise SystemExit(run(parse_args(sys.argv[1:])))
