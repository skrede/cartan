#!/usr/bin/env python3
"""Check every tracked C++ file against the file-size ceiling and its register."""

import argparse
import subprocess
import sys
from pathlib import Path

from size_rules import CEILING, REGISTRY_NAME, allowance, in_scope, parse_registry

AREAS = ("lib", "tests", "benchmarks", "examples", "python", "profiling")

EXIT_UNLISTED = 1
EXIT_STALE = 3
EXIT_GROWTH = 4
EXIT_REGISTRY = 5
EXIT_NO_INPUT = 6
EXIT_INFLATED = 7

EXIT_HELP = """exit codes:
  0  every overage is registered and every row still describes its file
  1  a file is over the ceiling and unregistered -- decompose it, or register it with a reason
  2  the arguments are wrong (argparse)
  3  a row is stale -- its file is under the ceiling or no longer exists; delete or correct the row
  4  a registered file has grown past its recorded count plus the tolerance -- re-record or decompose
  5  the register is missing, or a row is malformed and so cannot be checked
  6  the scan could not read what it was asked to read: no files at all, or a tracked file absent
  7  a row records more lines than its file has, which would buy the file unearned headroom
"""


def tracked_sources(root: Path) -> list[str]:
    listing = subprocess.run(
        ["git", "-C", str(root), "ls-files", "-z"],
        check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    ).stdout
    names = listing.decode("utf-8", "surrogateescape").split("\0")
    return sorted(name for name in names if name and in_scope(name))


def line_count(path: Path) -> int:
    text = path.read_bytes().decode("utf-8", "surrogateescape")
    return text.count("\n") + (1 if text and not text.endswith("\n") else 0)


def measure(root: Path, names: list[str]) -> tuple[dict[str, int], list[str]]:
    counts, absent = {}, []
    for name in names:
        path = root / name
        if path.is_file():
            counts[name] = line_count(path)
        else:
            absent.append(name)
    return counts, absent


def area_key(path: str, count: int) -> tuple[int, int, str]:
    area = path.split("/")[0]
    rank = AREAS.index(area) if area in AREAS else len(AREAS)
    return (rank, -count, path)


def print_overages(counts: dict[str, int]) -> None:
    over = [(path, count) for path, count in counts.items() if count > CEILING]
    for path, count in sorted(over, key=lambda item: area_key(*item)):
        print(f"| {path} | {count} |  |")


def report(messages) -> None:
    for message in messages:
        print(f"error: {message}", file=sys.stderr)


def stale_reason(path: str, counts: dict[str, int]) -> str:
    if path in counts:
        return f"it is now {counts[path]} lines, at or under the {CEILING}-line ceiling"
    return "no tracked file has that path; delete the row, or correct it if the file was renamed"


def check(counts: dict[str, int], recorded: dict[str, int]) -> int:
    over = {path: count for path, count in counts.items() if count > CEILING}
    live = [path for path in sorted(recorded) if path in over]
    findings = [
        (EXIT_UNLISTED, [f"{p} is {counts[p]} lines, over the {CEILING}-line ceiling, and is not "
                         f"registered in {REGISTRY_NAME}" for p in sorted(over) if p not in recorded]),
        (EXIT_STALE, [f"the {REGISTRY_NAME} row for {p} is stale: {stale_reason(p, counts)}"
                      for p in sorted(recorded) if p not in over]),
        (EXIT_GROWTH, [f"{p} has grown to {over[p]} lines from the {recorded[p]} recorded in "
                       f"{REGISTRY_NAME}, past its allowance of {allowance(recorded[p])}"
                       for p in live if over[p] > allowance(recorded[p])]),
        (EXIT_INFLATED, [f"the {REGISTRY_NAME} row for {p} records {recorded[p]} lines but the file "
                         f"is {over[p]}; re-record it down so the allowance tracks the file"
                         for p in live if recorded[p] > over[p]]),
    ]
    for _, messages in findings:
        report(messages)
    return next((code for code, messages in findings if messages), 0)


def load_registry(registry: Path, counts: dict[str, int]) -> tuple[dict[str, int], int]:
    if not registry.is_file():
        print(f"error: the register {registry} does not exist", file=sys.stderr)
        return {}, EXIT_REGISTRY
    recorded, faults = parse_registry(registry.read_text(encoding="utf-8"), counts)
    report(faults)
    return recorded, EXIT_REGISTRY if faults else 0


def scan(root: Path) -> tuple[dict[str, int], int]:
    try:
        names = tracked_sources(root)
    except (OSError, subprocess.CalledProcessError) as exc:
        print(f"error: cannot list the tracked files under {root}: {exc}", file=sys.stderr)
        return {}, EXIT_NO_INPUT
    counts, absent = measure(root, names)
    report(f"{path} is tracked but absent from the working tree, so its length cannot be measured; "
           "restore it, or remove it from the index" for path in absent)
    if not counts:
        print(f"error: no tracked C++ file was found under {root}", file=sys.stderr)
    return counts, EXIT_NO_INPUT if absent or not counts else 0


def run(args: argparse.Namespace) -> int:
    counts, failure = scan(args.source_root)
    if failure:
        return failure
    if args.print_overages:
        print_overages(counts)
        return 0
    recorded, failure = load_registry(args.registry or args.source_root / REGISTRY_NAME, counts)
    if failure:
        return failure
    status = check(counts, recorded)
    if status == 0:
        print(f"checked {len(counts)} tracked C++ files; {len(recorded)} registered overages, "
              "all live and within allowance")
    return status


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check every tracked C++ file against the file-size ceiling and its register.",
        epilog=EXIT_HELP,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--source-root", type=Path, default=Path("."),
                        help="repository root to check (default: the current directory)")
    parser.add_argument("--registry", type=Path, default=None,
                        help=f"register to read (default: {REGISTRY_NAME} under the source root)")
    parser.add_argument("--print-overages", action="store_true",
                        help="print the current overage set as register rows, write nothing, exit 0")
    return parser.parse_args(argv)


if __name__ == "__main__":
    raise SystemExit(run(parse_args(sys.argv[1:])))
