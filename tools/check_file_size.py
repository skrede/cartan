#!/usr/bin/env python3
"""Check every tracked C++ file against the file-size ceiling and its registry."""

import argparse
import re
import subprocess
import sys
from pathlib import Path

CEILING = 200
GROWTH_NUMERATOR = 11
GROWTH_DENOMINATOR = 10
REGISTRY_NAME = "EXCEPTIONS.md"
EXTENSIONS = frozenset({".h", ".hh", ".hpp", ".hxx", ".inl", ".ipp", ".c", ".cc", ".cpp", ".cxx"})
EXCLUDED_PREFIXES = ("benchmarks/third_party/",)
AREAS = ("lib", "tests", "benchmarks", "examples", "python", "profiling")

EXIT_UNLISTED = 1
EXIT_STALE = 3
EXIT_GROWTH = 4
EXIT_REGISTRY = 5
EXIT_NO_INPUT = 6

EXIT_HELP = """exit codes:
  0  every overage is registered, every row is live, no registered file has grown past its allowance
  1  a file is over the ceiling and unregistered -- decide: decompose it, or register it with a reason
  3  a registered row is stale -- its file is at or under the ceiling, or is no longer tracked; delete the row
  4  a registered file has grown past its recorded count plus the tolerance -- re-record the count or decompose
  5  the registry is missing or a row is malformed
  6  the scan found no files at all, which means the scope or the repository is wrong
"""

ROW_PATH = re.compile(r"^[A-Za-z0-9_./-]+\.[A-Za-z]+$")
COUNT_IN_REASON = re.compile(r"\b\d+\s*(?:lines|-line)\b|\d{3,}")


def in_scope(path: str) -> bool:
    return Path(path).suffix in EXTENSIONS and not path.startswith(EXCLUDED_PREFIXES)


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


def measure(root: Path, names: list[str]) -> dict[str, int]:
    present = ((name, root / name) for name in names)
    return {name: line_count(path) for name, path in present if path.is_file()}


def split_row(line: str) -> list[str] | None:
    stripped = line.strip()
    if not stripped.startswith("|"):
        return None
    cells = [cell.strip() for cell in stripped.strip("|").split("|")]
    return cells if cells and ROW_PATH.match(cells[0]) else None


def row_fault(cells: list[str], number: int) -> str | None:
    where = f"{REGISTRY_NAME} line {number}, {cells[0]}"
    if len(cells) < 3:
        return f"{where}: a row needs a path, a recorded line count and a reason"
    if not in_scope(cells[0]):
        return f"{where}: the path is outside the checked scope"
    if not cells[1].isdigit():
        return f"{where}: the recorded count column is not a number"
    if not cells[2]:
        return f"{where}: the reason is empty, which is a silent exception with extra steps"
    if COUNT_IN_REASON.search(cells[2]):
        return f"{where}: the reason restates a line count instead of saying what a split would harm"
    return None


def parse_registry(text: str) -> tuple[dict[str, int], list[str]]:
    recorded: dict[str, int] = {}
    faults: list[str] = []
    for number, line in enumerate(text.splitlines(), 1):
        cells = split_row(line)
        if cells is None:
            continue
        fault = row_fault(cells, number)
        if fault is not None:
            faults.append(fault)
        elif cells[0] in recorded:
            faults.append(f"{REGISTRY_NAME} line {number}, {cells[0]}: the path is registered twice")
        else:
            recorded[cells[0]] = int(cells[1])
    return recorded, faults


def allowance(recorded: int) -> int:
    return (recorded * GROWTH_NUMERATOR + GROWTH_DENOMINATOR - 1) // GROWTH_DENOMINATOR


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
    return "its file is no longer tracked in the checked scope"


def check(counts: dict[str, int], recorded: dict[str, int]) -> int:
    over = {path: count for path, count in counts.items() if count > CEILING}
    unlisted = sorted(path for path in over if path not in recorded)
    stale = sorted(path for path in recorded if path not in over)
    grown = sorted(path for path in recorded if path in over and over[path] > allowance(recorded[path]))
    report(f"{path} is {counts[path]} lines, over the {CEILING}-line ceiling, and is not "
           f"registered in {REGISTRY_NAME}" for path in unlisted)
    report(f"the {REGISTRY_NAME} row for {path} is stale: {stale_reason(path, counts)}"
           for path in stale)
    report(f"{path} has grown to {counts[path]} lines from the {recorded[path]} recorded in "
           f"{REGISTRY_NAME}, past its allowance of {allowance(recorded[path])}" for path in grown)
    if unlisted:
        return EXIT_UNLISTED
    if stale:
        return EXIT_STALE
    return EXIT_GROWTH if grown else 0


def load_registry(registry: Path) -> tuple[dict[str, int], int]:
    if not registry.is_file():
        print(f"error: the registry {registry} does not exist", file=sys.stderr)
        return {}, EXIT_REGISTRY
    recorded, faults = parse_registry(registry.read_text(encoding="utf-8"))
    report(faults)
    return recorded, EXIT_REGISTRY if faults else 0


def run(args: argparse.Namespace) -> int:
    root = args.source_root
    try:
        names = tracked_sources(root)
    except (OSError, subprocess.CalledProcessError) as exc:
        print(f"error: cannot list the tracked files under {root}: {exc}", file=sys.stderr)
        return EXIT_NO_INPUT
    counts = measure(root, names)
    if not counts:
        print(f"error: no tracked C++ file was found under {root}", file=sys.stderr)
        return EXIT_NO_INPUT
    if args.print_overages:
        print_overages(counts)
        return 0
    recorded, failure = load_registry(args.registry or root / REGISTRY_NAME)
    if failure:
        return failure
    status = check(counts, recorded)
    if status == 0:
        print(f"checked {len(counts)} tracked C++ files; {len(recorded)} registered overages, "
              "all live and within allowance")
    return status


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check every tracked C++ file against the file-size ceiling and its registry.",
        epilog=EXIT_HELP,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--source-root", type=Path, default=Path("."),
                        help="repository root to check (default: the current directory)")
    parser.add_argument("--registry", type=Path, default=None,
                        help=f"registry to read (default: {REGISTRY_NAME} under the source root)")
    parser.add_argument("--print-overages", action="store_true",
                        help="print the current overage set as registry rows, write nothing, and exit 0")
    return parser.parse_args(argv)


if __name__ == "__main__":
    raise SystemExit(run(parse_args(sys.argv[1:])))
