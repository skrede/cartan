#!/usr/bin/env python3
"""Compare two benchmark captures against the success-rate non-regression gate."""

import argparse
import json
import re
import statistics
import sys
from pathlib import Path

KNOWN_COUNTERS = ("Success_rate", "Success_pct")
PER_CELL_FLOOR = -1.0
DEFAULT_TOLERANCE = 1e-9

EXIT_REGRESSION = 1
EXIT_INPUT = 3

EXIT_HELP = """the gate is a conjunction of two clauses, and both must hold:
  (i)  net non-regression -- the summed success rate over the gated cells must be at least
       the baseline sum
  (ii) per-cell floor -- no single cell may regress by more than 1.00 percentage points from
       its baseline; the floor is negative-only, so no cell is ever penalized for improving

A cell whose delta is within --tolerance of zero counts as unchanged, and a cell regressing by
exactly the floor is accepted: the clause binds past 1.00 points, not at it.

exit codes:
  0  both clauses hold
  1  a clause fails
  2  the arguments are wrong (argparse)
  3  a capture cannot be read, or a baseline cell is absent from the candidate
"""


class CaptureError(Exception):
    pass


def load(path: Path) -> list:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise CaptureError(f"cannot read {path} as a benchmark capture: {exc}") from exc
    entries = payload.get("benchmarks")
    if not isinstance(entries, list) or not entries:
        raise CaptureError(f"{path} carries no non-empty benchmarks array")
    return entries


def resolve_counter(entries: list, path: Path, forced: str | None) -> str:
    if forced:
        return forced
    present = [name for name in KNOWN_COUNTERS if any(name in entry for entry in entries)]
    if not present:
        raise CaptureError(f"{path} carries none of the success-rate counters "
                           f"{', '.join(KNOWN_COUNTERS)}; name one with --counter")
    if len(present) > 1:
        raise CaptureError(f"{path} carries {' and '.join(present)}; name one with --counter")
    return present[0]


def cells(entries: list, counter: str) -> dict[str, float]:
    """Aggregate rows are derived from the repetitions, so read the repetitions and take their median."""
    grouped: dict[str, list[float]] = {}
    for entry in entries:
        key = entry.get("run_name") or entry.get("name")
        if not key or entry.get("run_type", "iteration") != "iteration" or counter not in entry:
            continue
        try:
            grouped.setdefault(key, []).append(float(entry[counter]))
        except (TypeError, ValueError) as exc:
            raise CaptureError(f"{key} carries a non-numeric {counter}: {entry[counter]}") from exc
    return {key: statistics.median(values) for key, values in grouped.items()}


def restrict(measured: dict[str, float], pattern: str | None) -> dict[str, float]:
    if not pattern:
        return measured
    try:
        selector = re.compile(pattern)
    except re.error as exc:
        raise CaptureError(f"--filter is not a regular expression: {exc}") from exc
    return {key: value for key, value in measured.items() if selector.search(key)}


def deltas(baseline: dict[str, float], candidate: dict[str, float]) -> dict[str, float]:
    missing = sorted(set(baseline) - set(candidate))
    if missing:
        shown = ", ".join(missing[:5]) + (" ..." if len(missing) > 5 else "")
        raise CaptureError(f"{len(missing)} baseline cells are absent from the candidate, so the "
                           f"summed comparison would be flattered by their absence: {shown}")
    if not baseline:
        raise CaptureError("no baseline cell carries the counter, so there is nothing to gate")
    return {key: candidate[key] - value for key, value in baseline.items()}


def print_regressions(measured: dict[str, float], moved: dict[str, float], tolerance: float) -> None:
    regressed = sorted((delta, key) for key, delta in moved.items() if delta < -tolerance)
    if not regressed:
        print("no cell regressed")
        return
    print(f"{len(regressed)} cells regressed:")
    for delta, key in regressed:
        print(f"  {delta:+7.2f} pp  {measured[key]:6.2f} -> {measured[key] + delta:6.2f}  {key}")


def print_unmatched(extra: list[str]) -> None:
    if not extra:
        return
    print(f"{len(extra)} candidate cells are not in the baseline and are excluded from the sum:")
    for key in extra:
        print(f"  {key}")


def verdict(moved: dict[str, float], tolerance: float) -> int:
    total = sum(moved.values())
    worst_key, worst = min(moved.items(), key=lambda item: item[1])
    named = f" at {worst_key}" if worst < -tolerance else ""
    print(f"summed delta: {total:+.2f} pp over {len(moved)} gated cells")
    print(f"worst single-cell regression: {min(worst, 0.0):+.2f} pp{named}")
    net_ok = total >= -tolerance
    floor_ok = worst >= PER_CELL_FLOOR - tolerance
    print(f"clause (i)  net non-regression: {'PASS' if net_ok else 'FAIL'}")
    print(f"clause (ii) per-cell floor {PER_CELL_FLOOR:.2f} pp: {'PASS' if floor_ok else 'FAIL'}")
    return 0 if net_ok and floor_ok else EXIT_REGRESSION


def run(args: argparse.Namespace) -> int:
    try:
        base_entries = load(args.baseline)
        counter = resolve_counter(base_entries, args.baseline, args.counter)
        baseline = restrict(cells(base_entries, counter), args.filter)
        candidate = restrict(cells(load(args.candidate), counter), args.filter)
        moved = deltas(baseline, candidate)
    except CaptureError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return EXIT_INPUT
    print(f"counter: {counter}")
    print_unmatched(sorted(set(candidate) - set(baseline)))
    print_regressions(baseline, moved, args.tolerance)
    return verdict(moved, args.tolerance)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare two benchmark captures against the success-rate non-regression gate.",
        epilog=EXIT_HELP,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("baseline", type=Path, help="the capture taken before the change")
    parser.add_argument("candidate", type=Path, help="the capture taken after it")
    parser.add_argument("--filter", default=None,
                        help="restrict the gated set to cell names matching this regular expression")
    parser.add_argument("--counter", default=None,
                        help=f"success-rate counter to read (default: whichever of "
                             f"{', '.join(KNOWN_COUNTERS)} the baseline carries)")
    parser.add_argument("--tolerance", type=float, default=DEFAULT_TOLERANCE,
                        help=f"delta magnitude counting as unchanged (default: {DEFAULT_TOLERANCE})")
    return parser.parse_args(argv)


if __name__ == "__main__":
    raise SystemExit(run(parse_args(sys.argv[1:])))
