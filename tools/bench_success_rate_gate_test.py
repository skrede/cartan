#!/usr/bin/env python3
"""Synthetic captures exercising every verdict the success-rate gate can reach."""

import json
import subprocess
import sys
import tempfile
from pathlib import Path

GATE = Path(__file__).resolve().parent / "bench_success_rate_gate.py"

BASE = {"bm_a": 95.0, "bm_b": 90.0, "bm_c": 80.0}


def write(path: Path, measured: dict, counter: str = "Success_rate", repetitions: int = 1) -> Path:
    entries = [{"name": name, "run_name": name, "run_type": "iteration", counter: value}
               for name, value in measured.items() for _ in range(repetitions)]
    path.write_text(json.dumps({"context": {}, "benchmarks": entries}), encoding="utf-8")
    return path


def run_gate(*args: str) -> tuple[int, str]:
    done = subprocess.run([sys.executable, str(GATE), *args], capture_output=True, text=True)
    return done.returncode, done.stdout + done.stderr


CASES = [
    ("an identical capture is accepted", dict(BASE), 0, "summed delta: +0.00 pp"),
    ("an identical capture reports no regression",
     dict(BASE), 0, "worst single-cell regression: +0.00 pp"),
    ("one cell one and a half points below baseline with a raised total",
     {"bm_a": 93.5, "bm_b": 92.0, "bm_c": 80.0}, 1, "clause (ii) per-cell floor -1.00 pp: FAIL"),
    ("a summed regression with every cell inside the floor",
     {"bm_a": 94.6, "bm_b": 89.6, "bm_c": 79.6}, 1, "clause (i)  net non-regression: FAIL"),
    ("a five point gain beside an exactly one point loss",
     {"bm_a": 100.0, "bm_b": 89.0, "bm_c": 80.0}, 0, "clause (ii) per-cell floor -1.00 pp: PASS"),
    ("a cell improving is never penalized", {"bm_a": 99.0, "bm_b": 90.0, "bm_c": 80.0}, 0, "no cell regressed"),
    ("a baseline cell absent from the candidate is a hard error",
     {"bm_a": 95.0, "bm_b": 90.0}, 3, "would be flattered by their absence"),
    ("a candidate cell absent from the baseline is reported and excluded",
     dict(BASE, bm_d=10.0), 0, "not in the baseline and are excluded from the sum"),
]


def check(label: str, code: int, output: str, want_code: int, want_text: str) -> bool:
    if code == want_code and want_text in output:
        print(f"ok   gate exit {code}  {label}")
        return True
    print(f"FAIL {label}: exit {code} (wanted {want_code})\n{output.strip()}", file=sys.stderr)
    return False


def run_table(root: Path) -> int:
    baseline = write(root / "baseline.json", BASE)
    failures = 0
    for index, (label, measured, want_code, want_text) in enumerate(CASES):
        candidate = write(root / f"candidate_{index}.json", measured)
        code, output = run_gate(str(baseline), str(candidate))
        failures += not check(label, code, output, want_code, want_text)
    return failures


def run_malformed(root: Path) -> int:
    baseline = write(root / "malformed_baseline.json", BASE)
    failures = 0

    truncated = root / "truncated.json"
    truncated.write_text('{"benchmarks": [{"name": "bm_a", "Success_rate": 95.0}', encoding="utf-8")
    failures += not check("a truncated capture is an error", *run_gate(str(baseline), str(truncated)),
                          3, "cannot read")

    empty = write(root / "empty.json", {})
    failures += not check("an empty capture is an error", *run_gate(str(baseline), str(empty)),
                          3, "carries no non-empty benchmarks array")

    uncounted = root / "uncounted.json"
    uncounted.write_text(json.dumps({"benchmarks": [{"name": "bm_a", "real_time": 1.0}]}),
                         encoding="utf-8")
    failures += not check("a capture with no success counter is an error",
                          *run_gate(str(uncounted), str(uncounted)), 3, "carries none of the")
    return failures


def run_reading(root: Path) -> int:
    baseline = write(root / "reading_baseline.json", BASE, repetitions=3)
    failures = 0

    aggregated = root / "aggregated.json"
    entries = json.loads(baseline.read_text(encoding="utf-8"))["benchmarks"]
    entries.append({"name": "bm_a_median", "run_name": "bm_a", "run_type": "aggregate",
                    "aggregate_name": "median", "Success_rate": 0.0})
    aggregated.write_text(json.dumps({"benchmarks": entries}), encoding="utf-8")
    failures += not check("an aggregate row does not enter the comparison",
                          *run_gate(str(baseline), str(aggregated)), 0, "summed delta: +0.00 pp")

    outside = write(root / "outside.json", {"bm_a": 95.0, "bm_b": 90.0, "bm_c": 70.0})
    failures += not check("the filter restricts the gated set",
                          *run_gate(str(baseline), str(outside), "--filter", "bm_[ab]"),
                          0, "over 2 gated cells")
    failures += not check("a regression outside the filter still fails unfiltered",
                          *run_gate(str(baseline), str(outside)), 1, "clause (ii)")

    renamed = write(root / "renamed.json", {"bm_a": 95.0}, counter="Success_pct")
    failures += not check("the counter name is resolved from the capture",
                          *run_gate(str(renamed), str(renamed)), 0, "counter: Success_pct")

    code, output = run_gate("--help")
    both = "net non-regression" in output and "per-cell floor" in output and "1.00" in output
    failures += not check("the help names both clauses and the floor", code,
                          output if both else "the help omits a clause", 0, "usage:")
    return failures


def main() -> int:
    with tempfile.TemporaryDirectory() as workspace:
        root = Path(workspace)
        failures = run_table(root) + run_malformed(root) + run_reading(root)
    total = len(CASES) + 8
    print(f"{total - failures} of {total} success-rate gate fixtures behaved as specified")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
