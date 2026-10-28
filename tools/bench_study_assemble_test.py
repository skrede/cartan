#!/usr/bin/env python3
"""Captures built to reach one verdict each in the assembler.

The assembler's whole reason for existing is that a sweep's conditions are
asserted by a hundred and forty-four separate records, so the cases that matter
are the ones where those records disagree. Each fixture below is a two-process
capture constructed to hit exactly one behavior, and each has been observed going
red against an assembler with the corresponding check removed.
"""

import subprocess
import sys
import tempfile
from pathlib import Path

SCRIPT = Path(__file__).resolve().parent / "bench_study_assemble.py"

FIXTURES = Path(__file__).resolve().parent / "bench_study_assemble_fixtures"

CASES = [
    ("two processes agreeing on every invariant assemble into one record set",
     "agreeing", 0, "merged 2 environment records into one"),
    ("a machine whose boost was re-enabled mid-capture refuses, naming both values",
     "boost_drifted", 1, "frequency_boost differs across the capture"),
    ("a machine value the capture could not read refuses rather than being merged away",
     "unread_value", 1, "could not read ['smt']"),
    ("two tier files with different column orders refuse rather than concatenate",
     "column_drift", 1, "columns differ from"),
    ("a table with no cells record refuses rather than assembling an empty tier",
     "no_cells", 1, "has no cells record to report"),
]


def run(fixture, out):
    done = subprocess.run(
        [sys.executable, str(SCRIPT), "--capture", str(FIXTURES / fixture),
         "--table", "c", "--tier", "cells", "--out", str(out)],
        capture_output=True, text=True)
    return done.returncode, done.stdout + done.stderr


def check(label, code, output, want_code, want_text):
    if code == want_code and want_text in output:
        print(f"ok   exit {code}  {label}")
        return True
    print(f"FAIL {label}: exit {code} (wanted {want_code}, wanting {want_text!r})\n"
          f"{output.strip()}", file=sys.stderr)
    return False


def main():
    failures = 0
    with tempfile.TemporaryDirectory() as directory:
        for index, (label, fixture, want_code, want_text) in enumerate(CASES):
            out = Path(directory) / f"case_{index}"
            code, output = run(fixture, out)
            failures += not check(label, code, output, want_code, want_text)
    total = len(CASES)
    print(f"{total - failures} of {total} study assembler fixtures behaved as specified")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
