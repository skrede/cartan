#!/usr/bin/env python3
"""Record sets built to reach one verdict each in the study report script.

A gate only ever run against records that pass cannot tell you it still refuses.
Each fixture below is a small capture constructed to hit exactly one behavior --
a refusal, an absence rendered as a clause, or a summary whose wording is the
claim -- and each has been observed going red against a script with the
corresponding check removed. A fixture never seen to fail proves only that the
tool did not crash.
"""

import subprocess
import sys
import tempfile
from pathlib import Path

SCRIPT = Path(__file__).resolve().parent / "bench_study_report.py"

FIXTURES = Path(__file__).resolve().parent / "bench_study_report_fixtures"

TARGETS = ["--from", "targets"]

AGGREGATES = ["--from", "aggregates"]

CASES = [
    ("a paired set with no common accepted target is reported, not divided by",
     "empty_pair_set", TARGETS, 0, "no paired target (n = 0)"),
    ("a declared participant the records lack, with no recorded absence, is an error",
     "missing_participant", TARGETS, 1, "declares the participant 'trac_ik'"),
    ("a declared participant recorded as absent is rendered as an absence and its reason",
     "absent_with_reason", TARGETS, 0, "trac_ik is absent from this table"),
    ("a kernel count on a participant the harness does not drive is refused",
     "kernel_on_uncountable", TARGETS, 1, "'trac_ik' is declared without a countable kernel"),
    ("a participant the records carry and nothing declares is refused",
     "undeclared_participant", TARGETS, 1, "carry rows for ['argmin_lbfgs']"),
    ("a verdict disagreeing with the self-report in both directions is summarized as both",
     "verdict_disagreement", TARGETS, 0, "| 1 / 6 (16.67%) | 1 / 6 (16.67%) |"),
    ("a non-finite error on an accepted solve is refused by line and column",
     "non_finite_error", TARGETS, 1, "column pos_err_m"),
    ("rows from two tables in one file are refused, naming both",
     "cross_table", TARGETS, 1, "names table 'a' in a report about table 'c'"),
    ("a line carrying the wrong number of fields is refused by line",
     "unbalanced_fields", TARGETS, 1, "line 10: carries a different number of fields"),
    ("a row standing on bounds no description declared is excluded and counted",
     "synthetic_provenance", TARGETS, 0, "Excluded from every figure here: 2 record rows"),
    ("a tier carrying no rows is refused as empty rather than as excluded",
     "empty_strata_tier", AGGREGATES, 1, "carries no rows"),
    ("the two tiers rebuilding one table agree figure for figure",
     "good_capture", AGGREGATES + ["--cross-check"], 0, "published figures agree"),
    ("an aggregate tier disagreeing with its per-target rows fails the cross-check",
     "tier_disagreement", AGGREGATES + ["--cross-check"], 1, "the two record tiers disagree"),
]


def run(fixture, arguments, out):
    done = subprocess.run(
        [sys.executable, str(SCRIPT), "--root", str(FIXTURES / fixture), "--table", "c",
         "--out", str(out), *arguments],
        capture_output=True, text=True)
    rendered = out.read_text(encoding="utf-8") if out.exists() else ""
    return done.returncode, done.stdout + done.stderr + rendered


def check(label, code, output, want_code, want_text):
    if code == want_code and want_text in output:
        print(f"ok   exit {code}  {label}")
        return True
    print(f"FAIL {label}: exit {code} (wanted {want_code}, wanting {want_text!r})\n"
          f"{output.strip()}", file=sys.stderr)
    return False


def run_fixtures(workspace):
    failures = 0
    for index, (label, fixture, arguments, want_code, want_text) in enumerate(CASES):
        out = workspace / f"table_{index}.md"
        code, output = run(fixture, arguments, out)
        failures += not check(label, code, output, want_code, want_text)
    return failures


def run_paths(workspace):
    """The script is offered to third parties to run against their own captures, so
    an input path that resolves out of the declared root is a refusal rather than a
    read."""
    out = workspace / "escape.md"
    escaping = FIXTURES / "verdict_disagreement" / "table_c_targets.csv"
    code, output = run("good_capture", TARGETS + ["--input", str(escaping)], out)
    return not check("an input resolving outside the declared root is refused", code, output,
                     1, "resolves outside the declared root")


def main():
    with tempfile.TemporaryDirectory() as directory:
        workspace = Path(directory)
        failures = run_fixtures(workspace) + run_paths(workspace)
    total = len(CASES) + 1
    print(f"{total - failures} of {total} study report fixtures behaved as specified")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
