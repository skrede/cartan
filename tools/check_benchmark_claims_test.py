#!/usr/bin/env python3
"""Fixture documents driving the benchmark-claim gate to each verdict it can reach.

A gate only ever run against a document that passes cannot tell you it still
fails. Every violating fixture here is otherwise clean, so the finding asserted
is unambiguously the rule under test.

Each fixture is additionally run against a gate with its own rule removed, and
the run is required to come back clean. That is what proves the fixture is held
by the rule rather than passing because the check was never reached.
"""

import re
import subprocess
import sys
import tempfile
from pathlib import Path

GATE = Path(__file__).resolve().parent / "check_benchmark_claims.py"

RECORD = "robot,solver,value\nur3e,cartan,1\n"
SOURCED = "<!-- source: evidence/cells.csv -->\n\n| robot | wall |\n|---|---|\n| UR3e | 27 ns |\n"

# Removing a rule's own line from the gate is what each fixture is checked
# against: if the fixture still fails, it was being held by some other rule.
WEAKENINGS = {
    "unsourced claim": 'findings.append(Finding(number, "unsourced claim"',
    "missing record": 'findings.append(Finding(number, "missing record"',
    "escaping marker": 'findings.append(Finding(number, "escaping marker"',
    "multiplier in the study": 'findings.append(Finding(number, "multiplier in the study"',
    "planning reference": 'findings.append(Finding(number, "planning reference"',
}


def build(root: Path, doc: str, records: dict[str, str]) -> tuple[Path, Path]:
    evidence = root / "evidence"
    evidence.mkdir(parents=True, exist_ok=True)
    for name, text in records.items():
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
    document = root / "benchmarks.md"
    document.write_text(doc)
    return document, evidence


def run_gate(document: Path, evidence: Path, root: Path, gate: Path = GATE) -> tuple[int, str]:
    done = subprocess.run(
        [sys.executable, str(gate), "--doc", str(document),
         "--evidence-root", str(evidence), "--repo-root", str(root)],
        capture_output=True, text=True)
    return done.returncode, done.stdout + done.stderr


def weakened_gate(root: Path, rule: str) -> Path:
    """The gate with one rule's finding suppressed, and nothing else changed.

    The statement is replaced by `pass` at its own indentation, spanning however
    many lines its parentheses take to close, so the rules around it are left
    exactly as they were.
    """
    source = GATE.read_text().splitlines(keepends=True)
    needle = WEAKENINGS[rule]
    out, depth, replacing = [], 0, False
    for line in source:
        if not replacing and needle in line:
            replacing = True
            out.append(line.split("findings.append")[0] + "pass\n")
            depth = line.count("(") - line.count(")")
            if depth <= 0:
                replacing = False
            continue
        if replacing:
            depth += line.count("(") - line.count(")")
            if depth <= 0:
                replacing = False
            continue
        out.append(line)
    path = root / f"weak_{re.sub(r'[^a-z]', '_', rule)}.py"
    path.write_text("".join(out))
    return path


CASES = [
    ("a figure under a marker that resolves",
     SOURCED, {"evidence/cells.csv": RECORD}, 0, "every figure is sourced", None),

    ("a figure outside every marker's scope",
     "## Results\n\nThe solver reaches 98.4% on that arm.\n",
     {"evidence/cells.csv": RECORD}, 1, "unsourced claim", "unsourced claim"),

    ("a marker naming a record that is not there",
     "<!-- source: evidence/absent.csv -->\n\n| robot | wall |\n|---|---|\n| UR3e | 27 ns |\n",
     {"evidence/cells.csv": RECORD}, 1, "names no file under", "missing record"),

    ("a marker escaping the evidence directory",
     "<!-- source: evidence/../../secrets.csv -->\n\n| robot | wall |\n|---|---|\n| UR3e | 27 ns |\n",
     {"evidence/cells.csv": RECORD}, 1, "resolves outside", "escaping marker"),

    ("a multiplier inside the iterative study",
     "## The iterative study\n\n<!-- source: evidence/cells.csv -->\n\n"
     "The kernels are 1.53x faster there.\n",
     {"evidence/cells.csv": RECORD}, 1, "multiplier in the study", "multiplier in the study"),

    ("a multiplier outside the study is left alone",
     "## Jacobian\n\n<!-- source: evidence/cells.csv -->\n\nIt is 2.24x faster than the reference.\n",
     {"evidence/cells.csv": RECORD}, 0, "every figure is sourced", None),

    ("a phase number anywhere in the document",
     SOURCED + "\nCaptured during phase 59.\n",
     {"evidence/cells.csv": RECORD}, 1, "planning reference", "planning reference"),

    ("a requirement identifier anywhere in the document",
     SOURCED + "\nThis satisfies BENCH-09.\n",
     {"evidence/cells.csv": RECORD}, 1, "planning reference", "planning reference"),

    ("a milestone name anywhere in the document",
     SOURCED + "\nShipped in the v0.4.3 milestone.\n",
     {"evidence/cells.csv": RECORD}, 1, "planning reference", "planning reference"),

    ("a marker does not vouch for the section after the next heading",
     SOURCED + "\n## Later\n\nIt settles at 2.0 rad there.\n",
     {"evidence/cells.csv": RECORD}, 1, "unsourced claim", "unsourced claim"),

    ("a bare count in prose is not a measurement",
     "## Method\n\nThe study runs three tables over six strata.\n",
     {"evidence/cells.csv": RECORD}, 0, "every figure is sourced", None),
]


def main() -> int:
    failures = 0
    for name, doc, records, expect_code, expect_text, rule in CASES:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            document, evidence = build(root, doc, records)
            code, output = run_gate(document, evidence, root)
            if code != expect_code or expect_text not in output:
                failures += 1
                print(f"FAIL {name}: exit {code} (wanted {expect_code}), output:\n{output}")
                continue
            if rule is None:
                print(f"ok   {name}")
                continue
            weak = weakened_gate(root, rule)
            weak_code, weak_output = run_gate(document, evidence, root, weak)
            if weak_code == 0:
                print(f"ok   {name}\n     red against the gate, clean without the "
                      f"{rule!r} rule; observed: {output.strip().splitlines()[0]}")
            else:
                failures += 1
                print(f"FAIL {name}: still fails with the {rule!r} rule removed:\n{weak_output}")

    print(f"\n{len(CASES)} fixture(s), {failures} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
