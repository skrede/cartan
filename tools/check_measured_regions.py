#!/usr/bin/env python3
"""Refuse a checked kinematics entry point inside a benchmark's measured block.

A benchmark cell measures whatever sits between ``for (auto _ : state)`` and its
closing brace. The checked forward-kinematics, Jacobian and velocity entry points
validate their arguments on every call, so one of them inside that block times
the guard as well as the kinematics and shifts a published number under an
unchanged benchmark name -- a change no compiler and no test can see. Each has an
``_unchecked`` sibling with the same body and no guard; that is what a measured
block calls, with the precondition established once outside it.

Reports ``file:line`` for every violation and exits non-zero if there is one.

Two limits worth stating rather than implying. It recognizes only the
google-benchmark measured block; a hand-rolled timing loop (the ``perf_fk_*``
mains) carries no syntactic marker and is not inspected. And it strips ``//``
comments naively, so an entry-point name inside a string literal would be a false
positive; none exists in the tree today.
"""

import argparse
import pathlib
import re
import sys

MEASURED_BLOCK_OPEN = re.compile(r"for\s*\(\s*auto\s+_\s*:\s*state\s*\)")

CHECKED_CALL = re.compile(
    r"\b(?:cartan::)?("
    r"forward_kinematics_matrix"
    r"|forward_kinematics"
    r"|space_jacobian"
    r"|body_jacobian"
    r"|end_effector_velocity"
    r")\(")


def strip_comment(line):
    """Drop a trailing line comment and a macro continuation backslash."""
    cut = line.find("//")
    if cut >= 0:
        line = line[:cut]
    return line.rstrip().removesuffix("\\")


def findings_in(path):
    """Yield (line_number, text) for each checked call inside a measured block."""
    found = []
    depth = None
    for number, raw in enumerate(path.read_text().split("\n"), start=1):
        line = strip_comment(raw)
        if depth is None:
            if MEASURED_BLOCK_OPEN.search(line):
                depth = 0
            continue
        depth += line.count("{") - line.count("}")
        match = CHECKED_CALL.search(line)
        if match:
            found.append((number, raw.strip()))
        if depth <= 0 and "}" in line:
            depth = None
    return found


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--benchmarks-root", required=True, type=pathlib.Path)
    args = parser.parse_args()

    sources = sorted(
        p for p in args.benchmarks_root.rglob("*")
        if p.suffix in (".cpp", ".h") and "third_party" not in p.parts)

    violations = []
    for source in sources:
        for number, text in findings_in(source):
            violations.append(f"{source}:{number}: {text}")

    if violations:
        print("checked entry point inside a measured block:", file=sys.stderr)
        print("\n".join(violations), file=sys.stderr)
        return 1

    print(f"measured regions clean in {len(sources)} benchmark translation units")
    return 0


if __name__ == "__main__":
    sys.exit(main())
