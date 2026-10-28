#!/usr/bin/env python3
"""Refuse a checked kinematics entry point inside a benchmark's measured block.

A benchmark cell measures whatever sits between ``for (auto _ : state)`` and its
closing brace. The checked forward-kinematics, Jacobian and velocity entry points
validate their arguments on every call, so one of them inside that block times
the guard as well as the kinematics and shifts a published number under an
unchanged benchmark name -- a change no compiler and no test can see. Each has an
``_unchecked`` sibling with the same body and no guard; that is what a measured
block calls, with the precondition established once outside it.

The leak can also be indirect: a fixture helper that calls a checked entry point
puts the same guard inside every measured block that calls the helper, and no
benchmark source names an entry point at all. ``--reach-root`` collects the
namespace-scope helpers that reach a checked entry point, and a call to one of
those inside a measured block is refused too.

Exit codes are distinct so a caller can tell a finding from a broken run:
0 clean, 1 a violation was found, 2 the check could not be trusted. Anything
else is a crash, and means the tool did not run rather than that the sources are
clean.

What it does NOT see, stated rather than implied:

* A hand-rolled timing loop. Only ``for (auto _ : state)`` is recognized, so the
  loops in ``perf_fk_cartan_ur3e.cpp`` and ``perf_fk_cartan_matrix_ur3e.cpp``
  are not inspected.
* Indirection deeper than one level, a helper defined inside a class or another
  function, or one reached through a template parameter or a function pointer.
* An entry point renamed or aliased; the five names are matched literally.
"""

import argparse
import pathlib
import re
import sys

MEASURED_BLOCK_OPEN = re.compile(r"for\s*\(\s*auto\s+_\s*:\s*state\s*\)")

ENTRY_POINTS = (
    "forward_kinematics_matrix",
    "forward_kinematics",
    "space_jacobian",
    "body_jacobian",
    "end_effector_velocity",
)

CHECKED_CALL = re.compile(r"\b(?:cartan::)?(?:" + "|".join(ENTRY_POINTS) + r")\(")
DEFINITION_NAME = re.compile(r"\b([A-Za-z_]\w*)\s*\(")
NOT_A_DEFINITION = re.compile(r"^\s*(namespace|struct|class|enum|union|using|typedef)\b")
OPENS_A_NAMESPACE = re.compile(r'^\s*(namespace\b|extern\s+"C")')


def strip_noise(text):
    """Blank out string literals and both comment forms, preserving the line count."""
    text = re.sub(r'"(?:\\.|[^"\\\n])*"', '""', text)
    text = re.sub(r"/\*.*?\*/", lambda m: "\n" * m.group(0).count("\n"), text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


def measured_lines(lines):
    """Line numbers inside a measured block, counting braces from the `for` line itself."""
    inside, depth, awaiting = set(), 0, False
    for number, line in enumerate(lines, start=1):
        if depth == 0 and not awaiting:
            match = MEASURED_BLOCK_OPEN.search(line)
            if not match:
                continue
            rest = line[match.end():]
            depth = rest.count("{") - rest.count("}")
            if depth > 0 or rest.strip():
                inside.add(number)
            else:
                awaiting = True
            continue
        inside.add(number)
        if awaiting:
            awaiting = False
            if "{" not in line:
                continue
        depth += line.count("{") - line.count("}")
        depth = max(depth, 0)
    return inside


def reaching_names(paths):
    """Namespace-scope function names whose body calls a checked entry point.

    `scope` tracks how many of the open braces belong to a namespace, so that
    "at namespace scope" stays true inside one -- which is where every helper
    this matters for is defined.
    """
    names = set()
    for path in paths:
        depth = scope = 0
        candidate = current = None
        reached = pending_namespace = False
        for line in strip_noise(path.read_text()).split("\n"):
            at_scope = depth == scope and current is None
            if at_scope and OPENS_A_NAMESPACE.match(line):
                pending_namespace = True
            elif at_scope and not NOT_A_DEFINITION.match(line):
                # A macro definition pastes its parameter into the name, so the
                # last identifier before `(` can be a bare suffix like `_matrix`.
                found = [f for f in DEFINITION_NAME.findall(line) if not f.startswith("_")]
                if found:
                    candidate = found[-1]
            opened, closed = line.count("{"), line.count("}")
            if opened > closed and depth == scope:
                if pending_namespace:
                    scope += 1
                    pending_namespace = False
                    candidate = None
                elif candidate:
                    current, reached = candidate, False
            if current and CHECKED_CALL.search(line):
                reached = True
            depth += opened - closed
            if current is not None and depth <= scope:
                if reached:
                    names.add(current)
                candidate = current = None
                reached = False
            scope = min(scope, depth)
    return names


def findings_in(path, indirect):
    """(line, text) for each refused call inside a measured block of one file."""
    raw = path.read_text().split("\n")
    clean = strip_noise(path.read_text()).split("\n")
    pattern = CHECKED_CALL
    if indirect:
        pattern = re.compile(
            CHECKED_CALL.pattern + r"|\b(?:" + "|".join(sorted(indirect)) + r")\(")
    return [(number, raw[number - 1].strip())
            for number in sorted(measured_lines(clean))
            if number <= len(clean) and pattern.search(clean[number - 1])]


def sources_under(root):
    """Every C++ source under root, with the vendored tree excluded by relative path."""
    return sorted(p for p in root.rglob("*")
                  if p.suffix in (".cpp", ".h")
                  and "third_party" not in p.relative_to(root).parts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--benchmarks-root", required=True, type=pathlib.Path)
    parser.add_argument("--reach-root", action="append", default=[], type=pathlib.Path)
    parser.add_argument("--min-sources", type=int, default=1)
    args = parser.parse_args()

    if not args.benchmarks_root.is_dir():
        print(f"refusing to report clean: {args.benchmarks_root} is not a directory",
              file=sys.stderr)
        return 2
    sources = sources_under(args.benchmarks_root)
    if len(sources) < args.min_sources:
        print(f"refusing to report clean: found {len(sources)} translation units under "
              f"{args.benchmarks_root}, expected at least {args.min_sources}",
              file=sys.stderr)
        return 2

    reach_sources = list(sources)
    for root in args.reach_root:
        if not root.is_dir():
            print(f"refusing to report clean: --reach-root {root} is not a directory",
                  file=sys.stderr)
            return 2
        reach_sources += sources_under(root)
    indirect = reaching_names(reach_sources)

    violations = [f"{source}:{number}: {text}"
                  for source in sources
                  for number, text in findings_in(source, indirect)]
    if violations:
        print("checked entry point reached from inside a measured block:", file=sys.stderr)
        print("\n".join(violations), file=sys.stderr)
        return 1

    reaching = f", {len(indirect)} helper(s) treated as reaching" if indirect else ""
    print(f"measured regions clean in {len(sources)} benchmark translation units{reaching}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
