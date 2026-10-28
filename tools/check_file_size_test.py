#!/usr/bin/env python3
"""Fixture repositories exercising every verdict the file-size gate can reach."""

import subprocess
import sys
import tempfile
from pathlib import Path

GATE = Path(__file__).resolve().parent / "check_file_size.py"
WHY = "one cohesive unit whose shared state splitting would scatter across files"
HEADER = "# register\n\n| Path | Lines at registration | Why |\n| --- | --- | --- |\n"


def build(root: Path, sources: dict[str, int], rows: list, extra: str = "") -> None:
    subprocess.run(["git", "-C", str(root), "init", "-q"], check=True)
    for name, lines in sources.items():
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("// x\n" * lines)
    subprocess.run(["git", "-C", str(root), "add", "-A"], check=True, capture_output=True)
    table = "".join(f"| {p} | {n} | {why} |\n" for p, n, why in rows)
    (root / "EXCEPTIONS.md").write_text(HEADER + table + extra)


def run_gate(root: Path, *args: str) -> tuple[int, str]:
    done = subprocess.run([sys.executable, str(GATE), "--source-root", str(root), *args],
                          capture_output=True, text=True)
    return done.returncode, done.stdout + done.stderr


CASES = [
    ("every overage registered", {"lib/big.h": 205}, [("lib/big.h", 205, WHY)], "", 0, "1 registered"),
    ("unregistered overage", {"lib/big.h": 205}, [], "", 1, "is not registered"),
    ("a .cpp overage is seen too", {"lib/big.cpp": 205}, [], "", 1, "lib/big.cpp"),
    ("a path merely resembling the vendored tree is not exempt",
     {"benchmarks/third_party_helpers/big.h": 205}, [], "", 1, "third_party_helpers"),
    ("the vendored tree is exempt",
     {"benchmarks/third_party/big.h": 900, "lib/big.h": 205}, [("lib/big.h", 205, WHY)], "", 0, "1 reg"),
    ("boundary: the ceiling itself passes", {"lib/edge.h": 200}, [], "", 0, "0 registered overages"),
    ("boundary: one line over fails", {"lib/edge.h": 201}, [], "", 1, "is 201 lines"),
    ("stale row, file under the ceiling",
     {"lib/small.h": 10, "lib/big.h": 205}, [("lib/small.h", 205, WHY), ("lib/big.h", 205, WHY)],
     "", 3, "at or under the"),
    ("stale row, no such file", {"lib/big.h": 205},
     [("lib/big.h", 205, WHY), ("lib/gone.h", 205, WHY)], "", 3, "no tracked file has that path"),
    ("grown past the allowance", {"lib/big.h": 300}, [("lib/big.h", 205, WHY)], "", 4, "has grown to"),
    ("recorded count above the file's length", {"lib/big.h": 205}, [("lib/big.h", 300, WHY)], "",
     7, "records 300 lines but the file is 205"),
    ("a row with no count column", {"lib/big.h": 205}, [], "| lib/big.h | a reason |\n", 5,
     "needs a path, a recorded line count and a reason"),
    ("a row with an empty reason", {"lib/big.h": 205}, [("lib/big.h", 205, "")], "", 5, "reason is empty"),
    ("a reason restating the count", {"lib/big.h": 205}, [("lib/big.h", 205, "it is 205 lines")],
     "", 5, "restates the file's line count"),
    ("a reason naming no split harm", {"lib/big.h": 205}, [("lib/big.h", 205, "a large file")],
     "", 5, "must name what splitting"),
    ("a reason that is only an admission", {"lib/big.h": 205},
     [("lib/big.h", 205, "over the ceiling and nobody has decided what to do")], "", 5,
     "must name what splitting"),
    ("a reason promising a future split", {"lib/big.h": 205},
     [("lib/big.h", 205, "cohesive today; splitting is planned")], "", 5, "promises future work"),
    ("a path outside the scope", {"lib/big.h": 205},
     [("lib/big.h", 205, WHY), ("docs/notes.txt", 205, WHY)], "", 5, "outside the checked scope"),
    ("a path registered twice", {"lib/big.h": 205},
     [("lib/big.h", 205, WHY), ("lib/big.h", 205, WHY)], "", 5, "registered twice"),
    ("a path carrying a space", {"lib/my file.h": 205}, [("lib/my file.h", 205, WHY)], "", 0, "1 reg"),
    ("a path in backticks", {"lib/big.h": 205}, [("`lib/big.h`", 205, WHY)], "", 0, "1 registered"),
    ("a path outside ASCII", {"lib/naïve.h": 205}, [("lib/naïve.h", 205, WHY)], "", 0, "1 registered"),
    ("no C++ file anywhere", {"README.md": 5}, [], "", 6, "no tracked C++ file"),
]


def check(label: str, code: int, output: str, want_code: int, want_text: str) -> bool:
    if code == want_code and want_text in output:
        return True
    print(f"FAIL {label}: exit {code} (wanted {want_code})\n{output.strip()}", file=sys.stderr)
    return False


def run_table(root: Path) -> int:
    failures = 0
    for label, sources, rows, extra, want_code, want_text in CASES:
        case = root / label.replace(" ", "_").replace(",", "").replace("'", "")
        case.mkdir()
        build(case, sources, rows, extra)
        code, output = run_gate(case)
        failures += not check(label, code, output, want_code, want_text)
    return failures


def run_tree_faults(root: Path) -> int:
    failures = 0
    absent = root / "absent"
    absent.mkdir()
    build(absent, {"lib/big.h": 205, "lib/kept.h": 10}, [("lib/big.h", 205, WHY)])
    (absent / "lib/big.h").unlink()
    failures += not check("a tracked file missing from the working tree", *run_gate(absent),
                          6, "tracked but absent from the working tree")

    bare = root / "no_register"
    bare.mkdir()
    build(bare, {"lib/big.h": 205}, [])
    (bare / "EXCEPTIONS.md").unlink()
    failures += not check("the register itself missing", *run_gate(bare), 5, "does not exist")
    return failures


def run_counting(root: Path) -> int:
    failures = 0
    tail = root / "no_trailing_newline"
    tail.mkdir()
    build(tail, {"lib/big.h": 200}, [])
    (tail / "lib/big.h").write_text("// x\n" * 200 + "// last")
    failures += not check("a final line with no newline still counts", *run_gate(tail), 1, "is 201 lines")

    printer = root / "printer"
    printer.mkdir()
    build(printer, {"lib/big.h": 205}, [])
    before = (printer / "EXCEPTIONS.md").read_text()
    code, output = run_gate(printer, "--print-overages")
    unchanged = (printer / "EXCEPTIONS.md").read_text() == before
    failures += not check("the print flag writes nothing", code, output if unchanged else "wrote",
                          0, "| lib/big.h | 205 |")
    return failures


def main() -> int:
    with tempfile.TemporaryDirectory() as workspace:
        root = Path(workspace)
        failures = run_table(root) + run_tree_faults(root) + run_counting(root)
    total = len(CASES) + 4
    print(f"{total - failures} of {total} size-gate fixtures behaved as specified")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
