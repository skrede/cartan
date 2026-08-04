#!/usr/bin/env python3
"""Fixture repositories driving the forbidden-call gate to each verdict it can reach.

A gate only ever run against a tree that passes cannot tell you it still fails.
Every violating fixture here is otherwise clean, so the message asserted is
unambiguously the rule under test.
"""

import subprocess
import sys
import tempfile
from pathlib import Path

GATE = Path(__file__).resolve().parent / "check_forbidden_calls.py"

PIN = "cartan_acquire_meios()\nset(MEIOS_BUILD_EVAL_PYTHON OFF)\n"
GOOD_CALL = "auto loaded = meios::load(path, opts.description, log);\n"


def build(root: Path, sources: dict[str, str]) -> None:
    subprocess.run(["git", "-C", str(root), "init", "-q"], check=True)
    for name, text in sources.items():
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
    subprocess.run(["git", "-C", str(root), "add", "-A"], check=True, capture_output=True)


def run_gate(root: Path) -> tuple[int, str]:
    done = subprocess.run([sys.executable, str(GATE), "--source-root", str(root)],
                          capture_output=True, text=True)
    return done.returncode, done.stdout + done.stderr


CASES = [
    ("a clean tree with a legitimate call and a legitimate pin",
     {"lib/urdf.h": GOOD_CALL, "cmake/Supplier.cmake": PIN}, 0, "is pinned off in"),
    ("the sink-less two-argument load",
     {"lib/urdf.h": "auto loaded = meios::load(path, opts);\n", "cmake/Supplier.cmake": PIN},
     1, "lib/urdf.h:1"),
    ("the one-argument load, which is the same sink-less overload",
     {"lib/urdf.h": "auto loaded = meios::load(path);\n", "cmake/Supplier.cmake": PIN},
     1, "with 1 argument(s)"),
    ("load_into",
     {"lib/urdf.h": "meios::load_into(path, opts, sink, log);\n", "cmake/Supplier.cmake": PIN},
     1, "forwards one of the five fields"),
    ("the claim-erasing asset policy",
     {"lib/urdf.h": "opts.on_missing = meios::missing_asset::skip;\n",
      "cmake/Supplier.cmake": PIN}, 1, "clears the completeness claim"),
    ("the evaluation backend switched on",
     {"lib/urdf.h": GOOD_CALL, "cmake/Supplier.cmake": PIN,
      "CMakeLists.txt": "set(MEIOS_BUILD_EVAL_PYTHON ON)\n"}, 1, "CMakeLists.txt:1"),
    ("the pin deleted",
     {"lib/urdf.h": GOOD_CALL, "cmake/Supplier.cmake": "cartan_acquire_meios()\n"},
     3, "restore set(MEIOS_BUILD_EVAL_PYTHON OFF)"),
    ("the three-argument call spread over several lines",
     {"lib/urdf.h": "auto loaded = meios::load(\n    path,\n    opts,\n    log);\n",
      "cmake/Supplier.cmake": PIN}, 0, "1 tracked C++ file"),
    ("a C++ line comment naming a forbidden call",
     {"lib/urdf.h": "// meios::load_into(path, opts, sink, log) is never called\n" + GOOD_CALL,
      "cmake/Supplier.cmake": PIN}, 0, "no forbidden call"),
    ("a C++ block comment naming a forbidden call",
     {"lib/urdf.h": "/* meios::load(path, opts)\n   is never called */\n" + GOOD_CALL,
      "cmake/Supplier.cmake": PIN}, 0, "no forbidden call"),
    ("a CMake comment naming the option enabled",
     {"lib/urdf.h": GOOD_CALL,
      "cmake/Supplier.cmake": "# never set(MEIOS_BUILD_EVAL_PYTHON ON)\n" + PIN},
     0, "no forbidden call"),
    ("the option enabled through option()",
     {"lib/urdf.h": GOOD_CALL, "cmake/Supplier.cmake": PIN,
      "CMakeLists.txt": 'option(MEIOS_BUILD_EVAL_PYTHON "evaluate with a python backend" YES)\n'},
     1, "CMakeLists.txt:1"),
    ("the option enabled through a cache entry",
     {"lib/urdf.h": GOOD_CALL, "cmake/Supplier.cmake": PIN,
      "CMakeLists.txt": 'set(MEIOS_BUILD_EVAL_PYTHON 1 CACHE BOOL "evaluate expressions")\n'},
     1, "CMakeLists.txt:1"),
    ("the option enabled through a preset cache variable",
     {"lib/urdf.h": GOOD_CALL, "cmake/Supplier.cmake": PIN,
      "CMakePresets.json": '{"cacheVariables": {"MEIOS_BUILD_EVAL_PYTHON": "true"}}\n'},
     1, "CMakePresets.json:1"),
    ("a preset pinning the option off satisfies the pin on its own",
     {"lib/urdf.h": GOOD_CALL,
      "CMakePresets.json": '{"cacheVariables": {"MEIOS_BUILD_EVAL_PYTHON": "OFF"}}\n'},
     0, "CMakePresets.json"),
    ("a C++ file outside the checked areas is not scanned",
     {"lib/urdf.h": GOOD_CALL, "cmake/Supplier.cmake": PIN,
      "profiling/probe.cpp": "meios::load_into(path, opts, sink, log);\n"}, 0, "no forbidden call"),
    ("a tree holding neither a C++ nor a build file",
     {"README.md": "meios::load_into(path, opts, sink, log)\n"}, 6, "no tracked C++ or build file"),
]


def check(label: str, code: int, output: str, want_code: int, want_text: str) -> bool:
    if code == want_code and want_text in output:
        return True
    print(f"FAIL {label}: exit {code} (wanted {want_code})\n{output.strip()}", file=sys.stderr)
    return False


def main() -> int:
    failures = 0
    with tempfile.TemporaryDirectory() as workspace:
        for label, sources, want_code, want_text in CASES:
            case = Path(workspace) / label.replace(" ", "_").replace(",", "")
            case.mkdir()
            build(case, sources)
            code, output = run_gate(case)
            failures += not check(label, code, output, want_code, want_text)
    total = len(CASES)
    print(f"{total - failures} of {total} forbidden-call fixtures behaved as specified")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
