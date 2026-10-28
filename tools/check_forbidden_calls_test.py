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

PIN = "cartan_acquire_meios()\nset(MEIOS_EVAL_PYTHON_SUPPORT OFF)\n"
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
      "CMakeLists.txt": "set(MEIOS_EVAL_PYTHON_SUPPORT ON)\n"}, 1, "CMakeLists.txt:1"),
    ("the pin deleted",
     {"lib/urdf.h": GOOD_CALL, "cmake/Supplier.cmake": "cartan_acquire_meios()\n"},
     3, "restore set(MEIOS_EVAL_PYTHON_SUPPORT OFF)"),
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
      "cmake/Supplier.cmake": "# never set(MEIOS_EVAL_PYTHON_SUPPORT ON)\n" + PIN},
     0, "no forbidden call"),
    ("the option enabled through option()",
     {"lib/urdf.h": GOOD_CALL, "cmake/Supplier.cmake": PIN,
      "CMakeLists.txt": 'option(MEIOS_EVAL_PYTHON_SUPPORT "evaluate with a python backend" YES)\n'},
     1, "CMakeLists.txt:1"),
    ("the option enabled through a cache entry",
     {"lib/urdf.h": GOOD_CALL, "cmake/Supplier.cmake": PIN,
      "CMakeLists.txt": 'set(MEIOS_EVAL_PYTHON_SUPPORT 1 CACHE BOOL "evaluate expressions")\n'},
     1, "CMakeLists.txt:1"),
    ("the option enabled through a preset cache variable",
     {"lib/urdf.h": GOOD_CALL, "cmake/Supplier.cmake": PIN,
      "CMakePresets.json": '{"cacheVariables": {"MEIOS_EVAL_PYTHON_SUPPORT": "true"}}\n'},
     1, "CMakePresets.json:1"),
    ("a preset pinning the option off satisfies the pin on its own",
     {"lib/urdf.h": GOOD_CALL,
      "CMakePresets.json": '{"cacheVariables": {"MEIOS_EVAL_PYTHON_SUPPORT": "OFF"}}\n'},
     0, "CMakePresets.json"),
    ("a C++ file outside the checked areas is not scanned",
     {"lib/urdf.h": GOOD_CALL, "cmake/Supplier.cmake": PIN,
      "profiling/probe.cpp": "meios::load_into(path, opts, sink, log);\n"}, 0, "no forbidden call"),
    ("a tree holding neither a C++ nor a build file",
     {"README.md": "meios::load_into(path, opts, sink, log)\n"}, 6, "no tracked C++ or build file"),

    ("a version argument on the supplier's find_package call",
     {"lib/urdf.h": GOOD_CALL,
      "cmake/Supplier.cmake": PIN + "find_package(meios 1.2.3 CONFIG QUIET GLOBAL)\n"},
     1, "a version argument on the supplier's find_package call"),
    ("the revision pin naming a branch instead of a full SHA",
     {"lib/urdf.h": GOOD_CALL,
      "cmake/Supplier.cmake": PIN + "set(CARTAN_MEIOS_REVISION master)\n"},
     1, "is not a full 40-character hex commit SHA"),
    ("pugixml re-entering the direct dependency set",
     {"lib/urdf.h": GOOD_CALL, "cmake/Supplier.cmake": PIN,
      "lib/cartan-urdf/CMakeLists.txt": "find_package(pugixml CONFIG REQUIRED)\n"},
     1, "pugixml enters cartan only transitively"),
    ("a target other than cartan_urdf linking the supplier",
     {"lib/urdf.h": GOOD_CALL, "cmake/Supplier.cmake": PIN,
      "lib/cartan-lie/CMakeLists.txt": "target_link_libraries(cartan_lie INTERFACE meios::urdf)\n"},
     1, "only cartan_urdf may name the supplier"),
    ("cartan_urdf itself linking the supplier is the one allowed target",
     {"lib/urdf.h": GOOD_CALL, "cmake/Supplier.cmake": PIN,
      "lib/cartan-urdf/CMakeLists.txt": "target_link_libraries(cartan_urdf INTERFACE meios::urdf)\n"},
     0, "no forbidden call"),

    ("a decorative attribute", {"lib/attr.h": "[[nodiscard]] int f();\n", "cmake/Supplier.cmake": PIN},
     1, "[[nodiscard]] is a decorative attribute"),
    ("a compiler-required attribute is accepted",
     {"lib/attr.h": "[[noreturn]] void die();\n", "cmake/Supplier.cmake": PIN},
     0, "no forbidden call"),

    ("axis squaredNorm re-derived in the model sink",
     {"lib/cartan-urdf/include/cartan/urdf/detail/model_sink.h": "auto n = axis.squaredNorm();\n",
      "cmake/Supplier.cmake": PIN}, 1, "the axis gate lives in build.h"),
    ("axis isfinite re-derived in the model sink",
     {"lib/cartan-urdf/include/cartan/urdf/detail/model_sink.h":
      "if (!std::isfinite(axis_sq)) { return; }\n", "cmake/Supplier.cmake": PIN},
     1, "the axis gate lives in build.h"),
    ("screw_axis::revolute called from the model sink",
     {"lib/cartan-urdf/include/cartan/urdf/detail/model_sink.h":
      "auto a = screw_axis<double>::revolute(axis, point);\n", "cmake/Supplier.cmake": PIN},
     1, "bypasses the axis gate in build.h"),
    ("squaredNorm in build.h itself is not scanned",
     {"lib/cartan-urdf/include/cartan/urdf/build.h": "auto n = axis.squaredNorm();\n",
      "cmake/Supplier.cmake": PIN}, 0, "no forbidden call"),
    ("isfinite on an unrelated scalar in the detail directory is not axis re-derivation",
     {"lib/cartan-urdf/include/cartan/urdf/detail/narrowing.h":
      "if (!std::isfinite(value)) { return; }\n", "cmake/Supplier.cmake": PIN},
     0, "no forbidden call"),

    ("the process-authority sentence missing from the guide",
     {"lib/urdf.h": GOOD_CALL, "cmake/Supplier.cmake": PIN,
      "docs/guides/robot-descriptions.md":
      "### The unrestricted Python backend\nNo warning here.\n"},
     1, "does not appear in the unrestricted-backend section verbatim"),
    ("the process-authority sentence present but outside its section",
     {"lib/urdf.h": GOOD_CALL, "cmake/Supplier.cmake": PIN,
      "docs/guides/robot-descriptions.md":
      "### The restricted Python backend\n"
      "An expression inside a description evaluates with the process's own authority.\n\n"
      "### The unrestricted Python backend\nNothing here.\n"},
     1, "appears outside the unrestricted-backend section"),
    ("the guide recommending the unrestricted backend",
     {"lib/urdf.h": GOOD_CALL, "cmake/Supplier.cmake": PIN,
      "docs/guides/robot-descriptions.md":
      "### The unrestricted Python backend\n"
      "An expression inside a description evaluates with the process's own authority.\n\n"
      "We recommend the unrestricted backend.\n"},
     1, "recommends or prefers the unrestricted evaluator"),
    ("the guide with the sentence correctly placed and no endorsement",
     {"lib/urdf.h": GOOD_CALL, "cmake/Supplier.cmake": PIN,
      "docs/guides/robot-descriptions.md":
      "### The unrestricted Python backend\n"
      "An expression inside a description evaluates with the process's own authority.\n"},
     0, "no forbidden call"),
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
