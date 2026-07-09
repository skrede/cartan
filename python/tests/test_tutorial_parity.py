"""Python tutorial smoke tests and C++/Python tutorial CSV parity."""

from __future__ import annotations

import csv
import os
import subprocess
import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
PYTHON_ROOT = REPO_ROOT / "python"
TUTORIALS = PYTHON_ROOT / "tutorials"


def _env() -> dict[str, str]:
    env = os.environ.copy()
    env["PYTHONNOUSERSITE"] = "1"
    env["PYTHONPATH"] = str(PYTHON_ROOT)
    return env


def _run_python_tutorial(name: str, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(TUTORIALS / name), *args],
        cwd=REPO_ROOT,
        env=_env(),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )


def _read_csv(path: Path) -> dict[tuple[int, str], dict[str, str]]:
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert rows
    assert set(rows[0]) == {
        "target_index",
        "solver",
        "success",
        "wall_us",
        "pos_err",
        "multi_solutions",
    }
    return {(int(row["target_index"]), row["solver"]): row for row in rows}


def _cpp_tutorial_executable() -> Path | None:
    env_path = os.environ.get("CARTAN_TUTORIAL_03_CPP")
    if env_path:
        path = Path(env_path)
        return path if path.exists() else None

    build = REPO_ROOT / "build"
    if not build.exists():
        return None
    candidates = [
        path for path in build.rglob("03_ik_composition")
        if path.is_file() and os.access(path, os.X_OK)
    ]
    return max(candidates, key=lambda path: path.stat().st_mtime) if candidates else None


@pytest.mark.parametrize(
    "script",
    [
        "01_urdf_walkthrough.py",
        "02_fk_and_jacobians.py",
        "03_ik_composition.py",
    ],
)
def test_python_tutorial_scripts_smoke(script: str) -> None:
    result = _run_python_tutorial(script)
    assert result.returncode == 0, result.stdout + result.stderr


def test_python_tutorial_03_writes_machine_readable_csv(tmp_path: Path) -> None:
    csv_path = tmp_path / "python_tutorial_03.csv"
    result = _run_python_tutorial("03_ik_composition.py", "--csv", str(csv_path))
    assert result.returncode == 0, result.stdout + result.stderr

    rows = _read_csv(csv_path)
    assert len(rows) == 100
    assert {solver for _, solver in rows} == {"pieper_6r_solver", "projected_lm"}
    assert {target for target, _ in rows} == set(range(50))


def test_cpp_and_python_tutorial_03_csv_position_errors_match(
    tmp_path: Path,
) -> None:
    cpp = _cpp_tutorial_executable()
    if cpp is None:
        pytest.skip("C++ tutorial executable is not built")

    cpp_csv = tmp_path / "cpp.csv"
    py_csv = tmp_path / "python.csv"
    cpp_result = subprocess.run(
        [str(cpp), "--csv", str(cpp_csv)],
        cwd=REPO_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if cpp_result.returncode != 0 or not cpp_csv.exists():
        pytest.skip("C++ tutorial executable was built before CSV support")

    py_result = _run_python_tutorial("03_ik_composition.py", "--csv", str(py_csv))
    assert py_result.returncode == 0, py_result.stdout + py_result.stderr

    cpp_rows = _read_csv(cpp_csv)
    py_rows = _read_csv(py_csv)
    assert set(cpp_rows) == set(py_rows)

    for key in sorted(cpp_rows):
        assert cpp_rows[key]["success"] == py_rows[key]["success"]
        cpp_err = float(cpp_rows[key]["pos_err"])
        py_err = float(py_rows[key]["pos_err"])
        assert abs(cpp_err - py_err) <= 1e-9


def test_python_cmake_registers_tutorial_smoke_entries() -> None:
    text = (PYTHON_ROOT / "CMakeLists.txt").read_text()
    for name in (
        "python_tutorial_01_urdf_walkthrough",
        "python_tutorial_02_fk_and_jacobians",
        "python_tutorial_03_ik_composition",
    ):
        assert name in text
    assert "$<TARGET_FILE_DIR:_core>" in text
