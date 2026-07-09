"""Strict type-checker gate on _stubs_sample.py.

Runs pyright and mypy as subprocesses against the sample script that exercises
every public binding. Both tests are marked slow because invoking external
static-analysis tools takes several seconds, putting them outside the wall-time
budget of the primary fast suite.

Strict mode enablement:

- pyright: strict mode is opted in via the file-level `# pyright: strict`
  marker in `_stubs_sample.py` (the `--strict` CLI option was removed in
  pyright 1.1.400+).
- mypy: strict mode is enabled via the `--strict` CLI flag, which has been
  stable for many releases.
"""

from __future__ import annotations

import json
import site
import shutil
import subprocess
import sys
import sysconfig
from pathlib import Path

import pytest


SAMPLE_SCRIPT = Path(__file__).parent / "_stubs_sample.py"
PYTHON_PACKAGE_DIR = SAMPLE_SCRIPT.parents[1]


def _run_checker(cmd: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, capture_output=True, text=True, check=False)


def _pyright_extra_paths() -> list[str]:
    candidates = [
        PYTHON_PACKAGE_DIR,
        *(Path(p) for p in sys.path if p),
        Path(site.getusersitepackages()),
    ]
    candidates.extend(
        Path(p) for p in site.getsitepackages() if p)
    candidates.extend(
        Path(p) for p in (
            sysconfig.get_paths().get("purelib"),
            sysconfig.get_paths().get("platlib"),
        ) if p)

    seen: set[str] = set()
    out: list[str] = []
    for path in candidates:
        if not path.exists():
            continue
        resolved = str(path.resolve())
        if resolved in seen:
            continue
        seen.add(resolved)
        out.append(resolved)
    return out


@pytest.mark.slow
@pytest.mark.skipif(shutil.which("pyright") is None, reason="pyright not on PATH")
def test_stubs_pyright_strict(tmp_path: Path) -> None:
    """Strict-mode pyright must report zero errors on the sample script.

    Strict mode is opted in by the `# pyright: strict` marker at the top of
    `_stubs_sample.py`; no CLI flag is required (or supported in recent
    pyright versions).
    """
    config_path = tmp_path / "pyrightconfig.json"
    config_path.write_text(
        json.dumps({
            "include": [str(SAMPLE_SCRIPT)],
            "extraPaths": _pyright_extra_paths(),
            "pythonPath": sys.executable,
        }),
        encoding="utf-8")
    result = _run_checker(["pyright", "--project", str(config_path)])
    assert result.returncode == 0, (
        f"pyright reported errors on {SAMPLE_SCRIPT}:\n"
        f"--- stdout ---\n{result.stdout}\n"
        f"--- stderr ---\n{result.stderr}"
    )


@pytest.mark.slow
@pytest.mark.skipif(shutil.which("mypy") is None, reason="mypy not on PATH")
def test_stubs_mypy_strict() -> None:
    """mypy --strict must report zero errors on the sample script."""
    result = _run_checker(["mypy", "--strict", str(SAMPLE_SCRIPT)])
    assert result.returncode == 0, (
        f"mypy --strict reported errors on {SAMPLE_SCRIPT}:\n"
        f"--- stdout ---\n{result.stdout}\n"
        f"--- stderr ---\n{result.stderr}"
    )
