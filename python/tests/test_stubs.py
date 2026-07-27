"""Strict type-checker gate on _stubs_sample.py.

Runs pyright and mypy as subprocesses against the sample script that exercises
every public binding. Both tests are marked slow because invoking external
static-analysis tools takes several seconds, putting them outside the wall-time
budget of the primary fast suite.

Each checker is run twice: once on the sample, which must pass, and once on a
copy whose optional magnitude annotation is narrowed to a bare ``float``, which
must fail. The second run is what gives the first one meaning. A checker that
resolves no stubs at all, or resolves a stale installed copy of the package,
passes both runs -- and has reported nothing. Two ways that happened before:
pyright silently ignores an absolute path in ``include`` and still exits 0, and
mypy prefers an installed package over the one under test. Both runs are read
for diagnostics naming the sample, not for the exit status, so an inconsistency
inside the generated stubs is neither mistaken for a verdict on the sample nor
able to satisfy the control.

The sample is copied into a scratch directory next to a generated config so the
config's ``include`` can name it relatively.

Strict mode enablement:

- pyright: strict mode is opted in via the file-level `# pyright: strict`
  marker in `_stubs_sample.py` (the `--strict` CLI option was removed in
  pyright 1.1.400+).
- mypy: strict mode is enabled via the `--strict` CLI flag, which has been
  stable for many releases.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sysconfig
import importlib.util
from pathlib import Path

import pytest


SAMPLE_SCRIPT = Path(__file__).parent / "_stubs_sample.py"

OPTIONAL_ANNOTATION = "analytical_error_metric: float | None = "
NARROWED_ANNOTATION = "analytical_error_metric: float = "


def _typed_package_parent() -> Path:
    """Parent directory of the cartan package this interpreter imports.

    Skips when that package ships no generated stubs: a checker pointed at it
    would resolve nothing and pass whatever it was handed.
    """
    spec = importlib.util.find_spec("cartan")
    if spec is None or spec.origin is None:
        pytest.skip("cartan is not importable, so there are no stubs to check")
    package = Path(spec.origin).parent
    if not (package / "analytical.pyi").exists():
        pytest.skip(
            f"the cartan package at {package} ships no analytical.pyi, so a "
            "checker run against it would prove nothing")
    return package.parent


def _interpreter_paths() -> list[str]:
    return [p for p in (sysconfig.get_paths().get("purelib"),
                        sysconfig.get_paths().get("platlib")) if p]


def _sample_copy(work: Path, narrowed: bool) -> Path:
    work.mkdir(parents=True, exist_ok=True)
    text = SAMPLE_SCRIPT.read_text(encoding="utf-8")
    if narrowed:
        assert OPTIONAL_ANNOTATION in text, "the control annotation moved"
        text = text.replace(OPTIONAL_ANNOTATION, NARROWED_ANNOTATION, 1)
    target = work / SAMPLE_SCRIPT.name
    target.write_text(text, encoding="utf-8")
    return target


def _run_pyright(work: Path, parent: Path, narrowed: bool) -> subprocess.CompletedProcess[str]:
    sample = _sample_copy(work, narrowed)
    (work / "pyrightconfig.json").write_text(
        json.dumps({
            "include": [sample.name],
            "extraPaths": [str(parent), *_interpreter_paths()],
        }),
        encoding="utf-8")
    return subprocess.run(
        ["pyright", "--project", str(work)],
        capture_output=True, text=True, check=False)


def _run_mypy(work: Path, parent: Path, narrowed: bool) -> subprocess.CompletedProcess[str]:
    sample = _sample_copy(work, narrowed)
    return subprocess.run(
        ["mypy", "--strict", "--no-incremental",
         "--cache-dir", str(work / "cache"), str(sample)],
        capture_output=True, text=True, check=False,
        env={**os.environ, "MYPYPATH": str(parent)})


def _require_started(label: str, result: subprocess.CompletedProcess[str]) -> None:
    """Skip when the checker never ran.

    A checker on PATH can still fail to import itself -- it lives in a site
    directory the harness suppressed, say. Nothing was verified either way, and
    reporting that as a type error would be a lie about the sample.
    """
    if result.returncode != 0 and not result.stdout.strip():
        pytest.skip(f"{label} did not run:\n{result.stderr.strip()}")


def _sample_errors(result: subprocess.CompletedProcess[str], sample: Path) -> list[str]:
    """Diagnostics naming the sample itself.

    The subject of this gate is the sample, not the generated stubs it reads:
    a stub emitted from a build without an optional backend is internally
    inconsistent with __init__.py, and that is a separate problem.
    """
    return [line.strip() for line in result.stdout.splitlines()
            if str(sample) in line and "error" in line]


def _report(label: str, result: subprocess.CompletedProcess[str]) -> str:
    return (f"--- {label} stdout ---\n{result.stdout}\n"
            f"--- {label} stderr ---\n{result.stderr}")


VACUOUS = (
    "narrowing the optional magnitude to a bare float drew no complaint, so "
    "this run resolved no stubs, or resolved a stale installed copy of the "
    "package rather than the one under test. A pass here means nothing.\n")


@pytest.mark.slow
@pytest.mark.skipif(shutil.which("pyright") is None, reason="pyright not on PATH")
def test_stubs_pyright_strict(tmp_path: Path) -> None:
    """Strict-mode pyright must accept the sample and reject the narrowed copy.

    Strict mode is opted in by the `# pyright: strict` marker at the top of
    `_stubs_sample.py`; no CLI flag is required (or supported in recent
    pyright versions).
    """
    parent = _typed_package_parent()

    accepted = _run_pyright(tmp_path / "sample", parent, narrowed=False)
    _require_started("pyright", accepted)
    assert not _sample_errors(accepted, tmp_path / "sample" / SAMPLE_SCRIPT.name), (
        f"pyright reported errors on {SAMPLE_SCRIPT}:\n{_report('pyright', accepted)}")

    control = _run_pyright(tmp_path / "narrowed", parent, narrowed=True)
    assert _sample_errors(control, tmp_path / "narrowed" / SAMPLE_SCRIPT.name), (
        VACUOUS + _report("pyright", control))


@pytest.mark.slow
@pytest.mark.skipif(shutil.which("mypy") is None, reason="mypy not on PATH")
def test_stubs_mypy_strict(tmp_path: Path) -> None:
    """mypy --strict must accept the sample and reject the narrowed copy."""
    parent = _typed_package_parent()

    accepted = _run_mypy(tmp_path / "sample", parent, narrowed=False)
    _require_started("mypy", accepted)
    assert not _sample_errors(accepted, tmp_path / "sample" / SAMPLE_SCRIPT.name), (
        f"mypy --strict reported errors on {SAMPLE_SCRIPT}:\n{_report('mypy', accepted)}")

    control = _run_mypy(tmp_path / "narrowed", parent, narrowed=True)
    assert _sample_errors(control, tmp_path / "narrowed" / SAMPLE_SCRIPT.name), (
        VACUOUS + _report("mypy", control))
