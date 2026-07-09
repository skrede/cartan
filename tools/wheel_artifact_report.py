#!/usr/bin/env python3
"""Create a wheel artifact audit report."""

from __future__ import annotations

import argparse
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile
from dataclasses import dataclass
from pathlib import Path


LIBSTDCXX = "libstdc++.so.6"
LIBGCC_S = "libgcc_s.so.1"


@dataclass(frozen=True)
class WheelInfo:
    path: Path
    size: int
    python_tag: str
    abi_tag: str
    platform_tag: str


@dataclass
class AuditState:
    auditwheel: str = "not available"
    readelf_needed: list[str] | None = None
    symbol_versions: list[str] | None = None
    failures: list[str] | None = None

    def __post_init__(self) -> None:
        if self.readelf_needed is None:
            self.readelf_needed = []
        if self.symbol_versions is None:
            self.symbol_versions = []
        if self.failures is None:
            self.failures = []


def _wheel_info(path: Path) -> WheelInfo:
    parts = path.name.removesuffix(".whl").split("-")
    if len(parts) < 5:
        return WheelInfo(path, path.stat().st_size, "unknown", "unknown", "unknown")
    return WheelInfo(path, path.stat().st_size, parts[-3], parts[-2], parts[-1])


def _run(cmd: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, capture_output=True, text=True, check=False)


def _extract_shared_objects(wheel: Path, directory: Path) -> list[Path]:
    out: list[Path] = []
    with zipfile.ZipFile(wheel) as archive:
        for member in archive.namelist():
            suffix = Path(member).suffix.lower()
            if suffix not in {".so", ".pyd", ".dylib", ".dll"}:
                continue
            target = directory / member
            target.parent.mkdir(parents=True, exist_ok=True)
            with archive.open(member) as src, target.open("wb") as dst:
                shutil.copyfileobj(src, dst)
            out.append(target)
    return out


def _auditwheel_show(wheel: Path, state: AuditState) -> None:
    auditwheel = shutil.which("auditwheel")
    if auditwheel is None:
        return
    result = _run([auditwheel, "show", str(wheel)])
    state.auditwheel = result.stdout + result.stderr
    if result.returncode != 0:
        state.failures.append(f"auditwheel show failed for {wheel.name}")


def _readelf_scan(wheel: Path, state: AuditState) -> None:
    readelf = shutil.which("readelf")
    if readelf is None:
        return
    with tempfile.TemporaryDirectory(prefix="cartan-wheel-audit-") as tmp:
        shared = _extract_shared_objects(wheel, Path(tmp))
        for lib in shared:
            dynamic = _run([readelf, "-d", str(lib)])
            if dynamic.returncode != 0:
                state.failures.append(f"readelf -d failed for {lib.name}")
            for match in re.finditer(r"Shared library: \[(.*?)\]", dynamic.stdout):
                state.readelf_needed.append(match.group(1))

            versions = _run([readelf, "--version-info", str(lib)])
            if versions.returncode != 0:
                continue
            for match in re.finditer(r"\b(?:GLIBC|GLIBCXX)_[0-9][0-9.]*\b", versions.stdout):
                state.symbol_versions.append(match.group(0))


def _contains_required_lib(text: str, lib: str, needed: list[str]) -> bool:
    return lib in text or lib in needed


def _write_report(path: Path, wheels: list[WheelInfo], state: AuditState) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as out:
        out.write("# Wheel Artifact Report\n\n")
        out.write("## Wheels\n\n")
        out.write("| Wheel | Size bytes | Python tag | ABI tag | Platform tag |\n")
        out.write("|-------|------------|------------|---------|--------------|\n")
        for wheel in wheels:
            out.write(
                f"| `{wheel.path.name}` | {wheel.size} | `{wheel.python_tag}` | "
                f"`{wheel.abi_tag}` | `{wheel.platform_tag}` |\n"
            )
        out.write("\n## auditwheel\n\n")
        if state.auditwheel == "not available":
            out.write("not available\n\n")
        else:
            out.write("```text\n")
            out.write(state.auditwheel.strip())
            out.write("\n```\n\n")
        out.write("## readelf NEEDED\n\n")
        if state.readelf_needed:
            for item in sorted(set(state.readelf_needed)):
                out.write(f"- `{item}`\n")
        else:
            out.write("not available\n")
        out.write("\n## GLIBC and GLIBCXX Symbol Versions\n\n")
        if state.symbol_versions:
            for item in sorted(set(state.symbol_versions)):
                out.write(f"- `{item}`\n")
        else:
            out.write("not available\n")
        out.write("\n## Failures\n\n")
        if state.failures:
            for item in state.failures:
                out.write(f"- {item}\n")
        else:
            out.write("None\n")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--wheel-dir", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--require-no-libstdcxx", action="store_true")
    parser.add_argument("--require-no-libgcc-s", action="store_true")
    args = parser.parse_args(argv)

    wheel_dir = Path(args.wheel_dir)
    wheels = sorted(wheel_dir.glob("*.whl"))
    state = AuditState()
    if not wheels:
        state.failures.append(f"no wheel files found in {wheel_dir}")

    is_linux = platform.system() == "Linux"
    for wheel in wheels:
        if is_linux:
            _auditwheel_show(wheel, state)
        _readelf_scan(wheel, state)

    audit_text = state.auditwheel if state.auditwheel != "not available" else ""
    require_libstdcxx = bool(args.require_no_libstdcxx)
    require_libgcc = bool(args.require_no_libgcc_s)
    if (require_libstdcxx or require_libgcc) and not is_linux:
        state.failures.append("required Linux dependency audit requested off Linux")
    if require_libstdcxx and _contains_required_lib(
        audit_text, LIBSTDCXX, state.readelf_needed
    ):
        state.failures.append(f"forbidden dependency found: {LIBSTDCXX}")
    if require_libgcc and _contains_required_lib(
        audit_text, LIBGCC_S, state.readelf_needed
    ):
        state.failures.append(f"forbidden dependency found: {LIBGCC_S}")
    if is_linux and (require_libstdcxx or require_libgcc):
        if shutil.which("auditwheel") is None:
            state.failures.append("auditwheel is required for Linux dependency audit")
        if shutil.which("readelf") is None:
            state.failures.append("readelf is required for Linux dependency audit")

    _write_report(Path(args.output), [_wheel_info(path) for path in wheels], state)
    return 1 if state.failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
