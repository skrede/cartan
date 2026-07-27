"""Shared pytest fixtures, and the guard that refuses a stale extension.

The extension carries a digest of every source it was compiled from, stamped at
build time by python/cmake/source_digest.cmake. The guard below recomputes that
digest from the tree and fails collection when the two disagree, so an edit
under lib/ -- where the behavior the bindings expose actually lives -- cannot be
tested against a module that predates it. The comparison is over content, so
refreshing the module's mtime does not defeat it, and it fails closed when the
sources or the stamp cannot be found rather than waving the run through.
"""

from __future__ import annotations

import hashlib
import os
from pathlib import Path

import numpy as np
import pytest

import cartan
import cartan._core as _core


REPO_ROOT = Path(__file__).resolve().parent.parent.parent
URDF_DIR = REPO_ROOT / "tests" / "fixtures" / "urdf"

DIGEST_ROOTS = ("lib", "python/src")
DIGEST_SUFFIXES = (".h", ".hpp", ".hxx", ".inl", ".cpp", ".cxx")


def _compiled_sources() -> list[Path]:
    found: list[Path] = []
    for root in DIGEST_ROOTS:
        directory = REPO_ROOT / root
        if not directory.is_dir():
            raise RuntimeError(
                f"cannot verify the extension: source root {directory} does not exist"
            )
        found.extend(
            p for p in directory.rglob("*") if p.suffix in DIGEST_SUFFIXES and p.is_file()
        )
    if not found:
        raise RuntimeError(f"cannot verify the extension: no sources under {DIGEST_ROOTS}")
    return found


def _source_digest() -> tuple[str, int]:
    entries = sorted(
        (p.relative_to(REPO_ROOT).as_posix(), p) for p in _compiled_sources()
    )
    payload = "".join(
        f"{name} {hashlib.sha256(path.read_bytes()).hexdigest()}\n" for name, path in entries
    )
    return hashlib.sha256(payload.encode()).hexdigest(), len(entries)


def _guard_against_a_stale_extension() -> None:
    path = Path(_core.__file__).resolve()
    print(f"cartan._core: {path}")
    print(f"sha256: {hashlib.sha256(path.read_bytes()).hexdigest()}")
    print(f"mtime: {path.stat().st_mtime}")

    prefix = os.environ.get("CARTAN_CORE_EXPECTED_PREFIX")
    if prefix and not str(path).startswith(prefix):
        raise RuntimeError(f"{path} lies outside the expected prefix {prefix}")

    stamped = getattr(_core, "__source_digest__", None)
    if stamped is None:
        raise RuntimeError(f"{path} carries no source digest; rebuild the extension")
    computed, count = _source_digest()
    print(f"source digest: stamped {stamped} / computed {computed} over {count} files")
    if stamped != computed:
        raise RuntimeError(
            f"{path} was built from other sources than the tree at {REPO_ROOT}: "
            f"stamped {stamped}, computed {computed} over {count} files"
        )


_guard_against_a_stale_extension()


@pytest.fixture(scope="session")
def planar_2r_chain() -> cartan.KinematicChain:
    """Two-link planar chain with revolute joints around z, links of length 1.0."""
    z = np.array([0.0, 0.0, 1.0])
    s1 = cartan.ScrewAxis.revolute(z, np.array([0.0, 0.0, 0.0]))
    s2 = cartan.ScrewAxis.revolute(z, np.array([1.0, 0.0, 0.0]))
    home = cartan.SE3.exp(np.array([0.0, 0.0, 0.0, 2.0, 0.0, 0.0]))
    limits = [cartan.JointLimits(-np.pi, np.pi)] * 2
    return cartan.KinematicChain(home, [s1, s2], limits)


@pytest.fixture(scope="session")
def cartanbot_chain() -> cartan.KinematicChain:
    """Always-on test fixture; six-joint serial arm."""
    if not hasattr(cartan, "load_urdf"):
        pytest.skip("cartan was built without URDF support")
    urdf = URDF_DIR / "cartanbot.urdf"
    if not urdf.exists():
        pytest.skip(f"URDF fixture missing: {urdf}")
    return cartan.load_urdf(str(urdf)).chain


def _maybe_load_extended(filename: str) -> cartan.KinematicChain | None:
    if not hasattr(cartan, "load_urdf"):
        return None
    urdf = URDF_DIR / "extended" / filename
    if not urdf.exists():
        return None
    return cartan.load_urdf(str(urdf)).chain


@pytest.fixture(scope="session")
def ur3e_chain() -> cartan.KinematicChain:
    chain = _maybe_load_extended("ur3e.urdf")
    if chain is None:
        pytest.skip("UR3e URDF fixture not available")
    return chain


@pytest.fixture(scope="session")
def ur5e_chain() -> cartan.KinematicChain:
    chain = _maybe_load_extended("ur5e.urdf")
    if chain is None:
        pytest.skip("UR5e URDF fixture not available")
    return chain


@pytest.fixture(scope="session")
def ur10_chain() -> cartan.KinematicChain:
    chain = _maybe_load_extended("ur10.urdf")
    if chain is None:
        pytest.skip("UR10 URDF fixture not available")
    return chain


@pytest.fixture(scope="session")
def ur16_chain() -> cartan.KinematicChain:
    chain = _maybe_load_extended("ur16.urdf")
    if chain is None:
        pytest.skip("UR16 URDF fixture not available")
    return chain


@pytest.fixture(scope="session")
def kr6_chain() -> cartan.KinematicChain:
    chain = _maybe_load_extended("kr6_sixx_r900.urdf")
    if chain is None:
        pytest.skip("KR6 URDF fixture not available (vendoring required for Pieper coverage)")
    return chain


@pytest.fixture(scope="session")
def irb120_chain() -> cartan.KinematicChain:
    chain = _maybe_load_extended("irb120.urdf")
    if chain is None:
        pytest.skip("IRB120 URDF fixture not available (vendoring required for Pieper coverage)")
    return chain


@pytest.fixture(scope="session")
def iiwa7_chain() -> cartan.KinematicChain:
    chain = _maybe_load_extended("iiwa7.urdf")
    if chain is None:
        pytest.skip("iiwa7 URDF fixture not available")
    return chain


@pytest.fixture(scope="session")
def iiwa14_chain() -> cartan.KinematicChain:
    chain = _maybe_load_extended("iiwa14.urdf")
    if chain is None:
        pytest.skip("iiwa14 URDF fixture not available")
    return chain


@pytest.fixture(scope="session")
def panda_chain() -> cartan.KinematicChain:
    chain = _maybe_load_extended("panda.urdf")
    if chain is None:
        pytest.skip("Panda URDF fixture not available")
    return chain
