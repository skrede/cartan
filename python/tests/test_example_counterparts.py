"""Documentation guards for Python feature parity and example counterparts."""

from __future__ import annotations

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
DOCS = REPO_ROOT / "docs" / "python.md"

FEATURE_TABLE_HEADER = (
    "Feature family | C++ API | Python API | Docs/examples coverage | Status | Notes"
)
EDITABLE_RECIPE = (
    "pip install --no-build-isolation --config-settings=editable.rebuild=true "
    "-Cbuild-dir=build -ve ."
)


def _docs_text() -> str:
    return DOCS.read_text()


def test_python_reference_has_required_feature_family_table() -> None:
    text = _docs_text()
    assert FEATURE_TABLE_HEADER in text
    for family in (
        "Lie groups",
        "Chains",
        "FK",
        "Jacobians",
        "URDF",
        "Iterative IK",
        "Analytical IK",
        "OPW",
        "Unwrap",
        "Exhaustive runner",
        "Install/export",
    ):
        assert f"{family} |" in text


def test_every_cpp_example_is_represented_in_python_reference() -> None:
    text = _docs_text()
    examples = sorted(
        path.relative_to(REPO_ROOT).as_posix()
        for path in (REPO_ROOT / "examples").rglob("*.cpp")
    )
    assert examples
    for example in examples:
        assert example in text


def test_python_reference_contains_install_and_composition_callouts() -> None:
    text = _docs_text()
    assert EDITABLE_RECIPE in text
    for token in ("meshcat", "hpp-fcl", "toppra", "cartan-serial-dynamics"):
        assert token in text
    for heading in (
        "Visualization",
        "Collision checking",
        "Trajectory optimization",
        "Dynamics",
    ):
        assert heading in text


def test_docs_index_links_python_reference() -> None:
    text = (REPO_ROOT / "docs" / "README.md").read_text()
    assert "[python](python.md)" in text
