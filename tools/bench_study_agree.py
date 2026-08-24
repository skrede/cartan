#!/usr/bin/env python3
"""One capture is many processes, so its environment record has to be earned.

The capture program takes a single robot and a single stratum, and its record
writer truncates on open, so a sweep is necessarily one process per cell and one
environment record per process. The report script accepts exactly one record per
root, and nominating one of them to speak for the rest would produce an artifact
whose stated conditions were measured during one cell out of a hundred and
forty-four.

So the records are merged rather than chosen: every field that must not vary
across a capture is compared across all of them, and a disagreement refuses the
assembly naming both values. That check is the only thing standing between a
machine whose boost was re-enabled halfway through and a published table that
says it was off throughout.
"""

import json

from bench_study_records import Refusal

INVARIANT = (
    "compiler", "standard_library", "eigen", "kernel", "cpu_model", "governor",
    "frequency_boost", "simultaneous_multithreading", "configure_command", "dependencies",
    "descriptions", "declared_participants", "absent_participants",
)


def canonical(value):
    return json.dumps(value, sort_keys=True)


def refuse_unread(document, path):
    """A machine value the capture could not read is the one value a reader most
    needs to know about, so an unread field stops the assembly rather than being
    merged away."""
    unread = document.get("unread_machine_values")
    if unread:
        raise Refusal(f"{path}: the capture could not read {sorted(unread)}, so this record "
                      f"cannot state the conditions its numbers were taken under")


def compare(field, first, document, first_path, path):
    if canonical(first.get(field)) == canonical(document.get(field)):
        return
    raise Refusal(f"{field} differs across the capture: {first_path} records "
                  f"{canonical(first.get(field))} and {path} records "
                  f"{canonical(document.get(field))}. One capture cannot carry two "
                  f"environments, so nothing is assembled from these records")


def load(path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as bad:
        raise Refusal(f"{path}: cannot be read as an environment record") from bad


def merge(paths, passes):
    if not paths:
        raise Refusal("no environment.json resolves below the capture root, so nothing states "
                      "what conditions these numbers were taken under")
    documents = [(path, load(path)) for path in paths]
    for path, document in documents:
        refuse_unread(document, path)
    first_path, first = documents[0]
    for path, document in documents[1:]:
        for field in INVARIANT:
            compare(field, first, document, first_path, path)
    merged = {field: first.get(field) for field in INVARIANT}
    merged["assembled_from"] = {
        "invocations": len(documents),
        "passes": passes,
        "note": "one process per table, robot and stratum; every invariant field above was "
                "compared across all of them and agreed",
    }
    merged["varied_per_invocation"] = {
        field: sorted({canonical(document.get(field)) for _, document in documents})
        for field in ("run_command",)
    }
    return merged
