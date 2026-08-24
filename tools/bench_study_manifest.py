#!/usr/bin/env python3
"""What the capture declared it would solve with, checked against what it wrote.

A table that is quietly one participant narrower reads exactly like a complete
one, and under system-provided comparator libraries a narrower table is the
ordinary case rather than an exotic one. So the two kinds of missing participant
are distinguished here: one the capture declared and neither wrote nor recorded
an absence for is a defect and stops the report, while one the capture recorded
as absent with a reason is rendered as a named absence inside the table it is
missing from.

Countability is declared the same way and for the same reason. A participant
whose iteration the harness does not drive has no kernel-evaluation count to
attribute, so it is excluded from that axis by construction -- and a record
giving it a count anyway is refused rather than plotted.
"""

import json
from pathlib import Path

from bench_study_records import Refusal, contained


class Manifest:
    def __init__(self, path, declared, absent):
        self.path = path
        self.declared = declared
        self.absent = absent

    def countable(self, participant):
        return self.declared.get(participant, False)

    def uncountable(self):
        return sorted(name for name, countable in self.declared.items() if not countable)


def entries(document, path, key, fields):
    listed = document.get(key)
    if not isinstance(listed, list):
        raise Refusal(f"{path}: carries no {key} array, so nothing states what this capture "
                      f"declared it would solve with")
    for entry in listed:
        if not all(field in entry for field in fields):
            raise Refusal(f"{path}: an entry of {key} is missing one of {list(fields)}: {entry}")
    return listed


def load_manifest(path):
    try:
        document = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError) as bad:
        raise Refusal(f"{path}: cannot be read as the capture's environment record") from bad
    declared = {entry["name"]: bool(entry["kernel_countable"])
                for entry in entries(document, path, "declared_participants",
                                     ("name", "kernel_countable"))}
    absent = {entry["name"]: entry["reason"]
              for entry in entries(document, path, "absent_participants", ("name", "reason"))}
    return Manifest(path, declared, absent)


def one_manifest(root, explicit):
    paths = [contained(explicit, root)] if explicit else sorted(root.rglob("environment.json"))
    if not paths:
        raise Refusal(f"no environment.json resolves below {root}, so nothing states which "
                      f"participants this capture declared")
    if len(paths) > 1:
        raise Refusal(f"{len(paths)} environment records resolve below {root}: {paths}")
    return load_manifest(paths[0])


def check_participants(manifest, present):
    """The declared list is the authority in both directions: a participant it does
    not name cannot be in the records either."""
    absences = []
    for participant in sorted(manifest.declared):
        if participant in present:
            continue
        if participant not in manifest.absent:
            raise Refusal(f"{manifest.path} declares the participant {participant!r} and these "
                          f"records carry no row for it, with no recorded absence saying why; a "
                          f"table missing a declared solver is a defect rather than an omission")
        absences.append((participant, manifest.absent[participant]))
    undeclared = sorted(present - set(manifest.declared))
    if undeclared:
        raise Refusal(f"the records carry rows for {undeclared}, which {manifest.path} does not "
                      f"declare; a participant nothing declared is a participant nothing "
                      f"identifies")
    return absences


def check_kernel_values(path, rows, manifest, fields, solver_column="solver"):
    for line, row in rows:
        participant = row[solver_column]
        if manifest.countable(participant):
            continue
        given = [name for name in fields if row.get(name, "") != ""]
        if given:
            raise Refusal(f"{path} line {line}: {participant!r} is declared without a countable "
                          f"kernel and this row gives it {', '.join(given)}; a count the harness "
                          f"cannot attribute is not a measurement")
