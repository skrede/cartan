#!/usr/bin/env python3
"""The record schema the study writes, and every refusal that reading it reaches.

A record file is untrusted input: it is offered to third parties to run this
tooling against their own captures. So a field that cannot be a number, a line
that carries a different number of fields from the header, a value that is
non-finite where the schema does not admit one, and a path that resolves outside
the declared root are all refusals naming what was refused and where. Silence on
a malformed input is how a reader ends up holding a table that looks fine.

The three tiers are one experiment at three resolutions: a row per target, a row
per cell, and a row per stratum carrying the paired per-target differences that
only the per-target rows can compute.
"""

import csv
import math
from pathlib import Path

IDENTITY = (
    "table", "robot", "limits_provenance", "stratum",
    "solver", "budget_index", "budget_requested", "budget_axis",
)

TARGET_COLUMNS = IDENTITY + (
    "budget_value", "target_id", "seed_id", "self_reported", "accepted", "pose_ok", "limits_ok",
    "pos_err_m", "ori_err_rad", "worst_limit_violation_rad", "fk_evals", "jac_evals",
    "iterations", "solver_tolerance", "wall_ns", "accuracy_target", "accuracy_target_met",
)

CELL_COLUMNS = IDENTITY + (
    "n_targets", "n_accepted", "n_self_reported", "n_false_success", "n_false_failure",
    "accept_rate", "budget_value_median", "pos_err_median_m", "pos_err_p95_m", "pos_err_p99_m",
    "ori_err_median_rad", "fk_evals_median", "jac_evals_median", "wall_ns_median", "wall_ns_cv",
    "solver_tolerance", "kernel_countable", "accuracy_target", "accuracy_target_met",
    "pos_err_median_accepted_m",
)

STRATA_COLUMNS = (
    "table", "robot", "limits_provenance", "stratum", "budget_index", "solver_a", "solver_b",
    "n_paired", "delta_wall_ns_median", "delta_wall_ns_p95", "delta_wall_ns_p99",
    "delta_pos_err_median_m", "delta_kernel_evals_median",
)

SUMMARY_FIELDS = CELL_COLUMNS[len(IDENTITY):]

PAIR_IDENTITY = STRATA_COLUMNS[:5]

TEXT_COLUMNS = frozenset({
    "table", "robot", "limits_provenance", "stratum", "solver", "budget_axis",
    "solver_a", "solver_b",
})

KERNEL_FIELDS = ("fk_evals", "jac_evals", "iterations")

CELL_KERNEL_FIELDS = ("fk_evals_median", "jac_evals_median")

DIVERGENCE_FIELDS = frozenset({"pos_err_m", "ori_err_rad", "worst_limit_violation_rad"})

UNCONDITIONED_FIELDS = frozenset({
    "pos_err_median_m", "pos_err_p95_m", "pos_err_p99_m", "ori_err_median_rad", "wall_ns_cv",
})

SYNTHETIC = "synthetic"

NO_SUCCESS_RATE = "unreachable"

KERNEL_AXIS = "kernel_evaluations"

TIERS = {"targets": TARGET_COLUMNS, "cells": CELL_COLUMNS, "strata": STRATA_COLUMNS}


class Refusal(Exception):
    pass


def number(text, where):
    try:
        return float(text)
    except (TypeError, ValueError) as bad:
        raise Refusal(f"{where}: {text!r} is not a number") from bad


def counted(text):
    """An empty count is unknown, not zero: the harness did not drive that solver."""
    return None if text == "" else float(text)


def flag(text):
    return text not in ("0", "", None)


def contained(path, root):
    """Resolve before testing containment: a symlink is a path that escapes later."""
    resolved = Path(path).resolve()
    if not resolved.is_relative_to(root):
        raise Refusal(f"{resolved} resolves outside the declared root {root}")
    return resolved


def admits_non_finite(row, name):
    """A diverged solve is recorded as an infinite error rather than dropped, and an
    order statistic over the whole target set carries that value to its worst end.
    Nothing else may be non-finite: an accepted solve with no error is a defect."""
    if name in DIVERGENCE_FIELDS:
        return not flag(row.get("accepted", ""))
    return name in UNCONDITIONED_FIELDS


def check_row(path, line, row, columns):
    for name in columns:
        if name in TEXT_COLUMNS or row[name] == "":
            continue
        where = f"{path} line {line}, column {name}"
        value = number(row[name], where)
        if not math.isfinite(value) and not admits_non_finite(row, name):
            raise Refusal(f"{where}: {row[name]!r} is not finite and this column admits no "
                          f"non-finite value")


def read_csv(path, columns):
    """Rows with their line numbers: a refusal that cannot name the line is one a
    reader cannot act on."""
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        header = tuple(reader.fieldnames or ())
        if header != columns:
            missing = [name for name in columns if name not in header]
            raise Refusal(f"{path}: expected the {len(columns)} declared columns, "
                          f"missing {missing or 'none'}, header was {list(header)}")
        rows = list(enumerate(reader, start=2))
    for line, row in rows:
        if None in row or None in row.values():
            raise Refusal(f"{path} line {line}: carries a different number of fields from the "
                          f"{len(columns)} declared columns")
        check_row(path, line, row, columns)
    return rows


def one_table(path, rows, table):
    for line, row in rows:
        if row["table"] != table:
            raise Refusal(f"{path} line {line}: names table {row['table']!r} in a report about "
                          f"table {table!r}; the periodic rules are separate experiments and a "
                          f"figure spanning them is not a comparison")


def without_synthetic(rows):
    """Bounds nobody declared cannot reject anything, so a limit-dependent figure
    computed over them is a figure about the invention. Measured: 0 of 1500 rows
    fail the limits check on the symmetric box against 410 of 1500 on the
    manufacturer's own bounds."""
    kept = [(line, row) for line, row in rows if row["limits_provenance"] != SYNTHETIC]
    return kept, len(rows) - len(kept)


def discover(root, table, tier):
    found = []
    for candidate in sorted(root.rglob(f"table_{table}_{tier}.csv")):
        resolved = contained(candidate, root)
        with resolved.open(newline="", encoding="utf-8") as handle:
            header = tuple(next(csv.reader(handle), []))
        if header == TIERS[tier]:
            found.append(resolved)
    return found


def one_file(root, table, tier, explicit):
    paths = [contained(name, root) for name in explicit] or discover(root, table, tier)
    if not paths:
        raise Refusal(f"no {tier} records for table {table} resolve below {root}")
    if len(paths) > 1:
        raise Refusal(f"{len(paths)} candidate {tier} files resolve below {root}: {paths}")
    return paths[0]
