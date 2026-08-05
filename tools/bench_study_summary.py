#!/usr/bin/env python3
"""The statistics the report publishes, computed the one way the harness computes them.

Order statistics are nearest rank over the whole target set with a non-finite
value sorted to the worst end, which is the rule the C++ writer implements for
the per-cell tier; the two therefore agree exactly rather than approximately, and
the cross-check between the tiers is meaningful instead of tolerant.

A paired per-target difference is defined only where both solvers were accepted
on the same target, so it is conditioned on both of them and says so wherever it
is rendered. Its count travels beside it: a difference over four targets and a
difference over four hundred are not the same claim.
"""

import math

from bench_study_records import (
    IDENTITY, KERNEL_AXIS, NO_SUCCESS_RATE, PAIR_IDENTITY, STRATA_COLUMNS, SUMMARY_FIELDS,
    counted, flag,
)

PAIRED_FIELDS = STRATA_COLUMNS[len(PAIR_IDENTITY) + 2:]


def order_statistic(values, quantile):
    """Nearest rank over every value, non-finite sorted to the worst end."""
    if not values:
        return math.nan
    ordered = sorted(value if math.isfinite(value) else math.inf for value in values)
    rank = int(quantile * len(ordered))
    return ordered[min(rank, len(ordered) - 1)]


def variation(values):
    if len(values) < 2:
        return math.nan
    count = len(values)
    mean = sum(values) / count
    squares = 0.0
    for value in values:
        squares += (value - mean) * (value - mean)
    return math.sqrt(squares / (count - 1)) / mean


def value(row, name):
    return float(row[name])


def empty_cell(row):
    return {
        "pos": [], "ori": [], "wall": [], "fk": [], "jac": [], "achieved": [], "accepted_pos": [],
        "accepted": 0, "reported": 0, "false_success": 0, "false_failure": 0,
        "tolerance": value(row, "solver_tolerance"),
        "rate": row["stratum"] != NO_SUCCESS_RATE,
        "accuracy_target": counted(row["accuracy_target"]),
        "accuracy_target_met": counted(row["accuracy_target_met"]),
    }


def accumulate(cell, row):
    cell["achieved"].append(value(row, "budget_value"))
    cell["pos"].append(value(row, "pos_err_m"))
    cell["ori"].append(value(row, "ori_err_rad"))
    cell["wall"].append(value(row, "wall_ns"))
    for name, key in (("fk_evals", "fk"), ("jac_evals", "jac")):
        if row[name] != "":
            cell[key].append(value(row, name))
    accepted, reported = flag(row["accepted"]), flag(row["self_reported"])
    if accepted:
        cell["accepted_pos"].append(value(row, "pos_err_m"))
    cell["accepted"] += 1 if accepted else 0
    cell["reported"] += 1 if reported else 0
    cell["false_success"] += 1 if reported and not accepted else 0
    cell["false_failure"] += 1 if accepted and not reported else 0


def summarize_rows(rows):
    cells = {}
    for _, row in rows:
        key = tuple(row[name] for name in IDENTITY)
        accumulate(cells.setdefault(key, empty_cell(row)), row)
    return {key: finalize(cell) for key, cell in cells.items()}


def finalize(cell):
    targets = len(cell["pos"])
    return {
        "n_targets": float(targets),
        "n_accepted": float(cell["accepted"]),
        "n_self_reported": float(cell["reported"]),
        "n_false_success": float(cell["false_success"]),
        "n_false_failure": float(cell["false_failure"]),
        "accept_rate": (cell["accepted"] / targets if targets and cell["rate"] else None),
        "budget_value_median": order_statistic(cell["achieved"], 0.5),
        "pos_err_median_m": order_statistic(cell["pos"], 0.5),
        "pos_err_p95_m": order_statistic(cell["pos"], 0.95),
        "pos_err_p99_m": order_statistic(cell["pos"], 0.99),
        "ori_err_median_rad": order_statistic(cell["ori"], 0.5),
        "fk_evals_median": order_statistic(cell["fk"], 0.5) if cell["fk"] else None,
        "jac_evals_median": order_statistic(cell["jac"], 0.5) if cell["jac"] else None,
        "wall_ns_median": order_statistic(cell["wall"], 0.5),
        "wall_ns_cv": variation(cell["wall"]),
        "solver_tolerance": cell["tolerance"],
        "kernel_countable": 1.0 if cell["fk"] else 0.0,
        "accuracy_target": cell["accuracy_target"],
        "accuracy_target_met": cell["accuracy_target_met"],
        "pos_err_median_accepted_m": (
            order_statistic(cell["accepted_pos"], 0.5) if cell["accepted_pos"] else None),
    }


def summarize_cells(rows):
    return {tuple(row[name] for name in IDENTITY): {name: counted(row[name])
                                                    for name in SUMMARY_FIELDS}
            for _, row in rows}


def by_target(rows):
    groups = {}
    for _, row in rows:
        key = tuple(row[name] for name in PAIR_IDENTITY)
        groups.setdefault(key, {}).setdefault(row["target_id"], {})[row["solver"]] = row
    return groups


def both_accepted(targets, first, second):
    for entry in targets.values():
        left, right = entry.get(first), entry.get(second)
        if left and right and flag(left["accepted"]) and flag(right["accepted"]):
            yield left, right


def differences(targets, first, second):
    matched = list(both_accepted(targets, first, second))
    kernel = all(row["budget_axis"] == KERNEL_AXIS for pair in matched for row in pair)
    wall = [value(left, "wall_ns") - value(right, "wall_ns") for left, right in matched]
    error = [value(left, "pos_err_m") - value(right, "pos_err_m") for left, right in matched]
    evals = [value(left, "budget_value") - value(right, "budget_value") for left, right in matched]
    return {
        "n_paired": float(len(matched)),
        "delta_wall_ns_median": order_statistic(wall, 0.5) if wall else None,
        "delta_wall_ns_p95": order_statistic(wall, 0.95) if wall else None,
        "delta_wall_ns_p99": order_statistic(wall, 0.99) if wall else None,
        "delta_pos_err_median_m": order_statistic(error, 0.5) if error else None,
        "delta_kernel_evals_median": order_statistic(evals, 0.5) if evals and kernel else None,
    }


def paired_from_targets(rows):
    paired = {}
    for key, targets in by_target(rows).items():
        solvers = sorted({name for entry in targets.values() for name in entry})
        for index, first in enumerate(solvers):
            for second in solvers[index + 1:]:
                paired[key + (first, second)] = differences(targets, first, second)
    return paired


def paired_from_strata(rows):
    return {tuple(row[name] for name in STRATA_COLUMNS[:len(PAIR_IDENTITY) + 2]):
            {name: counted(row[name]) for name in PAIRED_FIELDS}
            for _, row in rows}


def strata_text(paired):
    lines = [",".join(STRATA_COLUMNS)]
    for key in sorted(paired):
        fields = list(key) + [f"{paired[key][name]:.17g}" if paired[key][name] is not None else ""
                              for name in PAIRED_FIELDS]
        lines.append(",".join(fields))
    return "\n".join(lines) + "\n"
