#!/usr/bin/env python3
"""Rebuild one published study table from the records the harness emitted.

Standard library only, so a reader who clones the repository needs python3 and
nothing else to reproduce every figure in the report.

The two record tiers -- per-target rows and per-cell aggregates -- are read by
the same code path and rendered by the same formatter, so a figure taken from
the shipped aggregates is the figure the full-resolution rows produce. Order
statistics are nearest rank over the whole target set, and a non-finite error
sorts to the worst end rather than being dropped: a distribution reported only
over the targets a solver survived is conditioned on that solver's own success.

Nothing here combines two tables. Each periodic rule is its own experiment, and
a reader who wants to compare them has to do it deliberately.
"""

import argparse
import csv
import math
import sys
from pathlib import Path

IDENTITY = (
    "table", "robot", "limits_provenance", "stratum",
    "solver", "budget_index", "budget_value", "budget_axis",
)

TARGET_COLUMNS = IDENTITY + (
    "target_id", "seed_id", "self_reported", "accepted", "pose_ok", "limits_ok",
    "pos_err_m", "ori_err_rad", "worst_limit_violation_rad", "fk_evals", "jac_evals",
    "iterations", "solver_tolerance", "wall_ns",
)

CELL_COLUMNS = IDENTITY + (
    "n_targets", "n_accepted", "n_self_reported", "n_false_success", "n_false_failure",
    "accept_rate", "pos_err_median_m", "pos_err_p95_m", "pos_err_p99_m", "ori_err_median_rad",
    "fk_evals_median", "jac_evals_median", "wall_ns_median", "wall_ns_cv", "solver_tolerance",
)

SUMMARY_FIELDS = CELL_COLUMNS[len(IDENTITY):]

EXIT_REFUSED = 1

EXIT_HELP = """exit codes:
  0  the table was rebuilt
  1  no records resolve below the root, an input escapes it, or a column is missing
  2  the arguments are wrong (argparse)
"""


class Refusal(Exception):
    pass


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


def number(text):
    try:
        return float(text)
    except (TypeError, ValueError) as bad:
        raise Refusal(f"{text!r} is not a number") from bad


def flag(text):
    return text not in ("0", "", None)


def contained(path, root):
    """Resolve before testing containment: a symlink is a path that escapes later."""
    resolved = Path(path).resolve()
    if not resolved.is_relative_to(root):
        raise Refusal(f"{resolved} resolves outside the declared root {root}")
    return resolved


def read_csv(path, columns):
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        header = tuple(reader.fieldnames or ())
        if header != columns:
            missing = [name for name in columns if name not in header]
            raise Refusal(f"{path}: expected the {len(columns)} declared columns, "
                          f"missing {missing or 'none'}, header was {list(header)}")
        rows = list(reader)
    for line, row in enumerate(rows, start=2):
        if None in row or None in row.values():
            raise Refusal(f"{path} line {line}: carries {len(row)} fields against "
                          f"{len(columns)} columns")
    return rows


def discover(root, table, columns, explicit):
    if explicit:
        return [contained(name, root) for name in explicit]
    found = []
    for candidate in sorted(root.rglob(f"table_{table}.csv")):
        resolved = contained(candidate, root)
        with resolved.open(newline="", encoding="utf-8") as handle:
            header = tuple(next(csv.reader(handle), []))
        if header == columns:
            found.append(resolved)
    return found


def summarize_rows(rows, table):
    cells = {}
    for row in rows:
        if row["table"] != table:
            raise Refusal(f"a record names table {row['table']!r} in a report about {table!r}")
        cell = cells.setdefault(tuple(row[name] for name in IDENTITY), {
            "pos": [], "ori": [], "wall": [], "fk": [], "jac": [],
            "accepted": 0, "reported": 0, "false_success": 0, "false_failure": 0,
            "tolerance": number(row["solver_tolerance"]),
        })
        cell["pos"].append(number(row["pos_err_m"]))
        cell["ori"].append(number(row["ori_err_rad"]))
        cell["wall"].append(number(row["wall_ns"]))
        cell["fk"].append(number(row["fk_evals"]))
        cell["jac"].append(number(row["jac_evals"]))
        accepted, reported = flag(row["accepted"]), flag(row["self_reported"])
        cell["accepted"] += 1 if accepted else 0
        cell["reported"] += 1 if reported else 0
        cell["false_success"] += 1 if reported and not accepted else 0
        cell["false_failure"] += 1 if accepted and not reported else 0
    return {key: finalize(cell) for key, cell in cells.items()}


def finalize(cell):
    targets = len(cell["pos"])
    return {
        "n_targets": float(targets),
        "n_accepted": float(cell["accepted"]),
        "n_self_reported": float(cell["reported"]),
        "n_false_success": float(cell["false_success"]),
        "n_false_failure": float(cell["false_failure"]),
        "accept_rate": cell["accepted"] / targets if targets else 0.0,
        "pos_err_median_m": order_statistic(cell["pos"], 0.5),
        "pos_err_p95_m": order_statistic(cell["pos"], 0.95),
        "pos_err_p99_m": order_statistic(cell["pos"], 0.99),
        "ori_err_median_rad": order_statistic(cell["ori"], 0.5),
        "fk_evals_median": order_statistic(cell["fk"], 0.5),
        "jac_evals_median": order_statistic(cell["jac"], 0.5),
        "wall_ns_median": order_statistic(cell["wall"], 0.5),
        "wall_ns_cv": variation(cell["wall"]),
        "solver_tolerance": cell["tolerance"],
    }


def summarize_cells(rows, table):
    summaries = {}
    for row in rows:
        if row["table"] != table:
            raise Refusal(f"a record names table {row['table']!r} in a report about {table!r}")
        key = tuple(row[name] for name in IDENTITY)
        summaries[key] = {name: number(row[name]) for name in SUMMARY_FIELDS}
    return summaries


def markdown(text):
    """A record field is data; it must not be able to close a cell or open code."""
    return str(text).replace("|", r"\|").replace("`", r"\`").replace("\n", " ").replace("\r", " ")


def figure(value, targets):
    if not math.isfinite(value):
        return f"non-finite (n = {targets})"
    return f"{value:.6g} (n = {targets})"


def share(count, targets):
    if targets == 0:
        return "0 / 0"
    return f"{int(count)} / {int(targets)} ({100.0 * count / targets:.2f}%)"


HEADINGS = (
    "solver", "targets", "accepted", "self-reported", "false success", "false failure",
    "pos err median (m)", "pos err p95 (m)", "pos err p99 (m)", "ori err median (rad)",
    "fk evals median", "jac evals median", "wall median (ns)", "wall cv", "solver tolerance",
)


def solver_row(key, summary):
    targets = summary["n_targets"]
    counted = ("n_accepted", "n_self_reported", "n_false_success", "n_false_failure")
    measured = ("pos_err_median_m", "pos_err_p95_m", "pos_err_p99_m", "ori_err_median_rad",
                "fk_evals_median", "jac_evals_median", "wall_ns_median", "wall_ns_cv")
    cells = [markdown(key[IDENTITY.index("solver")]), f"{int(targets)}"]
    cells += [share(summary[name], targets) for name in counted]
    cells += [figure(summary[name], int(targets)) for name in measured]
    cells.append(f"{summary['solver_tolerance']:.6g}")
    return "| " + " | ".join(cells) + " |"


def render(source, table, summaries, tier):
    lines = [f"# Study table {markdown(table)}", "",
             f"Records: `{markdown(source)}` ({tier} tier).", "",
             "Every figure carries the number of targets it was computed over. Order statistics",
             "are nearest rank over the whole target set, with a non-finite error sorted to the",
             "worst end, so no column is conditioned on a solver's own success. Success means",
             "the harness's own verdict, recomputed from the returned joint vector; the solver's",
             "own claim is the separate self-reported column. No figure combines this table with",
             "another.", ""]
    grouped = {}
    for key, summary in summaries.items():
        grouped.setdefault(tuple(key[i] for i in (1, 2, 3, 5, 6, 7)), []).append((key, summary))
    for section in sorted(grouped):
        robot, provenance, stratum, index, value, axis = (markdown(part) for part in section)
        lines += [f"## {robot} -- {stratum} stratum, {provenance} limits, "
                  f"budget {index} = {value} {axis}", "",
                  "| " + " | ".join(HEADINGS) + " |",
                  "|" + "|".join(["---"] * len(HEADINGS)) + "|"]
        ordered = sorted(grouped[section], key=lambda pair: pair[0])
        lines += [solver_row(key, summary) for key, summary in ordered]
        lines.append("")
    return "\n".join(lines) + "\n"


def build(arguments):
    root = Path(arguments.root).resolve()
    if not root.is_dir():
        raise Refusal(f"{root} is not a directory")
    targets_tier = arguments.source == "targets"
    columns = TARGET_COLUMNS if targets_tier else CELL_COLUMNS
    paths = discover(root, arguments.table, columns, arguments.input)
    if not paths:
        raise Refusal(f"no {arguments.source} records for table {arguments.table} resolve "
                      f"below {root}")
    if len(paths) > 1:
        raise Refusal(f"{len(paths)} candidate record files resolve below {root}: {paths}")
    rows = read_csv(paths[0], columns)
    summaries = (summarize_rows if targets_tier else summarize_cells)(rows, arguments.table)
    return render(paths[0], arguments.table, summaries, arguments.source)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, epilog=EXIT_HELP,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", required=True,
                        help="the directory below which every input must resolve")
    parser.add_argument("--table", required=True, help="the periodic rule table to rebuild")
    parser.add_argument("--from", dest="source", required=True,
                        choices=("aggregates", "targets"), help="which record tier to read")
    parser.add_argument("--out", required=True, help="where the markdown table is written")
    parser.add_argument("--input", action="append", default=[],
                        help="an explicit record file, instead of discovery below the root")
    arguments = parser.parse_args(argv)
    try:
        Path(arguments.out).write_text(build(arguments), encoding="utf-8")
    except Refusal as refused:
        print(f"bench_study_report: {refused}", file=sys.stderr)
        return EXIT_REFUSED
    return 0


if __name__ == "__main__":
    sys.exit(main())
