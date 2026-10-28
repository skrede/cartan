#!/usr/bin/env python3
"""The published table: one section per cell, with its paired differences beneath it.

Every numeric cell carries the count it was computed over and every rate names
its denominator, because a survivor-conditioned mean and an unconditioned one are
indistinguishable once the denominator is dropped. A qualification is rendered
inside the table it qualifies rather than as a footnote elsewhere: an absent
participant, a participant excluded from the kernel-evaluation axis and a row
excluded for standing on bounds nobody declared are all stated here.

Nothing in this module can emit a ratio summarizing a table, a robot or the
study. The same experiment produced a twenty-six-fold and a fourteen-fold figure
on one robot depending only on machine contention, so a single quotable
multiplier is a claim about the machine that measured it.
"""

import math

from bench_study_clauses import (
    ACCURACY_CLAUSE, HEADINGS, KERNEL_CLAUSE, PAIRED_CLAUSE, PAIRED_HEADINGS, PREAMBLE,
    STRATUM_CLAUSE, UNCERTAINTY_CLAUSE, WALL_CLAUSE,
)
from bench_study_records import IDENTITY, NO_SUCCESS_RATE, Refusal

COUNTED = ("n_self_reported", "n_false_success", "n_false_failure")

MEASURED = ("budget_value_median", "pos_err_median_m", "pos_err_p95_m", "pos_err_p99_m",
            "ori_err_median_rad", "fk_evals_median", "jac_evals_median", "wall_ns_median",
            "wall_ns_cv")

DELTAS = ("delta_wall_ns_median", "delta_wall_ns_p95", "delta_wall_ns_p99",
          "delta_pos_err_median_m", "delta_kernel_evals_median")

SECTION_FIELDS = ("robot", "limits_provenance", "stratum", "budget_index")


def markdown(text):
    """A record field is data; it must not be able to close a cell or open code."""
    return str(text).replace("|", r"\|").replace("`", r"\`").replace("\n", " ").replace("\r", " ")


def figure(value, count):
    if value is None:
        return "not counted"
    if not math.isfinite(value):
        return f"non-finite (n = {count})"
    return f"{value:.6g} (n = {count})"


def share(count, targets):
    if not targets:
        return "0 / 0"
    return f"{int(count)} / {int(targets)} ({100.0 * count / targets:.2f}%)"


def accuracy_figure(summary):
    """The mode a cell ran under, and whether its target was actually reached."""
    target = summary["accuracy_target"]
    if target is None:
        return "budget mode"
    return f"{target:.6g}" + ("" if summary["accuracy_target_met"] else " (unmet)")


def accepted_cell(summary, targets):
    """A stratum whose targets are not known to be reachable has no success rate to
    render, so the count is rendered as the claim it is."""
    if summary["accept_rate"] is None:
        return f"{int(summary['n_accepted'])} of {int(targets)} claims, not a success rate"
    return share(summary["n_accepted"], targets)


def table_row(cells):
    return "| " + " | ".join(cells) + " |"


def headed(headings):
    return [table_row(headings), "|" + "|".join(["---"] * len(headings)) + "|"]


def solver_row(key, summary):
    targets = summary["n_targets"]
    cells = [markdown(key[IDENTITY.index(name)])
             for name in ("solver", "budget_requested", "budget_axis")]
    cells.append(f"{int(targets)}")
    cells.append(accepted_cell(summary, targets))
    cells += [share(summary[name], targets) for name in COUNTED]
    cells += [figure(summary[name], int(targets)) for name in MEASURED]
    cells.append(f"{summary['solver_tolerance']:.6g}")
    cells.append(accuracy_figure(summary))
    cells.append(figure(summary["pos_err_median_accepted_m"], int(summary["n_accepted"])))
    return table_row(cells)


def paired_row(key, difference, members):
    count = int(difference["n_paired"])
    denominator = min(summary["n_targets"] for summary in members)
    cells = [markdown(key[-2]), markdown(key[-1]), share(count, denominator)]
    cells += [("no paired target (n = 0)" if count == 0 else figure(difference[name], count))
              for name in DELTAS]
    cells += [figure(summary["wall_ns_cv"], int(summary["n_targets"])) for summary in members]
    return table_row(cells)


def absence_clause(participant, reason):
    return (f"**{markdown(participant)} is absent from this table.** The capture declared it and "
            f"recorded why it did not run: {markdown(reason)}. Its rows are missing rather than "
            f"zero, and no figure here is computed as if it had participated.")


def kernel_exclusion_clause(participants):
    named = ", ".join(f"**{markdown(name)}**" for name in participants)
    return (f"Excluded from the kernel-evaluation axis: {named}. The harness does not drive that "
            f"solver's own iteration, so no evaluation can be attributed to it, and a record "
            f"giving it a count is refused rather than plotted.")


def synthetic_clause(count):
    return (f"Excluded from every figure here: {count} record rows whose joint bounds were not "
            f"read from a robot description. A box nobody declared cannot reject a configuration, "
            f"so a limit-dependent figure computed over one describes the invention.")


def preamble(study):
    lines = [f"# Study table {markdown(study.table)}", "",
             f"Records: `{markdown(study.records)}` ({markdown(study.tier)} tier).",
             f"Paired differences: {markdown(study.paired_source)}.",
             f"Participants declared by: `{markdown(study.manifest.path)}`.", "",
             PREAMBLE, "", WALL_CLAUSE, "", UNCERTAINTY_CLAUSE, "", KERNEL_CLAUSE, ""]
    if any(summary["accuracy_target"] is not None for summary in study.summaries.values()):
        lines += [ACCURACY_CLAUSE, ""]
    if any(key[IDENTITY.index("stratum")] == NO_SUCCESS_RATE for key in study.summaries):
        lines += [STRATUM_CLAUSE, ""]
    for participant, reason in study.absences:
        lines += [absence_clause(participant, reason), ""]
    if study.uncountable:
        lines += [kernel_exclusion_clause(study.uncountable), ""]
    if study.excluded:
        lines += [synthetic_clause(study.excluded), ""]
    return lines


def sections(summaries):
    grouped = {}
    for key, summary in summaries.items():
        section = tuple(key[IDENTITY.index(name)] for name in SECTION_FIELDS)
        grouped.setdefault(section, []).append((key, summary))
    return grouped


def member(members, solver, pair):
    for key, summary in members:
        if key[IDENTITY.index("solver")] == solver:
            return summary
    raise Refusal(f"the paired row {pair} names {solver!r}, which the per-cell records of that "
                  f"cell do not carry")


def unpaired_clause(members):
    return (f"No paired difference is defined in this cell: {len(members)} participant resolved, "
            f"and a paired per-target difference is defined only between two.")


def paired_table(study, section, members):
    keys = sorted(key for key in study.paired if key[:5] == (study.table,) + section)
    if not keys:
        return ["", unpaired_clause(members)]
    lines = ["", PAIRED_CLAUSE, ""] + headed(PAIRED_HEADINGS)
    for key in keys:
        pair = [member(members, key[-2], key), member(members, key[-1], key)]
        lines.append(paired_row(key, study.paired[key], pair))
    return lines


def render(study):
    lines = preamble(study)
    grouped = sections(study.summaries)
    for section in sorted(grouped):
        robot, provenance, stratum, rung = (markdown(part) for part in section)
        members = sorted(grouped[section], key=lambda pair: pair[0])
        lines += [f"## {robot} -- {stratum} stratum, {provenance} limits, budget rung {rung}", ""]
        lines += headed(HEADINGS)
        lines += [solver_row(key, summary) for key, summary in members]
        lines += paired_table(study, section, members)
        lines.append("")
    return "\n".join(lines) + "\n"
