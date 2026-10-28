#!/usr/bin/env python3
"""The sentences a figure in this study cannot be read correctly without.

Each one states what a column is conditioned on, what it is not commensurable
with, or what it excludes. They live beside the table rather than in the
surrounding document because a qualification that can be detached from its table
will be.
"""

PREAMBLE = (
    "Every figure carries the number of targets it was computed over. Order statistics are "
    "nearest rank over the whole target set, with a non-finite error sorted to the worst end, so "
    "no column is conditioned on a solver's own success unless its name says so. Success means "
    "the harness's own verdict, recomputed from the returned joint vector; the solver's own claim "
    "is the separate self-reported column. No figure combines this table with another."
)

WALL_CLAUSE = (
    "A wall-clock column is a diagnostic and not a comparison. A participant budgeted by wall "
    "clock and a participant budgeted by kernel evaluations are not commensurable, contention "
    "moves the first and leaves the second, and the machine's state at capture time is in the "
    "environment record beside these figures."
)

UNCERTAINTY_CLAUSE = (
    "The stated uncertainty on a wall-clock figure is the coefficient of variation of the "
    "per-target wall times within the cell, carried through from the harness's own aggregate row. "
    "It is a within-run dispersion rather than a between-repetition one: this study runs a single "
    "repetition, so no repetition coefficient of variation exists to quote, and manufacturing one "
    "by resampling would describe the resampling."
)

PAIRED_CLAUSE = (
    "A paired difference is defined only on the targets both solvers were accepted on, so every "
    "figure below is conditioned on both of them and carries the paired count against the number "
    "of targets the cell ran. A kernel-evaluation difference exists only where both solvers are "
    "driven on that axis; where one is budgeted by wall clock the column is empty rather than "
    "zero, because a duration and an evaluation count are not commensurable."
)

KERNEL_CLAUSE = (
    "A kernel-evaluation count exists only where the harness drives the solver's own iteration. "
    "Where it does not, the column reads *not counted*: the count is absent, not zero, and it is "
    "never derived from the wall-clock column beside it."
)

ACCURACY_CLAUSE = (
    "Under the accuracy mode each solver is asked for its own calibrated tolerance so that the "
    "accuracies they achieve match, and both numbers are in the table: the accuracy target beside "
    "the tolerance it took to reach it. The accuracy every figure reports is the one the harness "
    "recomputed, never the one a solver was asked for. A target whose calibration did not "
    "converge is marked *unmet* rather than published as reached."
)

STRATUM_CLAUSE = (
    "The unreachable stratum has no success rate. Its targets are not known to be reachable, so a "
    "solver reporting failure there is right rather than unsuccessful, and what it is scored on "
    "is the three-outcome breakdown in the reachability artifact beside this one."
)

HEADINGS = (
    "solver", "budget requested", "budget axis", "targets", "accepted", "self-reported",
    "false success", "false failure", "budget achieved median", "pos err median (m)",
    "pos err p95 (m)", "pos err p99 (m)", "ori err median (rad)", "fk evals median",
    "jac evals median", "wall median (ns)", "wall cv", "solver tolerance", "accuracy target",
    "pos err median accepted (m)",
)

PAIRED_HEADINGS = (
    "solver a", "solver b", "paired targets", "a - b wall median (ns)", "a - b wall p95 (ns)",
    "a - b wall p99 (ns)", "a - b pos err median (m)", "a - b kernel evals median", "wall cv a",
    "wall cv b",
)
