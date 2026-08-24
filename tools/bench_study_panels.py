#!/usr/bin/env python3
"""The four figures the study's cell records carry signal for.

Two candidates were drawn and discarded against this capture: the verdict's
disagreement with each participant's self-report, which is zero false successes
in 1 215 000 solves and 27 false failures, and the unreachable stratum's
breakdown, which is a property of the target set and identical for all five
participants. Both are sentences, and a bar chart of five equal bars reads as a
comparison while carrying none.

Pooling is by counts where a count exists -- an accept rate is the summed
acceptances over the summed targets, never a mean of nine per-robot rates -- and
by median across robots where the record already holds a per-cell median. The
axis labels say which, because a median of medians is a summary of summaries.
"""

import statistics

import matplotlib.pyplot as plt

from figure_style import COLOR, DASH, MARKER, ORDER, WIDTH, label, legend_on

RATED_STRATA = ("reachable", "warm_start_trajectory", "near_singular", "limit_adjacent", "boundary")

ROBOTS = ("universal_robots_ur3e", "universal_robots_ur5e", "universal_robots_ur10",
          "universal_robots_ur16e", "abb_irb120", "kuka_kr6_r900", "franka_panda",
          "kuka_lbr_iiwa14_r820", "kuka_lbr_med14_r820")

SHORT = {"universal_robots_ur3e": "UR3e", "universal_robots_ur5e": "UR5e",
         "universal_robots_ur10": "UR10", "universal_robots_ur16e": "UR16e",
         "abb_irb120": "IRB 120", "kuka_kr6_r900": "KR 6 R900",
         "franka_panda": "Panda", "kuka_lbr_iiwa14_r820": "iiwa 14",
         "kuka_lbr_med14_r820": "Med 14"}

TOP_RUNG = "5"


def accept_rate(rows) -> float:
    targets = sum(int(r["n_targets"]) for r in rows)
    return sum(int(r["n_accepted"]) for r in rows) / targets if targets else float("nan")


def matching(rows, **fields):
    return [r for r in rows if all(r[key] == value for key, value in fields.items())]


def median_of(rows, column):
    values = [float(r[column]) for r in rows if r[column] != ""]
    return statistics.median(values) if values else float("nan")


def grid_on(axis) -> None:
    axis.grid(True, linewidth=0.5, alpha=0.6)
    axis.set_axisbelow(True)
    for side in ("top", "right"):
        axis.spines[side].set_visible(False)


def success_against_budget(cells, _palette):
    figure, axes = plt.subplots(1, len(RATED_STRATA), figsize=(11, 2.5), sharey=True)
    rungs = sorted({r["budget_index"] for r in cells}, key=int)
    for axis, stratum in zip(axes, RATED_STRATA):
        for name in ORDER:
            rates = [accept_rate(matching(cells, solver=name, stratum=stratum, budget_index=b))
                     for b in rungs]
            axis.plot(range(len(rungs)), rates, color=COLOR[name], linestyle=DASH[name],
                      marker=MARKER[name], markersize=3.4, linewidth=WIDTH[name],
                      alpha=0.7 if WIDTH[name] > 2 else 1.0)
        axis.set_title(label(stratum))
        axis.set_xticks(range(len(rungs)))
        axis.set_xticklabels(rungs)
        axis.set_ylim(-0.03, 1.03)
        axis.set_xlabel("budget rung")
        grid_on(axis)
    axes[0].set_ylabel("solved, of targets")
    legend_on(figure, ORDER, reserve=0.16)
    return figure


def accuracy_reached(cells, _palette):
    """A range from the median to the p95, not a bar: on a logarithmic axis a bar
    is drawn from whatever the axis happens to bottom out at, so its length is a
    property of the view rather than of the measurement. The p95 is taken over
    every target the participant was given, unconverged ones included, which is
    why the single-start arms reach a tenth of a metre."""
    figure, axis = plt.subplots(figsize=(9, 3.2))
    width = 0.16
    for offset, name in enumerate(ORDER):
        centers = [i + (offset - 2) * width for i in range(len(RATED_STRATA))]
        rows = [matching(cells, solver=name, stratum=s, budget_index=TOP_RUNG)
                for s in RATED_STRATA]
        medians = [median_of(r, "pos_err_median_m") for r in rows]
        p95s = [median_of(r, "pos_err_p95_m") for r in rows]
        for center, low, high in zip(centers, medians, p95s):
            axis.plot([center, center], [low, high], color=COLOR[name], linewidth=1.4, alpha=0.55)
        axis.plot(centers, medians, linestyle="none", marker=MARKER[name], markersize=5,
                  color=COLOR[name])
        axis.plot(centers, p95s, linestyle="none", marker="_", markersize=6,
                  color=COLOR[name], markeredgewidth=1.4)
    axis.set_yscale("log")
    axis.set_xticks(range(len(RATED_STRATA)))
    axis.set_xticklabels([label(s) for s in RATED_STRATA])
    axis.set_ylabel("position error (m)\nmarker: median across robots, up to the p95")
    grid_on(axis)
    legend_on(figure, ORDER, reserve=0.18)
    return figure


def kernel_work(cells, _palette):
    """The participants whose budget is a count of kernel evaluations spend that
    budget identically -- same algorithm, same iterations -- so the counts panel
    is what makes the times panel a comparison of kernels rather than of solvers.
    The wall-clock-budgeted comparator has no bar here: it is not doing the same
    work, and putting it beside these would invite reading across denominations."""
    countable = [name for name in ORDER
                 if any(r["kernel_countable"] == "1" for r in matching(cells, solver=name))]
    figure, axes = plt.subplots(1, 2, figsize=(9.5, 3.2))
    panels = ((axes[0], "fk_evals_median", 1.0, "kernel evaluations to the solution"),
              (axes[1], "wall_ns_median", 1e-3, "wall time at that work (µs)"))
    for axis, column, scale, title in panels:
        width = 0.8 / len(countable)
        for offset, name in enumerate(countable):
            centers = [i + (offset - (len(countable) - 1) / 2) * width
                       for i in range(len(RATED_STRATA))]
            values = [median_of(matching(cells, solver=name, stratum=s, budget_index=TOP_RUNG),
                                column) * scale for s in RATED_STRATA]
            axis.bar(centers, values, width=width * 0.9, color=COLOR[name], linewidth=0)
        axis.set_title(title)
        axis.set_xticks(range(len(RATED_STRATA)))
        axis.set_xticklabels([label(s) for s in RATED_STRATA], rotation=30, ha="right")
        grid_on(axis)
    axes[0].set_ylabel("median across robots")
    legend_on(figure, countable, reserve=0.16)
    return figure


def success_by_robot(cells, _palette):
    figure, axis = plt.subplots(figsize=(9, 3.0))
    width = 0.16
    for offset, name in enumerate(ORDER):
        centers = [i + (offset - 2) * width for i in range(len(ROBOTS))]
        rates = [accept_rate(matching(cells, solver=name, robot=robot, budget_index=TOP_RUNG))
                 for robot in ROBOTS]
        axis.bar(centers, rates, width=width * 0.9, color=COLOR[name], linewidth=0)
    axis.set_xticks(range(len(ROBOTS)))
    axis.set_xticklabels([SHORT[r] for r in ROBOTS], rotation=30, ha="right")
    axis.set_ylim(0, 1.03)
    axis.set_ylabel("solved at the top rung,\nof targets over all strata")
    grid_on(axis)
    legend_on(figure, ORDER, reserve=0.16)
    return figure


FIGURES = {
    "success-against-budget": success_against_budget,
    "accuracy-reached": accuracy_reached,
    "kernel-work": kernel_work,
    "success-by-robot": success_by_robot,
}
