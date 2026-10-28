#!/usr/bin/env python3
"""The two-theme surface the study's figures are drawn on.

A figure in this repository is read on a white page and on a dark one, so each
is rendered twice from one call and the document picks between them with
``prefers-color-scheme``; one rendering is washed out on whichever theme it was
not drawn for.

Rendering is pinned to be byte-stable -- glyphs written as paths, a fixed
element-id salt, no timestamp -- so a regenerated figure that differs from the
one in the tree differs because the records did.

The participants are coloured in pairs and separated within a pair by line
style, because the restarting and single-start arms of one solver are the
comparison a reader makes first.
"""

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

SERIES = (
    ("cartan_lm", "#2f6fb2", "-", "o", 3.2),
    ("cartan_restart_lm", "#2f6fb2", "--", "o", 3.2),
    ("pinocchio_lm", "#d17c1f", "-", "x", 1.4),
    ("pinocchio_restart_lm", "#d17c1f", "--", "x", 1.4),
    ("trac_ik", "#7d5bbe", "-", "s", 1.6),
)

COLOR = {row[0]: row[1] for row in SERIES}

DASH: dict = {row[0]: row[2] for row in SERIES}

MARKER: dict = {row[0]: row[3] for row in SERIES}

WIDTH = {row[0]: row[4] for row in SERIES}

ORDER = [row[0] for row in SERIES]

THEMES = {
    "light": {"ink": "#1c1c1c", "muted": "#5c5c5c", "grid": "#d8d8d8", "paper": "#ffffff"},
    "dark": {"ink": "#e8e8e8", "muted": "#a0a0a0", "grid": "#3a3a3a", "paper": "#0d1117"},
}


def apply(theme: str) -> dict:
    palette = THEMES[theme]
    matplotlib.rcParams.update({
        "svg.fonttype": "path",
        "svg.hashsalt": "cartan-study",
        "figure.facecolor": palette["paper"],
        "axes.facecolor": palette["paper"],
        "axes.edgecolor": palette["muted"],
        "axes.labelcolor": palette["ink"],
        "axes.titlecolor": palette["ink"],
        "text.color": palette["ink"],
        "xtick.color": palette["muted"],
        "ytick.color": palette["muted"],
        "xtick.labelcolor": palette["ink"],
        "ytick.labelcolor": palette["ink"],
        "grid.color": palette["grid"],
        "font.size": 9,
        "axes.titlesize": 10,
        "legend.frameon": False,
        "figure.dpi": 100,
    })
    return palette


def label(name: str) -> str:
    return name.replace("_", " ")


def legend_on(figure, names, reserve: float = 0.22) -> None:
    """``reserve`` is the fraction of figure height kept clear at the bottom; the
    legend is laid into it after the axes have been fitted above it."""
    handles = [Line2D([], [], color=COLOR[n], linestyle=DASH[n], marker=MARKER[n],
                      markersize=4, linewidth=WIDTH[n]) for n in names]
    figure.tight_layout(rect=(0, reserve, 1, 1))
    figure.legend(handles, [label(n) for n in names], loc="lower center",
                  ncol=len(names), bbox_to_anchor=(0.5, 0.0))


def render(build, out_dir, name: str) -> list:
    """``build`` draws one figure against the palette it is handed; it is called
    once per theme because the rc state it draws under is global."""
    written = []
    for theme in THEMES:
        palette = apply(theme)
        figure = build(palette)
        path = out_dir / f"{name}-{theme}.svg"
        figure.savefig(path, format="svg", bbox_inches="tight", metadata={"Date": None})
        plt.close(figure)
        written.append(path)
    return written
