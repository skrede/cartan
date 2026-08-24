# Study evidence — capture of 2026-08-11

Everything the published study reports was produced by one recorded run on one machine under
stated conditions. This directory is that run's evidence.

## What ships here

| Path | What it is | Resolution |
|---|---|---|
| `raw/2026-08-11/ladder/` | the iso-budget sweep: per-cell aggregates, per-stratum paired differences, reachability breakdowns | one row per (table, robot, limits provenance, stratum, solver, budget rung) |
| `raw/2026-08-11/accuracy/{1e-05,1e-06,1e-07}/` | the matched-accuracy sweep, one directory per accuracy target | one row per cell, at a single budget rung |
| `raw/2026-08-11/iso_accuracy_calibration.csv` | what each solver had to be asked for to deliver each accuracy | 810 rows |
| `raw/2026-08-11/*/environment.json` | the conditions the numbers were taken under, merged across every process of the capture | one record per record set |

The shipped records are **aggregates**. The full per-target rows — 6.9 million of them, one per solve —
are not in this repository; they are a release asset, `table_{a,b,c}_targets.csv.gz`, produced under
an ignored build path at capture time.

The target sets themselves are not shipped either. The capture writes them to its `--sidecar-dir`,
which defaults to the ignored `study-targets/`, and they follow from the harness, the pinned
descriptions and a fixed pool seed alone -- every target's draw is seeded by a hash of its own
identity, so a rerun of the same command on the same toolchain reproduces them byte for byte.

**What the shipped set is sufficient for.** Every published table rebuilds from these aggregates
alone, with nothing but `python3` and `tools/bench_study_report.py`; that is the reproducibility this
repository offers, and it is a claim about *rebuilding the published tables*, not about *rerunning the
study* — rerunning needs the comparator libraries, which this repository neither vendors nor
undertakes to make buildable, so a reader without them can reproduce every number without being able
to regenerate it. At capture time both tiers were rebuilt independently and cross-checked against each
other: 64 800 published figures per table agreed between the aggregate and per-target tiers.

Rebuild a table with:

```
python3 tools/bench_study_report.py --root docs/benchmarks/raw/2026-08-11/ladder \
    --table c --from aggregates --out table_c.md
```

## Why the record sets are split by mode

The analysis tooling identifies a cell by table, robot, limits provenance, stratum, solver and budget
index. A matched-accuracy run reuses the top rung's budget index, so accuracy rows and top-rung budget
rows share an identity and one silently displaces the other when both sit in one file. Keeping the two
modes in separate directories is what avoids that; it is a property of the tooling rather than of the
experiment, and the two modes are separate experiments in any case.

## Conditions

| | |
|---|---|
| CPU | AMD Ryzen 7 5800X3D, 8 physical cores |
| Simultaneous multithreading | off, read back from the kernel after writing |
| Frequency boost | off, read back |
| Governor | `performance` (`amd-pstate-epp`) |
| Compiler | GCC 16.1.1, Release |
| Passes | 3 independent sweeps of the full matrix |

Repeatability across the three passes, as the coefficient of variation of each cell's median wall time:

| solver | cells | CV median | CV max |
|---|---|---|---|
| `cartan_lm` | 1836 | 0.275% | 6.4% |
| `cartan_restart_lm` | 1836 | 0.266% | 6.3% |
| `pinocchio_lm` | 1836 | 0.462% | 11.4% |
| `pinocchio_restart_lm` | 1836 | 0.434% | 10.2% |
| `trac_ik` | 1836 | 1.120% | 25.3% |

The four work-budgeted participants sit under half a percent, which is what makes "the machine was
quiet" evidence rather than an assertion. The comparator's wider spread is a property of how it is
budgeted rather than of the machine: a wall-clock-budgeted solver does however much work fits inside
its cap, so the work it performs varies between passes where a work-budgeted solver's does not.

## What the participants are

`cartan_lm` and `cartan_restart_lm` are this repository's Levenberg-Marquardt, single-start and
restarting.

`pinocchio_lm` and `pinocchio_restart_lm` are **this repository's Levenberg-Marquardt driven over
Pinocchio's kinematics kernels** — Pinocchio ships no inverse-kinematics solver, so there is no
Pinocchio solver here to compare against. Both peer participants call `framesForwardKinematics`,
`computeFrameJacobian` and `log6`; everything above those calls is this repository's code, including
the damping update, the stall and divergence rules, and the joint-limit gate. That is deliberate: it
is what makes the pair a comparison of kinematics kernels with the algorithm held fixed. It also means
these rows say nothing about Pinocchio as a solver. Sharing this repository's termination rules moves
the peer's success rate on the near-singular stratum by 2.8 points relative to a peer terminating on
its own rules.

`trac_ik` is the external comparator, budgeted by wall clock where every other participant is budgeted
by work units. The two denominations are not commensurable and no figure here converts between them.

## Known limitation in this capture

`pinocchio_restart_lm`'s rows in `raw/2026-08-11/accuracy/` were run against a calibration entry that
was measured while that participant was not restarting, so its requested tolerance there is a
single-start figure. Those rows must not be read as matched-accuracy results. The defect is confined
to that one participant in the matched-accuracy sets: the calibration table's other entries reproduce
exactly on re-run, and the iso-budget sweep in `ladder/` never reads the calibration table at all, so
every figure derived from it is unaffected.

`trac_ik` seeds its internal restarts from `rand()` and does not reproduce bit-exactly between runs.
Every other participant does.
