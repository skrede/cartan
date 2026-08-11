# Benchmarks

This page publishes two kinds of measurement, captured on one machine on 2026-08-11 and kept
apart because they answer different questions.

The **iterative study** is a peer comparison among iterative inverse-kinematics solvers. It is not
a comparison against one incumbent: cartan's Levenberg-Marquardt and a Levenberg-Marquardt driven
over another library's kinematics kernels differ by less than the spread between repetitions on
the axis that matters to a solver's user, which is how often it returns a usable answer. What the
study varies, one axis at a time, is the kinematics kernels underneath a fixed algorithm, and the
search strategy above fixed kernels.

The **microbenchmarks** measure single operations — forward kinematics, the Jacobian, and the Lie
group primitives underneath both. There a ratio against a reference implementation is a meaningful
quantity, and this page states it.

Every number below carries a source marker naming the record file it was computed from. Those
records ship in this repository under `docs/benchmarks/raw/`, and
`tools/check_benchmark_claims.py` refuses a claim whose record is missing.

## Conditions

<!-- source: docs/benchmarks/raw/2026-08-11/ladder/environment.json -->

| | |
|---|---|
| CPU | AMD Ryzen 7 5800X3D, 8 physical cores |
| Simultaneous multithreading | off, read back from the kernel after writing |
| Frequency boost | off, read back from the kernel after writing |
| Governor | `performance` (`amd-pstate-epp`) |
| Kernel | 6.18.42-1-lts |
| Compiler | GCC 16.1.1, Release |
| Standard library | libstdc++ 16 (20260728) |
| Eigen | 3.4.0 |
| Study passes | 3 independent sweeps of the full matrix |
| Microbenchmark repetitions | 5, median reported |

Dependencies are recorded in three classes, because they are three different kinds of evidence. A
**fetched** dependency is pinned to an immutable commit by this project's build. A **discovered**
one is whatever the machine resolved, and carries the version it reported. A **generated** one was
produced by a code generator and carries the generator's own provenance.

| dependency | class | identity |
|---|---|---|
| `benchmark` | fetched | 1.9.5 at `192ef100` |
| `meios` | fetched | `b772a310` |
| `argmin` | fetched | `864d558c` |
| `pinocchio` | discovered | 4.0.0 |
| `orocos_kdl` | discovered | 1.5.3 |
| `trac_ik` | discovered | 2.0.0 at `9511f2f3` |
| `nlopt` | discovered | 2.11.0 |
| `lapack` | discovered | 3.11.0 |
| `ik_geo` | discovered | 0.1.2 |
| `ikfast_kr6r900` | generated | ikfast `0x1000004c`, kinematics hash `06b2c8e082c6110f0c322529e18b8ef5` |

The robot descriptions are fetched from their upstream repositories rather than transcribed, so a
joint limit in this study is the one the manufacturer's description declares.

## The participants

`cartan_lm` and `cartan_restart_lm` are this repository's Levenberg-Marquardt, single-start and
restarting.

`pinocchio_lm` and `pinocchio_restart_lm` are **this repository's Levenberg-Marquardt driven over
Pinocchio's kinematics kernels**. Pinocchio ships no inverse-kinematics solver, so there is no
Pinocchio solver here to compare against. Both call `framesForwardKinematics`,
`computeFrameJacobian` and `log6`; everything above those calls is this repository's code,
including the damping update, the stall and divergence rules, and the joint-limit gate. That is
what makes the pair a comparison of kinematics kernels with the algorithm held fixed, and it also
means these rows say nothing about Pinocchio as a solver. Sharing this repository's termination
rules moves the peer's success rate on the near-singular stratum by 2.8 points relative to a peer
terminating on its own rules.

`trac_ik` is the external comparator. It is budgeted by wall clock where every other participant is
budgeted by kernel evaluations. The two denominations are not commensurable, and no figure on this
page converts between them.

## Method

<!-- source: docs/benchmarks/raw/2026-08-11/ladder/table_c_cells.csv -->

**One feasible set per run.** The harness constructs a single feasible set and hands it to every
participant. No participant can be given a different problem from the one beside it, because
there is no per-solver limits argument to give it.

**Success is the harness's verdict, not the solver's claim.** Every returned joint vector is run
back through forward kinematics and scored against the target. The solver's own return code is
recorded separately, as the self-reported column, so the two can disagree in the record and be
counted when they do.

**Six strata.** Targets are drawn as `reachable`, `boundary`, `near_singular`, `limit_adjacent`,
`warm_start_trajectory` and `unreachable`. The unreachable stratum carries no success rate: its
targets are not known to be reachable, so a solver reporting failure there is right rather than
unsuccessful, and what it is scored on is the three-outcome breakdown below.

**A budget ladder, not a timeout.** Each participant is run at six budget rungs — 9, 27, 81, 243,
729 and 2187 kernel evaluations — so what is published is success against compute rather than
success at one arbitrary cut-off. The wall-clock-budgeted comparator is run at the rungs its own
denomination admits, 1.5 ms through 50 ms.

**Two limits provenances.** Every cell is run once against the manufacturer's declared joint
limits and once against the symmetric ±π box that the previous version of this study used, so the
effect of that choice is a measured axis rather than an assumption.

**Three periodic rules, three tables.** A joint's range may span more than a full turn, which makes
"the same configuration" ambiguous. Rather than pick one convention, the study runs three:

| table | rule |
|---|---|
| A | a joint whose declared range spans at least a full turn is treated as unbounded |
| B | unbounded while solving, canonicalized for verification and comparison |
| C | canonicalized to (-π, π] throughout |

Each table is its own experiment with its own records and its own conclusions. Nothing on this page
combines two of them, and the analysis script emits no statistic that spans them.

## What the declared limits actually are

<!-- source: docs/benchmarks/raw/2026-08-11/ladder/environment.json -->

The periodic rule only has a subject where a joint's range spans a full turn, and the limits
change only bites where the declared range differs from ±π. Both vary strongly by robot:

| robot | joints | wider than ±π | narrower | spans ≥ a full turn | unbounded | range excluding zero |
|---|---|---|---|---|---|---|
| `universal_robots_ur3e` | 6 | 6 | 0 | 6 | 1 | 0 |
| `universal_robots_ur5e` | 6 | 6 | 0 | 6 | 0 | 0 |
| `universal_robots_ur10` | 6 | 6 | 0 | 6 | 0 | 0 |
| `universal_robots_ur16e` | 6 | 6 | 0 | 6 | 0 | 0 |
| `kuka_kr6_r900` | 6 | 3 | 3 | 2 | 0 | 0 |
| `abb_irb120` | 6 | 1 | 5 | 1 | 0 | 0 |
| `franka_panda` | 7 | 1 | 6 | 0 | 0 | 1 |
| `kuka_lbr_iiwa14_r820` | 7 | 0 | 7 | 0 | 0 | 0 |
| `kuka_lbr_med14_r820` | 7 | 0 | 7 | 0 | 0 | 0 |

Two consequences follow directly, and both are findings rather than caveats.

**The three periodic tables have no subject on three of the nine robots.** The Franka arm and the
two seven-axis KUKA arms declare no joint spanning a full turn, so tables A, B and C measure the
identical problem there. On the four UR arms nearly every joint qualifies, and one UR3e joint is
declared genuinely unbounded.

**The symmetric ±π box was not uniformly the easier problem.** It is *tighter* than the declared
range on every UR joint and on the sixth joint of the ABB and KUKA six-axis arms, and *looser* on
the Franka arm and both seven-axis KUKA arms. One Franka joint's declared range is
[-3.07, -0.07] rad and does not contain zero at all, so the symmetric box admitted configurations
the real robot cannot reach.

## The iterative study

The three tables agree closely enough that publishing all three in full would be repetition; the
per-stratum ladders below are table C, and the records for A and B ship beside them and rebuild
with the same command. Where A or B differs materially it is stated.

### Success against compute budget

<!-- source: docs/benchmarks/raw/2026-08-11/ladder/table_c_cells.csv -->

Median over the nine robots, manufacturer limits, table C. Each cell is the harness-verified
acceptance rate at that budget rung. Columns are the five participants in the order
`cartan_lm` / `cartan_restart_lm` / `pinocchio_lm` / `pinocchio_restart_lm` / `trac_ik`.

| stratum | 9 | 27 | 81 | 243 | 729 | 2187 |
|---|---|---|---|---|---|---|
| reachable | 0.0/0.0/0.1/0.1/98.6 | 20.3/20.3/27.7/27.7/99.4 | 71.8/83.5/71.8/83.9/99.9 | 72.0/96.1/72.0/96.2/100.0 | 72.0/99.6/72.0/99.6/100.0 | 72.0/100.0/72.0/100.0/100.0 |
| boundary | 0.0/0.0/0.0/0.0/97.0 | 10.2/10.2/13.4/13.4/99.6 | 53.2/54.0/53.4/54.2/99.6 | 54.8/80.0/54.8/80.2/99.8 | 54.8/97.2/54.8/97.4/100.0 | 54.8/100.0/54.8/100.0/100.0 |
| near_singular | 0.0/0.0/0.0/0.0/96.0 | 5.8/5.8/7.8/7.8/98.0 | 57.2/59.2/59.2/61.0/98.6 | 76.4/93.0/76.4/93.2/99.2 | 76.8/99.4/76.8/99.4/99.8 | 76.8/100.0/76.8/100.0/99.8 |
| limit_adjacent | 0.0/0.0/0.0/0.0/98.0 | 18.2/18.2/24.4/24.4/99.6 | 61.4/80.6/61.6/81.6/100.0 | 63.2/95.6/63.2/96.0/100.0 | 63.2/99.8/63.2/99.8/100.0 | 63.2/100.0/63.2/100.0/100.0 |
| warm_start_trajectory | 0.2/0.2/10.2/10.2/99.8 | 91.4/91.4/91.4/90.0/100.0 | 96.2/97.6/96.2/97.6/100.0 | 95.4/100.0/95.4/100.0/100.0 | 95.4/100.0/95.4/100.0/100.0 | 95.4/100.0/95.4/100.0/100.0 |

The comparator's column is not on the same budget denomination as the four beside it and its
progression across these six columns is a progression across its own wall-clock rungs. Read down
a column, not across the participant groups within one.

Three things are visible in this table, and each is the answer to one of the study's questions.

**The restart strategy is what closes the gap.** A single-start Levenberg-Marquardt saturates well
below full success on every stratum — 72.0% reachable, 54.8% boundary, 63.2% limit-adjacent — and
adding budget past 243 evaluations does not move it, because its failures are wrong-basin rather
than under-converged. The same solver restarted reaches 100.0% on every stratum by the top rung.

**The kinematics kernels contribute nothing to success.** `cartan_lm` and `pinocchio_lm` are
identical to the digit at every rung from 243 upward, as are `cartan_restart_lm` and
`pinocchio_restart_lm`. That is the expected result of holding the algorithm fixed and swapping
only the kernels underneath it, and it is what licenses reading the two restarting rows as one
finding rather than two.

**The restarting solver matches the external comparator.** At the top rung both reach 100.0% on
four of five strata; on `near_singular` the restarting solvers reach 100.0% against the
comparator's 99.8%. No claim about relative *speed* follows from this table, for the reason given
under the wall-time section below.

### Kernel evaluations

<!-- source: docs/benchmarks/raw/2026-08-11/ladder/table_c_cells.csv -->

**Excluded from this axis: `trac_ik`.** The harness does not drive that solver's own iteration, so
no kernel evaluation can be attributed to it. Its cells read *not counted* in the records; the
count is absent rather than zero, and it is never derived from the wall-clock figure beside it.

At the top rung on the reachable stratum, `cartan_lm` and `pinocchio_lm` spend the same number of
forward-kinematics evaluations per solve — 12 to 13 across the nine robots — and the two restarting
participants the same 15 to 26 as each other. Equal counts on this axis are what makes the wall
time beside them a measurement of the kernels rather than of the search.

### Do the return codes tell the truth?

<!-- source: docs/benchmarks/raw/2026-08-11/ladder/table_c_cells.csv -->

Over 243 000 scored targets per participant, manufacturer limits, all strata and all rungs:

| participant | targets | claimed success that was not | claimed failure that was not |
|---|---|---|---|
| `cartan_lm` | 243 000 | 0 | 10 |
| `cartan_restart_lm` | 243 000 | 0 | 5 |
| `pinocchio_lm` | 243 000 | 0 | 11 |
| `pinocchio_restart_lm` | 243 000 | 0 | 1 |
| `trac_ik` | 243 000 | 0 | 0 |

No participant reported a success the harness could not confirm. This is the question the previous
version of this page could not answer about its own solvers, because it took their success from
the return value and never gated on a recomputed pose; here every participant's claim and the
harness's verdict are separate recorded columns, and they disagree on 27 targets out of 1 215 000,
always in the conservative direction.

### Reachability claims

<!-- source: docs/benchmarks/raw/2026-08-11/ladder/table_c_reachability.csv -->

Success rate and reachability-claim correctness are different questions. On the unreachable
stratum, manufacturer limits, table C, summed across robots:

| outcome | count |
|---|---|
| targets in the stratum | 4 500 |
| correctly reported unreachable | 2 673 |
| falsely claimed reached | 0 |
| reclassified as in fact reachable | 1 827 |

The reclassification count is the interesting one: 40.6% of the targets this stratum constructed
as unreachable turned out to be solvable, which is a statement about the difficulty of generating
a genuinely unreachable pose rather than about any solver. Every participant produced the same
three counts. No participant claimed to have reached a target it had not.

### What changed when the joint limits became the manufacturer's

<!-- source: docs/benchmarks/raw/2026-08-11/ladder/table_c_cells.csv -->

Median change in acceptance rate, manufacturer limits minus the symmetric ±π box, at the top rung:

| stratum | `cartan_lm` | `cartan_restart_lm` | `pinocchio_lm` | `trac_ik` |
|---|---|---|---|---|
| reachable | −24.0 | +0.0 | −24.0 | +0.0 |
| boundary | −10.8 | +0.0 | −10.8 | +0.0 |
| near_singular | −9.4 | +0.0 | −9.4 | −0.2 |
| limit_adjacent | −33.0 | +0.0 | −33.0 | +0.0 |
| warm_start_trajectory | −4.4 | +0.0 | −4.4 | +0.0 |

**The direction is not uniform, and it is not uniform by robot.** On the limit-adjacent stratum the
single-start solver loses 62.2 points on the Franka arm, 45.2 on the ABB arm and 33.0 to 39.4 on
the three KUKA arms — but 0.2 to 2.0 points on the four UR arms, whose declared ranges are wider
than the box they replaced. Across the whole record set there are 260 cells in which the
manufacturer's limits produced the *higher* acceptance rate, the largest being the KR 6 on the
warm-start stratum at the lowest rung: 94.0% under declared limits against 16.0% under the
symmetric box.

So "the old numbers were measured on an easier problem" is true for the Franka arm and the ABB
arm, and false for the UR arms. A success rate that rises after the limits change is a correct
result on a differently-shaped feasible set, not a defect.

**A multi-start solver is insensitive to the change.** Both restarting participants and the
external comparator move by 0.0 to 0.2 points on every stratum. What the limits change moves is
the single-start solver, whose failures were already wrong-basin.

### Matched accuracy

<!-- source: docs/benchmarks/raw/2026-08-11/accuracy/1e-06/table_c_cells.csv -->
<!-- source: docs/benchmarks/raw/2026-08-11/iso_accuracy_calibration.csv -->

The budget ladder holds compute fixed and lets accuracy fall where it may. This mode does the
reverse: each participant's own requested tolerance is calibrated until the error the harness
recomputes lands on a target, and the calibrated tolerance and the achieved error are published
together. Table C, manufacturer limits, reachable stratum, median over the nine robots.

| accuracy target | participant | success % | achieved position error (m) | robots on target |
|---|---|---|---|---|
| 1e-05 | `cartan_lm` | 72.0 | 8.34e-06 | 8/9 |
| 1e-05 | `cartan_restart_lm` | 100.0 | 8.56e-06 | 9/9 |
| 1e-05 | `pinocchio_lm` | 72.0 | 8.28e-06 | 8/9 |
| 1e-05 | `trac_ik` | 100.0 | 6.39e-06 | 2/9 |
| 1e-06 | `cartan_lm` | 72.0 | 8.86e-07 | 5/9 |
| 1e-06 | `cartan_restart_lm` | 100.0 | 8.73e-07 | 5/9 |
| 1e-06 | `pinocchio_lm` | 72.0 | 8.86e-07 | 5/9 |
| 1e-06 | `trac_ik` | 100.0 | 3.64e-07 | 1/9 |
| 1e-07 | `cartan_lm` | 72.0 | 7.30e-08 | 3/9 |
| 1e-07 | `cartan_restart_lm` | 99.9 | 7.90e-08 | 5/9 |
| 1e-07 | `pinocchio_lm` | 72.0 | 7.30e-08 | 3/9 |
| 1e-07 | `trac_ik` | 99.9 | 2.44e-08 | 0/9 |

The "robots on target" column is the honest limit of this mode: calibration does not always find a
requested tolerance that lands on the wanted accuracy, and the comparator is the hardest to
calibrate because it stops on a component-wise bound and overshoots a two-norm target. Its
achieved error is consistently tighter than asked for, which is the same fact as its longer solve
times rather than a separate one.

`pinocchio_restart_lm` is omitted from this table. Its rows in the matched-accuracy records were
run against a calibration entry measured while that participant was not restarting, so its
requested tolerance there is a single-start figure and those rows are not matched-accuracy
results. The defect is confined to that participant in this mode; the budget ladder never reads
the calibration table.

### Wall time, and why it is not a comparison here

<!-- source: docs/benchmarks/raw/2026-08-11/ladder/table_c_cells.csv -->

The records carry a wall-time column for every cell and it is a diagnostic, not a ranking. Four
participants are budgeted by kernel evaluations and stop when the work runs out; the external
comparator is budgeted by wall clock and stops when the time runs out. Two solvers that stopped
for different reasons have not been raced. The comparator's longer times and its tighter achieved
error are the same fact about how it was budgeted, and neither is evidence about speed.

Establishing a speed comparison against it needs a capture in which the accuracy binds and the
budget does not, on both sides. That experiment has not been run, so this page makes no speed
claim against the external comparator.

A comparison the records *do* support is the one against Pinocchio's kernels, since both sides are
driven by the same algorithm at the same kernel budget — but the participants that produced these
ladder records carry counting instrumentation, and the peer's inline counters cost it 1.2% to 3.1%.
A ratio computed from these records would therefore be biased in this repository's favour, so none
is published. The uninstrumented kernel comparison is the microbenchmark section below.

## Closed-form solvers

This section is separate from everything above it, and a figure must not be carried across.

A closed-form solve has no budget, no iteration count and no convergence tolerance. Success against
a compute budget is therefore not a question that can be asked of it, and forcing it onto that axis
compares quantities that do not correspond. What can be asked is whether it is exact, how many
branches of the inverse it returns, and how much of the pose space it admits. Those are the axes
here, and there are no others.

**This section publishes no timing.** That is deliberate, and it is why these figures did not need
the recorded machine conditions the study above required: exactness, branch count and coverage are
not sensitive to machine contention. A timing comparison among closed-form solvers is a different
experiment that has not been run, and nothing here may be read as one.

### Exactness and coverage

<!-- source: docs/benchmarks/raw/2026-08-11/closed_form_reference.csv -->

| robot | solver | exact | branches | pose-space coverage | max position error (m) | max orientation error (rad) | poses |
|---|---|---|---|---|---|---|---|
| ABB IRB 120 | `cartan_opw_6r` | yes | – | 41.9% | 5.13e-13 | 1.70e-13 | 10 000 |
| KUKA KR 6 R900 | `cartan_opw_6r` | yes | – | 46.1% | 3.10e-13 | 5.49e-13 | 10 000 |
| KUKA KR 6 R900 | `opw_kinematics` | yes | 15 168 | – | – | – | – |
| KUKA KR 6 R900 | `ikfast_kr6r900` | yes | 15 168 | – | – | – | – |
| KUKA KR 6 R900 | `ik_geo` | yes | 15 168 | – | – | – | – |

Coverage is the fraction of a bounding-box-uniform pose set the solver admits; that probe
deliberately over-samples outside the reachable workspace, so the figure is a property of the
chain's workspace against its bounding box and is not a success rate. A dash marks a quantity not
measured for that row rather than a zero: the three witness rows record branch agreement against
this library's solver on one arm, not an independent pose measurement of their own.

**Comparators absent from this capture appear as a named absence with a reason, never as a missing
row.** None was absent here; when one is, the record tool reports it in the form

```
ABSENT opw_kinematics on kr6_sixx: its comparison benchmark was not built, so the
comparator was not found when this build was configured
```

### What each comparator is, and how independent it is

| comparator | class | identity |
|---|---|---|
| `cartan_opw_6r` | this library | the analytic 6R solver this repository ships |
| `opw_kinematics` | discovered source checkout | tag 0.5.5, commit `8a32bda8197c50bd0d60dfe1d12ecb4c13111b72` |
| `ikfast_kr6r900` | generated, vendored | ikfast `0x1000004c`, kinematics hash `06b2c8e082c6110f0c322529e18b8ef5` |
| `ik_geo` | discovered crate | the published Rust crate `ik-geo` 0.1.2, its `spherical_two_parallel` solver, driven through a C interface shim |

The build records `opw_kinematics` under its commit rather than its tag, because that project
interpolates its own version from a package manifest and declares no literal version to read; both
are given above.

Every branch of every solver is classified by this library's own branch classifier before being
compared, so a match never depends on either side's ordering. On 2 000 poses of the KR 6 R900 each
comparator returned 15 168 branches and every one of them matched:

| comparator | formulation | max joint disagreement (rad) |
|---|---|---|
| `opw_kinematics` | shares this library's ortho-parallel derivation | 0 |
| `ik_geo` | independent — subproblem decomposition | 3.89e-13 |
| `ikfast_kr6r900` | independent — algebraic elimination, code-generated | 1.27e-10 |

**The difference between those three numbers is information, not noise.** A solver sharing the
derivation agrees bit for bit, and a zero there means the two implementations compute the same
expressions. The two that derive the inverse independently agree to machine precision instead,
which is the stronger result: it is evidence that the closed form itself is right rather than that
two copies of one formula behave alike. `ik_geo` is named here by its published crate and release
and is not interchangeable with any other implementation of the same algorithm; no figure produced
against a different implementation of it appears in this section.

## Forward kinematics

<!-- source: docs/benchmarks/raw/2026-08-11/microbench/fk_pinocchio.json -->

Median of 5 repetitions, nanoseconds, captured under the conditions above with no instrumentation
on either side. **The microbenchmarks run on a different robot set from the study**: they use this
repository's own chain fixtures, which include three arms the study excludes for want of a fetched
description, and exclude the four UR arms the study covers. Against Pinocchio's
`framesForwardKinematics`, which computes and stores every intermediate placement:

| robot | Pinocchio | `forward_kinematics` (quaternion) | `forward_kinematics_matrix` | matrix, static chain | matrix / Pinocchio |
|---|---|---|---|---|---|
| UR3e | 328.1 | 424.0 | 266.6 | 273.2 | 0.81× |
| ABB IRB 120 | 325.1 | 441.1 | 267.0 | 241.7 | 0.82× |
| KUKA KR 6 R900 | 325.2 | 442.0 | 266.6 | 241.7 | 0.82× |
| Franka Panda | 377.3 | 487.1 | 311.4 | 274.7 | 0.83× |
| Kinova Jaco2 | 324.9 | 440.3 | 266.5 | 241.8 | 0.82× |
| Fetch | 377.5 | 527.8 | 307.6 | 298.0 | 0.81× |
| Rethink Baxter | 377.5 | 527.8 | 307.2 | 298.2 | 0.81× |
| KUKA LBR Med 14 | 376.9 | 486.3 | 310.7 | 274.9 | 0.82× |
| 3R planar | 163.9 | 190.9 | 133.5 | 127.2 | 0.81× |

**This page previously reported that cartan's forward kinematics is slightly slower than the
reference implementations. That described the quaternion path, and the table it appeared in listed
neither matrix cell.** Both statements below are what this capture measures:

- The quaternion path, `forward_kinematics`, is **1.17× to 1.40× slower** than Pinocchio. It
  returns a quaternion pose, which is what costs it here.
- The matrix path, `forward_kinematics_matrix`, is **0.81× to 0.83× of Pinocchio on every robot in
  the set** — faster, and unusually consistent across chain sizes. On a compile-time-sized chain it
  is 0.73× to 0.83×.

Maximum coefficient of variation across the cells in this table is 1.37%.

<!-- source: docs/benchmarks/raw/2026-08-11/microbench/fk_comparison_benchmarks.json -->

Against KDL's `JntToCart`, in a separate binary that carries no matrix cell:

| robot | KDL | `forward_kinematics` (quaternion) | ratio |
|---|---|---|---|
| UR3e | 345.9 | 428.5 | 1.24× |
| ABB IRB 120 | 341.7 | 447.2 | 1.31× |
| KUKA KR 6 R900 | 342.0 | 440.0 | 1.29× |
| Franka Panda | 397.4 | 484.6 | 1.22× |
| Kinova Jaco2 | 342.2 | 440.0 | 1.29× |
| Fetch | 392.7 | 529.8 | 1.35× |
| Rethink Baxter | 392.4 | 530.0 | 1.35× |
| KUKA LBR Med 14 | 397.2 | 487.2 | 1.23× |
| 3R planar | 184.6 | 164.4 | 0.89× |

No matrix-versus-KDL cell exists, so this page states none. The quaternion cell is common to both
binaries and agrees between them to 1.1% — 424.0 against 428.5 on UR3e — which is what makes the
two tables above comparable at all.

## Jacobian

<!-- source: docs/benchmarks/raw/2026-08-11/microbench/jacobian_comparison_benchmarks.json -->

The like-for-like comparison is **random q → Jacobian**: cartan's `space_jacobian` runs
`forward_kinematics` inside the timed loop, matching KDL's `JntToJac`, which recomputes forward
kinematics internally. The marginal Jacobian, with the pose already computed, is a different
quantity and is reported in its own column rather than as the comparison figure — it applies only
to callers that already hold the pose.

| robot | KDL q→J | cartan q→J | cartan q→J, static | cartan J given FK | KDL / cartan |
|---|---|---|---|---|---|
| UR3e | 1112.9 | 495.9 | 479.0 | 64.3 | 2.24× |
| ABB IRB 120 | 1127.6 | 509.8 | 491.0 | 61.2 | 2.21× |
| KUKA KR 6 R900 | 1123.5 | 509.5 | 490.9 | 61.1 | 2.20× |
| Franka Panda | 1381.0 | 567.2 | 553.9 | 132.0 | 2.43× |
| Kinova Jaco2 | 1123.7 | 509.0 | 491.1 | 61.1 | 2.21× |
| Fetch | 1382.3 | 597.8 | 574.0 | 119.6 | 2.31× |
| Rethink Baxter | 1358.4 | 597.1 | 574.2 | 119.6 | 2.27× |
| KUKA LBR Med 14 | 1382.1 | 563.2 | 554.0 | 132.0 | 2.45× |
| 3R planar | 473.0 | 223.7 | 220.5 | 33.4 | 2.11× |

cartan's analytic product-of-exponentials Jacobian is **2.11× to 2.45× faster than KDL** on the
whole-solve basis, and 2.14× to 2.49× on a compile-time-sized chain. Maximum coefficient of
variation across these cells is 1.14%. This claim survived the audit that rebuilt the rest of this
page: it carries none of that audit's defects, and the figures here are a fresh reproduction of it
rather than the earlier capture's.

## Lie group operations

<!-- source: docs/benchmarks/raw/2026-08-11/microbench/lie_group_benchmarks.json -->

Per-operation microbenchmarks in nanoseconds, median of 5 repetitions. These are internal to this
library — no cross-library counterpart is measured — and the harness cycles varied inputs so the
optimizer cannot fold an operation down to a store.

| group | exp | log | compose | inverse | adjoint | coadjoint |
|---|---|---|---|---|---|---|
| SO(2) | 24.1 | 15.5 | 9.5 | 8.1 | – | – |
| SE(2) | 35.6 | 34.2 | 33.0 | 11.2 | 2.7 | – |
| SO(3) | 32.5 | 20.9 | 7.4 | 5.7 | 3.2 | 3.2 |
| SE(3) | 70.1 | 75.1 | 15.9 | 14.1 | 25.7 | 48.3 |

Maximum coefficient of variation across these cells is 0.21%.

## Reproducing

What this repository offers is **rebuilding every published table from the shipped records**, with
`python3` and nothing else:

```
python3 tools/bench_study_report.py --root docs/benchmarks/raw/2026-08-11/ladder \
    --table c --from aggregates --out table_c.md
```

That is a claim about rebuilding the published tables, not about rerunning the study. **Rerunning
it additionally requires the comparator libraries, and obtaining those is the reader's own.** This
project does not fetch, vendor or patch a library it does not own: `pinocchio`, `orocos_kdl`,
`trac_ik`, `nlopt` and `lapack` are discovered on the machine if they are there, and each
comparison is omitted with a warning when its dependency is absent.

The capture was configured and run with:

```
cmake -S . -B build/bench -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DCARTAN_BUILD_BENCHMARKS=ON -DCARTAN_BUILD_URDF=ON -DCARTAN_BUILD_ARGMIN=ON \
    -DCARTAN_FETCH_BENCHMARK_DEPS=ON -DCARTAN_TRAC_IK_SOURCE_DIR=<checkout> \
    -DCMAKE_PREFIX_PATH=<pinocchio environment>
build/bench/benchmarks/ik_study_capture --table c --robot <robot> --limits description \
    --stratum <stratum> --targets 500 --all-budgets --out-dir <records> --sidecar-dir <sidecar>
```

The full configure line, every per-invocation run command, and the machine state read back from
the kernel are in `docs/benchmarks/raw/2026-08-11/ladder/environment.json`. The microbenchmarks
were run with `--benchmark_repetitions=5 --benchmark_report_aggregates_only=true`.

The shipped records are per-cell aggregates. The per-target rows — 6.9 million of them, one per
solve — are a release asset rather than a file in this repository; at capture time both tiers were
rebuilt independently and cross-checked, and 64 800 published figures per table agreed between
them. `docs/benchmarks/README.md` records what ships, at what resolution, and what is known to be
wrong with it.
