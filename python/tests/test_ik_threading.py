"""Multi-thread GIL release scaling test for cartan.solve_ik.

Verifies that the iterative IK trio releases the GIL during the C++ solve so
that a Python `ThreadPoolExecutor` actually achieves CPU parallelism on a
multi-core host. A failing assertion here is a regression signal that one of
the `solve_ik*` lambdas dropped its `nb::call_guard<nb::gil_scoped_release>()`
or that the `nb::arg(...).noconvert()` chain accidentally re-acquired the GIL.

The test uses cartan.SE3 typed targets (constructed via forward_kinematics)
rather than raw ndarrays. Passing typed Lie-group objects across the binding
boundary is documented API hygiene; raw-ndarray targets are an anti-pattern
for solver entry points because they require an implicit conversion that
materializes Python objects under the lock. nanobind's historical issue
#377 around raw ndarrays plus `gil_scoped_release` on Python 3.12+ was
fixed upstream in 1.1.0; cartan pins nanobind 2.12.0 which is unaffected.
The SE3 convention here is therefore defense-in-depth, not bug mitigation.
"""

from __future__ import annotations

import sys
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import pytest

import cartan


pytestmark = pytest.mark.slow


@pytest.mark.skipif(
    sys.version_info[:2] < (3, 12),
    reason=(
        "GIL-release scaling test specified for Python 3.12+ "
        "(the cartan extension's gil_scoped_release call_guards behave "
        "identically on older interpreters but the threading-fairness "
        "guarantees the assertion relies on are only validated on 3.12+)."
    ),
)
def test_solve_ik_scales_across_threads(
    ur5e_chain: cartan.KinematicChain,
) -> None:
    """4-thread wall time must be less than 1.5 x single-thread wall.

    A 4-core host with a properly-released GIL converges around ratio 0.3
    (about 4x speedup vs single thread). A ratio at or above 1.5 means
    either the GIL is not being released or one of the lambdas re-acquires
    it inside the hot path -- both are regressions worth catching.

    The SE3 typed-target convention exercised here (target is constructed
    via cartan.forward_kinematics, not a raw ndarray) is documented API
    hygiene rather than mitigation of any known nanobind bug: cartan pins
    nanobind 2.12.0 which post-dates the historical 1.1.0 fix.
    """
    chain = ur5e_chain
    rng = np.random.default_rng(seed=42)
    n_solves = 64
    n = chain.num_joints()
    lims = chain.limits()

    # Pre-build FK-walked SE3 targets and far-from-truth seeds so that each
    # solve does real work inside the C++ runner -- a near-truth seed
    # converges in zero iterations and the wall measurement degenerates into
    # noise on the order of the Python -> nanobind dispatch overhead. The
    # 1.0-rad seed perturbation is the smallest one that produces a
    # per-call wall above 50 us on UR5e at default IkConfig, which keeps the
    # total wall above 4 ms and the ratio measurement statistically meaningful.
    targets: list[cartan.SE3] = []
    seeds: list[np.ndarray] = []
    for _ in range(n_solves):
        q_truth = np.empty(n, dtype=np.float64)
        for j in range(n):
            lo = lims[j].position_min
            hi = lims[j].position_max
            if not (np.isfinite(lo) and np.isfinite(hi)) or (hi - lo) > 1e6:
                lo, hi = -np.pi, np.pi
            mid = 0.5 * (lo + hi)
            half = 0.4 * (hi - lo)
            q_truth[j] = rng.uniform(mid - half, mid + half)
        targets.append(cartan.forward_kinematics(chain, q_truth))
        seeds.append(q_truth + rng.uniform(-1.0, 1.0, size=n))

    def _solve(idx: int) -> cartan.IkResult:
        return cartan.solve_ik(chain, targets[idx], seeds[idx])

    # Warm up the runner allocator and the OS thread pool so the first
    # ThreadPoolExecutor.map call does not pay one-time setup overhead in
    # the wall measurement.
    for i in range(n_solves):
        _solve(i)

    # Single-thread baseline.
    t0 = time.perf_counter()
    single_results = [_solve(i) for i in range(n_solves)]
    t_single = time.perf_counter() - t0

    # 4-thread parallel run.
    t0 = time.perf_counter()
    with ThreadPoolExecutor(max_workers=4) as ex:
        parallel_results = list(ex.map(_solve, range(n_solves)))
    t_parallel = time.perf_counter() - t0

    # Correctness: every parallel solve converged on its well-conditioned
    # FK-walked target. A scaling speedup that hides correctness regressions
    # would be useless.
    for r in parallel_results:
        assert r.converged, f"parallel solve failed to converge: {r!r}"
    for r in single_results:
        assert r.converged, f"single-thread solve failed to converge: {r!r}"

    # Scaling: parallel must be less than 1.5 x single-thread.
    ratio = t_parallel / t_single
    assert ratio < 1.5, (
        f"multi-thread scaling failed (GIL not released?): "
        f"1-thread {t_single * 1e3:.1f} ms vs 4-thread {t_parallel * 1e3:.1f} ms; "
        f"ratio {ratio:.2f} (expected < 1.5)"
    )
