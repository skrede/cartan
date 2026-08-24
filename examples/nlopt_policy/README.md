# Writing a solve policy around a third-party optimizer

This directory is an example, not library surface. cartan itself builds against
Eigen and nothing else; the NLopt dependency is acquired here, by this
directory's own option, and never by a project-wide one.

## What it shows

A `solve_policy` is the extension point cartan drives its IK runners through. A
policy written over cartan's own primitives is steppable by construction, so it
teaches nothing about the hard case. NLopt is the hard case: `nlopt::opt` only
offers `optimize()`, which runs to completion and hands back a result.

`nlopt_slsqp` and `nlopt_bobyqa` bridge that gap by bounding each call with
`set_maxeval` and re-entering from the previous iterate, so one cartan work unit
is one bounded `optimize()` call. A caller with a deadline spends work units
until it runs out instead of surrendering control to the optimizer. That is the
part worth copying when wrapping any optimizer that owns its own loop.

## What it deliberately does not claim

These policies allocate — through `nlopt::opt`, through the `std::vector<double>`
the C API requires, and through the restart RNG — so they cannot appear in
cartan's real-time surface. Their step granularity is a bounded `optimize()`
call rather than a single iteration, so worst-case latency follows the evaluation
budget rather than a fixed instruction count.

They also live in `namespace cartan_examples`, not `namespace cartan`. A name
that exists only when an optional dependency is present is exactly what the
library's own name-invariance gate forbids, and an outside author adding a policy
would be in their own namespace anyway.

## Building

```sh
cmake -S . -B build -DCARTAN_BUILD_EXAMPLES=ON -DCARTAN_EXAMPLE_NLOPT_POLICY=ON
cmake --build build --target nlopt_policy_example
```

NLopt is taken from an installed package when one is present and fetched
otherwise. The interface target `cartan_examples::nlopt_policy` carries the
include path, the dependency and `CARTAN_EXAMPLE_HAS_NLOPT`; the benchmark,
profiling and policy-test targets that measure these policies key on it.
