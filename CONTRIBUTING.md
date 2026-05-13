# Contributing to Cartan

Thanks for your interest in contributing. Cartan is an academic / research
library aimed at robotics teaching and research; contributions of all
sizes -- documentation fixes, bug reports, code improvements, new
features -- are welcome.

## Quick Links

- [Filing an Issue](#filing-an-issue)
- [Submitting a Pull Request](#submitting-a-pull-request)
- [Building and Testing](#building-and-testing)
- [Coding Conventions](#coding-conventions)
- [Commit Message Format](#commit-message-format)
- [Branching Model](#branching-model)
- [License](#license)

## Filing an Issue

- **Bug reports.** Include a minimal reproducer (a short program that
  triggers the issue), the compiler + version (`g++ --version` or
  equivalent), the CMake version, and the platform. Numerical-issue
  reports should include the input pose / joint configuration that
  triggers the misbehavior plus the observed and expected output.
- **Feature requests.** Describe the use case first, the proposed API
  shape second. Concrete user stories make it easier to evaluate the
  proposal against the library's scope.
- **Questions.** Open an issue tagged "question" -- there is no Discord
  or Slack to fragment discussion.

If you're not sure whether something is a bug or expected behavior, an
issue is the right place to ask.

## Submitting a Pull Request

1. **Fork** the repository and create a feature branch from `develop`
   (or from the active `milestone/<version>` branch if you're
   contributing to in-progress milestone work -- ask first if unsure).
2. **Build and test** locally. Cartan's full ctest suite should pass
   (`ctest --output-on-failure` in your build directory) before you
   submit. New features need new tests; new tests need to pass.
3. **Match the coding conventions.** See [Coding Conventions](#coding-conventions)
   below. PRs that diverge stylistically will be asked to converge before
   review; this saves everyone time.
4. **Write a clear PR description.** What problem does this solve? What
   approach did you take? What alternatives did you consider? Link
   relevant issues.
5. **One logical change per PR.** Don't bundle unrelated cleanups with
   feature work. If you spot an unrelated issue while working on
   something else, file it separately.

## Building and Testing

See [docs/getting-started.md](docs/getting-started.md) for the install
and build instructions for end users. For contributors:

```bash
# Configure the dev preset (Debug + tests + sanitizers enabled by default
# on at least one CI configuration).
cmake --preset dev

# Build everything.
cmake --build build/dev -j$(nproc)

# Run the test suite.
ctest --test-dir build/dev --output-on-failure
```

The `dev-full` preset also builds examples, fuzz, and property tests.
Benchmarks and the pinocchio comparison suite are gated behind separate
CMake flags (`CARTAN_BUILD_BENCHMARKS`, `CARTAN_BUILD_PINOCCHIO`); they
need their respective external dependencies installed.

## Coding Conventions

Cartan targets idiomatic C++23. The conventions below carry over from
internal codebase rules and apply to all public contributions.

### Language and style

- **C++23** -- use modern features where they improve clarity:
  `concepts`, `std::expected`, `std::ranges`, `if constexpr`,
  designated initializers, etc. Don't reach for C++23 features purely
  for novelty; reach for them where they make the code clearer.
- **Eigen is the only required dependency.** NLopt, argmin/nablapp,
  pinocchio, and TRAC-IK are optional and gated behind CMake feature
  flags.
- **Naming:** lowercase types and functions (`so3`, `kinematic_chain`,
  `forward_kinematics`); member variable prefix `m_` (`m_omega`,
  `m_chain`); template parameters in `CamelCase` (`Scalar`, `Joints`).
- **Braces:** Allman style (opening brace on its own line for
  functions and classes; same-line braces for control flow are
  acceptable). 4-space indentation, no tabs.
- **Return types:** prefer the traditional form (`T func()`) over
  trailing return types (`auto func() -> T`) unless the trailing form
  is required by templates or concepts.
- **File / class size guidance:** classes around 100 NLOC (200 max
  ideal); functions 5-15 lines with delegation; files match class
  length. Readability overrides these when splitting hurts comprehension.

### Headers

- **Header guards** in the form `HPP_GUARD_CARTAN_<MODULE>_<FILE>_H`.
  No `#pragma once`.
- **No matching closing-namespace comment** (`// namespace cartan` is
  noise after the closing brace; just close the brace).
- **No matching `#endif` comment** for the include guard.

### Include ordering

Includes are grouped into three sections separated by a single blank
line:

1. Internal project includes (`#include "..."`).
2. Third-party library includes (`#include <eigen/...>`,
   `#include <catch2/...>`, etc.).
3. Standard library includes (`#include <vector>`, `#include <expected>`).

Within each section, group by folder location with a blank line between
folder groups. Within each folder group, sort by line length first, then
alphabetically.

### Documentation comments

Every public class, function, template, and concept has a doc-comment
naming its purpose, parameters, return value, and edge cases worth
flagging. Keep doc-comments at the why level for non-obvious behavior;
don't restate what well-named identifiers already communicate.

### Testing

- **Unit tests** via Catch2: cover every public API path.
- **Property-based tests** via RapidCheck: for numerical types
  (Lie groups, FK, Jacobians), property tests catch what handpicked
  unit tests miss.
- **Compile-fail tests** for frame-tag safety: ensure mis-composed
  transforms are rejected at compile time.
- **Benchmark coverage** is encouraged for hot-path changes (Lie
  algebra, FK, IK) but not required for every PR.

## Commit Message Format

```
{Prefix}: {summary sentence in present-tense imperative}.

- {what was done; one bullet per logical item}
- {another item if applicable}
```

Allowed prefixes:

| Prefix         | When to use |
|----------------|-------------|
| `Feature:`     | New user-visible capability or API surface. |
| `Fix:`         | Bug fix or correctness-affecting change. |
| `Refactor:`    | Internal reorganization with no user-visible behavior change. |
| `Docs:`        | Documentation-only changes (README, docs/, doc-comments). |
| `Examples:`    | Changes to `examples/` only. |
| `Optimization:` | Performance change with no API or correctness impact. |
| `WIP:`         | Work-in-progress commits whose code does not yet compile or pass tests. Use sparingly; squash before merging. |

The summary line is brief and descriptive (under 72 characters where
possible). The bullet list expands on the what. Single-item commits may
omit the bullet list.

**One logical change per commit.** A PR with one tightly-scoped change
gets reviewed and merged faster than a PR bundling multiple changes.

## Branching Model

```
master   <- develop   <- milestone/<version>
(releases)  (integration)  (active work)
```

- **master** holds tagged releases.
- **develop** is the integration branch; milestone branches merge into
  develop when each milestone ships.
- **milestone/v0.X.0** branches host active work on a specific
  milestone. External contributors should normally branch from
  `develop`; if you're contributing to in-progress milestone work,
  ask first via an issue or PR comment.
- Merge direction is always milestone -> develop -> master. Reverse
  merges (master -> develop, etc.) do not happen.

## License

Cartan is released under the [Apache License 2.0](LICENSE). By
submitting a contribution you agree that your contribution is licensed
under the same terms. There is no separate Contributor License Agreement
(CLA); the Apache 2.0 grant in the standard "Submission of Contributions"
clause (Section 5) applies.
