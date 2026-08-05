# ik-geo foreign-function shim (for the ik-geo comparison benchmark)

`ikgeo_comparison_benchmarks.cpp` cross-checks cartan's `opw_6r_solver` against
**ik-geo** (Elias & Wen, RPI) — an independent geometric subproblem-decomposition
IK solver. ik-geo's *C++* port's spherical solver is broken for roll-pitch-roll
wrists (it returns least-squares garbage; see
`.planning/2026-07-07-ikgeo-witness-incompatible.md`), so the benchmark drives a
Rust implementation of the algorithm through this shim.

Everything in this directory is cartan's own code. The implementation it binds is
the published `ik-geo` crate at **0.1.2**
(<https://crates.io/crates/ik-geo>, repository `Verdant-Evolution/ik-geo-rust`),
an ordinary registry dependency resolved through the crate's **public** `robot`
module — no source is copied into this tree and no library is patched.
`Cargo.lock` is committed, so the resolution is fixed rather than re-resolved per
machine, and `benchmarks/CMakeLists.txt` builds with `--locked`.

This is a different codebase by a different author than the implementation the
earlier shim bound. Any figure this benchmark produces describes ik-geo 0.1.2 and
must name it; no number measured against the earlier implementation carries over.

`cargo` is invoked automatically by `benchmarks/CMakeLists.txt`; the resulting
`libikgeo_ffi.a` is linked only into the one benchmark. Pass
`-DCARTAN_IKGEO_FFI_LIB=<path>` to link a prebuilt shim instead.

## Exported entry points
- `ikgeo_spherical_two_parallel(h[18] col-major, p[21] col-major, r[9] row-major, t[3], out_q, out_is_ls) -> count`
- `ikgeo_irb6640(r[9] row-major, t[3], out_q, out_is_ls) -> count`

Each solution carries its least-squares flag, so a degenerate solve is never
reported as exact.

## Conventions (must match the caller)
- Rotation `r` is **row-major** (`Matrix3::from_row_slice`); nalgebra and ik-geo's
  own test CSVs are column-major, so transpose accordingly if reusing those.
- `h`/`p` are **column-major** (`Matrix3x6` / `Matrix3x7` `from_column_slice`).
- ik-geo hardcodes the tool offset `R_6T = I`; a non-identity home rotation is
  reconciled caller-side via `r = R_target * R_home^{-1}`.

## Building it by hand
The crate's optimizer dependency vendors a CMake project whose declared minimum
predates CMake 4.0, which refuses it. The build raises the floor for that
subprocess only:

```
CMAKE_POLICY_VERSION_MINIMUM=3.5 cargo build --release --locked
```
