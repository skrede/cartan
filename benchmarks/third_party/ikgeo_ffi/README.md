# ik-geo FFI shim (for the ik-geo comparison benchmark)

`ikgeo_comparison_benchmarks.cpp` cross-checks cartan's `opw_6r_solver` against
**ik-geo** (Elias & Wen, RPI) — an independent geometric subproblem-decomposition
IK solver. Because ik-geo's *C++* port's spherical solver is broken for
roll-pitch-roll wrists (it returns least-squares garbage; see
`.planning/2026-07-07-ikgeo-witness-incompatible.md`), the benchmark drives
ik-geo's **Rust** crate through this small C FFI shim.

`cargo` is invoked automatically by `benchmarks/CMakeLists.txt`; the resulting
`libikgeo_ffi.a` is linked only into the one benchmark. Pass
`-DCARTAN_IKGEO_FFI_LIB=<path>` to use a prebuilt shim instead.

## Contents
- `Cargo.toml`, `src/lib.rs` — the FFI shim (this project). Exposes:
  - `ikgeo_spherical_two_parallel(h[18] col-major, p[21] col-major, r[9] row-major, t[3], out_q, out_is_ls) -> count`
  - `ikgeo_irb6640(r[9] row-major, t[3], out_q, out_is_ls) -> count`
- `ik-geo-rust/` — vendored ik-geo Rust source, `rpiRobotics/ik-geo` @ `a3a1675`
  (BSD-3-Clause, `LICENSE` retained). One local patch: `inverse_kinematics`'s
  `mod auxiliary` changed from `pub(crate)` to `pub` so the `Kinematics` struct
  can be constructed from the shim.

## Conventions (must match the caller)
- Rotation `r` is **row-major** (`Matrix3::from_row_slice`); nalgebra and ik-geo's
  own test CSVs are column-major, so transpose accordingly if reusing those.
- `h`/`p` are **column-major** (`Matrix3x6` / `Matrix3x7` `from_column_slice`).
- ik-geo hardcodes the tool offset `R_6T = I`; a non-identity home rotation is
  reconciled caller-side via `r = R_target * R_home^{-1}`.

## Regenerate the vendored source
```
git clone https://github.com/rpiRobotics/ik-geo.git
cd ik-geo && git checkout a3a1675
sed -i 's/^pub(crate) mod auxiliary;/pub mod auxiliary;/' \
    rust/src/inverse_kinematics/mod.rs
# copy rust/{src,Cargo.toml,build.rs} into ik-geo-rust/
```
