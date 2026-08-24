use ik_geo::nalgebra::{Matrix3, Matrix3x6, OMatrix, Vector3, U3, U7};
use ik_geo::robot::{irb6640, spherical_two_parallel, IKSolver, Robot};

/// The factory's point-column type. nalgebra ships no 3x7 alias, and this
/// spelling is the same type the crate names in its own signature.
type Matrix3x7 = OMatrix<f64, U3, U7>;

/// Writes at most eight solutions, joint-major, and their least-squares flags.
/// A degenerate solve must stay distinguishable from an exact one, so the flag
/// travels with every solution rather than being dropped.
fn emit(robot: &Robot, rm: Matrix3<f64>, tv: Vector3<f64>,
        out_q: *mut f64, out_is_ls: *mut u8) -> usize {
    let solutions = robot.ik(rm, tv);
    let n = solutions.len().min(8);
    for (i, (q, is_ls)) in solutions.iter().take(n).enumerate() {
        for j in 0..6 {
            unsafe { *out_q.add(i * 6 + j) = q[j]; }
        }
        unsafe { *out_is_ls.add(i) = u8::from(*is_ls); }
    }
    n
}

/// r: 9 doubles, ROW-major 3x3. t: 3 doubles. out_q: >= 8*6 doubles.
/// out_is_ls: >= 8 bytes. Returns solution count (<= 8).
#[no_mangle]
pub extern "C" fn ikgeo_irb6640(r: *const f64, t: *const f64,
                                out_q: *mut f64, out_is_ls: *mut u8) -> usize {
    let rs = unsafe { std::slice::from_raw_parts(r, 9) };
    let ts = unsafe { std::slice::from_raw_parts(t, 3) };
    let rm = Matrix3::from_row_slice(rs);
    let tv = Vector3::new(ts[0], ts[1], ts[2]);
    emit(&irb6640(), rm, tv, out_q, out_is_ls)
}

/// h: 18 doubles COLUMN-major (3x6). p: 21 doubles COLUMN-major (3x7).
/// r: 9 doubles ROW-major (3x3). t: 3 doubles. Returns solution count (<= 8).
#[no_mangle]
pub extern "C" fn ikgeo_spherical_two_parallel(h: *const f64, p: *const f64,
        r: *const f64, t: *const f64, out_q: *mut f64, out_is_ls: *mut u8) -> usize {
    let hs = unsafe { std::slice::from_raw_parts(h, 18) };
    let ps = unsafe { std::slice::from_raw_parts(p, 21) };
    let rs = unsafe { std::slice::from_raw_parts(r, 9) };
    let ts = unsafe { std::slice::from_raw_parts(t, 3) };
    let robot = spherical_two_parallel(
        Matrix3x6::from_column_slice(hs), Matrix3x7::from_column_slice(ps));
    let rm = Matrix3::from_row_slice(rs);
    let tv = Vector3::new(ts[0], ts[1], ts[2]);
    emit(&robot, rm, tv, out_q, out_is_ls)
}
