use linear_subproblem_solutions_rust::inverse_kinematics::hardcoded::irb6640;
use nalgebra::{Matrix3, Vector3};

/// r: 9 doubles, ROW-major 3x3. t: 3 doubles. out_q: >= 8*6 doubles.
/// out_is_ls: >= 8 bytes. Returns solution count (<= 8).
#[no_mangle]
pub extern "C" fn ikgeo_irb6640(r: *const f64, t: *const f64,
                                out_q: *mut f64, out_is_ls: *mut u8) -> usize {
    let rs = unsafe { std::slice::from_raw_parts(r, 9) };
    let ts = unsafe { std::slice::from_raw_parts(t, 3) };
    let rm = Matrix3::from_row_slice(rs);
    let tv = Vector3::new(ts[0], ts[1], ts[2]);
    let (qs, isls) = irb6640(&rm, &tv);
    let n = qs.len().min(8);
    for i in 0..n {
        for j in 0..6 {
            unsafe { *out_q.add(i * 6 + j) = qs[i][j]; }
        }
        unsafe { *out_is_ls.add(i) = if isls[i] { 1 } else { 0 }; }
    }
    n
}

use linear_subproblem_solutions_rust::inverse_kinematics::spherical_two_parallel;
use linear_subproblem_solutions_rust::inverse_kinematics::auxiliary::Kinematics;
use nalgebra::Matrix3x6;
use linear_subproblem_solutions_rust::inverse_kinematics::auxiliary::Matrix3x7;

/// h: 18 doubles COLUMN-major (3x6). p: 21 doubles COLUMN-major (3x7).
/// r: 9 doubles ROW-major (3x3). t: 3 doubles. Returns solution count (<= 8).
#[no_mangle]
pub extern "C" fn ikgeo_spherical_two_parallel(h: *const f64, p: *const f64,
        r: *const f64, t: *const f64, out_q: *mut f64, out_is_ls: *mut u8) -> usize {
    let hs = unsafe { std::slice::from_raw_parts(h, 18) };
    let ps = unsafe { std::slice::from_raw_parts(p, 21) };
    let rs = unsafe { std::slice::from_raw_parts(r, 9) };
    let ts = unsafe { std::slice::from_raw_parts(t, 3) };
    let mut kin = Kinematics::<6, 7>::new();
    kin.h = Matrix3x6::from_column_slice(hs);
    kin.p = Matrix3x7::from_column_slice(ps);
    let rm = Matrix3::from_row_slice(rs);
    let tv = Vector3::new(ts[0], ts[1], ts[2]);
    let (qs, isls) = spherical_two_parallel(&rm, &tv, &kin);
    let n = qs.len().min(8);
    for i in 0..n {
        for j in 0..6 { unsafe { *out_q.add(i * 6 + j) = qs[i][j]; } }
        unsafe { *out_is_ls.add(i) = if isls[i] { 1 } else { 0 }; }
    }
    n
}
