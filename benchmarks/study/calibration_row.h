#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_CALIBRATION_ROW_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_CALIBRATION_ROW_H

/// @file calibration_row.h
/// @brief What one solver had to be asked for, and what that delivered.
///
/// The requested tolerance and the achieved accuracy are both on the row
/// because they are different numbers whose ratio differs per solver, and the
/// number of solves the achieved figure was measured over is on it because an
/// accuracy over a handful of targets is not the same claim as one over two
/// hundred.
///
/// `converged` is what keeps the two apart when the search fails: a row whose
/// flag is false carries the closest accuracy the solver reached rather than
/// the target it was asked for, and no reader of the table may treat it as met.
///
/// The provenance is on the row because the bounds a calibration ran under
/// change what tolerance delivers an accuracy: a figure searched against
/// invented bounds does not transfer to a run under the manufacturer's, and a
/// table without the column cannot even say which one it holds.

#include <cmath>
#include <string>
#include <format>
#include <string_view>

namespace cartan::bench
{

constexpr double k_accuracy_band = 0.1;

struct calibration_row
{
    std::string table;
    std::string robot;
    std::string limits_provenance;
    std::string solver;
    double accuracy_target;
    double calibrated_tolerance;
    double achieved_median;
    double achieved_p95;
    int n;
    bool converged;
};

/// Two conditions, and the second is the one a close figure hides. An achieved
/// median over a minority of the calibration subset describes the targets the
/// solver happened to solve rather than the subset the calibration is defined
/// over: a single accepted solve out of a hundred can land inside the band by
/// luck, and reading that as a calibrated accuracy is the claim this whole mode
/// exists to stop making.
inline bool target_reached(double achieved_median, int n, double target, int subset)
{
    return 2 * n > subset && std::abs(achieved_median - target) <= k_accuracy_band * target;
}

inline std::string_view calibration_header()
{
    return "table,robot,limits_provenance,solver,accuracy_target,calibrated_tolerance,"
           "achieved_median,achieved_p95,n,converged";
}

/// An accuracy measured over nothing is written empty rather than as a number.
/// A zero there reads as a perfect result rather than as an absent one, and a
/// solver that met its requested tolerance on no target has no distribution.
inline std::string achieved_field(double value, int n)
{
    return n == 0 ? std::string{} : std::format("{:.17g}", value);
}

inline std::string calibration_csv_row(const calibration_row& row)
{
    return std::format("{},{},{},{},{:.17g},{:.17g},{},{},{},{:d}", row.table, row.robot,
        row.limits_provenance, row.solver, row.accuracy_target, row.calibrated_tolerance,
        achieved_field(row.achieved_median, row.n), achieved_field(row.achieved_p95, row.n), row.n,
        row.converged);
}

}

#endif
