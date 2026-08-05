#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_TARGET_RECORD_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_TARGET_RECORD_H

/// @file target_record.h
/// @brief One adjudicated target, and the columns it is written under.
///
/// The column names here are the column names the analysis script reads. A
/// rename on either side is a silent break: the script would still run and
/// would rebuild a table missing whatever it could not find.
///
/// `budget_requested` is what the rung asked for and `budget_value` is what the
/// solve achieved, in the unit named by `budget_axis`. They are two columns
/// because they are two numbers, and the previously published study reported
/// the first as though it were the second.

#include "verdict.h"
#include "counting_chain.h"

#include <format>
#include <string>
#include <cstdint>
#include <string_view>

namespace cartan::bench
{

struct target_record
{
    std::string_view table;
    std::string_view robot;
    std::string_view provenance;
    std::string_view stratum;
    std::string_view solver;
    std::string_view budget_axis;
    int budget_index;
    std::int64_t budget_requested;
    std::int64_t budget_value;
    int target_id;
    int seed_id;
    int iterations;
    double solver_tolerance;
    std::int64_t wall_ns;
    kernel_counts counts;
    verdict adjudication;
    bool kernel_countable;
    bool success_rate_reportable;
};

/// A count the harness did not take is written empty, never zero. Zero would
/// place the participant at the origin of the kernel-evaluation axis, which is a
/// stronger and wronger claim than an acknowledged gap -- and no count may ever
/// be derived from a duration to fill it.
inline std::string count_field(std::int64_t count, bool countable)
{
    return countable ? std::to_string(count) : std::string{};
}

inline std::string_view target_record_header()
{
    return "table,robot,limits_provenance,stratum,solver,budget_index,budget_requested,"
           "budget_axis,budget_value,target_id,seed_id,self_reported,accepted,pose_ok,limits_ok,"
           "pos_err_m,ori_err_rad,worst_limit_violation_rad,fk_evals,jac_evals,iterations,"
           "solver_tolerance,wall_ns";
}

/// A field that could carry a separator is quoted and its own quotes doubled,
/// per RFC 4180 section 2.
inline std::string csv_field(std::string_view value)
{
    if (value.find_first_of(",\"\n\r") == std::string_view::npos)
    {
        return std::string{value};
    }
    std::string quoted{'"'};
    for (const char character : value)
    {
        if (character == '"')
        {
            quoted.push_back('"');
        }
        quoted.push_back(character);
    }
    quoted.push_back('"');
    return quoted;
}

inline std::string csv_row(const target_record& row)
{
    const auto& seen = row.adjudication;
    return std::format(
        "{},{},{},{},{},{},{},{},{},{},{},{:d},{:d},{:d},{:d},{:.17g},{:.17g},{:.17g},{},{},{},"
        "{:.17g},{}",
        csv_field(row.table), csv_field(row.robot), csv_field(row.provenance),
        csv_field(row.stratum), csv_field(row.solver), row.budget_index, row.budget_requested,
        csv_field(row.budget_axis), row.budget_value, row.target_id, row.seed_id,
        seen.self_reported, seen.accepted, seen.pose_ok, seen.limits_ok, seen.pos_err,
        seen.ori_err, seen.worst_limit_violation,
        count_field(row.counts.fk, row.kernel_countable),
        count_field(row.counts.jac, row.kernel_countable),
        count_field(row.iterations, row.kernel_countable), row.solver_tolerance, row.wall_ns);
}

}

#endif
