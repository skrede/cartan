#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_CELL_AGGREGATE_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_CELL_AGGREGATE_H

/// @file cell_aggregate.h
/// @brief The in-repository tier: one row summarizing a cell's targets.
///
/// This tier is what a reader who clones gets, so it has to be sufficient on
/// its own to rebuild every published figure. Every statistic here is over the
/// cell's whole target set, never over the targets a solver survived.

#include "target_record.h"

#include <cmath>
#include <format>
#include <limits>
#include <string>
#include <vector>
#include <cstddef>
#include <numeric>
#include <utility>
#include <algorithm>
#include <string_view>

namespace cartan::bench
{

namespace detail
{

/// Nearest-rank order statistic with a non-finite error sorted to the worst end
/// rather than dropped. The analysis script computes these by the same rule,
/// which is what makes the two output tiers agree exactly rather than closely.
inline double order_statistic(std::vector<double> values, double quantile)
{
    if (values.empty())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    for (auto& value : values)
    {
        if (!std::isfinite(value))
        {
            value = std::numeric_limits<double>::infinity();
        }
    }
    std::sort(values.begin(), values.end());
    const auto rank = static_cast<std::size_t>(quantile * static_cast<double>(values.size()));
    return values[std::min(rank, values.size() - 1)];
}

inline double variation(const std::vector<double>& values)
{
    if (values.size() < 2)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const auto count = static_cast<double>(values.size());
    const double mean = std::accumulate(values.begin(), values.end(), 0.0) / count;
    double squares = 0.0;
    for (const double value : values)
    {
        squares += (value - mean) * (value - mean);
    }
    return std::sqrt(squares / (count - 1.0)) / mean;
}

}

struct cell_accumulator
{
    std::string identity;
    double tolerance;
    std::vector<double> pos_err;
    std::vector<double> ori_err;
    std::vector<double> wall_ns;
    std::vector<double> fk_evals;
    std::vector<double> jac_evals;
    int accepted;
    int self_reported;
    int false_success;
    int false_failure;

    cell_accumulator(std::string cell, double gate)
        : identity(std::move(cell))
        , tolerance(gate)
        , pos_err()
        , ori_err()
        , wall_ns()
        , fk_evals()
        , jac_evals()
        , accepted(0)
        , self_reported(0)
        , false_success(0)
        , false_failure(0)
    {
    }

    void add(const target_record& row)
    {
        pos_err.push_back(row.adjudication.pos_err);
        ori_err.push_back(row.adjudication.ori_err);
        wall_ns.push_back(static_cast<double>(row.wall_ns));
        fk_evals.push_back(static_cast<double>(row.counts.fk));
        jac_evals.push_back(static_cast<double>(row.counts.jac));
        accepted += row.adjudication.accepted ? 1 : 0;
        self_reported += row.adjudication.self_reported ? 1 : 0;
        false_success += (row.adjudication.self_reported && !row.adjudication.accepted) ? 1 : 0;
        false_failure += (!row.adjudication.self_reported && row.adjudication.accepted) ? 1 : 0;
    }
};

inline std::string cell_identity(const target_record& row)
{
    return std::format("{},{},{},{},{},{},{},{}",
        csv_field(row.table), csv_field(row.robot), csv_field(row.provenance),
        csv_field(row.stratum), csv_field(row.solver), row.budget_index, row.budget_value,
        csv_field(row.budget_axis));
}

inline std::string_view cell_header()
{
    return "table,robot,limits_provenance,stratum,solver,budget_index,budget_value,budget_axis,"
           "n_targets,n_accepted,n_self_reported,n_false_success,n_false_failure,accept_rate,"
           "pos_err_median_m,pos_err_p95_m,pos_err_p99_m,ori_err_median_rad,fk_evals_median,"
           "jac_evals_median,wall_ns_median,wall_ns_cv,solver_tolerance";
}

inline std::string cell_row(const cell_accumulator& cell)
{
    const auto targets = static_cast<int>(cell.pos_err.size());
    return std::format("{},{},{},{},{},{},{:.17g},{:.17g},{:.17g},{:.17g},{:.17g},{:.17g},"
                       "{:.17g},{:.17g},{:.17g},{:.17g}",
        cell.identity, targets, cell.accepted, cell.self_reported, cell.false_success,
        cell.false_failure,
        targets == 0 ? 0.0 : static_cast<double>(cell.accepted) / static_cast<double>(targets),
        detail::order_statistic(cell.pos_err, 0.5), detail::order_statistic(cell.pos_err, 0.95),
        detail::order_statistic(cell.pos_err, 0.99), detail::order_statistic(cell.ori_err, 0.5),
        detail::order_statistic(cell.fk_evals, 0.5), detail::order_statistic(cell.jac_evals, 0.5),
        detail::order_statistic(cell.wall_ns, 0.5), detail::variation(cell.wall_ns),
        cell.tolerance);
}

}

#endif
