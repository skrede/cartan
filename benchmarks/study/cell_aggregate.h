#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_CELL_AGGREGATE_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_CELL_AGGREGATE_H

/// @file cell_aggregate.h
/// @brief The in-repository tier: one row summarizing a cell's targets.
///
/// This tier is what a reader who clones gets, so it has to be sufficient on
/// its own to rebuild every published figure. Every statistic here is over the
/// cell's whole target set, never over the targets a solver survived -- with one
/// named exception. `pos_err_median_accepted_m` is the accuracy of the solves
/// the harness accepted, which is what the accuracy mode matches across solvers:
/// a failed solve has no accuracy to match, and without this figure the shipped
/// tier could not show that the accuracies were matched at all. It is published
/// beside the unconditioned median rather than in place of it, and its name says
/// what it is conditioned on.

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
    std::string accuracy_target;
    std::string accuracy_target_met;
    double tolerance;
    bool kernel_countable;
    bool success_rate_reportable;
    std::vector<double> achieved;
    std::vector<double> pos_err;
    std::vector<double> pos_err_accepted;
    std::vector<double> ori_err;
    std::vector<double> wall_ns;
    std::vector<double> fk_evals;
    std::vector<double> jac_evals;
    int accepted;
    int self_reported;
    int false_success;
    int false_failure;

    cell_accumulator(std::string cell, std::string target, std::string met, double gate,
        bool countable, bool reportable)
        : identity(std::move(cell))
        , accuracy_target(std::move(target))
        , accuracy_target_met(std::move(met))
        , tolerance(gate)
        , kernel_countable(countable)
        , success_rate_reportable(reportable)
        , achieved()
        , pos_err()
        , pos_err_accepted()
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
        achieved.push_back(static_cast<double>(row.budget_value));
        pos_err.push_back(row.adjudication.pos_err);
        if (row.adjudication.accepted)
        {
            pos_err_accepted.push_back(row.adjudication.pos_err);
        }
        ori_err.push_back(row.adjudication.ori_err);
        wall_ns.push_back(static_cast<double>(row.wall_ns));
        if (row.kernel_countable)
        {
            fk_evals.push_back(static_cast<double>(row.counts.fk));
            jac_evals.push_back(static_cast<double>(row.counts.jac));
        }
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
        csv_field(row.stratum), csv_field(row.solver), row.budget_index, row.budget_requested,
        csv_field(row.budget_axis));
}

inline std::string_view cell_header()
{
    return "table,robot,limits_provenance,stratum,solver,budget_index,budget_requested,"
           "budget_axis,n_targets,n_accepted,n_self_reported,n_false_success,n_false_failure,"
           "accept_rate,budget_value_median,pos_err_median_m,pos_err_p95_m,pos_err_p99_m,"
           "ori_err_median_rad,fk_evals_median,jac_evals_median,wall_ns_median,wall_ns_cv,"
           "solver_tolerance,kernel_countable,accuracy_target,accuracy_target_met,"
           "pos_err_median_accepted_m";
}

/// An order statistic over nothing is written empty rather than as a number, so
/// a participant whose kernel evaluations the harness cannot count is visibly
/// uncounted in the shipped tier as well as in the per-target rows.
inline std::string statistic_field(const std::vector<double>& values, double quantile)
{
    if (values.empty())
    {
        return std::string{};
    }
    return std::format("{:.17g}", detail::order_statistic(values, quantile));
}

/// A stratum whose targets are not known to be reachable has no success rate to
/// report, and an empty column is how the tooling says so. A number there would
/// be a failure count for problems the solver was right to refuse.
inline std::string accept_rate_field(const cell_accumulator& cell)
{
    const auto targets = static_cast<double>(cell.pos_err.size());
    if (!cell.success_rate_reportable || targets == 0.0)
    {
        return std::string{};
    }
    return std::format("{:.17g}", static_cast<double>(cell.accepted) / targets);
}

inline std::string cell_row(const cell_accumulator& cell)
{
    const auto targets = static_cast<int>(cell.pos_err.size());
    return std::format("{},{},{},{},{},{},{},{:.17g},{:.17g},{:.17g},{:.17g},{:.17g},{},{},"
                       "{:.17g},{:.17g},{:.17g},{:d},{},{},{}",
        cell.identity, targets, cell.accepted, cell.self_reported, cell.false_success,
        cell.false_failure, accept_rate_field(cell),
        detail::order_statistic(cell.achieved, 0.5), detail::order_statistic(cell.pos_err, 0.5),
        detail::order_statistic(cell.pos_err, 0.95), detail::order_statistic(cell.pos_err, 0.99),
        detail::order_statistic(cell.ori_err, 0.5), statistic_field(cell.fk_evals, 0.5),
        statistic_field(cell.jac_evals, 0.5), detail::order_statistic(cell.wall_ns, 0.5),
        detail::variation(cell.wall_ns), cell.tolerance, cell.kernel_countable,
        cell.accuracy_target, cell.accuracy_target_met,
        statistic_field(cell.pos_err_accepted, 0.5));
}

}

#endif
