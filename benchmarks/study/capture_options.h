#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_CAPTURE_OPTIONS_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_CAPTURE_OPTIONS_H

/// @file capture_options.h
/// @brief What the untimed pass was asked to capture.
///
/// One table per run, by refusal rather than by convention: a request naming
/// two periodic rules is turned away here, because the solves under two rules
/// are two experiments and a figure spanning them is not a comparison.
///
/// The two comparison modes are exclusive by construction: a run either asks
/// every participant for one tolerance and reports the accuracies that produced,
/// or asks each participant for its own calibrated tolerance so that the
/// accuracies match. The accuracy mode names its target and the table it reads
/// it from, and neither half of that pair means anything without the other.

#include "feasible_set.h"
#include "option_values.h"

#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <stdexcept>
#include <filesystem>
#include <string_view>

namespace cartan::bench
{

struct capture_options
{
    std::string table;
    std::string robot;
    std::string stratum;
    std::string provenance;
    std::string command;
    int targets;
    int budget_index;
    double accuracy_target;
    bool all_budgets;
    bool gate;
    bool selfcheck;
    bool include_unconverged;
    std::filesystem::path out_dir;
    std::filesystem::path sidecar_dir;
    std::filesystem::path calibration;

    capture_options()
        : table("c")
        , robot("irb120")
        , stratum("reachable")
        , provenance("description")
        , command()
        , targets(200)
        , budget_index(k_budget_points - 1)
        , accuracy_target(std::numeric_limits<double>::quiet_NaN())
        , all_budgets(false)
        , gate(false)
        , selfcheck(false)
        , include_unconverged(false)
        , out_dir("study-cells")
        , sidecar_dir("study-targets")
        , calibration()
    {
    }

    bool iso_accuracy() const { return !std::isnan(accuracy_target); }

    std::vector<double> accuracy_targets() const
    {
        return iso_accuracy() ? std::vector<double>{accuracy_target} : std::vector<double>{};
    }
};

namespace detail
{

inline bool apply_option(capture_options& options, std::string_view flag, std::string_view value)
{
    if (flag == "--table") { options.table = one_table(value); return true; }
    if (flag == "--rule") { options.table = table_key(rule_from_name(value)); return true; }
    if (flag == "--robot") { options.robot = value; return true; }
    if (flag == "--stratum") { options.stratum = value; return true; }
    if (flag == "--limits") { options.provenance = one_provenance(value); return true; }
    if (flag == "--targets") { options.targets = to_count(value, 1); return true; }
    if (flag == "--budget-index") { options.budget_index = one_budget_index(value); return true; }
    if (flag == "--out-dir") { options.out_dir = value; return true; }
    if (flag == "--sidecar-dir") { options.sidecar_dir = value; return true; }
    if (flag == "--calibration") { options.calibration = value; return true; }
    if (flag == "--iso-accuracy")
    {
        options.accuracy_target = one_accuracy_target(value);
        return true;
    }
    return false;
}

inline bool apply_flag(capture_options& options, std::string_view flag)
{
    if (flag == "--gate") { options.gate = true; return true; }
    if (flag == "--strata-selfcheck") { options.selfcheck = true; return true; }
    if (flag == "--all-budgets") { options.all_budgets = true; return true; }
    if (flag == "--include-unconverged") { options.include_unconverged = true; return true; }
    return false;
}

inline void refuse_half_a_mode(const capture_options& options)
{
    if (options.iso_accuracy() == !options.calibration.empty())
    {
        return;
    }
    throw std::runtime_error(options.iso_accuracy()
            ? "--iso-accuracy names a target but no --calibration table says what each solver has "
              "to be asked for to reach it"
            : "--calibration names a table that no mode reads: pass --iso-accuracy to run against "
              "it");
}

}

inline capture_options parse_capture_options(int argc, char** argv)
{
    capture_options options;
    for (int i = 0; i < argc; ++i)
    {
        options.command += (i == 0 ? "" : " ");
        options.command += argv[i];
    }
    for (int i = 1; i < argc; ++i)
    {
        const std::string_view flag{argv[i]};
        if (detail::apply_flag(options, flag))
        {
            continue;
        }
        if (i + 1 >= argc)
        {
            throw std::runtime_error(std::string{flag} + ": expects a value");
        }
        if (!detail::apply_option(options, flag, argv[i + 1]))
        {
            throw std::runtime_error(std::string{flag} + ": unknown option");
        }
        i += 1;
    }
    detail::refuse_half_a_mode(options);
    return options;
}

}

#endif
