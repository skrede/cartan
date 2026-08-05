#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_CAPTURE_OPTIONS_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_CAPTURE_OPTIONS_H

/// @file capture_options.h
/// @brief What the untimed pass was asked to capture.
///
/// One table per run, by refusal rather than by convention: a request naming
/// two periodic rules is turned away here, because the solves under two rules
/// are two experiments and a figure spanning them is not a comparison.

#include "budget.h"
#include "feasible_set.h"

#include <string>
#include <charconv>
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
    bool all_budgets;
    bool gate;
    bool selfcheck;
    std::filesystem::path out_dir;
    std::filesystem::path sidecar_dir;

    capture_options()
        : table("c")
        , robot("irb120")
        , stratum("reachable")
        , provenance("description")
        , command()
        , targets(200)
        , budget_index(k_budget_points - 1)
        , all_budgets(false)
        , gate(false)
        , selfcheck(false)
        , out_dir("study-cells")
        , sidecar_dir("study-targets")
    {
    }
};

namespace detail
{

inline int to_count(std::string_view text, int least)
{
    int value = 0;
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() || value < least)
    {
        throw std::runtime_error(std::string{text} + ": not a count of at least "
            + std::to_string(least));
    }
    return value;
}

inline std::string one_table(std::string_view value)
{
    if (value.size() != 1 || value.find_first_not_of("abc") != std::string_view::npos)
    {
        throw std::runtime_error(std::string{value}
            + ": a statistic spanning tables is not a comparison. The three periodic rules "
              "produce three experiments whose solves differ, so a run captures one table and "
              "the tables are published as separate artifacts");
    }
    return std::string{value};
}

inline std::string one_provenance(std::string_view value)
{
    if (value != "description" && value != "synthetic")
    {
        throw std::runtime_error(std::string{value}
            + ": limits are read from a description or invented here, and a row says which");
    }
    return std::string{value};
}

inline int one_budget_index(std::string_view value)
{
    const int index = to_count(value, 0);
    if (index >= k_budget_points)
    {
        throw std::runtime_error(
            std::string{value} + ": the ladder has " + std::to_string(k_budget_points) + " rungs");
    }
    return index;
}

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
    return false;
}

inline bool apply_flag(capture_options& options, std::string_view flag)
{
    if (flag == "--gate") { options.gate = true; return true; }
    if (flag == "--strata-selfcheck") { options.selfcheck = true; return true; }
    if (flag == "--all-budgets") { options.all_budgets = true; return true; }
    return false;
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
    return options;
}

}

#endif
