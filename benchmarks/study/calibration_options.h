#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_CALIBRATION_OPTIONS_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_CALIBRATION_OPTIONS_H

/// @file calibration_options.h
/// @brief What the offline calibration pass was asked to measure.
///
/// The accuracy override exists so that a target no solver can reach can be
/// asked for deliberately: the search's behavior at the far end of its interval
/// is the part a reader has least reason to take on trust, and a run that can
/// only ask for reachable targets cannot demonstrate it.

#include "budget.h"
#include "iso_accuracy.h"
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

struct calibration_options
{
    std::string table;
    std::string robot;
    std::string provenance;
    std::string command;
    int targets;
    double accuracy;
    std::filesystem::path out;

    calibration_options()
        : table("c")
        , robot("irb120")
        , provenance("description")
        , command()
        , targets(k_calibration_targets)
        , accuracy(std::numeric_limits<double>::quiet_NaN())
        , out("iso_accuracy_calibration.csv")
    {
    }

    std::vector<double> accuracy_targets() const
    {
        if (std::isnan(accuracy))
        {
            return {k_accuracy_targets.begin(), k_accuracy_targets.end()};
        }
        return {accuracy};
    }
};

namespace detail
{

inline bool apply_calibration_option(
    calibration_options& options, std::string_view flag, std::string_view value)
{
    if (flag == "--table") { options.table = one_table(value); return true; }
    if (flag == "--robot") { options.robot = value; return true; }
    if (flag == "--limits") { options.provenance = one_provenance(value); return true; }
    if (flag == "--targets") { options.targets = to_count(value, 1); return true; }
    if (flag == "--accuracy") { options.accuracy = a_distance(value); return true; }
    if (flag == "--out") { options.out = value; return true; }
    return false;
}

}

inline calibration_options parse_calibration_options(int argc, char** argv)
{
    calibration_options options;
    for (int i = 0; i < argc; ++i)
    {
        options.command += (i == 0 ? "" : " ");
        options.command += argv[i];
    }
    for (int i = 1; i + 1 < argc; i += 2)
    {
        const std::string_view flag{argv[i]};
        if (!detail::apply_calibration_option(options, flag, argv[i + 1]))
        {
            throw std::runtime_error(std::string{flag} + ": unknown option");
        }
    }
    if (argc % 2 == 0)
    {
        throw std::runtime_error(std::string{argv[argc - 1]} + ": expects a value");
    }
    return options;
}

}

#endif
