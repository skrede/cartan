#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_MANIFEST_FIELDS_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_MANIFEST_FIELDS_H

/// @file manifest_fields.h
/// @brief How each section of the environment record is rendered.

#include "json_writer.h"
#include "machine_state.h"
#include "build_manifest.h"
#include "description_chain.h"

#include <format>
#include <string>
#include <vector>
#include <cstddef>

namespace cartan::bench
{

struct capture_parameters
{
    int targets_per_stratum;
    int repetitions;
    std::vector<int> budget_ladder;
    std::vector<double> accuracy_targets;
    int comparator_time_cap_ms;
    std::string run_command;
};

struct description_deviation
{
    std::string robot;
    double value;
};

namespace detail
{

inline std::string dependency_object(const dependency_record& record)
{
    const bool countable = !record.kernel_countable.empty();
    return json_object(
        {json_field("name", record.name),
         json_field("acquisition_class", record.acquisition_class),
         json_field("provider", record.provider),
         json_field("version_or_revision", record.version_or_revision),
         json_string("kernel_countable") + ": "
             + (countable ? (record.kernel_countable == "1" ? "true" : "false") : "null")},
        "    ");
}

inline std::string description_object(const description_spec& spec, const description_pin& pin)
{
    std::vector<std::string> arguments;
    for (const auto& [name, value] : spec.args)
    {
        arguments.push_back(json_field(name, value));
    }
    return json_object(
        {json_field("repository", pin.repository), json_field("revision", pin.revision),
         json_field("path", std::string{spec.relative_path}),
         json_string("args") + ": " + json_object(arguments, "      ")},
        "    ");
}

inline std::string descriptions_array()
{
    const auto pins = build_description_pins();
    std::vector<std::string> objects;
    for (const auto& spec : description_specs())
    {
        for (const auto& pin : pins)
        {
            if (pin.repository_key == spec.repository_key)
            {
                objects.push_back(description_object(spec, pin));
            }
        }
    }
    return json_array(objects, "  ");
}

inline std::string numbers(const std::vector<int>& values)
{
    std::string body{"["};
    for (std::size_t i = 0; i < values.size(); ++i)
    {
        body += (i == 0 ? "" : ", ") + std::to_string(values[i]);
    }
    return body + "]";
}

inline std::string numbers(const std::vector<double>& values)
{
    std::string body{"["};
    for (std::size_t i = 0; i < values.size(); ++i)
    {
        body += (i == 0 ? "" : ", ") + std::format("{:.17g}", values[i]);
    }
    return body + "]";
}

inline std::string parameters_object(const capture_parameters& parameters)
{
    return json_object(
        {json_string("targets_per_stratum") + ": "
             + std::to_string(parameters.targets_per_stratum),
         json_string("repetitions") + ": " + std::to_string(parameters.repetitions),
         json_string("budget_ladder") + ": " + numbers(parameters.budget_ladder),
         json_string("accuracy_targets") + ": " + numbers(parameters.accuracy_targets),
         json_string("comparator_time_cap_ms") + ": "
             + std::to_string(parameters.comparator_time_cap_ms)},
        "  ");
}

inline std::vector<std::string> machine_fields(const std::vector<machine_value>& state)
{
    std::vector<std::string> fields;
    for (const auto& value : state)
    {
        fields.push_back(json_optional(value.name, value.value, value.read()));
    }
    return fields;
}

inline std::string unread_object(const std::vector<machine_value>& state)
{
    std::vector<std::string> fields;
    for (const auto& value : state)
    {
        if (!value.read())
        {
            fields.push_back(json_field(value.name, value.reason));
        }
    }
    return json_object(fields, "  ");
}

}

}

#endif
