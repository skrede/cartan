#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_MANIFEST_FIELDS_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_MANIFEST_FIELDS_H

/// @file manifest_fields.h
/// @brief How each section of the environment record is rendered.

#include "exclusions.h"
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

struct named_count
{
    std::string name;
    int value;
};

/// The design's own sizes, not the sizes one run was asked for: a reader
/// checking whether a published figure came from the study this file describes
/// needs the parameters the study is defined by, with what this capture drew
/// beside them.
struct capture_parameters
{
    std::vector<named_count> targets_per_stratum;
    std::vector<named_count> comparator_time_cap_ms;
    std::vector<int> budget_ladder;
    std::vector<double> accuracy_targets;
    int repetitions;
    int targets_requested;
    std::string run_command;
};

struct description_deviation
{
    std::string robot;
    double value;
};

/// What this run refused to emit, and what the robot's own bounds did to the
/// three periodic rules. Both are findings rather than absences, and neither is
/// readable off the rows it is about.
struct study_notes
{
    bool rules_coincide;
    std::vector<int> qualifying_joints;
    std::vector<excluded_cell> exclusions;
    std::string population;
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

inline std::string counts_object(const std::vector<named_count>& counts, const char* indent)
{
    std::vector<std::string> fields;
    for (const auto& [name, value] : counts)
    {
        fields.push_back(json_string(name) + ": " + std::to_string(value));
    }
    return json_object(fields, indent);
}

inline std::string parameters_object(const capture_parameters& parameters)
{
    return json_object(
        {json_string("targets_per_stratum") + ": "
             + counts_object(parameters.targets_per_stratum, "    "),
         json_string("comparator_time_cap_ms") + ": "
             + counts_object(parameters.comparator_time_cap_ms, "    "),
         json_string("repetitions") + ": " + std::to_string(parameters.repetitions),
         json_string("budget_ladder") + ": " + numbers(parameters.budget_ladder),
         json_string("accuracy_targets") + ": " + numbers(parameters.accuracy_targets),
         json_string("targets_requested") + ": " + std::to_string(parameters.targets_requested)},
        "  ");
}

inline std::string excluded_object(const excluded_cell& cell)
{
    return json_object({json_field("robot", cell.robot), json_field("stratum", cell.stratum),
                           json_field("table", cell.table), json_field("reason", cell.reason)},
        "      ");
}

inline std::string notes_object(const study_notes& notes)
{
    std::vector<std::string> excluded;
    for (const auto& cell : notes.exclusions)
    {
        excluded.push_back(excluded_object(cell));
    }
    return json_object(
        {json_string("rules_coincide") + ": " + (notes.rules_coincide ? "true" : "false"),
            json_string("qualifying_joints") + ": " + numbers(notes.qualifying_joints),
            json_field("stratum_population", notes.population),
            json_string("excluded_cells") + ": " + json_array(excluded, "    ")},
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
