#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_MANIFEST_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_MANIFEST_H

/// @file manifest.h
/// @brief The environment record written beside every capture.
///
/// A reader who has the same libraries has to be able to tell exactly what was
/// measured: which compiler and standard library, which revision of every
/// dependency, which robot descriptions at which commits, what the machine's
/// governor and boost state were, and what the study itself was asked to do.
/// Everything here is read from the build or from the machine at run time --
/// nothing is asserted.

#include "manifest_fields.h"
#include "participant_list.h"

#include <format>
#include <string>
#include <vector>
#include <fstream>
#include <utility>
#include <stdexcept>
#include <filesystem>

namespace cartan::bench
{

namespace detail
{

/// The declared list travels with the records because a table missing a solver
/// reads exactly like a table that never declared one. Nothing downstream can
/// tell the two apart from the rows alone, so the authority is written here.
inline std::string declared_array()
{
    std::vector<std::string> entries;
    for (const auto& declared : declared_participants())
    {
        entries.push_back(json_object({json_field("name", declared.name),
                                          json_field("supplier", declared.supplier),
                                          json_string("kernel_countable") + ": "
                                              + (declared.kernel_countable ? "true" : "false")},
            "    "));
    }
    return json_array(entries, "  ");
}

}

/// The absence list is the study's own, not the build's: a comparator the build
/// did not resolve and a participant the study declared are different lists, and
/// the reader needs the second one.
inline void write_manifest(
    const std::filesystem::path& directory,
    const capture_parameters& parameters,
    const std::vector<absent_participant>& absent,
    const description_deviation& measured,
    const study_notes& notes)
{
    std::filesystem::create_directories(directory);
    const auto path = directory / "environment.json";
    std::ofstream out(path);
    if (!out)
    {
        throw std::runtime_error(path.string() + ": cannot be opened for writing");
    }

    std::vector<std::string> dependencies;
    for (const auto& record : build_dependencies())
    {
        dependencies.push_back(detail::dependency_object(record));
    }
    std::vector<std::string> absences;
    for (const auto& entry : absent)
    {
        absences.push_back(
            json_object({json_field("name", entry.name), json_field("reason", entry.reason)},
                "    "));
    }

    const auto state = read_machine_state();
    std::vector<std::string> fields{
        json_field("compiler", CARTAN_BENCH_COMPILER),
        json_field("standard_library", standard_library_identification()),
        json_field("eigen", eigen_version())};
    for (auto& field : detail::machine_fields(state))
    {
        fields.push_back(std::move(field));
    }
    fields.push_back(json_string("unread_machine_values") + ": " + detail::unread_object(state));
    fields.push_back(json_field("configure_command", CARTAN_BENCH_CONFIGURE_COMMAND));
    fields.push_back(json_field("run_command", parameters.run_command));
    fields.push_back(json_string("dependencies") + ": " + json_array(dependencies, "  "));
    fields.push_back(json_string("declared_participants") + ": " + detail::declared_array());
    fields.push_back(json_string("absent_participants") + ": " + json_array(absences, "  "));
    fields.push_back(json_string("descriptions") + ": " + detail::descriptions_array());
    fields.push_back(
        json_string("capture_parameters") + ": " + detail::parameters_object(parameters));
    fields.push_back(json_string("study_notes") + ": " + detail::notes_object(notes));
    fields.push_back(json_string("description_fk_max_deviation") + ": "
        + json_object({json_field("robot", measured.robot),
                          json_string("value") + ": " + std::format("{:.17g}", measured.value)},
            "  "));

    out << json_object(fields, "") << '\n';
}

}

#endif
