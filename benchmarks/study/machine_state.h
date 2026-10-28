#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_MACHINE_STATE_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_MACHINE_STATE_H

/// @file machine_state.h
/// @brief The machine as it was at run time, read rather than assumed.
///
/// Governor, frequency boost and simultaneous multithreading move wall-clock by
/// more than most of the differences a study reports, and none of them is
/// visible in the build. A value the program cannot read is carried as unread
/// with the reason beside it: an absent field and a field a reader may take for
/// the default are different claims.

#include <string>
#include <vector>
#include <fstream>
#include <utility>
#include <filesystem>
#include <string_view>

namespace cartan::bench
{

struct machine_value
{
    std::string name;
    std::string value;
    std::string reason;

    bool read() const { return reason.empty(); }
};

namespace detail
{

inline machine_value first_line(std::string_view name, const std::filesystem::path& path)
{
    std::ifstream source(path);
    std::string line;
    if (!source || !std::getline(source, line))
    {
        return machine_value{
            std::string{name}, "", "not readable on this machine: " + path.string()};
    }
    return machine_value{std::string{name}, line, ""};
}

/// The two interfaces that report frequency boost invert each other -- boost=1
/// and no_turbo=1 are opposite states -- so the value carries the file it came
/// from rather than a bare digit whose meaning depends on which one answered.
inline machine_value first_alternative(
    std::string_view name, const std::vector<std::filesystem::path>& candidates)
{
    machine_value last{std::string{name}, "", "no candidate path was consulted"};
    for (const auto& candidate : candidates)
    {
        last = first_line(name, candidate);
        if (last.read())
        {
            last.value += " (" + candidate.string() + ")";
            return last;
        }
    }
    return last;
}

inline machine_value tagged_line(
    std::string_view name, const std::filesystem::path& path, std::string_view tag)
{
    std::ifstream source(path);
    std::string line;
    while (source && std::getline(source, line))
    {
        const auto colon = line.find(':');
        if (colon != std::string::npos && line.substr(0, tag.size()) == tag)
        {
            return machine_value{
                std::string{name}, line.substr(line.find_first_not_of(" \t", colon + 1)), ""};
        }
    }
    return machine_value{
        std::string{name}, "",
        "no '" + std::string{tag} + "' entry in " + path.string()};
}

}

/// The five values the previous capture recorded by hand, in the order its
/// environment table lists them.
inline std::vector<machine_value> read_machine_state()
{
    return {
        detail::first_line("kernel", "/proc/sys/kernel/osrelease"),
        detail::first_line("governor", "/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"),
        detail::first_alternative("frequency_boost",
            {"/sys/devices/system/cpu/cpufreq/boost",
             "/sys/devices/system/cpu/intel_pstate/no_turbo"}),
        detail::first_line("simultaneous_multithreading", "/sys/devices/system/cpu/smt/active"),
        detail::tagged_line("cpu_model", "/proc/cpuinfo", "model name")};
}

}

#endif
