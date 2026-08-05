#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_CAPTURE_OPTIONS_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_CAPTURE_OPTIONS_H

/// @file capture_options.h
/// @brief What the untimed pass was asked to capture.

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
    int targets;
    bool gate;
    std::filesystem::path out_dir;
    std::filesystem::path sidecar_dir;

    capture_options()
        : table("c")
        , robot("irb120")
        , stratum("reachable")
        , targets(200)
        , gate(false)
        , out_dir("study-cells")
        , sidecar_dir("study-targets")
    {
    }
};

namespace detail
{

inline int to_count(std::string_view text)
{
    int value = 0;
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
    if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() || value <= 0)
    {
        throw std::runtime_error(std::string{text} + ": not a positive target count");
    }
    return value;
}

inline bool apply_option(capture_options& options, std::string_view flag, std::string_view value)
{
    if (flag == "--table") { options.table = value; return true; }
    if (flag == "--robot") { options.robot = value; return true; }
    if (flag == "--stratum") { options.stratum = value; return true; }
    if (flag == "--targets") { options.targets = to_count(value); return true; }
    if (flag == "--out-dir") { options.out_dir = value; return true; }
    if (flag == "--sidecar-dir") { options.sidecar_dir = value; return true; }
    return false;
}

}

inline capture_options parse_capture_options(int argc, char** argv)
{
    capture_options options;
    for (int i = 1; i < argc; ++i)
    {
        const std::string_view flag{argv[i]};
        if (flag == "--gate")
        {
            options.gate = true;
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
