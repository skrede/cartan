#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_BUILD_MANIFEST_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_BUILD_MANIFEST_H

/// @file build_manifest.h
/// @brief What the configure resolved, carried into the program that publishes it.
///
/// The acquisition module decides where every dependency came from and what
/// version it reports; those findings arrive here as compile definitions and are
/// written out beside the records. A number and the identity of the code that
/// produced it travel together or the number means nothing.

#include <Eigen/Core>

#include <string>
#include <vector>
#include <cstddef>
#include <string_view>

namespace cartan::bench
{

struct dependency_record
{
    std::string name;
    std::string acquisition_class;
    std::string provider;
    std::string version_or_revision;
    std::string kernel_countable;
};

struct absent_participant
{
    std::string name;
    std::string reason;
};

struct description_pin
{
    std::string repository_key;
    std::string repository;
    std::string revision;
};

namespace detail
{

inline std::vector<std::string> split(std::string_view text, char separator)
{
    std::vector<std::string> parts;
    std::size_t start = 0;
    while (start <= text.size() && !text.empty())
    {
        const auto stop = text.find(separator, start);
        parts.emplace_back(text.substr(start, stop - start));
        if (stop == std::string_view::npos)
        {
            break;
        }
        start = stop + 1;
    }
    return parts;
}

inline std::string field(const std::vector<std::string>& fields, std::size_t index)
{
    return index < fields.size() ? fields[index] : std::string{};
}

inline std::vector<std::vector<std::string>> records(std::string_view manifest)
{
    std::vector<std::vector<std::string>> parsed;
    for (const auto& record : split(manifest, '@'))
    {
        parsed.push_back(split(record, '|'));
    }
    return parsed;
}

}

inline std::vector<dependency_record> build_dependencies()
{
    std::vector<dependency_record> resolved;
    for (const auto& fields : detail::records(CARTAN_BENCH_DEPENDENCY_MANIFEST))
    {
        resolved.push_back(dependency_record{
            detail::field(fields, 0), detail::field(fields, 1), detail::field(fields, 2),
            detail::field(fields, 3), detail::field(fields, 4)});
    }
    return resolved;
}

inline std::vector<absent_participant> build_absences()
{
    std::vector<absent_participant> absent;
    for (const auto& fields : detail::records(CARTAN_BENCH_ABSENCE_MANIFEST))
    {
        absent.push_back(absent_participant{detail::field(fields, 0), detail::field(fields, 1)});
    }
    return absent;
}

inline std::vector<description_pin> build_description_pins()
{
    std::vector<description_pin> pins;
    for (const auto& fields : detail::records(CARTAN_BENCH_DESCRIPTION_MANIFEST))
    {
        pins.push_back(description_pin{
            detail::field(fields, 0), detail::field(fields, 1), detail::field(fields, 2)});
    }
    return pins;
}

inline std::string standard_library_identification()
{
#if defined(_GLIBCXX_RELEASE)
    return "libstdc++ " + std::to_string(_GLIBCXX_RELEASE) + " ("
        + std::to_string(__GLIBCXX__) + ")";
#elif defined(_LIBCPP_VERSION)
    return "libc++ " + std::to_string(_LIBCPP_VERSION);
#elif defined(_MSVC_STL_VERSION)
    return "msvc stl " + std::to_string(_MSVC_STL_UPDATE);
#else
    return "unidentified standard library";
#endif
}

inline std::string eigen_version()
{
    return std::to_string(EIGEN_WORLD_VERSION) + "." + std::to_string(EIGEN_MAJOR_VERSION) + "."
        + std::to_string(EIGEN_MINOR_VERSION);
}

}

#endif
