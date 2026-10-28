#ifndef HPP_GUARD_CARTAN_BENCH_STUDY_TARGET_FIXTURE_H
#define HPP_GUARD_CARTAN_BENCH_STUDY_TARGET_FIXTURE_H

/// @file target_fixture.h
/// @brief The generated targets themselves, written beside the records.
///
/// A record says what happened to a target; this says what the target was. Both
/// are needed to reproduce a figure, and only one of them was ever published.
/// Every value is written at seventeen significant digits, so two runs of the
/// same command produce byte-identical files and a reader can diff them.

#include "target_pool.h"

#include <format>
#include <string>
#include <cstddef>
#include <fstream>
#include <stdexcept>
#include <filesystem>
#include <string_view>

namespace cartan::bench
{

namespace detail
{

inline std::string joint_columns(std::string_view prefix, int joints)
{
    std::string header;
    for (int i = 0; i < joints; ++i)
    {
        header += "," + std::string{prefix} + std::to_string(i);
    }
    return header;
}

template <int N>
std::string joint_values(const typename target_entry<N>::position_type& q, bool present)
{
    std::string row;
    for (int i = 0; i < N; ++i)
    {
        row += "," + (present ? std::format("{:.17g}", q(i)) : std::string{});
    }
    return row;
}

template <int N>
std::string fixture_row(const target_entry<N>& entry)
{
    const Eigen::Matrix3d rotation = entry.pose.rotation().matrix();
    std::string row = std::format("{},{},{:.17g}", entry.target_id, entry.rng_seed,
        entry.classifier_value);
    for (int i = 0; i < 3; ++i)
    {
        row += std::format(",{:.17g}", entry.pose.translation()(i));
    }
    for (int i = 0; i < 9; ++i)
    {
        row += std::format(",{:.17g}", rotation(i / 3, i % 3));
    }
    return row + joint_values<N>(entry.generated_from, entry.from_configuration)
        + joint_values<N>(entry.seed, true);
}

}

template <int N>
void write_target_fixture(
    const std::filesystem::path& directory,
    std::string_view table,
    std::string_view robot,
    const target_pool<N>& pool)
{
    std::filesystem::create_directories(directory);
    const auto name = std::format("fixture_{}_{}_{}.csv", table, robot,
        stratum_name(pool.which()));
    std::ofstream out(directory / name);
    if (!out)
    {
        throw std::runtime_error((directory / name).string() + ": cannot be opened for writing");
    }
    out << "target_id,rng_seed," << stratum_notes(pool.which()).classifier
        << ",px,py,pz,r00,r01,r02,r10,r11,r12,r20,r21,r22"
        << detail::joint_columns("source_q", N) << detail::joint_columns("seed_q", N) << '\n';
    for (int i = 0; i < pool.size(); ++i)
    {
        out << detail::fixture_row<N>(pool.entry(i)) << '\n';
    }
}

}

#endif
