#ifndef HPP_GUARD_CARTAN_TESTS_SUPPORT_URDF_STAGE_PARITY_H
#define HPP_GUARD_CARTAN_TESTS_SUPPORT_URDF_STAGE_PARITY_H

/// The two front halves under comparison and the fixture set they run over.
/// Both stage the same document into a parsed_model: one parses the XML itself,
/// the other drives the description reader's push through the staging sink.
/// Everything downstream of the staging step is shared, so a difference here is
/// the only way the two can hand the extractor different robots.
///
/// Each row carries the joint count its document declares. A fixture that
/// yields a different number fails rather than comparing whatever it did yield:
/// an empty or truncated comparison passes vacuously and is indistinguishable
/// from agreement.
///
/// A row also names the joints whose <limit> declares effort and velocity but
/// no lower or upper. The two front halves disagree about exactly those, and
/// about nothing else: the reader's limit record holds four plain numbers, so a
/// bound the document never wrote arrives as a declared zero and cannot be told
/// from one the document wrote as zero. The comparison asserts that difference
/// set exactly, so a new divergence and the day the distinction is recovered
/// both turn it red.

#include "urdf_stage_probe.h"

#include <cartan/urdf.h>

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>
#include <ostream>
#include <optional>
#include <filesystem>
#include <string_view>

namespace cartan::testing
{

struct fixture_row
{
    std::string path;
    std::size_t joints;
    std::vector<std::string> partial_limits;
};

inline std::vector<fixture_row> stage_parity_fixtures()
{
    std::vector<fixture_row> rows{
        {"extractor_serial_minimal.urdf", 3, {}}, {"extractor_fixed_merge.urdf", 4, {}},
        {"extractor_continuous_wrist.urdf", 3, {"wrist_roll"}},
        {"extractor_branched.urdf", 4, {}}, {"parser_minimal.urdf", 2, {}},
        {"semantic_axis_omitted.urdf", 1, {}}, {"semantic_continuous_no_limit.urdf", 1, {}},
        {"narrowing_overflow.urdf", 1, {}}, {"cartanbot.urdf", 8, {"joint5"}}};
#ifdef CARTAN_URDF_EXTENDED_TESTS
    for (const fixture_row& row : std::vector<fixture_row>{
             {"extended/ur3e.urdf", 10, {}}, {"extended/ur5e.urdf", 10, {}},
             {"extended/ur10.urdf", 10, {}}, {"extended/ur16.urdf", 10, {}},
             {"extended/iiwa14.urdf", 9, {}}, {"extended/iiwa7.urdf", 9, {}},
             {"extended/panda.urdf", 16, {}}, {"extended/irb120.urdf", 9, {}},
             {"extended/kr6_sixx_r900.urdf", 9, {}}})
    {
        rows.push_back(row);
    }
#endif
    return rows;
}

inline std::vector<std::string> undeclared_bound_fields(const fixture_row& row)
{
    std::vector<std::string> fields;
    for (const std::string& joint : row.partial_limits)
    {
        fields.push_back(joint + " limit lower declared");
        fields.push_back(joint + " limit upper declared");
    }
    return fields;
}

/// The scalar count the fixture set contains, measured before this comparison
/// existed and excluding the one document only the wider scalar can carry. A
/// run that reaches fewer values is comparing less than the set holds.
#ifdef CARTAN_URDF_EXTENDED_TESTS
inline constexpr std::size_t staged_value_floor = 2070;
#else
inline constexpr std::size_t staged_value_floor = 462;
#endif

inline std::filesystem::path stage_fixture_file(const std::string& relative)
{
    return std::filesystem::path{CARTAN_TESTS_FIXTURE_DIR} / "urdf" / relative;
}

/// The reader's front half, staged but not extracted. The chain finish() builds
/// is discarded and only the sink's own refusal is reported: one fixture in the
/// set is a branched tree the extractor legitimately refuses, and reading that
/// refusal as a staging failure would drop the fixture from the comparison.
template <typename Scalar>
cartan::expected<parsed_model<Scalar>, urdf_error> staged_by_reader(
    const std::filesystem::path& path, const load_options& opts)
{
    detail::diagnostic_sink log;
    auto read = meios::load(path, opts.description, log);
    if (!read) { return cartan::unexpected(detail::failure_from(read.error())); }
    detail::model_sink<Scalar> sink(opts);
    detail::push_model(read->robot, sink);
    if (sink.failure()) { return cartan::unexpected(*sink.failure()); }
    return sink.staged();
}

/// A fixture the narrower scalar cannot carry must be refused by both front
/// halves, with the same kind, before its comparison is skipped: a divergence
/// hiding behind one path's refusal would otherwise never be seen.
template <typename Scalar>
std::optional<stage_tally> compare_front_halves(const fixture_row& row, const load_options& opts)
{
    const std::filesystem::path path = stage_fixture_file(row.path);
    auto parsed = cartan::parse_urdf_file<Scalar>(path);
    auto staged = staged_by_reader<Scalar>(path, opts);
    INFO("fixture " << row.path);
    if (!parsed.has_value() || !staged.has_value())
    {
        REQUIRE_FALSE(parsed.has_value());
        REQUIRE_FALSE(staged.has_value());
        CHECK(parsed.error().kind == staged.error().kind);
        return std::nullopt;
    }
    stage_tally tally = probe_model<Scalar>(row.path, *parsed, *staged);
    REQUIRE(tally.joints == row.joints);
    CHECK(tally.numeric_differences == 0);
    CHECK(tally.worst_difference == 0.0);
    CHECK(tally.differences == undeclared_bound_fields(row));
    return tally;
}

template <typename Scalar>
std::vector<stage_tally> compare_fixture_set()
{
    const load_options opts;
    std::vector<stage_tally> out;
    for (const fixture_row& row : stage_parity_fixtures())
    {
        if (auto tally = compare_front_halves<Scalar>(row, opts)) { out.push_back(*tally); }
    }
    return out;
}

inline std::size_t report_tallies(std::ostream& to, std::string_view instantiation,
                                  const std::vector<stage_tally>& tallies)
{
    std::size_t numeric = 0;
    for (const stage_tally& tally : tallies)
    {
        to << instantiation << ' ' << tally.fixture << ": joints " << tally.joints << ", compared "
           << tally.compared << ", equal " << tally.equal << ", scalars " << tally.numeric
           << ", NaN pairs " << tally.nan_agreements << ", worst difference "
           << tally.worst_difference << '\n';
        for (const std::string& what : tally.differences) { to << "    differs at " << what << '\n'; }
        numeric += tally.numeric;
    }
    return numeric;
}

}

#endif
