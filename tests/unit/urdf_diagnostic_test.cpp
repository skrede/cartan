#include <cartan/urdf.h>
#include <cartan/urdf/detail/code_map.h>
#include <cartan/urdf/detail/diagnostic_sink.h>

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <algorithm>
#include <filesystem>
#include <system_error>

/// Asserts what the diagnostics channel owes its caller: a successful load
/// reports records the caller can tier by severity, a warn record can be tied
/// to the reader code that produced it, a completeness claim the reader
/// withholds does not refuse the load, and relaxing the missing-asset policy
/// leaves a structural failure hard. The vendored real-world cases compile in
/// only under CARTAN_URDF_EXTENDED_TESTS.

namespace
{

std::filesystem::path fixture_path(const char* name)
{
    return std::filesystem::path{CARTAN_TESTS_FIXTURE_DIR} / "urdf" / name;
}

bool is_warn(const cartan::urdf_diagnostic& record)
{
    return record.severity == cartan::urdf_severity::warn;
}

bool is_error(const cartan::urdf_diagnostic& record)
{
    return record.severity == cartan::urdf_severity::error;
}

}

#ifdef CARTAN_URDF_EXTENDED_TESTS

TEST_CASE("diagnostics: a degraded load reports at the warn tier", "[urdf_diagnostic]")
{
    auto loaded = cartan::load_urdf<double>(fixture_path("extended/irb120.urdf"));
    REQUIRE(loaded.has_value());

    const auto& records = loaded->diagnostics;
    CHECK(std::count_if(records.begin(), records.end(), is_warn) > 0);
    CHECK(std::count_if(records.begin(), records.end(), is_error) == 0);
}

TEST_CASE("diagnostics: a warn record names the unresolved asset behind it", "[urdf_diagnostic]")
{
    auto loaded = cartan::load_urdf<double>(fixture_path("extended/irb120.urdf"));
    REQUIRE(loaded.has_value());

    const auto& records = loaded->diagnostics;
    CHECK(std::any_of(records.begin(), records.end(), [](const cartan::urdf_diagnostic& record) {
        return is_warn(record) && record.meios_code == meios::diagnostic_code::unresolved_asset;
    }));
}

TEST_CASE("diagnostics: a withheld completeness claim does not refuse the load", "[urdf_diagnostic]")
{
    auto loaded = cartan::load_urdf<double>(fixture_path("extended/irb120.urdf"));
    REQUIRE(loaded.has_value());

    CHECK_FALSE(has(loaded->claims, meios::completeness::parsed));
    CHECK(has(loaded->claims, meios::completeness::topology_valid));
}

#endif

TEST_CASE("diagnostics: the relaxed asset policy leaves a structural failure hard",
          "[urdf_diagnostic]")
{
    auto loaded = cartan::load_urdf<double>(fixture_path("xacro_unresolved_include.urdf.xacro"));

    REQUIRE_FALSE(loaded.has_value());
    REQUIRE(loaded.error().meios_code.has_value());
    CHECK(*loaded.error().meios_code == meios::diagnostic_code::unresolved_include);
}

TEST_CASE("diagnostics: an unmapped reader code surfaces as unknown_error carrying that code",
          "[urdf_diagnostic]")
{
    auto loaded = cartan::load_urdf<double>(fixture_path("xacro_unresolved_include.urdf.xacro"));

    REQUIRE_FALSE(loaded.has_value());
    CHECK(loaded.error().kind == cartan::urdf_failure::unknown_error);
    REQUIRE(loaded.error().meios_code.has_value());
    CHECK(*loaded.error().meios_code == meios::diagnostic_code::unresolved_include);
}

/// Asserted against the table rather than through a load: at the pinned reader
/// revision the identity pass reports an undeclared link for every undeclared
/// reference, and it runs before the topology pass, so no load can deliver
/// invalid_topology. The arm stands for the day that ordering changes.
TEST_CASE("diagnostics: the reader's topology code maps to the link-reference kind",
          "[urdf_diagnostic]")
{
    CHECK(cartan::detail::failure_from_code(meios::diagnostic_code::invalid_topology)
          == cartan::urdf_failure::unknown_link_reference);
}

TEST_CASE("diagnostics: every arm the reader can log through is captured", "[urdf_diagnostic]")
{
    cartan::detail::diagnostic_sink sink;
    meios::log_sink& arms = sink;
    const meios::source_location at{std::filesystem::path{"robot.urdf"}, 7, 3};
    const meios::operation_failure cause{
        meios::operation_kind::open,
        std::make_error_code(std::errc::no_such_file_or_directory)};

    arms.log(meios::level::info, "bare");
    arms.log(meios::level::warn, at, "located");
    arms.log(meios::level::warn, meios::diagnostic_code::unresolved_asset, at, "coded");
    arms.log(meios::level::error, meios::diagnostic_code::cannot_open, at, cause, "caused");

    REQUIRE(sink.records().size() == 4);
    CHECK(sink.records()[0].severity == cartan::urdf_severity::info);
    CHECK(sink.records()[1].severity == cartan::urdf_severity::warn);
    CHECK(sink.records()[1].location.has_value());
    CHECK(sink.records()[2].meios_code == meios::diagnostic_code::unresolved_asset);
    CHECK(sink.records()[3].severity == cartan::urdf_severity::error);
    CHECK(sink.records()[3].meios_code == meios::diagnostic_code::cannot_open);
    CHECK(sink.records()[3].message.find("open") != std::string::npos);
}
