#include <cartan/urdf.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <string>
#include <filesystem>

/// Asserts what the description-reading front end of the loader owes its
/// caller: a xacro document becomes a chain, a plain URDF agrees with the
/// in-tree parser on the chain it yields, a malformed document comes back as a
/// typed error carrying the line it failed on, and a sink that was never
/// pushed into cannot manufacture a chain.
///
/// One guarantee here is structural rather than observed: the sink is reached
/// only on the success arm, because the failure arm returns before a sink is
/// constructed. A runtime case cannot witness a call that does not happen, so
/// the assertion for it is made at source level by a separate gate.

using Catch::Approx;

namespace
{

std::filesystem::path fixture_path(const char* name)
{
    return std::filesystem::path{CARTAN_TESTS_FIXTURE_DIR} / "urdf" / name;
}

}

TEST_CASE("sink: a plain URDF yields the chain the in-tree parser yields", "[urdf_sink]")
{
    const auto path = fixture_path("cartanbot.urdf");

    auto loaded = cartan::load_urdf<double>(path);
    auto parsed = cartan::parse_urdf_file<double>(path);

    REQUIRE(loaded.has_value());
    REQUIRE(parsed.has_value());

    auto built = cartan::build_chain<double>(*parsed);
    REQUIRE(built.has_value());

    CHECK(loaded->chain.num_joints() == built->chain.num_joints());
    CHECK(loaded->metadata.joint_names == built->metadata.joint_names);
    CHECK(loaded->metadata.base_link_name == built->metadata.base_link_name);
    CHECK(loaded->metadata.tool_link_name == built->metadata.tool_link_name);
}

TEST_CASE("sink: a xacro description expands and becomes a chain", "[urdf_sink]")
{
    auto result = cartan::load_urdf<double>(fixture_path("xacro_minimal.urdf.xacro"));

    REQUIRE(result.has_value());
    REQUIRE(result->chain.num_joints() == 2);
    REQUIRE(result->metadata.joint_names.size() == 2);
    CHECK(result->metadata.joint_names[0] == "link_1_joint");
    CHECK(result->metadata.joint_names[1] == "tool_joint");
    CHECK(result->metadata.base_link_name == "base_link");
    CHECK(result->metadata.tool_link_name == "tool");
    // The two joint origins are 0.2*1 and 0.2*2, so the tool sits at 0.6 only
    // if the expression in the origin was evaluated rather than copied.
    CHECK(result->chain.home().translation()(2) == Approx(0.6).margin(1e-12));
}

TEST_CASE("sink: a malformed document returns a typed error naming the line", "[urdf_sink]")
{
    auto result = cartan::load_urdf<double>(fixture_path("parser_unclosed.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::malformed_xml);
    REQUIRE(result.error().location.has_value());
    CHECK(result.error().location->line == 14);
}

TEST_CASE("sink: finishing an empty push cannot manufacture a chain", "[urdf_sink]")
{
    cartan::detail::model_sink<double> sink{cartan::load_options{}};

    sink.finish();

    CHECK_FALSE(sink.result().has_value());
    CHECK(sink.staged().links.empty());
    CHECK(sink.staged().joints.empty());
}
