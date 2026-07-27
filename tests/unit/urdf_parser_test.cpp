#include <cartan/urdf.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string>
#include <filesystem>

using Catch::Approx;

namespace
{

std::filesystem::path fixture_path(const char* name)
{
    return std::filesystem::path{CARTAN_TESTS_FIXTURE_DIR} / "urdf" / name;
}

}

TEST_CASE("parser: minimal well-formed URDF round-trips", "[urdf_parser]")
{
    auto result = cartan::parse_urdf_file<double>(fixture_path("parser_minimal.urdf"));

    REQUIRE(result.has_value());
    const auto& model = *result;

    CHECK(model.robot_name == "parser_minimal");
    REQUIRE(model.links.size() == 3);
    CHECK(model.links[0].name == "base_link");
    CHECK(model.links[1].name == "link_1");
    CHECK(model.links[2].name == "link_2");

    REQUIRE(model.links[0].inertial.has_value());
    CHECK(model.links[0].inertial->mass == Approx(1.0).margin(1e-12));
    CHECK(model.links[0].inertial->inertia(0, 0) == Approx(0.1).margin(1e-12));
    CHECK(model.links[0].inertial->inertia(2, 2) == Approx(0.1).margin(1e-12));

    REQUIRE(model.links[1].inertial.has_value());
    CHECK(model.links[1].inertial->mass == Approx(2.5).margin(1e-12));
    CHECK(model.links[1].inertial->com(2) == Approx(0.25).margin(1e-12));

    CHECK_FALSE(model.links[2].inertial.has_value());

    REQUIRE(model.joints.size() == 2);

    const auto& j1 = model.joints[0];
    CHECK(j1.name == "joint_1");
    CHECK(j1.kind == cartan::parsed_joint_kind::revolute);
    CHECK(j1.parent_link == "base_link");
    CHECK(j1.child_link == "link_1");
    CHECK(j1.axis(2) == Approx(1.0).margin(1e-12));
    CHECK(j1.origin.translation()(2) == Approx(0.1).margin(1e-12));
    REQUIRE(j1.position_min.has_value());
    REQUIRE(j1.position_max.has_value());
    CHECK(*j1.position_min == Approx(-3.14159).margin(1e-9));
    CHECK(*j1.position_max == Approx(3.14159).margin(1e-9));
    REQUIRE(j1.velocity_max.has_value());
    CHECK(*j1.velocity_max == Approx(2.0).margin(1e-12));
    REQUIRE(j1.effort_max.has_value());
    CHECK(*j1.effort_max == Approx(50.0).margin(1e-12));

    const auto& j2 = model.joints[1];
    CHECK(j2.name == "joint_2");
    CHECK(j2.kind == cartan::parsed_joint_kind::continuous);
    CHECK_FALSE(j2.position_min.has_value());
    CHECK_FALSE(j2.position_max.has_value());
}

TEST_CASE("parser: unclosed XML reports malformed_xml with line info", "[urdf_parser]")
{
    auto result = cartan::parse_urdf_file<double>(fixture_path("parser_unclosed.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::malformed_xml);
    REQUIRE(result.error().location.has_value());
    CHECK(result.error().location->line > 0);
    CHECK(!result.error().detail.empty());
}

TEST_CASE("parser: unknown joint type reports unsupported_joint_type", "[urdf_parser]")
{
    auto result = cartan::parse_urdf_file<double>(fixture_path("parser_unknown_joint.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::unsupported_joint_type);
    CHECK(result.error().detail.find("screw") != std::string::npos);
}

TEST_CASE("parser: unknown parent link reports unknown_parent_link", "[urdf_parser]")
{
    auto result = cartan::parse_urdf_file<double>(fixture_path("parser_unknown_parent.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::unknown_parent_link);
    CHECK(result.error().detail.find("ghost") != std::string::npos);
}

TEST_CASE("parser: mimic joint reports mimic_joint_unsupported", "[urdf_parser]")
{
    auto result = cartan::parse_urdf_file<double>(fixture_path("parser_mimic.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::mimic_joint_unsupported);
}

// --- Adversarial / strict-reject cases (parse-time, no kinematic walk) ---
//
// These exercise the untrusted-input surface. Each malformed input must be
// rejected with a typed error whose detail names the offending joint or link.
// parse_urdf_file never walks the tree, so none of these hang even pre-fix;
// the pre-fix defect is silent acceptance, which the REQUIRE_FALSE catches.

TEST_CASE("parser: self-loop joint (parent == child) is rejected", "[urdf_parser][urdf_strict]")
{
    auto result = cartan::parse_urdf_file<double>(fixture_path("adversarial_self_loop.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::cyclic_kinematic_tree);
    CHECK(result.error().detail.find("self_loop_joint") != std::string::npos);
}

TEST_CASE("parser: kinematic cycle (link revisited) is rejected", "[urdf_parser][urdf_strict]")
{
    auto result = cartan::parse_urdf_file<double>(fixture_path("adversarial_cycle.urdf"));

    REQUIRE_FALSE(result.has_value());
    // link_a is the child of two joints, so the cycle surfaces as a
    // multi-parent (non-tree) rejection that names the offending link.
    CHECK(result.error().kind == cartan::urdf_failure::multi_parent_link);
    CHECK(result.error().detail.find("link_a") != std::string::npos);
}

TEST_CASE("parser: multi-parent link is rejected", "[urdf_parser][urdf_strict]")
{
    auto result = cartan::parse_urdf_file<double>(fixture_path("adversarial_multi_parent.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::multi_parent_link);
    CHECK(result.error().detail.find("tip") != std::string::npos);
}

TEST_CASE("parser: duplicate link name is rejected", "[urdf_parser][urdf_strict]")
{
    auto result = cartan::parse_urdf_file<double>(fixture_path("adversarial_duplicate_name.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::duplicate_name);
    CHECK(result.error().detail.find("link_1") != std::string::npos);
}

TEST_CASE("parser: non-finite numeric attribute is rejected", "[urdf_parser][urdf_strict]")
{
    auto result = cartan::parse_urdf_file<double>(fixture_path("adversarial_non_finite.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::non_finite_value);
    CHECK(result.error().detail.find("nan_limit_joint") != std::string::npos);
}

// --- Narrowing to the chain's scalar type ---
//
// A bound that is finite as a double and overflows to an infinity as a float has
// to be caught at the conversion. Afterwards it is indistinguishable from the
// +/-infinity the builder deliberately writes for a continuous joint, which is
// why the same description must load at one precision and be refused at the
// other. Both halves are asserted: a rejection at both precisions would mean the
// bound was refused for some other reason entirely.

TEST_CASE("parser: a bound that overflows the scalar type is rejected at that "
          "scalar only", "[urdf_parser][urdf_strict]")
{
    const auto path = fixture_path("narrowing_overflow.urdf");

    auto as_double = cartan::parse_urdf_file<double>(path);
    REQUIRE(as_double.has_value());
    REQUIRE(as_double->joints.size() == 1);
    REQUIRE(as_double->joints[0].position_min.has_value());
    CHECK(std::isfinite(*as_double->joints[0].position_min));
    CHECK(std::isfinite(*as_double->joints[0].position_max));

    auto as_float = cartan::parse_urdf_file<float>(path);
    REQUIRE_FALSE(as_float.has_value());
    CHECK(as_float.error().kind == cartan::urdf_failure::non_finite_value);
    CHECK(as_float.error().detail.find("overflow_joint") != std::string::npos);
}

/// The pre-cast gate that shipped before this rejection existed, and the value
/// the builder would have received once it passed. Neither half is cartan code,
/// which is the point: the loader looked only at the double.
TEST_CASE("parser: the pre-cast finiteness test admits a value the cast overflows",
    "[urdf_parser][urdf_strict]")
{
    const double bound = 1e300;
    CHECK(std::isfinite(bound));
    CHECK(std::isinf(static_cast<float>(bound)));
    CHECK(std::isinf(static_cast<float>(-bound)));
}

TEST_CASE("parser: a reversed <limit> is rejected by the limits factory",
    "[urdf_parser][urdf_strict]")
{
    auto parsed = cartan::parse_urdf_file<double>(fixture_path("reversed_limit.urdf"));
    REQUIRE(parsed.has_value());

    auto built = cartan::build_chain<double>(*parsed);
    REQUIRE_FALSE(built.has_value());
    CHECK(built.error().kind == cartan::urdf_failure::invalid_joint_limit);
    CHECK(built.error().detail.find("reversed_joint") != std::string::npos);
}

TEST_CASE("parser: a continuous joint still loads at both scalars",
    "[urdf_parser][urdf_strict]")
{
    const auto path = fixture_path("extractor_continuous_wrist.urdf");

    auto as_double = cartan::load_urdf<double>(path);
    REQUIRE(as_double.has_value());
    auto as_float = cartan::load_urdf<float>(path);
    REQUIRE(as_float.has_value());

    bool saw_unbounded = false;
    for (const auto& lim : as_double->chain.limits())
    {
        saw_unbounded = saw_unbounded
            || (std::isinf(lim.position_min()) && std::isinf(lim.position_max()));
    }
    CHECK(saw_unbounded);
}
