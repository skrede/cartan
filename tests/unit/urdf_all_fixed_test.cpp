#include <cartan/urdf.h>

#include <cartan/lie/se3.h>

#include <cartan/types.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <string>
#include <filesystem>

/// A description whose joints are all fixed poses no inverse-kinematics
/// problem, so the chain loader refuses it and names the entry point that
/// answers it: the base-to-tool transform of the rigid assembly. The branched
/// case is refused on both routes -- the transform is only well defined when
/// the merge leaves one leaf, and a branched description leaves several.

using Catch::Approx;

namespace
{

std::filesystem::path fixture_path(const char* name)
{
    return std::filesystem::path{CARTAN_TESTS_FIXTURE_DIR} / "urdf" / name;
}

}

TEST_CASE("all-fixed: a chain-shaped description with no movable joint is refused",
          "[urdf_all_fixed]")
{
    auto result = cartan::load_urdf<double>(fixture_path("all_fixed_linear.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::no_movable_joint);
    CHECK(result.error().detail.find("load_urdf_transform") != std::string::npos);
}

TEST_CASE("all-fixed: a description with no joint at all is refused the same way",
          "[urdf_all_fixed]")
{
    auto result = cartan::load_urdf<double>(fixture_path("single_link.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::no_movable_joint);
    CHECK(result.error().detail.find("load_urdf_transform") != std::string::npos);
}

/// The branched shape is refused as branched rather than as a description with
/// no movable joint, and in that order: the no-movable-joint refusal points at
/// the transform entry point, which cannot answer a branched description
/// either, so answering it first would send the caller in a circle.
TEST_CASE("all-fixed: a branched description is refused as branched, naming its leaves",
          "[urdf_all_fixed]")
{
    auto result = cartan::load_urdf<double>(fixture_path("all_fixed_branched.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::branched_kinematic_tree);
    CHECK(result.error().detail.find("left_mount") != std::string::npos);
    CHECK(result.error().detail.find("right_mount") != std::string::npos);
}

TEST_CASE("all-fixed: the transform entry point composes the fixture's own origins",
          "[urdf_all_fixed]")
{
    auto result = cartan::load_urdf_transform<double>(fixture_path("all_fixed_linear.urdf"));

    REQUIRE(result.has_value());

    // The fixture's first joint sits at mount_origin turned a quarter turn
    // about z, so the second joint's offset arrives with its x and y swapped
    // and its new x negated.
    const cartan::vector3<double> mount_origin{0.1, 0.2, 0.3};
    const cartan::vector3<double> tool_offset{0.05, 0.0, 0.4};
    const cartan::vector3<double> expected{
        mount_origin.x() - tool_offset.y(),
        mount_origin.y() + tool_offset.x(),
        mount_origin.z() + tool_offset.z()};

    CHECK(result->translation().x() == Approx(expected.x()).margin(1e-12));
    CHECK(result->translation().y() == Approx(expected.y()).margin(1e-12));
    CHECK(result->translation().z() == Approx(expected.z()).margin(1e-12));

    const cartan::vector3<double> turned =
        result->rotation().act(cartan::vector3<double>{1.0, 0.0, 0.0});
    CHECK(turned.x() == Approx(0.0).margin(1e-12));
    CHECK(turned.y() == Approx(1.0).margin(1e-12));
}

TEST_CASE("all-fixed: a description with one link and no joint is the identity",
          "[urdf_all_fixed]")
{
    auto result = cartan::load_urdf_transform<double>(fixture_path("single_link.urdf"));

    REQUIRE(result.has_value());
    CHECK(result->translation().norm() == Approx(0.0).margin(1e-12));
    CHECK((result->rotation().matrix() - cartan::matrix3<double>::Identity()).norm()
          == Approx(0.0).margin(1e-12));
}

/// The regression gate for the defect this repaired: the walk stopped at the
/// branch link without folding either leaf, and the loader answered with an
/// identity transform and a tool link equal to the base link.
TEST_CASE("all-fixed: the transform entry point refuses a branched description",
          "[urdf_all_fixed]")
{
    auto result = cartan::load_urdf_transform<double>(fixture_path("all_fixed_branched.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::branched_kinematic_tree);
    CHECK(result.error().detail.find("left_mount") != std::string::npos);
}
