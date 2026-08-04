#include "../support/urdf_stage_parity.h"

#include <cartan/urdf.h>

#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <iostream>
#include <filesystem>
#include <string_view>

/// Exact equality of the model the two front halves stage from the same
/// document -- every scalar the chain extractor consumes, compared by bit
/// pattern, in both scalar instantiations.
///
/// This is the stronger and the more diagnosable of the two parity assertions
/// this suite makes. The forward-kinematics agreement in urdf_parity_test.cpp
/// compares the end of the pipeline within a tolerance; this compares the
/// middle of it exactly, and so names the joint and the field a difference came
/// from rather than reporting that a pose moved. Everything downstream of the
/// staging step is shared by both halves, which is what makes a comparison here
/// sufficient evidence about the whole pipeline.
///
/// The synthetic fixtures always run; the vendored real-world descriptions
/// compile in only under CARTAN_URDF_EXTENDED_TESTS.

namespace
{

namespace fs = std::filesystem;

template <typename Scalar>
void check_staged_parity(std::string_view instantiation)
{
    auto tallies = cartan::testing::compare_fixture_set<Scalar>();
    const std::size_t scalars = cartan::testing::report_tallies(std::cout, instantiation, tallies);
    INFO(instantiation << " instantiation over " << tallies.size() << " fixtures");
    CHECK(scalars >= cartan::testing::staged_value_floor);
}

}

TEST_CASE("urdf stage parity: the two front halves stage bit-identical values in double",
          "[urdf_parity]")
{
    check_staged_parity<double>("double");
}

TEST_CASE("urdf stage parity: the two front halves stage bit-identical values in float",
          "[urdf_parity]")
{
    check_staged_parity<float>("float");
}

/// What the one difference between the two halves costs on a joint that is not
/// continuous. The in-tree parser keeps "the document declared no lower bound"
/// and the extractor refuses the joint over it; the reader's limit record holds
/// four plain numbers and cannot carry the distinction, so the same document
/// yields a joint silently pinned to [0, 0] and no refusal at all. No other
/// fixture has this shape, which is why it is asserted here rather than
/// inferred from the difference set.
TEST_CASE("urdf stage parity: an undeclared bound pins a revolute joint on one path only",
          "[urdf_parity]")
{
    const fs::path urdf_path =
        fs::path{CARTAN_TESTS_FIXTURE_DIR} / "urdf" / "partial_limit_revolute.urdf";

    auto parsed = cartan::parse_urdf_file<double>(urdf_path);
    REQUIRE(parsed.has_value());
    REQUIRE(parsed->joints.size() == 1);
    CHECK_FALSE(parsed->joints[0].position_min.has_value());
    auto extracted = cartan::build_chain<double>(*parsed);
    REQUIRE_FALSE(extracted.has_value());
    CHECK(extracted.error().kind == cartan::urdf_failure::missing_joint_limit);

    auto loaded = cartan::load_urdf<double>(urdf_path);
    REQUIRE(loaded.has_value());
    CHECK(loaded->chain.limits()[0].position_min() == 0.0);
    CHECK(loaded->chain.limits()[0].position_max() == 0.0);
}
