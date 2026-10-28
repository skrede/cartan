#include <cartan/urdf.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <filesystem>

/// What the loader carries out of a description it accepts: the chain, the
/// names, the inertials and the per-joint velocity and effort bounds that live
/// beside it. These are assertions about the shipping loader alone -- nothing
/// here compares it against a second reader.
///
/// The last case is the exception, and it asserts an absence: one shape of
/// document is accepted that should be refused, and the refusal is not
/// recoverable inside this library. It is written as an assertion rather than
/// left as prose so that the day it is recoverable, this file goes red and the
/// paragraph below has to be deleted rather than left to rot.

using Catch::Approx;

namespace
{

std::filesystem::path fixture_path(const char* name)
{
    return std::filesystem::path{CARTAN_TESTS_FIXTURE_DIR} / "urdf" / name;
}

}

TEST_CASE("document: an accepted description carries its joints, names and inertials",
          "[urdf_document]")
{
    auto result = cartan::load_urdf<double>(fixture_path("parser_minimal.urdf"));

    REQUIRE(result.has_value());
    const auto& chain = result->chain;
    const auto& meta = result->metadata;

    REQUIRE(chain.num_joints() == 2);
    CHECK(meta.base_link_name == "base_link");
    CHECK(meta.tool_link_name == "link_2");
    REQUIRE(meta.joint_names.size() == 2);
    CHECK(meta.joint_names[0] == "joint_1");
    CHECK(meta.joint_names[1] == "joint_2");

    CHECK(chain.limits()[0].position_min() == Approx(-3.14159).margin(1e-9));
    CHECK(chain.limits()[0].position_max() == Approx(3.14159).margin(1e-9));
    CHECK(std::isinf(chain.limits()[1].position_min()));
    CHECK(std::isinf(chain.limits()[1].position_max()));

    REQUIRE(meta.velocity_max.size() == 2);
    REQUIRE(meta.velocity_max[0].has_value());
    CHECK(*meta.velocity_max[0] == Approx(2.0).margin(1e-12));
    REQUIRE(meta.effort_max[0].has_value());
    CHECK(*meta.effort_max[0] == Approx(50.0).margin(1e-12));
    CHECK_FALSE(meta.velocity_max[1].has_value());
    CHECK_FALSE(meta.effort_max[1].has_value());

    // link_2 declares no <inertial>, so the side-table holds two entries for
    // three links: an absent body is absent rather than defaulted to zero mass.
    REQUIRE(meta.link_inertials.size() == 2);
    CHECK(meta.link_inertials[0].link_name == "base_link");
    CHECK(meta.link_inertials[0].mass == Approx(1.0).margin(1e-12));
    CHECK(meta.link_inertials[0].inertia(0, 0) == Approx(0.1).margin(1e-12));
    CHECK(meta.link_inertials[0].inertia(2, 2) == Approx(0.1).margin(1e-12));
    CHECK(meta.link_inertials[1].link_name == "link_1");
    CHECK(meta.link_inertials[1].mass == Approx(2.5).margin(1e-12));
    CHECK(meta.link_inertials[1].com(2) == Approx(0.25).margin(1e-12));
}

TEST_CASE("document: a continuous joint is unbounded at both scalars", "[urdf_document]")
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
    CHECK(std::isinf(as_float->chain.limits()[2].position_min()));
    CHECK(std::isinf(as_float->chain.limits()[2].position_max()));
}

/// A <limit> that declares velocity and effort but neither lower nor upper.
/// The description reader's limit record is four plain numbers, and an absent
/// attribute leaves its field at zero, so the bound the document never wrote
/// arrives indistinguishable from one written as zero. On a continuous joint
/// that is invisible, because the builder substitutes infinities and never
/// reads the staged pair. On a revolute joint it is not: the builder's refusal
/// of an unset bound cannot fire, joint_limits accepts the degenerate interval,
/// and the caller receives a chain whose joint cannot move, with no diagnostic
/// anywhere.
///
/// Nothing in this library can recover the distinction -- the information is
/// gone before the value is staged -- and a rule refusing every [0, 0] interval
/// would also refuse a document that legitimately declares one. The request for
/// the missing optionality sits upstream. So this asserts the pinned joint
/// exactly: the day the distinction survives the read, the load below fails
/// instead and this case goes red.
TEST_CASE("document: an undeclared bound pins a revolute joint and reports nothing",
          "[urdf_document]")
{
    auto loaded = cartan::load_urdf<double>(fixture_path("partial_limit_revolute.urdf"));

    REQUIRE(loaded.has_value());
    REQUIRE(loaded->chain.num_joints() == 1);
    CHECK(loaded->chain.limits()[0].position_min() == 0.0);
    CHECK(loaded->chain.limits()[0].position_max() == 0.0);
    for (const auto& record : loaded->diagnostics)
    {
        CHECK(record.severity != cartan::urdf_severity::error);
    }
}
