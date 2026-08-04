#include <cartan/urdf.h>

#include <cartan/urdf/detail/narrowing.h>

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <filesystem>

/// The loader is the library's only untrusted-input surface, and these are the
/// documents it must refuse: a joint naming a link nobody declared, a topology
/// that is not a tree, a name declared twice, a number the chain's scalar type
/// cannot carry. Every case asserts a typed refusal, and every refusal that can
/// name the offending joint or link is asserted to name it -- a rejection whose
/// message says only "invalid" costs the caller the same work as no rejection.
///
/// Each fixture is chosen so it clears every gate except the one under test.
/// The two cases that do not read a document are here because they are the two
/// halves of the same boundary: what the loader refuses to narrow, and what the
/// construction spelling beneath it still lets through.

namespace
{

std::filesystem::path fixture_path(const char* name)
{
    return std::filesystem::path{CARTAN_TESTS_FIXTURE_DIR} / "urdf" / name;
}

}

TEST_CASE("adversarial: a joint naming an undeclared parent link is refused", "[urdf_adversarial]")
{
    auto result = cartan::load_urdf<double>(fixture_path("parser_unknown_parent.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::unknown_parent_link);
    CHECK(result.error().detail.find("ghost_link") != std::string::npos);
}

/// The self-loop joint makes link_a the child of two joints -- its own and the
/// root's -- so it surfaces as a non-tree topology rather than as a cycle. The
/// kind is the reader's classification of the same document, and it is the more
/// precise of the two: the walk never runs, because the link set is already not
/// a tree.
TEST_CASE("adversarial: a self-loop joint is refused as a non-tree topology", "[urdf_adversarial]")
{
    auto result = cartan::load_urdf<double>(fixture_path("adversarial_self_loop.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::multi_parent_link);
    CHECK(result.error().detail.find("link_a") != std::string::npos);
}

TEST_CASE("adversarial: a kinematic cycle is refused", "[urdf_adversarial]")
{
    auto result = cartan::load_urdf<double>(fixture_path("adversarial_cycle.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::multi_parent_link);
    CHECK(result.error().detail.find("link_a") != std::string::npos);
}

TEST_CASE("adversarial: a link with two parent joints is refused", "[urdf_adversarial]")
{
    auto result = cartan::load_urdf<double>(fixture_path("adversarial_multi_parent.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::multi_parent_link);
    CHECK(result.error().detail.find("tip") != std::string::npos);
}

TEST_CASE("adversarial: a duplicate link name is refused", "[urdf_adversarial]")
{
    auto result = cartan::load_urdf<double>(fixture_path("adversarial_duplicate_name.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::duplicate_name);
    CHECK(result.error().detail.find("link_1") != std::string::npos);
}

/// The refusal names the attribute and the line rather than the joint, which is
/// enough to find it in the document and is asserted as what it is.
TEST_CASE("adversarial: a numeric attribute that reads as NaN is refused", "[urdf_adversarial]")
{
    auto result = cartan::load_urdf<double>(fixture_path("adversarial_non_finite.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::non_finite_value);
    CHECK(result.error().detail.find("lower") != std::string::npos);
    REQUIRE(result.error().location.has_value());
    CHECK(result.error().location->line == 9);
}

TEST_CASE("adversarial: a reversed <limit> is refused", "[urdf_adversarial]")
{
    auto result = cartan::load_urdf<double>(fixture_path("reversed_limit.urdf"));

    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().kind == cartan::urdf_failure::invalid_joint_limit);
    CHECK(result.error().detail.find("reversed_joint") != std::string::npos);
}

/// An <axis> magnitude whose square overflows normalizes to the zero vector,
/// not to a NaN, so the zero-axis guard and every finiteness test downstream
/// pass while the joint contributes identity to the kinematics forever. The
/// squaring is what overflows, so this is not a single-precision problem:
/// 1e200 does it in double.
TEST_CASE("adversarial: an <axis> whose squared magnitude overflows is refused at that "
          "scalar only", "[urdf_adversarial]")
{
    const auto path = fixture_path("axis_overflow.urdf");

    auto as_double = cartan::load_urdf<double>(path);
    REQUIRE(as_double.has_value());
    CHECK(as_double->chain.axes()[0].is_revolute());

    auto as_float = cartan::load_urdf<float>(path);
    REQUIRE_FALSE(as_float.has_value());
    CHECK(as_float.error().kind == cartan::urdf_failure::non_finite_value);
    CHECK(as_float.error().detail.find("overflow_axis_joint") != std::string::npos);
}

/// The residual, asserted so it is not misread as closed. The construction
/// spelling stays deliberately unvalidated, so a chain assembled in code rather
/// than loaded from a description can still hold an axis that is finite, zero,
/// and no longer a joint. Neither input is a NaN: a zero vector and an
/// overflowing one both take a branch of normalized() that never divides badly.
TEST_CASE("adversarial: the unvalidated construction spelling still yields a dead axis",
          "[urdf_adversarial]")
{
    cartan::vector3<float> huge;
    huge << 1e30f, 0, 0;
    const auto overflowed = cartan::screw_axis<float>::revolute(huge, {0, 0, 0});
    CHECK(overflowed.to_vector().allFinite());
    CHECK_FALSE(overflowed.is_revolute());
    CHECK(overflowed.omega().norm() == 0.0f);

    cartan::vector3<double> huge_d;
    huge_d << 1e200, 0, 0;
    CHECK_FALSE(cartan::screw_axis<double>::revolute(huge_d, {0, 0, 0}).is_revolute());
}

/// The narrowing step is the only place a caller asking for a scalar narrower
/// than the reader's double can be told which value that scalar cannot hold,
/// and it has to refuse in both directions. A bound that overflows becomes an
/// infinity indistinguishable from the one the builder writes for a continuous
/// joint; a nonzero bound that collapses to exactly zero pins the joint it
/// bounds. Both are unrecoverable once the cast has happened, so both are
/// refused before the value is handed on.
TEST_CASE("adversarial: a value the narrower scalar cannot carry is refused in either "
          "direction", "[urdf_adversarial]")
{
    float narrow = 42.0f;
    CHECK(cartan::detail::narrow_into<float>(1e300, narrow).has_value());
    CHECK(cartan::detail::narrow_into<float>(-1e300, narrow).has_value());
    CHECK(cartan::detail::narrow_into<float>(1e-320, narrow).has_value());
    CHECK(narrow == 42.0f);

    CHECK_FALSE(cartan::detail::narrow_into<float>(0.0, narrow).has_value());
    CHECK(narrow == 0.0f);
    CHECK_FALSE(cartan::detail::narrow_into<float>(0.25, narrow).has_value());
    CHECK(narrow == 0.25f);

    double wide = 0.0;
    CHECK_FALSE(cartan::detail::narrow_into<double>(1e300, wide).has_value());
    CHECK_FALSE(cartan::detail::narrow_into<double>(1e-320, wide).has_value());
}
