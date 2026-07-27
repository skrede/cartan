#include <cartan/serial/chain/static_chain.h>
#include <cartan/serial/chain/joint_tags.h>
#include <cartan/serial/chain/screw_axis.h>
#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/chain_concept.h>
#include <cartan/serial/chain/chain_failure.h>
#include <cartan/serial/chain/joint_state.h>
#include <cartan/serial/chain/kinematic_chain.h>
#include <cartan/serial/fk/jacobian.h>
#include <cartan/serial/fk/forward_kinematics.h>

#include <cartan/lie/se3.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>

#include <array>
#include <limits>
#include <string>
#include <cstddef>
#include <numbers>

using Catch::Approx;

namespace
{

template <typename Scalar>
cartan::joint_limits<Scalar> unwrap_limits(Scalar lo, Scalar hi)
{
    auto made = cartan::joint_limits<Scalar>::make(lo, hi);
    REQUIRE(made.has_value());
    return *made;
}

template <typename Scalar>
cartan::se3<Scalar> nonfinite_home()
{
    return cartan::se3<Scalar>(
        cartan::so3<Scalar>::identity(),
        cartan::vector3<Scalar>(
            std::numeric_limits<Scalar>::quiet_NaN(), Scalar(0), Scalar(0)));
}

using chain_3r = cartan::static_chain<double,
    cartan::revolute_z, cartan::revolute_y, cartan::revolute_z>;

std::array<cartan::screw_axis<double>, 3> make_3r_screw_axes()
{
    using namespace cartan;
    auto s0 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s1 = screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0.5});
    auto s2 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 1.0});
    return std::array<screw_axis<double>, 3>{s0, s1, s2};
}

cartan::se3<double> make_3r_home()
{
    using namespace cartan;
    return se3<double>(so3<double>::identity(), vector3<double>{1.0, 0.0, 1.0});
}

std::array<cartan::joint_limits<double>, 3> make_3r_limits()
{
    auto lim = unwrap_limits<double>(-std::numbers::pi, std::numbers::pi);
    return std::array<cartan::joint_limits<double>, 3>{lim, lim, lim};
}

chain_3r make_3r_static_chain()
{
    auto made = chain_3r::make(make_3r_home(), make_3r_screw_axes(), make_3r_limits());
    REQUIRE(made.has_value());
    return *made;
}

}

// ============================================================================
// Compile-time joint count
// ============================================================================

TEST_CASE("static_chain joints count is compile-time", "[static_chain]")
{
    static_assert(chain_3r::joints == 3);
    SUCCEED();
}

// ============================================================================
// Single-joint construction
// ============================================================================

TEST_CASE("static_chain single revolute joint", "[static_chain]")
{
    using namespace cartan;

    auto s0 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto lim = unwrap_limits<double>(-std::numbers::pi, std::numbers::pi);

    auto made = static_chain<double, revolute_z>::make(
        se3<double>::identity(), {s0}, {lim});
    REQUIRE(made.has_value());

    REQUIRE(made->num_joints() == 1);
    REQUIRE(made->joints == 1);
}

// ============================================================================
// Three-joint accessor correctness
// ============================================================================

TEST_CASE("static_chain 3R axis accessors", "[static_chain]")
{
    auto axes = make_3r_screw_axes();
    auto sc = make_3r_static_chain();

    REQUIRE((sc.axis(0).to_vector() - axes[0].to_vector()).norm() < 1e-14);
    REQUIRE((sc.axis(1).to_vector() - axes[1].to_vector()).norm() < 1e-14);
    REQUIRE((sc.axis(2).to_vector() - axes[2].to_vector()).norm() < 1e-14);
}

// ============================================================================
// axes() returns iterable range of correct length
// ============================================================================

TEST_CASE("static_chain axes() range", "[static_chain]")
{
    auto expected_axes = make_3r_screw_axes();
    auto sc = make_3r_static_chain();

    const auto& axes = sc.axes();
    REQUIRE(axes.size() == 3);

    REQUIRE((axes[0].to_vector() - expected_axes[0].to_vector()).norm() < 1e-14);
    REQUIRE((axes[1].to_vector() - expected_axes[1].to_vector()).norm() < 1e-14);
    REQUIRE((axes[2].to_vector() - expected_axes[2].to_vector()).norm() < 1e-14);
}

// ============================================================================
// home() returns the provided SE3 pose
// ============================================================================

TEST_CASE("static_chain home() returns construction pose", "[static_chain]")
{
    using namespace cartan;

    vector3<double> t{1.0, 2.0, 3.0};
    auto s0 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto lim = unwrap_limits<double>(-std::numbers::pi, std::numbers::pi);

    auto made = static_chain<double, revolute_z>::make(
        se3<double>(so3<double>::identity(), t), {s0}, {lim});
    REQUIRE(made.has_value());

    REQUIRE((made->home().translation() - t).norm() < 1e-14);
}

// ============================================================================
// limits() returns the provided joint limits
// ============================================================================

TEST_CASE("static_chain limits() returns construction limits", "[static_chain]")
{
    using namespace cartan;

    auto s0 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s1 = screw_axis<double>::revolute({0, 1, 0}, {0, 0, 0.5});
    auto lim0 = unwrap_limits<double>(-1.0, 1.0);
    auto lim1 = unwrap_limits<double>(-2.0, 2.0);

    auto made = static_chain<double, revolute_z, revolute_y>::make(
        se3<double>::identity(), {s0, s1}, {lim0, lim1});
    REQUIRE(made.has_value());

    const auto& limits = made->limits();
    REQUIRE(limits.size() == 2);
    REQUIRE(limits[0].position_min() == Approx(-1.0));
    REQUIRE(limits[0].position_max() == Approx(1.0));
    REQUIRE(limits[1].position_min() == Approx(-2.0));
    REQUIRE(limits[1].position_max() == Approx(2.0));
}

// ============================================================================
// Concept satisfaction
// ============================================================================

TEST_CASE("static_chain satisfies chain concept", "[static_chain]")
{
    static_assert(cartan::chain<chain_3r>);
    SUCCEED();
}

// ============================================================================
// Prismatic joint tag
// ============================================================================

TEST_CASE("static_chain with prismatic joint tag", "[static_chain]")
{
    using namespace cartan;

    auto s0 = screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s1 = screw_axis<double>::prismatic({0, 0, 1});
    auto lim = unwrap_limits<double>(-10.0, 10.0);

    auto made = static_chain<double, revolute_z, prismatic_z>::make(
        se3<double>::identity(), {s0, s1}, {lim, lim});
    REQUIRE(made.has_value());

    REQUIRE(made->num_joints() == 2);
    REQUIRE(made->axis(0).is_revolute());
    REQUIRE(made->axis(1).is_prismatic());
}

// ============================================================================
// The factory's tag-versus-axis verdict, at both scalars
// ============================================================================

TEMPLATE_TEST_CASE("static_chain::make accepts either sign of a tag's axis",
    "[static_chain][tag_axis]", double, float)
{
    using namespace cartan;
    using Scalar = TestType;

    auto lim = unwrap_limits<Scalar>(Scalar(-1), Scalar(1));

    auto positive = screw_axis<Scalar>::revolute({0, 0, 1}, vector3<Scalar>::Zero());
    auto negative = screw_axis<Scalar>::revolute({0, 0, -1}, vector3<Scalar>::Zero());

    using two_z = static_chain<Scalar, revolute_z, revolute_z>;

    REQUIRE(two_z::make(se3<Scalar>::identity(), {positive, positive}, {lim, lim})
                .has_value());
    REQUIRE(two_z::make(se3<Scalar>::identity(), {negative, negative}, {lim, lim})
                .has_value());
    REQUIRE(two_z::make(se3<Scalar>::identity(), {positive, negative}, {lim, lim})
                .has_value());
}

// The vendored KR6 description really does carry `axis: 0 0 -1` on its first
// joint and `axis: -1 0 0` on two more, so a sign-rejecting check would make
// that robot inexpressible as a static chain rather than make it safer.
TEMPLATE_TEST_CASE("static_chain::make keeps a real negative-axis robot expressible",
    "[static_chain][tag_axis]", double, float)
{
    using namespace cartan;
    using Scalar = TestType;

    auto lim = unwrap_limits<Scalar>(Scalar(-3), Scalar(3));

    auto a1 = screw_axis<Scalar>::revolute(
        {0, 0, -1}, vector3<Scalar>(Scalar(0), Scalar(0), Scalar(0)));
    auto a2 = screw_axis<Scalar>::revolute(
        {0, 1, 0}, vector3<Scalar>(Scalar(0.025), Scalar(0), Scalar(0.400)));
    auto a4 = screw_axis<Scalar>::revolute(
        {-1, 0, 0}, vector3<Scalar>(Scalar(0.480), Scalar(0), Scalar(0.435)));

    auto made = static_chain<Scalar, revolute_z, revolute_y, revolute_x>::make(
        se3<Scalar>::identity(), {a1, a2, a4}, {lim, lim, lim});
    REQUIRE(made.has_value());
}

TEMPLATE_TEST_CASE("static_chain::make rejects an axis about another principal direction",
    "[static_chain][tag_axis]", double, float)
{
    using namespace cartan;
    using Scalar = TestType;

    auto lim = unwrap_limits<Scalar>(Scalar(-1), Scalar(1));
    auto about_y = screw_axis<Scalar>::revolute({0, 1, 0}, vector3<Scalar>::Zero());

    auto made = static_chain<Scalar, revolute_z>::make(
        se3<Scalar>::identity(), {about_y}, {lim});

    REQUIRE_FALSE(made.has_value());
    REQUIRE(made.error() == chain_failure::tag_axis_contradiction);
}

// A near-principal axis is a contradiction too: the tag is a compile-time claim
// about an axis the same caller supplies, so there is nothing for a tolerance to
// absorb. One epsilon of deviation is the sharpest form of the case.
TEMPLATE_TEST_CASE("static_chain::make rejects a near-principal axis at one epsilon",
    "[static_chain][tag_axis]", double, float)
{
    using namespace cartan;
    using Scalar = TestType;

    auto lim = unwrap_limits<Scalar>(Scalar(-1), Scalar(1));
    Scalar eps = std::numeric_limits<Scalar>::epsilon();
    auto tilted = screw_axis<Scalar>::revolute(
        vector3<Scalar>(eps, Scalar(0), Scalar(1)), vector3<Scalar>::Zero());

    REQUIRE(tilted.omega()(0) != Scalar(0));

    auto made = static_chain<Scalar, revolute_z>::make(
        se3<Scalar>::identity(), {tilted}, {lim});

    REQUIRE_FALSE(made.has_value());
    REQUIRE(made.error() == chain_failure::tag_axis_contradiction);
}

TEMPLATE_TEST_CASE("static_chain::make rejects a prismatic direction under a revolute tag",
    "[static_chain][tag_axis]", double, float)
{
    using namespace cartan;
    using Scalar = TestType;

    auto lim = unwrap_limits<Scalar>(Scalar(-1), Scalar(1));
    auto slide = screw_axis<Scalar>::prismatic({0, 0, 1});
    auto turn = screw_axis<Scalar>::revolute({0, 0, 1}, vector3<Scalar>::Zero());

    auto under_revolute = static_chain<Scalar, revolute_z>::make(
        se3<Scalar>::identity(), {slide}, {lim});
    REQUIRE_FALSE(under_revolute.has_value());
    REQUIRE(under_revolute.error() == chain_failure::tag_axis_contradiction);

    auto under_prismatic = static_chain<Scalar, prismatic_z>::make(
        se3<Scalar>::identity(), {turn}, {lim});
    REQUIRE_FALSE(under_prismatic.has_value());
    REQUIRE(under_prismatic.error() == chain_failure::tag_axis_contradiction);
}

TEMPLATE_TEST_CASE("static_chain::make rejects a nonfinite axis or home pose",
    "[static_chain][tag_axis]", double, float)
{
    using namespace cartan;
    using Scalar = TestType;

    auto lim = unwrap_limits<Scalar>(Scalar(-1), Scalar(1));
    auto good = screw_axis<Scalar>::revolute({0, 0, 1}, vector3<Scalar>::Zero());

    for (Scalar poison : {std::numeric_limits<Scalar>::quiet_NaN(),
                          std::numeric_limits<Scalar>::infinity(),
                          -std::numeric_limits<Scalar>::infinity()})
    {
        auto poisoned = screw_axis<Scalar>::revolute(
            vector3<Scalar>(Scalar(0), Scalar(0), Scalar(1)),
            vector3<Scalar>(poison, Scalar(0), Scalar(0)));

        auto bad_axis = static_chain<Scalar, revolute_z>::make(
            se3<Scalar>::identity(), {poisoned}, {lim});
        REQUIRE_FALSE(bad_axis.has_value());
        REQUIRE(bad_axis.error() == chain_failure::non_finite_input);
    }

    auto bad_home = static_chain<Scalar, revolute_z>::make(
        nonfinite_home<Scalar>(), {good}, {lim});
    REQUIRE_FALSE(bad_home.has_value());
    REQUIRE(bad_home.error() == chain_failure::non_finite_input);
}

// The verdict is a returned value, so it cannot differ between a build that
// defines NDEBUG and one that does not. This case is the assertion that the
// outcome is a value at all -- the property the previous debug-only assert,
// which was compiled out of every release build, did not have.
TEMPLATE_TEST_CASE("static_chain::make reports a contradiction as a value, not an abort",
    "[static_chain][tag_axis]", double, float)
{
    using namespace cartan;
    using Scalar = TestType;

    auto lim = unwrap_limits<Scalar>(Scalar(-1), Scalar(1));
    auto about_y = screw_axis<Scalar>::revolute({0, 1, 0}, vector3<Scalar>::Zero());

    auto made = static_chain<Scalar, revolute_z>::make(
        se3<Scalar>::identity(), {about_y}, {lim});

    REQUIRE_FALSE(made.has_value());
    REQUIRE(message(made.error())
            == std::string("Screw axis contradicts its compile-time joint tag"));
}

// ============================================================================
// FK equivalence: static_chain vs kinematic_chain
// ============================================================================

namespace
{

std::array<Eigen::Vector3d, 5> sample_configurations()
{
    std::array<Eigen::Vector3d, 5> configs;
    configs[0] = Eigen::Vector3d::Zero();
    configs[1] = Eigen::Vector3d{0.5, -0.3, 0.8};
    configs[2] = Eigen::Vector3d{-1.0, 1.2, -0.7};
    configs[3] = Eigen::Vector3d{
        std::numbers::pi / 4, std::numbers::pi / 3, -std::numbers::pi / 6};
    configs[4] = Eigen::Vector3d{2.1, -1.5, 0.3};
    return configs;
}

}

TEST_CASE("FK on static_chain matches kinematic_chain", "[static_chain][fk]")
{
    using namespace cartan;

    auto sc = make_3r_static_chain();
    kinematic_chain<double, 3> kc(
        make_3r_home(), make_3r_screw_axes(), make_3r_limits());

    for (const auto& q : sample_configurations())
    {
        auto fk_sc = forward_kinematics(sc, q);
        auto fk_kc = forward_kinematics(kc, q);
        REQUIRE(fk_sc.has_value());
        REQUIRE(fk_kc.has_value());

        REQUIRE((fk_sc->end_effector.matrix() - fk_kc->end_effector.matrix()).norm()
                < 1e-12);

        for (std::size_t i = 0; i < 3; ++i)
        {
            REQUIRE((fk_sc->intermediates[i].matrix()
                     - fk_kc->intermediates[i].matrix()).norm() < 1e-12);
        }
    }
}

// ============================================================================
// Space Jacobian equivalence: static_chain vs kinematic_chain
// ============================================================================

TEST_CASE("Space Jacobian on static_chain matches kinematic_chain",
    "[static_chain][jacobian]")
{
    using namespace cartan;

    auto sc = make_3r_static_chain();
    kinematic_chain<double, 3> kc(
        make_3r_home(), make_3r_screw_axes(), make_3r_limits());

    for (const auto& q : sample_configurations())
    {
        auto fk_sc = forward_kinematics(sc, q);
        auto fk_kc = forward_kinematics(kc, q);
        REQUIRE(fk_sc.has_value());
        REQUIRE(fk_kc.has_value());

        auto J_sc = space_jacobian(sc, *fk_sc);
        auto J_kc = space_jacobian(kc, *fk_kc);
        REQUIRE(J_sc.has_value());
        REQUIRE(J_kc.has_value());

        REQUIRE((*J_sc - *J_kc).norm() < 1e-12);
    }
}

// ============================================================================
// Body Jacobian equivalence: static_chain vs kinematic_chain
// ============================================================================

TEST_CASE("Body Jacobian on static_chain matches kinematic_chain",
    "[static_chain][jacobian]")
{
    using namespace cartan;

    auto sc = make_3r_static_chain();
    kinematic_chain<double, 3> kc(
        make_3r_home(), make_3r_screw_axes(), make_3r_limits());

    for (const auto& q : sample_configurations())
    {
        auto fk_sc = forward_kinematics(sc, q);
        auto fk_kc = forward_kinematics(kc, q);
        REQUIRE(fk_sc.has_value());
        REQUIRE(fk_kc.has_value());

        auto Jb_sc = body_jacobian(sc, *fk_sc);
        auto Jb_kc = body_jacobian(kc, *fk_kc);
        REQUIRE(Jb_sc.has_value());
        REQUIRE(Jb_kc.has_value());

        REQUIRE((*Jb_sc - *Jb_kc).norm() < 1e-12);
    }
}
