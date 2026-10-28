// The unchecked kinematics entry points answer the right value, asserted
// against a closed form rather than against their own checked wrapper.
//
// jacobian_compile_test pins the type of each checked-and-unchecked pair. A
// value assertion cannot be built by comparing the two: every checked entry
// point delegates to its unchecked sibling, so a wrong sibling moves both sides
// and the comparison holds. The oracle below is the planar-3R closed form and
// shares no code with the library.
//
// The fixture is the unit-link planar 3R: three +z axes through x = 0, 1, 2 and
// a home end-effector at (3, 0, 0), so the end-effector traces
// p = (sum cos, sum sin) over the cumulative joint angles.

#include "../support/kinematics_helpers.h"
#include "../support/static_chain_factories.h"

#include "../fixtures/chain_factories.h"

#include <cartan/serial/fk/velocity.h>
#include <cartan/serial/chain/chain_concept.h>
#include <cartan/serial/fk/forward_kinematics.h>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>

namespace spp = cartan;

namespace
{

using dynamic_chain = spp::kinematic_chain<double, spp::dynamic>;

std::array<double, 3> cumulative(const Eigen::Vector3d& q)
{
    return {q(0), q(0) + q(1), q(0) + q(1) + q(2)};
}

Eigen::Vector2d closed_form_position(const Eigen::Vector3d& q)
{
    const auto a = cumulative(q);
    return {std::cos(a[0]) + std::cos(a[1]) + std::cos(a[2]),
            std::sin(a[0]) + std::sin(a[1]) + std::sin(a[2])};
}

/// V_s = (omega, v) with v = p_dot - omega x p, omega along +z for a planar arm.
spp::vector6<double> closed_form_space_twist(
    const Eigen::Vector3d& q, const Eigen::Vector3d& dq)
{
    const auto a = cumulative(q);
    const auto da = cumulative(dq);
    const double omega = da[2];
    const Eigen::Vector2d p = closed_form_position(q);

    double px_dot = 0.0;
    double py_dot = 0.0;
    for (int i = 0; i < 3; ++i)
    {
        px_dot -= std::sin(a[static_cast<std::size_t>(i)]) * da[static_cast<std::size_t>(i)];
        py_dot += std::cos(a[static_cast<std::size_t>(i)]) * da[static_cast<std::size_t>(i)];
    }

    spp::vector6<double> twist;
    twist << 0.0, 0.0, omega, px_dot + omega * p.y(), py_dot - omega * p.x(), 0.0;
    return twist;
}

Eigen::Vector3d probe_configuration()
{
    Eigen::Vector3d q;
    q << 0.3, -0.5, 0.7;
    return q;
}

}

TEST_CASE("unchecked forward kinematics matches the closed form on all three chain forms",
    "[fk][oracle]")
{
    auto chain = spp::fixtures::make_3r_planar_chain<double>();
    auto tagged = spp::testing::make_3r_planar_static<double>();
    auto dynamic = chain.to_dynamic();
    spp::detail::generic_chain_wrapper<dynamic_chain> wrapped{dynamic};

    const Eigen::Vector3d q = probe_configuration();
    const Eigen::VectorXd q_dyn = q;
    const Eigen::Vector2d expected = closed_form_position(q);

    REQUIRE((spp::forward_kinematics_unchecked(chain, q).end_effector.translation().head<2>()
             - expected).norm() < 1e-12);
    REQUIRE((spp::forward_kinematics_unchecked(tagged, q).end_effector.translation().head<2>()
             - expected).norm() < 1e-12);
    REQUIRE((spp::forward_kinematics_unchecked(wrapped, q_dyn).end_effector.translation().head<2>()
             - expected).norm() < 1e-12);
    REQUIRE((spp::testing::fk_at(chain, q).end_effector.translation().head<2>()
             - expected).norm() < 1e-12);
}

TEST_CASE("unchecked end-effector velocity matches the closed-form space twist",
    "[velocity][oracle]")
{
    auto chain = spp::fixtures::make_3r_planar_chain<double>();

    const Eigen::Vector3d q = probe_configuration();
    Eigen::Vector3d dq;
    dq << 0.2, 0.1, -0.4;

    const spp::vector6<double> expected = closed_form_space_twist(q, dq);
    REQUIRE(expected.norm() > 0.5);

    REQUIRE((spp::end_effector_velocity_unchecked(chain, q, dq) - expected).norm() < 1e-12);
    REQUIRE((spp::testing::velocity_at(chain, q, dq) - expected).norm() < 1e-12);
}
