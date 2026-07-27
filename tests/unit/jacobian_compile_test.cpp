#include "../support/kinematics_helpers.h"
#include "../support/joint_limits_helpers.h"

#include "cartan/serial_chain.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <numbers>
#include <utility>
#include <type_traits>

namespace spp = cartan;
using Catch::Approx;

// ============================================================================
// The paired contract of every checked kinematics entry point
//
// Each entry point is asserted twice: the checked name answers with the
// expected instantiation over its value type and the chain failure, and the
// unchecked sibling answers with the plain value type. Either assertion alone
// would be satisfied by deleting the other function -- asserting only the
// checked type survives a deleted sibling, asserting only the unchecked type
// survives a reverted check.
//
// The configure-time gate covers the complementary failure: a *new* entry point
// added with no check at all. It counts declarations and cannot see a contract
// that regressed on an existing one, which is what these assertions catch.
//
// The three chain types below select the three distinct overload families:
// kinematic_chain, the compile-time-tagged static_chain, and any type meeting
// the chain concept.
// ============================================================================

using chain_runtime = spp::kinematic_chain<double, 3>;
using chain_tagged = spp::static_chain<double, spp::revolute_z, spp::revolute_z, spp::revolute_z>;
using chain_generic = spp::detail::generic_chain_wrapper<spp::kinematic_chain<double, spp::dynamic>>;

template <typename C> using scalar_of = typename C::scalar_type;
template <typename C> using q_of = typename spp::joint_state<scalar_of<C>, C::joints>::position_type;
template <typename C> using fk_of = spp::fk_result<scalar_of<C>, C::joints>;
template <typename C> using fk_mat_of = spp::fk_matrix_result<scalar_of<C>, C::joints>;
template <typename C> using jac_of = spp::jacobian_matrix<scalar_of<C>, C::joints>;
template <typename T> using checked_of = spp::expected<T, spp::chain_failure>;

template <typename C> using fk_checked = decltype(spp::forward_kinematics(std::declval<const C&>(), std::declval<const q_of<C>&>()));
template <typename C> using fk_plain = decltype(spp::forward_kinematics_unchecked(std::declval<const C&>(), std::declval<const q_of<C>&>()));
template <typename C> using fk_mat_checked = decltype(spp::forward_kinematics_matrix(std::declval<const C&>(), std::declval<const q_of<C>&>()));
template <typename C> using fk_mat_plain = decltype(spp::forward_kinematics_matrix_unchecked(std::declval<const C&>(), std::declval<const q_of<C>&>()));
template <typename C> using space_checked = decltype(spp::space_jacobian(std::declval<const C&>(), std::declval<const fk_of<C>&>()));
template <typename C> using space_plain = decltype(spp::space_jacobian_unchecked(std::declval<const C&>(), std::declval<const fk_of<C>&>()));
template <typename C> using body_checked = decltype(spp::body_jacobian(std::declval<const C&>(), std::declval<const fk_of<C>&>()));
template <typename C> using body_plain = decltype(spp::body_jacobian_unchecked(std::declval<const C&>(), std::declval<const fk_of<C>&>()));
template <typename C> using space_mat_checked = decltype(spp::space_jacobian(std::declval<const C&>(), std::declval<const fk_mat_of<C>&>()));
template <typename C> using space_mat_plain = decltype(spp::space_jacobian_unchecked(std::declval<const C&>(), std::declval<const fk_mat_of<C>&>()));
template <typename C> using velocity_checked = decltype(spp::end_effector_velocity(std::declval<const C&>(), std::declval<const q_of<C>&>(), std::declval<const q_of<C>&>()));
template <typename C> using velocity_plain = decltype(spp::end_effector_velocity_unchecked(std::declval<const C&>(), std::declval<const q_of<C>&>(), std::declval<const q_of<C>&>()));

static_assert(std::is_same_v<fk_checked<chain_runtime>, checked_of<fk_of<chain_runtime>>>);
static_assert(std::is_same_v<fk_plain<chain_runtime>, fk_of<chain_runtime>>);
static_assert(std::is_same_v<fk_checked<chain_tagged>, checked_of<fk_of<chain_tagged>>>);
static_assert(std::is_same_v<fk_plain<chain_tagged>, fk_of<chain_tagged>>);
static_assert(std::is_same_v<fk_checked<chain_generic>, checked_of<fk_of<chain_generic>>>);
static_assert(std::is_same_v<fk_plain<chain_generic>, fk_of<chain_generic>>);

static_assert(std::is_same_v<fk_mat_checked<chain_runtime>, checked_of<fk_mat_of<chain_runtime>>>);
static_assert(std::is_same_v<fk_mat_plain<chain_runtime>, fk_mat_of<chain_runtime>>);
static_assert(std::is_same_v<fk_mat_checked<chain_tagged>, checked_of<fk_mat_of<chain_tagged>>>);
static_assert(std::is_same_v<fk_mat_plain<chain_tagged>, fk_mat_of<chain_tagged>>);

static_assert(std::is_same_v<space_checked<chain_runtime>, checked_of<jac_of<chain_runtime>>>);
static_assert(std::is_same_v<space_plain<chain_runtime>, jac_of<chain_runtime>>);
static_assert(std::is_same_v<body_checked<chain_runtime>, checked_of<jac_of<chain_runtime>>>);
static_assert(std::is_same_v<body_plain<chain_runtime>, jac_of<chain_runtime>>);
static_assert(std::is_same_v<space_checked<chain_tagged>, checked_of<jac_of<chain_tagged>>>);
static_assert(std::is_same_v<space_plain<chain_tagged>, jac_of<chain_tagged>>);
static_assert(std::is_same_v<body_checked<chain_tagged>, checked_of<jac_of<chain_tagged>>>);
static_assert(std::is_same_v<body_plain<chain_tagged>, jac_of<chain_tagged>>);
static_assert(std::is_same_v<space_checked<chain_generic>, checked_of<jac_of<chain_generic>>>);
static_assert(std::is_same_v<space_plain<chain_generic>, jac_of<chain_generic>>);
static_assert(std::is_same_v<body_checked<chain_generic>, checked_of<jac_of<chain_generic>>>);
static_assert(std::is_same_v<body_plain<chain_generic>, jac_of<chain_generic>>);

static_assert(std::is_same_v<space_mat_checked<chain_runtime>, checked_of<jac_of<chain_runtime>>>);
static_assert(std::is_same_v<space_mat_plain<chain_runtime>, jac_of<chain_runtime>>);
static_assert(std::is_same_v<space_mat_checked<chain_tagged>, checked_of<jac_of<chain_tagged>>>);
static_assert(std::is_same_v<space_mat_plain<chain_tagged>, jac_of<chain_tagged>>);

static_assert(std::is_same_v<velocity_checked<chain_runtime>, checked_of<spp::vector6<double>>>);
static_assert(std::is_same_v<velocity_plain<chain_runtime>, spp::vector6<double>>);

// ============================================================================
// Helper: 3R planar robot (same as forward_kinematics_test)
// ============================================================================

static spp::kinematic_chain<double, 3> make_3r_chain()
{
    double L = 1.0;
    spp::vector3<double> home_trans;
    home_trans << 3 * L, 0, 0;
    auto home = spp::se3<double>(spp::so3<double>::identity(), home_trans);

    auto s1 = spp::screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s2 = spp::screw_axis<double>::revolute({0, 0, 1}, {L, 0, 0});
    auto s3 = spp::screw_axis<double>::revolute({0, 0, 1}, {2 * L, 0, 0});

    auto lim = spp::testing::limits(-std::numbers::pi, std::numbers::pi);

    return spp::kinematic_chain<double, 3>(
        home,
        {s1, s2, s3},
        {lim, lim, lim});
}

// ============================================================================
// jacobian_matrix type alias resolves correctly
// ============================================================================

TEST_CASE("jacobian_matrix fixed is 6xN", "[jacobian]")
{
    using jm = spp::jacobian_matrix<double, 3>;
    static_assert(jm::RowsAtCompileTime == 6);
    static_assert(jm::ColsAtCompileTime == 3);
    SUCCEED();
}

TEST_CASE("jacobian_matrix dynamic is 6xDynamic", "[jacobian]")
{
    using jm = spp::jacobian_matrix<double, spp::dynamic>;
    static_assert(jm::RowsAtCompileTime == 6);
    static_assert(jm::ColsAtCompileTime == Eigen::Dynamic);
    SUCCEED();
}

// ============================================================================
// space_jacobian basic API
// ============================================================================

TEST_CASE("space_jacobian returns 6x3 for 3R chain", "[jacobian]")
{
    auto chain = make_3r_chain();
    Eigen::Vector3d q = Eigen::Vector3d::Zero();
    auto fk = spp::testing::fk_at(chain, q);

    auto J_s = spp::testing::space_jacobian_at(chain, fk);

    REQUIRE(J_s.rows() == 6);
    REQUIRE(J_s.cols() == 3);
}

// ============================================================================
// body_jacobian basic API
// ============================================================================

TEST_CASE("body_jacobian returns 6x3 for 3R chain", "[jacobian]")
{
    auto chain = make_3r_chain();
    Eigen::Vector3d q = Eigen::Vector3d::Zero();
    auto fk = spp::testing::fk_at(chain, q);

    auto J_b = spp::testing::body_jacobian_at(chain, fk);

    REQUIRE(J_b.rows() == 6);
    REQUIRE(J_b.cols() == 3);
}

// ============================================================================
// end_effector_velocity basic API
// ============================================================================

TEST_CASE("end_effector_velocity at rest is the zero twist", "[velocity]")
{
    auto chain = make_3r_chain();
    Eigen::Vector3d q = Eigen::Vector3d::Zero();
    Eigen::Vector3d dq = Eigen::Vector3d::Zero();

    spp::vector6<double> vel = spp::testing::velocity_at(chain, q, dq);

    REQUIRE(vel.size() == 6);
    REQUIRE(vel.norm() < 1e-15);
}
