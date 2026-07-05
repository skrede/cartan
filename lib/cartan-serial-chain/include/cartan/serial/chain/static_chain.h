#ifndef HPP_GUARD_CARTAN_SERIAL_CHAIN_STATIC_CHAIN_H
#define HPP_GUARD_CARTAN_SERIAL_CHAIN_STATIC_CHAIN_H

/// Compile-time parameterized serial chain with joint type tags.
///
/// static_chain encodes joint types (revolute/prismatic and axis direction)
/// as template parameters while storing runtime link data (home pose, screw
/// axes, joint limits) in fixed-size arrays. This enables compile-time
/// dispatch and specialization in FK/Jacobian/IK while retaining full
/// runtime flexibility for link geometry.

#include "cartan/serial/chain/joint_kind.h"
#include "cartan/serial/chain/joint_tags.h"
#include "cartan/serial/chain/screw_axis.h"
#include "cartan/serial/chain/joint_limits.h"
#include "cartan/serial/chain/chain_concept.h"

#include "cartan/lie/se3.h"

#include <array>
#include <tuple>
#include <cassert>
#include <cstddef>
#include <utility>
#include <concepts>
#include <type_traits>

namespace cartan
{

namespace detail
{

/// Map a compile-time joint tag to the runtime joint_kind it describes. Used to
/// check that a static_chain's stored screw axes agree with their tags.
template <joint_tag Tag>
[[nodiscard]] constexpr joint_kind tag_joint_kind()
{
    if constexpr (std::same_as<Tag, revolute_x>) return joint_kind::revolute_x;
    else if constexpr (std::same_as<Tag, revolute_y>) return joint_kind::revolute_y;
    else if constexpr (std::same_as<Tag, revolute_z>) return joint_kind::revolute_z;
    else if constexpr (std::same_as<Tag, prismatic_x>) return joint_kind::prismatic_x;
    else if constexpr (std::same_as<Tag, prismatic_y>) return joint_kind::prismatic_y;
    else return joint_kind::prismatic_z;
}

}

/// Compile-time parameterized serial kinematic chain.
///
/// Joint types are encoded as template parameters via joint tags (revolute_x,
/// revolute_y, revolute_z, prismatic_x, prismatic_y, prismatic_z). Runtime
/// link data (home pose, screw axes, joint limits) is stored in fixed-size
/// std::array containers sized by the parameter pack.
template <typename Scalar, joint_tag... Joints>
class static_chain
{
    static_assert(std::is_floating_point_v<Scalar>,
        "static_chain requires a floating-point Scalar type");
    static_assert(sizeof...(Joints) > 0,
        "static_chain requires at least one joint");

public:
    using scalar_type = Scalar;
    static constexpr int joints = static_cast<int>(sizeof...(Joints));
    using limits_storage = std::array<joint_limits<Scalar>, sizeof...(Joints)>;
    using axes_storage = std::array<screw_axis<Scalar>, sizeof...(Joints)>;

    /// Construct a static chain from home configuration, screw axes, and limits.
    static_chain(
        const se3<Scalar>& home,
        axes_storage axes,
        limits_storage limits)
        : m_home(home)
        , m_axes(std::move(axes))
        , m_limits(std::move(limits))
    {
        assert(axes_match_tags(m_axes)
               && "static_chain screw axis contradicts its compile-time joint tag");
    }

    /// True when every stored screw axis classifies to the joint_kind implied
    /// by its compile-time tag. A tag/axis mismatch (e.g. a y-axis revolute
    /// screw under a revolute_z tag) would otherwise be silently mis-evaluated
    /// by the tag-dispatched FK/Jacobian fast paths, so the constructor asserts
    /// on this predicate in debug builds.
    [[nodiscard]] static bool axes_match_tags(const axes_storage& axes)
    {
        return [&]<std::size_t... Is>(std::index_sequence<Is...>)
        {
            using joint_tuple = std::tuple<Joints...>;
            return (... && (detect_joint_kind(axes[Is])
                            == detail::tag_joint_kind<
                                   std::tuple_element_t<Is, joint_tuple>>()));
        }(std::make_index_sequence<sizeof...(Joints)>{});
    }

    /// Home configuration (M matrix): end-effector pose at zero joint angles.
    [[nodiscard]] const se3<Scalar>& home() const { return m_home; }

    /// Number of joints in the chain.
    [[nodiscard]] int num_joints() const { return joints; }

    /// Access a single screw axis by index.
    [[nodiscard]] const screw_axis<Scalar>& axis(int i) const
    {
        return m_axes[static_cast<std::size_t>(i)];
    }

    /// Space-frame screw axes.
    [[nodiscard]] const axes_storage& axes() const { return m_axes; }

    /// Joint limits.
    [[nodiscard]] const limits_storage& limits() const { return m_limits; }

private:
    se3<Scalar> m_home;
    axes_storage m_axes;
    limits_storage m_limits;
};

static_assert(chain<static_chain<double, revolute_z, revolute_y, revolute_z>>,
    "static_chain must satisfy chain concept");

}

#endif
