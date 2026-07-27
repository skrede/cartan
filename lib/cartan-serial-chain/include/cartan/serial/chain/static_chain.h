#ifndef HPP_GUARD_CARTAN_SERIAL_CHAIN_STATIC_CHAIN_H
#define HPP_GUARD_CARTAN_SERIAL_CHAIN_STATIC_CHAIN_H

/// Compile-time parameterized serial chain with joint type tags.
///
/// static_chain encodes joint types (revolute/prismatic and axis direction)
/// as template parameters while storing runtime link data (home pose, screw
/// axes, joint limits) in fixed-size arrays. This enables compile-time
/// dispatch and specialization in FK/Jacobian/IK while retaining full
/// runtime flexibility for link geometry.

#include "cartan/serial/chain/joint_tags.h"
#include "cartan/serial/chain/screw_axis.h"
#include "cartan/serial/chain/chain_concept.h"
#include "cartan/serial/chain/chain_failure.h"
#include "cartan/serial/chain/joint_limits.h"

#include "cartan/serial/chain/detail/tag_axis_check.h"

#include "cartan/lie/se3.h"

#include "cartan/expected.h"

#include <array>
#include <cstddef>
#include <utility>
#include <type_traits>

namespace cartan
{

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

    /// Validated construction, the only path to a static_chain value.
    ///
    /// An axis that contradicts its tag is refused as a value rather than as a
    /// debug assert: the tag-dispatched evaluation reads the component the tag
    /// names as the signed magnitude, so a y-axis screw under a revolute_z tag
    /// silently freezes that joint, and an assert reporting it would vanish
    /// under the NDEBUG a release build defines. Finiteness is checked here
    /// because the normalizing screw-axis factories are deliberately
    /// unvalidated and a chain is the first place that sees all of its axes;
    /// this is the same reason kinematic_chain validates at construction.
    ///
    /// A prismatic tag requires its axis's angular part to be exactly zero,
    /// which is stricter than screw_axis::from_vector's sqrt-epsilon test for
    /// the same question: a six-vector with a tiny but nonzero omega is
    /// accepted there as prismatic and refused here. Deliberate -- the
    /// prismatic specialization returns a pure translation, so a residual
    /// rotation it silently drops is the same class of wrong model the tag
    /// check exists to refuse.
    static cartan::expected<static_chain, chain_failure> make(
        const se3<Scalar>& home,
        axes_storage axes,
        limits_storage limits)
    {
        if (!home.matrix().allFinite() || !axes_are_finite(axes))
        {
            return cartan::unexpected(chain_failure::non_finite_input);
        }
        if (!detail::axes_match_tags_exactly<Scalar, Joints...>(axes))
        {
            return cartan::unexpected(chain_failure::tag_axis_contradiction);
        }
        return static_chain(home, std::move(axes), std::move(limits));
    }

    /// Home configuration (M matrix): end-effector pose at zero joint angles.
    const se3<Scalar>& home() const { return m_home; }

    /// Number of joints in the chain.
    int num_joints() const { return joints; }

    /// Access a single screw axis by index.
    const screw_axis<Scalar>& axis(int i) const
    {
        return m_axes[static_cast<std::size_t>(i)];
    }

    /// Space-frame screw axes.
    const axes_storage& axes() const { return m_axes; }

    /// Joint limits.
    const limits_storage& limits() const { return m_limits; }

private:
    se3<Scalar> m_home;
    axes_storage m_axes;
    limits_storage m_limits;

    static_chain(
        const se3<Scalar>& home,
        axes_storage axes,
        limits_storage limits)
        : m_home(home)
        , m_axes(std::move(axes))
        , m_limits(std::move(limits))
    {
    }

    static bool axes_are_finite(const axes_storage& axes)
    {
        for (const auto& axis : axes)
        {
            if (!axis.to_vector().allFinite())
            {
                return false;
            }
        }
        return true;
    }
};

static_assert(chain<static_chain<double, revolute_z, revolute_y, revolute_z>>,
    "static_chain must satisfy chain concept");

}

#endif
