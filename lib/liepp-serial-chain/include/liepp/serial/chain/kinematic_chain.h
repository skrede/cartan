#ifndef HPP_GUARD_LIEPP_CHAIN_KINEMATIC_CHAIN_H
#define HPP_GUARD_LIEPP_CHAIN_KINEMATIC_CHAIN_H

/// @file kinematic_chain.h
/// @brief Product of Exponentials (PoE) kinematic chain model.
///
/// Stores screw axes, home configuration (M matrix), and joint limits
/// for an N-joint serial chain. Supports both fixed (compile-time N)
/// and dynamic (runtime N) sizing via storage_trait.
///
/// Lynch & Park, Modern Robotics, Eq. 4.10, p. 138:
///   T(q) = exp([S1]q1) ... exp([Sn]qn) * M
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 4, p. 119-158.

#include "liepp/chain/screw_axis.h"
#include "liepp/chain/joint_limits.h"
#include "liepp/chain/storage_trait.h"

#include "liepp/lie/se3.h"

#include <vector>
#include <cassert>
#include <type_traits>

namespace liepp
{

/// Kinematic chain in Product of Exponentials form.
/// @tparam N  Number of joints (compile-time), or liepp::dynamic for runtime.
/// @tparam Scalar  Floating-point type.
///
/// Lynch & Park, Modern Robotics, Eq. 4.10, p. 138:
///   T(q) = exp([S1]q1) ... exp([Sn]qn) * M
template <typename Scalar = double, int N = dynamic>
class kinematic_chain
{
    static_assert(std::is_floating_point_v<Scalar>, "kinematic_chain requires a floating-point Scalar type");
public:
    using screw_storage = detail::storage_t<N, screw_axis<Scalar>>;
    using limits_storage = detail::storage_t<N, joint_limits<Scalar>>;

    /// Construct a kinematic chain from home configuration, screw axes, and limits.
    /// @param home   End-effector pose at zero configuration (the M matrix).
    /// @param axes   Space-frame screw axes S1..Sn.
    /// @param limits Joint position/velocity limits.
    kinematic_chain(
        const se3<Scalar>& home,
        screw_storage axes,
        limits_storage limits)
        : m_home(home)
        , m_axes(std::move(axes))
        , m_limits(std::move(limits))
    {
        assert(m_axes.size() == m_limits.size());
    }

    /// Home configuration (M matrix): end-effector pose at zero joint angles.
    [[nodiscard]] const se3<Scalar>& home() const { return m_home; }

    /// Space-frame screw axes.
    [[nodiscard]] const screw_storage& axes() const { return m_axes; }

    /// Joint limits.
    [[nodiscard]] const limits_storage& limits() const { return m_limits; }

    /// Number of joints in the chain.
    [[nodiscard]] int num_joints() const
    {
        return static_cast<int>(m_axes.size());
    }

    /// Convert a fixed-size chain to a dynamic chain.
    /// Only available when N is a fixed (non-dynamic) value.
    [[nodiscard]] kinematic_chain<Scalar, dynamic> to_dynamic() const
        requires (N != dynamic)
    {
        std::vector<screw_axis<Scalar>> dyn_axes(m_axes.begin(), m_axes.end());
        std::vector<joint_limits<Scalar>> dyn_limits(m_limits.begin(), m_limits.end());
        return kinematic_chain<Scalar, dynamic>(
            m_home, std::move(dyn_axes), std::move(dyn_limits));
    }

private:
    se3<Scalar> m_home;         ///< End-effector home pose (M matrix)
    screw_storage m_axes;       ///< Space-frame screw axes S1..Sn
    limits_storage m_limits;    ///< Joint limits
};

} // namespace liepp

#endif
