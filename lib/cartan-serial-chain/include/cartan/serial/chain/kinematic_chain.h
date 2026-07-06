#ifndef HPP_GUARD_CARTAN_SERIAL_CHAIN_KINEMATIC_CHAIN_H
#define HPP_GUARD_CARTAN_SERIAL_CHAIN_KINEMATIC_CHAIN_H

/// Product of Exponentials (PoE) kinematic chain model.
///
/// Stores screw axes, home configuration (M matrix), and joint limits
/// for an N-joint serial chain. Supports both fixed (compile-time N)
/// and dynamic (runtime N) sizing via storage_trait.
///
/// Lynch & Park, Modern Robotics, Eq. 4.10, p. 138:
///   T(q) = exp([S1]q1) ... exp([Sn]qn) * M
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 4, p. 119-158.

#include "cartan/serial/chain/screw_axis.h"
#include "cartan/serial/chain/joint_limits.h"
#include "cartan/serial/chain/joint_kind.h"
#include "cartan/serial/chain/storage_trait.h"
#include "cartan/serial/chain/chain_concept.h"

#include "cartan/lie/se3.h"

#include <vector>
#include <cstddef>
#include <stdexcept>
#include <type_traits>

namespace cartan
{

/// Kinematic chain in Product of Exponentials form. N is the joint count
/// at compile time (or cartan::dynamic for runtime). Scalar is the
/// floating-point type used throughout the chain.
///
/// Lynch & Park, Modern Robotics, Eq. 4.10, p. 138:
///   T(q) = exp([S1]q1) ... exp([Sn]qn) * M
template <typename Scalar = double, int N = dynamic>
class kinematic_chain
{
    static_assert(std::is_floating_point_v<Scalar>, "kinematic_chain requires a floating-point Scalar type");
public:
    using scalar_type = Scalar;
    static constexpr int joints = N;

    using screw_storage = detail::storage_t<N, screw_axis<Scalar>>;
    using limits_storage = detail::storage_t<N, joint_limits<Scalar>>;
    using kind_storage = detail::storage_t<N, joint_kind>;

    /// Construct a kinematic chain from home configuration, screw axes, and limits.
    kinematic_chain(
        const se3<Scalar>& home,
        screw_storage axes,
        limits_storage limits)
        : m_home(home)
        , m_axes(std::move(axes))
        , m_limits(std::move(limits))
    {
        // Real runtime invariant: the screw-axis count must match the
        // joint-limit count. A debug-only assert would be compiled out under
        // -DNDEBUG (Release), letting a malformed chain construct silently in
        // a shipped build. With exceptions enabled we fail loudly by throwing;
        // on exceptions-off targets (bare-metal and ESP-IDF default) a bare
        // throw would not even compile, so we fail-stop deterministically via a
        // trap instruction instead. Either way a malformed chain never
        // constructs silently.
        if (m_axes.size() != m_limits.size())
        {
#if defined(__cpp_exceptions)
            throw std::invalid_argument(
                "kinematic_chain: screw-axis count must match joint-limit count");
#else
            __builtin_trap();
#endif
        }
        if constexpr (N == dynamic)
        {
            m_kinds.resize(m_axes.size());
        }
        for (std::size_t i = 0; i < m_axes.size(); ++i)
        {
            m_kinds[i] = detect_joint_kind(m_axes[i]);
        }
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

    /// Access a single screw axis by index (bounds-checked, `.at()` semantics).
    /// Throws std::out_of_range for negative or too-large indices so a bad
    /// index can never read outside the underlying storage.
    [[nodiscard]] const screw_axis<Scalar>& axis(int i) const
    {
        if (i < 0 || static_cast<std::size_t>(i) >= m_axes.size())
        {
            throw std::out_of_range("kinematic_chain::axis: joint index out of range");
        }
        return m_axes[static_cast<std::size_t>(i)];
    }

    /// Cached joint_kind for joint i, used by FK/Jacobian fast-path dispatch.
    [[nodiscard]] joint_kind kind(int i) const
    {
        return m_kinds[static_cast<std::size_t>(i)];
    }

    /// All cached joint_kinds.
    [[nodiscard]] const kind_storage& kinds() const { return m_kinds; }

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
    kind_storage m_kinds{};     ///< Cached axis classification per joint
};

static_assert(chain<kinematic_chain<double, 3>>,
    "kinematic_chain<double, 3> must satisfy chain concept");
static_assert(chain<kinematic_chain<double, dynamic>>,
    "kinematic_chain<double, dynamic> must satisfy chain concept");

}

#endif
