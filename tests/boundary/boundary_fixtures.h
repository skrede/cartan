#ifndef HPP_GUARD_CARTAN_TESTS_BOUNDARY_BOUNDARY_FIXTURES_H
#define HPP_GUARD_CARTAN_TESTS_BOUNDARY_BOUNDARY_FIXTURES_H

#include "cartan/serial_chain.h"

#include <array>
#include <vector>
#include <numbers>
#include <utility>
#include <functional>

namespace cartan::fixtures
{

inline constexpr int six_joints = 6;

/// One layout shared by every six-joint fixture below: revolute z-axis joints
/// spaced along x, so a result computed through one entry point is comparable
/// with the same configuration computed through another.
template <typename Scalar>
screw_axis<Scalar> spaced_revolute_axis(int i)
{
    const Scalar link = Scalar(0.4);
    vector3<Scalar> point;
    point << Scalar(i) * link, Scalar(0), Scalar(0);
    vector3<Scalar> direction;
    direction << Scalar(0), Scalar(0), Scalar(1);
    return screw_axis<Scalar>::revolute(direction, point);
}

template <typename Scalar>
joint_limits<Scalar> full_turn_limits()
{
    return *joint_limits<Scalar>::make(
        -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>);
}

/// Neither screw_axis nor joint_limits has a default constructor, so the
/// fixed-size arrays are built by expanding the index sequence rather than
/// filled after the fact.
template <typename Scalar>
std::array<screw_axis<Scalar>, six_joints> six_joint_axes()
{
    return []<std::size_t... Is>(std::index_sequence<Is...>)
    {
        return std::array<screw_axis<Scalar>, six_joints>{
            spaced_revolute_axis<Scalar>(static_cast<int>(Is))...};
    }(std::make_index_sequence<six_joints>{});
}

template <typename Scalar>
std::array<joint_limits<Scalar>, six_joints> six_joint_limits()
{
    return []<std::size_t... Is>(std::index_sequence<Is...>)
    {
        return std::array<joint_limits<Scalar>, six_joints>{
            (static_cast<void>(Is), full_turn_limits<Scalar>())...};
    }(std::make_index_sequence<six_joints>{});
}

template <typename Scalar>
se3<Scalar> spaced_home(int joints)
{
    vector3<Scalar> translation;
    translation << Scalar(joints) * Scalar(0.4), Scalar(0), Scalar(0);
    return se3<Scalar>(so3<Scalar>::identity(), translation);
}

template <typename Scalar>
se3<Scalar> six_joint_home()
{
    return spaced_home<Scalar>(six_joints);
}

/// Sized at runtime, so a joint vector of any length can be handed to the
/// entry points under test, and so a cached result can be produced from a
/// chain of a different joint count than the one it is later handed to.
template <typename Scalar>
kinematic_chain<Scalar, dynamic> make_dynamic_chain(int joints)
{
    std::vector<screw_axis<Scalar>> axes;
    std::vector<joint_limits<Scalar>> limits;
    for (int i = 0; i < joints; ++i)
    {
        axes.push_back(spaced_revolute_axis<Scalar>(i));
        limits.push_back(full_turn_limits<Scalar>());
    }

    return kinematic_chain<Scalar, dynamic>(
        spaced_home<Scalar>(joints), std::move(axes), std::move(limits));
}

template <typename Scalar>
kinematic_chain<Scalar, dynamic> make_six_joint_dynamic_chain()
{
    return make_dynamic_chain<Scalar>(six_joints);
}

/// Same layout with the joint count in the type. A joint vector of the wrong
/// length is unrepresentable here, which is the point: what remains testable at
/// this entry point is finiteness.
template <typename Scalar>
kinematic_chain<Scalar, six_joints> make_six_joint_fixed_chain()
{
    return kinematic_chain<Scalar, six_joints>(
        six_joint_home<Scalar>(), six_joint_axes<Scalar>(), six_joint_limits<Scalar>());
}

template <typename Scalar>
using six_joint_static_chain = static_chain<Scalar,
    revolute_z, revolute_z, revolute_z, revolute_z, revolute_z, revolute_z>;

template <typename Scalar>
six_joint_static_chain<Scalar> make_six_joint_static_chain()
{
    return six_joint_static_chain<Scalar>(
        six_joint_home<Scalar>(), six_joint_axes<Scalar>(), six_joint_limits<Scalar>());
}

/// A chain type that is neither kinematic_chain nor static_chain, so a call
/// through it selects the concept-constrained overloads that partial ordering
/// otherwise hides behind the two concrete ones.
template <typename Scalar>
class dynamic_chain_adapter
{
public:
    using scalar_type = Scalar;
    static constexpr int joints = dynamic;

    explicit dynamic_chain_adapter(const kinematic_chain<Scalar, dynamic>& chain)
        : m_chain(chain)
    {
    }

    const se3<Scalar>& home() const { return m_chain.get().home(); }

    int num_joints() const { return m_chain.get().num_joints(); }

    const screw_axis<Scalar>& axis(int i) const { return m_chain.get().axis(i); }

    const std::vector<screw_axis<Scalar>>& axes() const { return m_chain.get().axes(); }

    const std::vector<joint_limits<Scalar>>& limits() const { return m_chain.get().limits(); }

private:
    std::reference_wrapper<const kinematic_chain<Scalar, dynamic>> m_chain;
};

/// A uniform joint vector of a caller-chosen length. The length is a parameter
/// because the cases under test are the lengths that do not match the chain.
template <typename Scalar>
Eigen::VectorX<Scalar> joint_vector(int size, Scalar value)
{
    return Eigen::VectorX<Scalar>::Constant(size, value);
}

template <typename Scalar>
Eigen::Vector<Scalar, six_joints> fixed_joint_vector(Scalar value)
{
    return Eigen::Vector<Scalar, six_joints>::Constant(value);
}

}

#endif
