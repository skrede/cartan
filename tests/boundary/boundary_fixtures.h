#ifndef HPP_GUARD_CARTAN_TESTS_BOUNDARY_BOUNDARY_FIXTURES_H
#define HPP_GUARD_CARTAN_TESTS_BOUNDARY_BOUNDARY_FIXTURES_H

#include "cartan/serial_chain.h"

#include <vector>
#include <numbers>
#include <utility>

namespace cartan::fixtures
{

/// Six revolute z-axis joints spaced along x, sized at runtime so a joint
/// vector of any length can be handed to the entry points under test.
template <typename Scalar>
kinematic_chain<Scalar, dynamic> make_six_joint_dynamic_chain()
{
    const Scalar link = Scalar(0.4);
    vector3<Scalar> home_translation;
    home_translation << Scalar(6) * link, Scalar(0), Scalar(0);

    std::vector<screw_axis<Scalar>> axes;
    std::vector<joint_limits<Scalar>> limits;
    for (int i = 0; i < 6; ++i)
    {
        vector3<Scalar> point;
        point << Scalar(i) * link, Scalar(0), Scalar(0);
        vector3<Scalar> axis;
        axis << Scalar(0), Scalar(0), Scalar(1);
        axes.push_back(screw_axis<Scalar>::revolute(axis, point));
        limits.push_back(joint_limits<Scalar>{
            -std::numbers::pi_v<Scalar>, std::numbers::pi_v<Scalar>});
    }

    return kinematic_chain<Scalar, dynamic>(
        se3<Scalar>(so3<Scalar>::identity(), home_translation),
        std::move(axes),
        std::move(limits));
}

/// A uniform joint vector of a caller-chosen length. The length is a parameter
/// because the cases under test are the lengths that do not match the chain.
template <typename Scalar>
Eigen::VectorX<Scalar> joint_vector(int size, Scalar value)
{
    return Eigen::VectorX<Scalar>::Constant(size, value);
}

}

#endif
