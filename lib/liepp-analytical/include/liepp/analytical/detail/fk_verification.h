#ifndef HPP_GUARD_LIEPP_ANALYTICAL_DETAIL_FK_VERIFICATION_H
#define HPP_GUARD_LIEPP_ANALYTICAL_DETAIL_FK_VERIFICATION_H

#include "liepp/serial/chain/static_chain.h"
#include "liepp/serial/chain/joint_tags.h"
#include "liepp/serial/fk/forward_kinematics.h"

#include "liepp/lie/se3.h"

namespace liepp::detail
{

template <typename Scalar, joint_tag... Joints>
[[nodiscard]] bool verify_analytical_solution(
    const static_chain<Scalar, Joints...>& chain,
    const Eigen::Vector<Scalar, sizeof...(Joints)>& q,
    const se3<Scalar>& target,
    bool check_orientation,
    Scalar position_tolerance = Scalar(1e-6),
    Scalar orientation_tolerance = Scalar(1e-6))
{
    auto fk = forward_kinematics(chain, q);
    Scalar position_error = (fk.end_effector.translation() - target.translation()).norm();
    if (position_error >= position_tolerance)
        return false;
    if (!check_orientation)
        return true;
    Scalar orientation_error = (fk.end_effector.rotation().inverse() * target.rotation()).log().norm();
    return orientation_error < orientation_tolerance;
}

}

#endif
