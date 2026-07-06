#ifndef HPP_GUARD_CARTAN_SERIAL_IK_POLICY_ERROR_WEIGHT_H
#define HPP_GUARD_CARTAN_SERIAL_IK_POLICY_ERROR_WEIGHT_H

#include "cartan/types.h"

namespace cartan
{

template <typename Scalar = double>
struct error_weight
{
    vector6<Scalar> weights{vector6<Scalar>::Ones()};

    vector6<Scalar> apply(const vector6<Scalar>& v) const
    {
        return weights.cwiseProduct(v);
    }

    Scalar weighted_angular_norm(const vector6<Scalar>& v) const
    {
        return (weights.template head<3>().cwiseProduct(v.template head<3>())).norm();
    }

    Scalar weighted_linear_norm(const vector6<Scalar>& v) const
    {
        return (weights.template tail<3>().cwiseProduct(v.template tail<3>())).norm();
    }
};

}

#endif
