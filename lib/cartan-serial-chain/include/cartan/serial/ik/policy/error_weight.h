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
};

}

#endif
