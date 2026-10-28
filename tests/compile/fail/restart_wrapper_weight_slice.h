#ifndef HPP_GUARD_CARTAN_TESTS_COMPILE_RESTART_WRAPPER_WEIGHT_SLICE_H
#define HPP_GUARD_CARTAN_TESTS_COMPILE_RESTART_WRAPPER_WEIGHT_SLICE_H

// The fixture shared by the weighted-wrapper rejection and its control, so the
// two differ only in the inner policy and the rejection is attributable to that.

#include "cartan/lie/se3.h"
#include "cartan/lie/so3.h"
#include "cartan/serial/ik/policy/error_weight.h"
#include "cartan/serial/chain/screw_axis.h"
#include "cartan/serial/chain/joint_limits.h"
#include "cartan/serial/chain/kinematic_chain.h"
#include "cartan/serial/ik/wrapper/restart_wrapper.h"

#include <numbers>

namespace cartan::compile_gate
{

using chain_type = cartan::kinematic_chain<double, 2>;

inline chain_type make_planar_2r()
{
    auto s1 = cartan::screw_axis<double>::revolute({0, 0, 1}, {0, 0, 0});
    auto s2 = cartan::screw_axis<double>::revolute({0, 0, 1}, {1, 0, 0});

    cartan::vector3<double> home_trans;
    home_trans << 2.0, 0.0, 0.0;
    auto home = cartan::se3<double>(cartan::so3<double>::identity(), home_trans);

    auto lim = cartan::joint_limits<double>::make(
        -std::numbers::pi, std::numbers::pi).value();
    return chain_type(home, {s1, s2}, {lim, lim});
}

template <typename Wrapper>
int setup_with_weight(Wrapper& wrapper)
{
    auto chain = make_planar_2r();
    auto target = cartan::se3<double>::identity();
    auto q0 = cartan::joint_state<double, 2>::position_type::Zero().eval();
    cartan::convergence_criteria<double> criteria{};

    cartan::error_weight<double> weight;
    weight.weights << 0.1, 0.1, 0.1, 1.0, 1.0, 1.0;

    wrapper.setup(chain, target, q0, criteria, weight);
    return 0;
}

}

#endif
