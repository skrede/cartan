#ifndef HPP_GUARD_CARTAN_SERIAL_IK_IK_RESULT_H
#define HPP_GUARD_CARTAN_SERIAL_IK_IK_RESULT_H

/// @file ik_result.h
/// @brief IK result and error types for inverse kinematics solve outcomes.
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 6.2, p. 227-233.

#include "cartan/serial/ik/ik_status.h"

#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/storage_trait.h"

#include <type_traits>

namespace cartan
{

/// Successful IK result containing solution configuration and diagnostics.
template <typename Scalar = double, int N = dynamic>
struct ik_result
{
    static_assert(std::is_floating_point_v<Scalar>, "ik_result requires a floating-point Scalar type");

    joint_state<Scalar, N> solution;
    Scalar final_error_norm{};
    int iterations{};
    int solver_index{};
};

/// IK error containing failure diagnostics.
template <typename Scalar = double, int N = dynamic>
struct ik_error
{
    static_assert(std::is_floating_point_v<Scalar>, "ik_error requires a floating-point Scalar type");

    ik_failure reason;
    ik_termination_reason termination_reason{ik_termination_reason::unknown};
    typename joint_state<Scalar, N>::position_type last_q;
    Scalar last_error_norm{};
    Scalar condition_number{};
    bool near_singular{};
};

}

#endif
