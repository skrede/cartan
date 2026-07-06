#ifndef HPP_GUARD_CARTAN_SERIAL_IK_IK_RESULT_H
#define HPP_GUARD_CARTAN_SERIAL_IK_IK_RESULT_H

/// IK result and error types for inverse kinematics solve outcomes.
///
/// Reference: Lynch & Park, Modern Robotics, Ch. 6.2, p. 227-233.

#include "cartan/serial/ik/ik_status.h"

#include "cartan/serial/chain/joint_state.h"
#include "cartan/serial/chain/storage_trait.h"

#include <limits>
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

namespace detail
{

/// Poison default for an ik_error's diagnostic joint vector: NaN-filled for a
/// fixed-size chain, empty for a dynamic one. A field left at this default was
/// never populated by the solver; a NaN sentinel makes an accidental read fail
/// loudly -- it propagates through arithmetic and, unlike a large finite value,
/// survives angle wrapping -- instead of masquerading as the plausible all-zero
/// home configuration.
template <typename Scalar, int N>
typename joint_state<Scalar, N>::position_type poison_joint_position()
{
    using position_type = typename joint_state<Scalar, N>::position_type;
    if constexpr (N == dynamic)
    {
        return position_type{};
    }
    else
    {
        return position_type::Constant(std::numeric_limits<Scalar>::quiet_NaN());
    }
}

}

/// IK error containing failure diagnostics. Every payload field defaults to a
/// NaN poison so an unpopulated diagnostic surfaces as an obvious failure rather
/// than a plausible value (a zero last_q reads as the home pose; a zero
/// last_error_norm reads as "converged").
template <typename Scalar = double, int N = dynamic>
struct ik_error
{
    static_assert(std::is_floating_point_v<Scalar>, "ik_error requires a floating-point Scalar type");

    ik_failure reason{ik_failure::iteration_limit};
    ik_termination_reason termination_reason{ik_termination_reason::unknown};
    typename joint_state<Scalar, N>::position_type last_q{detail::poison_joint_position<Scalar, N>()};
    Scalar last_error_norm{std::numeric_limits<Scalar>::quiet_NaN()};
    Scalar condition_number{std::numeric_limits<Scalar>::quiet_NaN()};
    bool near_singular{};
};

}

#endif
