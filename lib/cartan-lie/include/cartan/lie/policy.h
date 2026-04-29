#ifndef HPP_GUARD_CARTAN_LIE_POLICY_H
#define HPP_GUARD_CARTAN_LIE_POLICY_H

#include <concepts>
#include <type_traits>

namespace cartan
{

/// Policy that normalizes group elements on construction and asserts validity.
/// Use for safety-critical paths where inputs may not be pre-validated.
struct strict_policy
{
    static constexpr bool normalize_on_construct = true;
    static constexpr bool assert_valid = true;
};

/// Policy that skips normalization and assertions for maximum performance.
/// Use on hot paths where inputs are known-valid (e.g., from prior computation).
struct fast_policy
{
    static constexpr bool normalize_on_construct = false;
    static constexpr bool assert_valid = false;
};

/// Tag type for the unchecked "trusted unit" construction path on so3 / se3.
/// Use when the caller can prove the input is already a unit quaternion
/// (e.g. exact axis-angle output from exp, conjugate of a unit, identity).
/// The construction skips normalize() unconditionally; debug builds still
/// validate via assert().
struct trusted_unit_t {};
inline constexpr trusted_unit_t trusted_unit{};

/// Selects the stricter of two policies: if either normalizes, the result normalizes.
/// Used for mixed-policy compose operations (D-08).
template <typename P1, typename P2>
using stricter_policy = std::conditional_t<
    P1::normalize_on_construct || P2::normalize_on_construct,
    strict_policy,
    fast_policy>;

/// Concept constraining valid Lie group policy types.
template <typename P>
concept lie_group_policy = requires
{
    { P::normalize_on_construct } -> std::convertible_to<bool>;
    { P::assert_valid } -> std::convertible_to<bool>;
};

}

#endif
