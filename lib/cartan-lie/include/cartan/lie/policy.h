#ifndef HPP_GUARD_LIEPP_LIE_POLICY_H
#define HPP_GUARD_LIEPP_LIE_POLICY_H

#include <concepts>
#include <type_traits>

namespace liepp
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

} // namespace liepp

#endif
