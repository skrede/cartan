#ifndef HPP_GUARD_CARTAN_SERIAL_CHAIN_DETAIL_TAG_AXIS_CHECK_H
#define HPP_GUARD_CARTAN_SERIAL_CHAIN_DETAIL_TAG_AXIS_CHECK_H

/// Exact agreement between a compile-time joint tag and a stored screw axis.
///
/// A tag is a caller's compile-time claim about an axis the same caller
/// supplies, so the two either agree or the caller has contradicted itself.
/// Nothing here is approximate: an axis that only nearly matches belongs to the
/// runtime classifier, where no tag exists and the axis arrives from a
/// description rather than from the caller's own hand.

#include "cartan/serial/chain/joint_tags.h"
#include "cartan/serial/chain/screw_axis.h"

#include "cartan/types.h"

#include <array>
#include <tuple>
#include <cstddef>
#include <utility>

namespace cartan
{

namespace detail
{

/// Whether an axis is exactly the +/-e_k that Tag names.
///
/// Both signs are accepted: only the axis line is constrained. A principal
/// axis pointing along -e_k is how descriptions spell a joint that turns the
/// other way -- three joints of the vendored KR6 do -- and the tag-dispatched
/// evaluation recovers the sign from the stored component, so forcing a tag to
/// mean one sign would make those robots inexpressible rather than safer.
template <joint_tag Tag, typename Scalar>
bool axis_matches_tag(const screw_axis<Scalar>& axis)
{
    if constexpr (Tag::is_revolute)
    {
        const vector3<Scalar> reference = Tag::template omega<Scalar>();
        return axis.omega() == reference || axis.omega() == -reference;
    }
    else
    {
        const vector3<Scalar> reference = Tag::template direction<Scalar>();
        return axis.omega() == vector3<Scalar>::Zero()
               && (axis.v() == reference || axis.v() == -reference);
    }
}

/// Whether every axis agrees exactly with the tag at its own position.
template <typename Scalar, joint_tag... Joints>
bool axes_match_tags_exactly(
    const std::array<screw_axis<Scalar>, sizeof...(Joints)>& axes)
{
    return [&]<std::size_t... Is>(std::index_sequence<Is...>)
    {
        using joint_tuple = std::tuple<Joints...>;
        return (... && axis_matches_tag<std::tuple_element_t<Is, joint_tuple>>(
                           axes[Is]));
    }(std::make_index_sequence<sizeof...(Joints)>{});
}

template <typename Scalar, std::size_t N>
bool axes_are_finite(const std::array<screw_axis<Scalar>, N>& axes)
{
    for (const auto& axis : axes)
    {
        if (!axis.to_vector().allFinite())
        {
            return false;
        }
    }
    return true;
}

}

}

#endif
