#ifndef HPP_GUARD_LIEPP_SERIAL_CHAIN_JOINT_TAGS_H
#define HPP_GUARD_LIEPP_SERIAL_CHAIN_JOINT_TAGS_H

/// @file joint_tags.h
/// @brief Joint type tags for compile-time chain descriptions.
///
/// Six tag types representing the principal revolute and prismatic joints.
/// Used as template parameter packs in static_chain to enable compile-time
/// axis knowledge and if-constexpr dispatch on joint type.

#include "liepp/types.h"

#include <concepts>

namespace liepp
{

/// Tag for a revolute joint about the X axis.
struct revolute_x
{
    static constexpr bool is_revolute = true;

    template <typename Scalar>
    [[nodiscard]] static constexpr vector3<Scalar> omega()
    {
        return {Scalar(1), Scalar(0), Scalar(0)};
    }
};

/// Tag for a revolute joint about the Y axis.
struct revolute_y
{
    static constexpr bool is_revolute = true;

    template <typename Scalar>
    [[nodiscard]] static constexpr vector3<Scalar> omega()
    {
        return {Scalar(0), Scalar(1), Scalar(0)};
    }
};

/// Tag for a revolute joint about the Z axis.
struct revolute_z
{
    static constexpr bool is_revolute = true;

    template <typename Scalar>
    [[nodiscard]] static constexpr vector3<Scalar> omega()
    {
        return {Scalar(0), Scalar(0), Scalar(1)};
    }
};

/// Tag for a prismatic joint along the X axis.
struct prismatic_x
{
    static constexpr bool is_revolute = false;

    template <typename Scalar>
    [[nodiscard]] static constexpr vector3<Scalar> direction()
    {
        return {Scalar(1), Scalar(0), Scalar(0)};
    }
};

/// Tag for a prismatic joint along the Y axis.
struct prismatic_y
{
    static constexpr bool is_revolute = false;

    template <typename Scalar>
    [[nodiscard]] static constexpr vector3<Scalar> direction()
    {
        return {Scalar(0), Scalar(1), Scalar(0)};
    }
};

/// Tag for a prismatic joint along the Z axis.
struct prismatic_z
{
    static constexpr bool is_revolute = false;

    template <typename Scalar>
    [[nodiscard]] static constexpr vector3<Scalar> direction()
    {
        return {Scalar(0), Scalar(0), Scalar(1)};
    }
};

/// Concept constraining types usable as joint tags in a static_chain parameter pack.
template <typename T>
concept joint_tag = requires
{
    { T::is_revolute } -> std::convertible_to<bool>;
};

}

#endif
