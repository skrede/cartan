#ifndef HPP_GUARD_LIEPP_SERIAL_CHAIN_STORAGE_TRAIT_H
#define HPP_GUARD_LIEPP_SERIAL_CHAIN_STORAGE_TRAIT_H

/// @file storage_trait.h
/// @brief Fixed/dynamic storage selector for kinematic chain containers.
///
/// Provides a compile-time selector between std::array (fixed N) and
/// std::vector (dynamic) storage, controlled by the liepp::dynamic sentinel.

#include "liepp/types.h"

#include <array>
#include <vector>
#include <cstddef>

namespace liepp
{

namespace detail
{

/// Primary template: fixed-size storage using std::array.
template <int N, typename T>
struct storage_selector
{
    using type = std::array<T, static_cast<std::size_t>(N)>;
    static constexpr bool is_fixed = true;
};

/// Specialization: dynamic storage using std::vector.
template <typename T>
struct storage_selector<dynamic, T>
{
    using type = std::vector<T>;
    static constexpr bool is_fixed = false;
};

/// Convenience alias for storage type selection.
template <int N, typename T>
using storage_t = typename storage_selector<N, T>::type;

} // namespace detail
} // namespace liepp

#endif
