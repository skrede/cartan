#ifndef HPP_GUARD_CARTAN_URDF_DETAIL_NARROWING_H
#define HPP_GUARD_CARTAN_URDF_DETAIL_NARROWING_H

/// Checked conversion of the description reader's double-precision values into
/// the chain's scalar type.
///
/// The reader is fixed at double and its result carries no template parameter,
/// so this cast is the only place a caller asking for a narrower scalar can be
/// told which value that scalar cannot hold.

#include "cartan/urdf/error.h"

#include "cartan/types.h"

#include <meios/math/vector3.h>

#include <cmath>
#include <limits>
#include <optional>

namespace cartan::detail
{

/// nullopt means accepted. A double outside the target type's range converts by
/// an implementation-defined rule rather than reliably to an infinity, so the
/// magnitude is tested before the cast instead of the result after it. The
/// mirrored direction is tested after it, because underflow to zero is a
/// defined conversion: a nonzero value that collapses to exactly zero is as
/// unrecoverable as one that overflows, and it pins the joint whose bound it
/// was rather than unbounding it.
template <typename Scalar>
std::optional<urdf_failure> narrow_into(double value, Scalar& out) noexcept
{
    if (!std::isfinite(value)
        || std::abs(value) > static_cast<double>(std::numeric_limits<Scalar>::max()))
    {
        return urdf_failure::non_finite_value;
    }
#if defined(_MSC_VER)
// MSVC constant-folds this cast at call sites that pass a literal beyond the
// target's range and reports C4756, even though the magnitude check above makes
// that path unreachable.
#pragma warning(push)
#pragma warning(disable : 4756)
#endif
    const Scalar narrowed = static_cast<Scalar>(value);
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
    if (narrowed == Scalar(0) && value != 0.0) { return urdf_failure::non_finite_value; }
    out = narrowed;
    return std::nullopt;
}

template <typename Scalar>
std::optional<urdf_failure> narrow_vector(const meios::vector3<double>& value,
                                          vector3<Scalar>& out) noexcept
{
    if (narrow_into(value.x, out(0)) || narrow_into(value.y, out(1))
        || narrow_into(value.z, out(2)))
    {
        return urdf_failure::non_finite_value;
    }
    return std::nullopt;
}

}

#endif
