#ifndef HPP_GUARD_CARTAN_TESTS_SUPPORT_JOINT_LIMITS_HELPERS_H
#define HPP_GUARD_CARTAN_TESTS_SUPPORT_JOINT_LIMITS_HELPERS_H

// One short spelling for the joint limits a test, fixture or example needs when
// the values are known-good and the point of the code is something else. It
// belongs here and not in the library: a helper that discards a typed error is
// exactly what the library must not offer.

#include "expected_helpers.h"

#include <cartan/serial/chain/joint_limits.h>
#include <cartan/serial/chain/chain_failure.h>

#include <optional>
#include <type_traits>

namespace cartan::testing
{

/// Construct joint limits, aborting with the rejection reason when the values
/// are not sound. Aborting rather than throwing keeps the helper usable from
/// the exceptions-off targets.
///
/// type_identity_t stops the optional bounds from taking part in deduction, so
/// limits(-1.0, 1.0, 2.0) picks Scalar from the position bounds alone instead
/// of failing to reconcile double against std::optional<Scalar>.
template <typename Scalar>
cartan::joint_limits<Scalar> limits(
    Scalar position_min,
    Scalar position_max,
    std::optional<std::type_identity_t<Scalar>> velocity_max = std::nullopt,
    std::optional<std::type_identity_t<Scalar>> effort_max = std::nullopt,
    std::optional<std::type_identity_t<Scalar>> acceleration_max = std::nullopt)
{
    return unwrap(
        cartan::joint_limits<Scalar>::make(
            position_min, position_max, velocity_max, effort_max, acceleration_max),
        "cartan::testing::limits");
}

}

#endif
