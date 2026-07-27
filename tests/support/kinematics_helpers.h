#ifndef HPP_GUARD_CARTAN_TESTS_SUPPORT_KINEMATICS_HELPERS_H
#define HPP_GUARD_CARTAN_TESTS_SUPPORT_KINEMATICS_HELPERS_H

// Unwrapping forms of the checked kinematics entry points, for tests driven by
// Catch2. A refusal fails the enclosing assertion instead of aborting the way
// cartan::testing::unwrap does: these consume solver output and drawn
// configurations as well as fixture literals, so taking the process down would
// discard every other case's result.
//
// Tests go through the checked entry point and unwrap here rather than reaching
// for the unchecked sibling: the unchecked sibling would not exercise the
// contract the suite exists to cover.

#include <cartan/types.h>

#include <cartan/serial/fk/jacobian.h>
#include <cartan/serial/fk/velocity.h>
#include <cartan/serial/fk/jacobian_matrix.h>
#include <cartan/serial/fk/forward_kinematics.h>
#include <cartan/serial/fk/forward_kinematics_matrix.h>

#include <catch2/catch_test_macros.hpp>

#include <utility>

namespace cartan::testing
{

template <typename T, typename E>
T checked(cartan::expected<T, E> held)
{
    REQUIRE(held.has_value());
    return std::move(*held);
}

template <typename Chain, typename Derived>
fk_result<typename Chain::scalar_type, Chain::joints> fk_at(
    const Chain& chain, const Eigen::MatrixBase<Derived>& q)
{
    return checked(cartan::forward_kinematics(chain, q));
}

template <typename Chain, typename Derived>
fk_matrix_result<typename Chain::scalar_type, Chain::joints> fk_matrix_at(
    const Chain& chain, const Eigen::MatrixBase<Derived>& q)
{
    return checked(cartan::forward_kinematics_matrix(chain, q));
}

/// Fk is deduced so that both the exponential-form and the matrix-form results
/// reach their respective overloads through one helper.
template <typename Chain, typename Fk>
jacobian_matrix<typename Chain::scalar_type, Chain::joints> space_jacobian_at(
    const Chain& chain, const Fk& fk)
{
    return checked(cartan::space_jacobian(chain, fk));
}

template <typename Chain, typename Fk>
jacobian_matrix<typename Chain::scalar_type, Chain::joints> body_jacobian_at(
    const Chain& chain, const Fk& fk)
{
    return checked(cartan::body_jacobian(chain, fk));
}

template <typename Chain, typename DerivedQ, typename DerivedDq>
vector6<typename Chain::scalar_type> velocity_at(
    const Chain& chain,
    const Eigen::MatrixBase<DerivedQ>& q,
    const Eigen::MatrixBase<DerivedDq>& dq)
{
    return checked(cartan::end_effector_velocity(chain, q, dq));
}

}

#endif
