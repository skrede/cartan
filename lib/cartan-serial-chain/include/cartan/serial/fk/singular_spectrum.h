#ifndef HPP_GUARD_CARTAN_SERIAL_FK_SINGULAR_SPECTRUM_H
#define HPP_GUARD_CARTAN_SERIAL_FK_SINGULAR_SPECTRUM_H

/// The one decomposition every singularity measure is read off: the singular
/// values of a body Jacobian, and of a chain at a configuration.
///
/// A caller who wants more than one measure asks for the spectrum once and
/// passes it around, which is why fk/singularity_analysis.h carries no
/// per-measure (chain, q) overload.

#include "cartan/expected.h"

#include "cartan/serial/fk/jacobian.h"
#include "cartan/serial/fk/forward_kinematics.h"
#include "cartan/serial/fk/singularity_failure.h"

#include <Eigen/SVD>

#include <cmath>
#include <type_traits>

namespace cartan
{

/// The matrix type the decomposition runs through.
///
/// Eigen takes a decomposition's diagonal size from the fixed dimension when
/// only one of the two is fixed, so JacobiSVD over a 6-by-dynamic Jacobian
/// declares a fixed six singular values and aborts resizing them for a chain
/// that does not have exactly six joints. Decomposing a fully dynamic copy
/// costs one allocation on an analysis path and answers for every joint count.
template <typename Derived>
using svd_matrix_t = std::conditional_t<
    Derived::ColsAtCompileTime == Eigen::Dynamic,
    Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic>,
    typename Derived::PlainObject>;

template <typename Derived>
using singular_values_t = typename Eigen::JacobiSVD<svd_matrix_t<Derived>>::SingularValuesType;

/// The characteristic length divides the Jacobian's linear rows, which only a
/// positive finite value can do. Zero and NaN leave the decomposition reading a
/// spectrum it never computed; an infinite one annihilates the linear block and
/// answers a plausible spectrum for a Jacobian the caller did not ask about; a
/// negative one negates three rows, which is an orthogonal transformation, so it
/// answers exactly as its magnitude would without saying so.
template <typename Scalar>
bool is_valid_characteristic_length(Scalar length)
{
    return length > Scalar(0) && std::isfinite(length);
}

/// Singular values of a Jacobian, largest first, whose linear rows -- the last
/// three, in the omega-first convention -- are divided by a characteristic
/// length so they are commensurable with the dimensionless angular rows above
/// them. Without that division the product of them has no coherent unit and
/// their ratio compares incommensurables. The default of one reproduces the
/// unnormalized arithmetic exactly.
///
/// The guard precedes the decomposition because constructing one over an empty
/// matrix is itself the fault: Eigen's preconditioner resizes a fixed-size
/// vector to zero, which trips an assertion in a checked build and faults in
/// one without.
template <typename Derived>
cartan::expected<singular_values_t<Derived>, singularity_failure> singular_values(
    const Eigen::MatrixBase<Derived>& jacobian,
    typename Derived::Scalar length = typename Derived::Scalar(1))
{
    if (!is_valid_characteristic_length(length))
    {
        return cartan::unexpected(singularity_failure::invalid_length);
    }
    if (jacobian.cols() == 0)
    {
        return cartan::unexpected(singularity_failure::empty_spectrum);
    }

    svd_matrix_t<Derived> scaled = jacobian;
    scaled.template bottomRows<3>() /= length;

    constexpr unsigned int options = (Derived::ColsAtCompileTime == Eigen::Dynamic)
        ? (Eigen::ComputeThinU | Eigen::ComputeThinV)
        : (Eigen::ComputeFullU | Eigen::ComputeFullV);
    // GCC reports the decomposition's singular values as possibly uninitialized
    // when it inlines this copy at -O2 and above; it cannot see that Eigen fills
    // them before returning. Clang is clean on the same source, and every
    // optimization level below -O2 is clean on GCC. Bracketed on every GCC that
    // raises it rather than one version, because a version-scoped bracket leaves
    // the same false positive unhandled on the compilers below the newest.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
    Eigen::JacobiSVD<svd_matrix_t<Derived>> svd(scaled, options);
    return svd.singularValues();
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif
}

/// singular_values of the chain's body Jacobian at q.
///
/// This is the spelling to reach for. The measures are defined on the body
/// Jacobian, and handing the space Jacobian to the overload above compiles and
/// answers a different question. It runs through the checked kinematics, so a q
/// whose length disagrees with the chain, or which carries a non-finite
/// component, is reported rather than read past.
template <typename Chain, typename Vector>
cartan::expected<
    singular_values_t<jacobian_matrix<typename Chain::scalar_type, Chain::joints>>,
    singularity_failure>
singular_values(
    const Chain& chain,
    const Vector& q,
    typename Chain::scalar_type length = typename Chain::scalar_type(1))
{
    auto fk = forward_kinematics(chain, q);
    if (!fk)
    {
        return cartan::unexpected(singularity_failure::invalid_configuration);
    }
    auto jacobian = body_jacobian(chain, *fk);
    if (!jacobian)
    {
        return cartan::unexpected(singularity_failure::invalid_configuration);
    }
    return singular_values(*jacobian, length);
}

}

#endif
