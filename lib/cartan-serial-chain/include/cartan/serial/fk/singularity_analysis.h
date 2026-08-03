#ifndef HPP_GUARD_CARTAN_SERIAL_FK_SINGULARITY_ANALYSIS_H
#define HPP_GUARD_CARTAN_SERIAL_FK_SINGULARITY_ANALYSIS_H

/// Singularity analysis of a body Jacobian: its spectrum, and the four measures
/// read off that spectrum.
///
/// One decomposition feeds every measure, so a caller who wants more than one
/// asks for the singular values once and passes them around. That is why there
/// is no per-measure (chain, q) overload: four of them would hide four
/// decompositions behind four one-line calls.

#include "cartan/serial/fk/jacobian.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include <Eigen/SVD>

#include <limits>
#include <optional>
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

/// Singular values of a Jacobian, largest first, whose linear rows -- the last
/// three, in the omega-first convention -- are divided by a characteristic
/// length so they are commensurable with the dimensionless angular rows above
/// them. Without that division the product below has no coherent unit and the
/// ratio compares incommensurables. The default of one reproduces the
/// unnormalized arithmetic exactly.
///
/// Empty where the Jacobian has no columns. The guard precedes the
/// decomposition because constructing one over an empty matrix is itself the
/// fault: Eigen's preconditioner resizes a fixed-size vector to zero, which
/// trips an assertion in a checked build and faults in one without.
template <typename Derived>
singular_values_t<Derived> singular_values(
    const Eigen::MatrixBase<Derived>& jacobian,
    typename Derived::Scalar length = typename Derived::Scalar(1))
{
    svd_matrix_t<Derived> scaled = jacobian;
    scaled.template bottomRows<3>() /= length;

    if (scaled.cols() == 0)
    {
        return singular_values_t<Derived>{};
    }

    constexpr unsigned int options = (Derived::ColsAtCompileTime == Eigen::Dynamic)
        ? (Eigen::ComputeThinU | Eigen::ComputeThinV)
        : (Eigen::ComputeFullU | Eigen::ComputeFullV);
    return Eigen::JacobiSVD<svd_matrix_t<Derived>>(scaled, options).singularValues();
}

/// singular_values of the chain's body Jacobian at q.
///
/// This is the spelling to reach for. The measures below are defined on the
/// body Jacobian, and handing the space Jacobian to the overload above compiles
/// and answers a different question.
template <typename Chain, typename Vector>
singular_values_t<jacobian_matrix<typename Chain::scalar_type, Chain::joints>> singular_values(
    const Chain& chain,
    const Vector& q,
    typename Chain::scalar_type length = typename Chain::scalar_type(1))
{
    return singular_values(
        body_jacobian_unchecked(chain, forward_kinematics_unchecked(chain, q)), length);
}

/// Ratio of the largest singular value to the smallest: one at an isotropic
/// Jacobian, growing without bound towards a singularity and infinite at one.
///
/// Absent where the spectrum is empty, which is the chain with no joints.
template <typename Vector>
std::optional<typename Vector::Scalar> condition_number(const Vector& sigma)
{
    using scalar = typename Vector::Scalar;

    if (sigma.size() == 0)
    {
        return std::nullopt;
    }
    const scalar smallest = sigma(sigma.size() - 1);
    if (!(smallest > scalar(0)))
    {
        return std::numeric_limits<scalar>::infinity();
    }
    return sigma(0) / smallest;
}

/// Yoshikawa's manipulability measure, the product of the singular values --
/// the volume of the manipulability ellipsoid up to a constant factor.
///
/// Yoshikawa, Manipulability of Robotic Mechanisms, International Journal of
/// Robotics Research 4(2), 1985.
///
/// Undefined, rather than one, on an empty spectrum: the product over an empty
/// set is one, which would report a chain with no joints as maximally
/// manipulable.
template <typename Vector>
std::optional<typename Vector::Scalar> manipulability(const Vector& sigma)
{
    if (sigma.size() == 0)
    {
        return std::nullopt;
    }
    return sigma.prod();
}

/// Salisbury's isotropy index, the inverse condition number: one where the
/// ellipsoid is a sphere and zero at a singularity.
///
/// Salisbury & Craig, Articulated Hands: Force Control and Kinematic Issues,
/// International Journal of Robotics Research 1(1), 1982.
template <typename Vector>
std::optional<typename Vector::Scalar> isotropy(const Vector& sigma)
{
    using scalar = typename Vector::Scalar;

    if (sigma.size() == 0 || !(sigma(0) > scalar(0)))
    {
        return std::nullopt;
    }
    return sigma(sigma.size() - 1) / sigma(0);
}

/// The condition number at or above which a configuration counts as
/// near-singular.
///
/// A screening convention, not a property of any robot: a ratio of 1e3 means
/// the Jacobian's most and least responsive directions differ by three orders
/// of magnitude. How close is too close depends on the task and on the
/// actuator headroom, which is why the threshold is an argument.
template <typename Scalar>
inline constexpr Scalar default_singularity_threshold_v = Scalar(1e3);

/// Whether the configuration is within threshold of a singularity, or nullopt
/// where the spectrum is empty and the question has no answer.
template <typename Vector>
std::optional<bool> is_near_singular(
    const Vector& sigma,
    typename Vector::Scalar threshold =
        default_singularity_threshold_v<typename Vector::Scalar>)
{
    auto kappa = condition_number(sigma);
    if (!kappa)
    {
        return std::nullopt;
    }
    return *kappa >= threshold;
}

/// is_near_singular at a configuration of the chain.
///
/// Writing `if (is_near_singular(chain, q))` on the optional compiles and asks
/// whether the question was answerable, not whether the configuration is near a
/// singularity. Test the value, or pass the answer for the unanswerable case
/// explicitly.
template <typename Chain, typename Vector>
std::optional<bool> is_near_singular(
    const Chain& chain,
    const Vector& q,
    typename Chain::scalar_type threshold =
        default_singularity_threshold_v<typename Chain::scalar_type>,
    typename Chain::scalar_type length = typename Chain::scalar_type(1))
{
    return is_near_singular(singular_values(chain, q, length), threshold);
}

}

#endif
