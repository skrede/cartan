#ifndef HPP_GUARD_CARTAN_SERIAL_FK_SINGULARITY_ANALYSIS_H
#define HPP_GUARD_CARTAN_SERIAL_FK_SINGULARITY_ANALYSIS_H

/// The four measures of distance to a singularity, read off one spectrum.
///
/// Every no-answer case carries a name from singularity_failure rather than
/// collapsing into a single absence, so a caller can tell a chain with no
/// joints from a configuration that produced no Jacobian.

#include "cartan/expected.h"

#include "cartan/serial/fk/singular_spectrum.h"
#include "cartan/serial/fk/singularity_failure.h"

#include <limits>

namespace cartan
{

/// Ratio of the largest singular value to the smallest: one at an isotropic
/// Jacobian, growing without bound towards a singularity and infinite at one.
/// That infinity is a measurement and not a failure -- a singular configuration
/// has an infinite condition number.
template <typename Vector>
cartan::expected<typename Vector::Scalar, singularity_failure>
condition_number(const Vector& sigma)
{
    using scalar = typename Vector::Scalar;

    if (sigma.size() == 0)
    {
        return cartan::unexpected(singularity_failure::empty_spectrum);
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
cartan::expected<typename Vector::Scalar, singularity_failure>
manipulability(const Vector& sigma)
{
    if (sigma.size() == 0)
    {
        return cartan::unexpected(singularity_failure::empty_spectrum);
    }
    return sigma.prod();
}

/// Salisbury's isotropy index, the inverse condition number: one where the
/// ellipsoid is a sphere and zero at a singularity.
///
/// Salisbury & Craig, Articulated Hands: Force Control and Kinematic Issues,
/// International Journal of Robotics Research 1(1), 1982.
template <typename Vector>
cartan::expected<typename Vector::Scalar, singularity_failure>
isotropy(const Vector& sigma)
{
    using scalar = typename Vector::Scalar;

    if (sigma.size() == 0)
    {
        return cartan::unexpected(singularity_failure::empty_spectrum);
    }
    if (!(sigma(0) > scalar(0)))
    {
        return cartan::unexpected(singularity_failure::zero_spectrum);
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

/// Whether the configuration is within threshold of a singularity.
///
/// Writing `if (is_near_singular(sigma))` asks whether the question was
/// answerable, not whether the configuration is near a singularity: an errored
/// expected is falsy exactly as an empty optional was. Test the value, or
/// answer the unanswerable case explicitly.
template <typename Vector>
cartan::expected<bool, singularity_failure> is_near_singular(
    const Vector& sigma,
    typename Vector::Scalar threshold =
        default_singularity_threshold_v<typename Vector::Scalar>)
{
    auto kappa = condition_number(sigma);
    if (!kappa)
    {
        return cartan::unexpected(kappa.error());
    }
    return *kappa >= threshold;
}

/// is_near_singular at a configuration of the chain, carrying the same
/// truth-test trap as the overload above.
template <typename Chain, typename Vector>
cartan::expected<bool, singularity_failure> is_near_singular(
    const Chain& chain,
    const Vector& q,
    typename Chain::scalar_type threshold =
        default_singularity_threshold_v<typename Chain::scalar_type>,
    typename Chain::scalar_type length = typename Chain::scalar_type(1))
{
    auto sigma = singular_values(chain, q, length);
    if (!sigma)
    {
        return cartan::unexpected(sigma.error());
    }
    return is_near_singular(*sigma, threshold);
}

}

#endif
