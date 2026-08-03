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

/// The level at or below which a computed singular value is rounding rather
/// than measurement: the largest singular value scaled by the working precision
/// and the size of the spectrum. This is the numerical rank criterion LAPACK
/// and Eigen's own SVDBase::setThreshold apply, and it is what makes "singular"
/// a portable answer. A structurally rank-deficient Jacobian -- two joints on
/// one screw axis, say -- has an exactly zero smallest singular value in exact
/// arithmetic, but the decomposition of it lands on a clean zero on one
/// instruction set and on a value at this floor on another. Only a test against
/// the floor calls both of them singular; a test against zero calls one of them
/// merely ill-conditioned and reports a ratio built entirely from rounding.
template <typename Scalar>
Scalar spectrum_resolution_floor(Scalar largest, Eigen::Index count)
{
    return largest * std::numeric_limits<Scalar>::epsilon() * static_cast<Scalar>(count);
}

/// Ratio of the largest singular value to the smallest: one at an isotropic
/// Jacobian, growing without bound towards a singularity and infinite at one.
/// That infinity is a measurement and not a failure -- a singular configuration
/// has an infinite condition number.
///
/// "At one" means at or below the resolution floor above, not at exactly zero.
/// Past that floor the smallest singular value is the decomposition's rounding
/// and the ratio built from it is rounding too, so it is reported as the
/// infinity it is approximating rather than as whichever large number the
/// instruction set happened to produce.
template <typename Vector>
cartan::expected<typename Vector::Scalar, singularity_failure>
condition_number(const Vector& sigma)
{
    using scalar = typename Vector::Scalar;

    if (sigma.size() == 0)
    {
        return cartan::unexpected(singularity_failure::empty_spectrum);
    }
    const scalar largest = sigma(0);
    const scalar smallest = sigma(sigma.size() - 1);
    if (!(smallest > spectrum_resolution_floor(largest, sigma.size())))
    {
        return std::numeric_limits<scalar>::infinity();
    }
    return largest / smallest;
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
///
/// It is the inverse of condition_number above and answers at the same
/// resolution floor, so the pair cannot disagree about whether a spectrum is
/// singular: where that reports infinity this reports exactly zero.
template <typename Vector>
cartan::expected<typename Vector::Scalar, singularity_failure>
isotropy(const Vector& sigma)
{
    using scalar = typename Vector::Scalar;

    if (sigma.size() == 0)
    {
        return cartan::unexpected(singularity_failure::empty_spectrum);
    }
    const scalar largest = sigma(0);
    if (!(largest > scalar(0)))
    {
        return cartan::unexpected(singularity_failure::zero_spectrum);
    }
    const scalar smallest = sigma(sigma.size() - 1);
    if (!(smallest > spectrum_resolution_floor(largest, sigma.size())))
    {
        return scalar(0);
    }
    return smallest / largest;
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
