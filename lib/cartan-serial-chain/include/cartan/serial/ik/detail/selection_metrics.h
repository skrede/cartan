#ifndef HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_SELECTION_METRICS_H
#define HPP_GUARD_CARTAN_SERIAL_IK_DETAIL_SELECTION_METRICS_H

/// The metric each selection objective ranks a candidate configuration on, and
/// the comparison that ranks two of them.
///
/// The single-policy and the racing selection paths both read this, so the two
/// answer the same question the same way.

#include "cartan/serial/ik/ik_status.h"

#include "cartan/serial/fk/jacobian.h"
#include "cartan/serial/fk/forward_kinematics.h"

#include <Eigen/SVD>

#include <cmath>
#include <optional>

namespace cartan::detail
{

constexpr bool maximizes(ik_objective objective)
{
    return objective == ik_objective::max_manipulability
        || objective == ik_objective::max_isotropy;
}

constexpr bool reads_jacobian(ik_objective objective)
{
    return objective == ik_objective::max_manipulability
        || objective == ik_objective::max_isotropy;
}

template <typename Chain>
bool mixes_revolute_and_prismatic(const Chain& chain)
{
    bool revolute = false;
    bool prismatic = false;
    for (int i = 0; i < chain.num_joints(); ++i)
    {
        (chain.axis(i).is_revolute() ? revolute : prismatic) = true;
    }
    return revolute && prismatic;
}

/// The combinations a selection cannot answer, reported so setup can refuse
/// them rather than rank on a fabricated number.
///
/// A chain with no joints has no singular values, so neither Jacobian measure
/// is defined on it. A joint-space displacement over a chain mixing revolute
/// and prismatic joints would add radians to metres; the scale that would make
/// them commensurable is a caller's to state and not this library's to assume,
/// so the combination is refused instead. The characteristic length divides the
/// Jacobian's linear rows, which a zero, negative or non-finite value cannot.
template <typename Chain, typename Scalar>
std::optional<ik_status> selection_admissibility(
    ik_objective objective,
    const Chain& chain,
    Scalar length)
{
    if (!(length > Scalar(0)) || !std::isfinite(length))
    {
        return ik_status::unsupported_configuration;
    }
    if (reads_jacobian(objective) && chain.num_joints() == 0)
    {
        return ik_status::unsupported_configuration;
    }
    if (objective == ik_objective::min_joint_distance && mixes_revolute_and_prismatic(chain))
    {
        return ik_status::unsupported_configuration;
    }
    return std::nullopt;
}

/// Singular values of the body Jacobian whose linear rows -- the last three, in
/// the omega-first convention -- are divided by a characteristic length, so they
/// are commensurable with the dimensionless angular rows above them. Yoshikawa
/// normalizes by a characteristic length for that reason; without it the product
/// below has no coherent unit and the ratio compares incommensurables.
///
/// Yoshikawa, Manipulability of Robotic Mechanisms, International Journal of
/// Robotics Research 4(2), 1985 -- the product of the singular values.
/// Salisbury & Craig, Articulated Hands: Force Control and Kinematic Issues,
/// International Journal of Robotics Research 1(1), 1982 -- the inverse
/// condition number.
///
/// Both are undefined, rather than one or zero, where the decomposition has no
/// singular values: the product over an empty set is one, which would report a
/// chain with no joints as maximally manipulable. The guard precedes the
/// decomposition because constructing it over an empty matrix is itself the
/// fault -- Eigen's preconditioner resizes a fixed-size vector to zero, which
/// trips an assertion in a checked build and faults in one without.
template <typename Chain, typename Vector>
std::optional<typename Chain::scalar_type> jacobian_metric(
    ik_objective objective,
    const Chain& chain,
    const Vector& q,
    typename Chain::scalar_type length)
{
    using scalar = typename Chain::scalar_type;

    if (chain.num_joints() == 0)
    {
        return std::nullopt;
    }

    auto J = body_jacobian_unchecked(chain, forward_kinematics_unchecked(chain, q));
    J.template bottomRows<3>() /= length;

    constexpr unsigned int svd_opts = (Chain::joints == dynamic)
        ? (Eigen::ComputeThinU | Eigen::ComputeThinV)
        : (Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::JacobiSVD<jacobian_matrix<scalar, Chain::joints>> svd(J, svd_opts);
    const auto& sigma = svd.singularValues();

    if (objective == ik_objective::max_manipulability)
    {
        scalar product{1};
        for (int i = 0; i < static_cast<int>(sigma.size()); ++i)
        {
            product *= sigma(i);
        }
        return product;
    }

    if (!(sigma(0) > scalar(0)))
    {
        return std::nullopt;
    }
    return sigma(sigma.size() - 1) / sigma(0);
}

/// The metric a candidate is ranked on, absent where the objective defines
/// none. `speed` accepts the first converged candidate and ranks nothing.
template <typename Chain, typename Vector>
std::optional<typename Chain::scalar_type> selection_metric(
    ik_objective objective,
    const Chain& chain,
    const Vector& q,
    const Vector& reference,
    typename Chain::scalar_type error_norm,
    typename Chain::scalar_type length)
{
    switch (objective)
    {
    case ik_objective::min_error_norm:
        return error_norm;
    case ik_objective::min_joint_distance:
        return (q - reference).norm();
    case ik_objective::max_manipulability:
    case ik_objective::max_isotropy:
        return jacobian_metric(objective, chain, q, length);
    case ik_objective::speed:
        break;
    }
    return std::nullopt;
}

/// A strict improvement in the objective's own direction, which makes the
/// selection independent of the order the candidates arrive in. An absent
/// metric never displaces an incumbent, so a candidate whose measure is
/// undefined cannot win by arriving first.
template <typename Scalar>
bool improves_on(
    ik_objective objective,
    const std::optional<Scalar>& candidate,
    const std::optional<Scalar>& incumbent)
{
    if (!candidate)
    {
        return false;
    }
    if (!incumbent)
    {
        return true;
    }
    return maximizes(objective) ? *candidate > *incumbent : *candidate < *incumbent;
}

}

#endif
